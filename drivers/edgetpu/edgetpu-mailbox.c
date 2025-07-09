// SPDX-License-Identifier: GPL-2.0
/*
 * Utility functions of mailbox protocol for Edge TPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <asm/page.h>
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mmzone.h> /* MAX_ORDER_NR_PAGES */
#include <linux/slab.h>

#include "edgetpu-device-group.h"
#include "edgetpu-iremap-pool.h"
#include "edgetpu-kci.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-mmu.h"
#include "edgetpu-sw-watchdog.h"
#include "edgetpu-wakelock.h"
#include "edgetpu.h"

/*
 * Checks if @size is a valid circular queue size, which should be a positive
 * number and less than or equal to MAX_QUEUE_SIZE.
 */
static inline bool valid_circular_queue_size(u32 size)
{
	if (!size || size > MAX_QUEUE_SIZE)
		return false;
	return true;
}

/* Return context ID for mailbox. */
static inline enum edgetpu_context_id
edgetpu_mailbox_context_id(struct edgetpu_mailbox *mailbox)
{
	if (!mailbox)
		return EDGETPU_CONTEXT_INVALID;
	return EDGETPU_CONTEXT_VII_BASE + mailbox->mailbox_id - 1;
}

/* Sets mailbox->cmd_queue_tail and corresponding CSR on device. */
static void edgetpu_mailbox_set_cmd_queue_tail(struct edgetpu_mailbox *mailbox,
					       u32 value)
{
	mailbox->cmd_queue_tail = value;
	EDGETPU_MAILBOX_CMD_QUEUE_WRITE_SYNC(mailbox, tail, value);
}

/* Sets mailbox->resp_queue_head and corresponding CSR on device. */
static void edgetpu_mailbox_set_resp_queue_head(struct edgetpu_mailbox *mailbox,
						u32 value)
{
	mailbox->resp_queue_head = value;
	EDGETPU_MAILBOX_RESP_QUEUE_WRITE(mailbox, head, value);
}

/*
 * Allocates and returns a mailbox given the index of this mailbox,
 * also enables the mailbox.
 *
 * Caller holds mgr->mailboxes_lock.
 */
static struct edgetpu_mailbox *
edgetpu_mailbox_create_locked(struct edgetpu_mailbox_manager *mgr, uint index)
{
	struct edgetpu_mailbox *mailbox = kzalloc(sizeof(*mailbox), GFP_ATOMIC);

	if (!mailbox)
		return ERR_PTR(-ENOMEM);
	mailbox->mailbox_id = index;
	mailbox->etdev = mgr->etdev;
	mailbox->context_csr_base = mgr->get_context_csr_base(index);
	mailbox->cmd_queue_csr_base = mgr->get_cmd_queue_csr_base(index);
	mailbox->resp_queue_csr_base = mgr->get_resp_queue_csr_base(index);
	edgetpu_mailbox_init_doorbells(mailbox);

	return mailbox;
}

/* Caller must hold @mgr->mailboxes_lock. */
static int edgetpu_mailbox_remove_locked(struct edgetpu_mailbox_manager *mgr,
					 struct edgetpu_mailbox *mailbox)
{
	/* simple security checks */
	if (mailbox->mailbox_id >= mgr->num_mailbox ||
	    mgr->mailboxes[mailbox->mailbox_id] != mailbox) {
		return -EINVAL;
	}

	mgr->mailboxes[mailbox->mailbox_id] = NULL;
	/* KCI mailbox is a special case */
	if (mailbox->mailbox_id == KERNEL_MAILBOX_INDEX)
		edgetpu_kci_release(mgr->etdev, mailbox->internal.kci);
	kfree(mailbox);
	return 0;
}

/*
 * Disables the @index-th mailbox via setting CSR. Doesn't need
 * @mgr->mailboxes[index] be allocated.
 */
static void edgetpu_mailbox_disable_ith(struct edgetpu_mailbox_manager *mgr,
					uint index)
{
	struct edgetpu_mailbox mbox = {
		.etdev = mgr->etdev,
		.context_csr_base = mgr->get_context_csr_base(index),
	};

	edgetpu_mailbox_disable(&mbox);
}

static void edgetpu_vii_irq_handler(struct edgetpu_mailbox *mailbox)
{
	if (mailbox->internal.group)
		edgetpu_group_notify(mailbox->internal.group,
				     EDGETPU_EVENT_RESPDATA);
}

/*
 * Increases the command queue tail by @inc.
 *
 * The queue uses the mirrored circular buffer arrangement. Each index (head and
 * tail) has a wrap bit, represented by the constant CIRCULAR_QUEUE_WRAP_BIT.
 * Whenever an index is increased and will exceed the end of the queue, the wrap
 * bit is xor-ed.
 *
 * This method will update both mailbox->cmd_queue_tail and CSR on device.
 *
 * Caller ensures @inc is less than the space remain in the command queue.
 */
void edgetpu_mailbox_inc_cmd_queue_tail(struct edgetpu_mailbox *mailbox,
					u32 inc)
{
	u32 new_tail;

	new_tail = circular_queue_inc(mailbox->cmd_queue_tail, inc,
				      mailbox->cmd_queue_size);
	edgetpu_mailbox_set_cmd_queue_tail(mailbox, new_tail);
}

/*
 * Increases the response queue head by @inc.
 *
 * The queue uses the mirrored circular buffer arrangement. Each index (head and
 * tail) has a wrap bit, represented by the constant CIRCULAR_QUEUE_WRAP_BIT.
 * Whenever an index is increased and will exceed the end of the queue, the wrap
 * bit is xor-ed.
 *
 * This method will update both mailbox->resp_queue_head and CSR on device.
 *
 * Caller ensures @inc is less than the distance between resp_head and
 * resp_tail.
 */
void edgetpu_mailbox_inc_resp_queue_head(struct edgetpu_mailbox *mailbox,
					 u32 inc)
{
	u32 new_head;

	new_head = circular_queue_inc(mailbox->resp_queue_head, inc,
				      mailbox->resp_queue_size);
	edgetpu_mailbox_set_resp_queue_head(mailbox, new_head);
}

/*
 * Sets address and size of queue.
 *
 * Sets the queue address with @addr, a 36-bit address, and with size @size in
 * units of number of elements.
 *
 * Returns 0 on success.
 * -EINVAL is returned if @addr or @size is invalid.
 */
int edgetpu_mailbox_set_queue(struct edgetpu_mailbox *mailbox,
			      enum mailbox_queue_type type, u64 addr, u32 size)
{
	u32 low = addr & 0xffffffff;
	u32 high = addr >> 32;

	if (!valid_circular_queue_size(size))
		return -EINVAL;
	/* addr is a 36-bit address, checks if the higher bits are clear */
	if (high & 0xfffffff0)
		return -EINVAL;

	switch (type) {
	case MAILBOX_CMD_QUEUE:
		EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, cmd_queue_address_low,
					      low);
		EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, cmd_queue_address_high,
					      high);
		EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, cmd_queue_size, size);
		mailbox->cmd_queue_size = size;
		edgetpu_mailbox_set_cmd_queue_tail(mailbox, 0);
		EDGETPU_MAILBOX_CMD_QUEUE_WRITE(mailbox, head, 0);
		break;
	case MAILBOX_RESP_QUEUE:
		EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, resp_queue_address_low,
					      low);
		EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, resp_queue_address_high,
					      high);
		EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, resp_queue_size, size);
		mailbox->resp_queue_size = size;
		edgetpu_mailbox_set_resp_queue_head(mailbox, 0);
		EDGETPU_MAILBOX_RESP_QUEUE_WRITE(mailbox, tail, 0);
		break;
	}

	return 0;
}

/* Reset mailbox queues, clear out any commands/responses left from before. */
void edgetpu_mailbox_reset(struct edgetpu_mailbox *mailbox)
{
	edgetpu_mailbox_disable(mailbox);
	EDGETPU_MAILBOX_CMD_QUEUE_WRITE(mailbox, head, 0);
	edgetpu_mailbox_set_cmd_queue_tail(mailbox, 0);
	edgetpu_mailbox_set_resp_queue_head(mailbox, 0);
	EDGETPU_MAILBOX_RESP_QUEUE_WRITE(mailbox, tail, 0);
	edgetpu_mailbox_enable(mailbox);
}

/* Sets the priority of @mailbox. */
void edgetpu_mailbox_set_priority(struct edgetpu_mailbox *mailbox, u32 priority)
{
	EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, priority, priority);
}

struct edgetpu_mailbox *
edgetpu_mailbox_vii_add(struct edgetpu_mailbox_manager *mgr, uint id)
{
	struct edgetpu_mailbox *mailbox = NULL;
	unsigned long flags;

	write_lock_irqsave(&mgr->mailboxes_lock, flags);
	if (id == 0) {
		uint i;

		for (i = mgr->vii_index_from; i < mgr->vii_index_to; i++) {
			if (!mgr->mailboxes[i]) {
				id = i;
				break;
			}
		}
	} else {
		/* no mailbox available - returns busy */
		if (id < mgr->vii_index_from || id >= mgr->vii_index_to ||
		    mgr->mailboxes[id])
			id = 0;
	}

	/* no empty slot found */
	if (id == 0) {
		mailbox = ERR_PTR(-EBUSY);
	} else {
		mailbox = edgetpu_mailbox_create_locked(mgr, id);
		if (!IS_ERR(mailbox)) {
			mgr->mailboxes[id] = mailbox;
			mailbox->handle_irq = edgetpu_vii_irq_handler;
		}
	}
	write_unlock_irqrestore(&mgr->mailboxes_lock, flags);
	return mailbox;
}

/*
 * Requests the first @n P2P mailboxes, where @n should be the number of devices
 * in a virtual device group.
 *
 * The array of requested mailboxes will be assigned to @mailboxes on success.
 * @mailboxes must have dimension @n, @mailboxes[@skip_i] will be set to NULL.
 *
 * Returns 0 on success, or a negative errno on error.
 * Returns -EBUSY if any mailbox is using.
 *
 * Caller calls edgetpu_mailbox_enable() to enable the returned mailboxes.
 */
int edgetpu_mailbox_p2p_batch(struct edgetpu_mailbox_manager *mgr, uint n,
			      uint skip_i, struct edgetpu_mailbox **mailboxes)
{
	uint i, j;
	int ret;
	struct edgetpu_mailbox *mailbox;
	unsigned long flags;

	if (mgr->p2p_index_to - mgr->p2p_index_from < n)
		return -EINVAL;

	write_lock_irqsave(&mgr->mailboxes_lock, flags);

	memset(mailboxes, 0, sizeof(*mailboxes) * n);
	for (i = mgr->p2p_index_from, j = 0; j < n; i++, j++) {
		if (mgr->mailboxes[i]) {
			ret = -EBUSY;
			goto release;
		}
	}

	for (i = mgr->p2p_index_from, j = 0; j < n; i++, j++) {
		if (j == skip_i) {
			edgetpu_mailbox_disable_ith(mgr, i);
			continue;
		}

		mailbox = edgetpu_mailbox_create_locked(mgr, i);
		if (IS_ERR(mailbox)) {
			ret = PTR_ERR(mailbox);
			goto release;
		}
		mailboxes[j] = mgr->mailboxes[i] = mailbox;
	}

	write_unlock_irqrestore(&mgr->mailboxes_lock, flags);
	return 0;

release:
	write_unlock_irqrestore(&mgr->mailboxes_lock, flags);

	for (i = 0; i < n; i++) {
		if (mailboxes[i])
			edgetpu_mailbox_remove(mgr, mailboxes[i]);
		mailboxes[i] = NULL;
	}

	return ret;
}

/*
 * Every mailbox manager can allocate one mailbox for KCI to use.
 * -EBUSY is returned if the KCI mailbox is allocated and hasn't been removed
 * via edgetpu_mailbox_remove().
 */
struct edgetpu_mailbox *edgetpu_mailbox_kci(struct edgetpu_mailbox_manager *mgr)
{
	struct edgetpu_mailbox *mailbox;
	unsigned long flags;

	write_lock_irqsave(&mgr->mailboxes_lock, flags);
	if (mgr->mailboxes[KERNEL_MAILBOX_INDEX]) {
		mailbox = ERR_PTR(-EBUSY);
		goto out;
	}

	mailbox = edgetpu_mailbox_create_locked(mgr, KERNEL_MAILBOX_INDEX);
	if (!IS_ERR(mailbox))
		mgr->mailboxes[KERNEL_MAILBOX_INDEX] = mailbox;

out:
	write_unlock_irqrestore(&mgr->mailboxes_lock, flags);
	return mailbox;
}

/*
 * Removes a mailbox from the manager.
 * Returns 0 on success.
 */
int edgetpu_mailbox_remove(struct edgetpu_mailbox_manager *mgr, struct edgetpu_mailbox *mailbox)
{
	unsigned long flags;
	int ret;

	write_lock_irqsave(&mgr->mailboxes_lock, flags);
	ret = edgetpu_mailbox_remove_locked(mgr, mailbox);
	write_unlock_irqrestore(&mgr->mailboxes_lock, flags);

	return ret;
}

/*
 * The queue size of edgetpu_mailbox_attr has units in KB, convert it to use the
 * element size here.
 *
 * Returns a negative errno on error, or the converted size.
 */
static int convert_runtime_queue_size_to_fw(u32 queue_size, u32 element_size)
{
	const u32 runtime_unit = 1024;
	u32 ret;

	/* zero size is not allowed */
	if (queue_size == 0 || element_size == 0)
		return -EINVAL;
	/* A quick check to prevent the queue allocation failure. */
	if (queue_size > (MAX_ORDER_NR_PAGES << PAGE_SHIFT) / runtime_unit)
		return -ENOMEM;
	/*
	 * Kernel doesn't care whether queue_size * runtime_unit is a multiple
	 * of element_size.
	 */
	ret = queue_size * runtime_unit / element_size;
	/* hardware limitation */
	if (ret == 0 || ret > MAX_QUEUE_SIZE)
		return -EINVAL;
	return ret;
}

int edgetpu_mailbox_validate_attr(const struct edgetpu_mailbox_attr *attr)
{
	int size;

	size = convert_runtime_queue_size_to_fw(attr->cmd_queue_size,
						attr->sizeof_cmd);
	if (size < 0)
		return size;
	size = convert_runtime_queue_size_to_fw(attr->resp_queue_size,
						attr->sizeof_resp);
	if (size < 0)
		return size;
	return 0;
}

int edgetpu_mailbox_init_vii(struct edgetpu_vii *vii,
			     struct edgetpu_device_group *group)
{
	int cmd_queue_size, resp_queue_size;
	struct edgetpu_mailbox_manager *mgr = group->etdev->mailbox_manager;
	struct edgetpu_mailbox *mailbox;
	const struct edgetpu_mailbox_attr *attr = &group->mbox_attr;
	int ret;

	if (!group->etdomain || group->etdomain->pasid == IOMMU_PASID_INVALID)
		mailbox = edgetpu_mailbox_vii_add(mgr, 0);
	else
		mailbox = edgetpu_mailbox_vii_add(mgr, group->etdomain->pasid);
	if (IS_ERR(mailbox))
		return PTR_ERR(mailbox);

	cmd_queue_size = convert_runtime_queue_size_to_fw(attr->cmd_queue_size,
							  attr->sizeof_cmd);
	resp_queue_size = convert_runtime_queue_size_to_fw(
		attr->resp_queue_size, attr->sizeof_resp);

	edgetpu_mailbox_set_priority(mailbox, attr->priority);
	EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox,
				      cmd_queue_tail_doorbell_enable,
				      attr->cmdq_tail_doorbell);

	ret = edgetpu_mailbox_alloc_queue(group->etdev, mailbox, cmd_queue_size,
					  attr->sizeof_cmd, MAILBOX_CMD_QUEUE,
					  &vii->cmd_queue_mem);
	if (ret) {
		edgetpu_mailbox_remove(mgr, mailbox);
		return ret;
	}

	etdev_dbg(group->etdev,
		  "%s: mbox %u cmdq iova=%pad dma=%pad\n",
		  __func__, mailbox->mailbox_id, &vii->cmd_queue_mem.tpu_addr,
		  &vii->cmd_queue_mem.dma_addr);
	ret = edgetpu_mailbox_alloc_queue(group->etdev, mailbox,
					  resp_queue_size, attr->sizeof_resp,
					  MAILBOX_RESP_QUEUE,
					  &vii->resp_queue_mem);

	if (ret) {
		edgetpu_mailbox_free_queue(group->etdev, mailbox,
					   &vii->cmd_queue_mem);
		edgetpu_mailbox_remove(mgr, mailbox);
		return ret;
	}

	etdev_dbg(group->etdev, "%s: mbox %u rspq iova=%pad dma=%pad\n", __func__,
		  mailbox->mailbox_id, &vii->resp_queue_mem.tpu_addr,
		  &vii->resp_queue_mem.dma_addr);
	mailbox->internal.group = edgetpu_device_group_get(group);
	vii->etdev = group->etdev;
	vii->mailbox = mailbox;
	edgetpu_mailbox_enable(mailbox);
	return 0;
}

void edgetpu_mailbox_remove_vii(struct edgetpu_vii *vii)
{
	struct edgetpu_dev *etdev;

	etdev = vii->etdev;
	edgetpu_mailbox_free_queue(etdev, vii->mailbox, &vii->cmd_queue_mem);
	edgetpu_mailbox_free_queue(etdev, vii->mailbox, &vii->resp_queue_mem);
	if (vii->mailbox) {
		if (!vii->mailbox->internal.group->dev_inaccessible)
			edgetpu_mailbox_disable(vii->mailbox);
		edgetpu_device_group_put(vii->mailbox->internal.group);
		edgetpu_mailbox_remove(etdev->mailbox_manager, vii->mailbox);
		vii->mailbox = NULL;
	}
}

static int edgetpu_mailbox_do_alloc_queue(struct edgetpu_dev *etdev,
					  struct edgetpu_mailbox *mailbox, u32 queue_size,
					  u32 unit, edgetpu_queue_mem *mem)
{
	u32 size = unit * queue_size;

	/* Align queue size to page size for TPU MMU map. */
	size = __ALIGN_KERNEL(size, PAGE_SIZE);
	return edgetpu_iremap_alloc(etdev, size, mem,
				    edgetpu_mailbox_context_id(mailbox));
}

/*
 * Allocates memory for a queue.
 *
 * The total size (in bytes) of queue is @queue_size * @unit.
 * CSRs of @mailbox include queue_size and queue_address will be set on success.
 * @mem->dma_addr, @mem->vaddr, and @mem->size will be set.
 *
 * Returns 0 on success, or a negative errno on error.
 */
int edgetpu_mailbox_alloc_queue(struct edgetpu_dev *etdev,
				struct edgetpu_mailbox *mailbox, u32 queue_size,
				u32 unit, enum mailbox_queue_type type,
				edgetpu_queue_mem *mem)
{
	int ret = edgetpu_mailbox_do_alloc_queue(etdev, mailbox, queue_size, unit, mem);
	if (ret)
		return ret;

	ret = edgetpu_mailbox_set_queue(mailbox, type, mem->tpu_addr,
					queue_size);
	if (ret) {
		edgetpu_mailbox_free_queue(etdev, mailbox, mem);
		return ret;
	}
	return 0;
}

/*
 * Releases the queue memory previously allocated with
 * edgetpu_mailbox_alloc_queue().
 *
 * Does nothing if @mem->vaddr is NULL.
 */
void edgetpu_mailbox_free_queue(struct edgetpu_dev *etdev,
				struct edgetpu_mailbox *mailbox,
				edgetpu_queue_mem *mem)
{
	if (!mem->vaddr)
		return;

	edgetpu_iremap_free(etdev, mem, edgetpu_mailbox_context_id(mailbox));
}

/*
 * Creates a mailbox manager, one edgetpu device has one manager.
 */
struct edgetpu_mailbox_manager *
edgetpu_mailbox_create_mgr(struct edgetpu_dev *etdev,
			   const struct edgetpu_mailbox_manager_desc *desc)
{
	struct edgetpu_mailbox_manager *mgr;
	uint total = 0;

	total += 1; /* KCI mailbox */
	total += desc->num_vii_mailbox;
	total += desc->num_p2p_mailbox;
	total += desc->num_ext_mailbox;
	if (total > desc->num_mailbox)
		return ERR_PTR(-EINVAL);
	mgr = devm_kzalloc(etdev->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return ERR_PTR(-ENOMEM);

	mgr->etdev = etdev;
	mgr->num_mailbox = desc->num_mailbox;
	/* index 0 is reserved for KCI */
	mgr->vii_index_from = 1;
	mgr->vii_index_to = mgr->vii_index_from + desc->num_vii_mailbox;
	mgr->p2p_index_from = mgr->vii_index_to;
	mgr->p2p_index_to = mgr->p2p_index_from + desc->num_p2p_mailbox;
	mgr->ext_index_from = mgr->p2p_index_to;
	mgr->ext_index_to = mgr->ext_index_from + desc->num_ext_mailbox;

	mgr->get_context_csr_base = desc->get_context_csr_base;
	mgr->get_cmd_queue_csr_base = desc->get_cmd_queue_csr_base;
	mgr->get_resp_queue_csr_base = desc->get_resp_queue_csr_base;

	mgr->mailboxes = devm_kcalloc(etdev->dev, mgr->num_mailbox,
				      sizeof(*mgr->mailboxes), GFP_KERNEL);
	if (!mgr->mailboxes)
		return ERR_PTR(-ENOMEM);
	rwlock_init(&mgr->mailboxes_lock);
	mutex_init(&mgr->open_devices.lock);

	return mgr;
}

/* All requested mailboxes will be disabled and freed. */
void edgetpu_mailbox_remove_all(struct edgetpu_mailbox_manager *mgr)
{
	uint i;
	struct edgetpu_mailbox *kci_mailbox = NULL;
	unsigned long flags;

	if (IS_ERR_OR_NULL(mgr))
		return;
	write_lock_irqsave(&mgr->mailboxes_lock, flags);
	for (i = 0; i < mgr->num_mailbox; i++) {
		struct edgetpu_mailbox *mailbox = mgr->mailboxes[i];

		if (mailbox) {
			edgetpu_mailbox_disable(mailbox);
			/* KCI needs special handling */
			if (i == KERNEL_MAILBOX_INDEX)
				kci_mailbox = mailbox;
			else
				kfree(mailbox);
			mgr->mailboxes[i] = NULL;
		}
	}
	write_unlock_irqrestore(&mgr->mailboxes_lock, flags);

	/* Cancel KCI worker outside the lock, then free KCI mailbox. */
	if (kci_mailbox) {
		edgetpu_kci_release(mgr->etdev,
				    kci_mailbox->internal.kci);
		kfree(kci_mailbox);
	}
}

/*
 * The interrupt handler for KCI and VII mailboxes.
 *
 * This handler loops through such mailboxes with an interrupt pending and
 *  invokes their IRQ handlers.
 */
irqreturn_t edgetpu_mailbox_handle_irq(struct edgetpu_mailbox_manager *mgr)
{
	struct edgetpu_mailbox *mailbox;
	uint i;

	if (!mgr)
		return IRQ_NONE;

	read_lock(&mgr->mailboxes_lock);
	for (i = 0; i < mgr->vii_index_to; i++) {
		mailbox = mgr->mailboxes[i];
		if (!mailbox)
			continue;
		if (!EDGETPU_MAILBOX_RESP_QUEUE_READ(mailbox, doorbell_status))
			continue;
		EDGETPU_MAILBOX_RESP_QUEUE_WRITE(mailbox, doorbell_clear, 1);
		etdev_dbg(mgr->etdev, "mbox %u resp doorbell irq tail=%u\n",
			  i, EDGETPU_MAILBOX_RESP_QUEUE_READ(mailbox, tail));
		if (mailbox->handle_irq)
			mailbox->handle_irq(mailbox);
	}
	read_unlock(&mgr->mailboxes_lock);

	return IRQ_HANDLED;
}

void edgetpu_mailbox_init_doorbells(struct edgetpu_mailbox *mailbox)
{
	/* Clear any stale doorbells requested */
	EDGETPU_MAILBOX_RESP_QUEUE_WRITE(mailbox, doorbell_clear, 1);
	EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, cmd_queue_doorbell_clear, 1);
	/* Enable the command and response doorbell interrupts */
	EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, cmd_queue_doorbell_enable, 1);
	EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, resp_queue_doorbell_enable, 1);
}

void edgetpu_mailbox_reset_mailboxes(struct edgetpu_mailbox_manager *mgr)
{
	uint i;
	unsigned long flags;

	write_lock_irqsave(&mgr->mailboxes_lock, flags);
	/*
	 * Reset all the allocated mailboxes, starting from VII till
	 * external mailboxes.
	 */
	for (i = mgr->vii_index_from; i < mgr->ext_index_to; i++) {
		struct edgetpu_mailbox *mbox = mgr->mailboxes[i];

		if (!mbox)
			continue;
		edgetpu_mailbox_reset(mbox);
		edgetpu_mailbox_disable(mbox);
		edgetpu_mailbox_init_doorbells(mbox);
	}
	write_unlock_irqrestore(&mgr->mailboxes_lock, flags);
}

void edgetpu_mailbox_reinit_vii(struct edgetpu_device_group *group)
{
	int cmd_queue_size, resp_queue_size;
	struct edgetpu_mailbox *mailbox = group->vii.mailbox;
	const struct edgetpu_mailbox_attr *attr = &group->mbox_attr;

	cmd_queue_size = convert_runtime_queue_size_to_fw(attr->cmd_queue_size,
							  attr->sizeof_cmd);
	resp_queue_size = convert_runtime_queue_size_to_fw(
		attr->resp_queue_size, attr->sizeof_resp);

	etdev_dbg(group->etdev, "Restoring vii. workload_id=%u mbox_id=%u\n",
		  group->workload_id, mailbox->mailbox_id);

	etdev_dbg(group->etdev, "Priority: %d\n", attr->priority);
	etdev_dbg(group->etdev, "Tail doorbell %s",
		  attr->cmdq_tail_doorbell ? "enabled" : "disabled");
	etdev_dbg(group->etdev, "cmd queue: addr=%pad size=%u\n",
		  &group->vii.cmd_queue_mem.tpu_addr, cmd_queue_size);
	etdev_dbg(group->etdev, "resp queue: addr=%pad size=%u\n",
		  &group->vii.resp_queue_mem.tpu_addr, resp_queue_size);

	edgetpu_mailbox_set_priority(mailbox, attr->priority);
	EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, cmd_queue_tail_doorbell_enable,
				      attr->cmdq_tail_doorbell);
	edgetpu_mailbox_set_queue(mailbox, MAILBOX_CMD_QUEUE,
				  group->vii.cmd_queue_mem.tpu_addr,
				  cmd_queue_size);
	edgetpu_mailbox_set_queue(mailbox, MAILBOX_RESP_QUEUE,
				  group->vii.resp_queue_mem.tpu_addr,
				  resp_queue_size);
	edgetpu_mailbox_enable(mailbox);
}

static void edgetpu_mailbox_init_external_mailbox(struct edgetpu_external_mailbox *ext_mailbox)
{
	struct edgetpu_mailbox_attr attr;
	struct edgetpu_mailbox *mailbox;
	struct edgetpu_mailbox_descriptor *desc;
	uint i;

	attr = ext_mailbox->attr;

	for (i = 0; i < ext_mailbox->count; i++) {
		desc = &ext_mailbox->descriptors[i];
		mailbox = desc->mailbox;
		edgetpu_mailbox_set_priority(mailbox, attr.priority);
		EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, cmd_queue_tail_doorbell_enable,
					      attr.cmdq_tail_doorbell);
		edgetpu_mailbox_set_queue(mailbox, MAILBOX_CMD_QUEUE,
					  desc->cmd_queue_mem.tpu_addr,
					  attr.cmd_queue_size);
		edgetpu_mailbox_set_queue(mailbox, MAILBOX_RESP_QUEUE,
					  desc->resp_queue_mem.tpu_addr,
					  attr.resp_queue_size);
		edgetpu_mailbox_enable(mailbox);
	}
}

void edgetpu_mailbox_reinit_external_mailbox(struct edgetpu_device_group *group)
{
	struct edgetpu_external_mailbox *ext_mailbox = group->ext_mailbox;

	if (!ext_mailbox)
		return;

	etdev_dbg(group->etdev, "Restoring external attached %d mailboxes\n", ext_mailbox->count);
	edgetpu_mailbox_init_external_mailbox(ext_mailbox);
}

void edgetpu_mailbox_restore_active_mailbox_queues(struct edgetpu_dev *etdev)
{
	struct edgetpu_list_group *l;
	struct edgetpu_device_group *group;
	struct edgetpu_device_group **groups;
	size_t i, n = 0;

	mutex_lock(&etdev->groups_lock);
	groups = kmalloc_array(etdev->n_groups, sizeof(*groups), GFP_KERNEL);
	if (unlikely(!groups)) {
		/*
		 * Either the runtime is misbehaving (creates tons of groups),
		 * or the system is indeed OOM - we give up this restore
		 * process, which makes the runtime unable to communicate with
		 * the device through VII.
		 */
		mutex_unlock(&etdev->groups_lock);
		return;
	}
	/*
	 * Fetch the groups into an array to restore the VII without holding
	 * etdev->groups_lock. To prevent the potential deadlock that
	 * edgetpu_device_group_add() holds group->lock then etdev->groups_lock.
	 */
	etdev_for_each_group(etdev, l, group) {
		/*
		 * Quick skip without holding group->lock.
		 * Disbanded groups can never go back to the normal state.
		 */
		if (edgetpu_device_group_is_disbanded(group))
			continue;
		/*
		 * Increase the group reference to prevent the group being
		 * released after we release groups_lock.
		 */
		groups[n++] = edgetpu_device_group_get(group);
	}
	mutex_unlock(&etdev->groups_lock);

	/*
	 * We are not holding @etdev->groups_lock, what may race is:
	 *   1. The group is disbanding and being removed from @etdev.
	 *   2. A new group is adding to @etdev
	 *
	 * For (1.) the group will be marked as DISBANDED, so we check whether
	 * the group is finalized before performing VII re-init.
	 *
	 * For (2.), adding group to @etdev (edgetpu_device_group_add()) has
	 * nothing to do with VII, its VII will be set when the group is
	 * finalized.
	 */
	for (i = 0; i < n; i++) {
		group = groups[i];
		mutex_lock(&group->lock);
		/*
		 * If the group is just finalized or has mailbox attached in
		 * another process, this re-init is redundant but isn't harmful.
		 */
		if (edgetpu_group_finalized_and_attached(group)) {
			edgetpu_mailbox_reinit_vii(group);
			edgetpu_mailbox_reinit_external_mailbox(group);
		}
		mutex_unlock(&group->lock);
		edgetpu_device_group_put(group);
	}
	kfree(groups);
}

static int edgetpu_mailbox_external_alloc_queue_batch(struct edgetpu_external_mailbox *ext_mailbox)
{
	int ret, i;
	struct edgetpu_mailbox *mailbox;
	struct edgetpu_mailbox_attr attr;
	struct edgetpu_mailbox_descriptor *desc;
	struct edgetpu_dev *etdev = ext_mailbox->etdev;

	attr = ext_mailbox->attr;

	for (i = 0; i < ext_mailbox->count; i++) {
		desc = &ext_mailbox->descriptors[i];
		mailbox = desc->mailbox;
		ret = edgetpu_mailbox_do_alloc_queue(etdev, mailbox, attr.cmd_queue_size,
						     attr.sizeof_cmd, &desc->cmd_queue_mem);
		if (ret)
			goto undo;

		ret = edgetpu_mailbox_do_alloc_queue(etdev, mailbox, attr.resp_queue_size,
						     attr.sizeof_resp, &desc->resp_queue_mem);
		if (ret) {
			edgetpu_mailbox_free_queue(etdev, mailbox,
						   &desc->cmd_queue_mem);
			goto undo;
		}
	}
	return 0;
undo:
	while (i--) {
		desc = &ext_mailbox->descriptors[i];
		mailbox = desc->mailbox;
		edgetpu_mailbox_free_queue(etdev, mailbox, &desc->cmd_queue_mem);
		edgetpu_mailbox_free_queue(etdev, mailbox, &desc->resp_queue_mem);
	}
	return ret;
}

static void edgetpu_mailbox_external_free_queue_batch(struct edgetpu_external_mailbox *ext_mailbox)
{
	u32 i;
	struct edgetpu_mailbox *mailbox;
	struct edgetpu_mailbox_descriptor *desc;
	struct edgetpu_dev *etdev = ext_mailbox->etdev;

	for (i = 0; i < ext_mailbox->count; i++) {
		desc = &ext_mailbox->descriptors[i];
		mailbox = desc->mailbox;
		edgetpu_mailbox_free_queue(etdev, mailbox, &desc->cmd_queue_mem);
		edgetpu_mailbox_free_queue(etdev, mailbox, &desc->resp_queue_mem);
	}
}

/*
 * Checks if the indexes given for external mailboxes are in range of mailbox
 * manager(@mgr) managing the external mailboxes.
 */
static bool edgetpu_mailbox_external_check_range(struct edgetpu_mailbox_manager *mgr,
						 const int start, const int end)
{
	return (start <= end) && (mgr->ext_index_from <= start && mgr->ext_index_to > end);
}

/*
 * Allocates external mailboxes according to @ext_mailbox_req object and
 * associate it with @group.
 *
 * Caller should hold @group->lock
 */
static int edgetpu_mailbox_external_alloc(struct edgetpu_device_group *group,
					  struct edgetpu_external_mailbox_req *ext_mailbox_req)
{
	u32 i, j = 0, bmap, start, end;
	struct edgetpu_mailbox_manager *mgr = group->etdev->mailbox_manager;
	struct edgetpu_mailbox *mailbox;
	int ret = 0, count;
	struct edgetpu_external_mailbox *ext_mailbox;
	struct edgetpu_mailbox_attr attr;
	unsigned long flags;

	if (!edgetpu_device_group_is_finalized(group))
		return -EINVAL;

	if (group->ext_mailbox)
		return -EEXIST;

	if (!ext_mailbox_req)
		return -EINVAL;

	ret = edgetpu_mailbox_validate_attr(&ext_mailbox_req->attr);
	if (ret)
		return ret;

	attr = ext_mailbox_req->attr;

	if (!edgetpu_mailbox_external_check_range(mgr, ext_mailbox_req->start,
						  ext_mailbox_req->end))
		return -ERANGE;

	ext_mailbox = kzalloc(sizeof(*ext_mailbox), GFP_KERNEL);
	if (!ext_mailbox)
		return -ENOMEM;

	bmap = ext_mailbox_req->mbox_map;
	count = __sw_hweight32(bmap);

	ext_mailbox->descriptors =
		kcalloc(count, sizeof(struct edgetpu_mailbox_descriptor), GFP_KERNEL);
	if (!ext_mailbox->descriptors) {
		kfree(ext_mailbox);
		return -ENOMEM;
	}

	ext_mailbox->attr = attr;
	ext_mailbox->etdev = group->etdev;
	ext_mailbox->mbox_type = ext_mailbox_req->mbox_type;

	start = ext_mailbox_req->start;
	end = ext_mailbox_req->end;

	write_lock_irqsave(&mgr->mailboxes_lock, flags);
	while (bmap) {
		i = ffs(bmap) + start - 1;
		if (i > end) {
			ret = -EINVAL;
			goto unlock;
		}
		if (mgr->mailboxes[i]) {
			ret = -EBUSY;
			goto unlock;
		}
		bmap = bmap & (bmap - 1);
	}

	bmap = ext_mailbox_req->mbox_map;
	while (bmap) {
		i = ffs(bmap) + start - 1;
		mailbox = edgetpu_mailbox_create_locked(mgr, i);
		if (!IS_ERR(mailbox)) {
			mgr->mailboxes[i] = mailbox;
			ext_mailbox->descriptors[j++].mailbox = mailbox;
		} else {
			ret = PTR_ERR(mailbox);
			goto release;
		}
		bmap = bmap & (bmap - 1);
	}

	ext_mailbox->count = j;

	ret = edgetpu_mailbox_external_alloc_queue_batch(ext_mailbox);
	if (ret)
		goto release;
	write_unlock_irqrestore(&mgr->mailboxes_lock, flags);

	for (i = 0; i < count; i++) {
		mailbox = ext_mailbox->descriptors[i].mailbox;
		mailbox->internal.group = edgetpu_device_group_get(group);
	}
	group->ext_mailbox = ext_mailbox;
	return 0;
release:
	while (j--)
		edgetpu_mailbox_remove_locked(mgr, ext_mailbox->descriptors[j].mailbox);
unlock:
	write_unlock_irqrestore(&mgr->mailboxes_lock, flags);
	kfree(ext_mailbox->descriptors);
	kfree(ext_mailbox);
	return ret;
}

/* Caller must hold @group->lock. */
static void edgetpu_mailbox_external_free(struct edgetpu_device_group *group)
{
	struct edgetpu_mailbox_manager *mgr;
	struct edgetpu_mailbox *mailbox;
	struct edgetpu_external_mailbox *ext_mailbox;
	u32 i;

	ext_mailbox = group->ext_mailbox;
	if (!ext_mailbox)
		return;

	mgr = ext_mailbox->etdev->mailbox_manager;

	edgetpu_mailbox_external_free_queue_batch(ext_mailbox);

	for (i = 0; i < ext_mailbox->count; i++)  {
		mailbox = ext_mailbox->descriptors[i].mailbox;
		edgetpu_device_group_put(mailbox->internal.group);
		edgetpu_mailbox_remove(mgr, mailbox);
	}

	kfree(ext_mailbox->descriptors);
	kfree(ext_mailbox);
	group->ext_mailbox = NULL;
}

static int edgetpu_mailbox_external_alloc_enable(struct edgetpu_client *client,
						 struct edgetpu_external_mailbox_req *req)
{
	int ret = 0;
	struct edgetpu_device_group *group;

	mutex_lock(&client->group_lock);
	if (!client->group || !edgetpu_device_group_is_leader(client->group, client)) {
		mutex_unlock(&client->group_lock);
		return -EINVAL;
	}
	group = edgetpu_device_group_get(client->group);
	mutex_unlock(&client->group_lock);

	if (edgetpu_pm_get_if_powered(group->etdev->pm)) {
		mutex_lock(&group->lock);
		ret = edgetpu_mailbox_external_alloc(group, req);
		if (ret) {
			mutex_unlock(&group->lock);
			goto err;
		}
		edgetpu_mailbox_init_external_mailbox(group->ext_mailbox);
		ret = edgetpu_mailbox_activate_external_mailbox(group);
		mutex_unlock(&group->lock);
		edgetpu_pm_put(group->etdev->pm);
		goto out;
	} else {
		mutex_lock(&group->lock);
		ret = edgetpu_mailbox_external_alloc(group, req);
		mutex_unlock(&group->lock);
		goto out;
	}
err:
	edgetpu_pm_put(group->etdev->pm);
out:
	edgetpu_device_group_put(group);
	return ret;
}

static int edgetpu_mailbox_external_disable_free(struct edgetpu_client *client)
{
	struct edgetpu_device_group *group;

	mutex_lock(&client->group_lock);
	if (!client->group || !edgetpu_device_group_is_leader(client->group, client)) {
		mutex_unlock(&client->group_lock);
		return -EINVAL;
	}
	group = edgetpu_device_group_get(client->group);
	mutex_unlock(&client->group_lock);

	if (edgetpu_pm_get_if_powered(group->etdev->pm)) {
		mutex_lock(&group->lock);
		edgetpu_mailbox_external_disable_free_locked(group);
		mutex_unlock(&group->lock);
		edgetpu_pm_put(group->etdev->pm);
	} else {
		mutex_lock(&group->lock);
		edgetpu_mailbox_external_free(group);
		mutex_unlock(&group->lock);
	}

	edgetpu_device_group_put(group);
	return 0;
}

void edgetpu_mailbox_external_disable_free_locked(struct edgetpu_device_group *group)
{
	if (!group->dev_inaccessible) {
	/*
	 * Deactivate only fails if f/w is unresponsive which will put group
	 * in errored state or mailbox physically disabled before requesting
	 * deactivate which will never be the case.
	 */
		edgetpu_mailbox_deactivate_external_mailbox(group);
		edgetpu_mailbox_disable_external_mailbox(group);
	}
	edgetpu_mailbox_external_free(group);
}

static int edgetpu_mailbox_external_enable_by_id(struct edgetpu_client *client, int mailbox_id,
						 u32 client_priv)
{
	int ret;

	if (!edgetpu_wakelock_lock(client->wakelock)) {
		etdev_err(client->etdev, "Enabling mailbox %d needs wakelock acquired\n",
			  mailbox_id);
		edgetpu_wakelock_unlock(client->wakelock);
		return -EAGAIN;
	}

	etdev_dbg(client->etdev, "Enabling mailbox: %d\n", mailbox_id);

	ret = edgetpu_mailbox_activate(client->etdev, mailbox_id, client_priv, -1, false);
	if (ret)
		etdev_err(client->etdev, "Activate mailbox %d failed: %d", mailbox_id, ret);
	else
		edgetpu_wakelock_inc_event_locked(client->wakelock,
						  EDGETPU_WAKELOCK_EVENT_EXT_MAILBOX);
	edgetpu_wakelock_unlock(client->wakelock);
	return ret;
}

static int edgetpu_mailbox_external_disable_by_id(struct edgetpu_client *client, int mailbox_id)
{
	int ret = 0;

	/*
	 * A successful enable_ext() increases the wakelock event which prevents wakelock being
	 * released, so theoretically the check fail here can only happen when enable_ext() is
	 * failed or not called before.
	 */
	if (!edgetpu_wakelock_lock(client->wakelock)) {
		etdev_err(client->etdev, "Disabling mailbox %d needs wakelock acquired\n",
			  mailbox_id);
		edgetpu_wakelock_unlock(client->wakelock);
		return -EAGAIN;
	}

	etdev_dbg(client->etdev, "Disabling mailbox: %d\n", mailbox_id);

	edgetpu_mailbox_deactivate(client->etdev, mailbox_id);
	edgetpu_wakelock_dec_event_locked(client->wakelock, EDGETPU_WAKELOCK_EVENT_EXT_MAILBOX);
	edgetpu_wakelock_unlock(client->wakelock);
	return ret;
}

int edgetpu_mailbox_activate_external_mailbox(struct edgetpu_device_group *group)
{
	struct edgetpu_external_mailbox *ext_mailbox = group->ext_mailbox;
	uint vcid = group->vcid;
	u32 mbox_map = 0, i;
	int ret;

	if (!ext_mailbox)
		return -ENOENT;

	for (i = 0; i < ext_mailbox->count; i++)
		mbox_map |= BIT(ext_mailbox->descriptors[i].mailbox->mailbox_id);

	ret = edgetpu_mailbox_activate_bulk(ext_mailbox->etdev, mbox_map,
					    group->mbox_attr.client_priv, vcid, false);

	if (ret)
		etdev_err(group->etdev, "Activate mailbox bulk failed: %d", ret);
	return ret;
}

void edgetpu_mailbox_disable_external_mailbox(struct edgetpu_device_group *group)
{
	u32 i;
	struct edgetpu_external_mailbox *ext_mailbox = group->ext_mailbox;

	if (!ext_mailbox)
		return;

	for (i = 0; i < ext_mailbox->count; i++)
		edgetpu_mailbox_disable(ext_mailbox->descriptors[i].mailbox);

}

void edgetpu_mailbox_deactivate_external_mailbox(struct edgetpu_device_group *group)
{
	u32 i, mbox_map = 0;
	struct edgetpu_external_mailbox *ext_mailbox = group->ext_mailbox;

	if (!ext_mailbox)
		return;

	for (i = 0; i < ext_mailbox->count; i++)
		mbox_map |= BIT(ext_mailbox->descriptors[i].mailbox->mailbox_id);

	etdev_dbg(ext_mailbox->etdev, "Disabling mailboxes in map: %x\n", mbox_map);
	edgetpu_mailbox_deactivate_bulk(ext_mailbox->etdev, mbox_map);
}

int edgetpu_mailbox_enable_ext(struct edgetpu_client *client, int mailbox_id,
			       struct edgetpu_external_mailbox_req *ext_mailbox_req,
			       u32 client_priv)
{
	if (mailbox_id == EDGETPU_MAILBOX_ID_USE_ASSOC)
		return edgetpu_mailbox_external_alloc_enable(client, ext_mailbox_req);
	else
		return edgetpu_mailbox_external_enable_by_id(client, mailbox_id, client_priv);
}

int edgetpu_mailbox_disable_ext(struct edgetpu_client *client, int mailbox_id)
{
	if (mailbox_id == EDGETPU_MAILBOX_ID_USE_ASSOC)
		return edgetpu_mailbox_external_disable_free(client);
	else
		return edgetpu_mailbox_external_disable_by_id(client, mailbox_id);
}

int edgetpu_mailbox_activate_bulk(struct edgetpu_dev *etdev, u32 mailbox_map, u32 client_priv,
				  s16 vcid, bool first_open)
{
	struct edgetpu_handshake *eh = &etdev->mailbox_manager->open_devices;
	int ret = 0;

	mutex_lock(&eh->lock);
	if (mailbox_map & ~eh->fw_state)
		ret = edgetpu_kci_open_device(etdev->kci, mailbox_map & ~eh->fw_state, client_priv,
					      vcid, first_open);
	if (!ret) {
		eh->state |= mailbox_map;
		eh->fw_state |= mailbox_map;
	}
	mutex_unlock(&eh->lock);
	/*
	 * We are observing OPEN_DEVICE KCI fails while other KCIs (usage update / shutdown) still
	 * succeed and no firmware crash is reported. Kick off the firmware restart when we are
	 * facing this and hope this can rescue the device from the bad state.
	 */
	if (ret == -ETIMEDOUT)
		edgetpu_watchdog_bite(etdev, false);
	return ret;

}

int edgetpu_mailbox_activate(struct edgetpu_dev *etdev, u32 mailbox_id, u32 client_priv, s16 vcid,
			     bool first_open)
{
	return edgetpu_mailbox_activate_bulk(etdev, BIT(mailbox_id), client_priv, vcid, first_open);
}

void edgetpu_mailbox_deactivate_bulk(struct edgetpu_dev *etdev, u32 mailbox_map)
{
	struct edgetpu_handshake *eh = &etdev->mailbox_manager->open_devices;
	int ret = 0;

	mutex_lock(&eh->lock);
	if (mailbox_map & eh->fw_state)
		ret = edgetpu_kci_close_device(etdev->kci, mailbox_map & eh->fw_state);
	if (ret)
		etdev_err(etdev, "Deactivate mailbox for map %x failed: %d", mailbox_map, ret);
	/*
	 * Always clears the states, FW should never reject CLOSE_DEVICE requests unless it's
	 * unresponsive.
	 */
	eh->state &= ~mailbox_map;
	eh->fw_state &= ~mailbox_map;
	mutex_unlock(&eh->lock);
}

void edgetpu_mailbox_deactivate(struct edgetpu_dev *etdev, u32 mailbox_id)
{
	edgetpu_mailbox_deactivate_bulk(etdev, BIT(mailbox_id));
}

void edgetpu_handshake_clear_fw_state(struct edgetpu_handshake *eh)
{
	mutex_lock(&eh->lock);
	eh->fw_state = 0;
	mutex_unlock(&eh->lock);
}
