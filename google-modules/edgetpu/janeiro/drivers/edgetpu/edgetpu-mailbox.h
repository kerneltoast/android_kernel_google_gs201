/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Utility functions of mailbox protocol for Edge TPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_MAILBOX_H__
#define __EDGETPU_MAILBOX_H__

#include <linux/compiler.h>
#include <linux/irqreturn.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "edgetpu-internal.h"
#include "edgetpu.h"

#define CIRCULAR_QUEUE_WRAP_BIT (1 << 10)
#define CIRCULAR_QUEUE_INDEX_MASK (CIRCULAR_QUEUE_WRAP_BIT - 1)
#define CIRCULAR_QUEUE_VALID_MASK                                              \
	(CIRCULAR_QUEUE_INDEX_MASK | CIRCULAR_QUEUE_WRAP_BIT)
#define CIRCULAR_QUEUE_WRAPPED(idx) ((idx) & CIRCULAR_QUEUE_WRAP_BIT)
#define CIRCULAR_QUEUE_REAL_INDEX(idx) ((idx) & CIRCULAR_QUEUE_INDEX_MASK)

/* CMD_QUEUE_SIZE register is 10 bits wide */
#define MAX_QUEUE_SIZE (CIRCULAR_QUEUE_WRAP_BIT - 1)

/* the index of mailbox for kernel control interface */
#define KERNEL_MAILBOX_INDEX 0

/* Size of CSRs start from cmd_queue_csr_base can be mmap-ed to userspace. */
#define USERSPACE_CSR_SIZE 0x1000ul

/* Mailbox ID to indicate external mailboxes */
#define EDGETPU_MAILBOX_ID_USE_ASSOC -1

struct edgetpu_device_group;

struct edgetpu_mailbox {
	uint mailbox_id;
	struct edgetpu_dev *etdev;
	/* base offset for CSRs in struct edgetpu_mailbox_context_csr */
	u32 context_csr_base;
	/* base offset for CSRs in struct edgetpu_mailbox_cmd_queue_csr */
	u32 cmd_queue_csr_base;
	/* base offset for CSRs in struct edgetpu_mailbox_resp_queue_csr */
	u32 resp_queue_csr_base;

	/*
	 * Queue-related fields, all of them are in units of number of elements.
	 */

	u32 cmd_queue_size; /* size of cmd queue */
	u32 cmd_queue_tail; /* offset within the cmd queue */
	u32 resp_queue_size; /* size of resp queue */
	u32 resp_queue_head; /* offset within the resp queue */

	/* IRQ handler */
	void (*handle_irq)(struct edgetpu_mailbox *mailbox);

	/*
	 * Internal data. It's edgetpu_kci* for KCI mailbox,
	 * edgetpu_device_group* for VII mailboxes.
	 */
	union {
		struct edgetpu_kci *kci;
		struct edgetpu_device_group *group;
	} internal;
};

typedef struct edgetpu_coherent_mem edgetpu_queue_mem;

struct edgetpu_vii {
	/*
	 * The mailbox this VII uses, can be NULL when uninitialized or mailbox
	 * detached.
	 */
	struct edgetpu_mailbox *mailbox;
	struct edgetpu_dev *etdev;
	edgetpu_queue_mem cmd_queue_mem;
	edgetpu_queue_mem resp_queue_mem;
};

/* Structure to hold info about mailbox and its queues. */
struct edgetpu_mailbox_descriptor {
	struct edgetpu_mailbox *mailbox;
	edgetpu_queue_mem cmd_queue_mem;
	edgetpu_queue_mem resp_queue_mem;
};

enum edgetpu_ext_mailbox_type {
	EDGETPU_EXTERNAL_MAILBOX_TYPE_DSP,
	EDGETPU_EXTERNAL_MAILBOX_TYPE_AOC,
};

/* Structure to hold multiple external mailboxes allocated for a device group. */
struct edgetpu_external_mailbox {
	/* Number of external mailboxes allocated for a device group. */
	u32 count;
	/* Type of external mailbox */
	enum edgetpu_ext_mailbox_type mbox_type;
	/* Leader of device group. */
	struct edgetpu_dev *etdev;
	/* Array of external mailboxes info with length @count. */
	struct edgetpu_mailbox_descriptor *descriptors;
	/* Mailbox attribute for allocated external mailboxes. */
	struct edgetpu_mailbox_attr attr;
};

/* Structure used for requesting to allocate external mailboxes. */
struct edgetpu_external_mailbox_req {
	uint start; /* starting index of external mailbox in mailbox_manager */
	uint end; /* end index of external mailbox in mailbox_manager */
	uint mbox_map; /* bitmap of mailbox indexes to be allocated */
	enum edgetpu_ext_mailbox_type mbox_type; /* Type of external mailbox */
	struct edgetpu_mailbox_attr attr; /* mailbox attribute for allocation */
};

/*
 * Structure for recording the driver state vs FW state.
 *
 * Example usage:
 *   @state is a bit mask that denotes each mailbox to which an "OPEN_DEVICE"
 *   KCI has been sent.
 *   @fw_state is the bit mask of mailbox IDs for which the FW has received the
 *   "OPEN_DEVICE" KCI.
 *   In usual cases @state always equals @fw_state. But when the FW is reloaded,
 *   @fw_state is reset to zero, then this structure can be used to know the FW
 *   state is out-of-sync and need further actions.
 */
struct edgetpu_handshake {
	struct mutex lock;
	/* fields protected by @lock */
	u32 state;
	u32 fw_state;
};

typedef u32 (*get_csr_base_t)(uint index);

struct edgetpu_mailbox_manager {
	struct edgetpu_dev *etdev;
	/* total number of mailboxes that edgetpu device could provide */
	u8 num_mailbox;
	/* indices reserved for VII, the range is [from, to) */
	u8 vii_index_from, vii_index_to;
	/* indices reserved for P2P, the range is [from, to) */
	u8 p2p_index_from, p2p_index_to;
	/* indices reserved for external mailboxes */
	u8 ext_index_from, ext_index_to;
	rwlock_t mailboxes_lock;	/* protects mailboxes */
	struct edgetpu_mailbox **mailboxes;
	/* converts index (0 ~ num_mailbox - 1) of mailbox to CSR offset */
	get_csr_base_t get_context_csr_base;
	get_csr_base_t get_cmd_queue_csr_base;
	get_csr_base_t get_resp_queue_csr_base;
	struct edgetpu_handshake open_devices;
};

/* the structure to configure a mailbox manager */
struct edgetpu_mailbox_manager_desc {
	u8 num_mailbox;
	u8 num_vii_mailbox;
	u8 num_p2p_mailbox;
	u8 num_ext_mailbox;
	get_csr_base_t get_context_csr_base;
	get_csr_base_t get_cmd_queue_csr_base;
	get_csr_base_t get_resp_queue_csr_base;
};

/* Mailbox CSRs. The order and size are exactly the same as RTL defined. */

/* CSRs that can be accessed by AP kernel only, don't mmap them to userspace */
struct edgetpu_mailbox_context_csr {
	u32 context_enable;
	u32 priority;
	u32 cmd_queue_doorbell_enable;
	/* doorbell will be triggered automatically on cmd_queue_tail updated */
	u32 cmd_queue_tail_doorbell_enable;
	u32 cmd_queue_doorbell_clear;
	u32 cmd_queue_address_low;
	u32 cmd_queue_address_high;
	u32 cmd_queue_size;
	u32 resp_queue_doorbell_enable;
	u32 resp_queue_tail_doorbell_enable;
	u32 resp_queue_address_low;
	u32 resp_queue_address_high;
	u32 resp_queue_size;
	u32 config_spare_0;
	u32 config_spare_1;
	u32 config_spare_2;
	u32 config_spare_3;
} __packed;

/* CSRs that can be accessed by AP runtime */

struct edgetpu_mailbox_cmd_queue_csr {
	u32 doorbell_set;
	u32 doorbell_status;
	u32 head;
	u32 tail;
	u32 config;
	u32 error_status;
} __packed;

struct edgetpu_mailbox_resp_queue_csr {
	u32 doorbell_set;
	u32 doorbell_clear;
	u32 doorbell_status;
	u32 head;
	u32 tail;
	u32 config;
	u32 error_status;
} __packed;

/* Mailbox APIs */

/* to specify the operation is toward cmd or resp queue */
enum mailbox_queue_type {
	MAILBOX_CMD_QUEUE,
	MAILBOX_RESP_QUEUE
};

/*
 * Allocates the mailbox manager.
 *
 * Allocations are device-managed so no release function is needed to free the
 * manager.
 */
struct edgetpu_mailbox_manager *
edgetpu_mailbox_create_mgr(struct edgetpu_dev *etdev,
			   const struct edgetpu_mailbox_manager_desc *desc);

/* interrupt handler */
irqreturn_t edgetpu_mailbox_handle_irq(struct edgetpu_mailbox_manager *mgr);

/*
 * Removes the mailbox previously requested from a mailbox manager.
 *
 * This function doesn't change the state of mailbox enable/disable.
 */
int edgetpu_mailbox_remove(struct edgetpu_mailbox_manager *mgr, struct edgetpu_mailbox *mailbox);
/* Removes and disables all the mailboxes previously requested. */
void edgetpu_mailbox_remove_all(struct edgetpu_mailbox_manager *mgr);

/* configure mailbox */

/* set cmd/resp_queue's address and size */
int edgetpu_mailbox_set_queue(struct edgetpu_mailbox *mailbox,
			      enum mailbox_queue_type type, u64 addr, u32 size);
void edgetpu_mailbox_set_priority(struct edgetpu_mailbox *mailbox,
				  u32 priority);

/* Reset mailbox queues, clear out any commands/responses left from before. */
void edgetpu_mailbox_reset(struct edgetpu_mailbox *mailbox);

/*
 * Clears any stale doorbell requests and enables the doorbell interrupts
 * at the mailbox level
 */
void edgetpu_mailbox_init_doorbells(struct edgetpu_mailbox *mailbox);

/* utility functions for KCI */

/* requests the mailbox for KCI */
struct edgetpu_mailbox *edgetpu_mailbox_kci(
		struct edgetpu_mailbox_manager *mgr);
void edgetpu_mailbox_inc_cmd_queue_tail(struct edgetpu_mailbox *mailbox,
					u32 inc);
void edgetpu_mailbox_inc_resp_queue_head(struct edgetpu_mailbox *mailbox,
					 u32 inc);

/* utility functions for VII */

/*
 * Request the mailbox with mailbox_id equals @id.
 * @id = 0 means there is no preference, @mgr will return a spare mailbox.
 *
 * Caller calls edgetpu_mailbox_enable() to enable the returned mailbox.
 *
 * -EBUSY is returned if the requested @id is used or there is no mailbox
 * available.
 */
struct edgetpu_mailbox *
edgetpu_mailbox_vii_add(struct edgetpu_mailbox_manager *mgr, uint id);

/*
 * Validates the mailbox attributes.
 * Returns 0 if valid, otherwise a negative errno.
 *
 * See the error cases of EDGETPU_CREATE_GROUP in edgetpu.h for when will @attr
 * be considered as invalid.
 */
int edgetpu_mailbox_validate_attr(const struct edgetpu_mailbox_attr *attr);
/*
 * Sets mailbox and allocates queues to @vii.
 *
 * @group is the device group that @vii will be associated with,
 * @group->mbox_attr is used to set the VII mailbox attributes.
 *
 * @group->mbox_attr must be checked by edgetpu_mailbox_validate_attr() before
 * calling this function.
 *
 * Returns 0 on success.
 */
int edgetpu_mailbox_init_vii(struct edgetpu_vii *vii,
			     struct edgetpu_device_group *group);
void edgetpu_mailbox_remove_vii(struct edgetpu_vii *vii);


/*
 * Reset all mailboxes CSRs to valid values, needed after the device is power
 * gated.
 */
void edgetpu_mailbox_reset_mailboxes(struct edgetpu_mailbox_manager *mgr);


/* For VII and P2P mailboxes to allocate/free queue memory */

int edgetpu_mailbox_alloc_queue(struct edgetpu_dev *etdev,
				struct edgetpu_mailbox *mailbox, u32 queue_size,
				u32 unit, enum mailbox_queue_type type,
				edgetpu_queue_mem *mem);
void edgetpu_mailbox_free_queue(struct edgetpu_dev *etdev,
				struct edgetpu_mailbox *mailbox,
				edgetpu_queue_mem *mem);

/*
 * Re-programs the CSRs of queue addresses, context, priority etc. to @group's
 * VII mailbox.
 *
 * Caller holds @group->lock and ensures @group has mailbox attached.
 */
void edgetpu_mailbox_reinit_vii(struct edgetpu_device_group *group);

/*
 * Re-configure VII and external mailbox queues which have an active client, re-using
 * existing buffers
 */
void edgetpu_mailbox_restore_active_mailbox_queues(struct edgetpu_dev *etdev);

/* utility functions for P2P */

int edgetpu_mailbox_p2p_batch(struct edgetpu_mailbox_manager *mgr, uint n,
			      uint skip_i, struct edgetpu_mailbox **mailboxes);

/*
 * If @mailbox_id is EDGETPU_MAILBOX_ID_USE_ASSOC, use @ext_mailbox_req to
 * allocate external mailboxes and activate the allocated mailboxes.
 * Otherwise, activate the external mailbox with id @mailbox_id.
 */
int edgetpu_mailbox_enable_ext(struct edgetpu_client *client, int mailbox_id,
			       struct edgetpu_external_mailbox_req *ext_mailbox_req,
			       u32 client_priv);

/*
 * Notify firmware of an external mailboxes becoming inactive.
 */
int edgetpu_mailbox_disable_ext(struct edgetpu_client *client, int mailbox_id);

/*
 * Activates all mailboxes included in @mailbox_map, OPEN_DEVICE KCI will be sent.
 *
 * Returns what edgetpu_kci_open_device() returned.
 * Caller ensures device is powered on.
 */
int edgetpu_mailbox_activate_bulk(struct edgetpu_dev *etdev, u32 mailbox_map, u32 client_priv,
				  s16 vcid, bool first_open);

/*
 * Activates @mailbox_id, OPEN_DEVICE KCI will be sent.
 *
 * If @mailbox_id is known to be activated, KCI is not sent and this function
 * returns 0.
 *
 * Returns what edgetpu_kci_open_device() returned.
 * Caller ensures device is powered on.
 */
int edgetpu_mailbox_activate(struct edgetpu_dev *etdev, u32 mailbox_id, u32 client_priv, s16 vcid,
			     bool first_open);

/*
 * Similar to edgetpu_mailbox_activate_bulk() but sends CLOSE_DEVICE KCI with the @mailbox_map
 * instead.
 */
void edgetpu_mailbox_deactivate_bulk(struct edgetpu_dev *etdev, u32 mailbox_map);

/*
 * Similar to edgetpu_mailbox_activate() but sends CLOSE_DEVICE KCI instead.
 */
void edgetpu_mailbox_deactivate(struct edgetpu_dev *etdev, u32 mailbox_id);

/* Sets @eh->fw_state to 0. */
void edgetpu_handshake_clear_fw_state(struct edgetpu_handshake *eh);
/*
 * Disables and frees any external mailboxes allocated for @group.
 *
 * Caller must hold @group->lock and ensure if device is accessible.
 */
void edgetpu_mailbox_external_disable_free_locked(struct edgetpu_device_group *group);

/*
 * Re-programs the CSRs of queue addresses, context, priority etc. to @group's
 * external mailbox.
 *
 * Caller holds @group->lock and ensures @group has mailbox attached.
 */
void edgetpu_mailbox_reinit_external_mailbox(struct edgetpu_device_group *group);

/*
 * Activates external mailboxes in @group's ext_mailbox.
 *
 * Caller ensures device is powered and must hold @group->lock.
 */
int edgetpu_mailbox_activate_external_mailbox(struct edgetpu_device_group *group);

/*
 * Deactivates external mailboxes in @group's ext_mailbox.
 *
 * Caller ensures device is powered and must hold @group->lock.
 */
void edgetpu_mailbox_deactivate_external_mailbox(struct edgetpu_device_group *group);

/*
 * Disables external mailboxes in @group's ext_mailbox.
 *
 * Caller ensures device is powered and must hold @group->lock.
 */
void edgetpu_mailbox_disable_external_mailbox(struct edgetpu_device_group *group);

/* Utilities of circular queue operations */

/*
 * Returns the number of elements in a circular queue given its @head, @tail,
 * and @queue_size.
 */
static inline u32 circular_queue_count(u32 head, u32 tail, u32 queue_size)
{
	u32 ret;

	if (CIRCULAR_QUEUE_WRAPPED(tail) != CIRCULAR_QUEUE_WRAPPED(head))
		ret = queue_size - CIRCULAR_QUEUE_REAL_INDEX(head) +
		      CIRCULAR_QUEUE_REAL_INDEX(tail);
	else
		ret = tail - head;

	if (unlikely(ret > queue_size))
		return 0;

	return ret;
}

/* Increases @index of a circular queue by @inc. */
static inline u32 circular_queue_inc(u32 index, u32 inc, u32 queue_size)
{
	u32 new_index = CIRCULAR_QUEUE_REAL_INDEX(index) + inc;

	if (unlikely(new_index >= queue_size))
		return (index + inc - queue_size) ^ CIRCULAR_QUEUE_WRAP_BIT;
	else
		return index + inc;
}

/* Macros for accessing mailbox CSRs. */

/* Read mailbox register with no memory barrier / access ordering guarantee. */
#define EDGETPU_MAILBOX_READ(mailbox, base, type, field) \
	edgetpu_dev_read_32(mailbox->etdev, base + offsetof(type, field))

/*
 * Read mailbox register with memory barrier, ensuring the register read
 * completes prior to any following CPU reads by this thread.
 */
#define EDGETPU_MAILBOX_READ_SYNC(mailbox, base, type, field) \
	edgetpu_dev_read_32_sync(mailbox->etdev, base + offsetof(type, field))

#define EDGETPU_MAILBOX_CONTEXT_READ(mailbox, field) \
	EDGETPU_MAILBOX_READ(mailbox, mailbox->context_csr_base, \
			     struct edgetpu_mailbox_context_csr, field)

#define EDGETPU_MAILBOX_CMD_QUEUE_READ(mailbox, field) \
	EDGETPU_MAILBOX_READ(mailbox, mailbox->cmd_queue_csr_base, \
			     struct edgetpu_mailbox_cmd_queue_csr, field)

/* Read response queue register, no memory barrier / access ordering. */
#define EDGETPU_MAILBOX_RESP_QUEUE_READ(mailbox, field) \
	EDGETPU_MAILBOX_READ(mailbox, mailbox->resp_queue_csr_base, \
			     struct edgetpu_mailbox_resp_queue_csr, field)

/* Read response queue register with memory barrier. */
#define EDGETPU_MAILBOX_RESP_QUEUE_READ_SYNC(mailbox, field) \
	EDGETPU_MAILBOX_READ_SYNC(mailbox, mailbox->resp_queue_csr_base, \
			     struct edgetpu_mailbox_resp_queue_csr, field)

/* Write mailbox register with no memory barrier / access ordering guarantee. */
#define EDGETPU_MAILBOX_WRITE(mailbox, base, type, field, value) \
	edgetpu_dev_write_32(mailbox->etdev, base + offsetof(type, field), \
			     value)

/*
 * Write mailbox register with memory barrier, ensuring all CPU memory writes
 * by this thread complete prior to the register write.
 */
#define EDGETPU_MAILBOX_WRITE_SYNC(mailbox, base, type, field, value) \
	edgetpu_dev_write_32_sync(mailbox->etdev, \
				  base + offsetof(type, field), \
				  value)

/* Write context register with no memory barrier / access ordering. */
#define EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE(mailbox, mailbox->context_csr_base, \
			      struct edgetpu_mailbox_context_csr, field, value)

/* Write context register with memory barrier. */
#define EDGETPU_MAILBOX_CONTEXT_WRITE_SYNC(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE_SYNC(mailbox, mailbox->context_csr_base, \
				   struct edgetpu_mailbox_context_csr, field, \
				   value)

/* Write command queue register with no memory barrier / access ordering. */
#define EDGETPU_MAILBOX_CMD_QUEUE_WRITE(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE(mailbox, mailbox->cmd_queue_csr_base, \
			      struct edgetpu_mailbox_cmd_queue_csr, field, \
			      value)

/* Write command queue register with memory barrier. */
#define EDGETPU_MAILBOX_CMD_QUEUE_WRITE_SYNC(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE_SYNC(mailbox, mailbox->cmd_queue_csr_base, \
				   struct edgetpu_mailbox_cmd_queue_csr, \
				   field, value)

#define EDGETPU_MAILBOX_RESP_QUEUE_WRITE(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE(mailbox, mailbox->resp_queue_csr_base, \
			      struct edgetpu_mailbox_resp_queue_csr, field, \
			      value)

/* Enables a mailbox by setting CSR. */
static inline void edgetpu_mailbox_enable(struct edgetpu_mailbox *mailbox)
{
	EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, context_enable, 1);
}

/* Disables a mailbox by setting CSR. */
static inline void edgetpu_mailbox_disable(struct edgetpu_mailbox *mailbox)
{
	EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, context_enable, 0);
}

#endif /* __EDGETPU_MAILBOX_H__ */
