/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Implements utilities for virtual device group of EdgeTPU.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_DEVICE_GROUP_H__
#define __EDGETPU_DEVICE_GROUP_H__

#include <linux/eventfd.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/refcount.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "edgetpu-dram.h"
#include "edgetpu-internal.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-mapping.h"
#include "edgetpu-mmu.h"
#include "edgetpu.h"

/* entry of edgetpu_device_group#clients */
struct edgetpu_list_group_client {
	struct list_head list;
	struct edgetpu_client *client;
};

enum edgetpu_device_group_status {
	/* Waiting for new clients to join. */
	EDGETPU_DEVICE_GROUP_WAITING,
	/* Most operations can only apply on a finalized group. */
	EDGETPU_DEVICE_GROUP_FINALIZED,
	/*
	 * When a fatal error occurs, groups in FINALIZED status are transformed
	 * into this state. Operations on groups with this status mostly return
	 * ECANCELED. Once a member leaves an ERRORED group, the status is
	 * transitioned to DISBANDED.
	 */
	EDGETPU_DEVICE_GROUP_ERRORED,
	/* No operations except member leaving can be performed. */
	EDGETPU_DEVICE_GROUP_DISBANDED,
};

#define EDGETPU_EVENT_COUNT 2

/* eventfds registered for event notifications from kernel for a device group */
struct edgetpu_events {
	rwlock_t lock;
	struct eventfd_ctx *eventfds[EDGETPU_EVENT_COUNT];
};

struct edgetpu_device_group {
	/*
	 * Reference count.
	 * edgetpu_device_group_get() increases the counter by one and
	 * edgetpu_device_group_put() decreases it. This object will be freed
	 * when ref_count becomes zero.
	 */
	refcount_t ref_count;
	uint workload_id;
	struct edgetpu_dev *etdev;	/* the device opened by the leader */
	/*
	 * Whether mailbox attaching and detaching have effects on this group.
	 * This field is configured according to the priority field when
	 * creating this group.
	 */
	bool mailbox_detachable;
	/*
	 * Whether group->etdev is inaccessible.
	 * Some group operations will access device CSRs. If the device is known to be
	 * inaccessible (typically not powered on) then set this field to true to
	 * prevent HW interactions.
	 *
	 * This field is always false for !EDGETPU_HAS_WAKELOCK chipsets.
	 *
	 * For EDGETPU_HAS_MCP chipsets this field should be replaced with a
	 * boolean array with size @n_clients, but we don't have a chipset with
	 * EDGETPU_HAS_MCP && EDGETPU_HAS_WAKELOCK yet.
	 *
	 * Is not protected by @lock because this is only written when releasing the
	 * leader of this group.
	 */
	bool dev_inaccessible;
	/* Virtual context ID to be sent to the firmware. */
	u16 vcid;

	/* protects everything in the following comment block */
	struct mutex lock;
	/* fields protected by @lock */

	/*
	 * List of clients belonging to this group.
	 * The first client is the leader.
	 */
	struct list_head clients;
	uint n_clients;			/* number of clients in the list */
	/*
	 * Array of the clients belonging to this group.
	 * Clients in this field are same as @clients, but this field is
	 * allocated only when a group is finalized. See
	 * edgetpu_device_group_nth_etdev() for more details.
	 */
	struct edgetpu_client **members;
	enum edgetpu_device_group_status status;
	bool activated; /* whether this group's VII has ever been activated */
	struct edgetpu_vii vii;		/* VII mailbox */
	/*
	 * Context ID ranges from EDGETPU_CONTEXT_VII_BASE to
	 * EDGETPU_NCONTEXTS - 1.
	 * This equals EDGETPU_CONTEXT_INVALID or a token OR'ed with
	 * EDGETPU_CONTEXT_DOMAIN_TOKEN when the group has mailbox detached
	 * (means the group isn't in any context at this time).
	 */
	enum edgetpu_context_id context_id;
	/* The IOMMU domain being associated to this group */
	struct edgetpu_iommu_domain *etdomain;
	/* matrix of P2P mailboxes */
	struct edgetpu_p2p_mailbox **p2p_mailbox_matrix;
	/*
	 * External mailboxes associated with this group, only valid if
	 * external mailbox allocated and enabled.
	 */
	struct edgetpu_external_mailbox *ext_mailbox;

	/* Mask of errors set for this group. */
	uint fatal_errors;

	/* List of DMA fences owned by this group */
	struct list_head dma_fence_list;

	/* end of fields protected by @lock */

	/* TPU IOVA mapped to host DRAM space */
	struct edgetpu_mapping_root host_mappings;
	/* TPU IOVA mapped to buffers backed by dma-buf */
	struct edgetpu_mapping_root dmabuf_mappings;
	struct edgetpu_events events;
	/* Mailbox attributes used to create this group */
	struct edgetpu_mailbox_attr mbox_attr;
};

/*
 * Entry of edgetpu_dev#groups.
 *
 * Files other than edgetpu-device-group.c shouldn't need to access this
 * structure. Use macro etdev_for_each_group to access the groups under an
 * etdev.
 */
struct edgetpu_list_group {
	struct list_head list;
	struct edgetpu_device_group *grp;
};

/* Macro to loop through etdev->groups. */
#define etdev_for_each_group(etdev, l, g)                                      \
	for (l = list_entry(etdev->groups.next, typeof(*l), list), g = l->grp; \
	     &l->list != &etdev->groups;                                       \
	     l = list_entry(l->list.next, typeof(*l), list), g = l->grp)

/* Loop through group->clients (hold group->lock prior). */
#define for_each_list_group_client(c, group) \
	list_for_each_entry(c, &group->clients, list)

/*
 * Returns if the group is waiting for members to join.
 *
 * Caller holds @group->lock.
 */
static inline bool
edgetpu_device_group_is_waiting(const struct edgetpu_device_group *group)
{
	return group->status == EDGETPU_DEVICE_GROUP_WAITING;
}

/*
 * Returns if the group is finalized.
 *
 * Caller holds @group->lock.
 */
static inline bool
edgetpu_device_group_is_finalized(const struct edgetpu_device_group *group)
{
	return group->status == EDGETPU_DEVICE_GROUP_FINALIZED;
}

/*
 * Returns if the group is errored.
 *
 * Caller holds @group->lock.
 */
static inline bool
edgetpu_device_group_is_errored(const struct edgetpu_device_group *group)
{
	return group->status == EDGETPU_DEVICE_GROUP_ERRORED;
}

/*
 * Returns if the group is disbanded.
 *
 * Caller holds @group->lock.
 */
static inline bool
edgetpu_device_group_is_disbanded(const struct edgetpu_device_group *group)
{
	return group->status == EDGETPU_DEVICE_GROUP_DISBANDED;
}

/*
 * Return fatal error status for the group.
 *
 * Caller holds @group->lock.
 */
static inline uint edgetpu_group_get_fatal_errors_locked(struct edgetpu_device_group *group)
{
	return group->fatal_errors;
}

/*
 * Returns -ECANCELED if the status of group is ERRORED, otherwise returns -EINVAL.
 *
 * Caller holds @group->lock.
 */
static inline int edgetpu_group_errno(struct edgetpu_device_group *group)
{
	if (edgetpu_device_group_is_errored(group)) {
		etdev_err(group->etdev, "group %u error status 0x%x\n", group->workload_id,
			  edgetpu_group_get_fatal_errors_locked(group));
		return -ECANCELED;
	}
	return -EINVAL;
}

/* Increases ref_count of @group by one and returns @group. */
static inline struct edgetpu_device_group *
edgetpu_device_group_get(struct edgetpu_device_group *group)
{
	WARN_ON_ONCE(!refcount_inc_not_zero(&group->ref_count));
	return group;
}

/*
 * Decreases ref_count of @group by one.
 *
 * If @group->ref_count becomes 0, @group will be freed.
 */
void edgetpu_device_group_put(struct edgetpu_device_group *group);

/*
 * Allocates a device group with @client as the group leader.
 *
 * @client must not already belong to (either as a leader or a member) another
 * group. @client->group will be set as the returned group on success.
 *
 * Call edgetpu_device_group_put() when the returned group is not needed.
 *
 * Returns allocated group, or a negative errno on error.
 * Returns -EINVAL if the client already belongs to a group.
 */
struct edgetpu_device_group *
edgetpu_device_group_alloc(struct edgetpu_client *client,
			   const struct edgetpu_mailbox_attr *attr);

/*
 * Adds a client to the device group.
 *
 * @group->ref_count will be increased by two on success:
 * - @client->group will be set to @group.
 * - @group will be added to @client->etdev->groups.
 *
 * @client must not already belong to another group, otherwise -EINVAL is
 * returned.
 *
 * Returns 0 on success, or a negative errno on error.
 */
int edgetpu_device_group_add(struct edgetpu_device_group *group,
			     struct edgetpu_client *client);

/*
 * Returns the edgetpu_dev opened by the @n-th client in this group, 0-based.
 *
 * This function returns NULL if "and only if" @group is not finalized or @n is
 * invalid.
 *
 * Caller holds the group lock.
 *
 * Returns the pointer to the edgetpu_dev.
 */
static inline struct edgetpu_dev *edgetpu_device_group_nth_etdev(
		struct edgetpu_device_group *group, uint n)
{
	if (!group->members || n >= group->n_clients)
		return NULL;
	return group->members[n]->etdev;
}

/*
 * Let @client leave the group it belongs to.
 *
 * If @client is the leader of a group, the group will be marked as "disbanded".
 *
 * If @client is a member of a group, we have two cases depending on whether the
 * group is in status EDGETPU_DEVICE_GROUP_WAITING:
 * 1. If the group is waiting for members to join, this function simply removes
 *    @client from the group, and new members still can join the group.
 * 2. Otherwise, the group will be marked as "disbanded", no operations except
 *    members leaving can apply to the group.
 *
 * @client->group will be removed from @client->etdev->groups.
 * @client->group will be set as NULL.
 */
void edgetpu_device_group_leave(struct edgetpu_client *client);

/* Returns whether @client is the leader of @group. */
bool edgetpu_device_group_is_leader(struct edgetpu_device_group *group,
				    const struct edgetpu_client *client);

/*
 * Finalizes the group.
 *
 * A finalized group is not allowed to add new members.
 *
 * Returns 0 on success.
 * Returns -EINVAL if the group is not waiting for new members to join.
 */
int edgetpu_device_group_finalize(struct edgetpu_device_group *group);

/*
 * Maps buffer to a device group.
 *
 * @arg->device_address will be set as the mapped TPU VA on success.
 *
 * Returns zero on success or a negative errno on error.
 */
int edgetpu_device_group_map(struct edgetpu_device_group *group,
			     struct edgetpu_map_ioctl *arg);

/* Unmap a userspace buffer from a device group. */
int edgetpu_device_group_unmap(struct edgetpu_device_group *group,
			       u32 die_index, tpu_addr_t tpu_addr,
			       edgetpu_map_flag_t flags);

/* Sync the buffer previously mapped by edgetpu_device_group_map. */
int edgetpu_device_group_sync_buffer(struct edgetpu_device_group *group,
				     const struct edgetpu_sync_ioctl *arg);

/* Clear all mappings for a device group. */
void edgetpu_mappings_clear_group(struct edgetpu_device_group *group);

/* Return total size of all mappings for the group in bytes */
size_t edgetpu_group_mappings_total_size(struct edgetpu_device_group *group);

/*
 * Return context ID for group MMU mappings.
 *
 * Caller holds @group->lock to prevent race, the context ID may be changed by
 * edgetpu_group_{detach/attach}_mailbox.
 */
static inline enum edgetpu_context_id
edgetpu_group_context_id_locked(struct edgetpu_device_group *group)
{
	return group->context_id;
}

/* dump mappings in @group */
void edgetpu_group_mappings_show(struct edgetpu_device_group *group,
				 struct seq_file *s);

/*
 * Maps the VII mailbox CSR.
 *
 * Returns 0 on success.
 */
int edgetpu_mmap_csr(struct edgetpu_device_group *group,
		     struct vm_area_struct *vma, bool is_external);
/*
 * Maps the cmd/resp queue memory.
 *
 * Returns 0 on success.
 */
int edgetpu_mmap_queue(struct edgetpu_device_group *group,
		       enum mailbox_queue_type type,
		       struct vm_area_struct *vma, bool is_external);

/* Set group eventfd for event notification */
int edgetpu_group_set_eventfd(struct edgetpu_device_group *group, uint event_id,
			      int eventfd);

/* Unset previously-set group eventfd */
void edgetpu_group_unset_eventfd(struct edgetpu_device_group *group,
				 uint event_id);

/* Notify group of event */
void edgetpu_group_notify(struct edgetpu_device_group *group, uint event_id);

/* Is device in any group (and may be actively processing requests) */
bool edgetpu_in_any_group(struct edgetpu_dev *etdev);

/*
 * Enable or disable device group join lockout (as during f/w load).
 * Returns false if attempting to lockout group join but device is already
 * joined to a group.
 */
bool edgetpu_set_group_join_lockout(struct edgetpu_dev *etdev, bool lockout);

/* Notify @group about a fatal error for that group. */
void edgetpu_group_fatal_error_notify(struct edgetpu_device_group *group,
				      uint error_mask);
/* Notify all device groups of @etdev about a failure on the die */
void edgetpu_fatal_error_notify(struct edgetpu_dev *etdev, uint error_mask);

/* Return fatal error signaled bitmask for device group */
uint edgetpu_group_get_fatal_errors(struct edgetpu_device_group *group);

/*
 * Detach and release the mailbox resources of VII from @group.
 * Some group operations would be disabled when a group has no mailbox attached.
 *
 * Caller holds @group->lock.
 */
void edgetpu_group_detach_mailbox_locked(struct edgetpu_device_group *group);
/*
 * Before detaching the mailbox, send CLOSE_DEVICE KCI that claims the mailbox
 * is going to be unused.
 *
 * The KCI command is sent even when @group is configured as mailbox
 * non-detachable.
 */
void edgetpu_group_close_and_detach_mailbox(struct edgetpu_device_group *group);
/*
 * Request and attach the mailbox resources of VII to @group.
 *
 * Return 0 on success.
 *
 * Caller holds @group->lock.
 */
int edgetpu_group_attach_mailbox_locked(struct edgetpu_device_group *group);
/*
 * After (successfully) attaching the mailbox, send OPEN_DEVICE KCI.
 *
 * The KCI command is sent even when @group is configured as mailbox
 * non-detachable (because the mailbox was successfully "attached").
 */
int edgetpu_group_attach_and_open_mailbox(struct edgetpu_device_group *group);

/*
 * Checks whether @group has mailbox detached.
 *
 * Caller holds @group->lock.
 */
static inline bool
edgetpu_group_mailbox_detached_locked(const struct edgetpu_device_group *group)
{
	return group->context_id == EDGETPU_CONTEXT_INVALID ||
	       group->context_id & EDGETPU_CONTEXT_DOMAIN_TOKEN;
}

/*
 * Returns whether @group is finalized and has mailbox attached.
 *
 * Caller holds @group->lock.
 */
static inline bool
edgetpu_group_finalized_and_attached(const struct edgetpu_device_group *group)
{
	return edgetpu_device_group_is_finalized(group) &&
	       !edgetpu_group_mailbox_detached_locked(group);
}

#endif /* __EDGETPU_DEVICE_GROUP_H__ */
