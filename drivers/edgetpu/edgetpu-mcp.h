/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EdgeTPU multi-chip package management.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_MCP_H__
#define __EDGETPU_MCP_H__

#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "edgetpu-config.h"
#include "edgetpu-internal.h"

struct edgetpu_mcp {
	/* constant fields after initialization */

	u8 id;		/* the MCP ID matches etdev->mcp_id */
	u8 pkgtype;	/* package type, definition is chip-dependent */
	u8 total_num;	/* total number of etdevs expected by this MCP */
	u64 serial_num; /* serial number of the package */

	/* fields need to be locked before accessing */

	struct mutex lock;
	u8 cur_num;	/* current number of etdevs registered */
	/*
	 * Array of etdevs in this MCP, its length is @total_num.
	 *
	 * @etdevs[i] equals NULL means the etdev with mcp_die_index = i is not
	 * added yet.
	 *
	 * @etdevs[i] equals -ENODEV if that device is added but marked as
	 * failed by edgetpu_mcp_probe_fail().
	 *
	 * One should check with !IS_ERR_OR_NULL(etdevs[i]) before accessing.
	 */
	struct edgetpu_dev **etdevs;

	/* MCP-wide fatal errors pending runtime notification */
	uint errors_pending_mask;
	spinlock_t errors_pending_lock;
	struct work_struct errors_pending_work;	/* for notify via kworker */
};

#ifdef EDGETPU_HAS_MCP

/*
 * Registers @etdev to the MCP manager.
 *
 * @etdev->mcp_pkg_type, @etdev->mcp_id, and @etdev->mcp_die_index must be set
 * before this call.
 *
 * Returns 0 on success, or -errno on error.
 */
int edgetpu_mcp_add_etdev(struct edgetpu_dev *etdev);

/*
 * Init @mcp if all the dies in the mcp have been probed.
 *
 * After trying to probe each etdev on PCI, the mcp list is fully populated
 * with the corresponding etdevs. At this time the mcp list consists of
 * successfully probed *etdev and -ENODEV for etdevs that failed probe.
 * The mcp can then be initialized.
 *
 * Returns 0 on success, or -errno on error.
 */
int edgetpu_mcp_init_if_last(struct edgetpu_dev *etdev);

/*
 * Reverts edgetpu_mcp_add_etdev().
 *
 * @etdev->mcp_id and @etdev->mcp_die_index must be the same when called
 * edgetpu_mcp_add_etdev().
 */
void edgetpu_mcp_remove_etdev(struct edgetpu_dev *etdev);

/*
 * Marks etdev that failed to probe, set MCP entry to -ENODEV.
 */
void edgetpu_mcp_probe_fail(struct edgetpu_dev *etdev);

/*
 * Invokes @callback with each (currently) registered MCP.
 *
 * If @stop_on_err is true, this function stops when @callback returned non-zero
 * value. And that value is also returned by this function.
 * If @stop_on_err is false, @callback will be called exactly the number of
 * registered MCPs times, and this function will always return 0.
 *
 * @data can be any value, it will be directly passed to @callback.
 *
 * @callback is expected to return 0 on success, -errno otherwise.
 *
 * Don't call this or other edgetpu_mcp_* functions recursively to prevent dead
 * locking.
 *
 * It's @callback's responsibility to hold edgetpu_mcp's lock when access
 * non-constant fields of edgetpu_mcp.
 */
int edgetpu_mcp_each(bool stop_on_err, void *data,
		     int (*callback)(struct edgetpu_mcp *, void *));

/*
 * Invoke @callback on each device for the specified @mcp.
 *
 * @data can be any value, it will be directly passed to @callback.
 */
void edgetpu_mcp_foreach_etdev(struct edgetpu_mcp *mcp,
			       void (*callback)(struct edgetpu_dev *, void *),
			       void *data);

/*
 * Returns the MCP object @etdev belongs to.
 *
 * Returns NULL when such object is not found.
 *
 * Note: The returned pointer will be released when the last etdev is removed.
 * Don't use the returned pointer after edgetpu_mcp_remove_etdev() is called.
 */
struct edgetpu_mcp *edgetpu_mcp_of_etdev(struct edgetpu_dev *etdev);

/* Returns the next available MCP ID. */
int edgetpu_mcp_next_id(void);

/* To allocate / release structures for MCP management */
void edgetpu_mcp_init(void);
void edgetpu_mcp_exit(void);

#else /* !EDGETPU_HAS_MCP */

static inline struct edgetpu_mcp *
edgetpu_mcp_of_etdev(struct edgetpu_dev *etdev)
{
	return NULL;
}

static inline void edgetpu_mcp_init(void)
{
}

static inline void edgetpu_mcp_exit(void)
{
}

#endif /* EDGETPU_HAS_MCP */

#endif /* __EDGETPU_MCP_H__ */
