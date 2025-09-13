/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Interface to utilize IIF fence tables, the wait table and the signal table. Both tables will have
 * one entry per fence ID.
 *
 * - Wait table: Describes which IPs are waiting on each fence. This table will be written by the
 *               kernel driver only.
 *
 * - Signal table: Describes how many signals are remaining to unblock each fence. This table will
 *                 be initialized by the kernel driver and each signaler IP will update it.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#ifndef __IIF_IIF_FENCE_TABLE_H__
#define __IIF_IIF_FENCE_TABLE_H__

#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/types.h>

#include <iif/iif-shared.h>

#define IIF_CLEAR_LSB(b) ((b) & ((b) - 1))

/*
 * Iterates the type of IPs waiting on the fence of @fence_id.
 *
 * fence_table (input): Pointer to the fence table.
 * fence_id (input): ID of the fence to get IPs waiting on it.
 * ip (output): Type of IP waiting on the fence (enum iif_ip_type).
 * tmp (output): Temporary variable to iterate the wait table entry (int).
 */
#define for_each_waiting_ip(fence_table, fence_id, waiting_ip, tmp)                               \
	for (tmp = (fence_table)->wait_table[fence_id].waiting_ips, waiting_ip = __ffs(tmp); tmp; \
	     tmp = IIF_CLEAR_LSB(tmp), waiting_ip = __ffs(tmp))

/* The fence table which will be shared with the firmware side. */
struct iif_fence_table {
	struct iif_wait_table_entry *wait_table;
	struct iif_signal_table_entry *signal_table;
};

/*
 * Parses the fence table region from the device tree and map it to @fence_table.
 *
 * Returns 0 if succeeded. If it fails in mapping the table, returns -ENODEV.
 */
int iif_fence_table_init(const struct device_node *np, struct iif_fence_table *fence_table);

/*
 * Initializes the entry of @fence_id in the fence table.
 *
 * Since this function will be called only when the fence is initialized, we don't need any locks
 * to protect the entry.
 */
void iif_fence_table_init_fence_entry(struct iif_fence_table *fence_table, unsigned int fence_id,
				      unsigned int total_signalers);
/*
 * Sets waiting IP bit of the wait table entry of @fence_id.
 *
 * Since this function will be called by the `iif_fence_submit_waiter` function which protects the
 * entry by itself with holding its lock, we don't have to hold any locks here.
 */
void iif_fence_table_set_waiting_ip(struct iif_fence_table *fence_table, unsigned int fence_id,
				    enum iif_ip_type ip);

/*
 * Sets the number of remaining signalers to the signal table entry of @fence_id.
 *
 * This function should be called when either
 * - the signaler of the fence is AP, or
 * - the signaler is an IP but the IP is under the situation that it can't update the table by
 *   itself.
 *
 * Since this function will be called by the `iif_fence_signal{_with_status}` function which
 * protects the entry by itself with holding its lock, we don't have to hold any locks here.
 */
void iif_fence_table_set_remaining_signals(struct iif_fence_table *fence_table,
					   unsigned int fence_id, unsigned int remaining_signalers);

/*
 * Gets the number of remaining signalers from the signal table entry of @fence_id.
 *
 * This function should be called when either
 * - the signaler of the fence is AP, or
 * - the signaler is an IP but the IP is under the situation that it can't update the table by
 *   itself.
 *
 * Since this function will be called by the `iif_fence_signal{_with_status}` function which
 * protects the entry by itself with holding its lock, we don't have to hold any locks here.
 */
unsigned int iif_fence_table_get_remaining_signals(struct iif_fence_table *fence_table,
						   unsigned int fence_id);

/*
 * Sets the fence flag to the signal table entry of @fence_id.
 * See `enum iif_flag_bits` to understand meaning of each bit of @flags.
 *
 * This function should be called when either
 * - the signaler of the fence is AP, or
 * - the signaler is an IP but the IP is under the situation that it can't update the table by
 *   itself.
 *
 * Since this function will be called by the `iif_fence_signal{_with_status}` function which
 * protects the entry by itself with holding its lock, we don't have to hold any locks here.
 */
void iif_fence_table_set_flag(struct iif_fence_table *fence_table, unsigned int fence_id, u8 flag);

/*
 * Gets the fence flag from the signal table entry of @fence_id.
 *
 * This function should be called when either
 * - the signaler of the fence is AP, or
 * - the signaler is an IP but the IP is under the situation that it can't update the table by
 *   itself.
 *
 * Since this function will be called by the `iif_fence_signal{_with_status}` function which
 * protects the entry by itself with holding its lock, we don't have to hold any locks here.
 */
u8 iif_fence_table_get_flag(struct iif_fence_table *fence_table, unsigned int fence_id);

#endif /* __IIF_IIF_FENCE_TABLE_H__ */
