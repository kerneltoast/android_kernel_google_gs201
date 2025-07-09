/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Defines the data structures which can be shared by kernel drivers, firmware and user-space.
 *
 * Copyright (C) 2024 Google LLC
 */

#ifndef __IIF_SHARED_H__
#define __IIF_SHARED_H__

/*
 * This file is shared with firmware and user-space where has no Linux kernel sources. Therefore,
 * we should branch the headers according to whether we are going to build this file as kernel
 * drivers or not.
 */
#ifndef __KERNEL__
#include <stdint.h>
#else /* !__KERNEL__ */
#include <linux/types.h>
#endif /* __KERNEL__ */

#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */

/*
 * The max number of fences can be created per IP.
 * Increasing this value needs to increase the size of fence table.
 */
#define IIF_NUM_FENCES_PER_IP 1024

/* Type of IPs. */
enum iif_ip_type {
	IIF_IP_DSP,
	IIF_IP_TPU,
	IIF_IP_GPU,
	IIF_IP_AP,
	IIF_IP_NUM,

	/* Reserve the number of IP type to expand the fence table easily in the future. */
	IIF_IP_RESERVED = 16,
};

/*
 * Bit location of each IIF flag.
 * It will be set to the @flag field of the signal table per fence which has 1-byte size.
 */
enum iif_flag_bits {
	/*
	 * If this flag is set, the fence has been signaled with an error at least once.
	 * The waiters shouldn't consider the fence as unblocked until the number of remaining
	 * signals becomes 0.
	 */
	IIF_FLAG_ERROR_BIT,
};

/* Entry of the wait table. */
struct iif_wait_table_entry {
	/* The waiters waiting on the fence unblock. */
	uint8_t waiting_ips;
	/* Reserved. */
	uint8_t reserved[7];
} __packed;

/* Entry of the signal table. */
struct iif_signal_table_entry {
	/*
	 * The number of remaining signals to unblock the fence.
	 * If it becomes 0, it means that the fence has been unblocked. Note that the waiters should
	 * investigate @flag to confirm that if there was a fence error or not.
	 */
	uint16_t remaining_signals;
	/*
	 * The flag of the fence.
	 * See `enum iif_flag_bits` to understand the meaning of each bit.
	 */
	uint8_t flag;
	/* Reserved. */
	uint8_t reserved[5];
} __packed;

#endif /* __IIF_SHARED_H__ */
