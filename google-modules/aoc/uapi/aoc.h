/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */
/*
 * Google Whitechapel AoC Core Driver
 *
 * Copyright (c) 2020 Google LLC
 *
 */

#define AOC_IOCTL_MAGIC 0xac

#define AOC_IS_ONLINE _IOR(AOC_IOCTL_MAGIC, 5, int)

struct aoc_ion_handle {
	__u32 handle;
	__s32 fd;
};

#define AOC_IOCTL_ION_FD_TO_HANDLE _IOWR(AOC_IOCTL_MAGIC, 204, struct aoc_ion_handle)
#define AOC_IOCTL_DISABLE_MM _IOW(AOC_IOCTL_MAGIC, 205, __u32)
#define AOC_IOCTL_FORCE_VNOM _IOW(AOC_IOCTL_MAGIC, 206, __u32)
#define AOC_IOCTL_ENABLE_UART_TX _IOW(AOC_IOCTL_MAGIC, 207, __u32)
#define AOC_IOCTL_DISABLE_AP_RESETS _IOW(AOC_IOCTL_MAGIC, 208, __u32)
#define AOC_IOCTL_FORCE_SPEAKER_ULTRASONIC _IOW(AOC_IOCTL_MAGIC, 209, __u32)
#define AOC_IOCTL_VOLTE_RELEASE_MIF _IOW(AOC_IOCTL_MAGIC, 210, __u32)
