/* SPDX-License-Identifier: GPL-2.0 */
/*
 * SideBand Bit Multiplexer (SBBM).
 *
 * Copyright (C) 2020 Google, Inc.
 */

#ifndef __SBBM_H__
#define __SBBM_H__

#if IS_ENABLED(CONFIG_SBB_MUX)

/*
 * The list of signals supported by the sbb-mux driver.
 * All enum values need to have a matching element in the 'signals' sbb_signal
 * array defined in drivers/misc/sbb-mux/sbb-mux.h.
 * Add new signals at the end, just before SBB_SIG_NUM_SIGNALS.
 */
enum sbbm_signal_id {
	SBB_SIG_ZERO = 0, /* Constant value of 0, for testing purposes. */
	SBB_SIG_ONE, /* Constant value of 1, for testing purposes. */
	SBB_SIG_KERNEL_TEST, /* Test signal for kernel API. */
	SBB_SIG_USERLAND_TEST, /* Test signal for sysfs-driven changes. */
	SBB_SIG_USERLAND_GP0, /* Userland general purpose area of interest. */
	SBB_SIG_USERLAND_GP1, /* Userland general purpose area of interest. */
	SBB_SIG_USERLAND_GP2, /* Userland general purpose area of interest. */
	SBB_SIG_USERLAND_GP3, /* Userland general purpose area of interest. */
	SBB_SIG_KERNEL_GP0, /* Kernel general purpose area of interest. */
	SBB_SIG_KERNEL_GP1, /* Kernel general purpose area of interest. */
	SBB_SIG_KERNEL_GP2, /* Kernel general purpose area of interest. */
	SBB_SIG_KERNEL_GP3, /* Kernel general purpose area of interest. */
	SBB_SIG_APP_MISC0, /* Miscellaneous app area of interest. */
	SBB_SIG_APP_MISC1, /* Miscellaneous app area of interest. */
	SBB_SIG_APP_MISC2, /* Miscellaneous app area of interest. */
	SBB_SIG_APP_MISC3, /* Miscellaneous app area of interest. */
	SBB_SIG_APP_MISC4, /* Miscellaneous app area of interest. */
	SBB_SIG_APP_MISC5, /* Miscellaneous app area of interest. */
	SBB_SIG_APP_MISC6, /* Miscellaneous app area of interest. */
	SBB_SIG_APP_MISC7, /* Miscellaneous app area of interest. */
	SBB_SIG_APP_MISC8, /* Miscellaneous app area of interest. */
	SBB_SIG_APP_MISC9, /* Miscellaneous app area of interest. */
	SBB_SIG_CAMERA_MISC0, /* Miscellaneous camera area of interest. */
	SBB_SIG_CAMERA_MISC1, /* Miscellaneous camera area of interest. */
	SBB_SIG_CAMERA_MISC2, /* Miscellaneous camera area of interest. */
	SBB_SIG_CAMERA_MISC3, /* Miscellaneous camera area of interest. */
	SBB_SIG_CAMERA_MISC4, /* Miscellaneous camera area of interest. */
	SBB_SIG_CAMERA_MISC5, /* Miscellaneous camera area of interest. */
	SBB_SIG_CAMERA_MISC6, /* Miscellaneous camera area of interest. */
	SBB_SIG_CAMERA_MISC7, /* Miscellaneous camera area of interest. */
	SBB_SIG_CAMERA_MISC8, /* Miscellaneous camera area of interest. */
	SBB_SIG_CAMERA_MISC9, /* Miscellaneous camera area of interest. */
	SBB_SIG_UFS_ACTIVE_IDLE, /* UFS is active/idle and out of h8 tracking. */
	SBB_SIG_UFS_POWER_ON, /* UFS powered on tracking. */
	SBB_SIG_UFS_IO_OUTSTANDING, /* UFS I/O in progress tracking. */
	SBB_SIG_MODEM_CP2AP_WAKE_ISR, /* Modem needs attention from AP */
	SBB_SIG_PCIE_LINK_STATE, /* Track PCIe link state L2/out of L2 */
	SBB_SIG_NUM_SIGNALS /* The total number of signals. Has to be last. */
};

extern int sbbm_signal_update(enum sbbm_signal_id signal_id, bool value);

#define SBBM_SIGNAL_UPDATE(signal_id, val) sbbm_signal_update(signal_id, val)

#else

#define SBBM_SIGNAL_UPDATE(signal_id, val) (0)

#endif /* CONFIG_SBB_MUX */

#endif /* __SBBM_H__ */
