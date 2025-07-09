/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Debug Utilities
 *
 * Copyright (c) 2020 Google, LLC
 */

#include "lwis_device.h"

/* Functions to print debugging info */
int lwis_debug_print_device_info(struct lwis_device *lwis_dev);
int lwis_debug_print_event_states_info(struct lwis_device *lwis_dev, int lwis_event_dump_cnt);
int lwis_debug_print_transaction_info(struct lwis_device *lwis_dev);
int lwis_debug_print_register_io_history(struct lwis_device *lwis_dev);
int lwis_debug_print_buffer_info(struct lwis_device *lwis_dev);

/*
 * lwis_debug_crash_info_dump:
 * Use the customized function handle to print information from each device registered in LWIS
 * when usersapce crash.
 */
void lwis_debug_crash_info_dump(struct lwis_device *lwis_dev);

/* DebugFS specific functions */
int lwis_device_debugfs_setup(struct lwis_device *lwis_dev, struct dentry *dbg_root);
int lwis_device_debugfs_cleanup(struct lwis_device *lwis_dev);
