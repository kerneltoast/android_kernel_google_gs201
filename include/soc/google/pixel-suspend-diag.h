/* SPDX-License-Identifier: GPL-2.0 */
/*
 * pixel_suspend_diag.h - Get suspend diag information
 *
 * Copyright 2023 Google LLC
 */

#ifndef SUSPEND_DIAG_H
#define SUSPEND_DIAG_H

void *pixel_suspend_diag_get_info(void);
void pixel_suspend_diag_suspend_resume(void *dbg_snapshot_log, const char *action, bool start,
				       uint64_t curr_index);
bool pixel_suspend_diag_dev_pm_cb_end(void *dbg_snapshot_log, uint64_t first_log_idx,
				      uint64_t last_log_idx, struct device *dev);

#endif /* SUSPEND_DIAG_H */
