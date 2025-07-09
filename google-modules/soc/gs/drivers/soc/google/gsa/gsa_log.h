/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022 Google LLC
 */
#ifndef __LINUX_GSA_LOG_H
#define __LINUX_GSA_LOG_H

struct gsa_log;

struct gsa_log *gsa_log_init(struct platform_device *pdev);

ssize_t gsa_log_read(struct gsa_log *log, bool intermediate, char *buf);

#endif /* __LINUX_GSA_LOG_H */
