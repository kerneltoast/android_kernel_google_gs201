/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This header provides constants for gs101 Secure Log.
 *
 * Copyright 2020 Google LLC
 */

#ifndef GS101_SECLOG_H
#define GS101_SECLOG_H

/*
 * NOTE
 *
 * SECLOG_NUM_OF_CPU would be changed for each SoC
 */
#define SECLOG_NUM_OF_CPU		8

/* Secure log buffer information */
#define SECLOG_LOG_BUF_BASE		0xC3000000
#define SECLOG_LOG_BUF_SIZE		0x10000
#define SECLOG_LOG_BUF_TOTAL_SIZE	(SECLOG_LOG_BUF_SIZE * SECLOG_NUM_OF_CPU)

#endif	/* GS101_SECLOG_H */

