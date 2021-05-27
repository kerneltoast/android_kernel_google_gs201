/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef EXYNOS_DEBUG_H
#define EXYNOS_DEBUG_H

#if IS_ENABLED(CONFIG_S3C2410_WATCHDOG)
extern int s3c2410wdt_set_emergency_stop(int index);
extern int s3c2410wdt_set_emergency_reset(unsigned int timeout, int index);
extern int s3c2410wdt_keepalive_emergency(bool reset, int index);
extern void s3c2410wdt_reset_confirm(unsigned long mtime, int index);
extern int s3c2410wdt_keepalive_common(void);
#else
#define s3c2410wdt_set_emergency_stop(a)	(-1)
#define s3c2410wdt_set_emergency_reset(a, b)	do { } while (0)
#define s3c2410wdt_keepalive_emergency(a, b)	do { } while (0)
#define s3c2410wdt_reset_confirm(a, b)		do { } while (0)
#define s3c2410wdt_keepalive_common()		do { } while (0)
#endif

#endif


