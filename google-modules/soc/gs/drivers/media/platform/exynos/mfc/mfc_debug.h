/*
 * drivers/media/platform/exynos/mfc/mfc_debug.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_DEBUG_H
#define __MFC_DEBUG_H __FILE__

#include "mfc_memlog.h"

#define DEBUG

#ifdef DEBUG

extern unsigned int debug_level;
extern unsigned int debug_ts;
extern unsigned int debug_mode_en;
extern unsigned int dbg_enable;
extern unsigned int nal_q_dump;
extern unsigned int nal_q_disable;
extern unsigned int nal_q_parallel_disable;
extern unsigned int otf_dump;
extern unsigned int sfr_dump;
extern unsigned int llc_disable;
extern unsigned int slc_disable;
extern unsigned int slc_option;
extern unsigned int slc_partial_height_ratio;
extern unsigned int perf_boost_mode;
extern unsigned int drm_predict_disable;
extern unsigned int reg_test;
extern unsigned int meminfo_enable;
extern unsigned int memlog_level;
extern unsigned int logging_option;
extern unsigned int feature_option;
extern unsigned int regression_option;
extern unsigned int core_balance;
extern unsigned int sbwc_disable;
extern unsigned int sscd_report;
extern unsigned int hdr_dump;
extern unsigned int idle_suspend_enable;

#define mfc_debug(level, fmt, args...)					\
	do {								\
		if ((logging_option & MFC_LOGGING_PRINTK)		\
				&& (debug_level >= level))		\
			dev_info(ctx->dev->device, "[c:%d] %s:%d: " fmt,\
				ctx->num, __func__, __LINE__, ##args);	\
									\
		if ((ctx->dev->memlog.log_enable)			\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF)	\
			&& (memlog_level >= level))			\
			memlog_write_printf(ctx->dev->memlog.log_obj,	\
				MEMLOG_LEVEL_INFO,			\
				"[DEBUG][c:%d] %s:%d: " fmt,		\
				ctx->num, __func__, __LINE__, ##args);	\
	} while (0)

#define mfc_core_debug(level, fmt, args...)				\
	do {								\
		if ((logging_option & MFC_LOGGING_PRINTK)		\
				&& (debug_level >= level))		\
			dev_info(core->device, "%s:%d: " fmt,		\
				__func__, __LINE__, ##args);		\
									\
		if ((core->dev->memlog.log_enable)			\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF)	\
			&& (memlog_level >= level))			\
			memlog_write_printf(core->dev->memlog.log_obj,	\
				MEMLOG_LEVEL_INFO,			\
				"[DEBUG][%s]%s:%d: " fmt,		\
				core->name, __func__, __LINE__, ##args);\
	} while (0)

#define mfc_dev_debug(level, fmt, args...)				\
	do {								\
		if ((logging_option & MFC_LOGGING_PRINTK)		\
				&& (debug_level >= level))		\
			dev_info(dev->device, "%s:%d: " fmt,		\
				__func__, __LINE__, ##args);		\
									\
		if ((dev->memlog.log_enable)				\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF)	\
			&& (memlog_level >= level))			\
			memlog_write_printf(dev->memlog.log_obj,	\
				MEMLOG_LEVEL_INFO,			\
				"[DEBUG]%s:%d: " fmt,			\
				__func__, __LINE__, ##args);		\
	} while (0)

#else
#define mfc_debug(fmt, args...)
#define mfc_core_debug(fmt, args...)
#define mfc_dev_debug(fmt, args...)
#endif

#define mfc_debug_enter() mfc_debug(5, "enter\n")
#define mfc_debug_leave() mfc_debug(5, "leave\n")
#define mfc_core_debug_enter() mfc_core_debug(5, "enter\n")
#define mfc_core_debug_leave() mfc_core_debug(5, "leave\n")
#define mfc_dev_debug_enter() mfc_dev_debug(5, "enter\n")
#define mfc_dev_debug_leave() mfc_dev_debug(5, "leave\n")

/* ERROR */
#define mfc_pr_err(fmt, args...)					\
	do {								\
		if (logging_option & MFC_LOGGING_PRINTK)		\
			pr_err("[Exynos][MFC][ ERROR]: %s:%d: " fmt,    \
				__func__, __LINE__, ##args);		\
	} while (0)

#define mfc_dev_err(fmt, args...)				\
	do {							\
		if (logging_option & MFC_LOGGING_PRINTK)	\
			dev_err(dev->device, "%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
								\
		if ((dev->memlog.log_enable)			\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF))	\
			memlog_write_printf(dev->memlog.log_obj,\
				MEMLOG_LEVEL_ERR,		\
				"[ERROR]%s:%d: " fmt,		\
				__func__, __LINE__, ##args);	\
	} while (0)

#define mfc_core_err(fmt, args...)				\
	do {							\
		if (logging_option & MFC_LOGGING_PRINTK)	\
			dev_err(core->device, "%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
								\
		if ((core->dev->memlog.log_enable)		\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF))	\
			memlog_write_printf(core->dev->memlog.log_obj,\
				MEMLOG_LEVEL_ERR,		\
				"[ERROR][%s]%s:%d: " fmt,		\
				core->name, __func__, __LINE__, ##args);\
	} while (0)

#define mfc_ctx_err(fmt, args...)				\
	do {							\
		if (logging_option & MFC_LOGGING_PRINTK)	\
			dev_err(ctx->dev->device,		\
				"[c:%d] %s:%d: " fmt,		\
			ctx->num, __func__, __LINE__, ##args);	\
								\
		if ((ctx->dev->memlog.log_enable)		\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF))	\
			memlog_write_printf(ctx->dev->memlog.log_obj,\
				MEMLOG_LEVEL_ERR,		\
				"[ERROR][c:%d] %s:%d: " fmt,		\
				ctx->num, __func__, __LINE__, ##args);	\
	} while (0)

#define mfc_err(fmt, args...)							\
	do {									\
		if (logging_option & MFC_LOGGING_PRINTK)			\
			dev_err(core_ctx->core->device,				\
				"[c:%d] %s:%d: " fmt,				\
			core_ctx->num, __func__, __LINE__, ##args);		\
										\
		if ((core_ctx->core->dev->memlog.log_enable)			\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF))	\
			memlog_write_printf(core_ctx->core->dev->memlog.log_obj,\
				MEMLOG_LEVEL_ERR,				\
				"[ERROR][c:%d] %s:%d: " fmt,			\
				core_ctx->num, __func__, __LINE__, ##args);	\
	} while (0)

#define mfc_dev_info(fmt, args...)				\
	do {							\
		if (logging_option & MFC_LOGGING_PRINTK)	\
			dev_info(dev->device, "%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
								\
		if ((dev->memlog.log_enable)			\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF))	\
			memlog_write_printf(dev->memlog.log_obj,\
				MEMLOG_LEVEL_CAUTION,		\
				"[INFO ]%s:%d: " fmt,		\
				__func__, __LINE__, ##args);	\
	} while (0)

#define mfc_core_info(fmt, args...)				\
	do {							\
		if (logging_option & MFC_LOGGING_PRINTK)	\
			dev_info(core->device, "%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
								\
		if ((core->dev->memlog.log_enable)			\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF))\
			memlog_write_printf(core->dev->memlog.log_obj,	\
				MEMLOG_LEVEL_CAUTION,		\
				"[INFO ][%s]%s:%d: " fmt,		\
				core->name, __func__, __LINE__, ##args);\
	} while (0)


#define mfc_ctx_info(fmt, args...)				\
	do {							\
		if (logging_option & MFC_LOGGING_PRINTK)	\
			dev_info(ctx->dev->device,		\
				"[c:%d] %s:%d: " fmt,		\
				ctx->num, __func__, __LINE__, ##args);	\
								\
		if ((ctx->dev->memlog.log_enable)		\
			&& (logging_option & MFC_LOGGING_MEMLOG_PRINTF))	\
			memlog_write_printf(ctx->dev->memlog.log_obj,\
				MEMLOG_LEVEL_CAUTION,		\
				"[INFO ][c:%d] %s:%d: " fmt,	\
				ctx->num, __func__, __LINE__, ##args);	\
	} while (0)

#define MFC_TRACE_STR_LEN		80
#define MFC_TRACE_COUNT_MAX		1024
#define MFC_TRACE_COUNT_PRINT		30
#define MFC_TRACE_COUNT_PRINT_LONG	100
#define MFC_TRACE_LOG_STR_LEN		25
#define MFC_TRACE_LOG_COUNT_MAX		256
#define MFC_TRACE_LOG_COUNT_PRINT	20
#define MFC_TRACE_NAL_QUEUE_PRINT	15

#define MFC_DUMP_BUF_SIZE		0x600000

struct _mfc_trace {
	unsigned long long time;
	char str[MFC_TRACE_STR_LEN];
};

struct _mfc_trace_logging {
	unsigned long long time;
	char str[MFC_TRACE_LOG_STR_LEN];
};

/* If there is no core structure */
#define MFC_TRACE_DEV(fmt, args...)								\
	do {											\
		int cpu = raw_smp_processor_id();						\
		int cnt;									\
		cnt = atomic_inc_return(&dev->trace_ref) & (MFC_TRACE_COUNT_MAX - 1);		\
		dev->mfc_trace[cnt].time = cpu_clock(cpu);					\
		snprintf(dev->mfc_trace[cnt].str, MFC_TRACE_STR_LEN,				\
				fmt, ##args);				\
	} while (0)

/* If there is core structure */
#define MFC_TRACE_CORE(fmt, args...)								\
	do {											\
		int cpu = raw_smp_processor_id();						\
		int cnt;									\
		cnt = atomic_inc_return(&core->dev->trace_ref) & (MFC_TRACE_COUNT_MAX - 1);		\
		core->dev->mfc_trace[cnt].time = cpu_clock(cpu);					\
		snprintf(core->dev->mfc_trace[cnt].str, MFC_TRACE_STR_LEN,				\
				"[MFC%d] "fmt, core->id, ##args);				\
	} while (0)


/* If there is ctx structure */
#define MFC_TRACE_CTX(fmt, args...)								\
	do {											\
		int cpu = raw_smp_processor_id();						\
		int cnt;									\
		cnt = atomic_inc_return(&ctx->dev->trace_ref) & (MFC_TRACE_COUNT_MAX - 1);		\
		ctx->dev->mfc_trace[cnt].time = cpu_clock(cpu);					\
		snprintf(ctx->dev->mfc_trace[cnt].str, MFC_TRACE_STR_LEN,				\
				"[c:%d] " fmt, ctx->num, ##args);				\
	} while (0)

/* If there is core_ctx structure */
#define MFC_TRACE_CORE_CTX(fmt, args...)							\
	do {											\
		int cpu = raw_smp_processor_id();						\
		int cnt;									\
		cnt = atomic_inc_return(&core_ctx->core->dev->trace_ref) & (MFC_TRACE_COUNT_MAX - 1);\
		core_ctx->core->dev->mfc_trace[cnt].time = cpu_clock(cpu);				\
		snprintf(core_ctx->core->dev->mfc_trace[cnt].str, MFC_TRACE_STR_LEN,			\
				"[MFC%d][c:%d] " fmt, core_ctx->core->id,				\
				core_ctx->num, ##args);						\
	} while (0)


/* If there is no ctx structure */
#define MFC_TRACE_DEV_LT(fmt, args...)							\
	do {											\
		int cpu = raw_smp_processor_id();						\
		int cnt;									\
		cnt = atomic_inc_return(&dev->trace_ref_longterm) & (MFC_TRACE_COUNT_MAX - 1);	\
		dev->mfc_trace_longterm[cnt].time = cpu_clock(cpu);				\
		snprintf(dev->mfc_trace_longterm[cnt].str, MFC_TRACE_STR_LEN,			\
				fmt, ##args);							\
	} while (0)

/* If there is ctx structure */
#define MFC_TRACE_CTX_LT(fmt, args...)							\
	do {											\
		int cpu = raw_smp_processor_id();						\
		int cnt;									\
		cnt = atomic_inc_return(&ctx->dev->trace_ref_longterm) & (MFC_TRACE_COUNT_MAX - 1);	\
		ctx->dev->mfc_trace_longterm[cnt].time = cpu_clock(cpu);				\
		snprintf(ctx->dev->mfc_trace_longterm[cnt].str, MFC_TRACE_STR_LEN,			\
				"[c:%d] " fmt, ctx->num, ##args);				\
	} while (0)

/* If there is core structure */
#define MFC_TRACE_LOG_CORE(fmt, args...)							\
	do {											\
		int cpu = raw_smp_processor_id();						\
		int cnt;									\
		cnt = atomic_inc_return(&core->trace_ref_log) & (MFC_TRACE_LOG_COUNT_MAX - 1);	\
		core->mfc_trace_logging[cnt].time = cpu_clock(cpu);				\
		snprintf(core->mfc_trace_logging[cnt].str, MFC_TRACE_LOG_STR_LEN,		\
				fmt, ##args);							\
	} while (0)

/* Resource manager dedicated */
#define MFC_TRACE_RM(fmt, args...)								\
	do {											\
		int cpu = raw_smp_processor_id();						\
		int cnt;									\
		cnt = atomic_inc_return(&dev->trace_ref_rm) & (MFC_TRACE_COUNT_MAX - 1);	\
		dev->mfc_trace_rm[cnt].time = cpu_clock(cpu);					\
		snprintf(dev->mfc_trace_rm[cnt].str, MFC_TRACE_STR_LEN,				\
				fmt, ##args);				\
	} while (0)

#endif /* __MFC_DEBUG_H */
