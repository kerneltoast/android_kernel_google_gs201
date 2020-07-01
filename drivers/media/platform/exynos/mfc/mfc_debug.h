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
extern unsigned int mmcache_dump;
extern unsigned int mmcache_disable;
extern unsigned int llc_disable;
extern unsigned int slc_disable;
extern unsigned int perf_boost_mode;
extern unsigned int drm_predict_disable;
extern unsigned int reg_test;
extern unsigned int feature_option;
extern unsigned int regression_option;

#define mfc_debug(level, fmt, args...)				\
	do {							\
		if (debug_level >= level)				\
			dev_info(ctx->dev->device, "%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
	} while (0)

#define mfc_dev_debug(level, fmt, args...)				\
	do {							\
		if (debug_level >= level)				\
			dev_info(dev->device, "%s:%d: " fmt,	\
				__func__, __LINE__, ##args);	\
	} while (0)
#else
#define mfc_debug(fmt, args...)
#define mfc_dev_debug(fmt, args...)
#endif

#define mfc_debug_enter() mfc_debug(5, "enter\n")
#define mfc_debug_leave() mfc_debug(5, "leave\n")

#define mfc_dev_debug_enter() mfc_dev_debug(5, "enter\n")
#define mfc_dev_debug_leave() mfc_dev_debug(5, "leave\n")

#define mfc_err(fmt, args...)				\
	pr_err("[Exynos][MFC][ ERROR]: %s:%d: " fmt,	\
		       __func__, __LINE__, ##args)

#define mfc_dev_err(fmt, args...)			\
	dev_err(dev->device, "%s:%d: " fmt,		\
		       __func__, __LINE__, ##args)

#define mfc_ctx_err(fmt, args...)			\
	dev_err(ctx->dev->device, "[c:%d] %s:%d: " fmt,	\
			ctx->num,			\
		       __func__, __LINE__, ##args)

#define mfc_dev_info(fmt, args...)			\
	dev_info(dev->device, "%s:%d: " fmt,		\
			__func__, __LINE__, ##args)

#define mfc_ctx_info(fmt, args...)			\
	dev_info(ctx->dev->device, "[c:%d] %s:%d: " fmt,\
			ctx->num,			\
			__func__, __LINE__, ##args)

#define MFC_TRACE_STR_LEN		80
#define MFC_TRACE_COUNT_MAX		1024
#define MFC_TRACE_COUNT_PRINT		30
#define MFC_TRACE_LOG_STR_LEN		25
#define MFC_TRACE_LOG_COUNT_MAX		256
#define MFC_TRACE_LOG_COUNT_PRINT	20
#define MFC_TRACE_NAL_QUEUE_PRINT	25


struct _mfc_trace {
	unsigned long long time;
	char str[MFC_TRACE_STR_LEN];
};

struct _mfc_trace_logging {
	unsigned long long time;
	char str[MFC_TRACE_LOG_STR_LEN];
};

/* If there is no ctx structure */
#define MFC_TRACE_DEV(fmt, args...)					\
	do {								\
		int cpu = raw_smp_processor_id();			\
		int cnt;						\
		cnt = atomic_inc_return(&dev->trace_ref) &		\
			(MFC_TRACE_COUNT_MAX - 1);			\
		dev->mfc_trace[cnt].time = cpu_clock(cpu);		\
		snprintf(dev->mfc_trace[cnt].str, MFC_TRACE_STR_LEN,	\
				fmt, ##args);				\
	} while (0)

/* If there is ctx structure */
#define MFC_TRACE_CTX(fmt, args...)					\
	do {								\
		int cpu = raw_smp_processor_id();			\
		int cnt;						\
		cnt = atomic_inc_return(&ctx->dev->trace_ref) &		\
			(MFC_TRACE_COUNT_MAX - 1);			\
		ctx->dev->mfc_trace[cnt].time = cpu_clock(cpu);		\
		snprintf(ctx->dev->mfc_trace[cnt].str,			\
				MFC_TRACE_STR_LEN,			\
				"[c:%d] " fmt, ctx->num, ##args);	\
	} while (0)


/* If there is no ctx structure */
#define MFC_TRACE_DEV_LT(fmt, args...)					\
	do {								\
		int cpu = raw_smp_processor_id();			\
		int cnt;						\
		cnt = atomic_inc_return(&dev->trace_ref_longterm) &	\
			(MFC_TRACE_COUNT_MAX - 1);			\
		dev->mfc_trace_longterm[cnt].time = cpu_clock(cpu);	\
		snprintf(dev->mfc_trace_longterm[cnt].str,		\
				MFC_TRACE_STR_LEN, fmt, ##args);	\
	} while (0)

/* If there is ctx structure */
#define MFC_TRACE_CTX_LT(fmt, args...)					\
	do {								\
		int cpu = raw_smp_processor_id();			\
		int cnt;						\
		cnt = atomic_inc_return(&ctx->dev->trace_ref_longterm) &\
			(MFC_TRACE_COUNT_MAX - 1);			\
		ctx->dev->mfc_trace_longterm[cnt].time = cpu_clock(cpu);\
		snprintf(ctx->dev->mfc_trace_longterm[cnt].str,		\
				MFC_TRACE_STR_LEN,			\
				"[c:%d] " fmt, ctx->num, ##args);	\
	} while (0)

/* If there is no ctx structure */
#define MFC_TRACE_LOG_DEV(fmt, args...)					\
	do {								\
		int cpu = raw_smp_processor_id();			\
		int cnt;						\
		cnt = atomic_inc_return(&dev->trace_ref_log) &		\
			(MFC_TRACE_LOG_COUNT_MAX - 1);			\
		dev->mfc_trace_logging[cnt].time = cpu_clock(cpu);	\
		snprintf(dev->mfc_trace_logging[cnt].str,		\
				MFC_TRACE_LOG_STR_LEN, fmt, ##args);	\
	} while (0)

/* If there is ctx structure */
#define MFC_TRACE_LOG_CTX(fmt, args...)					\
	do {								\
		int cpu = raw_smp_processor_id();			\
		int cnt;						\
		cnt = atomic_inc_return(&dev->trace_ref_log) &		\
			(MFC_TRACE_LOG_COUNT_MAX - 1);			\
		ctx->dev->mfc_trace_logging[cnt].time = cpu_clock(cpu);	\
		snprintf(ctx->dev->mfc_trace_logging[cnt].str,		\
				MFC_TRACE_LOG_STR_LEN,			\
				"%d:" fmt, ctx->num, ##args);		\
	} while (0)


#endif /* __MFC_DEBUG_H */
