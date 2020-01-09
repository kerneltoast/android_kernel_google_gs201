/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright 2020 Google LLC
 *
 * Author: yulgon.kim@samsung.com
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#ifndef CONFIG_GS_ACPM_MODULE
#include "acpm_power_stats.h"
#else
struct power_stats_buffer;
#endif

/**
 * struct ipc_cmd_info - RX command buffer info for framework and plugins
 *
 * @rx_cmd:		pointer to the RX buffer entry to be dequeued.
 * @rx_cmd_indr:	pointer to the RX indirection buffer entry to be dequeued.
 */
struct ipc_cmd_info {
	u32 rx_cmd;
	u32 rx_cmd_indr;
};

/**
 * struct dbg_log_info - log buffer entry format
 */
struct dbg_log_info {
	char str[8];
	u32 val;
	u32 log_level;
};

/*
 * First MAJOR version to support "buffers" field in plugin_ops
 */
#define BUILD_INFO_MAJOR_BUFFERS 1

/**
 * struct build_info
 *
 * @build_version:	build version including builder, last char must be 0.
 * @major:		major version of API
 *				if major >= BUILD_INFO_MAJOR_BUFFERS then
 *					plugin_ops contains "buffers" field.
 * @minor:		minor version of API
 */
struct build_info {
	char build_version[48];
	char major;
	char minor;
};

/**
 * struct acpm_ops - framework callbacks to be provided to plugins
 *
 * @set_timer_event:	adds timer request which expires at every 'msec'.
 * @del_timer_event:	removes the existing timer request.
 * @insert_dbg_log:	inserts a log entry at predefined log buffer region.
 * @get_tx_dest:	does full check of tx queue and returns ptr to enqueue.
 * @enqueue_tx:		updated tx_front ptr of tx queue and generates mbox irq.
 * @get_total_size:	returns size_of(framework + all plugins)
 */
struct acpm_ops {
	s32 (*set_timer_event)(u32 plugin_id, u32 msec);
	void (*del_timer_event)(u32 plugin_id);
	void (*insert_dbg_log)(u32 plugin_id, struct dbg_log_info *log);
	u32 (*get_tx_dest)(u32 ch_num, u32 *index);
	void (*enqueue_tx)(u32 ch_num, u32 index);
	u32 (*get_total_size)(void);
	s32 (*secure_func)(void *plugin, u32 arg0, u32 arg1,
			   u32 arg2, u32 arg3);
	s32 (*external_plugin_func)(void *plugin, u32 pid,
				    u32 *arg0, u32 *arg1, u32 *arg2);
	s32 (*speedy_init)(void);
#ifdef CONFIG_MULTI_PMIC
	s32 (*speedy_read)(u8 channel, u32 addr);
	s32 (*speedy_write)(u8 channel, u32 addr, u32 data);
#else
	s32 (*speedy_read)(u32 addr);
	s32 (*speedy_write)(u32 addr, u32 data);
#endif
	void (*udelay)(u32 udelay);
	void (*intr_enable)(u32 pid, u32 intr);
	void (*intr_disable)(u32 pid, u32 intr);
	void (*preempt_disable_irq_save)(u32 *flag);
	void (*preempt_enable_irq_restore)(u32 *flag);
	void (*print)(u32 id, const char *s, u32 int_data, u32 level);
	struct power_stats_buffer *(*get_power_stats_buffer)(void);
};

/**
 * struct plugin_buffer - List of buffer exported from a plugin
 *
 * @size:	buffer size.
 * @address:	buffer address relative to APM SRAM start.
 * @next:	next plugin_buffer, address relative to APM SRAM start.
 * @name:	Name of the buffer, must be unique in the binary.
 */
struct plugin_buffer {
	u32 size;
	u32 address;
#ifndef CONFIG_GS_ACPM_MODULE
	const struct plugin_buffer *next;
#else
	u32 next;
#endif
	char name[12];
};

/**
 * struct plugin_ops - plugin callbacks to be provided to framework
 *
 * @ipc_handler:	handler to be executed when ipc for this plugin is arrived.
 * @irq_handler:	handler to be executed when hw irq for this plugin is arrived.
 * @timer_event_handler:handler to be executed when requested timer is expired.
 * @buffers:		buffers list, only exist
 *				if info.major >= BUILD_INFO_MAJOR_BUFFERS
 *			and info.build_version[47] == 0
 */
struct plugin_ops {
#ifndef CONFIG_GS_ACPM_MODULE
	s32 (*ipc_handler)(struct ipc_cmd_info *cmd, u32 ch_num);
	s32 (*irq_handler)(u32 intr);
	s32 (*timer_event_handler)(void);
	s32 (*extern_func)(u32 *arg0, u32 *arg1, u32 *arg2);
#else
	u32 ipc_handler;
	u32 irq_handler;
	u32 timer_event_handler;
	u32 extern_func;
#endif
	struct build_info info;
#ifndef CONFIG_GS_ACPM_MODULE
	const struct plugin_buffer *buffers;
#else
	const u32 buffers;
#endif
};

/**
 * struct timer_desc - A descriptor for timer request
 *
 * @period:		requested period that framework executes timer_event_handler.
 * @multiplier:
 */
struct timer_desc {
	u32 period;
	u32 multiplier;
};

/**
 * struct plugin - The basic plugin structure
 *
 * @id:			Predefined id for this plugin.
 * @base_addr:		Predefined base addr for this plugin. (entrypoint)
 * @acpm_ops:		Framework callbacks.
 * @plugin_ops:		Plugin callbacks.
 * @timer:		Timer descriptor for this plugin.
 * @is_attached:	For dynamic plugin support.
 * @size:		The size of this plugin.
 */
struct plugin {
	u32 id;
#ifndef CONFIG_GS_ACPM_MODULE
	void *base_addr;
	struct acpm_ops *acpm_ops;
	struct plugin_ops *plugin_ops;
#else
	u32 base_addr;
	u32 acpm_ops;
	u32 plugin_ops;
#endif
	u32 secure_func_mask;
	u32 extern_func_mask;
	struct timer_desc timer;
	u8 is_attached;
	u32 size;
	u8 stay_attached;
#ifndef CONFIG_GS_ACPM_MODULE
	const char *fw_name;
#else
	u32 fw_name;
#endif
};

typedef void (*pfn_plugin_init)(struct plugin *p);

enum ret_type {
	RET_OK,
	RET_FAIL,
};
#endif
