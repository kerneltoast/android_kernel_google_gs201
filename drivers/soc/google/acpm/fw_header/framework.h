/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright 2020 Google LLC
 *
 */

#ifndef __FRAMEWORK_H_
#define __FRAMEWORK_H_

#ifdef CONFIG_GS_ACPM_MODULE
#include "common.h"
#endif

/**
 * struct acpm_framework - General information for ACPM framework.
 *
 * @plugins:		Pointer to soc-specific plugins array.
 * @pid_framework:	Plugin ID for ACPM framework.
 * @pid_max:		# of plugins.
 * @ipc_max:		# of IPC channels.
 * @irq_max:		# of interrupt sources.
 * @ktime_index:	ktime information from the kernel.
 * @log_buf_rear:	Rear pointer of the log buffer.
 * @log_buf_front:	Front pointer of the log buffer.
 * @log_data:		Base address of the log buffer.
 * @log_entry_size:	Entry size of the log buffer.
 * @log_entry_len:	Length of the log buffer.
 * @ipc_base:		Base address of the IPC buffer.
 * @ipc_buf_tx_size:	Size of the IPC TX buffer.
 * @total_size:		Size of the framework and all plugins attached.
 * @fw_size:		Size of the ACPM framework.
 * @intr_to_skip:	Disabled interrupts.
 * @preemption_irq:	preemptive interrupts.
 * @intr_flag_offset:	Field offset for Mailbox interrupt pending register.
 */
struct acpm_framework {
#ifndef CONFIG_GS_ACPM_MODULE
	struct plugin *plugins;
#else
	u32 plugins;
#endif
	u32 num_plugins;
#ifndef CONFIG_GS_ACPM_MODULE
	struct ipc_channel *ipc_channels;
#else
	u32 ipc_channels;
#endif
	u32 num_ipc_channels;
	u32 pid_framework;
	u32 pid_max;
	u32 ipc_ap_max;
	u32 ipc_cp_max;
	u32 ipc_gnss_max;
	u32 ipc_wlbt_max;
	u32 ipc_max;
	u32 irq_max;
	u32 ktime_index;
	u32 log_buf_rear;
	u32 log_buf_front;
	u32 log_data;
	u32 log_entry_size;
	u32 log_entry_len;
	u32 ipc_base;
	u32 ipc_buf_tx_size;
	u32 fw_size;
	u32 intr_to_skip;
	u32 intr_flag_offset;
	struct build_info info;
	u32 preempt_irq_max;
	u32 nvic_max;
	u32 intr_stat;
	u32 intr_stat1;
	u32 preemption_irq;
	u32 preempt_plugin;
	u32 preempt_log_buf_rear;
	u32 preempt_log_buf_front;
	u32 preempt_log_data;
	u32 preempt_log_entry_len;
	u32 fvmap;
	u32 reserved[62];
	u32 err_log_async_channel;
};

/**
 * struct channel_info - IPC information of a channel
 *
 * @rx_rear:		Rear ptr for RX cmd queue. (for dequeue)
 * @rx_front:		Front ptr for RX cmd queue. (just for empty check)
 * @rx_base:		Predefined base addr for RX cmd queue.
 * @rx_indr_buf:	Predefined base addr for RX indirect buffer.
 * @tx_rear:		Rear ptr for TX queue.
 * @tx_front:		Front ptr for TX queue.
 * @tx_base:		Predefined base addr for TX queue.
 * @q_len:		Length of both TX/RX queue.
 * @q_elem_size:	Element size of both TX/RX queue.
 * @credit:		For preventing starvation of certain plugin.
 */
struct channel_info {
	u32 rx_rear;
	u32 rx_front;
	u32 rx_base;
	u32 rx_indr_buf;
	u32 rx_indr_buf_base;
	u32 rx_indr_buf_size;
	u32 tx_rear;
	u32 tx_front;
	u32 tx_base;
	u32 q_len;
	u32 q_elem_size;
	u32 credit;
	u32 mbox_addr;
};

/**
 * struct ipc_channel - descriptor for ipc channel.
 *
 * @id:			The ipc channel's id.
 * @field:		The ipc channel's field in mailbox status register.
 * @late_init_source: Use of this channel broadcast late_init_ipc
 * @owner:		This interrupt's Belonged plugin.
 * @type:		The ipc channel's memory type; QUEUE or Register.
 * @ch:			IPC information for this plugin's channel.
 * @ap_poll:		The flag indicating whether AP polls on this channel or not. (interrupt)
 */
struct ipc_channel {
	u32 id;
	u32 field:30; // Only 16 are manage by HW
	u32 late_init_source:1;
	u32 use_sacpm:1;
	s32 owner;
	u32 type;
	struct channel_info ch;
	u32 ap_poll;
};

/**
 * struct acpm_interrupt - descriptor for individual interrupt source.
 *
 * @field:		The interrupt's field in NVIC pending register.
 * @handler:		Handler for this interrupt.
 * @owner:		This interrupt's Belonged plugin.
 * @is_disabled:	Flag to skip this interrupt or not.
 */
struct acpm_interrupt {
	u32 field;
	s32 (*handler)(u32 intr);
	s32 owner;
	bool is_disabled;
	bool log_skip;
	bool idle_ip_skip;
	bool is_preemption;
};

/**
 * struct apm_peri - The abstraction for APM peripherals.
 *
 * @mpu_init:		Initializes Core's MPU. (Regions, ...)
 * @enable_systick:	Enables System Tick.
 * @disable_systick:	Disables System Tick.
 * @get_systick_cvr:	Get the current value register of System Tick.
 * @get_systick_icvr:	Get (MAX_VAL - CUR_VAL) of System Tick.
 * @get_systick_icvra:	Get (MAX_VAL - CUR_VAL) & MAX_VAL of System Tick.
 * @get_systick_csr:	Get the current status register of System Tick.
 * @enable_wdt:		Enables Watchdog Timer.
 * @disable_wdt:	Disables Watchdog Timer.
 * @enable_intr:	Enables specific interrupts on NVIC.
 * @get_enabled_intr:	Get enabled interrupts on NVIC.
 * @clr_pend_intr:	Clears pended interrupts on NVIC.
 * @get_pend_intr:	Get Pended interrupts on NVIC.
 * @get_mbox_pend_intr:	Get Pended interrupts on Mailbox.
 * @clr_mbox_pend_intr:	Clears pended interrupts on Mailbox.
 * @gen_mbox_intr_ap:	Generates Mailbox interrupt to AP.
 */
struct apm_peri {
	void (*power_mode_init)(void);
	void (*peri_timer_init)(void);
	void (*set_peri_timer_event)(u32 msec);
	void (*del_peri_timer_event)(void);
	u32 (*get_peri_timer_cvr)(void);
	u32 (*get_peri_timer_icvr)(void);
	u32 (*get_peri_timer_icvra)(void);
	void (*mpu_init)(void);
	void (*enable_systick)(void);
	void (*disable_systick)(void);
	u32 (*get_systick_cvr)(void);
	u32 (*get_systick_icvr)(void);
	u32 (*get_systick_icvra)(void);
	u32 (*get_systick_csr)(void);
	void (*enable_wdt)(void);
	void (*disable_wdt)(void);
	void (*enable_intr)(u32 idx, u32 intr);
	void (*disable_intr)(u32 idx, u32 intr);
	u32 (*get_enabled_intr)(u32 idx);
	void (*clr_pend_intr)(u32 idx, u32 intr);
	u32 (*get_active_intr)(u32 idx);
	u32 (*get_pend_intr)(u32 idx);
	u32 (*get_mbox_pend_intr)(u32 mbox_addr);
	void (*clr_mbox_pend_intr)(u32 mbox_addr, u32 intr);
	void (*gen_mbox_intr)(struct ipc_channel *ipc_ch);
	u32 (*set_idle_ip)(u32 is_idle, u32 intr_filed);
	s32 (*udelay_systick)(u32 udelay_us);
	void (*set_wdtrst_req)(void);
	void (*uart_write_str)(const char *str);
	void (*bl2_release)(void);
};

extern struct apm_peri apm_peri;

/**
 * struct plat_ops - Getters for platform specific data to ACPM framework.
 *
 * @plugin_base_init:		initializes base address of plugins.
 * @get_acpm_platform_data:	inits the remaining data.
 * @get_target_plugin:		returns next plugin to be executed.
 */
struct plat_ops {
	void (*plugin_base_init)(struct acpm_framework *acpm);
	void (*get_acpm_platform_data)(struct acpm_framework *acpm);
	u32 (*get_target_plugin)(u32 intr, void *acpm_data, u32 int_status, u32 *ch_num);
	void (*ipc_channel_init)(struct acpm_framework *acpm);
	void *(*get_plugins_extern_fn)(void);
	u32 (*get_mbox_addr)(u32 intr);
	u32 (*filter_buff_int_status)(u32 intr, u32 int_status);
	u32 (*filter_queue_int_status)(u32 intr, u32 int_status);
	void (*wait_for_intr)(void);
	u32 (*system_reset)(void);
	void (*preemption_enable)(void);
	void (*preemption_disable)(void);
	void (*restore_intr)(void);
	void (*preemption_disable_irq_save)(u32 *flag);
	void (*preemption_enable_irq_restore)(u32 *flag);
};

extern struct plat_ops plat_ops;

s32 shard_queue_ipc_dispatcher(u32 intr);
s32 shard_buffer_ipc_dispatcher(u32 intr);
s32 dummy_irq_handler(u32 intr);
void acpm_insert_dbg_log(u32 plugin_id, struct dbg_log_info *log);
char *acpm_strcpy(char *dest, const char *src);
extern void acpm_print(u32 id, const char *s, u32 int_data, u32 level);
extern void *get_intr_vector(u32 *irq_max);
extern void *get_preempt_intr_vector(u32 *irq_max);
extern u32 get_nvic_max(void);
extern struct acpm_framework acpm;

enum shard_buffer_type {
	TYPE_QUEUE = 1,
	TYPE_BUFFER,
};

#define LOG_ID_SHIFT			(28)
#define LOG_TIME_INDEX			(20)
#define LOG_LEVEL			(19)
#define AVAILABLE_TIME			(26000)

#define ACPM_DEBUG_PRINT

#ifdef ACPM_DEBUG_PRINT
#define ACPM_PRINT_ERR(id, s, int_data)	acpm_print(id, s, int_data, 0xF)
#define ACPM_PRINT(id, s, int_data)	acpm_print(id, s, int_data, 0x1)
#else
#define ACPM_PRINT_ERR(id, s, int_data)	do {} while (0)
#define ACPM_PRINT(a, b, c) do {} while (0)
#endif

#define true				(1)
#define false				(0)

/* IPC Protocol bit field definitions */
#define IPC_PRTC_OWN			(31)
#define IPC_PRTC_RSP			(30)
#define IPC_PRTC_INDR			(29)
#define IPC_PRTC_ID			(26)
#define IPC_PRTC_IDX			(0x7 << IPC_PRTC_ID)
#define IPC_PRTC_DP_ATTACH		(25)
#define IPC_PRTC_DP_DETACH		(24)
#define IPC_PRTC_DP_CMD			((1 << IPC_PRTC_DP_ATTACH) | (1 << IPC_PRTC_DP_DETACH))
#define IPC_PRTC_TEST			(23)
#define IPC_PRTC_STOP			(22)

#define ERR_NOPLUGIN			(10)
#define ERR_REJECTED			(11)
#define ERR_NOTYET			(12)

/* speedy definition */
struct speedyops {
	int (*init)(void);
#ifdef CONFIG_MULTI_PMIC
	int (*tx)(u8 channel, unsigned int reg, unsigned int val);
	int (*rx)(u8 channel, unsigned int reg);
#else
	int (*tx)(unsigned int reg, unsigned int val);
	int (*rx)(unsigned int reg);
#endif
};

extern struct speedyops speedyops;

#ifdef CONFIG_GS_ACPM_MODULE
/**
 * acpm_get_buffer - Get an ACPM named buffer. This handle old ACPM cases.
 *
 * @sram:	Pointer to ACPM sram.
 * @acpm:	Pointer to ACPM struct.
 * @name:	Name of the buffer we are looking for.
 * @addr:	On success, the address of the buffer we are looking for.
 * @size:	On success, the size of the buffer we are looking for.
 *
 * @return:	0 success (buffer found)
 *		-1 failed (buffer not found)
 */
static inline int acpm_get_buffer(char *sram, const struct acpm_framework *acpm,
				  const char *name, char **addr, u32 *size)
{
	struct plugin *plugins = (struct plugin *)(sram + acpm->plugins);
	u32 i;

	for (i = 0; i < acpm->num_plugins; i++) {
		const u32 *next;
		struct plugin_ops *plugin_ops;

		if (!plugins[i].plugin_ops)
			continue; /* No plugin attached */
		plugin_ops =
			(struct plugin_ops *)(sram + plugins[i].plugin_ops);
		if (plugin_ops->info.build_version
			    [sizeof(plugin_ops->info.build_version) - 1] != 0) {
			/*
			 * Old acpm has info.build_version[25],build_time[25]
			 * instead of info.build_version[48] and info.major,
			 * info.minor.
			 * 1) info.build_time[22] ==
			 *		new info.build_version[47] != 0
			 *    It's an old version and the string is too big
			 *    to fit in 23 char.
			 * 2) info.build_time[22] ==
			 *		new info.build_version[47] == 0
			 *    If it's an old version and the string fit in
			 *    23 char
			 *    In that case info.build_time[23] == 0, so
			 *    info.major == 0.
			 * 3) info.build_time[22] ==
			 *		new info.build_version[47] == 0
			 *    If it's a new version, info.major > 0
			 */
			continue; /* Old ACPM not supporting version */
		}
		if (plugin_ops->info.major < BUILD_INFO_MAJOR_BUFFERS)
			continue; /* No buffers in this struct */

		next = &plugin_ops->buffers;
		while (*next) {
			u32 i = 0;
			struct plugin_buffer *buffer =
				(struct plugin_buffer *)(sram + *next);
			while ((i < sizeof(buffer->name)) &&
			       (buffer->name[i] == name[i]) &&
			       (buffer->name[i] != 0)) {
				i++;
			}
			if (i < sizeof(buffer->name) &&
			    buffer->name[i] != name[i]) {
				next = &buffer->next;
				continue;
			}
			*addr = sram + buffer->address;
			*size = buffer->size;
			return 0;
		}
	}
	return -1;
}
#endif

#define speedy_init()			speedyops.init()
#ifdef CONFIG_MULTI_PMIC
#define speedy_tx(channel, reg, val)	speedyops.tx(channel, reg, val)
#define speedy_rx(channel, reg)		speedyops.rx(channel, reg)
#else
#define speedy_tx(reg, val)		speedyops.tx(reg, val)
#define speedy_rx(reg)			speedyops.rx(reg)
#endif

#define MAGIC				0x0BAD0BAD

#endif
