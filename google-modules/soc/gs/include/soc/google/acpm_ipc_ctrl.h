/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Header for ACPM_IPC.
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#ifndef __ACPM_IPC_CTRL_H__
#define __ACPM_IPC_CTRL_H__

typedef void (*ipc_callback)(unsigned int *cmd, unsigned int size);

struct ipc_config {
	int *cmd;
	unsigned int *indirection_base;
	unsigned int indirection_size;
	bool response;
};

#define ACPM_IPC_PROTOCOL_OWN			(31)
#define ACPM_IPC_PROTOCOL_RSP			(30)
#define ACPM_IPC_PROTOCOL_INDIRECTION		(29)
#define ACPM_IPC_PROTOCOL_ID			(26)
#define ACPM_IPC_PROTOCOL_IDX			(0x7 << ACPM_IPC_PROTOCOL_ID)
#define ACPM_IPC_PROTOCOL_DP_ATTACH		(25)
#define ACPM_IPC_PROTOCOL_DP_DETACH		(24)
#define ACPM_IPC_PROTOCOL_DP_CMD		((1 << IPC_PROTOCOL_DP_ATTACH)|\
						 (1 << IPC_PROTOCOL_DP_DETACH))
#define ACPM_IPC_PROTOCOL_TEST			(23)
#define ACPM_IPC_PROTOCOL_STOP			(22)
#define ACPM_IPC_PROTOCOL_SEQ_NUM		(16)
#define ACPM_IPC_PROTOCOL_SETTINGS		(13) /* 15-14 used as arg */

/* 4 types of settings requests can be made */
enum acpm_print_settings {
	ACPM_SET_UART_GPRIO_LEVEL,
	ACPM_SET_LOGB_GPRIO_LEVEL,
	ACPM_GET_UART_GPRIO_LEVEL,
	ACPM_GET_LOGB_GPRIO_LEVEL,
};

#define ACPM_FRAMEWORK_COMMAND_DEBUG	0x1

enum acpm_framework_debug_commands {
	ACPM_FRAMEWORK_COMMAND_DEBUG_DISABLE_WATCHDOG,
	ACPM_FRAMEWORK_COMMAND_DEBUG_ENABLE_WATCHDOG,
	ACPM_FRAMEWORK_COMMAND_DEBUG_SOFT_LOCKUP,
	ACPM_FRAMEWORK_COMMAND_DEBUG_HARD_LOCKUP,
	ACPM_FRAMEWORK_COMMAND_DEBUG_EXCEPTION,
	ACPM_FRAMEWORK_COMMAND_DEBUG_NOTIFY_SHUTDOWN,
	ACPM_FRAMEWORK_COMMAND_DEBUG_RAMDUMP_ON,
	ACPM_FRAMEWORK_COMMAND_DEBUG_MAX,
};

#if IS_ENABLED(CONFIG_GS_ACPM)
int acpm_ipc_request_channel(struct device_node *np,
			     ipc_callback handler,
			     unsigned int *id, unsigned int *size);
int acpm_ipc_release_channel(struct device_node *np,
			     unsigned int channel_id);
int acpm_ipc_send_data(unsigned int channel_id,
		       struct ipc_config *cfg);
int acpm_ipc_send_data_sync(unsigned int channel_id,
			    struct ipc_config *cfg);
int acpm_ipc_send_data_lazy(unsigned int channel_id,
			    struct ipc_config *cfg);
int acpm_ipc_set_ch_mode(struct device_node *np, bool polling);
int acpm_ipc_get_buffer(const char *name, char **addr, u32 *size);
void exynos_acpm_reboot(void);
void acpm_stop_log_and_dumpram(void);
u64 get_frc_time(void);
bool is_acpm_ipc_flushed(void);
bool acpm_ipc_get_rx_buffer_properties(unsigned int channel_id, void __iomem **base,
				       unsigned int *size);
bool acpm_ipc_get_tx_buffer_properties(unsigned int channel_id, void __iomem **base,
				       unsigned int *size);
void acpm_ipc_ring_doorbell(unsigned int channel_id);
#else

static inline int acpm_ipc_request_channel(struct device_node *np,
		ipc_callback handler,
		unsigned int *id, unsigned int *size)
{
	return 0;
}

static inline int acpm_ipc_release_channel(struct device_node *np,
		unsigned int channel_id)
{
	return 0;
}

static inline int acpm_ipc_send_data(unsigned int channel_id,
		struct ipc_config *cfg)
{
	return 0;
}

static inline int acpm_ipc_send_data_sync(unsigned int channel_id,
		struct ipc_config *cfg)
{
	return 0;
}

static inline int acpm_ipc_send_data_lazy(unsigned int channel_id,
		struct ipc_config *cfg)
{
	return 0;
}

static inline int acpm_ipc_set_ch_mode(struct device_node *np, bool polling)
{
	return 0;
}

static inline int acpm_ipc_get_buffer(const char *name, char **addr, u32 *size)
{
	return -1;
}

static inline void exynos_acpm_reboot(void)
{
}

static inline void acpm_stop_log_and_dumpram(void)
{
}

static inline u64 get_frc_time(void)
{
	return 0;
}

static inline bool is_acpm_ipc_flushed(void)
{
	return true;
}
#endif

#endif
