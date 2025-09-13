/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __EXYNOS_ADV_TRACER_H_
#define __EXYNOS_ADV_TRACER_H_

#include <linux/platform_device.h>

struct adv_tracer_info {
	unsigned int plugin_num;
	struct device *dev;
};

struct adv_tracer_ipc_cmd_raw {
	u32 cmd			:16;
	u32 response		:1;
	u32 overlay		:1;
	u32 ret_err		:1;
	u32 ok			:1;
	u32 busy		:1;
	u32 manual_polling	:1;
	u32 one_way		:1;
	u32 reserved		:1;
	u32 id			:4;
	u32 size		:4;
};

struct adv_tracer_ipc_cmd {
	union {
		struct adv_tracer_ipc_cmd_raw cmd_raw;
		unsigned int buffer[20];
	};
	unsigned int len;
};

struct adv_tracer_ipc_ch {
	struct list_head list;
	unsigned int id;
	unsigned int offset;
	unsigned int len;
	char id_name[4];

	struct adv_tracer_ipc_cmd *cmd;
	void __iomem *buff_regs;
	void (*ipc_callback)(struct adv_tracer_ipc_cmd *cmd, unsigned int len);

	spinlock_t ch_lock;
	struct mutex wait_lock;

	struct completion wait;
	bool polling;
	bool used;
};

struct adv_tracer_ipc_main {
	unsigned int num_channels;
	struct device *dev;
	struct adv_tracer_ipc_ch *channel;
	unsigned int irq;
	void __iomem *mailbox_base;
	void __iomem *shared_buffer;
	unsigned int mailbox_status;
	unsigned int intr_bitoffset;
	unsigned int dbgc_config;
	unsigned int dbgc_status;
	bool recovery;
};

struct adv_tracer_plugin {
	struct device_node *np;
	unsigned int id;
	unsigned int len;
	unsigned int enable;
};

typedef void (*ipc_callback)(struct adv_tracer_ipc_cmd *cmd, unsigned int size);

enum ipc_frmk_cmd {
	EAT_IPC_CMD_CH_INIT = 1,
	EAT_IPC_CMD_CH_RELEASE,
	EAT_IPC_CMD_CH_CLEAR,
	EAT_IPC_CMD_BOOT_DBGC,
	EAT_IPC_CMD_EXCEPTION_DBGC,
	EAT_IPC_CMD_IS_ATTACHED,
	EAT_IPC_CMD_CONFIG_UART = 0x101,
	EAT_IPC_CMD_CONFIG_READ_UART_MUX,
	EAT_IPC_CMD_CONFIG_WRITE_UART_MUX,
	EAT_IPC_CMD_FRM_LOAD_BINARY = 0x10ad,
	EAT_IPC_CMD_ARRAYDUMP = 0x8080,
	EAT_IPC_CMD_KERNEL_BOOT = 0xb002,
	EAT_IPC_CMD_COPY_DEBUG_LOG = 0xcb10,
	EAT_IPC_CMD_DEBUG_LOG_INFO,
	EAT_IPC_CMD_GET_NMI_INFO,
	EAT_IPC_CMD_GET_IRQ_INFO,
	EAT_IPC_CMD_APM_PING_TEST = 0xcc00,
	EAT_IPC_CMD_SLC_DUMP = 0xcc10,
	EAT_IPC_CMD_SJTAG_GET_PRODUCT_ID = 0xcd00,
	EAT_IPC_CMD_SJTAG_GET_PKHASH,
	EAT_IPC_CMD_SJTAG_BEGIN,
	EAT_IPC_CMD_SJTAG_GET_CHALLENGE,
	EAT_IPC_CMD_SJTAG_RUN_AUTH,
	EAT_IPC_CMD_SJTAG_END,
	EAT_IPC_CMD_SJTAG_GET_STATUS,
	EAT_IPC_CMD_SJTAG_GET_DBG_TIME,
	EAT_IPC_CMD_SJTAG_GET_PUBKEY,
	EAT_IPC_CMD_SJTAG_GET_AUTH_PARAMS,
};

enum ipc_frmk_cmd_id {
	ARR_IPC_CMD_ID_KERNEL_ARRAYDUMP = 0x1,
};

#define EAT_MAX_CHANNEL				(8)
#define EAT_FRM_CHANNEL				(0)
#define EAT_IPC_TIMEOUT				(100 * NSEC_PER_MSEC)

#define INTGR0					0x0020
#define INTCR0					0x0024
#define INTMR0					0x0028
#define INTSR0					0x002C
#define INTMSR0					0x0030
#define INTGR1					0x0040
#define INTCR1					0x0044
#define INTMR1					0x0048
#define INTSR1					0x004C
#define INTMSR1					0x0050
#define INTGR_DBGC_TO_AP			INTGR1
#define AP_INTCR				INTCR1
#define AP_INTMR				INTMR1
#define AP_INTSR				INTSR1
#define AP_INTMSR				INTMSR1
#define INTGR_AP_TO_DBGC			INTGR0
#define DBGC_INTCR				INTCR0
#define DBGC_INTMR				INTMR0
#define DBGC_INTSR				INTSR0
#define DBGC_INTMSR				INTMSR0

#define SR(n)					(0x80 + ((n) << 2))
#define INTR_FLAG_OFFSET                        16
#define FRAMEWORK_NAME				"FRM"

#if IS_ENABLED(CONFIG_EXYNOS_ADV_TRACER)
int adv_tracer_ipc_request_channel(struct device_node *np,
		ipc_callback handler, unsigned int *id, unsigned int *len);
int adv_tracer_ipc_release_channel(unsigned int id);
int adv_tracer_ipc_send_data(unsigned int id, struct adv_tracer_ipc_cmd *cmd);
int adv_tracer_ipc_send_data_polling(unsigned int id,
			struct adv_tracer_ipc_cmd *cmd);
int adv_tracer_ipc_send_data_polling_timeout(unsigned int id,
			struct adv_tracer_ipc_cmd *cmd,
			unsigned long timeout_ns);
int adv_tracer_ipc_send_data_async(unsigned int id,
			struct adv_tracer_ipc_cmd *cmd);
void adv_tracer_ipc_release_channel_by_name(const char *name);
#else
#define adv_tracer_ipc_request_channel(a, b, c, d)		(0)
#define adv_tracer_ipc_release_channel(a)			(0)
#define adv_tracer_ipc_send_data(a, b)				(0)
#define adv_tracer_ipc_send_data_polling(a, b)			(0)
#define adv_tracer_ipc_send_data_polling_timeout(a, b, c)	(0)
#define adv_tracer_ipc_send_data_async(a, b)			(0)
#define adv_tracer_ipc_release_channel_by_name(a)		do { } while (0)

#endif
#endif
