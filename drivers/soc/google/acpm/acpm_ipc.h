/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright 2020 Google LLC
 *
 */
#ifndef __ACPM_IPC_H_
#define __ACPM_IPC_H_

#include <soc/google/acpm_ipc_ctrl.h>
#include <linux/kernel-top.h>

struct buff_info {
	void __iomem *rear;
	void __iomem *front;
	void __iomem *base;
	void __iomem *direction;

	unsigned int size;
	unsigned int len;
	unsigned int d_buff_size;
};

struct callback_info {
	void (*ipc_callback)(unsigned int *cmd, unsigned int size);
	struct device_node *client;
	struct list_head list;
};

#define SEQ_NUM_MAX    64
struct acpm_ipc_ch {
	struct buff_info rx_ch;
	struct buff_info tx_ch;
	struct list_head list;

	unsigned int id;
	unsigned int type;
	unsigned int seq_num;
	unsigned int *cmd;
	spinlock_t rx_lock;
	spinlock_t tx_lock;
	spinlock_t ch_lock;

	struct completion wait;
	bool polling;
	DECLARE_BITMAP(bitmap_seqnum, SEQ_NUM_MAX - 1);
	struct ipc_config ch_cfg[SEQ_NUM_MAX];
};

struct acpm_ipc_info {
	unsigned int num_channels;
	struct device *dev;
	struct acpm_ipc_ch *channel;
	unsigned int irq;
	void __iomem *intr;
	void __iomem *sram_base;
	bool w_mode;
	struct acpm_framework *initdata;
	unsigned int initdata_base;
	unsigned int intr_status;
	unsigned int panic_action;
};

struct acpm_log_buff {
    void __iomem *log_buff_rear;
    void __iomem *log_buff_front;
    void __iomem *log_buff_base;
    unsigned int log_buff_len;
    unsigned int log_buff_size;
    unsigned int rear_index;
};

struct acpm_debug_info {
	unsigned int period;
	void __iomem *time_index;
	unsigned int num_timestamps;
	unsigned long long *timestamps;
	struct acpm_log_buff normal;
	struct acpm_log_buff preempt;
	void __iomem *dump_base;
	unsigned int dump_size;
	void __iomem *dump_dram_base;
	unsigned int debug_log_level;
	unsigned int retry_log;
	struct delayed_work acpm_log_work;
	unsigned int async_id; /* ACPM IPC_AP_ERR_LOG_ASYNC channel id */
	unsigned int async_size; /* ACPM IPC_AP_ERR_LOG_ASYNC channel queue sizes */
	struct kernel_top_context *ktop_cxt;

	spinlock_t lock; /* generic spin-lock for debug */
};

struct cpu_irq_info {
	const char      *name;
	int             irq_num;
	unsigned long   hwirq_num;
	unsigned int    irq_stat;
};

#define LOG_ID_SHIFT				(28)
#define LOG_IS_RAW_SHIFT			(27)
#define LOG_IS_ERR_SHIFT			(26)

#define BUSY_WAIT				(0)
#define SLEEP_WAIT				(1)
#define INTGR0					0x0020
#define INTCR0					0x0024
#define INTMR0					0x0028
#define INTSR0					0x002c
#define INTMSR0					0x0030
#define INTGR1					0x0040
#define INTMR1					0x0048
#define INTSR1					0x004c
#define INTMSR1					0x0050
#define	APM_INTGR				(INTGR1)
#define AP_INTMR				(INTMR0)
#define AP_INTCR				(INTCR0)
#define AP_INTSR				(INTSR0)
#define SR0					0x0080
#define SR1					0x0084
#define SR2					0x0088
#define SR3					0x008C

#define UNTIL_EQUAL(arg0, arg1, flag)			\
do {							\
	u64 timeout = sched_clock() + IPC_TIMEOUT;	\
	bool t_flag = true;				\
	do {						\
		if ((arg0) == (arg1)) {			\
			t_flag = false;			\
			break;				\
		} else {				\
			cpu_relax();			\
		}					\
	} while (timeout >= sched_clock());		\
	if (t_flag) {					\
		pr_err("%s %d Timeout error!\n",	\
				__func__, __LINE__);	\
	}						\
	(flag) = t_flag;				\
} while (0)

#define REGULATOR_INFO_ID	8

extern void timestamp_write(void);
extern void acpm_ramdump(void);
extern void acpm_fw_set_log_level(unsigned int on);
extern unsigned int acpm_fw_get_log_level(void);
extern void acpm_fw_set_retry_log_ctrl(bool enable);
extern unsigned int acpm_fw_get_retry_log_ctrl(void);
extern void acpm_ipc_set_waiting_mode(bool mode);

extern int acpm_ipc_remove(struct platform_device *pdev);
extern int acpm_ipc_probe(struct platform_device *pdev);
#endif
