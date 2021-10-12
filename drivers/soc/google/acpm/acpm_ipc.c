// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Google LLC
 *
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/sched/clock.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/bitmap.h>
#include <soc/google/exynos-debug.h>
#include <soc/google/debug-snapshot.h>

#include "acpm.h"
#include "acpm_ipc.h"
#include "../cal-if/fvmap.h"
#include "fw_header/framework.h"
#include "../vh/kernel/systrace.h"

#define IPC_TIMEOUT				(10000000)
/*The unit of cnt is 10us*/
/*10000 * 10us = 100ms*/
#define SW_CNT_TIMEOUT_100MS			(10000)
#define IPC_NB_RETRIES				5
#define APM_SYSTICK_PERIOD_US			(20345)
#define IPC_TIMEOUT_COUNT_US			(500*1000)

static struct acpm_ipc_info *acpm_ipc;
static struct workqueue_struct *update_log_wq;
static struct acpm_debug_info *acpm_debug;
static bool is_acpm_stop_log;
static bool acpm_stop_log_req;

static struct acpm_framework *acpm_initdata;
static void __iomem *acpm_srambase;
static void __iomem *fvmap_base_address;
static void __iomem *frc_ctrl;
static DEFINE_MUTEX(print_log_mutex);

void *get_fvmap_base(void)
{
	return fvmap_base_address;
}
EXPORT_SYMBOL_GPL(get_fvmap_base);

u64 get_frc_time(void)
{
	u32 lsb, msb1, msb2;
	u64 raw_time;

	if (!frc_ctrl)
		return 0;

	__raw_writel(0x10, frc_ctrl); /* Write CAPTURE bit to ALIVE_FRC */
	__raw_writel(0x00, frc_ctrl); /* Clear CAPTURE bit */

	/*
	 * ALIVE_FRC timer needs 3 24.576MHz cycles = 122ns
	 * to detect rising edge of CAPTURE signal
	 */
	ndelay(122);

	/*
	 * Read ALIVE_FRC counter values. Need to read MSB
	 * at least twice in case another user captures timer during reading
	 */
	do {
		msb1 = __raw_readl(frc_ctrl + 4 * 4);
		lsb = __raw_readl(frc_ctrl + 3 * 4);
		msb2 = __raw_readl(frc_ctrl + 4 * 4);
	} while (msb1 != msb2);

	raw_time = ((u64)msb1 << 32) | lsb;

	/* convert to 49.152Hz ticks to be synchronized timer reported by ACPM */
	return raw_time << 1;
}
EXPORT_SYMBOL_GPL(get_frc_time);

#define IPC_AP_FVP_CAL	0
bool is_acpm_ipc_busy(void)
{
	struct acpm_ipc_ch *channel;
	unsigned int channel_id = IPC_AP_FVP_CAL;
	unsigned int tx_front, rx_front;

	channel = &acpm_ipc->channel[channel_id];
	tx_front = __raw_readl(channel->tx_ch.front);
	rx_front = __raw_readl(channel->rx_ch.front);

	return (tx_front != rx_front);
}
EXPORT_SYMBOL_GPL(is_acpm_ipc_busy);

static int plugins_init(struct device_node *node)
{
	struct plugin *plugins;
	int i, len, ret = 0;
	const char *name = NULL;
	void __iomem *base_addr = NULL;
	const __be32 *prop;
	unsigned int offset;

	plugins = (struct plugin *)(acpm_srambase + acpm_initdata->plugins);

	for (i = 0; i < acpm_initdata->num_plugins; i++) {
		if (plugins[i].is_attached == 0)
			continue;

		name = (const char *)(acpm_srambase + plugins[i].fw_name);
		if (!plugins[i].fw_name || !name)
			continue;

		if (strstr(name, "DVFS") || strstr(name, "dvfs")) {
			prop = of_get_property(node, "fvmap_offset", &len);
			if (prop) {
				base_addr = acpm_srambase;
				base_addr += (plugins[i].base_addr & ~0x1);
				offset = be32_to_cpup(prop);
				base_addr += offset;
			}

			prop = of_get_property(node, "fvmap_addr", &len);
			if (prop) {
				base_addr = acpm_srambase;
				offset = be32_to_cpup(prop);
				base_addr += offset;
			}

			fvmap_base_address = base_addr;
		}
	}

	return ret;
}

static bool is_rt_dl_task_policy(void)
{
	return (current->policy == SCHED_FIFO ||
		current->policy == SCHED_RR ||
		current->policy == SCHED_DEADLINE);
}

int acpm_ipc_get_buffer(const char *name, char **addr, u32 *size)
{
	if (!acpm_srambase)
		return -1;
	return acpm_get_buffer(acpm_srambase, acpm_initdata, name, addr, size);
}
EXPORT_SYMBOL_GPL(acpm_ipc_get_buffer);

void acpm_ipc_set_waiting_mode(bool mode)
{
	acpm_ipc->w_mode = mode;
}

void acpm_fw_set_log_level(unsigned int level)
{
	acpm_debug->debug_log_level = level;

	if (!level)
		cancel_delayed_work_sync(&acpm_debug->acpm_log_work);
	else if (level <= 2)
		queue_delayed_work(update_log_wq, &acpm_debug->acpm_log_work,
			msecs_to_jiffies(acpm_debug->period));
}

unsigned int acpm_fw_get_log_level(void)
{
	return acpm_debug->debug_log_level;
}

void acpm_ramdump(void)
{
	if (acpm_debug->dump_size)
		memcpy(acpm_debug->dump_dram_base, acpm_debug->dump_base, acpm_debug->dump_size);
}

/*
 * -----------------------------------------------------------------
 * |               |      arg0     |     arg1      |      arg2     |
 * | u32 log_header| u32 MSBsystick|   u32 *msg    |    u32 val    |
 * -----------------------------------------------------------------
 * systick is 56 bits: arg0 = systicks[55:24]
 * log_header format:
 * [31:28]: id
 * [27]:    is_raw
 * [26]:    log_err
 * [23:0]:  systicks[23:0]
 */
static void acpm_log_print_helper(unsigned int head, unsigned int arg0,
				  unsigned int arg1, unsigned int arg2)
{
	u8 id, is_raw;
	u8 is_err = (head & (0x1 << LOG_IS_ERR_SHIFT)) >> LOG_IS_ERR_SHIFT;
	u64 time;
	char *str;

	if (acpm_debug->debug_log_level >= 1 || !is_err) {
		id = (head >> LOG_ID_SHIFT) & 0xf;
		is_raw = (head >> LOG_IS_RAW_SHIFT) & 0x1;
		if (is_raw) {
			pr_info("[ACPM_FW] : id:%u, %x, %x, %x\n",
				id, arg0, arg1, arg2);
		} else {
			time = ((u64) arg0 << 24) | ((u64) head & 0xffffff);
			/* report time in ns */
			time = (time * APM_SYSTICK_PERIOD_US) / 1000;
			str = (char *) acpm_srambase + (arg1 & 0xffffff);

			pr_info("[ACPM_FW] : %llu id:%u, %s, %x\n",
				time, id, str, arg2);
		}
	}
}

void acpm_log_print_buff(struct acpm_log_buff *buffer)
{
	unsigned int front, rear;
	unsigned int head, arg0, arg1, arg2;

	if (is_acpm_stop_log)
		return;

	/* ACPM Log data dequeue & print */
	front = __raw_readl(buffer->log_buff_front);
	rear = buffer->rear_index;

	while (rear != front) {
		head = __raw_readl(buffer->log_buff_base +
				   buffer->log_buff_size * rear);
		arg0 = __raw_readl(buffer->log_buff_base +
				   buffer->log_buff_size * rear + 4);
		arg1 = __raw_readl(buffer->log_buff_base +
				   buffer->log_buff_size * rear + 8);
		arg2 = __raw_readl(buffer->log_buff_base +
				   buffer->log_buff_size * rear + 12);

		acpm_log_print_helper(head, arg0, arg1, arg2);

		if (buffer->log_buff_len == (rear + 1))
			rear = 0;
		else
			rear++;

		buffer->rear_index = rear;
		front = __raw_readl(buffer->log_buff_front);
	}

	if (acpm_stop_log_req) {
		is_acpm_stop_log = true;
		acpm_ramdump();
	}
}

static void acpm_log_print(void)
{
	mutex_lock(&print_log_mutex);
	if (acpm_debug->debug_log_level >= 2)
		acpm_log_print_buff(&acpm_debug->preempt);
	acpm_log_print_buff(&acpm_debug->normal);
	mutex_unlock(&print_log_mutex);
}

void acpm_stop_log_and_dumpram(void)
{
	acpm_stop_log_req = true;
	acpm_log_print();
}
EXPORT_SYMBOL_GPL(acpm_stop_log_and_dumpram);

static void acpm_log_work_fn(struct work_struct *work)
{
	acpm_log_print();

	queue_delayed_work(update_log_wq, &acpm_debug->acpm_log_work,
			   msecs_to_jiffies(acpm_debug->period));
}

int acpm_ipc_set_ch_mode(struct device_node *np, bool polling)
{
	int reg;
	int i, len, req_ch_id;
	const __be32 *prop;

	if (!np)
		return -ENODEV;

	prop = of_get_property(np, "acpm-ipc-channel", &len);
	if (!prop)
		return -ENOENT;
	req_ch_id = be32_to_cpup(prop);

	for (i = 0; i < acpm_ipc->num_channels; i++) {
		if (acpm_ipc->channel[i].id == req_ch_id) {
			reg = __raw_readl(acpm_ipc->intr + AP_INTMR);
			reg &= ~(1 << acpm_ipc->channel[i].id);
			reg |= polling << acpm_ipc->channel[i].id;
			__raw_writel(reg, acpm_ipc->intr + AP_INTMR);

			acpm_ipc->channel[i].polling = polling;

			return 0;
		}
	}

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(acpm_ipc_set_ch_mode);

int acpm_ipc_request_channel(struct device_node *np,
			     ipc_callback handler,
			     unsigned int *id,
			     unsigned int *size)
{
	struct callback_info *cb;
	int i, len, req_ch_id;
	const __be32 *prop;
	unsigned long flags;

	if (!np)
		return -ENODEV;

	prop = of_get_property(np, "acpm-ipc-channel", &len);
	if (!prop)
		return -ENOENT;
	req_ch_id = be32_to_cpup(prop);

	for (i = 0; i < acpm_ipc->num_channels; i++) {
		if (acpm_ipc->channel[i].id == req_ch_id) {
			*id = acpm_ipc->channel[i].id;
			*size = acpm_ipc->channel[i].tx_ch.size;

			if (handler) {
				cb = devm_kzalloc(acpm_ipc->dev,
						  sizeof(struct callback_info),
						  GFP_KERNEL);
				if (!cb)
					return -ENOMEM;
				cb->ipc_callback = handler;
				cb->client = np;

				spin_lock_irqsave(&acpm_ipc->channel[i].ch_lock, flags);
				list_add(&cb->list, &acpm_ipc->channel[i].list);
				spin_unlock_irqrestore(&acpm_ipc->channel[i].ch_lock, flags);
			}

			return 0;
		}
	}

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(acpm_ipc_request_channel);

int acpm_ipc_release_channel(struct device_node *np,
			     unsigned int channel_id)
{
	struct acpm_ipc_ch *channel = &acpm_ipc->channel[channel_id];
	struct list_head *cb_list = &channel->list;
	struct callback_info *cb;
	unsigned long flags;

	list_for_each_entry(cb, cb_list, list) {
		if (cb->client == np) {
			spin_lock_irqsave(&channel->ch_lock, flags);
			list_del(&cb->list);
			spin_unlock_irqrestore(&channel->ch_lock, flags);
			devm_kfree(acpm_ipc->dev, cb);
			break;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(acpm_ipc_release_channel);

static void apm_interrupt_gen(unsigned int id)
{
	writel((1 << id), acpm_ipc->intr + APM_INTGR);
}

static void check_response(struct acpm_ipc_ch *channel, struct ipc_config *cfg)
{
	volatile unsigned int rx_front;
	void __iomem *base;
	unsigned int i, data, size;
	unsigned int cfg_seq;
	const void *src;
	unsigned long flags;

	spin_lock_irqsave(&channel->rx_lock, flags);

	rx_front = __raw_readl(channel->rx_ch.front);
	i = __raw_readl(channel->rx_ch.rear);

	base = channel->rx_ch.base;
	size = channel->rx_ch.size;

	while (i != rx_front) {
		/* Read seq_num out from ACPM */
		data = __raw_readl(base + size * i);
		data = (data >> ACPM_IPC_PROTOCOL_SEQ_NUM) & 0x3f;

		if (!data || (data >= SEQ_NUM_MAX))
			panic("[ACPM] Invalid seq_num %u of channel %u\n", data, channel->id);

		if (channel->ch_cfg[data - 1].response == true) {
			/*
			 * Copy responds to the Global ch_chg and clear bitmap[data-1]
			 * later after ch_chg[data-1] assigns to cfg->cmd
			 */
			src = base + size * i;
			memcpy_align_4(channel->ch_cfg[data - 1].cmd, src, size);
		} else
			clear_bit(data - 1, channel->bitmap_seqnum);

		i++;
		i = i % channel->rx_ch.len;
	}

	/* Make RX-Rear catch up with RX-Front */
	__raw_writel(rx_front, channel->rx_ch.rear);

	/* Clear ACPM IPC pending Interrupt */
	__raw_writel(1 << channel->id, acpm_ipc->intr + AP_INTCR);

	/*
	 * For cases cfg->response = true, copy cmd[] data from ch_chg to cfg->cmd.
	 * Clear_bit so that we won't go timeout when waiting for responses.
	 */
	if (cfg != NULL) {
		cfg_seq = (cfg->cmd[0] >> ACPM_IPC_PROTOCOL_SEQ_NUM) & 0x3f;
		data = (channel->ch_cfg[cfg_seq - 1].cmd[0] >> ACPM_IPC_PROTOCOL_SEQ_NUM) & 0x3f;
		if (data == cfg_seq) {
			memcpy_align_4(cfg->cmd,
				channel->ch_cfg[data - 1].cmd,
				channel->rx_ch.size);
			clear_bit(data - 1, channel->bitmap_seqnum);
		}
	}

	spin_unlock_irqrestore(&channel->rx_lock, flags);
}

static void dequeue_policy(struct acpm_ipc_ch *channel)
{
	unsigned int front;
	unsigned int rear;
	struct list_head *cb_list = &channel->list;
	struct callback_info *cb;
	unsigned long flags;

	spin_lock_irqsave(&channel->rx_lock, flags);

	pr_debug("[ACPM]%s, ipc_ch=%d, rx_ch.size=0x%X, type=0x%X\n",
			__func__, channel->id, channel->rx_ch.size, channel->type);

	if (channel->type == TYPE_BUFFER) {
		memcpy_align_4(channel->cmd, channel->rx_ch.base, channel->rx_ch.size);
		spin_unlock_irqrestore(&channel->rx_lock, flags);
		list_for_each_entry(cb, cb_list, list)
			if (cb && cb->ipc_callback)
				cb->ipc_callback(channel->cmd, channel->rx_ch.size);

		return;
	}

	/* IPC command dequeue */
	front = __raw_readl(channel->rx_ch.front);
	rear = __raw_readl(channel->rx_ch.rear);

	while (rear != front) {
		memcpy_align_4(channel->cmd,
			       channel->rx_ch.base + channel->rx_ch.size * rear,
			       channel->rx_ch.size);

		list_for_each_entry(cb, cb_list, list)
			if (cb && cb->ipc_callback)
				cb->ipc_callback(channel->cmd, channel->rx_ch.size);

		if (channel->rx_ch.len == (rear + 1))
			rear = 0;
		else
			rear++;

		if (!channel->polling)
			complete(&channel->wait);

		__raw_writel(rear, channel->rx_ch.rear);
		front = __raw_readl(channel->rx_ch.front);
	}

	spin_unlock_irqrestore(&channel->rx_lock, flags);
}

static irqreturn_t acpm_ipc_irq_handler(int irq, void *data)
{
	struct acpm_ipc_info *ipc = data;
	unsigned int status;
	int i;

	/* ACPM IPC INTERRUPT STATUS REGISTER */
	status = __raw_readl(acpm_ipc->intr + AP_INTSR);

	for (i = 0; i < acpm_ipc->num_channels; i++) {
		if (!ipc->channel[i].polling && (status & (0x1 << ipc->channel[i].id))) {
			/* ACPM IPC INTERRUPT PENDING CLEAR */
			__raw_writel(1 << ipc->channel[i].id, ipc->intr + AP_INTCR);
		}
	}

	ipc->intr_status = status;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t acpm_ipc_irq_handler_thread(int irq, void *data)
{
	struct acpm_ipc_info *ipc = data;
	int i;

	pr_debug("[ACPM]%s, status=0x%X\n", __func__, ipc->intr_status);
	for (i = 0; i < acpm_ipc->num_channels; i++)
		if (!ipc->channel[i].polling && (ipc->intr_status & (1 << i)))
			dequeue_policy(&ipc->channel[i]);

	return IRQ_HANDLED;
}

int acpm_ipc_send_data_sync(unsigned int channel_id, struct ipc_config *cfg)
{
	int ret;
	struct acpm_ipc_ch *channel;
	ATRACE_BEGIN(__func__);
	ret = acpm_ipc_send_data(channel_id, cfg);

	if (!ret) {
		channel = &acpm_ipc->channel[channel_id];

		if (!channel->polling && cfg->response) {
			ret = wait_for_completion_interruptible_timeout(&channel->wait,
									msecs_to_jiffies(50));
			if (!ret) {
				pr_err("[%s] ipc_timeout!!!\n", __func__);
				ret = -ETIMEDOUT;
			} else {
				ret = 0;
			}
		}
	}
	ATRACE_END();
	return ret;
}
EXPORT_SYMBOL_GPL(acpm_ipc_send_data_sync);

int __acpm_ipc_send_data(unsigned int channel_id, struct ipc_config *cfg, bool w_mode)
{
	volatile unsigned int tx_front, tx_rear, rx_front;
	unsigned int tmp_index;
	unsigned int seq_num;
	struct acpm_ipc_ch *channel;
	bool timeout_flag = 0;
	u64 timeout, now, frc;
	u32 retry_cnt = 0;
	u32 count = 0;
	unsigned long flags;
	unsigned int cnt_10us = 0;

	if (channel_id >= acpm_ipc->num_channels && !cfg)
		return -EIO;

	channel = &acpm_ipc->channel[channel_id];

	if (channel->tx_ch.len < 3)
		return -EIO;

	spin_lock_irqsave(&channel->tx_lock, flags);

	/*
	 * Reserve at least 2 elements in the queue to prevent buffer full and qfull.
	 * For those channels tx_ch.len = 1, it's not allowed to use acpm_ipc_send_data.
	 * For those channels tx_ch.len = 3, it's allowed to acpm_ipc_send once a request.
	 */
	for (;;) {
		tx_front = __raw_readl(channel->tx_ch.front);
		rx_front = __raw_readl(channel->rx_ch.front);

		if (((tx_front + 2) % channel->tx_ch.len) != rx_front)
			break;
		/* timeout if rx front can't catch up with tx front */
		if (count++ > IPC_TIMEOUT_COUNT_US)
			panic("[ACPM] channel %u tx f:%u rx f:%u timeout!\n",
				channel_id, tx_front, rx_front);
		/*
		 * Add 1us delay to avoid being occupied by kernel
		 * all the time since ACPM also does the same access.
		 */
		udelay(1);
	}

	tx_rear = __raw_readl(channel->tx_ch.rear);
	tmp_index = (tx_front + 1) % channel->tx_ch.len;

	/* buffer full check */
	if (tmp_index == tx_rear) {
		acpm_log_print();
		panic("[ACPM] channel %u tx buffer full!\n", channel_id);
	}

	if (!cfg->cmd) {
		/*
		 * We can't move it before taking the mutex,
		 * because cfg->cmd could be used as a barrier.
		 */
		spin_unlock_irqrestore(&channel->tx_lock, flags);
		return -EIO;
	}

	/* Check before a new request is sent. */
	check_response(channel, NULL);

	/* Prevent channel->seq_num from being re-used */
	do {
		if (++channel->seq_num == SEQ_NUM_MAX)
			channel->seq_num = 1;
	} while (test_bit(channel->seq_num - 1, channel->bitmap_seqnum));

	/* Clear ch_cfg for upcoming responses */
	memset(channel->ch_cfg[channel->seq_num - 1].cmd, 0,
		sizeof(int) * channel->rx_ch.size);
	/* Flag the index based on seq_num. (seq_num: 1~63, bitmap/ch_cfg: 0~62) */
	set_bit(channel->seq_num - 1, channel->bitmap_seqnum);
	channel->ch_cfg[channel->seq_num - 1].response = cfg->response;

	cfg->cmd[0] |= (channel->seq_num & 0x3f) << ACPM_IPC_PROTOCOL_SEQ_NUM;
	seq_num = (cfg->cmd[0] >> ACPM_IPC_PROTOCOL_SEQ_NUM) & 0x3f;

	memcpy_align_4(channel->tx_ch.base + channel->tx_ch.size * tx_front,
		       cfg->cmd,
		       channel->tx_ch.size);

	cfg->cmd[1] = 0;
	cfg->cmd[2] = 0;
	cfg->cmd[3] = 0;

	writel(tmp_index, channel->tx_ch.front);

	apm_interrupt_gen(channel->id);
	spin_unlock_irqrestore(&channel->tx_lock, flags);

	if (channel->polling && cfg->response) {
retry:
		timeout = sched_clock() + IPC_TIMEOUT;
		timeout_flag = false;
		do {
			check_response(channel, cfg);
			if (!test_bit(seq_num - 1, channel->bitmap_seqnum))
				break;
			now = sched_clock();
			if (timeout < now || cnt_10us > SW_CNT_TIMEOUT_100MS) {
				if (retry_cnt > IPC_NB_RETRIES) {
					timeout_flag = true;
					break;
				} else if (retry_cnt > 0) {
					unsigned int saved_debug_log_level =
					    acpm_debug->debug_log_level;
					frc = get_frc_time();
					pr_err("acpm_ipc retry %d, now = %llu, frc = %llu, timeout = %llu",
					       retry_cnt, now, frc, timeout);
					pr_err("I:0x%x %u s:%u RX r:%u f:%u TX r:%u f:%u\n",
					       __raw_readl(acpm_ipc->intr + AP_INTSR),
					       channel->id, seq_num,
					       __raw_readl(channel->rx_ch.rear),
					       __raw_readl(channel->rx_ch.front),
					       __raw_readl(channel->tx_ch.rear),
					       __raw_readl(channel->tx_ch.front));
					pr_err("SR0:0x%x MR0:0x%X MSR0:0x%X,"
						" SR1:0x%x MR1:0x%X MSR1:0x%X\n",
						__raw_readl(acpm_ipc->intr + INTSR0),
						__raw_readl(acpm_ipc->intr + INTMR0),
						__raw_readl(acpm_ipc->intr + INTMSR0),
						__raw_readl(acpm_ipc->intr + INTSR1),
						__raw_readl(acpm_ipc->intr + INTMR1),
						__raw_readl(acpm_ipc->intr + INTMSR1));

					++retry_cnt;
					acpm_debug->debug_log_level = 2;
					acpm_log_print();
					acpm_debug->debug_log_level = saved_debug_log_level;

					goto retry;
				} else {
					++retry_cnt;
					continue;
				}
				cnt_10us = 0;
			} else {
				if (w_mode) {
					/*assume at least 50us delay here*/
					usleep_range(50, 100);
					cnt_10us += 5;
				} else {
					udelay(10);
					cnt_10us++;
				}
			}
		} while (true);

		if (timeout_flag) {
			pr_err("%s Timeout error! now = %llu timeout = %llu ch:%u s:%u bitmap:%lx\n",
			       __func__, now, timeout, channel->id, seq_num,
			       channel->bitmap_seqnum[0]);

			acpm_ramdump();
			dump_stack();
			dbg_snapshot_do_dpm_policy(acpm_ipc->panic_action, "acpm_ipc timeout");
		}
	}

	return 0;
}

int acpm_ipc_send_data(unsigned int channel_id, struct ipc_config *cfg)
{
	int ret;
	ATRACE_BEGIN(__func__);
	ret = __acpm_ipc_send_data(channel_id, cfg, false);
	ATRACE_END();
	return ret;
}
EXPORT_SYMBOL_GPL(acpm_ipc_send_data);

int acpm_ipc_send_data_lazy(unsigned int channel_id, struct ipc_config *cfg)
{
	int ret;
	ATRACE_BEGIN(__func__);
	if (is_rt_dl_task_policy())
		ret = __acpm_ipc_send_data(channel_id, cfg, true);
	else
		ret = __acpm_ipc_send_data(channel_id, cfg, false);
	ATRACE_END();
	return ret;
}
EXPORT_SYMBOL_GPL(acpm_ipc_send_data_lazy);

static int log_buffer_init(struct device *dev, struct device_node *node)
{
	const __be32 *prop;
	unsigned int len = 0;
	unsigned int dump_base = 0;
	unsigned int dump_size = 0;
	void __iomem *base;

	acpm_debug = devm_kzalloc(dev, sizeof(struct acpm_debug_info), GFP_KERNEL);
	if (IS_ERR(acpm_debug))
		return PTR_ERR(acpm_debug);

	base = acpm_ipc->sram_base;
	acpm_debug->time_index = base + acpm_ipc->initdata->ktime_index;
	acpm_debug->normal.log_buff_rear = acpm_ipc->sram_base +
	    acpm_ipc->initdata->log_buf_rear;
	acpm_debug->normal.log_buff_front = acpm_ipc->sram_base +
	    acpm_ipc->initdata->log_buf_front;
	acpm_debug->normal.log_buff_base = acpm_ipc->sram_base +
	    acpm_ipc->initdata->log_data;
	acpm_debug->normal.log_buff_len =
	    acpm_ipc->initdata->log_entry_len;
	acpm_debug->normal.log_buff_size = acpm_ipc->initdata->log_entry_size;

	acpm_debug->preempt.log_buff_rear = acpm_ipc->sram_base +
	    acpm_ipc->initdata->preempt_log_buf_rear;
	acpm_debug->preempt.log_buff_front = acpm_ipc->sram_base +
	    acpm_ipc->initdata->preempt_log_buf_front;
	acpm_debug->preempt.log_buff_base = acpm_ipc->sram_base +
	    acpm_ipc->initdata->preempt_log_data;
	acpm_debug->preempt.log_buff_len =
	    acpm_ipc->initdata->preempt_log_entry_len;
	acpm_debug->preempt.log_buff_size = acpm_ipc->initdata->log_entry_size;

	prop = of_get_property(node, "debug-log-level", &len);
	if (prop)
		acpm_debug->debug_log_level = be32_to_cpup(prop);

	prop = of_get_property(node, "dump-base", &len);
	if (prop)
		dump_base = be32_to_cpup(prop);

	prop = of_get_property(node, "dump-size", &len);
	if (prop)
		dump_size = be32_to_cpup(prop);

	if (dump_base && dump_size) {
		acpm_debug->dump_base = ioremap(dump_base, dump_size);
		acpm_debug->dump_size = dump_size;
	}

	prop = of_get_property(node, "logging-period", &len);
	if (prop)
		acpm_debug->period = be32_to_cpup(prop);

	acpm_debug->dump_dram_base = kzalloc(acpm_debug->dump_size, GFP_KERNEL);

	pr_info("[ACPM] acpm framework SRAM dump to dram base: 0x%llx\n",
		virt_to_phys(acpm_debug->dump_dram_base));

	__raw_writel(0xffffffff, acpm_debug->time_index);

	spin_lock_init(&acpm_debug->lock);

	return 0;
}

static int channel_init(void)
{
	int i, j;
	unsigned int mask = 0;
	struct ipc_channel *ipc_ch;
	void __iomem *base;

	acpm_ipc->num_channels = acpm_ipc->initdata->ipc_ap_max;

	acpm_ipc->channel = devm_kzalloc(acpm_ipc->dev,
					 sizeof(struct acpm_ipc_ch) * acpm_ipc->num_channels,
					 GFP_KERNEL);

	for (i = 0; i < acpm_ipc->num_channels; i++) {
		ipc_ch = (struct ipc_channel *)(acpm_ipc->sram_base +
						acpm_ipc->initdata->ipc_channels);
		acpm_ipc->channel[i].polling = ipc_ch[i].ap_poll;
		acpm_ipc->channel[i].id = ipc_ch[i].id;
		acpm_ipc->channel[i].type = ipc_ch[i].type;
		mask |= acpm_ipc->channel[i].polling << acpm_ipc->channel[i].id;

		/* Channel's RX buffer info */
		base = acpm_ipc->sram_base;
		acpm_ipc->channel[i].rx_ch.size = ipc_ch[i].ch.q_elem_size;
		acpm_ipc->channel[i].rx_ch.len = ipc_ch[i].ch.q_len;
		acpm_ipc->channel[i].rx_ch.rear = base + ipc_ch[i].ch.tx_rear;
		acpm_ipc->channel[i].rx_ch.front = base + ipc_ch[i].ch.tx_front;
		acpm_ipc->channel[i].rx_ch.base = base + ipc_ch[i].ch.tx_base;
		/* Channel's TX buffer info */
		acpm_ipc->channel[i].tx_ch.size = ipc_ch[i].ch.q_elem_size;
		acpm_ipc->channel[i].tx_ch.len = ipc_ch[i].ch.q_len;
		acpm_ipc->channel[i].tx_ch.rear = base + ipc_ch[i].ch.rx_rear;
		acpm_ipc->channel[i].tx_ch.front = base + ipc_ch[i].ch.rx_front;
		acpm_ipc->channel[i].tx_ch.base = base + ipc_ch[i].ch.rx_base;
		acpm_ipc->channel[i].tx_ch.d_buff_size = ipc_ch[i].ch.rx_indr_buf_size;
		acpm_ipc->channel[i].tx_ch.direction = base + ipc_ch[i].ch.rx_indr_buf;

		acpm_ipc->channel[i].cmd = devm_kzalloc(
					acpm_ipc->dev,
					sizeof(unsigned int) * acpm_ipc->channel[i].tx_ch.size,
					GFP_KERNEL);

		for (j = 0; j < SEQ_NUM_MAX; j++) {
			acpm_ipc->channel[i].ch_cfg[j].cmd = devm_kzalloc(
						acpm_ipc->dev,
						sizeof(int) * acpm_ipc->channel[i].tx_ch.size,
						GFP_KERNEL);
		}

		init_completion(&acpm_ipc->channel[i].wait);
		spin_lock_init(&acpm_ipc->channel[i].rx_lock);
		spin_lock_init(&acpm_ipc->channel[i].tx_lock);
		spin_lock_init(&acpm_ipc->channel[i].ch_lock);
		INIT_LIST_HEAD(&acpm_ipc->channel[i].list);
	}

	__raw_writel(mask, acpm_ipc->intr + AP_INTMR);

	return 0;
}

static void acpm_error_log_ipc_callback(unsigned int *cmd, unsigned int size)
{
	acpm_log_print();
}

static int debug_acpm_ipc_panic_action_get(void *data, u64 *val)
{
	struct acpm_ipc_info *acpm_ipc = (struct acpm_ipc_info *)data;

	*val = acpm_ipc->panic_action;

	return 0;
}

static int debug_acpm_ipc_panic_action_set(void *data, u64 val)
{
	struct acpm_ipc_info *acpm_ipc = (struct acpm_ipc_info *)data;

	if (val < 0 || val >= GO_ACTION_MAX)
		return -ERANGE;
	acpm_ipc->panic_action = val;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_acpm_ipc_panic_action_fops,
			debug_acpm_ipc_panic_action_get,
			debug_acpm_ipc_panic_action_set,
			"%llu\n");

static void acpm_ipc_debugfs_init(struct acpm_ipc_info *acpm_ipc)
{
	struct dentry *den;

	den = debugfs_lookup("acpm_framework", NULL);
	if (!den)
		den = debugfs_create_dir("acpm_framework", NULL);
	debugfs_create_file("acpm_ipc_panic_action", 0644, den, acpm_ipc,
			    &debug_acpm_ipc_panic_action_fops);
}

int acpm_ipc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	int ret = 0, len;
	const __be32 *prop;

	if (!node) {
		dev_err(&pdev->dev, "cannot support non-dt devices\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "acpm_ipc probe\n");

	acpm_ipc = devm_kzalloc(&pdev->dev,
				sizeof(struct acpm_ipc_info), GFP_KERNEL);

	if (IS_ERR(acpm_ipc))
		return PTR_ERR(acpm_ipc);

	acpm_ipc->irq = irq_of_parse_and_map(node, 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	acpm_ipc->intr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(acpm_ipc->intr))
		return PTR_ERR(acpm_ipc->intr);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	acpm_ipc->sram_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(acpm_ipc->sram_base))
		return PTR_ERR(acpm_ipc->sram_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	frc_ctrl = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(frc_ctrl))
		return PTR_ERR(frc_ctrl);

	prop = of_get_property(node, "initdata-base", &len);
	if (prop) {
		acpm_ipc->initdata_base = be32_to_cpup(prop);
	} else {
		dev_err(&pdev->dev, "Parsing initdata_base failed.\n");
		return -EINVAL;
	}
	acpm_ipc->initdata = (struct acpm_framework *)(acpm_ipc->sram_base +
						       acpm_ipc->initdata_base);
	acpm_initdata = acpm_ipc->initdata;
	acpm_srambase = acpm_ipc->sram_base;

	if (of_property_read_u32(node, "panic-action",
				&acpm_ipc->panic_action))
		acpm_ipc->panic_action = GO_WATCHDOG_ID;

	acpm_ipc_debugfs_init(acpm_ipc);

	acpm_ipc->dev = &pdev->dev;

	channel_init();

	ret = devm_request_threaded_irq(&pdev->dev, acpm_ipc->irq,
					acpm_ipc_irq_handler,
					acpm_ipc_irq_handler_thread,
					IRQF_ONESHOT,
					dev_name(&pdev->dev), acpm_ipc);

	if (ret) {
		dev_err(&pdev->dev, "failed to register intr%d\n", ret);
		return ret;
	}

	log_buffer_init(&pdev->dev, node);

	update_log_wq = create_workqueue("acpm_log");
	if (!update_log_wq) {
		dev_err(&pdev->dev, "failed to create workqueue\n");
	}
	INIT_DELAYED_WORK(&acpm_debug->acpm_log_work, acpm_log_work_fn);
	acpm_fw_set_log_level(acpm_debug->debug_log_level);

	if (acpm_ipc_request_channel(node, acpm_error_log_ipc_callback,
				     &acpm_debug->async_id,
				     &acpm_debug->async_size)) {
		dev_err(&pdev->dev, "No asynchronous channel\n");
	}

	ret = plugins_init(node);
	dev_info(&pdev->dev, "acpm_ipc probe done.\n");
	return ret;
}

int acpm_ipc_remove(struct platform_device *pdev)
{
	return 0;
}
