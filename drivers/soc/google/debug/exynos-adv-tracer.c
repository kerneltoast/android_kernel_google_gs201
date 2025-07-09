// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/ktime.h>

#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-adv-tracer.h>
#include <soc/google/exynos-pmu-if.h>

static struct adv_tracer_info *eat_info;
static struct adv_tracer_ipc_main *eat_ipc;

#define ADV_TRACER_SSCOREDUMP

#ifdef ADV_TRACER_SSCOREDUMP
#include <linux/platform_data/sscoredump.h>

#define DEVICE_NAME "debugcore"

static void adv_tracer_ipc_sscd_release(struct device *dev)
{

}

static struct sscd_platform_data sscd_pdata;
static struct platform_device sscd_dev = {
	.name            = DEVICE_NAME,
	.driver_override = SSCD_NAME,
	.id              = -1,
	.dev             = {
		.platform_data = &sscd_pdata,
		.release       = adv_tracer_ipc_sscd_release,
	},
};
#endif


static void adv_tracer_ipc_read_buffer(void *dest, const void *src,
				      unsigned int len)
{
	const unsigned int *sp = src;
	unsigned int *dp = dest;
	int i;

	for (i = 0; i < len; i++)
		*dp++ = readl(sp++);
}

static void adv_tracer_ipc_write_buffer(void *dest, const void *src,
				       unsigned int len)
{
	const unsigned int *sp = src;
	unsigned int *dp = dest;
	int i;

	for (i = 0; i < len; i++)
		writel(*sp++, dp++);
}

static void adv_tracer_ipc_interrupt_clear(int id)
{
	__raw_writel((1 << eat_ipc->intr_bitoffset) << id,
			eat_ipc->mailbox_base + AP_INTCR);
}

static irqreturn_t adv_tracer_ipc_irq_handler(int irq, void *data)
{
	struct adv_tracer_ipc_main *ipc = data;
	struct adv_tracer_ipc_cmd *cmd;
	struct adv_tracer_ipc_ch *channel;
	unsigned int status;
	int i;

	/* ADV_TRACER IPC INTERRUPT STATUS REGISTER */
	status = __raw_readl(ipc->mailbox_base + AP_INTSR);
	status = status >> ipc->intr_bitoffset;

	for (i = 0; i < eat_ipc->num_channels; i++) {
		channel = &eat_ipc->channel[i];
		if (status & (0x1 << channel->id)) {
			cmd = channel->cmd;
			adv_tracer_ipc_read_buffer(cmd->buffer,
					channel->buff_regs, channel->len);
			/* ADV_TRACER IPC INTERRUPT PENDING CLEAR */
			adv_tracer_ipc_interrupt_clear(channel->id);
		}
	}
	ipc->mailbox_status = status;
	return IRQ_WAKE_THREAD;
}

static irqreturn_t adv_tracer_ipc_irq_handler_thread(int irq, void *data)
{
	struct adv_tracer_ipc_main *ipc = data;
	struct adv_tracer_ipc_cmd *cmd;
	struct adv_tracer_ipc_ch *channel;
	unsigned int status;
	int i;

	status = ipc->mailbox_status;

	for (i = 0; i < eat_ipc->num_channels; i++) {
		channel = &eat_ipc->channel[i];
		if (status & (1 << i)) {
			cmd = channel->cmd;
			if (cmd->cmd_raw.response)
				complete(&channel->wait);

			if (cmd->cmd_raw.one_way || cmd->cmd_raw.response) {
				/* HANDLE to Plugin handler */
				if (channel->ipc_callback)
					channel->ipc_callback(cmd, channel->len);
			}
		}
	}
	return IRQ_HANDLED;
}

static void adv_tracer_interrupt_gen(unsigned int id)
{
	writel(1 << id, eat_ipc->mailbox_base + INTGR_AP_TO_DBGC);
}

static void __adv_tracer_ipc_send_data(struct adv_tracer_ipc_ch *channel,
				      struct adv_tracer_ipc_cmd *cmd)
{
	memcpy(channel->cmd, cmd, sizeof(unsigned int) * channel->len);
	adv_tracer_ipc_write_buffer(channel->buff_regs, channel->cmd, channel->len);
	adv_tracer_interrupt_gen(channel->id);
}

int adv_tracer_ipc_send_data(unsigned int id, struct adv_tracer_ipc_cmd *cmd)
{
	struct adv_tracer_ipc_ch *channel = NULL;
	int ret;

	if (id >= EAT_MAX_CHANNEL && IS_ERR(cmd))
		return -EIO;

	channel = &eat_ipc->channel[id];
	cmd->cmd_raw.manual_polling = 0;
	cmd->cmd_raw.one_way = 0;
	cmd->cmd_raw.response = 0;

	mutex_lock(&channel->wait_lock);
	__adv_tracer_ipc_send_data(channel, cmd);

	ret = wait_for_completion_interruptible_timeout(&channel->wait,
			msecs_to_jiffies(1000));
	memcpy(cmd, channel->cmd, sizeof(unsigned int) * channel->len);
	mutex_unlock(&channel->wait_lock);

	if (!ret) {
		dev_err(eat_info->dev, "%d channel timeout error\n", id);
		return -EBUSY;
	}

	return 0;
}
EXPORT_SYMBOL(adv_tracer_ipc_send_data);

int adv_tracer_ipc_send_data_polling_timeout(unsigned int id,
					    struct adv_tracer_ipc_cmd *cmd,
					    unsigned long tmout_ns)
{
	struct adv_tracer_ipc_ch *channel = NULL;
	struct adv_tracer_ipc_cmd _cmd;
	ktime_t timeout, now;

	if (id >= EAT_MAX_CHANNEL && IS_ERR(cmd))
		return -EIO;

	channel = &eat_ipc->channel[id];
	cmd->cmd_raw.manual_polling = 1;
	cmd->cmd_raw.one_way = 0;
	cmd->cmd_raw.response = 0;

	spin_lock(&channel->ch_lock);
	__adv_tracer_ipc_send_data(channel, cmd);

	timeout = ktime_add_ns(ktime_get(), tmout_ns);
	do {
		now = ktime_get();
		adv_tracer_ipc_read_buffer(&_cmd.buffer[0],
				channel->buff_regs, 1);
	} while (!(_cmd.cmd_raw.response || ktime_after(now, timeout)));

	if (!_cmd.cmd_raw.response) {
		spin_unlock(&channel->ch_lock);
		dev_err(eat_info->dev, "%d channel timeout error\n", id);
		return -EBUSY;
	}
	adv_tracer_ipc_read_buffer(cmd->buffer, channel->buff_regs, channel->len);
	spin_unlock(&channel->ch_lock);

	if (!_cmd.cmd_raw.ok) {
		dev_err(eat_info->dev, "error return in ipc (%d)\n", id);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(adv_tracer_ipc_send_data_polling_timeout);

int adv_tracer_ipc_send_data_polling(unsigned int id,
				    struct adv_tracer_ipc_cmd *cmd)
{
	return adv_tracer_ipc_send_data_polling_timeout(id, cmd,
			EAT_IPC_TIMEOUT);
}
EXPORT_SYMBOL(adv_tracer_ipc_send_data_polling);

int adv_tracer_ipc_send_data_async(unsigned int id,
				  struct adv_tracer_ipc_cmd *cmd)
{
	struct adv_tracer_ipc_ch *channel = NULL;

	if (id >= EAT_MAX_CHANNEL && IS_ERR(cmd))
		return -EIO;

	channel = &eat_ipc->channel[id];
	cmd->cmd_raw.manual_polling = 0;
	cmd->cmd_raw.one_way = 1;
	cmd->cmd_raw.response = 0;

	spin_lock(&channel->ch_lock);
	__adv_tracer_ipc_send_data(channel, cmd);
	spin_unlock(&channel->ch_lock);

	return 0;
}
EXPORT_SYMBOL(adv_tracer_ipc_send_data_async);

struct adv_tracer_ipc_cmd *adv_tracer_ipc_get_channel_cmd(unsigned int id)
{
	struct adv_tracer_ipc_ch *channel = NULL;
	struct adv_tracer_ipc_cmd *cmd = NULL;

	if (IS_ERR(eat_ipc))
		goto out;

	channel = &eat_ipc->channel[id];
	if (IS_ERR(channel))
		goto out;

	if (!channel->used)
		goto out;

	cmd = channel->cmd;
out:
	return cmd;
}
EXPORT_SYMBOL(adv_tracer_ipc_get_channel_cmd);

struct adv_tracer_ipc_ch *eat_ipc_get_channel(unsigned int id)
{
	struct adv_tracer_ipc_ch *ipc_channel = NULL;

	if (IS_ERR(eat_ipc))
		goto out;

	ipc_channel = &eat_ipc->channel[id];
	if (IS_ERR(ipc_channel)) {
		dev_err(eat_info->dev, "%d channel is not allocated\n", id);
		ipc_channel = NULL;
	}
out:
	return ipc_channel;
}
EXPORT_SYMBOL(eat_ipc_get_channel);

static int adv_tracer_ipc_channel_init(unsigned int id, unsigned int offset,
				      unsigned int len, ipc_callback handler,
				      const char *name)
{
	struct adv_tracer_ipc_ch *channel = &eat_ipc->channel[id];

	if (channel->used) {
		dev_err(eat_info->dev, "channel%d is already reserved\n", id);
		return -1;
	}

	channel->id = id;
	channel->offset = offset;
	channel->len = len;
	strncpy(channel->id_name, name, sizeof(unsigned int) - 1);

	/* channel->buff_regs -> shared buffer by owns */
	channel->buff_regs = (void __iomem *)(eat_ipc->mailbox_base + offset);
	channel->cmd = devm_kzalloc(eat_ipc->dev,
			sizeof(struct adv_tracer_ipc_cmd), GFP_KERNEL);
	if (IS_ERR(channel->cmd))
		return PTR_ERR(channel->cmd);

	channel->ipc_callback = handler;
	channel->used = true;
	return 0;
}

int adv_tracer_ipc_release_channel(unsigned int id)
{
	struct adv_tracer_ipc_cmd *cmd;
	int ret;

	if (!eat_ipc->channel[id].used) {
		dev_err(eat_info->dev, "channel%d  is unsed\n", id);
		return -1;
	}

	cmd = adv_tracer_ipc_get_channel_cmd(EAT_FRM_CHANNEL);
	if (!cmd) {
		dev_err(eat_info->dev, "channel%d is failed to release\n", id);
		return -EIO;
	}

	cmd->cmd_raw.cmd = EAT_IPC_CMD_CH_RELEASE;
	cmd->buffer[1] = id;

	ret = adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, cmd);
	if (ret < 0) {
		dev_err(eat_info->dev, "channel%d is failed to release\n", id);
		return ret;
	}

	eat_ipc->channel[id].used = false;

	return 0;
}
EXPORT_SYMBOL(adv_tracer_ipc_release_channel);

void adv_tracer_ipc_release_channel_by_name(const char *name)
{
	struct adv_tracer_ipc_ch *ipc_ch = eat_ipc->channel;
	int i;

	for (i = 0; i < eat_ipc->num_channels; i++) {
		if (!ipc_ch[i].used)
			continue;

		if (!strcmp(name, ipc_ch[i].id_name)) {
			ipc_ch[i].used = 0;
			break;
		}
	}
}
EXPORT_SYMBOL(adv_tracer_ipc_release_channel_by_name);

int adv_tracer_ipc_request_channel(struct device_node *np,
				  ipc_callback handler,
				  unsigned int *id, unsigned int *len)
{
	const char *plugin_name;
	unsigned int plugin_len, offset;
	struct adv_tracer_ipc_cmd cmd;
	int ret;

	if (!np)
		return -ENODEV;

	if (of_property_read_u32(np, "plugin-len", &plugin_len))
		return -EINVAL;

	if (of_property_read_string(np, "plugin-name", &plugin_name))
		return -EINVAL;

	cmd.cmd_raw.cmd = EAT_IPC_CMD_IS_ATTACHED;
	memcpy(&cmd.buffer[1], plugin_name, sizeof(unsigned int));
	ret = adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, &cmd);
	if (ret < 0) {
		dev_err(eat_info->dev, "Do not exist %s plugin\n", plugin_name);
		return -ENODEV;
	}

	cmd.cmd_raw.cmd = EAT_IPC_CMD_CH_INIT;
	cmd.buffer[1] = plugin_len;
	memcpy(&cmd.buffer[2], plugin_name, sizeof(unsigned int));

	ret = adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, &cmd);
	if (ret) {
		dev_err(eat_info->dev, "channel%d is failed to request\n",
				EAT_FRM_CHANNEL);
		return -ENODEV;
	}

	*id = cmd.buffer[3];
	offset = SR(cmd.buffer[2]);
	*len = cmd.buffer[1];

	ret = adv_tracer_ipc_channel_init(*id, offset, *len, handler, plugin_name);
	if (ret) {
		dev_err(eat_info->dev, "channel%d is failed to init\n", *id);
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL(adv_tracer_ipc_request_channel);

static void adv_tracer_ipc_channel_clear(void)
{
	struct adv_tracer_ipc_ch *channel;

	channel = &eat_ipc->channel[EAT_FRM_CHANNEL];
	channel->cmd->cmd_raw.cmd = EAT_IPC_CMD_CH_CLEAR;
	adv_tracer_ipc_write_buffer(channel->buff_regs, channel->cmd, 1);
	adv_tracer_interrupt_gen(EAT_FRM_CHANNEL);
}

static void eat_ipc_dbgc_panic(void)
{
	u32 val = 0;

	eat_ipc->recovery = true;
	if (!eat_ipc->dbgc_config || !eat_ipc->dbgc_status) {
		dev_err(eat_ipc->dev, "pmu offset no data\n");
		return;
	}
	exynos_pmu_update(eat_ipc->dbgc_config, 1, 0);
	dev_emerg(eat_ipc->dev, "DBGC power off for recovery.\n");
	exynos_pmu_update(eat_ipc->dbgc_config, 1, 1);
	mdelay(10);
	exynos_pmu_read(eat_ipc->dbgc_status, &val);
	if (!val) {
		dev_err(eat_ipc->dev, "DBGC abnormal state!\n");
		panic("DBGC abnormal state!");
	}
}

static void eat_ipc_callback(struct adv_tracer_ipc_cmd *cmd, unsigned int len)
{
#ifdef ADV_TRACER_SSCOREDUMP
	struct sscd_platform_data *pdata = dev_get_platdata(&sscd_dev.dev);
#endif

	switch (cmd->cmd_raw.cmd) {
	case EAT_IPC_CMD_BOOT_DBGC:
		dev_info(eat_ipc->dev, "DBGC Boot!(cnt:%d)\n", cmd->buffer[1]);
		break;
	case EAT_IPC_CMD_EXCEPTION_DBGC:
		dev_err(eat_ipc->dev,
				"DBGC occurred exception! (SP:0x%x, LR:0x%x, PC:0x%x)\n",
				cmd->buffer[1], cmd->buffer[2], cmd->buffer[3]);
#ifdef ADV_TRACER_SSCOREDUMP
		if (pdata->sscd_report) {
			char msg[80];
			struct sscd_segment seg;

			memset(&seg, 0, sizeof(seg));
			seg.addr = (void *)cmd;
			seg.size = sizeof(*cmd);
			scnprintf(msg, sizeof(msg),
				"DBGC occurred exception! (SP: 0x%x, LR:0x%x, PC:0x%x)\n",
				cmd->buffer[1], cmd->buffer[2], cmd->buffer[3]);
			pdata->sscd_report(&sscd_dev, &seg, 1, 0, msg);
		}
#endif
		if (eat_ipc->recovery)
			panic("DBGC failed recovery!");
		eat_ipc_dbgc_panic();
		eat_ipc->recovery = true;
		break;
	default:
		break;
	}
}

static int adv_tracer_ipc_channel_probe(void)
{
	struct adv_tracer_ipc_cmd cmd;
	int i, ret;

	eat_ipc->num_channels = EAT_MAX_CHANNEL;
	eat_ipc->channel = devm_kcalloc(eat_ipc->dev, eat_ipc->num_channels,
			sizeof(struct adv_tracer_ipc_ch), GFP_KERNEL);

	if (IS_ERR(eat_ipc->channel))
		return PTR_ERR(eat_ipc->channel);

	for (i = 0; i < eat_ipc->num_channels; i++) {
		init_completion(&eat_ipc->channel[i].wait);
		spin_lock_init(&eat_ipc->channel[i].ch_lock);
		mutex_init(&eat_ipc->channel[i].wait_lock);
	}
	ret = adv_tracer_ipc_channel_init(EAT_FRM_CHANNEL, SR(16), 4,
			eat_ipc_callback, FRAMEWORK_NAME);
	if (ret) {
		dev_err(eat_info->dev, "failed to register Framework channel\n");
		return -EIO;
	}

	adv_tracer_ipc_channel_clear();

	cmd.cmd_raw.cmd = EAT_IPC_CMD_KERNEL_BOOT;
	ret = adv_tracer_ipc_send_data_async(EAT_FRM_CHANNEL, &cmd);
	if (ret)
		dev_err(eat_ipc->dev, "failed sending IPC to Framework. cmd(%d)\n",
				cmd.cmd_raw.cmd);

	return 0;
}

static int adv_tracer_ipc_init(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int ret = 0;
	struct cpumask aff_mask;
	const char *buf;

	eat_ipc = devm_kzalloc(&pdev->dev,
			sizeof(struct adv_tracer_ipc_main), GFP_KERNEL);
	if (!eat_ipc)
		return -ENODEV;

	/* Mailbox interrupt, AP -- Debug Core */
	eat_ipc->irq = platform_get_irq(pdev, 0);
	eat_ipc->recovery = false;

	/* Mailbox base register */
	eat_ipc->mailbox_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(eat_ipc->mailbox_base))
		return -ENODEV;

	adv_tracer_ipc_interrupt_clear(EAT_FRM_CHANNEL);
	ret = devm_request_threaded_irq(&pdev->dev, eat_ipc->irq,
			adv_tracer_ipc_irq_handler,
			adv_tracer_ipc_irq_handler_thread,
			IRQF_ONESHOT, dev_name(&pdev->dev), eat_ipc);
	if (ret) {
		dev_err(&pdev->dev, "failed to register interrupt: %d\n", ret);
		return ret;
	}
	ret = of_property_read_string(node, "irq_affinity", &buf);
	if (!ret) {
		cpulist_parse(buf, &aff_mask);
		ret = irq_set_affinity_hint(eat_ipc->irq, &aff_mask);
		if (ret) {
			dev_err(&pdev->dev, "failed to set interrupt affinity: %d\n", ret);
			return ret;
		}
	}
	eat_ipc->dev = &pdev->dev;

	ret = adv_tracer_ipc_channel_probe();
	if (ret) {
		dev_err(&pdev->dev, "failed to register channel: %d\n", ret);
		return ret;
	}

	if (of_property_read_u32(node, "pmu-dbgcore-config",
				&eat_ipc->dbgc_config))
		dev_err(&pdev->dev, "pmu-dbgcore-config is no data\n");
	if (of_property_read_u32(node, "pmu-dbgcore-status",
				&eat_ipc->dbgc_status))
		dev_err(&pdev->dev, "pmu-dbgcore-status is no data\n");
	if (of_property_read_u32(node, "intr-bitoffset",
				&eat_ipc->intr_bitoffset))
		eat_ipc->intr_bitoffset = INTR_FLAG_OFFSET;

#ifdef ADV_TRACER_SSCOREDUMP
	platform_device_register(&sscd_dev);
#endif

	dev_info(&pdev->dev, "%s successful.\n", __func__);
	return ret;
}

static int adv_tracer_probe(struct platform_device *pdev)
{
	struct adv_tracer_info *adv_tracer;
	int ret = 0;

	adv_tracer = devm_kzalloc(&pdev->dev, sizeof(struct adv_tracer_info), GFP_KERNEL);
	if (!adv_tracer)
		return -ENODEV;

	adv_tracer->dev = &pdev->dev;
	eat_info = adv_tracer;
	if (adv_tracer_ipc_init(pdev))
		goto out;

	dev_info(&pdev->dev, "%s successful.\n", __func__);
out:
	return ret;
}

static const struct of_device_id eat_match[] = {
	{ .compatible = "google,exynos-adv-tracer" },
	{},
};
MODULE_DEVICE_TABLE(of, eat_match);

static struct platform_driver eat_info_driver = {
	.probe	= adv_tracer_probe,
	.driver	= {
		.name = "exynos-adv-tracer",
		.owner	= THIS_MODULE,
		.of_match_table	= eat_match,
	},
};
module_platform_driver(eat_info_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos Advanced Tracer Driver");
