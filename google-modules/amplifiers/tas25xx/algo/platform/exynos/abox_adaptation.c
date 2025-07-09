/* sound/soc/samsung/abox/abox_adaptation.c
 *
 * ALSA SoC Audio Layer - Samsung Abox adaptation driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include "abox.h"
#include "abox_dma.h"
#include <sound/smart_amp.h>

#define TIMEOUT_MS 130
#define READ_WRITE_ALL_PARAM 0

#define DEBUG_ABOX_ADAPTATION

#ifdef DEBUG_ABOX_ADAPTATION
#define dbg_abox_adaptation(format, args...)	\
pr_info("[ABOX_ADAPTATION] %s: " format "\n", __func__, ## args)
#else
#define dbg_abox_adaptation(format, args...)
#endif /* DEBUG_ABOX_ADAPTATION */

static DECLARE_WAIT_QUEUE_HEAD(wq_read);
static DECLARE_WAIT_QUEUE_HEAD(wq_write);

struct abox_dma_data *dma_data;
struct maxim_dsm *read_maxdsm;

bool abox_ipc_irq_read_avail;
bool abox_ipc_irq_write_avail;
int dsm_offset;

#ifdef SMART_AMP
#define SMARTPA_ABOX_ERROR	0xF0F0F0F0
struct ti_smartpa_data *ti_smartpa_rd_data;
struct ti_smartpa_data ti_smartpa_rd_data_tmp;
struct ti_smartpa_data *ti_smartpa_wr_data;
struct ti_smartpa_data ti_smartpa_wr_data_tmp;
#endif /* SMART_AMP */

#ifdef SMART_AMP
int ti_smartpa_read(void *prm_data, int offset, int size)
{
	ABOX_IPC_MSG msg;
	int ret = 0;
	struct IPC_ERAP_MSG *erap_msg = &msg.msg.erap;

	ti_smartpa_rd_data = (struct ti_smartpa_data *)prm_data;

	msg.ipcid = IPC_ERAP;
	erap_msg->msgtype = REALTIME_EXTRA;
	erap_msg->param.raw.params[0] = TI_SMARTPA_VENDOR_ID;
	erap_msg->param.raw.params[1] = RD_DATA;
	erap_msg->param.raw.params[2] = offset;
	erap_msg->param.raw.params[3] = size;

	dbg_abox_adaptation("");
	abox_ipc_irq_read_avail = false;

	if (!dma_data) {
		pr_err("[TI-SmartPA:%s] dma_data is NULL", __func__);
		goto error;
	}

	ret = abox_request_ipc(dma_data->dev_abox, IPC_ERAP,
					 &msg, sizeof(msg), 0, 0);
	if (ret) {
		pr_err("%s: abox_request_ipc is failed: %d\n", __func__, ret);
		goto error;
	}

	ret = wait_event_interruptible_timeout(wq_read,
		abox_ipc_irq_read_avail != false, msecs_to_jiffies(TIMEOUT_MS));
	if (ret == 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto error;
	} else if (ret < 0) {
		pr_err("%s: wait_event error(%d)\n", __func__, ret);
		goto error;
	} else {
		memcpy(&ti_smartpa_rd_data->payload[0],
				&ti_smartpa_rd_data_tmp.payload[0],
				size);
	}

	if (((int32_t *)&ti_smartpa_rd_data->payload[0])[0] == SMARTPA_ABOX_ERROR)
		goto error;
	return 0;
error:
	return -1;
}
EXPORT_SYMBOL_GPL(ti_smartpa_read);

int ti_smartpa_write(void *prm_data, int offset, int size)
{
	ABOX_IPC_MSG msg;
	int ret = 0;
	struct IPC_ERAP_MSG *erap_msg = &msg.msg.erap;

	ti_smartpa_wr_data = (struct ti_smartpa_data *)prm_data;

	msg.ipcid = IPC_ERAP;
	erap_msg->msgtype = REALTIME_EXTRA;
	erap_msg->param.raw.params[0] = TI_SMARTPA_VENDOR_ID;
	erap_msg->param.raw.params[1] = WR_DATA;
	erap_msg->param.raw.params[2] = offset;
	erap_msg->param.raw.params[3] = size;

	memcpy(&erap_msg->param.raw.params[4],
		prm_data,
		min((sizeof(uint32_t) * size)
		, sizeof(erap_msg->param.raw)));

	dbg_abox_adaptation("");
#if 1 /* ToDo : TI AMP firmware don't send acknowledge about write IPC */
	abox_ipc_irq_write_avail = true;
#else
	abox_ipc_irq_write_avail = false;
#endif

	if (!dma_data) {
		pr_err("[TI-SmartPA:%s] dma_data is NULL", __func__);
		goto error;
	}

	ret = abox_request_ipc(dma_data->dev_abox, IPC_ERAP,
			&msg, sizeof(msg), 0, 0);
	if (ret) {
		pr_err("%s: abox_request_ipc is failed: %d\n", __func__, ret);
		goto error;
	}

	ret = wait_event_interruptible_timeout(wq_write,
		abox_ipc_irq_write_avail != false, msecs_to_jiffies(TIMEOUT_MS));
	if (ret == 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto error;
	} else if (ret < 0) {
		pr_err("%s: wait_event err(%d)\n", __func__, ret);
		goto error;
	} else {
		memcpy(&ti_smartpa_wr_data->payload[0],
				&ti_smartpa_wr_data_tmp.payload[0],
				size);
	}

	if (((int32_t *)&ti_smartpa_wr_data->payload[0])[0] == SMARTPA_ABOX_ERROR)
		goto error;
	return 0;
error:
	return -1;
}
EXPORT_SYMBOL_GPL(ti_smartpa_write);
#endif /* SMART_AMP */

static irqreturn_t abox_adaptation_irq_handler(int irq,
					void *dev_id, ABOX_IPC_MSG *msg)
{
	struct IPC_ERAP_MSG *erap_msg = &msg->msg.erap;
	irqreturn_t ret = IRQ_NONE;

	dbg_abox_adaptation("irq=%d, param[0]=%d",
				irq, erap_msg->param.raw.params[0]);

	switch (irq) {
	case IPC_ERAP:
		switch (erap_msg->msgtype) {
		case REALTIME_USB:
			/* relay to USB handler */
			/* pr_info("%s: type(0x%x)\n", __func__, erap_msg->msgtype); */
		break;
		case REALTIME_EXTRA:
#ifdef SMART_AMP
			if (erap_msg->param.raw.params[0] == TI_SMARTPA_VENDOR_ID) {
				if (erap_msg->param.raw.params[1] == RD_DATA) {
					memcpy(&ti_smartpa_rd_data_tmp.payload[0], &erap_msg->param.raw.params[4],
						min(sizeof(struct ti_smartpa_data), sizeof(erap_msg->param.raw)));
					abox_ipc_irq_read_avail = true;
					dbg_abox_adaptation("read_avail after parital read[%d]",
						abox_ipc_irq_read_avail);
					if (abox_ipc_irq_read_avail && waitqueue_active(&wq_read))
						wake_up_interruptible(&wq_read);
				} else if (erap_msg->param.raw.params[1] == WR_DATA) {
					memcpy(&ti_smartpa_wr_data_tmp.payload[0], &erap_msg->param.raw.params[4],
						min(sizeof(struct ti_smartpa_data), sizeof(erap_msg->param.raw)));
					abox_ipc_irq_write_avail = true;
					dbg_abox_adaptation("write_avail after parital read[%d]",
						abox_ipc_irq_write_avail);
					if (abox_ipc_irq_write_avail && waitqueue_active(&wq_write))
						wake_up_interruptible(&wq_write);
				} else {
					pr_err("[TI-SmartPA] %s: Invalid callback, %d", __func__,
						erap_msg->param.raw.params[1]);
				}
			}
			ret = IRQ_HANDLED;
#endif /* SMART_AMP */
		break;
		default:
			pr_err("%s: unknown message type(%d)\n", __func__, erap_msg->msgtype);
		break;
		}
	break;
	default:
		pr_err("%s: unknown command\n", __func__);
	break;
	}
	return ret;
}

static struct snd_soc_component_driver abox_adaptation_cmpt_drv = {
};

static struct snd_soc_dai_driver abox_adaptation_dai_drv = {
};

static int samsung_abox_adaptation_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *np_abox;
	struct platform_device *pdev_abox;
	int ret = 0;

	dma_data = devm_kzalloc(dev, sizeof(*dma_data), GFP_KERNEL);
	if (!dma_data) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, dma_data);

	dsm_offset = READ_WRITE_ALL_PARAM;

	np_abox = of_parse_phandle(np, "samsung,abox", 0);
	if (!np_abox) {
		dev_err(dev, "Failed to get abox device node\n");
		return -EPROBE_DEFER;
	}
	pdev_abox = of_find_device_by_node(np_abox);
	dma_data->dev_abox = &pdev_abox->dev;
	if (!dma_data->dev_abox) {
		dev_err(dev, "Failed to get abox platform device\n");
		return -EPROBE_DEFER;
	}
	dma_data->abox_data = dev_get_drvdata(dma_data->dev_abox);

	abox_register_ipc_handler(dma_data->dev_abox, IPC_ERAP,
			abox_adaptation_irq_handler, dma_data->abox_data);

	dev_info(dev, "%s\n", __func__);

	ret = devm_snd_soc_register_component(&pdev->dev, &abox_adaptation_cmpt_drv,
			&abox_adaptation_dai_drv, 1);
	if (ret < 0)
		dev_err(dev, "register component failed: %d\n", ret);

	return ret;
}

static int samsung_abox_adaptation_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id samsung_abox_adaptation_match[] = {
	{
		.compatible = "samsung,abox-adaptation",
	},
	{},
};
MODULE_DEVICE_TABLE(of, samsung_abox_adaptation_match);

static struct platform_driver samsung_abox_adaptation_driver = {
	.probe  = samsung_abox_adaptation_probe,
	.remove = samsung_abox_adaptation_remove,
	.driver = {
		.name = "samsung-abox-adaptation",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(samsung_abox_adaptation_match),
	},
};
module_platform_driver(samsung_abox_adaptation_driver);

/* Module information */
MODULE_AUTHOR("SeokYoung Jang, <quartz.jang@samsung.com>");
MODULE_DESCRIPTION("Samsung ASoC A-Box Adaptation Driver");
MODULE_ALIAS("platform:samsung-abox-adaptation");
MODULE_LICENSE("GPL");
