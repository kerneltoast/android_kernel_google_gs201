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
#include <sound/samsung/abox.h>
#include <algo/inc/tas_smart_amp_v2.h>
#include <sound/sec_adaptation.h>
#include "abox.h"

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

struct abox_platform_data *data;
struct maxim_dsm *read_maxdsm;

bool rd_avail;
bool wr_avail;
int dsm_offset;

#ifdef CONFIG_TAS25XX_ALGO
#define SMARTPA_ABOX_ERROR	0xF0F0F0F0
typedef struct ti_smartpa_data tisa_data_t;
tisa_data_t *rd_data;
tisa_data_t rd_data_tmp;
tisa_data_t *wr_data;
tisa_data_t wr_data_tmp;
#endif /* CONFIG_TAS25XX_ALGO */

#ifdef CONFIG_TAS25XX_ALGO
int ti_smartpa_read(void *prm_data, int offset, int size)
{
	ABOX_IPC_MSG msg;
	int ret = 0;
	struct IPC_ERAP_MSG *msg = &msg.msg.erap;

	rd_data = (tisa_data_t *)prm_data;

	msg.ipcid = IPC_ERAP;
	msg->msgtype = REALTIME_EXTRA;
	msg->param.raw.params[0] = TI_SMARTPA_VENDOR_ID;
	msg->param.raw.params[1] = RD_DATA;
	msg->param.raw.params[2] = offset;
	msg->param.raw.params[3] = size;

	dbg_abox_adaptation("");
	rd_avail = false;

	if (!data) {
		pr_err("[TI-SmartPA:%s] data is NULL", __func__);
		goto error;
	}

	ret = abox_request_ipc(&data->pdev_abox->dev, IPC_ERAP,
					 &msg, sizeof(msg), 0, 0);
	if (ret) {
		pr_err("%s: abox_request_ipc is failed: %d\n", __func__, ret);
		goto error;
	}

	ret = wait_event_interruptible_timeout(wq_read,
		rd_avail != false, msecs_to_jiffies(TIMEOUT_MS));
	if (ret == 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		goto error;
	} else if (ret < 0) {
		pr_err("%s: wait_event error(%d)\n", __func__, ret);
		goto error;
	} else {
		memcpy(&rd_data->payload[0],
				&rd_data_tmp.payload[0], size);
	}

	if (((int32_t *)&rd_data->payload[0])[0] == SMARTPA_ABOX_ERROR)
		goto error;
	return 0;

error:
	return -ENODATA;
}
EXPORT_SYMBOL_GPL(ti_smartpa_read);

int ti_smartpa_write(void *prm_data, int offset, int size)
{
	ABOX_IPC_MSG msg;
	int ret = 0;
	struct IPC_ERAP_MSG *msg = &msg.msg.erap;

	wr_data = (tisa_data_t *)prm_data;
	msg.ipcid = IPC_ERAP;
	msg->msgtype = REALTIME_EXTRA;
	msg->param.raw.params[0] = TI_SMARTPA_VENDOR_ID;
	msg->param.raw.params[1] = WR_DATA;
	msg->param.raw.params[2] = offset;
	msg->param.raw.params[3] = size;

	memcpy(&msg->param.raw.params[4], prm_data,
		min((sizeof(uint32_t) * size), sizeof(msg->param.raw)));

	dbg_abox_adaptation("");
	wr_avail = false;

	if (!data) {
		pr_err("[TI-SmartPA:%s] data is NULL", __func__);
		goto error;
	}

	ret = abox_request_ipc(&data->pdev_abox->dev, IPC_ERAP,
					 &msg, sizeof(msg), 0, 0);
	if (ret) {
		pr_err("%s: abox_request_ipc is failed: %d\n", __func__, ret);
		goto error;
	}

	ret = wait_event_interruptible_timeout(wq_write,
		wr_avail != false, msecs_to_jiffies(TIMEOUT_MS));
	if (ret == 0) {
		pr_err("%s: wait_event timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto error;
	} else if (ret < 0) {
		pr_err("%s: wait_event err(%d)\n", __func__, ret);
		goto error;
	} else {
		memcpy(&wr_data->payload[0],
				&wr_data_tmp.payload[0], size);
	}

	if (((int32_t *)&wr_data->payload[0])[0] == SMARTPA_ABOX_ERROR) {
		ret = -ENODATA;
		goto error;
	}
	return 0;

error:
	return ret;
}
EXPORT_SYMBOL_GPL(ti_smartpa_write);
#endif /* CONFIG_TAS25XX_ALGO */

static irqreturn_t abox_adaptation_irq_handler(int irq,
					void *dev_id, ABOX_IPC_MSG *msg)
{
	struct IPC_ERAP_MSG *msg = &msg->msg.erap;

	dbg_abox_adaptation("irq=%d, param[0]=%d avail(%d,%d)",
				irq, msg->param.raw.params[0],
				rd_avail, wr_avail);

	switch (irq) {
	case IPC_ERAP:
		switch (msg->msgtype) {
		case REALTIME_EXTRA:
#ifdef CONFIG_TAS25XX_ALGO
			if (msg->param.raw.params[0] == TI_SMARTPA_VENDOR_ID) {
				if (msg->param.raw.params[1] == RD_DATA) {
					memcpy(&rd_data_tmp.payload[0], &msg->param.raw.params[4],
						min(sizeof(tisa_data_t), sizeof(msg->param.raw)));
					rd_avail = true;
					dbg_abox_adaptation("read_avail after parital read[%d]",
						rd_avail);
					if (rd_avail && waitqueue_active(&wq_read))
						wake_up_interruptible(&wq_read);
				} else if (msg->param.raw.params[1] == WR_DATA) {
					memcpy(&wr_data_tmp.payload[0], &msg->param.raw.params[4],
						min(sizeof(tisa_data_t), sizeof(msg->param.raw)));
					wr_avail = true;
					dbg_abox_adaptation("write_avail after parital read[%d]",
						wr_avail);
					if (wr_avail && waitqueue_active(&wq_write))
						wake_up_interruptible(&wq_write);
				} else {
					pr_err("[TI-SmartPA] %s: Invalid callback, %d", __func__,
						msg->param.raw.params[1]);
				}
			}
#endif /* CONFIG_TAS25XX_ALGO */
			break;

		default:
			pr_err("%s: unknown message type\n", __func__);
			break;

		}
		break;

	default:
		pr_err("%s: unknown command\n", __func__);
		break;
	}

	return IRQ_HANDLED;
}

static struct snd_soc_platform_driver abox_adaptation = {
};

static int samsung_abox_adaptation_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *np_abox;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, data);
	dsm_offset = READ_WRITE_ALL_PARAM;

	np_abox = of_parse_phandle(np, "abox", 0);
	if (!np_abox) {
		dev_err(dev, "Failed to get abox device node\n");
		return -EPROBE_DEFER;
	}

	data->pdev_abox = of_find_device_by_node(np_abox);
	if (!data->pdev_abox) {
		dev_err(dev, "Failed to get abox platform device\n");
		return -EPROBE_DEFER;
	}

	data->abox_data = platform_get_drvdata(data->pdev_abox);
	abox_register_irq_handler(&data->pdev_abox->dev, IPC_ERAP,
			abox_adaptation_irq_handler, pdev);

	dev_info(dev, "%s exit\n", __func__);

	return snd_soc_register_platform(&pdev->dev, &abox_adaptation);
}

static int samsung_abox_adaptation_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
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
