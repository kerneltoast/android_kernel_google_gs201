/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * The main source file of Samsung Exynos SMFC Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mutex.h>

#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-sg.h>

#include "smfc.h"
#include "smfc-regs.h"

static irqreturn_t exynos_smfc_irq_handler(int irq, void *priv)
{
	return IRQ_HANDLED;
}

static int smfc_vb2_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
				unsigned int *num_planes, unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct smfc_ctx *ctx = vb2_get_drv_priv(vq);
	unsigned int i;

	/* FIXME: correct output*/
	*num_planes = 1;
	for (i = 0; i < *num_planes; i++) {
		sizes[i] = 100000;
		alloc_devs[i] = ctx->smfc->dev;
	}

	return 0;
}

static int smfc_vb2_buf_prepare(struct vb2_buffer *vb)
{
	/* TODO: fix configuration of payload */

	return 0;
}

static void smfc_vb2_buf_finish(struct vb2_buffer *vb)
{
}

static void smfc_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct smfc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(ctx->m2mctx, to_vb2_v4l2_buffer(vb));
}

static void smfc_vb2_lock(struct vb2_queue *vq)
{
	struct smfc_ctx *ctx = vb2_get_drv_priv(vq);
	mutex_lock(&ctx->smfc->video_device_mutex);
}

static void smfc_vb2_unlock(struct vb2_queue *vq)
{
	struct smfc_ctx *ctx = vb2_get_drv_priv(vq);
	mutex_unlock(&ctx->smfc->video_device_mutex);
}

static struct vb2_ops smfc_vb2_ops = {
	.queue_setup	= smfc_vb2_queue_setup,
	.buf_prepare	= smfc_vb2_buf_prepare,
	.buf_finish	= smfc_vb2_buf_finish,
	.buf_queue	= smfc_vb2_buf_queue,
	.wait_finish	= smfc_vb2_lock,
	.wait_prepare	= smfc_vb2_unlock,
};

static int smfc_queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct smfc_ctx *ctx = priv;
	int ret;

	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->ops = &smfc_vb2_ops;
	src_vq->mem_ops = &vb2_dma_sg_memops;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->ops = &smfc_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_sg_memops;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;

	return vb2_queue_init(dst_vq);
}

static int exynos_smfc_open(struct file *filp)
{
	struct smfc_dev *smfc = video_drvdata(filp);
	struct smfc_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(smfc->dev, "Failed to allocate smfc_ctx");
		return -ENOMEM;
	}

	ctx->m2mctx = v4l2_m2m_ctx_init(smfc->m2mdev, ctx, smfc_queue_init);
	if (IS_ERR(ctx->m2mctx)) {
		ret = PTR_ERR(ctx->m2mctx);
		dev_err(smfc->dev, "Failed(%d) to init m2m_ctx\n", ret);
		goto err_m2m_ctx_init;
	}


	v4l2_fh_init(&ctx->v4l2_fh, smfc->videodev);
	v4l2_fh_add(&ctx->v4l2_fh);

	filp->private_data = &ctx->v4l2_fh;

	if (!IS_ERR(smfc->clk_gate)) {
		ret = clk_prepare(smfc->clk_gate);
		if (!ret && !IS_ERR(smfc->clk_gate2))
			ret = clk_prepare(smfc->clk_gate2);
		if (ret) {
			clk_unprepare(smfc->clk_gate);
			dev_err(smfc->dev,
				"Failed(%d) to prepare gate clocks\n", ret);
			goto err_clk;
		}
	}

	ctx->smfc = smfc;

	return 0;
err_clk:
	v4l2_fh_del(&ctx->v4l2_fh);
	v4l2_fh_exit(&ctx->v4l2_fh);
	v4l2_m2m_ctx_release(ctx->m2mctx);
err_m2m_ctx_init:
	kfree(ctx);
	return ret;
}

static int exynos_smfc_release(struct file *filp)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(filp->private_data);

	v4l2_fh_del(&ctx->v4l2_fh);
	v4l2_m2m_ctx_release(ctx->m2mctx);

	if (!IS_ERR(ctx->smfc->clk_gate)) {
		clk_unprepare(ctx->smfc->clk_gate);
		if (!IS_ERR(ctx->smfc->clk_gate2))
			clk_unprepare(ctx->smfc->clk_gate2);
	}

	kfree(ctx);

	return 0;
}

static unsigned int exynos_smfc_poll(struct file *filp,
				     struct poll_table_struct *wait)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(filp->private_data);
	return v4l2_m2m_poll(filp, ctx->m2mctx, wait);
}

static int exynos_smfc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct smfc_ctx *ctx = v4l2_fh_to_smfc_ctx(filp->private_data);
	return v4l2_m2m_mmap(filp, ctx->m2mctx, vma);
}

static const struct v4l2_file_operations smfc_v4l2_fops = {
	.owner		= THIS_MODULE,
	.open		= exynos_smfc_open,
	.release	= exynos_smfc_release,
	.poll		= exynos_smfc_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= exynos_smfc_mmap,
};

static const struct v4l2_ioctl_ops smfc_v4l2_ioctl_ops = {
};

static void smfc_m2m_device_run(void *priv)
{
	/* TODO: H/W initialization */
}

static void smfc_m2m_job_abort(void *priv)
{
	/* TODO: aborting next job */
}

static struct v4l2_m2m_ops smfc_m2m_ops = {
	.device_run	= smfc_m2m_device_run,
	.job_abort	= smfc_m2m_job_abort,
};

static int smfc_init_v4l2(struct device *dev, struct smfc_dev *smfc)
{
	int ret;
	size_t str_len;

	strncpy(smfc->v4l2_dev.name, "exynos-hwjpeg",
		sizeof(smfc->v4l2_dev.name) - 1);
	smfc->v4l2_dev.name[sizeof(smfc->v4l2_dev.name) - 1] = '\0';

	ret = v4l2_device_register(dev, &smfc->v4l2_dev);
	if (ret) {
		dev_err(dev, "Failed to register v4l2 device\n");
		return ret;
	}

	smfc->videodev = video_device_alloc();
	if (!smfc->videodev) {
		dev_err(dev, "Failed to allocate video_device");
		ret = -ENOMEM;
		goto err_videodev_alloc;
	}

	str_len = sizeof(smfc->videodev->name);
	if (smfc->device_id < 0) {
		strncpy(smfc->videodev->name, MODULE_NAME, str_len);
		smfc->videodev->name[str_len - 1] = '\0';
	} else {
		scnprintf(smfc->videodev->name, str_len,
			  "%s.%d", MODULE_NAME, smfc->device_id);
	}

	mutex_init(&smfc->video_device_mutex);

	smfc->videodev->fops		= &smfc_v4l2_fops;
	smfc->videodev->ioctl_ops	= &smfc_v4l2_ioctl_ops;
	smfc->videodev->release		= video_device_release;
	smfc->videodev->lock		= &smfc->video_device_mutex;
	smfc->videodev->vfl_dir		= VFL_DIR_M2M;
	smfc->videodev->v4l2_dev	= &smfc->v4l2_dev;

	video_set_drvdata(smfc->videodev, smfc);

	smfc->m2mdev = v4l2_m2m_init(&smfc_m2m_ops);
	if (IS_ERR(smfc->m2mdev)) {
		ret = PTR_ERR(smfc->m2mdev);
		dev_err(dev, "Failed(%d) to create v4l2_m2m_device\n", ret);
		goto err_m2m_init;
	}

	/* TODO: promote the magic number 12 in public */
	ret = video_register_device(smfc->videodev, VFL_TYPE_GRABBER, 12);
	if (ret < 0) {
		dev_err(dev, "Failed(%d) to register video_device[%d]\n",
			ret, 12);
		goto err_register;
	}

	return 0;

err_register:
	v4l2_m2m_release(smfc->m2mdev);
err_m2m_init:
	video_device_release(smfc->videodev);
err_videodev_alloc:
	v4l2_device_unregister(&smfc->v4l2_dev);
	return ret;
}

static int smfc_init_clock(struct device *dev, struct smfc_dev *smfc)
{
	smfc->clk_gate = devm_clk_get(dev, "gate");
	if (IS_ERR(smfc->clk_gate)) {
		if (PTR_ERR(smfc->clk_gate) != -ENOENT) {
			dev_err(dev, "Failed(%ld) to get 'gate' clock",
				PTR_ERR(smfc->clk_gate));
			return PTR_ERR(smfc->clk_gate);
		}

		dev_info(dev, "'gate' clock is note present\n");
		smfc->clk_gate2 = ERR_PTR(-ENOENT);
		return 0;
	}

	smfc->clk_gate2 = devm_clk_get(dev, "gate2");
	if (IS_ERR(smfc->clk_gate2) && (PTR_ERR(smfc->clk_gate2) != -ENOENT)) {
		dev_err(dev, "Failed(%ld) to get 'gate2' clock\n",
				PTR_ERR(smfc->clk_gate2));
		clk_put(smfc->clk_gate);
		return PTR_ERR(smfc->clk_gate2);
	}

	return 0;
}

static int smfc_find_hw_version(struct device *dev, struct smfc_dev *smfc)
{
	int ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "Failed(%d) to get the local power\n", ret);
		return ret;
	}

	if (!IS_ERR(smfc->clk_gate)) {
		ret = clk_prepare_enable(smfc->clk_gate);
		if (!ret && !IS_ERR(smfc->clk_gate2))
			ret = clk_prepare_enable(smfc->clk_gate2);
		if (ret) {
			clk_disable_unprepare(smfc->clk_gate);
			dev_err(dev, "Failed(%d) to get gate clocks\n", ret);
			goto err_clk;
		}
	}

	if (ret >= 0) {
		smfc->hwver = readl(smfc->reg + REG_IP_VERSION_NUMBER);
		if (!IS_ERR(smfc->clk_gate)) {
			clk_disable_unprepare(smfc->clk_gate);
			if (!IS_ERR(smfc->clk_gate2))
				clk_disable_unprepare(smfc->clk_gate2);
		}
	}

err_clk:
	pm_runtime_put(dev);

	return ret;
}

static int exynos_smfc_probe(struct platform_device *pdev)
{
	struct smfc_dev *smfc;
	struct resource *res;
	int ret;

	smfc = devm_kzalloc(&pdev->dev, sizeof(*smfc), GFP_KERNEL);
	if (!smfc) {
		dev_err(&pdev->dev, "Failed to get allocate drvdata");
		return -ENOMEM;
	}

	smfc->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	smfc->reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(smfc->reg))
		return PTR_ERR(smfc->reg);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Failed to get IRQ resource");
		return -ENOENT;
	}

	ret = devm_request_irq(&pdev->dev, res->start, exynos_smfc_irq_handler,
				0, pdev->name, smfc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to install IRQ handler");
		return ret;
	}

	ret = smfc_init_clock(&pdev->dev, smfc);
	if (ret)
		return ret;

	smfc->device_id = of_alias_get_id(pdev->dev.of_node, "jpeg");
	if (smfc->device_id < 0) {
		dev_info(&pdev->dev,
			"device ID is not declared: unique device\n");
		smfc->device_id = -1;
	}

	pm_runtime_enable(&pdev->dev);

	ret = smfc_find_hw_version(&pdev->dev, smfc);
	if (ret < 0)
		return ret;

	ret = smfc_init_v4l2(&pdev->dev, smfc);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, smfc);

	dev_info(&pdev->dev, "Probed H/W Version: %02x.%02x.%04x\n",
			(smfc->hwver >> 24) & 0xFF, (smfc->hwver >> 16) & 0xFF,
			smfc->hwver & 0xFFFF);
	return 0;
}

static void smfc_deinit_clock(struct smfc_dev *smfc)
{
	if (!IS_ERR(smfc->clk_gate2))
		clk_put(smfc->clk_gate2);
	if (!IS_ERR(smfc->clk_gate))
		clk_put(smfc->clk_gate);
}

static int exynos_smfc_remove(struct platform_device *pdev)
{
	struct smfc_dev *smfc = platform_get_drvdata(pdev);

	smfc_deinit_clock(smfc);

	return 0;
}

static const struct of_device_id exynos_smfc_match[] = {
	{
		.compatible = "samsung,exynos-jpeg",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_smfc_match);

#ifdef CONFIG_PM_SLEEP
static int smfc_suspend(struct device *dev)
{
	return 0;
}

static int smfc_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM
static int smfc_runtime_resume(struct device *dev)
{
	return 0;
}

static int smfc_runtime_suspend(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops exynos_smfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(smfc_suspend, smfc_resume)
	SET_RUNTIME_PM_OPS(NULL, smfc_runtime_resume, smfc_runtime_suspend)
};

static struct platform_driver exynos_smfc_driver = {
	.probe		= exynos_smfc_probe,
	.remove		= exynos_smfc_remove,
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &exynos_smfc_pm_ops,
		.of_match_table = of_match_ptr(exynos_smfc_match),
	}
};

module_platform_driver(exynos_smfc_driver);

MODULE_AUTHOR("Cho KyongHo <pullip.cho@samsung.com>");
MODULE_DESCRIPTION("Exynos Still MFC(JPEG) V4L2 Driver");
MODULE_LICENSE("GPL");
