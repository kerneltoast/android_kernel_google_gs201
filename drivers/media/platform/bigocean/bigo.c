// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for BigOcean video accelerator
 *
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/platform_data/sscoredump.h>
#include <linux/soc/samsung/exynos-smc.h>

#include "bigo_io.h"
#include "bigo_iommu.h"
#include "bigo_of.h"
#include "bigo_pm.h"
#include "bigo_priv.h"
#include "bigo_slc.h"
#include "bigo_debug.h"

#define BIGO_DEVCLASS_NAME "video_codec"
#define BIGO_CHRDEV_NAME "bigocean"

#define DEFAULT_WIDTH 3840
#define DEFAULT_HEIGHT 2160
#define DEFAULT_FPS 60
#define BIGO_SMC_ID 0xd
#define BIGO_MAX_INST_NUM 16

static struct sscd_platform_data bigo_sscd_platdata;

static void bigo_sscd_release(struct device *dev)
{
	pr_debug("sscd release\n");
}

static struct platform_device bigo_sscd_dev = {
	.name            = BIGO_CHRDEV_NAME,
	.driver_override = SSCD_NAME,
	.id              = -1,
	.dev             = {
		.platform_data = &bigo_sscd_platdata,
		.release       = bigo_sscd_release,
	},
};

static void bigo_coredump(struct bigo_core *core, const char *crash_info)
{
	struct sscd_platform_data *sscd_platdata = &bigo_sscd_platdata;
	struct sscd_segment seg;

	if (!sscd_platdata->sscd_report)
		return;

	memset(&seg, 0, sizeof(seg));
	seg.addr = (void *)core->base;
	seg.size = core->regs_size;
	seg.flags = 0;
	seg.paddr = (void *)core->paddr;
	seg.vaddr = (void *)core->base;

	sscd_platdata->sscd_report(&bigo_sscd_dev, &seg, 1,
		SSCD_FLAGS_ELFARM64HDR, crash_info);
}

static inline int on_first_instance_open(struct bigo_core *core)
{
	int rc = bigo_pt_client_enable(core);

	if (rc)
		pr_info("failed to enable SLC");
#if IS_ENABLED(CONFIG_PM)
	rc = pm_runtime_get_sync(core->dev);
	if (rc)
		pr_err("failed to resume: %d\n", rc);
#endif
	return rc;
}

static inline void on_last_inst_close(struct bigo_core *core)
{
#if IS_ENABLED(CONFIG_PM)
	if (pm_runtime_put_sync_suspend(core->dev))
		pr_warn("failed to suspend\n");
#endif
	bigo_pt_client_disable(core);
	kfree(core->job.regs);
	core->job.regs = NULL;
}

static inline int bigo_add_inst(struct bigo_inst *inst, struct bigo_core *core)
{
	int count = 0;
	struct list_head *pos;

	list_for_each(pos, &core->instances)
		count++;

	if (count >= BIGO_MAX_INST_NUM)
		return -ENOMEM;

	list_add_tail(&inst->list, &core->instances);
	return 0;
}

static int bigo_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct bigo_core *core = container_of(inode->i_cdev, struct bigo_core, cdev);
	struct bigo_inst *inst;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst) {
		rc = -ENOMEM;
		pr_err("Failed to create instance\n");
		return rc;
	}
	INIT_LIST_HEAD(&inst->list);
	INIT_LIST_HEAD(&inst->buffers);
	mutex_init(&inst->lock);
	file->private_data = inst;
	inst->height = DEFAULT_WIDTH;
	inst->width = DEFAULT_HEIGHT;
	inst->fps = DEFAULT_FPS;
	inst->core = core;
	mutex_lock(&core->lock);
	if (list_empty(&core->instances)) {
		rc = on_first_instance_open(core);
		if (rc) {
			pr_err("failed to setup first instance");
			goto err;
		}
	}
	rc = bigo_add_inst(inst, core);
	if (rc) {
		pr_err("Reaches max number of supported instances\n");
		goto err;
	}
	mutex_unlock(&core->lock);
	bigo_update_qos(core);
	pr_info("opened bigocean instance\n");

	return 0;
err:
	mutex_unlock(&core->lock);
	kfree(inst);
	return rc;
}

static int bigo_release(struct inode *inode, struct file *file)
{
	struct bigo_core *core =
		container_of(inode->i_cdev, struct bigo_core, cdev);
	struct bigo_inst *inst = file->private_data;

	if (!inst || !core) {
		pr_err("No instance or core\n");
		return -EINVAL;
	}
	bigo_unmap_all(inst);
	mutex_lock(&core->lock);
	list_del(&inst->list);
	kfree(inst);
	if (list_empty(&core->instances))
		on_last_inst_close(core);
	mutex_unlock(&core->lock);
	bigo_update_qos(core);
	pr_info("closed bigocean instance\n");
	return 0;
}

static int bigo_run_job(struct bigo_core *core, struct bigo_job *job)
{
	long ret = 0;
	int rc = 0;
	u32 status = 0;

	bigo_bypass_ssmt_pid(core);
	bigo_push_regs(core, job->regs);
	bigo_core_enable(core);
	ret = wait_for_completion_timeout(&core->frame_done,
			msecs_to_jiffies(JOB_COMPLETE_TIMEOUT_MS));
	if (!ret) {
		pr_err("timed out waiting for HW\n");
		bigo_core_disable(core);
		rc = -ETIMEDOUT;
	} else {
		rc = 0;
	}

	status = bigo_check_status(core);
	ret = bigo_wait_disabled(core, BIGO_DISABLE_TIMEOUT_MS);
	if (rc || ret || core->debugfs.trigger_ssr) {
		if(core->debugfs.trigger_ssr)
			rc = -EFAULT;
		pr_err("timed out or failed to disable hw: %d, %ld, status: 0x%x\n",
				rc, ret, status);
		bigo_coredump(core, "bigo_timeout");
	}

	bigo_pull_regs(core, job->regs);
	*(u32 *)(job->regs + BIGO_REG_STAT) = status;
	if (rc || ret)
		rc = -ETIMEDOUT;
	return rc;
}

static int bigo_process(struct bigo_core *core, struct bigo_ioc_regs *desc)
{
	int rc = 0;
	struct bigo_job *job = &core->job;

	if (!desc) {
		pr_err("Invalid input\n");
		return -EINVAL;
	}
	if (desc->regs_size != core->regs_size) {
		pr_err("Register size passed from userspace(%u) is different(%u)\n",
		       (unsigned int)desc->regs_size, core->regs_size);
		return -EINVAL;
	}

	mutex_lock(&core->lock);

	if (!job->regs) {
		job->regs = kzalloc(core->regs_size, GFP_KERNEL);
		if (!job->regs) {
			rc = -ENOMEM;
			goto unlock;
		}
	}

	if (copy_from_user(job->regs, (void *)desc->regs, core->regs_size)) {
		pr_err("Failed to copy from user\n");
		rc = -EFAULT;
		goto unlock;
	}

	/*TODO(vinaykalia@): Replace this with EDF scheduler.*/
	rc = bigo_run_job(core, job);
	if (rc) {
		pr_err("Error running job\n");
		goto unlock;
	}

	if (copy_to_user((void *)desc->regs, job->regs, core->regs_size)) {
		pr_err("Failed to copy to user\n");
		rc = -EFAULT;
		goto unlock;
	}

unlock:
	mutex_unlock(&core->lock);
	return rc;
}

inline void bigo_config_frmrate(struct bigo_inst *inst, __u32 frmrate)
{
	mutex_lock(&inst->lock);
	inst->fps = frmrate;
	mutex_unlock(&inst->lock);
	bigo_update_qos(inst->core);
}

inline void bigo_config_frmsize(struct bigo_inst *inst,
				struct bigo_ioc_frmsize *frmsize)
{
	mutex_lock(&inst->lock);
	inst->height = frmsize->height;
	inst->width = frmsize->width;
	mutex_unlock(&inst->lock);
	bigo_update_qos(inst->core);
}

inline void bigo_config_secure(struct bigo_inst *inst, __u32 is_secure)
{
	mutex_lock(&inst->lock);
	inst->is_secure = is_secure;
	mutex_unlock(&inst->lock);
}

static long bigo_unlocked_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct bigo_inst *inst = file->private_data;
	struct bigo_core *core =
		container_of(file->f_inode->i_cdev, struct bigo_core, cdev);
	void __user *user_desc = (void __user *)arg;
	struct bigo_ioc_regs desc;
	struct bigo_ioc_mapping mapping;
	struct bigo_ioc_frmsize frmsize;
	struct bigo_cache_info cinfo;
	int rc = 0;

	if (_IOC_TYPE(cmd) != BIGO_IOC_MAGIC) {
		pr_err("Bad IOCTL\n");
		return -EINVAL;
	}
	if (_IOC_NR(cmd) > BIGO_CMD_MAXNR) {
		pr_err("Bad IOCTL\n");
		return -EINVAL;
	}
	if (!inst || !core) {
		pr_err("No instance or core\n");
		return -EINVAL;
	}
	switch (cmd) {
	case BIGO_IOCX_PROCESS:
		if (copy_from_user(&desc, user_desc, sizeof(desc))) {
			pr_err("Failed to copy from user\n");
			return -EFAULT;
		}

		if (inst->is_secure) {
			rc = exynos_smc(SMC_PROTECTION_SET, 0, BIGO_SMC_ID,
					SMC_PROTECTION_ENABLE);
			if (rc) {
				pr_err("failed to enable SMC_PROTECTION_SET: %d\n", rc);
				break;
			}
		}

		rc = bigo_process(core, &desc);
		if (rc)
			pr_err("Error processing data: %d\n", rc);

		if (inst->is_secure) {
			rc = exynos_smc(SMC_PROTECTION_SET, 0, BIGO_SMC_ID,
					SMC_PROTECTION_DISABLE);
			if (rc)
				pr_err("failed to disable SMC_PROTECTION_SET: %d\n", rc);
		}
		break;
	case BIGO_IOCX_MAP:
		if (copy_from_user(&mapping, user_desc, sizeof(mapping))) {
			pr_err("Failed to copy from user\n");
			return -EFAULT;
		}
		rc = bigo_map(core, inst, &mapping);
		if (rc)
			pr_err("Error mapping: %d\n", mapping.fd);
		if (copy_to_user(user_desc, &mapping, sizeof(mapping))) {
			pr_err("Failed to copy to user\n");
			rc = -EFAULT;
		}
		break;
	case BIGO_IOCX_UNMAP:
		if (copy_from_user(&mapping, user_desc, sizeof(mapping))) {
			pr_err("Failed to copy from user\n");
			return -EFAULT;
		}
		rc = bigo_unmap(inst, &mapping);
		if (rc)
			pr_err("Error un-mapping: %d\n", mapping.fd);
		break;
	case BIGO_IOCX_CONFIG_FRMRATE: {
		u32 frmrate = (u32)arg;

		bigo_config_frmrate(inst, frmrate);
		break;
	}
	case BIGO_IOCX_CONFIG_FRMSIZE:
		if (copy_from_user(&frmsize, user_desc, sizeof(frmsize))) {
			pr_err("Failed to copy from user\n");
			return -EFAULT;
		}
		bigo_config_frmsize(inst, &frmsize);
		break;
	case BIGO_IOCX_GET_CACHE_INFO:
		bigo_get_cache_info(inst->core, &cinfo);
		if (copy_to_user(user_desc, &cinfo, sizeof(cinfo))) {
			pr_err("Failed to copy to user\n");
			rc = -EFAULT;
		}
		break;
	case BIGO_IOCX_CONFIG_SECURE: {
		u32 is_secure = (u32)arg;
		bigo_config_secure(inst, is_secure);
		break;
	}
	case BIGO_IOCX_ABORT:
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static irqreturn_t bigo_isr(int irq, void *arg)
{
	struct bigo_core *core = (struct bigo_core *)arg;
	u32 bigo_stat;
	unsigned long flags;

	bigo_stat = bigo_core_readl(core, BIGO_REG_STAT);

	if (!(bigo_stat & BIGO_STAT_IRQ))
		return IRQ_NONE;

	spin_lock_irqsave(&core->status_lock, flags);
	core->stat_with_irq = bigo_stat;
	spin_unlock_irqrestore(&core->status_lock, flags);
	bigo_stat &= ~BIGO_STAT_IRQMASK;
	bigo_core_writel(core, BIGO_REG_STAT, bigo_stat);
	complete(&core->frame_done);
	return IRQ_HANDLED;
}

#if IS_ENABLED(CONFIG_PM)
static const struct dev_pm_ops bigo_pm_ops = {
	SET_RUNTIME_PM_OPS(bigo_runtime_suspend, bigo_runtime_resume, NULL)
};
#endif

static const struct file_operations bigo_fops = {
	.owner = THIS_MODULE,
	.open = bigo_open,
	.release = bigo_release,
	.unlocked_ioctl = bigo_unlocked_ioctl,
	.compat_ioctl = bigo_unlocked_ioctl,
};

static int init_chardev(struct bigo_core *core)
{
	int rc;

	cdev_init(&core->cdev, &bigo_fops);
	core->cdev.owner = THIS_MODULE;
	rc = alloc_chrdev_region(&core->devno, 0, 1, BIGO_CHRDEV_NAME);
	if (rc < 0) {
		pr_err("Failed to alloc chrdev region\n");
		goto err;
	}
	rc = cdev_add(&core->cdev, core->devno, 1);
	if (rc) {
		pr_err("Failed to register chrdev\n");
		goto err_cdev_add;
	}

	core->_class = class_create(THIS_MODULE, BIGO_DEVCLASS_NAME);
	if (IS_ERR(core->_class)) {
		rc = PTR_ERR(core->_class);
		goto err_class_create;
	}

	core->svc_dev = device_create(core->_class, NULL, core->cdev.dev, core,
				      BIGO_CHRDEV_NAME);
	if (IS_ERR(core->svc_dev)) {
		pr_err("device_create err\n");
		rc = PTR_ERR(core->svc_dev);
		goto err_device_create;
	}
	return rc;

err_device_create:
	class_destroy(core->_class);
err_class_create:
	cdev_del(&core->cdev);
err_cdev_add:
	unregister_chrdev_region(core->devno, 1);
err:
	return rc;
}

static void deinit_chardev(struct bigo_core *core)
{
	if (!core)
		return;

	device_destroy(core->_class, core->devno);
	class_destroy(core->_class);
	cdev_del(&core->cdev);
	unregister_chrdev_region(core->devno, 1);
}

static int bigo_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct bigo_core *core;

	core = devm_kzalloc(&pdev->dev, sizeof(struct bigo_core), GFP_KERNEL);
	if (!core) {
		rc = -ENOMEM;
		goto err;
	}

	mutex_init(&core->lock);
	INIT_LIST_HEAD(&core->instances);
	INIT_LIST_HEAD(&core->pm.opps);
	INIT_LIST_HEAD(&core->pm.bw);
	spin_lock_init(&core->status_lock);
	init_completion(&core->frame_done);
	core->dev = &pdev->dev;
	platform_set_drvdata(pdev, core);

	rc = init_chardev(core);
	if (rc) {
		pr_err("Failed to initialize chardev for bigocean: %d\n", rc);
		goto err_init_chardev;
	}

	rc = bigo_of_dt_parse(core);
	if (rc) {
		pr_err("Failed to parse DT node\n");
		goto err_dt_parse;
	}

	rc = bigo_init_io(core, bigo_isr);
	if (rc < 0) {
		pr_err("failed to request irq\n");
		goto err_io;
	}

	/* TODO(vinaykalia@): Move pm_runtime_enable call somewhere else? */
	pm_runtime_enable(&pdev->dev);
	rc = bigo_pm_init(core);
	if (rc) {
		pr_err("Failed to initialize power management\n");
		goto err_io;
	}

	rc = iommu_register_device_fault_handler(&pdev->dev, bigo_iommu_fault_handler, core);
	if (rc) {
		pr_err("failed to register iommu fault handler: %d\n", rc);
		goto err_fault_handler;
	}

	bigo_pt_client_register(pdev->dev.of_node, core);

	if(platform_device_register(&bigo_sscd_dev))
		pr_warn("Failed to register bigo_sscd_dev.\n");

	bigo_init_debugfs(core);

	return rc;

err_fault_handler:
	pm_runtime_disable(&pdev->dev);
err_io:
	bigo_of_dt_release(core);
err_dt_parse:
	deinit_chardev(core);
err_init_chardev:
	platform_set_drvdata(pdev, NULL);
err:
	return rc;
}

static int bigo_remove(struct platform_device *pdev)
{
	struct bigo_core *core = (struct bigo_core *)platform_get_drvdata(pdev);

	bigo_uninit_debugfs(core);
	platform_device_unregister(&bigo_sscd_dev);
	bigo_pt_client_unregister(core);
	iommu_unregister_device_fault_handler(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	bigo_of_dt_release(core);
	deinit_chardev(core);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id bigo_dt_match[] = {
	{ .compatible = "google,bigocean" },
	{}
};

static struct platform_driver bigo_driver = {
	.probe = bigo_probe,
	.remove = bigo_remove,
	.driver = {
		.name = "bigocean",
		.owner = THIS_MODULE,
		.of_match_table = bigo_dt_match,
#ifdef CONFIG_PM
		.pm = &bigo_pm_ops,
#endif
	},
};

module_platform_driver(bigo_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinay Kalia <vinaykalia@google.com>");
MODULE_DESCRIPTION("BigOcean driver");
