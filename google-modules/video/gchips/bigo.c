// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for BigWave/BigOcean video accelerator
 *
 * Copyright 2022 Google LLC.
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
#include <linux/kthread.h>
#include <linux/arm-smccc.h>
#include <uapi/linux/sched/types.h>

#include "bigo_io.h"
#include "bigo_iommu.h"
#include "bigo_of.h"
#include "bigo_pm.h"
#include "bigo_priv.h"
#include "bigo_slc.h"
#include "bigo_debug.h"
#include "bigo_prioq.h"

#define BIGO_DEVCLASS_NAME "video_codec"

#define DEFAULT_WIDTH 3840
#define DEFAULT_HEIGHT 2160
#define DEFAULT_FPS 60
#define BIGO_SMC_ID 0xd
#define BIGO_MAX_INST_NUM 16

#if IS_ENABLED(CONFIG_SOC_ZUMA)
#define BIGO_HBD_BIT BIT(21)
#define BIGO_CHRDEV_NAME "bigwave"
#else
#define BIGO_HBD_BIT BIT(17)
#define BIGO_CHRDEV_NAME "bigocean"
#endif

#define BIGO_IDLE_TIMEOUT_MS 1000

#define SMC_DRM_GET_PADDING_SIZE        ((unsigned int)(0x8200211E))

enum misc_command {
     BIGO_GET_PADDING_SIZE,
     BIGO_CONFIG_AFBC,
};

static int bigo_worker_thread(void *data);

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
	int rc;

	core->worker_thread = kthread_run(bigo_worker_thread, (void*)core,
					"bigo_worker_thread");
	if (IS_ERR(core->worker_thread)) {
		rc = PTR_ERR(core->worker_thread);
		core->worker_thread = NULL;
		pr_err("failed to create worker thread rc = %d\n", rc);
		goto exit;
	}

	sched_set_normal(core->worker_thread, -10);

	rc = bigo_pt_client_enable(core);
	if (rc) {
		pr_info("failed to enable SLC");
		kthread_stop(core->worker_thread);
		goto exit;
	}
#if IS_ENABLED(CONFIG_PM)
	rc = pm_runtime_get_sync(core->dev);
	if (rc) {
		pr_err("failed to resume: %d\n", rc);
		kthread_stop(core->worker_thread);
	}
#endif

exit:
	return rc;
}

static inline void on_last_inst_close(struct bigo_core *core)
{
#if IS_ENABLED(CONFIG_PM)
	if (pm_runtime_put_sync_suspend(core->dev))
		pr_warn("failed to suspend\n");
#endif
	bigo_pt_client_disable(core);
}

static inline int bigo_count_inst(struct bigo_core *core)
{
	int count = 0;
	struct list_head *pos;

	list_for_each(pos, &core->instances)
		count++;

	return count;
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
		goto err;
	}
	INIT_LIST_HEAD(&inst->list);
	INIT_LIST_HEAD(&inst->buffers);
	kref_init(&inst->refcount);
	mutex_init(&inst->lock);
	init_completion(&inst->job_comp);
	file->private_data = inst;
	inst->height = DEFAULT_WIDTH;
	inst->width = DEFAULT_HEIGHT;
	inst->fps = DEFAULT_FPS;
	inst->bpp = 1;
	inst->core = core;
	inst->idle = true;
	inst->job.regs_size = core->regs_size;
	inst->job.regs = kzalloc(core->regs_size, GFP_KERNEL);
	if (!inst->job.regs) {
		rc = -ENOMEM;
		pr_err("Failed to alloc job regs\n");
		goto err_first_inst;
	}
	mutex_lock(&core->lock);
	if (bigo_count_inst(core) >= BIGO_MAX_INST_NUM) {
		rc = -ENOMEM;
		pr_err("Reaches max number of supported instances\n");
		mutex_unlock(&core->lock);
		goto err_inst_open;
	}
	if (list_empty(&core->instances)) {
		rc = on_first_instance_open(core);
		if (rc) {
			pr_err("failed to setup first instance");
			mutex_unlock(&core->lock);
			goto err_inst_open;
		}
	}
	list_add_tail(&inst->list, &core->instances);
	mutex_unlock(&core->lock);
	bigo_mark_qos_dirty(core);
	pr_info("opened instance\n");
	return rc;

err_inst_open:
	kfree(inst->job.regs);
err_first_inst:
	kfree(inst);
err:
	return rc;
}

static void bigo_close(struct kref *ref)
{
	struct bigo_inst *inst = container_of(ref, struct bigo_inst, refcount);

	if (inst && inst->core) {
		clear_job_from_prioq(inst->core, inst);
		bigo_unmap_all(inst);
		bigo_mark_qos_dirty(inst->core);
		bigo_update_qos(inst->core);
		kfree(inst->job.regs);
		kfree(inst);
		pr_info("closed instance\n");
	}
}

static int bigo_release(struct inode *inode, struct file *file)
{
	struct bigo_inst *inst = file->private_data;
	struct bigo_core *core = inst->core;

	if (!inst || !core)
		return -EINVAL;

	mutex_lock(&core->lock);
	list_del(&inst->list);
	if (list_empty(&core->instances))
	{
		kthread_stop(core->worker_thread);
		on_last_inst_close(core);
	}
	mutex_unlock(&core->lock);

	kref_put(&inst->refcount, bigo_close);
	return 0;
}

static int bigo_run_job(struct bigo_core *core, struct bigo_job *job)
{
	long ret = 0;
	int rc = 0;
	u32 status = 0;
	unsigned long flags;
	struct bigo_inst* inst;

	inst = container_of(job, struct bigo_inst, job);
	bigo_bypass_ssmt_pid(core, inst->is_decoder_usage);
	bigo_push_regs(core, job->regs);
	bigo_core_enable(core);
	ret = wait_for_completion_timeout(&core->frame_done,
			msecs_to_jiffies(core->debugfs.timeout));
	if (!ret) {
		pr_err("last rd addr: 0x%x, last_wr_addr: 0x%x\n",
			bigo_core_readl(core, BIGO_REG_LAST_RD_AXI_ADDR),
			bigo_core_readl(core, BIGO_REG_LAST_WR_AXI_ADDR));
		pr_err("timed out waiting for HW for %u ms\n", core->debugfs.timeout);
		pr_err("last rd addr: 0x%x, last_wr_addr: 0x%x\n",
			bigo_core_readl(core, BIGO_REG_LAST_RD_AXI_ADDR),
			bigo_core_readl(core, BIGO_REG_LAST_WR_AXI_ADDR));

		spin_lock_irqsave(&core->status_lock, flags);
		core->stat_with_irq = bigo_core_readl(core, BIGO_REG_STAT);
		spin_unlock_irqrestore(&core->status_lock, flags);

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

inline void bigo_config_frmrate(struct bigo_inst *inst, __u32 frmrate)
{
	mutex_lock(&inst->lock);
	inst->fps = frmrate;
	mutex_unlock(&inst->lock);
	bigo_mark_qos_dirty(inst->core);
}

inline void bigo_config_frmsize(struct bigo_inst *inst,
				struct bigo_ioc_frmsize *frmsize)
{
	mutex_lock(&inst->lock);
	inst->height = frmsize->height;
	inst->width = frmsize->width;
	mutex_unlock(&inst->lock);
	bigo_mark_qos_dirty(inst->core);
}

inline void bigo_config_secure(struct bigo_inst *inst, __u32 is_secure)
{
	mutex_lock(&inst->lock);
	inst->is_secure = is_secure;
	mutex_unlock(&inst->lock);
}

inline void bigo_config_priority(struct bigo_inst *inst, __s32 priority)
{
	if (priority < 0 || priority >= BO_MAX_PRIO)
		return;
	mutex_lock(&inst->lock);
	inst->priority = priority;
	mutex_unlock(&inst->lock);
}

inline void bigo_config_afbc(struct bigo_inst *inst)
{
	mutex_lock(&inst->lock);
	inst->afbc = true;
	mutex_unlock(&inst->lock);
}

static int copy_regs_from_user(struct bigo_core *core,
			struct bigo_ioc_regs *desc,
			void __user *user_desc,
			struct bigo_job *job)
{
	if (!core || !desc || !user_desc || !job)
		return -EINVAL;

	if (copy_from_user(desc, user_desc, sizeof(*desc)))
		return -EFAULT;

	if (desc->regs_size != core->regs_size) {
		pr_err("Reg size of userspace(%u) is different(%u)\n",
		(unsigned int)desc->regs_size, core->regs_size);
		return -EINVAL;
	}

	if (copy_from_user(job->regs, (void *)desc->regs, desc->regs_size))
		return -EFAULT;

	return 0;
}

static int copy_regs_to_user(struct bigo_ioc_regs *desc,
				struct bigo_job *job)
{
	if (!desc || !job)
		return -EINVAL;

	if (copy_to_user((void *)desc->regs, job->regs, desc->regs_size))
		return -EFAULT;

	return 0;
}

static long bigo_unlocked_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct bigo_inst *inst = file->private_data;
	struct bigo_core *core =
		container_of(file->f_inode->i_cdev, struct bigo_core, cdev);
	void __user *user_desc = (void __user *)arg;
	struct bigo_ioc_mapping mapping;
	struct bigo_ioc_frmsize frmsize;
	struct bigo_cache_info cinfo;
	struct bigo_inst *curr_inst;
	bool found = false;
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
	mutex_lock(&core->lock);
	list_for_each_entry(curr_inst, &core->instances, list) {
		if (curr_inst == inst) {
			found = true;
			break;
		}
	}

	if (!found) {
		mutex_unlock(&core->lock);
		pr_err("this instance is invalid");
		return -EINVAL;
	}
	kref_get(&inst->refcount);
	mutex_unlock(&core->lock);
	switch (cmd) {
	case BIGO_IOCX_PROCESS:
	{
		struct bigo_ioc_regs desc;
		struct bigo_job *job = &inst->job;
		long ret;
		u32 hbd;
		u32 bpp;

		if (copy_regs_from_user(core, &desc, user_desc, job)) {
			pr_err("Failed to copy regs from user\n");
			rc = -EFAULT;
			break;
		}

		hbd = (((u32*)job->regs)[3]) & BIGO_HBD_BIT;
		bpp = hbd ? 2:1;
		if (bpp != inst->bpp) {
			inst->bpp = bpp;
			bigo_mark_qos_dirty(core);
		}

#if IS_ENABLED(CONFIG_SOC_ZUMA)
		inst->is_decoder_usage = !!(((uint8_t*)job->regs)[BIGO_REG_STAT] & BIGO_STAT_MODE);
#else
		inst->is_decoder_usage = true;
#endif
		if(enqueue_prioq(core, inst)) {
			pr_err("Failed enqueue frame\n");
			rc = -EFAULT;
			break;
		}

		ret = wait_for_completion_timeout(
			&inst->job_comp,
			msecs_to_jiffies(JOB_COMPLETE_TIMEOUT_MS * 16));
		if (!ret) {
			pr_err("timed out waiting for HW: %d\n", rc);
			clear_job_from_prioq(core, inst);
			rc = -ETIMEDOUT;
		} else {
			rc = (ret > 0) ? 0 : ret;
		}

		if (rc)
			break;

		rc = job->status;
		if(copy_regs_to_user(&desc, job)) {
			pr_err("Failed to copy regs to user\n");
			rc = -EFAULT;
		}
		break;
	}
	case BIGO_IOCX_MAP:
		if (copy_from_user(&mapping, user_desc, sizeof(mapping))) {
			pr_err("Failed to copy from user\n");
			rc = -EFAULT;
			break;
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
			rc = -EFAULT;
			break;
		}
		rc = bigo_unmap(inst, &mapping);
		if (rc)
			pr_err("Error un-mapping: %d\n", mapping.fd);
		break;
	case BIGO_IOCX_DMA_SYNC: {
		struct bigo_buf_sync sync;
		if (copy_from_user(&sync, user_desc, sizeof(sync))) {
			pr_err("Failed to copy from user\n");
			rc = -EFAULT;
			break;
		}
		rc = bigo_dma_sync(&sync);
		if (rc)
			pr_err("Error dma sync: %d\n", sync.fd);
		break;
	}
	case BIGO_IOCX_CONFIG_FRMRATE: {
		u32 frmrate = (u32)arg;

		bigo_config_frmrate(inst, frmrate);
		break;
	}
	case BIGO_IOCX_CONFIG_FRMSIZE:
		if (copy_from_user(&frmsize, user_desc, sizeof(frmsize))) {
			pr_err("Failed to copy from user\n");
			rc = -EFAULT;
			break;
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
	case BIGO_IOCX_CONFIG_PRIORITY: {
		s32 priority = (s32)arg;
		bigo_config_priority(inst, priority);
		break;
	}
	case BIGO_IOCX_MISC: {
		struct bigo_ioc_misc misc;
		if (copy_from_user(&misc, user_desc, sizeof(misc))) {
			pr_err("Failed to copy from user\n");
			rc = -EFAULT;
			break;
		}
		switch (misc.cmd) {
			case BIGO_GET_PADDING_SIZE: {
				struct arm_smccc_res res;
				uint64_t iova = (uint64_t)misc.data0;
				uint64_t offset = (uint64_t)misc.data1;
				int32_t size = (int32_t)misc.data2;

				arm_smccc_smc(SMC_DRM_GET_PADDING_SIZE, iova, offset, size,
								0, 0, 0, 0, &res);

				misc.ret = res.a0;
				misc.data0 = res.a1;
				break;
			}
			case BIGO_CONFIG_AFBC: {
				bigo_config_afbc(inst);
				misc.ret = 0;
				break;
			}
			default:
				rc = -EINVAL;
				break;
		}
		if (copy_to_user(user_desc, &misc, sizeof(misc))) {
			pr_err("Failed to copy to user\n");
			rc = -EFAULT;
		}
		break;
	}
	case BIGO_IOCX_ABORT:
		break;
	default:
		rc = -EINVAL;
		break;
	}

	kref_put(&inst->refcount, bigo_close);
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
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	bigo_stat &= ~BIGO_STAT_MODE;
	bigo_stat &= ~BIGO_STAT_CODING_MODE;
#endif
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

static inline void mark_instances_idle(struct bigo_core *core)
{
	struct bigo_inst *curr_inst;
	mutex_lock(&core->lock);
	list_for_each_entry(curr_inst, &core->instances, list)
		curr_inst->idle = true;
	mutex_unlock(&core->lock);
}

static int bigo_worker_thread(void *data)
{
	struct bigo_core *core = (struct bigo_core *)data;
	struct bigo_inst *inst;
	struct bigo_job *job = NULL;
	bool should_stop;
	int rc;

	if (!core)
		return -ENOMEM;

	while(1) {
		rc = wait_event_timeout(core->worker,
			dequeue_prioq(core, &job, &should_stop),
			msecs_to_jiffies(BIGO_IDLE_TIMEOUT_MS));
		if (!rc && !should_stop) {
			/* Mark all instances as IDLE since none of these
			 * instances queued a job for BIGO_IDLE_TIMEOUT_MS
			 */
			mark_instances_idle(core);
			bigo_clocks_off(core);
			bigo_mark_qos_dirty(core);
			pr_info("bigocean entered idle state\n");
			wait_event(core->worker,
				dequeue_prioq(core, &job, &should_stop));
			pr_info("bigocean resumed to work\n");
		}
		if(should_stop) {
			pr_info("worker thread received stop signal, exit\n");
			return 0;
		}
		if (!job)
			continue;

		inst = container_of(job, struct bigo_inst, job);

		if (inst->idle) {
			inst->idle = false;
			bigo_mark_qos_dirty(core);
		}

		bigo_update_qos(core);
		if (inst->is_secure) {
			rc = exynos_smc(SMC_PROTECTION_SET, 0, BIGO_SMC_ID,
					SMC_PROTECTION_ENABLE);
			if (rc) {
				pr_err("failed to enable SMC_PROTECTION_SET: %d\n", rc);
				goto done;
			}
		}

		rc = bigo_run_job(core, job);

		if (inst->is_secure) {
			if (exynos_smc(SMC_PROTECTION_SET, 0, BIGO_SMC_ID,
					SMC_PROTECTION_DISABLE))
				pr_err("failed to disable SMC_PROTECTION_SET: %d\n", rc);
		}

	done:
		job->status = rc;
		complete(&inst->job_comp);
	}
	return 0;
}
#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
static int bigo_itmon_notifier(struct notifier_block *nb, unsigned long action,
				void *nb_data)
{
	struct bigo_core *core;
	struct itmon_notifier *itmon_info = nb_data;
	int is_bo_itmon = 0;
	int ret = NOTIFY_OK;

	core = container_of(nb, struct bigo_core, itmon_nb);

	if (unlikely(!core) || IS_ERR_OR_NULL(itmon_info))
		return ret;

	if ((itmon_info->port && !strncmp("BW", itmon_info->port, 2))
		|| (itmon_info->client && !strncmp("BW", itmon_info->client, 2))
		|| (itmon_info->dest && !strncmp("BW", itmon_info->dest, 2))) {
		is_bo_itmon = 1;
	}

	if (!is_bo_itmon)
		return ret;

	dev_err(core->dev, "port %s client %s dest %s\n", itmon_info->port,
				itmon_info->client, itmon_info->dest);
	ret = NOTIFY_BAD;

	BUG();

	return ret;
}
#endif

static int bigo_probe(struct platform_device *pdev)
{
	int rc = 0;
	int i;
	struct bigo_core *core;

	core = devm_kzalloc(&pdev->dev, sizeof(struct bigo_core), GFP_KERNEL);
	if (!core) {
		rc = -ENOMEM;
		goto err;
	}

	mutex_init(&core->lock);
	spin_lock_init(&core->prioq.lock);
	INIT_LIST_HEAD(&core->instances);
	INIT_LIST_HEAD(&core->pm.opps);
	INIT_LIST_HEAD(&core->pm.bw);
	for(i = 0; i < BO_MAX_PRIO; ++i)
		INIT_LIST_HEAD(&core->prioq.queue[i]);
	spin_lock_init(&core->status_lock);
	init_completion(&core->frame_done);
	init_waitqueue_head(&core->worker);
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

	rc = bigo_pt_client_register(pdev->dev.of_node, core);
	if (rc == -EPROBE_DEFER) {
		pr_warn("pt_client returns -EPROBE_DEFER, try again later\n");
		goto err_pt_client;
	} else {
		rc = 0;
	}

	if(platform_device_register(&bigo_sscd_dev))
		pr_warn("Failed to register bigo_sscd_dev.\n");

	bigo_init_debugfs(core);

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	core->itmon_nb.notifier_call = bigo_itmon_notifier;
	itmon_notifier_chain_register(&core->itmon_nb);
#endif

	return rc;

err_pt_client:
	iommu_unregister_device_fault_handler(&pdev->dev);
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
	{ .compatible = "google,bigwave"},
	{ .compatible = "google,bigocean"},
	{}
};
MODULE_DEVICE_TABLE(of, bigo_dt_match);

static struct platform_driver bigo_driver = {
	.probe = bigo_probe,
	.remove = bigo_remove,
	.driver = {
		.name = BIGO_CHRDEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bigo_dt_match,
#if IS_ENABLED(CONFIG_PM)
		.pm = &bigo_pm_ops,
#endif
	},
};

module_platform_driver(bigo_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinay Kalia <vinaykalia@google.com>");
MODULE_DESCRIPTION("BigWave driver");
MODULE_IMPORT_NS(DMA_BUF);
