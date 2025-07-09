/*
 * drivers/media/platform/exynos/mfc/mfc_core.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/soc/samsung/exynos-smc.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/iommu.h>
#include <linux/pm_runtime.h>

#include "mfc_common.h"

#include "mfc_core_ops.h"
#include "mfc_core_isr.h"
#include "mfc_dec_v4l2.h"
#include "mfc_enc_v4l2.h"

#include "mfc_core_run.h"
#include "mfc_core_hwlock.h"
#include "mfc_debugfs.h"
#include "mfc_sync.h"
#include "mfc_meminfo.h"
#include "mfc_memlog.h"
#include "mfc_sysevent.h"

#include "mfc_core_pm.h"
#include "mfc_core_qos.h"
#include "mfc_core_meerkat.h"

#include "mfc_perf_measure.h"
#include "mfc_core_reg_api.h"
#include "mfc_core_hw_reg_api.h"
#include "mfc_llc.h"
#include "mfc_slc.h"

#include "mfc_qos.h"
#include "mfc_queue.h"
#include "mfc_utils.h"
#include "mfc_buf.h"
#include "mfc_mem.h"
#include "mfc_rm.h"

#define MFC_CORE_NAME			"mfc-core"

struct _mfc_trace_logging g_mfc_core_trace_logging[MFC_TRACE_LOG_COUNT_MAX];

#ifdef CONFIG_MFC_USE_COREDUMP
static struct sscd_platform_data mfc_core_sscd_platdata;

static void mfc_core_sscd_release(struct device *dev)
{
	dev_info(dev, "%s: sscd_dev is released\n", __func__);
}

static struct platform_device mfc_core_sscd_dev = {
	.name            = MFC_CORE_NAME,
	.driver_override = SSCD_NAME,
	.id              = -1,
	.dev             = {
		.platform_data = &mfc_core_sscd_platdata,
		.release       = mfc_core_sscd_release,
	},
};
#endif

void mfc_core_butler_worker(struct work_struct *work)
{
	struct mfc_core *core;

	core = container_of(work, struct mfc_core, butler_work);

	mfc_core_try_run(core);
}

static int __mfc_core_parse_mfc_qos_platdata(struct device_node *np,
		char *node_name, struct mfc_qos *qosdata, struct mfc_core *core)
{
	struct device_node *np_qos;

	np_qos = of_find_node_by_name(np, node_name);
	if (!np_qos) {
		dev_err(core->device, "%s: could not find mfc_qos_platdata node\n",
			node_name);
		return -EINVAL;
	}

	of_property_read_u32(np_qos, "thrd_mb", &qosdata->threshold_mb);
	of_property_read_u32(np_qos, "freq_mfc", &qosdata->freq_mfc);
	of_property_read_u32(np_qos, "freq_int", &qosdata->freq_int);
	of_property_read_u32(np_qos, "freq_mif", &qosdata->freq_mif);
	of_property_read_u32(np_qos, "mo_value", &qosdata->mo_value);
	of_property_read_u32(np_qos, "mo_10bit_value",
			&qosdata->mo_10bit_value);
	of_property_read_u32(np_qos, "mo_uhd_enc60_value",
			&qosdata->mo_uhd_enc60_value);
	of_property_read_u32(np_qos, "time_fw", &qosdata->time_fw);

	of_property_read_string(np_qos, "bts_scen", &qosdata->name);
	if (!qosdata->name) {
		mfc_pr_err("[QoS] bts_scen is missing in '%s' node", node_name);
		return -EINVAL;
	}

#ifdef CONFIG_MFC_USE_BTS
#ifndef CONFIG_MFC_NO_RENEWAL_BTS
	qosdata->bts_scen_idx = bts_get_scenindex(qosdata->name);
#endif
#endif

	return 0;
}

int mfc_core_sysmmu_fault_handler(struct iommu_fault *fault, void *param)
{
	struct mfc_core *core = (struct mfc_core *)param;
	unsigned int trans_info, fault_status;
	int ret;
	struct mfc_dev *dev = core->dev;

	mutex_lock(&dev->sysmmu_fault_mutex);

	if (core->core_pdata->trans_info_offset)
		trans_info = core->core_pdata->trans_info_offset;
	else
		trans_info = MFC_MMU_FAULT_TRANS_INFO;

	if (core->core_pdata->fault_status_offset)
		fault_status = core->core_pdata->fault_status_offset;
	else
		fault_status = MFC_MMU_INTERRUPT_STATUS;

	/* [OTF] If AxID is 1 in SYSMMU1 fault info, it is TS-MUX fault */
	if (core->has_hwfc) {
		if (core->has_2sysmmu) {
			if (MFC_MMU1_READL(fault_status) && ((MFC_MMU1_READL(trans_info) &
				  MFC_MMU_FAULT_TRANS_INFO_AXID_MASK) == 1)) {
				mfc_core_err("There is TS-MUX page fault. skip SFR dump\n");
				mutex_unlock(&dev->sysmmu_fault_mutex);
				return 0;
			}
		/* PMMU ID is 1 means that sysmmu1 in sysmmu v9 */
		} else if (core->core_pdata->fault_pmmuid_offset &&
				(((MFC_MMU0_READL(core->core_pdata->fault_pmmuid_offset) >>
				   core->core_pdata->fault_pmmuid_shift) & 0xFF) == 1)) {
			if ((MFC_MMU0_READL(trans_info) & MFC_MMU_FAULT_TRANS_INFO_AXID_MASK) == 1) {
				mfc_core_err("There is TS-MUX page fault. skip SFR dump\n");
				mutex_unlock(&dev->sysmmu_fault_mutex);
				return 0;
			}
		}
	}

	/* If sysmmu is used with other IPs, it should be checked whether it's an MFC fault */
	if (core->core_pdata->share_sysmmu) {
		if ((MFC_MMU0_READL(trans_info) & core->core_pdata->axid_mask)
				!= core->core_pdata->mfc_fault_num) {
			mfc_core_err("This is not a MFC page fault\n");
			mutex_unlock(&dev->sysmmu_fault_mutex);
			return 0;
		}
	}

	if (MFC_MMU0_READL(fault_status)) {
		/*
		 * Since it has become complicated to distinguish
		 * read or write of page fault in IP driver,
		 * the cause of page fault in logging data is unified into the page fault.
		 */
		core->logging_data->cause |= (1 << MFC_CAUSE_0PAGE_FAULT);
		core->logging_data->fault_status = MFC_MMU0_READL(fault_status);
		core->logging_data->fault_trans_info = MFC_MMU0_READL(trans_info);
	}

	if (core->has_2sysmmu) {
		if (MFC_MMU1_READL(fault_status)) {
			core->logging_data->cause |= (1 << MFC_CAUSE_1PAGE_FAULT);
			core->logging_data->fault_status = MFC_MMU1_READL(fault_status);
			core->logging_data->fault_trans_info = MFC_MMU1_READL(trans_info);
		}
	}
	core->logging_data->fault_addr = (unsigned int)(fault->event.addr);

	snprintf(core->crash_info, MFC_CRASH_INFO_LEN,
		"MFC-%d SysMMU PAGE FAULT at %#010llx (AxID: %#x), fault_status: %#x\n",
		core->id, fault->event.addr,
		core->logging_data->fault_trans_info, core->logging_data->fault_status);
	mfc_core_err("%s", core->crash_info);
	MFC_TRACE_CORE("%s", core->crash_info);

	call_dop(core, dump_and_stop_debug_mode, core);

	/*
	 * if return 0, sysmmu occurs kernel panic for debugging
	 * if -EAGAIN, sysmmu doesn't occur kernel panic (but need async-fault in dt).
	 */
	if (!core->dev->pdata->debug_mode && !debug_mode_en) {
		mfc_core_handle_error(core);
		ret = -EAGAIN;
	} else {
		ret = 0;
	}

	mutex_unlock(&dev->sysmmu_fault_mutex);

	return ret;
}

static int __mfc_core_parse_dt(struct device_node *np, struct mfc_core *core)
{
	struct mfc_core_platdata *pdata = core->core_pdata;
	struct device_node *np_qos;
	char node_name[50];
	int i;

	if (!np) {
		mfc_pr_err("there is no device node\n");
		return -EINVAL;
	}

	/* MFC version */
	of_property_read_u32(np, "ip_ver", &pdata->ip_ver);

	/* Sysmmu check */
	of_property_read_u32(np, "share_sysmmu", &pdata->share_sysmmu);
	of_property_read_u32(np, "axid_mask", &pdata->axid_mask);
	of_property_read_u32(np, "mfc_fault_num", &pdata->mfc_fault_num);
	of_property_read_u32(np, "trans_info_offset", &pdata->trans_info_offset);
	of_property_read_u32(np, "fault_status_offset", &pdata->fault_status_offset);
	of_property_read_u32(np, "fault_pmmuid_offset", &pdata->fault_pmmuid_offset);
	of_property_read_u32(np, "fault_pmmuid_shift", &pdata->fault_pmmuid_shift);

	/* LLC(Last Level Cache) */
	of_property_read_u32(np, "llc", &core->has_llc);
	of_property_read_u32(np, "need_llc_flush", &core->need_llc_flush);

	/* vOTF */
	of_property_read_u32(np, "mfc_votf_base", &pdata->mfc_votf_base);
	of_property_read_u32(np, "gdc_votf_base", &pdata->gdc_votf_base);
	of_property_read_u32(np, "dpu_votf_base", &pdata->dpu_votf_base);

	/* QoS */
	of_property_read_u32(np, "num_default_qos_steps",
			&pdata->num_default_qos_steps);
	of_property_read_u32(np, "num_encoder_qos_steps",
			&pdata->num_encoder_qos_steps);
	of_property_read_u32(np, "max_mb", &pdata->max_mb);
	of_property_read_u32(np, "mfc_freq_control", &pdata->mfc_freq_control);
	of_property_read_u32(np, "mo_control", &pdata->mo_control);
	of_property_read_u32(np, "bw_control", &pdata->bw_control);

	pdata->default_qos_table = devm_kzalloc(core->device,
			sizeof(struct mfc_qos) * pdata->num_default_qos_steps,
			GFP_KERNEL);
	for (i = 0; i < pdata->num_default_qos_steps; i++) {
		snprintf(node_name, sizeof(node_name), "mfc_d_qos_variant_%d", i);
		__mfc_core_parse_mfc_qos_platdata(np, node_name,
				&pdata->default_qos_table[i], core);
	}

	pdata->encoder_qos_table = devm_kzalloc(core->device,
			sizeof(struct mfc_qos) * pdata->num_encoder_qos_steps,
			GFP_KERNEL);
	for (i = 0; i < pdata->num_encoder_qos_steps; i++) {
		snprintf(node_name, sizeof(node_name), "mfc_e_qos_variant_%d", i);
		__mfc_core_parse_mfc_qos_platdata(np, node_name,
				&pdata->encoder_qos_table[i], core);
	}

	/* performance boost mode */
	pdata->qos_boost_table = devm_kzalloc(core->device,
			sizeof(struct mfc_qos_boost), GFP_KERNEL);
	np_qos = of_find_node_by_name(np, "mfc_perf_boost_table");
	if (!np_qos) {
		dev_err(core->device, "[QoS][BOOST] could not find mfc_perf_boost_table node\n");
		return -EINVAL;
	}
	of_property_read_u32(np_qos, "num_cluster",
			&pdata->qos_boost_table->num_cluster);
	of_property_read_u32(np_qos, "freq_mfc",
			&pdata->qos_boost_table->freq_mfc);
	of_property_read_u32(np_qos, "freq_int",
			&pdata->qos_boost_table->freq_int);
	of_property_read_u32(np_qos, "freq_mif",
			&pdata->qos_boost_table->freq_mif);
	of_property_read_u32_array(np_qos, "freq_cluster",
			&pdata->qos_boost_table->freq_cluster[0],
			pdata->qos_boost_table->num_cluster);

	of_property_read_string(np_qos, "bts_scen",
			&pdata->qos_boost_table->name);
	if (!pdata->qos_boost_table->name) {
		mfc_pr_err("[QoS][BOOST] bts_scen is missing in qos_boost node");
		return -EINVAL;
	}

#ifdef CONFIG_MFC_USE_BTS
#ifndef CONFIG_MFC_NO_RENEWAL_BTS
	pdata->qos_boost_table->bts_scen_idx =
		bts_get_scenindex(pdata->qos_boost_table->name);
	pdata->mfc_bw_index = bts_get_bwindex("mfc");
#endif
#endif

	return 0;
}

static int __mfc_core_register_resource(struct platform_device *pdev,
		struct mfc_core *core)
{
	struct device_node *np = core->device->of_node;
	struct device_node *iommu;
	struct device_node *hwfc;
	struct device_node *votf;
#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
	struct device_node *ssmt = NULL;
	struct device_node *sysreg = NULL;
#endif
	struct resource *res;
	int ret;
	int irq;

	mfc_perf_register(core);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		return -ENOENT;
	}
	core->mfc_mem = request_mem_region(res->start, resource_size(res), pdev->name);
	if (core->mfc_mem == NULL) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		return -ENOENT;
	}
	core->regs_base = ioremap_np(core->mfc_mem->start, resource_size(core->mfc_mem));
	if (core->regs_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap address region\n");
		goto err_ioremap;
	}

	iommu = of_get_child_by_name(np, "iommu");
	if (!iommu) {
		dev_err(&pdev->dev, "failed to get iommu node\n");
		goto err_ioremap_mmu0;
	}

	core->sysmmu0_base = of_iomap(iommu, 0);
	if (core->sysmmu0_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap sysmmu0 address region\n");
		goto err_ioremap_mmu0;
	}

	core->sysmmu1_base = of_iomap(iommu, 1);
	if (core->sysmmu1_base == NULL) {
		dev_dbg(&pdev->dev, "there is only one MFC sysmmu\n");
	} else {
		core->has_2sysmmu = 1;
	}

	hwfc = of_get_child_by_name(np, "hwfc");
	if (hwfc) {
		core->hwfc_base = of_iomap(hwfc, 0);
		if (core->hwfc_base == NULL) {
			core->has_hwfc = 0;
			dev_err(&pdev->dev, "failed to iomap hwfc address region\n");
			goto err_ioremap_hwfc;
		} else {
			core->has_hwfc = 1;
		}
	}

	votf = of_get_child_by_name(np, "votf");
	if (votf) {
		core->votf_base = of_iomap(votf, 0);
		if (core->votf_base == NULL) {
			core->has_mfc_votf = 0;
			dev_err(&pdev->dev, "failed to iomap votf address region\n");
			goto err_ioremap_votf;
		} else {
			core->has_mfc_votf = 1;
		}
	}

#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
	ssmt = of_get_child_by_name(np, "ssmt");
	if (!ssmt) {
		dev_err(&pdev->dev, "failed to get ssmt node\n");
		goto err_ioremap_ssmt0;
	}

	core->ssmt0_base = of_iomap(ssmt, 0);
	if (core->ssmt0_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap ssmt0 address region\n");
		goto err_ioremap_ssmt0;
	}

	core->ssmt1_base = of_iomap(ssmt, 1);
	if (core->ssmt1_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap ssmt1 address region\n");
		goto err_ioremap_ssmt1;
	}

	sysreg = of_get_child_by_name(np, "sysreg");
	if (!sysreg) {
		dev_err(&pdev->dev, "failed to get sysreg node\n");
		goto err_ioremap_sysreg;
	}

	core->sysreg_base = of_iomap(sysreg, 0);
	if (core->sysreg_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap sysreg address region\n");
		goto err_ioremap_sysreg;
	}
#endif

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get irq\n");
		goto err_res_irq;
	}
	core->irq = irq;
	ret = request_threaded_irq(core->irq, mfc_core_top_half_irq,
			mfc_core_irq, IRQF_ONESHOT, pdev->name, core);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to install irq (%d)\n", ret);
		goto err_res_irq;
	}

	return 0;

err_res_irq:
#if IS_ENABLED(CONFIG_SLC_PARTITION_MANAGER)
	iounmap(core->sysreg_base);
err_ioremap_sysreg:
	iounmap(core->ssmt1_base);
err_ioremap_ssmt1:
	iounmap(core->ssmt0_base);
err_ioremap_ssmt0:
#endif
	if (core->has_mfc_votf)
		iounmap(core->votf_base);
err_ioremap_votf:
	if (core->has_hwfc)
		iounmap(core->hwfc_base);
err_ioremap_hwfc:
	if (core->has_2sysmmu)
		iounmap(core->sysmmu1_base);
	iounmap(core->sysmmu0_base);
err_ioremap_mmu0:
	iounmap(core->regs_base);
err_ioremap:
	release_mem_region(core->mfc_mem->start, resource_size(core->mfc_mem));
	return -ENOENT;
}

static const struct mfc_core_ops mfc_core_ops = {
	.instance_init = mfc_core_instance_init,
	.instance_deinit = mfc_core_instance_deinit,
	.instance_open = mfc_core_instance_open,
	.instance_move_to = mfc_core_instance_move_to,
	.instance_move_from = mfc_core_instance_move_from,
	.instance_csd_parsing = mfc_core_instance_csd_parsing,
	.instance_dpb_flush = mfc_core_instance_dpb_flush,
	.instance_init_buf = mfc_core_instance_init_buf,
	.instance_q_flush = mfc_core_instance_q_flush,
	.instance_finishing = mfc_core_instance_finishing,
	.request_work = mfc_core_request_work,
};

#if IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)
extern struct imgloader_ops mfc_imgloader_ops;

static int __mfc_core_imgloader_desc_init(struct platform_device *pdev, struct mfc_core *core)
{
	struct imgloader_desc *desc = &core->mfc_imgloader_desc;

	desc->dev = &pdev->dev;
	desc->owner = THIS_MODULE;
	desc->ops = &mfc_imgloader_ops;
	desc->fw_name = MFC_FW_NAME;
	desc->name = core->name;
	desc->s2mpu_support = false;
	desc->fw_id = 0;

	return imgloader_desc_init(&core->mfc_imgloader_desc);
}
#endif

#ifdef CONFIG_MFC_USE_ITMON
static int __mfc_itmon_notifier(struct notifier_block *nb, unsigned long action,
				void *nb_data)
{
	struct mfc_core *core;
	struct itmon_notifier *itmon_info = nb_data;
	int is_mfc_itmon = 0, is_client = 0;
	int ret = NOTIFY_OK;

	core = container_of(nb, struct mfc_core, itmon_nb);

	if (IS_ERR_OR_NULL(itmon_info))
		return ret;

	/* print dump if it is an MFC ITMON error */
	if (itmon_info->port &&
		strncmp("MFC", itmon_info->port, sizeof("MFC") - 1) == 0) {
		is_mfc_itmon = 1;
		is_client = 1;
	} else if (itmon_info->client &&
		strncmp("MFC", itmon_info->client, sizeof("MFC") - 1) == 0) {
		is_mfc_itmon = 1;
		is_client = 1;
	} else if (itmon_info->dest &&
		strncmp("MFC", itmon_info->dest, sizeof("MFC") - 1) == 0) {
		is_mfc_itmon = 1;
		is_client = 0;
	}

	if (!is_mfc_itmon) {
		dev_err(core->device, "[MFCITMON] It is not mfc itmon\n\n");
		return ret;
	}

	dev_err(core->device, "[MFCITMON] mfc_itmon_notifier: +\n");
	dev_err(core->device, "[MFCITMON] MFC is %s\n", is_client ? "client" : "dest");
	if (!core->itmon_notified) {
		dev_err(core->device, "[MFCITMON] dump MFC information\n");
		core->itmon_notified = 1;
		call_dop(core, dump_and_stop_always, core);
	} else {
		dev_err(core->device, "[MFCITMON] MFC notifier has already been called. skip MFC information\n");
	}
	dev_err(core->device, "[MFCITMON] mfc_itmon_notifier: -\n");
	ret = NOTIFY_BAD;

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_SYSTEM_EVENT)
extern struct notifier_block mfc_core_nb;

static int __mfc_core_sysevent_desc_init(struct platform_device *pdev, struct mfc_core *core)
{
	int ret = 0;

	core->sysevent_desc.name = core->name;
	core->sysevent_desc.owner = THIS_MODULE;
	core->sysevent_desc.shutdown = mfc_sysevent_shutdown;
	core->sysevent_desc.powerup = mfc_sysevent_powerup;
	core->sysevent_desc.crash_shutdown = mfc_sysevent_crash_shutdown;
	core->sysevent_desc.dev = &pdev->dev;

	core->sysevent_dev = sysevent_register(&core->sysevent_desc);
	if (IS_ERR(core->sysevent_dev)) {
		ret = PTR_ERR(core->sysevent_dev);
		mfc_core_err("%s: sysevent_register failed :%d\n", pdev->name, ret);
	} else {
		mfc_core_info("%s: sysevent_register success\n", pdev->name);
	}

	return ret;
}
#endif

/* MFC probe function */
static int mfc_core_probe(struct platform_device *pdev)
{
	struct mfc_core *core;
	struct mfc_dev *dev;
	int ret = -ENOENT;
	int i;

	dev_info(&pdev->dev, "%s is called\n", __func__);

	core = devm_kzalloc(&pdev->dev, sizeof(struct mfc_core), GFP_KERNEL);
	if (!core) {
		dev_err(&pdev->dev, "Not enough memory for MFC device\n");
		return -ENOMEM;
	}

	core->device = &pdev->dev;

	/* set core id */
	of_property_read_u32(core->device->of_node, "id", &core->id);
	snprintf(core->name, sizeof(core->name), "MFC%d", core->id);

	/* register core to dev */
	dev = dev_get_drvdata(pdev->dev.parent);
	dev->core[core->id] = core;
	dev->num_core++;
	core->dev = dev;
	core->core_ops = &mfc_core_ops;

	core->core_pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct mfc_core_platdata), GFP_KERNEL);
	if (!core->core_pdata) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_pm;
	}

	ret = __mfc_core_parse_dt(core->device->of_node, core);
	if (ret)
		goto err_pm;

	atomic_set(&core->trace_ref_log, 0);
	core->mfc_trace_logging = g_mfc_core_trace_logging;

	mfc_core_pm_init(core);
	ret = __mfc_core_register_resource(pdev, core);
	if (ret)
		goto err_res_mem;

#if IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)
	ret = __mfc_core_imgloader_desc_init(pdev, core);
	if (ret)
		goto err_res_mem;
#endif

	init_waitqueue_head(&core->cmd_wq);
	mfc_core_init_listable_wq_dev(core);

	platform_set_drvdata(pdev, core);
	mfc_core_init_memlog(pdev);

	mfc_core_init_hwlock(core);
	mfc_create_bits(&core->work_bits);

	/* MFC meerkat timer */
	core->meerkat_wq = create_singlethread_workqueue("mfc_core/meerkat");
	if (!core->meerkat_wq) {
		dev_err(&pdev->dev, "failed to create workqueue for meerkat\n");
		goto err_wq_meerkat;
	}
	INIT_WORK(&core->meerkat_work, mfc_core_meerkat_worker);
	atomic_set(&core->meerkat_tick_running, 0);
	atomic_set(&core->meerkat_tick_cnt, 0);
	atomic_set(&core->meerkat_run, 0);
	timer_setup(&core->meerkat_timer, mfc_core_meerkat_tick, 0);

	/* MFC timer for HW idle checking */
	core->mfc_idle_wq = create_singlethread_workqueue("mfc_core/idle");
	if (!core->mfc_idle_wq) {
		dev_err(&pdev->dev, "failed to create workqueue for MFC QoS idle\n");
		goto err_wq_idle;
	}
	INIT_WORK(&core->mfc_idle_work, mfc_core_qos_idle_worker);
	timer_setup(&core->mfc_idle_timer, mfc_core_idle_checker, 0);
	mutex_init(&core->idle_qos_mutex);

	INIT_LIST_HEAD(&core->qos_queue);

	/* core butler worker */
	core->butler_wq = alloc_workqueue("mfc_core/butler", WQ_UNBOUND
					| WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	if (core->butler_wq == NULL) {
		dev_err(&pdev->dev, "failed to create workqueue for butler\n");
		goto err_butler_wq;
	}
	INIT_WORK(&core->butler_work, mfc_core_butler_worker);

	/* dump information call-back function */
	core->dump_ops = &mfc_core_dump_ops;

	atomic_set(&core->qos_req_cur, 0);
	mutex_init(&core->qos_mutex);

	mfc_core_info("[QoS] control: mfc_freq(%d), mo(%d), bw(%d)\n",
			core->core_pdata->mfc_freq_control,
			core->core_pdata->mo_control,
			core->core_pdata->bw_control);
	mfc_core_info("[QoS]-------------------Default table\n");
	for (i = 0; i < core->core_pdata->num_default_qos_steps; i++)
		mfc_core_info("[QoS] table[%d] mfc: %d, int: %d, mif: %d, bts_scen: %s(%d)\n",
				i,
				core->core_pdata->default_qos_table[i].freq_mfc,
				core->core_pdata->default_qos_table[i].freq_int,
				core->core_pdata->default_qos_table[i].freq_mif,
				core->core_pdata->default_qos_table[i].name,
				core->core_pdata->default_qos_table[i].bts_scen_idx);
	mfc_core_info("[QoS]-------------------Encoder only table\n");
	for (i = 0; i < core->core_pdata->num_encoder_qos_steps; i++)
		mfc_core_info("[QoS] table[%d] mfc: %d, int: %d, mif: %d, bts_scen: %s(%d)\n",
				i,
				core->core_pdata->encoder_qos_table[i].freq_mfc,
				core->core_pdata->encoder_qos_table[i].freq_int,
				core->core_pdata->encoder_qos_table[i].freq_mif,
				core->core_pdata->encoder_qos_table[i].name,
				core->core_pdata->encoder_qos_table[i].bts_scen_idx);

	ret = iommu_register_device_fault_handler(core->device,
			mfc_core_sysmmu_fault_handler, core);
	if (ret) {
		dev_err(&pdev->dev, "failed to register sysmmu fault handler %d\n", ret);
		ret = -EPROBE_DEFER;
		goto err_sysmmu_fault_handler;
	}

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	/* allocate Secure-DVA region */
	core->drm_fw_buf.buftype = MFCBUF_DRM_FW;
	core->drm_fw_buf.size = dev->variant->buf_size->firmware_code;
	if (mfc_mem_special_buf_alloc(dev, &core->drm_fw_buf)) {
		mfc_core_err("[F/W] Allocating DRM firmware buffer failed\n");
	}
#endif

	/* vOTF 1:1 mapping */
	core->domain = iommu_get_domain_for_dev(core->device);
	if (core->core_pdata->gdc_votf_base || core->core_pdata->dpu_votf_base) {
		ret = mfc_iommu_map_sfr(core);
		if (ret) {
			dev_err(&pdev->dev, "failed to map vOTF SFR\n");
			goto err_alloc_debug;
		}
	}

	core->logging_data = devm_kzalloc(&pdev->dev, sizeof(struct mfc_debug),
			GFP_KERNEL);
	if (!core->logging_data) {
		dev_err(&pdev->dev, "no memory for logging data\n");
		ret = -ENOMEM;
		goto err_alloc_debug;
	}

	mfc_client_pt_register(core);

#ifdef CONFIG_MFC_USE_ITMON
	core->itmon_nb.notifier_call = __mfc_itmon_notifier;
	itmon_notifier_chain_register(&core->itmon_nb);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_SYSTEM_EVENT)
	ret = __mfc_core_sysevent_desc_init(pdev, core);
	if (ret)
		goto err_alloc_debug;

	sysevent_notif_register_notifier(core->sysevent_desc.name, &mfc_core_nb);
#endif

#ifdef CONFIG_MFC_USE_COREDUMP
	if (platform_device_register(&mfc_core_sscd_dev)) {
		dev_err(&pdev->dev, "failed to register sscd_dev\n");
	} else {
		core->sscd_dev = &mfc_core_sscd_dev;

		core->dbg_info.size = MFC_DUMP_BUF_SIZE;
		core->dbg_info.addr = vmalloc(core->dbg_info.size);
		if (!core->dbg_info.addr)
			dev_err(&pdev->dev, "failed to alloc for debug buffer\n");
	}
#endif

	dev_info(&pdev->dev, "%s is completed\n", __func__);

	return 0;

err_alloc_debug:
#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	mfc_mem_special_buf_free(dev, &core->drm_fw_buf);
#endif
	iommu_unregister_device_fault_handler(&pdev->dev);
err_sysmmu_fault_handler:
	destroy_workqueue(core->butler_wq);
err_butler_wq:
	if (timer_pending(&core->mfc_idle_timer))
		del_timer(&core->mfc_idle_timer);
	destroy_workqueue(core->mfc_idle_wq);
err_wq_idle:
	if (timer_pending(&core->meerkat_timer))
		del_timer(&core->meerkat_timer);
	destroy_workqueue(core->meerkat_wq);
err_wq_meerkat:
	mfc_core_deinit_memlog(core);
#if IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)
	imgloader_desc_release(&core->mfc_imgloader_desc);
#endif
	free_irq(core->irq, core);
	if (core->sysreg_base)
		iounmap(core->sysreg_base);
	if (core->ssmt0_base)
		iounmap(core->ssmt0_base);
	if (core->ssmt1_base)
		iounmap(core->ssmt1_base);
	if (core->has_mfc_votf)
		iounmap(core->votf_base);
	if (core->has_hwfc)
		iounmap(core->hwfc_base);
	if (core->has_2sysmmu)
		iounmap(core->sysmmu1_base);
	iounmap(core->sysmmu0_base);
	iounmap(core->regs_base);
	release_mem_region(core->mfc_mem->start, resource_size(core->mfc_mem));
err_res_mem:
	mfc_core_pm_final(core);
err_pm:
	return ret;
}

/* Remove the driver */
static int mfc_core_remove(struct platform_device *pdev)
{
	struct mfc_core *core = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s++\n", __func__);

	if (core->dbg_info.addr)
		vfree(core->dbg_info.addr);
#ifdef CONFIG_MFC_USE_COREDUMP
	platform_device_unregister(&mfc_core_sscd_dev);
#endif
	iommu_unregister_device_fault_handler(&pdev->dev);
	if (timer_pending(&core->meerkat_timer))
		del_timer(&core->meerkat_timer);
	flush_workqueue(core->meerkat_wq);
	destroy_workqueue(core->meerkat_wq);
	if (timer_pending(&core->mfc_idle_timer))
		del_timer(&core->mfc_idle_timer);
	flush_workqueue(core->mfc_idle_wq);
	destroy_workqueue(core->mfc_idle_wq);
	flush_workqueue(core->butler_wq);
	destroy_workqueue(core->butler_wq);
	mfc_core_destroy_listable_wq_core(core);
	mfc_core_run_deinit_hw(core);
	free_irq(core->irq, core);
	if (core->sysreg_base)
		iounmap(core->sysreg_base);
	if (core->ssmt0_base)
		iounmap(core->ssmt0_base);
	if (core->ssmt1_base)
		iounmap(core->ssmt1_base);
	if (core->has_2sysmmu)
		iounmap(core->sysmmu1_base);
	iounmap(core->sysmmu0_base);
	iounmap(core->regs_base);
	mfc_client_pt_unregister(core);
	release_mem_region(core->mfc_mem->start, resource_size(core->mfc_mem));
	mfc_core_pm_final(core);
	mfc_core_deinit_memlog(core);
#if IS_ENABLED(CONFIG_EXYNOS_IMGLOADER)
	imgloader_desc_release(&core->mfc_imgloader_desc);
#endif

	dev_dbg(&pdev->dev, "%s--\n", __func__);
	return 0;
}

static void mfc_core_shutdown(struct platform_device *pdev)
{
	struct mfc_core *core = platform_get_drvdata(pdev);
	struct mfc_dev *dev = core->dev;
	int ret;

	mfc_core_info("%s core shutdown is called\n", core->name);
	if (core->shutdown) {
		mfc_core_info("%s core shutdown was already performed\n", core->name);
		return;
	}

	mutex_lock(&dev->mfc_mutex);
	if (!mfc_core_pm_get_pwr_ref_cnt(core)) {
		core->shutdown = 1;
		mfc_core_info("MFC is not running\n");
		mutex_unlock(&dev->mfc_mutex);
		return;
	}

	ret = mfc_core_get_hwlock_dev(core);
	if (ret < 0)
		mfc_core_err("Failed to get hwlock\n");

	mfc_core_risc_off(core);
	mfc_clear_all_bits(&core->work_bits);
	mfc_core_release_hwlock_dev(core);

	core->shutdown = 1;
	mutex_unlock(&dev->mfc_mutex);

	mfc_core_info("%s core shutdown completed\n", core->name);
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int mfc_core_suspend(struct device *device)
{
	struct mfc_core *core = platform_get_drvdata(to_platform_device(device));
	int ret;

	if (pm_runtime_status_suspended(device))
		return 0;

	if (!core) {
		dev_err(device, "no mfc device to run\n");
		return -EINVAL;
	}

	if (core->state == MFCCORE_ERROR) {
		dev_err(device, "Couldn't run HW. It's Error state\n");
		return -EINVAL;
	}

	if (core->num_inst == 0)
		return 0;

	if (core->sleep == 1) {
		mfc_core_info("MFC core is slept\n");
		return 0;
	}

	mfc_core_debug(2, "MFC core suspend is called\n");

	ret = mfc_core_get_hwlock_dev(core);
	if (ret < 0) {
		mfc_core_err("Failed to get hwlock\n");
		mfc_core_err("dev:0x%lx, bits:0x%lx, owned:%d, wl:%d, trans:%d\n",
				core->hwlock.dev, core->hwlock.bits,
				core->hwlock.owned_by_irq,
				core->hwlock.wl_count,
				core->hwlock.transfer_owner);
		return -EBUSY;
	}

	/*
	 * Prevent a timing issue that can occur when the power manager (PM)
	 * and the idle worker both call the mfc_core_suspend function at the
	 * same time.
	 */
	if (core->sleep == 1) {
		mfc_core_info("MFC core is slept\n");
		mfc_core_release_hwlock_dev(core);
		return 0;
	}

	if (!mfc_core_pm_get_pwr_ref_cnt(core)) {
		mfc_core_info("MFC power has not been turned on yet\n");
		mfc_core_release_hwlock_dev(core);
		return 0;
	}

	ret = mfc_core_run_sleep(core);
	if (ret) {
		mfc_core_err("Failed core_run_sleep\n");
		return -EFAULT;
	}

	if (core->has_llc && core->llc_on_status) {
		mfc_llc_flush(core);
		mfc_llc_disable(core);
	}

	if (core->has_slc && core->slc_on_status)
		mfc_slc_disable(core);

	mfc_core_release_hwlock_dev(core);

	mfc_core_debug(2, "MFC suspend is completed\n");

	return 0;
}

static int mfc_core_resume(struct device *device)
{
	struct mfc_core *core = platform_get_drvdata(to_platform_device(device));
	struct mfc_core_ctx *core_ctx;
	struct mfc_dev *dev;
	int ret;

	if (pm_runtime_status_suspended(device))
		return 0;

	if (!core) {
		dev_err(device, "no mfc core to run\n");
		return -EINVAL;
	}
	dev = core->dev;

	if (core->num_inst == 0)
		return 0;

	if ((core->idle_mode == MFC_IDLE_MODE_IDLE) && idle_suspend_enable) {
		core_ctx = core->core_ctx[core->curr_core_ctx];
		if (core_ctx) {
			/* Trigger idle resume instead of core resume */
			mfc_rm_qos_control(core_ctx->ctx, MFC_QOS_TRIGGER);
			return 0;
		} else {
			dev_err(device, "no mfc context to run\n");
			return -EINVAL;
		}
	}

	if (core->sleep == 0) {
		mfc_core_info("MFC core is not slept\n");
		return 0;
	}

	mfc_core_debug(2, "MFC core resume is called\n");

	ret = mfc_core_get_hwlock_dev(core);
	if (ret < 0) {
		mfc_core_err("Failed to get hwlock\n");
		mfc_core_err("dev:0x%lx, bits:0x%lx, owned:%d, wl:%d, trans:%d\n",
				core->hwlock.dev, core->hwlock.bits,
				core->hwlock.owned_by_irq,
				core->hwlock.wl_count,
				core->hwlock.transfer_owner);
		return -EBUSY;
	}

	if (core->has_llc && (core->llc_on_status == 0))
		mfc_llc_enable(core);

	if (core->has_slc && (core->slc_on_status == 0))
		mfc_slc_enable(core);

	core_ctx = core->core_ctx[core->curr_core_ctx];
	if (core_ctx)
		mfc_llc_handle_resol(core, core_ctx->ctx);

	ret = mfc_core_run_wakeup(core);
	if (ret) {
		mfc_core_err("Failed core_run_wakeup\n");
		return -EFAULT;
	}

	mfc_core_release_hwlock_dev(core);

	mfc_core_debug(2, "MFC resume is completed\n");

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_PM)
static int mfc_core_runtime_suspend(struct device *device)
{
	struct mfc_core *core = platform_get_drvdata(to_platform_device(device));
#if IS_ENABLED(CONFIG_EXYNOS_SYSTEM_EVENT)
	if (core->sysevent_dev)
		sysevent_put((void *)core->sysevent_dev);
#endif
	mfc_core_debug(3, "mfc runtime suspend\n");

	return 0;
}

static int mfc_core_runtime_idle(struct device *dev)
{
	return 0;
}

static int mfc_core_runtime_resume(struct device *device)
{
	struct mfc_core *core = platform_get_drvdata(to_platform_device(device));
#if IS_ENABLED(CONFIG_EXYNOS_SYSTEM_EVENT)
	struct sysevent_desc *desc = &core->sysevent_desc;
	void *retval = NULL;

	if (core->sysevent_dev) {
		retval = sysevent_get(desc->name);
		if (IS_ERR(retval))
			mfc_core_err("sysevent_get is failed in %s\n", desc->name);
	}
#endif
	mfc_core_debug(3, "mfc runtime resume\n");

	return 0;
}
#endif

/* Power management */
static const struct dev_pm_ops mfc_core_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mfc_core_suspend, mfc_core_resume)
	SET_RUNTIME_PM_OPS(
			mfc_core_runtime_suspend,
			mfc_core_runtime_resume,
			mfc_core_runtime_idle
	)
};

static const struct of_device_id exynos_mfc_core_match[] = {
	{
		.compatible = "samsung,exynos-mfc-core",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_mfc_core_match);

struct platform_driver mfc_core_driver = {
	.probe		= mfc_core_probe,
	.remove		= mfc_core_remove,
	.shutdown	= mfc_core_shutdown,
	.driver	= {
		.name	= MFC_CORE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &mfc_core_pm_ops,
		.of_match_table = of_match_ptr(exynos_mfc_core_match),
	},
};
