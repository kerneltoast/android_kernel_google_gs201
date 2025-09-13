// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Casablanca Platform-Specific Functions
 *
 * Copyright (c) 2022 Google, LLC
 */

#include "lwis_platform_casablanca.h"

#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <soc/google/bts.h>

#include "lwis_commands.h"
#include "lwis_device_dpm.h"
#include "lwis_debug.h"
#include "lwis_platform.h"

/*
 * module_param macro defines the parameter name, type and permission bits
 * in sysfs file system. 0644 represents that the owner can read and write the kernel
 * parameter using command line while other users can read it.
 */
bool lwis_casablanca_debug;
module_param(lwis_casablanca_debug, bool, 0644);

/* Uncomment to let kernel panic when IOMMU hits a page fault. */
/* #define ENABLE_PAGE_FAULT_PANIC */

int lwis_platform_probe(struct lwis_device *lwis_dev)
{
	struct lwis_platform *platform;
	int i;

	if (!lwis_dev)
		return -ENODEV;

	platform = kzalloc(sizeof(struct lwis_platform), GFP_KERNEL);
	if (IS_ERR_OR_NULL(platform))
		return -ENOMEM;

	lwis_dev->platform = platform;

	/* Enable runtime power management for the platform device */
	pm_runtime_enable(lwis_dev->k_dev);

	/* Only IOREG devices will access DMA resources */
	if (lwis_dev->type != DEVICE_TYPE_IOREG)
		return 0;

	/* Register to bts */
	for (i = 0; i < lwis_dev->bts_block_num; i++) {
		lwis_dev->bts_indexes[i] = bts_get_bwindex(lwis_dev->bts_block_names[i]);
		if (lwis_dev->bts_indexes[i] < 0) {
			dev_err(lwis_dev->dev, "Failed to register to BTS, ret: %d\n",
				lwis_dev->bts_indexes[i]);
			lwis_dev->bts_indexes[i] = BTS_UNSUPPORTED;
		}
	}

	return 0;
}

static int lwis_iommu_fault_handler(struct iommu_fault *fault, void *param)
{
	int ret;
	struct of_phandle_iterator it;
	struct lwis_device *lwis_dev = (struct lwis_device *)param;
	struct lwis_mem_page_fault_event_payload event_payload;

	pr_err("############ LWIS IOMMU PAGE FAULT ############\n");
	pr_err("\n");
	of_for_each_phandle(&it, ret, lwis_dev->k_dev->of_node, "iommus", 0, 0) {
		u64 iommus_reg;
		const char *port_name = NULL;
		struct device_node *iommus_info = of_node_get(it.node);

		of_property_read_u64(iommus_info, "reg", &iommus_reg);
		of_property_read_string(iommus_info, "port-name", &port_name);
		pr_info("Device [%s] registered IOMMUS :[%s] %#010llx.sysmmu\n", lwis_dev->name,
			port_name, iommus_reg);
		pr_err("\n");
	}
	pr_err("IOMMU Page Fault at Address: 0x%llx Flag: 0x%08x. Check dmesg for sysmmu errors\n",
	       fault->event.addr, fault->event.flags);
	pr_err("\n");
	lwis_debug_print_transaction_info(lwis_dev);
	pr_err("\n");
	lwis_debug_print_register_io_history(lwis_dev);
	pr_err("\n");
	lwis_debug_print_event_states_info(lwis_dev, /*lwis_event_dump_cnt=*/-1);
	pr_err("\n");
	lwis_debug_print_buffer_info(lwis_dev);
	pr_err("\n");
	pr_err("###############################################\n");

	event_payload.fault_address = fault->event.addr;
	event_payload.fault_flags = fault->event.flags;
	lwis_device_error_event_emit(lwis_dev, LWIS_ERROR_EVENT_ID_MEMORY_PAGE_FAULT,
				     &event_payload, sizeof(event_payload));

#ifdef ENABLE_PAGE_FAULT_PANIC
	return -EFAULT;
#else
	return -EAGAIN;
#endif /* ENABLE_PAGE_FAULT_PANIC */
}

static bool device_support_bts(struct lwis_device *lwis_dev)
{
	for (int i = 0; i < lwis_dev->bts_block_num; i++) {
		if (lwis_dev->bts_indexes[i] != BTS_UNSUPPORTED)
			return true;
	}
	return false;
}

int lwis_platform_device_enable(struct lwis_device *lwis_dev)
{
	int ret;
	int iommus_len = 0;
	struct lwis_platform *platform;

	if (!lwis_dev)
		return -ENODEV;

	platform = lwis_dev->platform;
	if (!platform)
		return -ENODEV;

	/* Upref the runtime power management controls for the platform dev */
	ret = pm_runtime_get_sync(lwis_dev->k_dev);
	if (ret < 0) {
		pr_err("Unable to enable platform device\n");
		return ret;
	}

	if (of_find_property(lwis_dev->k_dev->of_node, "iommus", &iommus_len) && iommus_len) {
		/* Activate IOMMU for the platform device */
		ret = iommu_register_device_fault_handler(lwis_dev->k_dev, lwis_iommu_fault_handler,
							  lwis_dev);
		if (ret < 0) {
			pr_err("Failed to register fault handler for the device: %d\n", ret);
			return ret;
		}
	}

	if (device_support_bts(lwis_dev) && lwis_dev->bts_scenario_name) {
		lwis_dev->bts_scenario = bts_get_scenindex(lwis_dev->bts_scenario_name);
		if (!lwis_dev->bts_scenario) {
			dev_err(lwis_dev->dev, "Failed to get default camera BTS scenario.\n");
			return -EINVAL;
		}
		bts_add_scenario(lwis_dev->bts_scenario);
	}
	return 0;
}

int lwis_platform_device_disable(struct lwis_device *lwis_dev)
{
	int iommus_len = 0;
	struct lwis_platform *platform;

	if (!lwis_dev)
		return -ENODEV;

	platform = lwis_dev->platform;
	if (!platform)
		return -ENODEV;

	if (device_support_bts(lwis_dev) && lwis_dev->bts_scenario_name)
		bts_del_scenario(lwis_dev->bts_scenario);

	/* We can't remove fault handlers, so there's no call corresponding
	 * to the iommu_register_device_fault_handler above
	 */

	lwis_platform_remove_qos(lwis_dev);

	if (of_find_property(lwis_dev->k_dev->of_node, "iommus", &iommus_len) && iommus_len) {
		/* Deactivate IOMMU */
		iommu_unregister_device_fault_handler(lwis_dev->k_dev);
	}

	/* Disable platform device */
	return pm_runtime_put_sync(lwis_dev->k_dev);
}

int lwis_platform_update_qos(struct lwis_device *lwis_dev, int value, int32_t clock_family)
{
	struct lwis_platform *platform;
	struct exynos_pm_qos_request __maybe_unused *qos_req;
	int __maybe_unused qos_class;

	if (!lwis_dev)
		return -ENODEV;

	platform = lwis_dev->platform;
	if (!platform)
		return -ENODEV;

	switch (clock_family) {
	case CLOCK_FAMILY_INTCAM:
		qos_req = &platform->pm_qos_int_cam;
		qos_class = PM_QOS_INTCAM_THROUGHPUT;
		break;
	case CLOCK_FAMILY_CAM:
		qos_req = &platform->pm_qos_cam;
		qos_class = PM_QOS_CAM_THROUGHPUT;
		break;
	case CLOCK_FAMILY_TNR:
		qos_req = &platform->pm_qos_tnr;
		qos_class = PM_QOS_TNR_THROUGHPUT;
		break;
	case CLOCK_FAMILY_MIF:
		qos_req = &platform->pm_qos_mem;
		qos_class = PM_QOS_BUS_THROUGHPUT;
		break;
	case CLOCK_FAMILY_INT:
		qos_req = &platform->pm_qos_int;
		qos_class = PM_QOS_DEVICE_THROUGHPUT;
		break;
	default:
		dev_err(lwis_dev->dev, "%s clk family %d is invalid\n", lwis_dev->name,
			lwis_dev->clock_family);
		return -EINVAL;
	}

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	if (!exynos_pm_qos_request_active(qos_req))
		exynos_pm_qos_add_request(qos_req, qos_class, value);
	else
		exynos_pm_qos_update_request(qos_req, value);
#endif

	if (lwis_casablanca_debug) {
		dev_info(lwis_dev->dev, "Updating clock for clock_family %d, freq to %u\n",
			 clock_family, value);
	}

	return 0;
}

static int find_bts_block(struct lwis_device *lwis_dev, struct lwis_device *target_dev,
			  struct lwis_qos_setting_v3 *qos_setting)
{
	int i;

	if (strcmp(qos_setting->bts_block_name, "") == 0) {
		if (target_dev->bts_block_num != 1 ||
		    target_dev->bts_block_names[0] != target_dev->name) {
			dev_err(lwis_dev->dev,
				"Device %s has %d bts blocks but no block name specified in qos setting\n",
				target_dev->name, target_dev->bts_block_num);
			return -EINVAL;
		}
		return 0;
	}
	for (i = 0; i < target_dev->bts_block_num; i++) {
		if (!strcmp(target_dev->bts_block_names[i], qos_setting->bts_block_name))
			return i;
	}
	dev_err(lwis_dev->dev, "Failed to find block name matching %s for device %s\n",
		qos_setting->bts_block_name, target_dev->name);
	return -EINVAL;
}

int lwis_platform_dpm_update_qos(struct lwis_device *lwis_dev, struct lwis_device *target_dev,
				 struct lwis_qos_setting_v3 *qos_setting)
{
	int ret = 0;

	switch (qos_setting->clock_family) {
	case CLOCK_FAMILY_MIF:
	case CLOCK_FAMILY_INT:
		if (qos_setting->frequency_hz >= 0 && target_dev->type == DEVICE_TYPE_DPM) {
			/* vote to qos if frequency is specified. The vote only available for dpm
			 * device
			 */
			ret = lwis_platform_update_qos(lwis_dev,
						       (int)(qos_setting->frequency_hz / 1000),
						       qos_setting->clock_family);
			if (ret) {
				dev_err(lwis_dev->dev,
					"Failed to vote to qos for clock family %d\n",
					qos_setting->clock_family);
			}
		} else {
			int bts_block = -1;
			int64_t peak_bw = 0;
			int64_t read_bw = 0;
			int64_t write_bw = 0;
			int64_t rt_bw = 0;

			bts_block = find_bts_block(lwis_dev, target_dev, qos_setting);
			if (bts_block < 0)
				return bts_block;

			read_bw = qos_setting->read_bw;
			write_bw = qos_setting->write_bw;
			peak_bw = (qos_setting->peak_bw > 0) ?
					  qos_setting->peak_bw :
					  ((read_bw > write_bw) ? read_bw : write_bw) / 4;
			rt_bw = (qos_setting->rt_bw > 0) ? qos_setting->rt_bw : 0;
			ret = lwis_platform_update_bts(target_dev, bts_block, peak_bw, read_bw,
						       write_bw, rt_bw);
			if (ret < 0) {
				dev_err(lwis_dev->dev,
					"Failed to update bandwidth to bts, ret: %d\n", ret);
			}
		}
		break;
	case CLOCK_FAMILY_TNR:
	case CLOCK_FAMILY_CAM:
	case CLOCK_FAMILY_INTCAM:
		/* convert value to KHz */
		ret = lwis_platform_update_qos(target_dev, (int)(qos_setting->frequency_hz / 1000),
					       qos_setting->clock_family);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Failed to apply core clock requirement for %s, ret: %d\n",
				target_dev->name, ret);
		}
		break;
	default:
		dev_err(lwis_dev->dev, "Invalid clock family %d\n", qos_setting->clock_family);
		ret = -EINVAL;
	}

	return ret;
}

int lwis_platform_remove_qos(struct lwis_device *lwis_dev)
{
	struct lwis_platform *platform;

	if (!lwis_dev)
		return -ENODEV;

	platform = lwis_dev->platform;
	if (!platform)
		return -ENODEV;

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
	if (exynos_pm_qos_request_active(&platform->pm_qos_int))
		exynos_pm_qos_remove_request(&platform->pm_qos_int);

	if (exynos_pm_qos_request_active(&platform->pm_qos_mem))
		exynos_pm_qos_remove_request(&platform->pm_qos_mem);

	if (exynos_pm_qos_request_active(&platform->pm_qos_int_cam))
		exynos_pm_qos_remove_request(&platform->pm_qos_int_cam);

	if (exynos_pm_qos_request_active(&platform->pm_qos_cam))
		exynos_pm_qos_remove_request(&platform->pm_qos_cam);

	if (exynos_pm_qos_request_active(&platform->pm_qos_tnr))
		exynos_pm_qos_remove_request(&platform->pm_qos_tnr);
#endif
	return 0;
}

int lwis_platform_update_bts(struct lwis_device *lwis_dev, int block, unsigned int bw_kb_peak,
			     unsigned int bw_kb_read, unsigned int bw_kb_write,
			     unsigned int bw_kb_rt)
{
	int ret = 0, bts_index = lwis_dev->bts_indexes[block];
	const char *block_name = lwis_dev->bts_block_names[block];
	struct bts_bw bts_request;

	if (block >= lwis_dev->bts_block_num) {
		dev_err(lwis_dev->dev, "Invalid block index %d, %s only has %d bts blocks\n", block,
			lwis_dev->name, lwis_dev->bts_block_num);
		return -EINVAL;
	}

	if (bts_index == BTS_UNSUPPORTED) {
		dev_err(lwis_dev->dev, "%s block %s doesn't support bts\n", lwis_dev->name,
			block_name);
		return -EINVAL;
	}

	bts_request.peak = bw_kb_peak;
	bts_request.read = bw_kb_read;
	bts_request.write = bw_kb_write;
	bts_request.rt = bw_kb_rt;
	ret = bts_update_bw(bts_index, bts_request);
	if (ret < 0) {
		dev_err(lwis_dev->dev, "Failed to update bandwidth to bts, ret: %d\n", ret);
	} else if (lwis_casablanca_debug) {
		dev_info(
			lwis_dev->dev,
			"Updated bandwidth to bts for device %s block %s: peak: %u, read: %u, write: %u, rt: %u\n",
			lwis_dev->name, block_name, bw_kb_peak, bw_kb_read, bw_kb_write, bw_kb_rt);
	}
	return ret;
}

int lwis_plaform_set_default_irq_affinity(unsigned int irq)
{
	const int cpu = 0x2;

	return irq_set_affinity_and_hint(irq, cpumask_of(cpu));
}
