/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Busan Platform-Specific Functions
 *
 * Copyright (c) 2021 Google, LLC
 */

#include "lwis_platform_busan.h"

#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <soc/google/bts.h>

#include "lwis_commands.h"
#include "lwis_device_dpm.h"
#include "lwis_debug.h"
#include "lwis_platform.h"

/* Uncomment to let kernel panic when IOMMU hits a page fault. */
/* #define ENABLE_PAGE_FAULT_PANIC */

int lwis_platform_probe(struct lwis_device *lwis_dev)
{
	struct lwis_platform *platform;
	int i;

	if (!lwis_dev) {
		return -ENODEV;
	}

	platform = kzalloc(sizeof(struct lwis_platform), GFP_KERNEL);
	if (IS_ERR_OR_NULL(platform)) {
		return -ENOMEM;
	}
	lwis_dev->platform = platform;

	/* Enable runtime power management for the platform device */
	pm_runtime_enable(&lwis_dev->plat_dev->dev);

	/* Only IOREG devices will access DMA resources */
	if (lwis_dev->type != DEVICE_TYPE_IOREG) {
		return 0;
	}

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
	of_for_each_phandle (&it, ret, lwis_dev->plat_dev->dev.of_node, "iommus", 0, 0) {
		u64 iommus_reg;
		const char *port_name = NULL;
		struct device_node *iommus_info = of_node_get(it.node);
		of_property_read_u64(iommus_info, "reg", &iommus_reg);
		of_property_read_string(iommus_info, "port-name", &port_name);
		pr_info("Device [%s] registered IOMMUS :[%s] %#010llx.sysmmu\n", lwis_dev->name,
			port_name, iommus_reg);
		pr_err("\n");
	}
	pr_err("IOMMU Page Fault at Address: 0x%px Flag: 0x%08x. Check dmesg for sysmmu errors\n",
	       (void *)fault->event.addr, fault->event.flags);
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

static bool lwis_device_support_bts(struct lwis_device *lwis_dev)
{
	int i;

	for (i = 0; i < lwis_dev->bts_block_num; i++) {
		if (lwis_dev->bts_indexes[i] != BTS_UNSUPPORTED) {
			return true;
		}
	}
	return false;
}

int lwis_platform_device_enable(struct lwis_device *lwis_dev)
{
	int ret;
	int iommus_len = 0;
	struct lwis_platform *platform;

	const int core_clock_qos = 67000;
	/* const int hpg_qos = 1; */

	if (!lwis_dev) {
		return -ENODEV;
	}

	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	/* Upref the runtime power management controls for the platform dev */
	ret = pm_runtime_get_sync(&lwis_dev->plat_dev->dev);
	if (ret < 0) {
		pr_err("Unable to enable platform device\n");
		return ret;
	}

	if (of_find_property(lwis_dev->plat_dev->dev.of_node, "iommus", &iommus_len) &&
	    iommus_len) {
		/* Activate IOMMU for the platform device */
		ret = iommu_register_device_fault_handler(&lwis_dev->plat_dev->dev,
							  lwis_iommu_fault_handler, lwis_dev);
		if (ret < 0) {
			pr_err("Failed to register fault handler for the device: %d\n", ret);
			return ret;
		}
	}

	if (lwis_dev->clock_family != CLOCK_FAMILY_INVALID &&
	    lwis_dev->clock_family < NUM_CLOCK_FAMILY) {
		ret = lwis_platform_update_qos(lwis_dev, core_clock_qos, lwis_dev->clock_family);
		if (ret < 0) {
			dev_err(lwis_dev->dev, "Failed to enable core clock\n");
			return ret;
		}
		/* TODO(b/173493818): We currently see some stability issue on specific device
		 * and sensor due to INT clock vote to 100 MHz. Set the minimum INT requirement
		 * to 200Mhz for now.
		 */
		ret = lwis_platform_update_qos(lwis_dev, 200000, CLOCK_FAMILY_INT);
		if (ret < 0) {
			dev_err(lwis_dev->dev, "Failed to initial INT clock\n");
			return ret;
		}
	}

	if (lwis_device_support_bts(lwis_dev) && lwis_dev->bts_scenario_name) {
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

	if (!lwis_dev) {
		return -ENODEV;
	}

	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	if (lwis_device_support_bts(lwis_dev) && lwis_dev->bts_scenario_name) {
		bts_del_scenario(lwis_dev->bts_scenario);
	}

	/* We can't remove fault handlers, so there's no call corresponding
	 * to the iommu_register_device_fault_handler above */

	lwis_platform_remove_qos(lwis_dev);

	if (of_find_property(lwis_dev->plat_dev->dev.of_node, "iommus", &iommus_len) &&
	    iommus_len) {
		/* Deactivate IOMMU */
		iommu_unregister_device_fault_handler(&lwis_dev->plat_dev->dev);
	}

	/* Disable platform device */
	return pm_runtime_put_sync(&lwis_dev->plat_dev->dev);
}

int lwis_platform_update_qos(struct lwis_device *lwis_dev, int value, int32_t clock_family)
{
	struct lwis_platform *platform;
	struct exynos_pm_qos_request *qos_req;
	int qos_class;

	if (!lwis_dev) {
		return -ENODEV;
	}

	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

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

	if (!exynos_pm_qos_request_active(qos_req)) {
		exynos_pm_qos_add_request(qos_req, qos_class, value);
	} else {
		exynos_pm_qos_update_request(qos_req, value);
	}

	dev_info(lwis_dev->dev, "Updating clock for clock_family %d, freq to %u\n", clock_family,
		 value);

	return 0;
}

int lwis_platform_remove_qos(struct lwis_device *lwis_dev)
{
	struct lwis_platform *platform;

	if (!lwis_dev) {
		return -ENODEV;
	}

	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	if (exynos_pm_qos_request_active(&platform->pm_qos_int)) {
		exynos_pm_qos_remove_request(&platform->pm_qos_int);
	}
	if (exynos_pm_qos_request_active(&platform->pm_qos_mem)) {
		exynos_pm_qos_remove_request(&platform->pm_qos_mem);
	}

	if (exynos_pm_qos_request_active(&platform->pm_qos_int_cam)) {
		exynos_pm_qos_remove_request(&platform->pm_qos_int_cam);
	}
	if (exynos_pm_qos_request_active(&platform->pm_qos_cam)) {
		exynos_pm_qos_remove_request(&platform->pm_qos_cam);
	}
	if (exynos_pm_qos_request_active(&platform->pm_qos_tnr)) {
		exynos_pm_qos_remove_request(&platform->pm_qos_tnr);
	}
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
	} else {
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
	return irq_set_affinity_hint(irq, cpumask_of(cpu));
}
