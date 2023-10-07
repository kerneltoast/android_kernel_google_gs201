// SPDX-License-Identifier: GPL-2.0
/*
 * Common platform interfaces for mobile TPU chips.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#include <linux/device.h>
#include <linux/gsa/gsa_tpu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-iremap-pool.h"
#include "edgetpu-mmu.h"
#include "edgetpu-mobile-platform.h"
#include "edgetpu-telemetry.h"
#include "mobile-firmware.h"
#include "mobile-pm.h"

static struct edgetpu_dev *edgetpu_debug_pointer;

/*
 * Log and trace buffers at the beginning of the remapped region,
 * pool memory afterwards.
 */
#define EDGETPU_POOL_MEM_OFFSET                                                                    \
	((EDGETPU_TELEMETRY_LOG_BUFFER_SIZE + EDGETPU_TELEMETRY_TRACE_BUFFER_SIZE) *               \
	 EDGETPU_NUM_CORES)

static void get_telemetry_mem(struct edgetpu_mobile_platform_dev *etmdev,
			      enum edgetpu_telemetry_type type, struct edgetpu_coherent_mem *mem)
{
	int i, offset = type == EDGETPU_TELEMETRY_TRACE ? EDGETPU_TELEMETRY_LOG_BUFFER_SIZE : 0;
	const size_t size = type == EDGETPU_TELEMETRY_LOG ? EDGETPU_TELEMETRY_LOG_BUFFER_SIZE :
								EDGETPU_TELEMETRY_TRACE_BUFFER_SIZE;

	for (i = 0; i < etmdev->edgetpu_dev.num_cores; i++) {
		mem[i].vaddr = etmdev->shared_mem_vaddr + offset;
		mem[i].dma_addr = EDGETPU_REMAPPED_DATA_ADDR + offset;
		mem[i].tpu_addr = EDGETPU_REMAPPED_DATA_ADDR + offset;
		mem[i].host_addr = 0;
		mem[i].size = size;
		offset += EDGETPU_TELEMETRY_LOG_BUFFER_SIZE + EDGETPU_TELEMETRY_TRACE_BUFFER_SIZE;
	}
}

static void edgetpu_mobile_get_telemetry_mem(struct edgetpu_mobile_platform_dev *etmdev)
{
	get_telemetry_mem(etmdev, EDGETPU_TELEMETRY_LOG, etmdev->log_mem);
	get_telemetry_mem(etmdev, EDGETPU_TELEMETRY_TRACE, etmdev->trace_mem);
}

static int edgetpu_platform_setup_fw_region(struct edgetpu_mobile_platform_dev *etmdev)
{
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	struct platform_device *gsa_pdev;
	struct device *dev = etdev->dev;
	struct resource r;
	struct device_node *np;
	int err;
	size_t region_map_size = EDGETPU_FW_SIZE_MAX + EDGETPU_REMAPPED_DATA_SIZE;

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "No memory region for firmware");
		return -ENODEV;
	}

	err = of_address_to_resource(np, 0, &r);
	of_node_put(np);
	if (err) {
		dev_err(dev, "No memory address assigned to firmware region");
		return err;
	}

	if (resource_size(&r) < region_map_size) {
		dev_err(dev, "Memory region for firmware too small (%zu bytes needed, got %llu)",
			region_map_size, resource_size(&r));
		return -ENOSPC;
	}

	/* Get GSA device from device tree */
	np = of_parse_phandle(dev->of_node, "gsa-device", 0);
	if (!np) {
		dev_warn(dev, "No gsa-device in device tree. Authentication not available");
	} else {
		gsa_pdev = of_find_device_by_node(np);
		if (!gsa_pdev) {
			dev_err(dev, "GSA device not found");
			of_node_put(np);
			return -ENODEV;
		}
		etmdev->gsa_dev = get_device(&gsa_pdev->dev);
		of_node_put(np);
	}

	etmdev->fw_region_paddr = r.start;
	etmdev->fw_region_size = EDGETPU_FW_SIZE_MAX;

	etmdev->shared_mem_vaddr = memremap(r.start + EDGETPU_REMAPPED_DATA_OFFSET,
					    EDGETPU_REMAPPED_DATA_SIZE, MEMREMAP_WC);
	if (!etmdev->shared_mem_vaddr) {
		dev_err(dev, "Shared memory remap failed");
		if (etmdev->gsa_dev)
			put_device(etmdev->gsa_dev);
		return -EINVAL;
	}
	etmdev->shared_mem_paddr = r.start + EDGETPU_REMAPPED_DATA_OFFSET;

	return 0;
}

static void edgetpu_platform_cleanup_fw_region(struct edgetpu_mobile_platform_dev *etmdev)
{
	if (etmdev->gsa_dev) {
		gsa_unload_tpu_fw_image(etmdev->gsa_dev);
		put_device(etmdev->gsa_dev);
	}
	if (!etmdev->shared_mem_vaddr)
		return;
	memunmap(etmdev->shared_mem_vaddr);
	etmdev->shared_mem_vaddr = NULL;
}

static int mobile_check_ext_mailbox_args(const char *func, struct edgetpu_dev *etdev,
					 struct edgetpu_ext_mailbox_ioctl *args)
{
	if (args->type != EDGETPU_EXT_MAILBOX_TYPE_TZ) {
		etdev_err(etdev, "%s: Invalid type %d != %d\n", func, args->type,
			  EDGETPU_EXT_MAILBOX_TYPE_TZ);
		return -EINVAL;
	}
	if (args->count != 1) {
		etdev_err(etdev, "%s: Invalid mailbox count: %d != 1\n", func, args->count);
		return -EINVAL;
	}
	return 0;
}

int edgetpu_chip_acquire_ext_mailbox(struct edgetpu_client *client,
				     struct edgetpu_ext_mailbox_ioctl *args)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(client->etdev);
	int ret;

	ret = mobile_check_ext_mailbox_args(__func__, client->etdev, args);
	if (ret)
		return ret;

	mutex_lock(&etmdev->tz_mailbox_lock);
	if (etmdev->secure_client) {
		etdev_err(client->etdev, "TZ mailbox already in use by PID %d\n",
			  etmdev->secure_client->pid);
		mutex_unlock(&etmdev->tz_mailbox_lock);
		return -EBUSY;
	}
	ret = edgetpu_mailbox_enable_ext(client, EDGETPU_TZ_MAILBOX_ID, NULL, 0);
	if (!ret)
		etmdev->secure_client = client;
	mutex_unlock(&etmdev->tz_mailbox_lock);
	return ret;
}

int edgetpu_chip_release_ext_mailbox(struct edgetpu_client *client,
				     struct edgetpu_ext_mailbox_ioctl *args)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(client->etdev);
	int ret = 0;

	ret = mobile_check_ext_mailbox_args(__func__, client->etdev,
					      args);
	if (ret)
		return ret;

	mutex_lock(&etmdev->tz_mailbox_lock);
	if (!etmdev->secure_client) {
		etdev_warn(client->etdev, "TZ mailbox already released\n");
		mutex_unlock(&etmdev->tz_mailbox_lock);
		return 0;
	}
	if (etmdev->secure_client != client) {
		etdev_err(client->etdev,
			  "TZ mailbox owned by different client\n");
		mutex_unlock(&etmdev->tz_mailbox_lock);
		return -EBUSY;
	}
	etmdev->secure_client = NULL;
	ret = edgetpu_mailbox_disable_ext(client, EDGETPU_TZ_MAILBOX_ID);
	mutex_unlock(&etmdev->tz_mailbox_lock);
	return ret;
}

void edgetpu_chip_client_remove(struct edgetpu_client *client)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(client->etdev);

	mutex_lock(&etmdev->tz_mailbox_lock);
	if (etmdev->secure_client == client) {
		etmdev->secure_client = NULL;
		edgetpu_mailbox_disable_ext(client, EDGETPU_TZ_MAILBOX_ID);
	}
	mutex_unlock(&etmdev->tz_mailbox_lock);
}

int edgetpu_chip_setup_mmu(struct edgetpu_dev *etdev)
{
	int ret;

	ret = edgetpu_mmu_attach(etdev, NULL);
	if (ret)
		dev_err(etdev->dev, "failed to attach IOMMU: %d", ret);
	return ret;
}

void edgetpu_chip_remove_mmu(struct edgetpu_dev *etdev)
{
	edgetpu_mmu_detach(etdev);
}

static void edgetpu_platform_parse_pmu(struct edgetpu_mobile_platform_dev *etmdev)
{
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	struct device *dev = etdev->dev;
	u32 reg;

	if (of_find_property(dev->of_node, "pmu-status-base", NULL) &&
	    !of_property_read_u32_index(dev->of_node, "pmu-status-base", 0, &reg)) {
		etmdev->pmu_status = devm_ioremap(dev, reg, 0x4);
		if (!etmdev->pmu_status)
			etdev_info(etdev, "Using ACPM for blk status query\n");
	}
}

static int edgetpu_platform_parse_ssmt(struct edgetpu_mobile_platform_dev *etmdev)
{
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	struct platform_device *pdev = to_platform_device(etdev->dev);
	struct resource *res;
	int ret;
	void __iomem *ssmt_base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ssmt");
	if (!res) {
		etdev_warn(etdev, "Failed to find SSMT register base");
		return -EINVAL;
	}
	ssmt_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ssmt_base)) {
		ret = PTR_ERR(ssmt_base);
		etdev_warn(etdev, "Failed to map SSMT register base: %d", ret);
		return ret;
	}
	etmdev->ssmt_base = ssmt_base;
	return 0;
}

static int edgetpu_platform_setup_irq(struct edgetpu_mobile_platform_dev *etmdev)
{
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	struct platform_device *pdev = to_platform_device(etdev->dev);
	int n = platform_irq_count(pdev);
	int ret;
	int i;

	etmdev->irq = devm_kmalloc_array(etdev->dev, n, sizeof(*etmdev->irq), GFP_KERNEL);
	if (!etmdev->irq)
		return -ENOMEM;

	for (i = 0; i < n; i++) {
		etmdev->irq[i] = platform_get_irq(pdev, i);
		ret = edgetpu_register_irq(etdev, etmdev->irq[i]);
		if (ret)
			goto rollback;
	}
	etmdev->n_irq = n;
	return 0;

rollback:
	while (i--)
		edgetpu_unregister_irq(etdev, etmdev->irq[i]);
	return ret;
}

static void edgetpu_platform_remove_irq(struct edgetpu_mobile_platform_dev *etmdev)
{
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	int i;

	for (i = 0; i < etmdev->n_irq; i++)
		edgetpu_unregister_irq(etdev, etmdev->irq[i]);
}

/*
 * Fetch and set the firmware context region from device tree.
 *
 * Maybe be unused since not all chips need this.
 */
static int __maybe_unused
edgetpu_mobile_platform_set_fw_ctx_memory(struct edgetpu_mobile_platform_dev *etmdev)
{
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	struct device *dev = etdev->dev;
	struct resource r;
	struct device_node *np;
	int ret;

	np = of_parse_phandle(dev->of_node, "memory-region", 1);
	if (!np) {
		etdev_warn(etdev, "No memory for firmware contexts");
		return -ENODEV;
	}

	ret = of_address_to_resource(np, 0, &r);
	of_node_put(np);
	if (ret) {
		etdev_warn(etdev, "No memory address for firmware contexts");
		return ret;
	}

	etmdev->fw_ctx_paddr = r.start;
	etmdev->fw_ctx_size = resource_size(&r);
	return 0;
}

static inline const char *get_driver_commit(void)
{
#if IS_ENABLED(CONFIG_MODULE_SCMVERSION)
	return THIS_MODULE->scmversion ?: "scmversion missing";
#elif defined(GIT_REPO_TAG)
	return GIT_REPO_TAG;
#else
	return "Unknown";
#endif
}

static int edgetpu_mobile_platform_probe(struct platform_device *pdev,
					 struct edgetpu_mobile_platform_dev *etmdev)
{
	struct device *dev = &pdev->dev;
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	struct resource *r;
	struct edgetpu_mapped_resource regs;
	int ret;
	struct edgetpu_iface_params iface_params[] = {
		/* Default interface  */
		{ .name = NULL },
		/* Common name for embedded SoC devices */
		{ .name = "edgetpu-soc" },
	};

	mutex_init(&etmdev->tz_mailbox_lock);

	platform_set_drvdata(pdev, etdev);
	etdev->dev = dev;
	etdev->num_cores = EDGETPU_NUM_CORES;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(r)) {
		dev_err(dev, "failed to get memory resource");
		return -ENODEV;
	}

	regs.phys = r->start;
	regs.size = resource_size(r);
	regs.mem = devm_ioremap_resource(dev, r);
	if (IS_ERR_OR_NULL(regs.mem)) {
		dev_err(dev, "failed to map registers");
		return -ENODEV;
	}

	mutex_init(&etmdev->platform_pwr.policy_lock);
	etmdev->platform_pwr.curr_policy = TPU_POLICY_MAX;

	ret = edgetpu_chip_pm_create(etdev);
	if (ret) {
		dev_err(dev, "Failed to initialize PM interface: %d", ret);
		return ret;
	}

	ret = edgetpu_platform_setup_fw_region(etmdev);
	if (ret) {
		dev_err(dev, "setup fw regions failed: %d", ret);
		goto out_shutdown;
	}

	ret = edgetpu_iremap_pool_create(etdev,
					 /* Base virtual address (kernel address space) */
					 etmdev->shared_mem_vaddr + EDGETPU_POOL_MEM_OFFSET,
					 /* Base DMA address */
					 EDGETPU_REMAPPED_DATA_ADDR + EDGETPU_POOL_MEM_OFFSET,
					 /* Base TPU address */
					 EDGETPU_REMAPPED_DATA_ADDR + EDGETPU_POOL_MEM_OFFSET,
					 /* Base physical address */
					 etmdev->shared_mem_paddr + EDGETPU_POOL_MEM_OFFSET,
					 /* Size */
					 EDGETPU_REMAPPED_DATA_SIZE - EDGETPU_POOL_MEM_OFFSET,
					 /* Granularity */
					 PAGE_SIZE);
	if (ret) {
		dev_err(dev, "failed to initialize remapped memory pool: %d", ret);
		goto out_cleanup_fw;
	}

	etdev->mcp_id = -1;
	etdev->mcp_die_index = 0;
	ret = edgetpu_device_add(etdev, &regs, iface_params, ARRAY_SIZE(iface_params));
	if (ret) {
		dev_err(dev, "edgetpu setup failed: %d", ret);
		goto out_destroy_iremap;
	}

	ret = edgetpu_platform_setup_irq(etmdev);
	if (ret) {
		dev_err(dev, "IRQ setup failed: %d", ret);
		goto out_remove_device;
	}

	ret = edgetpu_platform_parse_ssmt(etmdev);
	if (ret)
		dev_warn(dev, "SSMT setup failed (%d). Context isolation not enforced", ret);

	edgetpu_platform_parse_pmu(etmdev);

	etmdev->log_mem = devm_kcalloc(dev, etdev->num_cores, sizeof(*etmdev->log_mem), GFP_KERNEL);
	if (!etmdev->log_mem) {
		ret = -ENOMEM;
		goto out_remove_irq;
	}

#if IS_ENABLED(CONFIG_EDGETPU_TELEMETRY_TRACE)
	etmdev->trace_mem =
		devm_kcalloc(dev, etdev->num_cores, sizeof(*etmdev->log_mem), GFP_KERNEL);
	if (!etmdev->trace_mem) {
		ret = -ENOMEM;
		goto out_remove_irq;
	}
#endif

	edgetpu_mobile_get_telemetry_mem(etmdev);
	ret = edgetpu_telemetry_init(etdev, etmdev->log_mem, etmdev->trace_mem);
	if (ret)
		goto out_remove_irq;

	ret = edgetpu_mobile_firmware_create(etdev);
	if (ret) {
		dev_err(dev, "initialize firmware downloader failed: %d", ret);
		goto out_tel_exit;
	}

	etdev_dbg(etdev, "Creating thermal device");
	etdev->thermal = devm_tpu_thermal_create(etdev->dev, etdev);

	if (etmdev->after_probe) {
		ret = etmdev->after_probe(etmdev);
		if (ret) {
			dev_err(dev, "after_probe callback failed: %d", ret);
			goto out_destroy_fw;
		}
	}

	dev_info(dev, "%s edgetpu initialized. Build: %s", etdev->dev_name, get_driver_commit());

	/* Turn the device off unless a client request is already received. */
	edgetpu_pm_shutdown(etdev, false);

	edgetpu_debug_pointer = etdev;

	return 0;
out_destroy_fw:
	edgetpu_mobile_firmware_destroy(etdev);
out_tel_exit:
	edgetpu_telemetry_exit(etdev);
out_remove_irq:
	edgetpu_platform_remove_irq(etmdev);
out_remove_device:
	edgetpu_device_remove(etdev);
out_destroy_iremap:
	edgetpu_iremap_pool_destroy(etdev);
out_cleanup_fw:
	edgetpu_platform_cleanup_fw_region(etmdev);
out_shutdown:
	dev_dbg(dev, "Probe finished with error %d, powering down", ret);
	edgetpu_pm_shutdown(etdev, true);
	return ret;
}

static int edgetpu_mobile_platform_remove(struct platform_device *pdev)
{
	struct edgetpu_dev *etdev = platform_get_drvdata(pdev);
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);

	if (etmdev->before_remove)
		etmdev->before_remove(etmdev);
	edgetpu_mobile_firmware_destroy(etdev);
	edgetpu_platform_remove_irq(etmdev);
	edgetpu_pm_get(etdev->pm);
	edgetpu_telemetry_exit(etdev);
	edgetpu_device_remove(etdev);
	edgetpu_iremap_pool_destroy(etdev);
	edgetpu_platform_cleanup_fw_region(etmdev);
	edgetpu_pm_put(etdev->pm);
	edgetpu_pm_shutdown(etdev, true);
	edgetpu_mobile_pm_destroy(etdev);

	edgetpu_debug_pointer = NULL;

	return 0;
}
