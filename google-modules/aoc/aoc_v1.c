// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC v1 library
 *
 * Copyright (c) 2019-2023 Google LLC
 */

#include "aoc.h"
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos-pmu-if.h>
#include <soc/google/acpm_ipc_ctrl.h>

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
#include <soc/google/exynos-itmon.h>
#endif

#define SSMT_BYPASS_VALUE	0x80000000U
#define SSMT_NS_READ_PID(n)	(0x4000 + 4 * (n))
#define SSMT_NS_WRITE_PID(n)	(0x4200 + 4 * (n))

extern enum AOC_FW_STATE aoc_state;
extern struct platform_device *aoc_platform_device;
extern struct resource *aoc_sram_resource;
extern struct mutex aoc_service_lock;

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
static int aoc_itmon_notifier(struct notifier_block *nb, unsigned long action,
			      void *nb_data)
{
	struct aoc_prvdata *prvdata;
	struct itmon_notifier *itmon_info = nb_data;

	prvdata = container_of(nb, struct aoc_prvdata, itmon_nb);
	if (itmon_info->port && (strncmp(itmon_info->port, "AOC", sizeof("AOC") - 1) == 0))
		return NOTIFY_STOP;

	if (itmon_info->target_addr == 0)
		return NOTIFY_STOP;

	if ((itmon_info->target_addr >= aoc_sram_resource->start +
			prvdata->aoc_cp_aperture_start_offset) &&
	    (itmon_info->target_addr <= aoc_sram_resource->start +
			prvdata->aoc_cp_aperture_end_offset)) {
		dev_err(prvdata->dev,
			"Valid memory access triggered ITMON error. Please file a bug with bugreport and contents of /data/vendor/ssrdump\n");
		return NOTIFY_STOP;
	}

	return NOTIFY_OK;
}
#endif

static void acpm_aoc_reset_callback(unsigned int *cmd, unsigned int size)
{
	struct aoc_prvdata *prvdata;

	if (!aoc_platform_device)
		return;

	prvdata = platform_get_drvdata(aoc_platform_device);
	prvdata->aoc_reset_done = true;
	wake_up(&prvdata->aoc_reset_wait_queue);
}

bool aoc_release_from_reset(struct aoc_prvdata *prvdata)
{
	u32 pcu_value;
	void __iomem *pcu = aoc_sram_translate(prvdata->aoc_pcu_base);

	if (!pcu)
		return false;

	pcu_value = ioread32(pcu);

	pcu_value |= 1;
	iowrite32(pcu_value, pcu);

	return true;
}
EXPORT_SYMBOL_GPL(aoc_release_from_reset);

void request_aoc_on(struct aoc_prvdata *p, bool status)
{
	iowrite32(!!status, p->aoc_req_virt);
}
EXPORT_SYMBOL_GPL(request_aoc_on);

int wait_for_aoc_status(struct aoc_prvdata *p, bool status)
{
	unsigned long aoc_req_timeout;

	aoc_req_timeout = jiffies + (2 * HZ);
	while (time_before(jiffies, aoc_req_timeout)) {
		if (!!readl(p->aoc_req_virt + 0x40) == !!status)
			return 0;
		msleep(100);
	}

	return -ETIMEDOUT;
}
EXPORT_SYMBOL_GPL(wait_for_aoc_status);

__attribute__((unused))
int aoc_watchdog_restart(struct aoc_prvdata *prvdata,
	struct aoc_module_parameters *aoc_module_params)
{
	/* 4100 * 0.244 us * 100 = 100 ms */
	const int aoc_watchdog_value_ssr = 4100 * 100;
	const int aoc_reset_timeout_ms = 1000;
	const int aoc_reset_tries = 3;
	const u32 aoc_watchdog_control_ssr = 0x3F;
	const unsigned int custom_in_offset = 0x3AC4;
	const unsigned int custom_out_offset = 0x3AC0;
	int aoc_req_rc, rc;
	void __iomem *pcu;
	unsigned int custom_in;
	unsigned int custom_out;
	int ret;
	bool aoc_reset_successful;
	int i;

	pcu = aoc_sram_translate(prvdata->aoc_pcu_base);
	if (!pcu)
		return -ENODEV;

	if (*(aoc_module_params->aoc_disable_restart))
		return AOC_RESTART_DISABLED_RC;

	aoc_reset_successful = false;
	for (i = 0; i < aoc_reset_tries; i++) {
		dev_info(prvdata->dev, "asserting aoc_req\n");
		request_aoc_on(prvdata, true);
		aoc_req_rc = wait_for_aoc_status(prvdata, true);
		if (aoc_req_rc) {
			dev_err(prvdata->dev, "timed out waiting for aoc_ack\n");
			notify_timeout_aoc_status();
			continue;
		}
		dev_info(prvdata->dev, "resetting aoc\n");
		writel(AOC_PCU_WATCHDOG_KEY_UNLOCK, pcu + AOC_PCU_WATCHDOG_KEY_OFFSET);
		if ((readl(pcu + AOC_PCU_WATCHDOG_CONTROL_OFFSET) &
				AOC_PCU_WATCHDOG_CONTROL_KEY_ENABLED_MASK) == 0) {
			dev_err(prvdata->dev, "unlock aoc watchdog failed\n");
		}
		writel(aoc_watchdog_value_ssr, pcu + AOC_PCU_WATCHDOG_VALUE_OFFSET);
		writel(aoc_watchdog_control_ssr, pcu + AOC_PCU_WATCHDOG_CONTROL_OFFSET);

		dev_info(prvdata->dev, "waiting for aoc reset to finish\n");
		if (wait_event_timeout(prvdata->aoc_reset_wait_queue, prvdata->aoc_reset_done,
				       aoc_reset_timeout_ms) == 0) {
			ret = exynos_pmu_read(custom_out_offset, &custom_out);
			dev_err(prvdata->dev,
				"AoC reset timeout custom_out=%d, ret=%d\n", custom_out, ret);
			ret = exynos_pmu_read(custom_in_offset, &custom_in);
			dev_err(prvdata->dev,
				"AoC reset timeout custom_in=%d, ret=%d\n", custom_in, ret);
			dev_err(prvdata->dev, "PCU_WATCHDOG_CONTROL = 0x%x\n",
				readl(pcu + AOC_PCU_WATCHDOG_CONTROL_OFFSET));
			dev_err(prvdata->dev, "PCU_WATCHDOG_VALUE = 0x%x\n",
				readl(pcu + AOC_PCU_WATCHDOG_VALUE_OFFSET));
		} else {
			aoc_reset_successful = true;
			break;
		}
	}

	if (aoc_req_rc && *(aoc_module_params->aoc_panic_on_req_timeout)) {
		dev_err(prvdata->dev, "timed out too many times waiting for aoc_ack, triggering kernel panic\n");
		/* Sleep to ensure aocd can process notification of timeout before panic */
		msleep(1000);
		panic("AoC kernel panic: timed out waiting for aoc_ack");
	}

	if (!aoc_reset_successful) {
		/* Trigger acpm ramdump since we timed out the aoc reset request */
		dbg_snapshot_emergency_reboot("AoC Restart timed out");
		return -ETIMEDOUT;
	}
	reset_sensor_power(prvdata, false);
	dev_info(prvdata->dev, "aoc reset finished\n");
	prvdata->aoc_reset_done = false;

	/* Restore IOMMU settings by briefly setting AoC to runtime active. Since IOMMU is a
	 * supplier to AoC, it will be set to runtime active as a side effect. */
	rc = pm_runtime_set_active(prvdata->dev);
	if (rc < 0) {
		dev_err(prvdata->dev, "iommu restore failed: pm_runtime_resume rc = %d\n", rc);
		return rc;
	}
	rc = pm_runtime_set_suspended(prvdata->dev);
	if (rc < 0) {
		dev_err(prvdata->dev, "iommu restore failed: pm_runtime_suspend rc = %d\n", rc);
		return rc;
	}

	rc = start_firmware_load(prvdata->dev);
	if (rc) {
		dev_err(prvdata->dev, "load aoc firmware failed: rc = %d\n", rc);
		return rc;
	}

	return rc;
}
EXPORT_SYMBOL_GPL(aoc_watchdog_restart);

int platform_specific_probe(struct platform_device *pdev, struct aoc_prvdata *prvdata)
{
	unsigned int acpm_async_size;
	struct device *dev = &pdev->dev;
	struct device_node *aoc_node = dev->of_node;

	int rc = 0, ret = acpm_ipc_request_channel(aoc_node, acpm_aoc_reset_callback,
				       &prvdata->acpm_async_id, &acpm_async_size);
	if (ret < 0) {
		dev_err(dev, "failed to register acpm aoc reset callback\n");
		rc = -EIO;
	}

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	prvdata->itmon_nb.notifier_call = aoc_itmon_notifier;
	itmon_notifier_chain_register(&prvdata->itmon_nb);
#endif

	return rc;
}
EXPORT_SYMBOL_GPL(platform_specific_probe);

static void aoc_clear_gpio_interrupt(struct aoc_prvdata *prvdata)
{
#if defined(GPIO_INTERRUPT)
	int reg = GPIO_INTERRUPT, val;
	u32 *gpio_register =
		aoc_sram_translate(prvdata->aoc_gpio_base + ((reg / 32) * 12));

	val = ioread32(gpio_register);
	val &= ~(1 << (reg % 32));
	iowrite32(val, gpio_register);
#endif
}

void aoc_configure_hardware(struct aoc_prvdata *prvdata)
{
	aoc_clear_gpio_interrupt(prvdata);
}
EXPORT_SYMBOL_GPL(aoc_configure_hardware);

void trigger_aoc_ramdump(struct aoc_prvdata *prvdata)
{
	struct mbox_chan *channel = prvdata->mbox_channels[15].channel;
	static const uint32_t command[] = { 0, 0, 0, 0, 0x0deada0c, 0, 0, 0 };

	dev_notice(prvdata->dev, "Attempting to force AoC coredump\n");

	mbox_send_message(channel, (void *)&command);
}
EXPORT_SYMBOL_GPL(trigger_aoc_ramdump);

static irqreturn_t watchdog_int_handler(int irq, void *dev)
{
	trigger_aoc_ssr(false, NULL);
	return IRQ_HANDLED;
}

int configure_watchdog_interrupt(struct platform_device *pdev, struct aoc_prvdata *prvdata)
{
	int ret = 0;
	struct device *dev = &pdev->dev;

	prvdata->watchdog_irq = platform_get_irq_byname(pdev, "watchdog");
	if (prvdata->watchdog_irq < 0) {
		dev_err(dev, "failed to find watchdog irq\n");
		return -EIO;
	}

	irq_set_status_flags(prvdata->watchdog_irq, IRQ_NOAUTOEN);
	ret = devm_request_irq(dev, prvdata->watchdog_irq, watchdog_int_handler,
			       IRQF_TRIGGER_HIGH, dev_name(dev), dev);
	if (ret != 0) {
		dev_err(dev, "failed to register watchdog irq handler: %d\n",
			ret);
		return -EIO;
	}
	prvdata->first_fw_load = true;

	return ret;
}
EXPORT_SYMBOL_GPL(configure_watchdog_interrupt);

int configure_iommu_interrupts(struct device *dev, struct device_node *iommu_node,
		struct aoc_prvdata *prvdata)
{
	int rc = 0, ret = of_irq_get(iommu_node, 0);

	if (ret < 0) {
		dev_err(dev, "failed to find iommu non-secure irq: %d\n", ret);
		rc = ret;
		return rc;
	}
	prvdata->iommu_nonsecure_irq = ret;
	ret = of_irq_get(iommu_node, 1);
	if (ret < 0) {
		dev_err(dev, "failed to find iommu secure irq: %d\n", ret);
		rc = ret;
		return rc;
	}
	prvdata->iommu_secure_irq = ret;
	return rc;
}
EXPORT_SYMBOL_GPL(configure_iommu_interrupts);

#if IS_ENABLED(CONFIG_SOC_GS101)
void aoc_configure_ssmt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int stream_id;

	void __iomem *ssmt_base = devm_platform_ioremap_resource_byname(pdev, "ssmt_aoc");

	if (IS_ERR(ssmt_base)) {
		dev_err(dev, "ssmt_aoc base address failure: %ld\n", PTR_ERR(ssmt_base));
		return;
	}

	/* Configure registers NS_READ_PID_<n>, NS_WRITE_PID_<n> for each stream id */
	for (stream_id = 0; stream_id <= 32; stream_id++) {
		/* Skip over stream id 31 */
		if (stream_id == 31)
			continue;
		writel_relaxed(SSMT_BYPASS_VALUE, ssmt_base + SSMT_NS_READ_PID(stream_id));
		writel_relaxed(SSMT_BYPASS_VALUE, ssmt_base + SSMT_NS_WRITE_PID(stream_id));
	}

	devm_iounmap(dev, ssmt_base);
}
#else
void aoc_configure_ssmt(struct platform_device *pdev
	__attribute__((unused)))
{}
#endif
EXPORT_SYMBOL_GPL(aoc_configure_ssmt);

void configure_crash_interrupts(struct aoc_prvdata *prvdata, bool enable)
{
	if (prvdata->first_fw_load) {
		/* Default irq state of watchdog is off and iommu is on.
		 * When loading aoc firmware in first time
		 * Enable only irq of watchdog for balance irq state
		 */
		enable_irq(prvdata->watchdog_irq);
		prvdata->first_fw_load = false;
	} else if (enable) {
		enable_irq(prvdata->iommu_nonsecure_irq);
		enable_irq(prvdata->iommu_secure_irq);
		enable_irq(prvdata->watchdog_irq);
	} else {
		disable_irq_nosync(prvdata->iommu_nonsecure_irq);
		disable_irq_nosync(prvdata->iommu_secure_irq);
		/* Need to disable it to let APM handle it once we
		 * retrigger it in aoc_watchdog_restart.
		 */
		disable_irq_nosync(prvdata->watchdog_irq);
	}
}
EXPORT_SYMBOL_GPL(configure_crash_interrupts);
