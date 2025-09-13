// SPDX-License-Identifier: GPL-2.0
/*
 * Common EdgeTPU mobile power management support
 *
 * Copyright (C) 2021 Google, Inc.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gsa/gsa_tpu.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <soc/google/bts.h>
#include <soc/google/exynos_pm_qos.h>

#include <bcl.h>

#include "edgetpu-config.h"
#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-mobile-platform.h"
#include "edgetpu-pm.h"
#include "edgetpu-thermal.h"
#include "mobile-firmware.h"
#include "mobile-pm.h"

#include "edgetpu-pm.c"

/*
 * Encode INT/MIF values as a 16 bit pair in the 32-bit return value
 * (in units of MHz, to provide enough range)
 */
#define PM_QOS_INT_SHIFT                (16)
#define PM_QOS_MIF_MASK                 (0xFFFF)
#define PM_QOS_FACTOR                   (1000)

static int power_state = TPU_DEFAULT_POWER_STATE;

module_param(power_state, int, 0660);

#define MAX_VOLTAGE_VAL 1250000

#define BLOCK_DOWN_RETRY_TIMES 50
#define BLOCK_DOWN_MIN_DELAY_US 1000
#define BLOCK_DOWN_MAX_DELAY_US 1500

enum edgetpu_pwr_state edgetpu_active_states[EDGETPU_NUM_STATES] = {
	TPU_ACTIVE_UUD,
	TPU_ACTIVE_SUD,
	TPU_ACTIVE_UD,
	TPU_ACTIVE_NOM,
#if IS_ENABLED(CONFIG_ABROLHOS)
	TPU_ACTIVE_OD,
#endif /* IS_ENABLED(CONFIG_ABROLHOS) */
};

uint32_t *edgetpu_states_display = edgetpu_active_states;

static int mobile_pwr_state_init(struct device *dev)
{
	int ret;
	int curr_state;

	pm_runtime_enable(dev);
	curr_state = exynos_acpm_get_rate(TPU_ACPM_DOMAIN, 0);

	if (curr_state > TPU_OFF) {
		ret = pm_runtime_get_sync(dev);
		if (ret) {
			pm_runtime_put_noidle(dev);
			pm_runtime_disable(dev);
			dev_err(dev, "pm_runtime_get_sync returned %d\n", ret);
			return ret;
		}
	}

	ret = exynos_acpm_set_init_freq(TPU_ACPM_DOMAIN, curr_state);
	if (ret) {
		dev_err(dev, "error initializing tpu state: %d\n", ret);
		if (curr_state > TPU_OFF)
			pm_runtime_put_sync(dev);
		pm_runtime_disable(dev);
		return ret;
	}

	return ret;
}

static int edgetpu_core_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_CORE_DEBUG);
	return 0;
}

static int edgetpu_core_rate_set(void *data, u64 val)
{
	unsigned long dbg_rate_req;

	dbg_rate_req = TPU_DEBUG_REQ | TPU_CLK_CORE_DEBUG;
	dbg_rate_req |= val;

	return exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
}

static int edgetpu_ctl_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_CTL_DEBUG);
	return 0;
}

static int edgetpu_ctl_rate_set(void *data, u64 val)
{
	unsigned long dbg_rate_req;

	dbg_rate_req = TPU_DEBUG_REQ | TPU_CLK_CTL_DEBUG;
	dbg_rate_req |= 1000;

	return exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
}

static int edgetpu_axi_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_AXI_DEBUG);
	return 0;
}

static int edgetpu_axi_rate_set(void *data, u64 val)
{
	unsigned long dbg_rate_req;

	dbg_rate_req = TPU_DEBUG_REQ | TPU_CLK_AXI_DEBUG;
	dbg_rate_req |= 1000;

	return exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
}

static int edgetpu_apb_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_APB_DEBUG);
	return 0;
}

static int edgetpu_uart_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_UART_DEBUG);
	return 0;
}

static int edgetpu_vdd_int_m_set(void *data, u64 val)
{
	struct device *dev = (struct device *)data;
	unsigned long dbg_rate_req;

	if (val > MAX_VOLTAGE_VAL) {
		dev_err(dev, "Preventing INT_M voltage > %duV",
			MAX_VOLTAGE_VAL);
		return -EINVAL;
	}

	dbg_rate_req = TPU_DEBUG_REQ | TPU_VDD_INT_M_DEBUG;
	dbg_rate_req |= val;

	return exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
}

static int edgetpu_vdd_int_m_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_VDD_INT_M_DEBUG);
	return 0;
}

static int edgetpu_vdd_tpu_set(void *data, u64 val)
{
	int ret;
	struct device *dev = (struct device *)data;
	unsigned long dbg_rate_req;

	if (val > MAX_VOLTAGE_VAL) {
		dev_err(dev, "Preventing VDD_TPU voltage > %duV",
			MAX_VOLTAGE_VAL);
		return -EINVAL;
	}

	dbg_rate_req = TPU_DEBUG_REQ | TPU_VDD_TPU_DEBUG;
	dbg_rate_req |= val;

	ret = exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
	return ret;
}

static int edgetpu_vdd_tpu_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_VDD_TPU_DEBUG);
	return 0;
}

static int edgetpu_vdd_tpu_m_set(void *data, u64 val)
{
	int ret;
	struct device *dev = (struct device *)data;
	unsigned long dbg_rate_req;

	if (val > MAX_VOLTAGE_VAL) {
		dev_err(dev, "Preventing VDD_TPU voltage > %duV",
			MAX_VOLTAGE_VAL);
		return -EINVAL;
	}

	dbg_rate_req = TPU_DEBUG_REQ | TPU_VDD_TPU_M_DEBUG;
	dbg_rate_req |= val;

	ret = exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
	return ret;
}

static int edgetpu_vdd_tpu_m_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_VDD_TPU_M_DEBUG);
	return 0;
}

static int mobile_pwr_state_set_locked(struct edgetpu_mobile_platform_dev *etmdev, u64 val)
{
	int ret;
	int curr_state;
	struct edgetpu_dev *etdev = &etmdev->edgetpu_dev;
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	struct device *dev = etdev->dev;

	curr_state = exynos_acpm_get_rate(TPU_ACPM_DOMAIN, 0);

	dev_dbg(dev, "Power state %d -> %llu\n", curr_state, val);

	if (curr_state == TPU_OFF && val > TPU_OFF) {
		ret = pm_runtime_get_sync(dev);
		if (ret) {
			pm_runtime_put_noidle(dev);
			dev_err(dev, "pm_runtime_get_sync returned %d\n", ret);
			return ret;
		}
	}

	ret = platform_pwr->acpm_set_rate(TPU_ACPM_DOMAIN, (unsigned long)val);
	if (ret) {
		dev_err(dev, "error setting tpu state: %d\n", ret);
		pm_runtime_put_sync(dev);
		return ret;
	}

	if (curr_state != TPU_OFF && val == TPU_OFF) {
		ret = pm_runtime_put_sync(dev);
		if (ret) {
			dev_err(dev, "%s: pm_runtime_put_sync returned %d\n", __func__, ret);
			return ret;
		}
	}

	return ret;
}

static int mobile_pwr_state_get_locked(void *data, u64 *val)
{
	struct edgetpu_dev *etdev = (typeof(etdev))data;
	struct device *dev = etdev->dev;

	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN, 0);
	dev_dbg(dev, "current tpu state: %llu\n", *val);

	return 0;
}

static int mobile_pwr_state_set(void *data, u64 val)
{
	struct edgetpu_dev *etdev = (typeof(etdev))data;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	int ret = 0;

	mutex_lock(&platform_pwr->state_lock);
	platform_pwr->requested_state = val;
	if (val >= platform_pwr->min_state)
		ret = mobile_pwr_state_set_locked(etmdev, val);
	mutex_unlock(&platform_pwr->state_lock);
	return ret;
}

static int mobile_pwr_state_get(void *data, u64 *val)
{
	struct edgetpu_dev *etdev = (typeof(etdev))data;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	int ret;

	mutex_lock(&platform_pwr->state_lock);
	ret = mobile_pwr_state_get_locked(etdev, val);
	mutex_unlock(&platform_pwr->state_lock);
	return ret;
}

static int mobile_min_pwr_state_set(void *data, u64 val)
{
	struct edgetpu_dev *etdev = (typeof(etdev))data;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	int ret = 0;

	mutex_lock(&platform_pwr->state_lock);
	platform_pwr->min_state = val;
	if (val >= platform_pwr->requested_state)
		ret = mobile_pwr_state_set_locked(etmdev, val);
	mutex_unlock(&platform_pwr->state_lock);
	return ret;
}

static int mobile_min_pwr_state_get(void *data, u64 *val)
{
	struct edgetpu_dev *etdev = (typeof(etdev))data;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;

	mutex_lock(&platform_pwr->state_lock);
	*val = platform_pwr->min_state;
	mutex_unlock(&platform_pwr->state_lock);
	return 0;
}

static int mobile_pwr_policy_set(void *data, u64 val)
{
	struct edgetpu_dev *etdev = (typeof(etdev))data;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	int ret;

	mutex_lock(&platform_pwr->policy_lock);
	ret = exynos_acpm_set_policy(TPU_ACPM_DOMAIN, val);

	if (ret) {
		dev_err(etmdev->edgetpu_dev.dev,
			"unable to set policy %lld (ret %d)\n", val, ret);
		mutex_unlock(&platform_pwr->policy_lock);
		return ret;
	}

	platform_pwr->curr_policy = val;
	mutex_unlock(&platform_pwr->policy_lock);
	return 0;
}

static int mobile_pwr_policy_get(void *data, u64 *val)
{
	struct edgetpu_dev *etdev = (typeof(etdev))data;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;

	mutex_lock(&platform_pwr->policy_lock);
	*val = platform_pwr->curr_policy;
	mutex_unlock(&platform_pwr->policy_lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_pwr_policy, mobile_pwr_policy_get, mobile_pwr_policy_set,
			"%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_pwr_state, mobile_pwr_state_get, mobile_pwr_state_set, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_min_pwr_state, mobile_min_pwr_state_get, mobile_min_pwr_state_set,
			"%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_core_rate, edgetpu_core_rate_get,
			 edgetpu_core_rate_set, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_ctl_rate, edgetpu_ctl_rate_get,
			 edgetpu_ctl_rate_set, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_axi_rate, edgetpu_axi_rate_get,
			 edgetpu_axi_rate_set, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_apb_rate, edgetpu_apb_rate_get, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_uart_rate, edgetpu_uart_rate_get, NULL,
			 "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_vdd_int_m, edgetpu_vdd_int_m_get,
			 edgetpu_vdd_int_m_set, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_vdd_tpu, edgetpu_vdd_tpu_get,
			 edgetpu_vdd_tpu_set, "%llu\n");
DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_vdd_tpu_m, edgetpu_vdd_tpu_m_get,
			 edgetpu_vdd_tpu_m_set, "%llu\n");

static int mobile_get_initial_pwr_state(struct device *dev)
{
	switch (power_state) {
#if IS_ENABLED(CONFIG_ABROLHOS)
	case TPU_ACTIVE_OD:
	case TPU_DEEP_SLEEP_CLOCKS_SLOW:
	case TPU_DEEP_SLEEP_CLOCKS_FAST:
	case TPU_RETENTION_CLOCKS_SLOW:
#endif /* IS_ENABLED(CONFIG_ABROLHOS) */
	case TPU_ACTIVE_UUD:
	case TPU_ACTIVE_SUD:
	case TPU_ACTIVE_UD:
	case TPU_ACTIVE_NOM:
		dev_info(dev, "Initial power state: %d\n", power_state);
		break;
	case TPU_OFF:
#if IS_ENABLED(CONFIG_ABROLHOS)
	case TPU_DEEP_SLEEP_CLOCKS_OFF:
	case TPU_SLEEP_CLOCKS_OFF:
#endif /* IS_ENABLED(CONFIG_ABROLHOS) */
		dev_warn(dev, "Power state %d prevents control core booting", power_state);
		fallthrough;
	default:
		dev_warn(dev, "Power state %d is invalid\n", power_state);
		dev_warn(dev, "defaulting to active nominal\n");
		power_state = TPU_ACTIVE_NOM;
		break;
	}
	return power_state;
}

static int mobile_power_down(struct edgetpu_pm *etpm);

static int mobile_power_up(struct edgetpu_pm *etpm)
{
	struct edgetpu_dev *etdev = etpm->etdev;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	int ret;

	if (platform_pwr->is_block_down) {
		int times = 0;

		do {
			if (platform_pwr->is_block_down(etdev))
				break;
			usleep_range(BLOCK_DOWN_MIN_DELAY_US, BLOCK_DOWN_MAX_DELAY_US);
		} while (++times < BLOCK_DOWN_RETRY_TIMES);
		if (times >= BLOCK_DOWN_RETRY_TIMES && !platform_pwr->is_block_down(etdev))
			return -EAGAIN;
	}

	if (edgetpu_thermal_is_suspended(etdev->thermal)) {
		etdev_warn_ratelimited(etdev,
				       "power up rejected due to device thermal limit exceeded");
		return -EAGAIN;
	}

	ret = mobile_pwr_state_set(etpm->etdev, mobile_get_initial_pwr_state(etdev->dev));

	etdev_info(etpm->etdev, "Powering up\n");

	if (ret)
		return ret;

	if (platform_pwr->lpm_up)
		platform_pwr->lpm_up(etdev);

	edgetpu_chip_init(etdev);

	if (etdev->kci) {
		etdev_dbg(etdev, "Resetting KCI\n");
		edgetpu_kci_reinit(etdev->kci);
	}
	if (etdev->mailbox_manager) {
		etdev_dbg(etdev, "Resetting (VII/external) mailboxes\n");
		edgetpu_mailbox_reset_mailboxes(etdev->mailbox_manager);
	}

	if (!etdev->firmware)
		return 0;

	/*
	 * Why this function uses edgetpu_firmware_*_locked functions without explicitly holding
	 * edgetpu_firmware_lock:
	 *
	 * edgetpu_pm_get() is called in two scenarios - one is when the firmware loading is
	 * attempting, another one is when the user-space clients need the device be powered
	 * (usually through acquiring the wakelock).
	 *
	 * For the first scenario edgetpu_firmware_is_loading() below shall return true.
	 * For the second scenario we are indeed called without holding the firmware lock, but the
	 * firmware loading procedures (i.e. the first scenario) always call edgetpu_pm_get() before
	 * changing the firmware state, and edgetpu_pm_get() is blocked until this function
	 * finishes. In short, we are protected by the PM lock.
	 */

	if (edgetpu_firmware_is_loading(etdev))
		return 0;

	/* attempt firmware run */
	switch (edgetpu_firmware_status_locked(etdev)) {
	case FW_VALID:
		ret = edgetpu_firmware_restart_locked(etdev, false);
		break;
	case FW_INVALID:
		ret = edgetpu_firmware_run_default_locked(etdev);
		break;
	default:
		break;
	}
	if (ret) {
		mobile_power_down(etpm);
	} else {
#if IS_ENABLED(CONFIG_GOOGLE_BCL)
		if (!etmdev->bcl_dev)
			etmdev->bcl_dev = google_retrieve_bcl_handle();
		if (etmdev->bcl_dev)
			google_init_tpu_ratio(etmdev->bcl_dev);
#endif
	}

	return ret;
}

static void mobile_pm_cleanup_bts_scenario(struct edgetpu_dev *etdev)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	int performance_scenario = platform_pwr->performance_scenario;

	if (!performance_scenario)
		return;

	mutex_lock(&platform_pwr->scenario_lock);
	while (platform_pwr->scenario_count) {
		int ret = bts_del_scenario(performance_scenario);

		if (ret) {
			platform_pwr->scenario_count = 0;
			etdev_warn_once(etdev, "error %d in cleaning up BTS scenario %u\n", ret,
					performance_scenario);
			break;
		}
		platform_pwr->scenario_count--;
	}
	mutex_unlock(&platform_pwr->scenario_lock);
}

static int mobile_power_down(struct edgetpu_pm *etpm)
{
	struct edgetpu_dev *etdev = etpm->etdev;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	u64 val;
	int res = 0;
	int min_state = platform_pwr->min_state;

	etdev_info(etdev, "Powering down\n");

	if (min_state >= MIN_ACTIVE_STATE) {
		etdev_info(etdev, "Power down skipped due to min state = %d\n", min_state);
		return 0;
	}

	if (mobile_pwr_state_get(etdev, &val)) {
		etdev_warn(etdev, "Failed to read current power state\n");
		val = TPU_ACTIVE_NOM;
	}
	if (val == TPU_OFF) {
		etdev_dbg(etdev, "Device already off, skipping shutdown\n");
		return 0;
	}

	if (edgetpu_firmware_status_locked(etdev) == FW_VALID) {
		etdev_dbg(etdev, "Power down with valid firmware, device state = %d\n",
			  etdev->state);
		if (etdev->state == ETDEV_STATE_GOOD) {
			/* Update usage stats before we power off fw. */
			edgetpu_kci_update_usage_locked(etdev);
			platform_pwr->firmware_down(etdev);
			/* Ensure firmware is completely off */
			if (platform_pwr->lpm_down)
				platform_pwr->lpm_down(etdev);
			/* Indicate firmware is no longer running */
			etdev->state = ETDEV_STATE_NOFW;
		}
		edgetpu_kci_cancel_work_queues(etdev->kci);
	}

	if (etdev->firmware) {
		res = edgetpu_mobile_firmware_reset_cpu(etdev, true);

		/* TODO(b/198181290): remove -EIO once gsaproxy wakelock is implemented */
		if (res == -EAGAIN || res == -EIO)
			return -EAGAIN;
		if (res < 0)
			etdev_warn(etdev, "CPU reset request failed (%d)\n", res);
	}

	mobile_pwr_state_set(etdev, TPU_OFF);

	/* Remove our vote for INT/MIF state (if any) */
	exynos_pm_qos_update_request(&platform_pwr->int_min, 0);
	exynos_pm_qos_update_request(&platform_pwr->mif_min, 0);

	mobile_pm_cleanup_bts_scenario(etdev);

	/*
	 * It should be impossible that power_down() is called when secure_client is set.
	 * Non-null secure_client implies ext mailbox is acquired, which implies wakelock is
	 * acquired.
	 * Clear the state here just in case.
	 */
	etmdev->secure_client = NULL;

	return 0;
}

static int mobile_pm_after_create(struct edgetpu_pm *etpm)
{
	int ret;
	struct edgetpu_dev *etdev = etpm->etdev;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct device *dev = etdev->dev;
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	int curr_state;

	ret = mobile_pwr_state_init(dev);
	if (ret)
		return ret;

	mutex_init(&platform_pwr->policy_lock);
	mutex_init(&platform_pwr->state_lock);
	mutex_init(&platform_pwr->scenario_lock);

	exynos_pm_qos_add_request(&platform_pwr->int_min, PM_QOS_DEVICE_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&platform_pwr->mif_min, PM_QOS_BUS_THROUGHPUT, 0);

	platform_pwr->performance_scenario = bts_get_scenindex("tpu_performance");
	if (!platform_pwr->performance_scenario)
		dev_warn(etdev->dev, "tpu_performance BTS scenario not found\n");
	platform_pwr->scenario_count = 0;

	ret = mobile_pwr_state_set(etdev, mobile_get_initial_pwr_state(dev));
	if (ret)
		return ret;
	platform_pwr->debugfs_dir = debugfs_create_dir("power", edgetpu_fs_debugfs_dir());
	if (IS_ERR_OR_NULL(platform_pwr->debugfs_dir)) {
		dev_warn(etdev->dev, "Failed to create debug FS power");
		/* don't fail the procedure on debug FS creation fails */
		return 0;
	}
	debugfs_create_file("state", 0660, platform_pwr->debugfs_dir, etdev, &fops_tpu_pwr_state);
	debugfs_create_file("min_state", 0660, platform_pwr->debugfs_dir, etdev,
			    &fops_tpu_min_pwr_state);
	debugfs_create_file("policy", 0660, platform_pwr->debugfs_dir, etdev, &fops_tpu_pwr_policy);
	debugfs_create_file("vdd_tpu", 0660, platform_pwr->debugfs_dir, dev, &fops_tpu_vdd_tpu);
	debugfs_create_file("vdd_tpu_m", 0660, platform_pwr->debugfs_dir, dev, &fops_tpu_vdd_tpu_m);
	debugfs_create_file("vdd_int_m", 0660, platform_pwr->debugfs_dir, dev, &fops_tpu_vdd_int_m);
	debugfs_create_file("core_rate", 0660, platform_pwr->debugfs_dir, dev, &fops_tpu_core_rate);
	debugfs_create_file("ctl_rate", 0660, platform_pwr->debugfs_dir, dev, &fops_tpu_ctl_rate);
	debugfs_create_file("axi_rate", 0660, platform_pwr->debugfs_dir, dev, &fops_tpu_axi_rate);
	debugfs_create_file("apb_rate", 0440, platform_pwr->debugfs_dir, dev, &fops_tpu_apb_rate);
	debugfs_create_file("uart_rate", 0440, platform_pwr->debugfs_dir, dev, &fops_tpu_uart_rate);

	if (platform_pwr->after_create)
		ret = platform_pwr->after_create(etdev);
	if (ret)
		goto err_debugfs_remove;

	return 0;

err_debugfs_remove:
	debugfs_remove_recursive(platform_pwr->debugfs_dir);
	/* pm_runtime_{enable,get_sync} were called in mobile_pwr_state_init */

	curr_state = exynos_acpm_get_rate(TPU_ACPM_DOMAIN, 0);
	if (curr_state > TPU_OFF)
		pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	return ret;
}

static void mobile_pm_before_destroy(struct edgetpu_pm *etpm)
{
	struct edgetpu_dev *etdev = etpm->etdev;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;

	if (platform_pwr->before_destroy)
		platform_pwr->before_destroy(etdev);

	debugfs_remove_recursive(platform_pwr->debugfs_dir);
	pm_runtime_disable(etpm->etdev->dev);
	mobile_pm_cleanup_bts_scenario(etdev);
	exynos_pm_qos_remove_request(&platform_pwr->int_min);
	exynos_pm_qos_remove_request(&platform_pwr->mif_min);
}

static struct edgetpu_pm_handlers mobile_pm_handlers = {
	.after_create = mobile_pm_after_create,
	.before_destroy = mobile_pm_before_destroy,
	.power_up = mobile_power_up,
	.power_down = mobile_power_down,
};

int edgetpu_mobile_pm_create(struct edgetpu_dev *etdev)
{
	return edgetpu_pm_create(etdev, &mobile_pm_handlers);
}

void edgetpu_mobile_pm_destroy(struct edgetpu_dev *etdev)
{
	edgetpu_pm_destroy(etdev);
}

void edgetpu_mobile_pm_set_pm_qos(struct edgetpu_dev *etdev, u32 pm_qos_val)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	s32 int_val = (pm_qos_val >> PM_QOS_INT_SHIFT) * PM_QOS_FACTOR;
	s32 mif_val = (pm_qos_val & PM_QOS_MIF_MASK) * PM_QOS_FACTOR;

	etdev_dbg(etdev, "%s: pm_qos request - int = %d mif = %d\n", __func__, int_val, mif_val);

	exynos_pm_qos_update_request(&platform_pwr->int_min, int_val);
	exynos_pm_qos_update_request(&platform_pwr->mif_min, mif_val);
}

static void mobile_pm_activate_bts_scenario(struct edgetpu_dev *etdev)
{
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	int performance_scenario = platform_pwr->performance_scenario;

	/* bts_add_scenario() keeps track of reference count internally.*/
	int ret;

	if (!performance_scenario)
		return;
	mutex_lock(&platform_pwr->scenario_lock);
	ret = bts_add_scenario(performance_scenario);
	if (ret)
		etdev_warn_once(etdev, "error %d adding BTS scenario %u\n", ret,
				performance_scenario);
	else
		platform_pwr->scenario_count++;

	etdev_dbg(etdev, "BTS Scenario activated: %d\n", platform_pwr->scenario_count);
	mutex_unlock(&platform_pwr->scenario_lock);
}

static void mobile_pm_deactivate_bts_scenario(struct edgetpu_dev *etdev)
{
	/* bts_del_scenario() keeps track of reference count internally.*/
	int ret;
	struct edgetpu_mobile_platform_dev *etmdev = to_mobile_dev(etdev);
	struct edgetpu_mobile_platform_pwr *platform_pwr = &etmdev->platform_pwr;
	int performance_scenario = platform_pwr->performance_scenario;

	if (!performance_scenario)
		return;
	mutex_lock(&platform_pwr->scenario_lock);
	if (!platform_pwr->scenario_count) {
		mutex_unlock(&platform_pwr->scenario_lock);
		return;
	}
	ret = bts_del_scenario(performance_scenario);
	if (ret)
		etdev_warn_once(etdev, "error %d deleting BTS scenario %u\n", ret,
				performance_scenario);
	else
		platform_pwr->scenario_count--;

	etdev_dbg(etdev, "BTS Scenario deactivated: %d\n", platform_pwr->scenario_count);
	mutex_unlock(&platform_pwr->scenario_lock);
}

void edgetpu_mobile_pm_set_bts(struct edgetpu_dev *etdev, u16 bts_val)
{
	etdev_dbg(etdev, "%s: bts request - val = %u\n", __func__, bts_val);

	switch (bts_val) {
	case 0:
		mobile_pm_deactivate_bts_scenario(etdev);
		break;
	case 1:
		mobile_pm_activate_bts_scenario(etdev);
		break;
	default:
		etdev_warn(etdev, "%s: invalid BTS request value: %u\n", __func__, bts_val);
		break;
	}
}
