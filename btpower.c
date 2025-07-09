// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

/*
 * Bluetooth Power Switch Module
 * controls power to external Bluetooth device
 * with interface to power management device
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/btpower.h>
#include <linux/of_device.h>
#include <soc/qcom/cmd-db.h>
#include <soc/google/exynos-cpupm.h>

#if IS_ENABLED(CONFIG_BT_SLIM_QCA6390) || \
	IS_ENABLED(CONFIG_BT_SLIM_QCA6490) || \
	IS_ENABLED(CONFIG_BTFM_SLIM_WCN3990) || \
	IS_ENABLED(CONFIG_BTFM_SLIM_WCN7850)
#include "btfm_slim.h"
#endif
#include <linux/fs.h>

#define PWR_SRC_NOT_AVAILABLE -2
#define DEFAULT_INVALID_VALUE -1
#define PWR_SRC_INIT_STATE_IDX 0
#define BTPOWER_MBOX_MSG_MAX_LEN 64
#define BTPOWER_MBOX_TIMEOUT_MS 1000
#define XO_CLK_RETRY_COUNT_MAX 5

/**
 * enum btpower_vreg_param: Voltage regulator TCS param
 * @BTPOWER_VREG_VOLTAGE: Provides voltage level to be configured in TCS
 * @BTPOWER_VREG_MODE: Regulator mode
 * @BTPOWER_VREG_ENABLE: Set Voltage regulator enable config in TCS
 * @BTPOWER_VREG_PARAM_MAX: vreg param boundary
 */
enum btpower_vreg_param {
	BTPOWER_VREG_VOLTAGE = 0,
	BTPOWER_VREG_MODE,
	BTPOWER_VREG_ENABLE,
	BTPOWER_VREG_PARAM_MAX,
};
static const char vreg_param_str[BTPOWER_VREG_PARAM_MAX] = {'v', 'm', 'e'};

/**
 * enum btpower_tcs_seq: TCS sequence ID for trigger
 * @BTPOWER_TCS_UP_SEQ: TCS Sequence based on up trigger / Wake TCS
 * @BTPOWER_TCS_DOWN_SEQ: TCS Sequence based on down trigger / Sleep TCS
 * @BTPOWER_TCS_ALL_SEQ: Update for both up and down triggers
 * @BTPOWER_TCS_SEQ_MAX: TCS sequence ID boundary
 */
enum btpower_tcs_seq {
	BTPOWER_TCS_UP_SEQ = 0,
	BTPOWER_TCS_DOWN_SEQ,
	BTPOWER_TCS_ALL_SEQ,
	BTPOWER_TCS_SEQ_MAX,
};
static const char *const tcs_seq_str[BTPOWER_TCS_SEQ_MAX] =
	{"upval", "dwnval", "enable"};

enum power_src_pos {
	BT_RESET_GPIO = PWR_SRC_INIT_STATE_IDX,
	BT_VDD_AON_LDO,
	BT_VDD_DIG_LDO,
	BT_VDD_RFA1_LDO,
	BT_VDD_RFA2_LDO,
	BT_VDD_ASD_LDO,
	BT_VDD_XTAL_LDO,
	BT_VDD_PA_LDO,
	BT_VDD_CORE_LDO,
	BT_VDD_IO_LDO,
	BT_VDD_LDO,
	BT_VDD_RFA_0p8,
	BT_VDD_RFACMN,
	// these indexes GPIOs/regs value are fetched during crash.
	BT_RESET_GPIO_CURRENT,
	BT_SW_CTRL_GPIO_CURRENT,
	BT_VDD_AON_LDO_CURRENT,
	BT_VDD_DIG_LDO_CURRENT,
	BT_VDD_RFA1_LDO_CURRENT,
	BT_VDD_RFA2_LDO_CURRENT,
	BT_VDD_ASD_LDO_CURRENT,
	BT_VDD_XTAL_LDO_CURRENT,
	BT_VDD_PA_LDO_CURRENT,
	BT_VDD_CORE_LDO_CURRENT,
	BT_VDD_IO_LDO_CURRENT,
	BT_VDD_LDO_CURRENT,
	BT_VDD_RFA_0p8_CURRENT,
	BT_VDD_RFACMN_CURRENT
};

#define LOG(level, drvdata, fmt, ...)                                        \
{                                                                            \
	switch (level) {                                                     \
	case LOGLEVEL_ERR:                                                   \
		dev_err(&drvdata->pdev->dev, fmt "\n", ##__VA_ARGS__);       \
		logbuffer_log(drvdata->devlog, "[E] " fmt, ##__VA_ARGS__);   \
		break;                                                       \
	case LOGLEVEL_WARNING:                                               \
		dev_warn(&drvdata->pdev->dev, fmt "\n", ##__VA_ARGS__);      \
		logbuffer_log(drvdata->devlog, "[W] " fmt, ##__VA_ARGS__);   \
		break;                                                       \
	case LOGLEVEL_INFO:                                                  \
		dev_info(&drvdata->pdev->dev, fmt "\n", ##__VA_ARGS__);      \
		logbuffer_log(drvdata->devlog, "[I] " fmt, ##__VA_ARGS__);   \
		break;                                                       \
	case LOGLEVEL_DEBUG:                                                 \
	default:                                                             \
		dev_dbg(&drvdata->pdev->dev, fmt "\n", ##__VA_ARGS__);       \
		logbuffer_log(drvdata->devlog, "[D] " fmt, ##__VA_ARGS__);   \
	}                                                                    \
}
#define LOGE(drvdata, fmt, ...) LOG(LOGLEVEL_ERR, drvdata, fmt, ##__VA_ARGS__)
#define LOGW(drvdata, fmt, ...) LOG(LOGLEVEL_WARNING, drvdata, fmt, ##__VA_ARGS__)
#define LOGI(drvdata, fmt, ...) LOG(LOGLEVEL_INFO, drvdata, fmt, ##__VA_ARGS__)
#define LOGD(drvdata, fmt, ...) LOG(LOGLEVEL_DEBUG, drvdata, fmt, ##__VA_ARGS__)

#define SYNC_GPIO_SOURCE_CURRENT(drvdata, gpio, label)                       \
{                                                                            \
	if (gpio_is_valid(gpio)) {                                           \
		drvdata->bt_power_src_status[label ## _CURRENT] =            \
			gpio_get_value(gpio);                                \
		LOGD(drvdata, "%s(%d) value(%d)", #label, gpio,              \
			drvdata->bt_power_src_status[label ## _CURRENT]);    \
	} else {                                                             \
		drvdata->bt_power_src_status[label ## _CURRENT] =            \
			DEFAULT_INVALID_VALUE;                               \
		LOGD(drvdata, "%s not configured", #label);                  \
	}                                                                    \
}

#define SET_GPIO_SOURCE_STATE(drvdata, gpio, label, value)                    \
{                                                                             \
	if (gpio_is_valid(gpio)) {                                            \
		gpio_set_value(gpio, value);                                  \
		drvdata->bt_power_src_status[label] = value;                  \
		LOGD(drvdata, "Set %s(%d) value(%d)", #label, gpio,           \
			drvdata->bt_power_src_status[label]);                 \
	} else {                                                              \
		drvdata->bt_power_src_status[label] = DEFAULT_INVALID_VALUE;  \
		LOGD(drvdata, "%s not configured", #label);                   \
	}                                                                     \
}

// Regulator structure for QCA6174/QCA9377/QCA9379 BT SoC series
static struct bt_power_vreg_data bt_vregs_info_qca61x4_937x[] = {
	{NULL, "qcom,bt-vdd-aon", 928000, 928000, 0, false, false,
		{BT_VDD_AON_LDO, BT_VDD_AON_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-io", 1710000, 3460000, 0, false, false,
		{BT_VDD_IO_LDO, BT_VDD_IO_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-core", 3135000, 3465000, 0, false, false,
		{BT_VDD_CORE_LDO, BT_VDD_CORE_LDO_CURRENT}},
};

// Regulator structure for QCA6390 and QCA6490 BT SoC series
static struct bt_power_vreg_data bt_vregs_info_qca6x9x[] = {
	{NULL, "qcom,bt-vdd-io",      1800000, 1800000, 0, false, true,
		{BT_VDD_IO_LDO, BT_VDD_IO_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-aon",     966000,  966000,  0, false, true,
		{BT_VDD_AON_LDO, BT_VDD_AON_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfacmn",  950000,  950000,  0, false, true,
		{BT_VDD_RFACMN, BT_VDD_RFACMN_CURRENT}},
	/* BT_CX_MX */
	{NULL, "qcom,bt-vdd-dig",      966000,  966000,  0, false, true,
		{BT_VDD_DIG_LDO, BT_VDD_DIG_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa-0p8",  950000,  952000,  0, false, true,
		{BT_VDD_RFA_0p8, BT_VDD_RFA_0p8_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa1",     1900000, 1900000, 0, false, true,
		{BT_VDD_RFA1_LDO, BT_VDD_RFA1_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa2",     1900000, 1900000, 0, false, true,
		{BT_VDD_RFA2_LDO, BT_VDD_RFA2_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-asd",      2800000, 2800000, 0, false, true,
		{BT_VDD_ASD_LDO, BT_VDD_ASD_LDO_CURRENT}},
};


// Regulator structure for WCN7850 BT SoC series
static struct bt_power_vreg_data bt_vregs_info_wcn7850[] = {
	{NULL, "qcom,bt-vdd-io",      1800000, 1800000, 0, false, true,
		{BT_VDD_IO_LDO, BT_VDD_IO_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-aon",     950000,  950000,  0, false, true,
		{BT_VDD_AON_LDO, BT_VDD_AON_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfacmn",  950000,  950000,  0, false, true,
		{BT_VDD_RFACMN, BT_VDD_RFACMN_CURRENT}},
	/* BT_CX_MX */
	{NULL, "qcom,bt-vdd-dig",      950000,  950000,  0, false, true,
		{BT_VDD_DIG_LDO, BT_VDD_DIG_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa-0p8",  950000,  952000,  0, false, true,
		{BT_VDD_RFA_0p8, BT_VDD_RFA_0p8_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa1",     1900000, 1900000, 0, false, true,
		{BT_VDD_RFA1_LDO, BT_VDD_RFA1_LDO_CURRENT}},
	{NULL, "qcom,bt-vdd-rfa2",     1350000, 1350000, 0, false, true,
		{BT_VDD_RFA2_LDO, BT_VDD_RFA2_LDO_CURRENT}},
};

// Regulator structure for WCN399x BT SoC series
static const struct bt_power bt_vreg_info_wcn399x = {
	.compatible = "qcom,wcn3990",
	.vregs = (struct bt_power_vreg_data []) {
		{NULL, "qcom,bt-vdd-io",   1700000, 1900000, 0, false, false,
			{BT_VDD_IO_LDO, BT_VDD_IO_LDO_CURRENT}},
		{NULL, "qcom,bt-vdd-core", 1304000, 1304000, 0, false, false,
			{BT_VDD_CORE_LDO, BT_VDD_CORE_LDO_CURRENT}},
		{NULL, "qcom,bt-vdd-pa",   3000000, 3312000, 0, false, false,
			{BT_VDD_PA_LDO, BT_VDD_PA_LDO_CURRENT}},
		{NULL, "qcom,bt-vdd-xtal", 1700000, 1900000, 0, false, false,
			{BT_VDD_XTAL_LDO, BT_VDD_XTAL_LDO_CURRENT}},
	},
	.num_vregs = 4,
};

static const struct bt_power bt_vreg_info_qca6174 = {
	.compatible = "qcom,qca6174",
	.vregs = bt_vregs_info_qca61x4_937x,
	.num_vregs = ARRAY_SIZE(bt_vregs_info_qca61x4_937x),
};

static const struct bt_power bt_vreg_info_qca6390 = {
	.compatible = "qcom,qca6390",
	.vregs = bt_vregs_info_qca6x9x,
	.num_vregs = ARRAY_SIZE(bt_vregs_info_qca6x9x),
};

static const struct bt_power bt_vreg_info_qca6490 = {
	.compatible = "qcom,qca6490",
	.vregs = bt_vregs_info_qca6x9x,
	.num_vregs = ARRAY_SIZE(bt_vregs_info_qca6x9x),
};

static const struct bt_power bt_vreg_info_wcn7850 = {
	.compatible = "qcom,wcn7850",
	.vregs = bt_vregs_info_wcn7850,
	.num_vregs = ARRAY_SIZE(bt_vregs_info_wcn7850),
};

static struct bt_power bt_vreg_info_wcn6740 = {
	.compatible = "qcom,wcn6740",
	.vregs = NULL,
	.num_vregs = 0,
};

static const struct of_device_id bt_power_match_table[] = {
	{	.compatible = "qcom,qca6174", .data = &bt_vreg_info_qca6174},
	{	.compatible = "qcom,wcn3990", .data = &bt_vreg_info_wcn399x},
	{	.compatible = "qcom,qca6390", .data = &bt_vreg_info_qca6390},
	{	.compatible = "qcom,qca6490", .data = &bt_vreg_info_qca6490},
	{	.compatible = "qcom,wcn7850", .data = &bt_vreg_info_wcn7850},
	{	.compatible = "qcom,wcn6740", .data = &bt_vreg_info_wcn6740},
	{},
};

static int bt_power_vreg_set(struct btpower_platform_data *drvdata,
			     enum bt_power_modes mode);
static int btpower_enable_ipa_vreg(struct btpower_platform_data *drvdata);

static void btpower_uart_transport_locked(struct btpower_platform_data *drvdata,
					  bool locked)
{
	LOGD(drvdata, "%s: %s", __func__, (locked ? "busy" : "idle"));
	/* idle status: true - idle, false otherwise */
	exynos_update_ip_idle_status(drvdata->uart_idle_index, !locked);
}

static irqreturn_t btpower_host_wake_isr(int irq, void *data)
{
	struct btpower_platform_data *drvdata = data;
	int host_waking = gpio_get_value(drvdata->bt_gpio_host_wake);
	struct kernel_siginfo siginfo;
	int rc = 0;

	drvdata->hostwake_count += host_waking;
	LOGD(drvdata, "IRQ(%d -> %d) count(%u)", drvdata->hostwake_state,
		host_waking, drvdata->hostwake_count);

	if (drvdata->reftask_obs == NULL) {
		LOGI(drvdata, "ignore IRQ(%d) count(%u)", host_waking,
			drvdata->hostwake_count);
		return IRQ_HANDLED;
	}

	if (drvdata->hostwake_state == 2) {
		LOGD(drvdata, "IRQ(%d) count(%u) after flipped", host_waking,
			drvdata->hostwake_count);
		drvdata->hostwake_state = 0;
	}

	if (drvdata->hostwake_state != host_waking) {
		drvdata->hostwake_state = host_waking;
		if (host_waking == 1)
			btpower_uart_transport_locked(drvdata, true);
	} else {
		LOGW(drvdata, "IRQ(%d) count(%u) is flipping", host_waking,
			drvdata->hostwake_count);
		if (host_waking == 1)
			/* HIGH --> LOW --> HIGH because of incoming packets timing
			 * Ignore this IRQ since nothing is changing
			 */
			return IRQ_HANDLED;
		/* FW timer too short but LOW --> HIGH --> LOW too soon */
		drvdata->hostwake_state = 2;
		btpower_uart_transport_locked(drvdata, true);
	}

	/* Sending signal to HAL layer */
	memset(&siginfo, 0, sizeof(siginfo));
	siginfo.si_signo = SIGIO;
	siginfo.si_code = SI_QUEUE;
	siginfo.si_int = drvdata->hostwake_state;
	rc = send_sig_info(siginfo.si_signo, &siginfo, drvdata->reftask_obs);
	if (rc < 0) {
		LOGE(drvdata, "failed (%d) to send SIG to HAL(%d)", rc,
			drvdata->reftask_obs->pid);
	}
	return IRQ_HANDLED;
}

static int bt_vreg_enable(struct btpower_platform_data *drvdata,
			  struct bt_power_vreg_data *vreg)
{
	int rc = 0;

	LOGD(drvdata, "vreg_en for : %s", vreg->name);

	if (vreg->is_enabled)
		return rc;

	if ((vreg->min_vol != 0) && (vreg->max_vol != 0)) {
		rc = regulator_set_voltage(vreg->reg, vreg->min_vol,
					vreg->max_vol);
		if (rc < 0) {
			LOGE(drvdata, "regulator_enable(%s) failed. rc=%d",
				vreg->name, rc);
			goto out;
		}
	}

	if (vreg->load_curr >= 0) {
		rc = regulator_set_load(vreg->reg, vreg->load_curr);
		if (rc < 0) {
			LOGE(drvdata, "regulator_set_load(%s) failed rc=%d",
				vreg->name, rc);
			goto out;
		}
	}

	rc = regulator_enable(vreg->reg);
	if (rc < 0) {
		LOGE(drvdata, "regulator_enable(%s) failed. rc=%d", vreg->name, rc);
		goto out;
	}
	vreg->is_enabled = true;

out:
	return rc;
}

static int bt_vreg_enable_retention(struct btpower_platform_data *drvdata,
				    const struct bt_power_vreg_data *vreg)
{
	int rc = 0;

	if (!vreg)
		return rc;

	LOGD(drvdata, "enable_retention for : %s", vreg->name);

	if (!vreg->is_enabled || !vreg->is_retention_supp)
		return rc;

	if ((vreg->min_vol != 0) && (vreg->max_vol != 0)) {
		/* Set the min voltage to 0 */
		rc = regulator_set_voltage(vreg->reg, 0, vreg->max_vol);
		if (rc < 0) {
			LOGE(drvdata, "regulator_set_voltage(%s) failed rc=%d",
				vreg->name, rc);
			goto out;
		}
	}
	if (vreg->load_curr >= 0) {
		rc = regulator_set_load(vreg->reg, 0);
		if (rc < 0) {
			LOGE(drvdata, "regulator_set_load(%s) failed rc=%d",
				vreg->name, rc);
		}
	}

out:
	return rc;
}

static int bt_vreg_disable(struct btpower_platform_data *drvdata,
			   struct bt_power_vreg_data *vreg)
{
	int rc = 0;

	if (!vreg)
		return rc;

	LOGD(drvdata, "vreg_off for : %s", vreg->name);

	if (!vreg->is_enabled)
		return rc;

	rc = regulator_disable(vreg->reg);
	if (rc < 0) {
		LOGE(drvdata, "regulator_disable(%s) failed. rc=%d", vreg->name, rc);
		goto out;
	}
	vreg->is_enabled = false;

	if ((vreg->min_vol != 0) && (vreg->max_vol != 0)) {
		/* Set the min voltage to 0 */
		rc = regulator_set_voltage(vreg->reg, 0, vreg->max_vol);
		if (rc < 0) {
			LOGE(drvdata, "regulator_set_voltage(%s) failed rc=%d",
				vreg->name, rc);
			goto out;
		}
	}
	if (vreg->load_curr >= 0) {
		rc = regulator_set_load(vreg->reg, 0);
		if (rc < 0) {
			LOGE(drvdata, "regulator_set_load(%s) failed rc=%d",
				vreg->name, rc);
		}
	}

out:
	return rc;
}

static int bt_clk_enable(struct btpower_platform_data *drvdata,
			 struct bt_power_clk_data *clk)
{
	int rc = 0;

	LOGD(drvdata, "%s enabling", clk->name);

	/* Get the clock handle for vreg */
	if (!clk->clk || clk->is_enabled) {
		LOGE(drvdata, "error - node: %pK, clk->is_enabled:%d", clk->clk,
			clk->is_enabled);
		return -EINVAL;
	}

	rc = clk_prepare_enable(clk->clk);
	if (rc) {
		LOGE(drvdata, "failed to enable %s, rc(%d)", clk->name, rc);
		return rc;
	}

	clk->is_enabled = true;
	return rc;
}

static int bt_clk_disable(struct btpower_platform_data *drvdata,
			  struct bt_power_clk_data *clk)
{
	LOGD(drvdata, "%s disabling", clk->name);

	/* Get the clock handle for vreg */
	if (!clk->clk || !clk->is_enabled) {
		LOGE(drvdata, "error - node: %pK, clk->is_enabled:%d", clk->clk,
			clk->is_enabled);
		return -EINVAL;
	}
	clk_disable_unprepare(clk->clk);

	clk->is_enabled = false;
	return 0;
}

static void btpower_set_xo_clk_gpio_state(struct btpower_platform_data *drvdata,
					  bool enable)
{
	int xo_clk_gpio = drvdata->xo_gpio_clk;
	int retry = 0;
	int rc = 0;

	if (!gpio_is_valid(xo_clk_gpio))
		return;

	do {
		rc = gpio_request(xo_clk_gpio, "bt_xo_clk_gpio");
		if (rc == 0)
			break;
		if (retry++ >= XO_CLK_RETRY_COUNT_MAX) {
			LOGE(drvdata, "unable to request XO clk gpio %d (%d)",
				xo_clk_gpio, rc);
			return;
		}
		/* wait for ~(10 - 20) ms and try again */
		usleep_range(10000, 20000);
	} while (1);

	rc = gpio_get_value(xo_clk_gpio);
	if (enable) {
		gpio_direction_output(xo_clk_gpio, 1);
		/* XO CLK must be asserted for some time before BT_EN */
		usleep_range(5000, 7000);
	} else {
		/* Assert XO CLK ~(2-5)ms before off for valid latch in HW */
		usleep_range(4000, 6000);
		gpio_direction_output(xo_clk_gpio, 0);
	}

	if (rc != enable)
		LOGI(drvdata, "gpio(%d) %d to %d", xo_clk_gpio, rc,
			enable);

	gpio_free(xo_clk_gpio);
}

static int btpower_gpio_source_request(struct btpower_platform_data *drvdata,
				       int gpio, const char *label)
{
	int rc = gpio_request(gpio, label);
	if (rc) {
		LOGE(drvdata, "unable to request gpio %s(%d) (%d)", label, gpio, rc);
		return rc;
	}
	return rc;
}

static int btpower_gpio_acquire_output(struct btpower_platform_data *drvdata,
				       int gpio, const char *label, bool value)
{
	int rc;

	rc = btpower_gpio_source_request(drvdata, gpio, label);
	if (rc)
		return rc;

	rc = gpio_direction_output(gpio, value);
	if (rc) {
		LOGE(drvdata, "unable to set output gpio %s(%d) (%d)", label, gpio, rc);
		gpio_free(gpio);
		return rc;
	}
	return rc;
}

static int btpower_gpio_acquire_input(struct btpower_platform_data *drvdata,
				      int gpio, const char *label)
{
	int rc;

	rc = btpower_gpio_source_request(drvdata, gpio, label);
	if (rc)
		return rc;

	rc = gpio_direction_input(gpio);
	if (rc) {
		LOGE(drvdata, "unable to set input gpio %s(%d) (%d)", label, gpio, rc);
		gpio_free(gpio);
		return rc;
	}
	return rc;
}

static void bt_configure_wakeup_gpios(struct btpower_platform_data *drvdata, bool on)
{
	int bt_gpio_dev_wake = drvdata->bt_gpio_dev_wake;
	int bt_host_wake_gpio = drvdata->bt_gpio_host_wake;
	int rc;

	if (!on) {
		if (gpio_is_valid(bt_host_wake_gpio) && drvdata->pwr_state != BT_POWER_DISABLE) {
			LOGD(drvdata, "BT-OFF bt-hostwake-gpio(%d) IRQ(%d) value(%d)",
				bt_host_wake_gpio, drvdata->irq,
				gpio_get_value(bt_host_wake_gpio));
                        rc = disable_irq_wake(drvdata->irq);
                        if(rc) {
                          LOGE(drvdata, "Failed to disable IRQ wake");
                        }
			free_irq(drvdata->irq, drvdata);
		}

		if (gpio_is_valid(bt_gpio_dev_wake))
			gpio_set_value(bt_gpio_dev_wake, 0);
		return;
	}

	if (gpio_is_valid(bt_gpio_dev_wake)) {
		gpio_set_value(bt_gpio_dev_wake, 1);
		LOGD(drvdata, "BT-ON asserting BT_WAKE(%d)", bt_gpio_dev_wake);
	}

	if (gpio_is_valid(bt_host_wake_gpio) && drvdata->pwr_state == BT_POWER_DISABLE) {
		LOGD(drvdata, "BT-ON bt-host_wake-gpio(%d) IRQ(%d)",
			bt_host_wake_gpio, drvdata->irq);
		rc = request_irq(drvdata->irq, btpower_host_wake_isr,
				 IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				 "btpower_hostwake_isr", drvdata);
		if (rc)
			LOGE(drvdata, "unable to request IRQ %d (%d)",
				bt_host_wake_gpio, rc);
                rc = enable_irq_wake(drvdata->irq);
                if(rc) {
                  LOGE(drvdata, "Failed to enable IRQ wake");
                }
		drvdata->hostwake_state = -1;
		drvdata->hostwake_count = 0;
	}
}

static int bt_configure_gpios(struct btpower_platform_data *drvdata, bool on)
{
	int rc = 0;
	int bt_reset_gpio = drvdata->bt_gpio_sys_rst;
	int wl_reset_gpio = drvdata->wl_gpio_sys_rst;
	int bt_sw_ctrl_gpio = drvdata->bt_gpio_sw_ctrl;
	int bt_debug_gpio = drvdata->bt_gpio_debug;

	LOGI(drvdata, "BT_RESET_GPIO(%d) value(%d) enabling: %s", bt_reset_gpio,
		gpio_get_value(bt_reset_gpio), (on ? "True" : "False"));

	if (!on)
		bt_configure_wakeup_gpios(drvdata, on);

	/* always reset the controller no metter ON or OFF */
	SET_GPIO_SOURCE_STATE(drvdata, bt_reset_gpio, BT_RESET_GPIO, 0);
	msleep(on ? 100 : 50);
	SYNC_GPIO_SOURCE_CURRENT(drvdata, bt_sw_ctrl_gpio, BT_SW_CTRL_GPIO);

	if (!on)
		return 0;

	if (gpio_is_valid(wl_reset_gpio))
		LOGD(drvdata, "BT-ON wl-reset-gpio(%d) value(%d)",
			wl_reset_gpio, gpio_get_value(wl_reset_gpio));

	if (!gpio_is_valid(wl_reset_gpio) || gpio_get_value(wl_reset_gpio)) {
		btpower_set_xo_clk_gpio_state(drvdata, true);
		LOGI(drvdata, "BT-ON asserting BT_EN (with WLAN)");
		SET_GPIO_SOURCE_STATE(drvdata, bt_reset_gpio, BT_RESET_GPIO, 1);
	}
	if (gpio_is_valid(wl_reset_gpio) && !gpio_get_value(wl_reset_gpio)) {
		if (gpio_get_value(bt_reset_gpio)) {
			LOGW(drvdata, "WLAN OFF / BT ON too close. Delay BT_EN");
			SET_GPIO_SOURCE_STATE(drvdata, bt_reset_gpio,
				BT_RESET_GPIO, 0);
			msleep(100);
			LOGW(drvdata, "100ms delay for AON output to fully discharge");
		}
		btpower_set_xo_clk_gpio_state(drvdata, true);
		LOGI(drvdata, "BT-ON asserting BT_EN without WLAN");
		SET_GPIO_SOURCE_STATE(drvdata, bt_reset_gpio, BT_RESET_GPIO, 1);
	}

	msleep(50);

	bt_configure_wakeup_gpios(drvdata, on);

	/* Check if SW_CTRL is asserted */
	SYNC_GPIO_SOURCE_CURRENT(drvdata, bt_sw_ctrl_gpio, BT_SW_CTRL_GPIO);
	if (drvdata->bt_power_src_status[BT_SW_CTRL_GPIO_CURRENT] == 0) {
		/* SW_CTRL not asserted, assert debug GPIO */
		if (gpio_is_valid(bt_debug_gpio))
			gpio_set_value(bt_debug_gpio, 1);
		LOGW(drvdata, "BT_SW_CTRL_GPIO(%d) value(%d) not asserted", bt_sw_ctrl_gpio,
			drvdata->bt_power_src_status[BT_SW_CTRL_GPIO_CURRENT]);
	}

	return rc;
}

static int bluetooth_power(struct btpower_platform_data *drvdata,
			   enum bt_power_modes mode)
{
	int rc = 0;

	if (!drvdata) {
		LOGE(drvdata, "device not ready");
		return -ENODEV;
	}

	LOGD(drvdata, "%s: mode %d -> %d", __func__, drvdata->pwr_state, mode);

	switch (mode) {
	case BT_POWER_DISABLE:
		bt_configure_gpios(drvdata, false);
		btpower_uart_transport_locked(drvdata, false);
		drvdata->pwr_state = BT_POWER_DISABLE;
		goto clk_disable;
	case BT_POWER_ENABLE:
		rc = bt_power_vreg_set(drvdata, BT_POWER_ENABLE);
		if (rc < 0) {
			LOGE(drvdata, "Regulators config failed");
			goto vreg_disable;
		}
		/* Parse dt_info and check if a target requires clock voting.
		 * Enable BT clock when BT is on and disable it when BT is off
		 */
		if (drvdata->bt_chip_clk) {
			rc = bt_clk_enable(drvdata, drvdata->bt_chip_clk);
			if (rc < 0) {
				LOGE(drvdata, "CLK config failed");
				goto vreg_disable;
			}
		}
		drvdata->bt_power_src_status[BT_RESET_GPIO] =
			DEFAULT_INVALID_VALUE;
		rc = bt_configure_gpios(drvdata, true);
		if (rc < 0) {
			LOGE(drvdata, "GPIO config failed");
			goto clk_disable;
		}
		btpower_uart_transport_locked(drvdata, true);
		drvdata->pwr_state = BT_POWER_ENABLE;
		return rc;
	case BT_POWER_RETENTION:
		bt_power_vreg_set(drvdata, BT_POWER_RETENTION);
		drvdata->pwr_state = BT_POWER_RETENTION;
		return rc;
	default:
		LOGE(drvdata, "Invalid power mode: %d", mode);
		return -1;
	}

clk_disable:
	if (drvdata->bt_chip_clk)
		bt_clk_disable(drvdata, drvdata->bt_chip_clk);
vreg_disable:
	bt_power_vreg_set(drvdata, BT_POWER_DISABLE);
	return rc;
}

static int btpower_toggle_radio(void *data, bool blocked)
{
	struct btpower_platform_data *drvdata = data;
	/* BT-OFF: true; BT-ON: false */
	bool previous_blocked = drvdata->pwr_state == BT_POWER_DISABLE;

	if (previous_blocked != blocked)
		return drvdata->bt_power_setup(drvdata, !blocked);
	return 0;
}

static const struct rfkill_ops btpower_rfkill_ops = {
	.set_block = btpower_toggle_radio,
};

static ssize_t extldo_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "false\n");
}
static DEVICE_ATTR_RO(extldo);

static int btpower_rfkill_probe(struct platform_device *pdev,
				struct btpower_platform_data *drvdata)
{
	struct rfkill *rfkill;
	int ret;

	rfkill = rfkill_alloc("bt_power", &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			      &btpower_rfkill_ops, drvdata);
	if (!rfkill) {
		LOGE(drvdata, "rfkill allocate failed");
		return -ENOMEM;
	}

	/* add file into rfkill to handle LDO27 */
	ret = device_create_file(&pdev->dev, &dev_attr_extldo);
	if (ret < 0)
		LOGW(drvdata, "device create LDO file error (%d)", ret);

	/* force Bluetooth off during init to allow for user control */
	rfkill_init_sw_state(rfkill, true);
	drvdata->pwr_state = BT_POWER_DISABLE;
	drvdata->bt_power_setup(drvdata, BT_POWER_DISABLE);

	ret = rfkill_register(rfkill);
	if (ret) {
		LOGE(drvdata, "rfkill register failed=%d", ret);
		rfkill_destroy(rfkill);
		return ret;
	}

	drvdata->rfkill = rfkill;

	return 0;
}

static void btpower_rfkill_remove(struct platform_device *pdev)
{
	struct btpower_platform_data *drvdata = platform_get_drvdata(pdev);
	struct rfkill *rfkill;

	if (!drvdata || !drvdata->rfkill)
		return;

	LOGD(drvdata, "%s: entry", __func__);

	rfkill = drvdata->rfkill;
	drvdata->rfkill = NULL;
	device_remove_file(&pdev->dev, &dev_attr_extldo);
	rfkill_unregister(rfkill);
	rfkill_destroy(rfkill);
}

static int btpower_open(struct inode *inode, struct file *filp);
static int btpower_release(struct inode *inode, struct file *filp);
static long btpower_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static const struct file_operations bt_dev_fops = {
	.owner = THIS_MODULE,
	.open = btpower_open,
	.release = btpower_release,
	.unlocked_ioctl = btpower_ioctl,
	.compat_ioctl = btpower_ioctl,
};

static int btpower_chardev_create(struct btpower_platform_data *drvdata)
{
	dev_t bpdevt;
	struct class *bpcls;
	struct device *bpdev;
	int ret = 0;

	ret = alloc_chrdev_region(&bpdevt, 0, 1, "bt");
	if (ret || MAJOR(bpdevt) < 0) {
		LOGE(drvdata, "failed to register chardev number (%d)", ret);
		return ret;
	}
	cdev_init(&drvdata->cdev, &bt_dev_fops);
	drvdata->cdev.owner = THIS_MODULE;
	ret = cdev_add(&drvdata->cdev, bpdevt, 1);
	if (ret) {
		LOGE(drvdata, "failed to add chardev (%d)", ret);
                goto class_err;
	}
	LOGD(drvdata, "registered chardev number %d:%d",
		MAJOR(drvdata->cdev.dev), MINOR(drvdata->cdev.dev));

	bpcls = class_create(THIS_MODULE, "bt-dev");
	if (IS_ERR_OR_NULL(bpcls)) {
		ret = PTR_ERR(bpcls);
		LOGE(drvdata, "can't create class (%d)", ret);
		goto class_err;
	}

	bpdev = device_create(bpcls, NULL, drvdata->cdev.dev,
			drvdata, "btpower");
	if (IS_ERR_OR_NULL(bpdev)) {
		ret = PTR_ERR(bpdev);
		LOGE(drvdata, "failed to create device with sysfs (%d)", ret);
		goto device_err;
	}
	drvdata->cls = bpcls;
	return 0;

device_err:
	class_destroy(bpcls);
class_err:
	unregister_chrdev(MAJOR(drvdata->cdev.dev), "bt");
	return ret;
}

static void btpower_chardev_remove(struct btpower_platform_data *drvdata)
{
	if (!drvdata || !drvdata->cls)
		return;

	device_destroy(drvdata->cls, drvdata->cdev.dev);
	class_destroy(drvdata->cls);
	drvdata->cls = NULL;
	unregister_chrdev(MAJOR(drvdata->cdev.dev), "bt");
}

static int bt_dt_parse_vreg_info(struct btpower_platform_data *drvdata,
				 struct bt_power_vreg_data *vreg)
{
	int len, ret = 0;
	const __be32 *prop;
	char prop_name[MAX_PROP_SIZE];
	struct device *dev = &drvdata->pdev->dev;
	struct device_node *np = dev->of_node;
	const char *vreg_name = vreg->name;

	LOGD(drvdata, "vreg device tree parse for %s", vreg_name);

	snprintf(prop_name, sizeof(prop_name), "%s-supply", vreg_name);
	if (!of_parse_phandle(np, prop_name, 0)) {
		LOGW(drvdata, "%s is not provided in device tree", prop_name);
		return ret;
	}

	vreg->reg = regulator_get(dev, vreg_name);
	if (IS_ERR(vreg->reg)) {
		ret = PTR_ERR(vreg->reg);
		vreg->reg = NULL;
		LOGW(drvdata, "failed to get: %s error:%d", vreg_name, ret);
		return ret;
	}

	snprintf(prop_name, sizeof(prop_name), "%s-config", vreg_name);
	prop = of_get_property(dev->of_node, prop_name, &len);
	if (!prop || len != (4 * sizeof(__be32))) {
		LOGI(drvdata, "Property %s %s, use default", prop_name,
			prop ? "invalid format" : "doesn't exist");
	} else {
		vreg->min_vol = be32_to_cpup(&prop[0]);
		vreg->max_vol = be32_to_cpup(&prop[1]);
		vreg->load_curr = be32_to_cpup(&prop[2]);
		vreg->is_retention_supp = be32_to_cpup(&prop[3]);
	}

	LOGD(drvdata, "Got regulator: %s, min_vol: %u, max_vol: %u, load_curr: %u, is_retention_supp: %u",
		vreg->name, vreg->min_vol, vreg->max_vol, vreg->load_curr,
		vreg->is_retention_supp);
	return ret;
}

static int bt_dt_parse_clk_info(struct btpower_platform_data *drvdata,
				struct bt_power_clk_data **clk_data)
{
	int ret = -EINVAL;
	struct device *dev = &drvdata->pdev->dev;
	struct bt_power_clk_data *clk = NULL;
	struct device_node *np = dev->of_node;

	LOGD(drvdata, "%s: entry", __func__);

	*clk_data = NULL;
	if (!of_parse_phandle(np, "clocks", 0)) {
		LOGE(drvdata, "clocks is not provided in device tree");
		return ret;
	}

	clk = devm_kzalloc(dev, sizeof(*clk), GFP_KERNEL);
	if (!clk)
		return -ENOMEM;

	/* Parse clock name from node */
	ret = of_property_read_string_index(np, "clock-names", 0, &(clk->name));
	if (ret < 0) {
		LOGE(drvdata, "reading 'clock-names' failed ret=%d", ret);
		goto err;
	}

	clk->clk = devm_clk_get(dev, clk->name);
	if (IS_ERR(clk->clk)) {
		ret = PTR_ERR(clk->clk);
		LOGE(drvdata, "failed to get %s ret=%d", clk->name, ret);
		clk->clk = NULL;
		goto err;
	}

	*clk_data = clk;

	return ret;

err:
	devm_kfree(dev, clk);
	return ret;
}

static int bt_power_vreg_get(struct platform_device *pdev,
			     struct btpower_platform_data *drvdata)
{
	int num_vregs, i, ret = 0;
	const struct bt_power *pwrdata = of_device_get_match_data(&pdev->dev);

	if (!pwrdata) {
		LOGE(drvdata, "failed to get dev node");
		return -EINVAL;
	}

	memcpy(&drvdata->compatible, &pwrdata->compatible, sizeof(drvdata->compatible));
	drvdata->vreg_info = pwrdata->vregs;
	num_vregs = drvdata->num_vregs = pwrdata->num_vregs;
	for (i = 0; i < num_vregs; i++) {
		ret = bt_dt_parse_vreg_info(drvdata, &drvdata->vreg_info[i]);
		/* No point to go further if failed to get regulator handler */
		if (ret)
			break;
	}

	return ret;
}

static int bt_power_vreg_set(struct btpower_platform_data *drvdata,
			     enum bt_power_modes mode)
{
	int num_vregs, i, ret = 0;
	int log_indx;
	struct bt_power_vreg_data *vreg_info = NULL;

	num_vregs = drvdata->num_vregs;
	switch (mode) {
	case BT_POWER_DISABLE:
		for (i = 0; i < num_vregs; i++) {
			vreg_info = &drvdata->vreg_info[i];
			ret = bt_vreg_disable(drvdata, vreg_info);
		}
		break;
	case BT_POWER_ENABLE:
		for (i = 0; i < num_vregs; i++) {
			vreg_info = &drvdata->vreg_info[i];
			if (!vreg_info->reg)
				continue;
			log_indx = vreg_info->indx.init;
			drvdata->bt_power_src_status[log_indx] =
				DEFAULT_INVALID_VALUE;
			ret = bt_vreg_enable(drvdata, vreg_info);
			if (ret < 0)
				return ret;
			if (!vreg_info->is_enabled)
				continue;
			drvdata->bt_power_src_status[log_indx] =
				regulator_get_voltage(vreg_info->reg);
		}
		break;
	case BT_POWER_RETENTION:
		for (i = 0; i < num_vregs; i++) {
			vreg_info = &drvdata->vreg_info[i];
			ret = bt_vreg_enable_retention(drvdata, vreg_info);
		}
		break;
	default:
		LOGE(drvdata, "Invalid power mode: %d", mode);
		ret = -1;
	}
	return ret;
}

static void bt_power_vreg_put(struct btpower_platform_data *drvdata)
{
	int i;
	const struct bt_power_vreg_data *vreg_info = NULL;
	int num_vregs;

	if (!drvdata)
		return;

	num_vregs = drvdata->num_vregs;
	for (i = 0; i < num_vregs; i++) {
		vreg_info = &drvdata->vreg_info[i];
		if (vreg_info->reg)
			regulator_put(vreg_info->reg);
	}
}

static void btpower_gpios_source_release(struct btpower_platform_data *drvdata);
static int btpower_gpios_source_initialize(struct btpower_platform_data *drvdata)
{
	int rc = 0;

	if (!IS_ERR_OR_NULL(drvdata->pinctrl_default_state)) {
		rc = pinctrl_select_state(drvdata->pinctrls,
					drvdata->pinctrl_default_state);
		if (unlikely(rc))
			LOGW(drvdata, "failed to set default pinctrl state rc=%d", rc);
	}
	if (!IS_ERR_OR_NULL(drvdata->pinctrl_supply_state)) {
		rc = pinctrl_select_state(drvdata->pinctrls,
					drvdata->pinctrl_supply_state);
		if (unlikely(rc))
			LOGW(drvdata, "failed to set supply pinctrl state rc=%d", rc);
	}

	rc = btpower_gpio_acquire_output(drvdata, drvdata->bt_gpio_sys_rst,
					"bt_sys_rst_n", 0);
	if (rc) {
		drvdata->bt_gpio_sys_rst = -1;
		return rc;
	}

	if (gpio_is_valid(drvdata->bt_gpio_sw_ctrl)) {
		rc = btpower_gpio_acquire_input(drvdata, drvdata->bt_gpio_sw_ctrl,
						"bt_sw_ctrl_n");
		if (rc) {
			drvdata->bt_gpio_sw_ctrl = -1;
			goto gpio_failure;
		}
	}

	if (gpio_is_valid(drvdata->bt_gpio_debug)) {
		rc = btpower_gpio_acquire_output(drvdata, drvdata->bt_gpio_debug,
						"bt_debug_n", 0);
		if (rc) {
			drvdata->bt_gpio_debug = -1;
			goto gpio_failure;
		}
	}

	return 0;

gpio_failure:
	btpower_gpios_source_release(drvdata);
	return rc;
}

static void btpower_gpios_source_release(struct btpower_platform_data *drvdata)
{
	if (gpio_is_valid(drvdata->bt_gpio_debug)) {
		gpio_free(drvdata->bt_gpio_debug);
		drvdata->bt_gpio_debug = -1;
	}
	if (gpio_is_valid(drvdata->bt_gpio_sw_ctrl)) {
		gpio_free(drvdata->bt_gpio_sw_ctrl);
		drvdata->bt_gpio_sw_ctrl = -1;
	}
	gpio_free(drvdata->bt_gpio_sys_rst);
	drvdata->bt_gpio_sys_rst = -1;
}

static int bt_power_populate_dt_pinfo(struct platform_device *pdev,
				      struct btpower_platform_data *drvdata)
{
	int rc;

	LOGD(drvdata, "%s: entry", __func__);

	if (!drvdata)
		return -ENOMEM;

	if (!pdev->dev.of_node)
		return 0;

	rc = bt_power_vreg_get(pdev, drvdata);
	if (rc)
		return rc;

	drvdata->pinctrls = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(drvdata->pinctrls)) {
		LOGW(drvdata, "pinctrls not provided in device tree");
	} else {
		drvdata->pinctrl_default_state =
			pinctrl_lookup_state(drvdata->pinctrls, "default");
		drvdata->pinctrl_supply_state =
			pinctrl_lookup_state(drvdata->pinctrls, "supply");
	}
	if (IS_ERR(drvdata->pinctrl_default_state))
		LOGW(drvdata, "default pinctrl state not provided in device tree");
	if (IS_ERR(drvdata->pinctrl_supply_state))
		LOGW(drvdata, "supply pinctrl state not provided in device tree");

	drvdata->bt_gpio_sys_rst =
		of_get_named_gpio(pdev->dev.of_node, "qcom,bt-reset-gpio", 0);
	if (!gpio_is_valid(drvdata->bt_gpio_sys_rst)) {
		LOGE(drvdata, "bt-reset-gpio not provided in device tree");
		return -EIO;
	}

	drvdata->wl_gpio_sys_rst =
		of_get_named_gpio(pdev->dev.of_node, "qcom,wl-reset-gpio", 0);
	if (!gpio_is_valid(drvdata->wl_gpio_sys_rst))
		LOGI(drvdata, "wl-reset-gpio not provided in device tree");

	drvdata->bt_gpio_dev_wake =
		of_get_named_gpio(pdev->dev.of_node, "qcom,btwake_gpio", 0);
	if (!gpio_is_valid(drvdata->bt_gpio_dev_wake))
		LOGW(drvdata, "btwake-gpio not provided in device tree");

	drvdata->bt_gpio_host_wake =
		of_get_named_gpio(pdev->dev.of_node, "qcom,bthostwake_gpio", 0);
	if (!gpio_is_valid(drvdata->bt_gpio_host_wake)) {
		LOGW(drvdata, "bthostwake_gpio not provided in device tree");
	} else {
		drvdata->irq = gpio_to_irq(drvdata->bt_gpio_host_wake);
	}

	drvdata->bt_gpio_sw_ctrl =
		of_get_named_gpio(pdev->dev.of_node, "qcom,bt-sw-ctrl-gpio", 0);
	if (!gpio_is_valid(drvdata->bt_gpio_sw_ctrl))
		LOGI(drvdata, "bt-sw-ctrl-gpio not provided in device tree");

	drvdata->bt_gpio_debug =
		of_get_named_gpio(pdev->dev.of_node, "qcom,bt-debug-gpio", 0);
	if (!gpio_is_valid(drvdata->bt_gpio_debug))
		LOGI(drvdata, "bt-debug-gpio not provided in device tree");

	drvdata->xo_gpio_clk =
		of_get_named_gpio(pdev->dev.of_node, "qcom,xo-clk-gpio", 0);
	if (!gpio_is_valid(drvdata->xo_gpio_clk))
		LOGI(drvdata, "xo-clk-gpio not provided in device tree");

	rc = bt_dt_parse_clk_info(drvdata, &drvdata->bt_chip_clk);
	if (rc < 0)
		LOGI(drvdata, "clock not provided in device tree");

	drvdata->bt_power_setup = bluetooth_power;

	return 0;
}

static int bt_power_probe(struct platform_device *pdev)
{
	struct btpower_platform_data *drvdata, *pdata;
	int ret = 0;
	int itr;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->pdev = pdev;
	/* Fill whole array with -2 i.e NOT_AVAILABLE state by default
	 * for any GPIO or Reg handle.
	 */
	for (itr = PWR_SRC_INIT_STATE_IDX; itr < BT_POWER_SRC_SIZE; ++itr)
		drvdata->bt_power_src_status[itr] = PWR_SRC_NOT_AVAILABLE;

	if (pdev->dev.of_node) {
		drvdata->devlog = logbuffer_register("btpower");
		if (IS_ERR_OR_NULL(drvdata->devlog)) {
			dev_err(&pdev->dev, "Failed to register logbuffer\n");
			drvdata->devlog = NULL;
		}

		ret = bt_power_populate_dt_pinfo(pdev, drvdata);
		if (ret < 0) {
			LOGE(drvdata, "Failed to populate device tree info");
			goto free_pdata;
		}
		pdev->dev.platform_data = drvdata;
	} else if (pdev->dev.platform_data) {
		pdata = pdev->dev.platform_data;
		/* Optional data set to default if not provided */
		if (!pdata->bt_power_setup)
			pdata->bt_power_setup = bluetooth_power;

		if (IS_ERR_OR_NULL(pdata->devlog))
			pdata->devlog = logbuffer_register("btpower");
		if (IS_ERR_OR_NULL(pdata->devlog)) {
			dev_err(&pdev->dev, "Failed to register logbuffer\n");
			pdata->devlog = NULL;
		}

		memcpy(drvdata, pdata, sizeof(*drvdata));
	} else {
		dev_err(&pdev->dev, "Failed to get platform data\n");
		goto free_pdata;
	}
	drvdata->pwr_state = BT_POWER_DISABLE;

	ret = btpower_gpios_source_initialize(drvdata);
	if (ret < 0)
		goto free_pdata;

	drvdata->uart_idle_index = exynos_get_idle_ip_index("bluetooth");
	ret = btpower_rfkill_probe(pdev, drvdata);
	if (ret < 0)
		goto free_gpio;

	ret = btpower_chardev_create(drvdata);
	if (ret) {
		btpower_rfkill_remove(pdev);
		goto free_gpio;
	}

	btpower_aop_mbox_init(drvdata);

	platform_set_drvdata(pdev, drvdata);

	return 0;

free_gpio:
	btpower_gpios_source_release(drvdata);
free_pdata:
	if (!IS_ERR_OR_NULL(drvdata->devlog))
		logbuffer_unregister(drvdata->devlog);
	kfree(drvdata);
	return ret;
}

static int bt_power_remove(struct platform_device *pdev)
{
	struct btpower_platform_data *drvdata = platform_get_drvdata(pdev);

	LOGD(drvdata, "%s: entry", __func__);

	if (!drvdata)
		return 0;

	btpower_chardev_remove(drvdata);
	btpower_rfkill_remove(pdev);
	bt_power_vreg_put(drvdata);

	btpower_gpios_source_release(drvdata);
	if (!IS_ERR_OR_NULL(drvdata->devlog))
		logbuffer_unregister(drvdata->devlog);
	kfree(drvdata);

	return 0;
}

int btpower_register_slimdev(struct device *dev)
{
	struct btpower_platform_data *drvdata;

	if (dev == NULL || dev_get_drvdata(dev) == NULL) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		return -EINVAL;
	}
	drvdata = dev_get_drvdata(dev);
	drvdata->slim_dev = dev;
	LOGD(drvdata, "%s: ready", __func__);
	return 0;
}
EXPORT_SYMBOL(btpower_register_slimdev);

int btpower_get_chipset_version(struct btpower_platform_data *drvdata)
{
	return drvdata->chipset_version;
}
EXPORT_SYMBOL(btpower_get_chipset_version);

static void set_pwr_srcs_status(struct btpower_platform_data *drvdata,
				const struct bt_power_vreg_data *handle)
{
	int ldo_index;

	if (!handle)
		return;

	ldo_index = handle->indx.crash;
	if (handle->is_enabled && (regulator_is_enabled(handle->reg))) {
		drvdata->bt_power_src_status[ldo_index] =
			(int)regulator_get_voltage(handle->reg);
		LOGD(drvdata, "%s(%pK) value(%d)", handle->name, handle,
			drvdata->bt_power_src_status[ldo_index]);
	} else {
		drvdata->bt_power_src_status[ldo_index] = DEFAULT_INVALID_VALUE;
		LOGE(drvdata, "%s is_enabled: %d", handle->name, handle->is_enabled);
	}
}

static int btpower_open(struct inode *inode, struct file *filp)
{
	filp->private_data =
		container_of(inode->i_cdev, struct btpower_platform_data, cdev);
	return 0;
}

static int btpower_release(struct inode *inode, struct file *filp)
{
	struct btpower_platform_data *drvdata = filp->private_data;

	LOGD(drvdata, "filp %pK releasing", filp);

	/* delete the task if the caller is clossing the control node */
	if (filp == drvdata->reffilp_obs) {
		LOGD(drvdata, "OBS tid %d node released",
			drvdata->reftask_obs->pid);
		drvdata->reffilp_obs = NULL;
		drvdata->reftask_obs = NULL;
	}
	return 0;
}

static long btpower_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct btpower_platform_data *drvdata = file->private_data;
	int ret = 0, pwr_cntrl = 0;
	enum btpower_obs_param clk_cntrl;
	int chipset_version = 0;
	int itr, num_vregs;
	const struct bt_power_vreg_data *vreg_info = NULL;

	if (!drvdata) {
		pr_err("%s: device not ready\n", __func__);
		return -ENODEV;
	}

	switch (cmd) {
	case BT_CMD_OBS_SIGNAL_TASK:
		drvdata->reffilp_obs = file;
		drvdata->reftask_obs = get_current();
		drvdata->hostwake_state = -1;
		drvdata->hostwake_count = 0;
		LOGI(drvdata, "BT_CMD_OBS_SIGNAL_TASK tid %d filp %pK",
			drvdata->reftask_obs->pid, file);
		break;
	case BT_CMD_OBS_VOTE_CLOCK:
		if (!gpio_is_valid(drvdata->bt_gpio_dev_wake)) {
			LOGW(drvdata, "BT_CMD_OBS_VOTE_CLOCK bt_dev_wake_n(%d) not configured",
				drvdata->bt_gpio_dev_wake);
			return -EIO;
		}
		clk_cntrl = (enum btpower_obs_param)arg;
		switch (clk_cntrl) {
		case BTPOWER_OBS_CLK_OFF:
			btpower_uart_transport_locked(drvdata, false);
			ret = 0;
			break;
		case BTPOWER_OBS_CLK_ON:
			btpower_uart_transport_locked(drvdata, true);
			ret = 0;
			break;
		case BTPOWER_OBS_DEV_OFF:
			gpio_set_value(drvdata->bt_gpio_dev_wake, 0);
			ret = 0;
			break;
		case BTPOWER_OBS_DEV_ON:
			gpio_set_value(drvdata->bt_gpio_dev_wake, 1);
			ret = 0;
			break;
		default:
			LOGW(drvdata, "BT_CMD_OBS_VOTE_CLOCK cntrl(%d) unknown", clk_cntrl);
			return -EINVAL;
		}
		LOGD(drvdata, "BT_CMD_OBS_VOTE_CLOCK cntrl(%d) %s", clk_cntrl,
			gpio_get_value(drvdata->bt_gpio_dev_wake) ? "Assert" : "Deassert");
		break;
	case BT_CMD_SLIM_TEST:
#if IS_ENABLED(CONFIG_BT_SLIM_QCA6390) || \
	IS_ENABLED(CONFIG_BT_SLIM_QCA6490) || \
	IS_ENABLED(CONFIG_BTFM_SLIM_WCN3990) || \
	IS_ENABLED(CONFIG_BTFM_SLIM_WCN7850)
		if (!drvdata->slim_dev) {
			LOGE(drvdata, "slim_dev is null");
			return -EINVAL;
		}
		ret = btfm_slim_hw_init(drvdata->slim_dev->platform_data);
#endif
		break;
	case BT_CMD_PWR_CTRL:
		pwr_cntrl = (enum bt_power_modes)arg;
		if (drvdata->pwr_state == pwr_cntrl) {
			LOGW(drvdata, "BT_CMD_PWR_CTRL state(%d) already",
				drvdata->pwr_state);
			ret = 0;
			break;
		}
		LOGI(drvdata, "BT_CMD_PWR_CTRL pwr_cntrl: %d", pwr_cntrl);
		ret = bluetooth_power(drvdata, pwr_cntrl);
		break;
	case BT_CMD_CHIPSET_VERS:
		chipset_version = (int)arg;
		if (!chipset_version) {
			LOGE(drvdata, "got invalid soc version %x", chipset_version);
			drvdata->chipset_version = 0;
			break;
		}
		drvdata->chipset_version = chipset_version;
		LOGI(drvdata, "unified Current SOC Version : %x",
			drvdata->chipset_version);
		break;
	case BT_CMD_GET_CHIPSET_ID:
		if (copy_to_user((void __user *)arg,
				drvdata->compatible, MAX_PROP_SIZE)) {
			ret = -EFAULT;
		}
		break;
	case BT_CMD_CHECK_SW_CTRL:
		/* Check if SW_CTRL is asserted */
		LOGD(drvdata, "BT_CMD_CHECK_SW_CTRL");
		if (gpio_is_valid(drvdata->bt_gpio_sw_ctrl))
			return -EINVAL;
		SYNC_GPIO_SOURCE_CURRENT(drvdata, drvdata->bt_gpio_sw_ctrl,
			BT_SW_CTRL_GPIO);
		break;
	case BT_CMD_GETVAL_POWER_SRCS:
		LOGD(drvdata, "BT_CMD_GETVAL_POWER_SRCS");
		SYNC_GPIO_SOURCE_CURRENT(drvdata, drvdata->bt_gpio_sys_rst,
			BT_RESET_GPIO);
		SYNC_GPIO_SOURCE_CURRENT(drvdata, drvdata->bt_gpio_sw_ctrl,
			BT_SW_CTRL_GPIO);
		num_vregs = drvdata->num_vregs;
		for (itr = 0; itr < num_vregs; itr++) {
			vreg_info = &drvdata->vreg_info[itr];
			set_pwr_srcs_status(drvdata, vreg_info);
		}
		if (copy_to_user((void __user *)arg,
				 drvdata->bt_power_src_status,
				 sizeof(drvdata->bt_power_src_status))) {
			ret = -EFAULT;
		}
		break;
	case BT_CMD_SET_IPA_TCS_INFO:
		LOGD(drvdata, "BT_CMD_SET_IPA_TCS_INFO");
		btpower_enable_ipa_vreg(drvdata);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return ret;
}

static struct platform_driver bt_power_driver = {
	.probe = bt_power_probe,
	.remove = bt_power_remove,
	.driver = {
		.name = "bt_power",
		.of_match_table = bt_power_match_table,
	},
};
MODULE_DEVICE_TABLE(of, bt_power_match_table);

static int __init btpower_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&bt_power_driver);
	if (ret)
		pr_err("%s: platform_driver_register error: %d\n",
			__func__, ret);
	return ret;
}

int btpower_aop_mbox_init(struct btpower_platform_data *drvdata)
{
	struct mbox_client *mbox = &drvdata->mbox_client_data;
	struct mbox_chan *chan;
	int ret = 0;

	mbox->dev = &drvdata->pdev->dev;
	mbox->tx_block = true;
	mbox->tx_tout = BTPOWER_MBOX_TIMEOUT_MS;
	mbox->knows_txdone = false;

	drvdata->mbox_chan = NULL;
	chan = mbox_request_channel(mbox, 0);
	if (IS_ERR(chan)) {
		LOGE(drvdata, "failed to get mbox channel");
		return PTR_ERR(chan);
	}
	drvdata->mbox_chan = chan;

	ret = of_property_read_string(drvdata->pdev->dev.of_node,
		"qcom,vreg_ipa", &drvdata->vreg_ipa);
	if (ret) {
		LOGW(drvdata, "vreg for iPA not provided in device tree");
	} else {
		LOGD(drvdata, "Mbox channel initialized");
	}

	return 0;
}

static int btpower_aop_set_vreg_param(struct btpower_platform_data *drvdata,
		const char *vreg_name, enum btpower_vreg_param param,
		enum btpower_tcs_seq seq, int val)
{
	struct qmp_pkt pkt;
	char mbox_msg[BTPOWER_MBOX_MSG_MAX_LEN];
	int ret = 0;

	if (!vreg_name || param >= BTPOWER_VREG_PARAM_MAX ||
	    seq >= BTPOWER_TCS_SEQ_MAX)
		return -EINVAL;

	snprintf(mbox_msg, BTPOWER_MBOX_MSG_MAX_LEN,
		 "{class: wlan_pdc, res: %s.%c, %s: %d}", vreg_name,
		 vreg_param_str[param], tcs_seq_str[seq], val);
	LOGD(drvdata, "sending AOP Mbox msg: %s", mbox_msg);
	pkt.size = BTPOWER_MBOX_MSG_MAX_LEN;
	pkt.data = mbox_msg;
	ret = mbox_send_message(drvdata->mbox_chan, &pkt);
	if (ret < 0)
		LOGE(drvdata, "failed to send AOP mbox msg(%s) err(%d)", mbox_msg, ret);

	return ret;
}

static int btpower_enable_ipa_vreg(struct btpower_platform_data *drvdata)
{
	int ret = 0;

	if (drvdata->vreg_ipa_configured) {
		LOGD(drvdata, "IPA Vreg already configured");
		return 0;
	}

	if (!drvdata->vreg_ipa || !drvdata->mbox_chan) {
		LOGD(drvdata, "mbox/iPA vreg not specified");
		return ret;
	}

	ret = btpower_aop_set_vreg_param(drvdata, drvdata->vreg_ipa,
		BTPOWER_VREG_ENABLE, BTPOWER_TCS_UP_SEQ, 1);
	if (ret >= 0) {
		LOGD(drvdata, "Enabled iPA");
		drvdata->vreg_ipa_configured = true;
	}

	return ret;
}

static void __exit btpower_exit(void)
{
	platform_driver_unregister(&bt_power_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM Bluetooth power control driver");

module_init(btpower_init);
module_exit(btpower_exit);
