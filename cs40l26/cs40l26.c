// SPDX-License-Identifier: GPL-2.0
//
// cs40l26.c -- CS40L26 Boosted Haptic Driver with Integrated DSP and
// Waveform Memory with Advanced Closed Loop Algorithms and LRA protection
//
// Copyright 2022 Cirrus Logic, Inc.
//
// Author: Fred Treven <fred.treven@cirrus.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation.

#include "cs40l26.h"

static inline bool section_complete(struct cs40l26_owt_section *s)
{
	return s->delay ? true : false;
}

static u32 gpio_map_get(struct device_node *np, enum cs40l26_gpio_map gpio)
{
	const char *bank, *name = gpio == CS40L26_GPIO_MAP_A_PRESS ?
			"cirrus,press-index" : "cirrus,release-index";
	u32 val;

	if (!of_property_read_string_index(np, name, 0, &bank) &&
		!of_property_read_u32_index(np, name, 1, &val)) {
		if (!strncmp(bank, "RAM", 3))
			return (val & CS40L26_BTN_INDEX_MASK) |
				(1 << CS40L26_BTN_BANK_SHIFT);
		else if (!strncmp(bank, "ROM", 3))
			return val & CS40L26_BTN_INDEX_MASK;
	}

	return CS40L26_EVENT_MAP_GPI_DISABLE;
}

static int cs40l26_dsp_read(struct cs40l26_private *cs40l26, u32 reg, u32 *val)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret, i;
	u32 read_val;

	for (i = 0; i < CS40L26_DSP_TIMEOUT_COUNT; i++) {
		ret = regmap_read(regmap, reg, &read_val);
		if (ret)
			dev_dbg(dev, "Failed to read 0x%X, attempt(s) = %d\n",
					reg, i + 1);
		else
			break;

		usleep_range(CS40L26_DSP_TIMEOUT_US_MIN,
				CS40L26_DSP_TIMEOUT_US_MAX);
	}

	if (i >= CS40L26_DSP_TIMEOUT_COUNT) {
		dev_err(dev, "Timed out attempting to read 0x%X\n", reg);
		return -ETIME;
	}

	*val = read_val;

	return 0;
}

static int cs40l26_dsp_write(struct cs40l26_private *cs40l26, u32 reg, u32 val)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret, i;

	for (i = 0; i < CS40L26_DSP_TIMEOUT_COUNT; i++) {
		ret = regmap_write(regmap, reg, val);
		if (ret)
			dev_dbg(dev,
				"Failed to write to 0x%X, attempt(s) = %d\n",
				reg, i + 1);
		else
			break;

		usleep_range(CS40L26_DSP_TIMEOUT_US_MIN,
				CS40L26_DSP_TIMEOUT_US_MAX);
	}

	if (i >= CS40L26_DSP_TIMEOUT_COUNT) {
		dev_err(dev, "Timed out attempting to write to 0x%X\n", reg);
		return -ETIME;
	}

	return 0;
}

static int cs40l26_ack_read(struct cs40l26_private *cs40l26, u32 reg,
		u32 ack_val)
{
	struct device *dev = cs40l26->dev;
	int ret, i;
	u32 val;

	for (i = 0; i < CS40L26_DSP_TIMEOUT_COUNT; i++) {
		ret = cs40l26_dsp_read(cs40l26, reg, &val);
		if (ret)
			return ret;

		if (val == ack_val)
			break;

		usleep_range(CS40L26_DSP_TIMEOUT_US_MIN,
				CS40L26_DSP_TIMEOUT_US_MAX);
	}

	if (i >= CS40L26_DSP_TIMEOUT_COUNT) {
		dev_err(dev, "Ack timed out (0x%08X != 0x%08X) reg. 0x%08X\n",
				val, ack_val, reg);
		return -ETIME;
	}

	return 0;
}

int cs40l26_ack_write(struct cs40l26_private *cs40l26, u32 reg, u32 write_val,
		u32 reset_val)
{
	int ret;

	ret = cs40l26_dsp_write(cs40l26, reg, write_val);
	if (ret)
		return ret;

	return cs40l26_ack_read(cs40l26, reg, reset_val);
}
EXPORT_SYMBOL(cs40l26_ack_write);

int cs40l26_dsp_state_get(struct cs40l26_private *cs40l26, u8 *state)
{
	u32 reg, dsp_state;
	int ret = 0;

	if (cs40l26->fw_loaded)
		ret = cl_dsp_get_reg(cs40l26->dsp, "PM_CUR_STATE",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_PM_ALGO_ID, &reg);
	else
		reg = CS40L26_A1_PM_CUR_STATE_STATIC_REG;

	if (ret)
		return ret;

	ret = cs40l26_dsp_read(cs40l26, reg, &dsp_state);
	if (ret)
		return ret;

	switch (dsp_state) {
	case CS40L26_DSP_STATE_HIBERNATE:
		/* intentionally fall through */
	case CS40L26_DSP_STATE_SHUTDOWN:
		/* intentionally fall through */
	case CS40L26_DSP_STATE_STANDBY:
		/* intentionally fall through */
	case CS40L26_DSP_STATE_ACTIVE:
		*state = CS40L26_DSP_STATE_MASK & dsp_state;
		break;
	default:
		dev_err(cs40l26->dev, "DSP state %u is invalid\n", dsp_state);
		ret = -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL(cs40l26_dsp_state_get);

int cs40l26_set_pll_loop(struct cs40l26_private *cs40l26, unsigned int pll_loop)
{
	int ret, i;

	if (pll_loop != CS40L26_PLL_REFCLK_SET_OPEN_LOOP &&
			pll_loop != CS40L26_PLL_REFCLK_SET_CLOSED_LOOP) {
		dev_err(cs40l26->dev, "Invalid PLL Loop setting: %u\n",
				pll_loop);
		return -EINVAL;
	}

	/* Retry in case DSP is hibernating */
	for (i = 0; i < CS40L26_PLL_REFCLK_SET_ATTEMPTS; i++) {
		ret = regmap_update_bits(cs40l26->regmap, CS40L26_REFCLK_INPUT,
				CS40L26_PLL_REFCLK_LOOP_MASK, pll_loop <<
				CS40L26_PLL_REFCLK_LOOP_SHIFT);
		if (!ret)
			break;
	}

	if (i == CS40L26_PLL_REFCLK_SET_ATTEMPTS) {
		dev_err(cs40l26->dev, "Failed to configure PLL\n");
		return -ETIMEDOUT;
	}

	return 0;
}
EXPORT_SYMBOL(cs40l26_set_pll_loop);

int cs40l26_dbc_get(struct cs40l26_private *cs40l26, enum cs40l26_dbc dbc,
		unsigned int *val)
{
	struct device *dev = cs40l26->dev;
	unsigned int reg;
	int ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	mutex_lock(&cs40l26->lock);

	ret = cl_dsp_get_reg(cs40l26->dsp, cs40l26_dbc_names[dbc],
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_read(cs40l26->regmap, reg, val);
	if (ret)
		dev_err(dev, "Failed to read Dynamic Boost Control value\n");

err_pm:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(dev);

	return ret;
}
EXPORT_SYMBOL(cs40l26_dbc_get);

int cs40l26_dbc_set(struct cs40l26_private *cs40l26, enum cs40l26_dbc dbc,
		u32 val)
{
	struct device *dev = cs40l26->dev;
	unsigned int reg, max;
	int ret;

	if (dbc == CS40L26_DBC_TX_LVL_HOLD_OFF_MS)
		max = CS40L26_DBC_TX_LVL_HOLD_OFF_MS_MAX;
	else
		max = CS40L26_DBC_CONTROLS_MAX;

	if (val > max) {
		dev_err(dev, "DBC input %u out of bounds\n", val);
		return -EINVAL;
	}

	ret = cl_dsp_get_reg(cs40l26->dsp, cs40l26_dbc_names[dbc],
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = regmap_write(cs40l26->regmap, reg, val);
	if (ret)
		dev_err(dev, "Failed to write Dynamic Boost Control value\n");

	return ret;
}
EXPORT_SYMBOL(cs40l26_dbc_set);

static int cs40l26_pm_timeout_ticks_write(struct cs40l26_private *cs40l26,
		u32 ms, unsigned int lower_offset, unsigned int upper_offset)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	u32 lower_val, reg, ticks;
	u8 upper_val;
	int ret;

	if (ms > CS40L26_PM_TIMEOUT_MS_MAX) {
		dev_warn(dev, "Timeout (%u ms) invalid, using maximum\n", ms);
		ticks = CS40L26_PM_TIMEOUT_MS_MAX * CS40L26_PM_TICKS_MS_DIV;
	} else {
		ticks = ms * CS40L26_PM_TICKS_MS_DIV;
	}

	upper_val = (ticks >> CS40L26_PM_TIMEOUT_TICKS_UPPER_SHIFT) &
			CS40L26_PM_TIMEOUT_TICKS_UPPER_MASK;

	lower_val = ticks & CS40L26_PM_TIMEOUT_TICKS_LOWER_MASK;

	ret = cl_dsp_get_reg(cs40l26->dsp, "PM_TIMER_TIMEOUT_TICKS",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_PM_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = regmap_write(regmap, reg + lower_offset, lower_val);
	if (ret) {
		dev_err(dev, "Failed to write timeout ticks to 0x%08X\n",
				reg + lower_offset);
		return ret;
	}

	ret = regmap_write(regmap, reg + upper_offset, upper_val);
	if (ret)
		dev_err(dev, "Failed to write timeout ticks to 0x%08X\n",
				reg + upper_offset);

	return ret;
}

static int cs40l26_pm_timeout_ticks_read(struct cs40l26_private *cs40l26,
		unsigned int lower_offset, unsigned int upper_offset,
		u32 *ticks)
{
	u32 lower_val, upper_val, reg;
	int ret = 0;

	if (cs40l26->fw_loaded)
		ret = cl_dsp_get_reg(cs40l26->dsp, "PM_TIMER_TIMEOUT_TICKS",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_PM_ALGO_ID, &reg);
	else
		reg = CS40L26_A1_PM_TIMEOUT_TICKS_STATIC_REG;

	if (ret)
		return ret;

	ret = regmap_read(cs40l26->regmap, reg + lower_offset, &lower_val);
	if (ret)
		return ret;

	ret = regmap_read(cs40l26->regmap, reg + upper_offset, &upper_val);
	if (ret)
		return ret;

	*ticks = ((lower_val & CS40L26_PM_TIMEOUT_TICKS_LOWER_MASK) |
		((upper_val & CS40L26_PM_TIMEOUT_TICKS_UPPER_MASK) <<
		CS40L26_PM_TIMEOUT_TICKS_UPPER_SHIFT));

	return 0;
}

int cs40l26_pm_active_timeout_ms_set(struct cs40l26_private *cs40l26,
		u32 timeout_ms)
{
	return cs40l26_pm_timeout_ticks_write(cs40l26, timeout_ms,
			CS40L26_PM_ACTIVE_TIMEOUT_LOWER_OFFSET,
			CS40L26_PM_ACTIVE_TIMEOUT_UPPER_OFFSET);
}
EXPORT_SYMBOL(cs40l26_pm_active_timeout_ms_set);

int cs40l26_pm_active_timeout_ms_get(struct cs40l26_private *cs40l26,
		u32 *timeout_ms)
{
	u32 ticks;
	int ret;

	ret = cs40l26_pm_timeout_ticks_read(cs40l26,
			CS40L26_PM_ACTIVE_TIMEOUT_LOWER_OFFSET,
			CS40L26_PM_ACTIVE_TIMEOUT_UPPER_OFFSET, &ticks);
	if (ret)
		return ret;

	*timeout_ms = ticks / CS40L26_PM_TICKS_MS_DIV;

	return 0;
}
EXPORT_SYMBOL(cs40l26_pm_active_timeout_ms_get);

int cs40l26_pm_stdby_timeout_ms_set(struct cs40l26_private *cs40l26,
		u32 timeout_ms)
{
	return cs40l26_pm_timeout_ticks_write(cs40l26, timeout_ms,
			CS40L26_PM_STDBY_TIMEOUT_LOWER_OFFSET,
			CS40L26_PM_STDBY_TIMEOUT_UPPER_OFFSET);
}
EXPORT_SYMBOL(cs40l26_pm_stdby_timeout_ms_set);

int cs40l26_pm_stdby_timeout_ms_get(struct cs40l26_private *cs40l26,
		u32 *timeout_ms)
{
	u32 ticks;
	int ret;

	ret = cs40l26_pm_timeout_ticks_read(cs40l26,
			CS40L26_PM_STDBY_TIMEOUT_LOWER_OFFSET,
			CS40L26_PM_STDBY_TIMEOUT_UPPER_OFFSET, &ticks);
	if (ret)
		return ret;

	*timeout_ms = ticks / CS40L26_PM_TICKS_MS_DIV;

	return 0;
}
EXPORT_SYMBOL(cs40l26_pm_stdby_timeout_ms_get);

static void cs40l26_pm_runtime_setup(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;

	pm_runtime_mark_last_busy(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, CS40L26_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);

	cs40l26->pm_ready = true;
}

static void cs40l26_pm_runtime_teardown(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;

	pm_runtime_set_suspended(dev);
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);

	cs40l26->pm_ready = false;
}

static int cs40l26_check_pm_lock(struct cs40l26_private *cs40l26, bool *locked)
{
	int ret;
	unsigned int dsp_lock;

	ret = regmap_read(cs40l26->regmap, CS40L26_A1_PM_STATE_LOCKS_STATIC_REG
				+ CS40L26_DSP_LOCK3_OFFSET, &dsp_lock);
	if (ret)
		return ret;

	if (dsp_lock & CS40L26_DSP_LOCK3_MASK)
		*locked = true;
	else
		*locked = false;

	return 0;
}

static void cs40l26_remove_asp_scaling(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	u16 gain;

	if (cs40l26->pdata.asp_scale_pct >= CS40L26_GAIN_FULL_SCALE ||
			!cs40l26->scaling_applied)
		return;

	gain = cs40l26->gain_tmp;

	if (gain >= CS40L26_NUM_PCT_MAP_VALUES) {
		dev_err(dev, "Gain %u%% out of bounds\n", gain);
		return;
	}

	cs40l26->gain_pct = gain;
	cs40l26->scaling_applied = false;

	queue_work(cs40l26->vibe_workqueue, &cs40l26->set_gain_work);
}

int cs40l26_pm_state_transition(struct cs40l26_private *cs40l26,
		enum cs40l26_pm_state state)
{
	struct device *dev = cs40l26->dev;
	u32 cmd;
	u8 curr_state;
	bool dsp_lock;
	int ret, i;

	cmd = (u32) CS40L26_DSP_MBOX_PM_CMD_BASE + state;

	switch (state) {
	case CS40L26_PM_STATE_WAKEUP:
		ATRACE_BEGIN("CS40L26_PM_STATE_WAKEUP");
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				cmd, CS40L26_DSP_MBOX_RESET);
		if (ret)
			return ret;

		ATRACE_END();

		break;
	case CS40L26_PM_STATE_PREVENT_HIBERNATE:
		ATRACE_BEGIN("CS40L26_PM_STATE_PREVENT_HIBERNATE");
		for (i = 0; i < CS40L26_DSP_STATE_ATTEMPTS; i++) {
			ret = cs40l26_ack_write(cs40l26,
					CS40L26_DSP_VIRTUAL1_MBOX_1,
					cmd, CS40L26_DSP_MBOX_RESET);
			if (ret)
				return ret;

			ret = cs40l26_dsp_state_get(cs40l26, &curr_state);
			if (ret)
				return ret;

			if (curr_state == CS40L26_DSP_STATE_ACTIVE)
				break;

			if (curr_state == CS40L26_DSP_STATE_STANDBY) {
				ret = cs40l26_check_pm_lock(cs40l26, &dsp_lock);
				if (ret)
					return ret;

				if (dsp_lock)
					break;
			}
			usleep_range(5000, 5100);
		}

		if (i == CS40L26_DSP_STATE_ATTEMPTS) {
			dev_err(cs40l26->dev, "DSP not starting\n");
			return -ETIMEDOUT;
		}

		ATRACE_END();

		break;
	case CS40L26_PM_STATE_ALLOW_HIBERNATE:
		cs40l26->wksrc_sts = 0x00;
		ret = cs40l26_dsp_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				cmd);
		if (ret)
			return ret;

		break;
	case CS40L26_PM_STATE_SHUTDOWN:
		cs40l26->wksrc_sts = 0x00;
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
			cmd, CS40L26_DSP_MBOX_RESET);

		break;
	default:
		dev_err(dev, "Invalid PM state: %u\n", state);
		return -EINVAL;
	}

	cs40l26->pm_state = state;

	return 0;
}

static int cs40l26_dsp_start(struct cs40l26_private *cs40l26)
{
	u8 dsp_state;
	unsigned int val;
	int ret;

	ret = regmap_read(cs40l26->regmap, CS40L26_A1_DSP_REQ_ACTIVE_REG,
			&val);
	if (ret) {
		dev_err(cs40l26->dev, "Can't read REQ ACTIVE %d\n", ret);
		return ret;
	}

	if (val & CS40L26_DSP_PM_ACTIVE)
		dev_warn(cs40l26->dev, "REQ ACTIVE is 0x%x\n", val);

	ret = regmap_write(cs40l26->regmap, CS40L26_DSP1_CCM_CORE_CONTROL,
			CS40L26_DSP_CCM_CORE_RESET);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to reset DSP core\n");
		return ret;
	}

	ret = cs40l26_dsp_state_get(cs40l26, &dsp_state);
	if (ret)
		return ret;

	if (dsp_state != CS40L26_DSP_STATE_ACTIVE &&
			dsp_state != CS40L26_DSP_STATE_STANDBY) {
		dev_err(cs40l26->dev, "Failed to wake DSP core\n");
		return -EINVAL;
	}

	return 0;
}

static int cs40l26_dsp_pre_config(struct cs40l26_private *cs40l26)
{
	u32 halo_state, timeout_ms;
	u8 dsp_state;
	int ret, i;

	ret = cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_PREVENT_HIBERNATE);
	if (ret)
		return ret;

	ret = regmap_read(cs40l26->regmap, CS40L26_A1_DSP_HALO_STATE_REG,
			&halo_state);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get HALO state\n");
		return ret;
	}

	if (halo_state != CS40L26_DSP_HALO_STATE_RUN) {
		dev_err(cs40l26->dev, "DSP not Ready: HALO_STATE: %08X\n",
				halo_state);
		return -EINVAL;
	}

	ret = cs40l26_pm_active_timeout_ms_get(cs40l26, &timeout_ms);
	if (ret)
		return ret;

	for (i = 0; i < CS40L26_DSP_SHUTDOWN_MAX_ATTEMPTS; i++) {
		ret = cs40l26_dsp_state_get(cs40l26, &dsp_state);
		if (ret)
			return ret;

		if (dsp_state != CS40L26_DSP_STATE_SHUTDOWN &&
				dsp_state != CS40L26_DSP_STATE_STANDBY)
			dev_warn(cs40l26->dev, "DSP core not safe to kill\n");
		else
			break;

		usleep_range(CS40L26_MS_TO_US(timeout_ms),
			CS40L26_MS_TO_US(timeout_ms) + 100);
	}

	if (i == CS40L26_DSP_SHUTDOWN_MAX_ATTEMPTS) {
		dev_err(cs40l26->dev, "DSP Core could not be shut down\n");
		return -EINVAL;
	}

	ret = regmap_write(cs40l26->regmap, CS40L26_DSP1_CCM_CORE_CONTROL,
			CS40L26_DSP_CCM_CORE_KILL);
	if (ret)
		dev_err(cs40l26->dev, "Failed to kill DSP core\n");

	return ret;
}

static int cs40l26_mbox_buffer_read(struct cs40l26_private *cs40l26, u32 *val)
{
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;
	u32 base, last, len, write_ptr, read_ptr, mbox_response, reg;
	u32 buffer[CS40L26_DSP_MBOX_BUFFER_NUM_REGS];
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "QUEUE_BASE",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_MAILBOX_ALGO_ID,
			&reg);
	if (ret)
		return ret;

	ret = regmap_bulk_read(regmap, reg, buffer,
			CS40L26_DSP_MBOX_BUFFER_NUM_REGS);
	if (ret) {
		dev_err(dev, "Failed to read buffer contents\n");
		return ret;
	}

	base = buffer[0];
	len = buffer[1];
	write_ptr = buffer[2];
	read_ptr = buffer[3];
	last = base + ((len - 1) * CL_DSP_BYTES_PER_WORD);

	if ((read_ptr - CL_DSP_BYTES_PER_WORD) == write_ptr) {
		dev_err(dev, "Mailbox buffer is full, info missing\n");
		return -ENOSPC;
	}

	if (read_ptr == write_ptr) {
		dev_dbg(dev, "Reached end of queue\n");
		return 1;
	}

	ret = regmap_read(regmap, read_ptr, &mbox_response);
	if (ret) {
		dev_err(dev, "Failed to read from mailbox buffer\n");
		return ret;
	}

	if (read_ptr == last)
		read_ptr = base;
	else
		read_ptr += CL_DSP_BYTES_PER_WORD;

	ret = cl_dsp_get_reg(cs40l26->dsp, "QUEUE_RD",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_MAILBOX_ALGO_ID,
			&reg);
	if (ret)
		return ret;

	ret = regmap_write(regmap, reg, read_ptr);
	if (ret) {
		dev_err(dev, "Failed to update read pointer\n");
		return ret;
	}

	*val = mbox_response;

	return 0;
}

static int cs40l26_handle_mbox_buffer(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	u32 val = 0;
	int ret;

	while (!cs40l26_mbox_buffer_read(cs40l26, &val)) {
		if ((val & CS40L26_DSP_MBOX_CMD_INDEX_MASK)
				== CS40L26_DSP_MBOX_PANIC) {
			dev_err(dev, "DSP PANIC! Error condition: 0x%06X\n",
			(u32) (val & CS40L26_DSP_MBOX_CMD_PAYLOAD_MASK));
			return -ENOTRECOVERABLE;
		}

		if ((val & CS40L26_DSP_MBOX_CMD_INDEX_MASK) ==
				CS40L26_DSP_MBOX_WATERMARK) {
			dev_dbg(dev, "Mailbox: WATERMARK\n");
#ifdef CONFIG_DEBUG_FS
			ret = cl_dsp_logger_update(cs40l26->cl_dsp_db);
			if (ret)
				return ret;
#endif
			continue;
		}

		switch (val) {
		case CS40L26_DSP_MBOX_COMPLETE_MBOX:
			ATRACE_END();
			dev_dbg(dev, "Mailbox: COMPLETE_MBOX\n");
			complete_all(&cs40l26->erase_cont);
			cs40l26_vibe_state_update(cs40l26,
					CS40L26_VIBE_STATE_EVENT_MBOX_COMPLETE);
			break;
		case CS40L26_DSP_MBOX_COMPLETE_GPIO:
			ATRACE_END();
			dev_dbg(dev, "Mailbox: COMPLETE_GPIO\n");
			cs40l26_vibe_state_update(cs40l26,
					CS40L26_VIBE_STATE_EVENT_GPIO_COMPLETE);
			break;
		case CS40L26_DSP_MBOX_COMPLETE_I2S:
			ATRACE_END();
			dev_dbg(dev, "Mailbox: COMPLETE_I2S\n");
			/* ASP is interrupted */
			if (cs40l26->asp_enable)
				complete(&cs40l26->i2s_cont);
			break;
		case CS40L26_DSP_MBOX_TRIGGER_I2S:
			ATRACE_BEGIN("TRIGGER_I2S");
			dev_dbg(dev, "Mailbox: TRIGGER_I2S\n");
			complete(&cs40l26->i2s_cont);
			break;
		case CS40L26_DSP_MBOX_TRIGGER_CP:
			if (!cs40l26->vibe_state_reporting) {
				dev_err(dev, "vibe_state not supported\n");
				return -EPERM;
			}

			ATRACE_BEGIN("TRIGGER_CP");
			dev_dbg(dev, "Mailbox: TRIGGER_CP\n");
			cs40l26_vibe_state_update(cs40l26,
					CS40L26_VIBE_STATE_EVENT_MBOX_PLAYBACK);
			break;
		case CS40L26_DSP_MBOX_TRIGGER_GPIO:
			ATRACE_BEGIN("TRIGGER_GPIO");
			dev_dbg(dev, "Mailbox: TRIGGER_GPIO\n");
			cs40l26_vibe_state_update(cs40l26,
					CS40L26_VIBE_STATE_EVENT_GPIO_TRIGGER);
			break;
		case CS40L26_DSP_MBOX_PM_AWAKE:
			ATRACE_BEGIN("AWAKE");
			cs40l26->wksrc_sts |= CS40L26_WKSRC_STS_EN;
			ATRACE_END();
			dev_dbg(dev, "Mailbox: AWAKE\n");
			break;
		case CS40L26_DSP_MBOX_F0_EST_START:
			dev_dbg(dev, "Mailbox: F0_EST_START\n");
			break;
		case CS40L26_DSP_MBOX_F0_EST_DONE:
			dev_dbg(dev, "Mailbox: F0_EST_DONE\n");
			complete(&cs40l26->cal_f0_cont);
			break;
		case CS40L26_DSP_MBOX_REDC_EST_START:
			dev_dbg(dev, "Mailbox: REDC_EST_START\n");
			break;
		case CS40L26_DSP_MBOX_REDC_EST_DONE:
			dev_dbg(dev, "Mailbox: REDC_EST_DONE\n");
			complete(&cs40l26->cal_redc_cont);
			break;
		case CS40L26_DSP_MBOX_LE_EST_START:
			dev_dbg(dev, "Mailbox: LE_EST_START\n");
			break;
		case CS40L26_DSP_MBOX_LE_EST_DONE:
			dev_dbg(dev, "Mailbox: LE_EST_DONE\n");
			break;
		case CS40L26_DSP_MBOX_PEQ_CALCULATION_START:
			dev_dbg(dev, "Mailbox: PEQ_CALCULATION_START\n");
			break;
		case CS40L26_DSP_MBOX_PEQ_CALCULATION_DONE:
			dev_dbg(dev, "Mailbox: PEQ_CALCULATION_DONE\n");
			complete(&cs40l26->cal_dvl_peq_cont);
			break;
		case CS40L26_DSP_MBOX_SYS_ACK:
			dev_err(dev, "Mailbox: ACK\n");
			return -EPERM;
		default:
			dev_err(dev, "MBOX buffer value (0x%X) is invalid\n",
					val);
			return -EINVAL;
		}
	}

	return 0;
}

int cs40l26_copy_f0_est_to_dvl(struct cs40l26_private *cs40l26)
{
	u32 reg, f0_measured_q9_14, global_sample_rate, normalized_f0_q1_23;
	int ret, sample_rate;

	/* Must be awake and under mutex lock */
	ret = regmap_read(cs40l26->regmap, CS40L26_GLOBAL_SAMPLE_RATE,
							&global_sample_rate);
	if (ret)
		return ret;

	switch (global_sample_rate & CS40L26_GLOBAL_FS_MASK) {
	case CS40L26_GLOBAL_FS_48K:
		sample_rate = 48000;
		break;
	case CS40L26_GLOBAL_FS_96K:
		sample_rate = 96000;
		break;
	default:
		dev_warn(cs40l26->dev, "Invalid GLOBAL_FS, %08X",
							global_sample_rate);
		return -EINVAL;
	}

	ret = cl_dsp_get_reg(cs40l26->dsp, "F0_EST",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_F0_EST_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = regmap_read(cs40l26->regmap, reg, &f0_measured_q9_14);
	if (ret)
		return ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "LRA_NORM_F0",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_DVL_ALGO_ID, &reg);
	if (ret)
		return ret;

	normalized_f0_q1_23 = (f0_measured_q9_14 << 9) / sample_rate;
	ret = regmap_write(cs40l26->regmap, reg, normalized_f0_q1_23);

	return ret;
}
EXPORT_SYMBOL(cs40l26_copy_f0_est_to_dvl);

int cs40l26_asp_start(struct cs40l26_private *cs40l26)
{
	int ret;

	if (cs40l26->pdata.asp_scale_pct < CS40L26_GAIN_FULL_SCALE)
		queue_work(cs40l26->vibe_workqueue, &cs40l26->set_gain_work);

	ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_STOP_PLAYBACK, CS40L26_DSP_MBOX_RESET);
	if (ret) {
		dev_err(cs40l26->dev,
			"Failed to stop playback before I2S start\n");
		return ret;
	}

	reinit_completion(&cs40l26->i2s_cont);

	return cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
			CS40L26_DSP_MBOX_CMD_START_I2S, CS40L26_DSP_MBOX_RESET);
}
EXPORT_SYMBOL(cs40l26_asp_start);

void cs40l26_vibe_state_update(struct cs40l26_private *cs40l26,
		enum cs40l26_vibe_state_event event)
{
	if (!mutex_is_locked(&cs40l26->lock)) {
		dev_err(cs40l26->dev, "%s must be called under mutex lock\n",
				__func__);
		return;
	}

	dev_dbg(cs40l26->dev, "effects_in_flight = %d\n",
			cs40l26->effects_in_flight);

	switch (event) {
	case CS40L26_VIBE_STATE_EVENT_MBOX_PLAYBACK:
	case CS40L26_VIBE_STATE_EVENT_GPIO_TRIGGER:
		cs40l26_remove_asp_scaling(cs40l26);
		cs40l26->effects_in_flight = cs40l26->effects_in_flight <= 0 ? 1 :
			cs40l26->effects_in_flight + 1;
		break;
	case CS40L26_VIBE_STATE_EVENT_MBOX_COMPLETE:
	case CS40L26_VIBE_STATE_EVENT_GPIO_COMPLETE:
		cs40l26->effects_in_flight = cs40l26->effects_in_flight <= 0 ? 0 :
			cs40l26->effects_in_flight - 1;
		if (cs40l26->effects_in_flight == 0 && cs40l26->asp_enable)
			if (cs40l26_asp_start(cs40l26))
				return;
		break;
	case CS40L26_VIBE_STATE_EVENT_ASP_START:
		cs40l26->asp_enable = true;
		break;
	case CS40L26_VIBE_STATE_EVENT_ASP_STOP:
		cs40l26_remove_asp_scaling(cs40l26);
		cs40l26->asp_enable = false;
		break;
	default:
		dev_err(cs40l26->dev, "Invalid vibe state event: %d\n", event);
		break;
	}

	if (cs40l26->effects_in_flight)
		cs40l26->vibe_state = CS40L26_VIBE_STATE_HAPTIC;
	else if (cs40l26->asp_enable)
		cs40l26->vibe_state = CS40L26_VIBE_STATE_ASP;
	else
		cs40l26->vibe_state = CS40L26_VIBE_STATE_STOPPED;

#if !IS_ENABLED(CONFIG_INPUT_CS40L26_ATTR_UNDER_BUS)
	sysfs_notify(&cs40l26->input->dev.kobj, "default", "vibe_state");
#else
	sysfs_notify(&cs40l26->dev->kobj, "default", "vibe_state");
#endif
}
EXPORT_SYMBOL(cs40l26_vibe_state_update);

static int cs40l26_error_release(struct cs40l26_private *cs40l26,
		unsigned int err_rls)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	u32 err_sts, err_cfg;
	int ret;

	ret = regmap_read(regmap, CS40L26_ERROR_RELEASE, &err_sts);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get error status\n");
		return ret;
	}

	err_cfg = err_sts & ~BIT(err_rls);

	ret = regmap_write(cs40l26->regmap, CS40L26_ERROR_RELEASE, err_cfg);
	if (ret) {
		dev_err(dev, "Actuator Safe Mode release sequence failed\n");
		return ret;
	}

	err_cfg |= BIT(err_rls);

	ret = regmap_write(regmap, CS40L26_ERROR_RELEASE, err_cfg);
	if (ret) {
		dev_err(dev, "Actuator Safe Mode release sequence failed\n");
		return ret;
	}

	err_cfg &= ~BIT(err_rls);

	ret = regmap_write(cs40l26->regmap, CS40L26_ERROR_RELEASE, err_cfg);
	if (ret)
		dev_err(dev, "Actuator Safe Mode release sequence failed\n");

	return ret;
}

static int cs40l26_handle_irq1(struct cs40l26_private *cs40l26,
		enum cs40l26_irq1 irq1)
{
	struct device *dev = cs40l26->dev;
	u32 err_rls = 0;
	int ret = 0;
	unsigned int reg, val;

	switch (irq1) {
	case CS40L26_IRQ1_GPIO1_RISE:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO1_FALL:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO2_RISE:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO2_FALL:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO3_RISE:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO3_FALL:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO4_RISE:
	/* intentionally fall through */
	case CS40L26_IRQ1_GPIO4_FALL:
		if (cs40l26->wksrc_sts & CS40L26_WKSRC_STS_EN) {
			dev_dbg(dev, "GPIO%u %s edge detected\n",
					(irq1 / 2) + 1,
					(irq1 % 2) ? "falling" : "rising");
		}

		cs40l26->wksrc_sts |= CS40L26_WKSRC_STS_EN;
		break;
	case CS40L26_IRQ1_WKSRC_STS_ANY:
		dev_dbg(dev, "Wakesource detected (ANY)\n");

		ret = regmap_read(cs40l26->regmap, CS40L26_PWRMGT_STS, &val);
		if (ret) {
			dev_err(dev, "Failed to get Power Management Status\n");
			goto err;
		}

		cs40l26->wksrc_sts = (u8) ((val & CS40L26_WKSRC_STS_MASK) >>
				CS40L26_WKSRC_STS_SHIFT);

		ret = cl_dsp_get_reg(cs40l26->dsp, "LAST_WAKESRC_CTL",
				CL_DSP_XM_UNPACKED_TYPE, cs40l26->fw_id, &reg);
		if (ret)
			goto err;

		ret = regmap_read(cs40l26->regmap, reg, &val);
		if (ret) {
			dev_err(dev, "Failed to read LAST_WAKESRC_CTL\n");
			goto err;
		}
		cs40l26->last_wksrc_pol =
				(u8) (val & CS40L26_WKSRC_GPIO_POL_MASK);
		break;
	case CS40L26_IRQ1_WKSRC_STS_GPIO1:
	/* intentionally fall through */
	case CS40L26_IRQ1_WKSRC_STS_GPIO2:
	/* intentionally fall through */
	case CS40L26_IRQ1_WKSRC_STS_GPIO3:
	/* intentionally fall through */
	case CS40L26_IRQ1_WKSRC_STS_GPIO4:
		dev_dbg(dev, "GPIO%u event woke device from hibernate\n",
				irq1 - CS40L26_IRQ1_WKSRC_STS_GPIO1 + 1);

		if (cs40l26->wksrc_sts & cs40l26->last_wksrc_pol) {
			dev_dbg(dev, "GPIO%u falling edge detected\n",
					irq1 - 8);
			cs40l26->wksrc_sts |= CS40L26_WKSRC_STS_EN;
		} else {
			dev_dbg(dev, "GPIO%u rising edge detected\n",
					irq1 - 8);
		}
		break;
	case CS40L26_IRQ1_WKSRC_STS_SPI:
		dev_dbg(dev, "SPI event woke device from hibernate\n");
		break;
	case CS40L26_IRQ1_WKSRC_STS_I2C:
		dev_dbg(dev, "I2C event woke device from hibernate\n");
		break;
	case CS40L26_IRQ1_GLOBAL_EN_ASSERT:
		dev_dbg(dev, "Started power up seq. (GLOBAL_EN asserted)\n");
		break;
	case CS40L26_IRQ1_PDN_DONE:
		dev_dbg(dev,
			"Completed power down seq. (GLOBAL_EN cleared)\n");
		break;
	case CS40L26_IRQ1_PUP_DONE:
		dev_dbg(dev,
			"Completed power up seq. (GLOBAL_EN asserted)\n");
		break;
	case CS40L26_IRQ1_BST_OVP_FLAG_RISE:
		dev_warn(dev, "BST overvoltage warning\n");
		break;
	case CS40L26_IRQ1_BST_OVP_FLAG_FALL:
		dev_warn(dev,
			"BST voltage returned below warning threshold\n");
		break;
	case CS40L26_IRQ1_BST_OVP_ERR:
		dev_err(dev, "BST overvolt. error\n");
		err_rls = CS40L26_BST_OVP_ERR_RLS;
		break;
	case CS40L26_IRQ1_BST_DCM_UVP_ERR:
		dev_err(dev, "BST undervolt. error\n");
		err_rls = CS40L26_BST_UVP_ERR_RLS;
		break;
	case CS40L26_IRQ1_BST_SHORT_ERR:
		dev_err(dev, "LBST short detected\n");
		err_rls = CS40L26_BST_SHORT_ERR_RLS;
		break;
	case CS40L26_IRQ1_BST_IPK_FLAG:
		dev_dbg(dev, "Current is being limited by LBST inductor\n");
		break;
	case CS40L26_IRQ1_TEMP_WARN_RISE:
		dev_err(dev, "Die overtemperature warning\n");
		err_rls = CS40L26_TEMP_WARN_ERR_RLS;
		break;
	case CS40L26_IRQ1_TEMP_WARN_FALL:
		dev_warn(dev, "Die temperature returned below threshold\n");
		break;
	case CS40L26_IRQ1_TEMP_ERR:
		dev_err(dev,
			"Die overtemperature error\n");
		err_rls = CS40L26_TEMP_ERR_RLS;
		break;
	case CS40L26_IRQ1_AMP_ERR:
		dev_err(dev, "AMP short detected\n");
		err_rls = CS40L26_AMP_SHORT_ERR_RLS;
		break;
	case CS40L26_IRQ1_DC_WATCHDOG_RISE:
		dev_err(dev, "DC level detected\n");
		break;
	case CS40L26_IRQ1_DC_WATCHDOG_FALL:
		dev_warn(dev, "Previously detected DC level removed\n");
		break;
	case CS40L26_IRQ1_VIRTUAL1_MBOX_WR:
		dev_dbg(dev, "Virtual 1 MBOX write occurred\n");
		break;
	case CS40L26_IRQ1_VIRTUAL2_MBOX_WR:
		ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_EINT_1, BIT(irq1));
		if (ret) {
			dev_err(dev, "Failed to clear Mailbox IRQ\n");
			goto err;
		}

		return cs40l26_handle_mbox_buffer(cs40l26);
	default:
		dev_err(dev, "Unrecognized IRQ1 EINT1 status\n");
		return -EINVAL;
	}

	if (err_rls)
		ret = cs40l26_error_release(cs40l26, err_rls);

err:
	regmap_write(cs40l26->regmap, CS40L26_IRQ1_EINT_1, BIT(irq1));

	return ret;
}

static int cs40l26_handle_irq2(struct cs40l26_private *cs40l26,
		enum cs40l26_irq2 irq2)
{
	struct device *dev = cs40l26->dev;
	unsigned int val;
	u32 vbbr_status, vpbr_status;
	int ret;

	switch (irq2) {
	case CS40L26_IRQ2_PLL_LOCK:
		dev_dbg(dev, "PLL achieved lock\n");
		break;
	case CS40L26_IRQ2_PLL_PHASE_LOCK:
		dev_dbg(dev, "PLL achieved phase lock\n");
		break;
	case CS40L26_IRQ2_PLL_FREQ_LOCK:
		dev_dbg(dev, "PLL achieved frequency lock\n");
		break;
	case CS40L26_IRQ2_PLL_UNLOCK_RISE:
		dev_err(dev, "PLL has lost lock\n");
		break;
	case CS40L26_IRQ2_PLL_UNLOCK_FALL:
		dev_warn(dev, "PLL has regained lock\n");
		break;
	case CS40L26_IRQ2_PLL_READY:
		dev_dbg(dev, "PLL ready for use\n");
		break;
	case CS40L26_IRQ2_PLL_REFCLK_PRESENT:
		dev_warn(dev, "REFCLK present for PLL\n");
		break;
	case CS40L26_IRQ2_REFCLK_MISSING_RISE:
		dev_err(dev, "REFCLK input for PLL is missing\n");
		break;
	case CS40L26_IRQ2_REFCLK_MISSING_FALL:
		dev_warn(dev, "REFCLK reported missing is now present\n");
		break;
	case CS40L26_IRQ2_ASP_RXSLOT_CFG_ERR:
		dev_err(dev, "Misconfig. of ASP_RX 1 2 or 3 SLOT fields\n");
			break;
	case CS40L26_IRQ2_AUX_NG_CH1_ENTRY:
		dev_warn(dev,
			"CH1 data of noise gate has fallen below threshold\n");
		break;
	case CS40L26_IRQ2_AUX_NG_CH1_EXIT:
		dev_err(dev,
			"CH1 data of noise gate has risen above threshold\n");
		break;
	case CS40L26_IRQ2_AUX_NG_CH2_ENTRY:
		dev_warn(dev,
			"CH2 data of noise gate has fallen below threshold\n");
		break;
	case CS40L26_IRQ2_AUX_NG_CH2_EXIT:
		dev_err(dev,
			"CH2 data of noise gate has risen above threshold\n");
		break;
	case CS40L26_IRQ2_AMP_NG_ON_RISE:
		dev_warn(dev, "Amplifier entered noise-gated state\n");
		break;
	case CS40L26_IRQ2_AMP_NG_ON_FALL:
		dev_warn(dev, "Amplifier exited noise-gated state\n");
		break;
	case CS40L26_IRQ2_VPBR_FLAG:
		dev_err(dev,
			"VP voltage has dropped below brownout threshold\n");
		ret = regmap_read(cs40l26->regmap, CS40L26_VPBR_STATUS, &val);
		if (ret) {
			dev_err(dev, "Failed to get VPBR_STATUS\n");
			return ret;
		}

		vpbr_status = (val & CS40L26_VXBR_STATUS_MASK);
		dev_err(dev, "VPBR Attenuation applied = %u x 10^-4 dB\n",
				vpbr_status * CS40L26_VXBR_STATUS_DIV_STEP);
		break;
	case CS40L26_IRQ2_VPBR_ATT_CLR:
		dev_warn(dev,
			"Cleared attenuation applied by VP brownout event\n");
		break;
	case CS40L26_IRQ2_VBBR_FLAG:
		dev_err(dev,
			"VBST voltage has dropped below brownout threshold\n");
		ret = regmap_read(cs40l26->regmap, CS40L26_VBBR_STATUS, &val);
		if (ret) {
			dev_err(dev, "Failed to get VPBR_STATUS\n");
			return ret;
		}

		vbbr_status = (val & CS40L26_VXBR_STATUS_MASK);
		dev_err(dev, "VBBR Attenuation applied = %u x 10^-4 dB\n",
				vbbr_status * CS40L26_VXBR_STATUS_DIV_STEP);
		break;
	case CS40L26_IRQ2_VBBR_ATT_CLR:
		dev_warn(dev, "Cleared attenuation caused by VBST brownout\n");
		break;
	case CS40L26_IRQ2_I2C_NACK_ERR:
		dev_err(dev, "I2C interface NACK during Broadcast Mode\n");
		break;
	case CS40L26_IRQ2_VPMON_CLIPPED:
		dev_err(dev, "Input larger than full-scale value (VPMON)\n");
		break;
	case CS40L26_IRQ2_VBSTMON_CLIPPED:
		dev_err(dev, "Input larger than full-scale value (VBSTMON)\n");
		break;
	case CS40L26_IRQ2_VMON_CLIPPED:
		dev_err(dev, "Input larger than full-scale value (VMON)\n");
		break;
	case CS40L26_IRQ2_IMON_CLIPPED:
		dev_err(dev, "Input larger than full-scale value (IMON)\n");
		break;
	default:
		dev_err(dev, "Unrecognized IRQ1 EINT2 status\n");
		return -EINVAL;
	}

	/* write 1 to clear the interrupt flag */
	ret = regmap_write(cs40l26->regmap, CS40L26_IRQ1_EINT_2, BIT(irq2));
	if (ret)
		dev_err(dev, "Failed to clear IRQ1 EINT2 %u\n", irq2);

	return ret;
}

static irqreturn_t cs40l26_irq(int irq, void *data)
{
	struct cs40l26_private *cs40l26 = (struct cs40l26_private *)data;
	unsigned int sts, val, eint, mask, i, irq1_count = 0, irq2_count = 0;
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned long num_irq;
	int ret;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(dev, ret);

		dev_err(dev, "Interrupts missed\n");

		cs40l26_dsp_write(cs40l26, CS40L26_IRQ1_EINT_1,
				CS40L26_IRQ_EINT1_ALL_MASK);
		cs40l26_dsp_write(cs40l26, CS40L26_IRQ1_EINT_2,
				CS40L26_IRQ_EINT2_ALL_MASK);

		return IRQ_NONE;
	}

	mutex_lock(&cs40l26->lock);

	if (regmap_read(regmap, CS40L26_IRQ1_STATUS, &sts)) {
		dev_err(dev, "Failed to read IRQ1 Status\n");
		ret = IRQ_NONE;
		goto err;
	}

	if (sts != CS40L26_IRQ_STATUS_ASSERT) {
		dev_err(dev, "IRQ1 asserted with no pending interrupts\n");
		ret = IRQ_NONE;
		goto err;
	}

	ret = regmap_read(regmap, CS40L26_IRQ1_EINT_1, &eint);
	if (ret) {
		dev_err(dev, "Failed to read interrupts status 1\n");
		goto err;
	}

	ret = regmap_read(regmap, CS40L26_IRQ1_MASK_1, &mask);
	if (ret) {
		dev_err(dev, "Failed to get interrupts mask 1\n");
		goto err;
	}

	val = eint & ~mask;
	if (val) {
		num_irq = hweight_long(val);
		i = 0;
		while (irq1_count < num_irq && i < CS40L26_IRQ1_NUM_IRQS) {
			if (val & BIT(i)) {
				ret = cs40l26_handle_irq1(cs40l26, i);
				if (ret)
					goto err;
				else
					irq1_count++;
			}
			i++;
		}
	}

	ret = regmap_read(regmap, CS40L26_IRQ1_EINT_2, &eint);
	if (ret) {
		dev_err(dev, "Failed to read interrupts status 2\n");
		goto err;
	}

	ret = regmap_read(regmap, CS40L26_IRQ1_MASK_2, &mask);
	if (ret) {
		dev_err(dev, "Failed to get interrupts mask 2\n");
		goto err;
	}

	val = eint & ~mask;
	if (val) {
		num_irq = hweight_long(val);

		i = 0;
		while (irq2_count < num_irq && i < CS40L26_IRQ2_NUM_IRQS) {
			if (val & BIT(i)) {
				ret = cs40l26_handle_irq2(cs40l26, i);
				if (ret)
					goto err;
				else
					irq2_count++;
			}
			i++;
		}
	}

err:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(dev);

	/* if an error has occurred, all IRQs have not been successfully
	 * processed; however, IRQ_HANDLED is still returned if at least one
	 * interrupt request generated by CS40L26 was handled successfully.
	 */
	if (ret)
		dev_err(dev, "Failed to process IRQ (%d): %u\n", irq, ret);

	return (irq1_count + irq2_count) ? IRQ_HANDLED : IRQ_NONE;
}

static int cs40l26_pseq_find_end(struct cs40l26_private *cs40l26,
		struct cs40l26_pseq_op **op_end)
{
	struct cs40l26_pseq_op *op;

	list_for_each_entry(op, &cs40l26->pseq_op_head, list) {
		if (op->operation == CS40L26_PSEQ_OP_END)
			break;
	}

	if (op->operation != CS40L26_PSEQ_OP_END) {
		dev_err(cs40l26->dev, "Failed to find PSEQ list terminator\n");
		return -ENOENT;
	}

	*op_end = op;

	return 0;
}

int cs40l26_pseq_write(struct cs40l26_private *cs40l26, u32 addr,
		u32 data, bool update, u8 op_code)
{
	struct device *dev = cs40l26->dev;
	bool is_new = true;
	unsigned int l_addr_mask, u_addr_mask, u_data_mask, l_data_mask;
	unsigned int l_addr_shift, u_addr_shift, u_data_shift;
	struct cs40l26_pseq_op *op, *op_new, *op_end;
	unsigned int op_mask;
	int num_op_words;
	u32 *op_words;
	int ret;


	/*
	 * Due to a bug in the DSP ROM, if data[23] = 1 for a WRITE_FULL
	 * operation, then, when the DSP power-on write sequencer
	 * actually applies these writes when coming out of hibernate,
	 * the DSP sign-extends bit 23 to bits[31:24]. So, warn if it
	 * appears the PSEQ will not function as expected.
	 */
	if ((op_code == CS40L26_PSEQ_OP_WRITE_FULL) &&
				(data & BIT(23)) &&
				(((data & GENMASK(31, 24)) >> 24) != 0xFF)) {
		dev_warn(dev,
			"PSEQ to set data[31:24] to 0xFF reg: %08X, data: %08X",
								addr, data);
	}

	if (op_code == CS40L26_PSEQ_OP_WRITE_FULL) {
		num_op_words = CS40L26_PSEQ_OP_WRITE_FULL_WORDS;
		l_addr_shift = CS40L26_PSEQ_WRITE_FULL_LOWER_ADDR_SHIFT;
		l_addr_mask = CS40L26_PSEQ_WRITE_FULL_LOWER_ADDR_MASK;
		u_addr_shift = CS40L26_PSEQ_WRITE_FULL_UPPER_ADDR_SHIFT;
		u_addr_mask = CS40L26_PSEQ_WRITE_FULL_UPPER_ADDR_MASK;
		u_data_shift = CS40L26_PSEQ_WRITE_FULL_UPPER_DATA_SHIFT;
		u_data_mask = CS40L26_PSEQ_WRITE_FULL_UPPER_DATA_MASK;
		l_data_mask = CS40L26_PSEQ_WRITE_FULL_LOWER_DATA_MASK;
		op_mask = CS40L26_PSEQ_WRITE_FULL_OP_MASK;
	} else if (op_code == CS40L26_PSEQ_OP_WRITE_H16 ||
			op_code == CS40L26_PSEQ_OP_WRITE_L16) {
		if (addr & CS40L26_PSEQ_INVALID_ADDR) {
			dev_err(dev, "Invalid PSEQ address: 0x%08X\n", addr);
			return -EINVAL;
		}

		num_op_words = CS40L26_PSEQ_OP_WRITE_X16_WORDS;
		l_addr_shift = CS40L26_PSEQ_WRITE_X16_LOWER_ADDR_SHIFT;
		l_addr_mask = CS40L26_PSEQ_WRITE_X16_LOWER_ADDR_MASK;
		u_addr_shift = CS40L26_PSEQ_WRITE_X16_UPPER_ADDR_SHIFT;
		u_addr_mask = CS40L26_PSEQ_WRITE_X16_UPPER_ADDR_MASK;
		u_data_shift = CS40L26_PSEQ_WRITE_X16_UPPER_DATA_SHIFT;
		u_data_mask = CS40L26_PSEQ_WRITE_X16_UPPER_DATA_MASK;
		op_mask = CS40L26_PSEQ_WRITE_X16_OP_MASK;
	} else {
		dev_err(dev, "Invalid PSEQ OP code: 0x%02X\n", op_code);
		return -EINVAL;
	}

	op_words = devm_kcalloc(dev, num_op_words, sizeof(u32), GFP_KERNEL);
	if (!op_words)
		return -ENOMEM;

	op_words[0] = (op_code << CS40L26_PSEQ_OP_SHIFT);
	op_words[0] |= (addr & u_addr_mask) >> u_addr_shift;
	op_words[1] = (addr & l_addr_mask) << l_addr_shift;
	op_words[1] |= (data & u_data_mask) >> u_data_shift;
	if (op_code == CS40L26_PSEQ_OP_WRITE_FULL)
		op_words[2] = data & l_data_mask;

	list_for_each_entry(op, &cs40l26->pseq_op_head, list) {
		if (op->words[0] == op_words[0] && (op->words[1] & op_mask) ==
				(op_words[1] & op_mask) && update) {
			if (op->size != num_op_words) {
				dev_err(dev, "Failed to replace PSEQ op.\n");
				ret = -EINVAL;
				goto op_words_free;
			}
			is_new = false;
			break;
		}
	}

	op_new = devm_kzalloc(dev, sizeof(*op_new), GFP_KERNEL);
	if (!op_new) {
		ret = -ENOMEM;
		goto op_words_free;
	}
	op_new->size = num_op_words;
	op_new->words = op_words;
	op_new->operation = op_code;

	ret = cs40l26_pseq_find_end(cs40l26, &op_end);
	if (ret)
		goto op_new_free;

	if (((CS40L26_PSEQ_MAX_WORDS * CL_DSP_BYTES_PER_WORD) - op_end->offset)
				< (op_new->size * CL_DSP_BYTES_PER_WORD)) {
		dev_err(dev, "Not enough space in pseq to add op\n");
		ret = -ENOMEM;
		goto op_new_free;
	}

	if (is_new) {
		op_new->offset = op_end->offset;
		op_end->offset += (num_op_words * CL_DSP_BYTES_PER_WORD);
	} else {
		op_new->offset = op->offset;
	}

	ret = regmap_bulk_write(cs40l26->regmap, cs40l26->pseq_base +
			op_new->offset, op_new->words, op_new->size);
	if (ret) {
		dev_err(dev, "Failed to write PSEQ op.\n");
		goto op_new_free;
	}

	if (is_new) {
		ret = regmap_bulk_write(cs40l26->regmap,
				cs40l26->pseq_base + op_end->offset,
				op_end->words, op_end->size);
		if (ret) {
			dev_err(dev, "Failed to write PSEQ terminator\n");
			goto op_new_free;
		}

		list_add(&op_new->list, &cs40l26->pseq_op_head);
		cs40l26->pseq_num_ops++;
	} else {
		list_replace(&op->list, &op_new->list);
	}

	return 0;

op_new_free:
	devm_kfree(dev, op_new);

op_words_free:
	devm_kfree(dev, op_words);

	return ret;
}
EXPORT_SYMBOL(cs40l26_pseq_write);

static int cs40l26_pseq_multi_write(struct cs40l26_private *cs40l26,
		const struct reg_sequence *reg_seq, int num_regs, bool update,
		u8 op_code)
{
	int ret, i;

	for (i = 0; i < num_regs; i++) {
		ret = cs40l26_pseq_write(cs40l26, reg_seq[i].reg,
				reg_seq[i].def, update, op_code);
		if (ret)
			return ret;
	}

	return 0;
}

static int cs40l26_pseq_init(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	u32 words[CS40L26_PSEQ_MAX_WORDS], *op_words;
	struct cs40l26_pseq_op *pseq_op;
	int ret, i, j, num_words, read_size_words;
	u8 operation;

	INIT_LIST_HEAD(&cs40l26->pseq_op_head);
	cs40l26->pseq_num_ops = 0;

	ret = cl_dsp_get_reg(cs40l26->dsp, "POWER_ON_SEQUENCE",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_PM_ALGO_ID,
			&cs40l26->pseq_base);
	if (ret)
		return ret;

	/* read pseq memory space */
	i = 0;
	while (i < CS40L26_PSEQ_MAX_WORDS) {
		if ((CS40L26_PSEQ_MAX_WORDS - i) >
				CS40L26_MAX_I2C_READ_SIZE_WORDS)
			read_size_words = CS40L26_MAX_I2C_READ_SIZE_WORDS;
		else
			read_size_words = CS40L26_PSEQ_MAX_WORDS - i;

		ret = regmap_bulk_read(cs40l26->regmap,
			cs40l26->pseq_base + i * CL_DSP_BYTES_PER_WORD,
			words + i, read_size_words);
		if (ret) {
			dev_err(dev, "Failed to read from power on seq.\n");
			return ret;
		}
		i += read_size_words;
	}

	i = 0;
	while (i < CS40L26_PSEQ_MAX_WORDS) {
		operation = (words[i] & CS40L26_PSEQ_OP_MASK) >>
			CS40L26_PSEQ_OP_SHIFT;

		/* get num words for given operation */
		for (j = 0; j < CS40L26_PSEQ_NUM_OPS; j++) {
			if (cs40l26_pseq_op_sizes[j][0] == operation) {
				num_words = cs40l26_pseq_op_sizes[j][1];
				break;
			}
		}

		if (j == CS40L26_PSEQ_NUM_OPS) {
			dev_err(dev, "Failed to determine pseq op size\n");
			return -EINVAL;
		}

		op_words = kzalloc(num_words * CL_DSP_BYTES_PER_WORD,
				GFP_KERNEL);
		if (!op_words)
			return -ENOMEM;
		memcpy(op_words, &words[i], num_words * CL_DSP_BYTES_PER_WORD);

		pseq_op = devm_kzalloc(dev, sizeof(*pseq_op), GFP_KERNEL);
		if (!pseq_op) {
			ret = -ENOMEM;
			goto err_free;
		}

		pseq_op->size = num_words;
		pseq_op->offset = i * CL_DSP_BYTES_PER_WORD;
		pseq_op->operation = operation;
		pseq_op->words = op_words;
		list_add(&pseq_op->list, &cs40l26->pseq_op_head);

		cs40l26->pseq_num_ops++;
		i += num_words;

		if (operation == CS40L26_PSEQ_OP_END)
			break;

	}

	if (operation != CS40L26_PSEQ_OP_END) {
		dev_err(dev, "PSEQ END_OF_SCRIPT not found\n");
		return -E2BIG;
	}

	return ret;

err_free:
	kfree(op_words);

	return ret;
}

static int cs40l26_update_reg_defaults_via_pseq(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	int ret;

	ret = cs40l26_pseq_write(cs40l26, CS40L26_NGATE1_INPUT,
			CS40L26_DATA_SRC_DSP1TX4, true,
			CS40L26_PSEQ_OP_WRITE_L16);
	if (ret)
		return ret;

	ret = cs40l26_pseq_write(cs40l26, CS40L26_MIXER_NGATE_CH1_CFG,
			CS40L26_MIXER_NGATE_CH1_CFG_DEFAULT_NEW, true,
			CS40L26_PSEQ_OP_WRITE_FULL);
	if (ret) {
		dev_err(dev, "Failed to sequence Mixer Noise Gate\n");
		return ret;
	}

	/* set SPK_DEFAULT_HIZ to 1 */
	ret = cs40l26_pseq_write(cs40l26, CS40L26_TST_DAC_MSM_CONFIG,
			CS40L26_TST_DAC_MSM_CONFIG_DEFAULT_CHANGE_VALUE_H16,
			true, CS40L26_PSEQ_OP_WRITE_H16);
	if (ret)
		dev_err(dev, "Failed to sequence register default updates\n");

	return ret;
}

static int cs40l26_irq_update_mask(struct cs40l26_private *cs40l26, u32 reg,
		u32 val, u32 bit_mask)
{
	u32 eint_reg, cur_mask, new_mask;
	int ret;

	if (reg == CS40L26_IRQ1_MASK_1) {
		eint_reg = CS40L26_IRQ1_EINT_1;
	} else if (reg == CS40L26_IRQ1_MASK_2) {
		eint_reg = CS40L26_IRQ1_EINT_2;
	} else {
		dev_err(cs40l26->dev, "Invalid IRQ mask reg: 0x%08X\n", reg);
		return -EINVAL;
	}

	ret = regmap_read(cs40l26->regmap, reg, &cur_mask);
	if  (ret) {
		dev_err(cs40l26->dev, "Failed to get IRQ mask\n");
		return ret;
	}

	new_mask = (cur_mask & ~bit_mask) | val;

	/* Clear interrupt prior to masking/unmasking */
	ret = regmap_write(cs40l26->regmap, eint_reg, bit_mask);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to clear IRQ\n");
		return ret;
	}

	ret = regmap_write(cs40l26->regmap, reg, new_mask);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to update IRQ mask\n");
		return ret;
	}

	if (bit_mask & GENMASK(31, 16)) {
		ret = cs40l26_pseq_write(cs40l26, reg,
			(new_mask & GENMASK(31, 16)) >> 16,
			true, CS40L26_PSEQ_OP_WRITE_H16);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to update IRQ mask H16");
			return ret;
		}
	}

	if (bit_mask & GENMASK(15, 0)) {
		ret = cs40l26_pseq_write(cs40l26, reg,
			(new_mask & GENMASK(15, 0)),
			true, CS40L26_PSEQ_OP_WRITE_L16);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to update IRQ mask L16");
			return ret;
		}
	}

	return ret;
}

static int cs40l26_buzzgen_set(struct cs40l26_private *cs40l26, u16 freq,
		u16 level, u16 duration, u8 buzzgen_num)
{
	unsigned int base_reg, freq_reg, level_reg, duration_reg;
	int ret;

	/* BUZZ_EFFECTS1_BUZZ_xxx are initially populated by contents of OTP.
	 * The buzz specified by these controls is triggered by writing
	 * 0x01800080 to the DSP mailbox
	 */
	ret = cl_dsp_get_reg(cs40l26->dsp, "BUZZ_EFFECTS1_BUZZ_FREQ",
		CL_DSP_XM_UNPACKED_TYPE, CS40L26_BUZZGEN_ALGO_ID, &base_reg);
	if (ret)
		return ret;

	freq_reg = base_reg
			+ ((buzzgen_num) * CS40L26_BUZZGEN_CONFIG_OFFSET);
	level_reg = base_reg
			+ ((buzzgen_num) * CS40L26_BUZZGEN_CONFIG_OFFSET)
			+ CS40L26_BUZZGEN_LEVEL_OFFSET;
	duration_reg = base_reg
			+ ((buzzgen_num) * CS40L26_BUZZGEN_CONFIG_OFFSET)
			+ CS40L26_BUZZGEN_DURATION_OFFSET;

	ret = regmap_write(cs40l26->regmap, freq_reg, freq);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to write BUZZGEN frequency\n");
		return ret;
	}

	ret = regmap_write(cs40l26->regmap, level_reg, level);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to write BUZZGEN level\n");
		return ret;
	}

	ret = regmap_write(cs40l26->regmap, duration_reg, duration / 4);
	if (ret)
		dev_err(cs40l26->dev, "Failed to write BUZZGEN duration\n");

	return ret;
}

static int cs40l26_map_gpi_to_haptic(struct cs40l26_private *cs40l26,
		struct ff_effect *effect,
		struct cs40l26_uploaded_effect *ueffect)
{
	u16 button = effect->trigger.button;
	u8 gpio = (button & CS40L26_BTN_NUM_MASK) >> CS40L26_BTN_NUM_SHIFT;
	bool edge, ev_handler_bank_ram, owt, use_timeout;
	unsigned int fw_rev;
	u32 reg, write_val;
	int ret;

	edge = (button & CS40L26_BTN_EDGE_MASK) >> CS40L26_BTN_EDGE_SHIFT;

	switch (ueffect->wvfrm_bank) {
	case CS40L26_RAM_BANK_ID:
	case CS40L26_BUZ_BANK_ID:
		owt = false;
		ev_handler_bank_ram = true;
		break;
	case CS40L26_ROM_BANK_ID:
		owt = false;
		ev_handler_bank_ram = false;
		break;
	case CS40L26_OWT_BANK_ID:
		owt = true;
		ev_handler_bank_ram = true;
		break;
	default:
		dev_err(cs40l26->dev, "Effect bank %u not supported\n",
							ueffect->wvfrm_bank);
		return -EINVAL;
	}

	if (gpio != CS40L26_GPIO1) {
		dev_err(cs40l26->dev, "GPIO%u not supported on 0x%02X\n", gpio,
				cs40l26->revid);
		return -EINVAL;
	}

	reg = cs40l26->event_map_base + (edge ? 0 : 4);
	write_val = (ueffect->trigger_index & CS40L26_BTN_INDEX_MASK) |
			(ev_handler_bank_ram << CS40L26_BTN_BANK_SHIFT) |
			(owt << CS40L26_BTN_OWT_SHIFT);

	ret = regmap_write(cs40l26->regmap, reg, write_val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to update event map\n");
		return ret;
	}

	ret = cl_dsp_fw_rev_get(cs40l26->dsp, &fw_rev);
	if (ret)
		return ret;

	use_timeout = (!cs40l26->calib_fw &&
			fw_rev >= CS40L26_FW_GPI_TIMEOUT_MIN_REV) ||
			(cs40l26->calib_fw && fw_rev >=
			CS40L26_FW_GPI_TIMEOUT_CALIB_MIN_REV);

	if (use_timeout) {
		ret = cl_dsp_get_reg(cs40l26->dsp, "TIMEOUT_GPI_MS",
				CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (ret)
			return ret;

		ret = regmap_write(cs40l26->regmap, reg, effect->replay.length);
		if (ret)
			dev_warn(cs40l26->dev,
				"Failed to set GPI timeout, continuing...\n");
	}

	if (edge)
		ueffect->mapping = CS40L26_GPIO_MAP_A_PRESS;
	else
		ueffect->mapping = CS40L26_GPIO_MAP_A_RELEASE;

	return ret;
}

static struct cs40l26_uploaded_effect
		*cs40l26_uploaded_effect_find(struct cs40l26_private *cs40l26,
		int id)
{
	struct list_head *head = &cs40l26->effect_head;
	struct cs40l26_uploaded_effect *ueffect;

	if (list_empty(head)) {
		dev_dbg(cs40l26->dev, "Effect list is empty\n");
		return ERR_PTR(-ENODATA);
	}

	list_for_each_entry(ueffect, head, list) {
		if (ueffect->id == id)
			break;
	}

	if (ueffect->id != id) {
		dev_dbg(cs40l26->dev, "No such effect (ID = %d)\n", id);
		return ERR_PTR(-ENODEV);
	}

	return ueffect;
}

static bool cs40l26_is_no_wait_ram_index(struct cs40l26_private *cs40l26,
		u32 index)
{
	int i;

	for (i = 0; i < cs40l26->num_no_wait_ram_indices; i++) {
		if (cs40l26->no_wait_ram_indices[i] == index)
			return true;
	}

	return false;
}

static void cs40l26_set_gain_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 =
		container_of(work, struct cs40l26_private, set_gain_work);
	u16 gain;
	u32 reg;
	int ret;

	ret = cs40l26_pm_enter(cs40l26->dev);
	if (ret)
		return;

	mutex_lock(&cs40l26->lock);

	if (cs40l26->vibe_state == CS40L26_VIBE_STATE_ASP) {
		gain = (cs40l26->pdata.asp_scale_pct * cs40l26->gain_pct) /
				CS40L26_GAIN_FULL_SCALE;
		cs40l26->gain_tmp = cs40l26->gain_pct;
		cs40l26->gain_pct = gain;
		cs40l26->scaling_applied = true;
	} else {
		gain = cs40l26->gain_pct;
	}

	dev_dbg(cs40l26->dev, "%s: gain = %u%%\n", __func__, gain);

	/* Write Q21.2 value to SOURCE_ATTENUATION */
	ret = cl_dsp_get_reg(cs40l26->dsp, "SOURCE_ATTENUATION",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_write(cs40l26->regmap, reg, cs40l26_attn_q21_2_vals[gain]);
	if (ret)
		dev_err(cs40l26->dev, "Failed to set attenuation\n");

err_mutex:
	mutex_unlock(&cs40l26->lock);
	cs40l26_pm_exit(cs40l26->dev);
}

static void cs40l26_vibe_start_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 = container_of(work,
			struct cs40l26_private, vibe_start_work);
	struct device *dev = cs40l26->dev;
	struct cs40l26_uploaded_effect *ueffect;
	struct ff_effect *effect;
	unsigned int reg;
	u16 duration;
	bool invert;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return;

	mutex_lock(&cs40l26->lock);

	effect = cs40l26->trigger_effect;

	ueffect = cs40l26_uploaded_effect_find(cs40l26, effect->id);
	if (IS_ERR_OR_NULL(ueffect)) {
		dev_err(dev, "No such effect to play back\n");
		ret = PTR_ERR(ueffect);
		goto err_mutex;
	}

	duration = effect->replay.length;

	ret = cl_dsp_get_reg(cs40l26->dsp, "TIMEOUT_MS",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	ret = regmap_write(cs40l26->regmap, reg, duration);
	if (ret) {
		dev_err(dev, "Failed to set TIMEOUT_MS\n");
		goto err_mutex;
	}

	ret = cl_dsp_get_reg(cs40l26->dsp, "SOURCE_INVERT",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		goto err_mutex;

	switch (effect->direction) {
	case 0x0000:
		invert = false;
		break;
	case 0x8000:
		invert = true;
		break;
	default:
		dev_err(dev, "Invalid ff_effect direction: 0x%X\n",
				effect->direction);
		ret = -EINVAL;
		goto err_mutex;
	}

	ret = regmap_write(cs40l26->regmap, reg, invert);
	if (ret)
		goto err_mutex;

	switch (effect->u.periodic.waveform) {
	case FF_CUSTOM:
	case FF_SINE:
		ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				ueffect->trigger_index, CS40L26_DSP_MBOX_RESET);
		if (ret)
			goto err_mutex;

		cs40l26->cur_index = ueffect->trigger_index;
		break;
	default:
		dev_err(dev, "Invalid waveform type: 0x%X\n",
				effect->u.periodic.waveform);
		ret = -EINVAL;
		goto err_mutex;
	}

	if (!cs40l26->vibe_state_reporting)
		cs40l26_vibe_state_update(cs40l26,
				CS40L26_VIBE_STATE_EVENT_MBOX_PLAYBACK);

	reinit_completion(&cs40l26->erase_cont);
err_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(dev);
}

static void cs40l26_vibe_stop_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 = container_of(work,
			struct cs40l26_private, vibe_stop_work);
	bool skip_delay;
	u32 delay_us;
	int ret;

	dev_dbg(cs40l26->dev, "%s\n", __func__);

	ret = cs40l26_pm_enter(cs40l26->dev);
	if (ret)
		return;

	mutex_lock(&cs40l26->lock);

	delay_us = cs40l26->delay_before_stop_playback_us;
	skip_delay = cs40l26_is_no_wait_ram_index(cs40l26, cs40l26->cur_index);

	if (delay_us && !skip_delay) {
		mutex_unlock(&cs40l26->lock);

		dev_info(cs40l26->dev, "Applying delay\n");

		/* wait for SVC init phase to complete */
		usleep_range(delay_us, delay_us + 100);

		mutex_lock(&cs40l26->lock);
	} else {
		dev_info(cs40l26->dev, "Skipping delay\n");
	}

	if (cs40l26->vibe_state != CS40L26_VIBE_STATE_HAPTIC) {
		dev_warn(cs40l26->dev, "Attempted stop when vibe_state = %d\n",
				cs40l26->vibe_state);
		goto mutex_exit;
	}

	ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
				CS40L26_STOP_PLAYBACK, CS40L26_DSP_MBOX_RESET);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to stop playback\n");
		goto mutex_exit;
	}

mutex_exit:
	mutex_unlock(&cs40l26->lock);
	cs40l26_pm_exit(cs40l26->dev);
}

static void cs40l26_set_gain(struct input_dev *dev, u16 gain)
{
	struct cs40l26_private *cs40l26 = input_get_drvdata(dev);

	ATRACE_BEGIN(__func__);
	if (gain >= CS40L26_NUM_PCT_MAP_VALUES) {
		dev_err(cs40l26->dev, "Gain value %u %% out of bounds\n", gain);
		return;
	}

	cs40l26->gain_pct = gain;

	queue_work(cs40l26->vibe_workqueue, &cs40l26->set_gain_work);
	ATRACE_END();
}

static int cs40l26_playback_effect(struct input_dev *dev,
		int effect_id, int val)
{
	struct cs40l26_private *cs40l26 = input_get_drvdata(dev);
	struct ff_effect *effect;

	ATRACE_BEGIN(__func__);
	dev_dbg(cs40l26->dev, "%s: effect ID = %d, val = %d\n", __func__,
			effect_id, val);

	effect = &dev->ff->effects[effect_id];
	if (!effect) {
		dev_err(cs40l26->dev, "No such effect to playback\n");
		return -EINVAL;
	}

	cs40l26->trigger_effect = effect;

	if (val > 0)
		queue_work(cs40l26->vibe_workqueue, &cs40l26->vibe_start_work);
	else
		queue_work(cs40l26->vibe_workqueue, &cs40l26->vibe_stop_work);

	ATRACE_END();
	return 0;
}

int cs40l26_get_num_waves(struct cs40l26_private *cs40l26, u32 *num_waves)
{
	int ret;
	u32 reg, nwaves, nowt;

	ret = cl_dsp_get_reg(cs40l26->dsp, "NUM_OF_WAVES",
			CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_dsp_read(cs40l26, reg, &nwaves);
	if (ret)
		return ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "OWT_NUM_OF_WAVES_XM",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = cs40l26_dsp_read(cs40l26, reg, &nowt);
	if (ret)
		return ret;

	*num_waves = nwaves + nowt;

	return 0;
}
EXPORT_SYMBOL(cs40l26_get_num_waves);

static struct cl_dsp_owt_header *cs40l26_header(struct cs40l26_private *cs40l26,
		u8 index)
{
	if (!cs40l26->dsp || !cs40l26->dsp->wt_desc ||
			index >= cs40l26->dsp->wt_desc->owt.nwaves)
		return ERR_PTR(-EINVAL);

	return &cs40l26->dsp->wt_desc->owt.waves[index];
}

static int cs40l26_owt_get_wlength(struct cs40l26_private *cs40l26,
		u8 index, u32 *wlen_whole)
{
	struct device *dev = cs40l26->dev;
	struct cl_dsp_owt_header *entry;
	struct cl_dsp_memchunk ch;

	if (index == 0) {
		*wlen_whole = 0;
		return 0;
	}

	entry = cs40l26_header(cs40l26, index);
	if (IS_ERR(entry))
		return PTR_ERR(entry);

	switch (entry->type) {
	case WT_TYPE_V6_PCM_F0_REDC:
	case WT_TYPE_V6_PCM_F0_REDC_VAR:
	case WT_TYPE_V6_PWLE:
		break;
	default:
		dev_err(dev, "Cannot size waveform type %u\n", entry->type);
		return -EINVAL;
	}

	ch = cl_dsp_memchunk_create(entry->data, sizeof(u32));

	/* First 24 bits of each waveform is the length in samples @ 8 kHz */
	return cl_dsp_memchunk_read(cs40l26->dsp, &ch, 24, wlen_whole);
}

static void cs40l26_owt_set_section_info(struct cs40l26_private *cs40l26,
		struct cl_dsp_memchunk *ch,
		struct cs40l26_owt_section *sections, u8 nsections)
{
	int i;

	for (i = 0; i < nsections; i++) {
		cl_dsp_memchunk_write(ch, 8, sections[i].amplitude);
		cl_dsp_memchunk_write(ch, 8, sections[i].index);
		cl_dsp_memchunk_write(ch, 8, sections[i].repeat);
		cl_dsp_memchunk_write(ch, 8, sections[i].flags);
		cl_dsp_memchunk_write(ch, 16, sections[i].delay);

		if (sections[i].flags & CS40L26_WT_TYPE10_COMP_DURATION_FLAG) {
			cl_dsp_memchunk_write(ch, 8, 0x00); /* Pad */
			cl_dsp_memchunk_write(ch, 16, sections[i].duration);
		}
	}
}

static int cs40l26_owt_get_section_info(struct cs40l26_private *cs40l26,
		struct cl_dsp_memchunk *ch,
		struct cs40l26_owt_section *sections, u8 nsections)
{
	int ret = 0, i;

	for (i = 0; i < nsections; i++) {
		ret = cl_dsp_memchunk_read(cs40l26->dsp, ch, 8,
				&sections[i].amplitude);
		if (ret)
			return ret;

		ret = cl_dsp_memchunk_read(cs40l26->dsp, ch, 8,
				&sections[i].index);
		if (ret)
			return ret;

		ret = cl_dsp_memchunk_read(cs40l26->dsp, ch, 8,
				&sections[i].repeat);
		if (ret)
			return ret;

		ret = cl_dsp_memchunk_read(cs40l26->dsp, ch, 8,
				&sections[i].flags);
		if (ret)
			return ret;

		ret = cl_dsp_memchunk_read(cs40l26->dsp, ch, 16,
				&sections[i].delay);
		if (ret)
			return ret;

		if (sections[i].flags & CS40L26_WT_TYPE10_COMP_DURATION_FLAG) {
			/* Skip padding */
			ret = cl_dsp_memchunk_read(cs40l26->dsp, ch, 8, NULL);
			if (ret)
				return ret;

			ret = cl_dsp_memchunk_read(cs40l26->dsp, ch, 16,
					&sections[i].duration);
			if (ret)
				return ret;
		}
	}

	return ret;
}

static int cs40l26_owt_calculate_wlength(struct cs40l26_private *cs40l26,
	u8 nsections, u8 global_rep, u8 *data, u32 data_size_bytes,
	u32 *owt_wlen)
{
	u32 total_len = 0, section_len = 0, loop_len = 0, wlen_whole = 0;
	bool in_loop = false;
	int ret = 0, i;
	struct cs40l26_owt_section *sections;
	struct cl_dsp_memchunk ch;
	u32 dlen, wlen;

	if (nsections < 1) {
		dev_err(cs40l26->dev, "Not enough sections for composite\n");
		return -EINVAL;
	}

	sections = kcalloc(nsections, sizeof(struct cs40l26_owt_section),
			GFP_KERNEL);
	if (!sections)
		return -ENOMEM;

	ch = cl_dsp_memchunk_create((void *) data, data_size_bytes);
	ret = cs40l26_owt_get_section_info(cs40l26, &ch, sections, nsections);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get section info\n");
		goto err_free;
	}

	for (i = 0; i < nsections; i++) {
		ret = cs40l26_owt_get_wlength(cs40l26, sections[i].index,
				&wlen_whole);
		if (ret < 0) {
			dev_err(cs40l26->dev,
					"Failed to get wlength for index %u\n",
					sections[i].index);
			goto err_free;
		}

		if (wlen_whole & CS40L26_WT_TYPE10_WAVELEN_INDEF) {
			if (!(sections[i].flags &
					CS40L26_WT_TYPE10_COMP_DURATION_FLAG)) {
				dev_err(cs40l26->dev,
					"Indefinite entry needs duration\n");
				ret = -EINVAL;
				goto err_free;
			}

			wlen = CS40L26_WT_TYPE10_WAVELEN_MAX;
		} else {
			/* Length is 22 LSBs, filter out flags */
			wlen = wlen_whole & CS40L26_WT_TYPE10_WAVELEN_MAX;
		}

		dlen = 8 * sections[i].delay;

		if (sections[i].flags & CS40L26_WT_TYPE10_COMP_DURATION_FLAG) {
			if (wlen > (2 * sections[i].duration))
				wlen = 2 * sections[i].duration;
		}

		section_len = wlen + dlen;
		loop_len += section_len;

		if (sections[i].repeat == 0xFF) {
			in_loop = true;
		} else if (sections[i].repeat) {
			total_len += (loop_len * (sections[i].repeat + 1));

			in_loop = false;
			loop_len = 0;
		} else if (!in_loop) {
			total_len += section_len;
			loop_len = 0;
		}

		section_len = 0;
	}

	*owt_wlen = (total_len * (global_rep + 1)) |
			CS40L26_WT_TYPE10_WAVELEN_CALCULATED;

err_free:
	kfree(sections);

	return ret;
}

static int cs40l26_owt_upload(struct cs40l26_private *cs40l26, u8 *data,
		u32 data_size_bytes)
{
	struct device *dev = cs40l26->dev;
	struct cl_dsp *dsp = cs40l26->dsp;
	unsigned int write_reg, reg, wt_offset, wt_size_words, wt_base;
	int ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	ret = cl_dsp_get_reg(dsp, "OWT_NEXT_XM", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_read(cs40l26->regmap, reg, &wt_offset);
	if (ret) {
		dev_err(dev, "Failed to get wavetable offset\n");
		goto err_pm;
	}

	ret = cl_dsp_get_reg(dsp, "OWT_SIZE_XM", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto err_pm;

	ret = regmap_read(cs40l26->regmap, reg, &wt_size_words);
	if (ret) {
		dev_err(dev, "Failed to get available WT size\n");
		goto err_pm;
	}

	if ((wt_size_words * CL_DSP_BYTES_PER_WORD) < data_size_bytes) {
		dev_err(dev, "No space for OWT waveform\n");
		ret = -ENOSPC;
		goto err_pm;
	}

	ret = cl_dsp_get_reg(dsp, CS40L26_WT_NAME_XM, CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_VIBEGEN_ALGO_ID, &wt_base);
	if (ret)
		goto err_pm;

	write_reg = wt_base + (wt_offset * 4);

	ret = cl_dsp_raw_write(cs40l26->dsp, write_reg, data, data_size_bytes,
			CL_DSP_MAX_WLEN);
	if (ret) {
		dev_err(dev, "Failed to sync OWT\n");
		goto err_pm;
	}

	ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
			CS40L26_DSP_MBOX_CMD_OWT_PUSH, CS40L26_DSP_MBOX_RESET);
	if (ret)
		goto err_pm;

	dev_dbg(dev, "Successfully wrote waveform (%u bytes) to 0x%08X\n",
			data_size_bytes, write_reg);

err_pm:
	cs40l26_pm_exit(dev);

	return ret;
}

static u8 *cs40l26_ncw_amp_scaling(struct cs40l26_private *cs40l26, u8 amp,
		u8 nsections, void *in_data, u32 data_bytes)
{
	struct cs40l26_owt_section *sections;
	struct cl_dsp_memchunk in_ch, out_ch;
	u16 amp_product;
	u8 *out_data;
	int i, ret;

	if (nsections <= 0) {
		dev_err(cs40l26->dev, "Too few sections for NCW\n");
		return ERR_PTR(-EINVAL);
	}

	sections = kcalloc(nsections, sizeof(struct cs40l26_owt_section),
			GFP_KERNEL);
	if (!sections)
		return ERR_PTR(-ENOMEM);

	in_ch = cl_dsp_memchunk_create(in_data, data_bytes);

	ret = cs40l26_owt_get_section_info(cs40l26, &in_ch, sections,
			nsections);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get section info\n");
		goto sections_free;
	}

	for (i = 0; i < nsections; i++) {
		if (sections[i].index != 0) {
			amp_product = sections[i].amplitude * amp;
			sections[i].amplitude =
					(u8) DIV_ROUND_UP(amp_product, 100);
		}
	}

	out_data = kcalloc(data_bytes, sizeof(u8), GFP_KERNEL);
	if (!out_data) {
		ret = -ENOMEM;
		goto sections_free;
	}

	out_ch = cl_dsp_memchunk_create((void *) out_data, data_bytes);
	cs40l26_owt_set_section_info(cs40l26, &out_ch, sections, nsections);

sections_free:
	kfree(sections);

	return ret ? ERR_PTR(ret) : out_data;
}

static int cs40l26_owt_comp_data_size(struct cs40l26_private *cs40l26,
		u8 nsections, struct cs40l26_owt_section *sections)
{
	int i, size = 0;
	struct cl_dsp_owt_header *header;

	for (i = 0; i < nsections; i++) {
		if (sections[i].index == 0) {
			size += CS40L26_WT_TYPE10_SECTION_BYTES_MIN;
			continue;
		}

		header = cs40l26_header(cs40l26, sections[i].index);
		if (IS_ERR(header))
			return PTR_ERR(header);

		if (header->type == WT_TYPE_V6_COMPOSITE) {
			size += (header->size - 2) * 4;

			if (section_complete(&sections[i]))
				size += CS40L26_WT_TYPE10_SECTION_BYTES_MIN;
		} else {
			size += sections[i].duration ?
					CS40L26_WT_TYPE10_SECTION_BYTES_MAX :
					CS40L26_WT_TYPE10_SECTION_BYTES_MIN;
		}
	}

	return size;
}

static int cs40l26_refactor_owt_composite(struct cs40l26_private *cs40l26, s16 *in_data,
		u32 in_data_nibbles, u8 **out_data)
{
	u8 nsections, global_rep, out_nsections = 0;
	int ret = 0, pos_byte = 0, in_pos_nib = 2;
	int in_data_bytes = 2 * in_data_nibbles;
	int out_data_bytes = 0, data_bytes = 0;
	struct device *dev = cs40l26->dev;
	u8 delay_section_data[CS40L26_WT_TYPE10_SECTION_BYTES_MIN];
	u8 ncw_nsections, ncw_global_rep, *data, *ncw_data;
	struct cs40l26_owt_section *sections;
	struct cl_dsp_memchunk ch, out_ch;
	struct cl_dsp_owt_header *header;
	u16 section_size_bytes;
	u32 ncw_bytes, wlen;
	int i;

	ch = cl_dsp_memchunk_create((void *) in_data, in_data_bytes);
	/* Skip padding */
	ret = cl_dsp_memchunk_read(cs40l26->dsp, &ch, 8, NULL);
	if (ret)
		return ret;

	ret = cl_dsp_memchunk_read(cs40l26->dsp, &ch, 8, &nsections);
	if (ret)
		return ret;

	ret = cl_dsp_memchunk_read(cs40l26->dsp, &ch, 8, &global_rep);
	if (ret)
		return ret;

	sections = kcalloc(nsections, sizeof(struct cs40l26_owt_section),
			GFP_KERNEL);
	if (!sections)
		return -ENOMEM;

	ret = cs40l26_owt_get_section_info(cs40l26, &ch, sections, nsections);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to get section info\n");
		return ret;
	}

	data_bytes = cs40l26_owt_comp_data_size(cs40l26, nsections,
			sections);
	if (data_bytes <= 0) {
		dev_err(dev, "Failed to get OWT Composite Data Size\n");
		ret = data_bytes;
		goto sections_err_free;
	}

	data = kcalloc(data_bytes, sizeof(u8), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto sections_err_free;
	}

	cl_dsp_memchunk_flush(&ch);
	memset(&delay_section_data, 0, CS40L26_WT_TYPE10_SECTION_BYTES_MIN);

	for (i = 0; i < nsections; i++) {
		section_size_bytes = sections[i].duration ?
				CS40L26_WT_TYPE10_SECTION_BYTES_MAX :
				CS40L26_WT_TYPE10_SECTION_BYTES_MIN;

		if (sections[i].index == 0) {
			memcpy(data + pos_byte, in_data + in_pos_nib,
					section_size_bytes);
			pos_byte += section_size_bytes;
			in_pos_nib += section_size_bytes / 2;
			out_nsections++;
			continue;
		}

		if (sections[i].repeat != 0) {
			dev_err(dev, "Inner repeats not allowed for NCWs\n");
			ret = -EPERM;
			goto data_err_free;
		}

		header = cs40l26_header(cs40l26, sections[i].index);
		if (IS_ERR(header)) {
			ret = PTR_ERR(header);
			goto data_err_free;
		}

		if (header->type == WT_TYPE_V6_COMPOSITE) {
			ch = cl_dsp_memchunk_create(header->data, 8);
			/* Skip Wlength */
			ret = cl_dsp_memchunk_read(cs40l26->dsp, &ch, 24, NULL);
			if (ret)
				return ret;

			/* Skip Padding */
			ret = cl_dsp_memchunk_read(cs40l26->dsp, &ch, 8, NULL);
			if (ret)
				return ret;

			ret = cl_dsp_memchunk_read(cs40l26->dsp, &ch, 8,
					&ncw_nsections);
			if (ret)
				return ret;

			ret = cl_dsp_memchunk_read(cs40l26->dsp, &ch, 8,
					&ncw_global_rep);
			if (ret)
				return ret;

			if (ncw_global_rep != 0) {
				dev_err(dev,
					"No NCW support for outer repeat\n");
				ret = -EPERM;
				goto data_err_free;
			}

			cl_dsp_memchunk_flush(&ch);

			ncw_bytes = (header->size - 2) * 4;
			ncw_data = cs40l26_ncw_amp_scaling(cs40l26,
					sections[i].amplitude, ncw_nsections,
					header->data + 8, ncw_bytes);
			if (IS_ERR(ncw_data)) {
				ret = PTR_ERR(ncw_data);
				goto data_err_free;
			}

			memcpy(data + pos_byte, ncw_data, ncw_bytes);
			pos_byte += ncw_bytes;
			out_nsections += ncw_nsections;
			kfree(ncw_data);

			if (section_complete(&sections[i])) {
				ch = cl_dsp_memchunk_create((void *)
					delay_section_data,
					CS40L26_WT_TYPE10_SECTION_BYTES_MIN);

				cl_dsp_memchunk_write(&ch, 24, 0x000000);
				cl_dsp_memchunk_write(&ch, 8, 0x00);
				cl_dsp_memchunk_write(&ch, 16,
						sections[i].delay);

				memcpy(data + pos_byte,
					delay_section_data,
					CS40L26_WT_TYPE10_SECTION_BYTES_MIN);

				cl_dsp_memchunk_flush(&ch);

				pos_byte +=
					CS40L26_WT_TYPE10_SECTION_BYTES_MIN;
				out_nsections++;
			}
		} else {
			memcpy(data + pos_byte, in_data + in_pos_nib,
					section_size_bytes);
			pos_byte += section_size_bytes;
			out_nsections++;
		}
		in_pos_nib += section_size_bytes / 2;
	}

	out_data_bytes = data_bytes + CS40L26_WT_HEADER_COMP_SIZE;
	*out_data = kcalloc(out_data_bytes, sizeof(u8), GFP_KERNEL);
	if (!*out_data) {
		dev_err(dev, "Failed to allocate space for composite\n");
		ret = -ENOMEM;
		goto data_err_free;
	}

	out_ch = cl_dsp_memchunk_create((void *) *out_data, out_data_bytes);
	cl_dsp_memchunk_write(&out_ch, 16, CS40L26_WT_HEADER_DEFAULT_FLAGS);
	cl_dsp_memchunk_write(&out_ch, 8, WT_TYPE_V6_COMPOSITE);
	cl_dsp_memchunk_write(&out_ch, 24, CS40L26_WT_HEADER_OFFSET);
	cl_dsp_memchunk_write(&out_ch, 24, data_bytes / CL_DSP_BYTES_PER_WORD);

	ret = cs40l26_owt_calculate_wlength(cs40l26, out_nsections, global_rep,
			data, data_bytes, &wlen);
	if (ret) {
		kfree(out_data);
		goto data_err_free;
	}

	cl_dsp_memchunk_write(&out_ch, 24, wlen);
	cl_dsp_memchunk_write(&out_ch, 8, 0x00); /* Pad */
	cl_dsp_memchunk_write(&out_ch, 8, out_nsections);
	cl_dsp_memchunk_write(&out_ch, 8, global_rep);

	memcpy(*out_data + out_ch.bytes, data, data_bytes);

data_err_free:
	kfree(data);
sections_err_free:
	kfree(sections);

	return ret ? ret : out_data_bytes;
}

static u8 cs40l26_get_lowest_free_buzzgen(struct cs40l26_private *cs40l26)
{
	u8 buzzgen = 1;
	struct cs40l26_uploaded_effect *ueffect;

	if (list_empty(&cs40l26->effect_head))
		return buzzgen;

	list_for_each_entry(ueffect, &cs40l26->effect_head, list) {
		if (ueffect->wvfrm_bank == CS40L26_BUZ_BANK_ID)
			buzzgen++;
	}

	return buzzgen;
}

static int cs40l26_sine_upload(struct cs40l26_private *cs40l26,
		struct ff_effect *effect,
		struct cs40l26_uploaded_effect *ueffect)
{
	struct device *dev = cs40l26->dev;
	u8 lowest_free_buzzgen, level;
	u16 freq, period;
	int ret;

	if (effect->u.periodic.period < CS40L26_BUZZGEN_PER_MIN)
		period = CS40L26_BUZZGEN_PER_MIN;
	else if (effect->u.periodic.period > CS40L26_BUZZGEN_PER_MAX)
		period = CS40L26_BUZZGEN_PER_MAX;
	else
		period = effect->u.periodic.period;

	freq = CS40L26_MS_TO_HZ(period);

	if (effect->u.periodic.magnitude < CS40L26_BUZZGEN_LEVEL_MIN)
		level = CS40L26_BUZZGEN_LEVEL_MIN;
	else if (effect->u.periodic.magnitude > CS40L26_BUZZGEN_LEVEL_MAX)
		level = CS40L26_BUZZGEN_LEVEL_MAX;
	else
		level = effect->u.periodic.magnitude;

	lowest_free_buzzgen = cs40l26_get_lowest_free_buzzgen(cs40l26);
	dev_dbg(dev, "lowest_free_buzzgen: %d", lowest_free_buzzgen);

	if (lowest_free_buzzgen > CS40L26_BUZZGEN_NUM_CONFIGS) {
		dev_err(dev, "Unable to upload buzzgen effect\n");
		return -ENOSPC;
	}

	ret = cs40l26_buzzgen_set(cs40l26, freq, level,
				effect->replay.length, lowest_free_buzzgen);
	if (ret)
		return ret;

	ueffect->id = effect->id;
	ueffect->wvfrm_bank = CS40L26_BUZ_BANK_ID;
	ueffect->trigger_index = CS40L26_BUZZGEN_INDEX_START +
					lowest_free_buzzgen;

	return ret;
}

static int cs40l26_custom_upload(struct cs40l26_private *cs40l26,
		struct ff_effect *effect,
		struct cs40l26_uploaded_effect *ueffect)
{
	struct device *dev = cs40l26->dev;
	u32 nwaves, min_index, max_index, trigger_index;
	int ret, data_len, refactored_data_len;
	u8 *refactored_data;
	u16 index, bank;

	data_len = effect->u.periodic.custom_len;

	if (data_len > CS40L26_CUSTOM_DATA_SIZE) {
		if (cs40l26->raw_custom_data[1] == CS40L26_WT_TYPE12_IDENTIFIER) {
			refactored_data_len = cs40l26->raw_custom_data_len * 2;
			refactored_data = kcalloc(refactored_data_len, sizeof(u8), GFP_KERNEL);
			if (!refactored_data) {
				dev_err(dev, "Failed to allocate space for PWLE\n");
				return -ENOMEM;
			}

			memcpy(refactored_data, cs40l26->raw_custom_data, refactored_data_len);
		} else {
			refactored_data_len = cs40l26_refactor_owt_composite(cs40l26,
					cs40l26->raw_custom_data, data_len, &refactored_data);
			if (refactored_data_len <= 0) {
				dev_err(dev, "Failed to refactor OWT\n");
				return -ENOMEM;
			}
		}

		ret = cs40l26_owt_upload(cs40l26, refactored_data, refactored_data_len);
		kfree(refactored_data);
		if (ret)
			return ret;

		bank = (u16) CS40L26_OWT_BANK_ID;
		index = (u16) cs40l26->num_owt_effects;
	} else {
		bank = (u16) cs40l26->raw_custom_data[0];
		index = (u16) (cs40l26->raw_custom_data[1] &
				CS40L26_MAX_INDEX_MASK);
	}

	ret = cs40l26_get_num_waves(cs40l26, &nwaves);
	if (ret)
		return ret;

	switch (bank) {
	case CS40L26_RAM_BANK_ID:
		if (nwaves - cs40l26->num_owt_effects == 0) {
			dev_err(dev, "No waveforms in RAM bank\n");
			return -EINVAL;
		}

		min_index = CS40L26_RAM_INDEX_START;
		max_index = min_index + nwaves - cs40l26->num_owt_effects - 1;
		break;
	case CS40L26_ROM_BANK_ID:
		min_index = CS40L26_ROM_INDEX_START;
		max_index = CS40L26_ROM_INDEX_END;
		break;
	case CS40L26_OWT_BANK_ID:
		min_index = CS40L26_OWT_INDEX_START;
		max_index = CS40L26_OWT_INDEX_END;
		break;
	default:
		dev_err(dev, "Bank ID (%u) invalid\n", bank);
		return -EINVAL;
	}

	trigger_index = index + min_index;
	if (trigger_index < min_index || trigger_index > max_index) {
		dev_err(dev, "Index 0x%X out of bounds (0x%X - 0x%X)\n",
				trigger_index, min_index, max_index);
		return -EINVAL;
	}
	dev_dbg(dev, "ID = %d, trigger index = 0x%08X\n", effect->id,
			trigger_index);

	if (bank == CS40L26_OWT_BANK_ID)
		cs40l26->num_owt_effects++;

	ueffect->id = effect->id;
	ueffect->wvfrm_bank = bank;
	ueffect->trigger_index = trigger_index;

	return ret;
}

static int cs40l26_uploaded_effect_add(struct cs40l26_private *cs40l26,
		struct ff_effect *effect)
{
	struct device *dev = cs40l26->dev;
	bool is_new = false;
	struct cs40l26_uploaded_effect *ueffect;
	int ret;

	ueffect = cs40l26_uploaded_effect_find(cs40l26, effect->id);
	if (IS_ERR_OR_NULL(ueffect)) {
		is_new = true;
		ueffect = kzalloc(sizeof(*ueffect), GFP_KERNEL);
		if (!ueffect)
			return -ENOMEM;
	}

	if (effect->u.periodic.waveform == FF_CUSTOM) {
		ret = cs40l26_custom_upload(cs40l26, effect, ueffect);
	} else if (effect->u.periodic.waveform == FF_SINE) {
		ret = cs40l26_sine_upload(cs40l26, effect, ueffect);
	} else {
		dev_err(dev, "Periodic waveform type 0x%X not supported\n",
				effect->u.periodic.waveform);
		ret = -EINVAL;
	}

	if (ret)
		goto err_free;

	if (effect->trigger.button) {
		ret = cs40l26_map_gpi_to_haptic(cs40l26, effect, ueffect);
		if (ret)
			goto err_free;
	} else {
		ueffect->mapping = CS40L26_GPIO_MAP_INVALID;
	}

	if (is_new)
		list_add(&ueffect->list, &cs40l26->effect_head);

	return 0;
err_free:
	if (is_new)
		kfree(ueffect);

	return ret;
}

static void cs40l26_upload_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 = container_of(work,
			struct cs40l26_private, upload_work);
	struct device *cdev = cs40l26->dev;
	struct ff_effect *effect;
	u32 nwaves;
	int ret;

	ret = cs40l26_pm_enter(cdev);
	if (ret)
		return;

	mutex_lock(&cs40l26->lock);

	effect = &cs40l26->upload_effect;

	if (effect->type != FF_PERIODIC) {
		dev_err(cdev, "Effect type 0x%X not supported\n", effect->type);
		ret = -EINVAL;
		goto out_mutex;
	}

	ret = cs40l26_uploaded_effect_add(cs40l26, effect);
	if (ret)
		goto out_mutex;

	ret = cs40l26_get_num_waves(cs40l26, &nwaves);
	if (ret)
		goto out_mutex;

	dev_dbg(cdev, "Total number of waveforms = %u\n", nwaves);

out_mutex:
	mutex_unlock(&cs40l26->lock);

	cs40l26_pm_exit(cdev);

	cs40l26->upload_ret = ret;
}

static int cs40l26_upload_effect(struct input_dev *dev,
		struct ff_effect *effect, struct ff_effect *old)
{
	struct cs40l26_private *cs40l26 = input_get_drvdata(dev);
	int len = effect->u.periodic.custom_len;
	int ret;

	ATRACE_BEGIN(__func__);
	dev_dbg(cs40l26->dev, "%s: effect ID = %d\n", __func__, effect->id);

	memcpy(&cs40l26->upload_effect, effect, sizeof(struct ff_effect));

	if (effect->u.periodic.waveform == FF_CUSTOM) {
		cs40l26->raw_custom_data_len = len;

		cs40l26->raw_custom_data = kcalloc(len, sizeof(s16),
				GFP_KERNEL);
		if (!cs40l26->raw_custom_data) {
			ret = -ENOMEM;
			goto out_free;
		}

		if (copy_from_user(cs40l26->raw_custom_data,
				effect->u.periodic.custom_data,
				sizeof(s16) * len)) {
			dev_err(cs40l26->dev, "Failed to get user data\n");
			ret = -EFAULT;
			goto out_free;
		}
	}

	queue_work(cs40l26->vibe_workqueue, &cs40l26->upload_work);

	/* Wait for upload to finish */
	flush_work(&cs40l26->upload_work);

	ret = cs40l26->upload_ret;

out_free:
	memset(&cs40l26->upload_effect, 0, sizeof(struct ff_effect));
	kfree(cs40l26->raw_custom_data);
	cs40l26->raw_custom_data = NULL;
	ATRACE_END();

	return ret;
}

#if IS_ENABLED(CONFIG_INPUT_CS40L26_ATTR_UNDER_BUS)
const struct attribute_group *cs40l26_dev_attr_groups[] = {
	&cs40l26_dev_attr_group,
	&cs40l26_dev_attr_cal_group,
	&cs40l26_dev_attr_dbc_group,
	NULL,
};
#endif

static int cs40l26_erase_gpi_mapping(struct cs40l26_private *cs40l26,
		enum cs40l26_gpio_map mapping)
{
	int ret = 0;
	u32 reg;

	if (mapping == CS40L26_GPIO_MAP_A_PRESS)
		reg = CS40L26_A1_EVENT_MAP_1;
	else if (mapping == CS40L26_GPIO_MAP_A_RELEASE)
		reg = CS40L26_A1_EVENT_MAP_2;
	else
		ret = -EINVAL;

	if (ret) {
		dev_err(cs40l26->dev, "Invalid GPI mapping %u\n", mapping);
		return ret;
	}

	ret = regmap_write(cs40l26->regmap, reg, CS40L26_EVENT_MAP_GPI_DISABLE);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to clear GPI mapping %u\n",
				mapping);
		return ret;
	}

	return 0;
}

static int cs40l26_erase_owt(struct cs40l26_private *cs40l26,
		struct cs40l26_uploaded_effect *ueffect)
{
	u32 cmd = CS40L26_DSP_MBOX_CMD_OWT_DELETE_BASE;
	u32 index = ueffect->trigger_index;
	struct cs40l26_uploaded_effect *ueffect_tmp;
	int ret;

	cmd |= (index & 0xFF);

	ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1, cmd,
			CS40L26_DSP_MBOX_RESET);
	if (ret)
		return ret;

	/* Update indices for OWT waveforms uploaded after erased effect */
	list_for_each_entry(ueffect_tmp, &cs40l26->effect_head, list) {
		if (ueffect_tmp->wvfrm_bank == CS40L26_OWT_BANK_ID &&
				ueffect_tmp->trigger_index > index)
			ueffect_tmp->trigger_index--;
	}

	cs40l26->num_owt_effects--;

	return 0;
}

static void cs40l26_erase_worker(struct work_struct *work)
{
	struct cs40l26_private *cs40l26 = container_of(work,
			struct cs40l26_private, erase_work);
	struct cs40l26_uploaded_effect *ueffect;
	int effect_id, ret;
	u16 duration;

	ret = cs40l26_pm_enter(cs40l26->dev);
	if (ret)
		return;

	mutex_lock(&cs40l26->lock);

	effect_id = cs40l26->erase_effect->id;
	ueffect = cs40l26_uploaded_effect_find(cs40l26, effect_id);
	if (IS_ERR_OR_NULL(ueffect)) {
		dev_err(cs40l26->dev, "No such effect to erase (%d)\n",
				effect_id);
		ret = PTR_ERR(ueffect);
		goto out_mutex;
	}

	duration = (cs40l26->erase_effect->replay.length == 0) ?
		CS40L26_MAX_WAIT_VIBE_COMPLETE_MS :
		cs40l26->erase_effect->replay.length + CS40L26_ERASE_BUFFER_MS;

	/* Check for ongoing effect playback. */
	if (cs40l26->vibe_state == CS40L26_VIBE_STATE_HAPTIC) {
		/* Wait for effect to complete. */
		mutex_unlock(&cs40l26->lock);
		if (!wait_for_completion_timeout(&cs40l26->erase_cont,
				msecs_to_jiffies(duration))) {
			ret = -ETIME;
			dev_err(cs40l26->dev, "Failed to erase effect (%d)\n",
					effect_id);
			goto pm_err;
		}
		mutex_lock(&cs40l26->lock);
	}

	dev_dbg(cs40l26->dev, "%s: effect ID = %d\n", __func__, effect_id);

	if (ueffect->mapping != CS40L26_GPIO_MAP_INVALID) {
		ret = cs40l26_erase_gpi_mapping(cs40l26, ueffect->mapping);
		if (ret)
			goto out_mutex;
		ueffect->mapping = CS40L26_GPIO_MAP_INVALID;
	}

	if (ueffect->wvfrm_bank == CS40L26_OWT_BANK_ID)
		ret = cs40l26_erase_owt(cs40l26, ueffect);

	if (ret) {
		dev_err(cs40l26->dev, "Failed to erase effect: %d", ret);
		goto out_mutex;
	}

	list_del(&ueffect->list);
	kfree(ueffect);

out_mutex:
	mutex_unlock(&cs40l26->lock);
pm_err:
	cs40l26_pm_exit(cs40l26->dev);

	cs40l26->erase_ret = ret;
}

static int cs40l26_erase_effect(struct input_dev *dev, int effect_id)
{
	struct cs40l26_private *cs40l26 = input_get_drvdata(dev);
	struct ff_effect *effect;

	ATRACE_BEGIN(__func__);
	dev_dbg(cs40l26->dev, "%s: effect ID = %d\n", __func__, effect_id);

	effect = &dev->ff->effects[effect_id];
	if (!effect) {
		dev_err(cs40l26->dev, "No such effect to erase\n");
		return -EINVAL;
	}

	cs40l26->erase_effect = effect;

	queue_work(cs40l26->vibe_workqueue, &cs40l26->erase_work);

	/* Wait for erase to finish */
	flush_work(&cs40l26->erase_work);

	ATRACE_END();
	return cs40l26->erase_ret;
}

static int cs40l26_input_init(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	int ret;

	cs40l26->input = devm_input_allocate_device(dev);
	if (!cs40l26->input)
		return -ENOMEM;

	cs40l26->input->name = cs40l26->pdata.device_name;
	cs40l26->input->id.product = cs40l26->devid;
	cs40l26->input->id.version = cs40l26->revid;

	input_set_drvdata(cs40l26->input, cs40l26);
	input_set_capability(cs40l26->input, EV_FF, FF_PERIODIC);
	input_set_capability(cs40l26->input, EV_FF, FF_CUSTOM);
	input_set_capability(cs40l26->input, EV_FF, FF_SINE);
	input_set_capability(cs40l26->input, EV_FF, FF_GAIN);

	ret = input_ff_create(cs40l26->input, FF_MAX_EFFECTS);
	if (ret) {
		dev_err(dev, "Failed to create FF device: %d\n", ret);
		return ret;
	}

	/*
	 * input_ff_create() automatically sets FF_RUMBLE capabilities;
	 * we want to restrtict this to only FF_PERIODIC
	 */
	__clear_bit(FF_RUMBLE, cs40l26->input->ffbit);

	cs40l26->input->ff->upload = cs40l26_upload_effect;
	cs40l26->input->ff->playback = cs40l26_playback_effect;
	cs40l26->input->ff->set_gain = cs40l26_set_gain;
	cs40l26->input->ff->erase = cs40l26_erase_effect;

	ret = input_register_device(cs40l26->input);
	if (ret) {
		dev_err(dev, "Cannot register input device: %d\n", ret);
		return ret;
	}

#if !IS_ENABLED(CONFIG_INPUT_CS40L26_ATTR_UNDER_BUS)
	ret = sysfs_create_group(&cs40l26->input->dev.kobj,
			&cs40l26_dev_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&cs40l26->input->dev.kobj,
			&cs40l26_dev_attr_cal_group);
	if (ret) {
		dev_err(dev, "Failed to create cal sysfs group: %d\n", ret);
		return ret;
	}
	ret = sysfs_create_group(&cs40l26->input->dev.kobj,
			&cs40l26_dev_attr_dbc_group);
	if (ret) {
		dev_err(dev, "Failed to create DBC sysfs group\n");
		return ret;
	}
#else
	ret = sysfs_create_groups(&cs40l26->dev->kobj, cs40l26_dev_attr_groups);
	if (ret) {
		dev_err(dev, "Failed to create sysfs groups: %d\n", ret);
		return ret;
	}
#endif

	cs40l26->vibe_init_success = true;

	return ret;
}

static int cs40l26_part_num_resolve(struct cs40l26_private *cs40l26)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	int ret;
	u32 val;

	ret = regmap_read(regmap, CS40L26_DEVID, &val);
	if (ret) {
		dev_err(dev, "Failed to read device ID\n");
		return ret;
	}

	val &= CS40L26_DEVID_MASK;
	if (val != CS40L26_DEVID_A && val != CS40L26_DEVID_B && val !=
			CS40L26_DEVID_L27_A && val != CS40L26_DEVID_L27_B) {
		dev_err(dev, "Invalid device ID: 0x%06X\n", val);
		return -EINVAL;
	}

	cs40l26->devid = val;

	ret = regmap_read(regmap, CS40L26_REVID, &val);
	if (ret) {
		dev_err(dev, "Failed to read revision ID\n");
		return ret;
	}

	val &= CS40L26_REVID_MASK;

	switch (val) {
	case CS40L26_REVID_A1:
	case CS40L26_REVID_B0:
	case CS40L26_REVID_B1:
		cs40l26->revid = val;
		break;
	default:
		dev_err(dev, "Invalid device revision: 0x%02X\n", val);
		return -EINVAL;
	}

	dev_info(dev, "Cirrus Logic %s ID: 0x%06X, Revision: 0x%02X\n",
			CS40L26_DEV_NAME, cs40l26->devid, cs40l26->revid);

	return 0;
}

static int cs40l26_wksrc_config(struct cs40l26_private *cs40l26)
{
	u8 mask_wksrc;
	u32 val, mask;

	if (cs40l26->devid == CS40L26_DEVID_A ||
			cs40l26->devid == CS40L26_DEVID_L27_A)
		mask_wksrc = 1;
	else
		mask_wksrc = 0;

	val = BIT(CS40L26_IRQ1_WKSRC_STS_SPI) |
			(mask_wksrc << CS40L26_IRQ1_WKSRC_STS_GPIO2) |
			(mask_wksrc << CS40L26_IRQ1_WKSRC_STS_GPIO3) |
			(mask_wksrc << CS40L26_IRQ1_WKSRC_STS_GPIO4);

	mask = BIT(CS40L26_IRQ1_WKSRC_STS_ANY) |
			BIT(CS40L26_IRQ1_WKSRC_STS_GPIO1) |
			BIT(CS40L26_IRQ1_WKSRC_STS_I2C) |
			BIT(CS40L26_IRQ1_WKSRC_STS_SPI) |
			BIT(CS40L26_IRQ1_WKSRC_STS_GPIO2) |
			BIT(CS40L26_IRQ1_WKSRC_STS_GPIO3) |
			BIT(CS40L26_IRQ1_WKSRC_STS_GPIO4);

	return cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_1, val, mask);
}

static int cs40l26_gpio_config(struct cs40l26_private *cs40l26)
{
	u32 val, mask;
	u8 mask_gpio;
	int ret;

	if (cs40l26->devid == CS40L26_DEVID_A ||
			cs40l26->devid == CS40L26_DEVID_L27_A)
		mask_gpio = 1;
	else
		mask_gpio = 0;

	ret = cl_dsp_get_reg(cs40l26->dsp, "ENT_MAP_TABLE_EVENT_DATA_PACKED",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_EVENT_HANDLER_ALGO_ID,
			&cs40l26->event_map_base);
	if (ret)
		return ret;

	mask = (u32) (GENMASK(CS40L26_IRQ1_GPIO4_FALL,
			CS40L26_IRQ1_GPIO1_RISE));

	if (mask_gpio)
#if IS_ENABLED(CONFIG_GOOG_CUST)
		/* Extend the GPIO trigger mask to ignore GPIO1 falling edge */
		val = (u32) GENMASK(CS40L26_IRQ1_GPIO4_FALL,
				CS40L26_IRQ1_GPIO1_FALL);
#else
		val = (u32) GENMASK(CS40L26_IRQ1_GPIO4_FALL,
				CS40L26_IRQ1_GPIO2_RISE);
#endif
	else
		val = 0;

	ret = regmap_write(cs40l26->regmap, CS40L26_A1_EVENT_MAP_1,
			cs40l26->pdata.press_idx);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to map press GPI event\n");
		return ret;
	}

	ret = regmap_write(cs40l26->regmap, CS40L26_A1_EVENT_MAP_2,
			cs40l26->pdata.release_idx);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to map release GPI event\n");
		return ret;
	}

	return cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_1, val, mask);
}

static int cs40l26_brownout_prevention_init(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	struct regmap *regmap = cs40l26->regmap;
	u32 vpbr_atk_step = 0, vbbr_atk_step = 0;
	u32 vpbr_atk_rate = 0, vbbr_atk_rate = 0;
	u32 vpbr_rel_rate = 0, vbbr_rel_rate = 0;
	u32 vbbr_max_att = 0, vpbr_max_att = 0;
	u32 vbbr_thld = 0, vpbr_thld = 0;
	u32 vpbr_wait = 0, vbbr_wait = 0;
	u32 pseq_val = 0, pseq_mask = 0;
	u32 val;
	int ret;

	ret = regmap_read(regmap, CS40L26_BLOCK_ENABLES2, &val);
	if (ret) {
		dev_err(dev, "Failed to read block enables 2\n");
		return ret;
	}

	val |= ((cs40l26->pdata.vbbr_en << CS40L26_VBBR_EN_SHIFT)
			| (cs40l26->pdata.vpbr_en << CS40L26_VPBR_EN_SHIFT));

	ret = regmap_write(regmap, CS40L26_BLOCK_ENABLES2, val);
	if (ret) {
		dev_err(dev, "Failed to enable brownout prevention\n");
		return ret;
	}

	ret = cs40l26_pseq_write(cs40l26, CS40L26_BLOCK_ENABLES2,
			val, true, CS40L26_PSEQ_OP_WRITE_FULL);
	if (ret) {
		dev_err(dev, "Failed to sequence brownout prevention\n");
		return ret;
	}

	if (cs40l26->pdata.vbbr_en) {
		pseq_mask |= BIT(CS40L26_IRQ2_VBBR_ATT_CLR) |
				BIT(CS40L26_IRQ2_VBBR_FLAG);

		ret = regmap_read(regmap, CS40L26_VBBR_CONFIG, &val);
		if (ret) {
			dev_err(dev, "Failed to get VBBR config.\n");
			return ret;
		}

		if (cs40l26->pdata.vbbr_thld_mv) {
			if (cs40l26->pdata.vbbr_thld_mv
					>= CS40L26_VBBR_THLD_MV_MAX)
				vbbr_thld = CS40L26_VBBR_THLD_MAX;
			else if (cs40l26->pdata.vbbr_thld_mv
					<= CS40L26_VBBR_THLD_MV_MIN)
				vbbr_thld = CS40L26_VBBR_THLD_MIN;
			else
				vbbr_thld = cs40l26->pdata.vbbr_thld_mv /
					CS40L26_VBBR_THLD_MV_STEP;

			val &= ~CS40L26_VBBR_THLD_MASK;
			val |= (vbbr_thld & CS40L26_VBBR_THLD_MASK);
		}

		if (cs40l26->pdata.vbbr_max_att != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vbbr_max_att >=
					CS40L26_VXBR_MAX_ATT_MAX)
				vbbr_max_att = CS40L26_VXBR_MAX_ATT_MAX;
			else
				vbbr_max_att = cs40l26->pdata.vbbr_max_att;

			val &= ~CS40L26_VXBR_MAX_ATT_MASK;
			val |= ((vbbr_max_att << CS40L26_VXBR_MAX_ATT_SHIFT)
					& CS40L26_VXBR_MAX_ATT_MASK);
		}

		if (cs40l26->pdata.vbbr_atk_step) {
			if (cs40l26->pdata.vbbr_atk_step
					<= CS40L26_VXBR_ATK_STEP_MIN)
				vbbr_atk_step = CS40L26_VXBR_ATK_STEP_MIN;
			else if (cs40l26->pdata.vbbr_atk_step
					>= CS40L26_VXBR_ATK_STEP_MAX_DB)
				vbbr_atk_step = CS40L26_VXBR_ATK_STEP_MAX;
			else
				vbbr_atk_step = cs40l26->pdata.vbbr_atk_step;

			val &= ~CS40L26_VXBR_ATK_STEP_MASK;
			val |= ((vbbr_atk_step << CS40L26_VXBR_ATK_STEP_SHIFT)
					& CS40L26_VXBR_ATK_STEP_MASK);
		}

		if (cs40l26->pdata.vbbr_atk_rate !=
				CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vbbr_atk_rate
					> CS40L26_VXBR_ATK_RATE_MAX)
				vbbr_atk_rate = CS40L26_VXBR_ATK_RATE_MAX;
			else
				vbbr_atk_rate = cs40l26->pdata.vbbr_atk_rate;

			val &= ~CS40L26_VXBR_ATK_RATE_MASK;
			val |= ((vbbr_atk_rate << CS40L26_VXBR_ATK_RATE_SHIFT)
					& CS40L26_VXBR_ATK_RATE_MASK);
		}

		if (cs40l26->pdata.vbbr_wait != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vbbr_wait > CS40L26_VXBR_WAIT_MAX)
				vbbr_wait = CS40L26_VXBR_WAIT_MAX;
			else
				vbbr_wait = cs40l26->pdata.vbbr_wait;

			val &= ~CS40L26_VXBR_WAIT_MASK;
			val |= ((vbbr_wait << CS40L26_VXBR_WAIT_SHIFT)
					& CS40L26_VXBR_WAIT_MASK);
		}

		if (cs40l26->pdata.vbbr_rel_rate != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vbbr_rel_rate
					> CS40L26_VXBR_REL_RATE_MAX)
				vbbr_rel_rate = CS40L26_VXBR_REL_RATE_MAX;
			else
				vbbr_rel_rate = cs40l26->pdata.vbbr_rel_rate;

			val &= ~CS40L26_VXBR_REL_RATE_MASK;
			val |= ((vbbr_rel_rate << CS40L26_VXBR_REL_RATE_SHIFT)
					& CS40L26_VXBR_REL_RATE_MASK);
		}

		ret = regmap_write(regmap, CS40L26_VBBR_CONFIG, val);
		if (ret) {
			dev_err(dev, "Failed to write VBBR config.\n");
			return ret;
		}

		ret = cs40l26_pseq_write(cs40l26, CS40L26_VBBR_CONFIG,
				(val & GENMASK(31, 16)) >> 16,
				true, CS40L26_PSEQ_OP_WRITE_H16);
		if (ret)
			return ret;

		ret = cs40l26_pseq_write(cs40l26, CS40L26_VBBR_CONFIG,
				(val & GENMASK(15, 0)),
				true, CS40L26_PSEQ_OP_WRITE_L16);
		if (ret)
			return ret;
	}

	ret = regmap_read(regmap, CS40L26_VPBR_CONFIG, &val);
	if (ret) {
		dev_err(dev, "Failed to get VBBR config.\n");
		return ret;
	}

	if (cs40l26->pdata.vpbr_en) {
		pseq_mask |= BIT(CS40L26_IRQ2_VPBR_ATT_CLR) |
				BIT(CS40L26_IRQ2_VPBR_FLAG);

		if (cs40l26->pdata.vpbr_thld_mv) {
			if (cs40l26->pdata.vpbr_thld_mv
					>= CS40L26_VPBR_THLD_MV_MAX) {
				vpbr_thld = CS40L26_VPBR_THLD_MAX;
			} else if (cs40l26->pdata.vpbr_thld_mv
					<= CS40L26_VPBR_THLD_MV_MIN) {
				vpbr_thld = CS40L26_VPBR_THLD_MIN;
			} else {
				vpbr_thld = (cs40l26->pdata.vpbr_thld_mv /
						CS40L26_VPBR_THLD_MV_DIV)
						- CS40L26_VPBR_THLD_OFFSET;
			}

			cs40l26->vpbr_thld = vpbr_thld & CS40L26_VPBR_THLD_MASK;

			val &= ~CS40L26_VPBR_THLD_MASK;
			val |= (vpbr_thld & CS40L26_VPBR_THLD_MASK);

		}

		if (cs40l26->pdata.vpbr_max_att != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vpbr_max_att >=
					CS40L26_VXBR_MAX_ATT_MAX)
				vpbr_max_att = CS40L26_VXBR_MAX_ATT_MAX;
			else
				vpbr_max_att = cs40l26->pdata.vpbr_max_att;

			val &= ~CS40L26_VXBR_MAX_ATT_MASK;
			val |= ((vpbr_max_att << CS40L26_VXBR_MAX_ATT_SHIFT)
					& CS40L26_VXBR_MAX_ATT_MASK);
		}

		if (cs40l26->pdata.vpbr_atk_step) {
			if (cs40l26->pdata.vpbr_atk_step
					<= CS40L26_VXBR_ATK_STEP_MIN)
				vpbr_atk_step = CS40L26_VXBR_ATK_STEP_MIN;
			else if (cs40l26->pdata.vpbr_atk_step
					>= CS40L26_VXBR_ATK_STEP_MAX_DB)
				vpbr_atk_step = CS40L26_VXBR_ATK_STEP_MAX;
			else
				vpbr_atk_step = cs40l26->pdata.vpbr_atk_step;

			val &= ~CS40L26_VXBR_ATK_STEP_MASK;
			val |= ((vpbr_atk_step << CS40L26_VXBR_ATK_STEP_SHIFT)
					& CS40L26_VXBR_ATK_STEP_MASK);
		}

		if (cs40l26->pdata.vpbr_atk_rate !=
				CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vpbr_atk_rate
					> CS40L26_VXBR_ATK_RATE_MAX)
				vpbr_atk_rate = CS40L26_VXBR_ATK_RATE_MAX;
			else
				vpbr_atk_rate = cs40l26->pdata.vpbr_atk_rate;

			val &= ~CS40L26_VXBR_ATK_RATE_MASK;
			val |= ((vpbr_atk_rate << CS40L26_VXBR_ATK_RATE_SHIFT)
					& CS40L26_VXBR_ATK_RATE_MASK);

		}

		if (cs40l26->pdata.vpbr_wait != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vpbr_wait > CS40L26_VXBR_WAIT_MAX)
				vpbr_wait = CS40L26_VXBR_WAIT_MAX;
			else
				vpbr_wait = cs40l26->pdata.vpbr_wait;

			val &= ~CS40L26_VXBR_WAIT_MASK;
			val |= ((vpbr_wait << CS40L26_VXBR_WAIT_SHIFT)
					& CS40L26_VXBR_WAIT_MASK);
		}

		if (cs40l26->pdata.vpbr_rel_rate != CS40L26_VXBR_DEFAULT) {
			if (cs40l26->pdata.vpbr_rel_rate
					> CS40L26_VXBR_REL_RATE_MAX)
				vpbr_rel_rate = CS40L26_VXBR_REL_RATE_MAX;
			else
				vpbr_rel_rate = cs40l26->pdata.vpbr_rel_rate;

			val &= ~CS40L26_VXBR_REL_RATE_MASK;
			val |= ((vpbr_rel_rate << CS40L26_VXBR_REL_RATE_SHIFT)
					& CS40L26_VXBR_REL_RATE_MASK);
		}

		ret = regmap_write(regmap, CS40L26_VPBR_CONFIG, val);
		if (ret) {
			dev_err(dev, "Failed to write VPBR config.\n");
			return ret;
		}

		ret = cs40l26_pseq_write(cs40l26, CS40L26_VPBR_CONFIG,
				(val & GENMASK(31, 16)) >> 16,
				true, CS40L26_PSEQ_OP_WRITE_H16);
		if (ret)
			return ret;

		ret = cs40l26_pseq_write(cs40l26, CS40L26_VPBR_CONFIG,
				(val & GENMASK(15, 0)),
				true, CS40L26_PSEQ_OP_WRITE_L16);
		if (ret)
			return ret;
	}

	return cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_2,
			pseq_val, pseq_mask);
}

static int cs40l26_asp_config(struct cs40l26_private *cs40l26)
{
	struct reg_sequence *dsp1rx_config =
			kcalloc(2, sizeof(struct reg_sequence), GFP_KERNEL);
	int ret;

	if (!dsp1rx_config) {
		dev_err(cs40l26->dev, "Failed to allocate reg. sequence\n");
		return -ENOMEM;
	}

	dsp1rx_config[0].reg = CS40L26_DSP1RX1_INPUT;
	dsp1rx_config[0].def = CS40L26_DATA_SRC_ASPRX1;
	dsp1rx_config[1].reg = CS40L26_DSP1RX5_INPUT;
	dsp1rx_config[1].def = CS40L26_DATA_SRC_ASPRX2;

	ret = regmap_multi_reg_write(cs40l26->regmap, dsp1rx_config, 2);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to configure ASP\n");
		goto err_free;
	}

	ret = cs40l26_pseq_multi_write(cs40l26, dsp1rx_config, 2, true,
			CS40L26_PSEQ_OP_WRITE_L16);

err_free:
	kfree(dsp1rx_config);

	return ret;
}

static int cs40l26_bst_dcm_config(struct cs40l26_private *cs40l26)
{
	int ret = 0;
	u32 val;

	if (cs40l26->pdata.bst_dcm_en != CS40L26_BST_DCM_EN_DEFAULT) {
		ret = regmap_read(cs40l26->regmap, CS40L26_BST_DCM_CTL, &val);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to read BST_DCM_CTL\n");
			return ret;
		}

		val &= ~CS40L26_BST_DCM_EN_MASK;
		val |= cs40l26->pdata.bst_dcm_en << CS40L26_BST_DCM_EN_SHIFT;

		ret = regmap_write(cs40l26->regmap, CS40L26_BST_DCM_CTL, val);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to write BST_DCM_CTL\n");
			return ret;
		}

		ret = cs40l26_pseq_write(cs40l26, CS40L26_BST_DCM_CTL,
				val, true, CS40L26_PSEQ_OP_WRITE_FULL);
	}

	return ret;
}

static int cs40l26_zero_cross_config(struct cs40l26_private *cs40l26)
{
	int ret = 0;
	u32 reg;

	if (cs40l26->pdata.pwle_zero_cross) {
		ret = cl_dsp_get_reg(cs40l26->dsp, "PWLE_EXTEND_ZERO_CROSS",
				CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (ret)
			return ret;

		ret = regmap_write(cs40l26->regmap, reg, 1);
		if (ret)
			dev_err(cs40l26->dev, "Failed to set PWLE_EXTEND_ZERO_CROSS\n");

	}

	return ret;
}

static int calib_device_tree_config(struct cs40l26_private *cs40l26)
{
	int ret = 0;
	u32 reg, bst_ctl, bst_ctl_cfg;

	if (cs40l26->pdata.f0_default <= CS40L26_F0_EST_MAX &&
			cs40l26->pdata.f0_default >= CS40L26_F0_EST_MIN) {
		ret = cl_dsp_get_reg(cs40l26->dsp, "F0_OTP_STORED",
				CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (ret)
			return ret;

		ret = regmap_write(cs40l26->regmap, reg,
				cs40l26->pdata.f0_default);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to write default f0\n");
			return ret;
		}
	}

	if (cs40l26->pdata.redc_default && cs40l26->pdata.redc_default <=
			CS40L26_UINT_24_BITS_MAX) {
		ret = cl_dsp_get_reg(cs40l26->dsp, "REDC_OTP_STORED",
				CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (ret)
			return ret;

		ret = regmap_write(cs40l26->regmap, reg,
				cs40l26->pdata.redc_default);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to write default ReDC\n");
			return ret;
		}
	}

	if (cs40l26->pdata.q_default <= CS40L26_Q_EST_MAX &&
			cs40l26->pdata.q_default >= CS40L26_Q_EST_MIN) {
		ret = cl_dsp_get_reg(cs40l26->dsp, "Q_STORED",
				CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (ret)
			return ret;

		ret = regmap_write(cs40l26->regmap, reg,
				cs40l26->pdata.q_default);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to write default Q\n");
			return ret;
		}
	}

	if (cs40l26->pdata.boost_ctl <= CS40L26_BST_VOLT_MAX &&
			cs40l26->pdata.boost_ctl >= CS40L26_BST_VOLT_MIN) {
		bst_ctl = ((cs40l26->pdata.boost_ctl - CS40L26_BST_VOLT_MIN)
						/ CS40L26_BST_VOLT_STEP) + 1;

		ret = regmap_write(cs40l26->regmap, CS40L26_VBST_CTL_1,
				bst_ctl);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to write VBST limit\n");
			return ret;
		}

		ret = cs40l26_pseq_write(cs40l26, CS40L26_VBST_CTL_1, bst_ctl,
				true, CS40L26_PSEQ_OP_WRITE_L16);
		if (ret)
			return ret;

		ret = regmap_read(cs40l26->regmap, CS40L26_VBST_CTL_2,
				&bst_ctl_cfg);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to get VBST config\n");
			return ret;
		}

		bst_ctl_cfg |= (1 << CS40L26_BST_CTL_LIM_EN_SHIFT);

		ret = regmap_write(cs40l26->regmap, CS40L26_VBST_CTL_2,
				bst_ctl_cfg);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to write VBST config\n");
			return ret;
		}

		ret = cs40l26_pseq_write(cs40l26, CS40L26_VBST_CTL_2,
				bst_ctl_cfg, true, CS40L26_PSEQ_OP_WRITE_FULL);
	}

	return ret;
}

static int cs40l26_bst_ipk_config(struct cs40l26_private *cs40l26)
{
	u32 val, bst_ipk_ma = cs40l26->pdata.bst_ipk / MILLIAMPS_PER_AMPS;
	int ret;

	if (bst_ipk_ma < CS40L26_BST_IPK_MILLIAMP_MIN ||
			bst_ipk_ma > CS40L26_BST_IPK_MILLIAMP_MAX) {
		val = CS40L26_BST_IPK_DEFAULT;
		dev_dbg(cs40l26->dev, "Using default BST_IPK\n");
	} else {
		val = (bst_ipk_ma / CS40L26_BST_IPK_CTL_STEP_SIZE) -
				CS40L26_BST_IPK_CTL_RESERVED;
	}

	ret = regmap_write(cs40l26->regmap, CS40L26_BST_IPK_CTL, val);
	if (ret) {
		dev_err(cs40l26->dev, "Failed to update BST peak current\n");
		return ret;
	}

	ret = cs40l26_pseq_write(cs40l26, CS40L26_BST_IPK_CTL, val,
			true, CS40L26_PSEQ_OP_WRITE_L16);
	if (ret)
		return ret;

	return cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_1, 0,
			BIT(CS40L26_IRQ1_BST_IPK_FLAG));
}

static int cs40l26_lbst_short_test(struct cs40l26_private *cs40l26)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int err;
	int ret;

	ret = regmap_update_bits(regmap, CS40L26_VBST_CTL_2,
			CS40L26_BST_CTL_SEL_MASK, CS40L26_BST_CTL_SEL_FIXED);
	if (ret) {
		dev_err(dev, "Failed to set VBST_CTL_2\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L26_VBST_CTL_1,
			CS40L26_BST_CTL_MASK, CS40L26_BST_CTL_VP);
	if (ret) {
		dev_err(dev, "Failed to set VBST_CTL_1\n");
		return ret;
	}

	/* Set GLOBAL_EN; safe because DSP is guaranteed to be off here */
	ret = regmap_update_bits(regmap, CS40L26_GLOBAL_ENABLES,
			CS40L26_GLOBAL_EN_MASK, 1);
	if (ret) {
		dev_err(dev, "Failed to set GLOBAL_EN\n");
		return ret;
	}

	/* Wait until boost converter is guaranteed to be powered up */
	usleep_range(CS40L26_BST_TIME_MIN_US, CS40L26_BST_TIME_MAX_US);

	ret = regmap_read(regmap, CS40L26_ERROR_RELEASE, &err);
	if (ret) {
		dev_err(dev, "Failed to get ERROR_RELEASE contents\n");
		return ret;
	}

	if (err & BIT(CS40L26_BST_SHORT_ERR_RLS)) {
		dev_err(dev, "FATAL: Boost shorted at startup\n");
		return ret;
	}

	/* Clear GLOBAL_EN; safe because DSP is guaranteed to be off here */
	ret = regmap_update_bits(regmap, CS40L26_GLOBAL_ENABLES,
			CS40L26_GLOBAL_EN_MASK, 0);
	if (ret) {
		dev_err(dev, "Failed to clear GLOBAL_EN\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L26_VBST_CTL_2,
			CS40L26_BST_CTL_SEL_MASK, CS40L26_BST_CTL_SEL_CLASS_H);
	if (ret) {
		dev_err(dev, "Failed to set VBST_CTL_2\n");
		return ret;
	}

	ret = regmap_update_bits(regmap, CS40L26_VBST_CTL_1,
			CS40L26_BST_CTL_MASK, CS40L26_BST_CTL_VP);
	if (ret)
		dev_err(dev, "Failed to set VBST_CTL_1\n");

	return ret;
}

static int cs40l26_handle_errata(struct cs40l26_private *cs40l26)
{
	int ret, num_writes;

	if (!cs40l26->pdata.expl_mode_enabled) {
		ret = cs40l26_lbst_short_test(cs40l26);
		if (ret)
			return ret;

		num_writes = CS40L26_ERRATA_A1_NUM_WRITES;
	} else {
		num_writes = CS40L26_ERRATA_A1_EXPL_EN_NUM_WRITES;
	}

	return cs40l26_pseq_multi_write(cs40l26, cs40l26_a1_errata, num_writes,
			false, CS40L26_PSEQ_OP_WRITE_FULL);
}

int cs40l26_dbc_enable(struct cs40l26_private *cs40l26, u32 enable)
{
	unsigned int reg;
	int ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "FLAGS", CL_DSP_XM_UNPACKED_TYPE,
			CS40L26_EXT_ALGO_ID, &reg);
	if (ret)
		return ret;

	ret = regmap_update_bits(cs40l26->regmap, reg, CS40L26_DBC_ENABLE_MASK,
			enable << CS40L26_DBC_ENABLE_SHIFT);
	if (ret)
		dev_err(cs40l26->dev, "Failed to %s DBC\n",
				(enable == 1) ? "enable" : "disable");

	return ret;
}
EXPORT_SYMBOL(cs40l26_dbc_enable);

static int cs40l26_handle_dbc_defaults(struct cs40l26_private *cs40l26)
{
	unsigned int i;
	u32 val;
	int ret;

	for (i = 0; i < CS40L26_DBC_NUM_CONTROLS; i++) {
		val = cs40l26->pdata.dbc_defaults[i];

		if (val != CS40L26_DBC_USE_DEFAULT) {
			ret = cs40l26_dbc_set(cs40l26, i, val);
			if (ret)
				return ret;
		}
	}

	if (cs40l26->pdata.dbc_enable_default) {
		ret = cs40l26_dbc_enable(cs40l26, 1);
		if (ret)
			return ret;
	}

	return 0;
}

static int cs40l26_dsp_config(struct cs40l26_private *cs40l26)
{
	struct regmap *regmap = cs40l26->regmap;
	struct device *dev = cs40l26->dev;
	unsigned int val;
	u32 reg, nwaves, value;
	int ret;

	ret = regmap_update_bits(regmap, CS40L26_PWRMGT_CTL,
			CS40L26_MEM_RDY_MASK, 1 << CS40L26_MEM_RDY_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to set MEM_RDY to initialize RAM\n");
		return ret;
	}

	ret = cl_dsp_get_reg(cs40l26->dsp, "CALL_RAM_INIT",
			CL_DSP_XM_UNPACKED_TYPE, cs40l26->fw_id, &reg);
	if (ret)
		return ret;

	ret = cs40l26_dsp_write(cs40l26, reg, 1);
	if (ret)
		return ret;

	cs40l26->fw_loaded = true;

#ifdef CONFIG_DEBUG_FS
	cs40l26_debugfs_init(cs40l26);
#endif

	ret = cs40l26_pseq_init(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_update_reg_defaults_via_pseq(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_handle_errata(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_dsp_start(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_PREVENT_HIBERNATE);
	if (ret)
		return ret;

	/* ensure firmware running */
	ret = cl_dsp_get_reg(cs40l26->dsp, "HALO_STATE",
			     CL_DSP_XM_UNPACKED_TYPE, cs40l26->fw_id, &reg);
	if (ret)
		return ret;

	ret = regmap_read(regmap, reg, &val);
	if (ret) {
		dev_err(dev, "Failed to read HALO_STATE\n");
		return ret;
	}

	if (val != CS40L26_DSP_HALO_STATE_RUN) {
		dev_err(dev, "Firmware in unexpected state: 0x%X\n", val);
		return ret;
	}

	ret = cs40l26_irq_update_mask(cs40l26, CS40L26_IRQ1_MASK_1, 0,
			BIT(CS40L26_IRQ1_AMP_ERR) | BIT(CS40L26_IRQ1_TEMP_ERR) |
			BIT(CS40L26_IRQ1_BST_SHORT_ERR) |
			BIT(CS40L26_IRQ1_BST_DCM_UVP_ERR) |
			BIT(CS40L26_IRQ1_BST_OVP_ERR) |
			BIT(CS40L26_IRQ1_VIRTUAL2_MBOX_WR));
	if (ret)
		return ret;

	ret = cs40l26_wksrc_config(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_gpio_config(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_bst_dcm_config(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_bst_ipk_config(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_handle_dbc_defaults(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_zero_cross_config(cs40l26);
	if (ret)
		return ret;

	if (!cs40l26->vibe_init_success) {
		ret = calib_device_tree_config(cs40l26);
		if (ret)
			return ret;
	}

	ret = cs40l26_brownout_prevention_init(cs40l26);
	if (ret)
		return ret;

	cs40l26_pm_runtime_setup(cs40l26);

	ret = cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_ALLOW_HIBERNATE);
	if (ret)
		return ret;

	ret = cs40l26_pm_enter(dev);
	if (ret)
		return ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "TIMEOUT_MS",
			CL_DSP_XM_UNPACKED_TYPE, CS40L26_VIBEGEN_ALGO_ID, &reg);
	if (ret)
		goto pm_err;

	ret = regmap_write(regmap, reg, 0);
	if (ret) {
		dev_err(dev, "Failed to set TIMEOUT_MS\n");
		goto pm_err;
	}

	ret = cs40l26_asp_config(cs40l26);
	if (ret)
		goto pm_err;

	ret = cs40l26_get_num_waves(cs40l26, &nwaves);
	if (ret)
		goto pm_err;

	dev_info(dev, "%s loaded with %u RAM waveforms\n", CS40L26_DEV_NAME,
			nwaves);

	cs40l26->num_owt_effects = 0;

	value = (cs40l26->comp_enable_redc << CS40L26_COMP_EN_REDC_SHIFT) |
			(cs40l26->comp_enable_f0 << CS40L26_COMP_EN_F0_SHIFT);

	if (cs40l26->fw_id != CS40L26_FW_CALIB_ID) {
		ret = cl_dsp_get_reg(cs40l26->dsp, "COMPENSATION_ENABLE",
				CL_DSP_XM_UNPACKED_TYPE,
				CS40L26_VIBEGEN_ALGO_ID, &reg);
		if (ret)
			goto pm_err;

		ret = regmap_write(cs40l26->regmap, reg, value);
		if (ret)
			dev_err(dev, "Failed to configure compensation\n");
	}

pm_err:
	cs40l26_pm_exit(dev);

	return ret;
}

static void cs40l26_gain_adjust(struct cs40l26_private *cs40l26, s32 adjust)
{
	u16 total, asp, change;

	asp = cs40l26->pdata.asp_scale_pct;

	if (adjust < 0) {
		change = (u16) ((adjust * -1) & 0xFFFF);
		if (asp < change)
			total = 0;
		else
			total = asp - change;
	} else {
		change = (u16) (adjust & 0xFFFF);
		total = asp + change;
		if (total > CS40L26_GAIN_FULL_SCALE)
			total = CS40L26_GAIN_FULL_SCALE;
	}

	cs40l26->pdata.asp_scale_pct = total;
}

int cs40l26_svc_le_estimate(struct cs40l26_private *cs40l26, unsigned int *le)
{
	struct device *dev = cs40l26->dev;
	unsigned int reg, le_est = 0;
	int ret, i;

	ret = cs40l26_ack_write(cs40l26, CS40L26_DSP_VIRTUAL1_MBOX_1,
			CS40L26_DSP_MBOX_CMD_LE_EST, CS40L26_DSP_MBOX_RESET);
	if (ret)
		return ret;

	ret = cl_dsp_get_reg(cs40l26->dsp, "LE_EST_STATUS",
			     CL_DSP_YM_UNPACKED_TYPE, CS40L26_SVC_ALGO_ID,
			     &reg);
	if (ret)
		return ret;

	for (i = 0; i < CS40L26_SVC_LE_MAX_ATTEMPTS; i++) {
		usleep_range(CS40L26_SVC_LE_EST_TIME_US,
				CS40L26_SVC_LE_EST_TIME_US + 100);
		ret = regmap_read(cs40l26->regmap, reg, &le_est);
		if (ret) {
			dev_err(dev, "Failed to get LE_EST_STATUS\n");
			return ret;
		}

		dev_info(dev, "Measured Le Estimation = %u\n", le_est);

		if (le_est)
			break;
	}

	*le = le_est;

	return 0;
}
EXPORT_SYMBOL(cs40l26_svc_le_estimate);

static int cs40l26_tuning_select_from_svc_le(struct cs40l26_private *cs40l26,
		unsigned int le, u32 *tuning_num)
{
	int ret = 0;
	int i;

	if (le) {
		for (i = 0; i < cs40l26->num_svc_le_vals; i++) {
			if (le >= cs40l26->svc_le_vals[i]->min &&
					le <= cs40l26->svc_le_vals[i]->max) {
				*tuning_num = cs40l26->svc_le_vals[i]->n;

				cs40l26_gain_adjust(cs40l26,
					cs40l26->svc_le_vals[i]->gain_adjust);
				break;
			}
		}
	}

	if (!le || i == cs40l26->num_svc_le_vals)
		dev_warn(cs40l26->dev, "Using default tunings\n");

	return ret;
}

static char **cs40l26_get_tuning_names(struct cs40l26_private *cs40l26,
					int *actual_num_files, u32 tuning)
{
	char **coeff_files;
	int i, file_count = 0;

	coeff_files = kcalloc(
			CS40L26_MAX_TUNING_FILES, sizeof(char *), GFP_KERNEL);
	if (!coeff_files)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < CS40L26_MAX_TUNING_FILES; i++) {
		coeff_files[i] =
			kzalloc(CS40L26_TUNING_FILE_NAME_MAX_LEN, GFP_KERNEL);
		if (!coeff_files[i])
			goto err_free;
	}

	if (tuning) {
		snprintf(coeff_files[file_count++],
			CS40L26_TUNING_FILE_NAME_MAX_LEN, "%s%d%s",
			CS40L26_WT_FILE_PREFIX, tuning,
			CS40L26_TUNING_FILE_SUFFIX);
	} else {
		strscpy(coeff_files[file_count++],
			CS40L26_WT_FILE_NAME,
			CS40L26_TUNING_FILE_NAME_MAX_LEN);
	}

	if (tuning) {
		snprintf(coeff_files[file_count++],
			CS40L26_TUNING_FILE_NAME_MAX_LEN, "%s%d%s",
			CS40L26_SVC_TUNING_FILE_PREFIX, tuning,
			CS40L26_TUNING_FILE_SUFFIX);
	} else {
		strscpy(coeff_files[file_count++],
			CS40L26_SVC_TUNING_FILE_NAME,
			CS40L26_TUNING_FILE_NAME_MAX_LEN);
	}
	if (cl_dsp_algo_is_present(cs40l26->dsp, CS40L26_LF0T_ALGO_ID))
		strscpy(coeff_files[file_count++],
			CS40L26_LF0T_FILE_NAME,
			CS40L26_TUNING_FILE_NAME_MAX_LEN);

	if (cl_dsp_algo_is_present(cs40l26->dsp, CS40L26_DVL_ALGO_ID))
		strscpy(coeff_files[file_count++],
			CS40L26_DVL_FILE_NAME,
			CS40L26_TUNING_FILE_NAME_MAX_LEN);

	if (cs40l26->fw_id == CS40L26_FW_ID) {
		if (cl_dsp_algo_is_present(cs40l26->dsp, CS40L26_A2H_ALGO_ID))
			strscpy(coeff_files[file_count++],
				CS40L26_A2H_TUNING_FILE_NAME,
				CS40L26_TUNING_FILE_NAME_MAX_LEN);
	} else {
		strscpy(coeff_files[file_count++],
			CS40L26_CALIB_BIN_FILE_NAME,
			CS40L26_TUNING_FILE_NAME_MAX_LEN);
	}

	*actual_num_files = file_count;
	return coeff_files;

err_free:
	for (; i >= 0; i--)
		kfree(coeff_files[i]);
	kfree(coeff_files);
	*actual_num_files = 0;
	return ERR_PTR(-ENOMEM);
}

static int cs40l26_coeff_load(struct cs40l26_private *cs40l26, u32 tuning)
{
	struct device *dev = cs40l26->dev;
	const struct firmware *coeff;
	int i, ret, num_files_to_load;
	char **coeff_files;

	coeff_files = cs40l26_get_tuning_names(
					cs40l26, &num_files_to_load, tuning);
	if (IS_ERR(coeff_files))
		return PTR_ERR(coeff_files);

	for (i = 0; i < num_files_to_load; i++) {
		ret = request_firmware(&coeff, coeff_files[i], dev);
		if (ret) {
			dev_warn(dev, "Continuing...\n");
			continue;
		}

		ret = cl_dsp_coeff_file_parse(cs40l26->dsp, coeff);
		if (ret)
			dev_warn(dev, "Failed to load %s, %d. Continuing...\n",
					coeff_files[i], ret);
		else
			dev_info(dev, "%s Loaded Successfully\n",
					coeff_files[i]);

		release_firmware(coeff);
	}

	kfree(coeff_files);

	return 0;
}

static int cs40l26_change_fw_control_defaults(struct cs40l26_private *cs40l26)
{
	int ret;

	ret = cs40l26_pm_stdby_timeout_ms_set(cs40l26,
			cs40l26->pdata.pm_stdby_timeout_ms);
	if (ret)
		return ret;

	return cs40l26_pm_active_timeout_ms_set(cs40l26,
			cs40l26->pdata.pm_active_timeout_ms);
}

static int cs40l26_get_fw_params(struct cs40l26_private *cs40l26)
{
	u32 id, min_rev, rev, branch;
	int ret, maj, min, patch;

	ret = cl_dsp_fw_rev_get(cs40l26->dsp, &rev);
	if (ret)
		return ret;

	branch = CL_DSP_GET_MAJOR(rev);
	maj = (int) branch;
	min = (int) CL_DSP_GET_MINOR(rev);
	patch = (int) CL_DSP_GET_PATCH(rev);

	ret = cl_dsp_fw_id_get(cs40l26->dsp, &id);
	if (ret)
		return ret;

	switch (id) {
	case CS40L26_FW_ID:
		if (branch == CS40L26_FW_BRANCH) {
			min_rev = CS40L26_FW_MIN_REV;
			cs40l26->vibe_state_reporting = true;
		} else if (branch == CS40L26_FW_MAINT_BRANCH) {
			min_rev = CS40L26_FW_MAINT_MIN_REV;
			cs40l26->vibe_state_reporting = false;
		} else {
			ret = -EINVAL;
		}
		break;
	case CS40L26_FW_CALIB_ID:
		if (branch == CS40L26_FW_CALIB_BRANCH) {
			min_rev = CS40L26_FW_CALIB_MIN_REV;
			cs40l26->vibe_state_reporting = true;
		} else if (branch == CS40L26_FW_MAINT_CALIB_BRANCH) {
			min_rev = CS40L26_FW_MAINT_CALIB_MIN_REV;
			cs40l26->vibe_state_reporting = false;
		} else {
			ret = -EINVAL;
		}
		break;
	default:
		dev_err(cs40l26->dev, "Invalid FW ID: 0x%06X\n", id);
		return -EINVAL;
	}

	if (ret) {
		dev_err(cs40l26->dev, "Rev. Branch 0x%02X invalid\n", maj);
		return ret;
	}

	if (rev < min_rev) {
		dev_err(cs40l26->dev, "Invalid firmware revision: %d.%d.%d\n",
				maj, min, patch);
		return -EINVAL;
	}

	cs40l26->fw_id = id;

	dev_info(cs40l26->dev, "Firmware revision %d.%d.%d\n", maj, min, patch);

	return 0;
}

static int cs40l26_cl_dsp_reinit(struct cs40l26_private *cs40l26)
{
	int ret;

	if (cs40l26->dsp) {
		ret = cl_dsp_destroy(cs40l26->dsp);
		if (ret) {
			dev_err(cs40l26->dev, "Failed to destroy DSP struct\n");
			return ret;
		}

		cs40l26->dsp = NULL;
	}

	cs40l26->dsp = cl_dsp_create(cs40l26->dev, cs40l26->regmap);
	if (IS_ERR(cs40l26->dsp))
		return PTR_ERR(cs40l26->dsp);

	return cl_dsp_wavetable_create(cs40l26->dsp, CS40L26_VIBEGEN_ALGO_ID,
		CS40L26_WT_NAME_XM, CS40L26_WT_NAME_YM, CS40L26_WT_FILE_NAME);
}

static int cs40l26_fw_upload(struct cs40l26_private *cs40l26)
{
	bool svc_le_required = cs40l26->num_svc_le_vals && !cs40l26->calib_fw;
	struct device *dev = cs40l26->dev;
	u32 rev, branch, tuning_num = 0;
	const struct firmware *fw;
	int ret;
	unsigned int le = 0;

	cs40l26->fw_loaded = false;

	ret = cs40l26_cl_dsp_reinit(cs40l26);
	if (ret)
		return ret;

	if (cs40l26->calib_fw)
		ret = request_firmware(&fw, CS40L26_FW_CALIB_NAME, dev);
	else
		ret = request_firmware(&fw, CS40L26_FW_FILE_NAME, dev);

	if (ret) {
		release_firmware(fw);
		return ret;
	}

	ret = cs40l26_dsp_pre_config(cs40l26);
	if (ret)
		return ret;

	ret = cl_dsp_firmware_parse(cs40l26->dsp, fw, true);
	release_firmware(fw);
	if (ret)
		return ret;

	ret = cs40l26_change_fw_control_defaults(cs40l26);
	if (ret)
		return ret;

	ret = cs40l26_get_fw_params(cs40l26);
	if (ret)
		return ret;

	if (svc_le_required) {
		ret = cl_dsp_fw_rev_get(cs40l26->dsp, &rev);
		if (ret)
			return ret;

		branch = CL_DSP_GET_MAJOR(rev);

		switch (branch) {
		case CS40L26_FW_MAINT_BRANCH:
			ret = cs40l26_dsp_config(cs40l26);
			if (ret)
				return ret;

			ret = cs40l26_pm_enter(dev);
			if (ret)
				return ret;

			ret = cs40l26_svc_le_estimate(cs40l26, &le);
			if (ret)
				dev_warn(dev, "svc_le est failed, %d", ret);

			cs40l26_pm_exit(dev);

			cs40l26_pm_runtime_teardown(cs40l26);

			ret = cs40l26_dsp_pre_config(cs40l26);
			if (ret)
				return ret;

			break;

		case CS40L26_FW_BRANCH:
			le = cs40l26->svc_le_est_stored;
			break;

		default:
			dev_err(dev, "Invalid firmware branch, %d", branch);
			return -EINVAL;
		}

		ret = cs40l26_tuning_select_from_svc_le(cs40l26,
							le, &tuning_num);
		if (ret)
			return ret;
	}

	ret = cs40l26_coeff_load(cs40l26, tuning_num);
	if (ret)
		return ret;

	return cs40l26_dsp_config(cs40l26);
}

int cs40l26_fw_swap(struct cs40l26_private *cs40l26, const u32 id)
{
	struct device *dev = cs40l26->dev;
	bool re_enable = false;
	int ret = 0;

	if (cs40l26->fw_loaded) {
		disable_irq(cs40l26->irq);
		cs40l26_pm_runtime_teardown(cs40l26);
		re_enable = true;
	}

	switch (cs40l26->revid) {
	case CS40L26_REVID_A1:
	case CS40L26_REVID_B0:
	case CS40L26_REVID_B1:
		break;
	default:
		dev_err(dev, "pseq unrecognized revid: %d\n", cs40l26->revid);
		return -EINVAL;
	}

	/* reset pseq END_OF_SCRIPT to location from ROM */
	ret = cs40l26_dsp_write(cs40l26, CS40L26_PSEQ_ROM_END_OF_SCRIPT,
			CS40L26_PSEQ_OP_END << CS40L26_PSEQ_OP_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to reset pseq END_OF_SCRIPT %d\n", ret);
		return ret;
	}

	if (id == CS40L26_FW_CALIB_ID)
		cs40l26->calib_fw = true;
	else
		cs40l26->calib_fw = false;

	ret = cs40l26_fw_upload(cs40l26);
	if (ret)
		return ret;

	if (cs40l26->fw_defer && cs40l26->fw_loaded) {
		ret = devm_request_threaded_irq(dev, cs40l26->irq, NULL,
				cs40l26_irq, IRQF_ONESHOT | IRQF_SHARED |
				IRQF_TRIGGER_LOW, "cs40l26", cs40l26);
		if (ret) {
			dev_err(dev, "Failed to request threaded IRQ: %d\n",
					ret);
			return ret;
		}

		cs40l26->fw_defer = false;
	}

	if (re_enable)
		enable_irq(cs40l26->irq);

	return ret;
}
EXPORT_SYMBOL(cs40l26_fw_swap);

static int cs40l26_handle_svc_le_nodes(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	int i, ret = 0, init_count, node_count = 0;
	struct fwnode_handle *child;
	unsigned int min, max, index;
	const char *gain_adjust_str;
	const char *node_name;
	s32 gain_adjust;

	init_count = device_get_child_node_count(dev);
	if (!init_count)
		return 0;

	cs40l26->svc_le_vals = devm_kcalloc(dev, init_count,
			sizeof(struct cs40l26_svc_le *), GFP_KERNEL);

	if (!cs40l26->svc_le_vals)
		return -ENOMEM;

	device_for_each_child_node(dev, child) {
		node_name = fwnode_get_name(child);

		if (strncmp(node_name, CS40L26_SVC_DT_PREFIX, 6))
			continue;

		if (fwnode_property_read_u32(child, "cirrus,min", &min)) {
			dev_err(dev, "No minimum value for SVC LE node\n");
			continue;
		}

		if (fwnode_property_read_u32(child, "cirrus,max", &max)) {
			dev_err(dev, "No maximum value for SVC LE node\n");
			continue;
		}

		if (max <= min) {
			dev_err(dev, "Max <= Min, SVC LE node malformed\n");
			continue;
		}

		if (fwnode_property_read_string(child, "cirrus,gain-adjust",
				&gain_adjust_str)) {
			gain_adjust = 0;
		} else {
			ret = kstrtos32(gain_adjust_str, 10, &gain_adjust);
			if (ret) {
				dev_warn(dev, "Failed to get gain adjust\n");
				gain_adjust = 0;
			}
		}

		if (fwnode_property_read_u32(child, "cirrus,index", &index)) {
			dev_err(dev, "No index specified for SVC LE node\n");
			continue;
		}

		for (i = 0; i < node_count; i++) {
			if (index == cs40l26->svc_le_vals[i]->n)
				break;
		}

		if (i < node_count) {
			dev_err(dev, "SVC LE nodes must have unique index\n");
			return -EINVAL;
		}

		cs40l26->svc_le_vals[node_count] =
			devm_kzalloc(dev, sizeof(struct cs40l26_svc_le),
					GFP_KERNEL);

		if (!cs40l26->svc_le_vals[node_count]) {
			ret = -ENOMEM;
			goto err;
		}

		cs40l26->svc_le_vals[node_count]->min = min;
		cs40l26->svc_le_vals[node_count]->max = max;
		cs40l26->svc_le_vals[node_count]->gain_adjust = gain_adjust;
		cs40l26->svc_le_vals[node_count]->n = index;
		node_count++;
	}

	if (node_count != init_count)
		dev_warn(dev, "%d platform nodes unused for SVC LE\n",
				init_count - node_count);

	return node_count;

err:
	devm_kfree(dev, cs40l26->svc_le_vals);
	return ret;
}

static int cs40l26_no_wait_ram_indices_get(struct cs40l26_private *cs40l26,
		struct device_node *np)
{
	int ret, i;

	cs40l26->num_no_wait_ram_indices = of_property_count_u32_elems(np,
		"cirrus,no-wait-ram-indices");

	if (cs40l26->num_no_wait_ram_indices <= 0)
		return 0;

	cs40l26->no_wait_ram_indices = devm_kcalloc(cs40l26->dev,
			cs40l26->num_no_wait_ram_indices, sizeof(u32),
			GFP_KERNEL);
	if (!cs40l26->no_wait_ram_indices)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "cirrus,no-wait-ram-indices",
			cs40l26->no_wait_ram_indices,
			cs40l26->num_no_wait_ram_indices);
	if (ret)
		goto err_free;

	for (i = 0; i < cs40l26->num_no_wait_ram_indices; i++)
		cs40l26->no_wait_ram_indices[i] += CS40L26_RAM_INDEX_START;

	return 0;

err_free:
	devm_kfree(cs40l26->dev, cs40l26->no_wait_ram_indices);
	cs40l26->num_no_wait_ram_indices = 0;
	return ret;
}

static int cs40l26_handle_platform_data(struct cs40l26_private *cs40l26)
{
	struct device *dev = cs40l26->dev;
	struct device_node *np = dev->of_node;
	const char *str = NULL;
	int ret;
	u32 val;

	if (!np) {
		dev_err(dev, "No platform data found\n");
		return -ENOENT;
	}

	if (!of_property_read_string(np, "input-device-name", &str))
		cs40l26->pdata.device_name = str;
	else
		cs40l26->pdata.device_name = CS40L26_INPUT_DEV_NAME;

	if (of_property_read_bool(np, "cirrus,fw-defer"))
		cs40l26->fw_defer = true;

	if (of_property_read_bool(np, "cirrus,calib-fw"))
		cs40l26->calib_fw = true;

	if (of_property_read_bool(np, "cirrus,bst-expl-mode-disable"))
		cs40l26->pdata.expl_mode_enabled = false;
	else
		cs40l26->pdata.expl_mode_enabled = true;

	cs40l26->pdata.vbbr_en =
			of_property_read_bool(np, "cirrus,vbbr-enable");

	if (!of_property_read_u32(np, "cirrus,vbbr-thld-mv", &val))
		cs40l26->pdata.vbbr_thld_mv = val;

	if (!of_property_read_u32(np, "cirrus,vbbr-max-att-db", &val))
		cs40l26->pdata.vbbr_max_att = val;
	else
		cs40l26->pdata.vbbr_max_att = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vbbr-atk-step", &val))
		cs40l26->pdata.vbbr_atk_step = val;

	if (!of_property_read_u32(np, "cirrus,vbbr-atk-rate", &val))
		cs40l26->pdata.vbbr_atk_rate = val;

	if (!of_property_read_u32(np, "cirrus,vbbr-wait", &val))
		cs40l26->pdata.vbbr_wait = val;
	else
		cs40l26->pdata.vbbr_wait = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vbbr-rel-rate", &val))
		cs40l26->pdata.vbbr_rel_rate = val;
	else
		cs40l26->pdata.vbbr_rel_rate = CS40L26_VXBR_DEFAULT;

	cs40l26->pdata.vpbr_en =
			of_property_read_bool(np, "cirrus,vpbr-enable");

	if (!of_property_read_u32(np, "cirrus,vpbr-thld-mv", &val))
		cs40l26->pdata.vpbr_thld_mv = val;

	if (!of_property_read_u32(np, "cirrus,vpbr-max-att-db", &val))
		cs40l26->pdata.vpbr_max_att = val;
	else
		cs40l26->pdata.vpbr_max_att = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vpbr-atk-step", &val))
		cs40l26->pdata.vpbr_atk_step = val;

	if (!of_property_read_u32(np, "cirrus,vpbr-atk-rate", &val))
		cs40l26->pdata.vpbr_atk_rate = val;
	else
		cs40l26->pdata.vpbr_atk_rate = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vpbr-wait", &val))
		cs40l26->pdata.vpbr_wait = val;
	else
		cs40l26->pdata.vpbr_wait = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,vpbr-rel-rate", &val))
		cs40l26->pdata.vpbr_rel_rate = val;
	else
		cs40l26->pdata.vpbr_rel_rate = CS40L26_VXBR_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,bst-dcm-en", &val))
		cs40l26->pdata.bst_dcm_en = val;
	else
		cs40l26->pdata.bst_dcm_en = CS40L26_BST_DCM_EN_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,bst-ipk-microamp", &val))
		cs40l26->pdata.bst_ipk = val;
	else
		cs40l26->pdata.bst_ipk = 0;

	if (!of_property_read_u32(np, "cirrus,pm-stdby-timeout-ms", &val))
		cs40l26->pdata.pm_stdby_timeout_ms = val;
	else
		cs40l26->pdata.pm_stdby_timeout_ms =
				CS40L26_PM_STDBY_TIMEOUT_MS_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,pm-active-timeout-ms", &val))
		cs40l26->pdata.pm_active_timeout_ms = val;
	else
		cs40l26->pdata.pm_active_timeout_ms =
				CS40L26_PM_ACTIVE_TIMEOUT_MS_DEFAULT;

	ret = cs40l26_handle_svc_le_nodes(cs40l26);
	if (ret < 0)
		cs40l26->num_svc_le_vals = 0;
	else
		cs40l26->num_svc_le_vals = ret;

	cs40l26->pdata.asp_scale_pct = CS40L26_GAIN_FULL_SCALE;
	cs40l26->gain_pct = CS40L26_GAIN_FULL_SCALE;
	cs40l26->gain_tmp = CS40L26_GAIN_FULL_SCALE;
	if (!of_property_read_u32(np, "cirrus,asp-gain-scale-pct", &val)) {
		if (val <= CS40L26_GAIN_FULL_SCALE)
			cs40l26->pdata.asp_scale_pct = val;
		else
			dev_warn(dev, "ASP scaling > 100 %%, using maximum\n");
	}

	if (!of_property_read_u32(np, "cirrus,boost-ctl-microvolt", &val))
		cs40l26->pdata.boost_ctl = val;
	else
		cs40l26->pdata.boost_ctl = CS40L26_BST_CTL_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,f0-default", &val))
		cs40l26->pdata.f0_default = val;

	if (!of_property_read_u32(np, "cirrus,redc-default", &val))
		cs40l26->pdata.redc_default = val;

	if (!of_property_read_u32(np, "cirrus,q-default", &val))
		cs40l26->pdata.q_default = val;

	if (of_property_read_bool(np, "cirrus,dbc-enable"))
		cs40l26->pdata.dbc_enable_default = true;
	else
		cs40l26->pdata.dbc_enable_default = false;

	if (!of_property_read_u32(np, "cirrus,dbc-env-rel-coef", &val))
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_ENV_REL_COEF] = val;
	else
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_ENV_REL_COEF] =
				CS40L26_DBC_USE_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,dbc-fall-headroom", &val))
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_FALL_HEADROOM] = val;
	else
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_FALL_HEADROOM] =
				CS40L26_DBC_USE_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,dbc-rise-headroom", &val))
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_RISE_HEADROOM] = val;
	else
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_RISE_HEADROOM] =
				CS40L26_DBC_USE_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,dbc-tx-lvl-hold-off-ms", &val))
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_TX_LVL_HOLD_OFF_MS] =
				val;
	else
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_TX_LVL_HOLD_OFF_MS] =
				CS40L26_DBC_USE_DEFAULT;

	if (!of_property_read_u32(np, "cirrus,dbc-tx-lvl-thresh-fs", &val))
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_TX_LVL_THRESH_FS] = val;
	else
		cs40l26->pdata.dbc_defaults[CS40L26_DBC_TX_LVL_THRESH_FS] =
				CS40L26_DBC_USE_DEFAULT;

	if (of_property_read_bool(np, "cirrus,pwle-zero-cross-en"))
		cs40l26->pdata.pwle_zero_cross = true;
	else
		cs40l26->pdata.pwle_zero_cross = false;

	cs40l26->pdata.press_idx = gpio_map_get(np, CS40L26_GPIO_MAP_A_PRESS);
	cs40l26->pdata.release_idx = gpio_map_get(np, CS40L26_GPIO_MAP_A_RELEASE);

	return cs40l26_no_wait_ram_indices_get(cs40l26, np);
}

int cs40l26_probe(struct cs40l26_private *cs40l26,
		struct cs40l26_platform_data *pdata)
{
	struct device *dev = cs40l26->dev;
	int ret;

	mutex_init(&cs40l26->lock);

	cs40l26->vibe_workqueue = alloc_ordered_workqueue("vibe_workqueue",
			WQ_HIGHPRI);
	if (!cs40l26->vibe_workqueue) {
		ret = -ENOMEM;
		goto err;
	}

	INIT_WORK(&cs40l26->vibe_start_work, cs40l26_vibe_start_worker);
	INIT_WORK(&cs40l26->vibe_stop_work, cs40l26_vibe_stop_worker);
	INIT_WORK(&cs40l26->set_gain_work, cs40l26_set_gain_worker);
	INIT_WORK(&cs40l26->upload_work, cs40l26_upload_worker);
	INIT_WORK(&cs40l26->erase_work, cs40l26_erase_worker);

	ret = devm_regulator_bulk_get(dev, CS40L26_NUM_SUPPLIES,
			cs40l26_supplies);
	if (ret) {
		dev_err(dev, "Failed to request core supplies: %d\n", ret);
		goto err;
	}

	if (pdata) {
		cs40l26->pdata = *pdata;
	} else if (cs40l26->dev->of_node) {
		ret = cs40l26_handle_platform_data(cs40l26);
		if (ret)
			goto err;
	} else {
		dev_err(dev, "No platform data found\n");
		ret = -ENOENT;
		goto err;
	}

	ret = regulator_bulk_enable(CS40L26_NUM_SUPPLIES, cs40l26_supplies);
	if  (ret) {
		dev_err(dev, "Failed to enable core supplies\n");
		goto err;
	}

	cs40l26->reset_gpio = devm_gpiod_get_optional(dev, "reset",
			GPIOD_OUT_LOW);
	if (IS_ERR(cs40l26->reset_gpio)) {
		dev_err(dev, "Failed to get reset GPIO\n");

		ret = PTR_ERR(cs40l26->reset_gpio);
		cs40l26->reset_gpio = NULL;
		goto err;
	}

	usleep_range(CS40L26_MIN_RESET_PULSE_WIDTH,
			CS40L26_MIN_RESET_PULSE_WIDTH + 100);

	gpiod_set_value_cansleep(cs40l26->reset_gpio, 1);

	usleep_range(CS40L26_CONTROL_PORT_READY_DELAY,
			CS40L26_CONTROL_PORT_READY_DELAY + 100);

	/*
	 * The DSP may lock up if a haptic effect is triggered via
	 * GPI event or control port and the PLL is set to closed-loop.
	 *
	 * Set PLL to open-loop and remove any default GPI mappings
	 * to prevent this while the driver is loading and configuring RAM
	 * firmware.
	 */

	ret = cs40l26_set_pll_loop(cs40l26, CS40L26_PLL_REFCLK_SET_OPEN_LOOP);
	if (ret)
		goto err;

	ret = cs40l26_erase_gpi_mapping(cs40l26, CS40L26_GPIO_MAP_A_PRESS);
	if (ret)
		goto err;

	ret = cs40l26_erase_gpi_mapping(cs40l26, CS40L26_GPIO_MAP_A_RELEASE);
	if (ret)
		goto err;

	ret = cs40l26_part_num_resolve(cs40l26);
	if (ret)
		goto err;

	/* Set LRA to high-z to avoid fault conditions */
	ret = regmap_update_bits(cs40l26->regmap, CS40L26_TST_DAC_MSM_CONFIG,
			CS40L26_SPK_DEFAULT_HIZ_MASK, 1 <<
			CS40L26_SPK_DEFAULT_HIZ_SHIFT);
	if (ret) {
		dev_err(dev, "Failed to set LRA to HI-Z\n");
		goto err;
	}

	cs40l26->pm_ready = false;

	init_completion(&cs40l26->i2s_cont);
	init_completion(&cs40l26->erase_cont);
	init_completion(&cs40l26->cal_f0_cont);
	init_completion(&cs40l26->cal_redc_cont);
	init_completion(&cs40l26->cal_dvl_peq_cont);

	if (!cs40l26->fw_defer) {
		ret = cs40l26_fw_upload(cs40l26);
		if (ret)
			goto err;

		ret = devm_request_threaded_irq(dev, cs40l26->irq, NULL,
				cs40l26_irq, IRQF_ONESHOT | IRQF_SHARED |
				IRQF_TRIGGER_LOW, "cs40l26", cs40l26);
		if (ret) {
			dev_err(dev, "Failed to request threaded IRQ\n");
			goto err;
		}
	}

	ret = cs40l26_input_init(cs40l26);
	if (ret)
		goto err;

	INIT_LIST_HEAD(&cs40l26->effect_head);

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, cs40l26_devs,
			CS40L26_NUM_MFD_DEVS, NULL, 0, NULL);
	if (ret) {
		dev_err(dev, "Failed to register codec component\n");
		goto err;
	}

	return 0;
err:
	cs40l26_remove(cs40l26);

	return ret;
}
EXPORT_SYMBOL(cs40l26_probe);

int cs40l26_remove(struct cs40l26_private *cs40l26)
{
	struct regulator *vp_consumer =
			cs40l26_supplies[CS40L26_VP_SUPPLY].consumer;
	struct regulator *va_consumer =
			cs40l26_supplies[CS40L26_VA_SUPPLY].consumer;

	disable_irq(cs40l26->irq);
	mutex_destroy(&cs40l26->lock);


	if (cs40l26->pm_ready)
		cs40l26_pm_runtime_teardown(cs40l26);

	if (cs40l26->vibe_workqueue) {
		cancel_work_sync(&cs40l26->vibe_start_work);
		cancel_work_sync(&cs40l26->vibe_stop_work);
		cancel_work_sync(&cs40l26->set_gain_work);
		cancel_work_sync(&cs40l26->upload_work);
		cancel_work_sync(&cs40l26->erase_work);
		destroy_workqueue(cs40l26->vibe_workqueue);
	}

	if (vp_consumer)
		regulator_disable(vp_consumer);

	if (va_consumer)
		regulator_disable(va_consumer);

	gpiod_set_value_cansleep(cs40l26->reset_gpio, 0);

	if (cs40l26->vibe_init_success) {
#if !IS_ENABLED(CONFIG_INPUT_CS40L26_ATTR_UNDER_BUS)
		sysfs_remove_group(&cs40l26->input->dev.kobj,
				&cs40l26_dev_attr_group);
		sysfs_remove_group(&cs40l26->input->dev.kobj,
				&cs40l26_dev_attr_cal_group);
		sysfs_remove_group(&cs40l26->input->dev.kobj,
			&cs40l26_dev_attr_dbc_group);
#else
		sysfs_remove_groups(&cs40l26->dev->kobj,
				cs40l26_dev_attr_groups);
#endif
	}

#ifdef CONFIG_DEBUG_FS
	cs40l26_debugfs_cleanup(cs40l26);
#endif

	if (cs40l26->input)
		input_unregister_device(cs40l26->input);

	return 0;
}
EXPORT_SYMBOL(cs40l26_remove);

int cs40l26_pm_enter(struct device *dev)
{
	int ret;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		cs40l26_resume_error_handle(dev, ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(cs40l26_pm_enter);

void cs40l26_pm_exit(struct device *dev)
{
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
}
EXPORT_SYMBOL(cs40l26_pm_exit);

int cs40l26_suspend(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);

	if (!cs40l26->pm_ready) {
		dev_dbg(dev, "Suspend call ignored\n");
		return 0;
	}

	dev_dbg(cs40l26->dev, "%s: Enabling hibernation\n", __func__);

	return cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_ALLOW_HIBERNATE);
}
EXPORT_SYMBOL(cs40l26_suspend);

int cs40l26_sys_suspend(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct i2c_client *i2c_client = to_i2c_client(dev);

	dev_dbg(cs40l26->dev, "System suspend, disabling IRQ\n");

	disable_irq(i2c_client->irq);

	return 0;
}
EXPORT_SYMBOL(cs40l26_sys_suspend);

int cs40l26_sys_suspend_noirq(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct i2c_client *i2c_client = to_i2c_client(dev);

	dev_dbg(cs40l26->dev, "Late system suspend, re-enabling IRQ\n");
	enable_irq(i2c_client->irq);

	return 0;
}
EXPORT_SYMBOL(cs40l26_sys_suspend_noirq);

void cs40l26_resume_error_handle(struct device *dev, int ret)
{
	dev_err(dev, "PM Runtime Resume failed: %d\n", ret);

	pm_runtime_set_active(dev);

	cs40l26_pm_exit(dev);
}
EXPORT_SYMBOL(cs40l26_resume_error_handle);

int cs40l26_resume(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);

	if (!cs40l26->pm_ready) {
		dev_dbg(dev, "Resume call ignored\n");
		return 0;
	}

	dev_dbg(cs40l26->dev, "%s: Disabling hibernation\n", __func__);

	return cs40l26_pm_state_transition(cs40l26,
			CS40L26_PM_STATE_PREVENT_HIBERNATE);
}
EXPORT_SYMBOL(cs40l26_resume);

int cs40l26_sys_resume(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct i2c_client *i2c_client = to_i2c_client(dev);

	dev_dbg(cs40l26->dev, "System resume, re-enabling IRQ\n");

	enable_irq(i2c_client->irq);

	return 0;
}
EXPORT_SYMBOL(cs40l26_sys_resume);

int cs40l26_sys_resume_noirq(struct device *dev)
{
	struct cs40l26_private *cs40l26 = dev_get_drvdata(dev);
	struct i2c_client *i2c_client = to_i2c_client(dev);

	dev_dbg(cs40l26->dev, "Early system resume, disabling IRQ\n");

	disable_irq(i2c_client->irq);

	return 0;
}
EXPORT_SYMBOL(cs40l26_sys_resume_noirq);

const struct dev_pm_ops cs40l26_pm_ops = {
	SET_RUNTIME_PM_OPS(cs40l26_suspend, cs40l26_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(cs40l26_sys_suspend, cs40l26_sys_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(cs40l26_sys_suspend_noirq, cs40l26_sys_resume_noirq)
};
EXPORT_SYMBOL_GPL(cs40l26_pm_ops);

MODULE_DESCRIPTION("CS40L26 Boosted Mono Class D Amplifier for Haptics");
MODULE_AUTHOR("Fred Treven, Cirrus Logic Inc. <fred.treven@cirrus.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("7.0.0");
