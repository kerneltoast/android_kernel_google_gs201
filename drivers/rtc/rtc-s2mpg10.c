// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/rtc/rtc-s2mpg10.c
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/rtc.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/mfd/samsung/rtc-s2mpg10.h>
#include <linux/mfd/samsung/s2mpg10.h>
#include <linux/mfd/samsung/s2mpg10-register.h>

/*#define CONFIG_WEEKDAY_ALARM_ENABLE*/

static void s2m_data_to_tm(u8 *data, struct rtc_time *tm)
{
	// tm->tm_msec = (data[RTC_MSEC] & 0x0f) + (data[RTC_MSEC] & 0xf0) * 10;
	tm->tm_sec = data[RTC_SEC] & 0x7f;
	tm->tm_min = data[RTC_MIN] & 0x7f;
	tm->tm_hour = data[RTC_HOUR] & 0x1f;
	tm->tm_wday = __fls(data[RTC_WEEKDAY] & 0x7f);
	tm->tm_mday = data[RTC_DATE] & 0x1f;
	tm->tm_mon = (data[RTC_MONTH] & 0x0f) - 1;
	tm->tm_year = (data[RTC_YEAR] & 0x7f) + 100;
	tm->tm_yday = 0;
	tm->tm_isdst = 0;
}

static int s2m_tm_to_data(struct rtc_time *tm, u8 *data)
{
	// data[RTC_MSEC] = ((tm->tm_msec / 10) << 4) | (tm->tm_msec % 10);
	data[RTC_SEC] = tm->tm_sec;
	data[RTC_MIN] = tm->tm_min;

	if (tm->tm_hour >= 12)
		data[RTC_HOUR] = tm->tm_hour | BIT(HOUR_PM_SHIFT);
	else
		data[RTC_HOUR] = tm->tm_hour;

	data[RTC_WEEKDAY] = BIT(tm->tm_wday);
	data[RTC_DATE] = tm->tm_mday;
	data[RTC_MONTH] = tm->tm_mon + 1;
	data[RTC_YEAR] = tm->tm_year > 100 ? (tm->tm_year - 100) : 0;

	if (tm->tm_year < 100) {
		pr_warn("%s: SEC RTC cannot handle the year %d\n", __func__,
			1900 + tm->tm_year);
		return -EINVAL;
	}
	return 0;
}

static int s2m_rtc_update(struct s2m_rtc_info *info, enum S2M_RTC_OP op)
{
	u8 data, reg;
	int ret;

	if (!info || !info->iodev) {
		pr_err("%s: Invalid argument\n", __func__);
		return -EINVAL;
	}

	ret = s2mpg10_read_reg(info->i2c, S2MPG10_RTC_UPDATE, &data);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to read update reg(%d,%u)\n",
			__func__, ret, data);
		return ret;
	}

	data |= info->update_reg;

	switch (op) {
	case S2M_RTC_READ:
		reg = BIT(RTC_RUDR_SHIFT);
		break;
	case S2M_RTC_WRITE_TIME:
		reg = BIT(RTC_WUDR_SHIFT);
		break;
	case S2M_RTC_WRITE_ALARM:
		reg = BIT(RTC_AUDR_SHIFT);
		break;
	default:
		dev_err(info->dev, "%s: invalid op(%d)\n", __func__, op);
		return -EINVAL;
	}

	data &= ~reg;
	ret = s2mpg10_write_reg(info->i2c, S2MPG10_RTC_UPDATE, data);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write update reg(%d,%u)\n",
			__func__, ret, data);
		return ret;
	}

	usleep_range(50, 51);

	data |= reg;
	ret = s2mpg10_write_reg(info->i2c, S2MPG10_RTC_UPDATE, data);
	if (ret < 0)
		dev_err(info->dev, "%s: fail to write update reg(%d,%u)\n",
			__func__, ret, data);
	else
		usleep_range(1000, 1000);

	return ret;
}

static int s2m_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct s2m_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS];
	int ret;

	mutex_lock(&info->lock);
	ret = s2m_rtc_update(info, S2M_RTC_READ);
	if (ret < 0)
		goto out;

	ret = s2mpg10_bulk_read(info->i2c, S2MPG10_RTC_SEC, NR_RTC_CNT_REGS,
				data);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to read time reg(%d)\n", __func__,
			ret);
		goto out;
	}

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(0x%02x)%s\n",
		 __func__, data[RTC_YEAR] + 2000, data[RTC_MONTH],
		 data[RTC_DATE], data[RTC_HOUR] & 0x1f, data[RTC_MIN],
		 data[RTC_SEC], data[RTC_WEEKDAY],
		 data[RTC_HOUR] & BIT(HOUR_PM_SHIFT) ? "PM" : "AM");

	s2m_data_to_tm(data, tm);
	ret = rtc_valid_tm(tm);
out:
	mutex_unlock(&info->lock);
	return ret;
}

static int s2m_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct s2m_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS];
	int ret;

	ret = s2m_tm_to_data(tm, data);
	if (ret < 0)
		return ret;

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(0x%02x)%s\n",
		 __func__, data[RTC_YEAR] + 2000, data[RTC_MONTH],
		 data[RTC_DATE], data[RTC_HOUR] & 0x1f, data[RTC_MIN],
		 data[RTC_SEC], data[RTC_WEEKDAY],
		 data[RTC_HOUR] & BIT(HOUR_PM_SHIFT) ? "PM" : "AM");

	mutex_lock(&info->lock);
	ret = s2mpg10_bulk_write(info->i2c, S2MPG10_RTC_SEC, NR_RTC_CNT_REGS,
				 data);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write time reg(%d)\n", __func__,
			ret);
		goto out;
	}

	ret = s2m_rtc_update(info, S2M_RTC_WRITE_TIME);
out:
	mutex_unlock(&info->lock);
	return ret;
}

static int s2m_rtc_check_rtc_time(struct s2m_rtc_info *info)
{
	u8 data[NR_RTC_CNT_REGS];
	struct rtc_time tm;
	struct timespec64 sys_time;
	time64_t rtc_time;
	int ret;

	/* Read RTC TIME */
	ret = s2m_rtc_update(info, S2M_RTC_READ);
	if (ret < 0)
		goto out;

	ret = s2mpg10_bulk_read(info->i2c, S2MPG10_RTC_SEC, NR_RTC_CNT_REGS,
				data);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to read time reg(%d)\n", __func__,
			ret);
		goto out;
	}

	/* Get system time */
	ktime_get_real_ts64(&sys_time);

	/* Convert RTC TIME to seconds since 01-01-1970 00:00:00. */
	s2m_data_to_tm(data, &tm);
	rtc_time = rtc_tm_to_time64(&tm);

	if (abs(rtc_time - sys_time.tv_sec) > 2) {
		/* Set RTC TIME */
		rtc_time64_to_tm(sys_time.tv_sec, &tm);
		ret = s2m_tm_to_data(&tm, data);
		if (ret < 0) {
			dev_err(info->dev, "%s: fail to tm_to_data(%d)\n",
				__func__, ret);
			goto out;
		}

		ret = s2mpg10_bulk_write(info->i2c, S2MPG10_RTC_SEC,
					 NR_RTC_CNT_REGS, data);
		if (ret < 0) {
			dev_err(info->dev, "%s: fail to write time reg(%d)\n",
				__func__, ret);
			goto out;
		}

		ret = s2m_rtc_update(info, S2M_RTC_WRITE_TIME);

		dev_warn(info->dev,
			 "%s: adjust RTC TIME: sys_time: %llu, rtc_time: %lld\n",
			 __func__, sys_time.tv_sec, rtc_time);

		dev_info(info->dev,
			 "%s: %d-%02d-%02d %02d:%02d:%02d(0x%02x)%s\n",
			 __func__, data[RTC_YEAR] + 2000, data[RTC_MONTH],
			 data[RTC_DATE], data[RTC_HOUR] & 0x1f, data[RTC_MIN],
			 data[RTC_SEC], data[RTC_WEEKDAY],
			 data[RTC_HOUR] & BIT(HOUR_PM_SHIFT) ? "PM" : "AM");
	}
out:
	return ret;
}

static int s2m_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct s2m_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS];
	u8 reg, val;
	int ret;

	mutex_lock(&info->lock);
	ret = s2m_rtc_update(info, S2M_RTC_READ);
	if (ret < 0)
		goto out;

	ret = s2mpg10_bulk_read(info->i2c, S2MPG10_RTC_A0SEC, NR_RTC_CNT_REGS,
				data);
	if (ret < 0) {
		dev_err(info->dev, "%s:%d fail to read alarm reg(%d)\n",
			__func__, __LINE__, ret);
		goto out;
	}

	s2m_data_to_tm(data, &alrm->time);

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(%d)\n", __func__,
		 alrm->time.tm_year + 1900, alrm->time.tm_mon + 1,
		 alrm->time.tm_mday, alrm->time.tm_hour, alrm->time.tm_min,
		 alrm->time.tm_sec, alrm->time.tm_wday);

	alrm->enabled = info->alarm_enabled;
	alrm->pending = 0;

	switch (info->iodev->device_type) {
	case S2MPG10X:
		reg = S2MPG10_PM_STATUS2;
		break;
	default:
		/* If this happens the core function has a problem */
		WARN_ON(1);
		ret = -ENXIO;
		goto out;
	}

	ret = s2mpg10_read_reg(info->pmic_i2c, reg, &val); /* i2c for PM */
	if (ret < 0) {
		dev_err(info->dev, "%s:%d fail to read STATUS2 reg(%d)\n",
			__func__, __LINE__, ret);
		goto out;
	}

	if (val & BIT(RTCA0E_SHIFT))
		alrm->pending = 1;
out:
	mutex_unlock(&info->lock);
	return ret;
}

static int s2m_rtc_set_alarm_enable(struct s2m_rtc_info *info, bool enabled)
{
	if (!info->use_irq)
		return -EPERM;

	if (enabled && !info->alarm_enabled) {
		info->alarm_enabled = true;
		enable_irq(info->alarm0_irq);
	} else if (!enabled && info->alarm_enabled) {
		info->alarm_enabled = false;
		disable_irq(info->alarm0_irq);
	}
	return 0;
}

static int s2m_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct s2m_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS];
	int ret, i;

	mutex_lock(&info->lock);
	ret = s2m_tm_to_data(&alrm->time, data);
	if (ret < 0)
		goto out;

	dev_info(info->dev, "%s: %d-%02d-%02d %02d:%02d:%02d(0x%02x)%s\n",
		 __func__, data[RTC_YEAR] + 2000, data[RTC_MONTH],
		 data[RTC_DATE], data[RTC_HOUR] & 0x1f, data[RTC_MIN],
		 data[RTC_SEC], data[RTC_WEEKDAY],
		 data[RTC_HOUR] & BIT(HOUR_PM_SHIFT) ? "PM" : "AM");

	if (info->alarm_check) {
		for (i = 0; i < NR_RTC_CNT_REGS; i++)
			data[i] &= ~BIT(ALARM_ENABLE_SHIFT);

		ret = s2mpg10_bulk_write(info->i2c, S2MPG10_RTC_A0SEC,
					 NR_RTC_CNT_REGS, data);
		if (ret < 0) {
			dev_err(info->dev,
				"%s: fail to disable alarm reg(%d)\n", __func__,
				ret);
			goto out;
		}

		ret = s2m_rtc_update(info, S2M_RTC_WRITE_ALARM);
		if (ret < 0)
			goto out;
	}

	for (i = 0; i < NR_RTC_CNT_REGS; i++)
		data[i] |= BIT(ALARM_ENABLE_SHIFT);

	ret = s2mpg10_bulk_write(info->i2c, S2MPG10_RTC_A0SEC, NR_RTC_CNT_REGS,
				 data);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write alarm reg(%d)\n",
			__func__, ret);
		goto out;
	}

	ret = s2m_rtc_update(info, S2M_RTC_WRITE_ALARM);
	if (ret < 0)
		goto out;

	if (info->use_alarm_workaround) {
		ret = s2m_rtc_check_rtc_time(info);
		if (ret < 0)
			goto out;
	}

	ret = s2m_rtc_set_alarm_enable(info, alrm->enabled);
out:
	mutex_unlock(&info->lock);
	return ret;
}

static int s2m_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct s2m_rtc_info *info = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&info->lock);
	ret = s2m_rtc_set_alarm_enable(info, enabled);
	mutex_unlock(&info->lock);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int s2m_rtc_wake_lock_timeout(struct device *dev, unsigned int msec)
{
	struct wakeup_source *ws = NULL;

	if (!dev->power.wakeup) {
		pr_err("%s: Not register wakeup source\n", __func__);
		goto err;
	}

	ws = dev->power.wakeup;
	__pm_wakeup_event(ws, msec);

	return 0;
err:
	return -1;
}
#endif

static irqreturn_t s2m_rtc_alarm_irq(int irq, void *data)
{
	struct s2m_rtc_info *info = data;

	if (!info->rtc_dev)
		return IRQ_HANDLED;

	dev_info(info->dev, "%s:irq(%d)\n", __func__, irq);

	rtc_update_irq(info->rtc_dev, 1, RTC_IRQF | RTC_AF);

#ifdef CONFIG_PM_SLEEP
	if (s2m_rtc_wake_lock_timeout(info->dev, 500) < 0)
		return IRQ_NONE;
#endif

	return IRQ_HANDLED;
}

static const struct rtc_class_ops s2m_rtc_ops = {
	.read_time = s2m_rtc_read_time,
	.set_time = s2m_rtc_set_time,
	.read_alarm = s2m_rtc_read_alarm,
	.set_alarm = s2m_rtc_set_alarm,
	.alarm_irq_enable = s2m_rtc_alarm_irq_enable,
};

static void s2m_rtc_optimize_osc(struct s2m_rtc_info *info,
				 struct s2mpg10_platform_data *pdata)
{
	int ret = 0;

	/* edit option for OSC_BIAS_UP */
	if (pdata->osc_bias_up >= 0) {
		ret = s2mpg10_update_reg(info->i2c, S2MPG10_RTC_CAPSEL,
					 pdata->osc_bias_up
						 << OSC_BIAS_UP_SHIFT,
					 BIT(OSC_BIAS_UP_SHIFT));
		if (ret < 0) {
			dev_err(info->dev,
				"%s: fail to write OSC_BIAS_UP(%d)\n", __func__,
				pdata->osc_bias_up);
			return;
		}
	}

	/* edit option for CAP_SEL */
	if (pdata->cap_sel >= 0) {
		ret = s2mpg10_update_reg(info->i2c, S2MPG10_RTC_CAPSEL,
					 pdata->cap_sel << CAP_SEL_SHIFT,
					 CAP_SEL_MASK);
		if (ret < 0) {
			dev_err(info->dev, "%s: fail to write CAP_SEL(%d)\n",
				__func__, pdata->cap_sel);
			return;
		}
	}

	/* edit option for OSC_CTRL */
	if (pdata->osc_xin >= 0) {
		ret = s2mpg10_update_reg(info->i2c, S2MPG10_RTC_OSCCTRL,
					 pdata->osc_xin << OSC_XIN_SHIFT,
					 OSC_XIN_MASK);
		if (ret < 0) {
			dev_err(info->dev, "%s: fail to write OSC_CTRL(%d)\n",
				__func__, pdata->osc_xin);
			return;
		}
	}
	if (pdata->osc_xout >= 0) {
		ret = s2mpg10_update_reg(info->i2c, S2MPG10_RTC_OSCCTRL,
					 pdata->osc_xout << OSC_XOUT_SHIFT,
					 OSC_XOUT_MASK);
		if (ret < 0) {
			dev_err(info->dev, "%s: fail to write OSC_CTRL(%d)\n",
				__func__, pdata->osc_xout);
			return;
		}
	}
}

static bool s2m_is_jigonb_low(struct s2m_rtc_info *info)
{
	int ret, reg;
	u8 val, mask;

	switch (info->iodev->device_type) {
	case S2MPG10X:
		reg = S2MPG10_PM_STATUS1;
		mask = BIT(1);
		break;
	default:
		WARN_ON(1);
		return false;
	}

	ret = s2mpg10_read_reg(info->i2c, reg, &val);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to read status1 reg(%d)\n",
			__func__, ret);
		return false;
	}

	return !(val & mask);
}

static void s2m_rtc_enable_wtsr_smpl(struct s2m_rtc_info *info,
				     struct s2mpg10_platform_data *pdata)
{
	u8 wtsr_val, smpl_val;
	int ret;

	if (pdata->wtsr_smpl->check_jigon && s2m_is_jigonb_low(info))
		pdata->wtsr_smpl->smpl_en = false;

	wtsr_val = (pdata->wtsr_smpl->wtsr_en << WTSR_EN_SHIFT) |
		   WTSR_TIMER_BITS(pdata->wtsr_smpl->wtsr_timer_val);

	if (pdata->wtsr_smpl->coldrst_en)
		wtsr_val |= (COLDRST_EN_MASK |
			((pdata->wtsr_smpl->coldrst_timer_val
			<< COLDRST_TIMER_SHIFT) & COLDRST_TIMER_MASK));
	else
		wtsr_val &= ~COLDRST_EN_MASK;

	smpl_val = (pdata->wtsr_smpl->smpl_en << SMPL_EN_SHIFT) |
		   SMPL_TIMER_BITS(pdata->wtsr_smpl->smpl_timer_val);

	dev_info(info->dev, "%s: WTSR: %s, COLDRST : %s, SMPL: %s\n", __func__,
		 pdata->wtsr_smpl->wtsr_en ? "enable" : "disable",
		 pdata->wtsr_smpl->coldrst_en ? "enable" : "disable",
		 pdata->wtsr_smpl->smpl_en ? "enable" : "disable");

	ret = s2mpg10_write_reg(info->i2c, S2MPG10_RTC_SMPL, smpl_val);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write SMPL reg(%d)\n", __func__,
			ret);
		return;
	}

	ret = s2mpg10_write_reg(info->i2c, S2MPG10_RTC_WTSR, wtsr_val);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write WTSR reg(%d)\n", __func__,
			ret);
		return;
	}

	info->wtsr_en = pdata->wtsr_smpl->wtsr_en;
	info->smpl_en = pdata->wtsr_smpl->smpl_en;
	info->coldrst_en = pdata->wtsr_smpl->coldrst_en;
}

static void s2m_rtc_disable_wtsr_smpl(struct s2m_rtc_info *info,
				      struct s2mpg10_platform_data *pdata)
{
	int ret;

	dev_info(info->dev, "%s: disable SMPL\n", __func__);
	ret = s2mpg10_update_reg(info->i2c, S2MPG10_RTC_SMPL, 0, SMPL_EN_MASK);
	if (ret < 0)
		dev_err(info->dev, "%s: fail to update SMPL reg(%d)\n",
			__func__, ret);

	if (info->iodev->pmic_rev == S2MPG10_EVT0) {
		dev_info(info->dev,
			 "%s: disable COLDRST only, WTSR time as 250msec\n",
			 __func__);
		ret = s2mpg10_update_reg(info->i2c, S2MPG10_RTC_WTSR, 0,
					 WTSRT_MASK | COLDRST_EN_MASK);
		if (ret < 0)
			dev_err(info->dev, "%s: fail to update WTSR reg(%d)\n",
				__func__, ret);
	} else {
		dev_info(info->dev, "%s: disable WTSR\n", __func__);
		ret = s2mpg10_update_reg(info->i2c, S2MPG10_RTC_WTSR, 0,
					 WTSR_EN_MASK | COLDRST_EN_MASK);
		if (ret < 0)
			dev_err(info->dev, "%s: fail to update SMPL reg(%d)\n",
				__func__, ret);
	}
}

static int s2m_rtc_init_reg(struct s2m_rtc_info *info,
			    struct s2mpg10_platform_data *pdata)
{
	u8 data, update_val, ctrl_val, capsel_val;
	int ret;

	ret = s2mpg10_read_reg(info->i2c, S2MPG10_RTC_UPDATE, &update_val);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to read update reg(%d)\n",
			__func__, ret);
		return ret;
	}

	info->update_reg = update_val & ~(info->wudr_mask |
					  BIT(RTC_FREEZE_SHIFT) |
					  BIT(RTC_RUDR_SHIFT) |
					  info->audr_mask);

	ret = s2mpg10_write_reg(info->i2c, S2MPG10_RTC_UPDATE,
				info->update_reg);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write update reg(%d)\n",
			__func__, ret);
		return ret;
	}

	s2m_rtc_update(info, S2M_RTC_READ);

	ret = s2mpg10_read_reg(info->i2c, S2MPG10_RTC_CTRL, &ctrl_val);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to read control reg(%d)\n",
			__func__, ret);
		return ret;
	}

	ret = s2mpg10_read_reg(info->i2c, S2MPG10_RTC_CAPSEL, &capsel_val);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to read cap_sel reg(%d)\n",
			__func__, ret);
		return ret;
	}

	/* If the value of RTC_CTRL register is 0, RTC registers were reset */
	if ((ctrl_val & BIT(MODEL24_SHIFT)) && ((capsel_val & 0xf0) == 0xf0))
		return 0;

	/* Set RTC control register : Binary mode, 24hour mode */
	data = BIT(MODEL24_SHIFT);
	ret = s2mpg10_write_reg(info->i2c, S2MPG10_RTC_CTRL, data);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write CTRL reg(%d)\n", __func__,
			ret);
		return ret;
	}

	ret = s2m_rtc_update(info, S2M_RTC_WRITE_ALARM);
	if (ret < 0)
		return ret;

	capsel_val |= 0xf0;
	ret = s2mpg10_write_reg(info->i2c, S2MPG10_RTC_CAPSEL, capsel_val);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to write CAP_SEL reg(%d)\n",
			__func__, ret);
		return ret;
	}

	if (pdata->init_time) {
		dev_info(info->dev, "%s: initialize RTC time\n", __func__);
		ret = s2m_rtc_set_time(info->dev, pdata->init_time);
	} else {
		dev_info(info->dev,
			 "%s: RTC initialize is not operated: This causes a weekday problem\n",
			 __func__);
	}
	return ret;
}

static int s2m_rtc_probe(struct platform_device *pdev)
{
	struct s2mpg10_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg10_platform_data *pdata = dev_get_platdata(iodev->dev);
	struct s2m_rtc_info *info;
	int irq_base;
	int ret = 0;

	info = devm_kzalloc(&pdev->dev, sizeof(struct s2m_rtc_info),
			    GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	irq_base = pdata->irq_base;
	if (!irq_base) {
		dev_err(&pdev->dev, "Failed to get irq base %d\n", irq_base);
		return -ENODEV;
	}

	mutex_init(&info->lock);
	info->dev = &pdev->dev;
	info->iodev = iodev;
	info->i2c = iodev->rtc;
	info->pmic_i2c = iodev->pmic;
	info->alarm_check = true;
	info->use_alarm_workaround = false;

	switch (info->iodev->device_type) {
	case S2MPG10X:
		info->alarm0_irq = irq_base + S2MPG10_IRQ_RTCA0_INT2;
		break;
	default:
		/* If this happens the core function has a problem */
		WARN_ON(1);
		ret = -ENXIO;
		goto err_rtc_init_reg;
	}

	platform_set_drvdata(pdev, info);

	ret = s2m_rtc_init_reg(info, pdata);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to initialize RTC reg:%d\n", ret);
		goto err_rtc_init_reg;
	}

	/* enable wtsr smpl */
	if (pdata->wtsr_smpl)
		s2m_rtc_enable_wtsr_smpl(info, pdata);

	s2m_rtc_optimize_osc(info, pdata);

	ret = device_init_wakeup(&pdev->dev, true);
	if (ret < 0) {
		pr_err("%s: device_init_wakeup fail(%d)\n", __func__, ret);
		goto err_init_wakeup;
	}

	/* request alarm0 interrupt */
	ret = devm_request_threaded_irq(&pdev->dev, info->alarm0_irq, NULL,
					s2m_rtc_alarm_irq, 0, "rtc-alarm0",
					info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request alarm IRQ: %d: %d\n",
			info->alarm0_irq, ret);
		goto err_rtc_irq;
	}

	disable_irq(info->alarm0_irq);
	disable_irq(info->alarm0_irq);
	info->use_irq = true;

	info->rtc_dev = devm_rtc_device_register(&pdev->dev, "s2mpg10-rtc",
						 &s2m_rtc_ops, THIS_MODULE);

	if (IS_ERR(info->rtc_dev)) {
		ret = PTR_ERR(info->rtc_dev);
		dev_err(&pdev->dev, "Failed to register RTC device: %d\n", ret);
		goto err_rtc_dev_register;
	}

	enable_irq(info->alarm0_irq);
	return 0;

err_rtc_dev_register:
	devm_free_irq(&pdev->dev, info->alarm0_irq, info);
err_rtc_irq:
err_init_wakeup:
err_rtc_init_reg:
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&info->lock);

	return ret;
}

static int s2m_rtc_remove(struct platform_device *pdev)
{
	struct s2m_rtc_info *info = platform_get_drvdata(pdev);

	if (!info->alarm_enabled)
		enable_irq(info->alarm0_irq);

#ifdef CONFIG_PM_SLEEP
	if (info->dev->power.wakeup)
		device_init_wakeup(&pdev->dev, false);
#endif
	mutex_destroy(&info->lock);

	return 0;
}

static void s2m_rtc_shutdown(struct platform_device *pdev)
{
	/* disable wtsr, smpl */
	struct s2m_rtc_info *info = platform_get_drvdata(pdev);
	struct s2mpg10_platform_data *pdata =
		dev_get_platdata(info->iodev->dev);

	if (info->wtsr_en || info->smpl_en || info->coldrst_en)
		s2m_rtc_disable_wtsr_smpl(info, pdata);

	/* w/a for WTSR_EN */
	/* 0x204[3] == 1 -> shutdown, 0x204[3] == 0 -> not shutdown */
	if (system_state == SYSTEM_POWER_OFF) {
		if (info->iodev->pmic_rev == S2MPG10_EVT0)
			s2mpg10_update_reg(info->i2c, S2MPG10_RTC_CAPSEL, 0x08, 0x08);
	} else if (system_state == SYSTEM_RESTART) {
		if (info->iodev->pmic_rev != S2MPG10_EVT0) {
			s2mpg10_update_reg(info->i2c, S2MPG10_RTC_WTSR,
				COLDRST_TIMER_MASK | COLDRST_EN_MASK | WTSRT_MASK | WTSR_EN_MASK,
				GENMASK(6, 0));
			s2mpg10_update_reg(info->i2c, S2MPG10_RTC_CAPSEL, 0x00, 0x08);
		}
	}
}

static const struct platform_device_id s2m_rtc_id[] = {
	{ "s2mpg10-rtc", 0 },
	{},
};

static struct platform_driver s2m_rtc_driver = {
	.driver = {
		   .name = "s2mpg10-rtc",
		   .owner = THIS_MODULE,
		    },
	.probe = s2m_rtc_probe,
	.remove = s2m_rtc_remove,
	.shutdown = s2m_rtc_shutdown,
	.id_table = s2m_rtc_id,
};

module_platform_driver(s2m_rtc_driver);

/* Module information */
MODULE_DESCRIPTION("Samsung RTC driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
