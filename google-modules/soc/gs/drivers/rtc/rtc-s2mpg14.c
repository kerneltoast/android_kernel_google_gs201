// SPDX-License-Identifier: GPL-2.0+
/*
 * rtc-s2mpg14.c
 *
 * Copyright (c) 2022 Samsung Electronics Co., Ltd
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
#include <linux/mfd/samsung/rtc-s2mpg14.h>
#include <linux/mfd/samsung/s2mpg14.h>
#include <linux/mfd/samsung/s2mpg14-register.h>

static void s2m_data_to_tm(u8 *data, struct rtc_time *tm)
{
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

	if (tm->tm_year < 100)
		return -EINVAL;

	return 0;
}

static int s2m_rtc_update(struct s2m_rtc_info *info, enum S2M_RTC_OP op)
{
	u8 data, reg;
	int ret;

	if (!info || !info->iodev)
		return -EINVAL;

	ret = s2mpg14_read_reg(info->i2c, S2MPG14_RTC_UPDATE, &data);
	if (ret < 0) {
		dev_err(info->dev, "fail to read update reg(%d,%u)\n",
			ret, data);
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
		dev_err(info->dev, "invalid op(%d)\n", op);
		return -EINVAL;
	}

	data &= ~reg;
	ret = s2mpg14_write_reg(info->i2c, S2MPG14_RTC_UPDATE, data);
	if (ret < 0) {
		dev_err(info->dev, "fail to write update reg(%d,%u)\n",
			ret, data);
		return ret;
	}

	usleep_range(50, 51);

	data |= reg;
	ret = s2mpg14_write_reg(info->i2c, S2MPG14_RTC_UPDATE, data);
	if (ret < 0)
		dev_err(info->dev, "fail to write update reg(%d,%u)\n",
			ret, data);
	else
		usleep_range(1000, 1000);

	return ret;
}

static void s2m_rtc_print_time(struct s2m_rtc_info *info, u8 *time)
{
	dev_info(info->dev, "%d-%02d-%02d %02d:%02d:%02d(0x%02x)%s\n",
		 time[RTC_YEAR] + 2000,
		 time[RTC_MONTH],
		 time[RTC_DATE],
		 time[RTC_HOUR] & 0x1f,
		 time[RTC_MIN],
		 time[RTC_SEC],
		 time[RTC_WEEKDAY],
		 time[RTC_HOUR] & BIT(HOUR_PM_SHIFT) ? "PM" : "AM");
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

	ret = s2mpg14_bulk_read(info->i2c, S2MPG14_RTC_SEC, NR_RTC_CNT_REGS,
				data);
	if (ret < 0) {
		dev_err(info->dev, "%s: fail to read time reg(%d)\n", __func__,
			ret);
		goto out;
	}

	s2m_rtc_print_time(info, data);

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

	s2m_rtc_print_time(info, data);

	mutex_lock(&info->lock);
	ret = s2mpg14_bulk_write(info->i2c, S2MPG14_RTC_SEC, NR_RTC_CNT_REGS,
				 data);
	if (ret < 0) {
		dev_err(info->dev, "fail to write time reg(%d)\n", ret);
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
		return ret;

	ret = s2mpg14_bulk_read(info->i2c, S2MPG14_RTC_SEC, NR_RTC_CNT_REGS,
				data);
	if (ret < 0) {
		dev_err(info->dev, "fail to read time reg(%d)\n", ret);
		return ret;
	}

	/* Get system time */
	ktime_get_real_ts64(&sys_time);

	/* Convert RTC TIME to seconds since 01-01-1970 00:00:00. */
	s2m_data_to_tm(data, &tm);
	rtc_time = rtc_tm_to_time64(&tm);

	if (abs(rtc_time - sys_time.tv_sec) <= 2)
		return ret;

	/* Set RTC TIME */
	rtc_time64_to_tm(sys_time.tv_sec, &tm);
	ret = s2m_tm_to_data(&tm, data);
	if (ret < 0) {
		dev_err(info->dev, "fail to tm_to_data(%d)\n", ret);
		return ret;
	}

	ret = s2mpg14_bulk_write(info->i2c, S2MPG14_RTC_SEC,
				 NR_RTC_CNT_REGS, data);
	if (ret < 0) {
		dev_err(info->dev, "fail to write time reg(%d)\n", ret);
		return ret;
	}

	ret = s2m_rtc_update(info, S2M_RTC_WRITE_TIME);

	dev_warn(info->dev,
		 "adjust RTC TIME: sys_time: %llu, rtc_time: %lld\n",
		 sys_time.tv_sec, rtc_time);

	s2m_rtc_print_time(info, data);

	return ret;
}

static int s2m_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct s2m_rtc_info *info = dev_get_drvdata(dev);
	u8 data[NR_RTC_CNT_REGS];
	u8 val;
	int ret;

	mutex_lock(&info->lock);
	ret = s2m_rtc_update(info, S2M_RTC_READ);
	if (ret < 0)
		goto out;

	ret = s2mpg14_bulk_read(info->i2c, S2MPG14_RTC_A0SEC, NR_RTC_CNT_REGS,
				data);
	if (ret < 0) {
		dev_err(info->dev, "%d fail to read alarm reg(%d)\n", __LINE__,
			ret);
		goto out;
	}

	s2m_rtc_print_time(info, data);

	s2m_data_to_tm(data, &alrm->time);

	alrm->enabled = info->alarm_enabled;
	alrm->pending = 0;

	ret = s2mpg14_read_reg(info->pmic_i2c, S2MPG14_PM_STATUS2, &val);
	if (ret < 0) {
		dev_err(info->dev, "%d fail to read STATUS2 reg(%d)\n", __LINE__,
			ret);
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

	s2m_rtc_print_time(info, data);

	if (info->alarm_check) {
		for (i = 0; i < NR_RTC_CNT_REGS; i++)
			data[i] &= ~BIT(ALARM_ENABLE_SHIFT);

		ret = s2mpg14_bulk_write(info->i2c, S2MPG14_RTC_A0SEC,
					 NR_RTC_CNT_REGS, data);
		if (ret < 0) {
			dev_err(info->dev, "fail to disable alarm reg(%d)\n", ret);
			goto out;
		}

		ret = s2m_rtc_update(info, S2M_RTC_WRITE_ALARM);
		if (ret < 0)
			goto out;
	}

	for (i = 0; i < NR_RTC_CNT_REGS; i++)
		data[i] |= BIT(ALARM_ENABLE_SHIFT);

	ret = s2mpg14_bulk_write(info->i2c, S2MPG14_RTC_A0SEC, NR_RTC_CNT_REGS,
				 data);
	if (ret < 0) {
		dev_err(info->dev, "fail to write alarm reg(%d)\n", ret);
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

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int s2m_rtc_wake_lock_timeout(struct device *dev, unsigned int msec)
{
	struct wakeup_source *ws = NULL;

	if (!dev->power.wakeup)
		return -1;

	ws = dev->power.wakeup;
	__pm_wakeup_event(ws, msec);

	return 0;
}
#endif

static irqreturn_t s2m_rtc_alarm_irq(int irq, void *data)
{
	struct s2m_rtc_info *info = data;

	if (!info->rtc_dev)
		return IRQ_HANDLED;

	rtc_update_irq(info->rtc_dev, 1, RTC_IRQF | RTC_AF);

#if IS_ENABLED(CONFIG_PM_SLEEP)
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
				 struct s2mpg14_platform_data *pdata)
{
	int ret = 0;

	/* edit option for OSC_BIAS_UP */
	if (pdata->osc_bias_up >= 0) {
		ret = s2mpg14_update_reg(info->i2c, S2MPG14_RTC_CAPSEL,
					 pdata->osc_bias_up
						 << OSC_BIAS_UP_SHIFT,
					 BIT(OSC_BIAS_UP_SHIFT));
		if (ret < 0) {
			dev_err(info->dev, "fail to write OSC_BIAS_UP(%d)\n",
				pdata->osc_bias_up);
			return;
		}
	}

	/* edit option for CAP_SEL */
	if (pdata->cap_sel >= 0) {
		ret = s2mpg14_update_reg(info->i2c, S2MPG14_RTC_CAPSEL,
					 pdata->cap_sel << CAP_SEL_SHIFT,
					 CAP_SEL_MASK);
		if (ret < 0) {
			dev_err(info->dev, "fail to write CAP_SEL(%d)\n",
				pdata->cap_sel);
			return;
		}
	}

	/* edit option for OSC_CTRL */
	if (pdata->osc_xin >= 0) {
		ret = s2mpg14_update_reg(info->i2c, S2MPG14_RTC_OSC_CTRL,
					 pdata->osc_xin << OSC_XIN_SHIFT,
					 OSC_XIN_MASK);
		if (ret < 0) {
			dev_err(info->dev, "fail to write OSC_CTRL(%d)\n",
				pdata->osc_xin);
			return;
		}
	}
	if (pdata->osc_xout >= 0) {
		ret = s2mpg14_update_reg(info->i2c, S2MPG14_RTC_OSC_CTRL,
					 pdata->osc_xout << OSC_XOUT_SHIFT,
					 OSC_XOUT_MASK);
		if (ret < 0) {
			dev_err(info->dev, "fail to write OSC_CTRL(%d)\n",
				pdata->osc_xout);
			return;
		}
	}
}

static bool s2m_is_jigonb_low(struct s2m_rtc_info *info)
{
	int ret;
	u8 val;

	ret = s2mpg14_read_reg(info->i2c, S2MPG14_PM_STATUS1, &val);
	if (ret < 0) {
		dev_err(info->dev, "fail to read status1 reg(%d)\n", ret);
		return false;
	}

	return !(val & BIT(1));
}

static void s2m_rtc_enable_wtsr_smpl(struct s2m_rtc_info *info,
				     struct s2mpg14_platform_data *pdata)
{
	u8 wtsr_val, smpl_val;
	int ret;
	bool wtsr_en, coldrst_en, smpl_en;

	wtsr_en = pdata->wtsr_smpl->wtsr_en;
	coldrst_en = pdata->wtsr_smpl->coldrst_en;

	if (pdata->wtsr_smpl->check_jigon && s2m_is_jigonb_low(info))
		pdata->wtsr_smpl->smpl_en = false;
	smpl_en = pdata->wtsr_smpl->smpl_en;

	wtsr_val = (wtsr_en << WTSR_EN_SHIFT) |
		   WTSR_TIMER_BITS(pdata->wtsr_smpl->wtsr_timer_val);

	if (coldrst_en)
		wtsr_val |= (COLDRST_EN_MASK |
			((pdata->wtsr_smpl->coldrst_timer_val
			<< COLDRST_TIMER_SHIFT) & COLDRST_TIMER_MASK));
	else
		wtsr_val &= ~COLDRST_EN_MASK;

	smpl_val = (smpl_en << SMPL_EN_SHIFT) |
		   SMPL_TIMER_BITS(pdata->wtsr_smpl->smpl_timer_val);

	dev_info(info->dev, "WTSR: %s, COLDRST : %s, SMPL: %s\n",
		 wtsr_en ? "enable" : "disable",
		 coldrst_en ? "enable" : "disable",
		 smpl_en ? "enable" : "disable");

	ret = s2mpg14_write_reg(info->i2c, S2MPG14_RTC_SMPL, smpl_val);
	if (ret < 0) {
		dev_err(info->dev, "fail to write SMPL reg(%d)\n", ret);
		return;
	}

	ret = s2mpg14_write_reg(info->i2c, S2MPG14_RTC_WTSR, wtsr_val);
	if (ret < 0) {
		dev_err(info->dev, "fail to write WTSR reg(%d)\n", ret);
		return;
	}

	info->wtsr_en = wtsr_en;
	info->smpl_en = smpl_en;
	info->coldrst_en = coldrst_en;
}

static void s2m_rtc_disable_wtsr_smpl(struct s2m_rtc_info *info,
				      struct s2mpg14_platform_data *pdata)
{
	int ret;

	dev_dbg(info->dev, "disable SMPL\n");
	ret = s2mpg14_update_reg(info->i2c, S2MPG14_RTC_SMPL, 0, SMPL_EN_MASK);
	if (ret < 0)
		dev_err(info->dev, "fail to update SMPL reg(%d)\n", ret);

	ret = s2mpg14_update_reg(info->i2c, S2MPG14_RTC_WTSR, 0,
				 WTSR_EN_MASK | COLDRST_EN_MASK);
	if (ret < 0)
		dev_err(info->dev, "fail to update SMPL reg(%d)\n", ret);
}

static int s2m_rtc_init_reg(struct s2m_rtc_info *info,
			    struct s2mpg14_platform_data *pdata)
{
	u8 data, update_val, ctrl_val, nonce0_val[NONCE0_CNT];
	bool is_nonce0_val_zero = true;
	int i;
	int ret;

	ret = s2mpg14_read_reg(info->i2c, S2MPG14_RTC_UPDATE, &update_val);
	if (ret < 0) {
		dev_err(info->dev, "fail to read update reg(%d)\n", ret);
		return ret;
	}

	info->update_reg = update_val & ~(info->wudr_mask |
					  BIT(RTC_FREEZE_SHIFT) |
					  BIT(RTC_RUDR_SHIFT) |
					  info->audr_mask);

	ret = s2mpg14_write_reg(info->i2c, S2MPG14_RTC_UPDATE,
				info->update_reg);
	if (ret < 0) {
		dev_err(info->dev, "fail to write update reg(%d)\n", ret);
		return ret;
	}

	s2m_rtc_update(info, S2M_RTC_READ);

	ret = s2mpg14_read_reg(info->i2c, S2MPG14_RTC_CTRL, &ctrl_val);
	if (ret < 0) {
		dev_err(info->dev, "fail to read control reg(%d)\n", ret);
		return ret;
	}

	ret = s2mpg14_bulk_read(info->i2c, S2MPG14_RTC_NONCE0_0, NONCE0_CNT,
				nonce0_val);
	if (ret < 0) {
		dev_err(info->dev, "fail to read NONCE0 regs(%d)\n", ret);
		return ret;
	}

	for (i = 0; i < NONCE0_CNT; i++) {
		if (nonce0_val[i] != 0) {
			is_nonce0_val_zero = false;
			break;
		}
	}

	/* If the value of RTC_CTRL register is 0, RTC registers were reset */
	if ((ctrl_val & BIT(MODEL24_SHIFT)) && !is_nonce0_val_zero) {
		return 0;
	}

	/* Set RTC control register : Binary mode, 24hour mode */
	data = BIT(MODEL24_SHIFT);
	ret = s2mpg14_write_reg(info->i2c, S2MPG14_RTC_CTRL, data);
	if (ret < 0) {
		dev_err(info->dev, "fail to write CTRL reg(%d)\n", ret);
		return ret;
	}

	ret = s2m_rtc_update(info, S2M_RTC_WRITE_ALARM);
	if (ret < 0)
		return ret;

	ret = s2mpg14_write_reg(info->i2c, S2MPG14_RTC_NONCE0_0, NONCE0_0_SPECIAL_VAL);
	if (ret < 0) {
		dev_err(info->dev, "fail to write RTC_NONCE0_0 reg(%d)\n", ret);
		return ret;
	}

	if (pdata->init_time) {
		dev_info(info->dev, "initialize RTC time\n");
		ret = s2m_rtc_set_time(info->dev, pdata->init_time);
	} else {
		dev_info(info->dev,
			 "RTC initialize is not operated: This causes a weekday problem\n");
	}
	return ret;
}

static int s2m_rtc_probe(struct platform_device *pdev)
{
	struct s2mpg14_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpg14_platform_data *pdata = dev_get_platdata(iodev->dev);
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

	if (info->iodev->device_type != S2MPG14X) {
		ret = -ENXIO;
		goto err_rtc_init_reg;
	}
	info->alarm0_irq = irq_base + S2MPG14_IRQ_RTCA0_INT2;

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
		dev_err(&pdev->dev, "device_init_wakeup fail(%d)\n", ret);
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

	info->rtc_dev = devm_rtc_device_register(&pdev->dev, "s2mpg14-rtc",
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

#if IS_ENABLED(CONFIG_PM_SLEEP)
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
	struct s2mpg14_platform_data *pdata =
		dev_get_platdata(info->iodev->dev);

	if (info->wtsr_en || info->smpl_en || info->coldrst_en)
		s2m_rtc_disable_wtsr_smpl(info, pdata);

	if (system_state == SYSTEM_RESTART) {
		s2mpg14_update_reg(info->i2c, S2MPG14_RTC_WTSR,
				   COLDRST_TIMER_MASK | COLDRST_EN_MASK |
				   WTSRT_MASK | WTSR_EN_MASK,
				   GENMASK(6, 0));
	}
}

static const struct platform_device_id s2m_rtc_id[] = {
	{ "s2mpg14-rtc", 0 },
	{},
};

static struct platform_driver s2m_rtc_driver = {
	.driver = {
		   .name = "s2mpg14-rtc",
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
