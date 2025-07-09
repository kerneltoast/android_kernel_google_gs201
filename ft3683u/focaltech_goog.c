/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Copyright (c) 2021 Google LLC
 *    Author: TsoHsien(Blackbear) Chou <blackbearchou@google.com>
 */

#include "focaltech_core.h"
#include "focaltech_common.h"
#include "focaltech_test/focaltech_test.h"

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)

#include <goog_touch_interface.h>

static irqreturn_t goog_fts_irq_ts(int irq, void *data)
{
    struct fts_ts_data *ts_data = data;

    ts_data->isr_timestamp = ktime_get();
    return IRQ_WAKE_THREAD;
}

extern int int_test_has_interrupt;
static irqreturn_t goog_fts_irq_handler(int irq, void *data)
{
    int_test_has_interrupt++;
    fts_data->coords_timestamp = fts_data->isr_timestamp;
    fts_irq_read_report();

    return IRQ_HANDLED;
}

static int google_enter_normal_sensing(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int i = 0;
    u8 gesture_mode = 0;
    u8 power_mode = 0;
    mutex_lock(&ts_data->reg_lock);

    for (i = 0; i < 200; i++) {
        ret = fts_write_reg(FTS_REG_WAKEUP, FTS_WAKEUP_VALUE);
        if (ret < 0) {
          FTS_ERROR("Write reg(%x) = %x fail", FTS_REG_WAKEUP, FTS_WAKEUP_VALUE);
          goto exit;
        }

        ret = fts_read_reg(FTS_REG_POWER_MODE, &power_mode);
        if (ret < 0) {
          FTS_ERROR("read reg0xA5 fails");
          goto exit;
        }

        if (power_mode != 3)
            break;

        usleep_range(1000, 1000);
    }

    if (i >= 200) {
        FTS_ERROR("Enter normal mode failed");
        goto exit;
    } else {
        FTS_INFO("Enter normal mode (%d ms)", i);
    }


    ret = fts_read_reg(FTS_REG_GESTURE_EN, &gesture_mode);
    if (ret < 0) {
        FTS_ERROR("Read reg(%x) fails", FTS_REG_GESTURE_EN);
        goto exit;
    }
    if (gesture_mode) {
      FTS_INFO("Exit gesture mode");
      gesture_mode = 0;
      ret = fts_write_reg(FTS_REG_GESTURE_EN, gesture_mode);
      if (ret < 0) {
        FTS_ERROR("Write reg(%x) = %x fail", FTS_REG_GESTURE_EN, gesture_mode);
        goto exit;
      }
    }

exit:
    mutex_unlock(&ts_data->reg_lock);
    return ret;
}

static int goog_fts_ts_suspend(struct device *dev)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    FTS_FUNC_ENTER();

    if (ts_data->fw_loading) {
        FTS_INFO("fw upgrade in process, can't suspend");
        return 0;
    }

    FTS_INFO("Prepare to suspend device");
    /* Disable irq */
    fts_irq_disable();

    FTS_INFO("Do reset on suspend");
    fts_reset_proc(FTS_RESET_INTERVAL);

    ret = fts_wait_tp_to_valid();
    if (ret != 0) {
        FTS_ERROR("Suspend has been cancelled by wake up timeout");
        return ret;
    }
    FTS_INFO("Device has been reset");

    FTS_DEBUG("make TP enter into sleep mode");
    mutex_lock(&ts_data->reg_lock);
    ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
    ts_data->is_deepsleep = true;
    mutex_unlock(&ts_data->reg_lock);
    if (ret < 0)
      FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

    ret = fts_pinctrl_select_suspend(ts_data);
    if (ret < 0)
      FTS_ERROR("set pinctrl suspend fail, ret=%d", ret);

    FTS_FUNC_EXIT();
    return 0;
}

static int goog_fts_ts_resume(struct device *dev)
{
    struct fts_ts_data *ts_data = fts_data;
    int ret = 0;

    FTS_FUNC_ENTER();
    FTS_INFO("Prepare to resume device\n");

    ret = fts_pinctrl_select_normal(ts_data);
    if (ret < 0)
      FTS_ERROR("set pinctrl normal fail, ret=%d", ret);

    ret = google_enter_normal_sensing(ts_data);
    if (ret < 0) {
      FTS_ERROR("Fail to enter normal power mode, trigger reset to recover\n");
      fts_reset_proc(FTS_RESET_INTERVAL);

      ret = fts_wait_tp_to_valid();
      if (ret != 0) {
        FTS_ERROR("Resume has been cancelled by wake up timeout");
        return ret;
      }
    }

    fts_update_feature_setting(ts_data);

    ts_data->is_deepsleep = false;
    fts_irq_enable();

    FTS_FUNC_EXIT();
    FTS_INFO("Device resumed");
    return 0;
};

static const struct dev_pm_ops goog_fts_dev_pm_ops = {
    .suspend = goog_fts_ts_suspend,
    .resume = goog_fts_ts_resume,
};

extern int fts_test_get_raw(int *raw, u8 tx, u8 rx);
extern int fts_test_get_short(int *short_data, u8 tx, u8 rx);
extern int fts_test_get_short_ch_to_gnd(int *res, u8 *ab_ch, u8 tx, u8 rx);
extern int fts_test_get_short_ch_to_ch(int *res, u8 *ab_ch, u8 tx, u8 rx);
extern size_t google_internal_sttw_setting_read(char *buf, size_t buf_size);

// Reference: proc_test_raw_show
static int goog_selfttest_test_raw(void)
{
    int ret = 0;
    int i = 0;
    int node_num = 0;
    u8 tx = 0;
    u8 rx = 0;
    int *raw = NULL;
    bool result = 0;
    char print_buf[512];
    int count = 0;
    struct mc_sc_threshold *thr = &fts_ftest->ic.mc_sc.thr;

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }
    /* get Tx channel number */
    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }
    /* get Rx channel number */
    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx * rx;
    raw = fts_malloc(node_num * sizeof(int));
    if (!raw) {
        FTS_ERROR("malloc memory for raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /* get raw data */
    fts_test_get_raw(raw, tx, rx);
    result = compare_array(raw,
                           thr->rawdata_h_min,
                           thr->rawdata_h_max,
                           false);

    /* output raw data */
    count += scnprintf(print_buf + count, 512 - count, "     ");
    for (i = 0; i < rx; i++)
        count += scnprintf(print_buf + count, 512 - count, " RX%02d ", (i + 1));

    for (i = 0; i < node_num; i++) {
        if ((i % rx) == 0) {
            FTS_INFO("%s\n", &print_buf[0]);
            count = 0;
            count += scnprintf(print_buf + count, 512 - count, "TX%02d:%5d,",
                               (i / rx  + 1), raw[i]);
        } else
            count += scnprintf(print_buf + count, 512 - count, "%5d,", raw[i]);
    }

    FTS_INFO("%s\n", &print_buf[0]);
    count = 0;

    FTS_INFO("\n\n");
    FTS_INFO("Rawdata Test %s\n", result? "PASS" : "NG");

    if (!result)
      ret = -1;

exit:
    if (raw)
        fts_free(raw);

    fts_proc_test_exit();
    enter_work_mode();

    return ret;
}

// Reference: proc_test_short_show
static int goog_selfttest_test_short(void)
{
    int ret = 0;
    int i = 0;
    int node_num = 0;
    u8 tx = 0;
    u8 rx = 0;
    int *short_data = NULL;
    int *short_data_cg = NULL;
    int *short_data_cc = NULL;
    bool result = 1;
    bool cg_result = 1;
    bool cc_result = 1;
    int code = 0;
    struct fts_test *tdata = fts_ftest;
    u8 ab_ch[SC_NUM_MAX + 1] = { 0 };
    u8 ab_ch_num = 0;
    int temp = 0;
    int j = 0;
    int adc_cnt = 0;
    bool is_cc_short = false;
    bool is_cg_short = false;
    int tmp_num = 0;
    char print_buf[512];
    int count = 0;

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }
    /* get Tx channel number */
    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }
    /* get Rx channel number */
    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx + rx;
    short_data = fts_malloc(node_num * sizeof(int));
    short_data_cg = fts_malloc(node_num * sizeof(int));
    short_data_cc = fts_malloc(node_num * sizeof(int));
    if (!short_data || !short_data_cg || !short_data_cc) {
        FTS_ERROR("malloc memory for raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /* get short all data */
    fts_test_get_short(short_data, tx, rx);

    for (i = 0; i < node_num; i++) {
        code = short_data[i];

        if (code > 1500) {
            FTS_INFO("adc(%d) > 1500 fail", code);
            result = false;
            continue;
        }

        if ((212 - ((code * 250 / 2047) + 40)) == 0) {
            short_data[i] = 50000;
            continue;
        }
        short_data[i] = fts_abs(((code * 25 / 2047 + 4) * 2005) /
                                (212 - ((code * 250 / 2047) + 40)));
        if (short_data[i] < tdata->ic.mc_sc.thr.basic.short_cg) {
            ab_ch_num++;
            ab_ch[ab_ch_num] = i;
            result = false;
        }
    }
    /* output short data */
    count += scnprintf(print_buf + count, 512 - count, "TX:");
    for (i = 0; i < tx; i++) {
        count += scnprintf(print_buf + count, 512 - count, "%d,", short_data[i]);
    }
    FTS_INFO("%s\n", &print_buf[0]);
    count = 0;

    count += scnprintf(print_buf + count, 512 - count, "RX:");
    for (i = tx; i < node_num; i++) {
        count += scnprintf(print_buf + count, 512 - count,"%d,", short_data[i]);
    }
    FTS_INFO("%s\n", &print_buf[0]);
    count = 0;

    if (result == true) goto short_end;

    ab_ch[0] = ab_ch_num;
    if (ab_ch_num) {
        FTS_INFO("\nabnormal ch:[%*ph]\n", ab_ch_num, ab_ch);
    }
    /********************get short cg********************/
    fts_test_get_short_ch_to_gnd(short_data_cg, ab_ch, tx, rx);
    for (i = 0; i < ab_ch_num; i++) {
        temp = short_data_cg[i];
        if ((212 - ((temp * 250 / 2047) + 40)) == 0) {
            short_data_cg[i] = 50000;
            continue;
        }
        short_data_cg[i] = fts_abs(((temp * 25 / 2047 + 4) * 2005) /
                                   (212 - ((temp * 250 / 2047) + 40)));
        if (short_data_cg[i] < tdata->ic.mc_sc.thr.basic.short_cg) {
            cg_result = false;
            if (!is_cg_short) {
                FTS_INFO("\nGND Short:\n");
                is_cg_short = true;
            }

            if (ab_ch[i + 1] <= tx) {
                count += scnprintf(print_buf + count, 512 - count,
                    "Tx%d with GND:", ab_ch[i + 1]);
            } else {
                count += scnprintf(print_buf + count, 512 - count,
                    "Rx%d with GND:", (ab_ch[i + 1] - tx));
            }
            count += scnprintf(print_buf + count, 512 - count,
                "%d(K)", short_data_cg[i]);
            FTS_INFO("%s\n", &print_buf[0]);
            count = 0;
        }
    }


    /********************get short cc********************/
    tmp_num = ab_ch_num * (ab_ch_num - 1) / 2;
    tmp_num = (tmp_num > node_num) ? node_num : tmp_num;
    fts_test_get_short_ch_to_ch(short_data_cc, ab_ch, tx, rx);

    for (i = 0; i < ab_ch_num; i++) {
        for (j = i + 1; j < ab_ch_num; j++) {
            if (adc_cnt >= tmp_num)
                break;

            temp = short_data_cc[adc_cnt];
            if ((212 - ((temp * 250 / 2047) + 40)) == 0) {
                short_data_cc[adc_cnt] = 50000;
                continue;
            }
            short_data_cc[adc_cnt] = fts_abs(((temp * 25 / 2047 + 4) * 2005) /
                                           (212 - ((temp * 250 / 2047) + 40)));
            if (short_data_cc[adc_cnt] < tdata->ic.mc_sc.thr.basic.short_cc) {
                cc_result = false;
                if (!is_cc_short) {
                    FTS_INFO("\nMutual Short:\n");
                    is_cc_short = true;
                }

                if (ab_ch[i + 1] <= tx) {
                    count += scnprintf(print_buf + count, 512 - count,
                        "Tx%d with", (ab_ch[i + 1]));
                } else {
                    count += scnprintf(print_buf + count, 512 - count,
                        "Rx%d with", (ab_ch[i + 1] - tx));
                }

                if (ab_ch[j + 1] <= tx) {
                    count += scnprintf(print_buf + count, 512 - count,
                        " Tx%d", (ab_ch[j + 1] ) );
                } else {
                    count += scnprintf(print_buf + count, 512 - count,
                        " Rx%d", (ab_ch[j + 1] - tx));
                }
                count += scnprintf(print_buf + count, 512 - count,
                    ":%d(K)\n", short_data_cc[adc_cnt]);
                FTS_INFO("%s\n", &print_buf[0]);
                count = 0;
            }
            adc_cnt++;
        }
    }

short_end:
    FTS_INFO("\n\n");
    FTS_INFO("Short Test %s\n", result? "PASS" : "NG");
    if (!result)
      ret = -1;

exit:
    if (short_data)
        fts_free(short_data);
    fts_free(short_data_cg);
    fts_free(short_data_cc);

    fts_proc_test_exit();
    enter_work_mode();

    return ret;
}

static int gti_selftest(void *private_data, struct gti_selftest_cmd *cmd)
{
    int ret = 0;
    cmd->result = GTI_SELFTEST_RESULT_FAIL;

    ret = goog_selfttest_test_raw();
    if (ret < 0) {
        FTS_ERROR("goog_selfttest_test_raw failed,ret=%d\n", ret);
        return ret;
    }

    ret = goog_selfttest_test_short();
    if (ret < 0) {
        FTS_ERROR("goog_selfttest_test_short failed,ret=%d\n", ret);
        return ret;
    }

    cmd->result = GTI_SELFTEST_RESULT_PASS;
    return 0;
}

// Reference: proc_test_fwver_show
static int goog_internel_get_fw_version(u8 *fw_major_ver,
        u8 *fw_minor_ver, u8 *vendor_id)
{
    int ret = 0;

    ret = fts_read_reg(FTS_REG_FW_MAJOR_VER, fw_major_ver);
    if (ret < 0) {
        FTS_ERROR("FWVER read major version fail,ret=%d\n", ret);
        goto exit;
    }

    ret = fts_read_reg(FTS_REG_FW_MINOR_VER, fw_minor_ver);
    if (ret < 0) {
        FTS_ERROR("FWVER read minor version fail,ret=%d\n", ret);
        goto exit;
    }

    ret = fts_read_reg(FTS_REG_VENDOR_ID, vendor_id);
    if (ret < 0) {
        FTS_ERROR("FWVER read vendor id fail,ret=%d\n", ret);
        goto exit;
    }

    FTS_INFO("Vendor ID:%#02x, Firmware Ver:%02x.%02x\n",
             *vendor_id, *fw_major_ver, *fw_minor_ver);
exit:
    return ret;
}

// Reference: fts_driverinfo_show
static int gti_get_fw_version(void *private_data,
                              struct gti_fw_version_cmd *cmd)
{
    int ret = 0;
    int count = 0;
    u8 fw_major_ver = 0;
    u8 fw_minor_ver = 0;
    u8 vendor_id = 0;
    struct fts_ts_data *ts_data = private_data;
    struct fts_ts_platform_data *pdata = ts_data->pdata;
    char *buf;
    size_t buf_size = 0;

    ret = goog_internel_get_fw_version(&fw_major_ver,
        &fw_minor_ver, &vendor_id);
    if (ret < 0)
      goto exit;

    buf = cmd->buffer;
    buf_size = sizeof(cmd->buffer);

    count += snprintf(buf + count, buf_size - count, "\n");

    count += snprintf(buf + count, buf_size - count, "Firmware Ver:%02x.%02x\n",
                      fw_major_ver, fw_minor_ver);

    count += snprintf(buf + count, buf_size - count, "Vendor ID:%#02x\n",
                      vendor_id);

    count += snprintf(buf + count, buf_size - count, "Driver Ver:%s\n",
                      FTS_DRIVER_VERSION);

    count += snprintf(buf + count, buf_size - count, "Resolution:(%d,%d)~(%d,%d)\n",
                      pdata->x_min, pdata->y_min, pdata->x_max, pdata->y_max);

    count += snprintf(buf + count, buf_size - count, "Max Touches:%d\n",
                      pdata->max_touch_number);

    count += snprintf(buf + count, buf_size - count,
                      "reset gpio:%d,int gpio:%d,irq:%d\n",
                      pdata->reset_gpio, pdata->irq_gpio, ts_data->irq);

    count += snprintf(buf + count, buf_size - count, "IC ID:0x%02x%02x\n",
                      ts_data->ic_info.ids.chip_idh,
                      ts_data->ic_info.ids.chip_idl);

    count += snprintf(buf + count, buf_size - count,
                      "BUS:%s,mode:%d,max_freq:%d\n", "SPI",
                      ts_data->spi->mode, ts_data->spi->max_speed_hz);

    count += google_internal_sttw_setting_read(buf + count, buf_size - count);
exit:

    return ret;
}

// Reference: fts_irq_store
static int gti_set_irq_mode(void *private_data,
                              struct gti_irq_cmd *cmd)
{
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    if (cmd->setting == GTI_IRQ_MODE_ENABLE) {
        FTS_INFO("enable irq");
        fts_irq_enable();
    } else {
        FTS_INFO("disable irq");
        fts_irq_disable();
    }
    mutex_unlock(&input_dev->mutex);
    return 0;
}

// Reference: fts_irq_show
static int gti_get_irq_mode(void *private_data,
                              struct gti_irq_cmd *cmd)
{
    cmd->setting = fts_data->irq_disabled ? GTI_IRQ_MODE_DISABLE
        : GTI_IRQ_MODE_ENABLE;

    return 0;
}

// Reference: fts_hw_reset_show
static int gti_reset(void *private_data, struct gti_reset_cmd *cmd)
{
    struct input_dev *input_dev = fts_data->input_dev;
    struct fts_ts_data *ts_data = private_data;
    int ret = 0;

    mutex_lock(&input_dev->mutex);
    if (cmd->setting == GTI_RESET_MODE_SW) {
      ret = fts_write_reg(FTS_TMP_REG_SOFT_RESET, 0xAA);
      if (ret < 0) {
        FTS_ERROR("write 0xAA to reg 0xFC fails");
        goto exit;
      }

      ret = fts_write_reg(FTS_TMP_REG_SOFT_RESET, 0x66);
      if (ret < 0) {
        FTS_ERROR("write 0x66 to reg 0xFC fails");
        goto exit;
      }
    } else if (cmd->setting == GTI_RESET_MODE_HW || cmd->setting == GTI_RESET_MODE_AUTO) {
      fts_reset_proc(0);
      fts_update_feature_setting(ts_data);
    } else {
      ret = -EOPNOTSUPP;
    }

exit:
    mutex_unlock(&input_dev->mutex);

    return ret;
}

// Reference: proc_grip_read
static int gti_get_grip_mode(void *private_data, struct gti_grip_cmd *cmd)
{
    struct fts_ts_data *ts_data = fts_data;

    cmd->setting = (ts_data->enable_fw_grip % 2) ?
        GTI_GRIP_ENABLE : GTI_GRIP_DISABLE;

    return 0;
}

// Reference: proc_grip_write
static int gti_set_grip_mode(void *private_data, struct gti_grip_cmd *cmd)
{
    struct fts_ts_data *ts_data = fts_data;
    int ret = 0;

    ts_data->enable_fw_grip = (cmd->setting == GTI_GRIP_ENABLE) ?
        FW_GRIP_ENABLE : FW_GRIP_DISABLE;

    FTS_INFO("switch fw_grip to %u\n", ts_data->enable_fw_grip);

    ret = fts_set_grip_mode(ts_data, ts_data->enable_fw_grip);
    if (ret < 0)
        return ret;

    return 0;
}

// Reference:
static int gti_ping(void *private_data, struct gti_ping_cmd *cmd)
{
    int ret = 0;
    u8 fw_major_ver = 0;
    u8 fw_minor_ver = 0;
    u8 vendor_id = 0;

    ret = goog_internel_get_fw_version(&fw_major_ver,
        &fw_minor_ver, &vendor_id);

    return ret;
}

// Reference:
static int gti_get_sensing_mode(void *private_data, struct gti_sensing_cmd *cmd)
{
    struct fts_ts_data *ts_data = fts_data;

    cmd->setting = (!ts_data->is_deepsleep) ?
        GTI_SENSING_MODE_ENABLE : GTI_SENSING_MODE_DISABLE;
    return 0;
}

// Reference:
static int gti_set_sensing_mode(void *private_data, struct gti_sensing_cmd *cmd)
{
    if (cmd->setting == GTI_SENSING_MODE_ENABLE) {
        goog_fts_ts_resume(NULL);
    } else {
        goog_fts_ts_suspend(NULL);
    }

    return 0;
}

static void goog_register_options(struct gti_optional_configuration *options,
    struct fts_ts_data *ts)
{
    //options->calibrate = gti_calibrate;
    //options->get_context_driver = gti_get_context_driver;
    //options->get_coord_filter_enabled = get_coord_filter_enabled;
    options->get_fw_version = gti_get_fw_version;
    options->get_grip_mode = gti_get_grip_mode;
    options->get_irq_mode = gti_get_irq_mode;
    //options->get_mutual_sensor_data = get_mutual_sensor_data;
    options->get_palm_mode = gti_get_palm_mode;
    options->get_scan_mode = gti_get_scan_mode;
    options->get_screen_protector_mode = gti_get_screen_protector_mode;
    //options->get_self_sensor_data = get_self_sensor_data;
    options->get_sensing_mode = gti_get_sensing_mode;
    options->ping = gti_ping;
    //options->post_irq_thread_fn = goodix_ts_post_threadirq_func;
    options->reset = gti_reset;
    options->selftest = gti_selftest;
    //options->set_continuous_report = set_continuous_report;
    //options->set_coord_filter_enabled = set_coord_filter_enabled;
    //options->set_gesture_config = syna_set_gesture_config;
    options->set_grip_mode = gti_set_grip_mode;
    options->set_irq_mode = gti_set_irq_mode;
    options->set_palm_mode = gti_set_palm_mode;
    //options->set_panel_speed_mode = gti_set_panel_speed_mode;
    //options->set_report_rate = gti_set_report_rate;
    options->set_scan_mode = gti_set_scan_mode;
    options->set_screen_protector_mode = gti_set_screen_protector_mode;
    options->set_sensing_mode = gti_set_sensing_mode;
}

static int gti_default_handler(void *private_data, enum gti_cmd_type cmd_type,
    struct gti_union_cmd_data *cmd)
{
    int ret = -EOPNOTSUPP;

    switch (cmd_type) {
    case GTI_CMD_NOTIFY_DISPLAY_STATE:
    case GTI_CMD_NOTIFY_DISPLAY_VREFRESH:
    case GTI_CMD_SET_HEATMAP_ENABLED:
        ret = 0;
        break;
    default:
        break;
    }

    return ret;
}


void goog_fts_input_report_b(struct fts_ts_data *data)
{
    int i = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 max_touch_num = data->pdata->max_touch_number;
    struct ts_event *events = data->events;
    struct goog_touch_interface *gti = data->gti;
    struct input_dev *input_dev = data->input_dev;

    goog_input_lock(gti);

    goog_input_set_timestamp(gti, input_dev, data->coords_timestamp);

    for (i = 0; i < data->touch_point; i++) {
        if (EVENT_DOWN(events[i].flag)) {
            goog_input_mt_slot(gti, input_dev, events[i].id);
            goog_input_mt_report_slot_state(gti, input_dev, MT_TOOL_FINGER, true);

            goog_input_report_abs(gti, input_dev, ABS_MT_TOUCH_MAJOR, events[i].major);
            goog_input_report_abs(gti, input_dev, ABS_MT_TOUCH_MINOR, events[i].minor);
            goog_input_report_abs(gti, input_dev, ABS_MT_POSITION_X, events[i].x);
            goog_input_report_abs(gti, input_dev, ABS_MT_POSITION_Y, events[i].y);
            goog_input_report_abs(gti, input_dev, ABS_MT_ORIENTATION,
                                  (s16) (((s8) events[i].orientation) * 2048 / 45));

            touchs |= BIT(events[i].id);
            data->touchs |= BIT(events[i].id);
            if ((data->log_level >= 2) ||
                ((1 == data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
              FTS_DEBUG("[B]P%d(%d, %d)[ma:%d(%d),mi:%d(%d),p:%d,o:%d] DOWN!",
                        events[i].id,
                        events[i].x,
                        events[i].y,
                        events[i].major, events[i].major / data->pdata->mm2px,
                        events[i].minor, events[i].minor / data->pdata->mm2px,
                        events[i].p,
                        events[i].orientation);
            }
        } else {  //EVENT_UP
            goog_input_mt_slot(gti, input_dev, events[i].id);
            goog_input_mt_report_slot_state(gti, input_dev, MT_TOOL_FINGER, false);

            data->touchs &= ~BIT(events[i].id);
            if (data->log_level >= 1) {
                FTS_DEBUG("[B1]P%d UP!", events[i].id);
            }
        }
    }

    if (unlikely(data->touchs ^ touchs)) {
        for (i = 0; i < max_touch_num; i++)  {
            if (BIT(i) & (data->touchs ^ touchs)) {
                if (data->log_level >= 1) {
                    FTS_DEBUG("[B2]P%d UP!", i);
                }
                va_reported = true;
                goog_input_mt_slot(gti, input_dev, i);
                goog_input_mt_report_slot_state(gti, input_dev, MT_TOOL_FINGER, false);
            }
        }
    }
    data->touchs = touchs;

    if (va_reported) {
        /* touchs==0, there's no point but key */
        if (EVENT_NO_DOWN(data) || (!touchs)) {
            if (data->log_level >= 1) {
                FTS_DEBUG("[B]Points All Up!");
            }
            goog_input_report_key(gti, input_dev, BTN_TOUCH, 0);
        } else {
            goog_input_report_key(gti, input_dev, BTN_TOUCH, 1);
        }
    }
    goog_input_sync(gti, input_dev);

    goog_input_unlock(gti);
}

int goog_parse_dt(struct device_node *np, struct fts_ts_platform_data *pdata)
{
    int panel_id = -1;

    if (!np || !pdata)
      return -EPROBE_DEFER;

    panel_id = goog_get_panel_id(np);
    if (panel_id < 0) {
        FTS_ERROR("Unable to get panel");
        return -EPROBE_DEFER;
    }
    pdata->panel_id = panel_id;

    goog_get_firmware_name(np, panel_id, pdata->fw_name,
            sizeof(pdata->fw_name));
    goog_get_test_limits_name(np, panel_id, pdata->test_limits_name,
            sizeof(pdata->test_limits_name));

    return 0;
}

void goog_gti_probe(struct fts_ts_data *ts_data)
{
    struct gti_optional_configuration *options = NULL;
    struct fts_ts_platform_data *pdata = NULL;
    int retval = 0;
    int irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;

    if (!ts_data || !ts_data->dev || !ts_data->pdata) {
        FTS_ERROR("ts_data / ts_data->dev / ts_data->pdata is null");
        goto err;
    }

    /* release the interrupt and register the gti irq later. */
    free_irq(ts_data->irq, ts_data);

    options = devm_kzalloc(ts_data->dev,
        sizeof(struct gti_optional_configuration), GFP_KERNEL);
    if (!options) {
        FTS_ERROR("options devm_kzalloc fail");
        goto err;
    }
    goog_register_options(options, ts_data);

    ts_data->gti = goog_touch_interface_probe(
        ts_data, ts_data->dev, ts_data->input_dev,
        gti_default_handler, options);
    if (!ts_data->gti) {
        FTS_ERROR("Failed to initialize GTI");
        goto err;
    }

    retval = goog_pm_register_notification(ts_data->gti, &goog_fts_dev_pm_ops);
    if (retval < 0)
        FTS_ERROR("Failed to register GTI pm");


    FTS_INFO("Register IRQ by GTI.");
    pdata = ts_data->pdata;
    ts_data->irq = gpio_to_irq(pdata->irq_gpio);

    FTS_INFO("gti register irq:%d, flag:%x", ts_data->irq, irq_flags);
    retval = goog_devm_request_threaded_irq(ts_data->gti, ts_data->dev,
        ts_data->irq, goog_fts_irq_ts, goog_fts_irq_handler,
        irq_flags, "fts_ts", ts_data);

    if (retval < 0)
        FTS_ERROR("Failed to request GTI IRQ");

err:
    if (options)
      devm_kfree(ts_data->dev, options);
}

void goog_gti_remove(struct fts_ts_data *ts_data)
{
    if (!ts_data->gti)
      return;

    goog_pm_unregister_notification(ts_data->gti);

    goog_devm_free_irq(ts_data->gti, ts_data->dev, ts_data->irq);
    goog_touch_interface_remove(ts_data->gti);
    ts_data->gti = NULL;
}

#endif /* IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) */

