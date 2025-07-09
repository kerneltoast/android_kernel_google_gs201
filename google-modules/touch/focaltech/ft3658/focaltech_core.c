/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*****************************************************************************
*
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#elif defined(CONFIG_ARCH_MSM)
#include <linux/msm_drm_notify.h>
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1     /* Early-suspend level */
#endif
#include <linux/types.h>
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define FTS_DRIVER_PEN_NAME                 "fts_ts,pen"
#define INTERVAL_READ_REG                   200  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
static int register_panel_bridge(struct fts_ts_data *ts);
static void unregister_panel_bridge(struct drm_bridge *bridge);
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
static void fts_offload_set_running(struct fts_ts_data *ts_data, bool running);
static void fts_populate_frame(struct fts_ts_data *ts_data, int populate_channel_types);
static void fts_offload_push_coord_frame(struct fts_ts_data *ts);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD) || \
    IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
static int fts_get_heatmap(struct fts_ts_data *ts_data);
#endif
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);
static void fts_update_motion_filter(struct fts_ts_data *ts, u8 touches);

static char *status_list_str[STATUS_CNT_END] = {
    "Baseline refreshed",
    "Baseline refreshed",
    "Palm",
    "Water",
    "Grip",
    "Glove",
    "Edge palm",
    "RESET",
};

static char *feature_list_str[FW_CNT_END] = {
    "FW_GLOVE",
    "FW_GRIP",
    "FW_PALM",
    "FW_HEATMAP",
    "FW_CONTINUOUS",
};

static char *status_baseline_refresh_str[4] = {
    "Baseline refreshed: none",
    "Baseline refreshed: removing touch",
    "Baseline refreshed: removing water",
    "Baseline refreshed: removing shell iron",
};

int fts_check_cid(struct fts_ts_data *ts_data, u8 id_h)
{
    int i = 0;
    struct ft_chip_id_t *cid = &ts_data->ic_info.cid;
    u8 cid_h = 0x0;

    if (cid->type == 0)
        return -ENODATA;

    for (i = 0; i < FTS_MAX_CHIP_IDS; i++) {
        cid_h = ((cid->chip_ids[i] >> 8) & 0x00FF);
        if (cid_h && (id_h == cid_h)) {
            return 0;
        }
    }

    return -ENODATA;
}

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int fts_wait_tp_to_valid(void)
{
    int ret = 0;
    int cnt = 0;
    u8 idh = 0;
    struct fts_ts_data *ts_data = fts_data;
    u8 chip_idh = ts_data->ic_info.ids.chip_idh;
    u16 retry_duration = 0;

    do {
        ret = fts_read_reg(FTS_REG_CHIP_ID, &idh);

        if (ret == 0 && ((idh == chip_idh) || (fts_check_cid(ts_data, idh) == 0))) {
            FTS_INFO("TP Ready,Device ID:0x%02x, retry:%d", idh, cnt);
            return 0;
        }

        cnt++;
        if (ret == -EIO) {
            fts_reset_proc(FTS_RESET_INTERVAL);
            retry_duration += FTS_RESET_INTERVAL;
        } else {
            msleep(INTERVAL_READ_REG);
            retry_duration += INTERVAL_READ_REG;
        }

    } while (retry_duration < TIMEOUT_READ_REG);

    FTS_ERROR("Wait tp timeout");
    return -ETIMEDOUT;
}

/*****************************************************************************
*  Name: fts_tp_state_recovery
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_tp_state_recovery(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    /* wait tp stable */
    fts_wait_tp_to_valid();
    /* recover all firmware modes based on the settings of driver side. */
    fts_ex_mode_recovery(ts_data);
    /* recover TP gesture state 0xD0 */
    fts_gesture_recovery(ts_data);
    FTS_FUNC_EXIT();
}

int fts_reset_proc(int hdelayms)
{
    FTS_DEBUG("tp reset");
    gpio_direction_output(fts_data->pdata->reset_gpio, 0);
    /* The minimum reset duration is 1 ms. */
    msleep(1);
    gpio_direction_output(fts_data->pdata->reset_gpio, 1);
    if (hdelayms) {
        msleep(hdelayms);
    }

    return 0;
}

void fts_irq_disable(void)
{
    unsigned long irqflags;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (!fts_data->irq_disabled) {
        disable_irq_nosync(fts_data->irq);
        fts_data->irq_disabled = true;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_irq_enable(void)
{
    unsigned long irqflags = 0;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (fts_data->irq_disabled) {
        enable_irq(fts_data->irq);
        fts_data->irq_disabled = false;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_hid2std(void)
{
    int ret = 0;
    u8 buf[3] = {0xEB, 0xAA, 0x09};

    if (fts_data->bus_type != FTS_BUS_TYPE_I2C)
        return;

    ret = fts_write(buf, 3);
    if (ret < 0) {
        FTS_ERROR("hid2std cmd write fail");
    } else {
        msleep(10);
        buf[0] = buf[1] = buf[2] = 0;
        ret = fts_read(NULL, 0, buf, 3);
        if (ret < 0) {
            FTS_ERROR("hid2std cmd read fail");
        } else if ((0xEB == buf[0]) && (0xAA == buf[1]) && (0x08 == buf[2])) {
            FTS_DEBUG("hidi2c change to stdi2c successful");
        } else {
            FTS_DEBUG("hidi2c change to stdi2c not support or fail");
        }
    }
}

static int fts_match_cid(struct fts_ts_data *ts_data,
                         u16 type, u8 id_h, u8 id_l, bool force)
{
#ifdef FTS_CHIP_ID_MAPPING
    u32 i = 0;
    u32 j = 0;
    struct ft_chip_id_t chip_id_list[] = FTS_CHIP_ID_MAPPING;
    u32 cid_entries = sizeof(chip_id_list) / sizeof(struct ft_chip_id_t);
    u16 id = (id_h << 8) + id_l;

    memset(&ts_data->ic_info.cid, 0, sizeof(struct ft_chip_id_t));
    for (i = 0; i < cid_entries; i++) {
        if (!force && (type == chip_id_list[i].type)) {
            break;
        } else if (force && (type == chip_id_list[i].type)) {
            FTS_INFO("match cid,type:0x%x", (int)chip_id_list[i].type);
            ts_data->ic_info.cid = chip_id_list[i];
            return 0;
        }
    }

    if (i >= cid_entries) {
        return -ENODATA;
    }

    for (j = 0; j < FTS_MAX_CHIP_IDS; j++) {
        if (id == chip_id_list[i].chip_ids[j]) {
            FTS_DEBUG("cid:%x==%x", id, chip_id_list[i].chip_ids[j]);
            FTS_INFO("match cid,type:0x%x", (int)chip_id_list[i].type);
            ts_data->ic_info.cid = chip_id_list[i];
            return 0;
        }
    }

    return -ENODATA;
#else
    return -EINVAL;
#endif
}

static int fts_get_chip_types(
    struct fts_ts_data *ts_data,
    u8 id_h, u8 id_l, bool fw_valid)
{
    u32 i = 0;
    struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
    u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

    if ((0x0 == id_h) || (0x0 == id_l)) {
        FTS_ERROR("id_h/id_l is 0");
        return -EINVAL;
    }

    FTS_DEBUG("verify id:0x%02x%02x", id_h, id_l);
    for (i = 0; i < ctype_entries; i++) {
        if (VALID == fw_valid) {
            if (((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl))
                || (!fts_match_cid(ts_data, ctype[i].type, id_h, id_l, 0)))
                break;
        } else {
            if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
                || ((id_h == ctype[i].pb_idh) && (id_l == ctype[i].pb_idl))
                || ((id_h == ctype[i].bl_idh) && (id_l == ctype[i].bl_idl))) {
                break;
            }
        }
    }

    if (i >= ctype_entries) {
        return -ENODATA;
    }

    fts_match_cid(ts_data, ctype[i].type, id_h, id_l, 1);
    ts_data->ic_info.ids = ctype[i];
    return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 *id)
{
    int ret = 0;
    u8 chip_id[2] = { 0 };
    u8 id_cmd[4] = { 0 };
    u32 id_cmd_len = 0;

    id_cmd[0] = FTS_CMD_START1;
    id_cmd[1] = FTS_CMD_START2;
    ret = fts_write(id_cmd, 2);
    if (ret < 0) {
        FTS_ERROR("start cmd write fail");
        return ret;
    }

    msleep(FTS_CMD_START_DELAY);
    id_cmd[0] = FTS_CMD_READ_ID;
    id_cmd[1] = id_cmd[2] = id_cmd[3] = 0x00;
    if (ts_data->ic_info.is_incell)
        id_cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
    else
        id_cmd_len = FTS_CMD_READ_ID_LEN;
    ret = fts_read(id_cmd, id_cmd_len, chip_id, 2);
    if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
        FTS_ERROR("read boot id fail,read:0x%02x%02x", chip_id[0], chip_id[1]);
        return -EIO;
    }

    id[0] = chip_id[0];
    id[1] = chip_id[1];
    return 0;
}

/*****************************************************************************
* Name: fts_get_ic_information
* Brief: read chip id to get ic information, after run the function, driver w-
*        ill know which IC is it.
*        If cant get the ic information, maybe not focaltech's touch IC, need
*        unregister the driver
* Input:
* Output:
* Return: return 0 if get correct ic information, otherwise return error code
*****************************************************************************/
static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int cnt = 0;
    u8 chip_id[2] = { 0 };

    ts_data->ic_info.is_incell = FTS_CHIP_IDC;
    ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;

    do {
        ret = fts_read_reg(FTS_REG_CHIP_ID, &chip_id[0]);
        ret = fts_read_reg(FTS_REG_CHIP_ID2, &chip_id[1]);
        if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
            FTS_DEBUG("chip id read invalid, read:0x%02x%02x",
                      chip_id[0], chip_id[1]);
        } else {
            ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], VALID);
            if (!ret)
                break;
            else
                FTS_DEBUG("TP not ready, read:0x%02x%02x",
                          chip_id[0], chip_id[1]);
        }

        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    if ((cnt * INTERVAL_READ_REG) >= TIMEOUT_READ_REG) {
        FTS_INFO("fw is invalid, need read boot id");
        if (ts_data->ic_info.hid_supported) {
            fts_hid2std();
        }

        ret = fts_read_bootid(ts_data, &chip_id[0]);
        if (ret <  0) {
            FTS_ERROR("read boot id fail");
            return ret;
        }

        ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], INVALID);
        if (ret < 0) {
            FTS_ERROR("can't get ic informaton");
            return ret;
        }
    }

    FTS_INFO("get ic information, chip id = 0x%02x%02x(cid type=0x%x)",
             ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl,
             ts_data->ic_info.cid.type);

    return 0;
}

/*****************************************************************************
*  Reprot related
*****************************************************************************/
static void fts_show_touch_buffer(u8 *data, int datalen)
{
    int i = 0;
    int count = 0;
    char *tmpbuf = NULL;

    tmpbuf = kzalloc(1024, GFP_KERNEL);
    if (!tmpbuf) {
        FTS_ERROR("tmpbuf zalloc fail");
        return;
    }

    for (i = 0; i < datalen; i++) {
        count += scnprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
        if (count >= 1024)
            break;
    }
    FTS_DEBUG("point buffer:%s", tmpbuf);

    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }
}

void fts_release_all_finger(void)
{
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;
#if FTS_MT_PROTOCOL_B_EN
    u32 finger_count = 0;
    u32 max_touches = ts_data->pdata->max_touch_number;
#endif

    mutex_lock(&ts_data->report_mutex);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    for (finger_count = 0; finger_count < max_touches; finger_count++) {
        ts_data->offload.coords[finger_count].status = COORD_STATUS_INACTIVE;
        ts_data->offload.coords[finger_count].major = 0;
        ts_data->offload.coords[finger_count].minor = 0;
        ts_data->offload.coords[finger_count].pressure = 0;
        ts_data->offload.coords[finger_count].rotation = 0;
    }

    if (ts_data->touch_offload_active_coords && ts_data->offload.offload_running) {
        fts_offload_push_coord_frame(ts_data);
    } else {
#endif
#if FTS_MT_PROTOCOL_B_EN
    for (finger_count = 0; finger_count < max_touches; finger_count++) {
        input_mt_slot(input_dev, finger_count);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
#else
    input_mt_sync(input_dev);
#endif
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_sync(input_dev);

#if FTS_PEN_EN
    input_report_key(ts_data->pen_dev, BTN_TOOL_PEN, 0);
    input_report_key(ts_data->pen_dev, BTN_TOUCH, 0);
    input_sync(ts_data->pen_dev);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    }
#endif

    ts_data->touchs = 0;
    ts_data->key_state = 0;
    mutex_unlock(&ts_data->report_mutex);
}

/*****************************************************************************
* Name: fts_input_report_key
* Brief: process key events,need report key-event if key enable.
*        if point's coordinate is in (x_dim-50,y_dim-50) ~ (x_dim+50,y_dim+50),
*        need report it to key event.
*        x_dim: parse from dts, means key x_coordinate, dimension:+-50
*        y_dim: parse from dts, means key y_coordinate, dimension:+-50
* Input:
* Output:
* Return: return 0 if it's key event, otherwise return error code
*****************************************************************************/
static int fts_input_report_key(struct fts_ts_data *data, int index)
{
    int i = 0;
    int x = data->events[index].x;
    int y = data->events[index].y;
    int *x_dim = &data->pdata->key_x_coords[0];
    int *y_dim = &data->pdata->key_y_coords[0];

    if (!data->pdata->have_key) {
        return -EINVAL;
    }
    for (i = 0; i < data->pdata->key_number; i++) {
        if ((x >= x_dim[i] - FTS_KEY_DIM) && (x <= x_dim[i] + FTS_KEY_DIM) &&
            (y >= y_dim[i] - FTS_KEY_DIM) && (y <= y_dim[i] + FTS_KEY_DIM)) {
            if (EVENT_DOWN(data->events[index].flag)
                && !(data->key_state & (1 << i))) {
                input_report_key(data->input_dev, data->pdata->keys[i], 1);
                data->key_state |= (1 << i);
                FTS_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
            } else if (EVENT_UP(data->events[index].flag)
                       && (data->key_state & (1 << i))) {
                input_report_key(data->input_dev, data->pdata->keys[i], 0);
                data->key_state &= ~(1 << i);
                FTS_DEBUG("Key%d(%d,%d) Up!", i, x, y);
            }
            return 0;
        }
    }
    return -EINVAL;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_report_b(struct fts_ts_data *data)
{
    int i = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 max_touch_num = data->pdata->max_touch_number;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (fts_input_report_key(data, i) == 0) {
            continue;
        }

        va_reported = true;

        if (EVENT_DOWN(events[i].flag)) {
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
            data->offload.coords[events[i].id].status = COORD_STATUS_FINGER;
            data->offload.coords[events[i].id].x = events[i].x;
            data->offload.coords[events[i].id].y = events[i].y;
            data->offload.coords[events[i].id].pressure = events[i].p;
            data->offload.coords[events[i].id].major = events[i].major;
            data->offload.coords[events[i].id].minor = events[i].minor;
            /* Rotation is not supported by firmware */
            data->offload.coords[events[i].id].rotation = 0;
            if (!data->offload.offload_running) {
#endif
            input_mt_slot(data->input_dev, events[i].id);
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x00;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x00;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].major);
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MINOR, events[i].minor);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
            }
#endif
            touchs |= BIT(events[i].id);
            data->touchs |= BIT(events[i].id);
            if ((data->log_level >= 2) ||
                ((1 == data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
                FTS_DEBUG("[B]P%d(%d, %d)[ma:%d,mi:%d,p:%d] DOWN!",
                          events[i].id,
                          events[i].x,
                          events[i].y,
                          events[i].major,
                          events[i].minor,
                          events[i].p);
            }
        } else {  //EVENT_UP
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
            data->offload.coords[events[i].id].status = COORD_STATUS_INACTIVE;
            if (!data->offload.offload_running) {
#endif
                input_mt_slot(data->input_dev, events[i].id);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
            }
#endif
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
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
                data->offload.coords[i].status = COORD_STATUS_INACTIVE;
                if (!data->offload.offload_running) {
#endif
                input_mt_slot(data->input_dev, i);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
                }
#endif
            }
        }
    }
    data->touchs = touchs;

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    if (!data->offload.offload_running) {
#endif
    if (va_reported) {
        /* touchs==0, there's no point but key */
        if (EVENT_NO_DOWN(data) || (!touchs)) {
            if (data->log_level >= 1) {
                FTS_DEBUG("[B]Points All Up!");
            }
            input_report_key(data->input_dev, BTN_TOUCH, 0);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }
    input_set_timestamp(data->input_dev, data->coords_timestamp);
    input_sync(data->input_dev);
    fts_update_motion_filter(data, data->point_num);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    }
#endif
    return 0;
}

#else
static int fts_input_report_a(struct fts_ts_data *data)
{
    int i = 0;
    int touchs = 0;
    bool va_reported = false;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (fts_input_report_key(data, i) == 0) {
            continue;
        }

        va_reported = true;
        if (EVENT_DOWN(events[i].flag)) {
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, events[i].id);
#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x00;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x00;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].major);
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MINOR, events[i].minor);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            input_mt_sync(data->input_dev);

            if ((data->log_level >= 2) ||
                ((1 == data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
                FTS_DEBUG("[A]P%d(%d, %d)[ma:%d,mi:%d,p:%d] DOWN!",
                          events[i].id,
                          events[i].x,
                          events[i].y,
                          events[i].major,
                          events[i].minor,
                          events[i].p);
            }
            touchs++;
        }
    }

    /* last point down, current no point but key */
    if (data->touchs && !touchs) {
        va_reported = true;
    }
    data->touchs = touchs;

    if (va_reported) {
        if (EVENT_NO_DOWN(data)) {
            if (data->log_level >= 1) {
                FTS_DEBUG("[A]Points All Up!");
            }
            input_report_key(data->input_dev, BTN_TOUCH, 0);
            input_mt_sync(data->input_dev);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }
    input_set_timestamp(data->input_dev, data->timestamp);
    input_sync(data->input_dev);
    return 0;
}
#endif

#if FTS_PEN_EN
static int fts_input_pen_report(struct fts_ts_data *data)
{
    struct input_dev *pen_dev = data->pen_dev;
    struct pen_event *pevt = &data->pevent;
    u8 *buf = data->point_buf;


    if (buf[3] & 0x08)
        input_report_key(pen_dev, BTN_STYLUS, 1);
    else
        input_report_key(pen_dev, BTN_STYLUS, 0);

    if (buf[3] & 0x02)
        input_report_key(pen_dev, BTN_STYLUS2, 1);
    else
        input_report_key(pen_dev, BTN_STYLUS2, 0);

    pevt->inrange = (buf[3] & 0x20) ? 1 : 0;
    pevt->tip = (buf[3] & 0x01) ? 1 : 0;
    pevt->x = ((buf[4] & 0x0F) << 8) + buf[5];
    pevt->y = ((buf[6] & 0x0F) << 8) + buf[7];
    pevt->p = ((buf[8] & 0x0F) << 8) + buf[9];
    pevt->id = buf[6] >> 4;
    pevt->flag = buf[4] >> 6;
    pevt->tilt_x = (buf[10] << 8) + buf[11];
    pevt->tilt_y = (buf[12] << 8) + buf[13];
    pevt->tool_type = BTN_TOOL_PEN;

    if (data->log_level >= 2  ||
        ((1 == data->log_level) && (FTS_TOUCH_DOWN == pevt->flag))) {
        FTS_DEBUG("[PEN]x:%d,y:%d,p:%d,inrange:%d,tip:%d,flag:%d DOWN!",
                  pevt->x, pevt->y, pevt->p, pevt->inrange,
                  pevt->tip, pevt->flag);
    }

    if ( (data->log_level >= 1) && (!pevt->inrange)) {
        FTS_DEBUG("[PEN]UP!");
    }

    input_report_abs(pen_dev, ABS_X, pevt->x);
    input_report_abs(pen_dev, ABS_Y, pevt->y);
    input_report_abs(pen_dev, ABS_PRESSURE, pevt->p);

    /* check if the pen support tilt event */
    if ((pevt->tilt_x != 0) || (pevt->tilt_y != 0)) {
        input_report_abs(pen_dev, ABS_TILT_X, pevt->tilt_x);
        input_report_abs(pen_dev, ABS_TILT_Y, pevt->tilt_y);
    }

    input_report_key(pen_dev, BTN_TOUCH, pevt->tip);
    input_report_key(pen_dev, BTN_TOOL_PEN, pevt->inrange);
    input_sync(pen_dev);

    return 0;
}
#endif

static int fts_read_touchdata(struct fts_ts_data *data)
{
    int ret = 0;
    u8 *buf = data->point_buf;
    u8 cmd[1] = { 0 };

#if IS_ENABLED(GOOGLE_REPORT_MODE)
    u8 regB2_data[FTS_CUSTOMER_STATUS_LEN] = { 0 };
    u8 check_regB2_status[2] = { 0 };
    int i;

    if (data->work_mode == FTS_REG_WORKMODE_WORK_VALUE) {
        /* If fw_heatmap_mode is enableed compressed heatmap, to read register
         * 0xB2 before fts_get_heatamp() to get the length of compressed
         * heatmap first.
         */
        if (data->fw_heatmap_mode == FW_HEATMAP_MODE_COMPRESSED) {
            cmd[0] = FTS_REG_CUSTOMER_STATUS;
            fts_read(cmd, 1, regB2_data, FTS_CUSTOMER_STATUS_LEN);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD) || \
    IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
            data->compress_heatmap_wlen = (regB2_data[2] << 8) + regB2_data[3];
#endif
        }
    }
#endif

    cmd[0] = FTS_CMD_READ_TOUCH_DATA;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD) || \
    IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    ret = fts_get_heatmap(data);
    if (ret < 0)
        return ret;
    memcpy(buf + 1, data->heatmap_raw, data->pnt_buf_size - 1);
#else
    ret = fts_read(cmd, 1, buf + 1, data->pnt_buf_size - 1);
    if (ret < 0) {
        FTS_ERROR("touch data(%x) abnormal,ret:%d", buf[1], ret);
        return -EIO;
    }
#endif

    if (data->gesture_mode) {
        ret = fts_gesture_readdata(data, true);
        if (ret == 0) {
            FTS_INFO("succuss to get gesture data in irq handler");
            return 1;
        }
    }

#if IS_ENABLED(GOOGLE_REPORT_MODE)
    if (data->work_mode == FTS_REG_WORKMODE_WORK_VALUE) {
        /* If fw_heatmap_mode is disabled heatmap or enableed uncompressed
         * heatmap, to read register 0xB2 after fts_get_heatamp().
         */
        if (data->fw_heatmap_mode != FW_HEATMAP_MODE_COMPRESSED) {
            cmd[0] = FTS_REG_CUSTOMER_STATUS;
            fts_read(cmd, 1, regB2_data, FTS_CUSTOMER_STATUS_LEN);
        }

        check_regB2_status[0] = regB2_data[0] ^ data->current_host_status[0] ;
        if (check_regB2_status[0]) { // current_status is different with previous_status
            for (i = STATUS_BASELINE_REFRESH_B1; i < STATUS_CNT_END; i++) {
                if ((i == STATUS_BASELINE_REFRESH_B1) && (check_regB2_status[0] & 0x03)) {
                    FTS_INFO("-------%s\n",
                        status_baseline_refresh_str[regB2_data[0] & 0x03]);
                } else {
                    bool status_changed = check_regB2_status[0] & (1 << i);
                    bool new_status = regB2_data[0] & (1 << i);
                    if (status_changed) {
                        FTS_INFO("-------%s %s\n", status_list_str[i],
                            new_status ? "enter" : "exit");
                        if (i == STATUS_RESET && new_status) {
                            /* Write 0x01 to register(0xEC) to clear the reset
                             * flag in bit 7 of register(0xB2).
                             */
                            fts_write_reg(FTS_REG_CLR_RESET, 0x01);
                        }
                    }
                }
            }
            data->current_host_status[0] = regB2_data[0];
        }
        check_regB2_status[1] =
            (regB2_data[1] ^ data->current_host_status[1]) & FTS_CUSTOMER_STATUS1_MASK;
        if (check_regB2_status[1]) {
            bool feature_changed;
            bool feature_enabled;
            FTS_ERROR("FW settings dose not match host side, host: 0x%x, B2[1]:0x%x\n",
                data->current_host_status[1], regB2_data[1]);
            for (i = FW_GLOVE; i < FW_CNT_END; i++) {
                feature_changed = check_regB2_status[1] & (1 << i);
                feature_enabled = regB2_data[1] & (1 << i);
                if (feature_changed) {
                    FTS_INFO("-------%s setting %s\n", feature_list_str[i],
                        feature_enabled ? "enable" : "disable");
                }
            }
            /* The status in data->current_host_status[1] are updated in
             * fts_update_host_feature_setting().
             */

            /* recover touch firmware state. */
            fts_tp_state_recovery(data);
        }
    }
#endif

    if (data->log_level >= 3) {
        fts_show_touch_buffer(buf, data->pnt_buf_size);
    }

    return ret;
}

static int fts_read_parse_touchdata(struct fts_ts_data *data)
{
    int ret = 0;
    int i = 0;
    u8 pointid = 0;
    int base = 0;
    struct ts_event *events = data->events;
    int max_touch_num = data->pdata->max_touch_number;
    u8 *buf = data->point_buf;

    ret = fts_read_touchdata(data);
    if (ret) {
        return ret;
    }

#if FTS_PEN_EN
    if ((buf[2] & 0xF0) == 0xB0) {
        fts_input_pen_report(data);
        return 2;
    }
#endif

    data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
    data->touch_point = 0;

    if (data->ic_info.is_incell) {
        if ((data->point_num == 0x0F) && (buf[2] == 0xFF) && (buf[3] == 0xFF)
            && (buf[4] == 0xFF) && (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
            FTS_DEBUG("touch buff is 0xff, need recovery state");
            fts_release_all_finger();
            fts_tp_state_recovery(data);
            data->point_num = 0;
            return -EIO;
        }
    }

    if (data->point_num > max_touch_num) {
        FTS_DEBUG("invalid point_num(%d)", data->point_num);
        data->point_num = 0;
        return -EIO;
    }

    for (i = 0; i < max_touch_num; i++) {
        base = FTS_ONE_TCH_LEN * i;
        pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
        if (pointid >= FTS_MAX_ID)
            break;
        else if (pointid >= max_touch_num) {
            FTS_ERROR("ID(%d) beyond max_touch_number", pointid);
            return -EINVAL;
        }

        data->touch_point++;
        events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
        events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
        events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
        events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
        events[i].p = (((buf[FTS_TOUCH_AREA_POS + base] << 1) & 0x02) +
                       (buf[FTS_TOUCH_PRE_POS + base] & 0x01)) *
                       FTS_PRESSURE_SCALE;
        events[i].minor =
            ((buf[FTS_TOUCH_PRE_POS + base] >> 1) & 0x7F) * data->pdata->mm2px;
        events[i].major =
            ((buf[FTS_TOUCH_AREA_POS + base] >> 1) & 0x7F) * data->pdata->mm2px;

        if (EVENT_DOWN(events[i].flag) && (data->point_num == 0)) {
            FTS_INFO("abnormal touch data from fw");
            return -EIO;
        }
    }

    if (data->touch_point == 0) {
        FTS_INFO("no touch point information(%02x)", buf[2]);
        return -EIO;
    }

    return 0;
}

static void fts_irq_read_report(void)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr(1);
#endif

#if FTS_POINT_REPORT_CHECK_EN
    fts_prc_queue_work(ts_data);
#endif

    ret = fts_read_parse_touchdata(ts_data);
    if (ret == 0) {
        mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
        fts_input_report_b(ts_data);
#else
        fts_input_report_a(ts_data);
#endif
        mutex_unlock(&ts_data->report_mutex);
    }
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    ret = touch_offload_reserve_frame(&ts_data->offload,
                                      &ts_data->reserved_frame);
    if (ret != 0) {
        PR_LOGD("Could not reserve a frame: error=%d.\n", ret);
        /* Stop offload when there are no buffers available. */
        fts_offload_set_running(ts_data, false);
    } else {
        fts_offload_set_running(ts_data, true);
        PR_LOGD("reserve a frame ok");
        fts_populate_frame(ts_data, 0xFFFFFFFF);

        ret = touch_offload_queue_frame(&ts_data->offload,
                                        ts_data->reserved_frame);
        if (ret != 0) {
            FTS_ERROR("Failed to queue reserved frame: error=%d.\n", ret);
        }
    }
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    heatmap_read(&ts_data->v4l2, ktime_to_ns(ts_data->coords_timestamp));
#endif

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr(0);
#endif
}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
static void fts_ts_aggregate_bus_state(struct fts_ts_data *ts)
{
    /* Complete or cancel any outstanding transitions */
    cancel_work_sync(&ts->suspend_work);
    cancel_work_sync(&ts->resume_work);

    if ((ts->bus_refmask == 0 &&
        ts->power_status == FTS_TS_STATE_SUSPEND) ||
        (ts->bus_refmask != 0 &&
        ts->power_status != FTS_TS_STATE_SUSPEND))
        return;

    if (ts->bus_refmask == 0)
        queue_work(ts->ts_workqueue, &ts->suspend_work);
    else
        queue_work(ts->ts_workqueue, &ts->resume_work);
}

int fts_ts_set_bus_ref(struct fts_ts_data *ts, u16 ref, bool enable)
{
    int result = 0;

    mutex_lock(&ts->bus_mutex);

    if ((enable && (ts->bus_refmask & ref)) ||
        (!enable && !(ts->bus_refmask & ref))) {
        mutex_unlock(&ts->bus_mutex);
        return -EINVAL;
    }

    if (enable) {
        /* IRQs can only keep the bus active. IRQs received while the
         * bus is transferred to AOC should be ignored.
         */
        if (ref == FTS_TS_BUS_REF_IRQ && ts->bus_refmask == 0)
            result = -EAGAIN;
        else
            ts->bus_refmask |= ref;
    } else
        ts->bus_refmask &= ~ref;
    fts_ts_aggregate_bus_state(ts);

    mutex_unlock(&ts->bus_mutex);

    /* When triggering a wake, wait up to one second to resume. SCREEN_ON
     * and IRQ references do not need to wait.
     */
    if (enable &&
        ref != FTS_TS_BUS_REF_SCREEN_ON && ref != FTS_TS_BUS_REF_IRQ) {
        wait_for_completion_timeout(&ts->bus_resumed, HZ);
        if (ts->power_status != FTS_TS_STATE_POWER_ON) {
            FTS_ERROR("Failed to wake the touch bus.\n");
            result = -ETIMEDOUT;
        }
    }

    return result;
}

struct drm_connector *get_bridge_connector(struct drm_bridge *bridge)
{
    struct drm_connector *connector;
    struct drm_connector_list_iter conn_iter;

    drm_connector_list_iter_begin(bridge->dev, &conn_iter);
    drm_for_each_connector_iter(connector, &conn_iter) {
        if (connector->encoder == bridge->encoder)
            break;
    }
    drm_connector_list_iter_end(&conn_iter);
    return connector;
}

static bool bridge_is_lp_mode(struct drm_connector *connector)
{
    if (connector && connector->state) {
        struct exynos_drm_connector_state *s =
            to_exynos_connector_state(connector->state);
        return s->exynos_mode.is_lp_mode;
    }
    return false;
}

static void panel_bridge_enable(struct drm_bridge *bridge)
{
    struct fts_ts_data *ts =
        container_of(bridge, struct fts_ts_data, panel_bridge);

    if (!ts->is_panel_lp_mode)
        fts_ts_set_bus_ref(ts, FTS_TS_BUS_REF_SCREEN_ON, true);
}

static void panel_bridge_disable(struct drm_bridge *bridge)
{
    struct fts_ts_data *ts =
        container_of(bridge, struct fts_ts_data, panel_bridge);

    if (bridge->encoder && bridge->encoder->crtc) {
        const struct drm_crtc_state *crtc_state = bridge->encoder->crtc->state;

        if (drm_atomic_crtc_effectively_active(crtc_state))
            return;
    }

    fts_ts_set_bus_ref(ts, FTS_TS_BUS_REF_SCREEN_ON, false);
}

static void panel_bridge_mode_set(struct drm_bridge *bridge,
    const struct drm_display_mode *mode,
    const struct drm_display_mode *adjusted_mode)
{
    struct fts_ts_data *ts =
        container_of(bridge, struct fts_ts_data, panel_bridge);

    if (!ts->connector || !ts->connector->state)
        ts->connector = get_bridge_connector(bridge);

    ts->is_panel_lp_mode = bridge_is_lp_mode(ts->connector);
    fts_ts_set_bus_ref(ts, FTS_TS_BUS_REF_SCREEN_ON, !ts->is_panel_lp_mode);

    if (adjusted_mode) {
        int vrefresh = drm_mode_vrefresh(adjusted_mode);

        if (ts->display_refresh_rate != vrefresh) {
            FTS_INFO("refresh rate(Hz) changed to %d from %d\n",
                vrefresh, ts->display_refresh_rate);
            ts->display_refresh_rate = vrefresh;
        }
    }
}

static const struct drm_bridge_funcs panel_bridge_funcs = {
    .enable = panel_bridge_enable,
    .disable = panel_bridge_disable,
    .mode_set = panel_bridge_mode_set,
};

static int register_panel_bridge(struct fts_ts_data *ts)
{
    FTS_FUNC_ENTER();
#ifdef CONFIG_OF
    ts->panel_bridge.of_node = ts->spi->dev.of_node;
#endif
    ts->panel_bridge.funcs = &panel_bridge_funcs;
    drm_bridge_add(&ts->panel_bridge);
    FTS_FUNC_EXIT();
    return 0;
}

static void unregister_panel_bridge(struct drm_bridge *bridge)
{
    struct drm_bridge *node;

    FTS_FUNC_ENTER();
    drm_bridge_remove(bridge);

    if (!bridge->dev) /* not attached */
        return;

    drm_modeset_lock(&bridge->dev->mode_config.connection_mutex, NULL);
    list_for_each_entry(node, &bridge->encoder->bridge_chain, chain_node)
        if (node == bridge) {
            if (bridge->funcs->detach)
                bridge->funcs->detach(bridge);
            list_del(&bridge->chain_node);
            break;
        }
    drm_modeset_unlock(&bridge->dev->mode_config.connection_mutex);
    bridge->dev = NULL;
    FTS_FUNC_EXIT();
}
#endif

/* Update a state machine used to toggle control of the touch IC's motion
 * filter.
 */
static void fts_update_motion_filter(struct fts_ts_data *ts, u8 touches)
{
    /* Motion filter timeout, in milliseconds */
    const u32 mf_timeout_ms = 500;
    u8 next_state;

    next_state = ts->mf_state;
    if (ts->mf_mode == MF_OFF) {
        next_state = MF_UNFILTERED;
    } else if (ts->mf_mode == MF_DYNAMIC) {
        /* Determine the next filter state. The motion filter is enabled by
        * default and it is disabled while a single finger is touching the
        * screen. If another finger is touched down or if a timeout expires,
        * the motion filter is reenabled and remains enabled until all fingers
        * are lifted.
        */
        next_state = ts->mf_state;
        switch (ts->mf_state) {
        case MF_FILTERED:
            if (touches == 1) {
                next_state = MF_UNFILTERED;
                ts->mf_downtime = ktime_get();
            }
            break;
        case MF_UNFILTERED:
            if (touches == 0) {
                next_state = MF_FILTERED;
            } else if (touches > 1 || ktime_after(ktime_get(),
                ktime_add_ms(ts->mf_downtime, mf_timeout_ms))) {
                next_state = MF_FILTERED_LOCKED;
            }
            break;
        case MF_FILTERED_LOCKED:
            if (touches == 0)
                next_state = MF_FILTERED;
            break;
        }
    } else if (ts->mf_mode == MF_ON) {
        next_state = MF_FILTERED;
    } else {
        /* Set MF_DYNAMIC as default when an invalid value is found. */
        ts->mf_mode = MF_DYNAMIC;
        return;
    }

    /* Send command to update firmware continuous report */
    if ((next_state == MF_UNFILTERED) !=
        (ts->mf_state == MF_UNFILTERED)) {
        bool en = (next_state == MF_UNFILTERED) ? true : false;
        fts_set_continuous_mode(ts, en);
    }
    ts->mf_state = next_state;
}

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD) || \
    IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
static void fts_show_heatmap_data(struct fts_ts_data *ts_data) {
    int i;
    int idx_buff;
    u8 tx = ts_data->pdata->tx_ch_num;
    u8 rx = ts_data->pdata->rx_ch_num;
    FTS_DEBUG("Show mutual data:\n");

    idx_buff = 0;
    for (i = 0; i < rx; i++) {
        FTS_DEBUG("RX(%d):%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d",
            idx_buff,
            (s16)ts_data->heatmap_buff[idx_buff],
            (s16)ts_data->heatmap_buff[idx_buff + 1],
            (s16)ts_data->heatmap_buff[idx_buff + 2],
            (s16)ts_data->heatmap_buff[idx_buff + 3],
            (s16)ts_data->heatmap_buff[idx_buff + 4],
            (s16)ts_data->heatmap_buff[idx_buff + 5],
            (s16)ts_data->heatmap_buff[idx_buff + 6],
            (s16)ts_data->heatmap_buff[idx_buff + 7],
            (s16)ts_data->heatmap_buff[idx_buff + 8],
            (s16)ts_data->heatmap_buff[idx_buff + 9],
            (s16)ts_data->heatmap_buff[idx_buff + 10],
            (s16)ts_data->heatmap_buff[idx_buff + 11],
            (s16)ts_data->heatmap_buff[idx_buff + 12],
            (s16)ts_data->heatmap_buff[idx_buff + 13],
            (s16)ts_data->heatmap_buff[idx_buff + 14],
            (s16)ts_data->heatmap_buff[idx_buff + 15]);
            idx_buff += tx;
    }

    FTS_DEBUG("Show Tx self data:\n");
    FTS_DEBUG("Tx(idx_buff=%d):%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d",
        idx_buff,
        (s16)ts_data->heatmap_buff[idx_buff],
        (s16)ts_data->heatmap_buff[idx_buff + 1],
        (s16)ts_data->heatmap_buff[idx_buff + 2],
        (s16)ts_data->heatmap_buff[idx_buff + 3],
        (s16)ts_data->heatmap_buff[idx_buff + 4],
        (s16)ts_data->heatmap_buff[idx_buff + 5],
        (s16)ts_data->heatmap_buff[idx_buff + 6],
        (s16)ts_data->heatmap_buff[idx_buff + 7],
        (s16)ts_data->heatmap_buff[idx_buff + 8],
        (s16)ts_data->heatmap_buff[idx_buff + 9],
        (s16)ts_data->heatmap_buff[idx_buff + 10],
        (s16)ts_data->heatmap_buff[idx_buff + 11],
        (s16)ts_data->heatmap_buff[idx_buff + 12],
        (s16)ts_data->heatmap_buff[idx_buff + 13],
        (s16)ts_data->heatmap_buff[idx_buff + 14],
        (s16)ts_data->heatmap_buff[idx_buff + 15]);
    idx_buff += 16;

    FTS_DEBUG("Show Rx self data:\n");
    FTS_DEBUG("Rx(idx_buff=%d)%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d",
        idx_buff,
        (short)ts_data->heatmap_buff[idx_buff],
        (short)ts_data->heatmap_buff[idx_buff + 1],
        (short)ts_data->heatmap_buff[idx_buff + 2],
        (short)ts_data->heatmap_buff[idx_buff + 3],
        (short)ts_data->heatmap_buff[idx_buff + 4],
        (short)ts_data->heatmap_buff[idx_buff + 5],
        (short)ts_data->heatmap_buff[idx_buff + 6],
        (short)ts_data->heatmap_buff[idx_buff + 7],
        (short)ts_data->heatmap_buff[idx_buff + 8],
        (short)ts_data->heatmap_buff[idx_buff + 9],
        (short)ts_data->heatmap_buff[idx_buff + 10],
        (short)ts_data->heatmap_buff[idx_buff + 11],
        (short)ts_data->heatmap_buff[idx_buff + 12],
        (short)ts_data->heatmap_buff[idx_buff + 13],
        (short)ts_data->heatmap_buff[idx_buff + 14],
        (short)ts_data->heatmap_buff[idx_buff + 15]);
    idx_buff += 16;

    FTS_DEBUG("Rx(idx_buff=%d)%5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d",
        idx_buff,
        (short)ts_data->heatmap_buff[idx_buff],
        (short)ts_data->heatmap_buff[idx_buff + 1],
        (short)ts_data->heatmap_buff[idx_buff + 2],
        (short)ts_data->heatmap_buff[idx_buff + 3],
        (short)ts_data->heatmap_buff[idx_buff + 4],
        (short)ts_data->heatmap_buff[idx_buff + 5],
        (short)ts_data->heatmap_buff[idx_buff + 6],
        (short)ts_data->heatmap_buff[idx_buff + 7],
        (short)ts_data->heatmap_buff[idx_buff + 8],
        (short)ts_data->heatmap_buff[idx_buff + 9],
        (short)ts_data->heatmap_buff[idx_buff + 10],
        (short)ts_data->heatmap_buff[idx_buff + 11],
        (short)ts_data->heatmap_buff[idx_buff + 12],
        (short)ts_data->heatmap_buff[idx_buff + 13],
        (short)ts_data->heatmap_buff[idx_buff + 14],
        (short)ts_data->heatmap_buff[idx_buff + 15]);
    idx_buff += 16;

    FTS_DEBUG("Rx(idx_buff=%d)%5d %5d", idx_buff,
        (short)ts_data->heatmap_buff[idx_buff],
        (short)ts_data->heatmap_buff[idx_buff + 1]);
    idx_buff += 2;
    FTS_DEBUG("Print done, idx_buff=%d", idx_buff);
}
#endif /* GOOGLE_HEATMAP_DEBUG */

static int fts_ptflib_decoder(struct fts_ts_data *ts_data, const u16 *in_array,
    const int in_array_size, u16 *out_array, const int out_array_max_size)
{
    const u16 ESCAPE_MASK = 0xF000;
    const u16 ESCAPE_BIT = 0x8000;

    int i;
    int j;
    int out_array_size = 0;
    u16 prev_word = 0;
    u16 repetition = 0;
    u16 *temp_out_array = out_array;

    for (i = 0; i < in_array_size; i++) {
        /* The data form firmware is big-endian, and needs to transfer it to
         * little-endian.
         */
        u16 curr_word = (u16)(*((u8*)&in_array[i]) << 8) +
                        *((u8*)&in_array[i] + 1);
        if ((curr_word & ESCAPE_MASK) == ESCAPE_BIT) {
            repetition = (curr_word & ~ESCAPE_MASK);
            if (out_array_size + repetition > out_array_max_size)
                break;
            for (j = 0; j < repetition; j++) {
                *temp_out_array++ = prev_word;
                out_array_size++;
            }
        } else {
            if (out_array_size >= out_array_max_size)
                break;
            *temp_out_array++ = curr_word;
            out_array_size++;
            prev_word = curr_word;
        }
    }

    if (i != in_array_size || out_array_size != out_array_max_size) {
        FTS_ERROR("%d (in=%d, out=%d, rep=%d, out_max=%d).\n",
            i, in_array_size, out_array_size,
            repetition, out_array_max_size);
        memset(out_array, 0, out_array_max_size * sizeof(u16));
        return -1;
    }

    return out_array_size;
}

extern void transpose_raw(u8 *src, u8 *dist, int tx, int rx, bool big_endian);
static int fts_get_heatmap(struct fts_ts_data *ts_data) {
    int ret = 0;
    int i;
    int idx_buff = 0;
    int node_num = 0;
    int self_node = 0;
    int mutual_data_size = 0;
    int self_data_size = 0;
    int total_data_size = 0;
    u8 cmd[1] = {0};
    u8 tx = ts_data->pdata->tx_ch_num;
    u8 rx = ts_data->pdata->rx_ch_num;
    int idx_ms_raw = 0;
    int idx_ss_tx_raw = 0;
    int idx_ss_rx_raw = 0;
    int idx_water_ss_tx_raw = 0;
    int idx_water_ss_rx_raw = 0;

#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
    FTS_FUNC_ENTER();
#endif

    node_num = tx * rx;
    self_node = tx + rx;
    /* The mutual sensing raw data size : 16*34*2=1088 */
    mutual_data_size = node_num * sizeof(u16);
    /* The self sensing raw data size : 68*2=136 */
    self_data_size = FTS_SELF_DATA_LEN * sizeof(u16);
    /* The index of mutual sensing data : 91+(68*2)*2=363 */
    idx_ms_raw = FTS_CAP_DATA_LEN + self_data_size * 2;
    /* The tx index of water self sensing data : 91+34*2=159 */
    idx_water_ss_tx_raw = FTS_CAP_DATA_LEN + rx * sizeof(u16);
    /* The rx index of water self sensing data : 91 */
    idx_water_ss_rx_raw = FTS_CAP_DATA_LEN;
    /* The tx index of normal self sensing data : 91+68*2+34*2=295 */
    idx_ss_tx_raw = FTS_CAP_DATA_LEN + self_data_size + rx * sizeof(u16);
    /* The rx index of normal self sensing data : 91+68*2=227 */
    idx_ss_rx_raw = FTS_CAP_DATA_LEN + self_data_size;

    if (!ts_data->heatmap_buff) {
        FTS_ERROR("The heatmap_buff is not allocated!!");
        ret = -ENOMEM;
        goto exit;
    }

    if (!ts_data->fw_heatmap_mode) {
        FTS_ERROR("The firmware heatmap is not enabled!!");
        ret = -EINVAL;
        goto exit;
    }

    cmd[0] = FTS_CMD_READ_TOUCH_DATA;
    if (ts_data->fw_heatmap_mode == FW_HEATMAP_MODE_UNCOMPRESSED) {
        /* The format of uncompressed heatmap from touch chip.
         *
         * |- cap header (91) -|- Water-SS -|- Normal-SS -|- Normal-MS -|
         * |-        91       -|-   68*2   -|-   68*2    -|-  16*34*2  -|
         */

        /* Total touch data: (cap header(91) + heatmap(N-MS + W-SS + N-SS)). */
        total_data_size = FTS_CAP_DATA_LEN + self_data_size * 2 +
                          mutual_data_size;

        if (total_data_size > ts_data->heatmap_raw_size) {
            FTS_DEBUG("Warning : The total touch data size is %d!!",
                total_data_size);
            total_data_size = ts_data->heatmap_raw_size;
        }

        ret = fts_read(cmd, 1, ts_data->heatmap_raw, total_data_size);
        if (ret < 0) {
            FTS_ERROR("Failed to get heatmap raw data, ret=%d.", ret);
            ret = -EIO;
            goto exit;
        }

        /* Get the self-sensing type. */
        ts_data->self_sensing_type =
            ts_data->heatmap_raw[FTS_CAP_DATA_LEN - 1] & 0x80;

        /*
         * transform the order of MS from RX->TX, the output data is keep
         * big-endian.
         */
        transpose_raw(ts_data->heatmap_raw + idx_ms_raw, ts_data->trans_raw,
            tx, rx, true);
    } else {
        /* The format of compressed heatmap from touch chip.
         *
         * |- cap header -|- Water-SS -|- Normal-SS -|- compressed heatmap(MS)-|
         * |-     91     -|-   68*2   -|-   68*2    -|- (B2[1]<<8+B2[2])*2    -|
         */

        if (ts_data->compress_heatmap_wlen < 0 ||
            (ts_data->compress_heatmap_wlen * sizeof(u16)) > mutual_data_size) {
            FTS_DEBUG("Warning : The compressed heatmap size is %d!!",
                ts_data->compress_heatmap_wlen);
            ts_data->compress_heatmap_wlen = 0;
            memset(ts_data->trans_raw, 0, ts_data->trans_raw_size);
        }

        /* Total touch data:(cap header + W-SS + N-SS + compressed heatmap(N-MS)
         */
        total_data_size = FTS_CAP_DATA_LEN +
                          self_data_size * 2 +
                          ts_data->compress_heatmap_wlen * sizeof(u16);

        if (total_data_size > ts_data->heatmap_raw_size) {
            FTS_DEBUG("Warning : The total touch data size is %d!!",
                total_data_size);
            total_data_size = ts_data->heatmap_raw_size;
        }

        ret = fts_read(cmd, 1, ts_data->heatmap_raw, total_data_size);
        if (ret < 0) {
            FTS_ERROR("Failed to get compressed heatmap raw data,ret=%d.", ret);
            ret = -EIO;
            goto exit;
        }

        /* Get the self-sensing type. */
        ts_data->self_sensing_type =
            ts_data->heatmap_raw[FTS_CAP_DATA_LEN - 1] & 0x80;

        if (ts_data->compress_heatmap_wlen > 0) {
            /* decode the compressed data from heatmap_raw to heatmap_buff. */
            fts_ptflib_decoder(ts_data,
                (u16*)(&ts_data->heatmap_raw[idx_ms_raw]),
                ts_data->compress_heatmap_wlen,
                ts_data->heatmap_buff,
                mutual_data_size / sizeof(u16));

            /* MS: Transform the order from RX->TX. */
            /* After decoding, the data become to little-endian, but the output of
             * transpose_raw is big-endian.
             */
            transpose_raw(&((u8*)ts_data->heatmap_buff)[0], ts_data->trans_raw,
                tx, rx, false);
        }
    }
#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
    FTS_DEBUG("Copy matual data,idx_buff=%d,idx_ms_raw=%d.",
        idx_buff, idx_ms_raw);
#endif

    /* copy mutual sensing data. */
    for (i = 0; i < node_num; i++) {
        ((u16*)ts_data->heatmap_buff)[idx_buff++] =
            (u16)(ts_data->trans_raw[(i * 2)] << 8) +
            ts_data->trans_raw[(i * 2) + 1];
    }

    /* copy tx of Normal-SS. */
#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
    FTS_DEBUG("Copy the tx self data,idx_buff=%d,idx_ss_tx_raw=%d.",
        idx_buff, idx_ss_tx_raw);
#endif
    for (i = 0 ; i < tx; i++) {
        ((u16*)ts_data->heatmap_buff)[idx_buff++] =
            (u16)(ts_data->heatmap_raw[idx_ss_tx_raw + (i * 2)] << 8) +
            ts_data->heatmap_raw[idx_ss_tx_raw +(i * 2) + 1];
    }

    /* copy rx of Normal-SS. */
#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
    FTS_DEBUG("Copy the rx self data,idx_buff=%d,idx_ss_rx_raw=%d.",
        idx_buff, idx_ss_rx_raw);
#endif
    for (i = 0 ; i < rx; i++) {
        ((u16*)ts_data->heatmap_buff)[idx_buff++] =
            (u16)(ts_data->heatmap_raw[idx_ss_rx_raw + (i * 2)] << 8) +
            ts_data->heatmap_raw[idx_ss_rx_raw + (i * 2) + 1];
    }

    /* copy tx of Water-SS. */
#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
    FTS_DEBUG("Copy the tx of Water-SS,idx_buff=%d,idx_water_ss_tx_raw=%d.",
        idx_buff, idx_water_ss_tx_raw);
#endif
    for (i = 0 ; i < tx; i++) {
        ((u16*)ts_data->heatmap_buff)[idx_buff++] =
            (u16)(ts_data->heatmap_raw[idx_water_ss_tx_raw + (i * 2)] << 8) +
            ts_data->heatmap_raw[idx_water_ss_tx_raw +(i * 2) + 1];
    }

    /* copy rx of Water-SS. */
#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
    FTS_DEBUG("Copy the rx of Water-SS,idx_buff=%d,idx_water_ss_rx_raw=%d.",
        idx_buff, idx_water_ss_rx_raw);
#endif
    for (i = 0 ; i < rx; i++) {
        ((u16*)ts_data->heatmap_buff)[idx_buff++] =
            (u16)(ts_data->heatmap_raw[idx_water_ss_rx_raw + (i * 2)] << 8) +
            ts_data->heatmap_raw[idx_water_ss_rx_raw + (i * 2) + 1];
    }
    /* The format of heatmap data (U16) of heatmap_buff is:
     *
     * |-    MS   -|- Normal-SS -|- Water-SS -|
     * |-  16*34  -|-   16+34   -|-  16+34   -|
     */
#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
    FTS_FUNC_EXIT();
#endif
exit:
    return ret;
}
#endif /* CONFIG_TOUCHSCREEN_OFFLOAD || CONFIG_TOUCHSCREEN_HEATMAP */

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
static void fts_offload_set_running(struct fts_ts_data *ts_data, bool running)
{
    ts_data->offload.offload_running = running;
    /*
     * Disable firmware grip_suppression/palm_rejection when offload is running
     * and upper layer grip_suppression/palm_rejection is enabled.
     */
    if (running) {
        if (ts_data->enable_fw_grip < FW_GRIP_FORCE_DISABLE) {
            int new_fw_grip = ts_data->offload.config.filter_grip ?
                              FW_GRIP_DISABLE : FW_GRIP_ENABLE;
            if (ts_data->enable_fw_grip != new_fw_grip) {
                ts_data->enable_fw_grip = new_fw_grip;
                fts_set_grip_mode(ts_data, ts_data->enable_fw_grip);
            }
        }

        if (ts_data->enable_fw_palm < FW_PALM_FORCE_DISABLE) {
            int new_fw_palm = ts_data->offload.config.filter_palm ?
                              FW_PALM_DISABLE : FW_PALM_ENABLE;
            if (ts_data->enable_fw_palm != new_fw_palm) {
                ts_data->enable_fw_palm = new_fw_palm;
                fts_set_palm_mode(ts_data, ts_data->enable_fw_palm);
            }
        }
    } else {
        if (ts_data->enable_fw_grip < FW_GRIP_FORCE_DISABLE &&
            ts_data->enable_fw_grip != FW_GRIP_ENABLE) {
            ts_data->enable_fw_grip = FW_GRIP_ENABLE;
            fts_set_grip_mode(ts_data, ts_data->enable_fw_grip);
        }

        if (ts_data->enable_fw_palm < FW_PALM_FORCE_DISABLE &&
            ts_data->enable_fw_palm != FW_PALM_ENABLE) {
            ts_data->enable_fw_palm = FW_PALM_ENABLE;
            fts_set_palm_mode(ts_data, ts_data->enable_fw_palm);
        }
    }
}

static void fts_offload_report(void *handle,
    struct TouchOffloadIocReport *report)
{
    struct fts_ts_data *ts_data = (struct fts_ts_data *)handle;
    bool touch_down = 0;
    int i;
    int touch_count = 0;
    int tool_type;

    mutex_lock(&ts_data->report_mutex);

    input_set_timestamp(ts_data->input_dev, report->timestamp);

    for (i = 0; i < MAX_COORDS; i++) {
        if (report->coords[i].status != COORD_STATUS_INACTIVE) {
            input_mt_slot(ts_data->input_dev, i);
            touch_count++;
            touch_down = 1;
            input_report_key(ts_data->input_dev, BTN_TOUCH, touch_down);
            input_report_key(ts_data->input_dev, BTN_TOOL_FINGER, touch_down);
            switch (report->coords[i].status) {
            case COORD_STATUS_EDGE:
            case COORD_STATUS_PALM:
            case COORD_STATUS_CANCEL:
                tool_type = MT_TOOL_PALM;
                break;
            case COORD_STATUS_FINGER:
            default:
                tool_type = MT_TOOL_FINGER;
                break;
            }
            input_mt_report_slot_state(ts_data->input_dev, tool_type, 1);
            input_report_abs(ts_data->input_dev, ABS_MT_POSITION_X,
                report->coords[i].x);
            input_report_abs(ts_data->input_dev, ABS_MT_POSITION_Y,
                report->coords[i].y);
            input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MAJOR,
                report->coords[i].major);
            input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MINOR,
                report->coords[i].minor);
            input_report_abs(ts_data->input_dev, ABS_MT_PRESSURE,
                report->coords[i].pressure);
            input_report_abs(ts_data->input_dev, ABS_MT_ORIENTATION,
                report->coords[i].rotation);
        } else {
            input_mt_slot(ts_data->input_dev, i);
            input_report_abs(ts_data->input_dev, ABS_MT_PRESSURE, 0);
            input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, 0);
            input_report_abs(ts_data->input_dev, ABS_MT_TRACKING_ID, -1);
            input_report_abs(ts_data->input_dev, ABS_MT_ORIENTATION, 0);
        }
    }
    input_report_key(ts_data->input_dev, BTN_TOUCH, touch_down);
    input_report_key(ts_data->input_dev, BTN_TOOL_FINGER, touch_down);

    input_sync(ts_data->input_dev);
    fts_update_motion_filter(ts_data, touch_count);

    mutex_unlock(&ts_data->report_mutex);
}

static void fts_populate_coordinate_channel(struct fts_ts_data *ts_data,
    struct touch_offload_frame *frame,
    int channel)
{
    int i;
    u8 active_coords = 0;

    struct TouchOffloadDataCoord *dc =
        (struct TouchOffloadDataCoord *)frame->channel_data[channel];
    memset(dc, 0, frame->channel_data_size[channel]);
    dc->header.channel_type = TOUCH_DATA_TYPE_COORD;
    dc->header.channel_size = TOUCH_OFFLOAD_FRAME_SIZE_COORD;

    for (i = 0; i < MAX_COORDS; i++) {
        dc->coords[i].x = ts_data->offload.coords[i].x;
        dc->coords[i].y = ts_data->offload.coords[i].y;
        dc->coords[i].major = ts_data->offload.coords[i].major;
        dc->coords[i].minor = ts_data->offload.coords[i].minor;
        dc->coords[i].pressure = ts_data->offload.coords[i].pressure;
        dc->coords[i].rotation = ts_data->offload.coords[i].rotation;
        dc->coords[i].status = ts_data->offload.coords[i].status;
        if (dc->coords[i].status != COORD_STATUS_INACTIVE)
            active_coords += 1;
    }
    ts_data->touch_offload_active_coords = active_coords;
}

static void fts_populate_mutual_channel(struct fts_ts_data *ts_data,
    struct touch_offload_frame *frame, int channel)
{
    struct TouchOffloadData2d *mutual_strength =
        (struct TouchOffloadData2d *)frame->channel_data[channel];

    mutual_strength->tx_size = ts_data->pdata->tx_ch_num;
    mutual_strength->rx_size = ts_data->pdata->rx_ch_num;
    mutual_strength->header.channel_type = frame->channel_type[channel];
    mutual_strength->header.channel_size =
        TOUCH_OFFLOAD_FRAME_SIZE_2D(mutual_strength->rx_size,
            mutual_strength->tx_size);

    memcpy(mutual_strength->data, ts_data->heatmap_buff,
        mutual_strength->tx_size * mutual_strength->rx_size * sizeof(u16));
}

static void fts_populate_self_channel(struct fts_ts_data *ts_data,
    struct touch_offload_frame *frame, int channel)
{
    u8 ss_type = 0;
    int idx_ss_normal = ts_data->pdata->tx_ch_num * ts_data->pdata->rx_ch_num;
    int idx_ss_water = ts_data->pdata->tx_ch_num * ts_data->pdata->rx_ch_num +
        ts_data->pdata->tx_ch_num + ts_data->pdata->rx_ch_num;
    int ss_size =
        (ts_data->pdata->tx_ch_num + ts_data->pdata->rx_ch_num) * sizeof(u16);
    struct TouchOffloadData1d *self_strength =
        (struct TouchOffloadData1d *)frame->channel_data[channel];

    self_strength->tx_size = ts_data->pdata->tx_ch_num;
    self_strength->rx_size = ts_data->pdata->rx_ch_num;
    self_strength->header.channel_type = frame->channel_type[channel];
    self_strength->header.channel_size =
        TOUCH_OFFLOAD_FRAME_SIZE_1D(self_strength->rx_size,
            self_strength->tx_size);

    switch (frame->channel_type[channel] & ~TOUCH_SCAN_TYPE_SELF) {
    case TOUCH_DATA_TYPE_FILTERED:
        ss_type = SS_WATER;
        break;
    case TOUCH_DATA_TYPE_STRENGTH:
    default:
        ss_type = SS_NORMAL;
        break;
    }

    if (ss_type == SS_WATER) {
        /* Copy Water-SS. */
        memcpy(self_strength->data, ts_data->heatmap_buff + idx_ss_water,
            ss_size);
    } else {
        /* Copy Normal-SS. */
        memcpy(self_strength->data, ts_data->heatmap_buff + idx_ss_normal,
            ss_size);
    }
}

static void fts_populate_frame(struct fts_ts_data *ts_data, int populate_channel_types)
{
    static u64 index;
    int i;
    struct touch_offload_frame *frame = ts_data->reserved_frame;

    frame->header.index = index++;
    frame->header.timestamp = ts_data->coords_timestamp;

    /* Populate all channels */
    for (i = 0; i < frame->num_channels; i++) {
        if ((frame->channel_type[i] & populate_channel_types) == TOUCH_DATA_TYPE_COORD) {
            fts_populate_coordinate_channel(ts_data, frame, i);
        } else if ((frame->channel_type[i] & TOUCH_SCAN_TYPE_MUTUAL &
                    populate_channel_types) != 0) {
            fts_populate_mutual_channel(ts_data, frame, i);
        } else if ((frame->channel_type[i] & TOUCH_SCAN_TYPE_SELF &
                    populate_channel_types) != 0) {
            fts_populate_self_channel(ts_data, frame, i);
        }
    }
}

static void fts_offload_push_coord_frame(struct fts_ts_data *ts)
{
    int error;

    FTS_INFO("active coords %u.", ts->touch_offload_active_coords);

    error = touch_offload_reserve_frame(&ts->offload, &ts->reserved_frame);
    if (error != 0) {
        FTS_DEBUG("Could not reserve a frame: error=%d.\n", error);
    } else {
        fts_populate_frame(ts, TOUCH_DATA_TYPE_COORD);

        error = touch_offload_queue_frame(&ts->offload, ts->reserved_frame);
        if (error != 0) {
            FTS_ERROR("Failed to queue reserved frame: error=%d.\n", error);
        }
    }
}
#endif /* CONFIG_TOUCHSCREEN_OFFLOAD */

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
static bool v4l2_read_frame(struct v4l2_heatmap *v4l2)
{
    bool ret = true;

    struct fts_ts_data *ts_data =
        container_of(v4l2, struct fts_ts_data, v4l2);
#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
    FTS_FUNC_ENTER();
#endif
    if (ts_data->v4l2.width == ts_data->pdata->tx_ch_num &&
        ts_data->v4l2.height == ts_data->pdata->rx_ch_num) {
#if IS_ENABLED(GOOGLE_HEATMAP_DEBUG)
        FTS_DEBUG("v4l2 mutual strength data is ready.");
#endif
        memcpy(v4l2->frame, ts_data->heatmap_buff,
            ts_data->v4l2.width * ts_data->v4l2.height * sizeof(u16));
    } else {
        FTS_ERROR("size mismatched, (%lu, %lu) vs (%u, %u)!\n",
        ts_data->v4l2.width, ts_data->v4l2.height,
        ts_data->pdata->tx_ch_num, ts_data->pdata->rx_ch_num);
        ret = false;
    }

    return ret;
}
#endif /* CONFIG_TOUCHSCREEN_HEATMAP */

static irqreturn_t fts_irq_ts(int irq, void *data)
{
    struct fts_ts_data *ts_data = data;

    ts_data->isr_timestamp = ktime_get();
    return IRQ_WAKE_THREAD;
}

extern int int_test_has_interrupt;
static irqreturn_t fts_irq_handler(int irq, void *data)
{
#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    struct fts_ts_data *ts_data = fts_data;
    if (fts_ts_set_bus_ref(ts_data, FTS_TS_BUS_REF_IRQ, true) < 0) {
        if (!ts_data->gesture_mode) {
            /* Interrupt during bus suspend */
            FTS_INFO("Skipping stray interrupt since bus is suspended(power_status: %d)\n",
                ts_data->power_status);
            return IRQ_HANDLED;
        }
    }
#endif
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    if ((ts_data->suspended) && (ts_data->pm_suspend)) {
        ret = wait_for_completion_timeout(
                  &ts_data->pm_completion,
                  msecs_to_jiffies(FTS_TIMEOUT_COMERR_PM));
        if (!ret) {
            FTS_ERROR("Bus don't resume from pm(deep),timeout,skip irq");
            return IRQ_HANDLED;
        }
    }
#endif
    int_test_has_interrupt++;
    fts_data->coords_timestamp = fts_data->isr_timestamp;
    cpu_latency_qos_update_request(&ts_data->pm_qos_req, 100 /* usec */);
    fts_irq_read_report();
    cpu_latency_qos_update_request(&ts_data->pm_qos_req, PM_QOS_DEFAULT_VALUE);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    fts_ts_set_bus_ref(ts_data, FTS_TS_BUS_REF_IRQ, false);
#endif
    return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;

    ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
    FTS_INFO("irq:%d, flag:%x", ts_data->irq, pdata->irq_gpio_flags);
    ret = request_threaded_irq(ts_data->irq, fts_irq_ts, fts_irq_handler,
                               pdata->irq_gpio_flags,
                               FTS_DRIVER_NAME, ts_data);

    return ret;
}

#if FTS_PEN_EN
static int fts_input_pen_init(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct input_dev *pen_dev;
    struct fts_ts_platform_data *pdata = ts_data->pdata;

    FTS_FUNC_ENTER();
    pen_dev = input_allocate_device();
    if (!pen_dev) {
        FTS_ERROR("Failed to allocate memory for input_pen device");
        return -ENOMEM;
    }

    pen_dev->dev.parent = ts_data->dev;
    pen_dev->name = FTS_DRIVER_PEN_NAME;
    pen_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    __set_bit(ABS_X, pen_dev->absbit);
    __set_bit(ABS_Y, pen_dev->absbit);
    __set_bit(BTN_STYLUS, pen_dev->keybit);
    __set_bit(BTN_STYLUS2, pen_dev->keybit);
    __set_bit(BTN_TOUCH, pen_dev->keybit);
    __set_bit(BTN_TOOL_PEN, pen_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
    input_set_abs_params(pen_dev, ABS_X, pdata->x_min, pdata->x_max, 0, 0);
    input_set_abs_params(pen_dev, ABS_Y, pdata->y_min, pdata->y_max, 0, 0);
    input_set_abs_params(pen_dev, ABS_PRESSURE, 0, 4096, 0, 0);

    ret = input_register_device(pen_dev);
    if (ret) {
        FTS_ERROR("Input device registration failed");
        input_free_device(pen_dev);
        pen_dev = NULL;
        return ret;
    }

    ts_data->pen_dev = pen_dev;
    FTS_FUNC_EXIT();
    return 0;
}
#endif

static int fts_input_init(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int key_num = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;
    struct input_dev *input_dev;

    FTS_FUNC_ENTER();
    input_dev = input_allocate_device();
    if (!input_dev) {
        FTS_ERROR("Failed to allocate memory for input device");
        return -ENOMEM;
    }

    /* Init and register Input device */
    input_dev->name = FTS_DRIVER_NAME;
    if (ts_data->bus_type == FTS_BUS_TYPE_I2C)
        input_dev->id.bustype = BUS_I2C;
    else
        input_dev->id.bustype = BUS_SPI;
    input_dev->dev.parent = ts_data->dev;

    input_dev->uniq = "google_touchscreen";

    input_set_drvdata(input_dev, ts_data);

    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

    if (pdata->have_key) {
        FTS_INFO("set key capabilities");
        for (key_num = 0; key_num < pdata->key_number; key_num++)
            input_set_capability(input_dev, EV_KEY, pdata->keys[key_num]);
    }

#if FTS_MT_PROTOCOL_B_EN
    input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
#endif
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0x3F, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, 0x3F, 0, 0);
#if FTS_REPORT_PRESSURE_EN
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif
    /* Units are (-4096, 4096), representing the range between rotation
     * 90 degrees to left and 90 degrees to the right.
     */
    input_set_abs_params(input_dev, ABS_MT_ORIENTATION, -4096, 4096, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, MT_TOOL_FINGER, MT_TOOL_PALM, 0, 0);
    ret = input_register_device(input_dev);
    if (ret) {
        FTS_ERROR("Input device registration failed");
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }

#if FTS_PEN_EN
    ret = fts_input_pen_init(ts_data);
    if (ret) {
        FTS_ERROR("Input-pen device registration failed");
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }
#endif

    ts_data->input_dev = input_dev;
    FTS_FUNC_EXIT();
    return 0;
}

static int fts_report_buffer_init(struct fts_ts_data *ts_data)
{
    int point_num = 0;
    int events_num = 0;

    point_num = FTS_MAX_POINTS_SUPPORT;
    ts_data->pnt_buf_size = FTS_CAP_DATA_LEN;
    ts_data->point_buf = (u8 *)kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
    if (!ts_data->point_buf) {
        FTS_ERROR("failed to alloc memory for point buf");
        return -ENOMEM;
    }

    events_num = point_num * sizeof(struct ts_event);
    ts_data->events = (struct ts_event *)kzalloc(events_num, GFP_KERNEL);
    if (!ts_data->events) {
        FTS_ERROR("failed to alloc memory for point events");
        kfree_safe(ts_data->point_buf);
        return -ENOMEM;
    }

    return 0;
}

#if FTS_POWER_SOURCE_CUST_EN
/*****************************************************************************
* Power Control
*****************************************************************************/
#if FTS_PINCTRL_EN
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
    int ret = 0;

    ts->pinctrl = devm_pinctrl_get(ts->dev);
    if (IS_ERR_OR_NULL(ts->pinctrl)) {
        FTS_ERROR("Failed to get pinctrl, please check dts");
        ret = PTR_ERR(ts->pinctrl);
        goto err_pinctrl_get;
    }

    ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "ts_active");
    if (IS_ERR_OR_NULL(ts->pins_active)) {
        FTS_ERROR("Pin state[active] not found");
        ret = PTR_ERR(ts->pins_active);
        goto err_pinctrl_lookup;
    }

    ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "ts_suspend");
    if (IS_ERR_OR_NULL(ts->pins_suspend)) {
        FTS_ERROR("Pin state[suspend] not found");
        ret = PTR_ERR(ts->pins_suspend);
        goto err_pinctrl_lookup;
    }

    return 0;
err_pinctrl_lookup:
    if (ts->pinctrl) {
        devm_pinctrl_put(ts->pinctrl);
    }
err_pinctrl_get:
    ts->pinctrl = NULL;
    ts->pins_suspend = NULL;
    ts->pins_active = NULL;
    return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
    int ret = 0;
    FTS_DEBUG("Pins control select normal");
    if (ts->pinctrl && ts->pins_active) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
        if (ret < 0) {
            FTS_ERROR("Set normal pin state error:%d", ret);
        }
    }

    return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
    int ret = 0;
    FTS_DEBUG("Pins control select suspend");
    if (ts->pinctrl && ts->pins_suspend) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
        if (ret < 0) {
            FTS_ERROR("Set suspend pin state error:%d", ret);
        }
    }

    return ret;
}
#endif /* FTS_PINCTRL_EN */

static int fts_power_source_ctrl(struct fts_ts_data *ts_data, int enable)
{
    int ret = 0;

    if (IS_ERR_OR_NULL(ts_data->avdd)) {
        FTS_ERROR("avdd is invalid");
        return -EINVAL;
    }

    FTS_FUNC_ENTER();
    if (enable) {
        if (ts_data->power_disabled) {
            FTS_DEBUG("regulator enable !");
            ret = regulator_enable(ts_data->avdd);
            if (ret) {
                FTS_ERROR("enable avdd regulator failed,ret=%d", ret);
            }

            if (!IS_ERR_OR_NULL(ts_data->dvdd)) {
                ret = regulator_enable(ts_data->dvdd);
                if (ret) {
                    FTS_ERROR("enable dvdd regulator failed,ret=%d", ret);
                }
            }
            /* sleep 1 ms to power on avdd/dvdd to match spec. */
            msleep(1);
            gpio_direction_output(ts_data->pdata->reset_gpio, 1);
            ts_data->power_disabled = false;
        }
    } else {
        if (!ts_data->power_disabled) {
            FTS_DEBUG("regulator disable !");
            gpio_direction_output(ts_data->pdata->reset_gpio, 0);
            /* sleep 1 ms to power off avdd/dvdd to match spec. */
            msleep(1);
            ret = regulator_disable(ts_data->avdd);
            if (ret) {
                FTS_ERROR("disable avdd regulator failed,ret=%d", ret);
            }
            if (!IS_ERR_OR_NULL(ts_data->dvdd)) {
                ret = regulator_disable(ts_data->dvdd);
                if (ret) {
                    FTS_ERROR("disable dvdd regulator failed,ret=%d", ret);
                }
            }
            ts_data->power_disabled = true;
        }
    }

    FTS_FUNC_EXIT();
    return ret;
}

/*****************************************************************************
* Name: fts_power_source_init
* Brief: Init regulator power:avdd/dvdd(if have), generally, no dvdd
*        avdd---->avdd-supply in dts, kernel will auto add "-supply" to parse
*        Must be call after fts_gpio_configure() execute,because this function
*        will operate reset-gpio which request gpio in fts_gpio_configure()
* Input:
* Output:
* Return: return 0 if init power successfully, otherwise return error code
*****************************************************************************/
static int fts_power_source_init(struct fts_ts_data *ts_data)
{
    int ret = 0;

    FTS_FUNC_ENTER();

#if FTS_PINCTRL_EN
    fts_pinctrl_init(ts_data);
    fts_pinctrl_select_normal(ts_data);
#endif

    if (of_property_read_bool(ts_data->dev->of_node, "avdd-supply")) {
        ts_data->avdd = regulator_get(ts_data->dev, "avdd");
        if (IS_ERR_OR_NULL(ts_data->avdd)) {
            ret = PTR_ERR(ts_data->avdd);
            ts_data->avdd = NULL;
            FTS_ERROR("get avdd regulator failed,ret=%d", ret);
            return ret;
        }
    } else {
        FTS_ERROR("avdd-supply not found!");
    }

    if (of_property_read_bool(ts_data->dev->of_node, "vdd-supply")) {
        ts_data->dvdd = regulator_get(ts_data->dev, "vdd");

        if (IS_ERR_OR_NULL(ts_data->dvdd)) {
            ret = PTR_ERR(ts_data->dvdd);
            ts_data->dvdd = NULL;
            FTS_ERROR("get dvdd regulator failed,ret=%d", ret);
        }
    } else {
        FTS_ERROR("vdd-supply not found!");
    }

    ts_data->power_disabled = true;
    ret = fts_power_source_ctrl(ts_data, ENABLE);
    if (ret) {
        FTS_ERROR("fail to enable power(regulator)");
    }

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_power_source_exit(struct fts_ts_data *ts_data)
{
    fts_power_source_ctrl(ts_data, DISABLE);
#if FTS_PINCTRL_EN
    fts_pinctrl_select_suspend(ts_data);
#endif
    if (!IS_ERR_OR_NULL(ts_data->avdd)) {
        regulator_put(ts_data->avdd);
        ts_data->avdd = NULL;
    }

    if (!IS_ERR_OR_NULL(ts_data->dvdd)) {
        regulator_put(ts_data->dvdd);
        ts_data->dvdd = NULL;
    }

    return 0;
}

static int fts_power_source_suspend(struct fts_ts_data *ts_data)
{
    int ret = 0;

#if !defined(FTS_AOC_GESTURE_EN)
    ret = fts_power_source_ctrl(ts_data, DISABLE);
    if (ret < 0) {
        FTS_ERROR("power off fail, ret=%d", ret);
    }
#endif
#if FTS_PINCTRL_EN
    fts_pinctrl_select_suspend(ts_data);
#endif

    return ret;
}

static int fts_power_source_resume(struct fts_ts_data *ts_data)
{
    int ret = 0;
#if FTS_PINCTRL_EN
    fts_pinctrl_select_normal(ts_data);
#endif
#if !defined(FTS_AOC_GESTURE_EN)
    ret = fts_power_source_ctrl(ts_data, ENABLE);
    if (ret < 0) {
        FTS_ERROR("power on fail, ret=%d", ret);
    }
#endif
    return ret;
}
#endif /* FTS_POWER_SOURCE_CUST_EN */

static int fts_gpio_configure(struct fts_ts_data *data)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    /* request irq gpio */
    if (gpio_is_valid(data->pdata->irq_gpio)) {
        ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]irq gpio request failed");
            goto err_irq_gpio_req;
        }

        ret = gpio_direction_input(data->pdata->irq_gpio);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for irq gpio failed");
            goto err_irq_gpio_dir;
        }
    }

    /* request reset gpio */
    if (gpio_is_valid(data->pdata->reset_gpio)) {
        ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]reset gpio request failed");
            goto err_irq_gpio_dir;
        }

        ret = gpio_direction_output(data->pdata->reset_gpio, 0);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for reset gpio failed");
            goto err_reset_gpio_dir;
        }
    }

    FTS_FUNC_EXIT();
    return 0;

err_reset_gpio_dir:
    if (gpio_is_valid(data->pdata->reset_gpio))
        gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
    if (gpio_is_valid(data->pdata->irq_gpio))
        gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
    FTS_FUNC_EXIT();
    return ret;
}

static int fts_get_dt_coords(struct device *dev, char *name,
                             struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
    struct property *prop;
    struct device_node *np = dev->of_node;
    int coords_size;

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    coords_size = prop->length / sizeof(u32);
    if (coords_size != FTS_COORDS_ARR_SIZE) {
        FTS_ERROR("invalid:%s, size:%d", name, coords_size);
        return -EINVAL;
    }

    ret = of_property_read_u32_array(np, name, coords, coords_size);
    if (ret < 0) {
        FTS_ERROR("Unable to read %s, please check dts", name);
        pdata->x_min = FTS_X_MIN_DISPLAY_DEFAULT;
        pdata->y_min = FTS_Y_MIN_DISPLAY_DEFAULT;
        pdata->x_max = FTS_X_MAX_DISPLAY_DEFAULT;
        pdata->y_max = FTS_Y_MAX_DISPLAY_DEFAULT;
        return -ENODATA;
    } else {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
    }

    FTS_INFO("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max,
             pdata->y_min, pdata->y_max);
    return 0;
}

static int fts_check_panel_map(struct device_node *np,
    struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    int index;
    struct of_phandle_args panelmap;
    struct drm_panel *panel = NULL;

    if (of_property_read_bool(np, "focaltech,panel_map")) {
        for (index = 0 ;; index++) {
            ret = of_parse_phandle_with_fixed_args(np,
                    "focaltech,panel_map",
                    1,
                    index,
                    &panelmap);
            if (ret) {
                FTS_ERROR("Can't find display panel!\n");
                return -EPROBE_DEFER;
            }
            panel = of_drm_find_panel(panelmap.np);
            of_node_put(panelmap.np);
            if (!IS_ERR_OR_NULL(panel)) {
                pdata->panel = panel;
                pdata->initial_panel_index = panelmap.args[0];
                break;
            }
        }
    }
    return 0;
}

static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    struct device_node *np = dev->of_node;
    u32 temp_val = 0;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    u8 offload_id[4];
#endif

    FTS_FUNC_ENTER();

    /* Check if panel(s) exist or not. */
    ret = fts_check_panel_map(np, pdata);
    if (ret)
        return ret;

    ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
    if (ret < 0)
        FTS_ERROR("Unable to get display-coords");

    /* key */
    pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
    if (pdata->have_key) {
        ret = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key number undefined!");

        ret = of_property_read_u32_array(np, "focaltech,keys",
                                         pdata->keys, pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Keys undefined!");
        else if (pdata->key_number > FTS_MAX_KEYS)
            pdata->key_number = FTS_MAX_KEYS;

        ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
                                         pdata->key_x_coords,
                                         pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key Y Coords undefined!");

        ret = of_property_read_u32_array(np, "focaltech,key-y-coords",
                                         pdata->key_y_coords,
                                         pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key X Coords undefined!");

        FTS_INFO("VK Number:%d, key:(%d,%d,%d), "
                 "coords:(%d,%d),(%d,%d),(%d,%d)",
                 pdata->key_number,
                 pdata->keys[0], pdata->keys[1], pdata->keys[2],
                 pdata->key_x_coords[0], pdata->key_y_coords[0],
                 pdata->key_x_coords[1], pdata->key_y_coords[1],
                 pdata->key_x_coords[2], pdata->key_y_coords[2]);
    }

    /* reset, irq gpio info */
    pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
                        0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        FTS_ERROR("Unable to get reset_gpio");

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    pdata->offload_id = 0;
    ret = of_property_read_u8_array(np, "focaltech,touch_offload_id",
                                    offload_id, 4);
    if (ret == -EINVAL) {
        FTS_ERROR("Failed to read focaltech,touch_offload_id with error = %d\n",
            ret);
    } else {
        pdata->offload_id = *(u32 *)offload_id;
        FTS_DEBUG("Offload device ID = \"%c%c%c%c\" / 0x%08X\n",
            offload_id[0], offload_id[1], offload_id[2], offload_id[3],
            pdata->offload_id);
    }
#endif

    ret = of_property_read_u32(np, "focaltech,tx_ch_num", &temp_val);
    if (ret < 0) {
        FTS_ERROR("Unable to get tx_ch_num, please check dts");
    } else {
        pdata->tx_ch_num = temp_val;
        FTS_DEBUG("tx_ch_num = %d", pdata->tx_ch_num);
    }

    ret = of_property_read_u32(np, "focaltech,rx_ch_num", &temp_val);
    if (ret < 0) {
        FTS_ERROR("Unable to get rx_ch_num, please check dts");
    } else {
        pdata->rx_ch_num = temp_val;
        FTS_DEBUG("rx_ch_num = %d", pdata->rx_ch_num);
    }

    ret = of_property_read_u8(np, "focaltech,mm2px", &pdata->mm2px);
    if (ret < 0) {
        FTS_ERROR("Unable to get mm2px, please check dts");
        pdata->mm2px = 1;
    } else {
        FTS_DEBUG("mm2px = %d", pdata->mm2px);
    }

    pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
                      0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        FTS_ERROR("Unable to get irq_gpio");

    ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
    if (ret < 0) {
        FTS_ERROR("Unable to get max-touch-number, please check dts");
        pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
    } else {
        if (temp_val < 2)
            pdata->max_touch_number = 2; /* max_touch_number must >= 2 */
        else if (temp_val > FTS_MAX_POINTS_SUPPORT)
            pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
        else
            pdata->max_touch_number = temp_val;
    }

    FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
             pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);

    FTS_FUNC_EXIT();
    return 0;
}

static void fts_suspend_work(struct work_struct *work)
{
    struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
        suspend_work);

    FTS_DEBUG("Entry");

    mutex_lock(&ts_data->device_mutex);

    reinit_completion(&ts_data->bus_resumed);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    if (ts_data->power_status == FTS_TS_STATE_SUSPEND) {
        FTS_ERROR("Already suspended.\n");
        mutex_unlock(&ts_data->device_mutex);
        return;
    }
#endif
    fts_ts_suspend(ts_data->dev);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    ts_data->power_status = FTS_TS_STATE_SUSPEND;
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
    if (ts_data->tbn_register_mask) {
        tbn_release_bus(ts_data->tbn_register_mask);
        ts_data->tbn_owner = TBN_AOC;
    }
#endif
    mutex_unlock(&ts_data->device_mutex);
}

static void fts_resume_work(struct work_struct *work)
{
    struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
                                  resume_work);

    FTS_DEBUG("Entry");
    mutex_lock(&ts_data->device_mutex);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
    if (ts_data->tbn_register_mask) {
        tbn_request_bus(ts_data->tbn_register_mask);
        ts_data->tbn_owner = TBN_AP;
    }
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    if (ts_data->power_status == FTS_TS_STATE_POWER_ON) {
        FTS_ERROR("Already resumed.\n");
        mutex_unlock(&ts_data->device_mutex);
        return;
    }
#endif
    fts_ts_resume(ts_data->dev);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    ts_data->power_status = FTS_TS_STATE_POWER_ON;
#endif
    complete_all(&ts_data->bus_resumed);

    mutex_unlock(&ts_data->device_mutex);
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank = NULL;
    struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
                                  fb_notif);

    if (!evdata) {
        FTS_ERROR("evdata is null");
        return 0;
    }

    if (!(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("FB event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case FB_BLANK_UNBLANK:
        if (FB_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (FB_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case FB_BLANK_POWERDOWN:
        if (FB_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else if (FB_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("FB BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
static struct drm_panel *active_panel;

static int drm_check_dt(struct device_node *np)
{
    int i = 0;
    int count = 0;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(np, "panel", NULL);
    if (count <= 0) {
        FTS_ERROR("find drm_panel count(%d) fail", count);
        return -ENODEV;
    }

    for (i = 0; i < count; i++) {
        node = of_parse_phandle(np, "panel", i);
        panel = of_drm_find_panel(node);
        of_node_put(node);
        if (!IS_ERR(panel)) {
            FTS_INFO("find drm_panel successfully");
            active_panel = panel;
            return 0;
        }
    }

    FTS_ERROR("no find drm_panel");
    return -ENODEV;
}

static int drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct msm_drm_notifier *evdata = data;
    int *blank = NULL;
    struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
                                  fb_notif);

    if (!evdata) {
        FTS_ERROR("evdata is null");
        return 0;
    }

    if (!((event == DRM_PANEL_EARLY_EVENT_BLANK )
          || (event == DRM_PANEL_EVENT_BLANK))) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case DRM_PANEL_BLANK_UNBLANK:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case DRM_PANEL_BLANK_POWERDOWN:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("DRM BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#elif defined(CONFIG_ARCH_MSM)
static int drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct msm_drm_notifier *evdata = data;
    int *blank = NULL;
    struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
                                  fb_notif);

    if (!evdata) {
        FTS_ERROR("evdata is null");
        return 0;
    }

    if (!((event == MSM_DRM_EARLY_EVENT_BLANK )
          || (event == MSM_DRM_EVENT_BLANK))) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case MSM_DRM_BLANK_UNBLANK:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (MSM_DRM_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case MSM_DRM_BLANK_POWERDOWN:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else if (MSM_DRM_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("DRM BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void fts_ts_early_suspend(struct early_suspend *handler)
{
    struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
                                  early_suspend);

    cancel_work_sync(&fts_data->resume_work);
    fts_ts_suspend(ts_data->dev);
}

static void fts_ts_late_resume(struct early_suspend *handler)
{
    struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
                                  early_suspend);

    queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
}
#endif

static int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int pdata_size = sizeof(struct fts_ts_platform_data);

    FTS_FUNC_ENTER();
    ts_data->driver_probed = false;
    FTS_INFO("%s", FTS_DRIVER_VERSION);
    ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
    if (!ts_data->pdata) {
        FTS_ERROR("allocate memory for platform_data fail");
        return -ENOMEM;
    }

    if (ts_data->dev->of_node) {
        ret = fts_parse_dt(ts_data->dev, ts_data->pdata);
        if (ret)
            FTS_ERROR("device-tree parse fail");

#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
        ret = drm_check_dt(ts_data->dev->of_node);
        if (ret) {
            FTS_ERROR("parse drm-panel fail");
        }
#endif
#endif
    } else {
        if (ts_data->dev->platform_data) {
            memcpy(ts_data->pdata, ts_data->dev->platform_data, pdata_size);
        } else {
            FTS_ERROR("platform_data is null");
            return -ENODEV;
        }
    }

    ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
    if (!ts_data->ts_workqueue) {
        FTS_ERROR("create fts workqueue fail");
    }

    spin_lock_init(&ts_data->irq_lock);
    mutex_init(&ts_data->report_mutex);
    mutex_init(&ts_data->bus_lock);
    mutex_init(&ts_data->reg_lock);
    ts_data->is_deepsleep = false;

#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    ts_data->power_status = FTS_TS_STATE_POWER_ON;
    ts_data->bus_refmask = FTS_TS_BUS_REF_SCREEN_ON;
    mutex_init(&ts_data->bus_mutex);
#endif
    mutex_init(&ts_data->device_mutex);
    init_completion(&ts_data->bus_resumed);
    complete_all(&ts_data->bus_resumed);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
    if (register_tbn(&ts_data->tbn_register_mask)) {
        ret = -ENODEV;
        FTS_ERROR("Failed to register tbn context.\n");
        goto err_init_tbn;
    }
    ts_data->tbn_owner = TBN_AP;
    FTS_INFO("tbn_register_mask = %#x.\n", ts_data->tbn_register_mask);
#endif

    /* Init communication interface */
    ret = fts_bus_init(ts_data);
    if (ret) {
        FTS_ERROR("bus initialize fail");
        goto err_bus_init;
    }

    ret = fts_input_init(ts_data);
    if (ret) {
        FTS_ERROR("input initialize fail");
        goto err_input_init;
    }

    ret = fts_report_buffer_init(ts_data);
    if (ret) {
        FTS_ERROR("report buffer init fail");
        goto err_report_buffer;
    }

    ret = fts_gpio_configure(ts_data);
    if (ret) {
        FTS_ERROR("configure the gpios fail");
        goto err_gpio_config;
    }

#if FTS_POWER_SOURCE_CUST_EN
    ret = fts_power_source_init(ts_data);
    if (ret) {
        FTS_ERROR("fail to get power(regulator)");
        goto err_power_init;
    }
#endif

#if (!FTS_CHIP_IDC)
    fts_reset_proc(FTS_RESET_INTERVAL);
#endif

    ret = fts_get_ic_information(ts_data);
    if (ret) {
        FTS_ERROR("not focal IC, unregister driver");
        goto err_power_init;
    }

    ret = fts_create_apk_debug_channel(ts_data);
    if (ret) {
        FTS_ERROR("create apk debug node fail");
    }

#if GOOGLE_REPORT_MODE
    memset(ts_data->current_host_status, 0, FTS_CUSTOMER_STATUS_LEN);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    ts_data->fw_default_heatmap_mode = FW_HEATMAP_MODE_UNCOMPRESSED;
    /* Update ts_data->fw_default_heatmap_mode from firmware setting */
    fts_get_default_heatmap_mode(ts_data);
    ts_data->fw_heatmap_mode = ts_data->fw_default_heatmap_mode;
    ts_data->compress_heatmap_wlen = 0;
#endif
    ts_data->enable_fw_grip = FW_GRIP_ENABLE;
    ts_data->enable_fw_palm = FW_GRIP_ENABLE;
    ts_data->set_continuously_report = ENABLE;
    ts_data->glove_mode = DISABLE;
    fts_update_feature_setting(ts_data);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    ts_data->offload.caps.touch_offload_major_version = TOUCH_OFFLOAD_INTERFACE_MAJOR_VERSION;
    ts_data->offload.caps.touch_offload_minor_version = TOUCH_OFFLOAD_INTERFACE_MINOR_VERSION;
    ts_data->offload.caps.device_id = ts_data->pdata->offload_id;
    ts_data->offload.caps.display_width = ts_data->pdata->x_max;
    ts_data->offload.caps.display_height = ts_data->pdata->y_max;
    ts_data->offload.caps.tx_size = ts_data->pdata->tx_ch_num;
    ts_data->offload.caps.rx_size = ts_data->pdata->rx_ch_num;

    ts_data->offload.caps.bus_type = BUS_TYPE_SPI;
    ts_data->offload.caps.bus_speed_hz = 10000000;
    ts_data->offload.caps.heatmap_size = HEATMAP_SIZE_FULL;
    ts_data->offload.caps.touch_data_types = TOUCH_DATA_TYPE_COORD |
                                             TOUCH_DATA_TYPE_STRENGTH |
                                             TOUCH_DATA_TYPE_FILTERED |
                                             TOUCH_DATA_TYPE_RAW;
    ts_data->offload.caps.touch_scan_types = TOUCH_SCAN_TYPE_MUTUAL |
                                             TOUCH_SCAN_TYPE_SELF;
    ts_data->offload.caps.continuous_reporting = true;
    ts_data->offload.caps.noise_reporting = false;
    ts_data->offload.caps.cancel_reporting = true;
    ts_data->offload.caps.rotation_reporting = true;
    ts_data->offload.caps.size_reporting = true;
    ts_data->offload.caps.filter_grip = true;
    ts_data->offload.caps.filter_palm = true;
    ts_data->offload.caps.num_sensitivity_settings = 1;

    ts_data->offload.hcallback = (void *)ts_data;
    ts_data->offload.report_cb = fts_offload_report;
    touch_offload_init(&ts_data->offload);

    ts_data->touch_offload_active_coords = 0;
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD) || \
    IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)

    /* |-   MS    -|- Normal-SS -|- Water-SS  -|
     * |- Tx*Rx*2 -|- (Tx+Rx)*2 -|- (Tx+Rx)*2 -|
     * Total size = 1288 bytes
     */
    if (!ts_data->heatmap_buff) {
        ts_data->heatmap_buff_size = sizeof(u16) *
            ((ts_data->pdata->tx_ch_num * ts_data->pdata->rx_ch_num) +
            (ts_data->pdata->tx_ch_num + ts_data->pdata->rx_ch_num) * 2);
        FTS_DEBUG("Allocate heatmap_buff size=%d\n", ts_data->heatmap_buff_size);
        ts_data->heatmap_buff = kmalloc(ts_data->heatmap_buff_size, GFP_KERNEL);
        if (!ts_data->heatmap_buff) {
            FTS_ERROR("allocate heatmap_buff failed\n");
            goto err_heatmap_buff;
        }
    }

    /* |- cap header (91) -|- Water-SS -|- Normal-SS -|- Normal-MS -|
     * |-        91       -|-   68*2   -|-   68*2    -|-  16*34*2  -|
     * Total size = 1379 bytes
     */
    if (!ts_data->heatmap_raw) {
        int node_num = ts_data->pdata->tx_ch_num * ts_data->pdata->rx_ch_num;
        ts_data->heatmap_raw_size = FTS_CAP_DATA_LEN +
            ((node_num + FTS_SELF_DATA_LEN * 2) * sizeof(u16));
        FTS_DEBUG("Allocate heatmap_raw size=%d\n", ts_data->heatmap_raw_size);
        ts_data->heatmap_raw = kmalloc(ts_data->heatmap_raw_size, GFP_KERNEL);
        if (!ts_data->heatmap_raw) {
            FTS_ERROR("allocate heatmap_raw failed\n");
            goto err_heatmap_raw;
        }
    }

    /* |-   MS    -|
     * |- 16*34*2 -|
     */
    if (!ts_data->trans_raw) {
        int node_num = ts_data->pdata->tx_ch_num * ts_data->pdata->rx_ch_num;
        ts_data->trans_raw_size = node_num * sizeof(u16);
        FTS_DEBUG("Allocate trans_raw size=%d\n", ts_data->trans_raw_size);
        ts_data->trans_raw = kmalloc(ts_data->trans_raw_size, GFP_KERNEL);
        if (!ts_data->trans_raw) {
            FTS_ERROR("allocate trans_raw failed\n");
            goto err_trans_raw;
        }
    }
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    /*
     * Heatmap_probe must be called before irq routine is registered,
     * because heatmap_read is called from interrupt context.
    */
    ts_data->v4l2.parent_dev = ts_data->dev;
    ts_data->v4l2.input_dev = ts_data->input_dev;
    ts_data->v4l2.read_frame = v4l2_read_frame;
    ts_data->v4l2.width = ts_data->pdata->tx_ch_num;
    ts_data->v4l2.height = ts_data->pdata->rx_ch_num;
    /* 180 Hz operation */
    ts_data->v4l2.timeperframe.numerator = 1;
    ts_data->v4l2.timeperframe.denominator = 180;
    ret = heatmap_probe(&ts_data->v4l2);
    if (ret < 0) {
        FTS_ERROR("heatmap probe unsuccessfully!");
        goto err_heatmap_probe;
    } else {
        FTS_DEBUG("heatmap probe successfully!");
    }
#endif

    /* init motion filter mode and state. */
    ts_data->mf_mode = MF_DYNAMIC;
    ts_data->mf_state = MF_UNFILTERED;

    ret = fts_create_sysfs(ts_data);
    if (ret) {
        FTS_ERROR("create sysfs node fail");
    }

#if FTS_POINT_REPORT_CHECK_EN
    ret = fts_point_report_check_init(ts_data);
    if (ret) {
        FTS_ERROR("init point report check fail");
    }
#endif

    ret = fts_ex_mode_init(ts_data);
    if (ret) {
        FTS_ERROR("init glove/cover/charger fail");
    }

    ret = fts_gesture_init(ts_data);
    if (ret) {
        FTS_ERROR("init gesture fail");
    }

#if FTS_TEST_EN
    ret = fts_test_init(ts_data);
    if (ret) {
        FTS_ERROR("init production test fail");
    }
#endif

#if FTS_ESDCHECK_EN
    ret = fts_esdcheck_init(ts_data);
    if (ret) {
        FTS_ERROR("init esd check fail");
    }
#endif
    /* init pm_qos before interrupt registered. */
    cpu_latency_qos_add_request(&ts_data->pm_qos_req, PM_QOS_DEFAULT_VALUE);

    ret = fts_irq_registration(ts_data);
    if (ret) {
        FTS_ERROR("request irq failed");
        goto err_irq_req;
    }

#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    ret = register_panel_bridge(ts_data);
    if (ret < 0) {
        FTS_ERROR("register_panel_bridge failed. ret = 0x%08X\n", ret);
    }
#endif

    if (ts_data->ts_workqueue) {
        INIT_WORK(&ts_data->resume_work, fts_resume_work);
        INIT_WORK(&ts_data->suspend_work, fts_suspend_work);
    }

    ret = fts_fwupg_init(ts_data);
    if (ret) {
        FTS_ERROR("init fw upgrade fail");
    }

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
    init_completion(&ts_data->pm_completion);
    ts_data->pm_suspend = false;
#endif

#if defined(CONFIG_FB)
    ts_data->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts_data->fb_notif);
    if (ret) {
        FTS_ERROR("[FB]Unable to register fb_notifier: %d", ret);
    }
#elif defined(CONFIG_DRM_PANEL) || defined(CONFIG_ARCH_MSM)
    ts_data->fb_notif.notifier_call = drm_notifier_callback;
#if defined(CONFIG_DRM_PANEL)
    if (active_panel) {
        ret = drm_panel_notifier_register(active_panel, &ts_data->fb_notif);
        if (ret)
            FTS_ERROR("[DRM]drm_panel_notifier_register fail: %d\n", ret);
    }
#elif defined(CONFIG_ARCH_MSM)
    ret = msm_drm_register_client(&ts_data->fb_notif);
    if (ret) {
        FTS_ERROR("[DRM]Unable to register fb_notifier: %d\n", ret);
    }
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
    ts_data->early_suspend.suspend = fts_ts_early_suspend;
    ts_data->early_suspend.resume = fts_ts_late_resume;
    register_early_suspend(&ts_data->early_suspend);
#endif

    ts_data->work_mode = FTS_REG_WORKMODE_WORK_VALUE;
#if GOOGLE_REPORT_MODE
    fts_read_reg(FTS_REG_CUSTOMER_STATUS, &ts_data->current_host_status[0]);
    FTS_INFO("-------Palm mode %s\n",
        (ts_data->current_host_status[0] & (1 << STATUS_PALM)) ? "enter" : "exit");
    FTS_INFO("-------Water mode %s\n",
        (ts_data->current_host_status[0] & (1 << STATUS_WATER)) ? "enter" : "exit");
    FTS_INFO("-------Grip mode %s\n",
        (ts_data->current_host_status[0] & (1 << STATUS_GRIP)) ? "enter" : "exit");
    FTS_INFO("-------Glove mode %s\n",
        (ts_data->current_host_status[0] & (1 << STATUS_GLOVE)) ? "enter" : "exit");
    FTS_INFO("-------Edge palm %s\n",
        (ts_data->current_host_status[0] & (1 << STATUS_EDGE_PALM)) ? "enter" : "exit");
    FTS_INFO("-------Reset %s\n",
        (ts_data->current_host_status[0] & (1 << STATUS_RESET)) ? "enter" : "exit");
#endif

    ts_data->driver_probed = true;
    FTS_FUNC_EXIT();
    return 0;

err_irq_req:
    cpu_latency_qos_remove_request(&ts_data->pm_qos_req);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    heatmap_remove(&ts_data->v4l2);
err_heatmap_probe:
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD) || \
    IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    kfree_safe(ts_data->trans_raw);
err_trans_raw:
    kfree_safe(ts_data->heatmap_raw);
err_heatmap_raw:
    kfree_safe(ts_data->heatmap_buff);
err_heatmap_buff:
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    touch_offload_cleanup(&ts_data->offload);
#endif

#if FTS_POWER_SOURCE_CUST_EN
err_power_init:
    fts_power_source_exit(ts_data);
#endif
    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);
    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);
err_gpio_config:
    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);
err_report_buffer:
    input_unregister_device(ts_data->input_dev);
#if FTS_PEN_EN
    input_unregister_device(ts_data->pen_dev);
#endif
err_input_init:
    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);
err_bus_init:
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
    if (ts_data->tbn_register_mask)
        unregister_tbn(&ts_data->tbn_register_mask);
err_init_tbn:
#endif
    kfree_safe(ts_data->bus_tx_buf);
    kfree_safe(ts_data->bus_rx_buf);
    kfree_safe(ts_data->pdata);

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();

    free_irq(ts_data->irq, ts_data);

#if FTS_POINT_REPORT_CHECK_EN
    fts_point_report_check_exit(ts_data);
#endif
    fts_release_apk_debug_channel(ts_data);

#if FTS_TEST_EN
    /* remove the test nodes and sub-dir in /proc/focaltech_touch/selftest/ */
    fts_test_exit(ts_data);
#endif
    /* remove all nodes and sub-dir in /proc/focaltech_touch/ */
    fts_remove_sysfs(ts_data);

    fts_ex_mode_exit(ts_data);

    fts_fwupg_exit(ts_data);

#if FTS_ESDCHECK_EN
    fts_esdcheck_exit(ts_data);
#endif

    fts_gesture_exit(ts_data);
    fts_bus_exit(ts_data);

    input_unregister_device(ts_data->input_dev);
#if FTS_PEN_EN
    input_unregister_device(ts_data->pen_dev);
#endif

    cancel_work_sync(&ts_data->suspend_work);
    cancel_work_sync(&ts_data->resume_work);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
    if (ts_data->tbn_register_mask)
        unregister_tbn(&ts_data->tbn_register_mask);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    unregister_panel_bridge(&ts_data->panel_bridge);
#endif

    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);

    cpu_latency_qos_remove_request(&ts_data->pm_qos_req);

#if defined(CONFIG_FB)
    if (fb_unregister_client(&ts_data->fb_notif))
        FTS_ERROR("[FB]Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
    if (active_panel)
        drm_panel_notifier_unregister(active_panel, &ts_data->fb_notif);
#elif defined(CONFIG_ARCH_MSM)
    if (msm_drm_unregister_client(&ts_data->fb_notif))
        FTS_ERROR("[DRM]Error occurred while unregistering fb_notifier.\n");
#endif
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&ts_data->early_suspend);
#endif

#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    touch_offload_cleanup(&ts_data->offload);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD) || \
    IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    kfree_safe(ts_data->heatmap_buff);
    kfree_safe(ts_data->heatmap_raw);
    kfree_safe(ts_data->trans_raw);
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    heatmap_remove(&ts_data->v4l2);
#endif

    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);

    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);

#if FTS_POWER_SOURCE_CUST_EN
    fts_power_source_exit(ts_data);
#endif

    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);

    kfree_safe(ts_data->pdata);
    kfree_safe(ts_data);

    FTS_FUNC_EXIT();

    return 0;
}

static int fts_write_reg_safe(u8 reg, u8 write_val) {
    int ret = 0;
    int i;
    int j;
    u8 reg_val;

    for (i = 0; i < MAX_RETRY_CNT; i++) {
        ret = fts_write_reg(reg, write_val);
        if (ret < 0) {
            FTS_DEBUG("write 0x%X failed", reg);
            return ret;
        }
        for (j = 0; j < MAX_RETRY_CNT; j++) {
            reg_val = 0xFF;
            ret = fts_read_reg(reg, &reg_val);
            if (ret < 0) {
                FTS_DEBUG("read 0x%X failed", reg);
                return ret;
            }
            if (write_val == reg_val) {
                return ret;
            }
            msleep(1);
        }

        FTS_ERROR("%s failed, reg(0x%X), write_val(0x%x), reg_val(0x%x), " \
            "retry(%d)", __func__, reg, write_val, reg_val, i);
    }
    if (i == MAX_RETRY_CNT)
        ret = -EIO;
    return ret;
}

static void fts_update_host_feature_setting(struct fts_ts_data *ts_data,
    bool en, u8 fw_mode_setting){
    if (en)
        ts_data->current_host_status[1] |= 1 << fw_mode_setting;
    else
        ts_data->current_host_status[1] &= ~(1 << fw_mode_setting);
}

int fts_get_default_heatmap_mode(struct fts_ts_data *ts_data)
{
    int ret = 0;
    u8 value_heatmap = 0;
    u8 value_compressed = 0;
    u8 reg_heatmap = FTS_REG_HEATMAP_9E;
    u8 reg_compressed = FTS_REG_HEATMAP_ED;

    if (!ts_data->driver_probed || ts_data->fw_loading) {
        ret = fts_read_reg(reg_compressed, &value_compressed);
        if (ret) {
            FTS_ERROR("Read reg(%2X) error!", reg_compressed);
            goto exit;
        }

        ret = fts_read_reg(reg_heatmap, &value_heatmap);
        if (ret) {
            FTS_ERROR("Read reg(%2X) error!", reg_heatmap);
            goto exit;
        }

        if (value_heatmap == 0)
            ts_data->fw_default_heatmap_mode = FW_HEATMAP_MODE_DISABLE;
        else if (value_compressed)
            ts_data->fw_default_heatmap_mode = FW_HEATMAP_MODE_COMPRESSED;
        else
            ts_data->fw_default_heatmap_mode = FW_HEATMAP_MODE_UNCOMPRESSED;

exit:
        if (ret == 0) {
            FTS_DEBUG("Default fw_heatamp is %s and %s.\n",
                value_compressed? "compressed" : "uncompressed",
                value_heatmap ? "enabled" : "disabled");
        }
    }
    return ret;
}

int fts_set_heatmap_mode(struct fts_ts_data *ts_data, u8 heatmap_mode)
{
    int ret = 0;
    u8 value_heatmap = 0;
    u8 value_compressed = 0;
    u8 reg_heatmap = FTS_REG_HEATMAP_9E;
    u8 reg_compressed = FTS_REG_HEATMAP_ED;
    int count = 0;
    char tmpbuf[FTS_MESSAGE_LENGTH];

    switch (heatmap_mode) {
      case FW_HEATMAP_MODE_DISABLE:
          value_heatmap = DISABLE;
          count += scnprintf(tmpbuf + count, FTS_MESSAGE_LENGTH - count,
              "Disable fw_heatmap");
          break;
      case FW_HEATMAP_MODE_COMPRESSED:
          value_heatmap = ENABLE;
          value_compressed = ENABLE;
          count += scnprintf(tmpbuf + count, FTS_MESSAGE_LENGTH - count,
              "Enable compressed fw_heatmap");
          break;
      case FW_HEATMAP_MODE_UNCOMPRESSED:
          value_heatmap = ENABLE;
          value_compressed = DISABLE;
          count += scnprintf(tmpbuf + count, FTS_MESSAGE_LENGTH - count,
              "Enable uncompressed fw_heatmap");
          break;
      default:
          FTS_ERROR("The input heatmap more(%d) is invalid.", heatmap_mode);
          return -EINVAL;
    }

    ret = fts_write_reg_safe(reg_compressed, value_compressed);
    if (ret) {
        goto exit;
    }
    ret = fts_write_reg_safe(reg_heatmap, value_heatmap);
    if (ret == 0) {
      ts_data->fw_heatmap_mode = heatmap_mode;
      fts_update_host_feature_setting(ts_data, value_heatmap, FW_HEATMAP);
    }

exit:
    FTS_DEBUG("%s %s.\n", tmpbuf,
        (ret == 0) ? "successfully" : "unsuccessfully");

    return ret;
}

int fts_set_grip_mode(struct fts_ts_data *ts_data, u8 grip_mode)
{
    int ret = 0;
    bool en = grip_mode % 2;
    u8 value = en ? 0x00 : 0xAA;
    u8 reg = FTS_REG_EDGE_MODE_EN;

    ret = fts_write_reg_safe(reg, value);
    if (ret == 0) {
        fts_update_host_feature_setting(ts_data, en, FW_GRIP);
    }

    FTS_DEBUG("%s fw_grip(%d) %s.\n", en ? "Enable" : "Disable",
        ts_data->enable_fw_grip,
        (ret == 0)  ? "successfully" : "unsuccessfully");
    return ret;
}

int fts_set_palm_mode(struct fts_ts_data *ts_data, u8 palm_mode)
{
    int ret = 0;
    bool en = palm_mode % 2;
    u8 value = en ? ENABLE : DISABLE;
    u8 reg = FTS_REG_PALM_EN;

    ret = fts_write_reg_safe(reg, value);
    if (ret == 0) {
        fts_update_host_feature_setting(ts_data, en, FW_PALM);
    }

    FTS_DEBUG("%s fw_palm(%d) %s.\n", en ? "Enable" : "Disable",
        ts_data->enable_fw_palm,
        (ret == 0) ? "successfully" : "unsuccessfully");
    return ret;
}

int fts_set_continuous_mode(struct fts_ts_data *ts_data, bool en)
{
    int ret = 0;
    u8 value = en ? ENABLE : DISABLE;
    u8 reg = FTS_REG_CONTINUOUS_EN;

    mutex_lock(&ts_data->reg_lock);
    if (!ts_data->is_deepsleep) {
        ret = fts_write_reg_safe(reg, value);
        if (ret == 0) {
            ts_data->set_continuously_report = value;
            fts_update_host_feature_setting(ts_data, en, FW_CONTINUOUS);
        }

        PR_LOGD("%s fw_continuous %s.\n", en ? "Enable" : "Disable",
            (ret == 0) ? "successfully" : "unsuccessfully");
    }
    mutex_unlock(&ts_data->reg_lock);
    return ret;
}

int fts_set_glove_mode(struct fts_ts_data *ts_data, bool en)
{
    int ret = 0;
    u8 value = en ? ENABLE : DISABLE;
    u8 reg = FTS_REG_GLOVE_MODE_EN;

    ret = fts_write_reg_safe(reg, value);
    if (ret == 0) {
        ts_data->glove_mode = value;
        fts_update_host_feature_setting(ts_data, en, FW_GLOVE);
    }

    FTS_DEBUG("%s fw_glove %s.\n", en ? "Enable" : "Disable",
        (ret == 0) ? "successfully" : "unsuccessfully");
    return ret;
}

/**
 * fts_update_feature_setting()
 *
 * Restore the feature settings after the device resume.
 *
 * @param
 *    [ in] ts_data: touch driver handle.
 *
 */
void fts_update_feature_setting(struct fts_ts_data *ts_data)
{
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    fts_set_heatmap_mode(ts_data, ts_data->fw_default_heatmap_mode);
#endif

    fts_set_grip_mode(ts_data, ts_data->enable_fw_grip);

    fts_set_palm_mode(ts_data, ts_data->enable_fw_palm);

    fts_set_continuous_mode(ts_data, ts_data->set_continuously_report);

    fts_set_glove_mode(ts_data, ts_data->glove_mode);
}

static int fts_ts_suspend(struct device *dev)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    FTS_FUNC_ENTER();
    if (ts_data->bus_refmask)
        FTS_DEBUG("bus_refmask 0x%X\n", ts_data->bus_refmask);


    if (ts_data->suspended) {
        FTS_INFO("Already in suspend state");
        return 0;
    }

    if (ts_data->fw_loading) {
        FTS_INFO("fw upgrade in process, can't suspend");
        return 0;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_suspend();
#endif

    if (ts_data->gesture_mode) {
        fts_gesture_suspend(ts_data);
    } else {
        if (ts_data->bus_refmask == FTS_TS_BUS_REF_BUGREPORT &&
            ktime_ms_delta(ktime_get(), ts_data->bugreport_ktime_start) >
            30 * MSEC_PER_SEC) {
            fts_ts_set_bus_ref(ts_data, FTS_TS_BUS_REF_BUGREPORT, false);
            pm_relax(ts_data->dev);
            ts_data->bugreport_ktime_start = 0;
            FTS_DEBUG("Force release FTS_TS_BUS_REF_BUGREPORT reference bit.");
            return -EBUSY;
        }

        /* Disable irq */
        fts_irq_disable();

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
        fts_set_heatmap_mode(ts_data, FW_HEATMAP_MODE_DISABLE);
#endif
        FTS_DEBUG("make TP enter into sleep mode");
        mutex_lock(&ts_data->reg_lock);
        ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
        ts_data->is_deepsleep = true;
        mutex_unlock(&ts_data->reg_lock);
        if (ret < 0)
            FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

        if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
            ret = fts_power_source_suspend(ts_data);
            if (ret < 0) {
                FTS_ERROR("power enter suspend fail");
            }
#endif
        }
    }

    fts_release_all_finger();
    ts_data->suspended = true;
    FTS_FUNC_EXIT();
    return 0;
}


/**
 * Report a finger down event on the long press gesture area then immediately
 * report a cancel event(MT_TOOL_PALM).
 */
static void fts_report_cancel_event(struct fts_ts_data *ts_data)
{
    FTS_INFO("Report cancel event for UDFPS");

    mutex_lock(&ts_data->report_mutex);
    /* Finger down on UDFPS area. */
    input_mt_slot(ts_data->input_dev, 0);
    input_report_key(ts_data->input_dev, BTN_TOUCH, 1);
    input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, 1);
    input_report_abs(ts_data->input_dev, ABS_MT_POSITION_X,
        ts_data->fts_gesture_data.coordinate_x[0]);
    input_report_abs(ts_data->input_dev, ABS_MT_POSITION_Y,
        ts_data->fts_gesture_data.coordinate_y[0]);
    input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MAJOR,
        ts_data->fts_gesture_data.major[0]);
    input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MINOR,
        ts_data->fts_gesture_data.minor[0]);
#ifndef SKIP_PRESSURE
    input_report_abs(ts_data->input_dev, ABS_MT_PRESSURE, 1);
#endif
    input_report_abs(ts_data->input_dev, ABS_MT_ORIENTATION,
        ts_data->fts_gesture_data.orientation[0]);
    input_sync(ts_data->input_dev);

    /* Report MT_TOOL_PALM for canceling the touch event. */
    input_mt_slot(ts_data->input_dev, 0);
    input_report_key(ts_data->input_dev, BTN_TOUCH, 1);
    input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_PALM, 1);
    input_sync(ts_data->input_dev);

    /* Release touches. */
    input_mt_slot(ts_data->input_dev, 0);
#ifndef SKIP_PRESSURE
    input_report_abs(ts_data->input_dev, ABS_MT_PRESSURE, 0);
#endif
    input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, 0);
    input_report_abs(ts_data->input_dev, ABS_MT_TRACKING_ID, -1);
    input_report_key(ts_data->input_dev, BTN_TOUCH, 0);
    input_sync(ts_data->input_dev);
    mutex_unlock(&ts_data->report_mutex);
}

static void fts_check_finger_status(struct fts_ts_data *ts_data)
{
    int ret = 0;
    u8 power_mode = FTS_REG_POWER_MODE_SLEEP;
    ktime_t timeout = ktime_add_ms(ktime_get(), 500); /* 500ms. */

    /* If power mode is deep sleep mode, then reurn. */
    ret = fts_read_reg(FTS_REG_POWER_MODE, &power_mode);
    if (ret)
        return;

    if (power_mode == FTS_REG_POWER_MODE_SLEEP)
        return;

    while (ktime_get() < timeout) {
        ret = fts_gesture_readdata(ts_data, false);
        if (ret)
            break;

        if (ts_data->fts_gesture_data.gesture_id == FTS_GESTURE_ID_LPTW_DOWN) {
            msleep(30);
            continue;
        }

        if (ts_data->fts_gesture_data.gesture_id == FTS_GESTURE_ID_LPTW_UP ||
            ts_data->fts_gesture_data.gesture_id == FTS_GESTURE_ID_STTW) {
            fts_report_cancel_event(ts_data);
        }
        break;
    }
}

static int fts_ts_resume(struct device *dev)
{
    struct fts_ts_data *ts_data = fts_data;
    int ret = 0;

    FTS_FUNC_ENTER();
    if (!ts_data->suspended) {
        FTS_DEBUG("Already in awake state");
        return 0;
    }

    fts_release_all_finger();

    if (!ts_data->ic_info.is_incell) {
        if (!ts_data->gesture_mode) {
#if FTS_POWER_SOURCE_CUST_EN
            fts_power_source_resume(ts_data);
#endif
            fts_check_finger_status(ts_data);
        }

        fts_reset_proc(FTS_RESET_INTERVAL);
    }

    ret = fts_wait_tp_to_valid();
    if (ret != 0) {
        FTS_ERROR("Resume has been cancelled by wake up timeout");
#if FTS_POWER_SOURCE_CUST_EN
        if (!ts_data->gesture_mode)
            fts_power_source_suspend(ts_data);
#endif
        return ret;
    }

    ts_data->is_deepsleep = false;
    fts_ex_mode_recovery(ts_data);

#if FTS_ESDCHECK_EN
    fts_esdcheck_resume();
#endif

    if (ts_data->gesture_mode) {
        fts_gesture_resume(ts_data);
    } else {
        fts_irq_enable();
    }

    ts_data->suspended = false;
    FTS_FUNC_EXIT();
    return 0;
}

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
static int fts_pm_suspend(struct device *dev)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_INFO("system enters into pm_suspend");
    ts_data->pm_suspend = true;
    reinit_completion(&ts_data->pm_completion);
    return 0;
}

static int fts_pm_resume(struct device *dev)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_INFO("system resumes from pm_suspend");
    ts_data->pm_suspend = false;
    complete(&ts_data->pm_completion);
    return 0;
}

static const struct dev_pm_ops fts_dev_pm_ops = {
    .suspend = fts_pm_suspend,
    .resume = fts_pm_resume,
};
#endif

/*****************************************************************************
* TP Driver
*****************************************************************************/
static int fts_ts_probe(struct spi_device *spi)
{
    int ret = 0;
    struct fts_ts_data *ts_data = NULL;

    FTS_INFO("Touch Screen(SPI BUS) driver proboe...");
    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;
    spi->rt = true;
    ret = spi_setup(spi);
    if (ret) {
        FTS_ERROR("spi setup fail");
        return ret;
    }

    /* malloc memory for global struct variable */
    ts_data = (struct fts_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        FTS_ERROR("allocate memory for fts_data fail");
        return -ENOMEM;
    }

    fts_data = ts_data;
    ts_data->spi = spi;
    ts_data->dev = &spi->dev;
    ts_data->log_level = FTS_KEY_LOG_LEVEL;

    ts_data->bus_type = FTS_BUS_TYPE_SPI_V2;
    spi_set_drvdata(spi, ts_data);

    ret = fts_ts_probe_entry(ts_data);
    if (ret) {
        FTS_ERROR("Touch Screen(SPI BUS) driver probe fail");
        kfree_safe(ts_data);
        return ret;
    }

    FTS_INFO("Touch Screen(SPI BUS) driver probe successfully");
    return 0;
}

static void fts_ts_remove(struct spi_device *spi)
{
    fts_ts_remove_entry(spi_get_drvdata(spi));
}

static void fts_ts_shutdown(struct spi_device *spi)
{
    fts_ts_remove(spi);
}

static const struct spi_device_id fts_ts_id[] = {
    {FTS_DRIVER_NAME, 0},
    {},
};
static const struct of_device_id fts_dt_match[] = {
    {.compatible = "focaltech,ts", },
    {},
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct spi_driver fts_ts_driver = {
    .probe = fts_ts_probe,
    .remove = fts_ts_remove,
    .shutdown = fts_ts_shutdown,
    .driver = {
        .name = FTS_DRIVER_NAME,
        .owner = THIS_MODULE,
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
        .pm = &fts_dev_pm_ops,
#endif
        .of_match_table = of_match_ptr(fts_dt_match),
    },
    .id_table = fts_ts_id,
};

static int __init fts_ts_init(void)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    ret = spi_register_driver(&fts_ts_driver);
    if ( ret != 0 ) {
        FTS_ERROR("Focaltech touch screen driver init failed!");
    }
    FTS_FUNC_EXIT();
    return ret;
}

static void __exit fts_ts_exit(void)
{
    spi_unregister_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
