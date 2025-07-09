/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
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
* File Name: focaltech_common.h
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-16
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_FOCALTECH_COMMON_H__
#define __LINUX_FOCALTECH_COMMON_H__

#include "focaltech_config.h"
#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
#include <drm/drm_bridge.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <samsung/exynos_drm_connector.h>
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
#include <touch_bus_negotiator.h>
#endif

/*****************************************************************************
* Macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_VERSION                  "Focaltech V3.3 20201229"

#define BYTE_OFF_0(x)           (u8)((x) & 0xFF)
#define BYTE_OFF_8(x)           (u8)(((x) >> 8) & 0xFF)
#define BYTE_OFF_16(x)          (u8)(((x) >> 16) & 0xFF)
#define BYTE_OFF_24(x)          (u8)(((x) >> 24) & 0xFF)
#define FLAGBIT(x)              (0x00000001 << (x))
#define FLAGBITS(x, y)          ((0xFFFFFFFF >> (32 - (y) - 1)) & (0xFFFFFFFF << (x)))

#define FLAG_ICSERIALS_LEN      8
#define FLAG_HID_BIT            10
#define FLAG_IDC_BIT            11

#define IC_SERIALS              (FTS_CHIP_TYPE & FLAGBITS(0, FLAG_ICSERIALS_LEN-1))
#define IC_TO_SERIALS(x)        ((x) & FLAGBITS(0, FLAG_ICSERIALS_LEN-1))
#define FTS_CHIP_IDC            ((FTS_CHIP_TYPE & FLAGBIT(FLAG_IDC_BIT)) == FLAGBIT(FLAG_IDC_BIT))
#define FTS_HID_SUPPORTTED      ((FTS_CHIP_TYPE & FLAGBIT(FLAG_HID_BIT)) == FLAGBIT(FLAG_HID_BIT))

#define FTS_MAX_CHIP_IDS        8
#define FTS_RESET_INTERVAL      200

#define FTS_CHIP_TYPE_MAPPING   {{0x88, 0x56, 0x52, 0x00, 0x00, 0x00, 0x00, 0x56, 0xB2}}

#define FTS_STTW_E3_BUF_LEN                 13
#define FTS_LPTW_E2_BUF_LEN                 13
#define FTS_LPTW_E1_BUF_LEN                 12
#define FTS_LPTW_BUF_LEN                    (max(FTS_LPTW_E1_BUF_LEN, FTS_LPTW_E2_BUF_LEN))

#define FILE_NAME_LENGTH                    128
#define FTS_MESSAGE_LENGTH                  128
#define ENABLE                              1
#define DISABLE                             0
#define VALID                               1
#define INVALID                             0
#define MAX_RETRY_CNT                       3
#define FTS_CMD_START1                      0x55
#define FTS_CMD_START2                      0xAA
#define FTS_CMD_START_DELAY                 12
#define FTS_CMD_READ_ID                     0x90
#define FTS_CMD_READ_ID_LEN                 4
#define FTS_CMD_READ_ID_LEN_INCELL          1
#define FTS_CMD_READ_FW_CONF                0xA8
#define FTS_CMD_READ_TOUCH_DATA             0x01
/*register address*/
#define FTS_REG_INT_CNT                     0x8F
#define FTS_REG_FLOW_WORK_CNT               0x91
#define FTS_REG_WORKMODE                    0x00
#define FTS_REG_WORKMODE_FACTORY_VALUE      0x40
#define FTS_REG_WORKMODE_WORK_VALUE         0x00
#define FTS_REG_ESDCHECK_DISABLE            0x8D
#define FTS_REG_CHIP_ID                     0xA3
#define FTS_REG_CHIP_ID2                    0x9F
#define FTS_REG_POWER_MODE                  0xA5
#define FTS_REG_POWER_MODE_SLEEP            0x03
#define FTS_REG_FW_MAJOR_VER                0xA6
#define FTS_REG_FW_MINOR_VER                0xAD
#define FTS_REG_VENDOR_ID                   0xA8
#define FTS_REG_LCD_BUSY_NUM                0xAB
#define FTS_REG_FACE_DEC_MODE_EN            0xB0
#define FTS_REG_FACTORY_MODE_DETACH_FLAG    0xB4
#define FTS_REG_FACE_DEC_MODE_STATUS        0x01
#define FTS_REG_IDE_PARA_VER_ID             0xB5
#define FTS_REG_IDE_PARA_STATUS             0xB6
#define FTS_REG_GLOVE_MODE_EN               0xC0
#define FTS_REG_COVER_MODE_EN               0xC1
#define FTS_REG_PALM_EN                     0xC5
#define FTS_REG_CHARGER_MODE_EN             0x8B
#define FTS_REG_EDGE_MODE_EN                0x8C
#define FTS_REG_GESTURE_EN                  0xD0
#define FTS_REG_GESTURE_OUTPUT_ADDRESS      0xD3
#define FTS_REG_MODULE_ID                   0xE3
#define FTS_REG_LIC_VER                     0xE4
#define FTS_REG_ESD_SATURATE                0xED
#define FTS_REG_GESTURE_SWITCH              0xCF
#define FTS_REG_MONITOR_CTRL                0x86
#define FTS_REG_SENSE_ONOFF                 0xEA
#define FTS_REG_IRQ_ONOFF                   0xEB
#define FTS_REG_CLR_RESET                   0xEC

#define FTS_REG_HEATMAP_1E                  0x1E
#define FTS_REG_HEATMAP_ED                  0xED
#define FTS_REG_HEATMAP_9E                  0x9E

#define FTS_LPTW_REG_SET_E1                 0xE1
#define FTS_LPTW_REG_SET_E2                 0xE2
#define FTS_STTW_REG_SET_E3                 0xE3
#define FTS_GESTURE_MAJOR_MINOR             0xE5
#define FTS_REG_CONTINUOUS_EN               0xE7

#define FTS_REG_CUSTOMER_STATUS             0xB2    // follow FTS_CUSTOMER_STATUS.
                                                    // bit 0~1 : HOPPING
                                                    // bit 2   : PALM
                                                    // bit 3   : WATER
                                                    // bit 4   : GRIP
                                                    // bit 5   : GLOVE
                                                    // bit 6   : STTW
                                                    // bit 7   : LPWG
#define FTS_CAP_DATA_LEN                    91
#define FTS_SELF_DATA_LEN                   68
#define FTS_FULL_TOUCH_RAW_SIZE(tx_num, rx_num) \
    (FTS_CAP_DATA_LEN + ((tx_num) * (rx_num) + FTS_SELF_DATA_LEN * 2) * sizeof(u16))
#define FTS_PRESSURE_SCALE                  85      // 255 / 3
#define FTS_CUSTOMER_STATUS_LEN             4
#define FTS_CUSTOMER_STATUS1_MASK           0x0F
#define FTS_ORIENTATION_SCALE               45
#define FTS_GESTURE_ID_STTW                 0x25
#define FTS_GESTURE_ID_LPTW_DOWN            0x26
#define FTS_GESTURE_ID_LPTW_UP              0x27

#define FTS_SYSFS_ECHO_ON(buf)      (buf[0] == '1')
#define FTS_SYSFS_ECHO_OFF(buf)     (buf[0] == '0')

#define kfree_safe(pbuf) do {\
    if (pbuf) {\
        kfree(pbuf);\
        pbuf = NULL;\
    }\
} while(0)

/*****************************************************************************
*  Alternative mode (When something goes wrong, the modules may be able to solve the problem.)
*****************************************************************************/
/*
 * point report check
 * default: disable
 */
#define FTS_POINT_REPORT_CHECK_EN               0

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct ft_chip_t {
    u16 type;
    u8 chip_idh;
    u8 chip_idl;
    u8 rom_idh;
    u8 rom_idl;
    u8 pb_idh;
    u8 pb_idl;
    u8 bl_idh;
    u8 bl_idl;
};

struct ft_chip_id_t {
    u16 type;
    u16 chip_ids[FTS_MAX_CHIP_IDS];
};

struct ts_ic_info {
    bool is_incell;
    bool hid_supported;
    struct ft_chip_t ids;
    struct ft_chip_id_t cid;
};

#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
enum {
    FTS_TS_BUS_REF_SCREEN_ON    = 0x01,
    FTS_TS_BUS_REF_IRQ          = 0x02,
    FTS_TS_BUS_REF_FW_UPDATE    = 0x04,
    FTS_TS_BUS_REF_SYSFS        = 0x0008,
    FTS_TS_BUS_REF_FORCE_ACTIVE = 0x0010,
    FTS_TS_BUS_REF_BUGREPORT    = 0x0020,
};

enum TOUCH_POWER_MODE {
    FTS_TS_STATE_POWER_ON = 0,
    FTS_TS_STATE_SUSPEND,
};
#endif

/* Firmware Grip suppression mode.
 * 0 - Disable fw grip suppression.
 * 1 - Enable fw grip suppression.
 * 2 - Force disable fw grip suppression.
 * 3 - Force enable fw grip suppression.
 */
enum FW_GRIP_MODE {
    FW_GRIP_DISABLE,
    FW_GRIP_ENABLE,
    FW_GRIP_FORCE_DISABLE,
    FW_GRIP_FORCE_ENABLE,
};

/* Firmware Heatmap mode.
 * 0 - Disable fw heatmap.
 * 1 - Enable fw compressed heatmap.
 * 2 - Enable fw uncompressed heatmap.
 */
enum FW_HEATMAP_MODE {
    FW_HEATMAP_MODE_DISABLE,
    FW_HEATMAP_MODE_COMPRESSED,
    FW_HEATMAP_MODE_UNCOMPRESSED,
};

/* Firmware Palm rejection mode.
 * 0 - Disable fw palm rejection.
 * 1 - Enable fw palm rejection.
 * 2 - Force disable fw palm rejection.
 * 3 - Force enable fw palm rejection.
 */
enum FW_PALM_MODE {
    FW_PALM_DISABLE,
    FW_PALM_ENABLE,
    FW_PALM_FORCE_DISABLE,
    FW_PALM_FORCE_ENABLE,
};

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
enum TBN_OWRER {
    TBN_AP,
    TBN_AOC,
};
#endif

/*****************************************************************************
* DEBUG function define here
*****************************************************************************/
#undef pr_fmt
#define pr_fmt(fmt) "gtd: FTS_TS: " fmt
#if FTS_DEBUG_EN
#define FTS_DEBUG(fmt, ...) pr_info(fmt, ##__VA_ARGS__)
#define FTS_FUNC_ENTER() pr_debug("%s: Enter\n", __func__)
#define FTS_FUNC_EXIT() pr_debug("%s: Exit(%d)\n", __func__, __LINE__)
#else /* #if FTS_DEBUG_EN*/
#define FTS_DEBUG(fmt, ...)
#define FTS_FUNC_ENTER()
#define FTS_FUNC_EXIT()
#endif

#define FTS_INFO(fmt, ...) pr_info(fmt, ##__VA_ARGS__)
#define FTS_ERROR(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#define PR_LOGD(fmt, ...) pr_debug(fmt, ##__VA_ARGS__)

#endif /* __LINUX_FOCALTECH_COMMON_H__ */
