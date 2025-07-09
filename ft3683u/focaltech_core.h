/*
 *
 * FocalTech TouchScreen driver.
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
* File Name: focaltech_core.h

* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_FOCALTECH_CORE_H__
#define __LINUX_FOCALTECH_CORE_H__
/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/pm_qos.h>
#include "focaltech_common.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_MAX_POINTS_SUPPORT              10 /* constant value, can't be changed */
#define FTS_MAX_KEYS                        4
#define FTS_KEY_DIM                         10
#define FTS_ONE_TCH_LEN                     6
#define FTS_TOUCH_DATA_LEN  (FTS_MAX_POINTS_SUPPORT * FTS_ONE_TCH_LEN + 3)

#define FTS_MAX_ID                          0x0A
#define FTS_TOUCH_X_H_POS                   3
#define FTS_TOUCH_X_L_POS                   4
#define FTS_TOUCH_Y_H_POS                   5
#define FTS_TOUCH_Y_L_POS                   6
#define FTS_TOUCH_PRE_POS                   7
#define FTS_TOUCH_AREA_POS                  8
#define FTS_TOUCH_POINT_NUM                 1
#define FTS_TOUCH_EVENT_POS                 3
#define FTS_TOUCH_ID_POS                    5
#define FTS_COORDS_ARR_SIZE                 4
#define FTS_X_MIN_DISPLAY_DEFAULT           0
#define FTS_Y_MIN_DISPLAY_DEFAULT           0
#define FTS_X_MAX_DISPLAY_DEFAULT           720
#define FTS_Y_MAX_DISPLAY_DEFAULT           1280

#define FTS_TOUCH_DOWN                      0
#define FTS_TOUCH_UP                        1
#define FTS_TOUCH_CONTACT                   2
#define EVENT_DOWN(flag)                    ((FTS_TOUCH_DOWN == flag) || (FTS_TOUCH_CONTACT == flag))
#define EVENT_UP(flag)                      (FTS_TOUCH_UP == flag)
#define EVENT_NO_DOWN(data)                 (!data->point_num)

#define FTS_MAX_COMPATIBLE_TYPE             4
#define FTS_MAX_COMMMAND_LENGTH             16


#define FTS_TOUCH_OFF_E_XH                  0
#define FTS_TOUCH_OFF_XL                    1
#define FTS_TOUCH_OFF_ID_YH                 2
#define FTS_TOUCH_OFF_YL                    3
#define FTS_TOUCH_OFF_PRE                   4
#define FTS_TOUCH_OFF_MAJOR                 5
#define FTS_TOUCH_OFF_MINOR                 6
#define FTS_TOUCH_OFF_ORIENTATION           7

#define FTS_TOUCH_E_NUM                     1
#define FTS_ONE_TCH_LEN_V2                  8
#define FTS_TOUCH_DATA_LEN_V2  (FTS_MAX_POINTS_SUPPORT * FTS_ONE_TCH_LEN_V2 + 4)
#define FTS_HI_RES_X_MAX                    16
#define FTS_TOUCH_HIRES_X                   10

#define FTS_TOUCH_HIRES_EN                  1

#if FTS_TOUCH_HIRES_EN
#define FTS_TOUCH_HIRES(x) ((x) * FTS_TOUCH_HIRES_X / FTS_HI_RES_X_MAX)
#else
#define FTS_TOUCH_HIRES(x) ((x) / FTS_HI_RES_X_MAX)
#endif // FTS_TOUCH_HIRES_EN

#define FTS_PEN_HIRES_EN                    1
#define FTS_PEN_HIRES_X                     10





/*****************************************************************************
*  Alternative mode (When something goes wrong, the modules may be able to solve the problem.)
*****************************************************************************/
/*
 * For commnication error in PM(deep sleep) state
 */
#define FTS_PATCH_COMERR_PM                     0
#define FTS_TIMEOUT_COMERR_PM                   700

#define FTS_HIGH_REPORT                         0
#define FTS_SIZE_DEFAULT                        15


/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct ftxxxx_proc {
    struct proc_dir_entry *proc_entry;
    u8 opmode;
    u8 cmd_len;
    u8 cmd[FTS_MAX_COMMMAND_LENGTH];
};

struct fts_ts_platform_data {
    u32 irq_gpio;
    u32 reset_gpio;
    struct drm_panel *panel;
    u32 initial_panel_index;
    bool have_key;
    u32 key_number;
    u32 keys[FTS_MAX_KEYS];
    u32 key_y_coords[FTS_MAX_KEYS];
    u32 key_x_coords[FTS_MAX_KEYS];
    u32 x_max;
    u32 y_max;
    u32 x_min;
    u32 y_min;
    u32 max_touch_number;
    u32 tx_ch_num;
    u32 rx_ch_num;
    /* convert mm to pixel for major and minor */
    u8 mm2px;
    char fw_name[FILE_NAME_LENGTH];
    char test_limits_name[FILE_NAME_LENGTH];
    int panel_id;
};

struct ts_event {
    int x;      /*x coordinate */
    int y;      /*y coordinate */
    int p;      /* pressure */
    int flag;   /* touch event flag: 0 -- down; 1-- up; 2 -- contact */
    int id;     /*touch ID */
    int major;
    int minor;
    int orientation;
};

struct pen_event {
    int inrange;
    int tip;
    int x;      /*x coordinate */
    int y;      /*y coordinate */
    int p;      /* pressure */
    int flag;   /* touch event flag: 0 -- down; 1-- up; 2 -- contact */
    int id;     /*touch ID */
    int tilt_x;
    int tilt_y;
    int tool_type;
};

struct fts_gesture_st {
    union {
        struct {
            u8 gesture_id;
            u8 coordinate_x_msb;
            u8 coordinate_x_lsb;
            u8 coordinate_y_msb;
            u8 coordinate_y_lsb;
            u8 orientation;
            u8 major;
            u8 minor;
            u8 gesture_enable;
            u8 point_id;
            u8 point_num;
            u8 FOD_area;
            u8 touch_area;
            u8 even;
        } __attribute__((packed));
        u8 data[14];
    };
};

struct fw_status_ts {
  union {
    struct {
      unsigned char B0_b0_abnormal_reset:3;
      unsigned char B0_b3_water_state:1;
      unsigned char B0_b4_grip_status:1;
      unsigned char B0_b5_palm_status:1;
      unsigned char B0_b6_edge_palm_status:1;
      unsigned char B0_b7_reserved:1;

      unsigned char B1_b0_baseline:3;
      unsigned char B1_b3_noise_status:3;
      unsigned char B1_b6_INT2_status:1;
      unsigned char B1_b7_continuous_status:1;

      unsigned char B2_b0_frequency_hopping:3;
      unsigned char B2_b3_v_sync_status:1;
      unsigned char B2_b4_reserved:4;

      unsigned char B3_b0_glove_reg:1;
      unsigned char B3_b1_grip_reg:1;
      unsigned char B3_b2_palm_reg:1;
      unsigned char B3_b3_reserved:1;
      unsigned char B3_b4_continus_reg:1;
      unsigned char B3_b5_reserved:1;
      unsigned char B3_b6_heatmap_status:2;
    } __attribute__((packed));
    unsigned char data[4];
  };
};



enum SS_TYPE {
    SS_NORMAL,
    SS_WATER,
};

struct fts_ts_data {
    struct i2c_client *client;
    struct spi_device *spi;
    u32 spi_speed;
    struct device *dev;
    struct input_dev *input_dev;
    struct input_dev *pen_dev;
    struct fts_ts_platform_data *pdata;
    struct ts_ic_info ic_info;
    struct workqueue_struct *ts_workqueue;
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
    struct delayed_work fwupg_work;
#else
    struct work_struct fwupg_work;
#endif // IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
    struct delayed_work esdcheck_work;
    struct delayed_work prc_work;
    struct work_struct resume_work;
    struct work_struct suspend_work;
    struct pm_qos_request pm_qos_req;
    struct ftxxxx_proc proc;
    spinlock_t irq_lock;
    struct mutex report_mutex;
    struct mutex bus_lock;
    struct mutex reg_lock;
    struct mutex device_mutex;
    struct completion bus_resumed;
    struct fts_gesture_st fts_gesture_data;
    unsigned long intr_jiffies;
    int irq;
    int log_level;
    int fw_is_running;      /* confirm fw is running when using spi:default 0 */
    int dummy_byte;
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
    struct completion pm_completion;
    bool pm_suspend;
#endif
    bool suspended;
    bool fw_loading;
    bool irq_disabled;
    bool power_disabled;
    bool glove_mode;
    bool cover_mode;
    bool charger_mode;
    bool gesture_mode;      /* gesture enable or disable, default: disable */
    bool prc_mode;
    bool driver_probed;
    struct pen_event pevent;
    /* multi-touch */
    struct ts_event *events;
    u8 *bus_tx_buf;
    u8 *bus_rx_buf;
    int bus_type;
    u8 *point_buf;
    int pnt_buf_size;
    int touchs;
    int key_state;
    int touch_point;
    int point_num;

#if GOOGLE_REPORT_MODE
    struct fw_status_ts current_host_status;
#endif

    u8 work_mode;

    u8 enable_fw_grip;
    u8 enable_fw_palm;
    ktime_t isr_timestamp; /* Time that the event was first received from the
                        * touch IC, acquired during hard interrupt, in
                        * CLOCK_MONOTONIC */
    ktime_t coords_timestamp;
    bool is_deepsleep;
    struct proc_dir_entry *proc_touch_entry;
    struct regulator *avdd;
    struct regulator *dvdd;
#if FTS_PINCTRL_EN
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_active;
    struct pinctrl_state *pins_suspend;
#endif
#if defined(CONFIG_FB) || defined(CONFIG_DRM)
    struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
    struct goog_touch_interface *gti;
#endif // IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
};

enum FTS_BUS_TYPE {
    FTS_BUS_TYPE_NONE,
    FTS_BUS_TYPE_I2C,
    FTS_BUS_TYPE_SPI,
    FTS_BUS_TYPE_SPI_V2,
};

enum _FTS_TOUCH_ETYPE {
    TOUCH_DEFAULT = 0x00,
    TOUCH_PROTOCOL_v2 = 0x02,
    TOUCH_EXTRA_MSG = 0x08,
    TOUCH_PEN = 0x0B,
    TOUCH_GESTURE = 0x80,
    TOUCH_FW_INIT = 0x81,
    TOUCH_DEFAULT_HI_RES = 0x82,
    TOUCH_IGNORE = 0xFE,
    TOUCH_ERROR = 0xFF,
};


/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern struct fts_ts_data *fts_data;

/* communication interface */
int fts_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen);
int fts_read_reg(u8 addr, u8 *value);
int fts_write(u8 *writebuf, u32 writelen);
int fts_write_reg(u8 addr, u8 value);
void fts_hid2std(void);
int fts_bus_init(struct fts_ts_data *ts_data);
int fts_bus_exit(struct fts_ts_data *ts_data);
int fts_spi_transfer_direct(u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen);
int fts_bus_set_speed(struct fts_ts_data *ts_data, u32 speed);


/* Gesture functions */
int fts_gesture_init(struct fts_ts_data *ts_data);
int fts_gesture_exit(struct fts_ts_data *ts_data);
int fts_gesture_readdata(struct fts_ts_data *ts_data);

int fts_set_heatmap_mode(struct fts_ts_data *ts_data, u8 heatmap_mode);
int fts_set_grip_mode(struct fts_ts_data *ts_datam, u8 grip_mode);
int fts_set_palm_mode(struct fts_ts_data *ts_data, u8 palm_mode);
int fts_set_glove_mode(struct fts_ts_data *ts_data, bool en);

/* Apk and functions */
int fts_create_apk_debug_channel(struct fts_ts_data *);
void fts_release_apk_debug_channel(struct fts_ts_data *);

/* ADB functions */
int fts_create_sysfs(struct fts_ts_data *ts_data);
int fts_remove_sysfs(struct fts_ts_data *ts_data);

/* ESD */
#if FTS_ESDCHECK_EN
int fts_esdcheck_init(struct fts_ts_data *ts_data);
int fts_esdcheck_exit(struct fts_ts_data *ts_data);
int fts_esdcheck_switch(bool enable);
int fts_esdcheck_proc_busy(bool proc_debug);
int fts_esdcheck_set_intr(bool intr);
int fts_esdcheck_suspend(void);
int fts_esdcheck_resume(void);
#endif

/* Production test */
#if FTS_TEST_EN
int fts_test_init(struct fts_ts_data *ts_data);
int fts_test_exit(struct fts_ts_data *ts_data);
#endif

/* Point Report Check*/
#if FTS_POINT_REPORT_CHECK_EN
int fts_point_report_check_init(struct fts_ts_data *ts_data);
int fts_point_report_check_exit(struct fts_ts_data *ts_data);
void fts_prc_queue_work(struct fts_ts_data *ts_data);
#endif

/* FW upgrade */
int fts_fwupg_init(struct fts_ts_data *ts_data);
int fts_fwupg_exit(struct fts_ts_data *ts_data);
int fts_upgrade_bin(char *fw_name, bool force);
int fts_enter_test_environment(bool test_state);

/* Other */
int fts_reset_proc(int hdelayms);
int fts_check_cid(struct fts_ts_data *ts_data, u8 id_h);
int fts_wait_tp_to_valid(void);
void fts_release_all_finger(void);
void fts_tp_state_recovery(struct fts_ts_data *ts_data);
int fts_ex_mode_init(struct fts_ts_data *ts_data);
int fts_ex_mode_exit(struct fts_ts_data *ts_data);
int fts_ex_mode_recovery(struct fts_ts_data *ts_data);
void fts_update_feature_setting(struct fts_ts_data *ts_data);
void fts_irq_disable(void);
void fts_irq_enable(void);

/* Power Control */
#if FTS_PINCTRL_EN
int fts_pinctrl_select_normal(struct fts_ts_data *ts);
int fts_pinctrl_select_suspend(struct fts_ts_data *ts);
#endif /* FTS_PINCTRL_EN */

#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)
void fts_irq_read_report(void);

struct gti_scan_cmd;
struct gti_palm_cmd;
struct gti_screen_protector_mode_cmd;
int gti_set_scan_mode(void *private_data, struct gti_scan_cmd *cmd);
int gti_get_scan_mode(void *private_data, struct gti_scan_cmd *cmd);
int gti_set_palm_mode(void *private_data, struct gti_palm_cmd *cmd);
int gti_get_palm_mode(void *private_data, struct gti_palm_cmd *cmd);
int gti_set_screen_protector_mode(void *private_data,
    struct gti_screen_protector_mode_cmd *cmd);
int gti_get_screen_protector_mode(void *private_data,
    struct gti_screen_protector_mode_cmd *cmd);

char *goog_get_test_limit_name(void);
void goog_gti_probe(struct fts_ts_data *ts_data);
void goog_gti_remove(struct fts_ts_data *ts_data);
void goog_fts_input_report_b(struct fts_ts_data *data);
int goog_parse_dt(struct device_node *np, struct fts_ts_platform_data *pdata);
#endif // IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE)


#endif /* __LINUX_FOCALTECH_CORE_H__ */
