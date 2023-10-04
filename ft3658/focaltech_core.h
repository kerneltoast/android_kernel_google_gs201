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
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
#include <touch_offload.h>
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
#include <heatmap.h>
#endif

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_MAX_POINTS_SUPPORT              10 /* constant value, can't be changed */
#define FTS_MAX_KEYS                        4
#define FTS_KEY_DIM                         10
#define FTS_ONE_TCH_LEN                     6
#define FTS_TOUCH_DATA_LEN  (FTS_MAX_POINTS_SUPPORT * FTS_ONE_TCH_LEN + 3)

#define FTS_GESTURE_POINTS_MAX              1
#define FTS_GESTURE_DATA_LEN                14
#define FTS_GESTURE_TOTAL_DATA_SIZE         (FTS_GESTURE_POINTS_MAX * FTS_GESTURE_DATA_LEN)

#define FTS_MAX_ID                          0x0A
#define FTS_TOUCH_X_H_POS                   3
#define FTS_TOUCH_X_L_POS                   4
#define FTS_TOUCH_Y_H_POS                   5
#define FTS_TOUCH_Y_L_POS                   6
#define FTS_TOUCH_PRE_POS                   7
#define FTS_TOUCH_AREA_POS                  8
#define FTS_TOUCH_POINT_NUM                 2
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
    u32 irq_gpio_flags;
    u32 reset_gpio;
    u32 reset_gpio_flags;
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
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    u32 offload_id;
#endif
    u32 tx_ch_num;
    u32 rx_ch_num;
    /* convert mm to pixel for major and minor */
    u8 mm2px;
};

struct ts_event {
    int x;      /*x coordinate */
    int y;      /*y coordinate */
    int p;      /* pressure */
    int flag;   /* touch event flag: 0 -- down; 1-- up; 2 -- contact */
    int id;     /*touch ID */
    int area;
    int major;
    int minor;
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

/* Motion filter finite state machine (FSM) states
 * MF_FILTERED        - default coordinate filtering
 * MF_UNFILTERED      - unfiltered single-touch coordinates
 * MF_FILTERED_LOCKED - filtered coordinates. Locked until touch is lifted.
 */
typedef enum {
    MF_FILTERED,
    MF_UNFILTERED,
    MF_FILTERED_LOCKED,
} motion_filter_state_t;

/* Motion filter mode.
 *  MF_OFF    : 0 = Always unfilter.
 *  MF_DYNAMIC: 1 = Dynamic change motion filter.
 *  MF_ON     : 2 = Always filter by touch FW.
 */
enum MF_MODE {
    MF_OFF,
    MF_DYNAMIC,
    MF_ON,
};

/*
* gesture_id    - mean which gesture is recognised
* point_num     - points number of this gesture
* coordinate_x  - All gesture point x coordinate
* coordinate_y  - All gesture point y coordinate
* major         - All gesture major value
* minor         - All gesture minor value
* orientation   - All gesture orientation value
*/
struct fts_gesture_st {
    u8 gesture_id;
    u8 point_num;
    int coordinate_x[FTS_GESTURE_POINTS_MAX];
    int coordinate_y[FTS_GESTURE_POINTS_MAX];
    int major[FTS_GESTURE_POINTS_MAX];
    int minor[FTS_GESTURE_POINTS_MAX];
    int orientation[FTS_GESTURE_POINTS_MAX];
};

enum SS_TYPE {
    SS_NORMAL,
    SS_WATER,
};

struct fts_ts_data {
    struct i2c_client *client;
    struct spi_device *spi;
    struct device *dev;
    struct input_dev *input_dev;
    struct input_dev *pen_dev;
    struct fts_ts_platform_data *pdata;
    struct ts_ic_info ic_info;
    struct workqueue_struct *ts_workqueue;
    struct work_struct fwupg_work;
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
    u8 current_host_status[FTS_CUSTOMER_STATUS_LEN];
#endif

    /* Motion filter mode.
     *  MF_OFF    : 0 = Always unfilter.
     *  MF_DYNAMIC: 1 = Dynamic change motion filter.
     *  MF_ON     : 2 = Always filter by touch FW.
     */
    u8 mf_mode;
    /* Payload for continuously report. */
    u8 set_continuously_report;
    /* Motion filter finite state machine (FSM) state */
    motion_filter_state_t mf_state;
    /* Time of initial single-finger touch down. This timestamp is used to
     * compute the duration a single finger is touched before it is lifted.
     */
    ktime_t mf_downtime;
    ktime_t bugreport_ktime_start;
    u8 work_mode;

#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    u8 fw_heatmap_mode;
    u8 fw_default_heatmap_mode;
    int compress_heatmap_wlen;
#endif
    u8 enable_fw_grip;
    u8 enable_fw_palm;
    ktime_t isr_timestamp; /* Time that the event was first received from the
                        * touch IC, acquired during hard interrupt, in
                        * CLOCK_MONOTONIC */
    ktime_t coords_timestamp;
    bool is_deepsleep;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD) || \
    IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    u8 *heatmap_raw;
    u16 heatmap_raw_size;
    u8 *trans_raw;
    u16 trans_raw_size;
    u16 *heatmap_buff;
    u16 heatmap_buff_size;
    u8 self_sensing_type;
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
    struct touch_offload_context offload;
    struct touch_offload_frame *reserved_frame;
    u8 touch_offload_active_coords;
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
    struct v4l2_heatmap v4l2;
#endif
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
#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
    volatile int power_status;
    u16 bus_refmask;
    struct mutex bus_mutex;
    struct drm_bridge panel_bridge;
    struct drm_connector *connector;
    bool is_panel_lp_mode;
    int display_refresh_rate;
#endif
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
    u32 tbn_register_mask;
    u8  tbn_owner;
#endif
};

enum FTS_BUS_TYPE {
    FTS_BUS_TYPE_NONE,
    FTS_BUS_TYPE_I2C,
    FTS_BUS_TYPE_SPI,
    FTS_BUS_TYPE_SPI_V2,
};

#if GOOGLE_REPORT_MODE
enum FTS_CUSTOMER_STATUS {
    STATUS_BASELINE_REFRESH_B0,
    STATUS_BASELINE_REFRESH_B1,
    STATUS_PALM,
    STATUS_WATER,
    STATUS_GRIP,
    STATUS_GLOVE,
    STATUS_EDGE_PALM,
    STATUS_RESET,
    STATUS_CNT_END,
};

enum FTS_FW_MODE_SETTING{
    FW_GLOVE = 0,
    FW_GRIP,
    FW_PALM,
    FW_HEATMAP,
    FW_CONTINUOUS,
    FW_CNT_END,
};
#endif

enum FTS_SCAN_MODE {
    MODE_AUTO,
    MODE_NORMAL_ACTIVE,
    MODE_NORMAL_IDLE,
    MODE_LOW_POWER_ACTIVE,
    MODE_LOW_POWER_IDLE,
    MODE_CNT,
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

/* Gesture functions */
int fts_gesture_init(struct fts_ts_data *ts_data);
int fts_gesture_exit(struct fts_ts_data *ts_data);
void fts_gesture_recovery(struct fts_ts_data *ts_data);
int fts_gesture_readdata(struct fts_ts_data *ts_data, bool is_report);
int fts_gesture_suspend(struct fts_ts_data *ts_data);
int fts_gesture_resume(struct fts_ts_data *ts_data);

/* Heatmap and Offload*/
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD) || \
    IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
int fts_get_default_heatmap_mode(struct fts_ts_data *ts_data);
int fts_set_heatmap_mode(struct fts_ts_data *ts_data, u8 heatmap_mode);
#endif
int fts_set_grip_mode(struct fts_ts_data *ts_datam, u8 grip_mode);
int fts_set_palm_mode(struct fts_ts_data *ts_data, u8 palm_mode);
int fts_set_continuous_mode(struct fts_ts_data *ts_data, bool en);
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

#if IS_ENABLED(CONFIG_TOUCHSCREEN_PANEL_BRIDGE)
/* Bus reference tracking */
int fts_ts_set_bus_ref(struct fts_ts_data *ts, u16 ref, bool enable);
#endif

#endif /* __LINUX_FOCALTECH_CORE_H__ */
