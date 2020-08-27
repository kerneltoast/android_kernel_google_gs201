/*
  * fts.h
  *
  * FTS Capacitive touch screen controller (FingerTipS)
  *
  * Copyright (C) 2017, STMicroelectronics
  * Authors: AMG(Analog Mems Group)
  *
  *		marco.cali@st.com
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
  * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
  * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
  * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
  * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM
  * THE
  * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
  * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
  */

/*!
  * \file fts.h
  * \brief Contains all the definitions and structs used generally by the driver
  */

#ifndef _LINUX_FTS_I2C_H_
#define _LINUX_FTS_I2C_H_

#include <linux/device.h>
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
#include <linux/input/heatmap.h>
#endif
#include <linux/pm_qos.h>
#include <drm/drm_panel.h>
#include "fts_lib/ftsSoftware.h"
#include "fts_lib/ftsHardware.h"

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
#include <linux/input/touch_bus_negotiator.h>
#endif

#undef DYNAMIC_REFRESH_RATE
#undef CONFIG_DRM

/****************** CONFIGURATION SECTION ******************/
/** @defgroup conf_section	 Driver Configuration Section
  * Settings of the driver code in order to suit the HW set up and the
  *application behavior
  * @{
  */
/* **** CODE CONFIGURATION **** */
#define FTS_TS_DRV_NAME		"fts"	/* driver name */
#define FTS_TS_DRV_VERSION	"5.2.16.14"	/* driver version string
							 * */
#define FTS_TS_DRV_VER		0x0502100E	/* driver version u32 format */

/* #define DEBUG */	/* /< define to print more logs in the kernel log
			 * and better follow the code flow */
#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(fmt) "[ FTS ] " fmt
#endif

#define PINCTRL_STATE_ACTIVE    "pmx_ts_active"
#define PINCTRL_STATE_SUSPEND   "pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE   "pmx_ts_release"

#define DRIVER_TEST	/* /< if defined allow to use and test special functions
			  * of the driver and fts_lib from command shell
			  * (useful for enginering/debug operations) */

/* If both COMPUTE_INIT_METHOD and PRE_SAVED_METHOD are not defined,
 * driver will be automatically configured as GOLDEN_VALUE_METHOD
 */
/*#define COMPUTE_INIT_METHOD   Allow to compute init data on phone during
			      * production
			      */
#define SKIP_PRODUCTION_TEST /* Allow to skip Production test */

//#ifndef COMPUTE_INIT_METHOD
/*#define PRE_SAVED_METHOD  Pre-Saved Method used
			  * during production
			  */
//#endif

/*#define FW_H_FILE*/			/* include the FW data as header file */
#ifdef FW_H_FILE
#define FW_SIZE_NAME	myArray_size	/* FW data array size */
#define FW_ARRAY_NAME	myArray	/* FW data array name */
/*#define FW_UPDATE_ON_PROBE*/		/* No delay updating FW */
#endif

#ifndef FW_UPDATE_ON_PROBE
/* Include the Production Limit File as header file, can be commented to use a
  * .csv file instead */
/* #define LIMITS_H_FILE */
#ifdef LIMITS_H_FILE
	#define LIMITS_SIZE_NAME	myArray2_size	/* /< name of the
							 * variable
							  * in the limits header
							  *file which
							  * specified the
							  *dimension of
							  * the limits data
							  *array */
	#define LIMITS_ARRAY_NAME	myArray2	/* /< name of the
							 * variable in
							  * the limits header
							  *file which
							  * specified the limits
							  *data array */
#endif
#else
/* if execute fw update in the probe the limit file must be a .h */
#define LIMITS_H_FILE	/* /< include the Production Limit File as header file,
			 * DO NOT COMMENT! */
#define LIMITS_SIZE_NAME		myArray2_size	/* /< name of the
							 * variable
							  * in the limits header
							  *file
							  * which specified the
							  *dimension
							  * of the limits data
							  *array */
#define LIMITS_ARRAY_NAME		myArray2	/* /< name of the
							 * variable in the
							  * limits header file
							  *which specified
							  * the limits data
							  *array */
#endif

/* #define USE_ONE_FILE_NODE */	/* /< allow to enable/disable all the features
  * just using one file node */

#ifndef FW_UPDATE_ON_PROBE
#define EXP_FN_WORK_DELAY_MS 1000	/* /< time in ms elapsed after the probe
					  * to start the work which execute FW
					  *update
					  * and the Initialization of the IC */
#endif

/* **** END **** */


/* **** FEATURES USED IN THE IC **** */
/* Enable the support of keys */
/* #define PHONE_KEY */

#define GESTURE_MODE	/* /< enable the support of the gestures */
#ifdef GESTURE_MODE
	#define USE_GESTURE_MASK	/* /< the gestures to select are
					 * referred using
					  * a gesture bitmask instead of their
					  *gesture IDs */
#endif


#define CHARGER_MODE	/* /< enable the support to charger mode feature
			 * (comment to disable) */

#define GLOVE_MODE	/* /< enable the support to glove mode feature (comment
			 * to disable) */

#define COVER_MODE	/* /< enable the support to cover mode feature (comment
			 * to disable) */

#define STYLUS_MODE	/* /< enable the support to stylus mode feature (comment
			 * to disable) */

#define GRIP_MODE	/* /< enable the support to grip mode feature (comment
			 * to disable) */


/* **** END **** */


/* **** PANEL SPECIFICATION **** */
#define X_AXIS_MIN	0	/* /< min X coordinate of the display */
#define Y_AXIS_MIN	0	/* /< min Y coordinate of the display */
#define Y_AXIS_MAX	2959	/* /< Max Y coordinate of the display */
#define X_AXIS_MAX	1440	/* /< Max X coordinate of the display */

#define PRESSURE_MIN	0	/* /< min value of pressure reported */
#define PRESSURE_MAX	127	/* /< Max value of pressure reported */

#define DISTANCE_MIN	0	/* /< min distance between the tool and the
				 * display */
#define DISTANCE_MAX	127	/* /< Max distance between the tool and the
				 * display */

#define TOUCH_ID_MAX	10	/* /< Max number of simoultaneous touches
				 * reported */

#define AREA_MIN	PRESSURE_MIN	/* /< min value of Major/minor axis
					 * reported */
#define AREA_MAX	PRESSURE_MAX	/* /< Man value of Major/minor axis
					 * reported */
/* **** END **** */

/* #define SKIP_PRESSURE */

/**@}*/
/*********************************************************/
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
/* **** LOCAL HEATMAP FEATURE *** */
#define LOCAL_HEATMAP_WIDTH 7
#define LOCAL_HEATMAP_HEIGHT 7
#define LOCAL_HEATMAP_MODE 0xC1

struct heatmap_report {
	uint8_t prefix; /* always should be 0xA0 */
	uint8_t mode; /* mode should be 0xC1 for heatmap */

	uint16_t counter; /* LE order, should increment on each heatmap read */
	int8_t offset_x;
	uint8_t size_x;
	int8_t offset_y;
	uint8_t size_y;
	/* data is in LE order; order should be enforced after data is read */
	strength_t data[LOCAL_HEATMAP_WIDTH * LOCAL_HEATMAP_HEIGHT];
} __attribute__((packed));
/* **** END **** */
#endif
/*
  * Configuration mode
  *
  * bitmask which can assume the value defined as features in ftsSoftware.h or
  * the following values
  */

/** @defgroup mode_section	 IC Status Mode
  * Bitmask which keeps track of the features and working mode enabled in the
  * IC.
  * The meaning of the the LSB of the bitmask must be interpreted considering
  * that the value defined in @link feat_opt Feature Selection Option @endlink
  * correspond to the position of the corresponding bit in the mask
  * @{
  */
#define MODE_NOTHING 0x00000000	/* /< nothing enabled (sense off) */
#define MODE_ACTIVE(_mask, _sett)	\
	(_mask |= (SCAN_MODE_ACTIVE << 24) | (_sett << 16))
/* /< store the status of scan mode active and its setting */
#define MODE_LOW_POWER(_mask, _sett)	\
	(_mask |= (SCAN_MODE_LOW_POWER << 24) | (_sett << 16))
/* /< store the status of scan mode low power and its setting */
#define IS_POWER_MODE(_mask, _mode)	((_mask&(_mode<<24)) != 0x00)
/* /< check the current mode of the IC */

/** @}*/

#define CMD_STR_LEN	32	/* /< max number of parameters that can accept
				 * the
				  * MP file node (stm_fts_cmd) */

#define TSP_BUF_SIZE	PAGE_SIZE	/* /< max number of bytes printable on
					  * the shell in the normal file nodes
					  **/

/* Encapsulate display extinfo
 *
 * For some panels, it is insufficient to simply detect the panel ID and load
 * one corresponding firmware. The display driver exposes extended info read
 * from the display, but it is up to the touch driver to parse the data.
 */
struct fts_disp_extinfo {
	bool is_read;
	u8 size;
	u8 *data;
};

/**
  * Struct which contains information about the HW platform and set up
  */
struct fts_hw_platform_data {
	int (*power) (bool on);
	int switch_gpio;/* (optional) I2C switch */
	int irq_gpio;	/* /< number of the gpio associated to the interrupt pin
			 * */
	int reset_gpio;	/* /< number of the gpio associated to the reset pin */
	int disp_rate_gpio; /* disp_rate gpio: LOW=60Hz, HIGH=90Hz */
	const char *vdd_reg_name;	/* /< name of the VDD regulator */
	const char *avdd_reg_name;	/* /< name of the AVDD regulator */
	const char *fw_name;
	const char *limits_name;
	bool sensor_inverted;
	int x_axis_max;
	int y_axis_max;
	bool auto_fw_update;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	bool heatmap_mode_full_init;
#endif
	struct drm_panel *panel;
	u32 initial_panel_index;
	u32 *force_pi_cfg_ver;
};

/* Bits for the bus reference mask */
enum {
	FTS_BUS_REF_SCREEN_ON		= 0x01,
	FTS_BUS_REF_IRQ			= 0x02,
	FTS_BUS_REF_FW_UPDATE		= 0x04,
	FTS_BUS_REF_SYSFS		= 0x08,
	FTS_BUS_REF_FORCE_ACTIVE	= 0x10,
};

/* Motion filter finite state machine (FSM) states
 * FTS_MF_FILTERED        - default coordinate filtering
 * FTS_MF_UNFILTERED      - unfiltered single-touch coordinates
 * FTS_MF_FILTERED_LOCKED - filtered coordinates. Locked until touch is lifted.
 */
typedef enum {
	FTS_MF_FILTERED		= 0,
	FTS_MF_UNFILTERED	= 1,
	FTS_MF_FILTERED_LOCKED	= 2
} motion_filter_state_t;

/* Heatmap mode selection
 * FTS_HEATMAP_OFF	- no data read
 * FTS_HEATMAP_PARTIAL	- read partial frame
 *			(LOCAL_HEATMAP_WIDTH * LOCAL_HEATMAP_HEIGHT)
 * FTS_HEATMAP_FULL	- read full mutual sense strength frame
 */
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
enum {
	FTS_HEATMAP_OFF		= 0,
	FTS_HEATMAP_PARTIAL	= 1,
	FTS_HEATMAP_FULL	= 2
};
#endif
/*
  * Forward declaration
  */
struct fts_ts_info;

/*
  * Dispatch event handler
  * Return true if the handler has processed a pointer event
  */
typedef bool (*event_dispatch_handler_t)
	(struct fts_ts_info *info, unsigned char *data);

/**
  * Driver touch simulation details
  */
struct fts_touchsim{
	/* touch simulation coordinates */
	int x, y, x_step, y_step;

	/* timer to run the touch simulation code */
	struct hrtimer hr_timer;

	struct work_struct work;
	struct workqueue_struct *wq;

	/* True if the touch simulation is currently running */
	bool is_running;
};

/**
  * FTS capacitive touch screen device information
  * - dev             Pointer to the structure device \n
  * - client          client structure \n
  * - input_dev       Input device structure \n
  * - work            Work thread \n
  * - event_wq        Event queue for work thread \n
  * - event_dispatch_table  Event dispatch table handlers \n
  * - attrs           SysFS attributes \n
  * - mode            Device operating mode (bitmask) \n
  * - touch_id        Bitmask for touch id (mapped to input slots) \n
  * - stylus_id       Bitmask for tracking the stylus touches (mapped using the
  *                   touchId) \n
  * - timer           Timer when operating in polling mode \n
  * - power           Power on/off routine \n
  * - board           HW info retrieved from device tree \n
  * - vdd_reg         DVDD power regulator \n
  * - avdd_reg        AVDD power regulator \n
  * - resume_bit      Indicate if screen off/on \n
  * - fwupdate_stat   Store the result of a fw update triggered by the host \n
  * - notifier        Used for be notified from a suspend/resume event \n
  * - sensor_sleep    true suspend was called, false resume was called \n
  * - wakesrc         Wakeup Source struct \n
  * - input_report_mutex  mutex for handling the pressure of keys \n
  * - series_of_switches  to store the enabling status of a particular feature
  *                       from the host \n
  * - tbn             Touch Bus Negotiator context
  */
struct fts_ts_info {
	struct device           *dev;	/* Pointer to the device */
#ifdef I2C_INTERFACE
	struct i2c_client       *client;	/* I2C client structure */
#else
	struct spi_device       *client;	/* SPI client structure */
#endif
	struct input_dev        *input_dev;	/* Input device structure */

	struct work_struct suspend_work;	/* Suspend work thread */
	struct work_struct resume_work;	/* Resume work thread */
	struct workqueue_struct *event_wq;	/* Used for event handler, */
						/* suspend, resume threads */

	struct completion bus_resumed;		/* resume_work complete */

	struct pm_qos_request pm_qos_req;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	struct v4l2_heatmap v4l2;
#endif
	struct delayed_work fwu_work;	/* Work for fw update */
	struct workqueue_struct *fwu_workqueue;	/* Fw update work queue */
	event_dispatch_handler_t *event_dispatch_table;	/* Dispatch table */

	struct attribute_group attrs;	/* SysFS attributes */

	unsigned int mode;	/* Device operating mode */
				/* MSB - active or lpm */
	unsigned long touch_id;	/* Bitmask for touch id */
	unsigned long palm_touch_mask; /* Bitmask for palm touch */
	unsigned long grip_touch_mask; /* Bitmask for grip touch */
#ifdef STYLUS_MODE
	unsigned long stylus_id;	/* Bitmask for the stylus */
#endif

	ktime_t timestamp; /* time that the event was first received from the
		touch IC, acquired during hard interrupt, in CLOCK_MONOTONIC */

	struct fts_hw_platform_data     *board;	/* HW info from device tree */
	struct regulator        *vdd_reg;	/* DVDD power regulator */
	struct regulator        *avdd_reg;	/* AVDD power regulator */

	struct pinctrl       *ts_pinctrl;		/* touch pin control state holder */
	struct pinctrl_state *pinctrl_state_active;	/* Active pin state*/
	struct pinctrl_state *pinctrl_state_suspend;	/* Suspend pin state*/
	struct pinctrl_state *pinctrl_state_release;	/* Release pin state*/

	spinlock_t fts_int;	/* Spinlock to protect interrupt toggling */
	bool irq_enabled;	/* Interrupt state */

	struct mutex bus_mutex;	/* Protect access to the bus */
	unsigned int bus_refmask; /* References to the bus */

	int resume_bit;	/* Indicate if screen off/on */
	int fwupdate_stat;	/* Result of a fw update */
	int reflash_fw;	/* Attempt to reflash fw */
	int autotune_stat;	/* Attempt to autotune */

	struct fts_disp_extinfo extinfo;	/* Display extended info */

#ifdef CONFIG_DRM
	struct notifier_block notifier;	/* Notify on suspend/resume */
#endif
#ifdef DYNAMIC_REFRESH_RATE
	int display_refresh_rate;	/* Display rate in Hz */
#endif
	bool sensor_sleep;		/* True if suspend called */
	struct wakeup_source *wakesrc;	/* Wake Lock struct */

	/* input lock */
	struct mutex input_report_mutex;	/* Mutex for pressure report */

	/* switches for features */
	int gesture_enabled;	/* Gesture during suspend */
	int glove_enabled;	/* Glove mode */
	int charger_enabled;	/* Charger mode */
	int stylus_enabled;	/* Stylus mode */
	int cover_enabled;	/* Cover mode */
	int grip_enabled;	/* Grip mode */
#if IS_ENABLED(CONFIG_TOUCHSCREEN_HEATMAP)
	int heatmap_mode;	/* heatmap mode*/
#endif
	/* Stop changing motion filter and keep fw design */
	bool use_default_mf;
	/* Motion filter finite state machine (FSM) state */
	motion_filter_state_t mf_state;
	/* Time of initial single-finger touch down. This timestamp is used to
	 * compute the duration a single finger is touched before it is lifted.
	 */
	ktime_t mf_downtime;

#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN)
	struct tbn_context	*tbn;
#endif

	/* Allow only one thread to execute diag command code*/
	struct mutex diag_cmd_lock;
	/* Allow one process to open procfs node */
	bool diag_node_open;

	/* Touch simulation details */
	struct fts_touchsim touchsim;

	u8 scanning_frequency;

	/* Preallocated i/o read buffer */
	u8 io_read_buf[READ_CHUNK + DUMMY_FIFO];
	/* Preallocated i/o write buffer */
	u8 io_write_buf[WRITE_CHUNK + BITS_64 + DUMMY_FIFO];
	/* Preallocated i/o extra write buffer */
	u8 io_extra_write_buf[WRITE_CHUNK + BITS_64 + DUMMY_FIFO];

};

/* DSI display function used to read panel extinfo */
int dsi_panel_read_vendor_extinfo(struct drm_panel *panel, char *buffer,
				  size_t len);

int fts_chip_powercycle(struct fts_ts_info *info);
extern int input_register_notifier_client(struct notifier_block *nb);
extern int input_unregister_notifier_client(struct notifier_block *nb);

/* export declaration of functions in fts_proc.c */
extern int fts_proc_init(void);
extern int fts_proc_remove(void);

/* Bus reference tracking */
int fts_set_bus_ref(struct fts_ts_info *info, u16 ref, bool enable);

#endif
