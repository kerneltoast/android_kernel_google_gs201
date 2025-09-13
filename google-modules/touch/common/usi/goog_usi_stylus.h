/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google USI Stylus Support for Pixel devices.
 *
 * Copyright 2024 Google LLC.
 */

#ifndef _GOOG_USI_STYLUS_
#define _GOOG_USI_STYLUS_

#include <linux/input.h>

/**
 * typedef g_usi_handle_t - handle for a client to use Google USI subsystem API
 *
 * A client(e.g. vendor specific driver) registers itself as a client of
 * the Google USI subsystem by calling goog_usi_register() and it gets new handle.
 */
typedef void *g_usi_handle_t;
#define G_USI_INVALID_HANDLE		NULL

/**
 * enum G_USI_STYLUS_INFO - bitmap Flags indicate the specific stylus information available
 *                          from the paired(or lastly paired) stylus.
 *
 * The flags can be retrieved by calling goog_usi_get_stylus_info_flags()
 * The vendor driver can use the flags to see which stylus information is
 * available in the Google USI subsystem.
 *
 * G_USI_GID_FLAG          - set when GID is collected by goog_usi_report_gid()
 * G_USI_BATTERY_FLAG      - set when battery level is collected by goog_usi_report_battery()
 * G_USI_CAPABILITY_FLAG   - set when capability is collected by goog_usi_report_capability()
 * G_USI_FW_VERSION_FLAG   - set when FW version is collected by goog_usi_report_fw_version()
 * G_USI_USI_VERSION_FLAG  - set when USI version is collected by goog_usi_report_usi_version()
 * G_USI_VENDOR_EXT_FLAG   - set when VENDOR_EXT is collected by goog_usi_report_vendor_extension()
 * G_USI_PAIRING_INFO_FLAG - set when pariring information is collected by goog_usi_update_state()
 */
enum G_USI_STYLUS_INFO {
	G_USI_GID_FLAG          = 1U << 0, /* C.GetGID() */
	G_USI_BATTERY_FLAG      = 1U << 1, /* C.GetBattery() */
	G_USI_CAPABILITY_FLAG   = 1U << 2, /* C.GetCapability() */
	G_USI_FW_VERSION_FLAG   = 1U << 3, /* C.GetFirmwareVersion() */
	G_USI_USI_VERSION_FLAG	= 1U << 4, /* C.GetUsiVersion() */
	G_USI_VENDOR_EXT_FLAG	= 1U << 5, /* C.GetVendorExtension() */
	G_USI_PAIRING_INFO_FLAG	= 1U << 6
};

/**
 * enum G_USI_STATUS - stylus status used by goog_usi_update_status()
 *
 * Let the subsystem know the current stylus status.
 *
 * G_USI_UPDATE_NORMAL_PAIRING_DONE should provide the pairing information using g_usi_pairing_info
 * G_USI_UPDATE_FAST_PAIRING_DONE may provide the pairing information using g_usi_pairing_info
 */
enum G_USI_STATUS {
	G_USI_UPDATE_NORMAL_PAIRING_DONE	= 1, /* mandatory parameter - g_usi_pairing_info */
	G_USI_UPDATE_FAST_PAIRING_DONE		= 2,
	G_USI_UPDATE_CRC_FAILURE		= 3
};

/**
 * struct g_usi_pairing_info - pairing information
 *
 * @hash_id: stylus hash ID
 * @session_id: session_id assigned to the stylus by USI controller
 * @freq_sel: downlink frequency assigned to the stylus by USI controller
 * @additional_data: vendor-specific additional data that helps stylus performance
 */
struct g_usi_pairing_info {
	u8 hash_id[2];
	u8 session_id[2];
	u8 freq_sel;
	void *additional_data;
};

/**
 * struct g_usi_callbacks - callbacks to ask the vendor driver to do the vendor-specific action.
 *
 * @g_usi_send_uplink: It is called to send an uplink command to the paired stylus.
 *                     The subsystem calls this to cover the host communication defined
 *                     in chapter 7.3.3 in USI 2.0 spec.
 *
 *                     e.g.) HID Set Color/Width/Style/Button, HID Set/Get Diagnostic command
 *
 *                     It is not used for any HID GET except the Diagnostic command.
 *
 *                     For the details, please refer to chapter 7.3.3 Feature Reports in
 *                     USI Stylus and Device Technical specification 2.0.
 *
 * @g_usi_invalidate_stylus_info: It is called to invalidate the stylus information
 *                                maintained in the subsystem and ask the fresh one
 *                                from the paired stylus.
 */
struct g_usi_callbacks {
	/*
	 * sends 33-bits uplink in "beacon" and fill "response" with the response from the stylus
	 *
	 * On return, response[0 ~ 1] contains the 16 bits of data from the
	 * stylus and response[2] has EIA and error code, 0 if no error.
	 */
	int (*g_usi_send_uplink)(g_usi_handle_t handle, u8 beacon[5], u8 response[3]);
};

/**
 * struct g_usi - USI device information to register
 *
 * @dev: USI controller device
 * @bustype: USI controller bustype(BUS_SPI, BUS_I2C, ... refer to input.h for available bug types
 * @abs_x_max: maximum of x axis
 * @abs_y_max: maximum of y axis
 * @distance_max: maximum distance. should be 0 if distance is not supported by the USI controller
 * @tilt_min: minimum tilt(x and y)
 * @tilt_max: maximum tilt(x and y). should be 0 if tilt is not supported by the USI controller
 * @is_flex_beacon: 1(true) if the USI controller is using flex beacon, 0 for standard beacon
 * @cbs: callback functions for the subsystem to ask vendor driver to do actions
 */
struct g_usi {
	struct device *dev;
	u16 bustype;	/* BUS_SPI, BUS_I2C, ... see input.h for available bus type */

	/* H/W description */
	int abs_x_max;
	int abs_y_max;
	int distance_max;	/* 0 means distance is not supported */
	int tilt_min;
	int tilt_max;		/* 0 means tilt is not supported */

	bool is_flex_beacon;	/* true if the controller uses flex beacon */
	struct g_usi_callbacks *cbs;
};

/**
 * struct g_usi_event - USI motion event
 *
 * @timestamp: timestamp should be captured in a hard ISR context
 * @x: ABS_X
 * @y: ABS_Y
 * @pressure: ABS_PRESSURE
 * @tilt_X: ABS_TILT_X
 * @tilt_y: ABA_TILT_Y
 * @distance: ABS_DISTANCE
 * @buttons: BIT0: BTN_STYLUS, BIT1:BTN_STYLUS2, BIT3: BTN_TOOL_RUBBER
 *
 * The vendor driver provides a stylus motion event via goog_usi_send_event()
 */
struct g_usi_event {
	ktime_t timestamp; /* This should be captured in Hard ISR context */

	u32 x;		/* ABS_X */
	u32 y;		/* ABS_Y */
	u32 pressure;	/* ABS_PRESSURE */
	s32 tilt_x;	/* ABA_TILT_X */
	s32 tilt_y;	/* ABA_TILT_Y */
	u32 distance;	/* ABS_DISTANCE */
	u32 buttons;	/* BIT0: BTN_STYLUS, BIT1:BTN_STYLUS2, BIT3: BTN_TOOL_RUBBER */
};

/**
 * struct g_usi_event_leave - USI motion event leave
 *
 * @timestamp: timestamp should be captured in a hard ISR context
 *
 * The vendor driver provides a stylus leave event via goog_usi_send_event_leave()
 */
struct g_usi_event_leave {
	ktime_t timestamp; /* This should be captured in hardware IRS context */
};

/* register/unregister a USI device */
g_usi_handle_t goog_usi_register(struct g_usi *usi);
void goog_usi_unregister(g_usi_handle_t handle);

/* send motion event and leave event */
int goog_usi_send_event(g_usi_handle_t handle, struct g_usi_event *usi_event);
int goog_usi_send_event_leave(g_usi_handle_t handle, struct g_usi_event_leave *usi_event_leave);

/* save/restore the vendor-specific data */
int goog_usi_set_drvdata(g_usi_handle_t handle, void *data);
int goog_usi_get_drvdata(g_usi_handle_t handle, void **data);

/* save/restore stylus information to/from the driver */
/* Set GID0-GID5 from C.GetGID() */
int goog_usi_report_gid(g_usi_handle_t handle, const u8 *gid);
/* Set firmware version from G.GetFirmwareVersion() */
int goog_usi_report_fw_version(g_usi_handle_t handle, const u8 *fw_ver);
int goog_usi_get_fw_version(g_usi_handle_t handle, u8 *fw_ver);
/* Set USI version from C.GetUsiVersion() */
int goog_usi_report_usi_version(g_usi_handle_t handle, const u8 *usi_ver);
int goog_usi_get_usi_version(g_usi_handle_t handle, u8 *usi_ver);
/* Set vendor extension from C.GetVendorExtension() */
int goog_usi_report_vendor_extension(g_usi_handle_t handle, const u8 *vendor_ext);
int goog_usi_get_vendor_extension(g_usi_handle_t handle, u8 *vendor_ext);
/* Set capability from C.GetCapability() */
int goog_usi_report_capability(g_usi_handle_t handle, const u8 *cap);
/* Set battery capacity and low battery flag from C.GetBattery() */
int goog_usi_report_battery(g_usi_handle_t handle, const u8 *bat);
int goog_usi_get_battery(g_usi_handle_t handle, u8 *bat);
/* Get serial number of the stylus from GID collected via goog_usi_report_gid() */
int goog_usi_get_serial_number(g_usi_handle_t handle, u32 *sn_high, u32 *sn_low);
int goog_usi_get_serial_number_str(g_usi_handle_t handle, char *buf, int buf_size);
/* Get vendor ID&product ID of the stylus from GID collected via goog_usi_report_gid() */
int goog_usi_get_vid_pid(g_usi_handle_t handle, u16 *vid, u16 *pid);
/* Update current status */
int goog_usi_update_status(g_usi_handle_t handle, int type, void *data);
int goog_usi_get_pairing_info(g_usi_handle_t handle, struct g_usi_pairing_info *pairing_info);
int goog_usi_get_stylus_info_flags(g_usi_handle_t handle, u32 *info_flags);

#endif /* _GOOG_USI_STYLUS_ */
