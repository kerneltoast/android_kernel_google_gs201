/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Fingerprint Touch Handler
 */

#include <linux/types.h>

#ifndef _UAPI_FTH_HANDLER_H_
#define _UAPI_FTH_HANDLER_H_

#define FTH_IOCTL_SEND_KEY_EVENT             101
#define FTH_IOCTL_ENABLE_LPTW_EVENT_REPORT   102
#define FTH_IOCTL_DISABLE_LPTW_EVENT_REPORT  103
#define FTH_IOCTL_ACQUIRE_WAKELOCK           107
#define FTH_IOCTL_RELEASE_WAKELOCK           108
#define FTH_IOCTL_GET_TOUCH_FD_VERSION       109
//#define FTH_IOCTL_CONFIGURE_TOUCH_FD_V3    112
//#define FTH_IOCTL_CONFIGURE_TOUCH_FD_V4    113
//#define FTH_IOCTL_CONFIGURE_TOUCH_FD_V5    114
#define FTH_IOCTL_CONFIGURE_TOUCH_FD_V6      115

#define FTH_TOUCH_FD_VERSION_6 6

#define FTH_MAX_FD_EVENTS 128

#define FTH_MAX_FINGERS 10

#define FTH_LPTW_FINGER_SLOT -1

/*
 * enum fth_finger_events -
 *      enumeration of fth finger events
 * @FTH_EVENT_FINGER_UP - finger up detected
 * @FTH_EVENT_FINGER_DOWN - finger down detected
 * @FTH_EVENT_FINGER_MOVE - finger move detected
 */
enum fth_finger_events {
	FTH_EVENT_FINGER_UP,
	FTH_EVENT_FINGER_DOWN,
	FTH_EVENT_FINGER_MOVE
};

/*
 * struct fth_touch_event_v6 -
 *		used to send fd event
 */
struct fth_touch_event_v6 {
	__s64 time_us;
	__u16 X[FTH_MAX_FINGERS];
	__u16 Y[FTH_MAX_FINGERS];
	__s32 major;
	__s32 minor;
	__s32 orientation;
	__s32 slot;
	__s32 state;	// 0 = up, 1 = down, 2 = move.
	__s32 num_fingers;	// number of fingers
	_Bool touch_valid;
	_Bool updated[FTH_MAX_FINGERS];
};

/*
 * struct fth_fd_buf -
 *		used to send fd buf
 */
struct fth_fd_buf {
	__u32 num_events;
	struct fth_touch_event_v6 fd_events[FTH_MAX_FD_EVENTS];
};

/*
 * struct fth_key_event -
 *		used to send key event
 * @key - the key event to send
 * @value - value of the key event
 */
struct fth_key_event {
	__s32 key;
	__s32 value;
};

/*
 * struct fth_touch_fd_version -
 *		used to get touch finger detect version
 * @version: version number
 */
struct fth_touch_fd_version {
	__s32 version;
};

/*
 * struct fth_touch_config_v6 -
 *		used to configure touch finger detect
 * @version - touch FD version
 * @touch_fd_enable - flag to enable/disable touch finger detect
 * @rad_filter_enable - flag to enable/disable radius based filtering
 * @left - x-coordinate of top left corner of AOI
 * @top - y-coordinate of top left corner of AOI
 * @right - x-coordinate of bottom right corner of AOI
 * @bottom - y--coordinate of bottom right corner of AOI
 * @rad_x: movement radius in x direction
 * @rad_y: movement radius in y direction
 */
struct fth_touch_config_v6 {
	struct fth_touch_fd_version version;
	_Bool touch_fd_enable;
	_Bool rad_filter_enable;
	__s32 left;
	__s32 top;
	__s32 right;
	__s32 bottom;
	__s32 rad_x;
	__s32 rad_y;
};

#endif /* _UAPI_FTH_HANDLER_H_ */
