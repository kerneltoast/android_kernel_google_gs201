/* SPDX-License-Identifier: GPL-2.0 */

#ifndef TOUCHSCREEN_BUS_NEGOTIATOR_H
#define TOUCHSCREEN_BUS_NEGOTIATOR_H

#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kthread.h>

#define TBN_DEVICE_NAME "tbn"
#define TBN_CLASS_NAME "tbn"

#define TBN_REQUEST_BUS_TIMEOUT_MS 500
#define TBN_RELEASE_BUS_TIMEOUT_MS 500

enum tbn_mode {
	TBN_MODE_DISABLED = 0,
	TBN_MODE_GPIO,
	TBN_MODE_AOC_CHANNEL,
	TBN_MODE_MOCK,
};

enum tbn_bus_owner {
	TBN_BUS_OWNER_AP = 0,
	TBN_BUS_OWNER_AOC = 1,
};

enum TbnOperation : __u32 {
	TBN_OPERATION_IDLE = 0,
	TBN_OPERATION_AP_RELEASE_BUS,
	TBN_OPERATION_AP_REQUEST_BUS,
	TBN_OPERATION_AOC_RESET,
	TBN_OPERATION_AOC_SEND_LPTW_EVENT,
};

struct TbnEvent {
	__u32 id;
	enum TbnOperation operation;
} __packed;

struct TbnEventHeader {
    __u32 id;
    __s32 err;
    enum TbnOperation operation;
    __u8 data[52];
} __packed;

struct TbnEventResponse {
	__u32 id;
	__s32 err;
	enum TbnOperation operation;
	bool lptw_triggered;
} __packed;

struct TbnLptwEvent {
	__u32 id;
	__s32 err;
	enum TbnOperation operation;
	u16 x;
	u16 y;
	u16 major;
	u16 minor;
	s16 angle;
} __packed;

struct tbn_context {
	struct device *dev;
	struct completion bus_requested;
	struct completion bus_released;
	struct mutex dev_mask_mutex;
	u32 mode;
	u32 max_devices;
	u32 registered_mask;
	u32 requested_dev_mask;
	int aoc2ap_gpio;
	int ap2aoc_gpio;
	int aoc2ap_irq;
	struct task_struct *aoc_channel_task;

	/* event management */
	struct TbnEventResponse event_resp;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_TBN_AOC_CHANNEL_MODE)
	struct TbnEvent event;
	struct mutex event_lock;
	struct work_struct aoc_reset_work;
	struct workqueue_struct *event_wq;
#endif

	void (*lptw_event_cb)(struct TbnLptwEvent* lptw, void* user_data);
	void* lptw_event_cbdata;
};

int register_tbn(u32 *output);
void register_tbn_lptw_callback(void* callback, void* cbdata);
void unregister_tbn(u32 *output);
int tbn_request_bus_with_result(u32 dev_mask, bool *lptw_triggered);
int tbn_request_bus(u32 dev_mask);
int tbn_release_bus(u32 dev_mask);


#endif /* TOUCHSCREEN_BUS_NEGOTIATOR_H */
