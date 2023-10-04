/* SPDX-License-Identifier: GPL-2.0 */

#ifndef TOUCHSCREEN_BUS_NEGOTIATOR_H
#define TOUCHSCREEN_BUS_NEGOTIATOR_H

#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/device.h>

#define TBN_REQUEST_BUS_TIMEOUT_MS 500
#define TBN_RELEASE_BUS_TIMEOUT_MS 500

enum tbn_mode {
	TBN_MODE_DISABLED,
	TBN_MODE_GPIO,
	TBN_MODE_MAILBOX,
};

enum tbn_bus_owner {
	TBN_BUS_OWNER_AP = 0,
	TBN_BUS_OWNER_AOC = 1,
};

struct tbn_context {
	struct device *dev;
	struct completion bus_requested;
	struct completion bus_released;
	struct mutex dev_mask_mutex;
	u8 mode;
	u32 max_devices;
	u32 registered_mask;
	u32 requested_dev_mask;
	int aoc2ap_gpio;
	int ap2aoc_gpio;
	int aoc2ap_irq;
};

int register_tbn(u32 *output);
void unregister_tbn(u32 *output);
int tbn_request_bus(u32 dev_mask);
int tbn_release_bus(u32 dev_mask);


#endif /* TOUCHSCREEN_BUS_NEGOTIATOR_H */
