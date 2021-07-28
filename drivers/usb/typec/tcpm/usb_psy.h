// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019, Google LLC
 *
 * USB input current management.
 *
 */
#ifndef __USB_PSY__
#define __USB_PSY__
#include <misc/logbuffer.h>
#include <linux/power_supply.h>

struct usb_psy_ops {
	int (*tcpc_get_vbus_voltage_max_mv)(struct i2c_client *tcpc_client);
	int (*tcpc_set_vbus_voltage_max_mv)(struct i2c_client *tcpc_client,
					    unsigned int mv);
	int (*tcpc_get_vbus_voltage_mv)(struct i2c_client *tcpc_client);
	void (*tcpc_set_port_data_capable)(struct i2c_client *tcpc_client,
					   enum power_supply_usb_type
					   usb_type);
};

void usb_psy_set_sink_state(void *usb_psy, bool enabled);
void usb_psy_set_attached_state(void *usb_psy, bool attached);
void *usb_psy_setup(struct i2c_client *client, struct logbuffer *log,
		    struct usb_psy_ops *ops);
void usb_psy_teardown(void *usb_data);
void usb_psy_start_sdp_timeout(void *usb_psy);
#endif
