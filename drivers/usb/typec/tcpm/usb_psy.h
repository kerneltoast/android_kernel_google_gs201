// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019, Google LLC
 *
 * USB input current management.
 *
 */
#ifndef __USB_PSY__
#define __USB_PSY__
void *usb_psy_setup(struct i2c_client *client,
		    struct logbuffer *log);
void usb_psy_teardown(void *usb_data);
#endif
