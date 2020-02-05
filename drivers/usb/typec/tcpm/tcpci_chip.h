// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019, Google Inc
 *
 * CHIP TCPCI HEADER
 */

#ifndef __CHIP_TCPCI__
#define __CHIP_TCPCI__
extern int tcpc_get_vbus_voltage_max_mv(struct i2c_client *tcpc_client);
extern int tcpc_set_vbus_voltage_max_mv(struct i2c_client *tcpc_client,
					unsigned int mv);
extern int tcpc_get_vbus_voltage_mv(struct i2c_client *tcpc_client);
extern void tcpc_set_port_data_capable(struct i2c_client *tcpc_client,
				       enum power_supply_usb_type
				       usb_type);
#endif /*__CHIP_TCPCI__*/
