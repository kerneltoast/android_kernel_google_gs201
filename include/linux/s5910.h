// SPDX-License-Identifier: GPL-2.0-only

#ifndef _S5910_SPMI__
#define _S5910_SPMI__

struct device *s5910_get_device(struct device_node *node);
struct spmi_device  *s5910_get_spmi_device(struct device_node *node);
int s5910_shutdown_sequence(struct device *dev);
int s5910_turn_on_sequence(struct device *dev);
int s5910_check_lpm_mode(struct device *dev);

#endif
