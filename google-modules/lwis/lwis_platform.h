/*
 * Google LWIS Declarations of Platform-specific Functions
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_PLATFORM_H_
#define LWIS_PLATFORM_H_

#include "lwis_device.h"

/*
 *  lwis_platform_probe: handles platform-specific parts of
 *  device init
 */
int lwis_platform_probe(struct lwis_device *lwis_dev);

/*
 *  lwis_platform_device_enable: handles platform-specific parts of
 *  device enable
 */
int lwis_platform_device_enable(struct lwis_device *lwis_dev);

/*
 *  lwis_platform_device_disable: handles platform-specific parts of
 *  device disable
 */
int lwis_platform_device_disable(struct lwis_device *lwis_dev);

/*
 *  lwis_platform_update_qos: handles platform-specific parts of
 *  updating qos requirements. "value" is in KHz.
 */
int lwis_platform_update_qos(struct lwis_device *lwis_dev, int value, int32_t clock_family);

/*
 *  lwis_platform_remove_qos: handles platform-specific parts of
 *  removing qos requirements.
 */
int lwis_platform_remove_qos(struct lwis_device *lwis_dev);

/*
 *  lwis_platform_update_bts: handles platform-specific parts of
 *  updating bts requirement.
 */
int lwis_platform_update_bts(struct lwis_device *lwis_dev, unsigned int bw_kb_peak,
			     unsigned int bw_kb_read, unsigned int bw_kb_write,
			     unsigned int bw_kb_rt);

/*
 *  lwis_plaform_set_default_irq_affinity: handles platform-specific parts of
 *  setting default irq affinity.
 */
int lwis_plaform_set_default_irq_affinity(unsigned int irq);

#endif /* LWIS_PLATFORM_H_ */
