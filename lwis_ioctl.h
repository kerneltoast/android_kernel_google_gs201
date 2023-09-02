/*
 * Google LWIS IOCTL Handler
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_IOCTL_H_
#define LWIS_IOCTL_H_

#include "lwis_device.h"

/*
 *  lwis_ioctl_handler: Handle all IOCTL commands via the file descriptor.
 */
int lwis_ioctl_handler(struct lwis_client *lwis_client, unsigned int type, unsigned long param);

#endif /* LWIS_IOCTL_H_ */
