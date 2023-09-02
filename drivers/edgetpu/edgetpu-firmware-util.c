// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU firmware utilities.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "edgetpu-firmware-util.h"

char *edgetpu_fwutil_name_from_attr_buf(const char *buf)
{
	size_t len;
	char *name;

	len = strlen(buf);
	/* buf from sysfs attribute contains the last line feed character */
	if (len == 0 || buf[len - 1] != '\n')
		return ERR_PTR(-EINVAL);

	name = kstrdup(buf, GFP_KERNEL);
	if (!name)
		return ERR_PTR(-ENOMEM);
	/* name should not contain the last line feed character */
	name[len - 1] = '\0';
	return name;
}
