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
	if (len == 0)
		return ERR_PTR(-EINVAL);

	/* name should not contain the last line feed character */
	if (buf[len - 1] == '\n')
		len -= 1;

	name = kmemdup_nul(buf, len, GFP_KERNEL);
	if (!name)
		return ERR_PTR(-ENOMEM);

	return name;
}
