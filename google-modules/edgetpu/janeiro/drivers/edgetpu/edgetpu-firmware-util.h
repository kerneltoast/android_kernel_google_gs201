/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Implements utility functions for firmware management of EdgeTPU.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_FIRMWARE_UTIL_H__
#define __EDGETPU_FIRMWARE_UTIL_H__

/*
 * Parses firmware name from attribute buffer. User should free this buffer
 * after use.
 *
 * The name from attribute buffer should end with a line feed.
 *
 * returns -ENOMEM on memory allocation failure, -EINVAL on invalid buf format.
 */
char *edgetpu_fwutil_name_from_attr_buf(const char *attr_buf);

#endif /* __EDGETPU_FIRMWARE_UTIL_H__ */
