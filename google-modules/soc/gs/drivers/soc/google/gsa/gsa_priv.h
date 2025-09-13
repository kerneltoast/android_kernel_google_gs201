/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __LINUX_GSA_PRIV_H
#define __LINUX_GSA_PRIV_H

#include <linux/device.h>
#include <linux/types.h>

int gsa_send_cmd(struct device *dev, u32 cmd,
		 u32 *req, u32 req_argc,
		 u32 *rsp, u32 rsp_argc);

int gsa_send_simple_cmd(struct device *dev, u32 cmd);

int gsa_send_one_arg_cmd(struct device *dev, u32 cmd, u32 arg);

#endif /* __LINUX_GSA_PRIV_H */

