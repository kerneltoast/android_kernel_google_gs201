/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021, Google LLC
 *
 * MAX77759 EXPORT FUNCTIONS
 */
#ifndef __MAX77759_EXPORT_H__
#define __MAX77759_EXPORT_H__

int tcpm_get_partner_src_caps(struct tcpm_port *port, u32 **src_pdo);
void tcpm_put_partner_src_caps(u32 **src_pdo);

#endif  // __MAX77759_EXPORT_H__
