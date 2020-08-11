/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Samsung Electronics.
 *
 */

#ifndef __MODEM_CTRL_H__
#define __MODEM_CTRL_H__

#define MIF_INIT_TIMEOUT	(15 * HZ)

void modem_ctrl_set_kerneltime(struct modem_ctl *mc);
int modem_ctrl_check_offset_data(struct modem_ctl *mc);
void change_modem_state(struct modem_ctl *mc, enum modem_state state);

#endif /* __MODEM_CTRL_H__ */
