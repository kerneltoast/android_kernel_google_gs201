/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2021, Google, LLC. All rights reserved. */

#ifndef __S2MPG_REGULATOR_H
#define __S2MPG_REGULATOR_H

/* SEC_OPMODE_OFF       Regulator always OFF */
#define SEC_OPMODE_OFF		0
/*
 * SEC_OPMODE_SUSPEND   Regulator is changed by PWREN pin
 *                      If PWREN is high, regulator is on
 *                      If PWREN is low, regulator is off
 */
#define SEC_OPMODE_SUSPEND	1

/* SEC_OPMODE_LOWPOWER  Regulator is on in low-power mode */
#define SEC_OPMODE_LOWPOWER	2

/* SEC_OPMODE_ON        Regulator always ON */
#define SEC_OPMODE_ON 3

#define SEC_OPMODE_TCXO 2

#define SEC_OPMODE_MIF 2

#endif /* __S2MPG_REGULATOR_H */
