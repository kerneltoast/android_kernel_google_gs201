/* SPDX-License-Identifier: GPL-2.0
 * Copyright (C) 2013 ST Microelectronics S.A.
 * Copyright (C) 2009 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 */

#ifndef __ST21NFC_H
#define __ST21NFC_H

#define ST21NFC_MAGIC	0xEA

#define ST21NFC_NAME "st21nfc"
/*
 * ST21NFC power control via ioctl
 * ST21NFC_GET_WAKEUP :  poll gpio-level for Wakeup pin
 */
#define ST21NFC_GET_WAKEUP	      _IOR(ST21NFC_MAGIC, 0x01, unsigned int)
#define ST21NFC_PULSE_RESET		_IOR(ST21NFC_MAGIC, 0x02, unsigned int)
#define ST21NFC_SET_POLARITY_RISING   _IOR(ST21NFC_MAGIC, 0x03, unsigned int)
#define ST21NFC_SET_POLARITY_HIGH     _IOR(ST21NFC_MAGIC, 0x05, unsigned int)
#define ST21NFC_GET_POLARITY	      _IOR(ST21NFC_MAGIC, 0x07, unsigned int)
#define ST21NFC_RECOVERY              _IOR(ST21NFC_MAGIC, 0x08, unsigned int)
#define ST21NFC_CLK_ENABLE            _IOR(ST21NFC_MAGIC, 0x11, unsigned int)
#define ST21NFC_CLK_DISABLE           _IOR(ST21NFC_MAGIC, 0x12, unsigned int)
#define ST21NFC_CLK_STATE             _IOR(ST21NFC_MAGIC, 0x13, unsigned int)

#endif
