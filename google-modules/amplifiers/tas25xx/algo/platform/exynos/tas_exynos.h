/*
 **=============================================================================
 **Copyright (c) 2016  Texas Instruments Inc.
 **
 **This program is free software; you can redistribute it and/or modify it under
 **the terms of the GNU General Public License as published by the Free Software
 **Foundation; version 2.
 **
 **This program is distributed in the hope that it will be useful, but WITHOUT
 **ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 **FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details
 **
 **File:
 **	tas25xx-algo-bin-utils.c
 **
 **Description:
 **	Common algo related header.
 **
 **=============================================================================
 */
#ifndef __TAS_EXYNOS__
#define __TAS_EXYNOS__

#define TAS_PAYLOAD_SIZE	14

struct ti_smartpa_data {
	uint32_t payload[TAS_PAYLOAD_SIZE];
};

int ti_smartpa_read(void *prm_data, int offset, int size);
int ti_smartpa_write(void *prm_data, int offset, int size);

#endif /*__TAS_EXYNOS__*/
