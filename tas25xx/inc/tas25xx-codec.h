/*
 * =============================================================================
 * Copyright (c) 2016  Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.See the GNU General Public License for more details.
 *
 * File:
 *     tas25xx-codec.h
 *
 * Description:
 *     header file for tas25xx-codec.c
 *
 * =============================================================================
 */

#ifndef _TAS25XX_CODEC_H
#define _TAS25XX_CODEC_H

#include "tas25xx.h"

int tas25xx_register_codec(struct tas25xx_priv *p_tas25xx);
int tas25xx_deregister_codec(struct tas25xx_priv *p_tas25xx);

#endif /* _TAS25XX_CODEC_H */
