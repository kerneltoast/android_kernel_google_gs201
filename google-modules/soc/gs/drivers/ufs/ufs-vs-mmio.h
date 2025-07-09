/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2021 Google LLC
 */

#ifndef _UFS_VS_MMIO_H
#define _UFS_VS_MMIO_H

#if defined(CONFIG_SOC_GS101)
#include "gs101/ufs-vs-mmio.h"
#elif defined(CONFIG_SOC_GS201)
#include "gs201/ufs-vs-mmio.h"
#elif defined(CONFIG_SOC_ZUMA)
#include "zuma/ufs-vs-mmio.h"
#endif

#endif /*_UFS_VS_MMIO_H */
