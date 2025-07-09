/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2021 Google LLC
 */

#ifndef _UFS_CAL_IF_H
#define _UFS_CAL_IF_H

#if defined(CONFIG_SOC_GS101)
#include "gs101/ufs-cal-if.h"
#elif defined(CONFIG_SOC_GS201)
#include "gs201/ufs-cal-if.h"
#elif defined(CONFIG_SOC_ZUMA)
#include "zuma/ufs-cal-if.h"
#endif

#endif /*_UFS_CAL_IF_H */
