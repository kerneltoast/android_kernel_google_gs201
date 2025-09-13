/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2021 Google LLC
 */

#ifndef _UFS_DUMP_CMN_H
#define _UFS_DUMP_CMN_H

#if defined(CONFIG_SOC_GS101)
#include "gs101/ufs-dump.h"
#elif defined(CONFIG_SOC_GS201)
#include "gs201/ufs-dump.h"
#elif defined(CONFIG_SOC_ZUMA)
#include "zuma/ufs-dump.h"
#endif

#endif /*_UFS_DUMP_CMN_H */
