/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Chip-dependent configuration for TPU CPU.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#ifndef __JANEIRO_CONFIG_TPU_CPU_H__
#define __JANEIRO_CONFIG_TPU_CPU_H__

#define EDGETPU_REG_RESET_CONTROL                       0x190010
#define CPUPORESET					(1 << 1)
#define EDGETPU_REG_INSTRUCTION_REMAP_CONTROL           0x190050
#define EDGETPU_REG_INSTRUCTION_REMAP_BASE              0x190058
#define EDGETPU_REG_INSTRUCTION_REMAP_LIMIT             0x190060
#define EDGETPU_REG_INSTRUCTION_REMAP_NEW_BASE          0x190068
#define EDGETPU_REG_INSTRUCTION_REMAP_SECURITY          0x190070
#define EDGETPU_REG_SECURITY                            0x190048
#define EDGETPU_REG_POWER_CONTROL                       0x1A0008
#define EDGETPU_REG_LPM_CONTROL                         0x1D0000
#define PSTATE_SHIFT					1
#define PSTATE						(1 << PSTATE_SHIFT)
#define PREQ						(1 << 2)
#define PDENY						(1 << 3)
#define PACCEPT						(1 << 4)
#define PACTIVE						(1 << 6)

#endif /* __JANEIRO_CONFIG_TPU_CPU_H__ */
