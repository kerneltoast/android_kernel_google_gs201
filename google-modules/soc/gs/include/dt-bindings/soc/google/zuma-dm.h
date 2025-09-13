/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 Google, LLC.
 *
 * Device Tree binding constants for ZUMA
 */

#ifndef _DT_BINDINGS_ZUMA_H
#define _DT_BINDINGS_ZUMA_H

/* NUMBER FOR DVFS MANAGER */
#define DM_CPU_CL0	0
#define DM_CPU_CL1	1
#define DM_CPU_CL2	2
#define DM_MIF		3
#define DM_INT		4
#define DM_INTCAM	5
#define DM_CAM		6
#define DM_TPU		7
#define DM_TNR		8
#define DM_DISP		9
#define DM_MFC		10
#define DM_BW		11
#define DM_DSU		12
#define DM_BCI		13

/* CONSTRAINT TYPE */
#define CONSTRAINT_MIN	0
#define CONSTRAINT_MAX	1

#endif /* _DT_BINDINGS_ZUMA_H */
