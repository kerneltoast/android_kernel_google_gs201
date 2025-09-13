/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2022 Google, LLC.
 *
 * Device Tree binding constants for ZUMA devfreq
 */

#ifndef _DT_BINDINGS_ZUMA_DEVFREQ_H
#define _DT_BINDINGS_ZUMA_DEVFREQ_H
/* DEVFREQ TYPE LIST */
#define DEVFREQ_MIF			0
#define DEVFREQ_INT			1
#define DEVFREQ_DISP			2
#define DEVFREQ_CAM			3
#define DEVFREQ_INTCAM			4
#define DEVFREQ_TNR			5
#define DEVFREQ_MFC			6
#define DEVFREQ_BW			7
#define DEVFREQ_DSU			8
#define DEVFREQ_BCI			9
#define DEVFREQ_TYPE_END		10

/* ESS FLAG LIST */
#define ESS_FLAG_INT	3
#define ESS_FLAG_MIF	4
#define ESS_FLAG_ISP	5
#define ESS_FLAG_DISP	6
#define ESS_FLAG_INTCAM	7
#define ESS_FLAG_TPU	8
#define ESS_FLAG_TNR	9
#define ESS_FLAG_MFC	10
#define ESS_FLAG_BW	11
#define ESS_FLAG_DSU	12
#define ESS_FLAG_BCI	13


/* DEVFREQ GOV TYPE */
#define SIMPLE_INTERACTIVE 0
#define MEM_LATENCY 1
#define DSU_LATENCY 2

#endif
