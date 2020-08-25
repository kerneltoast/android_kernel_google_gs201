// SPDX-License-Identifier: GPL-2.0-only
/*
 * pt_trace_points.c
 *
 * System cache management.
 *
 * Copyright 2019 Google LLC
 *
 * Author: cozette@google.com
 */

#include "pt.h"

#define CREATE_TRACE_POINTS
#include "pt_trace.h"

EXPORT_SYMBOL_GPL(__tracepoint_pt_driver_log);
EXPORT_SYMBOL_GPL(__tracepoint_pt_resize_callback);
EXPORT_SYMBOL_GPL(__tracepoint_pt_enable);
