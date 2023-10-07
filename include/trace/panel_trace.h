/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Panel command trace support
 *
 * Copyright (C) 2022 Google, Inc.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM panel

#if !defined(_PANEL_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _PANEL_TRACE_H_

#include <linux/tracepoint.h>

TRACE_EVENT_CONDITION(dsi_tx,
	TP_PROTO(u8 type, const u8 *tx_buf, size_t length, bool last),
	TP_ARGS(type, tx_buf, length, last),
	TP_CONDITION(length > 0),
	TP_STRUCT__entry(
			__field(	u8,	type		)
			__dynamic_array(u8,	tx_buf, length	)
			__field(	bool,	last		)
		),
	TP_fast_assign(
			__entry->type = type;
			memcpy(__get_dynamic_array(tx_buf), tx_buf, length);
			__entry->last = last;
		),
	TP_printk("type=0x%02x length=%u last=%d tx=[%s]", __entry->type,
			  __get_dynamic_array_len(tx_buf), __entry->last,
			  __print_hex(__get_dynamic_array(tx_buf),
				      __get_dynamic_array_len(tx_buf)))
);

TRACE_EVENT_CONDITION(dsi_rx,
	TP_PROTO(u8 cmd, const u8 *rx_buf, size_t length),
	TP_ARGS(cmd, rx_buf, length),
	TP_CONDITION(length > 0),
	TP_STRUCT__entry(
			__field(	u8, cmd			)
			__dynamic_array(u8, rx_buf, length	)
		),
	TP_fast_assign(
			__entry->cmd = cmd;
			memcpy(__get_dynamic_array(rx_buf), rx_buf, length);
		),
	TP_printk("cmd=0x%02x length=%u rx=[%s]", __entry->cmd,
			  __get_dynamic_array_len(rx_buf),
			  __print_hex(__get_dynamic_array(rx_buf),
				      __get_dynamic_array_len(rx_buf)))
);

TRACE_EVENT(dsi_cmd_fifo_status,
	TP_PROTO(u8 header, u16 payload),
	TP_ARGS(header, payload),
	TP_STRUCT__entry(
			__field(u8,	header )
			__field(u16,	payload)
		),
	TP_fast_assign(
			__entry->header   = header;
			__entry->payload  = payload;
		),
	TP_printk("header=%d payload=%d", __entry->header, __entry->payload)
);

#endif /* _PANEL_TRACE_H_ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH trace/.

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE panel_trace

/* This part must be outside protection */
#include <trace/define_trace.h>
