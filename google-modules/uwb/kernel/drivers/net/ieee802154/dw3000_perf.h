/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#ifndef __DW3000_PERF_H
#define __DW3000_PERF_H

#include <linux/version.h>
#include <linux/perf_event.h>

static struct perf_event_attr perf_hw_attr[] = {
	{
		.type = PERF_TYPE_HARDWARE,
		.config = PERF_COUNT_HW_CPU_CYCLES,
		.size = sizeof(struct perf_event_attr),
		.pinned = 1,
		.disabled = 1,
	},
	{
		.type = PERF_TYPE_HARDWARE,
		.config = PERF_COUNT_HW_INSTRUCTIONS,
		.size = sizeof(struct perf_event_attr),
		.pinned = 1,
		.disabled = 1,
	},
	{
		.type = PERF_TYPE_HARDWARE,
		.config = PERF_COUNT_HW_BRANCH_INSTRUCTIONS,
		.size = sizeof(struct perf_event_attr),
		.pinned = 1,
		.disabled = 1,
	},
	{
		.type = PERF_TYPE_HARDWARE,
		.config = PERF_COUNT_HW_CACHE_MISSES,
		.size = sizeof(struct perf_event_attr),
		.pinned = 1,
		.disabled = 1,
	}
};

#define PERF_EVT_COUNT (ARRAY_SIZE(perf_hw_attr))

static const char *const perf_hw_evt_name[PERF_EVT_COUNT] = {
	"cpu cycles  ", "instructions", "branch insts", "cache misses"
};

static struct perf_event *perf_hw_evt[PERF_EVT_COUNT];

/* Callback function for perf event subsystem */
static void overflow_callback(struct perf_event *event,
			      struct perf_sample_data *data,
			      struct pt_regs *regs)
{
	struct dw3000 *dw = event->overflow_handler_context;

	event->hw.interrupts = 0;
	dev_warn(dw->dev, "test mode: counter overflow for event %x:%llx\n",
		 event->attr.type, event->attr.config);
}

static inline void perf_event_create_all(struct dw3000 *dw)
{
	int i;

	for (i = 0; i < PERF_EVT_COUNT; i++) {
		struct perf_event *evt = perf_event_create_kernel_counter(
			&perf_hw_attr[i], smp_processor_id(), NULL,
			overflow_callback, dw);
		if (IS_ERR(evt)) {
			dev_warn(
				dw->dev,
				"test mode: cannot create perf_hw_evt %d (err %ld)\n",
				i, PTR_ERR(evt));
			evt = NULL;
		}
		perf_hw_evt[i] = evt;
	}
}

static inline void perf_event_release_all(void)
{
	int i;

	for (i = 0; i < PERF_EVT_COUNT; i++) {
		struct perf_event *evt = perf_hw_evt[i];

		if (evt)
			perf_event_release_kernel(evt);
	}
}

static inline void perf_event_start_all(void)
{
	int i;

	for (i = 0; i < PERF_EVT_COUNT; i++) {
		struct perf_event *evt = perf_hw_evt[i];

		if (evt)
			perf_event_enable(evt);
	}
}

static inline void perf_event_stop_all(u64 *vals)
{
	u64 dummy[2];
	int i;

	for (i = 0; i < PERF_EVT_COUNT; i++) {
		struct perf_event *evt = perf_hw_evt[i];

		if (evt) {
			vals[i] = perf_event_read_value(evt, &dummy[0],
							&dummy[1]);
#if (KERNEL_VERSION(5, 5, 0) > LINUX_VERSION_CODE)
			perf_event_disable(evt);
			local64_set(&evt->count, 0);
#else
			perf_event_pause(evt, true);
#endif
		}
	}
}

#endif /* __DW3000_PERF_H */
