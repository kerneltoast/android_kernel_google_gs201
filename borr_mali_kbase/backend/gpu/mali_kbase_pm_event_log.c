// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2022 Google LLC. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <backend/gpu/mali_kbase_pm_event_log.h>

static inline u32 kbase_pm_next_log_event(
	struct kbase_pm_event_log *log)
{
	u32 ret = atomic_inc_return(&log->last_event);
	return ret % EVENT_LOG_MAX;
}

struct kbase_pm_event_log_event *kbase_pm_add_log_event(
	struct kbase_device *kbdev)
{
	struct kbase_pm_event_log *log = &kbdev->pm.backend.event_log;
	struct kbase_pm_event_log_event *ret = NULL;

	ret = &log->events[kbase_pm_next_log_event(log)];

	memset(ret, 0, sizeof(*ret));
	ret->timestamp = ktime_get();
	return ret;
}

/**
 * struct kbase_pm_event_log_metadata - Info about the event log.
 *
 * @magic: always 'kpel', helps find the log in memory dumps
 * @version: updated whenever the binary layout changes
 * @events_address: the memory address of the log, or in a file the offset
 *                  from the start of the metadata to the log
 * @num_events: the capacity of the event log
 * @event_stride: distance between log entries, to aid in parsing if only some
 *                entry types are supported by the parser
 **/
struct kbase_pm_event_log_metadata {
	char magic[4];
	u8 version;
	u64 events_address;
	u32 num_events;
	u32 event_stride;
} __attribute__((packed));

static struct kbase_pm_event_log_metadata global_event_log_metadata;

void kbase_pm_init_event_log(struct kbase_device *kbdev)
{
	struct kbase_pm_event_log_metadata *md =
			&global_event_log_metadata;
	atomic_set(&kbdev->pm.backend.event_log.last_event, -1);
	md->magic[0] = 'k';
	md->magic[1] = 'p';
	md->magic[2] = 'e';
	md->magic[3] = 'l';
	md->version = 1;
	md->num_events = EVENT_LOG_MAX;
	md->events_address = (u64)kbdev->pm.backend.event_log.events;
	md->event_stride = ((u8*)&kbdev->pm.backend.event_log.events[1] -
			    (u8*)&kbdev->pm.backend.event_log.events[0]);
}

u64 kbase_pm_max_event_log_size(struct kbase_device *kbdev)
{
	return sizeof(struct kbase_pm_event_log_metadata)
			+ sizeof(kbdev->pm.backend.event_log.events);
}

int kbase_pm_copy_event_log(struct kbase_device *kbdev,
		void *buffer, u64 size)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);
	if (size < kbase_pm_max_event_log_size(kbdev)) {
		return -EINVAL;
	}
	memcpy(buffer, &global_event_log_metadata,
	       sizeof(global_event_log_metadata));
	memcpy(((u8*)buffer) + sizeof(global_event_log_metadata),
	       &kbdev->pm.backend.event_log.events,
	       sizeof(kbdev->pm.backend.event_log.events));
	((struct kbase_pm_event_log_metadata*)buffer)->events_address =
			sizeof(struct kbase_pm_event_log_metadata);

	return 0;
}

