/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
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

/*
 * Power management API definitions used internally by GPU backend
 */

#ifndef _KBASE_BACKEND_PM_EVENT_LOG_H_
#define _KBASE_BACKEND_PM_EVENT_LOG_H_

#include <mali_kbase.h>
#include <mali_kbase_pm.h>

/**
 * kbase_pm_add_log_event - Add a newly-initialized event to the event log.
 *
 * @kbdev: Device pointer
 *
 * Return: a pointer to the event, which has been nulled out and had its
 * timestamp set to the current time.
 *
 */
struct kbase_pm_event_log_event *kbase_pm_add_log_event(
	struct kbase_device *kbdev);

#endif /* _KBASE_BACKEND_PM_INTERNAL_H_ */
