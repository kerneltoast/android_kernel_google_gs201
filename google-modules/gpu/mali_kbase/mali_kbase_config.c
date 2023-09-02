// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2011-2015, 2017, 2020-2021 ARM Limited. All rights reserved.
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

#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <mali_kbase_config_defaults.h>

int kbasep_platform_device_init(struct kbase_device *kbdev)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf *)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_init_func)
		return platform_funcs_p->platform_init_func(kbdev);

	return 0;
}

void kbasep_platform_device_term(struct kbase_device *kbdev)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf *)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_term_func)
		platform_funcs_p->platform_term_func(kbdev);
}

int kbasep_platform_device_late_init(struct kbase_device *kbdev)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf *)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_late_init_func)
		platform_funcs_p->platform_late_init_func(kbdev);

	return 0;
}

void kbasep_platform_device_late_term(struct kbase_device *kbdev)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf *)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_late_term_func)
		platform_funcs_p->platform_late_term_func(kbdev);
}

int kbasep_platform_context_init(struct kbase_context *kctx)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf *)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_handler_context_init_func)
		return platform_funcs_p->platform_handler_context_init_func(kctx);

	return 0;
}

void kbasep_platform_context_term(struct kbase_context *kctx)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf *)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_handler_context_term_func)
		platform_funcs_p->platform_handler_context_term_func(kctx);
}

void kbasep_platform_event_work_begin(void *param)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf*)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_handler_work_begin_func)
		platform_funcs_p->platform_handler_work_begin_func(param);
}

void kbasep_platform_event_work_end(void *param)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf*)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_handler_work_end_func)
		platform_funcs_p->platform_handler_work_end_func(param);
}

int kbasep_platform_fw_config_init(struct kbase_device *kbdev)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf*)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_fw_cfg_init_func)
		return platform_funcs_p->platform_fw_cfg_init_func(kbdev);

	return 0;
}

void kbasep_platform_event_core_dump(struct kbase_device *kbdev, const char* reason)
{
	struct kbase_platform_funcs_conf *platform_funcs_p;

	platform_funcs_p = (struct kbase_platform_funcs_conf*)PLATFORM_FUNCS;
	if (platform_funcs_p && platform_funcs_p->platform_handler_core_dump_func)
		platform_funcs_p->platform_handler_core_dump_func(kbdev, reason);
}

