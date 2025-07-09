/* SPDX-License-Identifier: GPL-2.0 */

/*
 *
 * (C) COPYRIGHT 2014-2017 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
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
 * SPDX-License-Identifier: GPL-2.0
 *
 */

/*
 * Copyright 2020-2021 Google LLC.
 *
 * Author: Sidath Senanayake <sidaths@google.com>
 */

#ifndef _KBASE_CONFIG_PLATFORM_H_
#define _KBASE_CONFIG_PLATFORM_H_

/**
 * Power management configuration
 *
 * Attached value: pointer to @ref kbase_pm_callback_conf
 * Default value: See @ref kbase_pm_callback_conf
 */
#ifdef CONFIG_MALI_PIXEL_GPU_PM
#define POWER_MANAGEMENT_CALLBACKS (&pm_callbacks)
extern struct kbase_pm_callback_conf pm_callbacks;
#else
#define POWER_MANAGEMENT_CALLBACKS (NULL)
#endif

/**
 * Clock Rate Trace configuration functions
 *
 * Attached value: pointer to @ref kbase_clk_rate_trace_op_conf
 * Default value: See @ref kbase_clk_rate_trace_op_conf
 */
#ifdef CONFIG_MALI_MIDGARD_DVFS
#define CLK_RATE_TRACE_OPS (&pixel_clk_rate_trace_ops)
extern struct kbase_clk_rate_trace_op_conf pixel_clk_rate_trace_ops;
#endif

/**
 * Platform specific configuration functions
 *
 * Attached value: pointer to @ref kbase_platform_funcs_conf
 * Default value: See @ref kbase_platform_funcs_conf
 */
#define PLATFORM_FUNCS (&platform_funcs)

extern struct kbase_platform_funcs_conf platform_funcs;

#ifdef CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING
#define PLATFORM_PROTECTED_CALLBACKS (&pixel_protected_ops);
extern struct protected_mode_ops pixel_protected_ops;
#endif /* CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING */

/**
 * DVFS Utilization evaluation period
 *
 * The amount of time (in milliseconds) between sucessive measurements of the
 * GPU utilization. This also affects how frequently the DVFS update logic runs.
 */
#define DEFAULT_PM_DVFS_PERIOD (20)

/* Linux includes */
#ifdef CONFIG_MALI_MIDGARD_DVFS
#include <linux/atomic.h>
#include <linux/hashtable.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>
#endif /* CONFIG_MALI_MIDGARD_DVFS */

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
#include <linux/atomic.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#endif /* IS_ENABLED(CONFIG_EXYNOS_ITMON) */

/* SOC level includes */
#if IS_ENABLED(CONFIG_GOOGLE_BCL)
#include <bcl.h>
#endif
#if IS_ENABLED(CONFIG_EXYNOS_PD)
#include <soc/google/exynos-pd.h>
#endif
#ifdef CONFIG_MALI_MIDGARD_DVFS
#ifdef CONFIG_MALI_PIXEL_GPU_QOS
#include <soc/google/exynos_pm_qos.h>
#endif /* CONFIG_MALI_MIDGARD_DVFS */
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

/* Pixel integration includes */
#ifdef CONFIG_MALI_MIDGARD_DVFS
#include "pixel_gpu_dvfs.h"
#endif /* CONFIG_MALI_MIDGARD_DVFS */

#include "pixel_gpu_uevent.h"

/* All port specific fields go here */
#define OF_DATA_NUM_MAX 200
#define CPU_FREQ_MAX INT_MAX

enum gpu_power_state {
	/*
	 * Mali GPUs have a hierarchy of power domains, which must be powered up
	 * in order and powered down in reverse order. Individual architectures
	 * and implementations may not allow each domain to be powered up or
	 * down independently of the others.
	 *
	 * The power state can thus be defined as the highest-level domain that
	 * is currently powered on.
	 *
	 * GLOBAL: JM, CSF: The frontend (JM, CSF), including registers.
	 *         CSF: The L2 and AXI interface, Tiler, and MMU.
	 * STACKS: JM, CSF: The shader cores.
	 *         JM: The L2 and AXI interface, Tiler, and MMU.
	 */
	GPU_POWER_LEVEL_OFF       = 0,
	GPU_POWER_LEVEL_GLOBAL    = 1,
	GPU_POWER_LEVEL_STACKS    = 2,
	GPU_POWER_LEVEL_NUM
};

/**
 * enum gpu_pm_domain - Power domains on the GPU.
 *
 * This enum lists the different power domains present on the Mali GPU.
 */
enum gpu_pm_domain {
	/**
	 * &GPU_PM_DOMAIN_TOP: GPU Top Level power domain
	 */
	GPU_PM_DOMAIN_TOP = 0,
	/**
	 * &GPU_PM_DOMAIN_CORES: GPU shader stacks power domain
	 */
	GPU_PM_DOMAIN_CORES,
	/* Insert new power domains here. */
	GPU_PM_DOMAIN_COUNT,
};

#ifdef CONFIG_MALI_MIDGARD_DVFS
/**
 * struct gpu_dvfs_opp_metrics - Metrics data for an operating point.
 *
 * @time_total:      The total amount of time (in ns) that the device was powered on and at this
 *                   operating point.
 * @time_last_entry: The time (in ns) since device boot that this operating point was used.
 * @entry_cont:      The number of times this operating point was used.
 */
struct gpu_dvfs_opp_metrics {
	u64 time_total;
	u64 time_last_entry;
	unsigned int entry_count;
};

/**
 * struct gpu_dvfs_opp - Data for a GPU operating point.
 *
 * @clk:          The frequencies (in kHz) of the GPU clocks.
 * @vol:          The voltages (in mV) of each GPU power domain. Obtained via ECT.
 *
 * @util_min:     The minimum threshold of utilization before the governor should consider a lower
 *                operating point.
 * @util_max:     The maximum threshold of utlization before the governor should consider moving to
 *                a higher operating point.
 * @mcu_util_max: The maximum threshold of MCU utilization above which the
 *                governor should consider moving to a higher OPP
 * @mcu_util_min: The minimum value after which if the gpu utilisation is also
 *                low, the DVFS should consider a lower OPP
 * @hysteresis:   A measure of how long the governor should keep the GPU at this operating point
 *                before moving to a lower one. For example, in the basic governor, this translates
 *                directly into &hr_timer ticks for the Mali DVFS utilization thread, but other
 *                governors may chose to use this value in different ways.
 *
 * @metrics:      Metrics data for this operating point.
 *
 * @qos.mif_min:  The minimum frequency (in kHz) for the memory interface (MIF).
 * @qos.int_min:  The minimum frequency (in kHz) for the internal memory network (INT).
 * @qos.cpu0_min: The minimum frequency (in kHz) for the little CPU cluster.
 * @qos.cpu1_min: The minimum frequency (in kHz) for the medium CPU cluster.
 * @qos.cpu2_max: The maximum frequency (in kHz) for the big CPU cluster.
 *
 * Unless specified otherwise, all data is obtained from device tree.
 */
struct gpu_dvfs_opp {
	/* Clocks */
	unsigned int clk[GPU_DVFS_CLK_COUNT];

	/* Voltages */
	unsigned int vol[GPU_DVFS_CLK_COUNT];

	int util_min;
	int util_max;
	int hysteresis;
	int mcu_util_max;
	int mcu_util_min;

	/* Metrics */
	struct gpu_dvfs_opp_metrics metrics;

	/* QOS values */
	struct {
		int mif_min;
		int int_min;
		int cpu0_min;
		int cpu1_min;
		int cpu2_max;
	} qos;
};

#ifdef CONFIG_MALI_PIXEL_GPU_QOS
/* Forward declaration of QOS request */
struct gpu_dvfs_qos_vote;
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

/* Forward declaration of per-UID metrics */
struct gpu_dvfs_metrics_uid_stats;
#endif /* CONFIG_MALI_MIDGARD_DVFS */

/**
 * struct pixel_context - Pixel GPU context
 *
 * @kbdev:                      The &struct kbase_device for the GPU.
 *
 * @pm.lock:                    &struct mutex used to ensure serialization of calls to kernel power
 *                              management functions on the GPU power domain devices held in
 *                              &pm.domain_devs.
 * @pm.state:                   Holds the current power state of the GPU.
 * @pm.domain_devs              Virtual pm domain devices.
 * @pm.domain_links             Links from pm domain devices to the real device.
 * @pm.domain:                  The power domain the GPU is in.
 * @pm.status_reg_offset:       Register offset to the G3D status in the PMU. Set via DT.
 * @pm.status_local_power_mask: Mask to extract power status of the GPU. Set via DT.
 * @pm.use_autosuspend:         Use autosuspend on the TOP domain if true, sync suspend if false.
 * @pm.autosuspend_delay:       Delay (in ms) before PM runtime should trigger auto suspend on TOP
 *                              domain if use_autosuspend is true.
 * @pm.bcl_dev:                 Pointer to the Battery Current Limiter device.
 * @pm.firmware_idle_hysteresis_time_ms The duration of the GPU idle hysteresis in milliseconds. Set via DT.
 * @pm.firmware_idle_hysteresis_gpu_sleep_scaler Factor for calculating GPU idle hysteresis
                                in case GPU sleep is enabled. &csf.gpu_idle_hysteresis_ms is eventually
                                (firmware_idle_hysteresis_time_ms / firmware_idle_hysteresis_gpu_sleep_scaler).
                                Set via DT.
 * @pm.cores_suspend_hysteresis_time_ms Hysteresis timeout for suspending CORES domain. Set via DT.
 * @pm.top_suspend_hysteresis_time_ms   Hysteresis timeout for suspending TOP domain. Set via DT.
 *
 * @tz_protection_enabled:      Storing the secure rendering state of the GPU. Access to this is
 *                              controlled by the HW access lock for the GPU associated with @kbdev.
 *
 * @dvfs.lock:                  &struct mutex used to control access to DVFS levels.
 *
 * @dvfs.control_wq:            Workqueue for processing DVFS utilization metrics.
 * @dvfs.control_work:          &struct work_struct storing link to Pixel GPU code to convert
 *                              incoming utilization data from the Mali driver into DVFS changes on
 *                              the GPU.
 * @dvfs.util:                  Stores incoming utilization metrics from the Mali driver.
 * @dvfs.mcu_util:              Stores incoming MCU utilisation metrics.
 * @dvfs.util_gl:               Percentage of utilization from a non-OpenCL work
 * @dvfs.util_cl:               Percentage of utilization from a OpenCL work.
 * @dvfs.clockdown_wq:          Delayed workqueue for clocking down the GPU after it has been idle
 *                              for a period of time.
 * @dvfs.clockdown_work:        &struct delayed_work_struct storing link to Pixel GPU code to set
 *                              the GPU to its minimum throughput level.
 * @dvfs.clockdown_hysteresis:  The time (in ms) the GPU can remained powered off before being set
 *                              to the minimum throughput level. Set via DT.
 *
 * @dvfs.clks:                  Array of clock data per GPU clock.
 *
 * @dvfs.table:                 Pointer to the DVFS table which is an array of &struct gpu_dvfs_opp
 * @dvfs.table_size:            Number of levels in in @dvfs.table.
 * @dvfs.level:                 The current last active level run on the GPU.
 * @dvfs.level_target:          The level at which the GPU should run at next power on.
 * @dvfs.level_max:             The maximum throughput level available on the GPU. Set via DT.
 * @dvfs.level_min:             The minimum throughput level available of the GPU. Set via DT.
 * @dvfs.level_scaling_max:     The maximum throughput level the GPU can run at. Should only be set
 *                              via &gpu_dvfs_update_level_lock().
 * @dvfs.level_scaling_min:     The minimum throughput level the GPU can run at. Should only be set
 *                              via &gpu_dvfs_update_level_lock().
 *
 * @dvfs.metrics.last_time:        The last time (in ns) since device boot that the DVFS metric
 *                                 logic was run.
 * @dvfs.metrics.last_power_state: The GPU's power state when the DVFS metric logic was last run.
 * @dvfs.metrics.last_level:       The GPU's level when the DVFS metric logic was last run.
 * @dvfs.metrics.transtab:         Pointer to the DVFS transition table.
 * @dvfs.metrics.work_uid_stats:   An array of pointers to the per-UID stats blocks currently
 *                                 resident in each of the GPU's job slots, or CSG slots.
 *                                 Access is controlled by the dvfs.metrics.lock.
 * @dvfs.metrics.uid_stats_table:  8-bits hash table of the per-UID stats blocks.
 *                                 Modification to the hash table itself and to its elements is
 *                                 protected by the dvfs.metrics.lock.
 *                                 Reading of the elements is also protected by the same lock.
 *                                 Elements are only added and never removed at run-time, so the
 *                                 removal of all elements on destruction is not protected.
 *
 * @dvfs.governor.curr:  The currently enabled DVFS governor.
 * @dvfs.governor.delay: Governor specific variable. The basic governor uses this to store the
 *                       remaining ticks before a lower throughput level will be set.
 *
 * @dvfs.qos.enabled:       Stores whether QOS requests have been set.
 * @dvfs.qos.level_last:    The level for which QOS requests were made. Negative if no QOS is set.
 * @dvfs.qos.int_min:       QOS vote structure for setting minimum INT clock
 * @dvfs.qos.mif_min:       QOS vote structure for setting minimum MIF clock
 * @dvfs.qos.cpu0_min:      QOS vote structure for setting minimum CPU cluster 0 (little) clock
 * @dvfs.qos.cpu1_min:      QOS vote structure for setting minimum CPU cluster 1 (medium) clock
 * @dvfs.qos.cpu2_max:      QOS vote structure for setting maximum CPU cluster 2 (big) clock
 *
 * @dvfs.qos.bts.enabled:   Stores whether Bus Traffic Shaping (BTS) is currently enabled
 * @dvfs.qos.bts.threshold: The G3D shader stack clock at which BTS will be enabled. Set via DT.
 * @dvfs.qos.bts.scenario:  The index of the BTS scenario to be used. Set via DT.
 *
 * @itmon.wq:     A workqueue for ITMON page table search.
 * @itmon.work:   The work item for the above.
 * @itmon.nb:     The ITMON notifier block.
 * @itmon.pa:     The faulting physical address.
 * @itmon.active: Active count, non-zero while a search is active.
 * @slc_demand:   Tracks demand for SLC space
 */
struct pixel_context {
	struct kbase_device *kbdev;

	struct {
		struct mutex lock;
		enum gpu_power_state state;

		struct device *domain_devs[GPU_PM_DOMAIN_COUNT];
		struct device_link *domain_links[GPU_PM_DOMAIN_COUNT];
		struct exynos_pm_domain *domain;
		unsigned int status_reg_offset;
		unsigned int status_local_power_mask;
		bool use_autosuspend;
		unsigned int autosuspend_delay;
#ifdef CONFIG_MALI_MIDGARD_DVFS
		struct gpu_dvfs_opp_metrics power_off_metrics;
		struct gpu_dvfs_opp_metrics power_on_metrics;
#endif /* CONFIG_MALI_MIDGARD_DVFS */
#if IS_ENABLED(CONFIG_GOOGLE_BCL)
		struct bcl_device *bcl_dev;
		struct notifier_block qos_nb;
#endif
		struct pixel_rail_state_log *rail_state_log;
		unsigned int firmware_idle_hysteresis_time_ms;
#ifdef CONFIG_MALI_PIXEL_GPU_SLEEP
		unsigned int firmware_idle_hysteresis_gpu_sleep_scaler;
		unsigned int cores_suspend_hysteresis_time_ms;
		unsigned int top_suspend_hysteresis_time_ms;
#endif /* CONFIG_MALI_PIXEL_GPU_SLEEP */
	} pm;

#ifdef CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING
	bool tz_protection_enabled;
#endif /* CONFIG_MALI_PIXEL_GPU_SECURE_RENDERING */

#ifdef CONFIG_MALI_MIDGARD_DVFS
	struct {
		struct mutex lock;

		struct workqueue_struct *control_wq;
		struct work_struct control_work;
		atomic_t util;
		atomic_t mcu_util;
#if !MALI_USE_CSF
		atomic_t util_gl;
		atomic_t util_cl;
#endif

		struct workqueue_struct *clockdown_wq;
		struct delayed_work clockdown_work;
		unsigned int clockdown_hysteresis;

		bool updates_enabled;
		struct gpu_dvfs_clk clks[GPU_DVFS_CLK_COUNT];

		struct gpu_dvfs_opp *table;
		int table_size;
		int step_up_val;
		int level;
#if MALI_USE_CSF
		int level_before_headroom;
#endif
		int level_target;
		int level_max;
		int level_min;
		int level_scaling_max;
		int level_scaling_min;
		int level_scaling_compute_min;
		struct gpu_dvfs_level_lock level_locks[GPU_DVFS_LEVEL_LOCK_COUNT];
#if MALI_USE_CSF
		u32 capacity_headroom;
		u32 capacity_history[8];
		u8 capacity_history_depth;
		u8 capacity_history_index;
#endif

		struct {
			enum gpu_dvfs_governor_type curr;
			int delay;
		} governor;

		struct {
			spinlock_t lock;
			u64 last_time;
			bool last_power_state;
			int last_level;
			int *transtab;
#if !MALI_USE_CSF
			struct gpu_dvfs_metrics_uid_stats *work_uid_stats[BASE_JM_MAX_NR_SLOTS * SLOT_RB_SIZE];
#else
			struct gpu_dvfs_metrics_uid_stats *work_uid_stats[MAX_SUPPORTED_CSGS];
#endif /* !MALI_USE_CSF */
			DECLARE_HASHTABLE(uid_stats_table, 8);
		} metrics;
#if MALI_USE_CSF
		struct {
			int mcu_protm_scale_num;
			int mcu_protm_scale_den;
			int mcu_down_util_scale_num;
			int mcu_down_util_scale_den;
		} tunable;
#endif

#ifdef CONFIG_MALI_PIXEL_GPU_QOS
		struct {
			bool enabled;
			int level_last;
			struct gpu_dvfs_qos_vote int_min;
			struct gpu_dvfs_qos_vote mif_min;
			struct gpu_dvfs_qos_vote cpu0_min;
			struct gpu_dvfs_qos_vote cpu1_min;
			struct gpu_dvfs_qos_vote cpu2_max;

#ifdef CONFIG_MALI_PIXEL_GPU_BTS
			struct {
				bool enabled;
				int threshold;
				unsigned int scenario;
			} bts;
#endif /* CONFIG_MALI_PIXEL_GPU_BTS */
		} qos;
#endif /* CONFIG_MALI_PIXEL_GPU_QOS */

#ifdef CONFIG_MALI_PIXEL_GPU_THERMAL
		struct {
			struct thermal_cooling_device *cdev;
		} tmu;
#endif /* CONFIG_MALI_PIXEL_GPU_THERMAL */
	} dvfs;
#endif /* CONFIG_MALI_MIDGARD_DVFS */

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	struct {
		struct workqueue_struct *wq;
		struct work_struct work;
		struct notifier_block nb;
		phys_addr_t pa;
		atomic_t active;
	} itmon;
#endif
#ifndef PIXEL_GPU_SLC_ACPM_SIGNAL
	atomic_t slc_demand;
#endif /* PIXEL_GPU_SLC_ACPM_SIGNAL */

	struct gpu_uevent_ctx gpu_uevent_ctx;
};

/**
 * struct pixel_platform_data - Per kbase_context Pixel specific platform data
 *
 * @kctx:       Handle to the parent kctx
 * @stats:      Tracks the dvfs metrics for the UID associated with this context
 * @slc_vote:   Tracks whether this context is voting for slc
 * @slc_demand: Tracks demand for SLC space
 */
struct pixel_platform_data {
	struct kbase_context *kctx;
	struct gpu_dvfs_metrics_uid_stats* stats;
	int slc_vote;
#ifndef PIXEL_GPU_SLC_ACPM_SIGNAL
	atomic_t slc_demand;
#endif /* PIXEL_GPU_SLC_ACPM_SIGNAL */
};

#endif /* _KBASE_CONFIG_PLATFORM_H_ */
