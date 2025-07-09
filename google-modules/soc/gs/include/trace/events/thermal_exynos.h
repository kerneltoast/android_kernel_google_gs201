/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM thermal_exynos

#if !defined(_TRACE_EXYNOS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_THERMAL_EXYNOS_H

#include <linux/tracepoint.h>

TRACE_EVENT(thermal_exynos_power_cpu_get_power,
	TP_PROTO(int tzid, int cpu, unsigned long freq, u32 *load,
		 size_t load_len, u32 dynamic_power, u32 static_power),

	TP_ARGS(tzid, cpu, freq, load, load_len, dynamic_power, static_power),

	TP_STRUCT__entry(
		__field(int, tzid)
		__field(int, cpu)
		__field(unsigned long, freq)
		__dynamic_array(u32, load, load_len)
		__field(size_t, load_len)
		__field(u32, dynamic_power)
		__field(u32, static_power)
	),

	TP_fast_assign(
		__entry->tzid = tzid;
		__entry->cpu = cpu;
		__entry->freq = freq;
		memcpy(__get_dynamic_array(load), load,
		       load_len * sizeof(*load));
		__entry->load_len = load_len;
		__entry->dynamic_power = dynamic_power;
		__entry->static_power = static_power;
	),

	TP_printk("thermal_zone_id=%d cpu=%d freq=%lu load={%s} dynamic_power=%d static_power=%d",
		  __entry->tzid, __entry->cpu, __entry->freq,
		  __print_array(__get_dynamic_array(load), __entry->load_len, 4),
		  __entry->dynamic_power, __entry->static_power)
);

TRACE_EVENT(thermal_exynos_power_cpu_limit,
	TP_PROTO(int tzid, int cpu, unsigned int freq,
		 unsigned long cdev_state, u32 power),

	TP_ARGS(tzid, cpu, freq, cdev_state, power),

	TP_STRUCT__entry(
		__field(int, tzid)
		__field(int, cpu)
		__field(unsigned int, freq)
		__field(unsigned long, cdev_state)
		__field(u32, power)
	),

	TP_fast_assign(
		__entry->tzid = tzid;
		__entry->cpu = cpu;
		__entry->freq = freq;
		__entry->cdev_state = cdev_state;
		__entry->power = power;
	),

	TP_printk("thermal_zone_id=%d cpu=%d freq=%u cdev_state=%lu power=%u",
		  __entry->tzid, __entry->cpu, __entry->freq, __entry->cdev_state,
		  __entry->power)
);

TRACE_EVENT(thermal_exynos_power_gpu_get_power,
	TP_PROTO(unsigned long freq, u32 load, u32 dynamic_power, u32 static_power),

	TP_ARGS(freq, load, dynamic_power, static_power),

	TP_STRUCT__entry(
		__field(unsigned long, freq)
		__field(u32, load)
		__field(u32, dynamic_power)
		__field(u32, static_power)
	),

	TP_fast_assign(
		__entry->freq = freq;
		__entry->load = load;
		__entry->dynamic_power = dynamic_power;
		__entry->static_power = static_power;
	),

	TP_printk("freq=%lu load=%d dynamic_power=%d static_power=%d",
		  __entry->freq, __entry->load, __entry->dynamic_power, __entry->static_power)
);

TRACE_EVENT(thermal_exynos_power_gpu_limit,
	TP_PROTO(unsigned int freq, unsigned long cdev_state, u32 power),

	TP_ARGS(freq, cdev_state, power),

	TP_STRUCT__entry(
		__field(unsigned int, freq)
		__field(unsigned long, cdev_state)
		__field(u32, power)
	),

	TP_fast_assign(
		__entry->freq = freq;
		__entry->cdev_state = cdev_state;
		__entry->power = power;
	),

	TP_printk("freq=%u cdev_state=%lu power=%u",
		  __entry->freq, __entry->cdev_state,
		  __entry->power)
);

TRACE_EVENT(thermal_exynos_power_allocator,
	TP_PROTO(struct thermal_zone_device *tz, u32 power_range, u32 max_allocatable_power,
		 int current_temp, s32 delta_temp, unsigned long cdev_state, bool is_hardlimited),
	TP_ARGS(tz, power_range, max_allocatable_power,
		current_temp, delta_temp, cdev_state, is_hardlimited),
	TP_STRUCT__entry(
		__field(int, tz_id)
		__field(u32, power_range)
		__field(u32, max_allocatable_power)
		__field(int, current_temp)
		__field(s32, delta_temp)
		__field(unsigned long, cdev_state)
		__field(bool, is_hardlimited)
	),
	TP_fast_assign(
		__entry->tz_id = tz->id;
		__entry->power_range = power_range;
		__entry->max_allocatable_power = max_allocatable_power;
		__entry->current_temp = current_temp;
		__entry->delta_temp = delta_temp;
		__entry->cdev_state = cdev_state;
		__entry->is_hardlimited = is_hardlimited;
	),

	TP_printk("thermal_zone_id=%d power_range=%u max_allocatable_power=%u current_temperature=%d delta_temperature=%d cdev_state=%lu is_hardlimited=%d",
		  __entry->tz_id, __entry->power_range, __entry->max_allocatable_power,
		  __entry->current_temp, __entry->delta_temp,
		  __entry->cdev_state, __entry->is_hardlimited)
);

TRACE_EVENT(thermal_exynos_acpm_bulk,
	TP_PROTO(u8 tz_id, u8 current_temp, u8 ctrl_temp, u8 cdev_state, s32 pid_et_p, s16 pid_power_range, s16 pid_p, s32 pid_i, s32 k_p, s32 k_i, u64 timestamp),
	TP_ARGS(tz_id, current_temp, ctrl_temp, cdev_state, pid_et_p, pid_power_range, pid_p, pid_i, k_p, k_i, timestamp),
	TP_STRUCT__entry(
		__field(u8, tz_id)
		__field(u8, current_temp)
		__field(u8, ctrl_temp)
		__field(u8, cdev_state)
		__field(s32, pid_et_p)
		__field(s16, pid_power_range)
		__field(s16, pid_p)
		__field(s32, pid_i)
		__field(s32, k_p)
		__field(s32, k_i)
		__field(u64, timestamp)
	),
	TP_fast_assign(
		__entry->tz_id = tz_id;
		__entry->current_temp = current_temp;
		__entry->ctrl_temp = ctrl_temp;
		__entry->cdev_state = cdev_state;
		__entry->pid_et_p = pid_et_p;
		__entry->pid_power_range = pid_power_range;
		__entry->pid_p = pid_p;
		__entry->pid_i = pid_i;
		__entry->k_p = k_p;
		__entry->k_i = k_i;
		__entry->timestamp = timestamp;
	),

	TP_printk("thermal_zone_id=%u current_temperature=%u ctrl_temp=%u cdev_state=%u pid_et_p=%d pid_power_range=%d pid_p=%d pid_i=%d k_p=%d k_i=%d timestamp=%llu",
		  __entry->tz_id, __entry->current_temp, __entry->ctrl_temp, __entry->cdev_state, __entry->pid_et_p, __entry->pid_power_range, __entry->pid_p, __entry->pid_i, __entry->k_p, __entry->k_i, __entry->timestamp)
);

TRACE_EVENT(thermal_exynos_acpm_high_overhead,
	TP_PROTO(int tz_id, u8 current_temp, u8 ctrl_temp, u8 cdev_state, s32 pid_et_p, s32 k_p, s32 k_i),
	TP_ARGS(tz_id, current_temp, ctrl_temp, cdev_state, pid_et_p, k_p, k_i),
	TP_STRUCT__entry(
		__field(int, tz_id)
		__field(u8, current_temp)
		__field(u8, ctrl_temp)
		__field(u8, cdev_state)
		__field(s32, pid_et_p)
		__field(s32, k_p)
		__field(s32, k_i)
	),
	TP_fast_assign(
		__entry->tz_id = tz_id;
		__entry->current_temp = current_temp;
		__entry->ctrl_temp = ctrl_temp;
		__entry->cdev_state = cdev_state;
		__entry->pid_et_p = pid_et_p;
		__entry->k_p = k_p;
		__entry->k_i = k_i;
	),

	TP_printk("thermal_zone_id=%d current_temperature=%u ctrl_temp=%u cdev_state=%u pid_et_p=%d k_p=%d k_i=%d",
		  __entry->tz_id, __entry->current_temp, __entry->ctrl_temp, __entry->cdev_state, __entry->pid_et_p, __entry->k_p, __entry->k_i)
);

TRACE_EVENT(thermal_exynos_power_allocator_pid,
	TP_PROTO(struct thermal_zone_device *tz, s32 err, s32 err_integral,
		 s64 p, s64 i, s32 output),
	TP_ARGS(tz, err, err_integral, p, i, output),
	TP_STRUCT__entry(
		__field(int, tz_id)
		__field(s32, err)
		__field(s32, err_integral)
		__field(s64, p)
		__field(s64, i)
		__field(s32, output)
	),
	TP_fast_assign(
		__entry->tz_id = tz->id;
		__entry->err = err;
		__entry->err_integral = err_integral;
		__entry->p = p;
		__entry->i = i;
		__entry->output = output;
	),

	TP_printk("thermal_zone_id=%d err=%d err_integral=%d p=%lld i=%lld output=%d",
		  __entry->tz_id, __entry->err, __entry->err_integral,
		  __entry->p, __entry->i, __entry->output)
);

TRACE_EVENT(thermal_exynos_dus_bci_freq_update_request,
	TP_PROTO(int dsu_freq, int bci_freq),

	TP_ARGS(dsu_freq, bci_freq),

	TP_STRUCT__entry(
		__field(int, dsu_freq)
		__field(int, bci_freq)
	),

	TP_fast_assign(
		__entry->dsu_freq = dsu_freq;
		__entry->bci_freq = bci_freq;
	),

	TP_printk("dsu_freq=%d bci_freq=%d", __entry->dsu_freq, __entry->bci_freq)
);

TRACE_EVENT(thermal_cpu_pressure,
	TP_PROTO(unsigned long pressure, int cpu),

	TP_ARGS(pressure, cpu),

	TP_STRUCT__entry(
		__field(unsigned long, pressure)
		__field(int, cpu)
	),

	TP_fast_assign(
		__entry->pressure = pressure;
		__entry->cpu = cpu;
	),

	TP_printk("pressure=%lu cpu=%d", __entry->pressure, __entry->cpu)
);

TRACE_EVENT(vendor_cdev_update,
	TP_PROTO(const char *cdev_type, unsigned long sysfs_req, unsigned long state),

	TP_ARGS(cdev_type, sysfs_req, state),

	TP_STRUCT__entry(
		__field(const char *, cdev_type)
		__field(unsigned long, sysfs_req)
		__field(unsigned long, state)
	),

	TP_fast_assign(
		__entry->cdev_type = cdev_type;
		__entry->sysfs_req = sysfs_req;
		__entry->state = state;
	),

	TP_printk("cdev:%s sysfs_req=%lu, state=%lu", __entry->cdev_type,
					__entry->sysfs_req, __entry->state)
);

TRACE_EVENT(thermal_exynos_hard_limit_cdev_update,
	TP_PROTO(const char *tmu_name, const char *cdev_type, bool is_hardlimited,
		 bool pid_switch_on, unsigned long prev_max_state, unsigned long state),

	TP_ARGS(tmu_name, cdev_type, is_hardlimited, pid_switch_on, prev_max_state, state),

	TP_STRUCT__entry(
		__field(const char *, tmu_name)
		__field(const char *, cdev_type)
		__field(bool, is_hardlimited)
		__field(bool, pid_switch_on)
		__field(unsigned long, prev_max_state)
		__field(unsigned long, state)
	),

	TP_fast_assign(
		__entry->tmu_name = tmu_name;
		__entry->cdev_type = cdev_type;
		__entry->is_hardlimited = is_hardlimited;
		__entry->pid_switch_on = pid_switch_on;
		__entry->prev_max_state = prev_max_state;
		__entry->state = state;
	),

	TP_printk("tmu_name:%s cdev:%s, is_hardlimited=%d, pid_switch_on=%d, prev_max_state=%lu, state=%lu",
					__entry->tmu_name, __entry->cdev_type,
					__entry->is_hardlimited, __entry->pid_switch_on,
					__entry->prev_max_state, __entry->state)
);

TRACE_EVENT(thermal_exynos_tpu_pause,
	TP_PROTO(const char *tmu_name, bool is_paused),

	TP_ARGS(tmu_name, is_paused),

	TP_STRUCT__entry(
		__field(const char *, tmu_name)
		__field(bool, is_paused)
	),

	TP_fast_assign(
		__entry->tmu_name = tmu_name;
		__entry->is_paused = is_paused;
	),

	TP_printk("tmu_name:%s is_paused=%d", __entry->tmu_name, __entry->is_paused)
);

TRACE_EVENT(thermal_exynos_cpu_pause,
	TP_PROTO(const char *tmu_name, const struct cpumask *cpus, bool is_cpu_paused),

	TP_ARGS(tmu_name, cpus, is_cpu_paused),

	TP_STRUCT__entry(
		__field(const char *, tmu_name)
		__bitmask(cpumask, num_possible_cpus())
		__field(bool, is_cpu_paused)
	),

	TP_fast_assign(
		__entry->tmu_name = tmu_name;
		__assign_bitmask(cpumask, cpumask_bits(cpus),
				 num_possible_cpus());
		__entry->is_cpu_paused = is_cpu_paused;
	),

	TP_printk("tmu_name:%s cpus=%s, is_cpu_paused=%d", __entry->tmu_name,
					__get_bitmask(cpumask), __entry->is_cpu_paused)
);

TRACE_EVENT(thermal_exynos_arm_update,
	TP_PROTO(const char *tmu_name, bool is_cpu_hw_throttled,
		 int ppm_throttle_level, int ppm_clr_level,
		 int mpmm_throttle_level, int mpmm_clr_level),

	TP_ARGS(tmu_name, is_cpu_hw_throttled, ppm_throttle_level, ppm_clr_level,
		mpmm_throttle_level, mpmm_clr_level),

	TP_STRUCT__entry(
		__field(const char *, tmu_name)
		__field(bool, is_cpu_hw_throttled)
		__field(int, ppm_throttle_level)
		__field(int, ppm_clr_level)
		__field(int, mpmm_throttle_level)
		__field(int, mpmm_clr_level)
	),

	TP_fast_assign(
		__entry->tmu_name = tmu_name;
		__entry->is_cpu_hw_throttled = is_cpu_hw_throttled;
		__entry->ppm_throttle_level = ppm_throttle_level;
		__entry->ppm_clr_level = ppm_clr_level;
		__entry->mpmm_throttle_level = mpmm_throttle_level;
		__entry->mpmm_clr_level = mpmm_clr_level;
	),

	TP_printk("tmu_name:%s is_cpu_hw_throttled=%d, ppm_throttle_lvl=0x%x, ppm_clr_lvl=0x%x, mpmm_throttle_lvl=0x%x, mpmm_clr_lvl=0x%x",
					__entry->tmu_name, __entry->is_cpu_hw_throttled,
					__entry->ppm_throttle_level, __entry->ppm_clr_level,
					__entry->mpmm_throttle_level, __entry->mpmm_clr_level)
);

TRACE_EVENT(thermal_exynos_allow_max_power,
	TP_PROTO(const char *tmu_name, bool is_hardlimited, char *cdev_type, unsigned long state),

	TP_ARGS(tmu_name, is_hardlimited, cdev_type, state),

	TP_STRUCT__entry(
		__field(const char *, tmu_name)
		__field(bool, is_hardlimited)
		__field(const char *, cdev_type)
		__field(unsigned long, state)
	),

	TP_fast_assign(
		__entry->tmu_name = tmu_name;
		__entry->is_hardlimited = is_hardlimited;
		__entry->cdev_type = cdev_type;
		__entry->state = state;
	),

	TP_printk("tmu_name:%s, is_hardlimited=%d, cdev:%s, target=%lu",
					__entry->tmu_name, __entry->is_hardlimited,
					__entry->cdev_type, __entry->state)
);

#endif /* _TRACE_THERMAL_EXYNOS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
