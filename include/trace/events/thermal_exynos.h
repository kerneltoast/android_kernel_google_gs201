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
	TP_PROTO(struct thermal_zone_device *tz,
		 u32 power_range, u32 max_allocatable_power,
		 int current_temp, s32 delta_temp),
	TP_ARGS(tz, power_range, max_allocatable_power,
		current_temp, delta_temp),
	TP_STRUCT__entry(
		__field(int, tz_id)
		__field(u32, power_range)
		__field(u32, max_allocatable_power)
		__field(int, current_temp)
		__field(s32, delta_temp)
	),
	TP_fast_assign(
		__entry->tz_id = tz->id;
		__entry->power_range = power_range;
		__entry->max_allocatable_power = max_allocatable_power;
		__entry->current_temp = current_temp;
		__entry->delta_temp = delta_temp;
	),

	TP_printk("thermal_zone_id=%d power_range=%u max_allocatable_power=%u current_temperature=%d delta_temperature=%d",
		  __entry->tz_id,
		  __entry->power_range, __entry->max_allocatable_power,
		  __entry->current_temp, __entry->delta_temp)
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
#endif /* _TRACE_THERMAL_EXYNOS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
