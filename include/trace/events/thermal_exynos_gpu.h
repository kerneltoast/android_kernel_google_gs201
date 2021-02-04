/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM thermal_exynos_gpu

#if !defined(_TRACE_EXYNOS_GPU_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_THERMAL_EXYNOS_GPU_H

#include <linux/tracepoint.h>

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
#endif /* _TRACE_THERMAL_EXYNOS_GPU_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
