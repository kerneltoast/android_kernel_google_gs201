// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform PMU (PPMU) driver for perf_event
 *
 * Copyright (C) Google LLC, 2019.
 *    Author:  Namhyung Kim  <namhyung@google.com>
 *
 * Based on ARM DSU and Intel RAPL PMU driver.
 */

#define pr_fmt(fmt)	"ppmu: " fmt

#include <linux/bug.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/perf_event.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/cpumask.h>
#include <linux/platform_device.h>

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_PPMU)

#include <soc/google/exynos-bcm_dbg.h>
#include <soc/google/exynos-bcm_dbg-dump.h>

/* artificial event number for cycles */
#define PPMU_EVT_CYCLES  0xff

/* bitmask for actual hardware event number */
#define PPMU_EVT_MASK  0xff

/* bitmask for maximum operation for certain events */
#define PPMU_MAX_OP_MASK  0x100

/* bitmap index of the cycles event */
#define PPMU_CYCLES_IDX  31

/* number of (general) counters in a PPMU monitor */
#define PPMU_NUM_CNT  8

/* timer period of debug core (1ms) */
#define PPMU_TIMER_PERIOD  1000

/* start of event code using multi-bit inputs */
#define PPMU_MULTI_BIT_EVT  0x20

/* size of bitmap which should include 'cycles' counter */
#define PPMU_CNT_MASK_SIZE  32

struct ppmu_hw_config {
	struct perf_event		*events[PPMU_NUM_CNT];
	DECLARE_BITMAP(used_mask, PPMU_CNT_MASK_SIZE);
};

/*
 * struct platform_pmu - Platform PMU descriptor
 *
 * @pmu_lock		: Protects accesses to PPMU register.
 * @ip_index		: PPMU IP index
 * @active		: number of active events in the PPMU
 * @enabled		: PPMU is enabled for this IP
 */
struct platform_pmu {
	struct pmu			pmu;
	struct ppmu_hw_config		hw_config;
	raw_spinlock_t			pmu_lock;
	int				ip_index;
	int				active;
	bool				enabled;
};

struct ppmu_dump_format {
	struct exynos_bcm_dump_info		info;
	struct exynos_bcm_accumulator_data	data;
};

/*
 * struct ppmu_debug_core - Debug core controls all PPMUs
 *
 * @enabled		: debug core is enabled
 * @cpu			: cpu which has perf events actually
 * @events		: number of enabled events (in all PPMUs)
 * @active		: number of active events (in all PPMUs)
 * @read		: number of events read in this cycle (in all PPMUs)
 *
 * The debug core is to control and measure counters in PPMUs.
 * The @events keeps number of enabled events (by perf_event_open syscall)
 * so that it can turn off the debug core after all the events are gone.
 *
 * The @active and @read are to refresh counter values (i.e. dump the current
 * counters from PPMUs) in a batch to avoid unnecessary communications with
 * debug core.  Note that @active and @events can be different because of
 * multiplexing in the perf_event core (maybe due to hardware restriction).
 *
 * The @cpu tells which cpu is currently in charge of accessing PPMU (through
 * the debug core).  Since it's a global resource, we only need a single cpu
 * and it's the first cpu online.  It'll be changed as cpu hotplug manages the
 * online cpumask and perf_events will be migrated as well.
 */
struct ppmu_debug_core {
	struct device			*dev;
	struct exynos_bcm_dbg_data	*data;
	struct platform_pmu		**ppmu;
	raw_spinlock_t			lock;
	bool				enabled;
	int				cpu;
	int				events;
	atomic_t			active;
	atomic_t			read;
};

static struct ppmu_debug_core *ppmu_dbg_core;
static cpumask_t ppmu_cpu_mask;
static enum cpuhp_state dyn_hp_state = CPUHP_INVALID;

void exynos_bcm_dbg_set_base_info(struct exynos_bcm_ipc_base_info *ipc_base_info,
				  enum exynos_bcm_event_id event_id,
				  enum exynos_bcm_event_dir direction,
				  enum exynos_bcm_ip_range ip_range);

int exynos_bcm_dbg_run_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
			    unsigned int *bcm_ip_run,
			    struct exynos_bcm_dbg_data *data);

int exynos_bcm_dbg_period_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
			       unsigned int *bcm_period,
			       struct exynos_bcm_dbg_data *data);

int exynos_bcm_dbg_mode_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
			     unsigned int *bcm_mode,
			     struct exynos_bcm_dbg_data *data);

int exynos_bcm_dbg_ip_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
			   unsigned int *bcm_ip_enable,
			   unsigned int bcm_ip_index,
			   struct exynos_bcm_dbg_data *data);

int exynos_bcm_dbg_pause_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
			      unsigned int *bcm_ip_pause,
			      unsigned int bcm_ip_index,
			      struct exynos_bcm_dbg_data *data);

int exynos_bcm_dbg_event_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
			      struct exynos_bcm_event *bcm_event,
			      unsigned int bcm_ip_index,
			      struct exynos_bcm_dbg_data *data);

int exynos_bcm_dbg_sample_id_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
				  struct exynos_bcm_sample_id *sample_id,
				  unsigned int bcm_ip_index,
				  struct exynos_bcm_dbg_data *data);

int exynos_bcm_dbg_dump_accumulators_ctrl(
			struct exynos_bcm_ipc_base_info *ipc_base_info,
			char *buf, size_t *buf_len, loff_t off, size_t size,
			struct exynos_bcm_dbg_data *data);

void exynos_bcm_dbg_set_dump(bool enable_klog, bool enable_file,
			     struct exynos_bcm_dbg_data *data);

static inline struct platform_pmu *to_platform_pmu(struct pmu *pmu)
{
	return container_of(pmu, struct platform_pmu, pmu);
}

#define PPMU_EXT_ATTR(_name, _func, _config)		\
	(&((struct dev_ext_attribute[]) {				\
		{							\
			.attr = __ATTR(_name, 0444, _func, NULL),	\
			.var = (void *)_config				\
		}							\
	})[0].attr.attr)

#define PPMU_FORMAT_ATTR(_name, _config)		\
	PPMU_EXT_ATTR(_name, platform_pmu_sysfs_format_show, (char *)_config)

#define PPMU_EVENT_ATTR(_name, _config)		\
	PPMU_EXT_ATTR(_name, platform_pmu_sysfs_event_show, (unsigned long)_config)

static ssize_t platform_pmu_sysfs_format_show(struct device *dev,
                                              struct device_attribute *attr,
                                              char *buf)
{
	struct dev_ext_attribute *eattr = container_of(attr,
					struct dev_ext_attribute, attr);
	return snprintf(buf, PAGE_SIZE, "%s\n", (char *)eattr->var);
}

static struct attribute *platform_pmu_format_attrs[] = {
	PPMU_FORMAT_ATTR(event, "config:0-31"),
	NULL,
};

static const struct attribute_group platform_pmu_format_attr_group = {
	.name = "format",
	.attrs = platform_pmu_format_attrs,
};

static ssize_t platform_pmu_sysfs_event_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct dev_ext_attribute *eattr = container_of(attr,
					struct dev_ext_attribute, attr);
	return snprintf(buf, PAGE_SIZE, "event=0x%lx\n", (unsigned long)eattr->var);
}

static struct attribute *platform_pmu_event_attrs[] = {
	PPMU_EVENT_ATTR(rd-actv, 0x0),
	PPMU_EVENT_ATTR(wr-actv, 0x1),
	PPMU_EVENT_ATTR(rd-req,  0x2),
	PPMU_EVENT_ATTR(wr-req,  0x3),
	PPMU_EVENT_ATTR(rd-data, 0x4),
	PPMU_EVENT_ATTR(wr-data, 0x5),
	PPMU_EVENT_ATTR(wr-resp, 0x6),
	PPMU_EVENT_ATTR(rd-last, 0x7),
	PPMU_EVENT_ATTR(wr-last, 0x8),
	PPMU_EVENT_ATTR(rd-req-blk, 0x10),
	PPMU_EVENT_ATTR(wr-req-blk, 0x11),
	PPMU_EVENT_ATTR(rd-data-blk, 0x12),
	PPMU_EVENT_ATTR(wr-data-blk, 0x13),
	PPMU_EVENT_ATTR(wr-resp-blk, 0x14),
	PPMU_EVENT_ATTR(rd-req-fix, 0x15),
	PPMU_EVENT_ATTR(wr-req-fix, 0x16),
	PPMU_EVENT_ATTR(any-req-fix, 0x17),
	PPMU_EVENT_ATTR(rd-req-wrap2, 0x18),
	PPMU_EVENT_ATTR(wr-req-wrap2, 0x19),
	PPMU_EVENT_ATTR(any-req-wrap2, 0x1a),
	PPMU_EVENT_ATTR(rd-req-bst, 0x1b),
	PPMU_EVENT_ATTR(wr-req-bst, 0x1c),
	PPMU_EVENT_ATTR(any-req-bst, 0x1d),
	PPMU_EVENT_ATTR(rd-smpl, 0x1e),
	PPMU_EVENT_ATTR(wr-smpl, 0x1f),
	/* below are only available in PMCNT2-3 and 6-7 */
	PPMU_EVENT_ATTR(sum-rw-actv, 0x20),
	PPMU_EVENT_ATTR(sum-rw-req, 0x21),
	PPMU_EVENT_ATTR(sum-rw-data, 0x22),
	PPMU_EVENT_ATTR(sum-rw-req-blk, 0x23),
	PPMU_EVENT_ATTR(sum-rd-lat, 0x24),
	PPMU_EVENT_ATTR(sum-wr-lat, 0x25),
	PPMU_EVENT_ATTR(rd-smpl-lat, 0x26),
	PPMU_EVENT_ATTR(wr-smpl-lat, 0x27),
	PPMU_EVENT_ATTR(rd-stl, 0x28),
	PPMU_EVENT_ATTR(wr-stl, 0x29),
	/* VC-based events */
	PPMU_EVENT_ATTR(rd-req-vc0, 0x30),
	PPMU_EVENT_ATTR(rd-req-vc1, 0x31),
	PPMU_EVENT_ATTR(rd-req-vc2, 0x32),
	PPMU_EVENT_ATTR(rd-req-vc3, 0x33),
	PPMU_EVENT_ATTR(wr-req-vc0, 0x38),
	PPMU_EVENT_ATTR(wr-req-vc1, 0x39),
	PPMU_EVENT_ATTR(wr-req-vc2, 0x3a),
	PPMU_EVENT_ATTR(wr-req-vc3, 0x3b),
	PPMU_EVENT_ATTR(sum-rd-lat-vc0, 0x40),
	PPMU_EVENT_ATTR(sum-rd-lat-vc1, 0x41),
	PPMU_EVENT_ATTR(sum-rd-lat-vc2, 0x42),
	PPMU_EVENT_ATTR(sum-rd-lat-vc3, 0x43),
	PPMU_EVENT_ATTR(sum-wr-lat-vc0, 0x48),
	PPMU_EVENT_ATTR(sum-wr-lat-vc1, 0x49),
	PPMU_EVENT_ATTR(sum-wr-lat-vc2, 0x4a),
	PPMU_EVENT_ATTR(sum-wr-lat-vc3, 0x4b),
	/* add artificial max events */
	PPMU_EVENT_ATTR(max-rw-actv, 0x120),
	PPMU_EVENT_ATTR(max-rw-req, 0x121),
	PPMU_EVENT_ATTR(max-rw-data, 0x122),
	PPMU_EVENT_ATTR(max-rw-req-blk, 0x123),
	PPMU_EVENT_ATTR(max-rd-lat, 0x124),
	PPMU_EVENT_ATTR(max-wr-lat, 0x125),
	PPMU_EVENT_ATTR(rd-max-smpl-lat, 0x126),
	PPMU_EVENT_ATTR(wr-max-smpl-lat, 0x127),
	/* add artificial cycles event */
	PPMU_EVENT_ATTR(cycles, PPMU_EVT_CYCLES),
	NULL,
};

static const struct attribute_group platform_pmu_events_attr_group = {
	.name = "events",
	.attrs = platform_pmu_event_attrs,
};

static ssize_t cpumask_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return cpumap_print_to_pagebuf(true, buf, &ppmu_cpu_mask);
}

static DEVICE_ATTR_RO(cpumask);

static struct attribute *platform_pmu_attrs[] = {
	&dev_attr_cpumask.attr,
	NULL,
};

static struct attribute_group platform_pmu_attr_group = {
	.attrs = platform_pmu_attrs,
};

static const struct attribute_group *platform_pmu_attr_groups[] = {
	&platform_pmu_format_attr_group,
	&platform_pmu_events_attr_group,
	&platform_pmu_attr_group,
	NULL,
};

static inline bool platform_pmu_counter_valid(u32 idx)
{
	return (idx < PPMU_NUM_CNT) || (idx == PPMU_CYCLES_IDX);
}

static int platform_pmu_get_event_idx(struct ppmu_hw_config *hw_config,
				      struct perf_event *event)
{
	unsigned long *used_mask = hw_config->used_mask;
	int idx;

	if (event->attr.config == PPMU_EVT_CYCLES) {
		if (test_and_set_bit(PPMU_CYCLES_IDX, used_mask))
			return -EAGAIN;
		return PPMU_CYCLES_IDX;
	}

	idx = find_first_zero_bit(used_mask, PPMU_NUM_CNT);
	if (idx >= PPMU_NUM_CNT)
		return -EAGAIN;

	/* for single-bit event, try to keep PMCNT[23] for multi-bit events */
	if ((idx == 2 || idx == 3) && event->attr.config < PPMU_MULTI_BIT_EVT) {
		if (!test_bit(4, used_mask))
			idx = 4;
		else if (!test_bit(5, used_mask))
			idx = 5;
	}

	/* for multi-bit event, don't used PMCNT[0145] */
	if (((idx % 4) < 2) && event->attr.config >= PPMU_MULTI_BIT_EVT) {
		if (!test_bit(2, used_mask))
			idx = 2;
		else if (!test_bit(3, used_mask))
			idx = 3;
		else if (!test_bit(6, used_mask))
			idx = 6;
		else if (!test_bit(7, used_mask))
			idx = 7;
		else
			return -EAGAIN;
	}

	set_bit(idx, hw_config->used_mask);
	return idx;
}

static inline u64 platform_pmu_read_counter(struct perf_event *event)
{
	struct platform_pmu *ppmu = to_platform_pmu(event->pmu);
	struct ppmu_dump_format *dump_base;
	u64 val;

	if (!platform_pmu_counter_valid(event->hw.idx))
		return 0;

	if (ppmu_dbg_core->data->dump_addr.p_addr == 0)
		return 0;

	dump_base = (void *)(ppmu_dbg_core->data->dump_addr.v_addr +
			     EXYNOS_BCM_KTIME_SIZE);

	if (event->hw.idx == PPMU_CYCLES_IDX)
		val = dump_base[ppmu->ip_index].data.ccnt;
	else
		val = dump_base[ppmu->ip_index].data.pmcnt[event->hw.idx];

	return val;
}

static void platform_pmu_write_counter(struct perf_event *event, u64 val)
{
	/* NOT supported */
}

static void platform_pmu_run_control(unsigned enable)
{
	struct exynos_bcm_ipc_base_info ipc_base_info;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_RUN_CONT,
					BCM_EVT_SET, BCM_ALL);
	exynos_bcm_dbg_run_ctrl(&ipc_base_info, &enable, ppmu_dbg_core->data);

	ppmu_dbg_core->enabled = enable;
}

static void platform_pmu_mode_control(unsigned mode)
{
	struct exynos_bcm_ipc_base_info ipc_base_info;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_MODE_CONT,
					BCM_EVT_SET, BCM_EACH);
	exynos_bcm_dbg_mode_ctrl(&ipc_base_info, &mode, ppmu_dbg_core->data);
}

static void platform_pmu_period_control(unsigned period)
{
	struct exynos_bcm_ipc_base_info ipc_base_info;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_PERIOD_CONT,
					BCM_EVT_SET, BCM_ALL);
	exynos_bcm_dbg_period_ctrl(&ipc_base_info, &period, ppmu_dbg_core->data);
}

static void platform_pmu_ip_control(struct platform_pmu *ppmu, unsigned enable)
{
	struct exynos_bcm_ipc_base_info ipc_base_info;

	if (ppmu->enabled == enable)
		return;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_IP_CONT,
				     BCM_EVT_SET, BCM_EACH);
	exynos_bcm_dbg_ip_ctrl(&ipc_base_info, &enable, ppmu->ip_index,
			       ppmu_dbg_core->data);

	ppmu->enabled = enable;
}

static void platform_pmu_event_control(struct platform_pmu *ppmu)
{
	int i;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_event events = {
		.index = 0,
	};

	for (i = 0; i < PPMU_NUM_CNT; i++) {
		struct perf_event *evt = ppmu->hw_config.events[i];

		if (evt != NULL)
			events.event[i] = evt->attr.config & PPMU_EVT_MASK;
		else
			events.event[i] = BCM_EVT_INVALID;
	}

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT,
				     BCM_EVT_SET, BCM_EACH);
	exynos_bcm_dbg_event_ctrl(&ipc_base_info, &events, ppmu->ip_index,
				  ppmu_dbg_core->data);
}

static void platform_pmu_event_sample_control(struct platform_pmu *ppmu)
{
	int i;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_sample_id sample_id = {
		.peak_mask = 0,  /* no sample mask/ID */
	};

	for (i = 0; i < PPMU_NUM_CNT; i++) {
		struct perf_event *evt = ppmu->hw_config.events[i];

		if (evt != NULL && (evt->attr.config & PPMU_MAX_OP_MASK))
			sample_id.peak_enable[i] = 1;
	}

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_SAMPLE_ID,
				     BCM_EVT_SET, BCM_EACH);

	exynos_bcm_dbg_sample_id_ctrl(&ipc_base_info, &sample_id,
				      ppmu->ip_index, ppmu_dbg_core->data);
}

static void platform_pmu_dump_control(void)
{
	struct exynos_bcm_ipc_base_info ipc_base_info;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_DUMP_ACCUMULATORS,
				     BCM_EVT_SET, BCM_EACH);

	exynos_bcm_dbg_dump_accumulators_ctrl(&ipc_base_info, NULL, 0, 0, 0,
					      ppmu_dbg_core->data);
}

static void platform_pmu_reset(void)
{
	int i;
	struct exynos_bcm_dbg_data *data = ppmu_dbg_core->data;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_event events = {
		.index = data->default_define_event,
	};
	struct exynos_bcm_sample_id *sample_id;

	platform_pmu_run_control(false);
	platform_pmu_mode_control(BCM_MODE_INTERVAL);
	exynos_bcm_dbg_set_dump(false, true, data);

	for (i = 0; i < PPMU_NUM_CNT; i++)
		events.event[i] = data->define_event[events.index].event[i];

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_PRE_DEFINE,
				     BCM_EVT_SET, BCM_ALL);
	exynos_bcm_dbg_event_ctrl(&ipc_base_info, &events, 0, data);

	sample_id = &data->define_sample_id[data->default_define_event];
	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_SAMPLE_ID,
				     BCM_EVT_SET, BCM_ALL);
	exynos_bcm_dbg_sample_id_ctrl(&ipc_base_info, sample_id, 0, data);

	for (i = 0; i < data->bcm_ip_nr; i++) {
		platform_pmu_ip_control(ppmu_dbg_core->ppmu[i],
					ppmu_dbg_core->data->initial_run_ip[i]);
	}
}

static void platform_pmu_update_counter(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	u64 delta, prev_count, new_count;

	do {
		/* We may also be called from the irq handler */
		prev_count = local64_read(&hwc->prev_count);
		new_count = platform_pmu_read_counter(event);
	} while (local64_cmpxchg(&hwc->prev_count, prev_count, new_count) !=
			prev_count);
	delta = new_count - prev_count;
	local64_add(delta, &event->count);
}

static void platform_pmu_set_event_period(struct perf_event *event)
{
	/* NOT supported */
	platform_pmu_write_counter(event, 0);
}

/*
 * PMU interfaces
 */
static void platform_pmu_read(struct perf_event *event)
{
	int read_count = atomic_inc_return(&ppmu_dbg_core->read);

	/* update counter values for first event read */
	if (read_count == 1)
		platform_pmu_dump_control();

	/*
	 * after reading all events, we should fetch new counter values
	 * from debug core again.  So reset the 'read' count.
	 */
	if (read_count >= atomic_read(&ppmu_dbg_core->active))
		atomic_set(&ppmu_dbg_core->read, 0);

	platform_pmu_update_counter(event);
}

static void platform_pmu_start(struct perf_event *event, int pmu_flags)
{
	platform_pmu_set_event_period(event);
	event->hw.state = 0;
}

static void platform_pmu_stop(struct perf_event *event, int pmu_flags)
{
	if (event->hw.state & PERF_HES_STOPPED)
		return;
	event->hw.state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
}

static int platform_pmu_add(struct perf_event *event, int evflags)
{
	struct platform_pmu *ppmu = to_platform_pmu(event->pmu);
	struct ppmu_hw_config *hw_config = &ppmu->hw_config;
	struct hw_perf_event *hwc = &event->hw;
	unsigned long flags;
	int idx;

	raw_spin_lock_irqsave(&ppmu->pmu_lock, flags);
	idx = platform_pmu_get_event_idx(hw_config, event);
	if (idx < 0) {
		raw_spin_unlock_irqrestore(&ppmu->pmu_lock, flags);
		return idx;
	}

	atomic_inc(&ppmu_dbg_core->active);
	ppmu->active++;
	hwc->idx = idx;

	if (idx < PPMU_NUM_CNT)
		hw_config->events[idx] = event;

	raw_spin_unlock_irqrestore(&ppmu->pmu_lock, flags);

	hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;

	if (evflags & PERF_EF_START)
		platform_pmu_start(event, PERF_EF_RELOAD);

	perf_event_update_userpage(event);
	return 0;
}

static void platform_pmu_del(struct perf_event *event, int evflags)
{
	struct platform_pmu *ppmu = to_platform_pmu(event->pmu);
	struct ppmu_hw_config *hw_config = &ppmu->hw_config;
	struct hw_perf_event *hwc = &event->hw;
	unsigned long flags;
	int idx = hwc->idx;

	platform_pmu_stop(event, PERF_EF_UPDATE);

	raw_spin_lock_irqsave(&ppmu->pmu_lock, flags);
	atomic_dec(&ppmu_dbg_core->active);
	ppmu->active--;

	if (idx < PPMU_NUM_CNT)
		hw_config->events[idx] = NULL;

	clear_bit(idx, hw_config->used_mask);
	raw_spin_unlock_irqrestore(&ppmu->pmu_lock, flags);

	perf_event_update_userpage(event);
}

static void platform_pmu_enable(struct pmu *pmu)
{
	struct platform_pmu *ppmu = to_platform_pmu(pmu);
	unsigned long flags;
	int i;

	raw_spin_lock_irqsave(&ppmu->pmu_lock, flags);
	/* event setting might be changed, update it */
	platform_pmu_event_sample_control(ppmu);
	platform_pmu_event_control(ppmu);

	/* activate debug core if an event is gonna be used */
	if (ppmu->active == 1) {
		raw_spin_lock(&ppmu_dbg_core->lock);
		/*
		 * this is the first event in any of PPMUs
		 * turn on the debug core PPMU tracking.
		 */
		if (!ppmu_dbg_core->enabled) {
			for (i = 0; i < ppmu_dbg_core->data->bcm_ip_nr; i++) {
				/* all PPMUs are disabled by default */
				platform_pmu_ip_control(ppmu_dbg_core->ppmu[i],
							false);
			}
			/* disable dump to klog and file */
			exynos_bcm_dbg_set_dump(false, false,
						ppmu_dbg_core->data);

			platform_pmu_mode_control(BCM_MODE_ACCUMULATOR);
			platform_pmu_period_control(PPMU_TIMER_PERIOD);
			/* set ppmu_dbg_core->enabled */
			platform_pmu_run_control(true);
		}
		raw_spin_unlock(&ppmu_dbg_core->lock);

		/*
		 * enable the PPMU when it's accessed
		 * it'll be disabled only if all events are gone
		 */
		platform_pmu_ip_control(ppmu, true);
	}

	raw_spin_unlock_irqrestore(&ppmu->pmu_lock, flags);
}

static void platform_pmu_disable(struct pmu *pmu)
{
}

static bool platform_pmu_validate_event(struct pmu *pmu,
					struct ppmu_hw_config *hw_config,
					struct perf_event *event)
{
	if (is_software_event(event))
		return true;
	/* Reject groups spanning multiple HW PMUs. */
	if (event->pmu != pmu)
		return false;
	return platform_pmu_get_event_idx(hw_config, event) >= 0;
}

/*
 * Make sure the group of events can be scheduled at once
 * on the PMU.
 */
static bool platform_pmu_validate_group(struct perf_event *event)
{
	struct perf_event *sibling, *leader = event->group_leader;
	struct ppmu_hw_config fake_hw;

	if (event->group_leader == event)
		return true;

	memset(fake_hw.used_mask, 0, sizeof(fake_hw.used_mask));
	if (!platform_pmu_validate_event(event->pmu, &fake_hw, leader))
		return false;
	for_each_sibling_event(sibling, leader) {
		if (!platform_pmu_validate_event(event->pmu, &fake_hw, sibling))
			return false;
	}
	return platform_pmu_validate_event(event->pmu, &fake_hw, event);
}

static void platform_pmu_event_destroy(struct perf_event *event)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&ppmu_dbg_core->lock, flags);
	if (--ppmu_dbg_core->events == 0)
		platform_pmu_reset();
	raw_spin_unlock_irqrestore(&ppmu_dbg_core->lock, flags);
}

static int platform_pmu_event_init(struct perf_event *event)
{
	struct platform_pmu *ppmu = to_platform_pmu(event->pmu);
	unsigned long flags;

	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	if (event->cpu < 0) {
		dev_dbg(ppmu->pmu.dev, "Can't support per-task counters\n");
		return -EINVAL;
	}

	if (has_branch_stack(event) ||
	    event->attr.exclude_user ||
	    event->attr.exclude_kernel ||
	    event->attr.exclude_hv ||
	    event->attr.exclude_idle ||
	    event->attr.exclude_host ||
	    event->attr.exclude_guest) {
		dev_dbg(ppmu->pmu.dev, "Can't support filtering\n");
		return -EINVAL;
	}

	event->cpu = ppmu_dbg_core->cpu;

	raw_spin_lock_irqsave(&ppmu->pmu_lock, flags);
	if (!platform_pmu_validate_group(event)) {
		raw_spin_unlock_irqrestore(&ppmu->pmu_lock, flags);
		return -EINVAL;
	}

	event->hw.config = event->attr.config;
	event->destroy = platform_pmu_event_destroy;

	raw_spin_lock(&ppmu_dbg_core->lock);
	ppmu_dbg_core->events++;
	raw_spin_unlock(&ppmu_dbg_core->lock);

	raw_spin_unlock_irqrestore(&ppmu->pmu_lock, flags);
	return 0;
}

static int platform_pmu_cpu_offline(unsigned int cpu)
{
	int target;
	int i;

	/* Check if exiting cpu is used for collecting PPMU events */
	if (!cpumask_test_and_clear_cpu(cpu, &ppmu_cpu_mask))
		return 0;

	ppmu_dbg_core->cpu = -1;
	/* Find a new cpu to collect ppmu events */
	target = cpumask_any_but(cpu_online_mask, cpu);

	/* Migrate ppmu events to the new target */
	if (target < nr_cpu_ids) {
		cpumask_set_cpu(target, &ppmu_cpu_mask);
		ppmu_dbg_core->cpu = target;

		for (i = 0; i < ppmu_dbg_core->data->bcm_ip_nr; i++) {
			perf_pmu_migrate_context(&ppmu_dbg_core->ppmu[i]->pmu,
						 cpu, target);
		}
	}
	return 0;
}

static int platform_pmu_cpu_online(unsigned int cpu)
{
	int target;

	/*
	 * Check if there is an online cpu which collects PPMU events already.
	 */
	target = cpumask_any_and(&ppmu_cpu_mask, cpu_online_mask);
	if (target < nr_cpu_ids)
		return 0;

	cpumask_set_cpu(cpu, &ppmu_cpu_mask);
	ppmu_dbg_core->cpu = cpu;
	return 0;
}

/*
 * platform_pmu_probe_pmu: Probe the PMU details for each IP.
 */
static int platform_pmu_probe_pmu(struct platform_device *pdev)
{
	size_t ppmu_ptr_sz = sizeof(*ppmu_dbg_core->ppmu);

	ppmu_dbg_core = devm_kzalloc(&pdev->dev, sizeof(*ppmu_dbg_core),
				     GFP_KERNEL);
	if (ppmu_dbg_core == NULL)
		return -ENOMEM;

	ppmu_dbg_core->dev = &pdev->dev;
	ppmu_dbg_core->data = platform_get_drvdata(pdev);
	raw_spin_lock_init(&ppmu_dbg_core->lock);

	ppmu_ptr_sz *= ppmu_dbg_core->data->bcm_ip_nr;
	ppmu_dbg_core->ppmu = devm_kzalloc(&pdev->dev, ppmu_ptr_sz, GFP_KERNEL);
	if (ppmu_dbg_core->ppmu == NULL)
		return -ENOMEM;

	atomic_set(&ppmu_dbg_core->active, 0);
	atomic_set(&ppmu_dbg_core->read, 0);

	return 0;
}

static struct platform_pmu *platform_pmu_alloc(void)
{
	struct platform_pmu *ppmu;

	ppmu = devm_kzalloc(ppmu_dbg_core->dev, sizeof(*ppmu), GFP_KERNEL);
	if (!ppmu)
		return ERR_PTR(-ENOMEM);

	raw_spin_lock_init(&ppmu->pmu_lock);

	return ppmu;
}

static void platform_pmu_free(struct platform_pmu *ppmu)
{
        devm_kfree(ppmu_dbg_core->dev, ppmu);
}

static int platform_pmu_init_pmu(struct platform_pmu *ppmu, const char *name)
{
	ppmu->pmu = (struct pmu) {
		.module		= THIS_MODULE,
		.name		= name,
		.capabilities	= PERF_PMU_CAP_NO_INTERRUPT,
		.task_ctx_nr	= perf_invalid_context,
		.pmu_enable	= platform_pmu_enable,
		.pmu_disable	= platform_pmu_disable,
		.event_init	= platform_pmu_event_init,
		.add		= platform_pmu_add,
		.del		= platform_pmu_del,
		.start		= platform_pmu_start,
		.stop		= platform_pmu_stop,
		.read		= platform_pmu_read,
		.attr_groups	= platform_pmu_attr_groups,
	};

	return perf_pmu_register(&ppmu->pmu, ppmu->pmu.name, -1);
}

int exynos_bcm_dbg_ppmu_init(struct platform_device *pdev)
{
        int i, ret;
	struct platform_pmu *ppmu;

	if (ppmu_dbg_core != NULL)
		return -EBUSY;

	ret = platform_pmu_probe_pmu(pdev);
	if (ret < 0)
		return ret;

	for (i = 0; i < ppmu_dbg_core->data->bcm_ip_nr; i++) {
		ppmu = platform_pmu_alloc();
		if (IS_ERR(ppmu)) {
			ret = PTR_ERR(ppmu);
			goto err;
		}

		ret = platform_pmu_init_pmu(ppmu,
				ppmu_dbg_core->data->bcm_ip_names[i]);
		if (ret < 0) {
			platform_pmu_free(ppmu);
			goto err;
		}

		ppmu->ip_index = i;
		ppmu_dbg_core->ppmu[i] = ppmu;
	}

	/*
	 * Install callbacks. Core will call them for each online cpu.
	 */
	dyn_hp_state = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
				"perf/arm64/ppmu:online",
				platform_pmu_cpu_online,
				platform_pmu_cpu_offline);
	if (dyn_hp_state >= 0)
		return 0;

err:
	while (--i >= 0) {
		perf_pmu_unregister(&ppmu_dbg_core->ppmu[i]->pmu);
		platform_pmu_free(ppmu_dbg_core->ppmu[i]);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_bcm_dbg_ppmu_init);

void exynos_bcm_dbg_ppmu_exit(struct platform_device *pdev)
{
	int i;

	if (dyn_hp_state >= 0) {
		cpuhp_remove_state_nocalls(dyn_hp_state);
		dyn_hp_state = CPUHP_INVALID;
	}

	for (i = 0; i < ppmu_dbg_core->data->bcm_ip_nr; i++) {
		perf_pmu_unregister(&ppmu_dbg_core->ppmu[i]->pmu);
		platform_pmu_free(ppmu_dbg_core->ppmu[i]);
	}

	devm_kfree(ppmu_dbg_core->dev, ppmu_dbg_core->ppmu);
	devm_kfree(ppmu_dbg_core->dev, ppmu_dbg_core);
	ppmu_dbg_core = NULL;
}
EXPORT_SYMBOL_GPL(exynos_bcm_dbg_ppmu_exit);

#endif // IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_PPMU)

MODULE_DESCRIPTION("Perf driver for Platform PMU");
MODULE_AUTHOR("Namhyung Kim <namhyung@google.com>");
MODULE_LICENSE("GPL v2");
