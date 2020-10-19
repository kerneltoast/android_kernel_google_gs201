// SPDX-License-Identifier: GPL-2.0-only
/*
 * GS101 SoC CPU Power Management driver
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * GS101 based board files should include this file.
 *
 */

#include <linux/cpumask.h>
#include <linux/slab.h>
#include <linux/tick.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/cpu_pm.h>
#include <linux/cpuidle.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <soc/google/exynos-cpupm.h>
#include <soc/google/cal-if.h>
#include <soc/google/exynos-pmu-if.h>
#include <soc/google/debug-snapshot.h>

/* Length of power mode name */
#define NAME_LEN	32

/* CPUPM statistics */
struct cpupm_stats {
	/* count of power mode entry */
	unsigned int entry_count;

	/* count of power mode entry cancllations */
	unsigned int cancel_count;

	/* power mode residency time */
	s64 residency_time;

	/* time entered power mode */
	ktime_t entry_time;
};

struct wakeup_mask {
	int mask_reg_offset;
	int stat_reg_offset;
	int mask;
};

struct wakeup_mask_config {
	int num_wakeup_mask;
	struct wakeup_mask *wakeup_masks;

	int num_eint_wakeup_mask;
	int *eint_wakeup_mask_reg_offset;
};

/*
 * Power modes
 * In CPUPM, power mode controls the power domain consisting of cpu and enters
 * the power mode by cpuidle. Basically, to enter power mode, all cpus in power
 * domain must be in POWERDOWN state, and sleep length of cpus must be smaller
 * than target_residency.
 */
struct power_mode {
	/* name of power mode, it is declared in device tree */
	char		name[NAME_LEN];

	/* power mode state, RUN or POWERDOWN */
	int		state;

	/* sleep length criterion of cpus to enter power mode */
	int		target_residency;

	/* type according to h/w configuration of power domain */
	int		type;

	/* index of cal, for POWERMODE_TYPE_CLUSTER */
	int		cal_id;

	/* cpus belonging to the power domain */
	struct cpumask	siblings;

	/*
	 * Among siblings, the cpus that can enter the power mode.
	 * Due to H/W constraint, only certain cpus need to enter power mode.
	 */
	struct cpumask	entry_allowed;

	/* disable count */
	atomic_t	disable;

	/*
	 * device attribute for sysfs,
	 * it supports for enabling or disabling this power mode
	 */
	struct device_attribute	attr;

	/* user's request for enabling/disabling power mode */
	bool		user_request;

	/* list of power mode */
	struct list_head	list;

	/* CPUPM statistics */
	struct cpupm_stats	stat;
	struct cpupm_stats	stat_snapshot;

	/* wakeup mask configuration for system idle */
	struct wakeup_mask_config *wm_config;
};

static LIST_HEAD(mode_list);

/*
 * Main struct of CPUPM
 * Each cpu has its own data structure and main purpose of this struct is to
 * manage the state of the cpu and the power modes containing the cpu.
 */
struct exynos_cpupm {
	/* cpu state, RUN or POWERDOWN */
	int			state;

	/* array to manage the power mode that contains the cpu */
	struct power_mode	*modes[POWERMODE_TYPE_END];
};

static struct exynos_cpupm __percpu *cpupm;
static bool __percpu *hotplug_ing;
static int system_suspended;

#define NSCODE_BASE		(0xBFFFF000)
#define CPU_STATE_BASE_OFFSET	0x2C
#define CPU_STATE_OFFSET(i)	(CPU_STATE_BASE_OFFSET + ((i) * 0x4))
#define CPU_STATE_SPARE_OFFSET	(0x54)
#define CANCEL_FLAG		BIT(5)

static void __iomem *nscode_base;

static void do_nothing(void *unused) { }

/******************************************************************************
 *                                    Notifier                                *
 ******************************************************************************/
static DEFINE_RWLOCK(notifier_lock);
static RAW_NOTIFIER_HEAD(notifier_chain);

int exynos_cpupm_notifier_register(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	write_lock_irqsave(&notifier_lock, flags);
	ret = raw_notifier_chain_register(&notifier_chain, nb);
	write_unlock_irqrestore(&notifier_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_cpupm_notifier_register);

static int exynos_cpupm_notify(int event, int v)
{
	int nr_calls;
	int ret = 0;

	read_lock(&notifier_lock);
	ret = __raw_notifier_call_chain(&notifier_chain, event, &v, -1, &nr_calls);
	read_unlock(&notifier_lock);

	return notifier_to_errno(ret);
}

/******************************************************************************
 *                                  IDLE_IP                                   *
 ******************************************************************************/
struct idle_ip {
	/* list of idle-ip */
	struct list_head	list;

	/* name of idle-ip */
	const char		*name;

	/* identity of idle-ip */
	unsigned int		index;

	/* non-idle count for cpuidle-profiler */
	unsigned int		count;
	unsigned int		count_profile;

	/* pmu offset for fixed idle-ip */
	unsigned int		pmu_offset;
};

#define PMU_IDLE_IP(x)			(0x03E0 + (0x4 * (x)))
#define IDLE_IP_MAX			64
#define FIX_IDLE_IP_MAX			4

static DEFINE_SPINLOCK(idle_ip_list_lock);
static DEFINE_SPINLOCK(idle_ip_bf_lock);
static LIST_HEAD(idle_ip_list);
static LIST_HEAD(fix_idle_ip_list);
static u32 idle_ip_check_count;
static u32 idle_ip_check_count_profile;
static u64 idle_ip_bf;
static u32 fix_idle_ip_bf;
static u32 idle_ip_count;
static u32 fix_idle_ip_count;

static bool check_fix_idle_ip(void)
{
	struct idle_ip *ip;

	fix_idle_ip_bf = 0;

	list_for_each_entry(ip, &fix_idle_ip_list, list) {
		unsigned int val;

		/*
		 * Each fix idle-ip use each IDLEIP register exclusivly.
		 * And they mark their idle state at last bit.
		 * ( 0 == non-idle, 1 == idle)
		 */
		exynos_pmu_read(ip->pmu_offset, &val);
		if (~val & 0x1)
			fix_idle_ip_bf |= BIT(ip->index);
	}

	if (fix_idle_ip_bf)
		return true;

	return false;
}

static void cpupm_profile_idle_ip(u64 idle_ip_bit_filed);
static bool check_idle_ip(void)
{
	unsigned long flags;

	spin_lock_irqsave(&idle_ip_bf_lock, flags);
	if (idle_ip_bf) {
		cpupm_profile_idle_ip(idle_ip_bf);
		spin_unlock_irqrestore(&idle_ip_bf_lock, flags);
		return true;
	}
	spin_unlock_irqrestore(&idle_ip_bf_lock, flags);

	return false;
}

/*
 * @index: index of idle-ip
 * @idle: idle status, (idle == 0)non-idle or (idle == 1)idle
 */
void exynos_update_ip_idle_status(int index, int idle)
{
	unsigned long flags;

	if (index < 0 || index > idle_ip_count)
		return;

	spin_lock_irqsave(&idle_ip_bf_lock, flags);
	if (idle)
		idle_ip_bf &= ~(BIT_ULL(index));
	else
		idle_ip_bf |= (BIT_ULL(index));
	spin_unlock_irqrestore(&idle_ip_bf_lock, flags);
}
EXPORT_SYMBOL_GPL(exynos_update_ip_idle_status);

/*
 * register idle-ip dynamically by name, return idle-ip index.
 */
int exynos_get_idle_ip_index(const char *name)
{
	struct idle_ip *ip;
	unsigned long flags;

	ip = kzalloc(sizeof(*ip), GFP_KERNEL);
	if (!ip)
		return -ENOMEM;

	spin_lock_irqsave(&idle_ip_list_lock, flags);

	if (idle_ip_count >= IDLE_IP_MAX) {
		spin_unlock_irqrestore(&idle_ip_list_lock, flags);
		pr_err("Up to 64 idle-ip can be supported. [failed %s].", name);
		goto free;
	}

	ip->name = name;
	ip->index = idle_ip_count++;
	list_add_tail(&ip->list, &idle_ip_list);

	spin_unlock_irqrestore(&idle_ip_list_lock, flags);

	exynos_update_ip_idle_status(ip->index, 0);

	return ip->index;
free:
	kfree(ip);
	return -ENOSPC;
}
EXPORT_SYMBOL_GPL(exynos_get_idle_ip_index);

/******************************************************************************
 *                               CPUPM profiler                               *
 ******************************************************************************/
static ktime_t cpupm_init_time;
static void cpupm_profile_begin(struct cpupm_stats *stat)
{
	stat->entry_time = ktime_get();
	stat->entry_count++;
}

static void cpupm_profile_end(struct cpupm_stats *stat, int cancel)
{
	if (!stat->entry_time)
		return;

	if (cancel) {
		stat->cancel_count++;
		return;
	}

	stat->residency_time +=
		ktime_to_ms(ktime_sub(ktime_get(), stat->entry_time));
	stat->entry_time = 0;
}

static void cpupm_profile_idle_ip(u64 idle_ip_bit_filed)
{
	struct idle_ip *ip;

	idle_ip_check_count++;
	list_for_each_entry(ip, &idle_ip_list, list) {
		/*
		 * Profile non-idle IP using @idle_ip_bit_filed
		 *
		 * bit == 0, it means idle.
		 * bit == 1, it means non-idle.
		 */
		if (idle_ip_bit_filed & (BIT_ULL(ip->index)))
			ip->count++;
	}
}

static int profiling;
static ktime_t profile_time;

static ssize_t profile_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct power_mode *mode;
	struct idle_ip *ip;
	s64 total;
	int ret = 0;

	if (profiling)
		return snprintf(buf, PAGE_SIZE, "Profile is ongoing\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"format : [mode] [entry_count] [cancel_count] [time] [(ratio)]\n\n");

	total = ktime_to_ms(profile_time);
	list_for_each_entry(mode, &mode_list, list) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%s %d %d %lld (%d%%)\n",
				mode->name,
				mode->stat_snapshot.entry_count,
				mode->stat_snapshot.cancel_count,
				mode->stat_snapshot.residency_time,
				mode->stat_snapshot.residency_time * 100 / total);
	}

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "IDLE-IP statstics\n");
	list_for_each_entry(ip, &idle_ip_list, list)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "* %-20s: block %d/%d\n",
				ip->name, ip->count_profile, idle_ip_check_count_profile);
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "(total %lldms)\n", total);

	return ret;
}

static ssize_t profile_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct power_mode *mode;
	struct idle_ip *ip;
	int input;

	if (kstrtoint(buf, 0, &input))
		return -EINVAL;

	input = !!input;
	if (profiling == input)
		return count;

	profiling = input;

	if (!input)
		goto stop_profile;

	preempt_disable();
	smp_call_function(do_nothing, NULL, 1);
	preempt_enable();

	list_for_each_entry(mode, &mode_list, list)
		mode->stat_snapshot = mode->stat;

	list_for_each_entry(ip, &idle_ip_list, list)
		ip->count_profile = ip->count;

	profile_time = ktime_get();
	idle_ip_check_count_profile = idle_ip_check_count;

	return count;

stop_profile:
#define delta(a, b)	(a = (b) - a)
#define field_delta(field)			\
	delta(mode->stat_snapshot.field, mode->stat.field)

	preempt_disable();
	smp_call_function(do_nothing, NULL, 1);
	preempt_enable();

	list_for_each_entry(mode, &mode_list, list) {
		field_delta(entry_count);
		field_delta(cancel_count);
		field_delta(residency_time);
	}

	list_for_each_entry(ip, &idle_ip_list, list)
		ip->count_profile = ip->count - ip->count_profile;

	profile_time = ktime_sub(ktime_get(), profile_time);
	idle_ip_check_count_profile = idle_ip_check_count - idle_ip_check_count_profile;

	return count;
}

static ssize_t time_in_state_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct power_mode *mode;
	struct idle_ip *ip;
	s64 total = ktime_to_ms(ktime_sub(ktime_get(), cpupm_init_time));
	int ret = 0;

	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"format : [mode] [entry_count] [cancel_count] [time] [(ratio)]\n\n");

	list_for_each_entry(mode, &mode_list, list) {
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"%s %d %d %lld (%d%%)\n",
				mode->name,
				mode->stat.entry_count,
				mode->stat.cancel_count,
				mode->stat.residency_time,
				mode->stat.residency_time * 100 / total);
	}

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");
	ret += snprintf(buf + ret, PAGE_SIZE - ret,
			"IDLE-IP statstics\n");
	list_for_each_entry(ip, &idle_ip_list, list)
		ret += snprintf(buf + ret, PAGE_SIZE - ret,
				"* %-20s: block %d/%d\n",
				ip->name, ip->count, idle_ip_check_count);
	ret += snprintf(buf + ret, PAGE_SIZE - ret, "\n");

	ret += snprintf(buf + ret, PAGE_SIZE - ret, "(total %lldms)\n", total);

	return ret;
}

static DEVICE_ATTR_RO(time_in_state);
static DEVICE_ATTR_RW(profile);

/******************************************************************************
 *                            CPU idle management                             *
 ******************************************************************************/
/*
 * State of CPUPM objects
 * All CPUPM objects have 2 states, RUN and POWERDOWN.
 *
 * @RUN
 * a state in which the power domain referred to by the object is turned on.
 *
 * @POWERDOWN
 * a state in which the power domain referred to by the object is turned off.
 * However, the power domain is not necessarily turned off even if the object
 * is in POEWRDOWN state because the cpu may be booting or executing power off
 * sequence.
 */
enum {
	CPUPM_STATE_RUN = 0,
	CPUPM_STATE_POWERDOWN,
};

/* Macros for CPUPM state */
#define set_state_run(object)		((object)->state = CPUPM_STATE_RUN)
#define set_state_powerdown(object)	((object)->state = CPUPM_STATE_POWERDOWN)
#define check_state_run(object)		((object)->state == CPUPM_STATE_RUN)
#define check_state_powerdown(object)	((object)->state == CPUPM_STATE_POWERDOWN)
#define is_valid_powermode(type)	((type >= 0) && (type < POWERMODE_TYPE_END))
/*
 * State of each cpu is managed by a structure declared by percpu, so there
 * is no need for protection for synchronization. However, when entering
 * the power mode, it is necessary to set the critical section to check the
 * state of cpus in the power domain, cpupm_lock is used for it.
 */
static spinlock_t cpupm_lock;

static void awake_cpus(const struct cpumask *cpus)
{
	int cpu;

	for_each_cpu_and(cpu, cpus, cpu_online_mask)
		smp_call_function_single(cpu, do_nothing, NULL, 1);
}

/*
 * disable_power_mode/enable_power_mode
 * It provides "disable" function to enable/disable power mode as required by
 * user or device driver. To handle multiple disable requests, it use the
 * atomic disable count, and disable the mode that contains the given cpu.
 */
void disable_power_mode(int cpu, int type)
{
	struct exynos_cpupm *pm;
	struct power_mode *mode;

	if (!is_valid_powermode(type))
		return;

	spin_lock(&cpupm_lock);

	pm = per_cpu_ptr(cpupm, cpu);
	mode = pm->modes[type];
	if (!mode) {
		spin_unlock(&cpupm_lock);
		return;
	}

	/*
	 * There are no entry allowed cpus, it means that mode is
	 * disabled, skip awaking cpus.
	 */
	if (cpumask_empty(&mode->entry_allowed)) {
		spin_unlock(&cpupm_lock);
		return;
	}

	/*
	 * The first mode disable request wakes the cpus to
	 * exit power mode
	 */
	if (atomic_inc_return(&mode->disable) > 0) {
		spin_unlock(&cpupm_lock);
		awake_cpus(&mode->siblings);
		return;
	}

	spin_unlock(&cpupm_lock);
}
EXPORT_SYMBOL_GPL(disable_power_mode);

void enable_power_mode(int cpu, int type)
{
	struct exynos_cpupm *pm;
	struct power_mode *mode;

	if (!is_valid_powermode(type))
		return;

	spin_lock(&cpupm_lock);

	pm = per_cpu_ptr(cpupm, cpu);
	mode = pm->modes[type];
	if (!mode) {
		spin_unlock(&cpupm_lock);
		return;
	}

	atomic_dec(&mode->disable);
	spin_unlock(&cpupm_lock);

	awake_cpus(&mode->siblings);
}
EXPORT_SYMBOL_GPL(enable_power_mode);

static int cpus_busy(int target_residency, const struct cpumask *cpus)
{
	int cpu;

	/*
	 * If there is even one cpu which is not in POWERDOWN state or has
	 * the smaller sleep length than target_residency, CPUPM regards
	 * it as BUSY.
	 */
	for_each_cpu_and(cpu, cpu_online_mask, cpus) {
		struct exynos_cpupm *pm = per_cpu_ptr(cpupm, cpu);

		if (check_state_run(pm))
			return -EBUSY;
	}

	return 0;
}

static int cpus_last_core_detecting(int request_cpu, const struct cpumask *cpus)
{
	int cpu;

	for_each_cpu_and(cpu, cpu_online_mask, cpus) {
		if (cpu == request_cpu)
			continue;

		if (cal_is_lastcore_detecting(cpu))
			return -EBUSY;
	}

	return 0;
}

#define BOOT_CPU	0
static int check_cluster_run(void)
{
	int cpu;
	struct power_mode *mode = NULL;

	for_each_online_cpu(cpu) {
		if (mode && cpumask_test_cpu(cpu, &mode->siblings))
			continue;

		mode = per_cpu_ptr(cpupm, cpu)->modes[POWERMODE_TYPE_CLUSTER];
		if (!mode)
			continue;

		if (cpumask_test_cpu(BOOT_CPU, &mode->siblings))
			continue;

		if (check_state_run(mode))
			return 1;
	}

	return 0;
}

static bool system_busy(struct power_mode *mode)
{
	if (mode->type != POWERMODE_TYPE_SYSTEM)
		return false;

	if (check_cluster_run())
		return true;

	if (check_idle_ip())
		return true;

	if (check_fix_idle_ip())
		return true;

	return false;
}

/*
 * In order to enter the power mode, the following conditions must be met:
 * 1. power mode should not be disabled
 * 2. the cpu attempting to enter must be a cpu that is allowed to enter the
 *    power mode.
 * 3. all cpus in the power domain must be in POWERDOWN state and the sleep
 *    length of the cpus must be less than target_residency.
 */
static bool entry_allow(int cpu, struct power_mode *mode)
{
	if (atomic_read(&mode->disable))
		return false;

	if (!cpumask_test_cpu(cpu, &mode->entry_allowed))
		return false;

	if (cpus_busy(mode->target_residency, &mode->siblings))
		return false;

	if (cpus_last_core_detecting(cpu, &mode->siblings))
		return false;

	if (system_busy(mode))
		return false;

	return true;
}

extern u32 exynos_eint_wake_mask_array[3];
static void set_wakeup_mask(struct wakeup_mask_config *wm_config)
{
	int i;

	if (!wm_config) {
		WARN_ONCE(1, "no wakeup mask information\n");
		return;
	}

	for (i = 0; i < wm_config->num_wakeup_mask; i++) {
		exynos_pmu_write(wm_config->wakeup_masks[i].stat_reg_offset, 0);
		exynos_pmu_write(wm_config->wakeup_masks[i].mask_reg_offset,
				 wm_config->wakeup_masks[i].mask);
	}

	for (i = 0; i < wm_config->num_eint_wakeup_mask; i++)
		exynos_pmu_write(wm_config->eint_wakeup_mask_reg_offset[i],
				 exynos_eint_wake_mask_array[i]);
}

static bool system_disabled;
static void enter_power_mode(int cpu, struct power_mode *mode)
{
	/*
	 * From this point on, it has succeeded in entering the power mode.
	 * It prepares to enter power mode, and makes the corresponding
	 * setting according to type of power mode.
	 */
	switch (mode->type) {
	case POWERMODE_TYPE_CLUSTER:
		cal_cluster_disable(mode->cal_id);
		break;
	case POWERMODE_TYPE_SYSTEM:
		if (system_disabled)
			return;
		if (unlikely(exynos_cpupm_notify(SICD_ENTER, 0)))
			return;

		cal_pm_enter(SYS_SICD);
		set_wakeup_mask(mode->wm_config);
		system_disabled = 1;

		break;
	}

	dbg_snapshot_cpuidle_mod(mode->name, 0, 0, DSS_FLAG_IN);
	set_state_powerdown(mode);

	cpupm_profile_begin(&mode->stat);
}

static void exit_power_mode(int cpu, struct power_mode *mode, int cancel)
{
	cpupm_profile_end(&mode->stat, cancel);

	/*
	 * Configure settings to exit power mode. This is executed by the
	 * first cpu exiting from power mode.
	 */
	set_state_run(mode);
	dbg_snapshot_cpuidle_mod(mode->name, 0, 0, DSS_FLAG_OUT);

	switch (mode->type) {
	case POWERMODE_TYPE_CLUSTER:
		cal_cluster_enable(mode->cal_id);
		break;
	case POWERMODE_TYPE_SYSTEM:
		if (!system_disabled)
			return;

		if (cancel)
			cal_pm_earlywakeup(SYS_SICD);
		else
			cal_pm_exit(SYS_SICD);
		exynos_cpupm_notify(SICD_EXIT, cancel);
		system_disabled = 0;

		break;
	}
}

/*
 * exynos_cpu_pm_enter() and exynos_cpu_pm_exit() called by cpu_pm_notify
 * to handle platform specific configuration to control cpu power domain.
 */
static void exynos_cpupm_enter(int cpu)
{
	struct exynos_cpupm *pm;
	int i;

	spin_lock(&cpupm_lock);
	pm = per_cpu_ptr(cpupm, cpu);

	/* Configure PMUCAL to power down core */
	cal_cpu_disable(cpu);

	/* Set cpu state to POWERDOWN */
	set_state_powerdown(pm);

	/* Try to enter power mode */
	for (i = 0; i < POWERMODE_TYPE_END; i++) {
		struct power_mode *mode = pm->modes[i];

		if (!mode)
			continue;

		if (entry_allow(cpu, mode))
			enter_power_mode(cpu, mode);
	}

	spin_unlock(&cpupm_lock);
}

static void exynos_cpupm_exit(int cpu, int cancel)
{
	struct exynos_cpupm *pm;
	int i;

	spin_lock(&cpupm_lock);
	pm = per_cpu_ptr(cpupm, cpu);

	/* Make settings to exit from mode */
	for (i = 0; i < POWERMODE_TYPE_END; i++) {
		struct power_mode *mode = pm->modes[i];

		if (!mode)
			continue;

		if (check_state_powerdown(mode))
			exit_power_mode(cpu, mode, cancel);
	}

	/* Set cpu state to RUN */
	set_state_run(pm);

	/* Configure PMUCAL to power up core */
	cal_cpu_enable(cpu);

	spin_unlock(&cpupm_lock);
}

static int exynos_cpu_pm_notify_callback(struct notifier_block *self,
					 unsigned long action, void *v)
{
	int cpu = smp_processor_id();
	int cpu_state;

	/* ignore CPU_PM event in suspend sequence */
	if (system_suspended)
		return NOTIFY_OK;

	switch (action) {
	case CPU_PM_ENTER:
		/*
		 * There are few block condition of C2.
		 *  - while cpu is hotpluging.
		 */
		if (exynos_cpupm_notify(C2_ENTER, 0) ||
		    *per_cpu_ptr(hotplug_ing, cpu))
			return NOTIFY_BAD;

		exynos_cpupm_enter(cpu);
		break;
	case CPU_PM_EXIT:
		cpu_state = readl_relaxed(nscode_base + CPU_STATE_OFFSET(cpu));
		exynos_cpupm_exit(cpu, cpu_state & CANCEL_FLAG);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_cpu_pm_notifier = {
	.notifier_call = exynos_cpu_pm_notify_callback,
};

/******************************************************************************
 *                               sysfs interface                              *
 ******************************************************************************/
static ssize_t idle_ip_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct idle_ip *ip;
	unsigned long flags;
	int ret = 0;

	list_for_each_entry(ip, &fix_idle_ip_list, list)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[fix%d] %s\n",
				ip->index, ip->name);

	spin_lock_irqsave(&idle_ip_list_lock, flags);

	list_for_each_entry(ip, &idle_ip_list, list)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "[%d] %s\n",
				ip->index, ip->name);

	spin_unlock_irqrestore(&idle_ip_list_lock, flags);

	return ret;
}
DEVICE_ATTR_RO(idle_ip);

static ssize_t power_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct power_mode *mode = container_of(attr, struct power_mode, attr);

	return snprintf(buf, PAGE_SIZE, "%s\n",
			atomic_read(&mode->disable) > 0 ? "disabled" : "enabled");
}

static ssize_t power_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct power_mode *mode = container_of(attr, struct power_mode, attr);
	unsigned int val;
	int cpu, type;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	cpu = cpumask_any(&mode->siblings);
	type = mode->type;

	val = !!val;
	if (mode->user_request == val)
		return count;

	mode->user_request = val;
	if (val)
		enable_power_mode(cpu, type);
	else
		disable_power_mode(cpu, type);

	return count;
}

static struct attribute *exynos_cpupm_attrs[] = {
	&dev_attr_idle_ip.attr,
	&dev_attr_time_in_state.attr,
	&dev_attr_profile.attr,
	NULL,
};

static struct attribute_group exynos_cpupm_group = {
	.name = "cpupm",
	.attrs = exynos_cpupm_attrs,
};

#define CPUPM_ATTR(_attr, _name, _mode, _show, _store)		\
	sysfs_attr_init(&_attr.attr);				\
	_attr.attr.name	= _name;				\
	_attr.attr.mode	= VERIFY_OCTAL_PERMISSIONS(_mode);	\
	_attr.show	= _show;				\
	_attr.store	= _store

/******************************************************************************
 *                               CPU HOTPLUG                                  *
 ******************************************************************************/
static int cpuhp_cpupm_online(unsigned int cpu)
{
	struct exynos_cpupm *pm = per_cpu_ptr(cpupm, cpu);
	struct power_mode *mode = pm->modes[POWERMODE_TYPE_CLUSTER];

	if (mode) {
		struct cpumask mask;

		cpumask_and(&mask, &mode->siblings, cpu_online_mask);

		if (cpumask_weight(&mask) == 0)
			cal_cluster_enable(mode->cal_id);
	}

	cal_cpu_enable(cpu);

	/*
	 * At this point, mark this cpu as finished the hotplug.
	 * Because the hotplug in sequence is done, this cpu could enter C2.
	 */
	*per_cpu_ptr(hotplug_ing, cpu) = false;

	return 0;
}

static int cpuhp_cpupm_offline(unsigned int cpu)
{
	struct exynos_cpupm *pm = per_cpu_ptr(cpupm, cpu);
	struct power_mode *mode = pm->modes[POWERMODE_TYPE_CLUSTER];

	/*
	 * At this point, mark this cpu as entering the hotplug.
	 * In order not to confusing ACPM, the cpu that entering the hotplug
	 * should not enter C2.
	 */
	*per_cpu_ptr(hotplug_ing, cpu) = true;

	cal_cpu_disable(cpu);

	if (mode) {
		struct cpumask mask;

		cpumask_and(&mask, &mode->siblings, cpu_online_mask);

		if (cpumask_weight(&mask) == 1)
			cal_cluster_disable(mode->cal_id);
	}

	return 0;
}

/******************************************************************************
 *                                Initialization                              *
 ******************************************************************************/
static void
wakeup_mask_init(struct device_node *mode_dn, struct power_mode *mode)
{
	struct device_node *dn, *child;
	struct wakeup_mask_config *wm_config;
	int count, i;

	wm_config = kzalloc(sizeof(*wm_config), GFP_KERNEL);
	if (!wm_config)
		return;

	/* initialize wakeup-mask */
	dn = of_find_node_by_name(mode_dn, "wakeup-masks");
	if (!dn) {
		pr_warn("wakeup-masks is omitted in device tree\n");
		goto fail;
	}

	count = of_get_child_count(dn);
	wm_config->num_wakeup_mask = count;
	wm_config->wakeup_masks = kcalloc(count, sizeof(struct wakeup_mask), GFP_KERNEL);
	if (!wm_config->wakeup_masks)
		goto fail;

	i = 0;
	for_each_child_of_node(dn, child) {
		of_property_read_u32(child, "mask-reg-offset",
				     &wm_config->wakeup_masks[i].mask_reg_offset);
		of_property_read_u32(child, "stat-reg-offset",
				     &wm_config->wakeup_masks[i].stat_reg_offset);
		of_property_read_u32(child, "mask",
				     &wm_config->wakeup_masks[i].mask);
		i++;
	}

	/* initialize eint-wakeup-mask */
	dn = of_find_node_by_name(mode_dn, "eint-wakeup-masks");
	if (!dn) {
		pr_warn("eint-wakeup-masks is omitted in device tree\n");
		goto fail;
	}

	count = of_get_child_count(dn);
	wm_config->num_eint_wakeup_mask = count;
	wm_config->eint_wakeup_mask_reg_offset = kcalloc(count, sizeof(int), GFP_KERNEL);
	if (!wm_config->eint_wakeup_mask_reg_offset)
		kfree(wm_config);

	i = 0;
	for_each_child_of_node(dn, child) {
		of_property_read_u32(child, "mask-reg-offset",
				     &wm_config->eint_wakeup_mask_reg_offset[i]);
		i++;
	}

	mode->wm_config = wm_config;

	return;

fail:
	kfree(wm_config->wakeup_masks);
	kfree(wm_config->eint_wakeup_mask_reg_offset);
	kfree(wm_config);
}

static int exynos_cpupm_mode_init(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;

	cpupm = alloc_percpu(struct exynos_cpupm);

	while ((dn = of_find_node_by_type(dn, "cpupm"))) {
		struct power_mode *mode;
		const char *buf;
		int cpu, ret;

		/*
		 * Power mode is dynamically generated according to
		 * what is defined in device tree.
		 */
		mode = kzalloc(sizeof(*mode), GFP_KERNEL);
		if (!mode)
			return -ENOMEM;

		strncpy(mode->name, dn->name, NAME_LEN - 1);

		ret = of_property_read_u32(dn, "target-residency",
					   &mode->target_residency);
		if (ret)
			return ret;

		ret = of_property_read_u32(dn, "type", &mode->type);
		if (ret)
			return ret;

		if (!is_valid_powermode(mode->type))
			return -EINVAL;

		if (!of_property_read_string(dn, "siblings", &buf)) {
			cpulist_parse(buf, &mode->siblings);
			cpumask_and(&mode->siblings, &mode->siblings,
				    cpu_possible_mask);
		}

		if (!of_property_read_string(dn, "entry-allowed", &buf)) {
			cpulist_parse(buf, &mode->entry_allowed);
			cpumask_and(&mode->entry_allowed, &mode->entry_allowed,
				    cpu_possible_mask);
		}

		of_property_read_u32(dn, "cal-id", &mode->cal_id);

		atomic_set(&mode->disable, 0);

		/*
		 * The users' request is set to enable since initialization
		 * state of power mode is enabled.
		 */
		mode->user_request = true;

		/*
		 * Initialize attribute for sysfs.
		 * The absence of entry allowed cpu is equivalent to this power
		 * mode being disabled. In this case, no attribute is created.
		 */
		if (!cpumask_empty(&mode->entry_allowed)) {
			CPUPM_ATTR(mode->attr, mode->name, 0644,
				   power_mode_show, power_mode_store);

			ret = sysfs_add_file_to_group(&pdev->dev.kobj,
						      &mode->attr.attr,
						      exynos_cpupm_group.name);
			if (ret)
				pr_warn("Failed to add sysfs or POWERMODE\n");
		}

		/* Connect power mode to the cpus in the power domain */
		for_each_cpu(cpu, &mode->siblings) {
			struct exynos_cpupm *pm = per_cpu_ptr(cpupm, cpu);

			pm->modes[mode->type] = mode;
		}

		list_add_tail(&mode->list, &mode_list);

		/*
		 * At the point of CPUPM initialization, all CPUs are already
		 * hotplugged in without calling cal_cluster_enable() because
		 * CPUPM cannot know cal-id at that time.
		 * Explicitly call cal_cluster_enable() here.
		 */
		if (mode->type == POWERMODE_TYPE_CLUSTER)
			cal_cluster_enable(mode->cal_id);

		if (mode->type == POWERMODE_TYPE_SYSTEM)
			wakeup_mask_init(dn, mode);
	}

	return 0;
}

static int fix_idle_ip_init(struct device_node *dn)
{
	struct device_node *child = of_get_child_by_name(dn, "idle-ip");
	int i;

	if (!child)
		return 0;

	fix_idle_ip_count = of_property_count_strings(dn, "fix-idle-ip");
	if (fix_idle_ip_count <= 0 || fix_idle_ip_count > FIX_IDLE_IP_MAX)
		return 0;

	for (i = 0; i < fix_idle_ip_count; i++) {
		struct idle_ip *ip;
		const char *name;

		ip = kzalloc(sizeof(*ip), GFP_KERNEL);
		if (!ip)
			return -ENOMEM;

		of_property_read_string_index(dn, "fix-idle-ip", i, &name);

		ip->name = name;
		ip->index = i;
		ip->pmu_offset = PMU_IDLE_IP(i);
		list_add_tail(&ip->list, &fix_idle_ip_list);
	}

	return 0;
}

static int exynos_cpupm_suspend_noirq(struct device *dev)
{
	system_suspended = 1;
	return 0;
}

static int exynos_cpupm_resume_noirq(struct device *dev)
{
	system_suspended = 0;
	return 0;
}

static const struct dev_pm_ops cpupm_pm_ops = {
	.suspend_noirq = exynos_cpupm_suspend_noirq,
	.resume_noirq = exynos_cpupm_resume_noirq,
};

static int exynos_cpupm_probe(struct platform_device *pdev)
{
	int ret, cpu;

	ret = fix_idle_ip_init(pdev->dev.of_node);
	if (ret)
		return ret;

	ret = sysfs_create_group(&pdev->dev.kobj, &exynos_cpupm_group);
	if (ret)
		pr_warn("Failed to create sysfs for CPUPM\n");

	/* Link CPUPM sysfs to /sys/devices/system/cpu/cpupm */
	if (sysfs_create_link(&cpu_subsys.dev_root->kobj,
			      &pdev->dev.kobj, "cpupm"))
		pr_err("Failed to link CPUPM sysfs to cpu\n");

	ret = exynos_cpupm_mode_init(pdev);
	if (ret)
		return ret;

	/* set PMU to power on */
	for_each_online_cpu(cpu)
		cal_cpu_enable(cpu);

	hotplug_ing = alloc_percpu(bool);
	cpuhp_setup_state(CPUHP_BP_PREPARE_DYN,
			  "AP_EXYNOS_CPU_POWER_UP_CONTROL",
			  cpuhp_cpupm_online, NULL);

	cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
			  "AP_EXYNOS_CPU_POWER_DOWN_CONTROL",
			  NULL, cpuhp_cpupm_offline);

	spin_lock_init(&cpupm_lock);

	nscode_base = ioremap(NSCODE_BASE, SZ_4K);

	ret = cpu_pm_register_notifier(&exynos_cpu_pm_notifier);
	if (ret)
		return ret;

	cpupm_init_time = ktime_get();

	return 0;
}

static const struct of_device_id of_exynos_cpupm_match[] = {
	{ .compatible = "samsung,exynos-cpupm", },
	{ },
};
MODULE_DEVICE_TABLE(of, of_exynos_cpupm_match);

static struct platform_driver exynos_cpupm_driver = {
	.driver = {
		.name = "exynos-cpupm",
		.owner = THIS_MODULE,
		.of_match_table = of_exynos_cpupm_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &cpupm_pm_ops,
#endif
	},
	.probe		= exynos_cpupm_probe,
};

MODULE_SOFTDEP("pre: exynos_mct");
MODULE_DESCRIPTION("Exynos CPUPM driver");
MODULE_LICENSE("GPL");
module_platform_driver(exynos_cpupm_driver);
