// SPDX-License-Identifier: GPL-2.0-only
/*
 * pixel-debug-test.c
 *
 * Utility module to trigger various intentional errors.
 *
 * Copyright (C) 2019 Google LLC.
 */

/*
 * When the module is loaded, it exposes the interface at
 * /sys/kernel/pixel_debug/trigger. Writing into this file with the
 * corresponding test name and parameters would trigger the
 * corresponding error.
 *
 * The test names are in the list of force_error_vector.errcmd.
 * E.g., echo panic>/sys/kernel/pixel_debug/trigger can trigger the
 * intentional panic.
 *
 * Two error triggers support additional parameters.
 * 1. "memcorrupt" can have an optional parameter "panic" to immediately panic
 * the kernel. E.g.,
 * "echo memcorrupt>/sys/kernel/pixel_debug/trigger" causes memory corruption,
 * but kernel may continue to run, unless KASAN configs are enabled and
 * triggers panic.
 * "echo memcorrupt panic>/sys/kernel/pixel_debug/trigger" causes memory
 * corruption, and call panic immediately.
 * 2. "reg_access" may take one or two parameters, separated by space.
 *    "reg_access <addr>" is a read request;
 *    "reg_access <addr> <value>" is a write request.
 * E.g., "echo "reg_access 0x17420000">/sys/kernel/pixel_debug/trigger" causes
 *	 results in reading address 0x17420000;
 * E.g., "echo "reg_access 0x17420000 0x12345678">
 *	  /sys/kernel/pixel_debug/trigger"
 *	 results in writing 0x12345678 into 0x17420000.
 */

#define pr_fmt(fmt) "PIXEL DEBUG TEST: %s() " fmt, __func__

#include <linux/cpu.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/nmi.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/suspend.h>
#include <soc/google/debug-test.h>

/*
 * Utility functions
 */
static inline void infinite_loop(void)
{
#if IS_ENABLED(CONFIG_ARM64)
	asm("b .");
#else
	for (;;)
		;
#endif
}

void pull_down_other_cpus(void)
{
#if IS_ENABLED(CONFIG_HOTPLUG_CPU)
	int cpu, ret, curr_cpu;

	curr_cpu = smp_processor_id();

	for_each_possible_cpu(cpu) {
		if (cpu == curr_cpu)
			continue;
		ret = remove_cpu(cpu);
		if (ret)
			pr_crit("CORE%d ret: %x\n", cpu, ret);
	}
#endif
}
EXPORT_SYMBOL_GPL(pull_down_other_cpus);

#define BUFFER_SIZE SZ_1K
static int recursive_loop(int remaining)
{
	char buf[BUFFER_SIZE];

	pr_crit("remaining = [%d]\n", remaining);

	/* Make sure compiler does not optimize this away. */
	memset(buf, (remaining & 0xff) | 0x1, BUFFER_SIZE);
	if (!remaining)
		return 0;
	return recursive_loop(remaining - 1);
}

static int get_index_of_space_or_null(char *arg)
{
	int i = 0;

	if (arg == NULL)
		return 0;

	/*
	 * A parameter ends with either a space, or \0.
	 */
	while (!isspace(arg[i]) && arg[i])
		i++;

	return i;
}

/*
 * Error trigger functions
 */
static void simulate_panic(char *arg)
{
	pr_crit("called!\n");

	panic("simulate_panic");

	/* Should not reach here */
	pr_crit("failed!\n");
}

static void simulate_bug(char *arg)
{
	pr_crit("called!\n");

	BUG();

	/* Should not reach here */
	pr_crit("failed!\n");
}

static void simulate_warn(char *arg)
{
	pr_crit("called\n");

	WARN_ON(1);
}

static void simulate_null(char *arg)
{
	pr_crit("called!\n");

	/* Intentional null pointer dereference */
	writeb('a', NULL);

	/* Should not reach here */
	pr_crit("failed!");
}

static void (*undefined_function)(void) = (void *)0x1234;
static void simulate_undefined_function(char *arg)
{
	pr_crit("function address=[%p]\n", undefined_function);

	undefined_function();

	/* Should not reach here */
	pr_crit("failed!");
}

static void simulate_double_free(char *arg)
{
	void *p;

	pr_crit("called!\n");

	p = kmalloc(sizeof(unsigned int), GFP_KERNEL);
	if (p) {
		*(unsigned int *)p = 0x0;
		kfree(p);
		msleep(1000);
		kfree(p);
	}
}

static void simulate_use_after_free(char *arg)
{
	unsigned int *p;

	pr_crit("called!\n");

	p = kmalloc(sizeof(int), GFP_KERNEL);
	kfree(p);
	*p = 0x1234;
}

static void simulate_memory_corruption(char *arg)
{
	int *ptr;
	int len;
	const char *panic_str = "panic";

	if (arg == NULL)
		len = 0;
	else
		len = min(strlen(arg), strlen(panic_str));

	pr_crit("arg [%s]\n", arg);

	ptr = kmalloc(sizeof(int), GFP_KERNEL);
	if (ptr) {
		*ptr++ = 4;
		*ptr = 2;

		if (len > 0 && !strncmp(panic_str, arg, len))
			panic("MEMORY CORRUPTION");
	}
}

static void simulate_low_memory(char *arg)
{
	int i = 0;

	pr_crit("called!\n");

	pr_crit("Allocating memory until failure!\n");
	while (kmalloc(128 * 1024, GFP_KERNEL))
		i++;
	pr_crit("Allocated %d KB!\n", i * 128);
}

static void simulate_softlockup(char *arg)
{
	pr_crit("called on CPU %u!\n", smp_processor_id());

	local_irq_disable();
	preempt_disable();
	local_irq_enable();

	/* If CONFIG_BOOTPARAM_SOFTLOCKUP_PANIC=y, this should cause a panic */
	infinite_loop();

	preempt_enable();

	/* Should not reach here */
	pr_crit("failed!");
}

static void simulate_spinlock_lockup(char *arg)
{
	spinlock_t debug_test_lock;

	pr_crit("called!\n");
	spin_lock_init(&debug_test_lock);

	spin_lock(&debug_test_lock);
	spin_lock(&debug_test_lock);

	/* Should not reach here */
	pr_crit("failed!");
}

/* timeout for dog bark/bite */
#define DELAY_TIME 30000

static void simulate_watchdog(char *arg)
{
	pr_crit("called!\n");

	pull_down_other_cpus();
	pr_crit("start to hang\n");
	local_irq_disable();
	mdelay(DELAY_TIME);
	local_irq_enable();

	/* Should not reach here */
	pr_crit("failed!");
}

static void simulate_writero(char *arg)
{
	unsigned long *ptr;

	pr_crit("called!\n");

	ptr = (unsigned long *)simulate_writero;
	*ptr ^= 0x12345678;

	/* Should not reach here */
	pr_crit("failed!");
}

static void simulate_buffer_overflow(char *arg)
{
	pr_crit("called!\n");

	recursive_loop(600);
}

static void simulate_schedule_while_atomic(char *arg)
{
	spinlock_t debug_test_lock;

	pr_crit("called!\n");
	spin_lock_init(&debug_test_lock);

	spin_lock(&debug_test_lock);
	msleep(1000);

	spin_lock(&debug_test_lock);
	/* Should not reach here */
	pr_crit("failed!");
}

static void simulate_register_access(char *arg)
{
	int ret, index = 0;
	unsigned long reg, val;
	char *tmp, *tmparg;
	void __iomem *addr;

	pr_crit("start with arg [%s]\n", arg);

	index = get_index_of_space_or_null(arg);
	if (index == 0) {
		pr_crit("no address given! Exit the test.\n");
		return;
	}
	if (index > PAGE_SIZE)
		return;

	tmp = kstrndup(arg, index, GFP_KERNEL);
	if (!tmp)
		return;

	ret = kstrtoul(tmp, 16, &reg);
	addr = ioremap(reg, 0x10);
	if (!addr) {
		pr_crit("failed to remap 0x%lx, quit\n", reg);
		kfree(tmp);
		return;
	}
	pr_crit("1st parameter: 0x%lx\n", reg);

	tmparg = &arg[index + 1];
	index = get_index_of_space_or_null(tmparg);
	if (index == 0) {
		pr_crit("there is no 2nd parameter\n");
		pr_crit("try to read 0x%lx\n", reg);

		ret = __raw_readl(addr);
		pr_crit("result : 0x%x\n", ret);

	} else {
		if (index > PAGE_SIZE) {
			kfree(tmp);
			return;
		}
		memcpy(tmp, tmparg, index);
		tmp[index] = '\0';
		ret = kstrtoul(tmp, 16, &val);
		pr_crit("2nd parameter: 0x%lx\n", val);
		pr_crit("try to write 0x%lx to 0x%lx\n", val, reg);

		__raw_writel(val, addr);
	}
	kfree(tmp);
	/* should not reach here */
	pr_crit("failed!");
}

static void simulate_svc(char *arg)
{
	pr_crit("called!\n");
#if IS_ENABLED(CONFIG_ARM64)
	asm("svc #0x0");

	/* Should not reach here */
	pr_crit("failed!\n");
#endif
}

static void simulate_undefined_memory(char *arg)
{
	pr_crit("called!\n");

#if IS_ENABLED(CONFIG_ARM64)
	asm volatile(".word 0xe7f001f2\n\t");

	/* Should not reach here */
	pr_crit("failed!\n");
#endif
}

static void simulate_pc_abort(char *arg)
{
	pr_crit("called!\n");

#if IS_ENABLED(CONFIG_ARM64)
	asm("add x30, x30, #0x1\n\t"
	    "ret"
	    ::: "x30");

	/* Should not reach here */
	pr_crit("failed!\n");
#endif
}

static void simulate_sp_abort(char *arg)
{
	pr_crit("called!\n");

#if IS_ENABLED(CONFIG_ARM64)
	/* X29(FP) or SP cannot be added to the clobber list */
	asm("mov x29, #0xff00\n\t"
	    "mov sp, #0xff00\n\t"
	    "ret");

	/* Should not reach here */
	pr_crit("failed!\n");
#endif
}

static void simulate_jump_zero(char *arg)
{
	pr_crit("called!\n");

#if IS_ENABLED(CONFIG_ARM64)
	asm("mov x0, #0x0\n\t"
	    "br x0"
	    ::: "x0");

	/* Should not reach here */
	pr_crit("failed!\n");
#endif
}

static int suspend_valid(suspend_state_t state)
{
	return 1;
}

static int suspend_begin(suspend_state_t state)
{
	pr_crit("called!\n");
	schedule_timeout_interruptible(msecs_to_jiffies(9000000));
	return -EINVAL;
}

static int suspend_enter(suspend_state_t state)
{
	return -EINVAL;
}

static const struct platform_suspend_ops suspend_ops = {
	.valid = suspend_valid,
	.begin = suspend_begin,
	.enter = suspend_enter,
};

static void simulate_suspend_hang(char *arg)
{
	suspend_set_ops(&suspend_ops);
}

/*
 * SOC dependent triggers
 */
static struct debug_trigger soc_test_trigger;

void debug_trigger_register(struct debug_trigger *soc_trigger, char *arch_name)
{
	pr_info("DEBUG TEST: [%s] test triggers are registered!", arch_name);
	soc_test_trigger.hard_lockup = soc_trigger->hard_lockup;
	soc_test_trigger.cold_reset = soc_trigger->cold_reset;
	soc_test_trigger.watchdog_emergency_reset =
		soc_trigger->watchdog_emergency_reset;
	soc_test_trigger.halt = soc_trigger->halt;
	soc_test_trigger.cacheflush = soc_trigger->cacheflush,
	soc_test_trigger.cpucontext = soc_trigger->cpucontext,
	soc_test_trigger.arraydump = soc_trigger->arraydump;
	soc_test_trigger.scandump = soc_trigger->scandump;
	soc_test_trigger.el3_assert = soc_trigger->el3_assert;
	soc_test_trigger.el3_panic = soc_trigger->el3_panic;
	soc_test_trigger.ecc = soc_trigger->ecc;
}
EXPORT_SYMBOL_GPL(debug_trigger_register);

static void simulate_hardlockup(char *arg)
{
	pr_crit("called!\n");

	if (soc_test_trigger.hard_lockup == NULL) {
		pr_crit("SOC specific trigger is not registered! Using default.\n");

		local_irq_disable();
		infinite_loop();

		/* Should not reach here */
		pr_crit("failed!\n");
	} else {
		(*soc_test_trigger.hard_lockup)(arg);
	}

}

static void simulate_cold_reset(char *arg)
{
	pr_crit("called!\n");
	if (soc_test_trigger.cold_reset == NULL) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.cold_reset)(arg);

	/* Should not reach here */
	pr_crit("failed!\n");
}

static void simulate_watchdog_emergency_reset(char *arg)
{
	pr_crit("called!\n");
	if (soc_test_trigger.watchdog_emergency_reset == NULL) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.watchdog_emergency_reset)(arg);

	/* Should not reach here */
	pr_crit("failed!\n");
}

static void simulate_halt(char *arg)
{
	pr_crit("called!\n");
	if (!soc_test_trigger.halt) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.halt)(arg);

	/* Should not reach here */
	pr_crit("failed!\n");
}

static void simulate_cacheflush(char *arg)
{
	pr_crit("called!\n");
	if (!soc_test_trigger.cacheflush) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.cacheflush)(arg);
}

static void simulate_cpucontext(char *arg)
{
	pr_crit("called!\n");
	if (!soc_test_trigger.cpucontext) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.cpucontext)(arg);
}

static void simulate_arraydump(char *arg)
{
	pr_crit("called!\n");
	if (!soc_test_trigger.arraydump) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.arraydump)(arg);
}

static void simulate_scandump(char *arg)
{
	pr_crit("called!\n");
	if (!soc_test_trigger.scandump) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.scandump)(arg);
}

static void simulate_el3_assert(char *arg)
{
	pr_crit("called!\n");
	if (!soc_test_trigger.el3_assert) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.el3_assert)(arg);
}

static void simulate_el3_panic(char *arg)
{
	pr_crit("called!\n");
	if (!soc_test_trigger.el3_panic) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.el3_panic)(arg);
}

static void simulate_ecc(char *arg)
{
	pr_crit("called!\n");
	if (!soc_test_trigger.ecc) {
		pr_crit("SOC specific trigger is not registered! Exit the test.\n");
		return;
	}

	(*soc_test_trigger.ecc)(arg);
}

/*
 * Error trigger definitions
 */
typedef void (*force_error_func)(char *arg);

struct force_error_item {
	char errcmd[SZ_32];
	force_error_func errfunc;
};

static const struct force_error_item force_error_vector[] = {
	/* General debug triggers */
	{ "panic",		&simulate_panic },
	{ "bug",		&simulate_bug },
	{ "warn",		&simulate_warn },
	{ "null",		&simulate_null },
	{ "undef_func",		&simulate_undefined_function },
	{ "double_free",	&simulate_double_free },
	{ "use_after_free",	&simulate_use_after_free },
	{ "memcorrupt",		&simulate_memory_corruption },
	{ "lowmem",		&simulate_low_memory },
	{ "softlockup",		&simulate_softlockup },
	{ "hardlockup",		&simulate_hardlockup },
	{ "spinlockup",		&simulate_spinlock_lockup },
	{ "watchdog",		&simulate_watchdog },
	{ "writero",		&simulate_writero },
	{ "overflow",		&simulate_buffer_overflow },
	{ "sched_atomic",	&simulate_schedule_while_atomic },
	{ "reg_access",		&simulate_register_access },
	{ "svc",		&simulate_svc },
	{ "undef",		&simulate_undefined_memory },
	{ "pcabort",		&simulate_pc_abort },
	{ "spabort",		&simulate_sp_abort },
	{ "jumpzero",		&simulate_jump_zero },
	{ "suspend_hang",	&simulate_suspend_hang },
	/* SOC dependent triggers */
	{ "cold_reset",		&simulate_cold_reset },
	{ "emerg_reset",	&simulate_watchdog_emergency_reset },
	{ "halt",		&simulate_halt },
	{ "cacheflush",		&simulate_cacheflush },
	{ "cpucontext",		&simulate_cpucontext },
	{ "arraydump",		&simulate_arraydump },
	{ "scandump",		&simulate_scandump },
	{ "el3_assert",		&simulate_el3_assert },
	{ "el3_panic",		&simulate_el3_panic },
	{ "ecc",		&simulate_ecc },
};

static void parse_and_trigger(const char *buf)
{
	int i;
	char *cmd;
	char *space = NULL, *param = NULL;
	int cmd_size = strlen(buf);

	pr_debug("cmd_size=%d, PAGE_SIZE=%lu\n", cmd_size, PAGE_SIZE);
	/*
	 * Extract the command before the first space.
	 */
	space = strchr(buf, ' ');
	if (space != NULL) {
		cmd_size = space - buf;
		param = space + 1;
	}

	cmd = kstrndup(buf, cmd_size, GFP_KERNEL);
	if (!cmd)
		return;

	for (i = 0; i < ARRAY_SIZE(force_error_vector); i++) {
		if (strcmp(cmd, force_error_vector[i].errcmd) != 0)
			continue;
		pr_debug("func param=%s\n", param);
		force_error_vector[i].errfunc(param);
		break;
	}

	if (i == ARRAY_SIZE(force_error_vector))
		pr_crit("no trigger exists for command [%s]!\n", cmd);

	kfree(cmd);
}

/*
 * Sysfs structures
 */
static struct kobject *pixel_debug_kobj;
static char *trigger;

static ssize_t trigger_write(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	char *token;
	pr_crit("count=%zu, buf=%s", count, buf);
	strscpy(trigger, buf, PAGE_SIZE);

	/*
	 * "echo" command appends a newline char by default. Replacing the
	 * newline char with '\0' because it is not part of the command.
	 */
	token = strsep(&trigger, "\n");
	parse_and_trigger(token);
	return count;
}

static struct kobj_attribute debug_attr =
	__ATTR(trigger, 0220, NULL, trigger_write);

/*
 * Module init and exit
 */
static int __init pixel_debug_test_init(void)
{
	trigger = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!trigger)
		return -ENOMEM;

	pixel_debug_kobj = kobject_create_and_add("pixel_debug", kernel_kobj);
	if (!pixel_debug_kobj) {
		pr_err("cannot create kobj for pixel_debug!");
		goto error;
	}
	if (sysfs_create_file(pixel_debug_kobj, &debug_attr.attr)) {
		pr_err("cannot create file in pixel_debug!");
		kobject_put(pixel_debug_kobj);
		goto error;
	}
	pr_info("initialized!");
	return 0;
error:
	kfree(trigger);
	trigger = NULL;
	return -ENOMEM;
}

static void __exit pixel_debug_test_exit(void)
{
	kfree(trigger);
	trigger = NULL;

	kobject_put(pixel_debug_kobj);
	pr_info("module exit!");
}

module_init(pixel_debug_test_init);
module_exit(pixel_debug_test_exit);

MODULE_DESCRIPTION("Module to trigger intentional errors.");
MODULE_AUTHOR("Siyuan Zhou <siyuanzhou@google.com>");
MODULE_LICENSE("GPL v2");
