// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include <linux/nmi.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>

#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-pmu-if.h>

#include "debug-snapshot-local.h"

typedef void (*force_error_func)(char *arg);

struct debug_test_desc {
	int enabled;
	int nr_cpu;
	int nr_little_cpu;
	int nr_mid_cpu;
	int nr_big_cpu;
	int little_cpu_start;
	int mid_cpu_start;
	int big_cpu_start;
	unsigned int ps_hold_control_offset;
	unsigned int *null_pointer_ui;
	int *null_pointer_si;
	void (*null_function)(void);
	struct dentry *debugfs_root;
	struct device *dev;
	spinlock_t debug_test_lock;
};

struct force_error_item {
	char errname[SZ_32];
	force_error_func errfunc;
};

static struct debug_test_desc exynos_debug_desc = { 0, };

static const char * const test_vector[] = {
	"KP",
	"SVC",
	"SFR 0x1ffffff0",
	"SFR 0x1ffffff0 0x12345678",
	"WP",
	"panic",
	"bug",
	"dabrt",
	"pabrt",
	"undef",
	"memcorrupt",
	"softlockup",
	"hardlockup 0",
	"hardlockup LITTLE",
	"hardlockup BIG",
	"spinlockup",
	"pcabort",
	"jumpzero",
	"writero",
	"danglingref",
	"dfree",
	"QDP",
	"spabort",
	"overflow",
	"cacheflush",
};

/* timeout for dog bark/bite */
#define DELAY_TIME 30000

static void pull_down_other_cpus(void)
{
#if IS_ENABLED(CONFIG_HOTPLUG_CPU)
	int cpu, ret;

	for (cpu = exynos_debug_desc.nr_cpu - 1; cpu > 0 ; cpu--) {
		ret = cpu_down(cpu);
		if (ret)
			dev_crit(exynos_debug_desc.dev, "%s() CORE%d ret: %x\n",
							__func__, cpu, ret);
	}
#endif
}

static int find_blank(char *arg)
{
	int i;

	for (i = 0; !isspace(arg[i]) && arg[i]; i++)
		;

	return i;
}

static void simulate_KP(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	*exynos_debug_desc.null_pointer_ui = 0x0; /* SVACE: intended */
}

static void simulate_DP(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	pull_down_other_cpus();
	dev_crit(exynos_debug_desc.dev, "%s() start to hanging\n", __func__);
	local_irq_disable();
	mdelay(DELAY_TIME);
	local_irq_enable();
	/* should not reach here */
}

static void simulate_QDP(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	dbg_snapshot_emergency_reboot(NULL);
	mdelay(DELAY_TIME);
	/* should not reach here */
}

static void simulate_SVC(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	asm("svc #0x0");
	/* should not reach here */
}

static void simulate_SFR(char *arg)
{
	int ret, index = 0;
	unsigned long reg, val;
	char *tmp, *tmparg;
	void __iomem *addr;

	dev_crit(exynos_debug_desc.dev, "%s() start\n", __func__);

	index = find_blank(arg);
	if (index > PAGE_SIZE)
		return;

	tmp = kmalloc(PAGE_SIZE, GFP_KERNEL);
	memcpy(tmp, arg, index);
	tmp[index] = '\0';

	ret = kstrtoul(tmp, 16, &reg);
	addr = ioremap(reg, 0x10);
	if (!addr) {
		dev_crit(exynos_debug_desc.dev, "failed to remap 0x%lx, quit\n", reg);
		kfree(tmp);
		return;
	}
	dev_crit(exynos_debug_desc.dev, "1st parameter: 0x%lx\n", reg);

	tmparg = &arg[index + 1];
	index = find_blank(tmparg);
	if (index == 0) {
		dev_crit(exynos_debug_desc.dev, "there is no 2nd parameter\n");
		dev_crit(exynos_debug_desc.dev, "try to read 0x%lx\n", reg);

		ret = __raw_readl(addr);
		dev_crit(exynos_debug_desc.dev, "result : 0x%x\n", ret);
	} else {
		if (index > PAGE_SIZE) {
			kfree(tmp);
			return;
		}
		memcpy(tmp, tmparg, index);
		tmp[index] = '\0';
		ret = kstrtoul(tmp, 16, &val);
		dev_crit(exynos_debug_desc.dev, "2nd parameter: 0x%lx\n", val);
		dev_crit(exynos_debug_desc.dev, "try to write 0x%lx to 0x%lx\n", val, reg);

		__raw_writel(val, addr);
	}
	kfree(tmp);
	/* should not reach here */
}

static void simulate_WP(char *arg)
{
	unsigned int ps_hold_control;

	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	exynos_pmu_read(exynos_debug_desc.ps_hold_control_offset, &ps_hold_control);
	exynos_pmu_write(exynos_debug_desc.ps_hold_control_offset,
			ps_hold_control & 0xFFFFFEFF);
}

static void simulate_PANIC(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	panic("simulate_panic");
}

static void simulate_BUG(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	BUG();
}

static void simulate_WARN(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	WARN_ON(1);
}

static void simulate_DABRT(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	*exynos_debug_desc.null_pointer_si = 0; /* SVACE: intended */
}

static void simulate_PABRT(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s() call address=[%pK]\n",
			__func__, exynos_debug_desc.null_function);

	exynos_debug_desc.null_function(); /* SVACE: intended */
}

static void simulate_UNDEF(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	asm volatile(".word 0xe7f001f2\n");
	unreachable();
}

static void simulate_DFREE(char *arg)
{
	unsigned int *p;

	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	p = kmalloc(sizeof(unsigned int), GFP_KERNEL);
	if (!p)
		return;

	*p = 0x0;
	kfree(p);
	msleep(1000);
	kfree(p); /* SVACE: intended */
}

static void simulate_DREF(char *arg)
{
	unsigned int *p;

	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	p = kmalloc(sizeof(unsigned int), GFP_KERNEL);
	if (!p)
		return;

	kfree(p);
	*p = 0x1234; /* SVACE: intended */
}

static void simulate_MCRPT(char *arg)
{
	int *ptr;

	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	ptr = kmalloc(sizeof(int), GFP_KERNEL);
	if (!ptr)
		return;

	*ptr++ = 4;
	*ptr = 2;
	panic("MEMORY CORRUPTION");
}

static void simulate_LOMEM(char *arg)
{
	int i = 0;

	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	dev_crit(exynos_debug_desc.dev, "Allocating memory until failure!\n");
	while (kmalloc(128 * 1024, GFP_KERNEL)) /* SVACE: intended */
		i++;
	dev_crit(exynos_debug_desc.dev, "Allocated %d KB!\n", i * 128);
}

static void simulate_SOFT_LOCKUP(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	local_irq_disable();
	preempt_disable();
	local_irq_enable();
	asm("b .");
	preempt_enable();
}

static void simulate_HARD_LOCKUP_handler(void *info)
{
	asm("b .");
}

static void simulate_HARD_LOCKUP(char *arg)
{
	int cpu;
	int start = -1;
	int end;
	int curr_cpu;

	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	if (!arg) {
		start = 0;
		end = exynos_debug_desc.nr_cpu;

		local_irq_disable();
		curr_cpu = raw_smp_processor_id();

		for (cpu = start; cpu < end; cpu++) {
			if (cpu == curr_cpu)
				continue;
			smp_call_function_single(cpu, simulate_HARD_LOCKUP_handler, 0, 0);
		}
		asm("b .");
		return;
	}

	if (!strcmp(arg, "LITTLE")) {
		if (exynos_debug_desc.little_cpu_start < 0 ||
				exynos_debug_desc.nr_little_cpu < 0) {
			dev_info(exynos_debug_desc.dev, "%s() no little cpu info\n", __func__);
			return;
		}
		start = exynos_debug_desc.little_cpu_start;
		end = start + exynos_debug_desc.nr_little_cpu - 1;
	} else if (!strcmp(arg, "MID")) {
		if (exynos_debug_desc.mid_cpu_start < 0 ||
				exynos_debug_desc.nr_mid_cpu < 0) {
			dev_info(exynos_debug_desc.dev, "%s() no mid cpu info\n", __func__);
			return;
		}
		start = exynos_debug_desc.mid_cpu_start;
		end = start + exynos_debug_desc.nr_mid_cpu - 1;
	} else if (!strcmp(arg, "BIG")) {
		if (exynos_debug_desc.big_cpu_start < 0 ||
				exynos_debug_desc.nr_big_cpu < 0) {
			dev_info(exynos_debug_desc.dev, "%s() no big cpu info\n", __func__);
			return;
		}
		start = exynos_debug_desc.big_cpu_start;
		end = start + exynos_debug_desc.nr_big_cpu - 1;
	}

	if (start >= 0) {
		preempt_disable();
		curr_cpu = raw_smp_processor_id();
		for (cpu = start; cpu <= end; cpu++) {
			if (cpu == curr_cpu)
				continue;
			smp_call_function_single(cpu,
					simulate_HARD_LOCKUP_handler, 0, 0);
		}
		if (curr_cpu >= start && curr_cpu <= end) {
			local_irq_disable();
			asm("b .");
		}
		preempt_enable();
		return;
	}

	if (kstrtoint(arg, 10, &cpu)) {
		dev_err(exynos_debug_desc.dev, "%s() input is invalid\n", __func__);
		return;
	}

	if (cpu < 0 || cpu >= exynos_debug_desc.nr_cpu) {
		dev_info(exynos_debug_desc.dev, "%s() input is invalid\n", __func__);
		return;
	}
	smp_call_function_single(cpu, simulate_HARD_LOCKUP_handler, 0, 0);
}

static void simulate_SPIN_LOCKUP(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	spin_lock(&exynos_debug_desc.debug_test_lock);
	spin_lock(&exynos_debug_desc.debug_test_lock);
}

static void simulate_PC_ABORT(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	asm("add x30, x30, #0x1\n"
	    "ret\n");
}

static void simulate_SP_ABORT(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	asm("mov x29, #0xff00\n"
	    "mov sp, #0xff00\n"
	    "ret\n");
}

static void simulate_JUMP_ZERO(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	asm("mov x0, #0x0\n"
	    "br x0\n");
}

static void simulate_UNALIGNED(char *arg)
{
	static u8 data[5] __aligned(4) = {1, 2, 3, 4, 5};
	u32 *p;
	u32 val = 0x12345678;

	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	p = (u32 *)(data + 1);
	dev_crit(exynos_debug_desc.dev, "data=[0x%pK] p=[0x%pKx]\n", data, p);
	if (*p == 0)
		val = 0x87654321;
	*p = val;
	dev_crit(exynos_debug_desc.dev, "val = [0x%x] *p = [0x%x]\n", val, *p);
}

static void simulate_WRITE_RO(char *arg)
{
	unsigned long *ptr;

	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	ptr = (unsigned long *)simulate_WRITE_RO;
	*ptr ^= 0x12345678;
}

#define BUFFER_SIZE SZ_1K

static int recursive_loop(int remaining)
{
	char buf[BUFFER_SIZE];

	dev_crit(exynos_debug_desc.dev, "%s() remainig = [%d]\n", __func__, remaining);

	/* Make sure compiler does not optimize this away. */
	memset(buf, (remaining & 0xff) | 0x1, BUFFER_SIZE);
	if (remaining)
		return recursive_loop(remaining - 1);
	return 0;
}

static void simulate_OVERFLOW(char *arg)
{
	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	recursive_loop(600);
}

static char *buffer[NR_CPUS];
static void simulate_CACHE_FLUSH_handler(void *info)
{
	int cpu = raw_smp_processor_id();
	u64 addr = virt_to_phys((void *)(buffer[cpu]));

	memset(buffer[cpu], 0x5A, PAGE_SIZE * 2);
	dbg_snapshot_set_debug_test_buffer_addr(addr, cpu);
	asm("b .");
}

static void simulate_CACHE_FLUSH_ALL(void *info)
{
	cache_flush_all();
}

static void simulate_CACHE_FLUSH(char *arg)
{
	u64 addr;
	int cpu;

	dev_crit(exynos_debug_desc.dev, "%s()\n", __func__);

	for_each_possible_cpu(cpu) {
		dbg_snapshot_set_debug_test_buffer_addr(0, cpu);
		buffer[cpu] = kmalloc(PAGE_SIZE * 2, GFP_KERNEL);
		memset(buffer[cpu], 0x3B, PAGE_SIZE * 2);
	}

	smp_call_function(simulate_CACHE_FLUSH_ALL, NULL, 1);
	cache_flush_all();

	smp_call_function(simulate_CACHE_FLUSH_handler, NULL, 0);
	for (cpu = 0; cpu < exynos_debug_desc.nr_cpu; cpu++) {
		if (cpu == raw_smp_processor_id())
			continue;

		while (!dbg_snapshot_get_debug_test_buffer_addr(cpu))
			;
		dev_crit(exynos_debug_desc.dev, "CPU %d STOPPING\n", cpu);
	}

	cpu = raw_smp_processor_id();
	addr = virt_to_phys((void *)(buffer[cpu]));
	memset(buffer[cpu], 0x5A, PAGE_SIZE * 2);
	dbg_snapshot_set_debug_test_buffer_addr(addr, cpu);
	dbg_snapshot_emergency_reboot("cache flush test");
}

static struct force_error_item force_error_vector[] = {
	{"KP",		&simulate_KP},
	{"DP",		&simulate_DP},
	{"QDP",		&simulate_QDP},
	{"SVC",		&simulate_SVC},
	{"SFR",		&simulate_SFR},
	{"WP",		&simulate_WP},
	{"panic",	&simulate_PANIC},
	{"bug",		&simulate_BUG},
	{"warn",	&simulate_WARN},
	{"dabrt",	&simulate_DABRT},
	{"pabrt",	&simulate_PABRT},
	{"undef",	&simulate_UNDEF},
	{"dfree",	&simulate_DFREE},
	{"danglingref",	&simulate_DREF},
	{"memcorrupt",	&simulate_MCRPT},
	{"lowmem",	&simulate_LOMEM},
	{"softlockup",	&simulate_SOFT_LOCKUP},
	{"hardlockup",	&simulate_HARD_LOCKUP},
	{"spinlockup",	&simulate_SPIN_LOCKUP},
	{"pcabort",	&simulate_PC_ABORT},
	{"spabort",	&simulate_SP_ABORT},
	{"jumpzero",	&simulate_JUMP_ZERO},
	{"unaligned",	&simulate_UNALIGNED},
	{"writero",	&simulate_WRITE_RO},
	{"overflow",	&simulate_OVERFLOW},
	{"cacheflush",	&simulate_CACHE_FLUSH},
};

static int debug_force_error(const char *val)
{
	int i;
	char *temp;
	char *ptr;

	for (i = 0; i < (int)ARRAY_SIZE(force_error_vector); i++) {
		if (!strncmp(val, force_error_vector[i].errname,
				strlen(force_error_vector[i].errname))) {
			temp = (char *)val;
			ptr = strsep(&temp, " ");	/* ignore the first token */
			ptr = strsep(&temp, " ");	/* take the second token */
			force_error_vector[i].errfunc(ptr);
			break;
		}
	}

	if (i == (int)ARRAY_SIZE(force_error_vector))
		dev_info(exynos_debug_desc.dev, "%s(): INVALID TEST CMD = [%s]\n",
				__func__, val);

	return 0;
}

static ssize_t exynos_debug_test_write(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char *buf;
	size_t buf_size;
	int i;

	buf_size = ((count + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;
	if (buf_size <= 0)
		return 0;

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return 0;

	if (copy_from_user(buf, user_buf, count)) {
		kfree(buf);
		return -EFAULT;
	}
	buf[count] = '\0';

	for (i = 0; buf[i] != '\0'; i++)
		if (buf[i] == '\n') {
			buf[i] = '\0';
			break;
		}

	dev_info(exynos_debug_desc.dev, "%s() user_buf=[%s]\n", __func__, buf);
	debug_force_error(buf);

	kfree(buf);
	return count;
}

static ssize_t exynos_debug_test_read(struct file *file,
				char __user *user_buf, size_t count,
				loff_t *ppos)
{
	char *buf;
	size_t copy_cnt;
	int i;
	int ret = 0;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return ret;

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"========== DEBUG TEST EXAMPLES ==========\n");
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "INPUT_ARG(S)\n");

	for (i = 0; i < (int)ARRAY_SIZE(test_vector); i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret,
				"%-30s\n", test_vector[i]);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"=========================================\n");

	copy_cnt = ret;
	ret = simple_read_from_buffer(user_buf, count, ppos, buf, copy_cnt);

	kfree(buf);
	return ret;
}
static const struct file_operations exynos_debug_test_file_fops = {
	.open	= simple_open,
	.read	= exynos_debug_test_read,
	.write	= exynos_debug_test_write,
	.llseek	= default_llseek,
};

static int exynos_debug_test_parsing_dt(struct device_node *np)
{
	int ret = 0;

	/* get data from device tree */
	ret = of_property_read_u32(np, "ps_hold_control_offset",
			&exynos_debug_desc.ps_hold_control_offset);
	if (ret) {
		dev_err(exynos_debug_desc.dev, "no data(ps_hold_control offset)\n");
		goto edt_desc_init_out;
	}

	ret = of_property_read_u32(np, "nr_cpu", &exynos_debug_desc.nr_cpu);
	if (ret) {
		dev_err(exynos_debug_desc.dev, "no data(nr_cpu)\n");
		goto edt_desc_init_out;
	}

	ret = of_property_read_u32(np, "little_cpu_start",
			&exynos_debug_desc.little_cpu_start);
	if (ret) {
		dev_info(exynos_debug_desc.dev, "no data(little_cpu_start)\n");
		exynos_debug_desc.little_cpu_start = -1;
	}

	ret = of_property_read_u32(np, "nr_little_cpu",	&exynos_debug_desc.nr_little_cpu);
	if (ret) {
		dev_info(exynos_debug_desc.dev, "no data(nr_little_cpu)\n");
		exynos_debug_desc.nr_little_cpu = -1;
	}

	ret = of_property_read_u32(np, "mid_cpu_start",	&exynos_debug_desc.mid_cpu_start);
	if (ret) {
		dev_info(exynos_debug_desc.dev, "no data(mid_cpu_start)\n");
		exynos_debug_desc.mid_cpu_start = -1;
	}

	ret = of_property_read_u32(np, "nr_mid_cpu", &exynos_debug_desc.nr_mid_cpu);
	if (ret) {
		dev_info(exynos_debug_desc.dev, "no data(nr_mid_cpu)\n");
		exynos_debug_desc.nr_mid_cpu = -1;
	}

	ret = of_property_read_u32(np, "big_cpu_start", &exynos_debug_desc.big_cpu_start);
	if (ret) {
		dev_info(exynos_debug_desc.dev, "no data(big_cpu_start)\n");
		exynos_debug_desc.big_cpu_start = -1;
	}

	ret = of_property_read_u32(np, "nr_big_cpu", &exynos_debug_desc.nr_big_cpu);
	if (ret) {
		dev_info(exynos_debug_desc.dev, "no data(nr_big_cpu)\n");
		exynos_debug_desc.nr_big_cpu = -1;
	}

	exynos_debug_desc.null_function = (void (*)(void))0x1234;
	spin_lock_init(&exynos_debug_desc.debug_test_lock);

	/* create debugfs test file */
	debugfs_create_file("test", 0644,
			exynos_debug_desc.debugfs_root,
			NULL, &exynos_debug_test_file_fops);
	ret = 0;
	exynos_debug_desc.enabled = 1;

edt_desc_init_out:
	return ret;
}

static int exynos_debug_test_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	dev_info(&pdev->dev, "%s() called\n", __func__);

	exynos_debug_desc.debugfs_root =
			debugfs_create_dir("exynos-debug-test", NULL);
	if (!exynos_debug_desc.debugfs_root) {
		dev_err(&pdev->dev, "cannot create debugfs dir\n");
		ret = -ENOMEM;
		goto edt_out;
	}

	exynos_debug_desc.dev = &pdev->dev;
	ret = exynos_debug_test_parsing_dt(np);
	if (ret)
		goto edt_out;

edt_out:
	dev_info(exynos_debug_desc.dev, "%s() ret=[0x%x]\n", __func__, ret);
	return ret;
}

static const struct of_device_id exynos_debug_test_matches[] = {
	{.compatible = "google,exynos-debug-test"},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_debug_test_matches);

static struct platform_driver exynos_debug_test_driver = {
	.probe		= exynos_debug_test_probe,
	.driver		= {
		.name	= "exynos-debug-test",
		.of_match_table	= of_match_ptr(exynos_debug_test_matches),
	},
};
module_platform_driver(exynos_debug_test_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos Debug Feature Test Driver");
