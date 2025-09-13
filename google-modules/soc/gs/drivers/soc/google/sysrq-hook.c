// SPDX-License-Identifier: GPL-2.0

#include <linux/kernel.h>
#include <linux/module.h>

#include <trace/hooks/sysrqcrash.h>

static void sysrq_hook_crash(void *hook_data, void *data)
{
	struct task_struct *tsk = data;
	panic("sysrq triggered crash by %.20s\n", tsk->comm);
}

/*
 * Module init and exit
 */
static int __init sysrq_hook_init(void)
{
	register_trace_android_vh_sysrq_crash(sysrq_hook_crash, NULL);
	return 0;
}

module_init(sysrq_hook_init);

MODULE_DESCRIPTION("Module to register sysrq vh.");
MODULE_AUTHOR("Woody Lin <woodylin@google.com>");
MODULE_LICENSE("GPL v2");
