// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *	      http://www.samsung.com/
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/gfp.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>

#include <soc/google/debug-snapshot.h>

struct ecc_irq_desc {
	int irq;
	struct cpumask affinity;
};

struct exynos_ecc_desc {
	struct ecc_irq_desc *ecc_irqs;
	struct cpumask ecc_online_cpus;
	raw_spinlock_t lock;
	int irq_num;
};

static struct exynos_ecc_desc ecc_desc;

static struct ecc_irq_desc *get_ecc_irq_desc(int irq)
{
	int i;
	struct ecc_irq_desc *desc = NULL;

	for (i = 0; i < ecc_desc.irq_num; i++) {
		if (ecc_desc.ecc_irqs[i].irq == irq) {
			desc = &ecc_desc.ecc_irqs[i];
			break;
		}
	}
	return desc;
}

static irqreturn_t exynos_ecc_handler(int irq, void *dev_id)
{
	struct ecc_irq_desc *ecc_irq_desc;
	struct irq_data *data;
	struct irq_desc *desc = NULL;
	int cpu = raw_smp_processor_id();

	data = irq_get_irq_data(irq);
	if (data)
		desc = irq_data_to_desc(data);

	if (desc && desc->action && desc->action->name)
		printk(KERN_ERR "[ECCIRQ] %s\n", desc->action->name);
	else
		printk(KERN_ERR "[ECCIRQ] irq=%d\n", irq);

	ecc_irq_desc = get_ecc_irq_desc(irq);
	if (!ecc_irq_desc)
		panic("Unexpected ECC IRQ(%d)", irq);

	if (!cpumask_test_cpu(cpu, &ecc_irq_desc->affinity))
		panic("ECC error in other cpu");

	dbg_snapshot_ecc_dump(true);

	return IRQ_HANDLED;
}

static int ecc_irq_online_cpu(unsigned int cpu)
{
	unsigned long flags;
	int i;

	raw_spin_lock_irqsave(&ecc_desc.lock, flags);
	for (i = 0; i < ecc_desc.irq_num; i++) {
		struct cpumask affinity;
		int core;

		if (!cpumask_test_cpu(cpu, &ecc_desc.ecc_irqs[i].affinity))
			continue;

		if (cpumask_any_and(&ecc_desc.ecc_irqs[i].affinity, &ecc_desc.ecc_online_cpus) >=
				nr_cpu_ids)
			enable_irq(ecc_desc.ecc_irqs[i].irq);

		cpumask_clear(&affinity);
		for_each_online_cpu(core) {
			if (cpumask_test_cpu(core, &ecc_desc.ecc_irqs[i].affinity))
				cpumask_set_cpu(core, &affinity);
		}
		irq_set_affinity_hint(ecc_desc.ecc_irqs[i].irq, &affinity);
	}
	cpumask_set_cpu(cpu, &ecc_desc.ecc_online_cpus);
	raw_spin_unlock_irqrestore(&ecc_desc.lock, flags);

	return 0;
}

static int ecc_irq_offline_cpu(unsigned int cpu)
{
	unsigned long flags;
	int i;

	raw_spin_lock_irqsave(&ecc_desc.lock, flags);
	cpumask_clear_cpu(cpu, &ecc_desc.ecc_online_cpus);
	for (i = 0; i < ecc_desc.irq_num; i++) {
		if (!cpumask_test_cpu(cpu, &ecc_desc.ecc_irqs[i].affinity))
			continue;
		if (cpumask_any_and(&ecc_desc.ecc_irqs[i].affinity, &ecc_desc.ecc_online_cpus) >=
				nr_cpu_ids)
			disable_irq_nosync(ecc_desc.ecc_irqs[i].irq);
	}
	raw_spin_unlock_irqrestore(&ecc_desc.lock, flags);

	return 0;
}

static int exynos_ecc_handler_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct property *prop;
	const char *name;
	char *buf = (char *)__get_free_page(GFP_KERNEL);
	int irq_num;
	int err = 0, idx = 0;

	if (!buf) {
		err = -ENOMEM;
		goto err_buf_alloc;
	}

	irq_num = of_property_count_strings(np, "interrupt-names");
	if (irq_num <= 0) {
		dev_err(&pdev->dev, "Invalid Interrupt-names property count(%d)\n", irq_num);
		err = -EINVAL;
		goto err_irq_name_counts;
	}

	ecc_desc.ecc_irqs = kzalloc(sizeof(*ecc_desc.ecc_irqs) * irq_num, GFP_KERNEL);
	if (!ecc_desc.ecc_irqs) {
		err = -ENOMEM;
		goto err_desc_alloc;
	}

	cpu_hotplug_disable();

	ecc_desc.irq_num = irq_num;

	of_property_for_each_string(np, "interrupt-names", prop, name) {
		unsigned int irq, val;
		struct cpumask affinity_mask;

		if (!name) {
			dev_err(&pdev->dev, "no such name\n");
			err = -EINVAL;
			break;
		}

		irq = platform_get_irq(pdev, idx);
		err = request_irq(irq, exynos_ecc_handler, IRQF_NOBALANCING, name, NULL);
		if (err) {
			dev_err(&pdev->dev, "unable to request irq%u for ecc handler[%s]\n", irq, name);
			break;
		}
		dev_info(&pdev->dev, "Success to request irq%u for ecc handler[%s]\n", irq, name);

		cpumask_clear(&affinity_mask);
		if (!of_property_read_u32_index(np, "interrupt-affinity", idx, &val)) {
			unsigned long bit, affinity;

			affinity = val;
			for_each_set_bit(bit, &affinity, nr_cpu_ids) {
				cpumask_set_cpu(bit, &affinity_mask);
			}
		} else {
			cpumask_copy(&affinity_mask, cpu_online_mask);
		}

		ecc_desc.ecc_irqs[idx].irq = irq;
		cpumask_copy(&ecc_desc.ecc_irqs[idx].affinity, &affinity_mask);

		cpumap_print_to_pagebuf(true, buf, &affinity_mask);
		dev_info(&pdev->dev, "affinity of irq%d is %s", irq, buf);
		irq_set_affinity_hint(irq, &affinity_mask);
		idx++;
	}

	if (irq_num != idx) {
		int i;

		dev_err(&pdev->dev, "failed, irq_num not matched(%d/%d)\n", idx, irq_num);
		for (i = 0; i < idx; i++)
			free_irq(ecc_desc.ecc_irqs[i].irq, NULL);

		kfree(ecc_desc.ecc_irqs);
		ecc_desc.ecc_irqs = NULL;
		ecc_desc.irq_num = 0;
		goto err_register_irq;
	}

	cpumask_copy(&ecc_desc.ecc_online_cpus, cpu_online_mask);
	raw_spin_lock_init(&ecc_desc.lock);
	cpuhp_setup_state(CPUHP_AP_ONLINE_DYN, "ecc:online",
			  ecc_irq_online_cpu, ecc_irq_offline_cpu);
err_register_irq:
	cpu_hotplug_enable();
err_desc_alloc:
err_irq_name_counts:
	free_page((unsigned long)buf);
err_buf_alloc:
	return err;
}

static const struct of_device_id exynos_ecc_handler_matches[] = {
	{ .compatible = "google,exynos-ecc-handler", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_ecc_handler_matches);

static struct platform_driver exynos_ecc_handler_driver = {
	.probe	= exynos_ecc_handler_probe,
	.driver	= {
		.name	= "exynos-ecc-handler",
		.of_match_table	= of_match_ptr(exynos_ecc_handler_matches),
	},
};
module_platform_driver(exynos_ecc_handler_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ECC Handler Driver");
