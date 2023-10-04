/*
 * Google LWIS Device Tree Parser
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-dt: " fmt

#include "lwis_dt.h"

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>

#include "lwis_clock.h"
#include "lwis_device_dpm.h"
#include "lwis_gpio.h"
#include "lwis_i2c.h"
#include "lwis_ioreg.h"
#include "lwis_regulator.h"

#define SHARED_STRING "shared-"
#define PULSE_STRING "pulse-"

/* Uncomment this to help debug device tree parsing. */
// #define LWIS_DT_DEBUG

static int parse_gpios(struct lwis_device *lwis_dev, char *name, bool *is_present)
{
	int count;
	struct device *dev;
	struct gpio_descs *list;

	*is_present = false;

	dev = &lwis_dev->plat_dev->dev;

	count = gpiod_count(dev, name);

	/* No GPIO pins found, just return */
	if (count <= 0) {
		return 0;
	}

	list = lwis_gpio_list_get(dev, name);
	if (IS_ERR(list)) {
		pr_err("Error parsing GPIO list %s (%ld)\n", name, PTR_ERR(list));
		return PTR_ERR(list);
	}

	/* The GPIO pins are valid, release the list as we do not need to hold
	   on to the pins yet */
	lwis_gpio_list_put(list, dev);
	*is_present = true;
	return 0;
}

static int parse_irq_gpios(struct lwis_device *lwis_dev)
{
	int count;
	int name_count;
	int event_count;
	int ret;
	struct device *dev;
	struct device_node *dev_node;
	struct gpio_descs *gpios;
	const char *name;
	char *irq_gpios_names = NULL;
	u64 *irq_gpios_events = NULL;
	int i;

	/* Initialize the data structure */
	strlcpy(lwis_dev->irq_gpios_info.name, "irq", LWIS_MAX_NAME_STRING_LEN);
	lwis_dev->irq_gpios_info.gpios = NULL;
	lwis_dev->irq_gpios_info.irq_list = NULL;
	lwis_dev->irq_gpios_info.is_shared = false;
	lwis_dev->irq_gpios_info.is_pulse = false;

	dev = &lwis_dev->plat_dev->dev;
	count = gpiod_count(dev, "irq");
	/* No irq GPIO pins found, just return */
	if (count <= 0) {
		return 0;
	}

	dev_node = dev->of_node;
	name_count = of_property_count_strings(dev_node, "irq-gpios-names");
	event_count = of_property_count_elems_of_size(dev_node, "irq-gpios-events", sizeof(u64));
	if (count != event_count || count != name_count) {
		pr_err("Count of irq-gpios-* is not match\n");
		return -EINVAL;
	}

	gpios = lwis_gpio_list_get(dev, "irq");
	if (IS_ERR(gpios)) {
		pr_err("Error parsing irq GPIO list (%ld)\n", PTR_ERR(gpios));
		return PTR_ERR(gpios);
	}
	lwis_dev->irq_gpios_info.gpios = gpios;

	irq_gpios_names = kmalloc(LWIS_MAX_NAME_STRING_LEN * name_count, GFP_KERNEL);
	if (IS_ERR_OR_NULL(irq_gpios_names)) {
		pr_err("Allocating event list failed\n");
		ret = -ENOMEM;
		goto error_parse_irq_gpios;
	}

	for (i = 0; i < name_count; ++i) {
		ret = of_property_read_string_index(dev_node, "irq-gpios-names", i, &name);
		if (ret < 0) {
			pr_err("Error get GPIO irq name list (%d)\n", ret);
			goto error_parse_irq_gpios;
		}
		strlcpy(irq_gpios_names + i * LWIS_MAX_NAME_STRING_LEN, name,
			LWIS_MAX_NAME_STRING_LEN);
	}

	ret = lwis_gpio_list_to_irqs(lwis_dev, &lwis_dev->irq_gpios_info, irq_gpios_names);
	if (ret) {
		pr_err("Error get GPIO irq list (%d)\n", ret);
		goto error_parse_irq_gpios;
	}

	irq_gpios_events = kmalloc(sizeof(u64) * event_count, GFP_KERNEL);
	if (IS_ERR_OR_NULL(irq_gpios_events)) {
		pr_err("Allocating event list failed\n");
		ret = -ENOMEM;
		goto error_parse_irq_gpios;
	}

	event_count = of_property_read_variable_u64_array(
		dev_node, "irq-gpios-events", irq_gpios_events, event_count, event_count);
	if (event_count != count) {
		pr_err("Error getting irq-gpios-events: %d\n", event_count);
		ret = event_count;
		goto error_parse_irq_gpios;
	}

	for (i = 0; i < event_count; ++i) {
		ret = lwis_interrupt_set_gpios_event_info(lwis_dev->irq_gpios_info.irq_list, i,
							  irq_gpios_events[i]);
		if (ret) {
			pr_err("Error setting event info for gpios interrupt %d %d\n", i, ret);
			goto error_parse_irq_gpios;
		}
	}

	kfree(irq_gpios_names);
	kfree(irq_gpios_events);
	return 0;

error_parse_irq_gpios:
	if (lwis_dev->irq_gpios_info.gpios) {
		lwis_gpio_list_put(lwis_dev->irq_gpios_info.gpios, dev);
		lwis_dev->irq_gpios_info.gpios = NULL;
	}
	if (irq_gpios_names) {
		kfree(irq_gpios_names);
	}
	if (irq_gpios_events) {
		kfree(irq_gpios_events);
	}
	return ret;
}

static int parse_settle_time(struct lwis_device *lwis_dev)
{
	struct device_node *dev_node;
	struct device *dev;

	dev = &lwis_dev->plat_dev->dev;
	dev_node = dev->of_node;
	lwis_dev->enable_gpios_settle_time = 0;

	of_property_read_u32(dev_node, "enable-gpios-settle-time",
			     &lwis_dev->enable_gpios_settle_time);
	return 0;
}

static int parse_regulators(struct lwis_device *lwis_dev)
{
	int i;
	int ret;
	int count;
	struct device_node *dev_node;
	struct device_node *dev_node_reg;
	const char *name;
	struct device *dev;
	int voltage;
	int voltage_count;

	dev = &lwis_dev->plat_dev->dev;
	dev_node = dev->of_node;

	count = of_property_count_elems_of_size(dev_node, "regulators", sizeof(u32));

	/* No regulators found, or entry does not exist, just return */
	if (count <= 0) {
		lwis_dev->regulators = NULL;
		return 0;
	}

	/* Voltage count is allowed to be less than regulator count,
	   regulator_set_voltage will not be called for the ones with
	   unspecified voltage */
	voltage_count =
		of_property_count_elems_of_size(dev_node, "regulator-voltages", sizeof(u32));

	lwis_dev->regulators = lwis_regulator_list_alloc(count);
	if (IS_ERR(lwis_dev->regulators)) {
		pr_err("Cannot allocate regulator list\n");
		return PTR_ERR(lwis_dev->regulators);
	}

	/* Parse regulator list and acquire the regulator pointers */
	for (i = 0; i < count; ++i) {
		dev_node_reg = of_parse_phandle(dev_node, "regulators", i);
		of_property_read_string(dev_node_reg, "regulator-name", &name);
		voltage = 0;
		if (i < voltage_count) {
			of_property_read_u32_index(dev_node, "regulator-voltages", i, &voltage);
		}
		ret = lwis_regulator_get(lwis_dev->regulators, (char *)name, voltage, dev);
		if (ret < 0) {
			pr_err("Cannot find regulator: %s\n", name);
			goto error_parse_reg;
		}
	}

#ifdef LWIS_DT_DEBUG
	lwis_regulator_print(lwis_dev->regulators);
#endif

	return 0;

error_parse_reg:
	/* In case of error, free all the other regulators that were alloc'ed */
	lwis_regulator_put_all(lwis_dev->regulators);
	lwis_regulator_list_free(lwis_dev->regulators);
	lwis_dev->regulators = NULL;
	return ret;
}

static int parse_clocks(struct lwis_device *lwis_dev)
{
	int i;
	int ret = 0;
	int count;
	struct device *dev;
	struct device_node *dev_node;
	const char *name;
	u32 rate;
	int clock_family;

	dev = &lwis_dev->plat_dev->dev;
	dev_node = dev->of_node;

	count = of_property_count_strings(dev_node, "clock-names");

	/* No clocks found, just return */
	if (count <= 0) {
		lwis_dev->clocks = NULL;
		return 0;
	}

	lwis_dev->clocks = lwis_clock_list_alloc(count);
	if (IS_ERR(lwis_dev->clocks)) {
		pr_err("Cannot allocate clocks list\n");
		return PTR_ERR(lwis_dev->clocks);
	}

	/* Parse and acquire clock pointers and frequencies, if applicable */
	for (i = 0; i < count; ++i) {
		of_property_read_string_index(dev_node, "clock-names", i, &name);
		/* It is allowed to omit clock rates for some of the clocks */
		ret = of_property_read_u32_index(dev_node, "clock-rates", i, &rate);
		rate = (ret == 0) ? rate : 0;

		ret = lwis_clock_get(lwis_dev->clocks, (char *)name, dev, rate);
		if (ret < 0) {
			pr_err("Cannot find clock: %s\n", name);
			goto error_parse_clk;
		}
	}

	/* It is allowed to omit clock rates for some of the clocks */
	ret = of_property_read_u32(dev_node, "clock-family", &clock_family);
	lwis_dev->clock_family = (ret == 0) ? clock_family : CLOCK_FAMILY_INVALID;

#ifdef LWIS_DT_DEBUG
	pr_info("%s: clock family %d", lwis_dev->name, lwis_dev->clock_family);
	lwis_clock_print(lwis_dev->clocks);
#endif

	return 0;

error_parse_clk:
	/* Put back the clock instances for the ones that were alloc'ed */
	for (i = 0; i < count; ++i) {
		lwis_clock_put_by_idx(lwis_dev->clocks, i, dev);
	}
	lwis_clock_list_free(lwis_dev->clocks);
	lwis_dev->clocks = NULL;
	return ret;
}

static int parse_pinctrls(struct lwis_device *lwis_dev, char *expected_state)
{
	int count;
	struct device *dev;
	struct device_node *dev_node;
	struct pinctrl *pc;
	struct pinctrl_state *pinctrl_state;

	dev = &lwis_dev->plat_dev->dev;
	dev_node = dev->of_node;

	lwis_dev->mclk_present = false;
	lwis_dev->shared_pinctrl = 0;
	count = of_property_count_strings(dev_node, "pinctrl-names");

	/* No pinctrl found, just return */
	if (count <= 0)
		return 0;

	/* Set up pinctrl */
	pc = devm_pinctrl_get(dev);
	if (IS_ERR(pc)) {
		pr_err("Cannot allocate pinctrl\n");
		return PTR_ERR(pc);
	}

	pinctrl_state = pinctrl_lookup_state(pc, expected_state);
	if (IS_ERR(pinctrl_state)) {
		pr_err("Cannot find pinctrl state %s\n", expected_state);
		devm_pinctrl_put(pc);
		return PTR_ERR(pinctrl_state);
	}

	/* Indicate if the pinctrl shared with other devices */
	of_property_read_u32(dev_node, "shared-pinctrl", &lwis_dev->shared_pinctrl);

	/* The pinctrl is valid, release it as we do not need to hold on to
	   the pins yet */
	devm_pinctrl_put(pc);

	lwis_dev->mclk_present = true;

	return 0;
}

static int parse_critical_irq_events(struct device_node *event_info, u64** irq_events)
{
	int ret;
	int critical_irq_events_num;
	u64 critical_irq_events;
	int i;

	critical_irq_events_num =
		of_property_count_elems_of_size(event_info, "critical-irq-events", 8);
	/* No Critical IRQ event found, just return */
	if (critical_irq_events_num <= 0) {
		return 0;
	}

	*irq_events = kmalloc(sizeof(u64) * critical_irq_events_num, GFP_KERNEL);
	if (*irq_events == NULL) {
		pr_err("Failed to allocate memory for critical events\n");
		return 0;
	}

	for (i = 0; i < critical_irq_events_num; ++i) {
		ret = of_property_read_u64_index(event_info, "critical-irq-events", i,
						 &critical_irq_events);
		if (ret < 0) {
			pr_err("Error adding critical irq events[%d]\n", i);
			kfree(*irq_events);
			*irq_events = NULL;
			return 0;
		}
		*irq_events[i] = critical_irq_events;
	}

	return critical_irq_events_num;
}

static int parse_interrupts(struct lwis_device *lwis_dev)
{
	int i;
	int ret;
	int count, event_infos_count;
	const char *name;
	struct device_node *dev_node;
	struct platform_device *plat_dev;
	struct of_phandle_iterator it;

	plat_dev = lwis_dev->plat_dev;
	dev_node = plat_dev->dev.of_node;

	count = platform_irq_count(plat_dev);

	/* No interrupts found, just return */
	if (count <= 0) {
		lwis_dev->irqs = NULL;
		return 0;
	}

	lwis_dev->irqs = lwis_interrupt_list_alloc(lwis_dev, count);
	if (IS_ERR(lwis_dev->irqs)) {
		pr_err("Failed to allocate IRQ list\n");
		return PTR_ERR(lwis_dev->irqs);
	}

	for (i = 0; i < count; ++i) {
		of_property_read_string_index(dev_node, "interrupt-names", i, &name);
		ret = lwis_interrupt_get(lwis_dev->irqs, i, (char *)name, plat_dev);
		if (ret) {
			pr_err("Cannot set irq %s\n", name);
			goto error_get_irq;
		}
	}

	event_infos_count = of_property_count_elems_of_size(dev_node, "interrupt-event-infos", 4);
	if (count != event_infos_count) {
		pr_err("DT numbers of irqs: %d != event infos: %d in DT\n", count,
		       event_infos_count);
		ret = -EINVAL;
		goto error_get_irq;
	}
	/* Get event infos */
	i = 0;
	of_for_each_phandle (&it, ret, dev_node, "interrupt-event-infos", 0, 0) {
		const char *irq_reg_space = NULL;
		bool irq_mask_reg_toggle;
		u64 irq_src_reg;
		u64 irq_reset_reg;
		u64 irq_mask_reg;
		int irq_events_num;
		int int_reg_bits_num;
		int critical_events_num = 0;
		u64 *irq_events;
		u32 *int_reg_bits;
		u64 *critical_events = NULL;
		int irq_reg_bid = -1;
		int irq_reg_bid_count;
		/* To match default value of reg-addr/value-bitwidth. */
		u32 irq_reg_bitwidth = 32;
		int j;
		struct device_node *event_info = of_node_get(it.node);

		irq_events_num = of_property_count_elems_of_size(event_info, "irq-events", 8);
		if (irq_events_num <= 0) {
			pr_err("Error getting irq-events: %d\n", irq_events_num);
			ret = -EINVAL;
			goto error_event_infos;
		}

		int_reg_bits_num = of_property_count_elems_of_size(event_info, "int-reg-bits", 4);
		if (irq_events_num != int_reg_bits_num || int_reg_bits_num <= 0) {
			pr_err("Error getting int-reg-bits: %d\n", int_reg_bits_num);
			ret = -EINVAL;
			goto error_event_infos;
		}

		irq_events = kmalloc(sizeof(u64) * irq_events_num, GFP_KERNEL);
		if (IS_ERR_OR_NULL(irq_events)) {
			ret = -ENOMEM;
			goto error_event_infos;
		}

		int_reg_bits = kmalloc(sizeof(u32) * int_reg_bits_num, GFP_KERNEL);
		if (IS_ERR_OR_NULL(int_reg_bits)) {
			ret = -ENOMEM;
			kfree(irq_events);
			goto error_event_infos;
		}

		irq_events_num = of_property_read_variable_u64_array(
			event_info, "irq-events", irq_events, irq_events_num, irq_events_num);
		if (irq_events_num != int_reg_bits_num) {
			pr_err("Error getting irq-events: %d\n", irq_events_num);
			ret = irq_events_num;
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		int_reg_bits_num =
			of_property_read_variable_u32_array(event_info, "int-reg-bits",
							    int_reg_bits, int_reg_bits_num,
							    int_reg_bits_num);
		if (irq_events_num != int_reg_bits_num) {
			pr_err("Error getting int-reg-bits: %d\n", int_reg_bits_num);
			ret = int_reg_bits_num;
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		ret = of_property_read_string(event_info, "irq-reg-space", &irq_reg_space);
		if (ret) {
			pr_err("Error getting irq-reg-space from dt: %d\n", ret);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		irq_reg_bid_count = of_property_count_strings(dev_node, "reg-names");

		if (irq_reg_bid_count <= 0) {
			pr_err("Error getting reg-names from dt: %d\n", irq_reg_bid_count);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}
		for (j = 0; j < irq_reg_bid_count; j++) {
			const char *bid_name;
			ret = of_property_read_string_index(dev_node, "reg-names", j, &bid_name);

			if (ret) {
				break;
			}
			if (!strcmp(bid_name, irq_reg_space)) {
				irq_reg_bid = j;
				break;
			}
		}
		if (irq_reg_bid < 0) {
			pr_err("Could not find a reg bid for %s\n", irq_reg_space);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		ret = of_property_read_u64(event_info, "irq-src-reg", &irq_src_reg);
		if (ret) {
			pr_err("Error getting irq-src-reg from dt: %d\n", ret);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		ret = of_property_read_u64(event_info, "irq-reset-reg", &irq_reset_reg);
		if (ret) {
			pr_err("Error getting irq-reset-reg from dt: %d\n", ret);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		ret = of_property_read_u64(event_info, "irq-mask-reg", &irq_mask_reg);
		if (ret) {
			pr_err("Error getting irq-mask-reg from dt: %d\n", ret);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		irq_mask_reg_toggle = of_property_read_bool(event_info, "irq-mask-reg-toggle");

		of_property_read_u32(event_info, "irq-reg-bitwidth", &irq_reg_bitwidth);

		critical_events_num = parse_critical_irq_events(event_info, &critical_events);

		ret = lwis_interrupt_set_event_info(
			lwis_dev->irqs, i, irq_reg_space, irq_reg_bid, (int64_t *)irq_events,
			irq_events_num, int_reg_bits, int_reg_bits_num, irq_src_reg, irq_reset_reg,
			irq_mask_reg, irq_mask_reg_toggle, irq_reg_bitwidth,
			(int64_t *)critical_events, critical_events_num);
		if (ret) {
			pr_err("Error setting event info for interrupt %d %d\n", i, ret);
			if (critical_events) {
				kfree(critical_events);
			}
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		of_node_put(event_info);
		i++;
		if (critical_events) {
			kfree(critical_events);
		}
		kfree(irq_events);
		kfree(int_reg_bits);
	}

#ifdef LWIS_DT_DEBUG
	lwis_interrupt_print(lwis_dev->irqs);
#endif

	return 0;
error_event_infos:
	for (i = 0; i < count; ++i) {
		// TODO(yromanenko): lwis_interrupt_put
	}
error_get_irq:
	lwis_interrupt_list_free(lwis_dev->irqs);
	lwis_dev->irqs = NULL;
	return ret;
}

static int parse_phys(struct lwis_device *lwis_dev)
{
	struct device *dev;
	struct device_node *dev_node;
	int i;
	int ret;
	int count;
	const char *name;

	dev = &(lwis_dev->plat_dev->dev);
	dev_node = dev->of_node;

	count = of_count_phandle_with_args(dev_node, "phys", "#phy-cells");

	/* No PHY found, just return */
	if (count <= 0) {
		lwis_dev->phys = NULL;
		return 0;
	}

	lwis_dev->phys = lwis_phy_list_alloc(count);
	if (IS_ERR(lwis_dev->phys)) {
		pr_err("Failed to allocate PHY list\n");
		return PTR_ERR(lwis_dev->phys);
	}

	for (i = 0; i < count; ++i) {
		of_property_read_string_index(dev_node, "phy-names", i, &name);
		ret = lwis_phy_get(lwis_dev->phys, (char *)name, dev);
		if (ret < 0) {
			pr_err("Error adding PHY[%d]\n", i);
			goto error_parse_phy;
		}
	}

#ifdef LWIS_DT_DEBUG
	lwis_phy_print(lwis_dev->phys);
#endif

	return 0;

error_parse_phy:
	for (i = 0; i < count; ++i) {
		lwis_phy_put_by_idx(lwis_dev->phys, i, dev);
	}
	lwis_phy_list_free(lwis_dev->phys);
	lwis_dev->phys = NULL;
	return ret;
}

static void parse_bitwidths(struct lwis_device *lwis_dev)
{
	struct device *dev;
	struct device_node *dev_node;
	u32 addr_bitwidth = 32;
	u32 value_bitwidth = 32;

	dev = &(lwis_dev->plat_dev->dev);
	dev_node = dev->of_node;

	of_property_read_u32(dev_node, "reg-addr-bitwidth", &addr_bitwidth);
#ifdef LWIS_DT_DEBUG
	pr_info("Addr bitwidth set to: %d\n", addr_bitwidth);
#endif

	of_property_read_u32(dev_node, "reg-value-bitwidth", &value_bitwidth);
#ifdef LWIS_DT_DEBUG
	pr_info("Value bitwidth set to: %d\n", value_bitwidth);
#endif

	lwis_dev->native_addr_bitwidth = addr_bitwidth;
	lwis_dev->native_value_bitwidth = value_bitwidth;
}

static int parse_power_seqs(struct lwis_device *lwis_dev, const char *seq_name,
			    struct lwis_device_power_sequence_list **list)
{
	char str_seq_name[LWIS_MAX_NAME_STRING_LEN];
	char str_seq_type[LWIS_MAX_NAME_STRING_LEN];
	char str_seq_delay[LWIS_MAX_NAME_STRING_LEN];
	struct device *dev;
	struct device_node *dev_node;
	int power_seq_count;
	int power_seq_type_count;
	int power_seq_delay_count;
	int i;
	int ret;
	const char *name;
	const char *type;
	int delay_us;
	int type_gpio_count = 0;
	int type_regulator_count = 0;

	scnprintf(str_seq_name, LWIS_MAX_NAME_STRING_LEN, "%s-seqs", seq_name);
	scnprintf(str_seq_type, LWIS_MAX_NAME_STRING_LEN, "%s-seq-types", seq_name);
	scnprintf(str_seq_delay, LWIS_MAX_NAME_STRING_LEN, "%s-seq-delays-us", seq_name);

	dev = &lwis_dev->plat_dev->dev;
	dev_node = dev->of_node;
	*list = NULL;

	power_seq_count = of_property_count_strings(dev_node, str_seq_name);
	power_seq_type_count = of_property_count_strings(dev_node, str_seq_type);
	power_seq_delay_count =
		of_property_count_elems_of_size(dev_node, str_seq_delay, sizeof(u32));

	/* No power-seqs found, just return */
	if (power_seq_count <= 0) {
		return 0;
	}
	if (power_seq_count != power_seq_type_count || power_seq_count != power_seq_delay_count) {
		pr_err("Count of power sequence %s is not match\n", str_seq_name);
		return -EINVAL;
	}

	*list = lwis_dev_power_seq_list_alloc(power_seq_count);
	if (IS_ERR(*list)) {
		pr_err("Failed to allocate power sequence list\n");
		return PTR_ERR(*list);
	}

	for (i = 0; i < power_seq_count; ++i) {
		ret = of_property_read_string_index(dev_node, str_seq_name, i, &name);
		if (ret < 0) {
			pr_err("Error adding power sequence[%d]\n", i);
			goto error_parse_power_seqs;
		}
		strscpy((*list)->seq_info[i].name, name, LWIS_MAX_NAME_STRING_LEN);

		ret = of_property_read_string_index(dev_node, str_seq_type, i, &type);
		if (ret < 0) {
			pr_err("Error adding power sequence type[%d]\n", i);
			goto error_parse_power_seqs;
		}
		strscpy((*list)->seq_info[i].type, type, LWIS_MAX_NAME_STRING_LEN);
		if (strcmp(type, "gpio") == 0) {
			type_gpio_count++;
		} else if (strcmp(type, "regulator") == 0) {
			type_regulator_count++;
		}

		ret = of_property_read_u32_index(dev_node, str_seq_delay, i, &delay_us);
		if (ret < 0) {
			pr_err("Error adding power sequence delay[%d]\n", i);
			goto error_parse_power_seqs;
		}
		(*list)->seq_info[i].delay_us = delay_us;
	}

#ifdef LWIS_DT_DEBUG
	lwis_dev_power_seq_list_print(*list);
#endif

	if (type_gpio_count > 0 && lwis_dev->gpios_list == NULL) {
		lwis_dev->gpios_list = lwis_gpios_list_alloc(type_gpio_count);
		if (IS_ERR(lwis_dev->gpios_list)) {
			pr_err("Failed to allocate gpios list\n");
			ret = PTR_ERR(lwis_dev->gpios_list);
			goto error_parse_power_seqs;
		}

		type_gpio_count = 0;
		for (i = 0; i < power_seq_count; ++i) {
			struct lwis_gpios_info *gpios_info;
			char *seq_item_name;
			struct device *dev;
			struct gpio_descs *descs;

			if (strcmp((*list)->seq_info[i].type, "gpio") != 0) {
				continue;
			}

			gpios_info = &lwis_dev->gpios_list->gpios_info[type_gpio_count];
			seq_item_name = (*list)->seq_info[i].name;
			dev = &lwis_dev->plat_dev->dev;
			descs = lwis_gpio_list_get(dev, seq_item_name);
			if (IS_ERR(descs)) {
				pr_err("Error parsing GPIO list %s (%ld)\n", seq_item_name,
				       PTR_ERR(descs));
				ret = PTR_ERR(descs);
				goto error_parse_power_seqs;
			}
			gpios_info->id = desc_to_gpio(descs->desc[0]);
			gpios_info->hold_dev = dev;
			/*
			 * The GPIO pins are valid, release the list as we do not need to hold
			 * on to the pins yet
			 */
			lwis_gpio_list_put(descs, dev);

			gpios_info->gpios = NULL;
			gpios_info->irq_list = NULL;
			strlcpy(gpios_info->name, seq_item_name, LWIS_MAX_NAME_STRING_LEN);

			if (strncmp(SHARED_STRING, seq_item_name, strlen(SHARED_STRING)) == 0) {
				gpios_info->is_shared = true;
			} else {
				gpios_info->is_shared = false;
			}
			if (strncmp(PULSE_STRING, seq_item_name, strlen(PULSE_STRING)) == 0) {
				gpios_info->is_pulse = true;
			} else {
				gpios_info->is_pulse = false;
			}
			type_gpio_count++;
		}
	}

	if (type_regulator_count > 0 && lwis_dev->regulators == NULL) {
		lwis_dev->regulators = lwis_regulator_list_alloc(type_regulator_count);
		if (IS_ERR(lwis_dev->regulators)) {
			pr_err("Failed to allocate regulator list\n");
			ret = PTR_ERR(lwis_dev->regulators);
			goto error_parse_power_seqs;
		}

		for (i = 0; i < power_seq_count; ++i) {
			struct device *dev;
			char *seq_item_name;

			if (strcmp((*list)->seq_info[i].type, "regulator") != 0) {
				continue;
			}

			dev = &lwis_dev->plat_dev->dev;
			seq_item_name = (*list)->seq_info[i].name;

			ret = lwis_regulator_get(lwis_dev->regulators, seq_item_name,
						 /*voltage=*/0, dev);
			if (ret < 0) {
				pr_err("Cannot find regulator: %s\n", seq_item_name);
				goto error_parse_power_seqs;
			}
		}
	}

	return 0;

error_parse_power_seqs:
	if (lwis_dev->regulators) {
		lwis_regulator_put_all(lwis_dev->regulators);
		lwis_regulator_list_free(lwis_dev->regulators);
		lwis_dev->regulators = NULL;
	}
	lwis_gpios_list_free(lwis_dev->gpios_list);
	lwis_dev->gpios_list = NULL;
	lwis_dev_power_seq_list_free(*list);
	*list = NULL;
	return ret;
}

static int parse_pm_hibernation(struct lwis_device *lwis_dev)
{
	struct device_node *dev_node;

	dev_node = lwis_dev->plat_dev->dev.of_node;
	lwis_dev->pm_hibernation = 1;

	of_property_read_u32(dev_node, "pm-hibernation", &lwis_dev->pm_hibernation);

	return 0;
}

static int parse_access_mode(struct lwis_device *lwis_dev)
{
	struct device_node *dev_node;

	dev_node = lwis_dev->plat_dev->dev.of_node;

	lwis_dev->is_read_only = of_property_read_bool(dev_node, "lwis,read-only");

	return 0;
}

static int parse_thread_priority(struct lwis_device *lwis_dev)
{
	struct device_node *dev_node;

	dev_node = lwis_dev->plat_dev->dev.of_node;
	lwis_dev->transaction_thread_priority = 0;
	lwis_dev->periodic_io_thread_priority = 0;

	of_property_read_u32(dev_node, "transaction-thread-priority",
			     &lwis_dev->transaction_thread_priority);
	of_property_read_u32(dev_node, "periodic-io-thread-priority",
			     &lwis_dev->periodic_io_thread_priority);

	return 0;
}

static int parse_i2c_lock_group_id(struct lwis_i2c_device *i2c_dev)
{
	struct device_node *dev_node;
	int ret;

	dev_node = i2c_dev->base_dev.plat_dev->dev.of_node;
	/* Set i2c_lock_group_id value to default */
	i2c_dev->i2c_lock_group_id = MAX_I2C_LOCK_NUM - 1;

	ret = of_property_read_u32(dev_node, "i2c-lock-group-id", &i2c_dev->i2c_lock_group_id);
	/* If no property in device tree, just return to use default */
	if (ret == -EINVAL) {
		return 0;
	}
	if (ret) {
		pr_err("i2c-lock-group-id value wrong\n");
		return ret;
	}
	if (i2c_dev->i2c_lock_group_id >= MAX_I2C_LOCK_NUM - 1) {
		pr_err("i2c-lock-group-id need smaller than MAX_I2C_LOCK_NUM - 1\n");
		return -EOVERFLOW;
	}

	return 0;
}

int lwis_base_parse_dt(struct lwis_device *lwis_dev)
{
	struct device *dev;
	struct device_node *dev_node;
	const char *name_str;
	int ret = 0;

	dev = &(lwis_dev->plat_dev->dev);
	dev_node = dev->of_node;

	if (!dev_node) {
		pr_err("Cannot find device node\n");
		return -ENODEV;
	}

	ret = of_property_read_string(dev_node, "node-name", &name_str);
	if (ret) {
		pr_err("Error parsing node name\n");
		return -EINVAL;
	}
	strlcpy(lwis_dev->name, name_str, LWIS_MAX_NAME_STRING_LEN);

	pr_debug("Device tree entry [%s] - begin\n", lwis_dev->name);

	ret = parse_gpios(lwis_dev, "shared-enable", &lwis_dev->shared_enable_gpios_present);
	if (ret) {
		pr_err("Error parsing shared-enable-gpios\n");
		return ret;
	}

	ret = parse_gpios(lwis_dev, "enable", &lwis_dev->enable_gpios_present);
	if (ret) {
		pr_err("Error parsing enable-gpios\n");
		return ret;
	}

	ret = parse_gpios(lwis_dev, "reset", &lwis_dev->reset_gpios_present);
	if (ret) {
		pr_err("Error parsing reset-gpios\n");
		return ret;
	}

	ret = parse_irq_gpios(lwis_dev);
	if (ret) {
		pr_err("Error parsing irq-gpios\n");
		return ret;
	}

	ret = parse_power_seqs(lwis_dev, "power-up", &lwis_dev->power_up_sequence);
	if (ret) {
		pr_err("Error parsing power-up-seqs\n");
		return ret;
	}

	ret = parse_power_seqs(lwis_dev, "power-down", &lwis_dev->power_down_sequence);
	if (ret) {
		pr_err("Error parsing power-down-seqs\n");
		return ret;
	}

	ret = parse_settle_time(lwis_dev);
	if (ret) {
		pr_err("Error parsing settle-time\n");
		return ret;
	}

	if (lwis_dev->regulators == NULL) {
		ret = parse_regulators(lwis_dev);
		if (ret) {
			pr_err("Error parsing regulators\n");
			return ret;
		}
	}

	ret = parse_clocks(lwis_dev);
	if (ret) {
		pr_err("Error parsing clocks\n");
		return ret;
	}

	ret = parse_pinctrls(lwis_dev, "mclk_on");
	if (ret) {
		pr_err("Error parsing mclk pinctrls\n");
		return ret;
	}

	ret = parse_interrupts(lwis_dev);
	if (ret) {
		pr_err("Error parsing interrupts\n");
		return ret;
	}

	ret = parse_phys(lwis_dev);
	if (ret) {
		pr_err("Error parsing phy's\n");
		return ret;
	}

	ret = parse_pm_hibernation(lwis_dev);
	if (ret) {
		pr_err("Error parsing pm hibernation\n");
		return ret;
	}

	parse_access_mode(lwis_dev);
	parse_thread_priority(lwis_dev);
	parse_bitwidths(lwis_dev);

	lwis_dev->bts_scenario_name = NULL;
	of_property_read_string(dev_node, "bts-scenario", &lwis_dev->bts_scenario_name);

	dev_node->data = lwis_dev;

	pr_debug("Device tree entry [%s] - end\n", lwis_dev->name);

	return ret;
}

int lwis_i2c_device_parse_dt(struct lwis_i2c_device *i2c_dev)
{
	struct device_node *dev_node;
	struct device_node *dev_node_i2c;
	int ret;

	dev_node = i2c_dev->base_dev.plat_dev->dev.of_node;

	dev_node_i2c = of_parse_phandle(dev_node, "i2c-bus", 0);
	if (!dev_node_i2c) {
		dev_err(i2c_dev->base_dev.dev, "Cannot find i2c-bus node\n");
		return -ENODEV;
	}

	i2c_dev->adapter = of_find_i2c_adapter_by_node(dev_node_i2c);
	if (!i2c_dev->adapter) {
		dev_err(i2c_dev->base_dev.dev, "Cannot find i2c adapter\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(dev_node, "i2c-addr", (u32 *)&i2c_dev->address);
	if (ret) {
		dev_err(i2c_dev->base_dev.dev, "Failed to read i2c-addr\n");
		return ret;
	}

	ret = parse_i2c_lock_group_id(i2c_dev);
	if (ret) {
		dev_err(i2c_dev->base_dev.dev, "Error parsing i2c lock group id\n");
		return ret;
	}

	return 0;
}

int lwis_ioreg_device_parse_dt(struct lwis_ioreg_device *ioreg_dev)
{
	struct device_node *dev_node;
	int i;
	int ret;
	int blocks;
	int reg_tuple_size;
	const char *name;

	dev_node = ioreg_dev->base_dev.plat_dev->dev.of_node;
	reg_tuple_size = of_n_addr_cells(dev_node) + of_n_size_cells(dev_node);

	blocks = of_property_count_elems_of_size(dev_node, "reg", reg_tuple_size * sizeof(u32));
	if (blocks <= 0) {
		dev_err(ioreg_dev->base_dev.dev, "No register space found\n");
		return -EINVAL;
	}

	ret = lwis_ioreg_list_alloc(ioreg_dev, blocks);
	if (ret) {
		dev_err(ioreg_dev->base_dev.dev, "Failed to allocate ioreg list\n");
		return ret;
	}

	for (i = 0; i < blocks; ++i) {
		of_property_read_string_index(dev_node, "reg-names", i, &name);
		ret = lwis_ioreg_get(ioreg_dev, i, (char *)name);
		if (ret) {
			dev_err(ioreg_dev->base_dev.dev, "Cannot set ioreg info for %s\n", name);
			goto error_ioreg;
		}
	}

	return 0;

error_ioreg:
	for (i = 0; i < blocks; ++i) {
		lwis_ioreg_put_by_idx(ioreg_dev, i);
	}
	lwis_ioreg_list_free(ioreg_dev);
	return ret;
}

int lwis_top_device_parse_dt(struct lwis_top_device *top_dev)
{
	/* To be implemented */
	return 0;
}
