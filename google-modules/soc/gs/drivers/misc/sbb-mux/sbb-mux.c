// SPDX-License-Identifier: GPL-2.0
/*
 * SideBand Bit Multiplexer (SBBM).
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/bitops.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <trace/hooks/systrace.h>

#include "sbb-mux.h"

static struct kobject *primary_sysfs_folder;
static struct kobject *gpios_sysfs_folder;
static struct kobject *signals_sysfs_folder;

static struct sbb_signal_tracker signal_trackers[SBB_SIG_NUM_SIGNALS] = { 0 };

static struct platform_device *platform_dev;
static struct sbb_gpio_tracker *gpio_trackers;
static int sbb_num_gpios;

static struct of_device_id sbb_mux_of_match[] = {
	{
		.compatible = "google,sbb-mux",
	},
	{}
};

static struct platform_driver sbb_mux_platform_driver = {
	.probe = sbb_mux_drv_probe,
	.remove = sbb_mux_drv_remove,
	.driver = {
		.name = "sbb-mux",
		.owner = THIS_MODULE,
		.of_match_table = sbb_mux_of_match,
	},
};

static int sbb_signal_find(const char *name)
{
	int i;

	if (!name)
		return -EINVAL;

	for (i = 0; i < SBB_SIG_NUM_SIGNALS; i++) {
		if (sysfs_streq(signals[i].name, name))
			return i;
	}

	return -EINVAL;
}

static ssize_t sbb_signal_id_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	enum sbbm_signal_id signal_id = ((struct ext_kobj_attribute *)attr)->id;

	return scnprintf(buf, PAGE_SIZE, "%d\n", signal_id);
}

static ssize_t sbb_signal_type_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	static const char *signal_type_strings[SBB_SIG_NUM_SIGNALS] = {
		"constant", "kernel", "userland"
	};
	int signal_id = ((struct ext_kobj_attribute *)attr)->id;
	int signal_type = signals[signal_id].type;

	return scnprintf(buf, PAGE_SIZE, "%s\n", signal_type_strings[signal_type]);
}

static ssize_t sbb_signal_value_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	enum sbbm_signal_id signal_id = ((struct ext_kobj_attribute *)attr)->id;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 signal_trackers[signal_id].value);
}

int sbbm_signal_update(enum sbbm_signal_id signal_id, bool value)
{
	if (signal_id < 0 || signal_id >= SBB_SIG_NUM_SIGNALS ||
	    signals[signal_id].type != KERNEL_DRIVEN)
		return -EINVAL;

	return sbb_signal_set_value(signal_id, !!value);
}

static int sbb_signal_set_value(enum sbbm_signal_id signal_id, int value)
{
	unsigned long flags;
	struct sbb_signal_tracker *tracker = &signal_trackers[signal_id];
	int gpio_id;

	spin_lock_irqsave(&tracker->lock, flags);

	__ATRACE_INT_PID(1, signals[signal_id].name, value);

	if (tracker->value == value) {
		spin_unlock_irqrestore(&tracker->lock, flags);
		return 0;
	}

	if (value != SBB_REFRESH_VALUE) {
		tracker->value = value;
		tracker->toggle_count++;
	}

	for_each_set_bit(gpio_id, &tracker->assigned_gpios_mask,
			 BITS_PER_LONG) {
		/*
		 * NB: locking the GPIO's lock is not necessary. It protects
		 * the GPIO -> signal association, but not the GPIO value.
		 * The GPIO value is implicitly protected by needing to hold
		 * the GPIO's signal's lock before changing the tracked signal.
		 */
		__ATRACE_INT_PID(1, gpio_trackers[gpio_id].name, tracker->value);
		gpiod_set_value(gpio_trackers[gpio_id].gd, tracker->value);
	}

	spin_unlock_irqrestore(&tracker->lock, flags);

	return 0;
}

static ssize_t sbb_signal_value_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t buf_len)
{
	enum sbbm_signal_id signal_id = ((struct ext_kobj_attribute *)attr)->id;
	char input_val;

	if (buf_len < 1) {
		pr_err("sbb-mux: Invalid store size of %zu for signal %s!",
		       buf_len, signals[signal_id].name);
		return -EINVAL;
	}

	input_val = buf[0];
	if (input_val != '0' && input_val != '1') {
		pr_err("sbb-mux: Expecting value of '0' or '1' for signal %s!",
		       signals[signal_id].name);
		return -EINVAL;
	}

	sbb_signal_set_value(signal_id, input_val == '1');

	return buf_len;
}

static ssize_t sbb_signal_toggle_count_show(struct kobject *kobj,
					    struct kobj_attribute *attr,
					    char *buf)
{
	enum sbbm_signal_id signal_id = ((struct ext_kobj_attribute *)attr)->id;

	return scnprintf(buf, PAGE_SIZE, "%lu\n",
			 signal_trackers[signal_id].toggle_count);
}

static ssize_t sbb_signal_assigned_gpios_show(struct kobject *kobj,
					      struct kobj_attribute *attr,
					      char *buf)
{
	enum sbbm_signal_id signal_id = ((struct ext_kobj_attribute *)attr)->id;
	struct sbb_signal_tracker *tracker = &signal_trackers[signal_id];
	int remaining_buffer_space = PAGE_SIZE;
	char *buf_pos = buf;
	unsigned long flags;
	unsigned long gpio_mask;
	int gpio_id;

	spin_lock_irqsave(&tracker->lock, flags);
	gpio_mask = tracker->assigned_gpios_mask;
	spin_unlock_irqrestore(&tracker->lock, flags);

	for_each_set_bit(gpio_id, &gpio_mask, BITS_PER_LONG) {
		int num_written_bytes;

		num_written_bytes =
			scnprintf(buf_pos, remaining_buffer_space, "%s\n",
				  gpio_trackers[gpio_id].name);

		if (num_written_bytes <= 0)
			break;

		remaining_buffer_space -= num_written_bytes;
		buf_pos += num_written_bytes;
	}

	return buf_pos - buf;
}

static int sbb_mux_init_sysfs_file(
	struct sysfs_file *file, struct kobject *parent, enum sbbm_signal_id id,
	const char *file_name, int mode,
	ssize_t (*show)(struct kobject *, struct kobj_attribute *, char *),
	ssize_t (*store)(struct kobject *, struct kobj_attribute *,
			 const char *, size_t))
{
	if (file->exists) {
		pr_err("File %s already exists for parent %p!", file_name,
		       parent);
		return -EEXIST;
	}

	sysfs_attr_init(&file->extended_attribute.base_attribute.attr);

	file->extended_attribute.base_attribute.attr.name = file_name;
	file->extended_attribute.base_attribute.attr.mode = mode;
	file->extended_attribute.base_attribute.show = show;
	file->extended_attribute.base_attribute.store = store;

	file->extended_attribute.id = id;

	file->exists = !sysfs_create_file(
		parent, &file->extended_attribute.base_attribute.attr);

	return !file->exists;
}

static void sbb_mux_cleanup_sysfs_file(struct kobject *parent,
				       struct sysfs_file *file)
{
	if (!file->exists)
		return;

	sysfs_remove_file(parent,
			  &file->extended_attribute.base_attribute.attr);
	file->exists = false;
}

static int sbb_mux_initialize_tracker(struct sbb_signal_tracker *tracker,
				      enum sbbm_signal_id signal_id)
{
	pr_info("sbb-mux: Initializing signal [%d, '%s']...", signal_id,
		signals[signal_id].name);

	tracker->signal_id = signal_id;

	spin_lock_init(&tracker->lock);

	tracker->sysfs_folder = kobject_create_and_add(signals[signal_id].name,
						       signals_sysfs_folder);
	if (!tracker->sysfs_folder) {
		pr_err("sbb-mux: Failed to create sysfs subfolder for "
		       "signal %s!",
		       signals[signal_id].name);
		return -EINVAL;
	}

	if (sbb_mux_init_sysfs_file(&tracker->id_file, tracker->sysfs_folder,
				    signal_id, "id", 0444, sbb_signal_id_show,
				    NULL)) {
		pr_err("Could not create 'id' sysfs file for signal %s!",
		       signals[signal_id].name);
		return -EINVAL;
	}

	if (sbb_mux_init_sysfs_file(&tracker->type_file, tracker->sysfs_folder,
				    signal_id, "type", 0444,
				    sbb_signal_type_show, NULL)) {
		pr_err("Could not create 'type' sysfs file for signal %s!",
		       signals[signal_id].name);
		return -EINVAL;
	}

	if (sbb_mux_init_sysfs_file(
		    &tracker->value_file, tracker->sysfs_folder, signal_id,
		    "value",
		    signals[signal_id].type == USERLAND_DRIVEN ? 0666 : 0444,
		    sbb_signal_value_show,
		    signals[signal_id].type == USERLAND_DRIVEN ?
			    sbb_signal_value_store :
			    NULL)) {
		pr_err("Could not create 'value' sysfs file for signal %s!",
		       signals[signal_id].name);
		return -EINVAL;
	}

	if (sbb_mux_init_sysfs_file(&tracker->toggle_count_file,
				    tracker->sysfs_folder, signal_id,
				    "toggle_count", 0444,
				    sbb_signal_toggle_count_show, NULL)) {
		pr_err("Could not create 'toggle_count' sysfs file for "
		       "signal %s!",
		       signals[signal_id].name);
		return -EINVAL;
	}

	if (sbb_mux_init_sysfs_file(&tracker->assigned_gpios_file,
				    tracker->sysfs_folder, signal_id,
				    "assigned_gpios", 0444,
				    sbb_signal_assigned_gpios_show, NULL)) {
		pr_err("Could not create 'assigned_gpios' sysfs file for "
		       "signal %s!",
		       signals[signal_id].name);
		return -EINVAL;
	}

	return 0;
}

static void sbb_mux_cleanup_tracker(struct sbb_signal_tracker *tracker)
{
	if (!tracker->sysfs_folder)
		return;

	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder, &tracker->id_file);
	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder, &tracker->type_file);
	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder, &tracker->value_file);
	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder,
				   &tracker->toggle_count_file);
	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder,
				   &tracker->assigned_gpios_file);

	kobject_put(tracker->sysfs_folder);
	tracker->sysfs_folder = NULL;
}

static int sbb_mux_initialize_sysfs_nodes(void)
{
	int i;

	if (primary_sysfs_folder) {
		pr_err("sbb-mux sysfs nodes already initialized!");
		return -EINVAL;
	}

	primary_sysfs_folder = kobject_create_and_add("sbb-mux", kernel_kobj);
	if (!primary_sysfs_folder) {
		pr_err("sbb-mux: Failed to create primary sysfs folder!");
		return -EINVAL;
	}

	signals_sysfs_folder =
		kobject_create_and_add("signals", primary_sysfs_folder);
	if (!signals_sysfs_folder) {
		pr_err("sbb-mux: Failed to create signals sysfs folder!");
		return -EINVAL;
	}

	for (i = 0; i < SBB_SIG_NUM_SIGNALS; i++) {
		if (sbb_mux_initialize_tracker(&signal_trackers[i], i))
			return -EINVAL;
	}

	return 0;
}

static void sbb_mux_clean_up_sysfs_nodes(void)
{
	int i;

	if (!primary_sysfs_folder)
		return;

	kobject_put(primary_sysfs_folder);
	primary_sysfs_folder = NULL;

	if (!signals_sysfs_folder)
		return;

	kobject_put(signals_sysfs_folder);
	signals_sysfs_folder = NULL;

	for (i = 0; i < SBB_SIG_NUM_SIGNALS; i++)
		sbb_mux_cleanup_tracker(&signal_trackers[i]);
}

static ssize_t sbb_gpio_tracked_signal_show(struct kobject *kobj,
					    struct kobj_attribute *attr,
					    char *buf)
{
	int gpio_id = ((struct ext_kobj_attribute *)attr)->id;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 signals[gpio_trackers[gpio_id].tracked_signal].name);
}

static ssize_t sbb_gpio_tracked_signal_store(struct kobject *kobj,
					     struct kobj_attribute *attr,
					     const char *buf, size_t buf_len)
{
	int gpio_id = ((struct ext_kobj_attribute *)attr)->id;
	struct sbb_gpio_tracker *gpio_tracker = &gpio_trackers[gpio_id];
	int current_signal_id;
	int target_signal_id;

	if (buf_len <= 0) {
		pr_err("sbb-mux: Malformed input for GPIO signal update!\n");
		return -EINVAL;
	}

	target_signal_id = sbb_signal_find(buf);
	if (target_signal_id < 0) {
		pr_err("sbb-mux: Signal not found!\n");
		return -EINVAL;
	}

	if (mutex_lock_interruptible(&gpio_tracker->lock)) {
		return -EINTR;
	}

	current_signal_id = gpio_tracker->tracked_signal;

	if (current_signal_id == target_signal_id) {
		mutex_unlock(&gpio_tracker->lock);
		pr_err("sbb-mux: GPIO %s already tracking signal %s!\n",
		       gpio_tracker->name, signals[target_signal_id].name);
		return -EINVAL;
	}

	spin_lock_irq(&signal_trackers[current_signal_id].lock);
	signal_trackers[current_signal_id].assigned_gpios_mask &=
		~(1 << gpio_id);
	spin_unlock_irq(&signal_trackers[current_signal_id].lock);

	spin_lock_irq(&signal_trackers[target_signal_id].lock);
	signal_trackers[target_signal_id].assigned_gpios_mask |= 1 << gpio_id;
	gpiod_set_value(gpio_tracker->gd,
			signal_trackers[target_signal_id].value);
	spin_unlock_irq(&signal_trackers[target_signal_id].lock);

	gpio_tracker->tracked_signal = target_signal_id;
	mutex_unlock(&gpio_tracker->lock);

	return buf_len;
}

static ssize_t sbb_gpio_value_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	int gpio_id = ((struct ext_kobj_attribute *)attr)->id;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 gpiod_get_value_cansleep(gpio_trackers[gpio_id].gd));
}

static int sbb_mux_initialize_gpio_tracker(struct sbb_gpio_tracker *tracker,
					   int gpio_id)
{
	int gpio_system_id;
	const char *default_signal_name;
	int default_signal_id;

	tracker->id = gpio_id;

	tracker->system_id = -1;

	mutex_init(&tracker->lock);

	if (of_property_read_string_index(platform_dev->dev.of_node,
					  "gpio_names", gpio_id,
					  &tracker->name)) {
		pr_err("sbb-mux: could not find name for GPIO #%d!\n", gpio_id);
		return -EINVAL;
	}

	gpio_system_id = of_get_gpio(platform_dev->dev.of_node, gpio_id);
	if (gpio_system_id < 0) {
		pr_err("sbb-mux: of_get_gpio failed for %d!\n", gpio_id);
		return gpio_system_id;
	}

	pr_info("sbb-mux: GPIO system id: %d.\n", gpio_system_id);

	if (devm_gpio_request_one(&platform_dev->dev, gpio_system_id,
				  GPIOF_OUT_INIT_LOW, tracker->name)) {
		pr_err("sbb-mux: GPIO request failed for %d!\n", gpio_id);
		return -EPROBE_DEFER;
	}

	tracker->gd = gpio_to_desc(gpio_system_id);
	if (!tracker->gd) {
		pr_err("sbb-mux: GPIO descriptor for %d not available!\n",
		       gpio_system_id);
		return -EINVAL;
	}

	tracker->system_id = gpio_system_id;

	tracker->sysfs_folder =
		kobject_create_and_add(tracker->name, gpios_sysfs_folder);
	if (!tracker->sysfs_folder) {
		pr_err("sbb-mux: Failed to create sysfs folder for GPIO %s!",
		       tracker->name);
		return -EINVAL;
	}

	if (sbb_mux_init_sysfs_file(&tracker->value_file, tracker->sysfs_folder,
				    gpio_id, "value", 0444, sbb_gpio_value_show,
				    NULL)) {
		pr_err("Could not create 'value' sysfs file for GPIO %s!",
		       tracker->name);
		return -EINVAL;
	}

	if (of_property_read_string_index(platform_dev->dev.of_node,
					  "default_signals", gpio_id,
					  &default_signal_name)) {
		pr_err("sbb-mux: could not find default signal name for "
		       "GPIO #%d! Assuming signal 0.\n",
		       gpio_id);
		default_signal_name = signals[SBB_SIG_ZERO].name;
		default_signal_id = SBB_SIG_ZERO;
	} else {
		default_signal_id = sbb_signal_find(default_signal_name);
		if (default_signal_id < 0) {
			pr_err("sbb-mux: could not find signal id for "
			       "signal %s for GPIO #%d! Will use signal 0.\n",
			       default_signal_name, gpio_id);
			default_signal_id = SBB_SIG_ZERO;
		}
	}

	tracker->tracked_signal = default_signal_id;

	signal_trackers[default_signal_id].assigned_gpios_mask |= 1 << gpio_id;

	if (sbb_mux_init_sysfs_file(&tracker->tracked_signal_file,
				    tracker->sysfs_folder, gpio_id,
				    "tracked_signal", 0666,
				    sbb_gpio_tracked_signal_show,
				    sbb_gpio_tracked_signal_store)) {
		pr_err("Could not create 'tracked_signal' sysfs file "
		       "for GPIO %s!",
		       tracker->name);
		return -EINVAL;
	}

	pr_info("Correctly initialized GPIO tracker #%d for GPIO %s!\n",
		gpio_system_id, tracker->name);

	return 0;
}

static void sbb_mux_cleanup_gpio_tracker(struct sbb_gpio_tracker *tracker)
{
	if (tracker->system_id != -1) {
		tracker->system_id = -1;
	}

	if (!tracker->sysfs_folder)
		return;

	kobject_put(tracker->sysfs_folder);
	tracker->sysfs_folder = NULL;

	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder,
				   &tracker->tracked_signal_file);
	sbb_mux_cleanup_sysfs_file(tracker->sysfs_folder, &tracker->value_file);
}

static void sbb_gpio_refresh_all(void)
{
	int i;

	for (i = 0; i < sbb_num_gpios; i++) {
		if (gpio_trackers[i].gd)
			sbb_signal_set_value(i, SBB_REFRESH_VALUE);
	}
}

static int sbb_mux_drv_probe(struct platform_device *dev)
{
	int num_gpios;
	int num_gpio_names;
	int i;
	struct sbb_gpio_tracker *new_gpio_trackers;

	pr_info("sbb-mux: Calling %s!\n", __func__);

	num_gpios = of_gpio_count(dev->dev.of_node);
	pr_info("sbb-mux: Num GPIOs: %d.\n", num_gpios);

	if (num_gpios <= 0) {
		pr_info("sbb-mux: Found %d GPIOs, will not track GPIOs.\n",
			num_gpios);
		return 0;
	}

	num_gpio_names =
		of_property_count_strings(dev->dev.of_node, "gpio_names");
	if (num_gpio_names != num_gpios) {
		pr_err("sbb-mux: Num GPIO names %d does not match the number "
		       "of declared GPIOs %d!\n",
		       num_gpio_names, num_gpios);
		return -EINVAL;
	}

	if (gpio_trackers) {
		pr_info("sbb-mux: gpio_trackers already initialized!\n");
		return -EINVAL;
	}

	gpios_sysfs_folder =
		kobject_create_and_add("gpios", primary_sysfs_folder);
	if (!gpios_sysfs_folder) {
		pr_err("sbb-mux: Failed to create gpios sysfs folder!");
		return -EINVAL;
	}

	new_gpio_trackers =
		kcalloc(num_gpios, sizeof(new_gpio_trackers[0]), GFP_KERNEL);
	if (!new_gpio_trackers) {
		pr_info("sbb-mux: allocation failed for new_gpio_trackers!\n");
		sbb_mux_drv_undo_probe(&new_gpio_trackers);
		return -ENOMEM;
	}

	platform_dev = dev;

	for (i = 0; i < num_gpios; i++) {
		int tracker_initialization = sbb_mux_initialize_gpio_tracker(
			&new_gpio_trackers[i], i);
		if (tracker_initialization) {
			sbb_mux_drv_undo_probe(&new_gpio_trackers);
			if (tracker_initialization == -EPROBE_DEFER) {
				pr_err("sbb-mux: cannot claim GPIO %d yet, "
				       "deferring probe.\n",
				       i);
				return -EPROBE_DEFER;
			}
			pr_err("sbb-mux: failed to initialize GPIO tracker "
			       "for GPIO %d, cannot recover.\n",
			       i);
			return -EINVAL;
		}
	}

	gpio_trackers = new_gpio_trackers;
	sbb_num_gpios = num_gpios;

	sbb_gpio_refresh_all();

	return 0;
}

static void sbb_mux_drv_undo_probe(struct sbb_gpio_tracker **gpio_trackers_ptr)
{
	int i;

	if (!gpios_sysfs_folder)
		return;

	kobject_put(gpios_sysfs_folder);
	gpios_sysfs_folder = NULL;

	if (!gpio_trackers_ptr || !*gpio_trackers_ptr)
		return;

	for (i = 0; i < sbb_num_gpios; i++)
		sbb_mux_cleanup_gpio_tracker(&(*gpio_trackers_ptr)[i]);

	kfree(*gpio_trackers_ptr);
	*gpio_trackers_ptr = NULL;
}

static int sbb_mux_drv_remove(struct platform_device *dev)
{
	pr_info("sbb-mux: Calling %s!\n", __func__);

	sbb_mux_drv_undo_probe(&gpio_trackers);

	return 0;
}

static int __init sbb_mux_init(void)
{
	signal_trackers[SBB_SIG_ONE].value = 1;

	if (sbb_mux_initialize_sysfs_nodes()) {
		pr_err("sbb-mux: Issue when initializing signal trackers, "
		       "could not load!\n");
		sbb_mux_clean_up_sysfs_nodes();
		return -EINVAL;
	}

	if (platform_driver_register(&sbb_mux_platform_driver))
		pr_err("sbb-mux: Error when registering driver!\n");

	pr_info("sbb-mux: Registered! :D\n");

	return 0;
}

static void __exit sbb_mux_exit(void)
{
	sbb_mux_clean_up_sysfs_nodes();
	pr_info("sbb-mux: Unregistered! :(\n");
}

module_init(sbb_mux_init);
module_exit(sbb_mux_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vincent Palomares");
MODULE_DESCRIPTION("SideBand Bit Multiplexer Driver");
MODULE_DEVICE_TABLE(of, sbb_mux_of_match);
