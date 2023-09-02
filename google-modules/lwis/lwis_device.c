/*
 * Google LWIS Base Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-dev: " fmt

#include "lwis_device.h"

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hashtable.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lwis_buffer.h"
#include "lwis_clock.h"
#include "lwis_commands.h"
#include "lwis_debug.h"
#include "lwis_device.h"
#include "lwis_device_dpm.h"
#include "lwis_device_slc.h"
#include "lwis_dt.h"
#include "lwis_event.h"
#include "lwis_gpio.h"
#include "lwis_init.h"
#include "lwis_ioctl.h"
#include "lwis_periodic_io.h"
#include "lwis_pinctrl.h"
#include "lwis_platform.h"
#include "lwis_transaction.h"
#include "lwis_version.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_CLASS_NAME "lwis"
#define LWIS_DEVICE_NAME "lwis"
#define LWIS_MAX_DEVICES (1U << MINORBITS)
#define MCLK_ON_STRING "mclk_on"
#define MCLK_OFF_STRING "mclk_off"

/* Define this to help debug power sequence */
#undef LWIS_PWR_SEQ_DEBUG

/* Global declaration for core lwis structure */
static struct lwis_core core;

static int lwis_open(struct inode *node, struct file *fp);
static int lwis_release(struct inode *node, struct file *fp);
static long lwis_ioctl(struct file *fp, unsigned int type, unsigned long param);
static unsigned int lwis_poll(struct file *fp, poll_table *wait);
static ssize_t lwis_read(struct file *fp, char __user *user_buf, size_t count, loff_t *pos);

static struct file_operations lwis_fops = {
	.owner = THIS_MODULE,
	.open = lwis_open,
	.release = lwis_release,
	.unlocked_ioctl = lwis_ioctl,
	.poll = lwis_poll,
	.read = lwis_read,
};

/*
 *  lwis_open: Opening an instance of a LWIS device
 */
static int lwis_open(struct inode *node, struct file *fp)
{
	struct lwis_device *lwis_dev;
	struct lwis_client *lwis_client;
	unsigned long flags;

	/* Making sure the minor number associated with fp exists */
	mutex_lock(&core.lock);
	lwis_dev = idr_find(core.idr, iminor(node));
	mutex_unlock(&core.lock);
	if (!lwis_dev) {
		pr_err("No device %d found\n", iminor(node));
		return -ENODEV;
	}
	dev_info(lwis_dev->dev, "Opening instance %d\n", iminor(node));

	lwis_client = kzalloc(sizeof(struct lwis_client), GFP_KERNEL);
	if (!lwis_client) {
		dev_err(lwis_dev->dev, "Failed to allocate lwis client\n");
		return -ENOMEM;
	}

	lwis_client->lwis_dev = lwis_dev;
	/* Initialize locks */
	mutex_init(&lwis_client->lock);
	spin_lock_init(&lwis_client->periodic_io_lock);
	spin_lock_init(&lwis_client->event_lock);

	/* Empty hash table for client event states */
	hash_init(lwis_client->event_states);

	/* The event queue itself is a linked list */
	INIT_LIST_HEAD(&lwis_client->event_queue);
	INIT_LIST_HEAD(&lwis_client->error_event_queue);

	/* Initialize the wait queue for the event queue */
	init_waitqueue_head(&lwis_client->event_wait_queue);

	/* Empty hash table for client allocated buffers */
	hash_init(lwis_client->allocated_buffers);

	/* Empty hash table for client enrolled buffers */
	hash_init(lwis_client->enrolled_buffers);

	/* Initialize the allocator */
	lwis_allocator_init(lwis_dev);

	/* Start transaction processor task */
	lwis_transaction_init(lwis_client);

	/* Start periodic io processor task */
	lwis_periodic_io_init(lwis_client);

	memset(&lwis_client->debug_info, 0, sizeof(lwis_client->debug_info));

	spin_lock_irqsave(&lwis_dev->lock, flags);
	list_add(&lwis_client->node, &lwis_dev->clients);
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	/* Storing the client handle in fp private_data for easy access */
	fp->private_data = lwis_client;

	lwis_client->is_enabled = false;
	return 0;
}

/* Cleans client contents to a reset state. This will hold the LWIS client mutex. */
static int lwis_cleanup_client(struct lwis_client *lwis_client)
{
	/* Let's lock the mutex while we're cleaning up to avoid other parts
	 * of the code from acquiring dangling pointers to the client or any
	 * of the client event states that are about to be freed
	 */
	mutex_lock(&lwis_client->lock);

	/* Clear event states for this client */
	lwis_client_event_states_clear(lwis_client);

	/* Clear the event queues */
	lwis_client_event_queue_clear(lwis_client);
	lwis_client_error_event_queue_clear(lwis_client);

	/* Clean up all periodic io state for the client */
	lwis_periodic_io_client_cleanup(lwis_client);

	/* Cancel all pending transactions for the client */
	lwis_transaction_clear(lwis_client);

	/* Run cleanup transactions. */
	lwis_transaction_client_cleanup(lwis_client);

	/* Disenroll and clear the table of allocated and enrolled buffers */
	lwis_client_allocated_buffers_clear(lwis_client);
	lwis_client_enrolled_buffers_clear(lwis_client);

	mutex_unlock(&lwis_client->lock);

	return 0;
}

/* Calling this requires holding lwis_dev->lock. */
static inline bool check_client_exists(const struct lwis_device *lwis_dev,
				       const struct lwis_client *lwis_client)
{
	struct lwis_client *p, *n;
	list_for_each_entry_safe (p, n, &lwis_dev->clients, node) {
		if (lwis_client == p) {
			return true;
		}
	}
	return false;
}

/* Release client and deletes its entry from the device's client list,
 * this assumes that LWIS device still exists and will hold LWIS device
 * and LWIS client locks. */
static int lwis_release_client(struct lwis_client *lwis_client)
{
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	int rc = 0;
	unsigned long flags;
	rc = lwis_cleanup_client(lwis_client);
	if (rc) {
		return rc;
	}

	/* Take this lwis_client off the list of active clients */
	spin_lock_irqsave(&lwis_dev->lock, flags);
	if (check_client_exists(lwis_dev, lwis_client)) {
		list_del(&lwis_client->node);
	} else {
		dev_err(lwis_dev->dev, "Trying to release a client tied to this device, "
				       "but the entry was not found on the clients list.");
	}
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	kfree(lwis_client);
	return 0;
}

/*
 *  lwis_release: Closing an instance of a LWIS device
 */
static int lwis_release(struct inode *node, struct file *fp)
{
	struct lwis_client *lwis_client = fp->private_data;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	int rc = 0;
	bool is_client_enabled = lwis_client->is_enabled;

	dev_info(lwis_dev->dev, "Closing instance %d\n", iminor(node));

	rc = lwis_release_client(lwis_client);

	/* Release the allocator and its cache */
	lwis_allocator_release(lwis_dev);

	mutex_lock(&lwis_dev->client_lock);
	/* Release power if client closed without power down called */
	if (is_client_enabled && lwis_dev->enabled > 0) {
		lwis_dev->enabled--;
		if (lwis_dev->enabled == 0) {
			dev_info(lwis_dev->dev, "No more client, power down\n");
			rc = lwis_dev_power_down_locked(lwis_dev);
		}
	}

	if (lwis_dev->enabled == 0) {
		if (lwis_dev->bts_index != BTS_UNSUPPORTED) {
			lwis_platform_update_bts(lwis_dev, /*bw_peak=*/0,
						 /*bw_read=*/0, /*bw_write=*/0, /*bw_rt=*/0);
		}
		/* remove voted qos */
		lwis_platform_remove_qos(lwis_dev);
		/* Release device event states if no more client is using */
		lwis_device_event_states_clear_locked(lwis_dev);
	}
	mutex_unlock(&lwis_dev->client_lock);

	/* Call device type specific close routines. */
	if (lwis_dev->enabled == 0) {
		if (lwis_dev->vops.close != NULL) {
			lwis_dev->vops.close(lwis_dev);
		}
	}

	return rc;
}

/*
 *  lwis_ioctl: I/O control function on a LWIS device
 *
 *  List of IOCTL types are defined in lwis_commands.h
 */
static long lwis_ioctl(struct file *fp, unsigned int type, unsigned long param)
{
	struct lwis_client *lwis_client;
	struct lwis_device *lwis_dev;

	lwis_client = fp->private_data;
	if (!lwis_client) {
		pr_err("Cannot find client instance\n");
		return -ENODEV;
	}

	lwis_dev = lwis_client->lwis_dev;
	if (!lwis_dev) {
		pr_err("Cannot find device instance\n");
		return -ENODEV;
	}

	return lwis_ioctl_handler(lwis_client, type, param);
}
/*
 *  lwis_poll: Event queue status function of LWIS
 *
 */
static unsigned int lwis_poll(struct file *fp, poll_table *wait)
{
	unsigned int mask = 0;
	struct lwis_client *lwis_client;

	lwis_client = fp->private_data;
	if (!lwis_client) {
		pr_err("Cannot find client instance\n");
		return POLLERR;
	}

	/* Add our wait queue to the poll table */
	poll_wait(fp, &lwis_client->event_wait_queue, wait);

	/* Check if we have anything in the event lists */
	if (lwis_client_error_event_peek_front(lwis_client, NULL) == 0) {
		mask |= POLLERR;
	} else if (lwis_client_event_peek_front(lwis_client, NULL) == 0) {
		mask |= POLLIN;
	}

	return mask;
}

static ssize_t lwis_read(struct file *fp, char __user *user_buf, size_t count, loff_t *pos)
{
	int ret = 0;
	/* Buffer to store information */
	const size_t buffer_size = 8192;
	char *buffer = kzalloc(buffer_size, GFP_KERNEL);
	if (!buffer) {
		pr_err("Failed to allocate read buffer\n");
		return -ENOMEM;
	}

	lwis_get_feature_flags(buffer, buffer_size);

	ret = simple_read_from_buffer(user_buf, count, pos, buffer, strlen(buffer));

	kfree(buffer);

	return ret;
}

static int lwis_base_setup(struct lwis_device *lwis_dev)
{
	int ret = 0;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_base_parse_dt(lwis_dev);
	if (ret) {
		pr_err("Failed to parse device tree\n");
	}
#else
	/* Non-device-tree init: Save for future implementation */
	ret = -ENOSYS;
#endif

	return ret;
}

/*
 *  lwis_assign_top_to_other: Assign top device to the devices probed before.
 */
static void lwis_assign_top_to_other(struct lwis_device *top_dev)
{
	struct lwis_device *lwis_dev;

	mutex_lock(&core.lock);
	list_for_each_entry (lwis_dev, &core.lwis_dev_list, dev_list) {
		lwis_dev->top_dev = top_dev;
	}
	mutex_unlock(&core.lock);
}

static int process_power_sequence(struct lwis_device *lwis_dev,
				  struct lwis_device_power_sequence_list *list, bool set_active,
				  bool skip_error)
{
	int ret = 0;
	int last_error = 0;
	int i;

	if (lwis_dev == NULL) {
		pr_err("lwis_dev is NULL\n");
		return -ENODEV;
	}

	if (list == NULL || list->count == 0) {
		dev_err(lwis_dev->dev, "No power_up_sequence defined\n");
		return -EINVAL;
	}

	for (i = 0; i < list->count; ++i) {
#ifdef LWIS_PWR_SEQ_DEBUG
		dev_info(lwis_dev->dev, "%s: %d - type:%s name:%s delay_us:%d", __func__, i,
			 list->seq_info[i].type, list->seq_info[i].name,
			 list->seq_info[i].delay_us);
#endif
		if (strcmp(list->seq_info[i].type, "regulator") == 0) {
			if (lwis_dev->regulators == NULL) {
				dev_err(lwis_dev->dev, "No regulators defined\n");
				ret = -EINVAL;
				if (!skip_error) {
					return ret;
				} else {
					last_error = ret;
					continue;
				}
			}
			if (set_active) {
				ret = lwis_regulator_enable_by_name(lwis_dev->regulators,
								    list->seq_info[i].name);
			} else {
				ret = lwis_regulator_disable_by_name(lwis_dev->regulators,
								     list->seq_info[i].name);
			}
			if (ret) {
				dev_err(lwis_dev->dev, "Error set regulators (%d)\n", ret);
				if (!skip_error) {
					return ret;
				}
				last_error = ret;
			}
		} else if (strcmp(list->seq_info[i].type, "gpio") == 0) {
			struct lwis_gpios_info *gpios_info = NULL;
			int set_value = 0;
			bool set_state = true;

			gpios_info = lwis_gpios_get_info_by_name(lwis_dev->gpios_list,
								 list->seq_info[i].name);
			if (IS_ERR(gpios_info)) {
				dev_err(lwis_dev->dev, "Get %s gpios info failed\n",
					list->seq_info[i].name);
				ret = PTR_ERR(gpios_info);
				if (!skip_error) {
					return ret;
				} else {
					last_error = ret;
					continue;
				}
			}

			if (set_active) {
				struct gpio_descs *gpios = NULL;
				gpios = lwis_gpio_list_get(&lwis_dev->plat_dev->dev,
							   list->seq_info[i].name);
				if (IS_ERR_OR_NULL(gpios)) {
					gpios_info->gpios = NULL;
					ret = PTR_ERR(gpios);
					if (ret == -EBUSY && gpios_info->is_shared) {
						dev_warn(
							lwis_dev->dev,
							"Shared gpios requested by another device\n");
						ret = 0;
						continue;
					} else {
						dev_err(lwis_dev->dev,
							"Failed to obtain gpio list (%d)\n", ret);
						if (!skip_error) {
							return ret;
						} else {
							last_error = ret;
							continue;
						}
					}
				}
				gpios_info->gpios = gpios;
				set_value = 1;
			} else {
				if (gpios_info->gpios == NULL) {
					if (gpios_info->is_shared) {
						continue;
					}
					dev_err(lwis_dev->dev, "No %s gpios defined\n",
						list->seq_info[i].name);
					ret = -ENODEV;
					if (!skip_error) {
						return ret;
					} else {
						last_error = ret;
						continue;
					}
				}
				set_value = 0;
			}

			if (gpios_info->is_shared && !set_active) {
				struct lwis_device *lwis_dev_it;
				struct lwis_gpios_info *gpios_info_it;

				/* Look up if gpio it's already acquired */
				mutex_lock(&core.lock);
				list_for_each_entry (lwis_dev_it, &core.lwis_dev_list, dev_list) {
					if ((lwis_dev->id != lwis_dev_it->id) &&
					    lwis_dev_it->enabled && lwis_dev_it->gpios_list) {
						gpios_info_it = lwis_gpios_get_info_by_name(
							lwis_dev_it->gpios_list,
							list->seq_info[i].name);
						if (IS_ERR(gpios_info_it)) {
							continue;
						}
						if (gpios_info_it->id == gpios_info->id &&
						    gpios_info_it->gpios == NULL) {
							dev_info(lwis_dev->dev,
								 "Handover shared GPIO to %s\n",
								 lwis_dev_it->name);
							gpios_info_it->gpios = gpios_info->gpios;
							gpios_info_it->hold_dev =
								gpios_info->hold_dev;
							set_state = false;
							gpios_info->gpios = NULL;
						}
						break;
					}
				}
				mutex_unlock(&core.lock);
			}
			if (!set_state) {
				continue;
			}

			if (gpios_info->is_pulse) {
				ret = lwis_gpio_list_set_output_value(gpios_info->gpios,
								      1 - set_value);
				if (ret) {
					dev_err(lwis_dev->dev, "Error set GPIO pins (%d)\n", ret);
					if (!skip_error) {
						return ret;
					}
					last_error = ret;
				}
				usleep_range(1000, 1500);
			}
			ret = lwis_gpio_list_set_output_value(gpios_info->gpios, set_value);
			if (ret) {
				dev_err(lwis_dev->dev, "Error set GPIO pins (%d)\n", ret);
				if (!skip_error) {
					return ret;
				}
				last_error = ret;
			}

			if (!set_active) {
				/* Release "ownership" of the GPIO pins */
				lwis_gpio_list_put(gpios_info->gpios, gpios_info->hold_dev);
				gpios_info->gpios = NULL;
			}
		} else if (strcmp(list->seq_info[i].type, "pinctrl") == 0) {
			bool set_state = true;

			if (set_active) {
				lwis_dev->mclk_ctrl = devm_pinctrl_get(&lwis_dev->plat_dev->dev);
				if (IS_ERR(lwis_dev->mclk_ctrl)) {
					dev_err(lwis_dev->dev, "Failed to get mclk\n");
					ret = PTR_ERR(lwis_dev->mclk_ctrl);
					lwis_dev->mclk_ctrl = NULL;
					if (!skip_error) {
						return ret;
					} else {
						last_error = ret;
						continue;
					}
				}
			} else {
				if (lwis_dev->mclk_ctrl == NULL) {
					dev_err(lwis_dev->dev, "No pinctrl defined\n");
					ret = -ENODEV;
					if (!skip_error) {
						return ret;
					} else {
						last_error = ret;
						continue;
					}
				}
			}

			if (lwis_dev->shared_pinctrl) {
				struct lwis_device *lwis_dev_it;
				/* Look up if pinctrl it's already acquired */
				mutex_lock(&core.lock);
				list_for_each_entry (lwis_dev_it, &core.lwis_dev_list, dev_list) {
					if ((lwis_dev->id != lwis_dev_it->id) &&
					    (lwis_dev_it->shared_pinctrl ==
					     lwis_dev->shared_pinctrl) &&
					    lwis_dev_it->enabled) {
						dev_info(lwis_dev->dev, "mclk already acquired\n");
						if (set_active) {
							devm_pinctrl_put(lwis_dev->mclk_ctrl);
						} else {
							/*
							 * Move mclk owner to the device who
							 * still using it
							 */
							lwis_dev_it->mclk_ctrl =
								lwis_dev->mclk_ctrl;
						}
						set_state = false;
						lwis_dev->mclk_ctrl = NULL;
						break;
					}
				}
				mutex_unlock(&core.lock);
			}

			if (set_state) {
				/* Set MCLK state */
				ret = lwis_pinctrl_set_state(lwis_dev->mclk_ctrl,
							     list->seq_info[i].name);
				if (ret) {
					dev_err(lwis_dev->dev, "Error setting %s state (%d)\n",
						list->seq_info[i].name, ret);
					if (set_active) {
						devm_pinctrl_put(lwis_dev->mclk_ctrl);
						lwis_dev->mclk_ctrl = NULL;
					}
					if (!skip_error) {
						return ret;
					} else {
						last_error = ret;
						continue;
					}
				}
				if (!set_active) {
					devm_pinctrl_put(lwis_dev->mclk_ctrl);
					lwis_dev->mclk_ctrl = NULL;
				}
			}
		}
		usleep_range(list->seq_info[i].delay_us, list->seq_info[i].delay_us);
	}

	if (last_error) {
		return last_error;
	}

	return ret;
}

static int lwis_dev_power_up_by_default(struct lwis_device *lwis_dev)
{
	int ret;

	if (lwis_dev == NULL) {
		pr_err("lwis_dev is NULL\n");
		return -ENODEV;
	}

	if (lwis_dev->regulators) {
		/* Enable all the regulators related to this sensor */
		ret = lwis_regulator_enable_all(lwis_dev->regulators);
		if (ret) {
			dev_err(lwis_dev->dev, "Error enabling regulators (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->enable_gpios_present) {
		struct gpio_descs *gpios;

		gpios = lwis_gpio_list_get(&lwis_dev->plat_dev->dev, "enable");
		if (IS_ERR_OR_NULL(gpios)) {
			dev_err(lwis_dev->dev, "Failed to obtain enable gpio list (%ld)\n",
				PTR_ERR(gpios));
			return PTR_ERR(gpios);
		}

		/*
		 * Setting enable_gpios to non-NULL to indicate that this lwis
		 * device is holding onto the GPIO pins.
		 */
		lwis_dev->enable_gpios = gpios;

		/* Set enable pins to 1 (i.e. asserted) */
		ret = lwis_gpio_list_set_output_value(gpios, 1);
		if (ret) {
			dev_err(lwis_dev->dev, "Error enabling GPIO pins (%d)\n", ret);
			return ret;
		}

		if (lwis_dev->enable_gpios_settle_time > 0) {
			usleep_range(lwis_dev->enable_gpios_settle_time,
				     lwis_dev->enable_gpios_settle_time);
		}
	}

	if (lwis_dev->shared_enable_gpios_present) {
		struct gpio_descs *gpios;

		gpios = lwis_gpio_list_get(&lwis_dev->plat_dev->dev, "shared-enable");
		if (IS_ERR_OR_NULL(gpios)) {
			if (PTR_ERR(gpios) == -EBUSY) {
				dev_warn(lwis_dev->dev,
					 "Shared gpios requested by another device\n");
			} else {
				dev_err(lwis_dev->dev, "Failed to obtain shared gpio list (%ld)\n",
					PTR_ERR(gpios));
				return PTR_ERR(gpios);
			}
		} else {
			lwis_dev->shared_enable_gpios = gpios;
			/* Set enable pins to 1 (i.e. asserted) */
			ret = lwis_gpio_list_set_output_value(gpios, 1);
			if (ret) {
				dev_err(lwis_dev->dev, "Error enabling GPIO pins (%d)\n", ret);
				return ret;
			}
		}
	}

	if (lwis_dev->mclk_present) {
		bool activate_mclk = true;

		lwis_dev->mclk_ctrl = devm_pinctrl_get(&lwis_dev->plat_dev->dev);
		if (IS_ERR(lwis_dev->mclk_ctrl)) {
			dev_err(lwis_dev->dev, "Failed to get mclk\n");
			ret = PTR_ERR(lwis_dev->mclk_ctrl);
			lwis_dev->mclk_ctrl = NULL;
			return ret;
		}

		if (lwis_dev->shared_pinctrl > 0) {
			struct lwis_device *lwis_dev_it;
			/* Look up if pinctrl it's already enabled */
			mutex_lock(&core.lock);
			list_for_each_entry (lwis_dev_it, &core.lwis_dev_list, dev_list) {
				if ((lwis_dev->id != lwis_dev_it->id) &&
				    (lwis_dev_it->shared_pinctrl == lwis_dev->shared_pinctrl) &&
				    lwis_dev_it->enabled) {
					activate_mclk = false;
					devm_pinctrl_put(lwis_dev->mclk_ctrl);
					lwis_dev->mclk_ctrl = NULL;
					dev_info(lwis_dev->dev, "mclk already acquired\n");
					break;
				}
			}
			mutex_unlock(&core.lock);
		}

		if (activate_mclk) {
			/* Set MCLK state to on */
			ret = lwis_pinctrl_set_state(lwis_dev->mclk_ctrl, MCLK_ON_STRING);
			if (ret) {
				dev_err(lwis_dev->dev, "Error setting %s state (%d)\n",
					MCLK_ON_STRING, ret);
				devm_pinctrl_put(lwis_dev->mclk_ctrl);
				lwis_dev->mclk_ctrl = NULL;
				return ret;
			}
		}
	}

	if (lwis_dev->reset_gpios_present) {
		struct gpio_descs *gpios;

		gpios = lwis_gpio_list_get(&lwis_dev->plat_dev->dev, "reset");
		if (IS_ERR_OR_NULL(gpios)) {
			dev_err(lwis_dev->dev, "Failed to obtain reset gpio list (%ld)\n",
				PTR_ERR(gpios));
			return PTR_ERR(gpios);
		}
		/*
		 * Setting reset_gpios to non-NULL to indicate that this lwis
		 * device is holding onto the GPIO pins.
		 */
		lwis_dev->reset_gpios = gpios;

		/* Set reset pin to 0 (i.e. deasserted) */
		ret = lwis_gpio_list_set_output_value(gpios, 0);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to set reset GPIOs to ACTIVE (%d)\n", ret);
			return ret;
		}

		usleep_range(1000, 1500);

		/* Set reset pin to 1 (i.e. asserted) */
		ret = lwis_gpio_list_set_output_value(gpios, 1);
		if (ret) {
			dev_err(lwis_dev->dev, "Failed to set reset GPIOs to DE-ACTIVE (%d)\n",
				ret);
			return ret;
		}
	}

	return 0;
}

/*
 * Power up a LWIS device, should be called when lwis_dev->enabled is 0
 * lwis_dev->client_lock should be held before this function.
 */
int lwis_dev_power_up_locked(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_i2c_device *i2c_dev = NULL;

	if (lwis_dev->type == DEVICE_TYPE_I2C) {
		i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
		if (i2c_dev->group_i2c_lock == NULL) {
			dev_err(lwis_dev->dev, "group_i2c_lock is NULL. Abort power up.\n");
			return -EINVAL;
		}
	}

	/* Let's do the platform-specific enable call */
	ret = lwis_platform_device_enable(lwis_dev);
	if (ret) {
		dev_err(lwis_dev->dev, "Platform-specific device enable fail: %d\n", ret);
		goto error_power_up;
	}

	if (lwis_dev->clocks) {
		/* Enable clocks */
		ret = lwis_clock_enable_all(lwis_dev->clocks);
		if (ret) {
			dev_err(lwis_dev->dev, "Error enabling clocks (%d)\n", ret);
			goto error_power_up;
		}
	}

	if (lwis_dev->type == DEVICE_TYPE_I2C) {
		mutex_lock(i2c_dev->group_i2c_lock);
	}
	if (lwis_dev->power_up_sequence) {
		ret = process_power_sequence(lwis_dev, lwis_dev->power_up_sequence,
					     /*set_active=*/true, /*skip_error=*/false);
		if (ret) {
			dev_err(lwis_dev->dev, "Error process_power_sequence (%d)\n", ret);
			if (lwis_dev->type == DEVICE_TYPE_I2C) {
				mutex_unlock(i2c_dev->group_i2c_lock);
			}
			goto error_power_up;
		}
	} else {
		ret = lwis_dev_power_up_by_default(lwis_dev);
		if (ret) {
			dev_err(lwis_dev->dev, "Error lwis_dev_power_up_by_default (%d)\n", ret);
			if (lwis_dev->type == DEVICE_TYPE_I2C) {
				mutex_unlock(i2c_dev->group_i2c_lock);
			}
			goto error_power_up;
		}
	}
	if (lwis_dev->type == DEVICE_TYPE_I2C) {
		mutex_unlock(i2c_dev->group_i2c_lock);
	}

	if (lwis_dev->phys) {
		/* Power on the PHY */
		ret = lwis_phy_set_power_all(lwis_dev->phys,
					     /* power_on = */ true);
		if (ret) {
			dev_err(lwis_dev->dev, "Error powering on PHY\n");
			goto error_power_up;
		}
	}

	if (lwis_dev->vops.device_enable) {
		ret = lwis_dev->vops.device_enable(lwis_dev);
		if (ret) {
			dev_err(lwis_dev->dev, "Error executing device enable function\n");
			goto error_power_up;
		}
	}

	/* Sleeping to make sure all pins are ready to go */
	usleep_range(2000, 2000);
	return 0;

	/* Error handling */
error_power_up:
	lwis_dev_power_down_locked(lwis_dev);
	return ret;
}

static int lwis_dev_power_down_by_default(struct lwis_device *lwis_dev)
{
	int last_error = 0;
	int ret;

	if (lwis_dev == NULL) {
		pr_err("lwis_dev is NULL\n");
		return -ENODEV;
	}

	if (lwis_dev->mclk_ctrl) {
		bool deactivate_mclk = true;

		if (lwis_dev->shared_pinctrl) {
			struct lwis_device *lwis_dev_it;
			/* Look up if pinctrl still used by other device */
			mutex_lock(&core.lock);
			list_for_each_entry (lwis_dev_it, &core.lwis_dev_list, dev_list) {
				if ((lwis_dev->id != lwis_dev_it->id) &&
				    (lwis_dev_it->shared_pinctrl == lwis_dev->shared_pinctrl) &&
				    lwis_dev_it->enabled) {
					/*
					 * Move mclk owner to the device who
					 * still using it
					 */
					lwis_dev_it->mclk_ctrl = lwis_dev->mclk_ctrl;
					lwis_dev->mclk_ctrl = NULL;
					deactivate_mclk = false;
					break;
				}
			}
			mutex_unlock(&core.lock);
		}

		if (deactivate_mclk) {
			/* Set MCLK state to off */
			ret = lwis_pinctrl_set_state(lwis_dev->mclk_ctrl, MCLK_OFF_STRING);
			if (ret) {
				dev_err(lwis_dev->dev, "Error setting %s state (%d)\n",
					MCLK_OFF_STRING, ret);
				last_error = ret;
			}
			devm_pinctrl_put(lwis_dev->mclk_ctrl);
			lwis_dev->mclk_ctrl = NULL;
		}
	}

	if (lwis_dev->reset_gpios_present && lwis_dev->reset_gpios) {
		/* Set reset pins to low */
		ret = lwis_gpio_list_set_output_value(lwis_dev->reset_gpios, 0);
		if (ret) {
			dev_err(lwis_dev->dev, "Error setting reset GPIOs to ACTIVE (%d)\n", ret);
			last_error = ret;
		}

		/* Release ownership of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->reset_gpios, &lwis_dev->plat_dev->dev);
		lwis_dev->reset_gpios = NULL;
	}

	if (lwis_dev->shared_enable_gpios_present && lwis_dev->shared_enable_gpios) {
		/* Set enable pins to 0 (i.e. deasserted) */
		ret = lwis_gpio_list_set_output_value(lwis_dev->shared_enable_gpios, 0);
		if (ret) {
			dev_err(lwis_dev->dev, "Error disabling GPIO pins (%d)\n", ret);
			last_error = ret;
		}

		/* Release "ownership" of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->shared_enable_gpios, &lwis_dev->plat_dev->dev);
		lwis_dev->shared_enable_gpios = NULL;
	}

	if (lwis_dev->enable_gpios_present && lwis_dev->enable_gpios) {
		/* Set enable pins to 0 (i.e. deasserted) */
		ret = lwis_gpio_list_set_output_value(lwis_dev->enable_gpios, 0);
		if (ret) {
			dev_err(lwis_dev->dev, "Error disabling GPIO pins (%d)\n", ret);
			last_error = ret;
		}

		/* Release "ownership" of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->enable_gpios, &lwis_dev->plat_dev->dev);
		lwis_dev->enable_gpios = NULL;
	}

	if (lwis_dev->regulators) {
		/* Disable all the regulators */
		ret = lwis_regulator_disable_all(lwis_dev->regulators);
		if (ret) {
			dev_err(lwis_dev->dev, "Error disabling regulators (%d)\n", ret);
			last_error = ret;
		}
	}

	if (last_error) {
		return last_error;
	}

	return 0;
}

/*
 * Power down a LWIS device, should be called when lwis_dev->enabled become 0
 * lwis_dev->client_lock should be held before this function.
 */
int lwis_dev_power_down_locked(struct lwis_device *lwis_dev)
{
	int ret;
	int last_error = 0;
	struct lwis_i2c_device *i2c_dev = NULL;

	if (lwis_dev->type == DEVICE_TYPE_I2C) {
		i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
		if (i2c_dev->group_i2c_lock == NULL) {
			dev_err(lwis_dev->dev, "group_i2c_lock is NULL. Abort power up.\n");
			return -EINVAL;
		}
	}

	if (lwis_dev->vops.device_disable) {
		ret = lwis_dev->vops.device_disable(lwis_dev);
		if (ret) {
			dev_err(lwis_dev->dev, "Error executing device disable function\n");
			last_error = ret;
		}
	}

	if (lwis_dev->phys) {
		/* Power on the PHY */
		ret = lwis_phy_set_power_all(lwis_dev->phys,
					     /* power_on = */ false);
		if (ret) {
			dev_err(lwis_dev->dev, "Error powering off PHY\n");
			last_error = ret;
		}
	}

	if (lwis_dev->type == DEVICE_TYPE_I2C) {
		mutex_lock(i2c_dev->group_i2c_lock);
	}
	if (lwis_dev->power_down_sequence) {
		ret = process_power_sequence(lwis_dev, lwis_dev->power_down_sequence,
					     /*set_active=*/false, /*skip_error=*/true);
		if (ret) {
			dev_err(lwis_dev->dev, "Error process_power_sequence (%d)\n", ret);
			last_error = ret;
		}
	} else {
		ret = lwis_dev_power_down_by_default(lwis_dev);
		if (ret) {
			dev_err(lwis_dev->dev, "Error lwis_dev_power_down_by_default (%d)\n", ret);
			last_error = ret;
		}
	}
	if (lwis_dev->type == DEVICE_TYPE_I2C) {
		mutex_unlock(i2c_dev->group_i2c_lock);
	}

	if (lwis_dev->clocks) {
		/* Disable all clocks */
		lwis_clock_disable_all(lwis_dev->clocks);
	}

	/* Let's do the platform-specific disable call */
	ret = lwis_platform_device_disable(lwis_dev);
	if (ret) {
		dev_err(lwis_dev->dev, "Platform-specific device disable fail: %d\n", ret);
		last_error = ret;
	}

	if (last_error) {
		return last_error;
	}

	return ret;
}

/*
 *  lwis_dev_power_seq_list_alloc:
 *  Allocate an instance of the lwis_device_power_sequence_info
 *  and initialize the data structures according to the number of
 *  lwis_device_power_sequence_info specified.
 */
struct lwis_device_power_sequence_list *lwis_dev_power_seq_list_alloc(int count)
{
	struct lwis_device_power_sequence_list *list;

	/* No need to allocate if count is invalid */
	if (count <= 0) {
		return ERR_PTR(-EINVAL);
	}

	list = kmalloc(sizeof(struct lwis_device_power_sequence_list), GFP_KERNEL);
	if (!list) {
		pr_err("Failed to allocate power sequence list\n");
		return ERR_PTR(-ENOMEM);
	}

	list->seq_info =
		kmalloc(count * sizeof(struct lwis_device_power_sequence_info), GFP_KERNEL);
	if (!list->seq_info) {
		pr_err("Failed to allocate lwis_device_power_sequence_info "
		       "instances\n");
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = count;

	return list;
}

/*
 *  lwis_dev_power_seq_list_free: Deallocate the
 *  lwis_device_power_sequence_info structure.
 */
void lwis_dev_power_seq_list_free(struct lwis_device_power_sequence_list *list)
{
	if (!list) {
		return;
	}

	if (list->seq_info) {
		kfree(list->seq_info);
	}

	kfree(list);
}

/*
 *  lwis_dev_power_seq_list_print:
 *  Print lwis_device_power_sequence_list content
 */
void lwis_dev_power_seq_list_print(struct lwis_device_power_sequence_list *list)
{
	int i;

	for (i = 0; i < list->count; ++i) {
		pr_info("type:%s  name:%s  delay_us:%d\n", list->seq_info[i].type,
			list->seq_info[i].name, list->seq_info[i].delay_us);
	}
}

static struct lwis_device *find_top_dev(void)
{
	struct lwis_device *lwis_dev;

	mutex_lock(&core.lock);
	list_for_each_entry (lwis_dev, &core.lwis_dev_list, dev_list) {
		if (lwis_dev->type == DEVICE_TYPE_TOP) {
			mutex_unlock(&core.lock);
			return lwis_dev;
		}
	}
	mutex_unlock(&core.lock);
	return NULL;
}

static void event_heartbeat_timer(struct timer_list *t)
{
	struct lwis_device *lwis_dev = from_timer(lwis_dev, t, heartbeat_timer);
	int64_t event_id = LWIS_EVENT_ID_HEARTBEAT | (int64_t)lwis_dev->id
							     << LWIS_EVENT_ID_EVENT_CODE_LEN;

	lwis_device_event_emit(lwis_dev, event_id, NULL, 0,
			       /*in_irq=*/false);

	mod_timer(t, jiffies + msecs_to_jiffies(1000));
}

/*
 *  lwis_find_dev_by_id: Find LWIS device by id.
 */
struct lwis_device *lwis_find_dev_by_id(int dev_id)
{
	struct lwis_device *lwis_dev;

	mutex_lock(&core.lock);
	list_for_each_entry (lwis_dev, &core.lwis_dev_list, dev_list) {
		if (lwis_dev->id == dev_id) {
			mutex_unlock(&core.lock);
			return lwis_dev;
		}
	}
	mutex_unlock(&core.lock);
	return NULL;
}

/*
 *  lwis_i2c_dev_is_in_use: Check i2c device is in use.
 */
bool lwis_i2c_dev_is_in_use(struct lwis_device *lwis_dev)
{
	struct lwis_device *lwis_dev_it;
	struct lwis_i2c_device *i2c_dev;

	if (lwis_dev->type != DEVICE_TYPE_I2C) {
		dev_err(lwis_dev->dev, "It's not i2c device\n");
		return false;
	}

	i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
	mutex_lock(&core.lock);
	list_for_each_entry (lwis_dev_it, &core.lwis_dev_list, dev_list) {
		if (lwis_dev_it->type == DEVICE_TYPE_I2C) {
			struct lwis_i2c_device *i2c_dev_it =
				container_of(lwis_dev_it, struct lwis_i2c_device, base_dev);
			/* Look up if i2c bus are still in use by other device*/
			if ((i2c_dev_it->state_pinctrl == i2c_dev->state_pinctrl) &&
			    (i2c_dev_it != i2c_dev) && lwis_dev_it->enabled) {
				mutex_unlock(&core.lock);
				return true;
			}
		}
	}
	mutex_unlock(&core.lock);
	return false;
}

void lwis_device_info_dump(const char *name, void (*func)(struct lwis_device *))
{
	struct lwis_device *lwis_dev_it;

	pr_info("LWIS Device Info Dump: %s\n\n", name);

	mutex_lock(&core.lock);
	list_for_each_entry (lwis_dev_it, &core.lwis_dev_list, dev_list) {
		func(lwis_dev_it);
	}
	mutex_unlock(&core.lock);
}

/*
 *  lwis_base_probe: Create a device instance for each of the LWIS device.
 */
int lwis_base_probe(struct lwis_device *lwis_dev, struct platform_device *plat_dev)
{
	int ret = 0;

	/* Allocate a minor number to this device */
	mutex_lock(&core.lock);
	ret = idr_alloc(core.idr, lwis_dev, 0, LWIS_MAX_DEVICES, GFP_KERNEL);
	mutex_unlock(&core.lock);
	if (ret >= 0) {
		lwis_dev->id = ret;
	} else {
		pr_err("Unable to allocate minor ID (%d)\n", ret);
		return ret;
	}

	/* Initialize enabled state */
	lwis_dev->enabled = 0;
	lwis_dev->clock_family = CLOCK_FAMILY_INVALID;

	/* Initialize client mutex */
	mutex_init(&lwis_dev->client_lock);

	/* Initialize an empty list of clients */
	INIT_LIST_HEAD(&lwis_dev->clients);

	/* Initialize event state hash table */
	hash_init(lwis_dev->event_states);

	/* Initialize the spinlock */
	spin_lock_init(&lwis_dev->lock);

	if (lwis_dev->type == DEVICE_TYPE_TOP) {
		lwis_dev->top_dev = lwis_dev;
		/* Assign top device to the devices probed before */
		lwis_assign_top_to_other(lwis_dev);
	} else {
		lwis_dev->top_dev = find_top_dev();
		if (lwis_dev->top_dev == NULL)
			pr_warn("Top device not probed yet");
	}

	/* Add this instance to the device list */
	mutex_lock(&core.lock);
	list_add(&lwis_dev->dev_list, &core.lwis_dev_list);
	mutex_unlock(&core.lock);

	lwis_dev->plat_dev = plat_dev;
	ret = lwis_base_setup(lwis_dev);
	if (ret) {
		pr_err("Error initializing LWIS device\n");
		goto error_init;
	}

	/* Upon success initialization, create device for this instance */
	lwis_dev->dev = device_create(core.dev_class, NULL, MKDEV(core.device_major, lwis_dev->id),
				      lwis_dev, LWIS_DEVICE_NAME "-%s", lwis_dev->name);
	if (IS_ERR(lwis_dev->dev)) {
		pr_err("Failed to create device\n");
		ret = PTR_ERR(lwis_dev->dev);
		goto error_init;
	}

	platform_set_drvdata(plat_dev, lwis_dev);

	/* Call platform-specific probe function */
	lwis_platform_probe(lwis_dev);

	lwis_device_debugfs_setup(lwis_dev, core.dbg_root);
	memset(&lwis_dev->debug_info, 0, sizeof(lwis_dev->debug_info));

	timer_setup(&lwis_dev->heartbeat_timer, event_heartbeat_timer, 0);

	dev_info(lwis_dev->dev, "Base Probe: Success\n");

	return ret;

	/* Error conditions */
error_init:
	lwis_base_unprobe(lwis_dev);
	mutex_lock(&core.lock);
	idr_remove(core.idr, lwis_dev->id);
	mutex_unlock(&core.lock);
	return ret;
}

/*
 *  lwis_base_unprobe: Cleanup a device instance
 */
void lwis_base_unprobe(struct lwis_device *unprobe_lwis_dev)
{
	struct lwis_device *lwis_dev, *temp;

	mutex_lock(&core.lock);
	list_for_each_entry_safe (lwis_dev, temp, &core.lwis_dev_list, dev_list) {
		if (lwis_dev == unprobe_lwis_dev) {
			pr_info("Destroy device %s id %d", lwis_dev->name, lwis_dev->id);
			lwis_device_debugfs_cleanup(lwis_dev);
			/* Release device clock list */
			if (lwis_dev->clocks) {
				lwis_clock_list_free(lwis_dev->clocks);
				lwis_dev->clocks = NULL;
			}
			/* Release device interrupt list */
			if (lwis_dev->irqs) {
				lwis_interrupt_list_free(lwis_dev->irqs);
				lwis_dev->irqs = NULL;
			}
			/* Release device regulator list */
			if (lwis_dev->regulators) {
				lwis_regulator_put_all(lwis_dev->regulators);
				lwis_regulator_list_free(lwis_dev->regulators);
				lwis_dev->regulators = NULL;
			}
			/* Release device phy list */
			if (lwis_dev->phys) {
				lwis_phy_list_free(lwis_dev->phys);
				lwis_dev->phys = NULL;
			}
			/* Release device power sequence list */
			if (lwis_dev->power_up_sequence) {
				lwis_dev_power_seq_list_free(lwis_dev->power_up_sequence);
				lwis_dev->power_up_sequence = NULL;
			}
			if (lwis_dev->power_down_sequence) {
				lwis_dev_power_seq_list_free(lwis_dev->power_down_sequence);
				lwis_dev->power_down_sequence = NULL;
			}
			/* Release device gpio list */
			if (lwis_dev->gpios_list) {
				lwis_gpios_list_free(lwis_dev->gpios_list);
				lwis_dev->gpios_list = NULL;
			}
			/* Release device gpio info irq list */
			if (lwis_dev->irq_gpios_info.irq_list) {
				lwis_interrupt_list_free(lwis_dev->irq_gpios_info.irq_list);
				lwis_dev->irq_gpios_info.irq_list = NULL;
			}
			if (lwis_dev->irq_gpios_info.gpios) {
				lwis_gpio_list_put(lwis_dev->irq_gpios_info.gpios,
						   &lwis_dev->plat_dev->dev);
				lwis_dev->irq_gpios_info.gpios = NULL;
			}
			/* Destroy device */
			if (!IS_ERR(lwis_dev->dev)) {
				device_destroy(core.dev_class,
					       MKDEV(core.device_major, lwis_dev->id));
			}
			list_del(&lwis_dev->dev_list);

			if (timer_pending(&lwis_dev->heartbeat_timer))
				del_timer(&lwis_dev->heartbeat_timer);
		}
	}
	mutex_unlock(&core.lock);
}

/*
 *  lwis_register_base_device: Create device class and device major number to
 *  the class of LWIS devices.
 *
 *  This is called once when the core LWIS driver is initialized.
 */
static int __init lwis_register_base_device(void)
{
	int ret = 0;

	/* Allocate ID management instance for device minor numbers */
	core.idr = kzalloc(sizeof(struct idr), GFP_KERNEL);
	if (!core.idr) {
		pr_err("Cannot allocate idr instance\n");
		return -ENOMEM;
	}

	mutex_lock(&core.lock);

	idr_init(core.idr);

	/* Acquire device major number and allocate the range to minor numbers
		 to the device */
	ret = alloc_chrdev_region(&core.lwis_devt, 0, LWIS_MAX_DEVICES, LWIS_DEVICE_NAME);
	if (ret) {
		pr_err("Error in allocating chrdev region\n");
		goto error_chrdev_alloc;
	}

	core.device_major = MAJOR(core.lwis_devt);

	/* Create a device class*/
	core.dev_class = class_create(THIS_MODULE, LWIS_CLASS_NAME);
	if (IS_ERR(core.dev_class)) {
		pr_err("Failed to create device class\n");
		ret = PTR_ERR(core.dev_class);
		goto error_class_create;
	}

	/* Allocate a character device */
	core.chr_dev = cdev_alloc();
	if (!core.chr_dev) {
		pr_err("Failed to allocate cdev\n");
		ret = -ENOMEM;
		goto error_cdev_alloc;
	}

	core.chr_dev->ops = &lwis_fops;

	ret = cdev_add(core.chr_dev, core.lwis_devt, LWIS_MAX_DEVICES);
	if (ret) {
		pr_err("Failed to add cdev\n");
		goto error_cdev_alloc;
	}

	INIT_LIST_HEAD(&core.lwis_dev_list);

#ifdef CONFIG_DEBUG_FS
	/* Create DebugFS directory for LWIS, if avaiable */
	core.dbg_root = debugfs_create_dir("lwis", NULL);
	if (IS_ERR_OR_NULL(core.dbg_root)) {
		/* No need to return error as this is just informational that
		   DebugFS is not present */
		pr_info("Failed to create DebugFS dir - DebugFS not present?");
		core.dbg_root = NULL;
	}
#endif

	mutex_unlock(&core.lock);

	return ret;

	/* Error conditions */
error_cdev_alloc:
	class_destroy(core.dev_class);
	core.dev_class = NULL;
error_class_create:
	unregister_chrdev_region(core.lwis_devt, LWIS_MAX_DEVICES);
error_chrdev_alloc:
	mutex_unlock(&core.lock);
	kfree(core.idr);
	core.idr = NULL;

	return ret;
}

static void lwis_unregister_base_device(void)
{
	mutex_lock(&core.lock);

#ifdef CONFIG_DEBUGFS
	debugfs_remove(core.dbg_root);
	core.dbg_root = NULL;
#endif

	cdev_del(core.chr_dev);
	core.chr_dev = NULL;

	class_destroy(core.dev_class);
	core.dev_class = NULL;

	unregister_chrdev_region(core.lwis_devt, LWIS_MAX_DEVICES);

	kfree(core.idr);
	core.idr = NULL;

	mutex_unlock(&core.lock);
}

/*
 *  lwis_base_device_init: Called during subsys_initcall routines.
 */
static int __init lwis_base_device_init(void)
{
	int ret = 0;

	pr_info("LWIS device initialization\n");

	/* Initialize the core struct */
	memset(&core, 0, sizeof(struct lwis_core));
	mutex_init(&core.lock);

	ret = lwis_register_base_device();
	if (ret) {
		pr_err("Failed to register LWIS base (%d)\n", ret);
		return ret;
	}

	ret = lwis_top_device_init();
	if (ret) {
		pr_err("Failed to lwis_top_device_init (%d)\n", ret);
		goto top_failure;
	}

	ret = lwis_ioreg_device_init();
	if (ret) {
		pr_err("Failed to lwis_ioreg_device_init (%d)\n", ret);
		goto ioreg_failure;
	}

	ret = lwis_i2c_device_init();
	if (ret) {
		pr_err("Failed to lwis_i2c_device_init (%d)\n", ret);
		goto i2c_failure;
	}

	ret = lwis_slc_device_init();
	if (ret) {
		pr_err("Failed to lwis_slc_device_init (%d)\n", ret);
		goto slc_failure;
	}

	ret = lwis_dpm_device_init();
	if (ret) {
		pr_err("Failed to lwis_dpm_device_init (%d)\n", ret);
		goto dpm_failure;
	}

	return 0;

dpm_failure:
	lwis_slc_device_deinit();
slc_failure:
	lwis_i2c_device_deinit();
i2c_failure:
	lwis_ioreg_device_deinit();
ioreg_failure:
	lwis_top_device_deinit();
top_failure:
	lwis_unregister_base_device();
	return ret;
}

/*
 *  lwis_base_device_deinit: Called when driver is unloaded.
 */
static void __exit lwis_driver_exit(void)
{
	struct lwis_device *lwis_dev, *temp;
	struct lwis_client *client, *client_temp;

	pr_info("%s Clean up LWIS devices.\n", __func__);
	list_for_each_entry_safe (lwis_dev, temp, &core.lwis_dev_list, dev_list) {
		pr_info("Destroy device %s id %d", lwis_dev->name, lwis_dev->id);
		lwis_device_debugfs_cleanup(lwis_dev);
		/* Disable lwis device events */
		lwis_device_event_enable(lwis_dev, LWIS_EVENT_ID_HEARTBEAT, false);
		if (lwis_dev->type == DEVICE_TYPE_I2C) {
			struct lwis_i2c_device *i2c_dev;
			i2c_dev = container_of(lwis_dev, struct lwis_i2c_device, base_dev);
			i2c_unregister_device(i2c_dev->client);
		}
		/* Relase each client registered with dev */
		list_for_each_entry_safe (client, client_temp, &lwis_dev->clients, node) {
			if (lwis_release_client(client))
				pr_info("Failed to release client.");
		}
		pm_runtime_disable(&lwis_dev->plat_dev->dev);
		/* Release device clock list */
		if (lwis_dev->clocks) {
			lwis_clock_list_free(lwis_dev->clocks);
		}
		/* Release device interrupt list */
		if (lwis_dev->irqs) {
			lwis_interrupt_list_free(lwis_dev->irqs);
		}
		/* Release device regulator list */
		if (lwis_dev->regulators) {
			lwis_regulator_put_all(lwis_dev->regulators);
			lwis_regulator_list_free(lwis_dev->regulators);
		}
		/* Release device phy list */
		if (lwis_dev->phys) {
			lwis_phy_list_free(lwis_dev->phys);
		}
		/* Release device power sequence list */
		if (lwis_dev->power_up_sequence) {
			lwis_dev_power_seq_list_free(lwis_dev->power_up_sequence);
		}
		if (lwis_dev->power_down_sequence) {
			lwis_dev_power_seq_list_free(lwis_dev->power_down_sequence);
		}
		/* Release device gpio list */
		if (lwis_dev->gpios_list) {
			lwis_gpios_list_free(lwis_dev->gpios_list);
		}
		/* Release device gpio info irq list */
		if (lwis_dev->irq_gpios_info.irq_list) {
			lwis_interrupt_list_free(lwis_dev->irq_gpios_info.irq_list);
		}
		if (lwis_dev->irq_gpios_info.gpios) {
			lwis_gpio_list_put(lwis_dev->irq_gpios_info.gpios,
					   &lwis_dev->plat_dev->dev);
		}
		if (lwis_dev->reset_gpios) {
			lwis_gpio_list_put(lwis_dev->reset_gpios, &lwis_dev->plat_dev->dev);
		}
		if (lwis_dev->enable_gpios) {
			lwis_gpio_list_put(lwis_dev->enable_gpios, &lwis_dev->plat_dev->dev);
		}
		if (lwis_dev->shared_enable_gpios) {
			lwis_gpio_list_put(lwis_dev->shared_enable_gpios, &lwis_dev->plat_dev->dev);
		}
		/* Release event subscription components */
		if (lwis_dev->type == DEVICE_TYPE_TOP) {
			lwis_dev->top_dev->subscribe_ops.release(lwis_dev);
		}

		/* Destroy device */
		device_destroy(core.dev_class, MKDEV(core.device_major, lwis_dev->id));
		list_del(&lwis_dev->dev_list);
		kfree(lwis_dev);
	}

	/* Deinit device classes */
	lwis_dpm_device_deinit();
	lwis_slc_device_deinit();
	lwis_i2c_device_deinit();
	lwis_ioreg_device_deinit();
	lwis_top_device_deinit();

	/* Unregister base lwis device */
	lwis_unregister_base_device();
}

subsys_initcall(lwis_base_device_init);
module_exit(lwis_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Google-ACMA");
MODULE_DESCRIPTION("LWIS Base Device Driver");
