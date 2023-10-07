/*
 * Google LWIS GPIO Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_GPIO_H_
#define LWIS_GPIO_H_

#include <linux/gpio/consumer.h>
#include "lwis_commands.h"
#include "lwis_interrupt.h"

/*
 * struct lwis_gpios_info
 * This structure is to store the gpios information
 */
struct lwis_gpios_info {
	int id;
	struct device *hold_dev;
	char name[LWIS_MAX_NAME_STRING_LEN];
	bool is_shared;
	bool is_pulse;
	struct gpio_descs *gpios;
	struct lwis_interrupt_list *irq_list;
};

/*
 * struct lwis_gpios_list
 * This structure is to store the gpios that have been acquired
 */
struct lwis_gpios_list {
	struct lwis_gpios_info *gpios_info;
	/* Count of gpios info */
	int count;
};

/*
 *  LWIS GPIO Interface Functions
 */

/* debug function */
void lwis_gpio_list_print(char *name, struct gpio_descs *gpios);

/*
 *  Acquire GPIO descriptors.
 */
struct gpio_descs *lwis_gpio_list_get(struct device *dev, const char *name);

/*
 *  Release GPIO descriptors.
 */
void lwis_gpio_list_put(struct gpio_descs *gpios, struct device *dev);

/*
 *  Set output value for all the GPIOs in the list.  This function takes active
 *  high/low into consideration already, i.e. 0 = deasserted, 1 = asserted.
 */
int lwis_gpio_list_set_output_value(struct gpio_descs *gpios, int value);

/*
 *  Set output value for all the GPIOs in the list.  This function ignores the
 *  active-low or open drain property of a GPIO and work on the raw line value,
 *  i.e. 0 = physical line low, 1 = physical line high.
 */
int lwis_gpio_list_set_output_value_raw(struct gpio_descs *gpios, int value);

/*
 *  Set all the GPIO pins in the list to input.
 */
int lwis_gpio_list_set_input(struct gpio_descs *gpios);

/*
 *  Allocate an instance of the lwis_gpios_info and initialize
 *  the data structures according to the number of lwis_gpios_info
 *  specified.
 */
struct lwis_gpios_list *lwis_gpios_list_alloc(int count);

/*
 *  Deallocate the lwis_gpios_list structure.
 */
void lwis_gpios_list_free(struct lwis_gpios_list *list);

/*
 *  Search the lwis_device_gpios_list and return the lwis_gpios_info
 *  if the name is matched
 */
struct lwis_gpios_info *lwis_gpios_get_info_by_name(struct lwis_gpios_list *list, char *name);

#endif /* LWIS_GPIO_H_ */
