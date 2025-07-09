/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Regulator Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_REGULATOR_H_
#define LWIS_REGULATOR_H_

#include <linux/regulator/consumer.h>
#include "lwis_commands.h"

struct lwis_regulator {
	struct regulator *reg;
	char name[LWIS_MAX_NAME_STRING_LEN];
	int voltage;
};

struct lwis_regulator_list {
	struct lwis_regulator *reg;
	int count;
};

/*
 *  lwis_regulator_list_alloc: Allocate an instance of the lwis_regulator_list
 *  and initialize the data structures according to the number of regulators
 *  specified.
 *  NOTE: This does not register the regulator structs.
 */
struct lwis_regulator_list *lwis_regulator_list_alloc(int num_regs);

/*
 *  lwis_regulator_list_free: Deallocate the lwis_regulator_list structure.
 */
void lwis_regulator_list_free(struct lwis_regulator_list *list);

/*
 *  lwis_regulator_get: Register the regulator by name.
 *  Returns: index number (>= 0) if success, -ve if error
 */
int lwis_regulator_get(struct lwis_regulator_list *list, char *name, int voltage,
		       struct device *dev);

/*
 *  lwis_regulator_put_by_idx: Unregister the regulator by index.
 *  Returns: 0 if success, -ve if error
 */
int lwis_regulator_put_by_idx(struct lwis_regulator_list *list, int index);

/*
 *  lwis_regulator_put_by_name: Unregister the regulator by name.
 *  Returns: 0 if success, -ve if error
 */
int lwis_regulator_put_by_name(struct lwis_regulator_list *list, char *name);

/*
 *  lwis_regulator_put_all: Unregister the all the regulators in the list
 *  Returns: 0 if success, -ve if error
 */
int lwis_regulator_put_all(struct lwis_regulator_list *list);

/*
 *  lwis_regulator_enable_by_idx: Turn on/enable the regulator by index.
 *  Returns: 0 if success, -ve if error
 */
int lwis_regulator_enable_by_idx(struct lwis_regulator_list *list, int index);

/*
 *  lwis_regulator_enable_by_name: Turn on/enable the regulator by name.
 *  Returns: 0 if success, -ve if error
 */
int lwis_regulator_enable_by_name(struct lwis_regulator_list *list, char *name);

/*
 *  lwis_regulator_enable_all: Turn on/enable all the regulators.
 *  Returns: 0 if success, -ve if error
 */
int lwis_regulator_enable_all(struct lwis_regulator_list *list);

/*
 *  lwis_regulator_disable_by_idx: Turn off/disable the regulator by index.
 *  Returns: 0 if success, -ve if error
 */
int lwis_regulator_disable_by_idx(struct lwis_regulator_list *list, int index);

/*
 *  lwis_regulator_disable_by_name: Turn off/disable the regulator by name.
 *  Returns: 0 if success, -ve if error
 */
int lwis_regulator_disable_by_name(struct lwis_regulator_list *list, char *name);

/*
 *  lwis_regulator_disable_all: Turn off/disable all the regulators.
 *  Returns: 0 if success, -ve if error
 */
int lwis_regulator_disable_all(struct lwis_regulator_list *list);

/*
 *  lwis_regulator_print: Debug function to print all the regulators in the
 *  supplied list.
 */
void lwis_regulator_print(struct lwis_regulator_list *list);

#endif /* LWIS_REGULATOR_H_ */
