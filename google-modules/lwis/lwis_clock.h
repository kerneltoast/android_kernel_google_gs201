/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Clock Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_CLOCK_H_
#define LWIS_CLOCK_H_

#include <linux/clk.h>
#include <linux/device.h>

/*
 *  LWIS Clock Structures
 */

struct lwis_clock {
	struct clk *clk;
	char *name;
	uint32_t rate;
};

struct lwis_clock_list {
	struct lwis_clock *clk;
	int count;
};

/*
 *  LWIS Clock Interface Functions
 */

/*
 *  lwis_clock_list_alloc: Allocate an instance of the lwis_clock_list
 *  and initialize the data structures according to the number of clocks
 *  specified.
 *  NOTE: This does not register the clock structs.
 */
struct lwis_clock_list *lwis_clock_list_alloc(int num_clks);

/*
 *  lwis_clock_list_free: Deallocate the lwis_clock_list structure.
 */
void lwis_clock_list_free(struct lwis_clock_list *list);

/*
 *  lwis_clock_get: Register the clock by name and store its assigned
 *  clock rate.
 *  Returns: index number (>= 0) if success, -ve if error
 */
int lwis_clock_get(struct lwis_clock_list *list, char *name, struct device *dev, uint32_t rate);

/*
 *  lwis_clock_put_by_idx: Unregister the clock by index.
 *  Returns: 0 if success, -ve if error
 */
int lwis_clock_put_by_idx(struct lwis_clock_list *list, int index, struct device *dev);

/*
 *  lwis_clock_put_by_name: Unregister the clock by name.
 *  Returns: 0 if success, -ve if error
 */
int lwis_clock_put_by_name(struct lwis_clock_list *list, char *name, struct device *dev);

/*
 *  lwis_clock_enable_by_idx: Enable clock by index.
 *  Returns: 0 if success, -ve if error
 */
int lwis_clock_enable_by_idx(struct lwis_clock_list *list, int index);

/*
 *  lwis_clock_enable_by_name: Enable clock by name.
 *  Returns: 0 if success, -ve if error
 */
int lwis_clock_enable_by_name(struct lwis_clock_list *list, char *name);

/*
 *  lwis_clock_enable_all: Enable all clocks.
 *  Returns: 0 if success, -ve if error
 */
int lwis_clock_enable_all(struct lwis_clock_list *list);

/*
 *  lwis_clock_disable_by_idx: Disable clock by index.
 */
void lwis_clock_disable_by_idx(struct lwis_clock_list *list, int index);

/*
 *  lwis_clock_disable_by_name: Disable clock by name.
 */
void lwis_clock_disable_by_name(struct lwis_clock_list *list, char *name);

/*
 *  lwis_clock_disable_all: Disable all clocks.
 */
void lwis_clock_disable_all(struct lwis_clock_list *list);

/*
 *  lwis_clock_print: Debug function to print all the clocks in the
 *  supplied list.
 */
void lwis_clock_print(struct lwis_clock_list *list);

#endif /* LWIS_CLOCK_H_ */
