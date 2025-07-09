/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS PHY Interface
 *
 * Copyright (c) 2018 Google, LLC
 */

#ifndef LWIS_PHY_H_
#define LWIS_PHY_H_

#include <linux/phy/phy.h>

struct lwis_phy {
	struct phy *phy;
	char *name;
};

struct lwis_phy_list {
	struct lwis_phy *phy;
	int count;
};

/*
 *  lwis_phy_list_alloc: Allocate an instance of the lwis_phy_list
 *  and initialize the data structures according to the number of PHYs
 *  specified.
 *  NOTE: This does not register the PHY structs.
 */
struct lwis_phy_list *lwis_phy_list_alloc(int count);

/*
 *  lwis_phy_list_free: Deallocate the lwis_phy_list structure.
 */
void lwis_phy_list_free(struct lwis_phy_list *list);

/*
 *  lwis_phy_get: Register the PHY by name.
 *  Returns: index number (>= 0) if success, -ve if error
 */
int lwis_phy_get(struct lwis_phy_list *list, char *name, struct device *dev);

/*
 *  lwis_phy_put_by_idx: Unregister the PHY by index.
 *  Returns: 0 if success, -ve if error
 */
int lwis_phy_put_by_idx(struct lwis_phy_list *list, int index, struct device *dev);

/*
 *  lwis_phy_put_by_name: Unregister the PHY by name.
 *  Returns: 0 if success, -ve if error
 */
int lwis_phy_put_by_name(struct lwis_phy_list *list, char *name, struct device *dev);

/*
 *  lwis_phy_set_power_by_idx: Turn power on/off for a specific PHY by index.
 *  Returns: 0 if success, -ve if error
 */
int lwis_phy_set_power_by_idx(struct lwis_phy_list *list, int index, bool power_on);

/*
 *  lwis_phy_set_power_by_name: Turn power on/off for a specific PHY by name.
 *  Returns: 0 if success, -ve if error
 */
int lwis_phy_set_power_by_name(struct lwis_phy_list *list, char *name, bool power_on);

/*
 *  lwis_phy_set_power_all: Turn power on/off for all PHYs in list.
 *  Returns: 0 if success, -ve if error
 */
int lwis_phy_set_power_all(struct lwis_phy_list *list, bool power_on);

/*
 *  lwis_phy_print: Debug function to print all the PHYs in the
 *  supplied list.
 */
void lwis_phy_print(struct lwis_phy_list *list);

#endif /* LWIS_PHY_H_ */
