/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2022 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */

#include "idle_region.h"
#include "trace.h"
#include <net/idle_region_nl.h>
#include <linux/errno.h>
#include <linux/limits.h>

static struct mcps802154_region_ops idle_region_ops;

struct idle_local {
	/**
	 * @region: Region instance returned to MCPS.
	 */
	struct mcps802154_region region;
	/**
	 * @llhw: Low-level device pointer.
	 */
	struct mcps802154_llhw *llhw;
	/**
	 * @params: Parameters.
	 */
	struct idle_params params;
	/**
	 * @access: Access returned to ca.
	 */
	struct mcps802154_access access;
};

static inline struct idle_local *
region_to_local(struct mcps802154_region *region)
{
	return container_of(region, struct idle_local, region);
}

static struct mcps802154_region *idle_open(struct mcps802154_llhw *llhw)
{
	struct idle_local *local;

	local = kzalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;
	local->llhw = llhw;
	local->region.ops = &idle_region_ops;

	/* Default value of parameters. */
	local->params.min_duration_dtu = llhw->anticip_dtu * 2;
	local->params.max_duration_dtu = 0;

	return &local->region;
}

static void idle_close(struct mcps802154_region *region)
{
	kfree(region_to_local(region));
}

static const struct nla_policy idle_param_nla_policy[IDLE_PARAM_ATTR_MAX + 1] = {
	[IDLE_PARAM_ATTR_MIN_DURATION_DTU] = NLA_POLICY_MIN(NLA_S32, 0),
	[IDLE_PARAM_ATTR_MAX_DURATION_DTU] = NLA_POLICY_MIN(NLA_S32, 0),
};

static int idle_set_parameters(struct mcps802154_region *region,
			       const struct nlattr *params,
			       struct netlink_ext_ack *extack)
{
	struct idle_local *local = region_to_local(region);
	struct nlattr *attrs[IDLE_PARAM_ATTR_MAX + 1];
	struct idle_params *p = &local->params;
	int min_duration_dtu, max_duration_dtu;
	int r;

	r = nla_parse_nested(attrs, IDLE_PARAM_ATTR_MAX, params,
			     idle_param_nla_policy, extack);
	if (r)
		return r;

	min_duration_dtu =
		attrs[IDLE_PARAM_ATTR_MIN_DURATION_DTU] ?
			nla_get_s32(attrs[IDLE_PARAM_ATTR_MIN_DURATION_DTU]) :
			p->min_duration_dtu;
	max_duration_dtu =
		attrs[IDLE_PARAM_ATTR_MAX_DURATION_DTU] ?
			nla_get_s32(attrs[IDLE_PARAM_ATTR_MAX_DURATION_DTU]) :
			p->max_duration_dtu;
	if (max_duration_dtu && min_duration_dtu &&
	    min_duration_dtu > max_duration_dtu)
		return -EINVAL;

	p->min_duration_dtu = min_duration_dtu;
	p->max_duration_dtu = max_duration_dtu;
	trace_region_idle_params(p);
	return 0;
}

static struct mcps802154_access_ops idle_access_ops = {};

static struct mcps802154_access *
idle_get_access(struct mcps802154_region *region, u32 next_timestamp_dtu,
		int next_in_region_dtu, int region_duration_dtu)
{
	struct idle_local *local = region_to_local(region);
	struct mcps802154_access *access = &local->access;
	const struct idle_params *p = &local->params;
	int left_region_duration_dtu = region_duration_dtu - next_in_region_dtu;
	int duration_dtu;

	if (!left_region_duration_dtu) {
		/* Region used with endless scheduler. */
		duration_dtu = p->max_duration_dtu;
	} else {
		/* Region used directly in on_demand scheduler. */
		if (left_region_duration_dtu < p->min_duration_dtu)
			return NULL;
		duration_dtu = left_region_duration_dtu;
	}

	trace_region_idle_get_access(next_timestamp_dtu, duration_dtu);
	access->method = MCPS802154_ACCESS_METHOD_IDLE;
	access->ops = &idle_access_ops;
	access->timestamp_dtu = next_timestamp_dtu;
	access->duration_dtu = duration_dtu;

	return access;
}

static struct mcps802154_region_ops idle_region_ops = {
	.owner = THIS_MODULE,
	.name = "idle",
	.open = idle_open,
	.close = idle_close,
	.set_parameters = idle_set_parameters,
	.get_demand = NULL, /* Not wanted. */
	.get_access = idle_get_access,
};

int mcps802154_idle_region_init(void)
{
	return mcps802154_region_register(&idle_region_ops);
}

void mcps802154_idle_region_exit(void)
{
	mcps802154_region_unregister(&idle_region_ops);
}
