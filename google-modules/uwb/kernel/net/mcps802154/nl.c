/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
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

#include <linux/rtnetlink.h>
#include <net/genetlink.h>
#include <linux/version.h>
#include <net/mcps802154_nl.h>

#include "mcps802154_i.h"
#include "llhw-ops.h"
#include "nl.h"

#define ATTR_STRING_SIZE 20
#define ATTR_STRING_POLICY                                          \
	{                                                           \
		.type = NLA_NUL_STRING, .len = ATTR_STRING_SIZE - 1 \
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
#define nla_strscpy nla_strlcpy
#endif

static struct genl_family mcps802154_nl_family;

static const struct nla_policy
	mcps802154_nl_calibration_policy[MCPS802154_CALIBRATIONS_ATTR_MAX + 1] = {
		[MCPS802154_CALIBRATIONS_ATTR_KEY] = { .type = NLA_NUL_STRING,
						       .len = 64 },
		[MCPS802154_CALIBRATIONS_ATTR_VALUE] = { .type = NLA_BINARY },
		[MCPS802154_CALIBRATIONS_ATTR_STATUS] = { .type = NLA_S32 },
	};

static const struct nla_policy
	mcps802154_nl_region_policy[MCPS802154_REGION_MAX + 1] = {
		[MCPS802154_REGION_ATTR_ID] = { .type = NLA_U32 },
		[MCPS802154_REGION_ATTR_NAME] = ATTR_STRING_POLICY,
		[MCPS802154_REGION_ATTR_PARAMS] = { .type = NLA_NESTED },
		[MCPS802154_REGION_ATTR_CALL] = { .type = NLA_U32 },
		[MCPS802154_REGION_ATTR_CALL_PARAMS] = { .type = NLA_NESTED },
	};

static const struct nla_policy mcps802154_nl_policy[MCPS802154_ATTR_MAX + 1] = {
	[MCPS802154_ATTR_HW] = { .type = NLA_U32 },
	[MCPS802154_ATTR_WPAN_PHY_NAME] = ATTR_STRING_POLICY,
	[MCPS802154_ATTR_SCHEDULER_NAME] = ATTR_STRING_POLICY,
	[MCPS802154_ATTR_SCHEDULER_PARAMS] = { .type = NLA_NESTED },
	[MCPS802154_ATTR_SCHEDULER_REGIONS] =
		NLA_POLICY_NESTED_ARRAY(mcps802154_nl_region_policy),
	[MCPS802154_ATTR_SCHEDULER_CALL] = { .type = NLA_U32 },
	[MCPS802154_ATTR_SCHEDULER_CALL_PARAMS] = { .type = NLA_NESTED },
	[MCPS802154_ATTR_SCHEDULER_REGION_CALL] = { .type = NLA_NESTED },
	[MCPS802154_ATTR_CALIBRATIONS] = { .type = NLA_NESTED },
	[MCPS802154_ATTR_PWR_STATS] = { .type = NLA_NESTED },

#ifdef CONFIG_MCPS802154_TESTMODE
	[MCPS802154_ATTR_TESTDATA] = { .type = NLA_NESTED },
#endif
};

/**
 * mcps802154_nl_send_hw() - Append device information to a netlink message.
 * @local: MCPS private data.
 * @msg: Message to write to.
 * @portid: Destination port address.
 * @seq: Message sequence number.
 * @flags: Message flags (0 or NLM_F_MULTI).
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_send_hw(struct mcps802154_local *local,
				 struct sk_buff *msg, u32 portid, u32 seq,
				 int flags)
{
	void *hdr;

	hdr = genlmsg_put(msg, portid, seq, &mcps802154_nl_family, flags,
			  MCPS802154_CMD_NEW_HW);
	if (!hdr)
		return -ENOBUFS;

	if (nla_put_u32(msg, MCPS802154_ATTR_HW, local->hw_idx) ||
	    nla_put_string(msg, MCPS802154_ATTR_WPAN_PHY_NAME,
			   wpan_phy_name(local->hw->phy)))
		goto error;

	if (local->ca.scheduler &&
	    nla_put_string(msg, MCPS802154_ATTR_SCHEDULER_NAME,
			   local->ca.scheduler->ops->name))
		goto error;

	genlmsg_end(msg, hdr);
	return 0;
error:
	genlmsg_cancel(msg, hdr);
	return -EMSGSIZE;
}

/**
 * mcps802154_nl_get_hw() - Request information about a device.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_get_hw(struct sk_buff *skb, struct genl_info *info)
{
	struct sk_buff *msg;
	struct mcps802154_local *local = info->user_ptr[0];

	msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	if (mcps802154_nl_send_hw(local, msg, info->snd_portid, info->snd_seq,
				  0)) {
		nlmsg_free(msg);
		return -ENOBUFS;
	}

	return genlmsg_reply(msg, info);
}

/**
 * mcps802154_nl_dump_hw() - Dump information on all devices.
 * @skb: Allocated message for response.
 * @cb: Netlink callbacks.
 *
 * Return: Size of response message, or error.
 */
static int mcps802154_nl_dump_hw(struct sk_buff *skb,
				 struct netlink_callback *cb)
{
	int start_idx = cb->args[0];
	int r = 0;
	struct mcps802154_local *local;

	rtnl_lock();
	local = mcps802154_get_first_by_idx(start_idx);
	if (local) {
		cb->args[0] = local->hw_idx + 1;
		r = mcps802154_nl_send_hw(local, skb,
					  NETLINK_CB(cb->skb).portid,
					  cb->nlh->nlmsg_seq, NLM_F_MULTI);
	}
	rtnl_unlock();

	return r ? r : skb->len;
}

/**
 * mcps802154_nl_set_regions() - Set the regions which populate the schedule.
 * @local: MCPS private data.
 * @scheduler_name: Name of the scheduler.
 * @regions_attr: Nested attribute containing regions parameters.
 * @extack: Extended ACK report structure.
 * @update: True if we need only to update region parameters.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_set_regions(struct mcps802154_local *local,
				     const char *scheduler_name,
				     const struct nlattr *regions_attr,
				     struct netlink_ext_ack *extack,
				     bool update)
{
	struct nlattr *request;
	struct nlattr *attrs[MCPS802154_REGION_MAX + 1];
	int r, rem;
	u32 region_id = 0;
	char region_name[ATTR_STRING_SIZE];

	if (!regions_attr)
		return -EINVAL;

	nla_for_each_nested (request, regions_attr, rem) {
		r = nla_parse_nested(attrs, MCPS802154_REGION_MAX, request,
				     mcps802154_nl_region_policy, extack);
		if (r)
			return r;

		if (!attrs[MCPS802154_REGION_ATTR_NAME])
			return -EINVAL;

		if (attrs[MCPS802154_REGION_ATTR_ID])
			region_id =
				nla_get_s32(attrs[MCPS802154_REGION_ATTR_ID]);
		nla_strscpy(region_name, attrs[MCPS802154_REGION_ATTR_NAME],
			    sizeof(region_name));
		mutex_lock(&local->fsm_lock);
		if (update)
			r = mcps802154_ca_set_region_parameters(
				local, scheduler_name, region_id, region_name,
				attrs[MCPS802154_REGION_ATTR_PARAMS], extack);
		else
			r = mcps802154_ca_set_region(
				local, scheduler_name, region_id, region_name,
				attrs[MCPS802154_REGION_ATTR_PARAMS], extack);
		mutex_unlock(&local->fsm_lock);
		if (r)
			return r;
	}

	return 0;
}

/**
 * mcps802154_nl_generic_set_params() - Set scheduler parameters and/or regions.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_generic_set_params(struct sk_buff *skb,
					    struct genl_info *info)
{
	struct mcps802154_local *local = info->user_ptr[0];
	int r;
	struct nlattr *params_attr =
		info->attrs[MCPS802154_ATTR_SCHEDULER_PARAMS];
	struct nlattr *regions_attr =
		info->attrs[MCPS802154_ATTR_SCHEDULER_REGIONS];
	struct nlattr *name_attr = info->attrs[MCPS802154_ATTR_SCHEDULER_NAME];
	char name[ATTR_STRING_SIZE];
	enum mcps802154_commands cmd = info->genlhdr->cmd;

	if (!name_attr ||
	    (cmd == MCPS802154_CMD_SET_SCHEDULER_PARAMS && !params_attr) ||
	    ((cmd == MCPS802154_CMD_SET_SCHEDULER_REGIONS ||
	      cmd == MCPS802154_CMD_SET_REGIONS_PARAMS) &&
	     (!regions_attr || params_attr)))
		return -EINVAL;

	nla_strscpy(name, name_attr, sizeof(name));

	if (params_attr) {
		mutex_lock(&local->fsm_lock);
		r = mcps802154_ca_scheduler_set_parameters(
			local, name, params_attr, info->extack);
		mutex_unlock(&local->fsm_lock);

		if (r)
			return r;
	}

	if (regions_attr)
		r = mcps802154_nl_set_regions(
			local, name, regions_attr, info->extack,
			cmd == MCPS802154_CMD_SET_REGIONS_PARAMS);

	return r;
}

/**
 * mcps802154_nl_set_scheduler() - Set the scheduler which manage the schedule.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_set_scheduler(struct sk_buff *skb,
				       struct genl_info *info)
{
	struct mcps802154_local *local = info->user_ptr[0];
	struct nlattr *params_attr =
		info->attrs[MCPS802154_ATTR_SCHEDULER_PARAMS];
	struct nlattr *regions_attr =
		info->attrs[MCPS802154_ATTR_SCHEDULER_REGIONS];
	char name[ATTR_STRING_SIZE];
	int r;

	if (!info->attrs[MCPS802154_ATTR_SCHEDULER_NAME])
		return -EINVAL;
	nla_strscpy(name, info->attrs[MCPS802154_ATTR_SCHEDULER_NAME],
		    sizeof(name));

	mutex_lock(&local->fsm_lock);
	r = mcps802154_ca_set_scheduler(local, name, params_attr, info->extack);
	mutex_unlock(&local->fsm_lock);
	if (r)
		return r;

	if (regions_attr)
		r = mcps802154_nl_set_regions(local, name, regions_attr,
					      info->extack, false);

	return r;
}

/**
 * mcps802154_nl_call_scheduler() - Call scheduler specific procedure.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_call_scheduler(struct sk_buff *skb,
					struct genl_info *info)
{
	struct mcps802154_local *local = info->user_ptr[0];
	int r;
	struct nlattr *params_attr =
		info->attrs[MCPS802154_ATTR_SCHEDULER_CALL_PARAMS];
	struct nlattr *call_attr = info->attrs[MCPS802154_ATTR_SCHEDULER_CALL];
	struct nlattr *name_attr = info->attrs[MCPS802154_ATTR_SCHEDULER_NAME];
	char name[ATTR_STRING_SIZE];
	u32 call_id;

	if (!name_attr || !call_attr)
		return -EINVAL;

	nla_strscpy(name, name_attr, sizeof(name));
	call_id = nla_get_u32(call_attr);

	mutex_lock(&local->fsm_lock);
	r = mcps802154_ca_scheduler_call(local, name, call_id, params_attr,
					 info);
	mutex_unlock(&local->fsm_lock);

	return r;
}

/**
 * mcps802154_nl_call_region() - Call region specific procedure.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_call_region(struct sk_buff *skb,
				     struct genl_info *info)
{
	struct mcps802154_local *local = info->user_ptr[0];
	struct nlattr *region_call_attr =
		info->attrs[MCPS802154_ATTR_SCHEDULER_REGION_CALL];
	struct nlattr *name_attr = info->attrs[MCPS802154_ATTR_SCHEDULER_NAME];
	struct nlattr *attrs[MCPS802154_REGION_MAX + 1];
	int r, call_id;
	char scheduler_name[ATTR_STRING_SIZE];
	u32 region_id = 0;
	char region_name[ATTR_STRING_SIZE];

	if (!name_attr || !region_call_attr)
		return -EINVAL;

	nla_strscpy(scheduler_name, name_attr, sizeof(scheduler_name));

	r = nla_parse_nested(attrs, MCPS802154_REGION_MAX, region_call_attr,
			     mcps802154_nl_region_policy, info->extack);
	if (r)
		return r;

	if (!attrs[MCPS802154_REGION_ATTR_NAME] ||
	    !attrs[MCPS802154_REGION_ATTR_CALL])
		return -EINVAL;

	if (attrs[MCPS802154_REGION_ATTR_ID])
		region_id = nla_get_s32(attrs[MCPS802154_REGION_ATTR_ID]);
	nla_strscpy(region_name, attrs[MCPS802154_REGION_ATTR_NAME],
		    sizeof(region_name));
	call_id = nla_get_u32(attrs[MCPS802154_REGION_ATTR_CALL]);

	mutex_lock(&local->fsm_lock);
	local->cur_cmd_info = info;
	r = mcps802154_ca_call_region(local, scheduler_name, region_id,
				      region_name, call_id,
				      attrs[MCPS802154_REGION_ATTR_CALL_PARAMS],
				      info);
	local->cur_cmd_info = NULL;
	mutex_unlock(&local->fsm_lock);

	return r;
}

/**
 * mcps802154_nl_close_scheduler() - Close current scheduler and its regions.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_close_scheduler(struct sk_buff *skb,
					 struct genl_info *info)
{
	struct mcps802154_local *local = info->user_ptr[0];
	mutex_lock(&local->fsm_lock);
	local->cur_cmd_info = info;
	mcps802154_ca_close(local);
	local->cur_cmd_info = NULL;
	mutex_unlock(&local->fsm_lock);

	return 0;
}

struct sk_buff *
mcps802154_region_call_alloc_reply_skb(struct mcps802154_llhw *llhw,
				       struct mcps802154_region *region,
				       u32 call_id, int approx_len)
{
	struct mcps802154_local *local = llhw_to_local(llhw);
	struct sk_buff *msg;
	void *hdr;
	struct nlattr *call, *params;

	if (WARN_ON(!local->cur_cmd_info))
		return NULL;

	msg = nlmsg_new(approx_len + NLMSG_HDRLEN, GFP_KERNEL);
	if (!msg)
		return NULL;

	hdr = genlmsg_put(msg, local->cur_cmd_info->snd_portid,
			  local->cur_cmd_info->snd_seq, &mcps802154_nl_family,
			  0, MCPS802154_CMD_CALL_REGION);
	if (!hdr)
		goto nla_put_failure;

	if (nla_put_u32(msg, MCPS802154_ATTR_HW, local->hw_idx))
		goto nla_put_failure;

	call = nla_nest_start(msg, MCPS802154_ATTR_SCHEDULER_REGION_CALL);
	if (!call)
		goto nla_put_failure;

	if (nla_put_string(msg, MCPS802154_REGION_ATTR_NAME,
			   region->ops->name) ||
	    nla_put_u32(msg, MCPS802154_REGION_ATTR_CALL, call_id))
		goto nla_put_failure;

	params = nla_nest_start(msg, MCPS802154_REGION_ATTR_CALL_PARAMS);
	if (!params)
		goto nla_put_failure;

	((void **)msg->cb)[0] = hdr;
	((void **)msg->cb)[1] = call;
	((void **)msg->cb)[2] = params;

	return msg;
nla_put_failure:
	kfree_skb(msg);
	return NULL;
}
EXPORT_SYMBOL(mcps802154_region_call_alloc_reply_skb);

int mcps802154_region_call_reply(struct mcps802154_llhw *llhw,
				 struct sk_buff *skb)
{
	struct mcps802154_local *local = llhw_to_local(llhw);
	void *hdr = ((void **)skb->cb)[0];
	struct nlattr *call = ((void **)skb->cb)[1];
	struct nlattr *params = ((void **)skb->cb)[2];

	/* Clear CB data for netlink core to own from now on. */
	memset(skb->cb, 0, sizeof(skb->cb));

	if (WARN_ON(!local->cur_cmd_info)) {
		kfree_skb(skb);
		return -EINVAL;
	}

	nla_nest_end(skb, params);
	nla_nest_end(skb, call);
	genlmsg_end(skb, hdr);

	return genlmsg_reply(skb, local->cur_cmd_info);
}
EXPORT_SYMBOL(mcps802154_region_call_reply);

struct sk_buff *
mcps802154_region_event_alloc_skb(struct mcps802154_llhw *llhw,
				  struct mcps802154_region *region, u32 call_id,
				  u32 portid, int approx_len, gfp_t gfp)
{
	struct mcps802154_local *local = llhw_to_local(llhw);
	struct sk_buff *msg;
	void *hdr;
	struct nlattr *call, *params;

	msg = nlmsg_new(approx_len + NLMSG_HDRLEN, gfp);
	if (!msg)
		return NULL;

	hdr = genlmsg_put(msg, portid, 0, &mcps802154_nl_family, 0,
			  MCPS802154_CMD_CALL_REGION);
	if (!hdr)
		goto nla_put_failure;

	if (nla_put_u32(msg, MCPS802154_ATTR_HW, local->hw_idx))
		goto nla_put_failure;

	call = nla_nest_start(msg, MCPS802154_ATTR_SCHEDULER_REGION_CALL);
	if (!call)
		goto nla_put_failure;

	if (nla_put_string(msg, MCPS802154_REGION_ATTR_NAME,
			   region->ops->name) ||
	    nla_put_u32(msg, MCPS802154_REGION_ATTR_CALL, call_id))
		goto nla_put_failure;

	params = nla_nest_start(msg, MCPS802154_REGION_ATTR_CALL_PARAMS);
	if (!params)
		goto nla_put_failure;

	((void **)msg->cb)[0] = hdr;
	((void **)msg->cb)[1] = call;
	((void **)msg->cb)[2] = params;

	return msg;
nla_put_failure:
	kfree_skb(msg);
	return NULL;
}
EXPORT_SYMBOL(mcps802154_region_event_alloc_skb);

int mcps802154_region_event(struct mcps802154_llhw *llhw, struct sk_buff *skb)
{
	struct mcps802154_local *local = llhw_to_local(llhw);
	void *hdr = ((void **)skb->cb)[0];
	struct nlmsghdr *nlhdr = nlmsg_hdr(skb);
	struct nlattr *call = ((void **)skb->cb)[1];
	struct nlattr *params = ((void **)skb->cb)[2];

	/* Clear CB data for netlink core to own from now on. */
	memset(skb->cb, 0, sizeof(skb->cb));

	nla_nest_end(skb, params);
	nla_nest_end(skb, call);
	genlmsg_end(skb, hdr);

	return genlmsg_unicast(wpan_phy_net(local->hw->phy), skb,
			       nlhdr->nlmsg_pid);
}
EXPORT_SYMBOL(mcps802154_region_event);

#ifdef CONFIG_MCPS802154_TESTMODE
/**
 * mcps802154_nl_testmode_do() - Run a testmode command.
 * @skb: Request message.
 * @info: Request information.
 *
 * This function is a passthrough to send testmode command to
 * the driver.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_testmode_do(struct sk_buff *skb,
				     struct genl_info *info)
{
	struct mcps802154_local *local = info->user_ptr[0];
	int r;

	if (!local->ops->testmode_cmd)
		return -EOPNOTSUPP;

	if (!info->attrs[MCPS802154_ATTR_TESTDATA])
		return -EINVAL;

	mutex_lock(&local->fsm_lock);
	local->cur_cmd_info = info;
	r = llhw_testmode_cmd(local,
			      nla_data(info->attrs[MCPS802154_ATTR_TESTDATA]),
			      nla_len(info->attrs[MCPS802154_ATTR_TESTDATA]));
	local->cur_cmd_info = NULL;
	mutex_unlock(&local->fsm_lock);
	return r;
}

struct sk_buff *
mcps802154_testmode_alloc_reply_skb(struct mcps802154_llhw *llhw, int approxlen)
{
	struct mcps802154_local *local = llhw_to_local(llhw);
	struct sk_buff *skb;
	void *hdr;
	struct nlattr *data;

	if (WARN_ON(!local->cur_cmd_info))
		return NULL;

	skb = nlmsg_new(approxlen + 100, GFP_KERNEL);
	if (!skb)
		return NULL;

	/* Append testmode header to the netlink message */
	hdr = genlmsg_put(skb, local->cur_cmd_info->snd_portid,
			  local->cur_cmd_info->snd_seq, &mcps802154_nl_family,
			  0, MCPS802154_CMD_TESTMODE);
	if (!hdr)
		goto nla_put_failure;

	/* Start putting nested testmode data into the netlink message */
	data = nla_nest_start(skb, MCPS802154_ATTR_TESTDATA);
	if (!data)
		goto nla_put_failure;

	/* We put our private variables there to keep them across layers */
	((void **)skb->cb)[0] = hdr;
	((void **)skb->cb)[1] = data;

	return skb;
nla_put_failure:
	kfree_skb(skb);
	return NULL;
}
EXPORT_SYMBOL(mcps802154_testmode_alloc_reply_skb);

int mcps802154_testmode_reply(struct mcps802154_llhw *llhw, struct sk_buff *skb)
{
	struct mcps802154_local *local = llhw_to_local(llhw);
	void *hdr = ((void **)skb->cb)[0];
	struct nlattr *data = ((void **)skb->cb)[1];

	/* Clear CB data for netlink core to own from now on. */
	memset(skb->cb, 0, sizeof(skb->cb));

	if (WARN_ON(!local->cur_cmd_info)) {
		kfree_skb(skb);
		return -EINVAL;
	}

	/* Stop putting nested testmode data into the netlink message */
	nla_nest_end(skb, data);
	genlmsg_end(skb, hdr);
	return genlmsg_reply(skb, local->cur_cmd_info);
}
EXPORT_SYMBOL(mcps802154_testmode_reply);
#endif

/**
 * mcps802154_nl_put_calibration() - put on calibration in msg.
 * @msg: Request message.
 * @key: calibration name
 * @status: status of reading operation, and length of calibration value when positive.
 * @data: calibration value, if available.
 * @onlykey: true to put only calibration key.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_put_calibration(struct sk_buff *msg, const char *key,
					 int status, void *data, bool onlykey)
{
	struct nlattr *calibration;
	int r;

	calibration = nla_nest_start(msg, 1);
	if (!calibration)
		return -EMSGSIZE;

	r = nla_put_string(msg, MCPS802154_CALIBRATIONS_ATTR_KEY, key);
	if (r)
		return -EMSGSIZE;
	if (onlykey)
		goto finish;

	if (status < 0)
		r = nla_put_s32(msg, MCPS802154_CALIBRATIONS_ATTR_STATUS,
				-status);
	else if (data != NULL)
		/* when positive, the status represent the data length. */
		r = nla_put(msg, MCPS802154_CALIBRATIONS_ATTR_VALUE, status,
			    data);
	if (r)
		return -EMSGSIZE;

finish:
	nla_nest_end(msg, calibration);
	return 0;
}

/**
 * mcps802154_nl_set_calibration() - Set calibrations parameters.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_set_calibration(struct sk_buff *skb,
					 struct genl_info *info)
{
	struct mcps802154_local *local = info->user_ptr[0];
	struct sk_buff *msg;
	void *hdr;
	int err;

	if (!local->ops->set_calibration)
		return -EOPNOTSUPP;

	msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq,
			  &mcps802154_nl_family, 0,
			  MCPS802154_CMD_SET_CALIBRATIONS);
	if (!hdr) {
		err = -ENOBUFS;
		goto failure;
	}

	if (nla_put_u32(msg, MCPS802154_ATTR_HW, local->hw_idx)) {
		err = -EMSGSIZE;
		goto nla_put_failure;
	}

	if (info->attrs[MCPS802154_ATTR_CALIBRATIONS]) {
		struct nlattr *attrs[MCPS802154_CALIBRATIONS_ATTR_MAX + 1];
		struct nlattr *calibrations, *input;
		int rem;

		nla_for_each_nested (
			input, info->attrs[MCPS802154_ATTR_CALIBRATIONS], rem) {
			char *key;
			int r;

			r = nla_parse_nested(
				attrs, MCPS802154_CALIBRATIONS_ATTR_MAX, input,
				mcps802154_nl_calibration_policy, info->extack);
			if (r)
				continue;

			if (!attrs[MCPS802154_CALIBRATIONS_ATTR_KEY])
				continue;
			key = nla_data(attrs[MCPS802154_CALIBRATIONS_ATTR_KEY]);

			if (!attrs[MCPS802154_CALIBRATIONS_ATTR_VALUE])
				r = -EINVAL;
			else {
				struct nlattr *value;

				value = attrs[MCPS802154_CALIBRATIONS_ATTR_VALUE];
				r = llhw_set_calibration(local, key,
							 nla_data(value),
							 nla_len(value));
			}
			if (r < 0) {
				calibrations = nla_nest_start(
					msg, MCPS802154_ATTR_CALIBRATIONS);
				/* Put the result in the response message. */
				err = mcps802154_nl_put_calibration(
					msg, key, r, NULL, false);
				if (err)
					goto nla_put_failure;
				nla_nest_end(msg, calibrations);
				break;
			}
		}
	}

	genlmsg_end(msg, hdr);
	return genlmsg_reply(msg, info);

nla_put_failure:
	genlmsg_cancel(msg, hdr);
failure:
	nlmsg_free(msg);
	return err;
}

/**
 * mcps802154_nl_get_calibration() - Set calibrations parameters.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_get_calibration(struct sk_buff *skb,
					 struct genl_info *info)
{
	struct nlattr *attrs[MCPS802154_CALIBRATIONS_ATTR_MAX + 1];
	struct mcps802154_local *local = info->user_ptr[0];
	struct nlattr *calibrations;
	struct nlattr *input;
	struct sk_buff *msg;
	void *hdr;
	char *key;
	u32 tmp[32];
	int err;
	int r;

	if (!local->ops->get_calibration)
		return -EOPNOTSUPP;

	/* NLMSG_DEFAULT_SIZE isn't enough for 4 antennas configuration.
	   So add a full page to maintain page alignment of the message size. */
	msg = nlmsg_new(NLMSG_DEFAULT_SIZE + PAGE_SIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq,
			  &mcps802154_nl_family, 0,
			  MCPS802154_CMD_GET_CALIBRATIONS);
	if (!hdr) {
		err = -ENOBUFS;
		goto failure;
	}

	/* Build the confirm message in same time as request message. */
	if (nla_put_u32(msg, MCPS802154_ATTR_HW, local->hw_idx)) {
		err = -EMSGSIZE;
		goto nla_put_failure;
	}

	calibrations = nla_nest_start(msg, MCPS802154_ATTR_CALIBRATIONS);
	if (info->attrs[MCPS802154_ATTR_CALIBRATIONS]) {
		int rem;

		nla_for_each_nested (
			input, info->attrs[MCPS802154_ATTR_CALIBRATIONS], rem) {
			r = nla_parse_nested(
				attrs, MCPS802154_CALIBRATIONS_ATTR_MAX, input,
				mcps802154_nl_calibration_policy, info->extack);
			if (r)
				continue;
			if (!attrs[MCPS802154_CALIBRATIONS_ATTR_KEY])
				continue;

			key = nla_data(attrs[MCPS802154_CALIBRATIONS_ATTR_KEY]);
			r = llhw_get_calibration(local, key, &tmp, sizeof(tmp));

			/* Put the result in the response message. */
			err = mcps802154_nl_put_calibration(msg, key, r, &tmp,
							    false);
			if (err)
				goto nla_put_failure;
		}
	} else if (local->ops->list_calibration) {
		const char *const *calibration;
		const char *const *entry;

		calibration = llhw_list_calibration(local);
		if (!calibration) {
			err = -ENOENT;
			goto nla_put_failure;
		}
		for (entry = calibration; *entry; entry++) {
			r = llhw_get_calibration(local, *entry, &tmp,
						 sizeof(tmp));

			/* Put the result in the response message. */
			err = mcps802154_nl_put_calibration(msg, *entry, r,
							    &tmp, false);
			if (err)
				goto nla_put_failure;
		}
	}
	nla_nest_end(msg, calibrations);

	genlmsg_end(msg, hdr);
	return genlmsg_reply(msg, info);

nla_put_failure:
	genlmsg_cancel(msg, hdr);
failure:
	nlmsg_free(msg);
	return err;
}

/**
 * mcps802154_nl_list_calibration() - Set calibrations parameters.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_list_calibration(struct sk_buff *skb,
					  struct genl_info *info)
{
	struct mcps802154_local *local = info->user_ptr[0];
	struct nlattr *calibrations;
	const char *const *list;
	const char *const *entry;
	struct sk_buff *msg;
	void *hdr;
	int err;

	if (!local->ops->list_calibration)
		return -EOPNOTSUPP;

	list = llhw_list_calibration(local);
	if (!list)
		return -ENOENT;
	/* NLMSG_DEFAULT_SIZE isn't enough for 4 antennas configuration.
	   So add a full page to maintain page alignment of the message size. */
	msg = nlmsg_new(NLMSG_DEFAULT_SIZE + PAGE_SIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq,
			  &mcps802154_nl_family, 0,
			  MCPS802154_CMD_LIST_CALIBRATIONS);
	if (!hdr) {
		err = -ENOBUFS;
		goto failure;
	}

	if (nla_put_u32(msg, MCPS802154_ATTR_HW, local->hw_idx)) {
		err = -EMSGSIZE;
		goto nla_put_failure;
	}

	calibrations = nla_nest_start(msg, MCPS802154_ATTR_CALIBRATIONS);
	for (entry = list; *entry; entry++) {
		/* Put the result in the response message. */
		err = mcps802154_nl_put_calibration(msg, *entry, 0, NULL, true);
		if (err)
			goto nla_put_failure;
	}
	nla_nest_end(msg, calibrations);

	genlmsg_end(msg, hdr);
	return genlmsg_reply(msg, info);

nla_put_failure:
	genlmsg_cancel(msg, hdr);
failure:
	nlmsg_free(msg);
	return err;
}

/**
 * mcps802154_nl_put_pwr_stats_state() - Put a power statistic state on a netlink message.
 * @msg: Netlink message.
 * @state: Related power statistic state.
 * @time: Duration of this state.
 * @count: Transitions count to this state.
 *
 * Return: 0 or error.
 */
static int
mcps802154_nl_put_pwr_stats_state(struct sk_buff *msg,
				  enum mcps802154_pwr_stats_attrs state,
				  u32 time, u32 count)
{
	struct nlattr *nl_pwr_stats_state;

	nl_pwr_stats_state = nla_nest_start(msg, state);
	if (!nl_pwr_stats_state)
		return -EMSGSIZE;
	if (nla_put_u32(msg, MCPS802154_PWR_STATS_STATE_ATTR_TIME, time))
		return -EMSGSIZE;
	if (nla_put_u32(msg, MCPS802154_PWR_STATS_STATE_ATTR_COUNT, count))
		return -EMSGSIZE;
	nla_nest_end(msg, nl_pwr_stats_state);
	return 0;
}

/**
 * mcps802154_nl_get_pwr_stats() - Get power statistics.
 * @skb: Request message.
 * @info: Request information.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_get_pwr_stats(struct sk_buff *skb,
				       struct genl_info *info)
{
	struct mcps802154_local *local = info->user_ptr[0];
	struct mcps802154_llhw *llhw = &local->llhw;
	struct sk_buff *msg;
	void *hdr;
	struct mcps802154_power_stats pwr_stats;
	struct nlattr *nl_pwr_stats;
	int rc;

	if (!local->ops->get_power_stats)
		return -EOPNOTSUPP;

	/* Get the power statistics from the low level hardware driver. */
	rc = local->ops->get_power_stats(llhw, &pwr_stats);
	if (rc)
		return rc;

	/* Build the response netlink message. */
	msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq,
			  &mcps802154_nl_family, 0,
			  MCPS802154_CMD_GET_PWR_STATS);
	if (!hdr) {
		rc = -ENOBUFS;
		goto failure;
	}

	nl_pwr_stats = nla_nest_start(msg, MCPS802154_ATTR_PWR_STATS);
	if (!nl_pwr_stats)
		goto nla_put_failure;

	/* Process the SLEEP state. */
	rc = mcps802154_nl_put_pwr_stats_state(
		msg, MCPS802154_PWR_STATS_ATTR_SLEEP,
		pwr_stats.power_state_stats[MCPS802154_PWR_STATE_SLEEP].dur /
			1000000,
		pwr_stats.power_state_stats[MCPS802154_PWR_STATE_SLEEP].count);
	if (rc)
		goto nla_put_failure;

	/* Process the IDLE state. */
	rc = mcps802154_nl_put_pwr_stats_state(
		msg, MCPS802154_PWR_STATS_ATTR_IDLE,
		pwr_stats.power_state_stats[MCPS802154_PWR_STATE_IDLE].dur /
			1000000,
		pwr_stats.power_state_stats[MCPS802154_PWR_STATE_IDLE].count);
	if (rc)
		goto nla_put_failure;

	/* Process the RX state. */
	rc = mcps802154_nl_put_pwr_stats_state(
		msg, MCPS802154_PWR_STATS_ATTR_RX,
		pwr_stats.power_state_stats[MCPS802154_PWR_STATE_RX].dur /
			1000000,
		pwr_stats.power_state_stats[MCPS802154_PWR_STATE_RX].count);
	if (rc)
		goto nla_put_failure;

	/* Process the TX state. */
	rc = mcps802154_nl_put_pwr_stats_state(
		msg, MCPS802154_PWR_STATS_ATTR_TX,
		pwr_stats.power_state_stats[MCPS802154_PWR_STATE_TX].dur /
			1000000,
		pwr_stats.power_state_stats[MCPS802154_PWR_STATE_TX].count);
	if (rc)
		goto nla_put_failure;

	/* Process the interrupts count. */
	if (nla_put_u32(msg, MCPS802154_PWR_STATS_ATTR_INTERRUPTS,
			pwr_stats.interrupts)) {
		rc = -EMSGSIZE;
		goto nla_put_failure;
	}

	nla_nest_end(msg, nl_pwr_stats);
	genlmsg_end(msg, hdr);
	return genlmsg_reply(msg, info);

nla_put_failure:
	genlmsg_cancel(msg, hdr);
failure:
	nlmsg_free(msg);
	return rc;
}

enum mcps802154_nl_internal_flags {
	MCPS802154_NL_NEED_HW = 1,
};

/**
 * mcps802154_get_from_info() - Retrieve private data from netlink request.
 * information.
 * @info: Request information.
 *
 * Return: Found MCPS data, or error pointer.
 */
static struct mcps802154_local *mcps802154_get_from_info(struct genl_info *info)
{
	struct nlattr **attrs = info->attrs;
	int hw_idx;
	struct mcps802154_local *local;

	ASSERT_RTNL();

	if (!attrs[MCPS802154_ATTR_HW])
		return ERR_PTR(-EINVAL);

	hw_idx = nla_get_u32(attrs[MCPS802154_ATTR_HW]);

	local = mcps802154_get_first_by_idx(hw_idx);
	if (!local || local->hw_idx != hw_idx)
		return ERR_PTR(-ENODEV);

	if (!net_eq(wpan_phy_net(local->hw->phy), genl_info_net(info)))
		return ERR_PTR(-ENODEV);

	return local;
}

/**
 * mcps802154_nl_pre_doit() - Called before single requests (but not dump).
 * @ops: Command to be executed ops structure.
 * @skb: Request message.
 * @info: Request information.
 *
 * Set MCPS private data in user_ptr[0] if needed, and lock RTNL to make it
 * stick.
 *
 * Return: 0 or error.
 */
static int mcps802154_nl_pre_doit(const struct genl_ops *ops,
				  struct sk_buff *skb, struct genl_info *info)
{
	struct mcps802154_local *local;

	if (ops->internal_flags & MCPS802154_NL_NEED_HW) {
		rtnl_lock();
		local = mcps802154_get_from_info(info);
		if (IS_ERR(local)) {
			rtnl_unlock();
			return PTR_ERR(local);
		}
		info->user_ptr[0] = local;
	}

	return 0;
}

/**
 * mcps802154_nl_post_doit() - Called after single requests (but not dump).
 * @ops: Command to be executed ops structure.
 * @skb: Request message.
 * @info: Request information.
 *
 * Release RTNL if needed.
 */
static void mcps802154_nl_post_doit(const struct genl_ops *ops,
				    struct sk_buff *skb, struct genl_info *info)
{
	if (ops->internal_flags & MCPS802154_NL_NEED_HW)
		rtnl_unlock();
}

static const struct genl_ops mcps802154_nl_ops[] = {
	{
		.cmd = MCPS802154_CMD_GET_HW,
		.doit = mcps802154_nl_get_hw,
		.dumpit = mcps802154_nl_dump_hw,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_SET_SCHEDULER,
		.doit = mcps802154_nl_set_scheduler,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_SET_SCHEDULER_PARAMS,
		.doit = mcps802154_nl_generic_set_params,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_CALL_SCHEDULER,
		.doit = mcps802154_nl_call_scheduler,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_CLOSE_SCHEDULER,
		.doit = mcps802154_nl_close_scheduler,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_SET_SCHEDULER_REGIONS,
		.doit = mcps802154_nl_generic_set_params,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_SET_REGIONS_PARAMS,
		.doit = mcps802154_nl_generic_set_params,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_CALL_REGION,
		.doit = mcps802154_nl_call_region,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
#ifdef CONFIG_MCPS802154_TESTMODE
	{
		.cmd = MCPS802154_CMD_TESTMODE,
		.doit = mcps802154_nl_testmode_do,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
#endif
	{
		.cmd = MCPS802154_CMD_SET_CALIBRATIONS,
		.doit = mcps802154_nl_set_calibration,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_GET_CALIBRATIONS,
		.doit = mcps802154_nl_get_calibration,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_LIST_CALIBRATIONS,
		.doit = mcps802154_nl_list_calibration,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
	{
		.cmd = MCPS802154_CMD_GET_PWR_STATS,
		.doit = mcps802154_nl_get_pwr_stats,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = MCPS802154_NL_NEED_HW,
	},
};

static struct genl_family mcps802154_nl_family __ro_after_init = {
	.name = MCPS802154_GENL_NAME,
	.version = 1,
	.maxattr = MCPS802154_ATTR_MAX,
	.policy = mcps802154_nl_policy,
	.netnsok = true,
	.pre_doit = mcps802154_nl_pre_doit,
	.post_doit = mcps802154_nl_post_doit,
	.ops = mcps802154_nl_ops,
	.n_ops = ARRAY_SIZE(mcps802154_nl_ops),
	.module = THIS_MODULE,
};

int __init mcps802154_nl_init(void)
{
	return genl_register_family(&mcps802154_nl_family);
}

void __exit mcps802154_nl_exit(void)
{
	genl_unregister_family(&mcps802154_nl_family);
}
