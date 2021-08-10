// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019, Google Inc
 *
 * USB input current management.
 *
 */

#include <linux/alarmtimer.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <misc/gvotable.h>
#include <misc/logbuffer.h>
#include <uapi/linux/sched/types.h>

#include "usb_psy.h"
#include "usb_icl_voter.h"

#ifndef CHAR_BIT
#define CHAR_BIT 8
#endif

/*
 * Greater than SDP_SUSPEND_UA for remaining in parity with in-market pixels.
 * Lesser than SDP_HS_CONN_UA for preventing.
 */
#define ONLINE_THRESHOLD_UA 50000

#define CDP_DCP_ICL_UA 1500000
#define SDP_DISCONNECT 0
#define SDP_SUSPEND_UA 2500
#define SDP_HS_CONN_UA 100000
#define SDP_HS_CONFIG_UA 500000
#define SDP_SS_CONN_UA 150000
#define SDP_SS_CONFIG_UA 900000
#define DEAD_BATTERY_UA 500000

/* Defer vote for pSnkStby */
#define BC_VOTE_DELAY_MS	3000

/* At least get one more try to meet sub state sync requirement */
#define ERR_RETRY_DELAY_MS 20
#define ERR_RETRY_COUNT    3

/* Fall back to DCP when neither SDP_*_CONN_UA nor SDP_*_CONFIG_UA is signalled */
#define SDP_ENUMERATION_TIMEOUT_MS	60000

/* Signal wakeup event to allow userspace to process the update */
#define DCP_UPDATE_MS			10000

struct usb_psy_data {
	struct logbuffer *log;

	struct i2c_client *tcpc_client;
	struct power_supply *usb_psy;
	struct power_supply *chg_psy;
	struct power_supply *main_chg_psy;
	enum power_supply_usb_type usb_type;

	/* Casts final vote on usb psy current max */
	struct gvotable_election *usb_icl_el;
	/* Combines the values from thermald and protocol stack */
	struct gvotable_election *usb_icl_combined_el;
	/*
	 * Combines the values from various voters of the protocol stack
	 * such as PD, BC1.2, TYPE, DATA stack.
	 */
	struct gvotable_election *usb_icl_proto_el;
	/* Casts 1/0 on Dead Battery condition */
	struct gvotable_election *dead_battery_el;

	/* Cached/Request usb ilim to charger psy */
	int current_max_cache;
	/* Current limits requested from USB data stack */
	int sdp_input_current_limit;

	struct usb_psy_ops *psy_ops;

	/* For voting current limit */
	char *chg_psy_name;
	/* For reading USB current now */
	char *main_chg_psy_name;

	/*
	 * Setting CURRENT limit on charger side can fail.
	 * Implement retry mechanism.
	 * Needs to be at RT priority to confirm to Type-C timing
	 * constraints.
	 */
	struct kthread_worker *wq;
	struct kthread_delayed_work icl_work;
	struct kthread_delayed_work bc_icl_work;
	struct kthread_delayed_work sdp_icl_work;
	atomic_t retry_count;

	/* Wq for scheduling work items setting POWER_SUPPLY_PROP_USB_TYPE */
	struct kthread_worker *usb_type_wq;
	/* Reset usb_type upon disconnect */
	struct kthread_work bc_reset_work;

	/* sink connected state from Type-C */
	bool sink_enabled;

	/* Tracks attached state from Type-C */
	bool attached;

	bool usb_configured;

	/* Alarm for forcing to DCP port on enumeration timeout */
	struct alarm sdp_timeout_alarm;
	/* Bottom half for sdp alarm */
	struct kthread_work sdp_timeout_work;
};

void init_vote(struct usb_vote *vote, const char *reason,
	       unsigned int priority, unsigned int val)
{
	strncpy(vote->reason, reason, sizeof(vote->reason));
	vote->priority = priority;
	vote->val = val;
}
EXPORT_SYMBOL_GPL(init_vote);

static int usb_get_current_max_ma(struct usb_psy_data *usb)
{
	union power_supply_propval val = {0};
	int ret;
	struct i2c_client *client = usb->tcpc_client;

	if (!usb->chg_psy_name)
		return usb->current_max_cache;

	if (!usb->chg_psy) {
		usb->chg_psy = power_supply_get_by_name(usb->chg_psy_name);
		if (IS_ERR_OR_NULL(usb->chg_psy)) {
			dev_err(&client->dev, "chg psy not up\n");
			return usb->current_max_cache;
		}
	}

	ret = power_supply_get_property(usb->chg_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	if (ret < 0)
		return ret;

	return val.intval;
}

static int usb_set_current_max_ma(struct usb_psy_data *usb,
				  int current_max)
{
	union power_supply_propval val = {0};
	int ret;
	struct i2c_client *client = usb->tcpc_client;

	if (!usb->chg_psy) {
		if (!usb->chg_psy_name) {
			dev_err(&client->dev, "chg_psy_name not found\n");
			return 0;
		}
		usb->chg_psy = power_supply_get_by_name(usb->chg_psy_name);
		if (IS_ERR_OR_NULL(usb->chg_psy)) {
			dev_err(&client->dev, "chg psy not up\n");
			return 0;
		}
	}

	val.intval = current_max;
	ret = power_supply_set_property(usb->chg_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX,
					&val);

	logbuffer_log(usb->log, "set input max current %d to %s, ret=%d",
		      current_max, usb->chg_psy_name, ret);
	return ret;
}

static void disable_proto_votes(struct gvotable_election *el, struct logbuffer *log,
				unsigned int map)
{
	struct usb_vote vote;
	int i;

	for (i = 0; i < sizeof(unsigned int) * CHAR_BIT; i++) {
		if (map & (0x1 << i)) {
			init_vote(&vote, proto_voter_reason[i], i, 0);
			gvotable_cast_vote(el, vote.reason, &vote, false);
		}
	}
}

static void set_sdp_current_limit(struct usb_psy_data *usb, int ua)
{
	struct gvotable_election *usb_icl_proto_el = usb->usb_icl_proto_el;
	struct logbuffer *log = usb->log;
	unsigned int voter_map = 0;
	struct usb_vote vote;
	bool configured;
	int ret;

	if (ua < 0)
		return;

	switch (ua) {
	case SDP_SUSPEND_UA:
		/* no need to change the usb_configured flag when entering SUSPEND */
		configured = usb->usb_configured;
		if (usb->usb_configured) {
			voter_map = BIT(USB_CONFIGURED);
			init_vote(&vote, proto_voter_reason[USB_SUSPEND], USB_SUSPEND, ua);
		} else {
			voter_map = BIT(USB_CONFIGURED) | BIT(USB_SUSPEND);
			init_vote(&vote, proto_voter_reason[USB_SUSPEND_UNCFG],
				  USB_SUSPEND_UNCFG, ua);
		}
		break;
	case SDP_DISCONNECT:
		configured = false;
		voter_map = BIT(USB_SUSPEND_UNCFG) | BIT(USB_CONFIGURED) | BIT(USB_SUSPEND);
		/* Set to SDP_HS_CONN_UA for USB Gadget Reset */
		init_vote(&vote, proto_voter_reason[BC12_SDP], BC12_SDP, SDP_HS_CONN_UA);
		break;
	case SDP_HS_CONN_UA:
	case SDP_SS_CONN_UA:
		alarm_cancel(&usb->sdp_timeout_alarm);
		configured = false;
		voter_map = BIT(USB_SUSPEND_UNCFG) | BIT(USB_CONFIGURED) | BIT(USB_SUSPEND);
		init_vote(&vote, proto_voter_reason[BC12_SDP], BC12_SDP, ua);
		break;
	case SDP_HS_CONFIG_UA:
	case SDP_SS_CONFIG_UA:
		alarm_cancel(&usb->sdp_timeout_alarm);
		configured = true;
		voter_map = BIT(USB_SUSPEND_UNCFG) | BIT(USB_SUSPEND);
		init_vote(&vote, proto_voter_reason[USB_CONFIGURED], USB_CONFIGURED, ua);
		break;
	default:
		logbuffer_log(log, "%s: current %d uA not supported", __func__, ua);
		return;
	}

	ret = gvotable_cast_vote(usb_icl_proto_el, vote.reason, &vote, true);
	logbuffer_log(log, "%s: %s voting usb proto_el: %d by %s",
		      __func__, ret < 0 ? "error" : "",  vote.val,
		      vote.reason);

	/* vote 0 to disable DEAD_BATTERY condition while entering CONFIGURED State */
	if (configured && !usb->usb_configured) {
		ret = gvotable_cast_vote(usb->dead_battery_el, USB_CFG_VOTER, 0, true);
		logbuffer_log(log, "%s: %svoting dead_battery_el: %u by %s", __func__,
			      ret < 0 ? "error " : "", 0, USB_CFG_VOTER);
	}
	usb->usb_configured = configured;

	if (voter_map)
		disable_proto_votes(usb_icl_proto_el, log, voter_map);
}

static void set_bc_current_limit(struct gvotable_election *usb_icl_proto_el,
				 enum power_supply_usb_type usb_type,
				 struct logbuffer *log)
{
	unsigned int voter_map = 0;
	struct usb_vote vote;
	int ret;

	switch (usb_type) {
	case POWER_SUPPLY_USB_TYPE_CDP:
	case POWER_SUPPLY_USB_TYPE_DCP:
		init_vote(&vote, proto_voter_reason[BC12_CDP_DCP],
			  BC12_CDP_DCP, CDP_DCP_ICL_UA);
		break;
	case POWER_SUPPLY_USB_TYPE_SDP:
		init_vote(&vote, proto_voter_reason[BC12_SDP],
			  BC12_SDP, SDP_HS_CONN_UA);
		break;
	default:
		voter_map = BIT(BC12_SDP) | BIT(USB_SUSPEND_UNCFG) | BIT(USB_CONFIGURED) |
			    BIT(USB_SUSPEND) | BIT(BC12_CDP_DCP);
		break;
	}

	/** Disable all votes for unknown type **/
	if (voter_map) {
		disable_proto_votes(usb_icl_proto_el, log, voter_map);
		return;
	}

	ret = gvotable_cast_vote(usb_icl_proto_el, vote.reason, &vote,
				 true);
	logbuffer_log(log, "%s: %s voting usb proto_el: %d by %s",
		      __func__, ret < 0 ? "error" : "",  vote.val,
		      vote.reason);
}

static int usb_psy_current_now_ma(struct usb_psy_data *usb, int *current_now)
{
	struct power_supply *psy = NULL;
	union power_supply_propval val;
	int ret;

	if (IS_ERR_OR_NULL(usb->main_chg_psy) && usb->main_chg_psy_name) {
		usb->main_chg_psy = power_supply_get_by_name(usb->main_chg_psy_name);
		if (IS_ERR_OR_NULL(usb->main_chg_psy))
			return -EAGAIN;
	}

	if (!IS_ERR_OR_NULL(usb->main_chg_psy))
		psy = usb->main_chg_psy;
	else if (!IS_ERR_OR_NULL(usb->chg_psy))
		psy = usb->chg_psy;

	if (!psy) {
		/* NOTE: you might want to log only once */
		logbuffer_log(usb->log, "Neither chg_psy nor main_chg_psy found for reading current_now");
		return -EINVAL;
	}

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		*current_now = val.intval;

	return ret;
}

static int usb_psy_data_get_prop(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct usb_psy_data *usb = power_supply_get_drvdata(psy);
	struct usb_psy_ops *ops = usb->psy_ops;
	struct i2c_client *client = usb->tcpc_client;
	int ret;

	if (!usb)
		return 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = usb->sink_enabled ? usb_get_current_max_ma(usb) > ONLINE_THRESHOLD_UA
			? 1 : 0 : 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = usb->sink_enabled;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		/* Report the voted value to reflect TA capability */
		val->intval = usb->current_max_cache;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		/* Report in uv */
		val->intval = ops->tcpc_get_vbus_voltage_max_mv(client) * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = usb_psy_current_now_ma(usb, &val->intval);
		if (ret < 0)
			return ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = ops->tcpc_get_vbus_voltage_mv(client) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = usb->sdp_input_current_limit;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = usb->usb_type;
		break;
	default:
		break;
	}

	return 0;
}

static int usb_psy_data_set_prop(struct power_supply *psy,
				 enum power_supply_property psp,
				 const union power_supply_propval *val)
{
	struct usb_psy_data *usb = power_supply_get_drvdata(psy);
	struct usb_psy_ops *ops = usb->psy_ops;
	struct i2c_client *client = usb->tcpc_client;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		usb->current_max_cache = val->intval;
		atomic_set(&usb->retry_count, ERR_RETRY_COUNT);
		kthread_mod_delayed_work(usb->wq, &usb->icl_work, 0);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		/* Enough to trigger just the uevent */
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		logbuffer_log(usb->log, "POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT lim:%d type:%d",
			      val->intval, usb->usb_type);
		/* Handle SDP current only */
		if (usb->usb_type != POWER_SUPPLY_USB_TYPE_SDP)
			return 0;
		usb->sdp_input_current_limit = val->intval;
		kthread_mod_delayed_work(usb->wq, &usb->sdp_icl_work, 0);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		usb->usb_type = val->intval;
		usb->usb_configured = false;
		ops->tcpc_set_port_data_capable(client, usb->usb_type);

		kthread_mod_delayed_work(usb->wq, &usb->bc_icl_work,
					 usb->usb_type != POWER_SUPPLY_USB_TYPE_UNKNOWN ?
					 msecs_to_jiffies(BC_VOTE_DELAY_MS) : 0);
		break;
	default:
		break;
	}
	power_supply_changed(usb->usb_psy);

	return 0;
}

void usb_psy_start_sdp_timeout(void *usb_psy)
{
	struct usb_psy_data *usb = usb_psy;

	if (!usb)
		return;

	if (usb->usb_type == POWER_SUPPLY_USB_TYPE_SDP)
		alarm_start_relative(&usb->sdp_timeout_alarm,
				     ms_to_ktime(SDP_ENUMERATION_TIMEOUT_MS));
}
EXPORT_SYMBOL_GPL(usb_psy_start_sdp_timeout);

void usb_psy_set_sink_state(void *usb_psy, bool enabled)
{
	struct usb_psy_data *usb = usb_psy;

	if (!usb || !usb->usb_psy)
		return;

	usb->sink_enabled = enabled;
	power_supply_changed(usb->usb_psy);
}
EXPORT_SYMBOL_GPL(usb_psy_set_sink_state);

static void bc_reset_work_item(struct kthread_work *work)
{
	struct usb_psy_data *usb = container_of(work, struct usb_psy_data, bc_reset_work);
	union power_supply_propval val = {.intval = POWER_SUPPLY_USB_TYPE_UNKNOWN};
	int ret;

	ret = power_supply_set_property(usb->usb_psy, POWER_SUPPLY_PROP_USB_TYPE, &val);
	logbuffer_log(usb->log, "%s: Resetting usb_type %s:%d", __func__, ret < 0 ? "error" :
		      "success", ret);
}

void usb_psy_set_attached_state(void *usb_psy, bool attached)
{
	struct usb_psy_data *usb = usb_psy;

	if (!usb || !usb->usb_psy)
		return;

	/* Reset upon disconnect as BC1.2 gets disabled before enabling data */
	if (usb->attached & !attached) {
		kthread_queue_work(usb->usb_type_wq, &usb->bc_reset_work);
		kthread_flush_work(&usb->bc_reset_work);

		/* Cancel sdp alarm if scheduled */
		alarm_cancel(&usb->sdp_timeout_alarm);
	}

	usb->attached = attached;
	power_supply_changed(usb->usb_psy);
}
EXPORT_SYMBOL_GPL(usb_psy_set_attached_state);

//todo: Expose settled ICL limit
static enum power_supply_property usb_psy_data_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
};

static enum power_supply_usb_type usb_psy_data_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_DCP,
};

static const struct power_supply_desc usb_psy_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = usb_psy_data_types,
	.num_usb_types = ARRAY_SIZE(usb_psy_data_types),
	.properties = usb_psy_data_props,
	.num_properties = ARRAY_SIZE(usb_psy_data_props),
	.get_property = usb_psy_data_get_prop,
	.set_property = usb_psy_data_set_prop,
};

static int vote_comp(struct usb_vote *v1, struct usb_vote *v2)
{
	int v1_pri = v1->priority, v2_pri = v2->priority;

	/** Separate WARN_ON's to triage failures **/
	WARN_ON(!v1);
	WARN_ON(!v2);

	if (v1_pri == v2_pri)
		return v2->val - v1->val;

	return v2_pri - v1_pri;
}

static void usb_icl_callback(struct gvotable_election *el,
			     const char *reason, void *result)
{
	struct usb_psy_data *usb = gvotable_get_data(el);
	union power_supply_propval val = {.strval = result};
	int ret;

	ret = power_supply_set_property(usb->usb_psy,
					POWER_SUPPLY_PROP_CURRENT_MAX, &val);
	logbuffer_log(usb->log,
		      "%s: %s:%d setting PROP_CURRRENT_MAX: %d by %s",
		      __func__, ret < 0 ? "error" : "success", ret,
		      val.intval, reason);
}

static void usb_icl_combined_callback(struct gvotable_election *el,
				      const char *reason, void *result)
{
	struct usb_psy_data *usb = gvotable_get_data(el);
	struct usb_vote *vote_result = result;
	int ret;

	ret = gvotable_cast_vote(usb->usb_icl_el,
				 icl_voter_reason[USB_ICL_COMB],
				 (void *)(long)vote_result->val, true);
	logbuffer_log(usb->log, "%s: %s:%d voting usb_icl_el: %d by %s",
		      __func__, ret < 0 ? "error" : "success", ret,
		      vote_result->val, icl_voter_reason[USB_ICL_COMB]);
}

/*
 * MIN between USB_ICL_THERMAL_VOTER and USB_ICL_PROTO_VOTER
 */
static int usb_icl_combined_comp(void *vote1, void *vote2)
{
	struct usb_vote *vote1_result = vote1, *vote2_result = vote2;

	if (vote1_result->val < vote2_result->val)
		return 1;
	else if (vote1_result->val > vote2_result->val)
		return -1;
	else
		return 0;
}

static void usb_icl_proto_callback(struct gvotable_election *el,
				   const char *reason, void *result)
{
	struct usb_psy_data *usb = gvotable_get_data(el);
	struct usb_vote *vote_result = result;
	struct usb_vote vote = {
		.reason = USB_ICL_PROTO_VOTER,
		.val = 0,
	};
	int ret;

	/*
	 * All the votes have been disabled when the result is null.
	 * Vote for 0 in that case.
	 * If not vote the value from the result.
	 */
	if (result)
		vote.val = vote_result->val;

	ret = gvotable_cast_vote(usb->usb_icl_combined_el,
				 USB_ICL_PROTO_VOTER, &vote, true);
	logbuffer_log(usb->log,
		      "%s: %s:%d voting usb_icl_combined_el: %d by %s",
		      __func__, ret < 0 ? "error" : "success",
		      ret, vote.val, vote.reason);
}

/*
 * Priority based voting. USB_ICL_DATA_SUSPEND has the highest priority.
 *
 */
static inline int usb_icl_proto_comp(void *vote1, void *vote2)
{
	return vote_comp(vote1, vote2);
}

/*
 * MIN vote:
 * result 0 means to clear DEAD_BATTERY vote to usb_icl_proto_el
 * result 1 means to vote DEAD_BATTERY_UA to usb_icl_proto_el
 */
static void dead_battery_callback(struct gvotable_election *el, const char *reason, void *result)
{
	struct usb_psy_data *usb = gvotable_get_data(el);
	unsigned long vote_result = (unsigned long)result;
	struct usb_vote vote;
	int ret;

	if (vote_result) {
		init_vote(&vote, proto_voter_reason[DEAD_BATTERY], DEAD_BATTERY, DEAD_BATTERY_UA);
		ret = gvotable_cast_vote(usb->usb_icl_proto_el, vote.reason, &vote, true);
	} else {
		init_vote(&vote, proto_voter_reason[DEAD_BATTERY], DEAD_BATTERY, 0);
		ret = gvotable_cast_vote(usb->usb_icl_proto_el, vote.reason, &vote, false);
	}

	logbuffer_log(usb->log, "%s: %s:%d %s usb_icl_proto_el: %lu by %s",
		      __func__, ret < 0 ? "error" : "success", ret,
		      vote_result ? "voting" : "clearing", vote.val,
		      proto_voter_reason[DEAD_BATTERY]);
}

static int debug_print_vote(char *str,  size_t len, const void *vote)
{
	struct usb_vote *usb_vote = (struct usb_vote *)vote;

	if (!vote)
		return 0;

	return scnprintf(str, len, "val:%u priority:%u", (unsigned int)
			 usb_vote->val, (unsigned int)usb_vote->priority);
}

static void icl_work_item(struct kthread_work *work)
{
	struct usb_psy_data *usb =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct usb_psy_data, icl_work);

	if (usb_set_current_max_ma(usb, usb->current_max_cache) < 0 &&
	    !atomic_sub_and_test(1, &usb->retry_count))
		kthread_mod_delayed_work(usb->wq, &usb->icl_work,
					 msecs_to_jiffies(ERR_RETRY_DELAY_MS));
}

static void bc_icl_work_item(struct kthread_work *work)
{
	struct usb_psy_data *usb =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct usb_psy_data, bc_icl_work);

	set_bc_current_limit(usb->usb_icl_proto_el, usb->usb_type, usb->log);
}

static void sdp_icl_work_item(struct kthread_work *work)
{
	struct kthread_delayed_work *sdp_work = container_of(work, struct kthread_delayed_work,
							     work);
	struct usb_psy_data *usb = container_of(sdp_work, struct usb_psy_data, sdp_icl_work);

	set_sdp_current_limit(usb, usb->sdp_input_current_limit);
}

static void sdp_timeout_work_item(struct kthread_work *work)
{
	struct usb_psy_data *usb = container_of(work, struct usb_psy_data, sdp_timeout_work);
	union power_supply_propval val = {.intval = POWER_SUPPLY_USB_TYPE_DCP};
	int ret;

	ret = power_supply_set_property(usb->usb_psy, POWER_SUPPLY_PROP_USB_TYPE, &val);
	logbuffer_log(usb->log, "%s: Falling back to DCP %s:%d", __func__, ret < 0 ? "error" :
		      "success", ret);
}

static enum alarmtimer_restart sdp_timeout_alarm_cb(struct alarm *alarm, ktime_t now)
{
	struct usb_psy_data *usb = container_of(alarm, struct usb_psy_data, sdp_timeout_alarm);

	pm_wakeup_event(&usb->tcpc_client->dev, DCP_UPDATE_MS);
	kthread_queue_work(usb->usb_type_wq, &usb->sdp_timeout_work);

	return ALARMTIMER_NORESTART;
}

void *usb_psy_setup(struct i2c_client *client, struct logbuffer *log,
		    struct usb_psy_ops *ops)
{
	struct usb_psy_data *usb;
	struct power_supply_config usb_cfg = {};
	struct device *dev = &client->dev;
	struct device_node *dn;
	void *ret;

	if (!ops)
		return ERR_PTR(-EINVAL);

	if (!ops->tcpc_get_vbus_voltage_max_mv ||
	    !ops->tcpc_set_vbus_voltage_max_mv ||
	    !ops->tcpc_get_vbus_voltage_mv ||
	    !ops->tcpc_set_port_data_capable)
		return ERR_PTR(-EINVAL);

	usb = devm_kzalloc(dev, sizeof(*usb), GFP_KERNEL);
	if (!usb)
		return ERR_PTR(-ENOMEM);

	usb->tcpc_client = client;
	usb->log = log;
	usb->psy_ops = ops;

	usb->usb_type_wq = kthread_create_worker(0, "wq-tcpm-usb-psy-usb-type");
	if (IS_ERR_OR_NULL(usb->usb_type_wq)) {
		dev_err(&client->dev, "wq-tcpm-usb-psy-usb-type failed to create\n");
		return usb->usb_type_wq;
	}

	alarm_init(&usb->sdp_timeout_alarm, ALARM_BOOTTIME, sdp_timeout_alarm_cb);
	kthread_init_work(&usb->sdp_timeout_work, sdp_timeout_work_item);
	kthread_init_work(&usb->bc_reset_work, bc_reset_work_item);

	dn = dev_of_node(dev);
	if (!dn) {
		dev_err(&client->dev, "of node not found\n");
		ret = ERR_PTR(-EINVAL);
		goto unreg_usb_type_wq;
	}

	usb->chg_psy_name = (char *)of_get_property(dn, "chg-psy-name", NULL);
	if (!usb->chg_psy_name) {
		dev_err(&client->dev, "chg-psy-name not set\n");
	} else {
		usb->chg_psy = power_supply_get_by_name(usb->chg_psy_name);
		if (IS_ERR_OR_NULL(usb->chg_psy))
			dev_err(&client->dev, "chg psy not up\n");
	}

	usb->main_chg_psy_name = (char *)of_get_property(dn, "main-chg-psy-name", NULL);

	usb_cfg.drv_data = usb;
	usb_cfg.of_node =  dev->of_node;
	usb->usb_psy = power_supply_register(dev, &usb_psy_desc, &usb_cfg);
	if (IS_ERR(usb->usb_psy)) {
		dev_err(dev, "usb: Power supply register failed");
		ret = usb->usb_psy;
		goto chg_psy_put;
	}
	usb->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

	/*
	 * PRIORITY VOTE : will have two voters.
	 * 1. USER
	 * 2. COMBINED
	 */
	usb->usb_icl_el =
		gvotable_create_int_election(USB_ICL_EL,
					     gvotable_comparator_int_min,
					     usb_icl_callback, usb);
	if (IS_ERR_OR_NULL(usb->usb_icl_el)) {
		ret = usb->usb_icl_el;
		goto usb_psy_unreg;
	}
	gvotable_set_vote2str(usb->usb_icl_el, gvotable_v2s_uint);

	/*
	 * MIN VOTE: will have two voters
	 * 1. Thermal
	 * 2. PROTOCOL vote
	 */
	usb->usb_icl_combined_el =
		gvotable_create_election(USB_ICL_COMBINED_EL,
					 sizeof(struct usb_vote),
					 usb_icl_combined_comp,
					 usb_icl_combined_callback, usb);
	if (IS_ERR_OR_NULL(usb->usb_icl_combined_el)) {
		ret = usb->usb_icl_combined_el;
		goto unreg_icl_el;
	}
	gvotable_set_vote2str(usb->usb_icl_combined_el, debug_print_vote);

	/*
	 * PRIORITY VOTE: Various players of the protocol stack such as
	 * PD, BC12, TYPEC, DEAD_BATTERY etc.
	 */
	usb->usb_icl_proto_el =
		gvotable_create_election(USB_ICL_PROTO_EL,
					 sizeof(struct usb_vote),
					 usb_icl_proto_comp,
					 usb_icl_proto_callback, usb);
	if (IS_ERR_OR_NULL(usb->usb_icl_proto_el)) {
		ret = usb->usb_icl_proto_el;
		goto unreg_icl_combined_el;
	}
	gvotable_set_vote2str(usb->usb_icl_proto_el, debug_print_vote);

	/*
	 * MIN VOTE: will have two voters
	 * 1. MSC_BATT
	 * 2. USB_CFG_VOTER
	 *
	 * Vote 1 to enable Dead Battery condition
	 * Vote 0 to disable Dead Battery condition
	 * Default: 1
	 */
	usb->dead_battery_el = gvotable_create_int_election(DEAD_BATTERY_EL,
							    gvotable_comparator_int_min,
							    dead_battery_callback, usb);
	if (IS_ERR_OR_NULL(usb->dead_battery_el)) {
		ret = usb->dead_battery_el;
		goto unreg_icl_proto_el;
	}
	gvotable_set_vote2str(usb->dead_battery_el, gvotable_v2s_uint);
	gvotable_set_default(usb->dead_battery_el, (void *)1);

	usb->wq = kthread_create_worker(0, "wq-tcpm-usb-psy");
	if (IS_ERR_OR_NULL(usb->wq)) {
		ret = usb->wq;
		goto unreg_dead_battery_el;
	}

	kthread_init_delayed_work(&usb->icl_work, icl_work_item);
	kthread_init_delayed_work(&usb->bc_icl_work, bc_icl_work_item);
	kthread_init_delayed_work(&usb->sdp_icl_work, sdp_icl_work_item);

	/* vote default value after icl_work is initialized */
	gvotable_use_default(usb->dead_battery_el, true);

	return usb;

unreg_dead_battery_el:
	gvotable_destroy_election(usb->dead_battery_el);
unreg_icl_proto_el:
	gvotable_destroy_election(usb->usb_icl_proto_el);
unreg_icl_combined_el:
	gvotable_destroy_election(usb->usb_icl_combined_el);
unreg_icl_el:
	gvotable_destroy_election(usb->usb_icl_el);
usb_psy_unreg:
	if (!IS_ERR_OR_NULL(usb->main_chg_psy))
		power_supply_put(usb->main_chg_psy);
	if (!IS_ERR_OR_NULL(usb->usb_psy))
		power_supply_unregister(usb->usb_psy);
chg_psy_put:
	if (!IS_ERR_OR_NULL(usb->chg_psy))
		power_supply_put(usb->chg_psy);
unreg_usb_type_wq:
	kthread_destroy_worker(usb->usb_type_wq);
	return ret;
}
EXPORT_SYMBOL_GPL(usb_psy_setup);

void usb_psy_teardown(void *usb_data)
{
	struct usb_psy_data *usb = (struct usb_psy_data *)usb_data;

	kthread_destroy_worker(usb->wq);
	gvotable_destroy_election(usb->dead_battery_el);
	gvotable_destroy_election(usb->usb_icl_proto_el);
	gvotable_destroy_election(usb->usb_icl_combined_el);
	gvotable_destroy_election(usb->usb_icl_el);
	if (!IS_ERR_OR_NULL(usb->chg_psy))
		power_supply_put(usb->chg_psy);
	if (!IS_ERR_OR_NULL(usb->main_chg_psy))
		power_supply_put(usb->main_chg_psy);
	if (!IS_ERR_OR_NULL(usb->usb_psy))
		power_supply_unregister(usb->usb_psy);
	kthread_destroy_worker(usb->usb_type_wq);
}
EXPORT_SYMBOL_GPL(usb_psy_teardown);

MODULE_DESCRIPTION("USB_PSY Module");
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_LICENSE("GPL");
