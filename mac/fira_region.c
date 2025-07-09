/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2022 Qorvo US, Inc.
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

#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/errno.h>

#include <net/mcps802154_schedule.h>
#include <net/fira_region_nl.h>

#include "fira_region.h"
#include "fira_region_call.h"
#include "fira_access.h"
#include "fira_session.h"
#include "fira_crypto.h"
#include "warn_return.h"

static struct mcps802154_region_ops fira_region_ops;
static bool do_crypto_selftest_on_module_init;

static void fira_report_event(struct work_struct *work)
{
	struct fira_local *local =
		container_of(work, struct fira_local, report_work);
	struct sk_buff *skb;
	int r;

	while (!skb_queue_empty(&local->report_queue)) {
		skb = skb_dequeue(&local->report_queue);
		r = mcps802154_region_event(local->llhw, skb);
		if (r == -ECONNREFUSED)
			/* TODO stop. */
			;
	}
}

static struct mcps802154_region *fira_open(struct mcps802154_llhw *llhw)
{
	struct fira_local *local;

	local = kzalloc(sizeof(*local), GFP_KERNEL);
	if (!local)
		return NULL;
	local->llhw = llhw;
	local->region.ops = &fira_region_ops;
	INIT_LIST_HEAD(&local->inactive_sessions);
	INIT_LIST_HEAD(&local->active_sessions);
	skb_queue_head_init(&local->report_queue);
	INIT_WORK(&local->report_work, fira_report_event);
	/* FIXME: Hack to simplify unit test, which is borderline. */
	local->block_duration_rx_margin_ppm = UWB_BLOCK_DURATION_MARGIN_PPM;
	fira_crypto_init(NULL);
	return &local->region;
}

static void fira_close(struct mcps802154_region *region)
{
	struct fira_local *local = region_to_local(region);
	struct fira_session *session, *s;

	list_for_each_entry_safe (session, s, &local->inactive_sessions,
				  entry) {
		fira_session_free(local, session);
	}

	cancel_work_sync(&local->report_work);
	skb_queue_purge(&local->report_queue);
	kfree_sensitive(local);
}

static void fira_notify_stop(struct mcps802154_region *region)
{
	struct fira_local *local = region_to_local(region);
	struct fira_session *session, *s;

	list_for_each_entry_safe (session, s, &local->active_sessions, entry) {
		fira_session_stop_controlees(session);
		fira_session_fsm_stop(local, session);
	}
}

static int fira_call(struct mcps802154_region *region, u32 call_id,
		     const struct nlattr *attrs, const struct genl_info *info)
{
	struct fira_local *local = region_to_local(region);

	switch (call_id) {
	case FIRA_CALL_GET_CAPABILITIES:
		return fira_get_capabilities(local, info);
	default:
		return fira_session_control(local, call_id, attrs, info);
	}
}

/**
 * fira_session_init_block_start_dtu() - Build the first block start dtu.
 * @local: FiRa context.
 * @session: Session context.
 * @timestamp_dtu: First access opportunity.
 */
static void fira_session_init_block_start_dtu(struct fira_local *local,
					      struct fira_session *session,
					      u32 timestamp_dtu)
{
	if (!session->block_start_valid) {
		const struct fira_session_params *params = &session->params;
		int dtu_freq_khz = local->llhw->dtu_freq_hz / 1000;

		session->block_start_valid = true;
		session->block_start_dtu =
			timestamp_dtu +
			params->initiation_time_ms * dtu_freq_khz;
		session->next_access_timestamp_dtu = session->block_start_dtu;
	}
}

/**
 * fira_get_next_session() - Find the next session which should have the
 * access.
 * @local: FiRa context.
 * @next_timestamp_dtu: Next access opportunity.
 * @max_duration_dtu: Max duration of the next access opportunity.
 * @adopted_demand: Output updated related to next session returned.
 *
 * Return: Pointer to the next session which should have the access.
 */
static struct fira_session *
fira_get_next_session(struct fira_local *local, u32 next_timestamp_dtu,
		      int max_duration_dtu,
		      struct fira_session_demand *adopted_demand)
{
	struct fira_session *adopted_session = NULL;
	struct fira_session *session;
	bool is_candidate_unsync, is_adopted_unsync;
	int max_unsync_duration_dtu = max_duration_dtu;
	int r;

	/*
	 * Little cheat to start all sessions as initiation_time_ms is a
	 * delay, and not a absolute time. This is the only function
	 * which change the session content.
	 */
	list_for_each_entry (session, &local->active_sessions, entry) {
		fira_session_init_block_start_dtu(local, session,
						  next_timestamp_dtu);
	}

	/*
	 * Reminder: active_sessions list is sorted by session->priority
	 * from highest priority to lowest priority.
	 */
	list_for_each_entry (session, &local->active_sessions, entry) {
		/*
		 * Welcome in sessions election!
		 *
		 * First, the candidate session will provide its wish in
		 * 'candidate_demand'.
		 * And then the candidate will be compared with the adopted
		 * session. The "best" will be become or stay the adopted
		 * session.
		 * So the session election will process candidate after
		 * candidate, to find the most appropriate session.
		 */
		struct fira_session_demand candidate_demand;

		/*
		 * Sessions with lower priority are not allowed to overlap
		 * the adopted session. But a lower priority can start and
		 * stop before the session adopted.
		 */
		if (adopted_session && adopted_session->params.priority !=
					       session->params.priority) {
			/* Is there some time left? */
			if (is_before_dtu(next_timestamp_dtu,
					  adopted_demand->timestamp_dtu))
				/*
				 * Limit max duration for session with lower
				 * priority to not overlap sessions which have
				 * an higher priority.
				 */
				max_duration_dtu =
					adopted_demand->timestamp_dtu -
					next_timestamp_dtu;
			else
				/* No more time left. */
				break;
			if (!max_unsync_duration_dtu ||
			    max_unsync_duration_dtu > max_duration_dtu)
				max_unsync_duration_dtu = max_duration_dtu;
		}

		is_candidate_unsync = session->params.device_type ==
					      FIRA_DEVICE_TYPE_CONTROLEE &&
				      !session->controlee.synchronised;
		/* Retrieve the wish of the session candidate. */
		r = fira_session_fsm_get_demand(
			local, session, next_timestamp_dtu,
			is_candidate_unsync ? max_unsync_duration_dtu :
					      max_duration_dtu,
			&candidate_demand);
		/* When 'r' is one, the session have a demand. */
		if (r != 1)
			/* The session doesn't have a demand. */
			continue;

		/*
		 * If there is no adopted session, the candidate is the
		 * adopted session.
		 */
		if (!adopted_session)
			goto candidate_adopted;
		/*
		 * Is session finish before the adopted session ?
		 * adopted_demand |              [-----]
		 * candidate      |   [------]
		 *              --+-----------------------> Time
		 */
		if (is_before_dtu(candidate_demand.timestamp_dtu +
					  candidate_demand.max_duration_dtu,
				  adopted_demand->timestamp_dtu))
			/*
			 * Candidate is adopted and replace the
			 * previous one.
			 */
			goto candidate_adopted;
		/*
		 * Is session start after the adopted session ?
		 * adopted_demand |   [------]
		 * candidate      |             [--------]
		 *              --+----------------------> Time
		 */
		if (is_before_dtu(adopted_demand->timestamp_dtu +
					  adopted_demand->max_duration_dtu,
				  candidate_demand.timestamp_dtu))
			/* Candidate is not adopted. */
			continue;
		/*
		 * The candidate session have an overlap with the adopted
		 * session. Try the negotiation first to find an agreement
		 * about the access usage.
		 *
		 * But take care, synchronized session have a better
		 * eloquence in case of negotiation failure with an
		 * unsynchronized session.
		 */
		is_adopted_unsync = adopted_session->params.device_type ==
					    FIRA_DEVICE_TYPE_CONTROLEE &&
				    !adopted_session->controlee.synchronised;
		/*
		 * The candidate session have an overlap with the adopted
		 * session.
		 *
		 * adopted_demand |   [------]
		 * candidate      |       [--------]
		 *              --+----------------------> Time
		 *
		 * Request a duration reduction to the adopted session.
		 */
		if (is_adopted_unsync &&
		    !is_before_dtu(candidate_demand.timestamp_dtu,
				   adopted_demand->timestamp_dtu)) {
			int limit_duration_dtu =
				candidate_demand.timestamp_dtu -
				adopted_demand->timestamp_dtu;
			struct fira_session_demand tmp;

			if (limit_duration_dtu)
				/* Ask to reduce the duration. */
				r = fira_session_fsm_get_demand(
					local, adopted_session,
					next_timestamp_dtu, limit_duration_dtu,
					&tmp);
			else
				/* Both sessions start at same time. */
				r = 0;
			if (r == 1) {
				/*
				 * The adopted session accept to
				 * reduction its max duration.
				 */
				*adopted_demand = tmp;
				max_unsync_duration_dtu = limit_duration_dtu;
				continue;
			}
			if (!is_candidate_unsync)
				/*
				 * In this corrupted world, synchronized
				 * session have better relation.
				 */
				goto candidate_adopted;
		}
		/*
		 * The candidate session have an overlap with the adopted
		 * session.
		 *
		 * adopted_demand |       [-----]
		 * candidate      |  [------]
		 *              --+----------------------> Time
		 *
		 * Request a duration reduction to the candidate session.
		 */
		if (is_candidate_unsync &&
		    !is_before_dtu(adopted_demand->timestamp_dtu,
				   candidate_demand.timestamp_dtu)) {
			int limit_duration_dtu = adopted_demand->timestamp_dtu -
						 candidate_demand.timestamp_dtu;
			struct fira_session_demand tmp;

			if (limit_duration_dtu)
				/* Ask to reduce the duration. */
				r = fira_session_fsm_get_demand(
					local, session, next_timestamp_dtu,
					limit_duration_dtu, &tmp);
			else
				/* Both sessions start at same time. */
				r = 0;
			if (r == 1) {
				/*
				 * The candidate session accept to
				 * reduction its max duration.
				 */
				adopted_session = session;
				*adopted_demand = tmp;
				max_unsync_duration_dtu = limit_duration_dtu;
				continue;
			}
			if (!is_adopted_unsync)
				/*
				 * In this corrupted world, synchronized
				 * session have better relation.
				 */
				continue;
		}

		/*
		 * Finally, negotiation between adopted and candidate fails.
		 * One of the session will probably have ranging not done.
		 * Choose the session which have the oldest access.
		 */
		if (is_before_dtu(session->last_access_timestamp_dtu,
				  adopted_session->last_access_timestamp_dtu))
			goto candidate_adopted;

		/* Candidate is not adopted. */
		continue;

	candidate_adopted:
		adopted_session = session;
		*adopted_demand = candidate_demand;
	}
	return adopted_session;
}

static struct mcps802154_access *
fira_get_access(struct mcps802154_region *region, u32 next_timestamp_dtu,
		int next_in_region_dtu, int region_duration_dtu)
{
	struct fira_local *local = region_to_local(region);
	/* 'fsd' acronyms is FiRa Session Demand. */
	struct fira_session_demand fsd;
	struct fira_session *session;
	int max_duration_dtu =
		region_duration_dtu ? region_duration_dtu - next_in_region_dtu :
				      0;

	session = fira_get_next_session(local, next_timestamp_dtu,
					max_duration_dtu, &fsd);
	if (session)
		return fira_session_fsm_get_access(local, session, &fsd);
	return NULL;
}

static int fira_get_demand(struct mcps802154_region *region,
			   u32 next_timestamp_dtu,
			   struct mcps802154_region_demand *next_demand)
{
	struct fira_local *local = region_to_local(region);
	/* 'fsd' for FiRa Session Demand. */
	struct fira_session_demand fsd;
	struct fira_session *session;

	session = fira_get_next_session(local, next_timestamp_dtu, 0, &fsd);
	if (session) {
		next_demand->timestamp_dtu = fsd.timestamp_dtu;
		next_demand->max_duration_dtu = fsd.max_duration_dtu;
		return 1;
	}
	return 0;
}

static struct mcps802154_region_ops fira_region_ops = {
	.owner = THIS_MODULE,
	.name = "fira",
	.open = fira_open,
	.close = fira_close,
	.notify_stop = fira_notify_stop,
	.call = fira_call,
	.get_access = fira_get_access,
	.get_demand = fira_get_demand,
};

struct fira_session *fira_get_session_by_session_id(struct fira_local *local,
						    u32 session_id)
{
	struct fira_session *session;

	list_for_each_entry (session, &local->active_sessions, entry) {
		if (session->id == session_id)
			return session;
	}
	list_for_each_entry (session, &local->inactive_sessions, entry) {
		if (session->id == session_id)
			return session;
	}
	return NULL;
}

void fira_check_all_missed_ranging(struct fira_local *local,
				   const struct fira_session *recent_session,
				   u32 timestamp_dtu)
{
	struct fira_session *session, *s;

	/*
	 * Process sessions with safe function, as the session FSM can leave
	 * the active list for many stop reasons.
	 */
	list_for_each_entry_safe (session, s, &local->active_sessions, entry) {
		if (recent_session == session)
			continue;
		/*
		 * Does the session started during the access of
		 * recent_session?
		 */
		if (!session->block_start_valid)
			continue;
		fira_session_fsm_check_missed_ranging(local, session,
						      timestamp_dtu);
	}
}

int __init fira_region_init(void)
{
	if (do_crypto_selftest_on_module_init)
		WARN_RETURN(fira_crypto_test());

	return mcps802154_region_register(&fira_region_ops);
}

void __exit fira_region_exit(void)
{
	mcps802154_region_unregister(&fira_region_ops);
}

module_param_named(crypto_selftest, do_crypto_selftest_on_module_init, bool, 0644);
module_init(fira_region_init);
module_exit(fira_region_exit);

MODULE_DESCRIPTION("FiRa Region for IEEE 802.15.4 MCPS");
MODULE_AUTHOR("Nicolas Schodet <nicolas.schodet@qorvo.com>");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
