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

#ifndef NET_MCPS802154_FPROC_H
#define NET_MCPS802154_FPROC_H

struct mcps802154_local;

/**
 * struct mcps802154_fproc_state - FProc FSM state.
 *
 * This structure contains the callbacks which are called on an event to handle
 * the transition from the active state.
 */
struct mcps802154_fproc_state {
	/** @name: State name. */
	const char *name;
	/** @enter: Run when the state is entered. */
	void (*enter)(struct mcps802154_local *local);
	/** @leave: Run when the state is left. */
	void (*leave)(struct mcps802154_local *local);
	/** @rx_frame: Handle frame reception. */
	void (*rx_frame)(struct mcps802154_local *local);
	/** @rx_timeout: Handle reception timeout. */
	void (*rx_timeout)(struct mcps802154_local *local);
	void (*rx_too_late)(struct mcps802154_local *local);
	/** @rx_error: Handle reception error. */
	void (*rx_error)(struct mcps802154_local *local,
			 enum mcps802154_rx_error_type error);
	/** @tx_done: Handle end of transmission. */
	void (*tx_done)(struct mcps802154_local *local);
	void (*tx_too_late)(struct mcps802154_local *local);
	/** @broken: Handle unrecoverable error. */
	void (*broken)(struct mcps802154_local *local);
	/** @timer_expired: Handle timer expiration, ignored if NULL. */
	void (*timer_expired)(struct mcps802154_local *local);
	/** @schedule_change: Handle schedule change. */
	void (*schedule_change)(struct mcps802154_local *local);
};

/** struct mcps802154_fproc - FProc private data. */
struct mcps802154_fproc {
	/** @state: Pointer to current state. */
	const struct mcps802154_fproc_state *state;
	/** @access: Access being handled. */
	struct mcps802154_access *access;
	/** @tx_skb: Buffer for frame being sent. */
	struct sk_buff *tx_skb;
	/** @frame_idx: Frame index for multiple frames method. */
	size_t frame_idx;
	/** @deferred: Pointer to region context requesting deferred call. */
	struct mcps802154_region *deferred;
};

extern const struct mcps802154_fproc_state mcps802154_fproc_stopped;

/**
 * mcps802154_fproc_init() - Initialize FProc.
 * @local: MCPS private data.
 */
void mcps802154_fproc_init(struct mcps802154_local *local);

/**
 * mcps802154_fproc_uninit() - Uninitialize FProc.
 * @local: MCPS private data.
 */
void mcps802154_fproc_uninit(struct mcps802154_local *local);

/**
 * mcps802154_fproc_change_state() - Change the active state.
 * @local: MCPS private data.
 * @new_state: State to switch to.
 */
void mcps802154_fproc_change_state(
	struct mcps802154_local *local,
	const struct mcps802154_fproc_state *new_state);

/**
 * mcps802154_fproc_access() - Get access and handle it.
 * @local: MCPS private data.
 * @next_timestamp_dtu: Date of next access opportunity.
 */
void mcps802154_fproc_access(struct mcps802154_local *local,
			     u32 next_timestamp_dtu);

/**
 * mcps802154_fproc_access_now() - Get access for current date, and handle it.
 * @local: MCPS private data.
 */
void mcps802154_fproc_access_now(struct mcps802154_local *local);

/**
 * mcps802154_fproc_access_done() - Done with the access, release it.
 * @local: MCPS private data.
 * @error: True when an error happens during the access.
 */
void mcps802154_fproc_access_done(struct mcps802154_local *local, bool error);

/**
 * mcps802154_fproc_access_reset() - Reset an access when things go wrong.
 * @local: MCPS private data.
 *
 * When an unexpected event is received, current transmitted frame and current
 * access are kept as the frame buffer is possibly used by the low level driver.
 * Later when the driver is reset or stopped, the buffer and the access can be
 * released.
 */
void mcps802154_fproc_access_reset(struct mcps802154_local *local);

/**
 * mcps802154_fproc_schedule_change() - Try a schedule change.
 * @local: MCPS private data.
 *
 * Inform the current state that the schedule has changed. To be called
 * exclusively from CA.
 */
void mcps802154_fproc_schedule_change(struct mcps802154_local *local);

/**
 * mcps802154_fproc_stopped_handle() - Go to stopped.
 * @local: MCPS private data.
 */
void mcps802154_fproc_stopped_handle(struct mcps802154_local *local);

/**
 * mcps802154_fproc_broken_handle() - Go to broken, or directly to stopped.
 * @local: MCPS private data.
 */
void mcps802154_fproc_broken_handle(struct mcps802154_local *local);

/**
 * mcps802154_fproc_nothing_handle() - Handle inactivity.
 * @local: MCPS private data.
 * @access: Current access to handle.
 *
 * Return: 0 or error.
 */
int mcps802154_fproc_nothing_handle(struct mcps802154_local *local,
				    struct mcps802154_access *access);

/**
 * mcps802154_fproc_idle_handle() - Handle inactivity with trust in
 * access->duration.
 * @local: MCPS private data.
 * @access: Current access to handle.
 *
 * Return: 0 or error.
 */
int mcps802154_fproc_idle_handle(struct mcps802154_local *local,
				 struct mcps802154_access *access);

/**
 * mcps802154_fproc_rx_handle() - Handle an RX access and change state.
 * @local: MCPS private data.
 * @access: Current access to handle.
 *
 * Return: 0 or error.
 */
int mcps802154_fproc_rx_handle(struct mcps802154_local *local,
			       struct mcps802154_access *access);

/**
 * mcps802154_fproc_tx_handle() - Handle an TX access and change state.
 * @local: MCPS private data.
 * @access: Current access to handle.
 *
 * Return: 0 or error.
 */
int mcps802154_fproc_tx_handle(struct mcps802154_local *local,
			       struct mcps802154_access *access);

/**
 * mcps802154_fproc_multi_handle() - Handle a multiple frames access and change
 * state.
 * @local: MCPS private data.
 * @access: Current access to handle.
 *
 * Return: 0 or error.
 */
int mcps802154_fproc_multi_handle(struct mcps802154_local *local,
				  struct mcps802154_access *access);

/**
 * mcps802154_fproc_vendor_handle() - Handle a multiple frames access manage by vendor.
 * @local: MCPS private data.
 * @access: Current access to handle.
 *
 * Return: 0 or error.
 */
int mcps802154_fproc_vendor_handle(struct mcps802154_local *local,
				   struct mcps802154_access *access);

#endif /* NET_MCPS802154_FPROC_H */
