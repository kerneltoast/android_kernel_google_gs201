/*
 * Google LWIS Transaction Processor
 *
 * Copyright (c) 2019 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_TRANSACTION_H_
#define LWIS_TRANSACTION_H_

#include "lwis_commands.h"

/* LWIS forward declarations */
struct lwis_device;
struct lwis_client;

/* Transaction entry. Each entry belongs to two queues:
 * 1) Event list: Transactions are sorted by event IDs. This is to search for
 *    the appropriate transactions to trigger.
 * 2) Process queue: When it's time to process, the transaction will be put
 *    into a queue.
 */
struct lwis_transaction {
	struct lwis_transaction_info info;
	struct lwis_transaction_response_header *resp;
	struct list_head event_list_node;
	struct list_head process_queue_node;
};

/* For debugging purposes, keeps track of the transaction information, as
 * well as the time it executes and the time it took to execute.
*/
struct lwis_transaction_history {
	struct lwis_transaction_info info;
	int64_t process_timestamp;
	int64_t process_duration_ns;
};

struct lwis_transaction_event_list {
	int64_t event_id;
	struct list_head list;
	struct hlist_node node;
};

int lwis_transaction_init(struct lwis_client *client);
int lwis_transaction_clear(struct lwis_client *client);
int lwis_transaction_client_flush(struct lwis_client *client);
int lwis_transaction_client_cleanup(struct lwis_client *client);

int lwis_transaction_event_trigger(struct lwis_client *client, int64_t event_id,
				   int64_t event_counter, struct list_head *pending_events,
				   bool in_irq);
int lwis_transaction_cancel(struct lwis_client *client, int64_t id);

void lwis_transaction_free(struct lwis_device *lwis_dev, struct lwis_transaction *transaction);

/* Expects lwis_client->transaction_lock to be acquired before calling
 * the following functions. */
int lwis_transaction_submit_locked(struct lwis_client *client,
				   struct lwis_transaction *transaction);
int lwis_transaction_replace_locked(struct lwis_client *client,
				    struct lwis_transaction *transaction);

#endif /* LWIS_TRANSACTION_H_ */
