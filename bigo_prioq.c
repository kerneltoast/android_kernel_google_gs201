// SPDX-License-Identifier: GPL-2.0-only
/*
 * Priority queue impletation with BigOcean
 *
 * Copyright 2021 Google LLC.
 *
 * Author: Ruofei Ma <ruofeim@google.com>
 */

#include <linux/kthread.h>
#include <linux/module.h>

#include "bigo_prioq.h"

int enqueue_prioq(struct bigo_core *core, struct bigo_inst *inst)
{
	struct bigo_job *job = &inst->job;

	if(!core || !inst)
		return -EINVAL;

	spin_lock(&core->prioq.lock);
	list_add_tail(&job->list, &core->prioq.queue[inst->priority]);
	set_bit(inst->priority, &core->prioq.bitmap);
	spin_unlock(&core->prioq.lock);

	wake_up(&core->worker);
	return 0;
}

bool dequeue_prioq(struct bigo_core *core, struct bigo_job **job,
			bool *should_stop)
{
	int high_prio;
	struct bigo_job *j = NULL;
	struct list_head *queue;
	if (!core || !job || !should_stop)
		return false;

	*should_stop = false;
	if(kthread_should_stop()) {
		*should_stop = true;
		return true;
	}

	spin_lock(&core->prioq.lock);
	high_prio = ffs(core->prioq.bitmap) - 1;
	if (high_prio < 0)
		goto exit;

	queue = &core->prioq.queue[high_prio];
	j = list_first_entry_or_null(queue, struct bigo_job, list);
	if (j) {
		list_del(&j->list);
		if (list_empty(queue))
			clear_bit(high_prio, &core->prioq.bitmap);
	}

exit:
	spin_unlock(&core->prioq.lock);
	*job = j;
	return *job != NULL;
}

void clear_job_from_prioq(struct bigo_core *core, struct bigo_inst *inst)
{
	int i;
	struct bigo_job *curr, *next;
	struct bigo_inst *curr_inst;
	spin_lock(&core->prioq.lock);
	for (i = 0; i < BO_MAX_PRIO; i++) {
		list_for_each_entry_safe(curr, next, &core->prioq.queue[i], list) {
			curr_inst = container_of(curr, struct bigo_inst, job);
			if (inst == curr_inst)
				list_del(&curr->list);
		}
	}
	spin_unlock(&core->prioq.lock);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ruofei Ma <ruofeim@google.com>");
