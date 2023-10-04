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
#ifndef __DW3000_STM_H
#define __DW3000_STM_H

struct dw3000;

/* Pending work bits */
enum { DW3000_IRQ_WORK = BIT(0),
       DW3000_COMMAND_WORK = BIT(1),
       DW3000_TIMER_WORK = BIT(2),
};

/* Custom function for command */
typedef int (*cmd_func)(struct dw3000 *dw, const void *in, void *out);

/* Generic command descriptor */
struct dw3000_stm_command {
	cmd_func cmd;
	const void *in;
	void *out;
	int ret;
};

/* DW3000 state machine */
struct dw3000_state {
	/* Pending work bitmap */
	unsigned long pending_work;
	/* Error recovery count */
	unsigned int recovery_count;
	/* Generic work argument */
	struct dw3000_stm_command *generic_work;
	/* Timer work argument */
	struct dw3000_stm_command timer_work;
	/* Event handler thread */
	struct task_struct *mthread;
	/* Wait queue */
	wait_queue_head_t work_wq;
	/* Enqueue generic mutex */
	struct mutex mtx;
};

/* Event handler of the state machine */
int dw3000_event_thread(void *data);

void dw3000_enqueue(struct dw3000 *dw, unsigned long work);
void dw3000_enqueue_irq(struct dw3000 *dw);
int dw3000_enqueue_generic(struct dw3000 *dw, struct dw3000_stm_command *cmd);
void dw3000_enqueue_timer(struct dw3000 *dw, struct dw3000_stm_command *cmd);

int dw3000_state_init(struct dw3000 *dw, int cpu);
int dw3000_state_start(struct dw3000 *dw);
int dw3000_state_stop(struct dw3000 *dw);

#endif /* __DW3000_STM_H */
