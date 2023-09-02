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
#include <linux/version.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/of.h>

#include "dw3000.h"
#include "dw3000_core.h"

#define DW3000_MIN_CLAMP_VALUE 460

/* First version with sched_setattr_nocheck: v4.16-rc1~164^2~5 */
#if (KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE)
#include <uapi/linux/sched/types.h>
#endif

static int dw3000_min_clamp_value = 0;

module_param_named(min_clamp_value, dw3000_min_clamp_value, int, 0644);
MODULE_PARM_DESC(min_clamp_value, "Sets the minimum cpu frequency the dw3000 thread must run at");


static void dw3000_get_clamp_from_dt(struct dw3000 *dw) {
	int dt_clamp = DW3000_MIN_CLAMP_VALUE;
	int ret;

	if (!dw->dev->of_node)
		return;
	/* debug value is priority  */
	if (dw3000_min_clamp_value) {
		dw->min_clamp_value = dw3000_min_clamp_value;
		dev_info(dw->dev, "using debug min clamp=%d\n", dw->min_clamp_value);
		return;
	}

	ret = of_property_read_u32(dw->dev->of_node, "min_clamp", &dt_clamp);
	if (ret) {
		dev_err(dw->dev, "error reading dt_clamp ret=%d\n", ret);
	}
	dw->min_clamp_value = dt_clamp;
	dev_info(dw->dev, "dt_clamp=%d\n", dw->min_clamp_value);
}

static inline int dw3000_set_sched_attr(struct dw3000 *dw, struct task_struct *p)
{
#if (KERNEL_VERSION(5, 9, 0) > LINUX_VERSION_CODE)
	struct sched_param sched_par = { .sched_priority = MAX_RT_PRIO - 2 };
	/* Increase thread priority */
	return sched_setscheduler(p, SCHED_FIFO, &sched_par);
#else
	struct sched_attr attr = { .sched_policy = SCHED_FIFO,
				   .sched_priority = MAX_RT_PRIO - 2,
				   .sched_flags = SCHED_FLAG_UTIL_CLAMP_MIN,
				   .sched_util_min = dw->min_clamp_value };
	return sched_setattr_nocheck(p, &attr);
#endif
}

/* Enqueue work item(s) */
void dw3000_enqueue(struct dw3000 *dw, unsigned long work)
{
	struct dw3000_state *stm = &dw->stm;
	unsigned long flags;

	spin_lock_irqsave(&stm->work_wq.lock, flags);
	stm->pending_work |= work;
	wake_up_locked(&stm->work_wq);
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

/* Enqueue a generic work and wait for execution */
int dw3000_enqueue_generic(struct dw3000 *dw, struct dw3000_stm_command *cmd)
{
	struct dw3000_state *stm = &dw->stm;
	unsigned long flags;
	int work = DW3000_COMMAND_WORK;

	if (current == stm->mthread) {
		/* We can't enqueue a new work from the same context and wait,
		   but it can be executed directly instead. */
		return cmd->cmd(dw, cmd->in, cmd->out);
	}

	/* Mutex is used in dw3000_enqueue_generic()
	* This protection will work with the spinlock in order to allow
	* the CPU to sleep and avoid ressources wasting during spinning
	*/
	if (mutex_lock_interruptible(&stm->mtx) == -EINTR) {
		dev_err(dw->dev, "work enqueuing interrupted by signal");
		return -EINTR;
	}
	/* Slow path if not in STM thread context */
	spin_lock_irqsave(&stm->work_wq.lock, flags);
	stm->pending_work |= work;
	stm->generic_work = cmd;
	wake_up_locked(&stm->work_wq);
	wait_event_interruptible_locked_irq(stm->work_wq,
					    !(stm->pending_work & work));
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
	mutex_unlock(&stm->mtx);
	return cmd->ret;
}

/* Enqueue a timer work and don't wait for execution because sleeping in not
 * possible from a timer callback function.
 */
void dw3000_enqueue_timer(struct dw3000 *dw, struct dw3000_stm_command *cmd)
{
	struct dw3000_state *stm = &dw->stm;
	unsigned long flags;

	spin_lock_irqsave(&stm->work_wq.lock, flags);
	if (stm->pending_work & DW3000_TIMER_WORK) {
		/* A timer cmd is already queued. */
		spin_unlock_irqrestore(&stm->work_wq.lock, flags);
		dev_err(dw->dev,
			"A timer cmd is already queued, this cmd will be ignored\n");
		return;
	}
	stm->pending_work |= DW3000_TIMER_WORK;
	/* The cmd should not be stored on the stack of the calling function. */
	stm->timer_work = *cmd;
	wake_up_locked(&stm->work_wq);
	/* Can't unlock in the event thread, when the cmd is finished, because
	 * the current function is executed in the timer function in atomic context.
	 * If the unlock is made in the event thread, a preempt leak warning
	 * occurs in call_timer_fn().
	 * So, it's less bad to unlock here.
	 */
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
	/* Can't return cmd->ret because it's not yet executed. */
}

/* Dequeue work item(s) */
void dw3000_dequeue(struct dw3000 *dw, unsigned long work)
{
	struct dw3000_state *stm = &dw->stm;
	unsigned long flags;

	spin_lock_irqsave(&stm->work_wq.lock, flags);
	stm->pending_work &= ~work;
	wake_up_locked(&stm->work_wq);
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

/* Enqueue IRQ work */
void dw3000_enqueue_irq(struct dw3000 *dw)
{
	struct dw3000_state *stm = &dw->stm;
	unsigned long flags;

	spin_lock_irqsave(&stm->work_wq.lock, flags);
	if (!(stm->pending_work & DW3000_IRQ_WORK)) {
		stm->pending_work |= DW3000_IRQ_WORK;
		disable_irq_nosync(dw->spi->irq);
	}
	wake_up_locked(&stm->work_wq);
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

void dw3000_clear_irq(struct dw3000 *dw)
{
	struct dw3000_state *stm = &dw->stm;
	unsigned long flags;

	spin_lock_irqsave(&stm->work_wq.lock, flags);
	stm->pending_work &= ~DW3000_IRQ_WORK;
	enable_irq(dw->spi->irq);
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

/* Wait for new work in the queue */
void dw3000_wait_pending_work(struct dw3000 *dw)
{
	struct dw3000_state *stm = &dw->stm;
	unsigned long flags;

	spin_lock_irqsave(&stm->work_wq.lock, flags);
	wait_event_interruptible_locked_irq(
		stm->work_wq, stm->pending_work || kthread_should_stop());
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);
}

/* Read work queue state */
unsigned long dw3000_get_pending_work(struct dw3000 *dw)
{
	struct dw3000_state *stm = &dw->stm;
	unsigned long work;
	unsigned long flags;

	spin_lock_irqsave(&stm->work_wq.lock, flags);
	work = stm->pending_work;
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);

	return work;
}

/* Chip detect work that run inside the high-priority thread below */
static int dw3000_detect_work(struct dw3000 *dw, const void *in, void *out)
{
	int rc;

	/* Now, read DEV_ID and initialise chip version */
	dev_notice(dw->dev, "checking device presence\n");
	rc = dw3000_check_devid(dw);
	if (rc) {
		dev_err(dw->dev, "device checking failed: %d\n", rc);
		dw3000_poweroff(dw); // Force power-off if error.
		return rc;
	}
	dev_notice(dw->dev, "device present\n");

	/* Read OTP data early */
	rc = dw3000_read_otp(dw, DW3000_READ_OTP_PID | DW3000_READ_OTP_LID);
	if (unlikely(rc)) {
		dev_err(dw->dev, "device OTP read has failed (%d)\n", rc);
		return rc;
	}

	/* Now, we just power-off the device waiting for it to be used by the
	 * MAC to avoid power consumption. Except if SPI tests are enabled. */
	rc = dw3000_spitests_enabled(dw) ? 0 : dw3000_poweroff(dw);
	return rc;
}

/* Event handling thread function */
int dw3000_event_thread(void *data)
{
	struct dw3000 *dw = data;
	struct dw3000_state *stm = &dw->stm;
	unsigned long pending_work = 0;

	/* Run until stopped */
	while (!kthread_should_stop()) {
		/* TODO: State independent errors (ex: PLL_HILO) */

		/* Pending work items */
		pending_work = dw3000_get_pending_work(dw);

		/* TODO: SPI/HW errors.
		 * Every function that uses SPI transmission must enqueue
		 * DW3000_ERROR_WORK item if any error occurs.
		 */

		/* Check IRQ activity */
		if (pending_work & DW3000_IRQ_WORK) {
			/* Handle the event in the ISR */
			dw3000_isr(dw);
			dw3000_clear_irq(dw);
			continue;
		}

		/* In nearly all states, we can execute generic works. */
		if (pending_work & DW3000_COMMAND_WORK) {
			struct dw3000_stm_command *cmd = stm->generic_work;
			bool is_detect_work = cmd->cmd == dw3000_detect_work;

			cmd->ret = cmd->cmd(dw, cmd->in, cmd->out);
			dw3000_dequeue(dw, DW3000_COMMAND_WORK);
			if (unlikely(is_detect_work &&
				     dw3000_spitests_enabled(dw))) {
				/* Run SPI tests if enabled after dw3000_detect_work. */
				dw3000_spitests(dw);
				/* Power down the device after SPI tests */
				dw3000_poweroff(dw);
			}
		}

		/* Execute the cmd from a timer handler that can't sleep. */
		if (pending_work & DW3000_TIMER_WORK) {
			struct dw3000_stm_command *cmd = &stm->timer_work;

			cmd->ret = cmd->cmd(dw, cmd->in, cmd->out);
			dw3000_dequeue(dw, DW3000_TIMER_WORK);
		}

		if (!pending_work) {
			/* Wait for more work */
			dw3000_wait_pending_work(dw);
		}
	}

	/* Make sure device is off */
	dw3000_remove(dw);
	/* Power down the device */
	dw3000_poweroff(dw);

	dev_dbg(dw->dev, "thread finished\n");
	return 0;
}

/* Prepare state machine */
int dw3000_state_init(struct dw3000 *dw, int cpu)
{
	struct dw3000_state *stm = &dw->stm;
	int rc;
	/* Clear memory */
	memset(stm, 0, sizeof(*stm));

	/* Wait queues */
	init_waitqueue_head(&stm->work_wq);

	mutex_init(&stm->mtx);

	/* SKIP: Setup timers (state timeout and ADC timers) */

	/* Init event handler thread */
	stm->mthread = kthread_create(dw3000_event_thread, dw, "dw3000-%s",
				      dev_name(dw->dev));
	if (IS_ERR(stm->mthread)) {
		int err = PTR_ERR(stm->mthread);
		stm->mthread = NULL;
		return err;
	}
	get_task_struct(stm->mthread);
	if (cpu >= 0)
		kthread_bind(stm->mthread, (unsigned)cpu);
	dw->dw3000_pid = stm->mthread->pid;

	/* Increase thread priority */
	dw3000_get_clamp_from_dt(dw);
	rc = dw3000_set_sched_attr(dw, stm->mthread);
	if (rc)
		dev_err(dw->dev, "dw3000_set_sched_attr failed: %d\n", rc);
	return 0;
}

/* Start state machine */
int dw3000_state_start(struct dw3000 *dw)
{
	struct dw3000_state *stm = &dw->stm;
	struct dw3000_stm_command cmd = { dw3000_detect_work, NULL, NULL };
	unsigned long flags;
	int rc;

	/* Ensure spurious IRQ that may come during dw3000_setup_irq() (because
	   IRQ pin is already HIGH) isn't handle by the STM thread. */
	spin_lock_irqsave(&stm->work_wq.lock, flags);
	stm->pending_work &= ~DW3000_IRQ_WORK;
	spin_unlock_irqrestore(&stm->work_wq.lock, flags);

	/* Start state machine thread */
	wake_up_process(stm->mthread);
	dev_dbg(dw->dev, "state machine started\n");

	/* Turn on power and de-assert reset GPIO */
	rc = dw3000_poweron(dw);
	if (rc) {
		dev_err(dw->dev, "device power on failed: %d\n", rc);
		return rc;
	}
	/* Ensure RESET GPIO for enough time */
	rc = dw3000_hardreset(dw);
	if (rc) {
		dev_err(dw->dev, "hard reset failed: %d\n", rc);
		return rc;
	}
	/* and wait SPI ready IRQ */
	rc = dw3000_wait_idle_state(dw);
	if (rc) {
		dev_err(dw->dev, "wait device power on failed: %d\n", rc);
		return rc;
	}
	/* Do chip detection and return result to caller */
	return dw3000_enqueue_generic(dw, &cmd);
}

/* Stop state machine */
int dw3000_state_stop(struct dw3000 *dw)
{
	struct dw3000_state *stm = &dw->stm;

	if (stm->mthread == NULL)
		return 0; /* already stopped or not created yet */

	/* Stop state machine thread */
	kthread_stop(stm->mthread);
	put_task_struct(stm->mthread);
	stm->mthread = NULL;

	dev_dbg(dw->dev, "state machine stopped\n");
	return 0;
}
