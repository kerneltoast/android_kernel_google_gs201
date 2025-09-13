/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __PA_KILL_CORE_H_
#define __PA_KILL_CORE_H_

long kill_processes(int oom_score_adj);
long kill_victim_task(struct task_struct *victim);

#endif
