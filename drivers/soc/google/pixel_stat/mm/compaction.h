/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MM_PIXEL_COMPACTION_H_
#define __MM_PIXEL_COMPACTION_H_

struct compact_control;

int compaction_sysfs(struct kobject *parent);
void remove_compaction_sysfs(void);

void vh_compaction_begin(void *, struct compact_control *, long *ts);
void vh_compaction_end(void *, struct compact_control *, long ts);

#endif
