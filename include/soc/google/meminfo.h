/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MM_PIXEL_MEMINFO_H__
#define __MM_PIXEL_MEMINFO_H__ __FILE__

struct seq_file;

struct meminfo {
	struct list_head list;

	/* return KB unit */
	unsigned long (*size_kb)(void *private);
	void *private;
	char *name;
};

void rvh_meminfo_proc_show(void *data, struct seq_file *m);
void register_meminfo(struct meminfo *meminfo);
void unregister_meminfo(struct meminfo *meminfo);

#endif
