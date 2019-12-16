/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SUBSYSTEM_COREDUMP_H
#define __SUBSYSTEM_COREDUMP_H

#include <linux/device.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>

#define SSCD_NAME			"sscoredump"

/* sscd segment ( similar to ELF memory segments) */
struct sscd_segment {
	void		*addr;
	u64		size;
	u64		flags;

	/* passed to elf sprogram header */
	void		*paddr;
	void		*vaddr;
} __packed;

/* sscd_report flags */
#define SSCD_FLAGS_ELFARM32HDR		0x0001
#define SSCD_FLAGS_ELFARM64HDR		0x0002

struct sscd_platform_data {
	/* report crash */
	int (*sscd_report)(struct platform_device *pdev,
			   struct sscd_segment *segs, u16 nsegs,
			   u64 flags, const char *crash_info);
};

#endif /* __SUBSYSTEM_COREDUMP_H */
