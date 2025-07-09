/* SPDX-License-Identifier: GPL-2.0  WITH Linux-syscall-note */
#ifndef _UAPI_MISC_CRASHINFO_H
#define _UAPI_MISC_CRASHINFO_H

#include <linux/types.h>

#define CRASHINFO_MAGIC        "crashnfo"
#define CRASHINFO_MAGIC_SIZE   8
#define CRASHINFO_REASON_SIZE  256

/*
 * The userspace client expects crashinfo_img_hdr to precede data.
 * Header version 0 to be as follows:
 */
struct crashinfo_img_hdr {
	/* must be CRASHINFO MAGIC. */
	__u8 magic[CRASHINFO_MAGIC_SIZE];

	__le32 header_version;
	__le32 header_size;
	__le32 flags;
	__le32 crc;

	__le64 coredump_size; /* size in bytes */
	char crashinfo[CRASHINFO_REASON_SIZE];
} __attribute__((packed));

#endif /* _UAPI_MISC_CRASHINFO_H */
