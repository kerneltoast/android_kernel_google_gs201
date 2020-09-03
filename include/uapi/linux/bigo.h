/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#ifndef _UAPI_BIGO_H_
#define _UAPI_BIGO_H_

#include <linux/ioctl.h>
#ifdef __KERNEL__
#include <linux/compiler.h>
#else
#define __user
#include <linux/types.h>
#endif

/*
 * Structures as parameters for BigOcean commands. It should be passed
 * into ioctl() as the third parameter.
 *
 * The usage and meaning of each field is documented in command
 * identifiers.
 */

struct bigo_ioc_regs {
	__u64 regs;
	__u32 regs_size;
};

struct bigo_ioc_mapping {
	int fd;
	__u32 iova;
	__u32 offset;
	__u32 size;
};

struct bigo_ioc_frmsize {
	__u32 height;
	__u32 width;
};

struct bigo_cache_info {
	__u32 size;
	__u32 pid;
};

/*
 * Helpers for defining command identifiers. User space should not
 * use these macros directly.
 *
 * <START OF HELPERS>
 */
#define BIGO_IOC_MAGIC 'B'

#define _BIGO_IO(nr) _IO(BIGO_IOC_MAGIC, nr)
#define _BIGO_IOR(nr, size) _IOR(BIGO_IOC_MAGIC, nr, size)
#define _BIGO_IOW(nr, size) _IOW(BIGO_IOC_MAGIC, nr, size)
#define _BIGO_IOWR(nr, size) _IOWR(BIGO_IOC_MAGIC, nr, size)

enum bigo_cmd_id {
	BIGO_CMD_PROCESS,
	BIGO_CMD_ABORT,
	BIGO_CMD_MAP,
	BIGO_CMD_UNMAP,
	BIGO_CMD_CONFIG_FRMRATE,
	BIGO_CMD_CONFIG_FRMSIZE,
	BIGO_CMD_GET_CACHE_INFO,
	BIGO_CMD_CONFIG_SECURE,
	BIGO_CMD_MAXNR,
};
/* <END OF HELPERS> */

#define BIGO_IOCX_PROCESS _BIGO_IOWR(BIGO_CMD_PROCESS, struct bigo_ioc_regs)
#define BIGO_IOCX_MAP _BIGO_IOWR(BIGO_CMD_MAP, struct bigo_ioc_mapping)
#define BIGO_IOCX_UNMAP _BIGO_IOW(BIGO_CMD_UNMAP, struct bigo_ioc_mapping)
#define BIGO_IOCX_CONFIG_FRMRATE _BIGO_IOW(BIGO_CMD_CONFIG_FRMRATE, __u32)
#define BIGO_IOCX_CONFIG_FRMSIZE                                               \
	_BIGO_IOW(BIGO_CMD_CONFIG_FRMSIZE, struct bigo_ioc_frmsize)
#define BIGO_IOCX_GET_CACHE_INFO                                               \
	_BIGO_IOR(BIGO_CMD_GET_CACHE_INFO, struct bigo_cache_info)
#define BIGO_IOCX_ABORT _BIGO_IO(BIGO_CMD_ABORT)
#define BIGO_IOCX_CONFIG_SECURE _BIGO_IOW(BIGO_CMD_CONFIG_SECURE, __u32)

#endif /* _UAPI_BIGO_H_ */
