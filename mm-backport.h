/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Backport mm APIs.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __MM_BACKPORT_H__
#define __MM_BACKPORT_H__

#include <linux/mm.h>
#include <linux/version.h>

#if KERNEL_VERSION(5, 6, 0) > LINUX_VERSION_CODE

/*
 * Define pin_user_pages* which are introduced in Linux 5.6.
 *
 * We simply define pin_user_pages* as get_user_pages* here so our driver can
 * prefer PIN over GET when possible.
 */
#ifndef FOLL_PIN

/* define as zero to prevent older get_user_pages* returning EINVAL */
#define FOLL_LONGTERM 0

#define pin_user_pages_fast get_user_pages_fast
#define unpin_user_page put_page

#endif /* FOLL_PIN */

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0) */

#endif /* __MM_BACKPORT_H__ */
