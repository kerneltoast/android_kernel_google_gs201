/* SPDX-License-Identifier: GPL-2.0 */
/*
 * TTY core internal functions
 */

#ifndef _TTY_INTERNAL_H
#define _TTY_INTERNAL_H

#define tty_msg(fn, tty, f, ...) \
	fn("%s %s: " f, tty_driver_name(tty), tty_name(tty), ##__VA_ARGS__)

#define tty_debug(tty, f, ...)	tty_msg(pr_debug, tty, f, ##__VA_ARGS__)
#define tty_info(tty, f, ...)	tty_msg(pr_info, tty, f, ##__VA_ARGS__)
#define tty_notice(tty, f, ...)	tty_msg(pr_notice, tty, f, ##__VA_ARGS__)
#define tty_warn(tty, f, ...)	tty_msg(pr_warn, tty, f, ##__VA_ARGS__)
#define tty_err(tty, f, ...)	tty_msg(pr_err, tty, f, ##__VA_ARGS__)

#define tty_info_ratelimited(tty, f, ...) \
		tty_msg(pr_info_ratelimited, tty, f, ##__VA_ARGS__)

/* tty_audit.c */
#ifdef CONFIG_AUDIT
void tty_audit_add_data(struct tty_struct *tty, const void *data, size_t size);
void tty_audit_tiocsti(struct tty_struct *tty, char ch);
#else
static inline void tty_audit_add_data(struct tty_struct *tty, const void *data,
				      size_t size)
{
}
static inline void tty_audit_tiocsti(struct tty_struct *tty, char ch)
{
}
#endif

#endif
