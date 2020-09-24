/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019-2020 Google LLC
 */

#ifndef __GOOGLE_LOGBUFFER_H_
#define __GOOGLE_LOGBUFFER_H_

#include <stdarg.h>

struct logbuffer;
#if IS_ENABLED(CONFIG_GOOGLE_LOGBUFFER)
void logbuffer_log(struct logbuffer *instance, const char *fmt, ...);
void logbuffer_vlog(struct logbuffer *instance, const char *fmt,
		    va_list args);
/*
 * Registers a new log buffer entry.
 * param name: name of the file in the /d/logbuffer/ directory.
 * returns the pointer to the logbuffer metadata.
 */
struct logbuffer *logbuffer_register(const char *name);

void logbuffer_unregister(struct logbuffer *instance);
#else
static inline void logbuffer_log(struct logbuffer *instance, const char *fmt, ...)
{
}

static inline void logbuffer_vlog(struct logbuffer *instance, const char *fmt, va_list args)
{
}

static inline struct logbuffer *logbuffer_register(const char *name)
{
	return NULL;
}

static inline void logbuffer_unregister(struct logbuffer *instance)
{
}
#endif
#endif /* __GOOGLE_LOGBUFFER_H_ */

