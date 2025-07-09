/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019-2022 Google LLC
 */

#ifndef __GOOGLE_GVOTABLE_H_
#define __GOOGLE_GVOTABLE_H_

#include <linux/types.h>
#include <linux/mutex.h>

#define GVOTABLE_MAX_REASON_LEN		32

#define GVOTABLE_PTR_TO_INT(v)		((int)(uintptr_t)(v))

struct gvotable_election;

typedef int (*gvotable_cmp_fn)(void *a, void *b);
typedef int (*gvotable_callback_fn)(struct gvotable_election *el,
				    const char *reason,
				    void *vote);

struct gvotable_election *
gvotable_create_election(const char *name, int vote_size,
			 int  (*cmp_fn)(void *, void *),
			 gvotable_callback_fn callback_fn,
			 void *data);

#define _GVOTABLE_EPC(a, ...) a ## __VA_ARGS__
#define GVOTABLE__ELECTION_HELPER(t) \
	_GVOTABLE_EPC(_GVOTABLE_EPC(gvotable_create_, t)_election)(name, cmp_fn, cb_fn, data)

/* NOTE: cannot change the data after init to avoid adding a lock for it */
struct gvotable_election *
gvotable_create_int_election(const char *name, gvotable_cmp_fn cmp_fn,
			     gvotable_callback_fn callback_fn,
			     void *data);

struct gvotable_election *
gvotable_create_bool_election(const char *name, gvotable_callback_fn cb_fn,
			      void *data);

int gvotable_destroy_election(struct gvotable_election *el);

struct gvotable_election *gvotable_election_get_handle(const char *name);

/* TODO: redesign this API  */
typedef int (*gvotable_foreach_callback_fn)(void *data, const char *reason,
					    void *vote);
int gvotable_election_for_each(struct gvotable_election *el,
			       gvotable_foreach_callback_fn callback_fn,
			       void *callback_data);
int gvotable_election_set_result(struct gvotable_election *el,
				 const char *reason, void *vote);

void *gvotable_get_data(struct gvotable_election *el);

int gvotable_get_current_reason(struct gvotable_election *el, char *reason,
				int max_reason_len);

int gvotable_set_default(struct gvotable_election *el, void *default_val);
int gvotable_get_default(struct gvotable_election *el, void **default_val);

int gvotable_election_set_name(struct gvotable_election *el, const char *name);

int gvotable_use_default(struct gvotable_election *el, bool default_is_enabled);

int gvotable_cast_vote(struct gvotable_election *el, const char *reason,
		       void *vote, bool enabled);
static inline int gvotable_cast_int_vote(struct gvotable_election *el,
					 const char *reason, int vote,
					 bool enabled)
{
	return gvotable_cast_vote(el, reason, (void *)(long)vote, enabled);
}
static inline int gvotable_cast_long_vote(struct gvotable_election *el,
					  const char *reason, long vote,
					  bool enabled)
{
	return gvotable_cast_vote(el, reason, (void *)vote, enabled);
}
static inline int gvotable_cast_bool_vote(struct gvotable_election *el,
					  const char *reason, bool vote)
{
	return gvotable_cast_vote(el, reason, 0, vote);
}

int gvotable_recast_ballot(struct gvotable_election *el, const char *reason,
			   bool enabled);
int gvotable_run_election(struct gvotable_election *el, bool force_callback);


int gvotable_get_vote(struct gvotable_election *el, const char *reason,
		      void **vote);
int gvotable_get_int_vote(struct gvotable_election *el, const char *reason);

int gvotable_is_enabled(struct gvotable_election *el, const char *reason,
			bool *enabled);

int gvotable_get_current_int_vote(struct gvotable_election *el);
int gvotable_get_current_vote(struct gvotable_election *el, const void **vote);
int gvotable_copy_current_result(struct gvotable_election *el, void *vote,
				 int vote_size);

int gvotable_comparator_uint_max(void *a, void *b);
int gvotable_comparator_uint_min(void *a, void *b);
int gvotable_comparator_int_max(void *a, void *b);
int gvotable_comparator_int_min(void *a, void *b);
int gvotable_comparator_most_recent(void *a, void *b);
int gvotable_comparator_least_recent(void *a, void *b);

/* dump, debug */
typedef int (*gvotable_v2sfn_t)(char *str, size_t len, const void *);
int gvotable_v2s_int(char *str,  size_t len, const void *vote);
int gvotable_v2s_uint(char *str, size_t len, const void *vote);
int gvotable_v2s_uint_hex(char *str, size_t len, const void *vote);
void gvotable_set_vote2str(struct gvotable_election *el,
			   gvotable_v2sfn_t vote2str);
int gvotable_disable_force_int_entry(struct gvotable_election *el);

#endif /* __GOOGLE_GVOTABLE_H_*/
