// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2022 Google LLC
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/lockdep_types.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/stringhash.h>
#include "gvotable.h"

#ifdef CONFIG_DEBUG_FS
# include <linux/debugfs.h>
# include <linux/seq_file.h>
#endif

#define VOTES_HISTORY_DEPTH  1
#define MAX_NAME_LEN        16
#define MAX_VOTE2STR_LEN    16

#define DEBUGFS_CAST_VOTE_REASON "DEBUGFS"
#define DEBUGFS_FORCE_VOTE_REASON "DEBUGFS_FORCE"

static const char default_reason[] = "Default";
#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_root;
#endif

static DEFINE_MUTEX(gvotable_lock);
static LIST_HEAD(gvotables);

/* a ballot is associated to a reason */
struct ballot {
	bool enabled;
	u32 reason_hash;
	char reason[GVOTABLE_MAX_REASON_LEN];

	u32 idx;
	void *vote[VOTES_HISTORY_DEPTH];
	int vote_size;	/* !=0 when copy is requested */

	u32 num_votes;

	struct list_head list;
};

struct gvotable_election {
	u32 hash;

	int	vote_size;	/* actual vote size */
	bool	use_alloc;	/* if true, use kalloc() for result and votes */

	char name[MAX_NAME_LEN];
	bool	has_name;	/* only elections with names are visible */

	struct mutex cb_lock;	/* see lock_result(), lock_election() */
	struct mutex re_lock;	/* see lock_result(), lock_election() */
	void	*owner;

	void	*result;	/* current result and reason */
	char	reason[GVOTABLE_MAX_REASON_LEN];
	bool	result_is_valid;

	void	*data;		/* _get_data() */

	gvotable_callback_fn callback;
	int (*cmp)(void *a, void *b);

	bool	auto_callback;	/* allow disabling callbacks (internal) */
	void	*default_vote;
	int	has_default_vote;	/* -1 no, 1 yes */

	void	*force_result;
	bool	force_result_is_enabled;

	struct list_head votes; /* actual ballots */
	u32 num_voters;	/* number of ballots */

	u32 num_votes;	/* number of votes */

	gvotable_v2sfn_t vote2str;
	bool	is_int_type;	/* int-type debugfs entries */
	bool	is_bool_type;

	/* some int-type votables must not have the force_int_* debugfs entry */
	bool	disable_force_int_entry;

	/* for lockdep */
	struct lock_class_key cb_lock_key;
	struct lock_class_key re_lock_key;

};

#define gvotable_lock_result(el) mutex_lock(&(el)->re_lock)
#define gvotable_unlock_result(el) mutex_unlock(&(el)->re_lock)

#define CONFIG_DEBUG_GVOTABLE_LOCKS
#ifdef CONFIG_DEBUG_GVOTABLE_LOCKS
static void gvotable_lock_election(struct gvotable_election *el)
{
	int ret;

	ret = mutex_trylock(&el->cb_lock);
	if (!ret) {
		WARN(el->owner == get_current(),
		     "%s cannot call this function from the callback\n",
		     el->has_name ? el->name : "<>");
		mutex_lock(&el->cb_lock);
	}

	el->owner = get_current();
	gvotable_lock_result(el);
}
#else
static inline void gvotable_lock_election(struct gvotable_election *el)
{
	mutex_lock(&(el)->cb_lock);
	mutex_lock(&(el)->re_lock);
}
#endif

#define gvotable_unlock_callback(el) mutex_unlock(&(el)->cb_lock)

static inline void gvotable_unlock_election(struct gvotable_election *el)
{
	mutex_unlock(&(el)->re_lock);
	mutex_unlock(&(el)->cb_lock);
}

struct election_slot {
	struct gvotable_election *el;
	struct list_head list;
	struct dentry *de;
};

int gvotable_comparator_uint_max(void *l, void *r)
{
	unsigned int a = *((unsigned int *)&l);
	unsigned int b = *((unsigned int *)&r);

	if (a > b)
		return 1;
	else if (a < b)
		return (-1);
	else
		return 0;
}
EXPORT_SYMBOL_GPL(gvotable_comparator_uint_max);

static int gvotable_comparator_int(void *l, void *r)
{
	int a = *((int *)&l);
	int b = *((int *)&r);

	if (a > b)
		return 1;
	else if (a < b)
		return (-1);
	else
		return 0;
}

/* compares l and r as integers */
int gvotable_comparator_int_max(void *a, void *b)
{
	return -gvotable_comparator_int(a, b);
}
EXPORT_SYMBOL_GPL(gvotable_comparator_int_max);

/* compares l and r as integers */
int gvotable_comparator_int_min(void *a, void *b)
{
	return gvotable_comparator_int(a, b);
}
EXPORT_SYMBOL_GPL(gvotable_comparator_int_min);

/* compares l and r as integers */
int gvotable_comparator_uint_min(void *a, void *b)
{
	return -gvotable_comparator_uint_max(a, b);
}
EXPORT_SYMBOL_GPL(gvotable_comparator_uint_min);

/* Always add new elements on head of the list */
int gvotable_comparator_most_recent(void *a, void *b)
{
	return (-1);
}
EXPORT_SYMBOL_GPL(gvotable_comparator_most_recent);

/* Always add new element on tail of the list */
int gvotable_comparator_least_recent(void *a, void *b)
{
	return 1;
}
EXPORT_SYMBOL_GPL(gvotable_comparator_least_recent);

/*
 * bool elections return 0 when there are NO votes active and a 1 value when
 * there is at least one vote.
 */
static int gvotable_comparator_bool(void *a, void *b)
{
	return gvotable_comparator_most_recent(a, b);
}

/* NONE add to the head of the list */
static int gvotable_comparator_none(void *a, void *b)
{
	return gvotable_comparator_most_recent(a, b);
}

int gvotable_v2s_int(char *str,  size_t len, const void *vote)
{
	return scnprintf(str, len, "%ld", (unsigned long)vote);
}
EXPORT_SYMBOL_GPL(gvotable_v2s_int);

int gvotable_v2s_uint(char *str, size_t len, const void *vote)
{
	return scnprintf(str, len, "%lu", (unsigned long)vote);
}
EXPORT_SYMBOL_GPL(gvotable_v2s_uint);

int gvotable_v2s_uint_hex(char *str, size_t len, const void *vote)
{
	return scnprintf(str, len, "0x%lx", (unsigned long)vote);
}
EXPORT_SYMBOL_GPL(gvotable_v2s_uint_hex);

/* GVotable internal hashing function */
static u32 gvotable_internal_hash(const char *str)
{
	return full_name_hash(NULL, str, strlen(str));
}

static void gvotable_internal_update_reason(struct gvotable_election *el,
					    const char *new_reason)
{
	strlcpy(el->reason, new_reason, GVOTABLE_MAX_REASON_LEN);
}

static void gvotable_internal_copy_result(struct gvotable_election *el,
					  void **result,
					  void *new_result)
{
	if (el->use_alloc)
		memcpy(*result, new_result, el->vote_size);
	else
		*result = new_result;
}

static void gvotable_internal_update_result(struct gvotable_election *el,
					    void *new_result)
{
	gvotable_internal_copy_result(el, &el->result, new_result);
	el->result_is_valid = true;
}

#define GVOTABLE_BOOL_TRUE_VALUE	((void *)1)
#define GVOTABLE_BOOL_FALSE_VALUE	((void *)0)

/*
 * Determine the new result for the election, return true if the el->callback
 * needs to be called for the election false otherwise. MUST return false if
 * the el->callback is invalid (NULL).
 * requires &->re_lock and &->cb_lock
 */
static bool gvotable_internal_run_election(struct gvotable_election *el)
{
	struct ballot *ballot;
	bool callback_required = false;

	if (el->force_result_is_enabled)
		return false;

	/* the first VALID ballot, the default vote or invalid result */
	list_for_each_entry(ballot, &el->votes, list) {
		if (!ballot->enabled)
			continue;

		/* Update reason if needed TODO: call *_set_result() */
		if (!el->result_is_valid ||
		    strncmp(el->reason, ballot->reason,
			    GVOTABLE_MAX_REASON_LEN)) {
			gvotable_internal_update_reason(el, ballot->reason);
			callback_required = el->auto_callback;
		}

		/* Update result if needed TODO: call *_set_result() */
		if (callback_required ||
		    el->cmp(el->result, ballot->vote[ballot->idx]) != 0) {
			void *new_result;

			/* any-type elections have a default int-type value */
			if (el->is_bool_type)
				new_result = GVOTABLE_BOOL_TRUE_VALUE;
			else
				new_result = ballot->vote[ballot->idx];

			gvotable_internal_update_result(el, new_result);
			callback_required = el->auto_callback;
		}

		/* updated also in gvotable_internal_update_result() */
		el->result_is_valid = true;
		goto exit_done;
	}

	/*
	 * Could not find a vote: use default if when set.
	 * bool-type elections always have a default int-type default vote.
	 */
	if (el->has_default_vote == 1) {
		/* TODO: call *_set_result() */
		if (!el->result_is_valid ||
		    strncmp(el->reason, default_reason,
			    GVOTABLE_MAX_REASON_LEN)) {
			gvotable_internal_update_reason(el, default_reason);
			callback_required = el->auto_callback;
		}

		/* TODO: call *_set_result() */
		if (callback_required ||
		    el->cmp(el->result, el->default_vote) != 0) {
			gvotable_internal_update_result(el, el->default_vote);
			callback_required = el->auto_callback;
		}

		/* updated also in gvotable_internal_update_result() */
		el->result_is_valid = true;
	} else {
		callback_required = el->result_is_valid && el->auto_callback;
		el->result_is_valid = false;
		el->reason[0] = 0; /* default to null reason */
	}

exit_done:
	return callback_required && el->callback;
}

/* requires &gvotable_lock */
static struct election_slot *gvotable_find_internal(const char *name)
{
	struct election_slot *slot;
	struct gvotable_election *el;
	unsigned int hash;

	if (!name)
		return NULL;

	hash = gvotable_internal_hash(name);

	list_for_each_entry(slot, &gvotables, list) {
		el = slot->el;
		if (hash == el->hash && el->has_name &&
		    (strncmp(el->name, name, MAX_NAME_LEN) == 0))
			return slot;
	}

	return NULL;
}

/* requires &gvotable_lock */
static struct election_slot *gvotable_find_internal_ptr(struct gvotable_election *el)
{
	struct election_slot *slot;

	list_for_each_entry(slot, &gvotables, list)
		if (slot->el == el)
			return slot;

	return NULL;
}

/* requires &gvotable_lock */
static void gvotable_add_internal(struct election_slot *slot)
{
	list_add(&slot->list, &gvotables);
}

/* requires &gvotable_lock */
static void gvotable_delete_internal(struct election_slot *slot)
{
	list_del(&slot->list);
	kfree(slot);
}

/* reader lock on election */
static struct ballot *gvotable_ballot_find_internal(struct gvotable_election *el,
						    const char *reason)
{
	struct ballot *ballot;
	u32 reason_hash;

	reason_hash = gvotable_internal_hash(reason);

	list_for_each_entry(ballot, &el->votes, list) {
		if (reason_hash == ballot->reason_hash &&
		    (strncmp(ballot->reason, reason,
			     GVOTABLE_MAX_REASON_LEN) == 0))
			return ballot;
	}
	return NULL;
}

int gvotable_election_for_each(struct gvotable_election *el,
			       gvotable_foreach_callback_fn callback_fn,
			       void *cb_data)
{
	struct ballot *ballot;
	int ret = 0;

	if (el->force_result_is_enabled) {
		ret = callback_fn(cb_data, DEBUGFS_FORCE_VOTE_REASON,
				  el->force_result);
		return ret;
	}

	gvotable_lock_result(el);

	list_for_each_entry(ballot, &el->votes, list) {
		if (!ballot->enabled)
			continue;

		ret = callback_fn(cb_data, ballot->reason,
				  ballot->vote[ballot->idx]);
		if (ret < 0)
			break;
	}

	gvotable_unlock_result(el);

	return ret;
}
EXPORT_SYMBOL_GPL(gvotable_election_for_each);

#ifdef CONFIG_DEBUG_FS
static int gvotable_debugfs_create_el_int(struct election_slot *slot);
static void gvotable_debugfs_create_el(struct election_slot *slot);
static void gvotable_debugfs_delete_el(struct election_slot *slot);
#else
static int gvotable_debugfs_create_el_int(struct election_slot *slot)
{
	return 0;
}

static void gvotable_debugfs_create_el(struct election_slot *slot)
{
}

static void gvotable_debugfs_delete_el(struct election_slot *slot)
{
}
#endif

/* Allow redefining the allocator: required for testing */
#ifndef gvotable_kzalloc
#define gvotable_kzalloc(p, f) kzalloc(sizeof(*(p)), f)
#endif

/* Allow redefining the allocator: required for testing */
#ifndef gvotable_needs_alloc
#define gvotable_needs_alloc(vote_size) \
	((vote_size) > sizeof(((struct ballot *)0)->vote[0]))
#endif

struct gvotable_election *
gvotable_create_election(const char *name, int vote_size,
			 int (*cmp_fn)(void *, void *),
			 gvotable_callback_fn callback_fn,
			 void *data)
{
	struct gvotable_election *el = NULL;
	struct election_slot *slot;

	if (!cmp_fn)
		cmp_fn = gvotable_comparator_none;

	mutex_lock(&gvotable_lock);

	if (name && gvotable_find_internal(name))
		goto done_exit;

	slot = gvotable_kzalloc(slot, GFP_KERNEL);
	if (!slot)
		goto done_exit;

	slot->el = gvotable_kzalloc(slot->el, GFP_KERNEL);
	if (!slot->el) {
		kfree(slot);
		goto done_exit;
	}

	mutex_init(&slot->el->re_lock);
	mutex_init(&slot->el->cb_lock);

	lockdep_register_key(&slot->el->re_lock_key);
	lockdep_register_key(&slot->el->cb_lock_key);

	lockdep_set_class(&slot->el->re_lock, &slot->el->re_lock_key);
	lockdep_set_class(&slot->el->cb_lock, &slot->el->cb_lock_key);

	INIT_LIST_HEAD(&slot->el->votes);
	slot->el->callback	= callback_fn;
	slot->el->auto_callback	= true;
	slot->el->cmp		= cmp_fn;
	slot->el->data		= data;
	slot->el->has_default_vote = -1;
	slot->el->vote_size	= vote_size;
	slot->el->use_alloc	= gvotable_needs_alloc(vote_size);

	/* preallocate result */
	if (slot->el->use_alloc) {
		slot->el->result = kzalloc(vote_size, GFP_KERNEL);
		if (!slot->el->result) {
			kfree(slot->el);
			kfree(slot);
			goto done_exit;
		}
	}

	if (name) {
		slot->el->has_name = true;
		slot->el->hash     = gvotable_internal_hash(name);
		strlcpy(slot->el->name, name, MAX_NAME_LEN);

		gvotable_debugfs_create_el(slot);
	}

	gvotable_add_internal(slot);
	el = slot->el;

done_exit:
	mutex_unlock(&gvotable_lock);
	return el;
}
EXPORT_SYMBOL_GPL(gvotable_create_election);

struct gvotable_election *
gvotable_create_int_election(const char *name,
			     int (*cmp_fn)(void *, void *),
			     gvotable_callback_fn cb_fn,
			     void *data)
{
	struct gvotable_election *el;

	el =  gvotable_create_election(name, sizeof(int), cmp_fn, cb_fn, data);
	if (!el)
		return NULL;

	el->is_int_type = true;
	if (name) {
		struct election_slot *slot;

		slot = gvotable_find_internal(name);
		if (slot)
			gvotable_debugfs_create_el_int(slot);
	}

	return el;
}
EXPORT_SYMBOL_GPL(gvotable_create_int_election);

/*
 * "bool" elections return 1 when there is at least one vote active and 0
 * otherwise. Actual votes are ignored and the result is always the state
 * of the votes.
 */
struct gvotable_election *
gvotable_create_bool_election(const char *name, gvotable_callback_fn cb_fn,
			      void *data)
{
	struct gvotable_election *el;

	el =  gvotable_create_election(name, sizeof(int),
				       gvotable_comparator_bool, cb_fn, data);
	if (!el)
		return NULL;

	/* the fist call to set_default doesn't run election */
	gvotable_set_default(el, GVOTABLE_BOOL_FALSE_VALUE);
	/* run the election to update the actual vote */
	gvotable_internal_run_election(el);
	el->is_bool_type = true;
	return el;
}
EXPORT_SYMBOL_GPL(gvotable_create_bool_election);

/*
 * Destroying an election involves removing all ballots and removing the
 * election (and all its links) from the election slot.
 * TODO: calls to the election API should validate the *el pointer with
 * find_internal before accessing the election.
 */
int gvotable_destroy_election(struct gvotable_election *el)
{
	struct ballot *tmp, *ballot;
	struct election_slot *slot;

	if (!el)
		return -EINVAL;

	gvotable_lock_result(el);

	/* TODO: mark el as pending deletion and fail all operations */
	list_for_each_entry_safe(ballot, tmp, &el->votes, list) {
		if (ballot->vote_size) {
			int i;

			for (i = 0; i < VOTES_HISTORY_DEPTH; i++) {
				kfree(ballot->vote[i]);
				ballot->vote[i] = NULL;
			}
		}

		kfree(ballot);
	}

	gvotable_unlock_result(el);

	/* Find slots associated with this handle and remove them */
	mutex_lock(&gvotable_lock);
	slot = gvotable_find_internal_ptr(el);
	while (slot) {
		gvotable_debugfs_delete_el(slot);
		gvotable_delete_internal(slot);
		slot = gvotable_find_internal_ptr(el);
	}
	mutex_unlock(&gvotable_lock);

	if (el->use_alloc)
		kfree(el->result);
	kfree(el);
	return 0;
}
EXPORT_SYMBOL_GPL(gvotable_destroy_election);

/*
 * Get a public election
 * TODO: the election can be destroyed while in use so SW needs to mark the
 * election as invalid somehow. One way to do this is to use find_internal()
 * to validate the election before accessing the fields (must make an
 * exception with nameless elections)
 */
struct gvotable_election *gvotable_election_get_handle(const char *name)
{
	struct election_slot *slot;

	if (!name)
		return NULL;

	mutex_lock(&gvotable_lock);
	slot = gvotable_find_internal(name);
	mutex_unlock(&gvotable_lock);

	return (slot) ? slot->el : NULL;
}
EXPORT_SYMBOL_GPL(gvotable_election_get_handle);

int gvotable_disable_force_int_entry(struct gvotable_election *el)
{
	if (!el || el->has_name)
		return -EINVAL;

	el->disable_force_int_entry = true;

	return 0;
}
EXPORT_SYMBOL_GPL(gvotable_disable_force_int_entry);

/* Set name of an election (makes election available for lookup) */
int gvotable_election_set_name(struct gvotable_election *el, const char *name)
{
	struct election_slot *slot;

	if (!el || !name)
		return -EINVAL;

	mutex_lock(&gvotable_lock);
	if (el->has_name || gvotable_find_internal(name)) {
		mutex_unlock(&gvotable_lock);
		return -EEXIST;
	}

	el->has_name = true;
	el->hash = gvotable_internal_hash(name);
	strlcpy(el->name, name, MAX_NAME_LEN);

	/* el->has_name ==> find internal will now find the election */
	slot = gvotable_find_internal(name);
	if (slot) {
		gvotable_debugfs_create_el(slot);
		if (slot->el->is_int_type)
			gvotable_debugfs_create_el_int(slot);
	}

	mutex_unlock(&gvotable_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(gvotable_election_set_name);

void gvotable_set_vote2str(struct gvotable_election *el,
			   gvotable_v2sfn_t vote2str)
{
	el->vote2str = vote2str;
}
EXPORT_SYMBOL_GPL(gvotable_set_vote2str);

static int gvotable_run_callback(struct gvotable_election *el)
{
	int ret;

	if (el->result_is_valid)
		ret = el->callback(el, el->reason, el->result);
	else
		ret = el->callback(el, NULL, NULL);

	return ret;
}

/* Set the default value, rerun the election when the value changes */
int gvotable_set_default(struct gvotable_election *el, void *default_val)
{
	bool changed;
	int ret = 0;

	/* boolean elections don't allow changing the default value */
	if (!el || el->is_bool_type)
		return -EINVAL;

	gvotable_lock_election(el);

	changed = el->has_default_vote == 1 && el->default_vote != default_val;
	el->default_vote = default_val;

	if (changed) {
		if (gvotable_internal_run_election(el)) {
			gvotable_unlock_result(el);
			ret = gvotable_run_callback(el);
		}
	} else {
		gvotable_unlock_result(el);
	}

	el->has_default_vote = 1;
	gvotable_unlock_callback(el);
	return ret;
}
EXPORT_SYMBOL_GPL(gvotable_set_default);

/* no need for locks */
int gvotable_get_default(struct gvotable_election *el, void **result)
{
	if (!el || !el->has_default_vote)
		return -EINVAL;

	gvotable_lock_election(el);
	gvotable_internal_copy_result(el, result, el->default_vote);
	gvotable_unlock_election(el);

	return 0;
}

/* Enable or disable usage of a default value for a given election */
int gvotable_use_default(struct gvotable_election *el, bool default_is_enabled)
{
	int ret = 0;

	/* boolean elections don't allow changing the default value */
	if (!el || el->is_bool_type)
		return -EINVAL;

	gvotable_lock_election(el);

	el->has_default_vote = default_is_enabled;
	if (gvotable_internal_run_election(el)) {
		gvotable_unlock_result(el);
		ret = gvotable_run_callback(el);
	} else {
		gvotable_unlock_result(el);
	}

	gvotable_unlock_callback(el);
	return ret;
}
EXPORT_SYMBOL_GPL(gvotable_use_default);

/* Retrieve data for an election */
void *gvotable_get_data(struct gvotable_election *el)
{
	return el ? el->data : NULL;
}
EXPORT_SYMBOL_GPL(gvotable_get_data);

/* NULL (0) is  a valid value when votes are integers */
static int gvotable_get_current_result_unlocked(struct gvotable_election *el,
						const void **result)
{
	if (el->force_result_is_enabled)
		*result = el->force_result;
	else if (el->result_is_valid)
		*result = el->result;
	else
		return -EAGAIN;

	return 0;
}

int gvotable_get_current_vote(struct gvotable_election *el, const void **vote)
{
	int ret;

	if (!el || !vote)
		return -EINVAL;

	gvotable_lock_result(el);
	ret = gvotable_get_current_result_unlocked(el, vote);
	gvotable_unlock_result(el);

	return ret;
}
EXPORT_SYMBOL_GPL(gvotable_get_current_vote);

int gvotable_get_current_int_vote(struct gvotable_election *el)
{
	const void *ptr;
	int ret;

	ret = gvotable_get_current_vote(el, &ptr);
	return (ret) ? ret : (uintptr_t)ptr;
}
EXPORT_SYMBOL_GPL(gvotable_get_current_int_vote);

/* copy the actual current vote */
int gvotable_copy_current_result(struct gvotable_election *el, void *vote,
				 int vote_size)
{
	const void *tmp;
	int ret;

	if (!el || !vote)
		return -EINVAL;
	if (vote_size != el->vote_size)
		return -ERANGE;

	gvotable_lock_result(el);
	ret = gvotable_get_current_result_unlocked(el, &tmp);
	if (ret == 0)
		memcpy(vote, tmp, vote_size);
	gvotable_unlock_result(el);

	return ret;
}
EXPORT_SYMBOL_GPL(gvotable_copy_current_result);

static int gvotable_get_current_reason_unlocked(struct gvotable_election *el,
						char *reason, int max_len)
{
	char *r = NULL;

	if (el->force_result_is_enabled)
		r = DEBUGFS_FORCE_VOTE_REASON;
	else if (el->result_is_valid)
		r = el->reason;

	return r ? strlcpy(reason, r, max_len) : -EAGAIN;
}

/* Retrieve current reason for election result. */
int gvotable_get_current_reason(struct gvotable_election *el, char *reason,
				int max_len)
{
	int len;

	if (!el || !reason)
		return -EINVAL;

	gvotable_lock_result(el);
	len = gvotable_get_current_reason_unlocked(el, reason, max_len);
	gvotable_unlock_result(el);
	return len;
}
EXPORT_SYMBOL_GPL(gvotable_get_current_reason);

/* Get vote associated with a specific reason */
int gvotable_get_vote(struct gvotable_election *el, const char *reason,
		      void **vote)
{
	struct ballot *ballot;

	if (!el || !reason || !vote)
		return -EINVAL;

	gvotable_lock_result(el);
	ballot = gvotable_ballot_find_internal(el, reason);
	if (!ballot) {
		gvotable_unlock_result(el);
		*vote = NULL;
		return (el->is_bool_type) ? 0 : -ENODEV;
	}

	if (!el->is_bool_type && !ballot->enabled) {
		gvotable_unlock_result(el);
		return -EINVAL;
	}

	*vote = ballot->vote[ballot->idx];

	gvotable_unlock_result(el);
	return 0;
}
EXPORT_SYMBOL_GPL(gvotable_get_vote);

int gvotable_get_int_vote(struct gvotable_election *el, const char *reason)
{
	void *ptr;
	int ret;

	ret = gvotable_get_vote(el, reason, &ptr);
	return ret ? ret : (uintptr_t)ptr;
}
EXPORT_SYMBOL_GPL(gvotable_get_int_vote);

/* Determine the reason is enabled */
int gvotable_is_enabled(struct gvotable_election *el, const char *reason,
			bool *enabled)
{
	struct ballot *ballot;

	if (!el || !reason || !enabled)
		return -EINVAL;

	gvotable_lock_result(el);
	ballot = gvotable_ballot_find_internal(el, reason);
	if (!ballot) {
		gvotable_unlock_result(el);
		return -ENODEV;
	}

	*enabled = ballot->enabled;
	gvotable_unlock_result(el);
	return 0;
}
EXPORT_SYMBOL_GPL(gvotable_is_enabled);

/* requires &el->re_lock */
static int gvotable_update_ballot(struct ballot *ballot, void *vote,
				  bool enabled)
{
	const int idx = (ballot->idx + 1) % VOTES_HISTORY_DEPTH;

	ballot->enabled = enabled;

	if (ballot->vote_size == 0) {
		ballot->vote[idx] = vote;
		goto exit_done;
	}

	if (!ballot->vote[idx]) {
		ballot->vote[idx] = kzalloc(ballot->vote_size, GFP_KERNEL);
		if (!ballot->vote[idx])
			return -ENOMEM;
	}

	memcpy(ballot->vote[idx], vote, ballot->vote_size);

exit_done:
	ballot->idx = idx;
	ballot->num_votes++;
	return 0;
}

/* requires &el->re_lock */
static void gvotable_add_ballot(struct gvotable_election *el,
				struct ballot *ballot,
				bool enabled)
{
	struct ballot *last, *tmp;
	void *vote = ballot->vote[ballot->idx];

	/* If this is the only element, just add */
	if (list_empty(&el->votes)) {
		list_add(&ballot->list, &el->votes);
		return;
	}

	/* disabled elements go to the end */
	if (enabled) {
		/* most recent (-1), least recent (1), min */
		list_for_each_entry(tmp, &el->votes, list) {
			if (el->cmp(vote, tmp->vote[tmp->idx]) < 0) {
				/* Add new element before current one */
				list_add_tail(&ballot->list, &tmp->list);
				return;
			}
		}
	}

	/* Add element after the last one */
	last = list_last_entry(&el->votes, struct ballot, list);
	list_add(&ballot->list, &last->list);

	el->num_votes++;
}

int gvotable_recast_ballot(struct gvotable_election *el, const char *reason,
			   bool enabled)
{
	struct ballot *ballot;
	int ret;

	gvotable_lock_election(el);

	ballot = gvotable_ballot_find_internal(el, reason);
	if (!ballot) {
		gvotable_unlock_election(el);
		return -EINVAL;
	}

	list_del(&ballot->list);
	ret = gvotable_update_ballot(ballot, ballot->vote[ballot->idx],
				     enabled);
	if (ret < 0) {
		gvotable_unlock_election(el);
		return ret;
	}

	gvotable_add_ballot(el, ballot, enabled);

	if (gvotable_internal_run_election(el)) {
		gvotable_unlock_result(el);
		ret = gvotable_run_callback(el);
	} else {
		gvotable_unlock_result(el);
	}

	gvotable_unlock_callback(el);
	return ret;
}
EXPORT_SYMBOL_GPL(gvotable_recast_ballot);

#define gvotable_ballot_size_ok(size) ((size) <= sizeof(void *))

int gvotable_run_election(struct gvotable_election *el, bool force_callback)
{
	bool callback;
	int ret = 0;

	/*
	 * In theory this should be calling gvotable_internal_run_election() even if the result
	 * doesn't change yet. In practice this is NOT necessary since the lock should ensure that
	 * you call the callback with the right value.
	 */
	gvotable_lock_election(el);
	callback = gvotable_internal_run_election(el);
	gvotable_unlock_result(el);

	if (!el->callback)
		goto exit_done;

	if (el->force_result_is_enabled) {
		ret = el->callback(el, DEBUGFS_FORCE_VOTE_REASON, el->force_result);
		goto exit_done;
	}

	if (callback || force_callback)
		ret = gvotable_run_callback(el);

exit_done:
	gvotable_unlock_callback(el);
	return ret;
}
EXPORT_SYMBOL_GPL(gvotable_run_election);

/*
 * overrides result and reason for "none" type votables.
 * This can only be called while in the callback for "none" votables.
 * NOTE: the API is not great
 */
int gvotable_election_set_result(struct gvotable_election *el,
				 const char *reason, void *result)
{
	if (!el || !reason || reason[0] == 0)
		return -EINVAL;
	/* a NULL vote is ok when we are not using copy */
	if (el->use_alloc && !result)
		return -EINVAL;
	if (el->cmp != gvotable_comparator_none) {
		WARN_ONCE(1, "Setting the result is not supported for a votable of this type");
		return -EINVAL;
	}

	gvotable_internal_update_reason(el, reason);
	gvotable_internal_update_result(el, result);
	return 0;
}
EXPORT_SYMBOL_GPL(gvotable_election_set_result);

int gvotable_cast_vote(struct gvotable_election *el, const char *reason,
		       void *vote, bool enabled)
{
	bool allocated = false;
	struct ballot *ballot;
	int ret;

	if (!el || !reason || reason[0] == 0)
		return -EINVAL;
	/* a NULL vote is ok when we are not using copy */
	if (el->use_alloc && !vote)
		return -EINVAL;

	gvotable_lock_election(el);

	ballot = gvotable_ballot_find_internal(el, reason);
	if (!ballot) {
		ballot = gvotable_kzalloc(ballot, GFP_KERNEL);
		if (!ballot) {
			gvotable_unlock_election(el);
			return -ENOMEM;
		}

		ballot->reason_hash = gvotable_internal_hash(reason);
		strlcpy(ballot->reason, reason, GVOTABLE_MAX_REASON_LEN);
		if (el->use_alloc)
			ballot->vote_size = el->vote_size;
		el->num_voters++;
		allocated = true;
	} else if (enabled) {
		list_del(&ballot->list);
	}

	if (el->is_bool_type)
		vote = (void *)(unsigned long)enabled;

	ret = gvotable_update_ballot(ballot, vote, enabled);
	if (ret < 0) {
		if (allocated)
			kfree(ballot);

		gvotable_unlock_election(el);
		return ret;
	}

	/* an existing vote is disabled in place */
	if (allocated || enabled)
		gvotable_add_ballot(el, ballot, enabled);

	if (gvotable_internal_run_election(el)) {
		gvotable_unlock_result(el);
		ret = gvotable_run_callback(el);
	} else {
		gvotable_unlock_result(el);
	}

	gvotable_unlock_callback(el);
	return ret;
}
EXPORT_SYMBOL_GPL(gvotable_cast_vote);

#ifdef CONFIG_DEBUG_FS

#define GVOTABLE_DEBUG_ATTRIBUTE(name, fn_read, fn_write) \
static const struct file_operations name = {	\
	.open	= simple_open,			\
	.llseek	= no_llseek,			\
	.read	= fn_read,			\
	.write	= fn_write,			\
}

/* requires lock on election */
static int gvotable_dump_election(char *buf, size_t len,
				  struct gvotable_election *el)
{
	gvotable_v2sfn_t vote2str = el->vote2str;
	char reason[GVOTABLE_MAX_REASON_LEN];
	int rc, count = 0;
	const void *vote;

	count += scnprintf(&buf[count], len - count, "%s:",
			   el->has_name ? el->name : " :");

	rc = gvotable_get_current_reason_unlocked(el, reason, sizeof(reason));
	if (rc < 0)
		count += scnprintf(&buf[count], len - count, " <%d>", rc);
	else
		count += scnprintf(&buf[count], len - count, " current=%s",
				   reason);

	rc = gvotable_get_current_result_unlocked(el, &vote);
	if (rc < 0) {
		count += scnprintf(&buf[count], len - count, " <%d>", rc);
	} else {
		count += scnprintf(&buf[count], len - count, " v=");
		if (vote2str)
			count += vote2str(&buf[count], len - count, vote);
		else
			count += scnprintf(&buf[count], len - count, "<>");
	}

	/* bool elections always have a default (0) vote */
	if (!el->is_bool_type && el->has_default_vote == 1) {
		count += scnprintf(&buf[count], len - count, " d=");
		if (vote2str)
			count += vote2str(&buf[count], len - count,
					  el->default_vote);
		else
			count += scnprintf(&buf[count], len - count, "<>");
	}

	count += scnprintf(&buf[count], len - count, "\n");
	return count;
}

/* requires &gvotable_lock */
static int gvotable_list_elections(char *buf, size_t len)
{
	struct election_slot *slot;
	int count = 0;

	if (list_empty(&gvotables))
		return 0;

	list_for_each_entry(slot, &gvotables, list) {
		gvotable_lock_result(slot->el);

		count += gvotable_dump_election(&buf[count], len - count,
					     slot->el);

		gvotable_unlock_result(slot->el);
	}

	return count;
}

static ssize_t debugfs_list_elections(struct file *filp,
				      char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	const int buf_size = 4096;
	char *buf;
	int len;

	buf = kzalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&gvotable_lock);
	len = gvotable_list_elections(buf, buf_size);
	if (!len)
		len = scnprintf(buf, buf_size, "data not available\n");
	mutex_unlock(&gvotable_lock);

	count = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	kfree(buf);
	return count;
}
GVOTABLE_DEBUG_ATTRIBUTE(debugfs_elections_fops, debugfs_list_elections, NULL);

/* requires lock on election */
static int gvotable_dump_ballot(char *buf, size_t len, struct ballot *ballot,
				gvotable_v2sfn_t vote2str)
{
	int count = 0;

	count += scnprintf(&buf[count], len - count, " %s", ballot->reason);
	count += scnprintf(&buf[count], len - count, " en=%d val=",
			   ballot->enabled);
	count += vote2str(&buf[count], len - count, ballot->vote[ballot->idx]);
	count += scnprintf(&buf[count], len - count, " #votes=%d",
			   ballot->num_votes);

	return count;
}

/* requires lock on election */
static int gvotable_list_ballots(char *buf, size_t len,
				 struct gvotable_election *el,
				 gvotable_v2sfn_t vote2str)
{
	struct ballot *ballot;
	int count = 0;

	if (!vote2str)
		vote2str = el->vote2str;
	if (!el || !vote2str)
		return -EINVAL;

	list_for_each_entry(ballot, &el->votes, list) {
		count += scnprintf(&buf[count], len - count, "%s:",
				   el->has_name ? el->name : " :");
		count += gvotable_dump_ballot(&buf[count], len - count, ballot,
					      vote2str);
		count += scnprintf(&buf[count], len - count, "\n");
	}

	return count;
}

static ssize_t debugfs_list_ballots(struct file *filp,
				    char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct election_slot *slot = filp->private_data;
	const int buf_size = 4096;
	char *buf;
	int len;

	buf = kzalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	gvotable_lock_result(slot->el);

	len = gvotable_list_ballots(buf, buf_size, slot->el, NULL);
	if (len < 0) {
		len = scnprintf(buf, buf_size, "data not available (%d)\n",
				len);
	} else {
		len += gvotable_dump_election(&buf[len], buf_size - len,
					      slot->el);
	}

	gvotable_unlock_result(slot->el);

	count = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	kfree(buf);

	return count;
}

GVOTABLE_DEBUG_ATTRIBUTE(debugfs_ballots_fops, debugfs_list_ballots, NULL);

static ssize_t debugfs_enable_vote(struct file *filp,
				   const char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct election_slot *slot = filp->private_data;
	char reason[GVOTABLE_MAX_REASON_LEN] = { 0 };
	int ret;

	ret = simple_write_to_buffer(reason, (sizeof(reason)-1), ppos, user_buf,
				     count);
	if (ret < 0)
		return -EFAULT;

	ret = gvotable_recast_ballot(slot->el, reason, true);
	if (ret < 0) {
		pr_err("cannot recast %s (%d)\n", reason, ret);
		return ret;
	}

	return count;
}

GVOTABLE_DEBUG_ATTRIBUTE(debugs_enable_vote_fops, NULL, debugfs_enable_vote);

static ssize_t debugfs_disable_vote(struct file *filp,
				    const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct election_slot *slot = filp->private_data;
	char reason[GVOTABLE_MAX_REASON_LEN] = { 0 };
	int ret;

	ret = simple_write_to_buffer(reason, (sizeof(reason)-1), ppos, user_buf,
				     count);
	if (ret < 0)
		return -EFAULT;

	ret = gvotable_recast_ballot(slot->el, reason, false);
	if (ret < 0) {
		pr_err("cannot recast %s (%d)\n", reason, ret);
		return ret;
	}

	return count;
}

GVOTABLE_DEBUG_ATTRIBUTE(debugs_disable_vote_fops, NULL, debugfs_disable_vote);

/* TODO: only enable for int votes */
static int debugfs_cast_int_vote(void *data, u64 val)
{
	struct election_slot *slot = data;
	bool enabled = false;
	int ret;

	ret = gvotable_is_enabled(slot->el, DEBUGFS_CAST_VOTE_REASON,
				  &enabled);
	if (ret < 0)
		pr_debug("vote not present\n");

	return gvotable_cast_vote(slot->el, DEBUGFS_CAST_VOTE_REASON,
				  (void *)val, enabled);
}

DEFINE_SIMPLE_ATTRIBUTE(debugfs_cast_int_vote_fops, NULL,
			debugfs_cast_int_vote, "%llu\n");

/* TODO: only enable for int votes */
static int debugfs_force_int_value(void *data, u64 val)
{
	struct election_slot *slot = data;
	u64 pre_val = (u64)slot->el->force_result;
	int ret = 0;

	gvotable_lock_election(slot->el);

	slot->el->force_result = (void *)val;
	gvotable_unlock_result(slot->el);

	if (!slot->el->callback)
		goto exit_done;

	if (slot->el->force_result_is_enabled && (pre_val != val))
		ret = slot->el->callback(slot->el, DEBUGFS_FORCE_VOTE_REASON,
					 slot->el->force_result);

exit_done:
	gvotable_unlock_callback(slot->el);
	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(debugfs_force_int_value_fops, NULL,
			debugfs_force_int_value, "%llu\n");

static int debugfs_force_int_active(void *data, u64 val)
{
	struct election_slot *slot = data;
	int ret = 0;

	gvotable_lock_election(slot->el);

	slot->el->force_result_is_enabled = !!val;
	gvotable_unlock_result(slot->el);

	if (!slot->el->callback)
		goto exit_done;

	if (slot->el->force_result_is_enabled)
		ret = slot->el->callback(slot->el, DEBUGFS_FORCE_VOTE_REASON,
					 slot->el->force_result);
	else
		ret = gvotable_run_callback(slot->el);

exit_done:
	gvotable_unlock_callback(slot->el);
	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(debugfs_force_int_active_fops, NULL,
			debugfs_force_int_active, "%llu\n");

static void gvotable_debugfs_cleanup(void)
{
	debugfs_remove_recursive(debugfs_root);
	debugfs_root = NULL;
}

static int gvotable_debugfs_create_el_int(struct election_slot *slot)
{
	if (!slot->de)
		return -ENOENT;

	debugfs_create_file("cast_int_vote", 0200, slot->de, slot,
			    &debugfs_cast_int_vote_fops);

	if (!slot->el->disable_force_int_entry) {
		debugfs_create_file("force_int_value", 0200, slot->de, slot,
				&debugfs_force_int_value_fops);
		debugfs_create_file("force_int_active", 0200, slot->de, slot,
				&debugfs_force_int_active_fops);
	}

	return 0;
}

static void gvotable_debugfs_create_el(struct election_slot *slot)
{
	/* TODO: add anon to a special directory */
	if (!slot->el->has_name)
		return;

	if (!debugfs_root) {
		debugfs_root = debugfs_create_dir("gvotables", 0);
		if (!debugfs_root) {
			pr_err("cannot create gvotables debug directory\n");
			return;
		}

		/* add: list all the elections including the anon ones */
		debugfs_create_file("elections", 0444, debugfs_root, NULL,
				    &debugfs_elections_fops);

		pr_err("gvotables debug directory OK\n");
	}

	slot->de = debugfs_create_dir(slot->el->name, debugfs_root);
	if (!slot->de) {
		pr_err("cannot create debugfs entry for slot=%p\n", slot);
		return;
	}

	debugfs_create_file("status", 0444, slot->de, slot,
			    &debugfs_ballots_fops);
	debugfs_create_file("enable_vote", 0200, slot->de, slot,
			    &debugs_enable_vote_fops);
	debugfs_create_file("disable_vote", 0200, slot->de, slot,
			    &debugs_disable_vote_fops);
}

static void gvotable_debugfs_delete_el(struct election_slot *slot)
{
	if (!slot->de)
		return;

	debugfs_remove_recursive(slot->de);
	slot->de = NULL;
}
#else /* CONFIG_DEBUG_FS */
static inline void gvotable_debugfs_cleanup(void)
{
}
#endif

static int __init gvotable_init(void)
{
	return 0;
}

static void __exit gvotable_exit(void)
{
	struct election_slot *slot, *tmp;

	gvotable_debugfs_cleanup();
	list_for_each_entry_safe(slot, tmp, &gvotables, list) {
		pr_debug("Destroying %p\n", slot->el);
		gvotable_destroy_election(slot->el);
	}
}

module_init(gvotable_init);
module_exit(gvotable_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luigi Zevola <zevola@google.com>");
MODULE_DESCRIPTION("Election library for shared resources");
MODULE_VERSION("0.01");
