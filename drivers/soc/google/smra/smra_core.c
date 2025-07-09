// SPDX-License-Identifier: GPL-2.0
/*
 * SMRA (Smart Readahead)
 *
 */

#define pr_fmt(fmt) "smra_core: " fmt

#include <linux/file.h>
#include <linux/ktime.h>
#include <linux/list_sort.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sort.h>

#include <trace/hooks/mm.h>

#include "smra_core.h"
#include "smra_procfs.h"
#include "smra_sysfs.h"

static DEFINE_RWLOCK(smra_rwlock);
static bool smra_enable = false;
static LIST_HEAD(smra_targets_list);

static struct kmem_cache *smra_metadata_cachep;

static unsigned long *origin_fault_around_bytes_ptr;
static unsigned long origin_fault_around_bytes_val;

atomic64_t smra_buffer_overflow_cnt = ATOMIC_INIT(0);

static inline void fault_around_disable(void)
{
	/*
	 * From should_fault_around() in mm/memory.c, setting @fault_aroud_bytes
	 * to PAGE_SIZE will disable fault around.
	 */
	*origin_fault_around_bytes_ptr = PAGE_SIZE;
}

static inline void fault_around_enable(void)
{
	if (unlikely(*origin_fault_around_bytes_ptr != PAGE_SIZE))
		pr_warn("fault_around_bytes is modified to %lu by other program"
			", now smra overwrites it to %lu",
			*origin_fault_around_bytes_ptr,
			origin_fault_around_bytes_val);
	*origin_fault_around_bytes_ptr = origin_fault_around_bytes_val;
}

/* Protected by target->buf_lock */
static struct smra_info_buffer *smra_buffer_setup(ssize_t size)
{
	struct smra_info_buffer *buf;

	buf = kmalloc(sizeof(struct smra_info_buffer), GFP_KERNEL);
	if (!buf) {
		pr_err("Failed to allocate smra_info_buffer\n");
		return NULL;
	}

	buf->fault_info = kmalloc_array(size, sizeof(struct smra_fault_info),
				  GFP_KERNEL);
	if (!buf->fault_info) {
		kfree(buf);
		pr_err("Failed to allocate info array for smra_info_buffer\n");
		return NULL;
	}

	buf->cur = 0;
	buf->size = size;
	return buf;
}

/* Protected by target->buf_lock */
static void smra_buffer_free(struct smra_info_buffer *buf)
{
	kfree(buf->fault_info);
	kfree(buf);
}

/*
 * The compare function for grouping @smra_fault_info by file and offset.
 * The intention is to group page fault of same file together and sort by
 * offset-ascending order at the beginning of post processing so that we
 * can merge multiple page fault of consecutive loclation into one to reduce
 * overhead when readaheading them using the madvise() syscall.
 */
static int info_file_cmp(const void *l, const void *r)
{

	struct smra_fault_info *l_info = (struct smra_fault_info *)l;
	struct smra_fault_info *r_info = (struct smra_fault_info *)r;

	if (l_info->inode != r_info->inode)
		return (unsigned long)l_info->inode >
			(unsigned long)r_info->inode ? 1 : -1;

	if (l_info->offset != r_info->offset)
		return l_info->offset > r_info->offset ? 1 : -1;

	return 0;
}

/*
 * The compare function for sorting @smra_metadata by timestamp, which is the
 * last step of post processing. This allows us to "replay" IO metadata by the
 * order of happening.
 */
static int info_time_cmp(void *prev, const struct list_head *l,
			 const struct list_head *r)
{
	struct smra_metadata *l_metadata = container_of(l, struct smra_metadata, list);
	struct smra_metadata *r_metadata = container_of(r, struct smra_metadata, list);

	if (l_metadata->time == r_metadata->time)
		return 0;

	return l_metadata->time > r_metadata->time ? 1 : -1;
}

static void info_swap(void *l, void *r, int size)
{
	struct smra_fault_info tmp;
	struct smra_fault_info *l_info = (struct smra_fault_info *)l;
	struct smra_fault_info *r_info = (struct smra_fault_info *)r;

	memcpy(&tmp, l_info, size);
	memcpy(l_info, r_info, size);
	memcpy(r_info, &tmp, size);
}

inline static bool ktime_within(ktime_t x, ktime_t y, s64 threshold)
{
	return abs(ktime_sub(x, y)) <= threshold;
}

/*
 * Helper to create new metadata from @info. The d_path() API is used to
 * transfer struct *file to the actual readable path.
 */
static struct smra_metadata *new_metadata_from_info(struct smra_fault_info *info)
{
	struct smra_metadata *metadata;
	char *path;

	metadata = kmem_cache_alloc(smra_metadata_cachep, GFP_KERNEL);
	if (!metadata)
		return ERR_PTR(-ENOMEM);

	/*
	 * d_path() will return error code if the filepath is too long.
	 * If the file is deleted, the path name will be prefixed with
	 * "(deleted)", which would be later filtered by the smra library.
	 */
	path = d_path(&info->file->f_path, metadata->buf, MAX_PATH_LEN);
	if (IS_ERR(path)) {
		kfree(metadata);
		return ERR_CAST(path);
	}

	metadata->offset = info->offset;
	metadata->time = info->time;
	metadata->nr_pages = 1;
	metadata->path = path;
	return metadata;
}

/*
 * Post processing by merging @smra_fault_info of consecutive location into
 * one and sort them by time. The purpose is to reduce overhead when replay
 * (less metadata, less madvise() syscall is used). Also we sort the final
 * metadata list by time so that when replaying, the page faults happen first
 * are readahead first.
 *
 * The overall post processing flow is as:
 * 1. group @smra_fault_info by file and sort them in offset-ascending order
 * 2. two pointer algorithem to iterate the sorted list to merge faults
 *    with consecutive location into one @smra_metadata
 * 3. sort the metadata list by time and return a list of smra_metadata
 *
 * context: This function is used when recording is stop and all pending
 * page faults are finished recorded. When post-processing, @buf is passed with
 * a separate copy of the original buffers. Hence no need to hold the
 * target->buf_lock and we are allowed to allocate memory with sleepable flags.
 */
static int do_post_processing(struct smra_info_buffer *buf, s64 merge_threshold,
			      struct list_head *footprint)
{
	struct smra_metadata *metadata, *next;
	int nr_pages = 0, nr_dup_pages = 0, metadata_cnt = 0;
	int i, j, err;

	if (buf->cur == 0) {
		pr_warn("Receive empty buffer, nothing to be processed\n");
		return 0;
	}

	if (buf->cur >= buf->size)
		pr_warn("Buffer is too small, please consider recording "
			"again with larger buffer\n");

	/*
	 * Rearrange info based on file and offset, Info of same file
	 * should be grouped and sorted in offset-ascending order
	 */
	sort(buf->fault_info, buf->cur, sizeof(struct smra_fault_info),
	     info_file_cmp, info_swap);

	metadata = new_metadata_from_info(&buf->fault_info[0]);
	if (IS_ERR(metadata)) {
		err = PTR_ERR(metadata);
		goto cleanup;
	}
	list_add_tail(&metadata->list, footprint);
	metadata_cnt = 1;

	/*
	 * Two-pointer to merge continuous smra_fault_info into smra_metadata.
	 * Extend the second pointer every round (e.g. j++) and test if it can
	 * be merged into the group represented by the first pointer (e.g.
	 * buf->info[i]). Split the metadata if:
	 * 1) a new file is detected
	 * 2) reaching non-continuous offset within the same file
	 * 3) offset is continuous, but the time gap is larger than
	 *    @smra_merge_threshold
	 */
	for (i = 0, j = 1; j < buf->cur; j++) {
		if (buf->fault_info[j].inode == buf->fault_info[i].inode &&
		    buf->fault_info[j].offset == buf->fault_info[j - 1].offset + 1 &&
		    ktime_within(buf->fault_info[i].time, buf->fault_info[j].time,
				 merge_threshold)) {
			metadata->nr_pages++;
			metadata->time = min(metadata->time, buf->fault_info[j].time);
			continue;
		}

		if (buf->fault_info[j].inode == buf->fault_info[i].inode &&
		    buf->fault_info[j].offset == buf->fault_info[j - 1].offset) {
			nr_dup_pages++;
			continue;
		}

		nr_pages += metadata->nr_pages;
		metadata = new_metadata_from_info(&buf->fault_info[j]);
		if (IS_ERR(metadata)) {
			err = PTR_ERR(metadata);
			goto cleanup;
		}
		list_add_tail(&metadata->list, footprint);
		metadata_cnt++;
		i = j;
	}
	nr_pages += metadata->nr_pages;

	/*
	 * Sort metadata by timestamp. When we replay, the earlist page fault
	 * will be readhead first.
	 */
	list_sort(NULL, footprint, info_time_cmp);
	pr_info("Merge %d pages into %d metadata covering %d pages, %d "
		"duplicated\n", buf->cur, metadata_cnt, nr_pages, nr_dup_pages);

	return 0;

cleanup:
	list_for_each_entry_safe(metadata, next, footprint, list) {
		list_del(&metadata->list);
		kfree(metadata);
	}
	return err;
}

void smra_post_processing_cleanup(struct list_head footprints[], int nr_targets)
{
	struct smra_metadata *metadata, *next;
	int i;

	for (i = 0; i < nr_targets; i++) {
		list_for_each_entry_safe(metadata, next, &footprints[i], list) {
			list_del(&metadata->list);
			kfree(metadata);
		}
	}
}

/*
 * Should be invoked when recording are finished (e.g. recording_on=false
 * && buffer_has_trace=true) and protected by the smra_sysfs_lock to
 * guarantee this is the only thread accessing the core data structure.

 * context: sysfs_lock should be hold for exclusive access, rw_lock is not
 * needed as the data is already synced when recording is finished.
 */
int smra_post_processing(pid_t target_pids[], int nr_targets,
			 s64 merge_threshold, struct list_head footprints[])
{
	struct smra_target *target;
	int i = 0, err;

	list_for_each_entry(target, &smra_targets_list, list) {
		BUG_ON(i >= nr_targets);
		BUG_ON(target->pid != target_pids[i]);

		pr_info("Start post processing pid %d\n", target_pids[i]);
		err = do_post_processing(target->buf, merge_threshold,
					 &footprints[i]);
		if (err) {
			smra_post_processing_cleanup(footprints, i);
			return err;
		}
		i++;
	}

	return 0;
}

/* Setup target pids and their buffers for recording */
int smra_setup(pid_t target_pids[], int nr_targets, int buffer_size)
{
	int i;
	struct smra_target *target;

	for (i = 0; i < nr_targets; i++) {
		target = kmalloc(sizeof(struct smra_target), GFP_KERNEL);
		if (!target)
			goto cleanup;
		target->buf = smra_buffer_setup(buffer_size);
		if (!target->buf) {
			kfree(target);
			goto cleanup;
		}
		target->pid = target_pids[i];
		spin_lock_init(&target->buf_lock);
		write_lock(&smra_rwlock);
		list_add_tail(&target->list, &smra_targets_list);
		write_unlock(&smra_rwlock);
	}

	return 0;

cleanup:
	write_lock(&smra_rwlock);
	list_for_each_entry(target, &smra_targets_list, list) {
		list_del(&target->list);
		smra_buffer_free(target->buf);
		kfree(target);
	}
	write_unlock(&smra_rwlock);
	return -ENOMEM;
}

void smra_start(void)
{
	write_lock(&smra_rwlock);
	fault_around_disable();
	smra_enable = true;
	write_unlock(&smra_rwlock);
}

void smra_stop(void)
{
	write_lock(&smra_rwlock);
	smra_enable = false;
	fault_around_enable();
	write_unlock(&smra_rwlock);
}

void smra_reset(void)
{
	int i;
	struct smra_target *target, *next;

	write_lock(&smra_rwlock);
	list_for_each_entry_safe(target, next, &smra_targets_list, list) {
		for (i = 0; i < target->buf->cur; i++)
			fput(target->buf->fault_info[i].file);
		smra_buffer_free(target->buf);
		list_del(&target->list);
		kfree(target);
	}
	atomic64_set(&smra_buffer_overflow_cnt, 0);
	write_unlock(&smra_rwlock);
}

/*
 * Helper to find target corresponds to @pid. Protected by
 * read_lock(&smra_rwlock)
 */
static struct smra_target *find_target(pid_t pid)
{
	struct smra_target *target;

	list_for_each_entry(target, &smra_targets_list, list) {
		if (target->pid == pid)
			return target;
	}

	return NULL;
}

static void rvh_do_read_fault(void *data, struct file *file, pgoff_t pgoff,
			      unsigned long *fault_around_bytes)
{
	int cur;
	pid_t tgid;
	struct smra_target *target;

	/*
	 * Should happen only once when we file page fault the first time. Store
	 * the pointer and the original value of @fault_around_bytes so that
	 * we can toggle fault around before/after recording.
	 */
	if (unlikely(!origin_fault_around_bytes_ptr)) {
		origin_fault_around_bytes_ptr = fault_around_bytes;
		origin_fault_around_bytes_val = *fault_around_bytes;
	}

	read_lock(&smra_rwlock);
	/*
	 * "Special" VMA mappings can enter the do_read_fault() path
	 * with file being NULL. Ex. vdso and uprobe.
	 */
	if (!smra_enable || !file)
		goto out;

	tgid = task_tgid_nr(current);
	target = find_target(tgid);
	if (!target)
		goto out;

	spin_lock(&target->buf_lock);
	if (target->buf->cur >= target->buf->size) {
		atomic64_inc(&smra_buffer_overflow_cnt);
		spin_unlock(&target->buf_lock);
		goto out;
	}
	/*
	 * Should fput() when users reset or restart recording.
	 * TODO: Investigate how we can remove this get_file() in the
	 * page fault path.
	 */
	get_file(file);
	cur = target->buf->cur;
	target->buf->fault_info[cur].file = file;
	target->buf->fault_info[cur].offset = pgoff;
	target->buf->fault_info[cur].inode = file_inode(file);
	target->buf->fault_info[cur].time = ktime_get();
	target->buf->cur++;
	spin_unlock(&target->buf_lock);

out:
	read_unlock(&smra_rwlock);
	return;
}

static int smra_vh_init(void)
{
	int ret;

	ret = register_trace_android_rvh_do_read_fault(rvh_do_read_fault, NULL);
	if (ret)
		return ret;

	return 0;
}

int __init smra_init(void)
{
	int err;

	err = smra_vh_init();
	if (err) {
		pr_err("Failed to initialize vendor hooks, error %d\n", err);
		return err;
	}

	smra_metadata_cachep = kmem_cache_create("smra_metadata",
						 sizeof(struct smra_metadata),
						 0, 0, NULL);
	if (!smra_metadata_cachep) {
		pr_err("Failed to create metadata cache\n");
		return -ENOMEM;
	}

	smra_procfs_init();
	smra_sysfs_init();

	return 0;
}

module_init(smra_init);

MODULE_LICENSE("GPL");
