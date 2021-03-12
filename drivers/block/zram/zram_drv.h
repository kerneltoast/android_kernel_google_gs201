/*
 * Compressed RAM block device
 *
 * Copyright (C) 2008, 2009, 2010  Nitin Gupta
 *               2012, 2013 Minchan Kim
 *
 * This code is released using a dual license strategy: BSD/GPL
 * You can choose the licence that better fits your requirements.
 *
 * Released under the terms of 3-clause BSD License
 * Released under the terms of GNU General Public License Version 2.0
 *
 */

#ifndef _ZRAM_DRV_H_
#define _ZRAM_DRV_H_

#include <linux/rwsem.h>
#include <linux/zsmalloc.h>
#include <linux/crypto.h>

#define SECTORS_PER_PAGE_SHIFT	(PAGE_SHIFT - SECTOR_SHIFT)
#define SECTORS_PER_PAGE	(1 << SECTORS_PER_PAGE_SHIFT)
#define ZRAM_LOGICAL_BLOCK_SHIFT 12
#define ZRAM_LOGICAL_BLOCK_SIZE	(1 << ZRAM_LOGICAL_BLOCK_SHIFT)
#define ZRAM_SECTOR_PER_LOGICAL_BLOCK	\
	(1 << (ZRAM_LOGICAL_BLOCK_SHIFT - SECTOR_SHIFT))


/*
 * The lower ZRAM_FLAG_SHIFT bits of table.flags is for
 * object size (excluding header), the higher bits is for
 * zram_pageflags.
 *
 * zram is mainly used for memory efficiency so we want to keep memory
 * footprint small so we can squeeze size and flags into a field.
 * The lower ZRAM_FLAG_SHIFT bits is for object size (excluding header),
 * the higher bits is for zram_pageflags.
 */
#define ZRAM_FLAG_SHIFT 24

/* Flags for zram pages (table[page_no].flags) */
enum zram_pageflags {
	/* zram slot is locked */
	ZRAM_LOCK = ZRAM_FLAG_SHIFT,
	ZRAM_SAME,	/* Page consists the same element */
	ZRAM_WB,	/* page is stored on backing_device */
	ZRAM_UNDER_WB,	/* page is under writeback */
	ZRAM_HUGE,	/* Incompressible page */
	ZRAM_IDLE,	/* not accessed page since last idle marking */

	__NR_ZRAM_PAGEFLAGS,
};

/*-- Data structures */

/* Allocated for each disk page */
struct zram_table_entry {
	union {
		unsigned long handle;
		unsigned long element;
	};
	unsigned long flags;
#ifdef CONFIG_ZRAM_MEMORY_TRACKING
	ktime_t ac_time;
#endif
};

enum zram_stat_item {
	COMPRESSED_SIZE,	/* compressed size of pages stored */
	NR_READ,		/* No. of reads */
	NR_WRITE,		/* No. of writes */
	NR_FAILED_READ,		/* can happen when memory is too low */
	NR_FAILED_WRITE,	/* can happen when memory is too low */
	NR_INVALID_IO, 		/* non-page-aligned I/O requests */
	NR_NOTIFY_FREE,		/* no. of swap slot free notifications */
	NR_SAME_PAGE,		/* no. of same element filled pages */
	NR_HUGE_PAGE,		/* no. of huge pages */
	NR_HUGE_PAGE_SINCE,	/* no. of huge pages since zram set up */
	NR_PAGE_STORED,		/* no. of pages currently stored */
	NR_WRITESTALL,		/* no. of write slow paths */
	NR_MISS_FREE,		/* no. of missed free */
	NR_BD_COUNT,		/* no. of pages in backing device */
	NR_BD_READ,		/* no. of reads from backing device */
	NR_BD_WRITE,		/* no. of writes from backing device */
	NR_ZRAM_STAT_ITEM,
};

struct zram_stats {
	long items[NR_ZRAM_STAT_ITEM];
};

struct zram {
	struct zram_table_entry *table;
	struct zs_pool *mem_pool;
	struct zcomp *comp;
	struct gendisk *disk;
	/* Prevent concurrent execution of device init */
	struct rw_semaphore init_lock;
	/*
	 * the number of pages zram can consume for storing compressed data
	 */
	unsigned long limit_pages;

	struct zram_stats __percpu *pcp_stats;
	atomic_long_t max_used_pages;

	/*
	 * This is the limit on amount of *uncompressed* worth of data
	 * we can store in a disk.
	 */
	u64 disksize;	/* bytes */
	char compressor[CRYPTO_MAX_ALG_NAME];
	/*
	 * zram is claimed so open request will be failed
	 */
	bool claim; /* Protected by bdev->bd_mutex */
	struct file *backing_dev;
#ifdef CONFIG_ZRAM_WRITEBACK
	spinlock_t wb_limit_lock;
	bool wb_limit_enable;
	u64 bd_wb_limit;
	struct block_device *bdev;
	unsigned int old_block_size;
	unsigned long *bitmap;
	unsigned long nr_pages;
#endif
#ifdef CONFIG_ZRAM_MEMORY_TRACKING
	struct dentry *debugfs_dir;
#endif
};


void zram_slot_lock(struct zram *zram, u32 index);
void zram_slot_unlock(struct zram *zram, u32 index);
void zram_slot_update(struct zram *zram, u32 index, unsigned long handle,
			unsigned int comp_len);

unsigned long zram_get_handle(struct zram *zram, u32 index);
size_t zram_get_obj_size(struct zram *zram, u32 index);
unsigned long zram_get_element(struct zram *zram, u32 index);
bool zram_test_flag(struct zram *zram, u32 index, enum zram_pageflags flag);

struct bio;
void zram_bio_endio(struct zram *zram, struct bio *bio, bool is_write, int err);
void zram_page_write_endio(struct zram *zram, struct page *page, int err);
#endif
