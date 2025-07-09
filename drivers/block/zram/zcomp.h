/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2014 Sergey Senozhatsky.
 */

#ifndef _ZCOMP_H_
#define _ZCOMP_H_
#include <linux/local_lock.h>

#include "zram_drv.h"

struct zcomp;
struct bio;

#define ZCOMP_ALGO_NAME_MAX 64
#define BATCH_ZCOMP_REQUEST (128)

/*
 * For compression request, zcomp generates a cookie and pass it to
 * the zcomp instance. The zcomp instance need to call zcomp_copy_buffer
 * with this cookie when it completes the compression.
 */
struct zcomp_cookie {
	struct zram *zram; /* zram instance generated the cookie */
	u32 index; /* requested page-sized block index in zram block */
	struct page *page; /* requested page for compression */
	struct bio *bio;
	struct list_head list; /* list for page pool at idle */
			       /* list for pended io at active */
};

struct zcomp_cookie_pool {
	struct list_head head;
	int count;
	spinlock_t lock;
};

struct zcomp_operation {
	int (*compress)(struct zcomp *comp, struct page *page, struct zcomp_cookie *cookie);
	int (*compress_async)(struct zcomp *comp, struct page *page, struct zcomp_cookie *cookie);
	int (*decompress)(struct zcomp *comp, void *src, unsigned int src_len, struct page *page);

	int (*create)(struct zcomp *comp, const char *name);
	void (*destroy)(struct zcomp *comp);
};

/* dynamic per-device compression frontend */
struct zcomp {
	struct zram *zram;
	void *private;
	const struct zcomp_operation *op;
	struct list_head list;
	struct zcomp_cookie_pool cookie_pool;
	unsigned long pend_request;
	struct list_head request_list;
	spinlock_t request_lock;

	struct hlist_node node;
	char algo_name[ZCOMP_ALGO_NAME_MAX];
};

ssize_t zcomp_available_show(const char *comp, char *buf);
bool zcomp_available_algorithm(const char *comp);

struct zcomp *zcomp_create(const char *comp, struct zram *zram);
void zcomp_destroy(struct zcomp *comp);

int zcomp_compress(struct zcomp *comp, u32 index, struct page *page,
			struct bio *bio);
int zcomp_decompress(struct zcomp *comp, u32 index, struct page *page);

int zcomp_register(const char *algo_name, const struct zcomp_operation *operation);
int zcomp_unregister(const char *algo_name);

int zcomp_copy_buffer(int err, void *buffer, int comp_len,
			struct zcomp_cookie *cookie);
#endif /* _ZCOMP_H_ */
