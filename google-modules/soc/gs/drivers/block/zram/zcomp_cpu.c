// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/crypto.h>
#include <linux/highmem.h>
#include <linux/cpuhotplug.h>
#include <linux/local_lock.h>

#include "zcomp.h"

static const char * const backends[] = {
#if IS_ENABLED(CONFIG_CRYPTO_LZO)
	"lzo",
	"lzo-rle",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_LZ4)
	"lz4",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_LZ4HC)
	"lz4hc",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_842)
	"842",
#endif
#if IS_ENABLED(CONFIG_CRYPTO_ZSTD)
	"zstd",
#endif
};

struct zcomp_strm {
	/* The members ->buffer and ->tfm are protected by ->lock. */
	local_lock_t lock;
	void *buffer;
	struct crypto_comp *tfm;
};

struct zcomp_strm *zcomp_stream_get(struct zcomp *comp)
{
	struct zcomp_strm *zstrm = comp->private;

	local_lock(&zstrm->lock);
	return this_cpu_ptr(zstrm);
}

void zcomp_stream_put(struct zcomp *comp)
{
	struct zcomp_strm *zstrm = comp->private;

	local_unlock(&zstrm->lock);
}

int zcomp_cpu_compress(struct zcomp *comp, struct page *page,
				struct zcomp_cookie *cookie)
{
	int err;
	unsigned int comp_len;
	void *src;
	struct zcomp_strm *stream;

	/*
	 * Our dst memory (stream->buffer) is always `2 * PAGE_SIZE' sized
	 * because sometimes we can endup having a bigger compressed data
	 * due to various reasons: for example compression algorithms tend
	 * to add some padding to the compressed buffer. Speaking of padding,
	 * comp algorithm `842' pads the compressed length to multiple of 8
	 * and returns -ENOSP when the dst memory is not big enough, which
	 * is not something that ZRAM wants to see. We can handle the
	 * `compressed_size > PAGE_SIZE' case easily in ZRAM, but when we
	 * receive -ERRNO from the compressing backend we can't help it
	 * anymore. To make `842' happy we need to tell the exact size of
	 * the dst buffer, zram_drv will take care of the fact that
	 * compressed buffer is too big.
	 */
	comp_len = PAGE_SIZE * 2;
	stream = zcomp_stream_get(comp);
	src = kmap_atomic(page);
	err = crypto_comp_compress(stream->tfm, src, PAGE_SIZE,
					stream->buffer, &comp_len);
	kunmap_atomic(src);
	if (unlikely(err)) {
		zcomp_stream_put(comp);
		pr_err("Compression failed! err=%d\n", err);
	}

	err = zcomp_copy_buffer(err, stream->buffer, comp_len, cookie);
	zcomp_stream_put(comp);

	return err;
}

int zcomp_cpu_decompress(struct zcomp *comp, void *src,
			unsigned int src_len, struct page *page)
{
	void *dst;
	unsigned int dst_len = PAGE_SIZE;
	struct zcomp_strm *stream;
	int ret;

	dst = kmap_atomic(page);
	stream = zcomp_stream_get(comp);
	ret = crypto_comp_decompress(stream->tfm, src, src_len,
							dst, &dst_len);
	zcomp_stream_put(comp);
	kunmap_atomic(dst);

	return ret;
}

static void zcomp_strm_free(struct zcomp_strm *zstrm)
{
	if (!IS_ERR_OR_NULL(zstrm->tfm))
		crypto_free_comp(zstrm->tfm);
	free_pages((unsigned long)zstrm->buffer, 1);
	zstrm->tfm = NULL;
	zstrm->buffer = NULL;
}

/*
 * Initialize zcomp_strm structure with ->tfm initialized by backend, and
 * ->buffer. Return a negative value on error.
 */
static int zcomp_strm_init(struct zcomp_strm *zstrm, struct zcomp *comp)
{
	zstrm->tfm = crypto_alloc_comp(comp->algo_name, 0, 0);
	/*
	 * allocate 2 pages. 1 for compressed data, plus 1 extra for the
	 * case when compressed size is larger than the original one
	 */
	zstrm->buffer = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, 1);
	if (IS_ERR_OR_NULL(zstrm->tfm) || !zstrm->buffer) {
		zcomp_strm_free(zstrm);
		return -ENOMEM;
	}
	return 0;
}

int zcomp_cpu_create(struct zcomp *comp, const char *name)
{
	int ret = -ENOMEM;

	comp->private = alloc_percpu(struct zcomp_strm);
	if (!comp->private)
		return -ENOMEM;

	ret = cpuhp_state_add_instance(CPUHP_ZCOMP_PREPARE, &comp->node);
	if (ret < 0)
		goto cleanup;

	__module_get(THIS_MODULE);
	return ret;
cleanup:
	free_percpu(comp->private);
	return ret;
}

void zcomp_cpu_destroy(struct zcomp *comp)
{
	cpuhp_state_remove_instance(CPUHP_ZCOMP_PREPARE, &comp->node);
	free_percpu(comp->private);
	module_put(THIS_MODULE);
}

const struct zcomp_operation zcomp_cpu_op = {
	.create = zcomp_cpu_create,
	.destroy = zcomp_cpu_destroy,
	.compress = zcomp_cpu_compress,
	.decompress = zcomp_cpu_decompress,
};

int zcomp_cpu_up_prepare(unsigned int cpu, struct hlist_node *node)
{
	struct zcomp *comp = hlist_entry(node, struct zcomp, node);
	struct zcomp_strm *zstrm;
	int ret;

	zstrm = per_cpu_ptr((struct zcomp_strm __percpu *)comp->private, cpu);
	local_lock_init(&zstrm->lock);

	ret = zcomp_strm_init(zstrm, comp);
	if (ret)
		pr_err("Can't allocate a compression stream\n");
	return ret;
}

int zcomp_cpu_dead(unsigned int cpu, struct hlist_node *node)
{
	struct zcomp *comp = hlist_entry(node, struct zcomp, node);
	struct zcomp_strm *zstrm;

	zstrm = per_cpu_ptr((struct zcomp_strm __percpu*)comp->private, cpu);
	zcomp_strm_free(zstrm);
	return 0;
}

static int __init zcomp_cpu_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(backends); i++) {
		ret = zcomp_register(backends[i], &zcomp_cpu_op);
		if (ret)
			goto out;
	}

	ret = cpuhp_setup_state_multi(CPUHP_ZCOMP_PREPARE,
			"block/zram/zcomp/cpu:prepare",
			zcomp_cpu_up_prepare, zcomp_cpu_dead);
	if (ret)
		goto out;

	return ret;

out:
	for (i = i - 1; i >= 0; i--)
		zcomp_unregister(backends[i]);

	return ret;
}

static void __exit zcomp_cpu_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(backends); i++)
		zcomp_unregister(backends[i]);

	cpuhp_remove_multi_state(CPUHP_ZCOMP_PREPARE);
}

module_init(zcomp_cpu_init);
module_exit(zcomp_cpu_exit);
MODULE_LICENSE("GPL");
