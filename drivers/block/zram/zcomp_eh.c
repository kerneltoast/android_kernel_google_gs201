// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>

#include "zcomp.h"
#include <linux/eh.h>

static void zcomp_eh_compress_done(int compr_ret, void *buffer,
				   unsigned int size, void *priv)
{
	zcomp_copy_buffer(compr_ret ? -EIO : 0, buffer, size, priv);
}

static int zcomp_eh_compress(struct zcomp *comp, struct page *page,
				struct zcomp_cookie *cookie)
{
	return eh_compress_page(comp->private, page, cookie);
}

static int zcomp_eh_decompress(struct zcomp *comp, void *src,
			unsigned int src_len, struct page *page)
{
	return eh_decompress_page(comp->private, src, src_len, page);
}

static void zcomp_eh_destroy(struct zcomp *comp)
{
	eh_destroy(comp->private);
	module_put(THIS_MODULE);
}

static int zcomp_eh_create(struct zcomp *comp, const char *name)
{
	struct eh_device *eh_dev = eh_create(zcomp_eh_compress_done);

	if (IS_ERR(eh_dev))
		return -ENODEV;

	comp->private = eh_dev;
	__module_get(THIS_MODULE);

	return 0;
}

const struct zcomp_operation zcomp_eh_op = {
	.create = zcomp_eh_create,
	.destroy = zcomp_eh_destroy,
	.compress_async = zcomp_eh_compress,
	.decompress = zcomp_eh_decompress,
};

static int __init zcomp_eh_init(void)
{
	return zcomp_register("lz77eh", &zcomp_eh_op);
}

static void __exit zcomp_eh_exit(void)
{
	zcomp_unregister("lz77eh");
}

module_init(zcomp_eh_init);
module_exit(zcomp_eh_exit);
MODULE_LICENSE("GPL");
