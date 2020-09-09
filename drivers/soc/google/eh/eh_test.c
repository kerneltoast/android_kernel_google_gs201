// SPDX-License-Identifier: GPL-2.0 only
/*
 *  Emerald Hill compression engine test module
 *
 *  Copyright (C) 2020 Google LLC
 *  Author: Petri Gynther <pgynther@google.com>
 *
 *  Derived from:
 *  Hardware Compressed RAM offload driver test module
 *  Copyright (C) 2015 The Chromium OS Authors
 *  Author: Sonny Rao <sonnyrao@chromium.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifdef CONFIG_GOOGLE_EH_DEBUG
#define DEBUG
#endif

#include <asm/atomic.h>
#include <asm/cacheflush.h>
#include <asm/irqflags.h>
#include <asm/page.h>
#include <linux/eh.h>
#include <linux/completion.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/timer.h>

struct eh_compr_test_completion {
	struct completion linux_cmpl;
	struct page *data_page;
	void *data_addr;
	unsigned int compr_status;
	unsigned int compr_size;
	unsigned int decompr_status;
};

static void eh_single_compress_test_cb(unsigned int status, void *data,
				       unsigned int size, void *priv)
{
	struct eh_compr_test_completion *ctc = priv;

	ctc->compr_status = status;

	if (status == EH_CDESC_COPIED || status == EH_CDESC_COMPRESSED) {
		ctc->compr_size = (size < PAGE_SIZE) ? size : PAGE_SIZE;
		memcpy(ctc->data_addr, data, ctc->compr_size);
	}

	complete_all(&ctc->linux_cmpl);
}

static int eh_single_compress_test(struct eh_device *eh_dev,
				   struct page *src_page,
				   struct eh_compr_test_completion *ctc)
{
	int ret;
	long remaining;

	init_completion(&ctc->linux_cmpl);

	ret = eh_compress_pages(eh_dev, &src_page, 1, ctc);
	if (ret)
		goto out;

	remaining = wait_for_completion_killable_timeout(&ctc->linux_cmpl, HZ);

	if (!remaining)
		ret = -ETIMEDOUT;

	pr_devel("%s: remaining %ld got status %u compressed size 0x%x\n",
		 __func__, remaining, ctc->compr_status, ctc->compr_size);
out:
	return ret;
}

static int eh_single_decompress_test(struct eh_device *eh_dev,
				     struct page *dst_page,
				     struct eh_compr_test_completion *ctc)
{
	int ret;
	void *compr_data;

	compr_data = kmap(ctc->data_page);
	ret = eh_decompress_page_sync(eh_dev, compr_data, ctc->compr_size,
				      dst_page);
	kunmap(ctc->data_page);

	pr_devel("%s: got status %d\n", __func__, ret);
	return ret;
}

static int eh_combined_compress_decompress_test(struct eh_device *eh_dev,
						int iter)
{
	struct eh_compr_test_completion test_cmpl;
	int ret = -ENOMEM;
	struct page *dest_page;
	void *dest_map;
	void *src_map;
	struct page *src_page;

	pr_devel("==== %s: running (iteration %d) ====\n", __func__, iter);

	memset(&test_cmpl, 0, sizeof(test_cmpl));
	test_cmpl.data_page = alloc_page(GFP_KERNEL);
	if (!test_cmpl.data_page)
		return -ENOMEM;
	test_cmpl.data_addr = kmap(test_cmpl.data_page);

	dest_page = alloc_page(GFP_KERNEL);
	if (!dest_page)
		goto free_test_cmpl;
	dest_map = kmap(dest_page);

	src_page = alloc_page(GFP_KERNEL);
	if (!src_page)
		goto free_dest;
	src_map = kmap(src_page);

	// test 0: page of all zeroes
	// expect: zero/same value detect
	memset(src_map, 0x0, PAGE_SIZE);

	if (iter % 5 == 1)
		// test 1: 512 bytes of random data
		// expect: compressed data fits in 1K buffer
		get_random_bytes(src_map, PAGE_SIZE / 8);
	else if (iter % 5 == 2)
		// test 2: 1024 bytes of random data
		// expect: compressed data fits in 2K buffer
		get_random_bytes(src_map, PAGE_SIZE / 4);
	else if (iter % 5 == 3)
		// test 3: 2048 bytes of random data
		// expect: compressed data fits in 3K (1K+2K) buffer
		get_random_bytes(src_map, PAGE_SIZE / 2);
	else if (iter % 5 == 4)
		// test 4: 3072 bytes of random data
		// expect: compressed data > 3K, not compressible
		get_random_bytes(src_map, 3 * PAGE_SIZE / 4);

	ret = eh_single_compress_test(eh_dev, src_page, &test_cmpl);
	if (ret) {
		pr_err("%s: single_compress_test failed (%d)\n", __func__, ret);
		goto free_src;
	}

	switch (test_cmpl.compr_status) {
	case EH_CDESC_IDLE:
	case EH_CDESC_ERROR_CONTINUE:
	case EH_CDESC_ERROR_HALTED:
	case EH_CDESC_PENDING:
		pr_err("%s: unexpected status %d\n", __func__,
		       test_cmpl.compr_status);
		ret = -1;
		break;

	case EH_CDESC_COMPRESSED:
		ret = eh_single_decompress_test(eh_dev, dest_page, &test_cmpl);
		if (ret) {
			pr_err("%s: single_decompress_test failed (%d)\n",
			       __func__, ret);
		}
		break;

	case EH_CDESC_COPIED:
		memcpy(dest_map, test_cmpl.data_addr, PAGE_SIZE);
		break;

	case EH_CDESC_ABORT:
		/* incompressible page, did not fit into 3K buffer */
		memcpy(dest_map, src_map, PAGE_SIZE);
		break;

	case EH_CDESC_ZERO:
		memset(dest_map, 0, PAGE_SIZE);
		break;
	}

	if (ret)
		goto free_src;

	if (memcmp(dest_map, src_map, PAGE_SIZE)) {
		pr_err("eh: data miscompare detected\n");
		pr_err("eh: ---- source buffer ----\n");
		print_hex_dump(KERN_ERR, "eh: ", DUMP_PREFIX_OFFSET, 16, 1,
			       src_map, PAGE_SIZE, true);
		pr_err("eh: ---- decompressed buffer ----\n");
		print_hex_dump(KERN_ERR, "eh: ", DUMP_PREFIX_OFFSET, 16, 1,
			       dest_map, PAGE_SIZE, true);
		pr_err("eh: ---- compressed buffer ----\n");
		print_hex_dump(KERN_ERR, "eh: ", DUMP_PREFIX_OFFSET, 16, 1,
			       test_cmpl.data_addr, test_cmpl.compr_size, true);
		ret = -1;
	}

free_src:
	kunmap(src_page);
	__free_page(src_page);

free_dest:
	kunmap(dest_page);
	__free_page(dest_page);

free_test_cmpl:
	kunmap(test_cmpl.data_page);
	__free_page(test_cmpl.data_page);

	return ret;
}

static struct completion eh_compress_bw_test_cmpl;
static unsigned long eh_compress_bw_test_end_ts;

static void eh_compress_bandwidth_test_cb(unsigned int status, void *data,
					  unsigned int size, void *priv)
{
	unsigned long is_last_page = (unsigned long)priv;

	if (is_last_page) {
		eh_compress_bw_test_end_ts = ktime_get_ns();
		pr_devel("%s: compression done for the last page\n", __func__);
		complete_all(&eh_compress_bw_test_cmpl);
	}
}

static int eh_compress_bandwidth_test(struct eh_device *eh_dev,
				      int data_size)
{
	unsigned long i;
	unsigned long page_count;
	struct page **pages;
	void *virt;
	int ret;
	unsigned long start_ts;
	unsigned long delta_us;
	unsigned long remaining;
	unsigned long is_last_page;

	init_completion(&eh_compress_bw_test_cmpl);
	page_count = eh_get_fifo_size(eh_dev);

	pages = kzalloc(page_count * sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	for (i = 0; i < page_count; i++) {
		pages[i] = alloc_page(GFP_KERNEL);
		if (!pages[i]) {
			ret = -ENOMEM;
			goto out;
		}
		virt = kmap(pages[i]);
		memset(virt, 0x0, PAGE_SIZE);
		get_random_bytes(virt, data_size);
		kunmap(pages[i]);
	}

	start_ts = ktime_get_ns();

	for (i = 0; i < page_count; i++) {
		pr_devel("%s: submit page %llu for compression\n", __func__, i);
		is_last_page = (i == (page_count - 1));
		ret = eh_compress_pages(eh_dev, &pages[i], 1, (void *)is_last_page);
		if (ret) {
			pr_err("%s: eh_compress_pages() failed: page %llu ret %d\n",
			       __func__, i, ret);
			goto out;
		}
	}

	remaining = wait_for_completion_killable_timeout(
		&eh_compress_bw_test_cmpl, 5 * HZ);
	if (!remaining) {
		pr_err("%s: test timed out\n", __func__);
		ret = -ETIMEDOUT;
		goto out;
	}

	delta_us = (eh_compress_bw_test_end_ts - start_ts) / 1000;
	pr_devel("%s: done: pages %llu | time %3llu usec | bw %4llu MB/s\n",
		 __func__, page_count, delta_us,
		 page_count * PAGE_SIZE / delta_us);
	ret = delta_us;

out:
	for (i = 0; i < page_count; i++) {
		if (pages[i])
			__free_page(pages[i]);
	}
	kfree(pages);

	return ret;
}

static int eh_test_compr_bw_2k(struct eh_device *eh_dev,
			       unsigned long num_iter)
{
	int ret;
	unsigned long i;
	unsigned long cumul_time_us = 0;

	for (i = 0; i < num_iter; i++) {
		ret = eh_compress_bandwidth_test(eh_dev, SZ_2K);
		if (ret < 0) {
			pr_err("%s: compress_bandwidth_test 2K: FAIL: i=%lu ret=%d\n",
			       eh_dev->name, i, ret);
			goto out;
		}
		cumul_time_us += ret;
	}

	pr_info("%s: compress_bandwidth_test 2K: PASS: iter %lu avg_time %lu usec\n",
		eh_dev->name, num_iter, cumul_time_us / num_iter);
	ret = 0;
out:
	return ret;
}

static int eh_test_compr_bw_3k(struct eh_device *eh_dev,
			       unsigned long num_iter)
{
	int ret;
	unsigned long i;
	unsigned long cumul_time_us = 0;

	for (i = 0; i < num_iter; i++) {
		ret = eh_compress_bandwidth_test(eh_dev, SZ_2K + SZ_1K);
		if (ret < 0) {
			pr_err("%s: compress_bandwidth_test 3K: FAIL: i=%lu ret=%d\n",
			       eh_dev->name, i, ret);
			goto out;
		}
		cumul_time_us += ret;
	}

	pr_info("%s: compress_bandwidth_test 3K: PASS: iter %lu avg_time %lu usec\n",
		eh_dev->name, num_iter, cumul_time_us / num_iter);
	ret = 0;
out:
	return ret;
}

static int eh_test_compr_decompr(struct eh_device *eh_dev,
				 unsigned long num_iter)
{
	int ret;
	unsigned long i;

	for (i = 0; i < num_iter; i++) {
		ret = eh_combined_compress_decompress_test(eh_dev, i);
		if (ret != 0) {
			pr_err("%s: compress_decompress_test: FAIL: i=%lu ret=%d\n",
			       eh_dev->name, i, ret);
			goto out;
		}
	}

	pr_info("%s: compress_decompress_test: PASS: iter %lu\n",
		eh_dev->name, num_iter);
out:
	return ret;
}

static unsigned long parse_iterations(const char *buf, size_t count)
{
	char cmd[32];
	char *nl;

	strlcpy(cmd, buf, sizeof(cmd));
	nl = strchr(cmd, '\n');
	if (nl)
		*nl = '\0';
	return simple_strtoul(cmd, NULL, 0);
}

static ssize_t test_compr_bw_2k(unsigned long num_iter)
{
	struct eh_device *eh_dev;
	int ret;

	eh_dev = eh_create(eh_compress_bandwidth_test_cb, NULL);
	if (!eh_dev)
		return -ENODEV;

	ret = eh_test_compr_bw_2k(eh_dev, num_iter);
	eh_destroy(eh_dev);
	return ret;
}

/*
 * sysfs: test_compr_bw_2k
 */
static ssize_t test_compr_bw_2k_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	unsigned long num_iter;

	num_iter = parse_iterations(buf, count);
	ret = test_compr_bw_2k(num_iter);
	return (ret == 0 ? count : -EIO);
}

static struct kobj_attribute eh_attr_test_compr_bw_2k =
	__ATTR_WO(test_compr_bw_2k);

static ssize_t test_compr_bw_3k(unsigned long num_iter)
{
	struct eh_device *eh_dev;
	int ret;

	eh_dev = eh_create(eh_compress_bandwidth_test_cb, NULL);
	if (!eh_dev)
		return -ENODEV;

	ret = eh_test_compr_bw_3k(eh_dev, num_iter);
	eh_destroy(eh_dev);
	return ret;
}

/*
 * sysfs: test_compr_bw_3k
 */
static ssize_t test_compr_bw_3k_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	unsigned long num_iter;

	num_iter = parse_iterations(buf, count);
	ret = test_compr_bw_3k(num_iter);
	return (ret == 0 ? count : -EIO);
}

static struct kobj_attribute eh_attr_test_compr_bw_3k =
	__ATTR_WO(test_compr_bw_3k);

static ssize_t test_compr_decompr(unsigned long num_iter)
{
	struct eh_device *eh_dev;
	int ret;

	eh_dev = eh_create(eh_single_compress_test_cb, NULL);
	if (!eh_dev)
		return -ENODEV;
	ret = eh_test_compr_decompr(eh_dev, num_iter);
	eh_destroy(eh_dev);
	return ret;
}

/*
 * sysfs: test_compr_decompr
 */
static ssize_t test_compr_decompr_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	int ret;
	unsigned long num_iter;

	num_iter = parse_iterations(buf, count);
	ret = test_compr_decompr(num_iter);
	return (ret == 0 ? count : -EIO);
}
static struct kobj_attribute eh_attr_test_compr_decompr =
	__ATTR_WO(test_compr_decompr);

/*
 * sysfs: test_all
 */
static ssize_t test_all_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	int ret;
	unsigned long num_iter;

	num_iter = parse_iterations(buf, count);

	ret = test_compr_bw_3k(num_iter);
	if (ret)
		goto out;
	ret = test_compr_bw_2k(num_iter);
	if (ret)
		goto out;
	ret = test_compr_decompr(num_iter);
	if (ret)
		goto out;

	pr_info("all tests passed\n");
out:
	return (ret == 0 ? count : -EIO);
}
static struct kobj_attribute eh_attr_test_all = __ATTR_WO(test_all);

static struct attribute *eh_test_attrs[] = {
	&eh_attr_test_compr_bw_2k.attr,
	&eh_attr_test_compr_bw_3k.attr,
	&eh_attr_test_compr_decompr.attr,
	&eh_attr_test_all.attr,
	NULL,
};

static struct attribute_group eh_test_attrs_group = {
	.attrs = eh_test_attrs,
};

static struct kobject *eh_test_kobj;

static int __init eh_test_init(void)
{
	eh_test_kobj = kobject_create_and_add("eh_test", kernel_kobj);

	if (!eh_test_kobj) {
		pr_err("cannot create kobj for eh_test!");
		return -ENOMEM;
	}

	if (sysfs_create_group(eh_test_kobj, &eh_test_attrs_group)) {
		pr_err("cannot create group in eh_test!");
		kobject_put(eh_test_kobj);
		return -ENOMEM;
	}

	return 0;
}

static void __exit eh_test_exit(void)
{
	sysfs_remove_group(eh_test_kobj, &eh_test_attrs_group);
	kobject_put(eh_test_kobj);
}

module_init(eh_test_init);
module_exit(eh_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Petri Gynther <pgynther@google.com>");
MODULE_DESCRIPTION("Emerald Hill compression engine test module");
