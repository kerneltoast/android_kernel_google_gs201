#include <linux/types.h>
#include <linux/debugfs.h>

#include "gcma_core.h"

static bool workingset = true;

bool workingset_filter_enabled(void)
{
	return workingset;
}

static int gcma_evict_write(void *data, u64 val)
{
	unsigned long nr_pages = val;

	evict_gcma_lru_pages(nr_pages);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(gcma_evict_fops, NULL, gcma_evict_write, "%llu\n");

int __init gcma_debugfs_init(void)
{
	struct dentry *gcma_debugfs_root;

	gcma_debugfs_root = debugfs_create_dir("gcma", NULL);
	if (!gcma_debugfs_root)
		return -ENOMEM;

	debugfs_create_bool("workingset", 0644, gcma_debugfs_root, &workingset);
	debugfs_create_file("evict", 0200, gcma_debugfs_root, NULL, &gcma_evict_fops);

	return 0;
}
