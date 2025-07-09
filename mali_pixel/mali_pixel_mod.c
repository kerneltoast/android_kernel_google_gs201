// SPDX-License-Identifier: GPL-2.0

#include "mali_pixel_mod.h"
#include <linux/module.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Pixel platform integration for GPU");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_AUTHOR("<sidaths@google.com>");
MODULE_VERSION("1.0");
MODULE_SOFTDEP("pre: slc_pmon");
MODULE_SOFTDEP("pre: slc_dummy");
MODULE_SOFTDEP("pre: slc_acpm");

static int __init mali_pixel_init(void)
{
	int ret = 0;

#ifdef CONFIG_MALI_PIXEL_STATS
	ret = mali_pixel_init_pixel_stats();
#endif
	if (ret)
		goto fail_pixel_stats;

#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER
	ret = platform_driver_register(&memory_group_manager_driver);
#endif
	if (ret)
		goto fail_mgm;

#ifdef CONFIG_MALI_PRIORITY_CONTROL_MANAGER
	ret = platform_driver_register(&priority_control_manager_driver);
#endif
	if (ret)
		goto fail_pcm;

#ifdef CONFIG_MALI_PROTECTED_MEMORY_ALLOCATOR
	ret = platform_driver_register(&protected_memory_allocator_driver);
#endif
	if (ret)
		goto fail_pma;

	goto exit;

fail_pma:
#ifdef CONFIG_MALI_PRIORITY_CONTROL_MANAGER
	platform_driver_unregister(&priority_control_manager_driver);
#endif

fail_pcm:
#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER
	platform_driver_unregister(&memory_group_manager_driver);
#endif

fail_pixel_stats:
	/* nothing to clean up here */
fail_mgm:
	/* nothing to clean up here */

exit:
	return ret;
}
module_init(mali_pixel_init);

static void __exit mali_pixel_exit(void)
{
#ifdef CONFIG_MALI_PROTECTED_MEMORY_ALLOCATOR
	platform_driver_unregister(&protected_memory_allocator_driver);
#endif
#ifdef CONFIG_MALI_PRIORITY_CONTROL_MANAGER
	platform_driver_unregister(&priority_control_manager_driver);
#endif
#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER
	platform_driver_unregister(&memory_group_manager_driver);
#endif
}
module_exit(mali_pixel_exit);
