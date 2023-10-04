// SPDX-License-Identifier: GPL-2.0

#include <linux/platform_device.h>

#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER
extern struct platform_driver memory_group_manager_driver;
#endif

#ifdef CONFIG_MALI_PRIORITY_CONTROL_MANAGER
extern struct platform_driver priority_control_manager_driver;
#endif

#ifdef CONFIG_MALI_PROTECTED_MEMORY_ALLOCATOR
extern struct platform_driver protected_memory_allocator_driver;
#endif

#ifdef CONFIG_MALI_PIXEL_STATS
extern int mali_pixel_init_pixel_stats(void);
#endif
