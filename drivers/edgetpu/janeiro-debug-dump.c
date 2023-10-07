// SPDX-License-Identifier: GPL-2.0

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP) || IS_ENABLED(CONFIG_EDGETPU_TEST)

#include "mobile-debug-dump.c"

#else /* IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP) || IS_ENABLED(CONFIG_EDGETPU_TEST) */

#include "edgetpu-debug-dump.c"

int edgetpu_debug_dump_init(struct edgetpu_dev *etdev)
{
	return 0;
}

void edgetpu_debug_dump_exit(struct edgetpu_dev *etdev)
{
}

#endif /* IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP) || IS_ENABLED(CONFIG_EDGETPU_TEST) */
