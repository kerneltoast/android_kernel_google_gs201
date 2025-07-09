/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_EXYNOS_PM_QOS_H
#define _LINUX_EXYNOS_PM_QOS_H
/* interface for the exynos_pm_qos_power infrastructure of the linux kernel.
 *
 * Mark Gross <mgross@linux.intel.com>
 */
#include <linux/plist.h>
#include <linux/notifier.h>
#include <linux/device.h>
#include <linux/workqueue.h>

enum {
	EXYNOS_PM_QOS_RESERVED = 0,
	PM_QOS_CLUSTER0_FREQ_MIN,
	PM_QOS_CLUSTER0_FREQ_MAX,
	PM_QOS_CLUSTER1_FREQ_MIN,
	PM_QOS_CLUSTER1_FREQ_MAX,
	PM_QOS_CLUSTER2_FREQ_MIN,
	PM_QOS_CLUSTER2_FREQ_MAX,
	PM_QOS_DEVICE_THROUGHPUT,
	PM_QOS_INTCAM_THROUGHPUT,
	PM_QOS_DEVICE_THROUGHPUT_MAX,
	PM_QOS_INTCAM_THROUGHPUT_MAX,
	PM_QOS_BUS_THROUGHPUT,
	PM_QOS_BUS_THROUGHPUT_MAX,
	PM_QOS_DISPLAY_THROUGHPUT,
	PM_QOS_DISPLAY_THROUGHPUT_MAX,
	PM_QOS_CAM_THROUGHPUT,
	PM_QOS_CAM_THROUGHPUT_MAX,
	PM_QOS_MFC_THROUGHPUT,
	PM_QOS_MFC_THROUGHPUT_MAX,
	PM_QOS_TNR_THROUGHPUT,
	PM_QOS_TNR_THROUGHPUT_MAX,
	PM_QOS_BW_THROUGHPUT,
	PM_QOS_BW_THROUGHPUT_MAX,
	PM_QOS_DSU_THROUGHPUT,
	PM_QOS_DSU_THROUGHPUT_MAX,
	PM_QOS_BCI_THROUGHPUT,
	PM_QOS_BCI_THROUGHPUT_MAX,
	PM_QOS_GPU_FREQ_MIN,
	PM_QOS_GPU_FREQ_MAX,
	PM_QOS_TPU_FREQ_MIN,
	PM_QOS_TPU_FREQ_MAX,
	EXYNOS_PM_QOS_NUM_CLASSES,
};

enum exynos_pm_qos_flags_status {
	EXYNOS_PM_QOS_FLAGS_UNDEFINED = -1,
	EXYNOS_PM_QOS_FLAGS_NONE,
	EXYNOS_PM_QOS_FLAGS_SOME,
	EXYNOS_PM_QOS_FLAGS_ALL,
};

#define EXYNOS_PM_QOS_DEFAULT_VALUE	(-1)

#define PM_QOS_DEVICE_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_INTCAM_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_DEVICE_THROUGHPUT_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_INTCAM_THROUGHPUT_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_BUS_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_DISPLAY_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_DISPLAY_THROUGHPUT_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_CAM_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_CAM_THROUGHPUT_MAX_DEFAULT_VALUE		INT_MAX
#define PM_QOS_MFC_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_MFC_THROUGHPUT_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_TNR_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_TNR_THROUGHPUT_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_BW_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_BW_THROUGHPUT_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_DSU_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_DSU_THROUGHPUT_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_BCI_THROUGHPUT_DEFAULT_VALUE	0
#define PM_QOS_BCI_THROUGHPUT_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_CLUSTER0_FREQ_MIN_DEFAULT_VALUE	0
#define PM_QOS_CLUSTER0_FREQ_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_CLUSTER1_FREQ_MIN_DEFAULT_VALUE	0
#define PM_QOS_CLUSTER1_FREQ_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_CLUSTER2_FREQ_MIN_DEFAULT_VALUE	0
#define PM_QOS_CLUSTER2_FREQ_MAX_DEFAULT_VALUE	INT_MAX
#define PM_QOS_GPU_FREQ_MIN_DEFAULT_VALUE	0
#define PM_QOS_GPU_FREQ_MAX_DEFAULT_VALUE	INT_MAX

struct exynos_pm_asynchronous_vote {
    struct work_struct work;
    s32 target_freq;
};

struct exynos_pm_qos_request {
	struct plist_node node;
	int exynos_pm_qos_class;
	struct delayed_work work; /* for exynos_pm_qos_update_request_timeout */
	const char *func;
	unsigned int line;
	struct exynos_pm_asynchronous_vote async_vote;
};

struct exynos_pm_qos_flags_request {
	struct list_head node;
	s32 flags;	/* Do not change to 64 bit */
};

enum exynos_pm_qos_type {
	EXYNOS_PM_QOS_UNINITIALIZED,
	EXYNOS_PM_QOS_MAX,		/* return the largest value */
	EXYNOS_PM_QOS_MIN,		/* return the smallest value */
	EXYNOS_PM_QOS_SUM		/* return the sum */
};

/*
 * Note: The lockless read path depends on the CPU accessing target_value
 * or effective_flags atomically.  Atomic access is only guaranteed on all CPU
 * types linux supports for 32 bit quantites
 */
struct exynos_pm_qos_constraints {
	struct plist_head list;
	s32 target_value;	/* Do not change to 64 bit */
	s32 default_value;
	s32 no_constraint_value;
	enum exynos_pm_qos_type type;
	struct blocking_notifier_head *notifiers;
	spinlock_t lock;	/* protect plist */
};

struct exynos_pm_qos_flags {
	struct list_head list;
	s32 effective_flags;	/* Do not change to 64 bit */
};

/* Action requested to exynos_pm_qos_update_target */
enum exynos_pm_qos_req_action {
	EXYNOS_PM_QOS_ADD_REQ,		/* Add a new request */
	EXYNOS_PM_QOS_UPDATE_REQ,	/* Update an existing request */
	EXYNOS_PM_QOS_REMOVE_REQ	/* Remove an existing request */
};

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS) || IS_ENABLED(CONFIG_EXYNOS_PM_QOS_MODULE)
#define exynos_pm_qos_add_request(arg...)					\
	exynos_pm_qos_add_request_trace(__func__, __LINE__, ##arg)

int exynos_pm_qos_update_target(struct exynos_pm_qos_constraints *c, struct plist_node *node,
				enum exynos_pm_qos_req_action action, int value);
bool exynos_pm_qos_update_flags(struct exynos_pm_qos_flags *pqf,
				struct exynos_pm_qos_flags_request *req,
				enum exynos_pm_qos_req_action action, s32 val);
void exynos_pm_qos_add_request_trace(const char *func, unsigned int line,
				     struct exynos_pm_qos_request *req,
				     int exynos_pm_qos_class,
				     s32 value);
void exynos_pm_qos_update_request(struct exynos_pm_qos_request *req,
				  s32 new_value);
void exynos_pm_qos_update_request_async(struct exynos_pm_qos_request *req,
					s32 new_value);
void exynos_pm_qos_update_request_timeout(struct exynos_pm_qos_request *req,
					  s32 new_value, unsigned long timeout_us);
void exynos_pm_qos_remove_request(struct exynos_pm_qos_request *req);

int exynos_pm_qos_request(int exynos_pm_qos_class);
int exynos_pm_qos_add_notifier(int exynos_pm_qos_class, struct notifier_block *notifier);
int exynos_pm_qos_remove_notifier(int exynos_pm_qos_class, struct notifier_block *notifier);
int exynos_pm_qos_request_active(struct exynos_pm_qos_request *req);
s32 exynos_pm_qos_read_value(struct exynos_pm_qos_constraints *c);
int exynos_pm_qos_read_req_value(int pm_qos_class, struct exynos_pm_qos_request *req);
#else
#define exynos_pm_qos_add_request(arg...)
#define exynos_pm_qos_remove_request(a)
#define exynos_pm_qos_update_request(a, b)
#endif
#endif
