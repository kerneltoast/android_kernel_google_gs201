// SPDX-License-Identifier: GPL-2.0-only
/*
 * This module exposes the interface to kernel space for specifying
 * QoS dependencies.  It provides infrastructure for registration of:
 *
 * Dependents on a QoS value : register requests
 * Watchers of QoS value : get notified when target QoS value changes
 *
 * This QoS design is best effort based.  Dependents register their QoS needs.
 * Watchers register to keep track of the current QoS needs of the system.
 *
 * There are 3 basic classes of QoS parameter: latency, timeout, throughput
 * each have defined units:
 * latency: usec
 * timeout: usec <-- currently not used.
 * throughput: kbs (kilo byte / sec)
 *
 * There are lists of exynos_pm_qos_objects each one wrapping requests, notifiers
 *
 * User mode requests on a QOS parameter register themselves to the
 * subsystem by opening the device node /dev/... and writing there request to
 * the node.  As long as the process holds a file handle open to the node the
 * client continues to be accounted for.  Upon file release the usermode
 * request is removed and a new qos target is computed.  This way when the
 * request that the application has is cleaned up when closes the file
 * pointer or exits the exynos_pm_qos_object will get an opportunity to clean up.
 *
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/rwlock.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/plist.h>

#include <linux/uaccess.h>
#include <linux/export.h>
#include <trace/events/power.h>

#include <soc/google/exynos_pm_qos.h>

# define plist_check_head(h)	do { } while (0)
/**
 * plist_add - add @node to @head
 *
 * @node:	&struct plist_node pointer
 * @head:	&struct plist_head pointer
 */
void exynos_plist_add(struct plist_node *node, struct plist_head *head)
{
	struct plist_node *first, *iter, *prev = NULL;
	struct list_head *node_next = &head->node_list;

	plist_check_head(head);
	WARN_ON(!plist_node_empty(node));
	WARN_ON(!list_empty(&node->prio_list));

	if (plist_head_empty(head))
		goto ins_node;

	iter = plist_first(head);
	first = iter;

	do {
		if (node->prio < iter->prio) {
			node_next = &iter->node_list;
			break;
		}

		prev = iter;
		iter = list_entry(iter->prio_list.next,
				  struct plist_node, prio_list);
	} while (iter != first);

	if (!prev || prev->prio != node->prio)
		list_add_tail(&node->prio_list, &iter->prio_list);
ins_node:
	list_add_tail(&node->node_list, node_next);

	plist_check_head(head);
}

/**
 * plist_del - Remove a @node from plist.
 *
 * @node:	&struct plist_node pointer - entry to be removed
 * @head:	&struct plist_head pointer - list head
 */
void exynos_plist_del(struct plist_node *node, struct plist_head *head)
{
	plist_check_head(head);

	if (!list_empty(&node->prio_list)) {
		if (node->node_list.next != &head->node_list) {
			struct plist_node *next;

			next = list_entry(node->node_list.next,
					  struct plist_node, node_list);

			/* add the next plist_node into prio_list */
			if (list_empty(&next->prio_list))
				list_add(&next->prio_list, &node->prio_list);
		}
		list_del_init(&node->prio_list);
	}

	list_del_init(&node->node_list);

	plist_check_head(head);
}

/*
 * locking rule: all changes to constraints or notifiers lists
 * or exynos_pm_qos_object list and exynos_pm_qos_objects need to happen with exynos_pm_qos_lock
 * held, taken with _irqsave.  One lock to rule them all
 */
struct exynos_pm_qos_object {
	struct exynos_pm_qos_constraints *constraints;
	char *name;
};

static DEFINE_RWLOCK(exynos_pm_qos_lock);

static struct exynos_pm_qos_object null_exynos_pm_qos;

static BLOCKING_NOTIFIER_HEAD(device_throughput_notifier);
static struct exynos_pm_qos_constraints device_tput_constraints = {
	.list = PLIST_HEAD_INIT(device_tput_constraints.list),
	.target_value = PM_QOS_DEVICE_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_DEVICE_THROUGHPUT_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &device_throughput_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(device_tput_constraints.lock),
};

static struct exynos_pm_qos_object device_throughput_pm_qos = {
	.constraints = &device_tput_constraints,
	.name = "device_throughput",
};

static BLOCKING_NOTIFIER_HEAD(device_throughput_max_notifier);
static struct exynos_pm_qos_constraints device_tput_max_constraints = {
	.list = PLIST_HEAD_INIT(device_tput_max_constraints.list),
	.target_value = PM_QOS_DEVICE_THROUGHPUT_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_DEVICE_THROUGHPUT_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &device_throughput_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(device_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object device_throughput_max_pm_qos = {
	.constraints = &device_tput_max_constraints,
	.name = "device_throughput_max",
};

static BLOCKING_NOTIFIER_HEAD(intcam_throughput_notifier);
static struct exynos_pm_qos_constraints intcam_tput_constraints = {
	.list = PLIST_HEAD_INIT(intcam_tput_constraints.list),
	.target_value = PM_QOS_INTCAM_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_INTCAM_THROUGHPUT_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &intcam_throughput_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(intcam_tput_constraints.lock),
};

static struct exynos_pm_qos_object intcam_throughput_pm_qos = {
	.constraints = &intcam_tput_constraints,
	.name = "intcam_throughput",
};

static BLOCKING_NOTIFIER_HEAD(intcam_throughput_max_notifier);
static struct exynos_pm_qos_constraints intcam_tput_max_constraints = {
	.list = PLIST_HEAD_INIT(intcam_tput_max_constraints.list),
	.target_value = PM_QOS_INTCAM_THROUGHPUT_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_INTCAM_THROUGHPUT_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &intcam_throughput_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(intcam_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object intcam_throughput_max_pm_qos = {
	.constraints = &intcam_tput_max_constraints,
	.name = "intcam_throughput_max",
};

static BLOCKING_NOTIFIER_HEAD(bus_throughput_notifier);
static struct exynos_pm_qos_constraints bus_tput_constraints = {
	.list = PLIST_HEAD_INIT(bus_tput_constraints.list),
	.target_value = PM_QOS_BUS_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_BUS_THROUGHPUT_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &bus_throughput_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(bus_tput_constraints.lock),
};

static struct exynos_pm_qos_object bus_throughput_pm_qos = {
	.constraints = &bus_tput_constraints,
	.name = "bus_throughput",
};

static BLOCKING_NOTIFIER_HEAD(bus_throughput_max_notifier);
static struct exynos_pm_qos_constraints bus_tput_max_constraints = {
	.list = PLIST_HEAD_INIT(bus_tput_max_constraints.list),
	.target_value = PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &bus_throughput_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(bus_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object bus_throughput_max_pm_qos = {
	.constraints = &bus_tput_max_constraints,
	.name = "bus_throughput_max",
};

static BLOCKING_NOTIFIER_HEAD(cluster2_freq_min_notifier);
static struct exynos_pm_qos_constraints cluster2_freq_min_constraints = {
	.list = PLIST_HEAD_INIT(cluster2_freq_min_constraints.list),
	.target_value = PM_QOS_CLUSTER2_FREQ_MIN_DEFAULT_VALUE,
	.default_value = PM_QOS_CLUSTER2_FREQ_MIN_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &cluster2_freq_min_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(cluster2_freq_min_constraints.lock),
};

static struct exynos_pm_qos_object cluster2_freq_min_pm_qos = {
	.constraints = &cluster2_freq_min_constraints,
	.name = "cluster2_freq_min",
};

static BLOCKING_NOTIFIER_HEAD(cluster2_freq_max_notifier);
static struct exynos_pm_qos_constraints cluster2_freq_max_constraints = {
	.list = PLIST_HEAD_INIT(cluster2_freq_max_constraints.list),
	.target_value = PM_QOS_CLUSTER2_FREQ_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_CLUSTER2_FREQ_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &cluster2_freq_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(cluster2_freq_max_constraints.lock),
};

static struct exynos_pm_qos_object cluster2_freq_max_pm_qos = {
	.constraints = &cluster2_freq_max_constraints,
	.name = "cluster2_freq_max",
};

static BLOCKING_NOTIFIER_HEAD(cluster1_freq_min_notifier);
static struct exynos_pm_qos_constraints cluster1_freq_min_constraints = {
	.list = PLIST_HEAD_INIT(cluster1_freq_min_constraints.list),
	.target_value = PM_QOS_CLUSTER1_FREQ_MIN_DEFAULT_VALUE,
	.default_value = PM_QOS_CLUSTER1_FREQ_MIN_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &cluster1_freq_min_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(cluster1_freq_min_constraints.lock),
};

static struct exynos_pm_qos_object cluster1_freq_min_pm_qos = {
	.constraints = &cluster1_freq_min_constraints,
	.name = "cluster1_freq_min",
};

static BLOCKING_NOTIFIER_HEAD(cluster1_freq_max_notifier);
static struct exynos_pm_qos_constraints cluster1_freq_max_constraints = {
	.list = PLIST_HEAD_INIT(cluster1_freq_max_constraints.list),
	.target_value = PM_QOS_CLUSTER1_FREQ_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_CLUSTER1_FREQ_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &cluster1_freq_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(cluster1_freq_max_constraints.lock),
};

static struct exynos_pm_qos_object cluster1_freq_max_pm_qos = {
	.constraints = &cluster1_freq_max_constraints,
	.name = "cluster1_freq_max",
};

static BLOCKING_NOTIFIER_HEAD(cluster0_freq_min_notifier);
static struct exynos_pm_qos_constraints cluster0_freq_min_constraints = {
	.list = PLIST_HEAD_INIT(cluster0_freq_min_constraints.list),
	.target_value = PM_QOS_CLUSTER0_FREQ_MIN_DEFAULT_VALUE,
	.default_value = PM_QOS_CLUSTER0_FREQ_MIN_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &cluster0_freq_min_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(cluster0_freq_min_constraints.lock),
};

static struct exynos_pm_qos_object cluster0_freq_min_pm_qos = {
	.constraints = &cluster0_freq_min_constraints,
	.name = "cluster0_freq_min",
};

static BLOCKING_NOTIFIER_HEAD(cluster0_freq_max_notifier);
static struct exynos_pm_qos_constraints cluster0_freq_max_constraints = {
	.list = PLIST_HEAD_INIT(cluster0_freq_max_constraints.list),
	.target_value = PM_QOS_CLUSTER0_FREQ_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_CLUSTER0_FREQ_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &cluster0_freq_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(cluster0_freq_max_constraints.lock),
};

static struct exynos_pm_qos_object cluster0_freq_max_pm_qos = {
	.constraints = &cluster0_freq_max_constraints,
	.name = "cluster0_freq_max",
};

static BLOCKING_NOTIFIER_HEAD(display_throughput_notifier);
static struct exynos_pm_qos_constraints display_tput_constraints = {
	.list = PLIST_HEAD_INIT(display_tput_constraints.list),
	.target_value = PM_QOS_DISPLAY_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_DISPLAY_THROUGHPUT_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &display_throughput_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(display_tput_constraints.lock),
};

static struct exynos_pm_qos_object display_throughput_pm_qos = {
	.constraints = &display_tput_constraints,
	.name = "display_throughput",
};

static BLOCKING_NOTIFIER_HEAD(display_throughput_max_notifier);
static struct exynos_pm_qos_constraints display_tput_max_constraints = {
	.list = PLIST_HEAD_INIT(display_tput_max_constraints.list),
	.target_value = PM_QOS_DISPLAY_THROUGHPUT_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_DISPLAY_THROUGHPUT_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &display_throughput_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(display_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object display_throughput_max_pm_qos = {
	.constraints = &display_tput_max_constraints,
	.name = "display_throughput_max",
};

static BLOCKING_NOTIFIER_HEAD(cam_throughput_notifier);
static struct exynos_pm_qos_constraints cam_tput_constraints = {
	.list = PLIST_HEAD_INIT(cam_tput_constraints.list),
	.target_value = PM_QOS_CAM_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_CAM_THROUGHPUT_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &cam_throughput_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(cam_tput_constraints.lock),
};

static struct exynos_pm_qos_object cam_throughput_pm_qos = {
	.constraints = &cam_tput_constraints,
	.name = "cam_throughput",
};

static BLOCKING_NOTIFIER_HEAD(cam_throughput_max_notifier);
static struct exynos_pm_qos_constraints cam_tput_max_constraints = {
	.list = PLIST_HEAD_INIT(cam_tput_max_constraints.list),
	.target_value = PM_QOS_CAM_THROUGHPUT_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_CAM_THROUGHPUT_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &cam_throughput_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(cam_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object cam_throughput_max_pm_qos = {
	.constraints = &cam_tput_max_constraints,
	.name = "cam_throughput_max",
};

static BLOCKING_NOTIFIER_HEAD(mfc_throughput_notifier);
static struct exynos_pm_qos_constraints mfc_tput_constraints = {
	.list = PLIST_HEAD_INIT(mfc_tput_constraints.list),
	.target_value = PM_QOS_MFC_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_MFC_THROUGHPUT_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &mfc_throughput_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(mfc_tput_constraints.lock),
};

static struct exynos_pm_qos_object mfc_throughput_pm_qos = {
	.constraints = &mfc_tput_constraints,
	.name = "mfc_throughput",
};

static BLOCKING_NOTIFIER_HEAD(gpu_freq_min_notifier);
static struct exynos_pm_qos_constraints gpu_freq_min_constraints = {
	.list = PLIST_HEAD_INIT(gpu_freq_min_constraints.list),
	.target_value = PM_QOS_GPU_FREQ_MIN_DEFAULT_VALUE,
	.default_value = PM_QOS_GPU_FREQ_MIN_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &gpu_freq_min_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(gpu_freq_min_constraints.lock),
};

static struct exynos_pm_qos_object gpu_freq_min_pm_qos = {
	.constraints = &gpu_freq_min_constraints,
	.name = "gpu_freq_min",
};

static BLOCKING_NOTIFIER_HEAD(gpu_freq_max_notifier);
static struct exynos_pm_qos_constraints gpu_freq_max_constraints = {
	.list = PLIST_HEAD_INIT(gpu_freq_max_constraints.list),
	.target_value = PM_QOS_GPU_FREQ_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_GPU_FREQ_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &gpu_freq_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(gpu_freq_max_constraints.lock),
};

static struct exynos_pm_qos_object gpu_freq_max_pm_qos = {
	.constraints = &gpu_freq_max_constraints,
	.name = "gpu_freq_max",
};

static BLOCKING_NOTIFIER_HEAD(mfc_throughput_max_notifier);
static struct exynos_pm_qos_constraints mfc_tput_max_constraints = {
	.list = PLIST_HEAD_INIT(mfc_tput_max_constraints.list),
	.target_value = PM_QOS_MFC_THROUGHPUT_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_MFC_THROUGHPUT_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &mfc_throughput_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(mfc_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object mfc_throughput_max_pm_qos = {
	.constraints = &mfc_tput_max_constraints,
	.name = "mfc_throughput_max",
};

static BLOCKING_NOTIFIER_HEAD(tnr_throughput_notifier);
static struct exynos_pm_qos_constraints tnr_tput_constraints = {
	.list = PLIST_HEAD_INIT(tnr_tput_constraints.list),
	.target_value = PM_QOS_TNR_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_TNR_THROUGHPUT_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &tnr_throughput_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(tnr_tput_constraints.lock),
};

static struct exynos_pm_qos_object tnr_throughput_pm_qos = {
	.constraints = &tnr_tput_constraints,
	.name = "tnr_throughput",
};

static BLOCKING_NOTIFIER_HEAD(tnr_throughput_max_notifier);
static struct exynos_pm_qos_constraints tnr_tput_max_constraints = {
	.list = PLIST_HEAD_INIT(tnr_tput_max_constraints.list),
	.target_value = PM_QOS_TNR_THROUGHPUT_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_TNR_THROUGHPUT_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &tnr_throughput_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(tnr_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object tnr_throughput_max_pm_qos = {
	.constraints = &tnr_tput_max_constraints,
	.name = "tnr_throughput_max",
};

static BLOCKING_NOTIFIER_HEAD(bo_throughput_notifier);
static struct exynos_pm_qos_constraints bo_tput_constraints = {
	.list = PLIST_HEAD_INIT(bo_tput_constraints.list),
	.target_value = PM_QOS_BO_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_BO_THROUGHPUT_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &bo_throughput_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(bo_tput_constraints.lock),
};

static struct exynos_pm_qos_object bo_throughput_pm_qos = {
	.constraints = &bo_tput_constraints,
	.name = "bo_throughput",
};

static BLOCKING_NOTIFIER_HEAD(bo_throughput_max_notifier);
static struct exynos_pm_qos_constraints bo_tput_max_constraints = {
	.list = PLIST_HEAD_INIT(bo_tput_max_constraints.list),
	.target_value = PM_QOS_BO_THROUGHPUT_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_BO_THROUGHPUT_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &bo_throughput_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(bo_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object bo_throughput_max_pm_qos = {
	.constraints = &bo_tput_max_constraints,
	.name = "bo_throughput_max",
};

static struct exynos_pm_qos_object *exynos_pm_qos_array[] = {
	&null_exynos_pm_qos,
	&cluster0_freq_min_pm_qos,
	&cluster0_freq_max_pm_qos,
	&cluster1_freq_min_pm_qos,
	&cluster1_freq_max_pm_qos,
	&cluster2_freq_min_pm_qos,
	&cluster2_freq_max_pm_qos,
	&device_throughput_pm_qos,
	&intcam_throughput_pm_qos,
	&device_throughput_max_pm_qos,
	&intcam_throughput_max_pm_qos,
	&bus_throughput_pm_qos,
	&bus_throughput_max_pm_qos,
	&display_throughput_pm_qos,
	&display_throughput_max_pm_qos,
	&cam_throughput_pm_qos,
	&cam_throughput_max_pm_qos,
	&mfc_throughput_pm_qos,
	&mfc_throughput_max_pm_qos,
	&tnr_throughput_pm_qos,
	&tnr_throughput_max_pm_qos,
	&bo_throughput_pm_qos,
	&bo_throughput_max_pm_qos,
	&gpu_freq_min_pm_qos,
	&gpu_freq_max_pm_qos,
};

/* unlocked internal variant */
static inline int exynos_pm_qos_get_value(struct exynos_pm_qos_constraints *c)
{
	struct plist_node *node;
	int total_value = 0;

	if (plist_head_empty(&c->list))
		return c->no_constraint_value;

	switch (c->type) {
	case EXYNOS_PM_QOS_MIN:
		return plist_first(&c->list)->prio;

	case EXYNOS_PM_QOS_MAX:
		return plist_last(&c->list)->prio;

	case EXYNOS_PM_QOS_SUM:
		plist_for_each(node, &c->list)
			total_value += node->prio;

		return total_value;

	default:
		/* runtime check for not using enum */
		WARN(1, "Unexpected PM_QOS type\n");
		return EXYNOS_PM_QOS_DEFAULT_VALUE;
	}
}

s32 exynos_pm_qos_read_value(struct exynos_pm_qos_constraints *c)
{
	return c->target_value;
}

/*
 * pm_qos_read_req_value - returns requested qos value
 * @pm_qos_class: identification of which qos value is requested
 * @req: request wanted to find set value
 *
 * This function returns the requested qos value by sysfs node.
 */
int exynos_pm_qos_read_req_value(int pm_qos_class, struct exynos_pm_qos_request *req)
{
	struct plist_node *p;
	unsigned long flags;

	read_lock_irqsave(&exynos_pm_qos_lock, flags);

	spin_lock(&exynos_pm_qos_array[pm_qos_class]->constraints->lock);
	plist_for_each(p, &exynos_pm_qos_array[pm_qos_class]->constraints->list) {
		if (req == container_of(p, struct exynos_pm_qos_request, node)) {
			spin_unlock(&exynos_pm_qos_array[pm_qos_class]->constraints->lock);
			read_unlock_irqrestore(&exynos_pm_qos_lock, flags);
			return p->prio;
		}
	}
	spin_unlock(&exynos_pm_qos_array[pm_qos_class]->constraints->lock);

	read_unlock_irqrestore(&exynos_pm_qos_lock, flags);

	return -ENODATA;
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_read_req_value);

static inline void exynos_pm_qos_set_value(struct exynos_pm_qos_constraints *c, s32 value)
{
	c->target_value = value;
}

static int exynos_pm_qos_debug_show(struct seq_file *s, void *unused)
{
	struct exynos_pm_qos_object *qos = (struct exynos_pm_qos_object *)s->private;
	struct exynos_pm_qos_constraints *c;
	struct exynos_pm_qos_request *req;
	char *type;
	unsigned long flags;
	int tot_reqs = 0;
	int active_reqs = 0;

	if (IS_ERR_OR_NULL(qos)) {
		pr_err("%s: bad qos param!\n", __func__);
		return -EINVAL;
	}
	c = qos->constraints;
	if (IS_ERR_OR_NULL(c)) {
		pr_err("%s: Bad constraints on qos?\n", __func__);
		return -EINVAL;
	}

	/* Lock to ensure we have a snapshot */
	write_lock_irqsave(&exynos_pm_qos_lock, flags);
	if (plist_head_empty(&c->list)) {
		seq_puts(s, "Empty!\n");
		goto out;
	}

	switch (c->type) {
	case EXYNOS_PM_QOS_MIN:
		type = "Minimum";
		break;
	case EXYNOS_PM_QOS_MAX:
		type = "Maximum";
		break;
	case EXYNOS_PM_QOS_SUM:
		type = "Sum";
		break;
	default:
		type = "Unknown";
	}

	plist_for_each_entry(req, &c->list, node) {
		char *state = "Default";

		if (req->node.prio != c->default_value) {
			active_reqs++;
			state = "Active";
		}
		tot_reqs++;
		seq_printf(s, "%d: %d: %s(%s:%d)\n", tot_reqs,
			   (req->node).prio, state,
			   req->func,
			   req->line);
	}

	seq_printf(s, "Type=%s, Value=%d, Requests: active=%d / total=%d\n",
		   type, exynos_pm_qos_get_value(c), active_reqs, tot_reqs);

out:
	write_unlock_irqrestore(&exynos_pm_qos_lock, flags);
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(exynos_pm_qos_debug);

/**
 * exynos_pm_qos_update_target - manages the constraints list and calls the notifiers
 *  if needed
 * @c: constraints data struct
 * @node: request to add to the list, to update or to remove
 * @action: action to take on the constraints list
 * @value: value of the request to add or update
 *
 * This function returns 1 if the aggregated constraint value has changed, 0
 *  otherwise.
 */
int exynos_pm_qos_update_target(struct exynos_pm_qos_constraints *c, struct plist_node *node,
				enum exynos_pm_qos_req_action action, int value)
{
	unsigned long flags;
	int prev_value, curr_value, new_value;
	int ret;

	read_lock_irqsave(&exynos_pm_qos_lock, flags);
	spin_lock(&c->lock);
	prev_value = exynos_pm_qos_get_value(c);
	if (value == EXYNOS_PM_QOS_DEFAULT_VALUE)
		new_value = c->default_value;
	else
		new_value = value;

	switch (action) {
	case EXYNOS_PM_QOS_REMOVE_REQ:
		exynos_plist_del(node, &c->list);
		break;
	case EXYNOS_PM_QOS_UPDATE_REQ:
		/*
		 * to change the list, we atomically remove, reinit
		 * with new value and add, then see if the extremal
		 * changed
		 */
		exynos_plist_del(node, &c->list);
		fallthrough;
	case EXYNOS_PM_QOS_ADD_REQ:
		plist_node_init(node, new_value);
		exynos_plist_add(node, &c->list);
		break;
	default:
		/* no action */
		break;
	}

	curr_value = exynos_pm_qos_get_value(c);
	exynos_pm_qos_set_value(c, curr_value);

	spin_unlock(&c->lock);
	read_unlock_irqrestore(&exynos_pm_qos_lock, flags);

	if (prev_value != curr_value) {
		ret = 1;
		if (c->notifiers)
			blocking_notifier_call_chain(c->notifiers,
						     (unsigned long)curr_value,
						     NULL);
	} else {
		ret = 0;
	}
	return ret;
}

/**
 * exynos_pm_qos_flags_remove_req - Remove device PM QoS flags request.
 * @pqf: Device PM QoS flags set to remove the request from.
 * @req: Request to remove from the set.
 */
static void exynos_pm_qos_flags_remove_req(struct exynos_pm_qos_flags *pqf,
					   struct exynos_pm_qos_flags_request *req)
{
	s32 val = 0;

	list_del(&req->node);
	list_for_each_entry(req, &pqf->list, node)
		val |= req->flags;

	pqf->effective_flags = val;
}

/**
 * exynos_pm_qos_update_flags - Update a set of PM QoS flags.
 * @pqf: Set of flags to update.
 * @req: Request to add to the set, to modify, or to remove from the set.
 * @action: Action to take on the set.
 * @val: Value of the request to add or modify.
 *
 * Update the given set of PM QoS flags and call notifiers if the aggregate
 * value has changed.  Returns 1 if the aggregate constraint value has changed,
 * 0 otherwise.
 */
bool exynos_pm_qos_update_flags(struct exynos_pm_qos_flags *pqf,
				struct exynos_pm_qos_flags_request *req,
				enum exynos_pm_qos_req_action action, s32 val)
{
	unsigned long irqflags;
	s32 prev_value, curr_value;

	read_lock_irqsave(&exynos_pm_qos_lock, irqflags);

	prev_value = list_empty(&pqf->list) ? 0 : pqf->effective_flags;

	switch (action) {
	case EXYNOS_PM_QOS_REMOVE_REQ:
		exynos_pm_qos_flags_remove_req(pqf, req);
		break;
	case EXYNOS_PM_QOS_UPDATE_REQ:
		exynos_pm_qos_flags_remove_req(pqf, req);
		fallthrough;
	case EXYNOS_PM_QOS_ADD_REQ:
		req->flags = val;
		INIT_LIST_HEAD(&req->node);
		list_add_tail(&req->node, &pqf->list);
		pqf->effective_flags |= val;
		break;
	default:
		break;
	}

	curr_value = list_empty(&pqf->list) ? 0 : pqf->effective_flags;

	read_unlock_irqrestore(&exynos_pm_qos_lock, irqflags);

	return prev_value != curr_value;
}

/**
 * exynos_pm_qos_request - returns current system wide qos expectation
 * @exynos_pm_qos_class: identification of which qos value is requested
 *
 * This function returns the current target value.
 */
int exynos_pm_qos_request(int exynos_pm_qos_class)
{
	return exynos_pm_qos_read_value(exynos_pm_qos_array[exynos_pm_qos_class]->constraints);
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_request);

int exynos_pm_qos_request_active(struct exynos_pm_qos_request *req)
{
	return req->exynos_pm_qos_class != 0;
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_request_active);

static void __exynos_pm_qos_update_request(struct exynos_pm_qos_request *req,
					   s32 new_value)
{
	int class;

	if (!req)
		return;

	class = req->exynos_pm_qos_class;

	if (new_value != req->node.prio)
		exynos_pm_qos_update_target(exynos_pm_qos_array[class]->constraints,
					    &req->node, EXYNOS_PM_QOS_UPDATE_REQ, new_value);
	trace_clock_set_rate(exynos_pm_qos_array[class]->name,
			exynos_pm_qos_request(class), raw_smp_processor_id());
}

/**
 * exynos_pm_qos_work_fn - the timeout handler of exynos_pm_qos_update_request_timeout
 * @work: work struct for the delayed work (timeout)
 *
 * This cancels the timeout request by falling back to the default at timeout.
 */
static void exynos_pm_qos_work_fn(struct work_struct *work)
{
	struct exynos_pm_qos_request *req = container_of(to_delayed_work(work),
						  struct exynos_pm_qos_request,
						  work);

	__exynos_pm_qos_update_request(req, EXYNOS_PM_QOS_DEFAULT_VALUE);
}

/**
 * exynos_pm_qos_add_request_trace - inserts new qos request into the list
 * @req: pointer to a preallocated handle
 * @exynos_pm_qos_class: identifies which list of qos request to use
 * @value: defines the qos request
 *
 * This function inserts a new entry in the exynos_pm_qos_class list of requested qos
 * performance characteristics.  It recomputes the aggregate QoS expectations
 * for the exynos_pm_qos_class of parameters and initializes the exynos_pm_qos_request
 * handle.  Caller needs to save this handle for later use in updates and
 * removal.
 */

void exynos_pm_qos_add_request_trace(const char *func, unsigned int line,
				     struct exynos_pm_qos_request *req,
				     int exynos_pm_qos_class, s32 value)
{
	if (!req) /*guard against callers passing in null */
		return;

	if (exynos_pm_qos_request_active(req)) {
		WARN(1, "%s called for already added request\n", __func__);
		return;
	}
	req->exynos_pm_qos_class = exynos_pm_qos_class;
	req->func = func;
	req->line = line;
	INIT_DELAYED_WORK(&req->work, exynos_pm_qos_work_fn);
	exynos_pm_qos_update_target(exynos_pm_qos_array[exynos_pm_qos_class]->constraints,
				    &req->node, EXYNOS_PM_QOS_ADD_REQ, value);
	trace_clock_set_rate(exynos_pm_qos_array[exynos_pm_qos_class]->name,
			exynos_pm_qos_request(exynos_pm_qos_class), raw_smp_processor_id());
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_add_request_trace);

/**
 * exynos_pm_qos_update_request - modifies an existing qos request
 * @req : handle to list element holding a exynos_pm_qos request to use
 * @value: defines the qos request
 *
 * Updates an existing qos request for the exynos_pm_qos_class of parameters along
 * with updating the target exynos_pm_qos_class value.
 *
 * Attempts are made to make this code callable on hot code paths.
 */
void exynos_pm_qos_update_request(struct exynos_pm_qos_request *req,
				  s32 new_value)
{
	if (!req) /*guard against callers passing in null */
		return;

	if (!exynos_pm_qos_request_active(req)) {
		WARN(1, "%s called for unknown object\n", __func__);
		return;
	}

	cancel_delayed_work_sync(&req->work);
	__exynos_pm_qos_update_request(req, new_value);
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_update_request);

/**
 * exynos_pm_qos_update_request_timeout - modifies an existing qos request temporarily.
 * @req : handle to list element holding a exynos_pm_qos request to use
 * @new_value: defines the temporal qos request
 * @timeout_us: the effective duration of this qos request in usecs.
 *
 * After timeout_us, this qos request is cancelled automatically.
 */
void exynos_pm_qos_update_request_timeout(struct exynos_pm_qos_request *req, s32 new_value,
					  unsigned long timeout_us)
{
	int class;

	if (!req)
		return;
	if (WARN(!exynos_pm_qos_request_active(req),
		 "%s called for unknown object.", __func__))
		return;

	cancel_delayed_work_sync(&req->work);

	class = req->exynos_pm_qos_class;
	if (new_value != req->node.prio)
		exynos_pm_qos_update_target(exynos_pm_qos_array[class]->constraints,
					    &req->node, EXYNOS_PM_QOS_UPDATE_REQ, new_value);

	schedule_delayed_work(&req->work, usecs_to_jiffies(timeout_us));
	trace_clock_set_rate(exynos_pm_qos_array[class]->name,
			exynos_pm_qos_request(class), raw_smp_processor_id());
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_update_request_timeout);

/**
 * exynos_pm_qos_remove_request - modifies an existing qos request
 * @req: handle to request list element
 *
 * Will remove pm qos request from the list of constraints and
 * recompute the current target value for the exynos_pm_qos_class.  Call this
 * on slow code paths.
 */
void exynos_pm_qos_remove_request(struct exynos_pm_qos_request *req)
{
	int class;

	if (!req) /*guard against callers passing in null */
		return;
		/* silent return to keep pcm code cleaner */

	if (!exynos_pm_qos_request_active(req)) {
		WARN(1, "%s called for unknown object\n", __func__);
		return;
	}

	cancel_delayed_work_sync(&req->work);

	class = req->exynos_pm_qos_class;
	exynos_pm_qos_update_target(exynos_pm_qos_array[class]->constraints,
				    &req->node, EXYNOS_PM_QOS_REMOVE_REQ,
				    EXYNOS_PM_QOS_DEFAULT_VALUE);
	memset(req, 0, sizeof(*req));
	trace_clock_set_rate(exynos_pm_qos_array[class]->name,
			exynos_pm_qos_request(class), raw_smp_processor_id());
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_remove_request);

/**
 * exynos_pm_qos_add_notifier - sets notification entry for changes to target value
 * @exynos_pm_qos_class: identifies which qos target changes should be notified.
 * @notifier: notifier block managed by caller.
 *
 * will register the notifier into a notification chain that gets called
 * upon changes to the exynos_pm_qos_class target value.
 */
int exynos_pm_qos_add_notifier(int exynos_pm_qos_class, struct notifier_block *notifier)
{
	int retval;
	struct exynos_pm_qos_constraints *constraints;

	constraints = exynos_pm_qos_array[exynos_pm_qos_class]->constraints;

	retval = blocking_notifier_chain_register(constraints->notifiers,
						  notifier);

	return retval;
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_add_notifier);

/**
 * exynos_pm_qos_remove_notifier - deletes notification entry from chain.
 * @exynos_pm_qos_class: identifies which qos target changes are notified.
 * @notifier: notifier block to be removed.
 *
 * will remove the notifier from the notification chain that gets called
 * upon changes to the exynos_pm_qos_class target value.
 */
int exynos_pm_qos_remove_notifier(int exynos_pm_qos_class, struct notifier_block *notifier)
{
	int retval;
	struct exynos_pm_qos_constraints *constraints;

	constraints = exynos_pm_qos_array[exynos_pm_qos_class]->constraints;

	retval = blocking_notifier_chain_unregister(constraints->notifiers,
						    notifier);

	return retval;
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_remove_notifier);

static int exynos_pm_qos_power_init(void)
{
	int ret = 0;
	int i;
	struct dentry *d;

	BUILD_BUG_ON(ARRAY_SIZE(exynos_pm_qos_array) != EXYNOS_PM_QOS_NUM_CLASSES);

	d = debugfs_create_dir("exynos_pm_qos", NULL);

	for (i = PM_QOS_CLUSTER0_FREQ_MIN; i < EXYNOS_PM_QOS_NUM_CLASSES; i++) {
		debugfs_create_file(exynos_pm_qos_array[i]->name, 0444, d,
				    (void *)exynos_pm_qos_array[i],
				    &exynos_pm_qos_debug_fops);
	}

	return ret;
}
late_initcall(exynos_pm_qos_power_init);

MODULE_LICENSE("GPL");
