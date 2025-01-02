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
#include "cal-if/acpm_dvfs.h"

struct exynos_pm_qos_object {
	struct exynos_pm_qos_constraints *constraints;
	char *name;
};

static struct exynos_pm_qos_object null_exynos_pm_qos;

SRCU_NOTIFIER_HEAD_STATIC(device_throughput_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(device_throughput_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(intcam_throughput_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(intcam_throughput_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(bus_throughput_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(bus_throughput_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(cluster2_freq_min_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(cluster2_freq_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(cluster1_freq_min_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(cluster1_freq_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(cluster0_freq_min_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(cluster0_freq_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(display_throughput_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(display_throughput_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(cam_throughput_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(cam_throughput_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(mfc_throughput_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(gpu_freq_min_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(gpu_freq_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(mfc_throughput_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(tnr_throughput_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(tnr_throughput_max_notifier);
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

SRCU_NOTIFIER_HEAD_STATIC(bw_throughput_notifier);
static struct exynos_pm_qos_constraints bw_tput_constraints = {
	.list = PLIST_HEAD_INIT(bw_tput_constraints.list),
	.target_value = PM_QOS_BW_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_BW_THROUGHPUT_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &bw_throughput_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(bw_tput_constraints.lock),
};

static struct exynos_pm_qos_object bw_throughput_pm_qos = {
	.constraints = &bw_tput_constraints,
	.name = "bw_throughput",
};

SRCU_NOTIFIER_HEAD_STATIC(bw_throughput_max_notifier);
static struct exynos_pm_qos_constraints bw_tput_max_constraints = {
	.list = PLIST_HEAD_INIT(bw_tput_max_constraints.list),
	.target_value = PM_QOS_BW_THROUGHPUT_MAX_DEFAULT_VALUE,
	.default_value = PM_QOS_BW_THROUGHPUT_MAX_DEFAULT_VALUE,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &bw_throughput_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(bw_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object bw_throughput_max_pm_qos = {
	.constraints = &bw_tput_max_constraints,
	.name = "bw_throughput_max",
};

SRCU_NOTIFIER_HEAD_STATIC(dsu_throughput_notifier);
static struct exynos_pm_qos_constraints dsu_tput_constraints = {
        .list = PLIST_HEAD_INIT(dsu_tput_constraints.list),
        .target_value = PM_QOS_DSU_THROUGHPUT_DEFAULT_VALUE,
        .default_value = PM_QOS_DSU_THROUGHPUT_DEFAULT_VALUE,
        .type = EXYNOS_PM_QOS_MAX,
        .notifiers = &dsu_throughput_notifier,
        .lock = __SPIN_LOCK_UNLOCKED(dsu_tput_constraints.lock),
};

static struct exynos_pm_qos_object dsu_throughput_pm_qos = {
        .constraints = &dsu_tput_constraints,
        .name = "dsu_throughput",
};

SRCU_NOTIFIER_HEAD_STATIC(dsu_throughput_max_notifier);
static struct exynos_pm_qos_constraints dsu_tput_max_constraints = {
        .list = PLIST_HEAD_INIT(dsu_tput_max_constraints.list),
        .target_value = PM_QOS_DSU_THROUGHPUT_MAX_DEFAULT_VALUE,
        .default_value = PM_QOS_DSU_THROUGHPUT_MAX_DEFAULT_VALUE,
        .type = EXYNOS_PM_QOS_MIN,
        .notifiers = &dsu_throughput_max_notifier,
        .lock = __SPIN_LOCK_UNLOCKED(dsu_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object dsu_throughput_max_pm_qos = {
        .constraints = &dsu_tput_max_constraints,
        .name = "dsu_throughput_max",
};

SRCU_NOTIFIER_HEAD_STATIC(bci_throughput_notifier);
static struct exynos_pm_qos_constraints bci_tput_constraints = {
        .list = PLIST_HEAD_INIT(bci_tput_constraints.list),
        .target_value = PM_QOS_BCI_THROUGHPUT_DEFAULT_VALUE,
        .default_value = PM_QOS_BCI_THROUGHPUT_DEFAULT_VALUE,
        .type = EXYNOS_PM_QOS_MAX,
        .notifiers = &bci_throughput_notifier,
        .lock = __SPIN_LOCK_UNLOCKED(bci_tput_constraints.lock),
};

static struct exynos_pm_qos_object bci_throughput_pm_qos = {
        .constraints = &bci_tput_constraints,
        .name = "bci_throughput",
};

SRCU_NOTIFIER_HEAD_STATIC(bci_throughput_max_notifier);
static struct exynos_pm_qos_constraints bci_tput_max_constraints = {
        .list = PLIST_HEAD_INIT(bci_tput_max_constraints.list),
        .target_value = PM_QOS_BCI_THROUGHPUT_MAX_DEFAULT_VALUE,
        .default_value = PM_QOS_BCI_THROUGHPUT_MAX_DEFAULT_VALUE,
        .type = EXYNOS_PM_QOS_MIN,
        .notifiers = &bci_throughput_max_notifier,
        .lock = __SPIN_LOCK_UNLOCKED(bci_tput_max_constraints.lock),
};

static struct exynos_pm_qos_object bci_throughput_max_pm_qos = {
        .constraints = &bci_tput_max_constraints,
        .name = "bci_throughput_max",
};

SRCU_NOTIFIER_HEAD_STATIC(tpu_freq_min_notifier);
static struct exynos_pm_qos_constraints tpu_freq_min_constraints = {
	.list = PLIST_HEAD_INIT(tpu_freq_min_constraints.list),
	.target_value = 0,
	.default_value = 0,
	.type = EXYNOS_PM_QOS_MAX,
	.notifiers = &tpu_freq_min_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(tpu_freq_min_constraints.lock),
};

static struct exynos_pm_qos_object tpu_freq_min_pm_qos = {
	.constraints = &tpu_freq_min_constraints,
	.name = "tpu_freq_min",
};

SRCU_NOTIFIER_HEAD_STATIC(tpu_freq_max_notifier);
static struct exynos_pm_qos_constraints tpu_freq_max_constraints = {
	.list = PLIST_HEAD_INIT(tpu_freq_max_constraints.list),
	.target_value = INT_MAX,
	.default_value = INT_MAX,
	.type = EXYNOS_PM_QOS_MIN,
	.notifiers = &tpu_freq_max_notifier,
	.lock = __SPIN_LOCK_UNLOCKED(tpu_freq_max_constraints.lock),
};

static struct exynos_pm_qos_object tpu_freq_max_pm_qos = {
	.constraints = &tpu_freq_max_constraints,
	.name = "tpu_freq_max",
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
	&bw_throughput_pm_qos,
	&bw_throughput_max_pm_qos,
	&dsu_throughput_pm_qos,
	&dsu_throughput_max_pm_qos,
	&bci_throughput_pm_qos,
	&bci_throughput_max_pm_qos,
	&gpu_freq_min_pm_qos,
	&gpu_freq_max_pm_qos,
	&tpu_freq_min_pm_qos,
	&tpu_freq_max_pm_qos,
};

static struct workqueue_struct *async_vote_wq;

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

	spin_lock(&exynos_pm_qos_array[pm_qos_class]->constraints->lock);
	plist_for_each(p, &exynos_pm_qos_array[pm_qos_class]->constraints->list) {
		if (req == container_of(p, struct exynos_pm_qos_request, node)) {
			spin_unlock(&exynos_pm_qos_array[pm_qos_class]->constraints->lock);
			return p->prio;
		}
	}
	spin_unlock(&exynos_pm_qos_array[pm_qos_class]->constraints->lock);

	return -ENODATA;
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_read_req_value);

static inline void exynos_pm_qos_set_value(struct exynos_pm_qos_constraints *c, s32 value)
{
	c->target_value = value;
}

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
	int prev_value, curr_value, new_value;
	int ret;

	spin_lock(&c->lock);
	prev_value = exynos_pm_qos_get_value(c);
	if (value == EXYNOS_PM_QOS_DEFAULT_VALUE)
		new_value = c->default_value;
	else
		new_value = value;

	switch (action) {
	case EXYNOS_PM_QOS_REMOVE_REQ:
		plist_del(node, &c->list);
		break;
	case EXYNOS_PM_QOS_UPDATE_REQ:
		/*
		 * to change the list, we atomically remove, reinit
		 * with new value and add, then see if the extremal
		 * changed
		 */
		plist_del(node, &c->list);
		fallthrough;
	case EXYNOS_PM_QOS_ADD_REQ:
		plist_node_init(node, new_value);
		plist_add(node, &c->list);
		break;
	default:
		/* no action */
		break;
	}

	curr_value = exynos_pm_qos_get_value(c);
	exynos_pm_qos_set_value(c, curr_value);

	spin_unlock(&c->lock);

	if (prev_value != curr_value) {
		ret = 1;
		if (c->notifiers)
			srcu_notifier_call_chain(c->notifiers,
						 (unsigned long)curr_value,
						 NULL);
	} else {
		ret = 0;
	}
	return ret;
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

static void exynos_pm_qos_async_vote_fn(struct work_struct *work)
{
	struct exynos_pm_asynchronous_vote *async_vote = container_of(work,
						  struct exynos_pm_asynchronous_vote,
						  work);

	struct exynos_pm_qos_request *req = container_of(async_vote,
						  struct exynos_pm_qos_request,
						  async_vote);

	exynos_pm_qos_update_request(req, async_vote->target_freq);
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
	INIT_WORK(&req->async_vote.work, exynos_pm_qos_async_vote_fn);
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

	__exynos_pm_qos_update_request(req, new_value);
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_update_request);

/**
 * exynos_pm_qos_update_request_async - async version of exynos_pm_qos_update_request
 * @req : handle to list element holding a exynos_pm_qos request to use
 * @value: defines the qos request
 *
 * If the async_vote_wq is initialized, defers the actual call to exynos_pm_qos_update_request
 * to said workqueue. Otherwise, calls it immediately.
 */
void exynos_pm_qos_update_request_async(struct exynos_pm_qos_request *req,
					s32 new_value)
{
	if (!req)
		return;

	if (!async_vote_wq || exynos_acpm_async_dvfs_enabled()) {
		exynos_pm_qos_update_request(req, new_value);
		return;
	}

	if (req->node.prio != new_value) {
		req->async_vote.target_freq = new_value;
		queue_work(async_vote_wq, &req->async_vote.work);
	}
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_update_request_async);

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

	retval = srcu_notifier_chain_register(constraints->notifiers, notifier);

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

	retval = srcu_notifier_chain_unregister(constraints->notifiers,
						notifier);

	return retval;
}
EXPORT_SYMBOL_GPL(exynos_pm_qos_remove_notifier);

static int exynos_pm_qos_power_init(void)
{
	BUILD_BUG_ON(ARRAY_SIZE(exynos_pm_qos_array) != EXYNOS_PM_QOS_NUM_CLASSES);

	if (!async_vote_wq)
		async_vote_wq = alloc_workqueue("async_vote_wq",
						WQ_FREEZABLE | WQ_UNBOUND,
						1);

	if (!async_vote_wq) {
		pr_err("%s: Couldn't create async_vote_wq workqueue!\n", __func__);
	}

	return 0;
}
late_initcall(exynos_pm_qos_power_init);

MODULE_LICENSE("GPL");
