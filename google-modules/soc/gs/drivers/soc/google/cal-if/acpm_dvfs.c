#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <soc/google/exynos_pm_qos.h>
#include <linux/slab.h>
#include <linux/sched/clock.h>

#include <soc/google/acpm_ipc_ctrl.h>
#include <soc/google/exynos-devfreq.h>
#include <linux/module.h>
#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
#include <dt-bindings/clock/gs101.h>
#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#include <dt-bindings/clock/zuma.h>
#endif

#include "acpm_dvfs.h"
#include "cmucal.h"

#if !IS_ENABLED(CONFIG_ARM_EXYNOS_DEVFREQ)
#define PM_QOS_BUS_THROUGHPUT (11)
#endif

#define ASYNC_DVFS_UNAVAILABLE 0xFFFFFFFF

static struct acpm_dvfs acpm_dvfs;

typedef struct {
	uint32_t *buffer_start;
	unsigned int num_dvfs_domain;
} acpm_async_buffer_t;

typedef struct {
	acpm_async_buffer_t running;
	acpm_async_buffer_t requested;
} acpm_async_dvfs_states_t;

static acpm_async_dvfs_states_t acpm_async_dvfs_states;
static bool async_dvfs_enabled = false;

/* Check whether to use ASYNC DVFS or not */
static bool async_dvfs_eligible(unsigned int domain_id)
{
	switch (domain_id) {
	case GET_IDX(ACPM_DVFS_CPUCL0):
	case GET_IDX(ACPM_DVFS_CPUCL1):
	case GET_IDX(ACPM_DVFS_CPUCL2):
	case GET_IDX(ACPM_DVFS_MIF):
	case GET_IDX(ACPM_DVFS_INT):
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	case GET_IDX(ACPM_DVFS_BCI):
	case GET_IDX(ACPM_DVFS_DSU):
#endif
		return true;
	default:
		return false;
	}
}

bool exynos_acpm_async_dvfs_enabled(void)
{
	return async_dvfs_enabled;
}
EXPORT_SYMBOL_GPL(exynos_acpm_async_dvfs_enabled);

static unsigned int async_dvfs_get_requested_freq(unsigned int id)
{
	unsigned int index = GET_IDX(id);

	if (index < acpm_async_dvfs_states.requested.num_dvfs_domain) {
		return acpm_async_dvfs_states.requested.buffer_start[index];
	} else {
		return 0;
	}
}

static unsigned int async_dvfs_get_running_freq(unsigned int id)
{
	unsigned int index = GET_IDX(id);

	if (index < acpm_async_dvfs_states.running.num_dvfs_domain) {
		return acpm_async_dvfs_states.running.buffer_start[index];
	} else {
		return 0;
	}
}

static void async_dvfs_set_requested_freq(unsigned int id, unsigned int rate)
{
	unsigned int index = GET_IDX(id);

	if (index < acpm_async_dvfs_states.requested.num_dvfs_domain) {
		acpm_async_dvfs_states.requested.buffer_start[index] = rate;
	}
}

int exynos_acpm_set_rate(unsigned int id, unsigned long rate)
{
	struct ipc_config config;
	unsigned int cmd[4];
	unsigned long long before, after, latency;
	int ret;

	if (acpm_dvfs.async_ch_num != ASYNC_DVFS_UNAVAILABLE && async_dvfs_enabled &&
	    async_dvfs_eligible(id)) {
		async_dvfs_set_requested_freq(id, rate);
		acpm_ipc_ring_doorbell(acpm_dvfs.async_ch_num);
		return 0;
	}

	config.cmd = cmd;
	if (id == GET_IDX(ACPM_DVFS_MIF) || id == GET_IDX(ACPM_DVFS_INT) ||
	    id == GET_IDX(ACPM_DVFS_CPUCL0) || id == GET_IDX(ACPM_DVFS_CPUCL1) ||
	    id == GET_IDX(ACPM_DVFS_CPUCL2))
		config.response = false;
	else
		config.response = true;
	config.cmd[0] = id;
	config.cmd[1] = (unsigned int)rate;
	config.cmd[2] = FREQ_REQ;
	config.cmd[3] = (u32)(sched_clock() / 1000000); /*record ktime ms*/

	before = sched_clock();
	ret = acpm_ipc_send_data_lazy(acpm_dvfs.ch_num, &config);
	after = sched_clock();
	latency = after - before;
	if (ret) {
		pr_err("%s:[%d] latency = %llu ret = %d",
			__func__, id, latency, ret);
		return ret;
	}

	return config.cmd[3];
}
EXPORT_SYMBOL_GPL(exynos_acpm_set_rate);

int exynos_acpm_set_init_freq(unsigned int dfs_id, unsigned long freq)
{
	struct ipc_config config;
	unsigned int cmd[4];
	unsigned long long before, after, latency;
	int ret, id;

	id = GET_IDX(dfs_id);

	config.cmd = cmd;
	config.response = true;
	config.cmd[0] = id;
	config.cmd[1] = (unsigned int)freq;
	config.cmd[2] = DATA_INIT;
	config.cmd[3] = SET_INIT_FREQ;

	before = sched_clock();
	ret = acpm_ipc_send_data_lazy(acpm_dvfs.ch_num, &config);
	after = sched_clock();
	latency = after - before;
	if (ret) {
		pr_err("%s:[%d] latency = %llu ret = %d",
			__func__, id, latency, ret);
		return ret;
	}

	return config.cmd[3];
}
EXPORT_SYMBOL_GPL(exynos_acpm_set_init_freq);

int exynos_acpm_get_rate(unsigned int id, unsigned long dbg_val)
{
	struct ipc_config config;
	unsigned int cmd[4];
	unsigned long long before, after, latency;
	int ret;

	config.cmd = cmd;
	config.response = true;
	config.cmd[0] = id;
	config.cmd[1] = dbg_val;
	config.cmd[2] = FREQ_GET;
	config.cmd[3] = (u32)(sched_clock() / 1000000); /*record ktime ms*/

	before = sched_clock();
	ret = acpm_ipc_send_data_lazy(acpm_dvfs.ch_num, &config);
	after = sched_clock();
	latency = after - before;
	if (ret)
		pr_err("%s:[%d] latency = %llu ret = %d", __func__,
			id, latency, ret);
	if (config.cmd[3])
		return config.cmd[3];

	return config.cmd[1];
}
EXPORT_SYMBOL(exynos_acpm_get_rate);

int exynos_acpm_set_volt_margin(unsigned int id, int volt)
{
	struct ipc_config config;
	unsigned int cmd[4];
	unsigned long long before, after, latency;
	int ret;

	config.cmd = cmd;
	config.response = true;
	config.cmd[0] = id;
	config.cmd[1] = volt;
	config.cmd[2] = MARGIN_REQ;
	config.cmd[3] = (u32)(sched_clock() / 1000000); /*record ktime ms*/

	before = sched_clock();
	ret = acpm_ipc_send_data_lazy(acpm_dvfs.ch_num, &config);
	after = sched_clock();
	latency = after - before;
	if (ret) {
		pr_err("%s:[%d] latency = %llu ret = %d",
			__func__, id, latency, ret);
		return ret;
	}

	return config.cmd[3];
}

int exynos_acpm_set_policy(unsigned int id, unsigned long policy)
{
	struct ipc_config config;
	unsigned int cmd[4];
	unsigned long long before, after, latency;
	int ret;

	config.cmd = cmd;
	config.response = true;
	config.cmd[0] = id;
	config.cmd[1] = policy;
	config.cmd[2] = POLICY_REQ;
	config.cmd[3] = (u32)(sched_clock() / 1000000); /*record ktime ms*/

	before = sched_clock();
	ret = acpm_ipc_send_data(acpm_dvfs.ch_num, &config);
	after = sched_clock();
	latency = after - before;
	if (ret) {
		pr_err("%s:[%d] latency = %llu ret = %d",
			__func__, id, latency, ret);
		return ret;
	}

	return config.cmd[3];
}
EXPORT_SYMBOL_GPL(exynos_acpm_set_policy);

void exynos_acpm_set_device(struct device *device)
{
	acpm_dvfs.device_node = device->of_node;
	acpm_dvfs.device = device;
}
EXPORT_SYMBOL_GPL(exynos_acpm_set_device);

static ssize_t async_dvfs_running_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	len += sysfs_emit_at(buf, len, "Current Async DVFS state:\n");
	len += sysfs_emit_at(buf, len, "CPUCL0:\t%d\n", async_dvfs_get_running_freq(ACPM_DVFS_CPUCL0));
	len += sysfs_emit_at(buf, len, "CPUCL1:\t%d\n", async_dvfs_get_running_freq(ACPM_DVFS_CPUCL1));
	len += sysfs_emit_at(buf, len, "CPUCL2:\t%d\n", async_dvfs_get_running_freq(ACPM_DVFS_CPUCL2));
	len += sysfs_emit_at(buf, len, "MIF:\t%d\n", async_dvfs_get_running_freq(ACPM_DVFS_MIF));
	len += sysfs_emit_at(buf, len, "INT:\t%d\n", async_dvfs_get_running_freq(ACPM_DVFS_INT));
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	len += sysfs_emit_at(buf, len, "BCI:\t%d\n", async_dvfs_get_running_freq(ACPM_DVFS_BCI));
	len += sysfs_emit_at(buf, len, "DSU:\t%d\n", async_dvfs_get_running_freq(ACPM_DVFS_DSU));
#endif
	len += sysfs_emit_at(buf, len, "\n");

	return len;
}

static ssize_t async_dvfs_requested_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	ssize_t len = 0;

	len += sysfs_emit_at(buf, len, "Requested Async DVFS state:\n");
	len += sysfs_emit_at(buf, len, "CPUCL0:\t%d\n", async_dvfs_get_requested_freq(ACPM_DVFS_CPUCL0));
	len += sysfs_emit_at(buf, len, "CPUCL1:\t%d\n", async_dvfs_get_requested_freq(ACPM_DVFS_CPUCL1));
	len += sysfs_emit_at(buf, len, "CPUCL2:\t%d\n", async_dvfs_get_requested_freq(ACPM_DVFS_CPUCL2));
	len += sysfs_emit_at(buf, len, "MIF:\t%d\n", async_dvfs_get_requested_freq(ACPM_DVFS_MIF));
	len += sysfs_emit_at(buf, len, "INT:\t%d\n", async_dvfs_get_requested_freq(ACPM_DVFS_INT));
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	len += sysfs_emit_at(buf, len, "BCI:\t%d\n", async_dvfs_get_requested_freq(ACPM_DVFS_BCI));
	len += sysfs_emit_at(buf, len, "DSU:\t%d\n", async_dvfs_get_requested_freq(ACPM_DVFS_DSU));
#endif
	len += sysfs_emit_at(buf, len, "\n");

	return len;
}

static ssize_t async_dvfs_enabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	len += sysfs_emit_at(buf, len, "%d\n", async_dvfs_enabled);
	return len;
}

static void async_dvfs_enable(bool enable)
{
	async_dvfs_set_requested_freq(ACPM_DVFS_CPUCL0, 0);
	async_dvfs_set_requested_freq(ACPM_DVFS_CPUCL1, 0);
	async_dvfs_set_requested_freq(ACPM_DVFS_CPUCL2, 0);
	async_dvfs_set_requested_freq(ACPM_DVFS_MIF, 0);
	async_dvfs_set_requested_freq(ACPM_DVFS_INT, 0);
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	async_dvfs_set_requested_freq(ACPM_DVFS_BCI, 0);
	async_dvfs_set_requested_freq(ACPM_DVFS_DSU, 0);
#endif
	async_dvfs_enabled = enable;
	if (async_dvfs_enabled) {
		pr_info("%s : Enabling Async DVFS\n", __func__);
	} else {
		pr_info("%s : Disabling Async DVFS\n", __func__);
	}
}

static ssize_t async_dvfs_enabled_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	if (acpm_dvfs.async_ch_num != ASYNC_DVFS_UNAVAILABLE) {
		if (count > 0 && buf) {
			if (buf[0] == '0')
				async_dvfs_enable(false);
			else if (buf[0] == '1')
				async_dvfs_enable(true);
		}
	}

	return count;
}

static DEVICE_ATTR_RO(async_dvfs_running);
static DEVICE_ATTR_RO(async_dvfs_requested);
static DEVICE_ATTR_RW(async_dvfs_enabled);

static struct attribute *async_dvfs_attrs[] = {
	&dev_attr_async_dvfs_running.attr,
	&dev_attr_async_dvfs_requested.attr,
	&dev_attr_async_dvfs_enabled.attr,
	NULL,
};

ATTRIBUTE_GROUPS(async_dvfs);

int exynos_acpm_dvfs_init(void)
{
	int ret, ret2;
	struct device_node *sub_node;

	ret = acpm_ipc_request_channel(acpm_dvfs.device_node, NULL, &acpm_dvfs.ch_num,
				       &acpm_dvfs.size);

	if (ret < 0)
		pr_err("acpm_dvfs_init fail ret = %d\n", ret);

	sub_node = of_find_node_by_name(acpm_dvfs.device_node, "async_dvfs");
	if (IS_ERR(sub_node)) {
		pr_info("%s : Async DVFS not enabled in device tree\n", __func__);
	} else {
		ret2 = acpm_ipc_request_channel(sub_node, NULL, &acpm_dvfs.async_ch_num,
						&acpm_dvfs.async_buffersize);

		if (ret2 < 0 || acpm_dvfs.async_buffersize == 0) {
			pr_info("%s : Async DVFS not enabled - acpm_ipc channel error\n", __func__);
			acpm_dvfs.async_ch_num = ASYNC_DVFS_UNAVAILABLE;
		} else {
			if (acpm_ipc_get_rx_buffer_properties(
				    acpm_dvfs.async_ch_num,
				    (void __iomem *)&acpm_async_dvfs_states.running.buffer_start,
				    &acpm_async_dvfs_states.running.num_dvfs_domain)) {
				acpm_async_dvfs_states.running.num_dvfs_domain /= sizeof(uint32_t);
			}

			if (acpm_ipc_get_tx_buffer_properties(
				    acpm_dvfs.async_ch_num,
				    (void __iomem *)&acpm_async_dvfs_states.requested.buffer_start,
				    &acpm_async_dvfs_states.requested.num_dvfs_domain)) {
				acpm_async_dvfs_states.requested.num_dvfs_domain /=
					sizeof(uint32_t);
			}

			ret2 = devm_device_add_groups(acpm_dvfs.device, async_dvfs_groups);
			if (ret2)
				dev_err(acpm_dvfs.device, "Failed to add device groups\n");

			async_dvfs_enable(true);
		}
	}

	return ret;
}

MODULE_LICENSE("GPL");
