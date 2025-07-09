// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/cpuidle.h>
#include <asm/barrier.h>
#include <asm/sysreg.h>
#include <soc/google/debug-snapshot.h>
#include <soc/google/sjtag-driver.h>

#include "core_regs.h"

#define CHANNEL		(0)
#define PORT		(1)
#define NONE		(-1)
#define ARR_SZ		(2)
#define FUNNEL_PORT_MAX	(8)
#define BUS_ADDR_MAX	(0xFFFFFFFFFULL)

#define etm_writel(base, val, off)	__raw_writel((val), (base) + (off))
#define etm_readl(base, off)		__raw_readl((base) + (off))

#define BDU_ENABLE 0x0
#define BDU_MUX_CTRL 0x4
#define BDU_ATID 0x8
#define BDU_AMATCH0 0x10
#define BDU_AMASK0 0x14
#define BDU_AMATCH1 0x18
#define BDU_AMASK1 0x1C
#define BDU_AMATCH_CTRL 0x20

static inline u64 read_trfcr(void)
{
	return read_sysreg_s(SYS_TRFCR_EL1);
}

static inline void write_trfcr(u64 val)
{
	write_sysreg_s(val, SYS_TRFCR_EL1);
	isb();
}

static inline void soft_lock(void __iomem *base)
{
	dsb(sy);
	isb();
	etm_writel(base, 0x0, LAR);
}

static inline void soft_unlock(void __iomem *base)
{
	etm_writel(base, OSLOCK_MAGIC, LAR);
	dsb(sy);
	isb();
	while ((etm_readl(base, LSR) & 0x02) != 0x00)
		;
}

struct etm_info {
	void __iomem	*base;
	bool		enabled;
	bool		status;
	u32		f_port[ARR_SZ];
};

struct funnel_info {
	void __iomem		*base;
	u32			port_status;
	u32			manual_port_status;
	struct platform_device	pdevice_funnel;
};
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
struct etf_info {
	void __iomem	*base;
	u32		f_port[ARR_SZ];
};
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
struct etr_info {
	void __iomem	*base;
	void __iomem	*sfr_base;
	u32		status;
	dma_addr_t	buf_addr;
	dma_addr_t	aux_buf_addr;
	void		*buf_vaddr;
	u64		buf_pointer;
	u32		qch_offset;
	bool		hwacg;
};
#endif
struct bdu_info {
	void __iomem	*base;
	u32		f_port[ARR_SZ];
	struct etf_info	etf;
	u64		filter_addr_match;
	u64		filter_addr_mask;
	bool		filter_rdwr_match;
	bool		filter_rdwr_mask;
	u32		filter_arpath_match;
	u32		filter_arpath_mask;
	u32		mux_ctrl;
};
struct trex_info {
	void __iomem		*dbgtrace_addr;
	u32			dbgtrace_val;
	u32			mux_ctrl_val;
	struct platform_device 	pdevice_trex;
};
struct exynos_etm_info {
	struct etm_info		*cpu;
	u32			etm_en_mask;
	spinlock_t		trace_lock;
	struct device		*dev;
	bool			enabled;
	bool			status;
	u32			boot_start;
	u32			etr_buf_size;
	u32			etr_aux_buf_size;
	u32			funnel_num;
	struct funnel_info	*funnel;
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	u32			etf_num;
	struct etf_info		*etf;
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	struct etr_info		etr;
#endif
	struct bdu_info		bdu;
	u32			trex_num;
	u32			trex_trace_format;
	struct trex_info	*trex;
};

static struct exynos_etm_info *ee_info;

static void set_trfcr(bool enable)
{
	u64 trfcr = read_trfcr();

	if (!!enable) {
		trfcr |= 0x63;
	} else {
		trfcr &= ~(0x63);
	}

	write_trfcr(trfcr);
}

static void exynos_etm_set_funnel_port(struct funnel_info *funnel,
				       int port, bool enabled)
{
	soft_unlock(funnel->base);
	funnel->port_status = etm_readl(funnel->base, FUNCTRL);

	if (enabled) {
		funnel->port_status |= BIT(port);
	} else {
		funnel->port_status &= ~BIT(port);
		funnel->port_status |= funnel->manual_port_status;
	}

	etm_writel(funnel->base, funnel->port_status, FUNCTRL);
	soft_lock(funnel->base);
}

static void exynos_etm_funnel_init(void)
{
	unsigned int i;
	struct funnel_info *funnel;
	unsigned int port;

	for (i = 0; i < ee_info->funnel_num; i++) {
		funnel = &ee_info->funnel[i];
		spin_lock(&ee_info->trace_lock);
		soft_unlock(funnel->base);
		port = etm_readl(funnel->base, FUNCTRL);
		port &= ~0x3ff;
		port |= 0x300;
		funnel->port_status |= port;
		funnel->port_status |= funnel->manual_port_status;
		etm_writel(funnel->base, funnel->port_status, FUNCTRL);
		etm_writel(funnel->base, 0x0, FUNPRIORCTRL);
		soft_lock(funnel->base);
		spin_unlock(&ee_info->trace_lock);
	}
}

static void exynos_etm_funnel_close(void)
{
	unsigned int i;
	struct funnel_info *funnel;

	for (i = 0; i < ee_info->funnel_num; i++) {
		funnel = &ee_info->funnel[i];
		spin_lock(&ee_info->trace_lock);
		soft_unlock(funnel->base);
		etm_writel(funnel->base, 0x300, FUNCTRL);
		soft_lock(funnel->base);
		spin_unlock(&ee_info->trace_lock);
	}
}

#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
static void exynos_etm_etf_enable(void)
{
	unsigned int i, port, channel;
	struct etf_info *etf;
	struct etr_info *etr = &ee_info->etr;
	struct funnel_info *funnel;

	if (etr->hwacg)
		writel_relaxed(0x1, etr->sfr_base + etr->qch_offset);

	for (i = 0; i < ee_info->etf_num; i++) {
		etf = &ee_info->etf[i];
		soft_unlock(etf->base);
		etm_writel(etf->base, 0x0, TMCCTL);
		//etm_writel(etf->base, 0x800, TMCRSZ);
		etm_writel(etf->base, 0x2, TMCMODE);
		etm_writel(etf->base, 0x0, TMCTGR);
		etm_writel(etf->base, 0x0, TMCFFCR);
		etm_writel(etf->base, 0x1, TMCCTL);
		soft_lock(etf->base);

		if (etf->f_port[CHANNEL] == NONE)
			continue;

		channel = etf->f_port[CHANNEL];
		port = etf->f_port[PORT];
		funnel = &ee_info->funnel[channel];

		spin_lock(&ee_info->trace_lock);
		exynos_etm_set_funnel_port(funnel, port, true);
		spin_unlock(&ee_info->trace_lock);
	}
}

static void exynos_etm_etf_disable(void)
{
	unsigned int i, port, channel;
	struct etf_info *etf;
	struct funnel_info *funnel;

	for (i = 0; i < ee_info->etf_num; i++) {
		etf = &ee_info->etf[i];
		soft_unlock(etf->base);
		etm_writel(etf->base, 0x0, TMCCTL);
		soft_lock(etf->base);

		if (etf->f_port[CHANNEL] == NONE)
			continue;

		channel = etf->f_port[CHANNEL];
		port = etf->f_port[PORT];
		funnel = &ee_info->funnel[channel];

		spin_lock(&ee_info->trace_lock);
		exynos_etm_set_funnel_port(funnel, port, false);
		spin_unlock(&ee_info->trace_lock);
	}
}
#endif

#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
static void exynos_etm_etr_enable(void)
{
	struct etr_info *etr = &ee_info->etr;
	dma_addr_t buf_addr;
	u64 buf_pointer;
	u32 etr_buf_size;

	if (etr->aux_buf_addr) {
		buf_addr = etr->aux_buf_addr;
		etr_buf_size = ee_info->etr_aux_buf_size;
		buf_pointer = 0;
	} else {
		buf_addr = etr->buf_addr;
		etr_buf_size = ee_info->etr_buf_size;
		buf_pointer = etr->buf_pointer;
	}

	if (etr->hwacg)
		writel_relaxed(0x1, etr->sfr_base + etr->qch_offset);

	soft_unlock(etr->base);
	etm_writel(etr->base, 0x0, TMCCTL);
	etm_writel(etr->base, etr_buf_size / 4, TMCRSZ);
	etm_writel(etr->base, 0x4, TMCTGR);
	etm_writel(etr->base, 0x0, TMCAXICTL);
	etm_writel(etr->base, lower_32_bits(buf_addr), TMCDBALO);
	etm_writel(etr->base, upper_32_bits(buf_addr), TMCDBAHI);
	etm_writel(etr->base, lower_32_bits(buf_pointer), TMCRWP);
	etm_writel(etr->base, upper_32_bits(buf_pointer), TMCRWPHI);
	etm_writel(etr->base, 0x0, TMCMODE);
	etm_writel(etr->base, 0x2001, TMCFFCR);
	etm_writel(etr->base, 0x1, TMCCTL);
	soft_lock(etr->base);
}

static void exynos_etm_etr_disable(void)
{
	struct etr_info *etr = &ee_info->etr;

	soft_unlock(etr->base);
	etm_writel(etr->base, 0x0, TMCCTL);
	if (!etr->aux_buf_addr) {
		etr->buf_pointer = etm_readl(etr->base, TMCRWPHI);
		etr->buf_pointer <<= 32;
		etr->buf_pointer |= etm_readl(etr->base, TMCRWP);
	}
	soft_lock(etr->base);
}

static void bdu_etr_enable(void)
{
	struct etr_info *etr = &ee_info->etr;
	dma_addr_t buf_addr;
	u64 buf_pointer;
	u64 etr_buf_size;

	if (etr->aux_buf_addr) {
		buf_addr = etr->aux_buf_addr;
		etr_buf_size = ee_info->etr_aux_buf_size;
		buf_pointer = 0;
	} else {
		buf_addr = etr->buf_addr;
		etr_buf_size = ee_info->etr_buf_size;
		buf_pointer = etr->buf_pointer;
	}

	if (etr->hwacg)
		writel_relaxed(0x1, etr->sfr_base + etr->qch_offset);

	soft_unlock(etr->base);
	etm_writel(etr->base, 0x0, TMCCTL);
	etm_writel(etr->base, etr_buf_size / 4, TMCRSZ);
	etm_writel(etr->base, 0x0, TMCTGR);
	etm_writel(etr->base, 0x108, TMCAXICTL);
	etm_writel(etr->base, lower_32_bits(buf_addr), TMCDBALO);
	etm_writel(etr->base, upper_32_bits(buf_addr), TMCDBAHI);
	etm_writel(etr->base, 0x0, TMCRWP);
	etm_writel(etr->base, 0x0, TMCRWPHI);
	etm_writel(etr->base, 0x0, TMCMODE);
	etm_writel(etr->base, 0x0, TMCFFCR);
	etm_writel(etr->base, 0x1, TMCCTL);
	soft_lock(etr->base);
}

static void bdu_etr_disable(void)
{
	struct etr_info *etr;

	etr = &ee_info->etr;

	if (etr->hwacg)
		writel_relaxed(0x0, etr->sfr_base + etr->qch_offset);

	soft_unlock(etr->base);
	etm_writel(etr->base, 0x0, TMCCTL);
	soft_lock(etr->base);
}

static void exynos_etm_smp_enable(void *ununsed);
void exynos_etm_trace_start(void);

static void smp_disable_idle_states(void *ptr)
{
	struct cpuidle_driver *drv;
	bool disable = *(bool *)ptr;
	int i;

	drv = cpuidle_get_driver();

	if (!drv) {
		dev_warn(ee_info->dev, "No idle driver registered.\n");
		return;
	}

	for (i = 1; i < drv->state_count; i++) {
		cpuidle_driver_state_disabled(drv, i, disable);
	}
}

/*
 * TODO: b/286355474 workaround for ETR_MIU dump to disable C2 idle
 * SMP call each CPU to disable idle states excpet for wfi.
 */
static void exynos_disable_idle_states(bool disable)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		smp_call_function_single(cpu, smp_disable_idle_states, &disable, 1);
	}
}

int gs_coresight_etm_external_etr_on(u64 buf_addr, u32 buf_size)
{
	struct etr_info *etr = &ee_info->etr;
	struct bdu_info *bdu = &ee_info->bdu;
	bool bdu_enable = etm_readl(bdu->base, BDU_ENABLE) & 0x1;
	unsigned int i;

	if (ee_info->enabled)
		return -EINVAL;

	if (sjtag_is_locked()) {
		dev_err(ee_info->dev, "Coresight requires SJTAG auth\n");
		return -EACCES;
	}

	if (etr->hwacg)
		writel_relaxed(0x1, etr->sfr_base + etr->qch_offset);

	etr->aux_buf_addr = buf_addr;
	ee_info->etr_aux_buf_size = buf_size;

	exynos_disable_idle_states(true);

	if (!bdu_enable) {
		for_each_possible_cpu(i) {
			ee_info->cpu[i].enabled = true;
		}

		for_each_online_cpu(i) {
			smp_call_function_single(i, exynos_etm_smp_enable, NULL,
						 1);
		}

		ee_info->enabled = true;
		exynos_etm_trace_stop();
		exynos_etm_trace_start();
	} else {
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
		bdu_etr_disable();
		bdu_etr_enable();
#endif
	}

	return 0;
}
EXPORT_SYMBOL_GPL(gs_coresight_etm_external_etr_on);

int gs_coresight_etm_external_etr_off(void)
{
	struct etr_info *etr = &ee_info->etr;
	struct bdu_info *bdu = &ee_info->bdu;
	bool bdu_enable = etm_readl(bdu->base, BDU_ENABLE) & 0x1;
	unsigned int i;

	if (!ee_info->etr_aux_buf_size)
		return -EINVAL;

	if (sjtag_is_locked()) {
		dev_err(ee_info->dev, "Coresight requires SJTAG auth\n");
		return -EACCES;
	}

	etr->aux_buf_addr = 0;
	ee_info->etr_aux_buf_size = 0;

	if (!bdu_enable) {
		ee_info->enabled = false;

		for_each_possible_cpu(i) {
			ee_info->cpu[i].enabled = false;
		}

		exynos_etm_trace_stop();

	} else {
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
		bdu_etr_disable();
#endif
	}

	exynos_disable_idle_states(false);

	return 0;
}
EXPORT_SYMBOL_GPL(gs_coresight_etm_external_etr_off);

#else
int gs_coresight_etm_external_etr_on(u64 buf_addr, u32 buf_size)
{
	return -EINVAL;
}

int gs_coresight_etm_external_etr_off(void)
{
	return -EINVAL;
}
#endif

static int exynos_etm_enable(unsigned int cpu)
{
	struct etm_info *info = &ee_info->cpu[cpu];
	struct etr_info *etr = &ee_info->etr;
	struct funnel_info *funnel;
	unsigned int channel, port;

	if (!info->enabled || info->status)
		return 0;

	soft_unlock(info->base);
	etm_writel(info->base, !ETM_EN, ETMCTLR);
	etm_writel(info->base, !OSLOCK, ETMOSLAR);
	etm_writel(info->base, !ETM_EN, ETMCTLR);
	dsb(sy);
	isb();

	/* Main control and Configuration */
	etm_writel(info->base, 0xc1, ETMCONFIG);
	etm_writel(info->base, 0x0, ETMEVENTCTL0R);
	etm_writel(info->base, 0x0, ETMEVENTCTL1R);
	etm_writel(info->base, 0x0, ETMSTALLCTLR);
	etm_writel(info->base, 0xc, ETMSYNCPR);
	etm_writel(info->base, cpu + 1, ETMTRACEIDR);
	etm_writel(info->base, 0x0, ETMTSCTLR);
	etm_writel(info->base, 0x201, ETMVICTLR);
	etm_writel(info->base, 0x0, ETMVIIECTLR);
	etm_writel(info->base, 0x0, ETMVISSCTLR);
	etm_writel(info->base, 0x2, ETMAUXCTLR);

	channel = info->f_port[CHANNEL];
	port = info->f_port[PORT];

	set_trfcr(true);

	spin_lock(&ee_info->trace_lock);
	funnel = &ee_info->funnel[channel];

	if (!ee_info->etm_en_mask) {
		if (etr->hwacg)
			writel_relaxed(0x1, etr->sfr_base + etr->qch_offset);

		exynos_etm_set_funnel_port(funnel, port, true);
	}
	ee_info->etm_en_mask |= 1 << cpu;
	spin_unlock(&ee_info->trace_lock);

	info->status = true;

	etm_writel(info->base, ETM_EN, ETMCTLR);
	dsb(sy);
	isb();
	etm_writel(info->base, OSLOCK, ETMOSLAR);

	soft_lock(info->base);
	return 0;
}

static int exynos_etm_disable(unsigned int cpu)
{
	struct etm_info *info = &ee_info->cpu[cpu];
	struct etr_info *etr = &ee_info->etr;
	struct funnel_info *funnel;
	unsigned int channel, port;

	if (!info->status)
		return 0;

	channel = info->f_port[CHANNEL];
	port = info->f_port[PORT];

	set_trfcr(false);

	spin_lock(&ee_info->trace_lock);
	ee_info->etm_en_mask &= ~(1 << cpu);
	funnel = &ee_info->funnel[channel];
	if (!ee_info->etm_en_mask) {
		if (etr->hwacg)
			writel_relaxed(0x1, etr->sfr_base + etr->qch_offset);
		exynos_etm_set_funnel_port(funnel, port, false);
	}
	spin_unlock(&ee_info->trace_lock);

	info->status = false;
	soft_unlock(info->base);
	etm_writel(info->base, !OSLOCK, ETMOSLAR);
	etm_writel(info->base, !ETM_EN, ETMCTLR);
	etm_writel(info->base, OSLOCK, ETMOSLAR);
	soft_lock(info->base);

	return 0;
}

static void exynos_etm_smp_enable(void *ununsed)
{
	unsigned int cpu = raw_smp_processor_id();
	struct etm_info *info = &ee_info->cpu[cpu];

	if (sjtag_is_locked())
		return;

	if (info->enabled)
		exynos_etm_enable(cpu);
	else
		exynos_etm_disable(cpu);

	dev_info(ee_info->dev, "CPU%d ETM %sabled\n",
			cpu, (info->enabled) ? "en" : "dis");
}

static ssize_t exynos_etm_print_info(char *buf)
{
	struct etm_info *info;
	struct funnel_info *funnel;
	struct etf_info *etf;
	unsigned long ctrl, sts, port_status, read_p;
	int i, channel, port, size = 0;

	if (sjtag_is_locked()) {
		size = scnprintf(buf, PAGE_SIZE, "Coresight requires SJTAG auth\n");
		return size;
	}


	size += scnprintf(buf + size, PAGE_SIZE - size,
			"\n---------------------------------------\n");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			" %-4s | %-10s | %-6s | %-4s | %-6s\n",
			"Core", "ETM status", "Funnel", "Port", "Status");

	for (i = 0; i < num_possible_cpus(); i++) {
		info = &ee_info->cpu[i];
		channel = info->f_port[CHANNEL];
		port = info->f_port[PORT];
		funnel = &ee_info->funnel[channel];
		spin_lock(&ee_info->trace_lock);
		port_status = (funnel->port_status >> port) & 0x1;
		spin_unlock(&ee_info->trace_lock);
		size += scnprintf(buf + size, PAGE_SIZE - size,
				" %-4d | %5sabled | %-6u | %-4d | %-6s\n",
				i, info->status ? "en" : "dis",
				channel, port, port_status ? "on" : "off");
	}
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"---------------------------------------\n");

	for (i = 0; i < ee_info->funnel_num; i++) {
		funnel = &ee_info->funnel[i];
		soft_unlock(funnel->base);
		ctrl = etm_readl(funnel->base, FUNCTRL);
		soft_lock(funnel->base);
		size += scnprintf(buf + size, PAGE_SIZE - size,
				"FUNNEL%d Status: 0x%lx\n", i, ctrl);
	}
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	for (i = 0; i < ee_info->etf_num; i++) {
		etf = &ee_info->etf[i];
		soft_unlock(etf->base);
		ctrl = etm_readl(etf->base, TMCCTL);
		sts = etm_readl(etf->base, TMCSTS);
		read_p = etm_readl(etf->base, TMCRWP);
		soft_lock(etf->base);
		size += scnprintf(buf + size, PAGE_SIZE - size,
				"ETF%d Control: %sabled, Status: 0x%lx, RWP Reg: 0x%px\n",
				i, ctrl & 0x1 ? "en" : "dis", sts, (void *)read_p);
	}
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	soft_unlock(ee_info->etr.base);
	ctrl = etm_readl(ee_info->etr.base, TMCCTL);
	sts = etm_readl(ee_info->etr.base, TMCSTS);
	read_p = etm_readl(ee_info->etr.base, TMCRWPHI);
	read_p <<= 32;
	read_p |= etm_readl(ee_info->etr.base, TMCRWP);
	soft_lock(ee_info->etr.base);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			"ETR Control: %sabled, Status: 0x%lx, RWP Reg: 0x%px, Save RWP: 0x%llx\n",
			ctrl & 0x1 ? "en" : "dis", sts, (void *)read_p,	ee_info->etr.buf_pointer);
#endif
	return size;
}

void exynos_etm_trace_start(void)
{
	char *buf;
	struct etr_info *etr = &ee_info->etr;

	if (!ee_info->enabled || ee_info->status)
		return;

	if (sjtag_is_locked()) {
		dev_err(ee_info->dev, "Coresight requires SJTAG auth\n");
		return;
	}

	buf = devm_kzalloc(ee_info->dev, PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return;

	if (etr->hwacg)
		writel_relaxed(0x1, etr->sfr_base + etr->qch_offset);

	ee_info->status = true;

	exynos_etm_funnel_init();
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	exynos_etm_etf_enable();
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	exynos_etm_etr_enable();
#endif
	exynos_etm_print_info(buf);
	dev_info(ee_info->dev, "%s\n", buf);
	devm_kfree(ee_info->dev, buf);
}

void exynos_etm_trace_stop(void)
{
	char *buf;

	if (!ee_info->status)
		return;

	if (sjtag_is_locked()) {
		dev_err(ee_info->dev, "Coresight requires SJTAG auth\n");
		return;
	}

	buf = devm_kzalloc(ee_info->dev, PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return;

#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	exynos_etm_etr_disable();
#endif
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	exynos_etm_etf_disable();
#endif
	exynos_etm_funnel_close();

	ee_info->status = false;
	exynos_etm_print_info(buf);
	dev_info(ee_info->dev, "%s\n", buf);
	devm_kfree(ee_info->dev, buf);
}

static int exynos_etm_c2_pm_notifier(struct notifier_block *self,
				     unsigned long action, void *v)
{
	int cpu = raw_smp_processor_id();

	switch (action) {
	case CPU_PM_ENTER:
		exynos_etm_disable(cpu);
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		exynos_etm_enable(cpu);
		break;
	case CPU_CLUSTER_PM_ENTER:
		break;
	case CPU_CLUSTER_PM_ENTER_FAILED:
	case CPU_CLUSTER_PM_EXIT:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block exynos_etm_c2_pm_nb = {
	.notifier_call = exynos_etm_c2_pm_notifier,
};

static int exynos_etm_pm_notifier(struct notifier_block *notifier,
				  unsigned long pm_event, void *v)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		exynos_etm_trace_stop();
		break;
	case PM_POST_SUSPEND:
		exynos_etm_trace_start();
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block exynos_etm_pm_nb = {
	.notifier_call = exynos_etm_pm_notifier,
};

static ssize_t etm_on_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	unsigned int i;
	bool on;
	int ret;

	if (sjtag_is_locked())
		return -EACCES;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	for_each_possible_cpu(i) {
		ee_info->cpu[i].enabled = on;
	}

	for_each_online_cpu(i) {
		smp_call_function_single(i, exynos_etm_smp_enable, NULL, 1);
	}

	return count;
}

static DEVICE_ATTR_WO(etm_on);

static ssize_t trace_on_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	bool on;
	int ret;

	if (sjtag_is_locked())
		return -EACCES;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	ee_info->enabled = on;
	if (on)
		exynos_etm_trace_start();
	else
		exynos_etm_trace_stop();

	return count;
}

static DEVICE_ATTR_WO(trace_on);

static ssize_t etm_info_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return exynos_etm_print_info(buf);
}

static DEVICE_ATTR_RO(etm_info);

static ssize_t manual_port_on_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct funnel_info *funnel;
	unsigned int port;

	if (sjtag_is_locked())
		return -EACCES;

	if (kstrtouint(buf, 0, &port))
		return -EINVAL;

	if (port >= FUNNEL_PORT_MAX)
		return -EINVAL;

	funnel = dev_get_drvdata(dev);

	spin_lock(&ee_info->trace_lock);
	soft_unlock(funnel->base);
	funnel->manual_port_status |= BIT(port);
	funnel->port_status = etm_readl(funnel->base, FUNCTRL);
	funnel->port_status |= funnel->manual_port_status;
	etm_writel(funnel->base, funnel->port_status, FUNCTRL);
	soft_lock(funnel->base);
	spin_unlock(&ee_info->trace_lock);

	return count;
}

static DEVICE_ATTR_WO(manual_port_on);

static ssize_t manual_port_off_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct funnel_info *funnel;
	unsigned int port;

	if (sjtag_is_locked())
		return -EACCES;

	if (kstrtouint(buf, 0, &port))
		return -EINVAL;

	if (port >= FUNNEL_PORT_MAX)
		return -EINVAL;

	funnel = dev_get_drvdata(dev);

	spin_lock(&ee_info->trace_lock);
	soft_unlock(funnel->base);
	funnel->manual_port_status &= ~BIT(port);
	funnel->port_status = etm_readl(funnel->base, FUNCTRL);
	funnel->port_status &= ~BIT(port);
	etm_writel(funnel->base, funnel->port_status, FUNCTRL);
	soft_lock(funnel->base);
	spin_unlock(&ee_info->trace_lock);

	return count;
}

static DEVICE_ATTR_WO(manual_port_off);

static ssize_t manual_port_status_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	ssize_t count = 0;
	struct funnel_info *funnel;

	funnel = dev_get_drvdata(dev);

	count = scnprintf(buf, PAGE_SIZE, "0x%x\n", funnel->manual_port_status);

	return count;
}

static DEVICE_ATTR_RO(manual_port_status);

static ssize_t port_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t count = 0;
	struct funnel_info *funnel;

	funnel = dev_get_drvdata(dev);

	count = scnprintf(buf, PAGE_SIZE, "0x%x\n", funnel->port_status);

	return count;
}

static DEVICE_ATTR_RO(port_status);

#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
static ssize_t etr_dump_read(struct file *filp,
			     struct kobject *kobj,
			     struct bin_attribute *attr,
			     char *buffer,
			     loff_t pos,
			     size_t size)
{
	if (pos > ee_info->etr_buf_size)
		return 0;

	if (pos + size > ee_info->etr_buf_size)
		size = ee_info->etr_buf_size - pos;

	memcpy(buffer, ee_info->etr.buf_vaddr + pos, size);

	return size;
}

static BIN_ATTR_RO(etr_dump, 0);

static struct bin_attribute *exynos_etr_sysfs_attrs[] = {
	&bin_attr_etr_dump,
	NULL,
};

static struct attribute_group exynos_etr_sysfs_group = {
	.bin_attrs = exynos_etr_sysfs_attrs,
};
#endif

static struct attribute *funnel_attrs[] = {
	&dev_attr_port_status.attr,
	&dev_attr_manual_port_on.attr,
	&dev_attr_manual_port_off.attr,
	&dev_attr_manual_port_status.attr,
	NULL,
};

ATTRIBUTE_GROUPS(funnel);

static ssize_t bdu_filter_addr_match_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bdu_info *bdu;
	u64 filter_addr_match;

	if (kstrtou64(buf, 0, &filter_addr_match))
		return -EINVAL;

	if (filter_addr_match > BUS_ADDR_MAX)
		return -EINVAL;

	bdu = &ee_info->bdu;
	bdu->filter_addr_match = filter_addr_match;

	return count;
}

static DEVICE_ATTR_WO(bdu_filter_addr_match);

static ssize_t bdu_filter_addr_mask_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bdu_info *bdu;
	u64 filter_addr_mask;

	if (kstrtou64(buf, 0, &filter_addr_mask))
		return -EINVAL;

	if (filter_addr_mask > BUS_ADDR_MAX)
		return -EINVAL;

	bdu = &ee_info->bdu;
	bdu->filter_addr_mask = filter_addr_mask;

	return count;
}

static DEVICE_ATTR_WO(bdu_filter_addr_mask);

static ssize_t bdu_filter_rdwr_match_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bdu_info *bdu;
	u32 filter_rdwr_match;

	if (kstrtou32(buf, 0, &filter_rdwr_match))
		return -EINVAL;

	if (filter_rdwr_match > 1)
		return -EINVAL;

	bdu = &ee_info->bdu;
	bdu->filter_rdwr_match = filter_rdwr_match;

	return count;
}

static DEVICE_ATTR_WO(bdu_filter_rdwr_match);

static ssize_t bdu_filter_rdwr_mask_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bdu_info *bdu;
	u32 filter_rdwr_mask;

	if (kstrtou32(buf, 0, &filter_rdwr_mask))
		return -EINVAL;

	if (filter_rdwr_mask > 1)
		return -EINVAL;

	bdu = &ee_info->bdu;
	bdu->filter_rdwr_mask = filter_rdwr_mask;

	return count;
}

static DEVICE_ATTR_WO(bdu_filter_rdwr_mask);

static ssize_t bdu_filter_arpath_match_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bdu_info *bdu;
	u32 filter_arpath_match;

	if (kstrtou32(buf, 0, &filter_arpath_match))
		return -EINVAL;

	if (filter_arpath_match > 0xFF)
		return -EINVAL;

	bdu = &ee_info->bdu;
	bdu->filter_arpath_match = filter_arpath_match;

	return count;
}

static DEVICE_ATTR_WO(bdu_filter_arpath_match);

static ssize_t bdu_filter_arpath_mask_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bdu_info *bdu;
	u32 filter_arpath_mask;

	if (kstrtou32(buf, 0, &filter_arpath_mask))
		return -EINVAL;

	if (filter_arpath_mask > 0xFF)
		return -EINVAL;

	bdu = &ee_info->bdu;
	bdu->filter_arpath_mask = filter_arpath_mask;

	return count;
}

static DEVICE_ATTR_WO(bdu_filter_arpath_mask);

static ssize_t bdu_filter_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t count = 0;
	struct bdu_info *bdu;
	bdu = &ee_info->bdu;

	count += scnprintf(buf + count, PAGE_SIZE - count, "Filter Values:\n");
	count += scnprintf(buf + count, PAGE_SIZE - count, "Address Match: 0x%llx\n",
			   bdu->filter_addr_match);
	count += scnprintf(buf + count, PAGE_SIZE - count, "Address Mask: 0x%llx\n",
			   bdu->filter_addr_mask);
	count += scnprintf(buf + count, PAGE_SIZE - count, "Read/Write Match: 0x%x\n",
			   bdu->filter_rdwr_match);
	count += scnprintf(buf + count, PAGE_SIZE - count, "Read/Write Mask: 0x%x\n",
			   bdu->filter_rdwr_mask);
	count += scnprintf(buf + count, PAGE_SIZE - count, "ARPATH Match: 0x%x\n",
			   bdu->filter_arpath_match);
	count += scnprintf(buf + count, PAGE_SIZE - count, "ARPATH Mask: 0x%x\n",
			   bdu->filter_arpath_mask);
	return count;
}

static DEVICE_ATTR_RO(bdu_filter_status);

static ssize_t bdu_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct funnel_info *funnel;
	struct bdu_info *bdu;
	struct etf_info *etf;
	unsigned long ctl, sts;
	int i, size = 0;
	u32 enable, mux_ctrl, atid, amatch0, amask0, amatch1, amask1,
		amatch_ctrl;
	u64 read_p;

	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "---------------------------------------\n");
	size += scnprintf(buf + size, PAGE_SIZE - size, "Funnel Registers\n");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "---------------------------------------\n");
	for (i = 0; i < ee_info->funnel_num; i++) {
		funnel = &ee_info->funnel[i];
		soft_unlock(funnel->base);
		ctl = etm_readl(funnel->base, FUNCTRL);
		soft_lock(funnel->base);
		size += scnprintf(buf + size, PAGE_SIZE - size, "FUNNEL%d Status : 0x%lx\n",
				  i, ctl);
	}

	bdu = &ee_info->bdu;
	spin_lock(&ee_info->trace_lock);
	enable = etm_readl(bdu->base, BDU_ENABLE);
	mux_ctrl = etm_readl(bdu->base, BDU_MUX_CTRL);
	atid = etm_readl(bdu->base, BDU_ATID);
	amatch0 = etm_readl(bdu->base, BDU_AMATCH0);
	amask0 = etm_readl(bdu->base, BDU_AMASK0);
	amatch1 = etm_readl(bdu->base, BDU_AMATCH1);
	amask1 = etm_readl(bdu->base, BDU_AMASK1);
	amatch_ctrl = etm_readl(bdu->base, BDU_AMATCH_CTRL);
	spin_unlock(&ee_info->trace_lock);

	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "---------------------------------------\n");
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU Registers\n");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "---------------------------------------\n");
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU Enable : 0x%x\n", enable);
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU Mux Ctrl : 0x%x\n", mux_ctrl);
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU ATID : 0x%x\n", atid);
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU AMatch0 : 0x%x\n", amatch0);
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU AMask0 : 0x%x\n", amask0);
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU AMatch1 : 0x%x\n", amatch1);
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU AMask1 : 0x%x\n", amask1);
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU AMatch Ctrl : 0x%x\n", amatch_ctrl);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "---------------------------------------\n");

	etf = &bdu->etf;
	soft_unlock(etf->base);
	ctl = etm_readl(etf->base, TMCCTL);
	sts = etm_readl(etf->base, TMCSTS);
	read_p = etm_readl(etf->base, TMCRWP);
	soft_lock(etf->base);

	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "---------------------------------------\n");
	size += scnprintf(buf + size, PAGE_SIZE - size, "BDU ETF Registers\n");
	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "---------------------------------------\n");
	size += scnprintf(buf + size, PAGE_SIZE - size, "ETF Control : %3sabled\n",
			  ctl & 0x1 ? "en" : "dis");
	size += scnprintf(buf + size, PAGE_SIZE - size, "ETF Status : 0x%lx\n", sts);
	size += scnprintf(buf + size, PAGE_SIZE - size, "ETF RWP Reg : 0x%llx\n", read_p);
	size += scnprintf(buf + size, PAGE_SIZE - size,
			  "---------------------------------------\n");

	return size;
}

static DEVICE_ATTR_RO(bdu_status);

static ssize_t trex_format_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	u32 trace_format;

	if (kstrtou32(buf, 0, &trace_format))
		return -EINVAL;

	if (trace_format > 1)
		return -EINVAL;

	ee_info->trex_trace_format = trace_format;

	return count;
}

static DEVICE_ATTR_WO(trex_format);


static ssize_t trex_bus_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct bdu_info *bdu;
	struct trex_info *trex;
	u32 format, val, trace_point;

	if (kstrtou32(buf, 0, &trace_point))
		return -EINVAL;

	if (trace_point > 6)
		return -EINVAL;

	bdu = &ee_info->bdu;
	trex = dev_get_drvdata(dev);
	format = ee_info->trex_trace_format;

	bdu->mux_ctrl = trex->mux_ctrl_val;
	val = 0;
	if (trace_point != 0)
		val = ((trex->dbgtrace_val + trace_point) << 16) | (format << 8) | 0x1;

	writel_relaxed(val, trex->dbgtrace_addr);

	return count;
}

static DEVICE_ATTR_WO(trex_bus);

static void bdu_etf_enable(void)
{
	struct bdu_info *bdu;

	bdu = &ee_info->bdu;

	soft_unlock(bdu->etf.base);
	etm_writel(bdu->etf.base, 0x0, TMCCTL);
	etm_writel(bdu->etf.base, 0x2, TMCMODE);
	etm_writel(bdu->etf.base, 0x0, TMCFFCR);
	etm_writel(bdu->etf.base, 0x100, TMCBUFWM);
	etm_writel(bdu->etf.base, 0x1, TMCCTL);
	soft_lock(bdu->etf.base);
}

static void bdu_setup(void)
{
	struct bdu_info *bdu;
	u32 mask0 = 0xFFFFFFFF;
	u32 mask1 = 0xFFFFFFFF;
	u32 match0 = 0;
	u32 match1 = 0;

	bdu = &ee_info->bdu;

	switch(ee_info->trex_trace_format) {
	case 0:
		match1 |= (bdu->filter_arpath_match << 17);
		mask1 &= (0xFE01FFFF) | (bdu->filter_arpath_mask << 17);
		match0 |= (bdu->filter_rdwr_match << 30);
		mask0 &= (0xBFFFFFFF) | (bdu->filter_rdwr_mask << 30);
		match0 |= (bdu->filter_addr_match >> 6);
		mask0 &= (0xC0000000) | (bdu->filter_addr_mask >> 6);
		match1 |= (bdu->filter_addr_match & 0x3F);
		mask1 &= (0xFFFFFFC0) | (bdu->filter_addr_mask & 0x3F);
		break;

	case 1:
		match0 |= (bdu->filter_arpath_match << 8);
		mask0 &= (0xFFFF00FF | (bdu->filter_arpath_mask << 8));
		match0 |= (bdu->filter_rdwr_match << 24);
		mask0 &= (0xFEFFFFFF) | (bdu->filter_rdwr_mask << 24);
		break;
	}

	etm_writel(bdu->base, 0x50, BDU_ATID);
	etm_writel(bdu->base, bdu->mux_ctrl, BDU_MUX_CTRL);
	etm_writel(bdu->base, match0, BDU_AMATCH0);
	etm_writel(bdu->base, mask0, BDU_AMASK0);
	etm_writel(bdu->base, match1, BDU_AMATCH1);
	etm_writel(bdu->base, mask1, BDU_AMASK1);
	etm_writel(bdu->base, 0x550F, BDU_AMATCH_CTRL);
	etm_writel(bdu->base, 0x1, BDU_ENABLE);
}

static void bdu_disable(void)
{
	struct bdu_info *bdu;

	bdu = &ee_info->bdu;

	etm_writel(bdu->base, 0x201, BDU_ENABLE);
	etm_writel(bdu->base, 0x0, BDU_ENABLE);
}

static ssize_t bdu_trace_en_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct bdu_info *bdu;
	struct etf_info *etf;
	bool en;
	unsigned int i;

	if (kstrtobool(buf, &en))
		return -EINVAL;

	bdu = &ee_info->bdu;
	etf = &ee_info->etf[1];

	if (en) {
		/* Disable all other funnels except for BDU */
		spin_lock(&ee_info->trace_lock);
		for (i = 0; i < ee_info->funnel_num; i++) {
			ee_info->funnel[i].port_status = 0x300;
		}
		ee_info->funnel[bdu->f_port[CHANNEL]].port_status |= BIT(bdu->f_port[PORT]);
		ee_info->funnel[etf->f_port[CHANNEL]].port_status |= BIT(etf->f_port[PORT]);
		for (i = 0; i < ee_info->funnel_num; i++) {
			soft_unlock(ee_info->funnel[i].base);
			etm_writel(ee_info->funnel[i].base, ee_info->funnel[i].port_status,
				   FUNCTRL);
			soft_lock(ee_info->funnel[i].base);
		}
		spin_unlock(&ee_info->trace_lock);

		/* Set up ETF */
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
		exynos_etm_etf_enable();
#endif

		/* Set up ETR */
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
		bdu_etr_enable();
#endif

		/* Set up BDU ETF */
		bdu_etf_enable();

		/* Set up BDU */
		bdu_setup();
	} else {
		/* Disable BDU */
		bdu_disable();

		/* Disable ETF */
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
		exynos_etm_etf_disable();
#endif

		/* Disable ETR */
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
		bdu_etr_disable();
#endif
	}

	return count;
}

static DEVICE_ATTR_WO(bdu_trace_en);

static struct attribute *exynos_etm_sysfs_attrs[] = {
	&dev_attr_etm_on.attr,
	&dev_attr_trace_on.attr,
	&dev_attr_etm_info.attr,
	NULL,
};

static struct attribute_group exynos_etm_sysfs_group = {
	.attrs = exynos_etm_sysfs_attrs,
	NULL,
};

static struct attribute *bdu_sysfs_attrs[] = {
	&dev_attr_bdu_filter_addr_match.attr,
	&dev_attr_bdu_filter_addr_mask.attr,
	&dev_attr_bdu_filter_rdwr_match.attr,
	&dev_attr_bdu_filter_rdwr_mask.attr,
	&dev_attr_bdu_filter_arpath_match.attr,
	&dev_attr_bdu_filter_arpath_mask.attr,
	&dev_attr_bdu_filter_status.attr,
	&dev_attr_bdu_status.attr,
	&dev_attr_bdu_trace_en.attr,
	NULL,
};

static const struct attribute_group bdu_sysfs_group = {
	.attrs = bdu_sysfs_attrs,
	NULL,
};

static struct attribute *trex_sysfs_attrs[] = {
	&dev_attr_trex_bus.attr,
	&dev_attr_trex_format.attr,
	NULL,
};

ATTRIBUTE_GROUPS(trex_sysfs);

static const struct attribute_group *exynos_coresight_sysfs_groups[] = {
	&exynos_etm_sysfs_group,
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	&exynos_etr_sysfs_group,
#endif
	&bdu_sysfs_group,
	NULL,
};

static int exynos_etm_cs_etm_init_dt(struct device *dev)
{
	struct device_node *np, *etm_np = dev->of_node;
	unsigned int offset, cs_base;
	int i = 0;
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	u32 reg[2];
#endif

	ee_info = devm_kzalloc(dev, sizeof(struct exynos_etm_info),
			GFP_KERNEL);
	if (!ee_info)
		return -ENOMEM;

	ee_info->dev = dev;
	ee_info->cpu = devm_kcalloc(dev, num_possible_cpus(),
			sizeof(struct etm_info), GFP_KERNEL);
	ee_info->etm_en_mask = 0x0;
	if (!ee_info->cpu)
		return -ENOMEM;

	if (of_property_read_u32(etm_np, "funnel-num", &ee_info->funnel_num))
		return -EINVAL;

	ee_info->funnel = devm_kcalloc(dev, ee_info->funnel_num,
			sizeof(struct funnel_info), GFP_KERNEL);
	if (!ee_info->funnel)
		return -ENOMEM;
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	if (of_property_read_u32(etm_np, "etf-num", &ee_info->etf_num))
		return -EINVAL;

	ee_info->etf = devm_kcalloc(dev, ee_info->etf_num,
			sizeof(struct etf_info), GFP_KERNEL);
	if (!ee_info->etf)
		return -ENOMEM;
#endif
	if (of_property_read_u32(etm_np, "cs_base", &cs_base))
		return -EINVAL;

	for_each_node_by_type(np, "etm") {
		if (of_property_read_u32(np, "offset", &offset))
			return -EINVAL;

		ee_info->cpu[i].base = devm_ioremap(dev,
				cs_base + offset, SZ_4K);
		if (!ee_info->cpu[i].base)
			return -ENOMEM;

		if (of_property_read_u32_array(np, "funnel-port",
				ee_info->cpu[i].f_port, 2))
			ee_info->cpu[i].f_port[CHANNEL] = NONE;
		i++;
	}
#ifdef CONFIG_EXYNOS_CORESIGHT_ETF
	i = 0;
	for_each_node_by_type(np, "etf") {
		if (of_property_read_u32(np, "offset", &offset))
			return -EINVAL;

		ee_info->etf[i].base = devm_ioremap(dev, cs_base + offset,
				SZ_1K);
		if (!ee_info->etf[i].base)
			return -ENOMEM;

		if (of_property_read_u32_array(np, "funnel-port",
				ee_info->etf[i].f_port, 2))
			ee_info->etf[i].f_port[CHANNEL] = NONE;
		i++;
	}
#endif
	i = 0;
	for_each_node_by_type(np, "funnel") {
		if (of_property_read_u32(np, "offset", &offset))
			return -EINVAL;

		ee_info->funnel[i].base = devm_ioremap(dev, cs_base + offset,
				SZ_256);

		if (!ee_info->funnel[i].base)
			return -ENOMEM;
		i++;
	}
#ifdef CONFIG_EXYNOS_CORESIGHT_ETR
	np = of_find_node_by_type(etm_np, "etr");
	if (!np)
		return -EINVAL;
	if (of_property_read_u32(np, "offset", &offset))
		return -EINVAL;
	ee_info->etr.base = devm_ioremap(dev, cs_base + offset, SZ_1K);
	if (!ee_info->etr.base)
		return -ENOMEM;

	if (of_property_read_u32(np, "buf-size", &ee_info->etr_buf_size))
		return -EINVAL;

	dma_set_mask_and_coherent(ee_info->dev, DMA_BIT_MASK(36));
	ee_info->etr.buf_vaddr = dma_alloc_coherent(ee_info->dev,
		ee_info->etr_buf_size, &ee_info->etr.buf_addr, GFP_KERNEL);
	if (!ee_info->etr.buf_vaddr || !ee_info->etr.buf_addr)
		return -ENOMEM;

	if (of_property_read_u32(np, "qch-offset", &offset))
		return 0;

	ee_info->etr.qch_offset = offset;

	if (of_property_read_u32_array(np, "sfr_base", reg, 2))
		return -EINVAL;

	ee_info->etr.sfr_base = ioremap(reg[0], reg[1]);
	if (!ee_info->etr.sfr_base)
		return -EINVAL;

	ee_info->etr.hwacg = true;
#endif
	np = of_find_node_by_type(etm_np, "bdu");
	if (!np)
		return -EINVAL;
	if (of_property_read_u32(np, "offset", &offset)) {
		return -EINVAL;
	}
	ee_info->bdu.base = ioremap(cs_base + offset, SZ_4K);
	if (!ee_info->bdu.base) {
		return -ENOMEM;
	}
	if (of_property_read_u32_array(np, "funnel-port",
				       ee_info->bdu.f_port, 2))
		ee_info->bdu.f_port[CHANNEL] = NONE;
	np = of_find_node_by_type(etm_np, "bdu_etf");
	if (!np)
		return -EINVAL;
	if (of_property_read_u32(np, "offset", &offset)) {
		return -EINVAL;
	}
	ee_info->bdu.etf.base = ioremap(cs_base + offset, SZ_4K);
	if (!ee_info->bdu.etf.base) {
		return -ENOMEM;
	}
	ee_info->bdu.filter_addr_mask = 0xFFFFFFFFF;
	ee_info->bdu.filter_rdwr_mask = 0x1;
	ee_info->bdu.filter_arpath_mask = 0xFF;
	if (of_property_read_u32(etm_np, "trex-num", &ee_info->trex_num))
		return -EINVAL;
	ee_info->trex = devm_kcalloc(dev, ee_info->trex_num, sizeof(struct trex_info), GFP_KERNEL);
	if (!ee_info->trex)
		return -ENOMEM;
	i = 0;
	for_each_node_by_type(np, "trex") {
		if (of_property_read_u32(np, "mux_ctrl", &ee_info->trex[i].mux_ctrl_val))
			return -EINVAL;
		if (of_property_read_u32(np, "dbg_trace_val", &ee_info->trex[i].dbgtrace_val))
			return -EINVAL;
		if (of_property_read_u32(np, "dbg_trace_addr", &offset))
			return -EINVAL;
		ee_info->trex[i].dbgtrace_addr = devm_ioremap(dev, offset, SZ_4);
		if (!ee_info->trex[i].dbgtrace_addr)
			return -ENOMEM;
		i++;
	}
	if (of_property_read_u32(etm_np, "boot-start", &ee_info->boot_start))
		return 0;
	return 0;
}

static int exynos_etm_manual_funnel_setup(struct device *dev)
{
	int ret = 0;
	int i;
	struct funnel_info *funnel;

	for (i = 0; i < ee_info->funnel_num; i++) {
		funnel = &ee_info->funnel[i];
		funnel->pdevice_funnel.dev.id = i;
		funnel->pdevice_funnel.dev.bus = dev->bus;
		funnel->pdevice_funnel.dev.groups = funnel_groups;
		funnel->pdevice_funnel.name = "cs_funnel";
		dev_set_name(&funnel->pdevice_funnel.dev, "cs_funnel%d", i);
		dev_set_drvdata(&funnel->pdevice_funnel.dev, funnel);
		ret = device_register(&funnel->pdevice_funnel.dev);
		if (ret) {
			dev_err(dev, "fail to register funnel device %d\n", i);
			i--;
			goto err;
		}
	}

	return ret;
err:
	for (; i >= 0; i--) {
		funnel = &ee_info->funnel[i];
		device_unregister(&funnel->pdevice_funnel.dev);
	}
	return ret;
}

static int trex_setup(struct device *dev)
{
	int ret = 0;
	int i;
	struct trex_info *trex;

	for (i = 0; i < ee_info->trex_num; i++) {
		trex = &ee_info->trex[i];
		trex->pdevice_trex.dev.id = i;
		trex->pdevice_trex.dev.bus = dev->bus;
		trex->pdevice_trex.dev.groups = trex_sysfs_groups;
		trex->pdevice_trex.name = "cs_trex";
		dev_set_name(&trex->pdevice_trex.dev, "cs_trex%d", i);
		dev_set_drvdata(&trex->pdevice_trex.dev, trex);
		ret = device_register(&trex->pdevice_trex.dev);
		if (ret) {
			dev_err(dev, "fail to register trex device %d\n", i);
			i--;
			goto err;
		}
	}

	return ret;
err:
	for (; i >= 0; i--) {
		trex = &ee_info->trex[i];
		device_unregister(&trex->pdevice_trex.dev);
	}
	return ret;
}

static int exynos_etm_probe(struct platform_device *pdev)
{
	unsigned int i;
	int ret;

	ret = exynos_etm_cs_etm_init_dt(&pdev->dev);
	if (ret < 0)
		return ret;

	spin_lock_init(&ee_info->trace_lock);
	register_pm_notifier(&exynos_etm_pm_nb);
	cpus_read_lock();
	cpuhp_setup_state_nocalls_cpuslocked(CPUHP_AP_ONLINE_DYN,
			"exynos-etm:online",
			exynos_etm_enable,
			exynos_etm_disable);
	cpu_pm_register_notifier(&exynos_etm_c2_pm_nb);
	cpus_read_unlock();

	if (ee_info->boot_start) {
		ee_info->enabled = true;
		for_each_possible_cpu(i) {
			ee_info->cpu[i].enabled = true;
		}

		for_each_online_cpu(i) {
			smp_call_function_single(i, exynos_etm_smp_enable, NULL, 1);
		}

		exynos_etm_trace_start();
	}

	if (sysfs_create_groups(&pdev->dev.kobj, exynos_coresight_sysfs_groups)) {
		dev_err(&pdev->dev, "fail to register exynos-etm sysfs\n");
		return -EFAULT;
	}

	exynos_etm_manual_funnel_setup(&pdev->dev);
	trex_setup(&pdev->dev);

	dev_info(&pdev->dev, "%s successful\n", __func__);
	return 0;
}

static const struct of_device_id exynos_etm_matches[] = {
	{.compatible = "google,exynos-etm"},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_etm_matches);

static struct platform_driver exynos_etm_driver = {
	.probe		= exynos_etm_probe,
	.driver		= {
		.name	= "exynos-etm",
		.of_match_table	= of_match_ptr(exynos_etm_matches),
	},
};
module_platform_driver(exynos_etm_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exynos ETM Driver");
