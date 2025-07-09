// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe RC(RootComplex) controller driver for gs101 SoC
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#ifndef __PCIE_EXYNOS_RC_H
#define __PCIE_EXYNOS_RC_H

#define MSI_64CAP_MASK	(0x1 << 23)
#define MSI_CONTROL	0x50
/* SYSREG NCLKOFF */
#define PCIE_SUB_CTRL_SLV_EN	(0x1 << 0)
#define PCIE_SLV_BUS_NCLK_OFF	(0x1 << 1)
#define PCIE_DBI_BUS_NCLK_OFF	(0x1 << 2)
/* PCIe ELBI registers */
#define PCIE_IRQ0			0x000
#define IRQ_INTA_ASSERT			(0x1 << 14)
#define IRQ_INTB_ASSERT			(0x1 << 16)
#define IRQ_INTC_ASSERT			(0x1 << 18)
#define IRQ_INTD_ASSERT			(0x1 << 20)
#define IRQ_RADM_PM_TO_ACK              (0x1 << 29)
#define PCIE_IRQ1			0x004
#define IRQ_LINK_DOWN_ASSERT		(0x1 << 10)
#define PCIE_IRQ2			0x008
#define IRQ_MSI_FALLING_ASSERT		(0x1 << 16)
#define IRQ_MSI_RISING_ASSERT		(0x1 << 17)
#define IRQ_RADM_CPL_TIMEOUT_ASSERT	(0x1 << 24)
#define PCIE_IRQ0_EN			0x010
#define IRQ_INTA_ENABLE			(0x1 << 14)
#define IRQ_INTB_ENABLE			(0x1 << 16)
#define IRQ_INTC_ENABLE			(0x1 << 18)
#define IRQ_INTD_ENABLE			(0x1 << 20)
#define PCIE_IRQ1_EN			0x014
#define IRQ_LINKDOWN_ENABLE_EVT1_1	(0x1 << 10) /* Exynos9820-EVT1.1 only */
#define IRQ_LINK_DOWN_ENABLE		(0x1 << 10)
#define PCIE_IRQ2_EN			0x018
#define IRQ_LINKDOWN_ENABLE_old		(0x1 << 14) /* Exynos9820-EVT0.0 only */
#define IRQ_MSI_CTRL_EN_FALLING_EDG	(0x1 << 16)
#define IRQ_MSI_CTRL_EN_RISING_EDG	(0x1 << 17)
#define IRQ_RADM_CPL_TIMEOUT_ENABLE	(0x1 << 24)
#define PCIE_APP_LTSSM_ENABLE		0x054
#define PCIE_ELBI_LTSSM_DISABLE		0x0
#define PCIE_ELBI_LTSSM_ENABLE		0x1
#define PCIE_SLV_PEND_SEL_NAK           0x03D8
#define PCIE_APP_XFER_PENDING           0x0074
#define PCIE_APP_REQ_EXIT_L1		0x06C
#define APP_INIT_RST			0x100
#define XMIT_PME_TURNOFF		0x118
#define PCIE_ELBI_RDLH_LINKUP		0x2C8
#define PCIE_ELBI_LTSSM_STATE_MASK	0x003F
#define PCIE_CXPL_DEBUG_INFO_H		0x2CC
#define PCIE_PM_DSTATE			0x2E8
#define PCIE_PM_DSTATE_MASK		0x0003
#define PCIE_LINKDOWN_RST_CTRL_SEL	0x3A0
#define PCIE_LINKDOWN_RST_MANUAL	(0x1 << 1)
#define PCIE_LINKDOWN_RST_FSM		(0x1 << 0)
#define PCIE_SOFT_RESET			0x3A4
#define SOFT_CORE_RESET			(0x1 << 0)
#define SOFT_PWR_RESET			(0x1 << 1)
#define SOFT_STICKY_RESET		(0x1 << 2)
#define SOFT_NON_STICKY_RESET		(0x1 << 3)
#define SOFT_PHY_RESET			(0x1 << 4)
#define PCIE_QCH_SEL			0x3A8
#define CLOCK_GATING_IN_L12		0x1
#define CLOCK_NOT_GATING		0x3
#define CLOCK_GATING_MASK		0x3
#define CLOCK_GATING_PMU_L1		(0x1 << 11)
#define CLOCK_GATING_PMU_L23READY	(0x1 << 10)
#define CLOCK_GATING_PMU_DETECT_QUIET	(0x1 << 9)
#define CLOCK_GATING_PMU_L12		(0x1 << 8)
#define CLOCK_GATING_PMU_ALL		(0xF << 8)
#define CLOCK_GATING_PMU_MASK		(0xF << 8)
#define CLOCK_GATING_APB_L1		(0x1 << 7)
#define CLOCK_GATING_APB_L23READY	(0x1 << 6)
#define CLOCK_GATING_APB_DETECT_QUIET	(0x1 << 5)
#define CLOCK_GATING_APB_L12		(0X1 << 4)
#define CLOCK_GATING_APB_ALL		(0xF << 4)
#define CLOCK_GATING_APB_MASK		(0xF << 4)
#define CLOCK_GATING_AXI_L1		(0x1 << 3)
#define CLOCK_GATING_AXI_L23READY	(0x1 << 2)
#define CLOCK_GATING_AXI_DETECT_QUIET	(0x1 << 1)
#define CLOCK_GATING_AXI_L12		(0x1 << 0)
#define CLOCK_GATING_AXI_ALL		(0xF << 0)
#define CLOCK_GATING_AXI_MASK		(0xF << 0)
#define PCIE_APP_REQ_EXIT_L1_MODE	0x3BC
#define L1_REQ_NAK_CONTROL		(0x3 << 4)	/* need to check */
#define L1_REQ_NAK_CONTROL_MASTER	(0x1 << 4)
#define PCIE_SW_WAKE			0x3D4
#define PCIE_STATE_HISTORY_CHECK	0xC00
#define HISTORY_BUFFER_ENABLE		0x3
#define HISTORY_BUFFER_CLEAR		(0x1 << 1)
#define HISTORY_BUFFER_CONDITION_SEL	(0x1 << 2)
#define PCIE_STATE_POWER_S		0xC04
#define PCIE_STATE_POWER_M		0xC08
#define PCIE_HISTORY_REG(x)		(0xC0C + ((x) * 0x4)) /* history_reg0: 0xC0C */
#define LTSSM_STATE(x)			(((x) >> 16) & 0x3f)
#define PM_DSTATE(x)			(((x) >> 8) & 0x7)
#define L1SUB_STATE(x)			(((x) >> 0) & 0x7)
#define PCIE_DMA_MONITOR1		0x460
#define PCIE_DMA_MONITOR2		0x464
#define PCIE_DMA_MONITOR3		0x468
#define FSYS1_MON_SEL_MASK		0xf
#define PCIE_MON_SEL_MASK		0xff

#define PCIE_MSTR_PEND_SEL_NAK		0x474
#define NACK_ENABLE			0x1

#define PCIE_DBI_L1_EXIT_DISABLE	0x1078
#define DBI_L1_EXIT_DISABLE		0x1

/* PCIe PMU registers */
#define IDLE_IP3_STATE			0x3EC
#define IDLE_IP_RC1_SHIFT		(31)
#define IDLE_IP_RC0_SHIFT		(30)
#define IDLE_IP3_MASK			0x3FC
#define WAKEUP_MASK			0x3944
#define WAKEUP_MASK_PCIE_WIFI		16
#define PCIE_PHY_CONTROL		0x071C
#define PCIE_PHY_CONTROL_MASK		0x1

/* PCIe DBI registers */
#define PM_CAP_OFFSET			0x40
#define PCIE_CAP_OFFSET			0x70
#define PCIE_LINK_CTRL_STAT		0x80
#define PCIE_LINK_CTRL_LINK_SPEED_MASK	0xf
#define PCIE_LINK_CTRL2_STAT2		0xA0
#define PCIE_LINK_CTRL2_TARGET_LINK_SPEED_MASK	0xfffffff0
#define PCIE_CAP_LINK_SPEED		0xf
#define PCIE_CAP_NEGO_LINK_WIDTH_MASK	0x3f
#define PCI_EXP_LNKCAP_MLW_X1		(0x1 << 4)
#define PCI_EXP_LNKCAP_L1EL_64USEC	(0x7 << 15)
/* previous definition is in 'include/uapi/linux/pci_regs.h:661', PCI_EXP_LNKCTL2_TLS: 0xf */
#define PCI_EXP_LNKCTL2_TLS_2_5GB	0x1
#define PCI_EXP_LNKCTL2_TLS_5_0GB	0x2
#define PCI_EXP_LNKCTL2_TLS_8_0GB	0x3
#define PCIE_CAP_CPL_TIMEOUT_VAL_MASK	0xf
#define PCIE_CAP_CPL_TIMEOUT_VAL_44MS_DEFALT	0x5
#define PCIE_CAP_CPL_TIMEOUT_VAL_6_2MS	0x2
#define PCIE_LINK_L1SS_CONTROL		0x168
#define PORT_LINK_TCOMMON_32US		(0x20 << 8)
#define LTR_L12_THRESHOLD_SCALE_1NS	(0x0 << 29) /* Latency Tolerance Reporting */
#define LTR_L12_THRESHOLD_SCALE_32NS	(0x1 << 29)
#define LTR_L12_THRESHOLD_SCALE_1024NS	(0x2 << 29)
#define LTR_L12_THRESHOLD_SCALE_32768NS	(0x3 << 29)
#define LTR_L12_THRESHOLD_VALUE_160	(0xa0 << 16)
#define PORT_LINK_L12_LTR_THRESHOLD     (0x40a0 << 16)
#define PCIE_LINK_L1SS_CONTROL2		0x16C
#define PORT_LINK_L1SS_ENABLE		(0xf << 0)
#define PORT_LINK_TPOWERON_90US		(0x49 << 0)
#define PORT_LINK_TPOWERON_130US	(0x69 << 0)
#define PORT_LINK_TPOWERON_180US	(0x89 << 0)
#define PORT_LINK_TPOWERON_200US	(0xA1 << 0)
#define PORT_LINK_TPOWERON_3100US	(0xfa << 0)
#define PORT_LINK_L1SS_T_PCLKACK	(0x3 << 6)
#define PORT_LINK_L1SS_T_L1_2		(0x4 << 2)
#define PORT_LINK_L1SS_T_POWER_OFF	(0x2 << 0)
#define PCIE_ACK_F_ASPM_CONTROL		0x70C
#define PCIE_L1_ENTERANCE_LATENCY      (0x7 << 27)
#define PCIE_L1_ENTERANCE_LATENCY_8us  (0x3 << 27)
#define PCIE_L1_ENTERANCE_LATENCY_16us (0x4 << 27)
#define PCIE_L1_ENTERANCE_LATENCY_32us (0x5 << 27)
#define PCIE_L1_ENTERANCE_LATENCY_64us (0x7 << 27)
#define PCIE_PORT_LINK_CONTROL		0x710

#define PCIE_MISC_CONTROL		0x8BC
#define DBI_RO_WR_EN			0x1

#define PCIE_COHERENCY_CONTROL_3_OFF	0x8E8

#define PCIE_AUX_CLK_FREQ_OFF		0xB40
#define PCIE_AUX_CLK_FREQ_24MHZ		0x18
#define PCIE_AUX_CLK_FREQ_26MHZ		0x1A
#define PCIE_L1_SUBSTATES_OFF		0xB44
#define PCIE_L1_SUB_VAL			0xEA

#define LINK_CONTROL2_LINK_STATUS2_REG	0xA0
#define PCIE_CAP_TARGET_LINK_SPEED_MASK 0xfffffff0
#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C
#define DIRECT_SPEED_CHANGE_ENABLE_MASK	0xfffdffff
#define DIRECT_SPEED_CHANGE_ENABLE	0x20000
#define EXYNOS_PORT_LOGIC_SPEED_CHANGE	(0x1 << 17)

#define MULTI_LANE_CONTROL_OFF		0x8c0
#define TARGET_LINK_WIDTH_MASK		0xffffffc0
#define DIRECT_LINK_WIDTH_CHANGE_SET	0x40

#define PCIE_ATU_VIEWPORT		0x900
#define EXYNOS_PCIE_ATU_REGION_INBOUND	(0x1 << 31)
#define EXYNOS_PCIE_ATU_REGION_OUTBOUND	(0x0 << 31)
#define EXYNOS_PCIE_ATU_REGION_INDEX2	(0x2 << 0)
#define EXYNOS_PCIE_ATU_REGION_INDEX1	(0x1 << 0)
#define EXYNOS_PCIE_ATU_REGION_INDEX0	(0x0 << 0)
#define PCIE_ATU_CR1			0x904
#define EXYNOS_PCIE_ATU_TYPE_MEM	(0x0 << 0)
#define EXYNOS_PCIE_ATU_TYPE_IO		(0x2 << 0)
#define EXYNOS_PCIE_ATU_TYPE_CFG0	(0x4 << 0)
#define EXYNOS_PCIE_ATU_TYPE_CFG1	(0x5 << 0)
#define PCIE_ATU_CR2			0x908
#define EXYNOS_PCIE_ATU_ENABLE		(0x1 << 31)
#define EXYNOS_PCIE_ATU_BAR_MODE_ENABLE	(0x1 << 30)
#undef PCIE_ATU_LOWER_BASE
#define PCIE_ATU_LOWER_BASE		0x90C
#undef PCIE_ATU_UPPER_BASE
#define PCIE_ATU_UPPER_BASE		0x910
#undef PCIE_ATU_LIMIT
#define PCIE_ATU_LIMIT			0x914
#undef PCIE_ATU_LOWER_TARGET
#define PCIE_ATU_LOWER_TARGET		0x918
#define EXYNOS_PCIE_ATU_BUS(x)		(((x) & 0xff) << 24)
#define EXYNOS_PCIE_ATU_DEV(x)		(((x) & 0x1f) << 19)
#define EXYNOS_PCIE_ATU_FUNC(x)		(((x) & 0x7) << 16)
#undef PCIE_ATU_UPPER_TARGET
#define PCIE_ATU_UPPER_TARGET		0x91C

#define PCIE_GEN2_CTRL_OFF		0x80C

#define PCIE_MSI_ADDR_LO		0x820
#define PCIE_MSI_ADDR_HI		0x824
#define PCIE_MSI_INTR0_ENABLE		0x828
#define PCIE_MSI_INTR0_MASK		0x82C
#define PCIE_MSI_INTR0_STATUS		0x830

/* PCIe SYSREG registers */
#define PCIE_WIFI0_PCIE_PHY_CONTROL	0xC
#define BIFURCATION_MODE_DISABLE	(0x1 << 16)
#define LINK1_ENABLE			(0x1 << 15)
#define LINK0_ENABLE			(0x1 << 14)
#define PCS_LANE1_ENABLE		(0x1 << 13)
#define PCS_LANE0_ENABLE		(0x1 << 12)

#define PCIE_SYSREG_SHARABILITY_CTRL	0x700
#define PCIE_SYSREG_SHARABLE_OFFSET	8
#define PCIE_SYSREG_SHARABLE_ENABLE	0x3
#define PCIE_SYSREG_SHARABLE_DISABLE	0x0

/* gs101: HSI1(GEN4A_0) & HSI2(GEN4A_1) */
#define PCIE_SYSREG_HSI1_SHARABILITY_CTRL	0x704
#define PCIE_SYSREG_HSI2_SHARABILITY_CTRL	0x730
#define PCIE_SYSREG_HSIX_SHARABLE_OFFSET	0
#define PCIE_SYSREG_HSIX_SHARABLE_ENABLE	0x3
#define PCIE_SYSREG_HSIX_SHARABLE_DISABLE	0x0
#define PCIE_SYSREG_HSIX_SHARABLE_MASK	0x3

/* Definitions for WIFI L1.2 */
#define WIFI_L1SS_CAPID			0x240
#define WIFI_L1SS_CAP			0x244
#define WIFI_L1SS_CONTROL		0x248
#define WIFI_L1SS_CONTROL2		0x24C
#define WIFI_L1SS_LTR_LATENCY		0x1B4
#define WIFI_L1SS_LINKCTRL		0xBC
#define WIFI_LINK_STATUS		0xBE
#define WIFI_PCI_EXP_DEVCTL2		0xD4
#define WIFI_PM_MNG_STATUS_CON		0x4C

/* LINK Control Register */
#define WIFI_ASPM_CONTROL_MASK		(0x3 << 0)
#define WIFI_ASPM_L1_ENTRY_EN		(0x2 << 0)
#define WIFI_USE_SAME_REF_CLK		(0x1 << 6)
#define WIFI_CLK_REQ_EN			(0x1 << 8)

/* L1SS Control Register */
#define WIFI_ALL_PM_ENABEL		(0xf << 0)
#define WIFI_PCIPM_L12_EN		(0x1 << 0)
#define WIFI_PCIPM_L11_EN		(0x1 << 1)
#define WIFI_ASPM_L12_EN		(0x1 << 2)
#define WIFI_ASPM_L11_EN		(0x1 << 3)
#define WIFI_COMMON_RESTORE_TIME	(0xa << 8)	/* Default Value */

/* L1SS LTR Latency Register */
#define MAX_NO_SNOOP_LAT_VALUE_3	(3 << 16)
#define MAX_SNOOP_LAT_VALUE_3		(3 << 0)
#define MAX_NO_SNOOP_LAT_SCALE_MS	(0x4 << 26)	/* 0x4(b'100) value = 1,047,576 ns */
#define MAX_SNOOP_LAT_SCALE_MS		(0x4 << 10)	/* 0x4(b'100) value = 1,047,576 ns */

/* ETC definitions */
#define IGNORE_ELECIDLE			1
#define ENABLE_ELECIDLE			0
#define	PCIE_DISABLE_CLOCK		0
#define	PCIE_ENABLE_CLOCK		1
#define PCIE_IS_IDLE			1
#define PCIE_IS_ACTIVE			0

/* PCIe PHY definitions */
#define PHY_PLL_STATE			0xBC
#define CHK_PHY_PLL_LOCK		0x3
#define PM_POWER_STATE			0x188
#define PM_STATE_MASK			0x07
#define PHY_PCS_REFCLK_EN		(0x1 << 4)
#define PHY_PCS_REFCLK_GATE_L12		(0x1 << 5)

/* For Set NCLK OFF to avoid system hang */
#define EXYNOS_PCIE_MAX_NAME_LEN        10
#define PCIE_L12ERR_CTRL                0x2F0
#define NCLK_OFF_OFFSET                 0x2

#define PCIE_ATU_CR1_OUTBOUND0		0x300000
#define PCIE_ATU_CR2_OUTBOUND0		0x300004
#define PCIE_ATU_LOWER_BASE_OUTBOUND0	0x300008
#define PCIE_ATU_UPPER_BASE_OUTBOUND0	0x30000C
#define PCIE_ATU_LIMIT_OUTBOUND0	0x300010
#define PCIE_ATU_LOWER_TARGET_OUTBOUND0	0x300014
#define PCIE_ATU_UPPER_TARGET_OUTBOUND0	0x300018

#define PCIE_ATU_CR1_OUTBOUND1		0x300200
#define PCIE_ATU_CR2_OUTBOUND1		0x300204
#define PCIE_ATU_LOWER_BASE_OUTBOUND1	0x300208
#define PCIE_ATU_UPPER_BASE_OUTBOUND1	0x30020C
#define PCIE_ATU_LIMIT_OUTBOUND1	0x300210
#define PCIE_ATU_LOWER_TARGET_OUTBOUND1	0x300214
#define PCIE_ATU_UPPER_TARGET_OUTBOUND1	0x300218

#define PCIE_ATU_CR1_OUTBOUND2		0x300400
#define PCIE_ATU_CR2_OUTBOUND2		0x300404
#define PCIE_ATU_LOWER_BASE_OUTBOUND2	0x300408
#define PCIE_ATU_UPPER_BASE_OUTBOUND2	0x30040C
#define PCIE_ATU_LIMIT_OUTBOUND2	0x300410
#define PCIE_ATU_LOWER_TARGET_OUTBOUND2	0x300414
#define PCIE_ATU_UPPER_TARGET_OUTBOUND2	0x300418

#define EXYNOS_IP_VER_OF_WHI   0x984500

#define EOM_PH_SEL_MAX                  128
#define EOM_DEF_VREF_MAX                256
#define VREF_MAX                        256
#define PHASE_MAX                       128

#define RX_CDR_LOCK                     0xE0C
#define RX_EFOM_DONE                    0x140C
#define RX_EFOM_BIT_WIDTH_SEL           0x12AC
#define ANA_RX_DFE_EOM_PI_STR_CTRL      0x988
#define ANA_RX_DFE_EOM_PI_DIVSEL_G12    0x980
#define ANA_RX_DFE_EOM_PI_DIVSEL_G32    0x984
#define RX_FOM_EN                       0x1050
#define RX_SSLMS_ADAP_COEF_SEL_7_0      0x119C
#define RX_EFOM_SETTLE_TIME             0x12A8
#define RX_EFOM_EOM_PH_SEL              0x1558
#define RX_EFOM_MODE                    0x1548
#define RX_SSLMS_ADAP_HOLD_PMAD         0x11A0
#define RX_EFOM_NUM_OF_SAMPLE           0x154C
#define RX_EFOM_NUM_OF_SAMPLE_13_8      0x154C
#define RX_EFOM_NUM_OF_SAMPLE_7_0       0x154C
#define MON_RX_EFOM_ERR_CNT_13_8        0x1564
#define MON_RX_EFOM_ERR_CNT_7_0         0x1560
#define RX_EFOM_DFE_VREF_CTRL           0x1550
#define RX_EFOM_START                   0x1540
#define RX_EFOM_NUMOF_SMPL_13_8         0xCAC
#define RX_EFOM_NUMOF_SMPL_7_0          0xCB0

struct pcie_eom_result {
	unsigned int phase;
	unsigned int vref;
	unsigned long err_cnt;
};

void exynos_pcie_rc_phy_init(struct dw_pcie_rp *pp);

#if !IS_ENABLED(CONFIG_EXYNOS_PCIE_IOMMU)
extern struct dma_map_ops exynos_pcie_dma_ops;

static void __maybe_unused pcie_sysmmu_enable(int hsi_block_num)
{
	pr_err("PCIe SysMMU is Enabled!!!\n");
}

static void __maybe_unused pcie_sysmmu_disable(int hsi_block_num)
{
	pr_err("PCIe SysMMU is NOT Enabled!!!\n");
}

static int __maybe_unused pcie_iommu_map(unsigned long iova, phys_addr_t paddr,
					 size_t size, int prot, int hsi_block_num)
{
	pr_err("PCIe SysMMU is Mapped!!!\n");
	return -ENODEV;
}

static size_t __maybe_unused pcie_iommu_unmap(unsigned long iova, size_t size,
					    int hsi_block_num)
{
	pr_err("PCIe SysMMU is Unmapped!!!\n");
	return 0;
}
#endif

#if !IS_ENABLED(CONFIG_GS_S2MPU)
static void __maybe_unused s2mpu_update_refcnt(struct device *dev,
					       dma_addr_t dma_addr, size_t size,
					       bool incr, enum dma_data_direction dir)
{
	pr_err("PCIe S2MPU is NOT Enabled!!!\n");
}
#endif

u32 exynos_pcie_rc_read_dbi(struct dw_pcie *dw_pcie, void __iomem *base, u32 reg, size_t size);
void exynos_pcie_rc_write_dbi(struct dw_pcie *dw_pcie, void __iomem *base,
			      u32 reg, size_t size, u32 val);
int exynos_pcie_rc_poweron(int ch_num);
void exynos_pcie_rc_poweroff(int ch_num);
int exynos_pcie_rc_l1ss_ctrl(int enable, int id, int ch_num);
int exynos_pcie_rc_set_outbound_atu(int ch_num, u32 target_addr, u32 offset, u32 size);
int exynos_pcie_rc_check_link_speed(int ch_num);
int exynos_pcie_rc_change_link_speed(int ch_num, int target_speed);
int exynos_pcie_l1_exit(int ch_num);
#endif
