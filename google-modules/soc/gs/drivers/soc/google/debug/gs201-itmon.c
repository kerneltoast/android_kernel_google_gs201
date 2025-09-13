// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/panic_notifier.h>
#include <soc/google/exynos-itmon.h>
#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-cpupm.h>

#define OFFSET_TMOUT_REG		(0x2000)
#define OFFSET_REQ_R			(0x0)
#define OFFSET_REQ_W			(0x20)
#define OFFSET_RESP_R			(0x40)
#define OFFSET_RESP_W			(0x60)
#define OFFSET_ERR_REPT			(0x20)
#define OFFSET_PROT_CHK			(0x100)
#define OFFSET_NUM			(0x4)

#define REG_INT_MASK			(0x0)
#define REG_INT_CLR			(0x4)
#define REG_INT_INFO			(0x8)
#define REG_EXT_INFO_0			(0x10)
#define REG_EXT_INFO_1			(0x14)
#define REG_EXT_INFO_2			(0x18)
#define REG_EXT_USER			(0x80)

#define REG_DBG_CTL			(0x10)
#define REG_TMOUT_INIT_VAL		(0x14)
#define REG_TMOUT_FRZ_EN		(0x18)
#define REG_TMOUT_FRZ_STATUS		(0x1C)
#define REG_TMOUT_BUF_WR_OFFSET		(0x20)

#define REG_TMOUT_BUF_POINT_ADDR	(0x20)
#define REG_TMOUT_BUF_ID		(0x24)

#define REG_TMOUT_BUF_PAYLOAD_0		(0x28)
#define REG_TMOUT_BUF_PAYLOAD_1		(0x30)
#define REG_TMOUT_BUF_PAYLOAD_2		(0x34)
#define REG_TMOUT_BUF_PAYLOAD_3		(0x38)
#define REG_TMOUT_BUF_PAYLOAD_4		(0x3C)

#define REG_PROT_CHK_CTL		(0x4)
#define REG_PROT_CHK_INT		(0x8)
#define REG_PROT_CHK_INT_ID		(0xC)
#define REG_PROT_CHK_START_ADDR_LOW	(0x10)
#define REG_PROT_CHK_END_ADDR_LOW	(0x14)
#define REG_PROT_CHK_START_END_ADDR_UPPER	(0x18)

#define RD_RESP_INT_ENABLE		(1 << 0)
#define WR_RESP_INT_ENABLE		(1 << 1)
#define ARLEN_RLAST_INT_ENABLE		(1 << 2)
#define AWLEN_WLAST_INT_ENABLE		(1 << 3)
#define INTEND_ACCESS_INT_ENABLE	(1 << 4)

#define BIT_PROT_CHK_ERR_OCCURRED(x)	(((x) & (0x1 << 0)) >> 0)
#define BIT_PROT_CHK_ERR_CODE(x)	(((x) & (0x7 << 1)) >> 1)

#define BIT_ERR_CODE(x)			(((x) & (0xF << 28)) >> 28)
#define BIT_ERR_OCCURRED(x)		(((x) & (0x1 << 27)) >> 27)
#define BIT_ERR_VALID(x)		(((x) & (0x1 << 26)) >> 26)
#define BIT_AXID(x)			(((x) & (0xFFFF)))
#define BIT_AXUSER(x)			(((x) & (0xFFFFFFFF)))
#define BIT_AXUSER_PERI(x)		(((x) & (0xFFFF << 16)) >> 16)
#define BIT_AXBURST(x)			(((x) & (0x3)))
#define BIT_AXPROT(x)			(((x) & (0x3 << 2)) >> 2)
#define BIT_AXLEN(x)			(((x) & (0xF << 16)) >> 16)
#define BIT_AXSIZE(x)			(((x) & (0x7 << 28)) >> 28)

#define ERRCODE_SLVERR			(0)
#define ERRCODE_DECERR			(1)
#define ERRCODE_UNSUPPORTED		(2)
#define ERRCODE_POWER_DOWN		(3)
#define ERRCODE_UNKNOWN_4		(4)
#define ERRCODE_UNKNOWN_5		(5)
#define ERRCODE_TMOUT			(6)

#define DATA				(0)
#define PERI				(1)
#define BUS_PATH_TYPE			(2)

#define TRANS_TYPE_WRITE		(0)
#define TRANS_TYPE_READ			(1)
#define TRANS_TYPE_NUM			(2)

#define FROM_CP				(0)
#define FROM_CPU			(2)

#define NOT_AVAILABLE_STR		"N/A"

#define TMOUT				(0xFFFFF)
#define TMOUT_TEST			(0x1)

#define PANIC_THRESHOLD			(10)

/* This value will be fixed */
#define INTEND_ADDR_START		(0)
#define INTEND_ADDR_END			(0)

#define log_dev_err(dev, fmt, ...)	\
do {									\
	dev_printk_emit(LOGLEVEL_ERR, dev, fmt, ##__VA_ARGS__);		\
	dbg_snapshot_itmon_backup_log(fmt, ##__VA_ARGS__);		\
} while (0)

#define log_dev_info(dev, fmt, ...)	\
do {									\
	dev_printk_emit(LOGLEVEL_INFO, dev, fmt, ##__VA_ARGS__);	\
	dbg_snapshot_itmon_backup_log(fmt, ##__VA_ARGS__);		\
} while (0)

#define log_dev_dbg(dev, fmt, ...)	\
do {									\
	dev_printk_emit(LOGLEVEL_DEBUG, dev, fmt, ##__VA_ARGS__);	\
	dbg_snapshot_itmon_backup_log(fmt, ##__VA_ARGS__);		\
} while (0)

enum err_type {
	FATAL = 0,
	DREX_TMOUT,
	CPU,
	IP,
	UNHANDLED,
	TYPE_MAX,
};

struct itmon_policy {
	char *name;
	int policy;
	bool error;
};

struct itmon_rpathinfo {
	unsigned int id;
	char *port_name;
	char *dest_name;
	unsigned int bits;
	unsigned int shift_bits;
};

struct itmon_clientinfo {
	char *port_name;
	unsigned int user;
	char *client_name;
	unsigned int bits;
};

struct itmon_nodegroup;

struct itmon_traceinfo {
	char *port;
	char *client;
	unsigned int user;
	char *dest;
	unsigned long target_addr;
	unsigned int errcode;
	bool read;
	bool onoff;
	bool path_dirty;
	bool dirty;
	unsigned long from;
	int path_type;
	char buf[SZ_32];
	unsigned int axsize;
	unsigned int axlen;
	unsigned int axburst;
	unsigned int axprot;
	bool baaw_prot;
	struct list_head list;
};

struct itmon_tracedata {
	unsigned int int_info;
	unsigned int ext_info_0;
	unsigned int ext_info_1;
	unsigned int ext_info_2;
	unsigned int ext_user;
	unsigned int dbg_mo_cnt;
	unsigned int prot_chk_ctl;
	unsigned int prot_chk_info;
	unsigned int prot_chk_int_id;
	unsigned int offset;
	struct itmon_traceinfo *ref_info;
	bool logging;
	bool read;
};

struct itmon_nodeinfo {
	unsigned int type;
	char *name;
	unsigned int phy_regs;
	bool err_enabled;
	bool prot_chk_enabled;
	bool addr_detect_enabled;
	bool retention;
	unsigned int time_val;
	bool tmout_enabled;
	bool tmout_frz_enabled;
	void __iomem *regs;
	struct itmon_tracedata tracedata;
	struct itmon_nodegroup *group;
	struct list_head list;
};

struct itmon_nodegroup {
	char *name;
	unsigned int phy_regs;
	bool ex_table;
	struct itmon_nodeinfo *nodeinfo;
	unsigned int nodesize;
	unsigned int path_type;
	void __iomem *regs;
	int irq;
};

struct itmon_platdata {
	const struct itmon_rpathinfo *rpathinfo;
	const struct itmon_clientinfo *clientinfo;
	struct itmon_nodegroup *nodegroup;
	struct list_head infolist[TRANS_TYPE_NUM];
	struct list_head datalist[TRANS_TYPE_NUM];
	ktime_t last_time;
	bool cp_crash_in_progress;
	unsigned int sysfs_tmout_val;

	struct itmon_policy *policy;
	unsigned int err_cnt_by_cpu;
	unsigned int panic_threshold;
	bool in_do_policy;
	bool probed;
};

struct itmon_dev {
	struct device *dev;
	struct itmon_platdata *pdata;
	int irq;
	int id;
	void __iomem *regs;
	spinlock_t ctrl_lock;
	struct itmon_notifier notifier_info;
	char itmon_pattern[SZ_32];
	atomic_t itmon_pattern_cnt;
};

struct itmon_panic_block {
	struct notifier_block nb_panic_block;
	struct itmon_dev *pdev;
};

static struct itmon_policy err_policy[] = {
	[FATAL]		= {"err_fatal",		0, false},
	[DREX_TMOUT]	= {"err_drex_tmout",	0, false},
	[CPU]		= {"err_cpu",		0, false},
	[IP]		= {"err_ip",		0, false},
	[UNHANDLED]	= {"err_unhandled",	0, false},
};

const static struct itmon_rpathinfo rpathinfo[] = {
	/* 0x8000_0000 - 0xf_ffff_ffff */

	/* NOCL0_M0 / M1 / M2 / M3 CPU specific */
	{0,	"CPU0",		"NOCL0_M0",	0x3F, 0},
	{0,	"CPU1",		"NOCL0_M1",	0x3F, 0},
	{0,	"CPU2",		"NOCL0_M2",	0x3F, 0},
	{0,	"CPU3",		"NOCL0_M3",	0x3F, 0},

	/* CORE_M0 / M1 / M2 / M3 Common */
	{1,	"AUR1",		"NOCL0_M",	0x3F, 0},
	{2,	"GPUMMU",	"NOCL0_M",	0x3F, 0},
	{3,	"AUR0",		"NOCL0_M",	0x3F, 0},
	{4,	"GPU0",		"NOCL0_M",	0x3F, 0},
	{5,	"GPU1",		"NOCL0_M",	0x3F, 0},
	{6,	"GPU2",		"NOCL0_M",	0x3F, 0},
	{7,	"GPU3",		"NOCL0_M",	0x3F, 0},
	{8,	"TPU",		"NOCL0_M",	0x3F, 0},
	{9,	"DPU0",		"NOCL0_M",	0x3F, 0},
	{10,	"DPU1",		"NOCL0_M",	0x3F, 0},
	{11,	"DPU2",		"NOCL0_M",	0x3F, 0},
	{12,	"CSIS0",	"NOCL0_M",	0x3F, 0},
	{13,	"CSIS1",	"NOCL0_M",	0x3F, 0},
	{14,	"DNS",		"NOCL0_M",	0x3F, 0},
	{15,	"GDC0",		"NOCL0_M",	0x3F, 0},
	{16,	"GDC1",		"NOCL0_M",	0x3F, 0},
	{17,	"GDC2",		"NOCL0_M",	0x3F, 0},
	{18,	"G3AA",		"NOCL0_M",	0x3F, 0},
	{19,	"IPP",		"NOCL0_M",	0x3F, 0},
	{20,	"MCSC0",	"NOCL0_M",	0x3F, 0},
	{21,	"MCSC1",	"NOCL0_M",	0x3F, 0},
	{22,	"MCSC2",	"NOCL0_M",	0x3F, 0},
	{23,	"TNR0",		"NOCL0_M",	0x3F, 0},
	{24,	"TNR1",		"NOCL0_M",	0x3F, 0},
	{25,	"TNR2",		"NOCL0_M",	0x3F, 0},
	{26,	"TNR3",		"NOCL0_M",	0x3F, 0},
	{27,	"TNR4",		"NOCL0_M",	0x3F, 0},
	{28,	"BO",		"NOCL0_M",	0x3F, 0},
	{29,	"G2D0",		"NOCL0_M",	0x3F, 0},
	{30,	"G2D1",		"NOCL0_M",	0x3F, 0},
	{31,	"G2D2",		"NOCL0_M",	0x3F, 0},
	{32,	"HSI2",		"NOCL0_M",	0x3F, 0},
	{33,	"MFC0",		"NOCL0_M",	0x3F, 0},
	{34,	"MFC1",		"NOCL0_M",	0x3F, 0},
	{35,	"MISC",		"NOCL0_M",	0x3F, 0},
	{36,	"ALIVE",	"NOCL0_M",	0x3F, 0},
	{37,	"AOC",		"NOCL0_M",	0x3F, 0},
	{38,	"CSSYS",	"NOCL0_M",	0x3F, 0},
	{39,	"GSA",		"NOCL0_M",	0x3F, 0},
	{40,	"HSI0",		"NOCL0_M",	0x3F, 0},
	{41,	"HSI1",		"NOCL0_M",	0x3F, 0},


	/* 0x0 - 0x7fff_ffff */

	{0,	"CPU0",		"NOCL0_DP",	0x3F, 0},
	{1,	"CPU1",		"NOCL0_DP",	0x3F, 0},
	{2,	"CPU2",		"NOCL0_DP",	0x3F, 0},
	{3,	"CPU3",		"NOCL0_DP",	0x3F, 0},
	{4,	"AUR1",		"NOCL0_DP",	0x3F, 0},
	{5,	"GPUMMU",	"NOCL0_DP",	0x3F, 0},
	{6,	"AUR0",		"NOCL0_DP",	0x3F, 0},
	{7,	"GPU0",		"NOCL0_DP",	0x3F, 0},
	{8,	"GPU1",		"NOCL0_DP",	0x3F, 0},
	{9,	"GPU2",		"NOCL0_DP",	0x3F, 0},
	{10,	"GPU3",		"NOCL0_DP",	0x3F, 0},
	{11,	"TPU",		"NOCL0_DP",	0x3F, 0},
	{12,	"DPU0",		"NOCL0_DP",	0x3F, 0},
	{13,	"DPU1",		"NOCL0_DP",	0x3F, 0},
	{14,	"DPU2",		"NOCL0_DP",	0x3F, 0},
	{15,	"CSIS0",	"NOCL0_DP",	0x3F, 0},
	{16,	"CSIS1",	"NOCL0_DP",	0x3F, 0},
	{17,	"DNS",		"NOCL0_DP",	0x3F, 0},
	{18,	"GDC0",		"NOCL0_DP",	0x3F, 0},
	{19,	"GDC1",		"NOCL0_DP",	0x3F, 0},
	{20,	"GDC2",		"NOCL0_DP",	0x3F, 0},
	{21,	"G3AA",		"NOCL0_DP",	0x3F, 0},
	{22,	"IPP",		"NOCL0_DP",	0x3F, 0},
	{23,	"MCSC0",	"NOCL0_DP",	0x3F, 0},
	{24,	"MCSC1",	"NOCL0_DP",	0x3F, 0},
	{25,	"MCSC2",	"NOCL0_DP",	0x3F, 0},
	{26,	"TNR0",		"NOCL0_DP",	0x3F, 0},
	{27,	"TNR1",		"NOCL0_DP",	0x3F, 0},
	{28,	"TNR2",		"NOCL0_DP",	0x3F, 0},
	{29,	"TNR3",		"NOCL0_DP",	0x3F, 0},
	{30,	"TNR4",		"NOCL0_DP",	0x3F, 0},
	{31,	"BO",		"NOCL0_DP",	0x3F, 0},
	{32,	"G2D0",		"NOCL0_DP",	0x3F, 0},
	{33,	"G2D1",		"NOCL0_DP",	0x3F, 0},
	{34,	"G2D2",		"NOCL0_DP",	0x3F, 0},
	{35,	"MFC0",		"NOCL0_DP",	0x3F, 0},
	{36,	"MFC1",		"NOCL0_DP",	0x3F, 0},
	{37,	"HSI2",		"NOCL0_DP",	0x3F, 0},
	{38,	"MISC",		"NOCL0_DP",	0x3F, 0},
	{39,	"ALIVE",	"NOCL0_DP",	0x3F, 0},
	{40,	"AOC",		"NOCL0_DP",	0x3F, 0},
	{41,	"CSSYS",	"NOCL0_DP",	0x3F, 0},
	{42,	"GSA",		"NOCL0_DP",	0x3F, 0},
	{43,	"HSI0",		"NOCL0_DP",	0x3F, 0},
	{44,	"HSI1",		"NOCL0_DP",	0x3F, 0},

	{0,	"AUR1",		"CORE_CCI",	0x7F, 0},
	{35,	"AUR1",		"CORE_CCI",	0x7F, 0},
	{70,	"AUR1",		"CORE_CCI",	0x7F, 0},
	{105,	"AUR1",		"CORE_CCI",	0x7F, 0},
	{1,	"GPUMMU",	"CORE_CCI",	0x7F, 0},
	{36,	"GPUMMU",	"CORE_CCI",	0x7F, 0},
	{71,	"GPUMMU",	"CORE_CCI",	0x7F, 0},
	{106,	"GPUMMU",	"CORE_CCI",	0x7F, 0},
	{2,	"AUR0",		"CORE_CCI",	0x7F, 0},
	{37,	"AUR0",		"CORE_CCI",	0x7F, 0},
	{72,	"AUR0",		"CORE_CCI",	0x7F, 0},
	{107,	"AUR0",		"CORE_CCI",	0x7F, 0},
	{3,	"GPU0",		"CORE_CCI",	0x7F, 0},
	{38,	"GPU0",		"CORE_CCI",	0x7F, 0},
	{73,	"GPU0",		"CORE_CCI",	0x7F, 0},
	{108,	"GPU0",		"CORE_CCI",	0x7F, 0},
	{4,	"GPU1",		"CORE_CCI",	0x7F, 0},
	{39,	"GPU1",		"CORE_CCI",	0x7F, 0},
	{74,	"GPU1",		"CORE_CCI",	0x7F, 0},
	{109,	"GPU1",		"CORE_CCI",	0x7F, 0},
	{5,	"GPU2",		"CORE_CCI",	0x7F, 0},
	{40,	"GPU2",		"CORE_CCI",	0x7F, 0},
	{75,	"GPU2",		"CORE_CCI",	0x7F, 0},
	{110,	"GPU2",		"CORE_CCI",	0x7F, 0},
	{6,	"GPU3",		"CORE_CCI",	0x7F, 0},
	{41,	"GPU3",		"CORE_CCI",	0x7F, 0},
	{76,	"GPU3",		"CORE_CCI",	0x7F, 0},
	{111,	"GPU3",		"CORE_CCI",	0x7F, 0},
	{7,	"TPU",		"CORE_CCI",	0x7F, 0},
	{42,	"TPU",		"CORE_CCI",	0x7F, 0},
	{77,	"TPU",		"CORE_CCI",	0x7F, 0},
	{112,	"TPU",		"CORE_CCI",	0x7F, 0},
	{9,	"DPU0",		"CORE_CCI",	0x7F, 0},
	{44,	"DPU0",		"CORE_CCI",	0x7F, 0},
	{79,	"DPU0",		"CORE_CCI",	0x7F, 0},
	{114,	"DPU0",		"CORE_CCI",	0x7F, 0},
	{9,	"DPU1",		"CORE_CCI",	0x7F, 0},
	{44,	"DPU1",		"CORE_CCI",	0x7F, 0},
	{79,	"DPU1",		"CORE_CCI",	0x7F, 0},
	{114,	"DPU1",		"CORE_CCI",	0x7F, 0},
	{10,	"DPU2",		"CORE_CCI",	0x7F, 0},
	{45,	"DPU2",		"CORE_CCI",	0x7F, 0},
	{80,	"DPU2",		"CORE_CCI",	0x7F, 0},
	{115,	"DPU2",		"CORE_CCI",	0x7F, 0},
	{11,	"CSIS0",	"CORE_CCI",	0x7F, 0},
	{46,	"CSIS0",	"CORE_CCI",	0x7F, 0},
	{81,	"CSIS0",	"CORE_CCI",	0x7F, 0},
	{116,	"CSIS0",	"CORE_CCI",	0x7F, 0},
	{12,	"CSIS1",	"CORE_CCI",	0x7F, 0},
	{47,	"CSIS1",	"CORE_CCI",	0x7F, 0},
	{82,	"CSIS1",	"CORE_CCI",	0x7F, 0},
	{117,	"CSIS1",	"CORE_CCI",	0x7F, 0},
	{13,	"DNS",		"CORE_CCI",	0x7F, 0},
	{48,	"DNS",		"CORE_CCI",	0x7F, 0},
	{82,	"DNS",		"CORE_CCI",	0x7F, 0},
	{117,	"DNS",		"CORE_CCI",	0x7F, 0},
	{14,	"GDC0",		"CORE_CCI",	0x7F, 0},
	{49,	"GDC0",		"CORE_CCI",	0x7F, 0},
	{84,	"GDC0",		"CORE_CCI",	0x7F, 0},
	{119,	"GDC0",		"CORE_CCI",	0x7F, 0},
	{15,	"GDC1",		"CORE_CCI",	0x7F, 0},
	{50,	"GDC1",		"CORE_CCI",	0x7F, 0},
	{85,	"GDC1",		"CORE_CCI",	0x7F, 0},
	{120,	"GDC1",		"CORE_CCI",	0x7F, 0},
	{16,	"GDC2",		"CORE_CCI",	0x7F, 0},
	{51,	"GDC2",		"CORE_CCI",	0x7F, 0},
	{86,	"GDC2",		"CORE_CCI",	0x7F, 0},
	{121,	"GDC2",		"CORE_CCI",	0x7F, 0},
	{17,	"G3AA",		"CORE_CCI",	0x7F, 0},
	{52,	"G3AA",		"CORE_CCI",	0x7F, 0},
	{87,	"G3AA",		"CORE_CCI",	0x7F, 0},
	{122,	"G3AA",		"CORE_CCI",	0x7F, 0},
	{17,	"IPP",		"CORE_CCI",	0x7F, 0},
	{52,	"IPP",		"CORE_CCI",	0x7F, 0},
	{88,	"IPP",		"CORE_CCI",	0x7F, 0},
	{113,	"IPP",		"CORE_CCI",	0x7F, 0},
	{19,	"MCSC0",	"CORE_CCI",	0x7F, 0},
	{54,	"MCSC0",	"CORE_CCI",	0x7F, 0},
	{89,	"MCSC0",	"CORE_CCI",	0x7F, 0},
	{124,	"MCSC0",	"CORE_CCI",	0x7F, 0},
	{20,	"MCSC1",	"CORE_CCI",	0x7F, 0},
	{55,	"MCSC1",	"CORE_CCI",	0x7F, 0},
	{90,	"MCSC1",	"CORE_CCI",	0x7F, 0},
	{125,	"MCSC1",	"CORE_CCI",	0x7F, 0},
	{21,	"MCSC2",	"CORE_CCI",	0x7F, 0},
	{56,	"MCSC2",	"CORE_CCI",	0x7F, 0},
	{91,	"MCSC2",	"CORE_CCI",	0x7F, 0},
	{126,	"MCSC2",	"CORE_CCI",	0x7F, 0},
	{22,	"TNR0",		"CORE_CCI",	0x7F, 0},
	{57,	"TNR0",		"CORE_CCI",	0x7F, 0},
	{92,	"TNR0",		"CORE_CCI",	0x7F, 0},
	{127,	"TNR0",		"CORE_CCI",	0x7F, 0},
	{23,	"TNR1",		"CORE_CCI",	0x7F, 0},
	{58,	"TNR1",		"CORE_CCI",	0x7F, 0},
	{93,	"TNR1",		"CORE_CCI",	0x7F, 0},
	{128,	"TNR1",		"CORE_CCI",	0x7F, 0},
	{24,	"TNR2",		"CORE_CCI",	0x7F, 0},
	{59,	"TNR2",		"CORE_CCI",	0x7F, 0},
	{94,	"TNR2",		"CORE_CCI",	0x7F, 0},
	{129,	"TNR2",		"CORE_CCI",	0x7F, 0},
	{25,	"TNR3",		"CORE_CCI",	0x7F, 0},
	{60,	"TNR3",		"CORE_CCI",	0x7F, 0},
	{95,	"TNR3",		"CORE_CCI",	0x7F, 0},
	{130,	"TNR3",		"CORE_CCI",	0x7F, 0},
	{26,	"TNR4",		"CORE_CCI",	0x7F, 0},
	{61,	"TNR4",		"CORE_CCI",	0x7F, 0},
	{96,	"TNR4",		"CORE_CCI",	0x7F, 0},
	{131,	"TNR4",		"CORE_CCI",	0x7F, 0},
	{27,	"BO",		"CORE_CCI",	0x7F, 0},
	{62,	"BO",		"CORE_CCI",	0x7F, 0},
	{97,	"BO",		"CORE_CCI",	0x7F, 0},
	{132,	"BO",		"CORE_CCI",	0x7F, 0},
	{28,	"G2D0",		"CORE_CCI",	0x7F, 0},
	{63,	"G2D0",		"CORE_CCI",	0x7F, 0},
	{98,	"G2D0",		"CORE_CCI",	0x7F, 0},
	{133,	"G2D0",		"CORE_CCI",	0x7F, 0},
	{29,	"G2D1",		"CORE_CCI",	0x7F, 0},
	{64,	"G2D1",		"CORE_CCI",	0x7F, 0},
	{99,	"G2D1",		"CORE_CCI",	0x7F, 0},
	{134,	"G2D1",		"CORE_CCI",	0x7F, 0},
	{30,	"G2D2",		"CORE_CCI",	0x7F, 0},
	{65,	"G2D2",		"CORE_CCI",	0x7F, 0},
	{100,	"G2D2",		"CORE_CCI",	0x7F, 0},
	{135,	"G2D2",		"CORE_CCI",	0x7F, 0},
	{31,	"HSI2",		"CORE_CCI",	0x7F, 0},
	{66,	"HSI2",		"CORE_CCI",	0x7F, 0},
	{101,	"HSI2",		"CORE_CCI",	0x7F, 0},
	{136,	"HSI2",		"CORE_CCI",	0x7F, 0},
	{32,	"MFC0",		"CORE_CCI",	0x7F, 0},
	{67,	"MFC0",		"CORE_CCI",	0x7F, 0},
	{102,	"MFC0",		"CORE_CCI",	0x7F, 0},
	{137,	"MFC0",		"CORE_CCI",	0x7F, 0},
	{33,	"MFC1",		"CORE_CCI",	0x7F, 0},
	{68,	"MFC1",		"CORE_CCI",	0x7F, 0},
	{103,	"MFC1",		"CORE_CCI",	0x7F, 0},
	{138,	"MFC1",		"CORE_CCI",	0x7F, 0},
	{34,	"MISC",		"CORE_CCI",	0x7F, 0},
	{69,	"MISC",		"CORE_CCI",	0x7F, 0},
	{104,	"MISC",		"CORE_CCI",	0x7F, 0},
	{139,	"MISC",		"CORE_CCI",	0x7F, 0},
	{140,	"ALIVE",	"CORE_CCI",	0x7F, 0},
	{141,	"AOC",		"CORE_CCI",	0x7F, 0},
	{142,	"CSSYS",	"CORE_CCI",	0x7F, 0},
	{143,	"GSA",		"CORE_CCI",	0x7F, 0},
	{144,	"HSI0",		"CORE_CCI",	0x7F, 0},
	{145,	"HSI1",		"CORE_CCI",	0x7F, 0},
};

const static struct itmon_clientinfo clientinfo[] = {
	{"GSA",		0x1, /*XXXX01*/		"SYSMMU_S2_GSA",	0x3},
	{"GSA",		0x2, /*XXXX10*/		"SYSMMU_S1_GSA",	0x3},
	{"GSA",		0x4, /*XXX100*/		"GME",			0x7},
	{"GSA",		0x0, /*000000*/		"CA32",			0x3F},
	{"GSA",		0x10,/*010000*/		"SSS_GSACORE",		0x3F},
	{"GSA",		0x20,/*100000*/		"DMA_GSACORE",		0x3F},
	{"GSA",		0x30,/*110000*/		"DAP",			0x3F},

	{"ALIVE",	0x1, /*XXXXX1*/		"SYSMMU_S2_APM",	0x1},
	{"ALIVE",	0x0, /*XX0000*/		"APM",			0xF},

	{"HSI0",	0x1, /*XXXX01*/		"SYSMMU_S2_HSI0",	0x3},
	{"HSI0",	0x2, /*XXXX10*/		"SYSMMU_S1_HSI0",	0x3},
	{"HSI0",	0x0, /*XXX000*/		"USB31DRD_LINK",	0x7},
	{"HSI0",	0x4, /*XXX100*/		"USB PCS",		0x7},

	{"HSI1",	0x1, /*XXXX01*/		"SYSMMU_S2_HSI1",	0x3},
	{"HSI1",	0x2, /*XXXX10*/		"SYSMMU_S1_HSI1",	0x3},
	{"HSI1",	0x0, /*XXX000*/		"PCIE_GEN4A",		0x7},
	{"HSI1",	0x2, /*XXX100*/		"PCIE_GEN4B",		0x7},

	{"CSSYS",	0x4, /*XXX100*/		"SYSMMU_S2_CPUCL0",	0x7},
	{"CSSYS",	0x0, /*XXX000*/		"CORESIGHT(ETR)",	0x7},
	{"CSSYS",	0x1, /*XXXX01*/		"CORESIGHT(AXI-AP)",	0x3},
	{"CSSYS",	0x2, /*XXXX10*/		"DBGC",			0x3},

	{"AOC",		0x1, /*XXXX01*/		"SYSMMU_S2_AOC",	0x3},
	{"AOC",		0x2, /*XXXX10*/		"SYSMMU_S1_AOC",	0x3},
	{"AOC",		0x0, /*XXXX00*/		"A32",			0x1F},
	{"AOC",		0x4, /*X00100*/		"HF0",			0x1F},
	{"AOC",		0x8, /*X01000*/		"HF1",			0x1F},
	{"AOC",		0xc, /*X01100*/		"F1",			0x1F},

	{"DPU0",	0x1, /*XXXX01*/		"SYSMMU_S2_DPU0",	0x3},
	{"DPU0",	0x2, /*XXXX10*/		"SYSMMU_S1_DPU0",	0x3},
	{"DPU0",	0x0, /*XXXX00*/		"DPU(M0)",		0x3},

	{"DPU1",	0x1, /*XXXX01*/		"SYSMMU_S2_DPU1",	0x3},
	{"DPU1",	0x2, /*XXXX10*/		"SYSMMU_S1_DPU1",	0x3},
	{"DPU1",	0x0, /*XXXX00*/		"DPU(M1)",		0x3},

	{"DPU2",	0x1, /*XXXX01*/		"SYSMMU_S2_DPU1",	0x3},
	{"DPU2",	0x2, /*XXXX10*/		"SYSMMU_S1_DPU1",	0x3},
	{"DPU2",	0x0, /*XXXX00*/		"DPU(M2)",		0x3},

	{"CSIS0",	0x1, /*XXXX01*/		"SYSMMU_S2_CSIS0",	0x3},
	{"CSIS0",	0x2, /*XXXX10*/		"SYSMMU_S1_CSIS0",	0x3},
	{"CSIS0",	0x0, /*X00000*/		"CSIC(DMA2)",		0x1F},
	{"CSIS0",	0x4, /*X00100*/		"CSIC(DMA3)",		0x1F},
	{"CSIS0",	0x8, /*X01000*/		"CSIC(ZSL0)",		0x1F},
	{"CSIS0",	0xC, /*X01100*/		"CSIC(ZSL1)",		0x1F},
	{"CSIS0",	0x10,/*X10000*/		"CSIC(ZSL2)",		0x1F},

	{"CSIS1",	0x1, /*XXXX01*/		"SYSMMU_S2_CSIS1",	0x3},
	{"CSIS1",	0x2, /*XXXX10*/		"SYSMMU_S1_CSIS1",	0x3},
	{"CSIS1",	0x0, /*000000*/		"CSIC(DMA0)",		0x3F},
	{"CSIS1",	0x8, /*001000*/		"CSIC(DMA1)",		0x3F},
	{"CSIS1",	0x10,/*010000*/		"CSIC(STRP0)",		0x3F},
	{"CSIS1",	0x18,/*011000*/		"CSIC(STRP1)",		0x3F},
	{"CSIS1",	0x20,/*100000*/		"CSIC(STRP2)",		0x3F},
	{"CSIS1",	0x4, /*X00100*/		"CSIC(PDP_STAT0)",	0x1F},
	{"CSIS1",	0xC, /*X01100*/		"CSIC(PDP_STAT1)",	0x1F},
	{"CSIS1",	0x14,/*X10100*/		"CSIC(PDP_AF0)",	0x1F},
	{"CSIS1",	0x1C,/*X11100*/		"CSIC(PDP_AF1)",	0x1F},

	{"G3AA",	0x1, /*XXXX01*/		"SYSMMU_S2_G3AA",	0x3},
	{"G3AA",	0x2, /*XXXX10*/		"SYSMMU_S1_G3AA",	0x3},
	{"G3AA",	0x0, /*XXXX00*/		"G3AA",			0x3},

	{"IPP",		0x1, /*XXXX01*/		"SYSMMU_S2_IPP",	0x3},
	{"IPP",		0x2, /*XXXX10*/		"SYSMMU_S1_IPP",	0x3},
	{"IPP",		0x0, /*000000*/		"IPP(THSTAT)",		0x3F},
	{"IPP",		0x8, /*001000*/		"IPP(FDPIG)",		0x3F},
	{"IPP",		0x10,/*010000*/		"IPP(RBGH0)",		0x3F},
	{"IPP",		0x18,/*011000*/		"IPP(RBGH1)",		0x3F},
	{"IPP",		0x20,/*100000*/		"IPP(RBGH2)",		0x3F},
	{"IPP",		0x04,/*000100*/		"IPP(ALIGN0)",		0x3F},
	{"IPP",		0x0C,/*001100*/		"IPP(ALIGN1)",		0x3F},
	{"IPP",		0x14,/*010100*/		"IPP(ALIGN2)",		0x3F},
	{"IPP",		0x1C,/*011100*/		"IPP(ALIGN3)",		0x3F},
	{"IPP",		0x24,/*100100*/		"IPP(ALIGN_STAT)",	0x3F},
	{"IPP",		0x2C,/*101100*/		"TNR_ALIGN(TNR_MSA0)",	0x3F},

	{"DNS",		0x1, /*XXXX01*/		"SYSMMU_S2_DNS",	0x3},
	{"DNS",		0x2, /*XXXX10*/		"SYSMMU_S1_DNS",	0x3},
	{"DNS",		0x0, /*X00000*/		"DNS0",			0x1F},
	{"DNS",		0x10,/*X10000*/		"DNS1",			0x1F},
	{"DNS",		0x4, /*X00100*/		"VRA",			0x1F},
	{"DNS",		0x8, /*X01000*/		"TNR_ALIGN(TNR_MSA1)",	0x1F},
	{"DNS",		0xC, /*001100*/		"ITSC(M0/M2)",		0x3F},
	{"DNS",		0x2C,/*101100*/		"ITSC(M1)",		0x3F},
	{"DNS",		0x1C,/*X11100*/		"ITP",			0x1F},

	{"MCSC0",	0x1, /*XXXX01*/		"SYSMMU_S2_MCSC0",	0x3},
	{"MCSC0",	0x2, /*XXXX10*/		"SYSMMU_S1_MCSC0",	0x3},
	{"MCSC0",	0x0, /*XXXX00*/		"ITSC(M0)",		0x3},

	{"MCSC1",	0x1, /*XXXX01*/		"SYSMMU_S2_MCSC1",	0x3},
	{"MCSC1",	0x2, /*XXXX10*/		"SYSMMU_S1_MCSC1",	0x3},
	{"MCSC1",	0x0, /*XX0000*/		"MCSC(M0)",		0xF},
	{"MCSC1",	0x4, /*XX0100*/		"MCSC(M2)",		0xF},
	{"MCSC1",	0x8, /*XX1000*/		"MCSC(M3)",		0xF},

	{"MCSC2",	0x1, /*XXXX01*/		"SYSMMU_S2_MCSC2",	0x3},
	{"MCSC2",	0x2, /*XXXX10*/		"SYSMMU_S1_MCSC2",	0x3},
	{"MCSC2",	0x0, /*XX0000*/		"MCSC(M1)",		0xF},
	{"MCSC2",	0x4, /*XX0100*/		"MCSC(M4)",		0xF},
	{"MCSC2",	0x8, /*XX1000*/		"MCSC(M5)",		0xF},

	{"TNR0",	0x1, /*XXXX01*/		"SYSMMU_S2_TNR0",	0x3},
	{"TNR0",	0x2, /*XXXX10*/		"SYSMMU_S1_TNR0",	0x3},
	{"TNR0",	0x0, /*XX0000*/		"TNR(M0)",		0xF},
	{"TNR0",	0x4, /*XX0100*/		"TNR(M1)",		0xF},
	{"TNR0",	0x8, /*XX1000*/		"TNR(M8)",		0xF},

	{"TNR1",	0x1, /*XXXX01*/		"SYSMMU_S2_TNR1",	0x3},
	{"TNR1",	0x2, /*XXXX10*/		"SYSMMU_S1_TNR1",	0x3},
	{"TNR1",	0x0, /*XXXX00*/		"TNR(M2)",		0x3},

	{"TNR2",	0x1, /*XXXX01*/		"SYSMMU_S2_TNR2",	0x3},
	{"TNR2",	0x2, /*XXXX10*/		"SYSMMU_S1_TNR2",	0x3},
	{"TNR2",	0x0, /*XXXX00*/		"TNR(M3)",		0x3},

	{"TNR3",	0x1, /*XXXX01*/		"SYSMMU_S2_TNR3",	0x3},
	{"TNR3",	0x2, /*XXXX10*/		"SYSMMU_S1_TNR3",	0x3},
	{"TNR3",	0x0, /*XXXX00*/		"TNR(M4)",		0x3},

	{"TNR4",	0x1, /*XXXX01*/		"SYSMMU_S2_TNR4",	0x3},
	{"TNR4",	0x2, /*XXXX10*/		"SYSMMU_S1_TNR4",	0x3},
	{"TNR4",	0x0, /*XX0000*/		"TNR(M5)",		0xF},
	{"TNR4",	0x4, /*XX0100*/		"TNR(M6)",		0xF},
	{"TNR4",	0x8, /*XX1000*/		"TNR(M7)",		0xF},

	{"BO",		0x1, /*XXXX01*/		"SYSMMU_S2_BO",		0x3},
	{"BO",		0x2, /*XXXX10*/		"SYSMMU_S1_BO",		0x3},
	{"BO",		0x0, /*XXXX00*/		"BO",			0x3},

	{"MFC0",	0x1, /*XXXX01*/		"SYSMMU_S2_MFC0",	0x3},
	{"MFC0",	0x2, /*XXXX10*/		"SYSMMU_S1_MFC0",	0x3},
	{"MFC0",	0x0, /*XXXX00*/		"MFC0",			0x3},

	{"MFC1",	0x1, /*XXXX01*/		"SYSMMU_S2_MFC1",	0x3},
	{"MFC1",	0x2, /*XXXX10*/		"SYSMMU_S1_MFC1",	0x3},
	{"MFC1",	0x0, /*XXXX00*/		"MFC1",			0x3},

	{"G2D0",	0x1, /*XXXX01*/		"SYSMMU_S2_G2D0",	0x3},
	{"G2D0",	0x2, /*XXXX10*/		"SYSMMU_S1_G2D0",	0x3},
	{"G2D0",	0x0, /*XXXX00*/		"G2D0",			0x3},

	{"G2D1",	0x1, /*XXXX01*/		"SYSMMU_S2_G2D1",	0x3},
	{"G2D1",	0x2, /*XXXX10*/		"SYSMMU_S1_G2D1",	0x3},
	{"G2D1",	0x0, /*XXXX00*/		"G2D1",			0x3},

	{"G2D2",	0x1, /*XXXX01*/		"SYSMMU_S2_G2D2",	0x3},
	{"G2D2",	0x2, /*XXXX10*/		"SYSMMU_S1_G2D2",	0x3},
	{"G2D2",	0x0, /*XXXX00*/		"G2D2",			0x3},

	{"HSI2",	0x1, /*XXXX01*/		"SYSMMU_S2_HSI2",	0x3},
	{"HSI2",	0x2, /*XXXX10*/		"SYSMMU_S1_HSI2",	0x3},
	{"HSI2",	0x0, /*XX0000*/		"PCIE_GEN4A",		0xF},
	{"HSI2",	0x4, /*XX0100*/		"PCIE_GEN4B",		0xF},
	{"HSI2",	0x8, /*XX1000*/		"UFS_EMBD",		0xF},
	{"HSI2",	0xC, /*XX1100*/		"MMC_CARD",		0xF},

	{"MISC",	0x1, /*XXXXX1*/		"SYSMMU_S2_MISC",	0x1},
	{"MISC",	0x10,/*X10000*/		"SYSMMU_S2_SSS",	0x1F},
	{"MISC",	0x0, /*X00000*/		"SSS",			0x1F},
	{"MISC",	0x2, /*XX0010*/		"RTIC",			0xF},
	{"MISC",	0x4, /*XX0100*/		"SPDMA0",		0xF},
	{"MISC",	0x6, /*XX0110*/		"PDMA0",			0xF},
	{"MISC",	0x8, /*XX1000*/		"DIT",			0xF},
	{"MISC",	0xA, /*XX1010*/		"SPDMA1",		0xF},
	{"MISC",	0xC, /*XX1100*/		"PDMA1",		0xF},

	{"GDC0",	0x1, /*XXXX01*/		"SYSMMU_S2_GDC0",	0x3},
	{"GDC0",	0x2, /*XXXX10*/		"SYSMMU_S1_GDC0",	0x3},
	{"GDC0",	0x0, /*XXX000*/		"GDC0(M0)",			0x7},
	{"GDC0",	0x4, /*XXX100*/		"GDC0(M1)",			0x7},

	{"GDC1",	0x1, /*XXXX01*/		"SYSMMU_S2_GDC1",	0x3},
	{"GDC1",	0x2, /*XXXX10*/		"SYSMMU_S1_GDC1",	0x3},
	{"GDC1",	0x0, /*XX0000*/		"GDC1(M0)",			0xF},
	{"GDC1",	0x4, /*XX0100*/		"GDC1(M1)",			0xF},
	{"GDC1",	0x8, /*XX1000*/		"SCSC(M2)",			0xF},

	{"GDC2",	0x1, /*XXXX01*/		"SYSMMU_S2_GDC2",	0x3},
	{"GDC2",	0x2, /*XXXX10*/		"SYSMMU_S1_GDC2",	0x3},
	{"GDC2",	0x0, /*XXX000*/		"SCSC(M0)",			0x7},
	{"GDC2",	0x4, /*XXX100*/		"SCSC(M1)",			0x7},

	{"GPU0",	0x1, /*XXXXX1*/		"SYSMMU_S2_GPU0",	0x1},
	{"GPU0",	0x0, /*XXXXX0*/		"GPU(M0)",		0x1},

	{"GPU1",	0x1, /*XXXXX1*/		"SYSMMU_S2_GPU1",	0x1},
	{"GPU1",	0x0, /*XXXXX0*/		"GPU(M1)",		0x1},

	{"GPU2",	0x1, /*XXXXX1*/		"SYSMMU_S2_GPU2",	0x1},
	{"GPU2",	0x0, /*XXXXX0*/		"GPU(M2)",		0x1},

	{"GPU3",	0x1, /*XXXXX1*/		"SYSMMU_S2_GPU3",	0x1},
	{"GPU3",	0x0, /*XXXXX0*/		"GPU(M3)",		0x1},

	{"TPU",		0x1, /*XXXX01*/		"SYSMMU_S2_TPU",	0x3},
	{"TPU",		0x2, /*XXXX10*/		"SYSMMU_S1_TPU",	0x3},
	{"TPU",		0x0, /*XXXX00*/		"TPU",			0x3},

	{"AUR0",	0x1, /*XXXX01*/		"SYSMMU_D0_S2_AUR1",	0x3},
	{"AUR0",	0x2, /*XXXX10*/		"SYSMMU_D0_S1_AUR1",	0x3},
	{"AUR0",	0x0, /*XXXX00*/		"AUR_M0",		0x3},

	{"AUR1",	0x1, /*XXXX01*/		"SYSMMU_D1_S2_AUR1",	0x3},
	{"AUR1",	0x2, /*XXXX10*/		"SYSMMU_D1_S1_AUR1",	0x3},
	{"AUR1",	0x0, /*XXXX00*/		"AUR_M1",		0x3},

	/* Cannot differentiate which cpu */
	{"CPU0",	0x40, /*bit 25*/	"CPU",			0x40},
	{"CPU1",	0x40, /*bit 25*/	"CPU",			0x40},
	{"CPU2",	0x40, /*bit 25*/	"CPU",			0x40},
	{"CPU3",	0x40, /*bit 25*/	"CPU",			0x40},
	/* Cannot differentiate others */
	{"CPU0",	0x0, /*bit 25*/		"NOT CPU",		0x40},
	{"CPU1",	0x0, /*bit 25*/		"NOT CPU",		0x40},
	{"CPU2",	0x0, /*bit 25*/		"NOT CPU",		0x40},
	{"CPU3",	0x0, /*bit 25*/		"NOT CPU",		0x40},

};

static struct itmon_nodeinfo vec_d0[] = {
	{M_NODE, "ALIVE",	0x1EA03000, 1, 1, 0, 0},
	{M_NODE, "AOC",		0x1EA13000, 1, 1, 0, 0},
	{M_NODE, "CSSYS",	0x1EA23000, 1, 1, 0, 0},
	{M_NODE, "GSA",		0x1EA33000, 1, 1, 0, 0},
	{M_NODE, "HSI0",	0x1EA43000, 1, 1, 0, 0},
	{M_NODE, "HSI1",	0x1EA53000, 1, 1, 0, 0},
	{T_S_NODE, "BUS0_M0",	0x1EA63000, 1, 1, 0, 0},
};

static struct itmon_nodeinfo vec_d1[] = {
	{M_NODE, "BO",		0x1F403000, 1, 1, 0, 0},
	{M_NODE, "CSIS0",	0x1F413000, 1, 1, 0, 0},
	{M_NODE, "CSIS1",	0x1F423000, 1, 1, 0, 0},
	{M_NODE, "DNS",		0x1F433000, 1, 1, 0, 0},
	{M_NODE, "DPU0",	0x1F443000, 1, 1, 0, 0},
	{M_NODE, "DPU1",	0x1F453000, 1, 1, 0, 0},
	{M_NODE, "DPU2",	0x1F463000, 1, 1, 0, 0},
	{M_NODE, "G2D0",	0x1F4A3000, 1, 1, 0, 0},
	{M_NODE, "G2D1",	0x1F4B3000, 1, 1, 0, 0},
	{M_NODE, "G2D2",	0x1F4C3000, 1, 1, 0, 0},
	{M_NODE, "G3AA",	0x1F4D3000, 1, 1, 0, 0},
	{M_NODE, "GDC0",	0x1F473000, 1, 1, 0, 0},
	{M_NODE, "GDC1",	0x1F483000, 1, 1, 0, 0},
	{M_NODE, "GDC2",	0x1F493000, 1, 1, 0, 0},
	{M_NODE, "HSI2",	0x1F4E3000, 1, 1, 0, 0},
	{M_NODE, "IPP",		0x1F4F3000, 1, 1, 0, 0},
	{M_NODE, "MCSC0",	0x1F503000, 1, 1, 0, 0},
	{M_NODE, "MCSC1",	0x1F513000, 1, 1, 0, 0},
	{M_NODE, "MCSC2",	0x1F523000, 1, 1, 0, 0},
	{M_NODE, "MFC0",	0x1F533000, 1, 1, 0, 0},
	{M_NODE, "MFC1",	0x1F543000, 1, 1, 0, 0},
	{M_NODE, "MISC",	0x1F553000, 1, 1, 0, 0},
	{M_NODE, "TNR0",	0x1F563000, 1, 1, 0, 0},
	{M_NODE, "TNR1",	0x1F573000, 1, 1, 0, 0},
	{M_NODE, "TNR2",	0x1F583000, 1, 1, 0, 0},
	{M_NODE, "TNR3",	0x1F593000, 1, 1, 0, 0},
	{M_NODE, "TNR4",	0x1F5A3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL2A_M0",	0x1F5B3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL2A_M1",	0x1F5C3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL2A_M2",	0x1F5D3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL2A_M3",	0x1F5E3000, 1, 1, 0, 0},
};

static struct itmon_nodeinfo vec_d2[] = {
	{M_NODE, "AUR0",	0x20403000, 1, 1, 0, 0},
	{M_NODE, "AUR1",	0x20413000, 1, 1, 0, 0},
	{M_NODE, "GPU0",	0x20433000, 1, 1, 0, 0},
	{M_NODE, "GPU1",	0x20443000, 1, 1, 0, 0},
	{M_NODE, "GPU2",	0x20453000, 1, 1, 0, 0},
	{M_NODE, "GPU3",	0x20463000, 1, 1, 0, 0},
	{M_NODE, "GPUMMU",	0x20423000, 1, 1, 0, 0},
	{T_M_NODE, "NOCL1A_S0",	0x20483000, 1, 1, 0, 0},
	{T_M_NODE, "NOCL1A_S1",	0x20493000, 1, 1, 0, 0},
	{T_M_NODE, "NOCL1A_S2",	0x204A3000, 1, 1, 0, 0},
	{T_M_NODE, "NOCL1A_S3",	0x204B3000, 1, 1, 0, 0},
	{M_NODE, "TPU",		0x20473000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL1A_M0",	0x204C3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL1A_M1",	0x204D3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL1A_M2",	0x204E3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL1A_M3",	0x204F3000, 1, 1, 0, 0},
};

static struct itmon_nodeinfo vec_d3[] = {
	{M_NODE, "CPU0",	0x1E403000, 1, 1, 0, 0},
	{M_NODE, "CPU1",	0x1E413000, 1, 1, 0, 0},
	{M_NODE, "CPU2",	0x1E423000, 1, 1, 0, 0},
	{M_NODE, "CPU3",	0x1E433000, 1, 1, 0, 0},
	{T_M_NODE, "NOCL0_S0",	0x1E443000, 1, 1, 0, 0},
	{T_M_NODE, "NOCL0_S1",	0x1E453000, 1, 1, 0, 0},
	{T_M_NODE, "NOCL0_S2",	0x1E463000, 1, 1, 0, 0},
	{T_M_NODE, "NOCL0_S3",	0x1E473000, 1, 1, 0, 0},
	{T_M_NODE, "NOCL0_S4",	0x1E483000, 1, 1, 0, 0},
	{S_NODE, "NOCL0_CCI",	0x1E493000, 1, 1, 0, 0},
	{S_NODE, "NOCL0_DP",	0x1E4A3000, 1, 1, 0, 0, TMOUT, 1},
	{T_S_NODE, "NOCL0_M0",	0x1E4B3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL0_M1",	0x1E4C3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL0_M2",	0x1E4D3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL0_M3",	0x1E4E3000, 1, 1, 0, 0},
};

static struct itmon_nodeinfo vec_p0[] = {
	{M_NODE, "CCI",		0x1E603000, 1, 1, 0, 0},
	{M_NODE, "CSSYS",	0x1E613000, 1, 1, 0, 0},
	{M_NODE, "NOCL0_DP",	0x1E623000, 1, 1, 0, 0},
	{S_NODE, "ALIVE",	0x1E633000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "CPUCL0",	0x1E643000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "EH",		0x1E653000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "GIC",		0x1E663000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "MIF0",	0x1E673000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "MIF1",	0x1E683000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "MIF2",	0x1E693000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "MIF3",	0x1E6A3000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "MISC",	0x1E6B3000, 1, 1, 0, 0, TMOUT, 1},
	{T_S_NODE, "NOCL0_M0",	0x1E6E3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL1_M1",	0x1E6F3000, 1, 1, 0, 0},
	{T_S_NODE, "NOCL2_M2",	0x1E703000, 1, 1, 0, 0},
	{S_NODE, "P0_NOCL0",	0x1E713000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "P1_NOCL0",	0x1E723000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "P2_NOCL0",	0x1E733000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "PERIC0",	0x1E6C3000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "PERIC1",	0x1E6D3000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "SLC",		0x1E743000, 1, 1, 0, 0, TMOUT, 1},
};

static struct itmon_nodeinfo vec_p1[] = {
	{T_M_NODE, "NOCL1B_S0",	0x1EC03000, 1, 1, 0, 0},
	{S_NODE, "AOC",		0x1EC13000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "GSA",		0x1EC23000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "HSI0",	0x1EC33000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "HSI1",	0x1EC43000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "P0_NOCL1B",	0x1EC53000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "P1_NOCL1B", 	0x1EC63000, 1, 1, 0, 0, TMOUT, 1},
};

static struct itmon_nodeinfo vec_p2[] = {
	{T_M_NODE, "NOCL2A_S0",	0x1F203000, 1, 1, 0, 0},
	{S_NODE, "BO",		0x1F213000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "CSIS",	0x1F223000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "DISP",	0x1F233000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "DNS",		0x1F243000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "DPU",		0x1F253000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "G2D",		0x1F273000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "G3AA",	0x1F283000, 1, 1, 0, 0, TMOUT, 0},
	{S_NODE, "GDC",		0x1F263000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "HSI2",	0x1F293000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "IPP",		0x1F2A3000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "ITP",		0x1F2B3000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "MCSC",	0x1F2C3000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "MFC",		0x1F2D3000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "P0_NOCL2A",	0x1F303000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "P1_NOCL2A",	0x1F313000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "PDP",		0x1F2E3000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "TNR",		0x1F2F3000, 1, 1, 0, 0, TMOUT, 1},
};

static struct itmon_nodeinfo vec_p3[] = {
	{T_M_NODE, "NOCL1A_S0",	0x20603000, 1, 1, 0},
	{S_NODE, "AUR", 	0x20613000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "G3D",		0x20623000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "P0_NOCL1A",	0x20643000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "P1_NOCL1A",	0x20653000, 1, 1, 0, 0, TMOUT, 1},
	{S_NODE, "TPU",		0x20633000, 1, 1, 0, 0, TMOUT, 1},
};

static struct itmon_nodegroup nodegroup[] = {
	{"BUS_DATA0", 0x1EA83000, 0, vec_d0, ARRAY_SIZE(vec_d0), DATA}, /* 637 */
	{"BUS_DATA1", 0x1F603000, 0, vec_d1, ARRAY_SIZE(vec_d1), DATA}, /* 640 */
	{"BUS_DATA2", 0x20513000, 0, vec_d2, ARRAY_SIZE(vec_d2), DATA}, /* 625 */
	{"BUS_DATA3", 0x1E503000, 0, vec_d3, ARRAY_SIZE(vec_d3), DATA}, /* 596 */
	{"BUS_PERI0", 0x1E773000, 0, vec_p0, ARRAY_SIZE(vec_p0), PERI}, /* 607 */
	{"BUS_PERI1", 0x1EC83000, 0, vec_p1, ARRAY_SIZE(vec_p1), PERI}, /* 638 */
	{"BUS_PERI2", 0x1F333000, 0, vec_p2, ARRAY_SIZE(vec_p2), PERI}, /* 641 */
	{"BUS_PERI3", 0x20673000, 0, vec_p3, ARRAY_SIZE(vec_p3), PERI}, /* 626 */
};

const static char *itmon_pathtype[] = {
	"DATA Path transaction",
	"Configuration(SFR) Path transaction",
};

/* Error Code Description */
const static char *itmon_errcode[] = {
	"Error Detect by the Slave(SLVERR)",
	"Decode error(DECERR)",
	"Unsupported transaction error",
	"Power Down access error",
	"Unsupported transaction",
	"Unsupported transaction",
	"Timeout error - response timeout in timeout value",
	"Invalid errorcode",
};

const static char *itmon_node_string[] = {
	"M_NODE",
	"TAXI_S_NODE",
	"TAXI_M_NODE",
	"S_NODE",
};

const static char *itmon_cpu_node_string[] = {
	"M_CPU",
	"SCI_IRPM",
	"SCI_CCM",
	"CCI",
};

const static char *itmon_drex_node_string[] = {
	"DREX_IRPS",
	"CORE_CCI",
	"CORE_M",
};

const static unsigned int itmon_invalid_addr[] = {
	0x03000000,
	0x04000000,
};

static struct itmon_dev *g_itmon;

/* declare notifier_list */
ATOMIC_NOTIFIER_HEAD(itmon_notifier_list);

static void itmon_pattern_reset(struct itmon_dev *itmon)
{
	atomic_set(&itmon->itmon_pattern_cnt, 0);
}

static void itmon_pattern_set(struct itmon_dev *itmon, const char *fmt, ...)
{
	va_list args;

	/* only take the first pattern even there could be multiple paths */
	if (atomic_inc_return(&itmon->itmon_pattern_cnt) > 1)
		return;

	va_start(args, fmt);
	vsnprintf(itmon->itmon_pattern, sizeof(itmon->itmon_pattern), fmt, args);
	va_end(args);
}

static const struct itmon_rpathinfo *itmon_get_rpathinfo(struct itmon_dev *itmon,
							unsigned int id,
							char *dest_name)
{
	struct itmon_platdata *pdata = itmon->pdata;
	const struct itmon_rpathinfo *rpath = NULL;
	int i;

	if (!dest_name)
		return NULL;

	for (i = 0; i < (int)ARRAY_SIZE(rpathinfo); i++) {
		if (pdata->rpathinfo[i].id == (id & pdata->rpathinfo[i].bits)) {
			if (dest_name && !strncmp(pdata->rpathinfo[i].dest_name, dest_name,
						  strlen(pdata->rpathinfo[i].dest_name))) {
				rpath = &pdata->rpathinfo[i];
				break;
			}
		}
	}
	return rpath;
}

static const struct itmon_clientinfo *itmon_get_clientinfo(struct itmon_dev *itmon,
							  char *port_name,
							  unsigned int user)
{
	struct itmon_platdata *pdata = itmon->pdata;
	const struct itmon_clientinfo *client = NULL;
	unsigned int val;
	int i;

	if (!port_name)
		return NULL;

	/* shift directly to SID bits */
	user >>= 19;

	for (i = 0; i < (int)ARRAY_SIZE(clientinfo); i++) {
		if (!strcmp(pdata->clientinfo[i].port_name, port_name)) {
			val = user & pdata->clientinfo[i].bits;
			if (val == pdata->clientinfo[i].user) {
				client = &pdata->clientinfo[i];
				break;
			}
		}
	}
	return client;
}

static void itmon_enable_addr_detect(struct itmon_dev *itmon,
				     struct itmon_nodeinfo *node, bool enabled)
{
	/* This feature is only for M_NODE */
	unsigned int tmp, val;
	unsigned int offset = OFFSET_PROT_CHK;

	val = __raw_readl(node->regs + offset + REG_PROT_CHK_CTL);
	val |= INTEND_ACCESS_INT_ENABLE;
	__raw_writel(val, node->regs + offset + REG_PROT_CHK_CTL);

	val = ((unsigned int)INTEND_ADDR_START & U32_MAX);
	__raw_writel(val, node->regs + offset + REG_PROT_CHK_START_ADDR_LOW);

	val = (unsigned int)(((unsigned long)INTEND_ADDR_START >> 32) & U16_MAX);
	__raw_writel(val, node->regs + offset
				+ REG_PROT_CHK_START_END_ADDR_UPPER);

	val = ((unsigned int)INTEND_ADDR_END & 0xFFFFFFFF);
	__raw_writel(val, node->regs + offset
			+ REG_PROT_CHK_END_ADDR_LOW);
	val = ((unsigned int)(((unsigned long)INTEND_ADDR_END >> 32)
							& 0XFFFF0000) << 16);
	tmp = readl(node->regs + offset + REG_PROT_CHK_START_END_ADDR_UPPER);
	__raw_writel(tmp | val, node->regs + offset
				+ REG_PROT_CHK_START_END_ADDR_UPPER);

	dev_dbg(itmon->dev, "ITMON - %s addr detect  enabled\n", node->name);
}

static void itmon_enable_prot_chk(struct itmon_dev *itmon,
			       struct itmon_nodeinfo *node,
			       bool enabled)
{
	unsigned int offset = OFFSET_PROT_CHK;
	unsigned int val = 0;

	if (enabled)
		val = RD_RESP_INT_ENABLE | WR_RESP_INT_ENABLE |
		     ARLEN_RLAST_INT_ENABLE | AWLEN_WLAST_INT_ENABLE;

	__raw_writel(val, node->regs + offset + REG_PROT_CHK_CTL);

	dev_dbg(itmon->dev, "ITMON - %s hw_assert enabled\n", node->name);
}

static void itmon_enable_err_report(struct itmon_dev *itmon,
			       struct itmon_nodeinfo *node,
			       bool enabled)
{
	struct itmon_platdata *pdata = itmon->pdata;
	unsigned int offset = OFFSET_REQ_R;

	if (!pdata->probed || !node->retention)
		__raw_writel(1, node->regs + offset + REG_INT_CLR);

	/* enable interrupt */
	__raw_writel(enabled, node->regs + offset + REG_INT_MASK);

	/* clear previous interrupt of req_write */
	offset = OFFSET_REQ_W;
	if (pdata->probed || !node->retention)
		__raw_writel(1, node->regs + offset + REG_INT_CLR);
	/* enable interrupt */
	__raw_writel(enabled, node->regs + offset + REG_INT_MASK);

	/* clear previous interrupt of response_read */
	offset = OFFSET_RESP_R;
	if (!pdata->probed || !node->retention)
		__raw_writel(1, node->regs + offset + REG_INT_CLR);
	/* enable interrupt */
	__raw_writel(enabled, node->regs + offset + REG_INT_MASK);

	/* clear previous interrupt of response_write */
	offset = OFFSET_RESP_W;
	if (!pdata->probed || !node->retention)
		__raw_writel(1, node->regs + offset + REG_INT_CLR);
	/* enable interrupt */
	__raw_writel(enabled, node->regs + offset + REG_INT_MASK);

	dev_dbg(itmon->dev,
		"ITMON - %s error reporting enabled\n", node->name);
}

static void itmon_enable_timeout(struct itmon_dev *itmon,
			       struct itmon_nodeinfo *node,
			       bool enabled)
{
	unsigned int offset = OFFSET_TMOUT_REG;

	/* Enable Timeout setting */
	__raw_writel(enabled, node->regs + offset + REG_DBG_CTL);

	/* set tmout interval value */
	__raw_writel(node->time_val,
		     node->regs + offset + REG_TMOUT_INIT_VAL);

	if (node->tmout_frz_enabled) {
		/* Enable freezing */
		__raw_writel(enabled,
			     node->regs + offset + REG_TMOUT_FRZ_EN);
	}
	dev_dbg(itmon->dev, "ITMON - %s timeout enabled\n", node->name);
}

static void itmon_init(struct itmon_dev *itmon, bool enabled)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *node;
	int i, j;

	for (i = 0; i < (int)ARRAY_SIZE(nodegroup); i++) {
		node = pdata->nodegroup[i].nodeinfo;
		for (j = 0; j < pdata->nodegroup[i].nodesize; j++) {
			if (node[j].type == S_NODE && node[j].tmout_enabled)
				itmon_enable_timeout(itmon, &node[j], true);

			if (node[j].err_enabled)
				itmon_enable_err_report(itmon, &node[j], true);

			if (node[j].prot_chk_enabled)
				itmon_enable_prot_chk(itmon, &node[j], true);

			if (node[j].addr_detect_enabled)
				itmon_enable_addr_detect(itmon, &node[j], true);

			dev_dbg(itmon->dev, "ITMON - %s init -\n", node[j].name);
		}
	}
}

void itmon_enable(bool enabled)
{
	if (g_itmon)
		itmon_init(g_itmon, enabled);
}

static void itmon_post_handler_apply_policy(struct itmon_dev *itmon,
					    int ret_value)
{
	struct itmon_platdata *pdata = itmon->pdata;

	switch (ret_value) {
	case NOTIFY_STOP:
		log_dev_err(itmon->dev, "notify calls response NOTIFY_STOP, refer to notifier log\n");
		pdata->policy[IP].error = true;
		break;
	case NOTIFY_BAD:
		log_dev_err(itmon->dev, "notify calls response NOTIFY_BAD, refer to notifier log\n");
		pdata->policy[FATAL].error = true;
		break;
	case NOTIFY_OK:
	case NOTIFY_DONE:
	default:
		log_dev_err(itmon->dev, "notify calls response NOTIFY_OK/DONE\n");
		pdata->policy[UNHANDLED].error = true;
		break;
	}
}

static void itmon_post_handler_to_notifier(struct itmon_dev *itmon,
					   struct itmon_traceinfo *info,
					   unsigned int trans_type)
{
	int ret = 0;

	/* After treatment by port */
	if (!info->port || strlen(info->port) < 1)
		return;

	itmon->notifier_info.port = info->port;
	itmon->notifier_info.client = info->client;
	itmon->notifier_info.dest = info->dest;
	itmon->notifier_info.read = info->read;
	itmon->notifier_info.target_addr = info->target_addr;
	itmon->notifier_info.errcode = info->errcode;
	itmon->notifier_info.onoff = info->onoff;

	log_dev_err(itmon->dev, "     +ITMON Notifier Call Information\n\n");

	/* call notifier_call_chain of itmon */
	ret = atomic_notifier_call_chain(&itmon_notifier_list,
						0, &itmon->notifier_info);
	itmon_post_handler_apply_policy(itmon, ret);

	log_dev_err(itmon->dev, "     -ITMON Notifier Call Information\n"
		    "-----------------------------------------------------------\n");
}

static void itmon_post_handler_by_dest(struct itmon_dev *itmon,
				       struct itmon_traceinfo *info,
				       unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;

	if (!info->dest || strlen(info->dest) < 1)
		return;

	if (info->errcode == ERRCODE_TMOUT) {
		int i;

		for (i = 0; i < (int)ARRAY_SIZE(itmon_drex_node_string); i++) {
			if (!strcmp(info->dest, itmon_drex_node_string[i])) {
				pdata->policy[DREX_TMOUT].error = true;
				break;
			}

		}
	}
}

static void itmon_post_handler_by_client(struct itmon_dev *itmon,
					 struct itmon_traceinfo *info,
					 unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;

	/* After treatment by port */
	if (!info->port || strlen(info->port) < 1)
		return;

	if (test_bit(FROM_CPU, &info->from)) {
		ktime_t now, interval;

		now = ktime_get();
		interval = ktime_sub(now, pdata->last_time);
		pdata->last_time = now;
		pdata->err_cnt_by_cpu++;
		if (pdata->err_cnt_by_cpu > pdata->panic_threshold)
			pdata->policy[CPU].error = true;

		if (info->errcode == ERRCODE_TMOUT) {
			pdata->policy[FATAL].error = true;
			log_dev_err(itmon->dev,
				    "Try to handle error, even CPU transaction detected - %s\n",
				itmon_errcode[info->errcode]);
		} else {
			log_dev_err(itmon->dev, "Skips CPU transaction detected - err_cnt_by_cpu: %u, interval: %lldns\n",
				    pdata->err_cnt_by_cpu, ktime_to_ns(interval));

			/* Ignore unhandled cpu errors */
			pdata->policy[UNHANDLED].error = false;
		}
	} else {
		if (info->errcode == ERRCODE_UNSUPPORTED)
			pdata->policy[FATAL].error = true;
	}
}

static void itmon_report_timeout(struct itmon_dev *itmon,
				struct itmon_nodeinfo *node,
				unsigned int trans_type)
{
	unsigned int id, payload0, payload1 = 0, payload2, payload3, payload4;
	unsigned int axid, user, valid, timeout, info;
	unsigned long addr;
	char *client_name, *port_name;
	const struct itmon_rpathinfo *port;
	const struct itmon_clientinfo *client;
	struct itmon_nodegroup *group = NULL;
	int i, num = (trans_type == TRANS_TYPE_READ ? SZ_128 : SZ_64);
	int rw_offset = (trans_type == TRANS_TYPE_READ ? 0 : REG_TMOUT_BUF_WR_OFFSET);
	int path_offset = 0;

	if (!node)
		return;

	group = node->group;
	if (group->path_type == DATA)
		path_offset = SZ_4;

	log_dev_err(itmon->dev,
		    "\n-----------------------------------------------------------\n"
		    "      ITMON Report (%s)\n"
		    "-----------------------------------------------------------\n"
		    "      Timeout Error Occurred : Client --> %s\n\n",
		    trans_type == TRANS_TYPE_READ ? "READ" : "WRITE", node->name);
	log_dev_err(itmon->dev,
		    "      TIMEOUT_BUFFER Information(NODE: %s)\n"
		    "	> NUM|   BLOCK|  CLIENT|VALID|TIMEOUT|      ID| PAYLOAD0|         ADDRESS| PAYLOAD4|\n",
		    node->name);

	for (i = 0; i < num; i++) {
		writel(i, node->regs + OFFSET_TMOUT_REG + REG_TMOUT_BUF_POINT_ADDR + rw_offset);
		id = readl(node->regs + OFFSET_TMOUT_REG + REG_TMOUT_BUF_ID + rw_offset);
		payload0 = readl(node->regs + OFFSET_TMOUT_REG + REG_TMOUT_BUF_PAYLOAD_0);
		if (path_offset == SZ_4)
			payload1 = readl(node->regs + OFFSET_TMOUT_REG + REG_TMOUT_BUF_PAYLOAD_1 + rw_offset);
		payload2 = readl(node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_PAYLOAD_2 + rw_offset);
		payload3 = readl(node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_PAYLOAD_3 + rw_offset);
		payload4 = readl(node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_PAYLOAD_4 + rw_offset);

		if (path_offset == SZ_4) {
			timeout = (payload0 & (unsigned int)(GENMASK(7, 4))) >> 4;
			user = payload1;
		} else {
			timeout = (payload0 & (unsigned int)(GENMASK(19, 16))) >> 16;
			user = (payload0 & (unsigned int)(GENMASK(15, 8))) >> 8;
		}

		addr = (((unsigned long)payload2 & GENMASK(15, 0)) << 32ULL);
		addr |= payload3;

		info = readl(node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_PAYLOAD_3 + rw_offset + path_offset);

		/* ID[5:0] 6bit : R-PATH */
		axid = id & GENMASK(5, 0);
		/* PAYLOAD[0] : Valid or Not valid */
		valid = payload0 & BIT(0);

		port = itmon_get_rpathinfo(itmon, axid, node->name);

		port_name = NOT_AVAILABLE_STR;
		client_name = NOT_AVAILABLE_STR;
		if (port) {
			port_name = port->port_name;
			if (user) {
				client = itmon_get_clientinfo(itmon, port_name, user);
				if (client)
					client_name = client->client_name;
			}
		}

		log_dev_err(itmon->dev,
			    "      > %03d|%8s|%8s|%5u|%7x|%08x|%08x|%pa[p]|%08x\n",
			    i, port_name, client_name, valid, timeout,
			    id, payload0, &addr, payload4);
	}
	log_dev_err(itmon->dev,
		    "-----------------------------------------------------------\n");
}

static unsigned int power(unsigned int param, unsigned int num)
{
	if (num == 0)
		return 1;
	return param * (power(param, num - 1));
}

static void itmon_report_prot_chk_rawdata(struct itmon_dev *itmon,
				     struct itmon_nodeinfo *node)
{
	unsigned int dbg_mo_cnt, prot_chk_ctl, prot_chk_info, prot_chk_int_id;

	dbg_mo_cnt = __raw_readl(node->regs + OFFSET_PROT_CHK);
	prot_chk_ctl = __raw_readl(node->regs +
				OFFSET_PROT_CHK + REG_PROT_CHK_CTL);
	prot_chk_info = __raw_readl(node->regs +
				OFFSET_PROT_CHK + REG_PROT_CHK_INT);
	prot_chk_int_id = __raw_readl(node->regs +
				OFFSET_PROT_CHK + REG_PROT_CHK_INT_ID);

	/* Output Raw register information */
	log_dev_err(itmon->dev,
		    "\n-----------------------------------------------------------\n"
		    "      Protocol Checker Raw Register Information (ITMON information)\n\n");
	log_dev_err(itmon->dev,
		    "      > %s(%s, 0x%08X)\n"
		    "      > REG(0x100~0x10C)      : 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
		    node->name, itmon_node_string[node->type],
		    node->phy_regs,
		    dbg_mo_cnt,
		    prot_chk_ctl,
		    prot_chk_info,
		    prot_chk_int_id);
	itmon_pattern_set(itmon, "from %s", node->name);
}

static void itmon_report_rawdata(struct itmon_dev *itmon,
				 struct itmon_nodeinfo *node,
				 unsigned int trans_type)
{
	struct itmon_tracedata *data = &node->tracedata;

	/* Output Raw register information */
	log_dev_err(itmon->dev,
		    "Raw Register Information ----------------------------------\n"
		    "      > %s(%s, 0x%08X)\n"
		    "      > REG(0x08~0x18)        : 0x%08X, 0x%08X, 0x%08X, 0x%08X\n"
		    "      > REG(0x80)             : 0x%08X\n"
		    "      > REG(0x100~0x10C)      : 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
		    node->name, itmon_node_string[node->type],
		    node->phy_regs + data->offset,
		    data->int_info,
		    data->ext_info_0,
		    data->ext_info_1,
		    data->ext_info_2,
		    data->ext_user,
		    data->dbg_mo_cnt,
		    data->prot_chk_ctl,
		    data->prot_chk_info,
		    data->prot_chk_int_id);
}

static void itmon_report_traceinfo(struct itmon_dev *itmon,
				   struct itmon_traceinfo *info,
				   unsigned int trans_type)
{
	if (!info->dirty)
		return;

	log_dev_err(itmon->dev,
		    "\n-----------------------------------------------------------\n"
		    "      Transaction Information\n\n"
		    "      > Client (User)  : %s %s (0x%X)\n"
		    "      > Target         : %s\n"
		    "      > Target Address : 0x%lX %s\n"
		    "      > Type           : %s\n"
		    "      > Error code     : %s\n\n",
		    info->port, info->client ? info->client : "", info->user,
		    info->dest ? info->dest : NOT_AVAILABLE_STR,
		    info->target_addr,
		    info->baaw_prot ? "(BAAW Remapped address)" : "",
		    trans_type == TRANS_TYPE_READ ? "READ" : "WRITE",
		    itmon_errcode[info->errcode]);

	itmon_pattern_set(itmon, "from %s %s to %s",
			  info->port, info->client ? info->client : "",
			  info->dest ? info->dest : NOT_AVAILABLE_STR);

	log_dev_err(itmon->dev,
		    "\n------------------------------------------------------------\n"
		    "      > Size           : %u bytes x %u burst => %u bytes\n"
		    "      > Burst Type     : %u (0:FIXED, 1:INCR, 2:WRAP)\n"
		    "      > Level          : %s\n"
		    "      > Protection     : %s\n"
		    "      > Path Type      : %s\n\n",
		    power(2, info->axsize), info->axlen + 1,
		    power(2, info->axsize) * (info->axlen + 1),
		    info->axburst,
		    info->axprot & BIT(0) ? "Privileged" : "Unprivileged",
		    info->axprot & BIT(1) ? "Non-secure" : "Secure",
		    itmon_pathtype[info->path_type]);
}

static void itmon_report_pathinfo(struct itmon_dev *itmon,
				  struct itmon_nodeinfo *node,
				  struct itmon_traceinfo *info,
				  unsigned int trans_type)

{
	struct itmon_tracedata *data = &node->tracedata;

	if (!info->path_dirty) {
		log_dev_err(itmon->dev,
			    "\n-----------------------------------------------------------\n"
			    "      ITMON Report (%s)\n"
			    "-----------------------------------------------------------\n"
			    "      PATH Information\n\n",
			    trans_type == TRANS_TYPE_READ ? "READ" : "WRITE");
		info->path_dirty = true;
	}
	switch (node->type) {
	case M_NODE:
		log_dev_info(itmon->dev,
			     "      > %14s, %8s(0x%08X)\n",
			     node->name, "M_NODE",
			     node->phy_regs + data->offset);
		break;
	case T_S_NODE:
		log_dev_info(itmon->dev,
			     "      > %14s, %8s(0x%08X)\n",
			     node->name, "T_S_NODE",
			     node->phy_regs + data->offset);
		break;
	case T_M_NODE:
		log_dev_info(itmon->dev,
			     "      > %14s, %8s(0x%08X)\n",
			     node->name, "T_M_NODE",
			     node->phy_regs + data->offset);
		break;
	case S_NODE:
		log_dev_info(itmon->dev,
			     "      > %14s, %8s(0x%08X)\n",
			     node->name, "S_NODE",
			     node->phy_regs + data->offset);
		break;
	}
}

static int itmon_parse_cpuinfo(struct itmon_dev *itmon,
			       struct itmon_nodeinfo *node,
			       struct itmon_traceinfo *info,
			       unsigned int userbit)
{
	struct itmon_tracedata *find_data = NULL;
	int cluster_num, core_num, i;
	int ret = -1;

	for (i = 0; i < (int)ARRAY_SIZE(itmon_cpu_node_string); i++) {
		if (!strcmp(node->name, itmon_cpu_node_string[i])) {
			core_num = ((userbit & GENMASK(4, 2)) >> 2);
			cluster_num = 0;
			scnprintf(info->buf, SZ_32 - 1, "CPU%d Cluster%d",
					core_num, cluster_num);
			find_data = &node->tracedata;
			find_data->ref_info = info;
			info->port = info->buf;
			set_bit(FROM_CPU, &info->from);
			ret = 0;
			break;
		}
	};

	return ret;
}

static void itmon_parse_traceinfo(struct itmon_dev *itmon,
				   struct itmon_nodeinfo *node,
				   unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_tracedata *data = &node->tracedata;
	struct itmon_traceinfo *new_info = NULL;
	const struct itmon_clientinfo *client = NULL;
	const struct itmon_rpathinfo *port = NULL;
	struct itmon_nodeinfo *find_node = NULL;
	struct itmon_tracedata *find_data = NULL;
	struct itmon_nodegroup *group = node->group;
	unsigned int errcode, axid;
	unsigned int userbit;
	int i;

	errcode = BIT_ERR_CODE(data->int_info);
	if (!BIT_ERR_VALID(data->int_info))
		return;

	if (node->type == M_NODE && !(errcode == ERRCODE_DECERR || errcode == ERRCODE_UNSUPPORTED))
		return;

	new_info = kmalloc(sizeof(struct itmon_traceinfo), GFP_ATOMIC);
	if (!new_info)
		return;

	axid = (unsigned int)BIT_AXID(data->int_info);
	if (group->path_type == DATA)
		userbit = BIT_AXUSER(data->ext_user);
	else
		userbit = BIT_AXUSER_PERI(data->ext_info_2);

	new_info->port = NULL;
	new_info->client = NULL;
	new_info->user = userbit;

	switch (node->type) {
	case S_NODE:
	case T_S_NODE:
		new_info->dest = node->name;
		port = itmon_get_rpathinfo(itmon, axid, node->name);
		list_for_each_entry(find_node, &pdata->datalist[trans_type], list) {
			if (find_node->type != M_NODE)
				continue;

			if (!itmon_parse_cpuinfo(itmon, find_node, new_info, userbit)) {
				break;
			} else if (port && !strcmp(find_node->name, port->port_name)) {
				new_info->port = port->port_name;
				client = itmon_get_clientinfo(itmon, new_info->port, userbit);
				if (client) {
					new_info->client = client->client_name;
					if (!strcmp(client->client_name, "CPU"))
						set_bit(FROM_CPU, &new_info->from);
				}

				find_data = &find_node->tracedata;
				find_data->ref_info = new_info;
				break;
			}
			if (port)
				continue;

			for (i = 0; i < (int)ARRAY_SIZE(rpathinfo); i++) {
				if (strcmp(find_node->name, pdata->rpathinfo[i].port_name)) {
					new_info->port = find_node->name;
					new_info->client = " ";
					find_data = &find_node->tracedata;
					find_data->ref_info = new_info;
					break;
				}
			}
		}
		if (!new_info->port) {
			new_info->port = "Failed to parsing";
			new_info->client = "Refer to Raw Node information";
		}
		break;
	case M_NODE:
		new_info->dest = "Refer to address";

		if (!itmon_parse_cpuinfo(itmon, node, new_info, userbit))
			break;

		new_info->port = node->name;
		client = itmon_get_clientinfo(itmon, node->name, userbit);
		new_info->client = client ? client->client_name : " ";
		break;
	default:
		log_dev_err(itmon->dev,
			    "Unknown Error - offset:%u\n", data->offset);
		return;
	}

	/* Last Information */
	new_info->path_type = group->path_type;
	new_info->target_addr =
		(((unsigned long)node->tracedata.ext_info_1
		& GENMASK(15, 0)) << 32ULL);
	new_info->target_addr |= node->tracedata.ext_info_0;
	new_info->errcode = errcode;
	new_info->dirty = true;
	new_info->axsize = BIT_AXSIZE(data->ext_info_1);
	new_info->axlen = BIT_AXLEN(data->ext_info_1);
	new_info->axburst = BIT_AXBURST(data->ext_info_2);
	new_info->axprot = BIT_AXPROT(data->ext_info_2);
	new_info->baaw_prot = false;

	for (i = 0; i < (int)ARRAY_SIZE(itmon_invalid_addr); i++) {
		if (new_info->target_addr == itmon_invalid_addr[i]) {
			new_info->baaw_prot = true;
			break;
		}
	}
	data->ref_info = new_info;
	list_add(&new_info->list, &pdata->infolist[trans_type]);
}

static void itmon_analyze_errnode(struct itmon_dev *itmon)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_traceinfo *info, *next_info;
	struct itmon_tracedata *data;
	struct itmon_nodeinfo *node, *next_node;
	unsigned int trans_type;
	int i;

	/* Parse */
	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		list_for_each_entry(node, &pdata->datalist[trans_type], list) {
			if (node->type == S_NODE || node->type == M_NODE || node->type == T_S_NODE)
				itmon_parse_traceinfo(itmon, node, trans_type);
		}
	}

	/* Report */
	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		list_for_each_entry(info, &pdata->infolist[trans_type], list) {
			info->path_dirty = false;
			list_for_each_entry(node, &pdata->datalist[trans_type], list) {
				if (!node)
					continue;

				data = &node->tracedata;
				if (data->ref_info == info)
					itmon_report_pathinfo(itmon, node, info, trans_type);
			}
			itmon_report_traceinfo(itmon, info, trans_type);
		}
	}

	/* Report Raw all tracedata and Clean-up tracedata and node */
	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		for (i = M_NODE; i < NODE_TYPE; i++) {
			list_for_each_entry_safe(node, next_node,
					&pdata->datalist[trans_type], list) {
				if (i == node->type) {
					itmon_report_rawdata(itmon, node, trans_type);
					list_del(&node->list);
					kfree(node);
				}
			}
		}
	}

	/* Rest works and Clean-up traceinfo */
	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		list_for_each_entry_safe(info, next_info, &pdata->infolist[trans_type], list) {
			itmon_post_handler_to_notifier(itmon, info, trans_type);
			itmon_post_handler_by_dest(itmon, info, trans_type);
			itmon_post_handler_by_client(itmon, info, trans_type);
			list_del(&info->list);
			kfree(info);
		}
	}
}

static void itmon_collect_errnode(struct itmon_dev *itmon,
			    struct itmon_nodegroup *group,
			    struct itmon_nodeinfo *node,
			    unsigned int offset)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *new_node = NULL;
	unsigned int int_info, info0, info1, info2, user;
	unsigned int prot_chk_ctl, prot_chk_info, prot_chk_int_id, dbg_mo_cnt;
	bool read = TRANS_TYPE_WRITE;
	bool req = false;

	int_info = __raw_readl(node->regs + offset + REG_INT_INFO);
	info0 = __raw_readl(node->regs + offset + REG_EXT_INFO_0);
	info1 = __raw_readl(node->regs + offset + REG_EXT_INFO_1);
	info2 = __raw_readl(node->regs + offset + REG_EXT_INFO_2);
	if (group->path_type == DATA)
		user = __raw_readl(node->regs + offset + REG_EXT_USER);

	dbg_mo_cnt = __raw_readl(node->regs + OFFSET_PROT_CHK);
	prot_chk_ctl = __raw_readl(node->regs +
				OFFSET_PROT_CHK + REG_PROT_CHK_CTL);
	prot_chk_info = __raw_readl(node->regs +
				OFFSET_PROT_CHK + REG_PROT_CHK_INT);
	prot_chk_int_id = __raw_readl(node->regs +
				OFFSET_PROT_CHK + REG_PROT_CHK_INT_ID);
	switch (offset) {
	case OFFSET_REQ_R:
		read = TRANS_TYPE_READ;
		fallthrough;
	case OFFSET_REQ_W:
		req = true;
		/* Only S-Node is able to make log to registers */
		break;
	case OFFSET_RESP_R:
		read = TRANS_TYPE_READ;
		fallthrough;
	case OFFSET_RESP_W:
		req = false;
		/* Only NOT S-Node is able to make log to registers */
		break;
	default:
		log_dev_err(itmon->dev, "Unknown Error - node:%s offset:%u\n",
			    node->name, offset);
		break;
	}

	new_node = kmalloc(sizeof(struct itmon_nodeinfo), GFP_ATOMIC);

	if (!new_node) {
		log_dev_err(itmon->dev, "failed to kmalloc for %s node %x offset\n",
			    node->name, offset);
		return;
	}

	/* Fill detected node information to tracedata's list */
	memcpy(new_node, node, sizeof(*new_node));
	new_node->tracedata.int_info = int_info;
	new_node->tracedata.ext_info_0 = info0;
	new_node->tracedata.ext_info_1 = info1;
	new_node->tracedata.ext_info_2 = info2;
	new_node->tracedata.ext_user = user;
	new_node->tracedata.dbg_mo_cnt = dbg_mo_cnt;
	new_node->tracedata.prot_chk_ctl = prot_chk_ctl;
	new_node->tracedata.prot_chk_info = prot_chk_info;
	new_node->tracedata.prot_chk_int_id = prot_chk_int_id;

	new_node->tracedata.offset = offset;
	new_node->tracedata.read = read;
	new_node->tracedata.ref_info = NULL;
	new_node->group = group;
	node->tracedata.logging = BIT_ERR_VALID(int_info);

	list_add(&new_node->list, &pdata->datalist[read]);
}

static int __itmon_search_node(struct itmon_dev *itmon,
			       struct itmon_nodegroup *group,
			       bool clear)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *node = NULL;
	unsigned int val, offset, freeze;
	unsigned long vec, bit = 0;
	int i, ret = 0;

	if (group->phy_regs) {
		if (group->ex_table)
			vec = (unsigned long)__raw_readq(group->regs);
		else
			vec = (unsigned long)__raw_readl(group->regs);
	} else {
		vec = GENMASK(group->nodesize, 0);
	}

	if (!vec)
		goto exit;

	node = group->nodeinfo;

	for_each_set_bit(bit, &vec, group->nodesize) {
		/* exist array */
		for (i = 0; i < OFFSET_NUM; i++) {
			offset = i * OFFSET_ERR_REPT;
			/* Check Request information */
			val = __raw_readl(node[bit].regs + offset + REG_INT_INFO);
			if (BIT_ERR_OCCURRED(val)) {
				/* This node occurs the error */
				itmon_collect_errnode(itmon, group, &node[bit], offset);
				if (clear)
					__raw_writel(1, node[bit].regs
							+ offset + REG_INT_CLR);
				ret = true;
			}
		}
		/* Check H/W assertion */
		if (node[bit].prot_chk_enabled) {
			val = __raw_readl(node[bit].regs + OFFSET_PROT_CHK +
						REG_PROT_CHK_INT);
			/*
			 * if timeout_freeze is enable,
			 * PROT_CHK interrupt is able to assert without any information
			 */
			if (BIT_PROT_CHK_ERR_OCCURRED(val) && (val & GENMASK(8, 1))) {
				itmon_report_prot_chk_rawdata(itmon, &node[bit]);
				pdata->policy[FATAL].error = true;
				ret = true;
			}
		}
		/* Check freeze enable node */
		if (node[bit].type == S_NODE && node[bit].tmout_frz_enabled) {
			val = __raw_readl(node[bit].regs + OFFSET_TMOUT_REG  +
						REG_TMOUT_FRZ_STATUS);
			freeze = val & (unsigned int)GENMASK(1, 0);
			if (freeze) {
				if (freeze & BIT(0))
					itmon_report_timeout(itmon, &node[bit], TRANS_TYPE_WRITE);
				if (freeze & BIT(1))
					itmon_report_timeout(itmon, &node[bit], TRANS_TYPE_READ);
				pdata->policy[FATAL].error = true;
				ret = true;
			}
		}
	}
exit:
	return ret;
}

static int itmon_search_node(struct itmon_dev *itmon,
			     struct itmon_nodegroup *group,
			     bool clear)
{
	int i, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&itmon->ctrl_lock, flags);

	if (group) {
		ret = __itmon_search_node(itmon, group, clear);
	} else {
		/* Processing all group & nodes */
		for (i = 0; i < (int)ARRAY_SIZE(nodegroup); i++) {
			group = &nodegroup[i];
			ret |= __itmon_search_node(itmon, group, clear);
		}
	}
	itmon_analyze_errnode(itmon);

	spin_unlock_irqrestore(&itmon->ctrl_lock, flags);
	return ret;
}

static void itmon_do_dpm_policy(struct itmon_dev *itmon)
{
	struct itmon_platdata *pdata = itmon->pdata;
	int i;

	/* This will stop recursive panic when dpm action is panic */
	pdata->in_do_policy = true;


	for (i = 0; i < TYPE_MAX; i++) {
		char buf[SZ_64];

		if (!pdata->policy[i].error)
			continue;

		scnprintf(buf, sizeof(buf), "itmon triggering %s %s",
			  pdata->policy[i].name, itmon->itmon_pattern);
		dbg_snapshot_do_dpm_policy(pdata->policy[i].policy, buf);
		pdata->policy[i].error = false;
	}

	pdata->in_do_policy = false;
}

static irqreturn_t itmon_irq_handler(int irq, void *data)
{
	struct itmon_dev *itmon = (struct itmon_dev *)data;
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodegroup *group = NULL;
	bool ret;
	int i;

#if IS_ENABLED(CONFIG_EXYNOS_CPUPM)
	system_is_in_itmon = true;
#endif

	itmon_pattern_reset(itmon);
	dbg_snapshot_itmon_irq_received();

	/* Search itmon group */
	for (i = 0; i < (int)ARRAY_SIZE(nodegroup); i++) {
		group = &pdata->nodegroup[i];
		log_dev_info(itmon->dev,
			     "%d irq, %s group, 0x%x\n",
			     irq, group->name,
			     group->phy_regs == 0 ? 0 : __raw_readl(group->regs));
	}

	ret = itmon_search_node(itmon, NULL, true);
	if (!ret) {
		log_dev_info(itmon->dev, "No errors found\n");
	} else {
		log_dev_err(itmon->dev, "\nError detected: err_cnt_by_cpu:%u\n",
			    pdata->err_cnt_by_cpu);

		/* This will stop recursive panic when dpm action is panic */
		if (!pdata->in_do_policy)
			itmon_do_dpm_policy(itmon);
	}

#if IS_ENABLED(CONFIG_EXYNOS_CPUPM)
	system_is_in_itmon = false;
#endif

	return IRQ_HANDLED;
}

void itmon_notifier_chain_register(struct notifier_block *block)
{
	atomic_notifier_chain_register(&itmon_notifier_list, block);
}
EXPORT_SYMBOL(itmon_notifier_chain_register);

void itmon_notifier_chain_unregister(struct notifier_block *block)
{
	atomic_notifier_chain_unregister(&itmon_notifier_list, block);
}
EXPORT_SYMBOL(itmon_notifier_chain_unregister);

static struct bus_type itmon_subsys = {
	.name = "itmon",
	.dev_name = "itmon",
};

static ssize_t itmon_timeout_fix_val_show(struct kobject *kobj,
					 struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "set timeout val: 0x%x\n",
			g_itmon->pdata->sysfs_tmout_val);
}

static ssize_t itmon_timeout_fix_val_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long val = 0;
	struct itmon_platdata *pdata = g_itmon->pdata;
	int ret;

	ret = kstrtoul(buf, 16, &val);
	if (!ret) {
		if (val > 0 && val <= 0xFFFFF)
			pdata->sysfs_tmout_val = val;
	} else {
		log_dev_err(g_itmon->dev, "kstrtoul return value is %d\n", ret);
	}

	return count;
}

static ssize_t itmon_timeout_val_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	unsigned long i, offset;
	ssize_t n = 0;
	unsigned long vec, bit = 0;
	struct itmon_nodegroup *group = NULL;
	struct itmon_nodeinfo *node;

	/* Processing all group & nodes */
	offset = OFFSET_TMOUT_REG;
	for (i = 0; i < ARRAY_SIZE(nodegroup); i++) {
		group = &nodegroup[i];
		node = group->nodeinfo;
		vec = GENMASK(group->nodesize, 0);
		bit = 0;
		for_each_set_bit(bit, &vec, group->nodesize) {
			if (node[bit].type == S_NODE) {
				n += scnprintf(buf + n, PAGE_SIZE - n,
					"%-12s : 0x%08X, timeout : 0x%x\n",
					node[bit].name, node[bit].phy_regs,
					__raw_readl(node[bit].regs +
						offset + REG_TMOUT_INIT_VAL));
			}
		}
	}
	return n;
}

static ssize_t itmon_timeout_freeze_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	unsigned long i, offset;
	ssize_t n = 0;
	unsigned long vec, bit = 0;
	struct itmon_nodegroup *group = NULL;
	struct itmon_nodeinfo *node;

	/* Processing all group & nodes */
	offset = OFFSET_TMOUT_REG;
	for (i = 0; i < ARRAY_SIZE(nodegroup); i++) {
		group = &nodegroup[i];
		node = group->nodeinfo;
		vec = GENMASK(group->nodesize, 0);
		bit = 0;
		for_each_set_bit(bit, &vec, group->nodesize) {
			if (node[bit].type == S_NODE) {
				n += scnprintf(buf + n, PAGE_SIZE - n,
					"%-12s : 0x%08X, timeout_freeze : %x\n",
					node[bit].name, node[bit].phy_regs,
					__raw_readl(node[bit].regs +
						offset + REG_TMOUT_FRZ_EN));
			}
		}
	}
	return n;
}

static ssize_t itmon_timeout_val_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	char *name;
	unsigned int offset, i;
	unsigned long vec, bit = 0;
	struct itmon_nodegroup *group = NULL;
	struct itmon_nodeinfo *node;
	struct itmon_platdata *pdata = g_itmon->pdata;

	name = (char *)kstrndup(buf, count, GFP_KERNEL);
	if (!name)
		return count;

	offset = OFFSET_TMOUT_REG;
	for (i = 0; i < (int)ARRAY_SIZE(nodegroup); i++) {
		group = &nodegroup[i];
		node = group->nodeinfo;
		vec = GENMASK(group->nodesize, 0);
		bit = 0;
		for_each_set_bit(bit, &vec, group->nodesize) {
			if (node[bit].type == S_NODE &&
				!strcmp(name, node[bit].name)) {
				__raw_writel(pdata->sysfs_tmout_val,
						node[bit].regs + offset +
							REG_TMOUT_INIT_VAL);
				node[bit].time_val = pdata->sysfs_tmout_val;
			}
		}
	}
	kfree(name);
	return count;
}

static ssize_t itmon_timeout_freeze_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	char *name;
	unsigned int val, offset, i;
	unsigned long vec, bit = 0;
	struct itmon_nodegroup *group = NULL;
	struct itmon_nodeinfo *node;

	name = (char *)kstrndup(buf, count, GFP_KERNEL);
	if (!name)
		return count;

	offset = OFFSET_TMOUT_REG;
	for (i = 0; i < (int)ARRAY_SIZE(nodegroup); i++) {
		group = &nodegroup[i];
		node = group->nodeinfo;
		vec = GENMASK(group->nodesize, 0);
		bit = 0;
		for_each_set_bit(bit, &vec, group->nodesize) {
			if (node[bit].type == S_NODE &&
				!strcmp(name, node[bit].name)) {
				val = __raw_readl(node[bit].regs +
						offset + REG_TMOUT_FRZ_EN);
				val = !val;
				__raw_writel(val, node[bit].regs +
						offset + REG_TMOUT_FRZ_EN);
				node[bit].tmout_frz_enabled = val;
			}
		}
	}
	kfree(name);
	return count;
}

static struct kobj_attribute itmon_timeout_fix_attr =
	__ATTR(set_val, 0644, itmon_timeout_fix_val_show,
			itmon_timeout_fix_val_store);
static struct kobj_attribute itmon_timeout_val_attr =
	__ATTR(timeout_val, 0644, itmon_timeout_val_show,
			itmon_timeout_val_store);
static struct kobj_attribute itmon_timeout_freeze_attr =
	__ATTR(timeout_freeze, 0644, itmon_timeout_freeze_show,
			itmon_timeout_freeze_store);

static struct attribute *itmon_sysfs_attrs[] = {
	&itmon_timeout_fix_attr.attr,
	&itmon_timeout_val_attr.attr,
	&itmon_timeout_freeze_attr.attr,
	NULL,
};

static struct attribute_group itmon_sysfs_group = {
	.attrs = itmon_sysfs_attrs,
};

static const struct attribute_group *itmon_sysfs_groups[] = {
	&itmon_sysfs_group,
	NULL,
};

static int itmon_logging_panic_handler(struct notifier_block *nb,
				     unsigned long l, void *buf)
{
	struct itmon_panic_block *itmon_panic = (struct itmon_panic_block *)nb;
	struct itmon_dev *itmon = itmon_panic->pdev;
	struct itmon_platdata *pdata = itmon->pdata;
	int ret;

	if (!IS_ERR_OR_NULL(itmon)) {
		/* Check error has been logged */
		ret = itmon_search_node(itmon, NULL, false);
		if (!ret) {
			log_dev_info(itmon->dev, "No errors found\n");
		} else {
			log_dev_err(itmon->dev,
				    "Error detected, err_cnt_by_cpu:%u\n",
				pdata->err_cnt_by_cpu);

			itmon_do_dpm_policy(itmon);
		}
	}
	return 0;
}

static void itmon_parse_dt(struct itmon_dev *itmon)
{
	struct device_node *np = itmon->dev->of_node;
	struct itmon_platdata *pdata = itmon->pdata;
	unsigned int val;
	int i;

	if (!of_property_read_u32(np, "panic_count", &val))
		pdata->panic_threshold = val;
	else
		pdata->panic_threshold = PANIC_THRESHOLD;

	log_dev_info(itmon->dev, "panic threshold: %d\n", pdata->panic_threshold);
	for (i = 0; i < TYPE_MAX; i++) {
		if (!of_property_read_u32(np, pdata->policy[i].name, &val))
			pdata->policy[i].policy = val;

		log_dev_info(itmon->dev, "policy: %s: [%d]\n",
			     pdata->policy[i].name, pdata->policy[i].policy);
	}
}


static int itmon_probe(struct platform_device *pdev)
{
	struct itmon_dev *itmon;
	struct itmon_panic_block *itmon_panic = NULL;
	struct itmon_platdata *pdata;
	struct itmon_nodeinfo *node;
	unsigned int irq_option = 0, irq;
	char *dev_name;
	int ret, i, j;

	itmon = devm_kzalloc(&pdev->dev,
				sizeof(struct itmon_dev), GFP_KERNEL);
	if (!itmon)
		return -ENOMEM;

	itmon->dev = &pdev->dev;

	spin_lock_init(&itmon->ctrl_lock);

	pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct itmon_platdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	pdata->cp_crash_in_progress = false;
	for (i = 0; i < TRANS_TYPE_NUM; i++) {
		INIT_LIST_HEAD(&pdata->datalist[i]);
		INIT_LIST_HEAD(&pdata->infolist[i]);
	}

	itmon->pdata = pdata;
	itmon->pdata->clientinfo = clientinfo;
	itmon->pdata->rpathinfo = rpathinfo;
	itmon->pdata->nodegroup = nodegroup;
	itmon->pdata->policy = err_policy;

	itmon_parse_dt(itmon);

	for (i = 0; i < (int)ARRAY_SIZE(nodegroup); i++) {
		dev_name = nodegroup[i].name;
		node = nodegroup[i].nodeinfo;

		if (nodegroup[i].phy_regs) {
			nodegroup[i].regs = devm_ioremap(&pdev->dev, nodegroup[i].phy_regs, SZ_16K);
			if (nodegroup[i].regs == NULL) {
				log_dev_err(&pdev->dev,
					    "failed to claim register region - %s\n",
					dev_name);
				return -ENOENT;
			}
		}
		irq = irq_of_parse_and_map(pdev->dev.of_node, i);
		nodegroup[i].irq = irq;

		ret = devm_request_irq(&pdev->dev, irq,
				       itmon_irq_handler, irq_option, dev_name, itmon);
		if (ret == 0) {
			log_dev_info(&pdev->dev,
				     "success to register request irq%u - %s\n",
				     irq, dev_name);
		} else {
			log_dev_err(&pdev->dev, "failed to request irq - %s\n",
				    dev_name);
			return -ENOENT;
		}

		for (j = 0; j < nodegroup[i].nodesize; j++) {
			node[j].regs = devm_ioremap(&pdev->dev, node[j].phy_regs, SZ_16K);
			if (node[j].regs == NULL) {
				log_dev_err(&pdev->dev,
					    "failed to claim register region - %s\n",
					    dev_name);
				return -ENOENT;
			}
		}
	}

	itmon_panic = devm_kzalloc(&pdev->dev, sizeof(struct itmon_panic_block),
				 GFP_KERNEL);
	if (itmon_panic) {
		itmon_panic->nb_panic_block.notifier_call = itmon_logging_panic_handler;
		itmon_panic->pdev = itmon;
		atomic_notifier_chain_register(&panic_notifier_list,
					       &itmon_panic->nb_panic_block);
	}

	platform_set_drvdata(pdev, itmon);
	itmon_init(itmon, true);
	itmon_pattern_reset(itmon);
	g_itmon = itmon;

	ret = subsys_system_register(&itmon_subsys, itmon_sysfs_groups);
	if (ret)
		log_dev_err(g_itmon->dev, "fail to register itmon subsys\n");

	pdata->probed = true;
	log_dev_info(&pdev->dev, "success to probe gs201 ITMON driver\n");

	return 0;
}

static int itmon_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int itmon_suspend(struct device *dev)
{
	return 0;
}

static int itmon_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct itmon_dev *itmon = platform_get_drvdata(pdev);
	struct itmon_platdata *pdata = itmon->pdata;

	/* re-enable ITMON if cp-crash progress is not starting */
	if (!pdata->cp_crash_in_progress)
		itmon_init(itmon, true);

	return 0;
}

static SIMPLE_DEV_PM_OPS(itmon_pm_ops, itmon_suspend, itmon_resume);
#define ITMON_PM	(&itmon_pm_ops)
#else
#define ITMON_PM	NULL
#endif

static const struct of_device_id itmon_dt_match[] = {
	{.compatible = "google,gs201-itmon", },
	{},
};
MODULE_DEVICE_TABLE(of, itmon_dt_match);

static struct platform_driver gs201_itmon_driver = {
	.probe = itmon_probe,
	.remove = itmon_remove,
	.driver = {
		   .name = "gs201-itmon",
		   .of_match_table = itmon_dt_match,
		   .pm = ITMON_PM,
		   },
};
module_platform_driver(gs201_itmon_driver);

MODULE_DESCRIPTION("Google GS201 ITMON DRIVER");
MODULE_AUTHOR("Hosung Kim <hosung0.kim@samsung.com");
MODULE_LICENSE("GPL v2");
