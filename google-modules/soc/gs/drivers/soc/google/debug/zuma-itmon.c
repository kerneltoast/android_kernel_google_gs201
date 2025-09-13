// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/panic_notifier.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/sched/clock.h>

#include <soc/google/exynos-itmon.h>
#include <soc/google/debug-snapshot.h>

/* #define MULTI_IRQ_SUPPORT_ITMON */
#define DBG_CTL				(0x1800)
#define ERR_LOG_STAT			(0x1804)
#define ERR_LOG_POP			(0x1808)
#define ERR_LOG_CLR			(0x180C)
#define ERR_LOG_EN_NODE			(0x1810)
#define ERR_LOG_INFO_0			(0x1840)
#define ERR_LOG_INFO_1			(0x1844)
#define ERR_LOG_INFO_2			(0x1848)
#define ERR_LOG_INFO_3			(0x184C)
#define ERR_LOG_INFO_4			(0x1850)
#define ERR_LOG_INFO_5			(0x1854)
#define ERR_LOG_INFO_6			(0x1858)

#define INT_EN				BIT(0)
#define ERR_LOG_EN			BIT(1)
#define TMOUT_EN			BIT(2)
#define	PROT_CHK_EN			BIT(3)
#define FIXED_DET_EN			BIT(4)

#define PRTCHK_M_CTL(x)			(0x1C00 + ((x) * 4))
#define PRTCHK_S_CTL(x)			(0x1D00 + ((x) * 4))
#define TMOUT_CTL(x)			(0x1900 + ((x) * 4))

#define OFFSET_TMOUT_REG		(0x1A00)

#define TMOUT_POINT_ADDR		(0x40)
#define TMOUT_RW_OFFSET			(0x20)
#define TMOUT_ID			(0x44)
#define TMOUT_PAYLOAD_0			(0x48)
#define TMOUT_PAYLOAD_1			(0x4C)
#define TMOUT_PAYLOAD_2			(0x50)
#define TMOUT_PAYLOAD_3			(0x54)
#define TMOUT_PAYLOAD_4			(0x58)
#define TMOUT_PAYLOAD_5			(0x5C)

#define PRTCHK_START_ADDR_LOW		(0x1B00)
#define PRTCHK_END_ADDR_LOW		(0x1B04)
#define PRTCHK_START_END_ADDR_UPPER	(0x1B08)

#define RD_RESP_INT_EN			BIT(0)
#define WR_RESP_INT_EN			BIT(1)
#define ARLEN_RLAST_INT_EN		BIT(2)
#define AWLEN_WLAST_INT_EN		BIT(3)
#define INTEND_ACCESS_INT_EN		BIT(4)

#define ERR_CODE(x)			((x) & (0xF))
#define MNODE_ID(x)			(((x) & (0xFF << 24)) >> 24)
#define DET_ID(x)			(((x) & (0xFF << 16)) >> 16)
#define RW(x)				(((x) & (0x1 << 12)) >> 12)

#define AXDOMAIN(x)			(((x) & ((0x1) << 31)) >> 31)
#define AXSIZE(x)			(((x) & (0x7 << 28)) >> 28)
#define AXCACHE(x)			(((x) & (0xF << 24)) >> 24)
#define AXQOD(x)			(((x) & (0xF << 20)) >> 20)
#define AXLEN(x)			(((x) & (0xF << 16)) >> 16)

#define PARSE_USER(x)			(((x) & (0x3F << 19)) >> 19)
#define PARSE_CPU_ID_CONFIG(x)		(((x) & (0x0F << 3)) >> 3)
#define PARSE_CPU_ID_DATA(x)		(((x) & (0x0F << 20)) >> 20)
#define PARSE_CPU_SRCATTR_TYPE(x)	(((x) & (0x3 << 27)) >> 27)
#define PARSE_CPU_SRCATTR_ACCS(x)	(((x) & (0x1 << 29)) >> 29)
#define PARSE_CPU_WRITE_EVIC(x)		(((x) & (0x1 << 26)) >> 26)

#define AXID(x)				(((x) & (0xFFFFFFFF)))
#define AXUSER(x, y)			((u64)((x) & (0xFFFFFFFF)) | (((y) & (0xFFFFFFFF)) << 32ULL))
#define ADDRESS(x, y)			((u64)((x) & (0xFFFFFFFF)) | (((y) & (0xFFFF)) << 32ULL))

#define AXBURST(x)			(((x) & (0x3 << 2)) >> 2)
#define AXPROT(x)			((x) & (0x3))

#define TMOUT_BIT_ERR_CODE(x)		(((x) & (0xF << 28)) >> 28)
#define TMOUT_BIT_ERR_OCCURRED(x)	(((x) & (0x1 << 27)) >> 27)
#define TMOUT_BIT_ERR_VALID(x)		(((x) & (0x1 << 26)) >> 26)
#define TMOUT_BIT_AXID(x)		(((x) & (0xFFFF)))
#define TMOUT_BIT_AXUSER(x)		(((x) & (0xFFFFFFFF)))
#define TMOUT_BIT_AXUSER_PERI(x)	(((x) & (0xFFFF << 16)) >> 16)
#define TMOUT_BIT_AXBURST(x)		(((x) & (0x3)))
#define TMOUT_BIT_AXPROT(x)		(((x) & (0x3 << 2)) >> 2)
#define TMOUT_BIT_AXLEN(x)		(((x) & (0xF << 16)) >> 16)
#define TMOUT_BIT_AXSIZE(x)		(((x) & (0x7 << 28)) >> 28)

#define TMOUT_PAYLOAD_0			(0x48)
#define TMOUT_PAYLOAD_1			(0x4C)
#define TMOUT_PAYLOAD_2			(0x50)
#define TMOUT_PAYLOAD_3			(0x54)
#define TMOUT_PAYLOAD_4			(0x58)
#define TMOUT_PAYLOAD_5			(0x5C)

#define ERR_SLVERR			(0)
#define ERR_DECERR			(1)
#define ERR_UNSUPPORTED			(2)
#define ERR_POWER_DOWN			(3)
#define ERR_INTEND			(4)
#define ERR_TMOUT			(6)
#define ERR_PRTCHK_ARID_RID		(8)
#define ERR_PRTCHK_AWID_RID		(9)
#define ERR_PRTCHK_ARLEN_RLAST_EL	(10)
#define ERR_PRTCHK_ARLEN_RLAST_NL	(11)
#define ERR_PRTCHK_AWLEN_WLAST_EL	(12)
#define ERR_PRTCHK_AWLEN_WLAST_NL	(13)
#define ERR_TMOUT_FREEZE		(15)

#define DATA				(0)
#define CONFIG				(1)
#define BUS_PATH_TYPE			(2)

#define TRANS_TYPE_WRITE		(0)
#define TRANS_TYPE_READ			(1)
#define TRANS_TYPE_NUM			(2)

#define CP_COMMON_STR			"MODEM_"
#define NO_NAME				"N/A"

#define TMOUT_VAL			(0xFFFFF)
#define TMOUT_TEST			(0x1)

#define ERR_THRESHOLD			(5)

/* This value will be fixed */
#define INTEND_ADDR_START		(0)
#define INTEND_ADDR_END			(0)

#define GET_IRQ(x)			((x) & 0xFFFF)
#define GET_AFFINITY(x)			(((x) >> 16) & 0xFFFF)

#define SET_IRQ(x)			((x) & 0xFFFF)
#define SET_AFFINITY(x)			(((x) & 0xFFFF) << 16)

#define OFFSET_DSS_CUSTOM		(0xF000)
#define CUSTOM_MAX_PRIO			(7)

#define CPU_NAME_LENGTH			(16)
#define	NOT_SUPPORT			(0xFF)

#define log_dev_err(dev, fmt, ...)	\
do {								\
	dev_err(dev, fmt, ##__VA_ARGS__);			\
	dbg_snapshot_itmon_backup_log(fmt, ##__VA_ARGS__);	\
} while (0)

#define log_dev_info(dev, fmt, ...)	\
do {								\
	dev_info(dev, fmt, ##__VA_ARGS__);			\
	dbg_snapshot_itmon_backup_log(fmt, ##__VA_ARGS__);	\
} while (0)

#define log_dev_dbg(dev, fmt, ...)	\
do {								\
	dev_dbg(dev, fmt, ##__VA_ARGS__);			\
	dbg_snapshot_itmon_backup_log(fmt, ##__VA_ARGS__);	\
} while (0)

enum err_type {
	TMOUT,
	PRTCHKER,
	DECERR,
	SLVERR,
	FATAL,
	TYPE_MAX,
};

struct itmon_policy {
	char *name;
	int policy_def;
	int policy;
	int prio;
	bool error;
};

static struct itmon_policy err_policy[] = {
	[TMOUT]         = {"err_tmout",         0, 0, 0, false},
	[PRTCHKER]      = {"err_prtchker",      0, 0, 0, false},
	[DECERR]        = {"err_decerr",        0, 0, 0, false},
	[SLVERR]        = {"err_slverr",        0, 0, 0, false},
	[FATAL]         = {"err_fatal",         0, 0, 0, false},
};

static const char * const itmon_dpm_action[] = {
	GO_DEFAULT,
	GO_PANIC,
	GO_WATCHDOG,
	GO_S2D,
	GO_ARRAYDUMP,
	GO_SCANDUMP,
	GO_HALT,
};

struct itmon_rpathinfo {
	unsigned short id;
	char port_name[16];
	char dest_name[16];
	unsigned short bits;
} __packed;

struct itmon_clientinfo {
	char port_name[16];
	unsigned short user;
	char client_name[24];
	unsigned short bits;
} __packed;

struct itmon_nodegroup;
struct itmon_nodeinfo;

struct itmon_traceinfo {
	u32 m_id;
	u32 s_id;
	unsigned int user;
	char *client;
	char *src;
	char *dest;
	struct itmon_nodeinfo *m_node;
	struct itmon_nodeinfo *s_node;
	u64 target_addr;
	u32 err_code;
	bool read;
	bool onoff;
	bool path_dirty;
	bool dirty;
	u32 path_type;
	char buf[SZ_64];
	u32 axsize;
	u32 axlen;
	u32 axburst;
	u32 axprot;
	bool baaw_prot;
	bool cpu_tran;
	struct list_head list;
};

struct itmon_tracedata {
	u32 info_0;
	u32 info_1;
	u32 info_2;
	u32 info_3;
	u32 info_4;
	u32 info_5;
	u32 info_6;
	u32 en_node;
	u32 m_id;
	u32 det_id;
	u32 err_code;
	struct itmon_traceinfo *ref_info;
	struct itmon_nodegroup *group;
	struct itmon_nodeinfo *m_node;
	struct itmon_nodeinfo *det_node;
	bool read;
	struct list_head list;
};

struct itmon_nodepolicy {
	union {
		u64 en;
		struct {
			u64 chk_set : 1;
			u64 prio : 3;
			u64 chk_errrpt : 1;
			u64 en_errrpt : 1;
			u64 chk_tmout : 1;
			u64 en_tmout : 1;
			u64 chk_prtchk : 1;
			u64 en_prtchk : 1;
			u64 chk_tmout_val : 1;
			u64 en_tmout_val : 28;
			u64 chk_freeze : 1;
			u64 en_freeze : 1;
			u64 chk_irq_mask : 1;
			u64 en_irq_mask : 1;
			u64 chk_job : 1;
			u64 chk_decerr_job : 1;
			u64 en_decerr_job : 3;
			u64 chk_slverr_job : 1;
			u64 en_slverr_job : 3;
			u64 chk_tmout_job : 1;
			u64 en_tmout_job : 3;
			u64 chk_prtchk_job : 1;
			u64 en_prtchk_job : 3;
		};
	};
} __packed;

struct itmon_nodeinfo {
	char name[16];
	u32 type : 3;
	u32 id : 5;
	u32 err_en : 1;
	u32 prt_chk_en : 1;
	u32 addr_detect_en : 1;
	u32 retention : 1;
	u32 tmout_en : 1;
	u32 tmout_frz_en : 1;
	u32 time_val : 28;
	u32 err_id : 8;
	u32 tmout_offset : 8;
	u32 prtchk_offset : 8;
	struct itmon_nodegroup *group;
	struct itmon_nodepolicy policy;
} __packed;

struct itmon_nodegroup {
	char name[16];
	u64 phy_regs;
	bool ex_table;
	struct itmon_nodeinfo *nodeinfo_phy;
	struct itmon_nodeinfo *nodeinfo;
	u32 nodesize;
	u32 path_type;
	void __iomem *regs;
	u32 irq;
	bool pd_support;
	bool pd_status;
	char pd_name[16];
	u32 src_in;
	bool frz_support;
} __packed;

struct itmon_keepdata {
	u32 magic;
	u64 base;
	u64 mem_clientinfo;
	u32 size_clientinfo;
	u32 num_clientinfo;
	u64 mem_nodegroup;
	u32 size_nodegroup;
	u32 num_nodegroup;
} __packed;

struct itmon_platdata {
	const struct itmon_rpathinfo *rpathinfo;
	struct itmon_clientinfo *clientinfo;
	struct itmon_nodegroup *nodegroup;

	int num_rpathinfo;
	int num_clientinfo;
	int num_nodegroup;

	struct list_head infolist[TRANS_TYPE_NUM];
	struct list_head datalist[TRANS_TYPE_NUM];
	unsigned long last_time;
	int last_errcnt;

	struct itmon_policy *policy;
	bool cpu_parsing;
	bool def_en;
	bool probed;
	struct adv_tracer_plugin *itmon_adv;
	bool adv_en;
	u32 sysfs_tmout_val;
	bool en;
};

struct itmon_dev {
	struct device *dev;
	struct itmon_platdata *pdata;
	struct itmon_keepdata *kdata;
	const struct of_device_id *match;
	u32 irq;
	int id;
	void __iomem *regs;
	spinlock_t ctrl_lock;	/* spinlock for itmon irq handle */
	struct itmon_notifier notifier_info;
	char itmon_pattern[SZ_32];
	atomic_t itmon_pattern_cnt;
};

struct itmon_panic_block {
	struct notifier_block nb_panic_block;
	struct itmon_dev *pdev;
};

static const struct itmon_rpathinfo rpathinfo[] = {
	/* NOCL0_IO0-1 */
	{0,	"AUR0",		"NOCL0_IO0",	0x3F},
	{1,	"AUR1",		"NOCL0_IO0",	0x3F},
	{2,	"BW",		"NOCL0_IO0",	0x3F},
	{3,	"TPU0",		"NOCL0_IO0",	0x3F},
	{4,	"TPU1",		"NOCL0_IO0",	0x3F},
	{5,	"G3DMMU",	"NOCL0_IO0",	0x3F},
	{6,	"G3D0",		"NOCL0_IO0",	0x3F},
	{6,	"G3D1",		"NOCL0_IO1",	0x3F},
	{7,	"DPUF0",	"NOCL0_IO0",	0x3F},
	{8,	"DPUF1",	"NOCL0_IO0",	0x3F},
	{9,	"ISPFE0",	"NOCL0_IO0",	0x3F},
	{10,	"ISPFE1",	"NOCL0_IO0",	0x3F},
	{11,	"ISPFE2",	"NOCL0_IO0",	0x3F},
	{12,	"ISPFE3",	"NOCL0_IO0",	0x3F},
	{13,	"RGBP2",	"NOCL0_IO0",	0x3F},
	{14,	"HSI2",		"NOCL0_IO0",	0x3F},
	{15,	"MFC0",		"NOCL0_IO0",	0x3F},
	{16,	"MFC1",		"NOCL0_IO0",	0x3F},
	{17,	"RGBP0",	"NOCL0_IO0",	0x3F},
	{18,	"RGBP1",	"NOCL0_IO0",	0x3F},
	{19,	"RGBP3",	"NOCL0_IO0",	0x3F},
	{20,	"RGBP4",	"NOCL0_IO0",	0x3F},
	{21,	"RGBP5",	"NOCL0_IO0",	0x3F},
	{22,	"RGBP6",	"NOCL0_IO0",	0x3F},
	{23,	"GDC0",		"NOCL0_IO0",	0x3F},
	{24,	"GDC1",		"NOCL0_IO0",	0x3F},
	{25,	"GDC2",		"NOCL0_IO0",	0x3F},
	{26,	"G2D0",		"NOCL0_IO0",	0x3F},
	{27,	"G2D1",		"NOCL0_IO0",	0x3F},
	{28,	"G2D2",		"NOCL0_IO0",	0x3F},
	{29,	"GSE",		"NOCL0_IO0",	0x3F},
	{30,	"MCSC0",	"NOCL0_IO0",	0x3F},
	{31,	"MCSC1",	"NOCL0_IO0",	0x3F},
	{32,	"MISC",		"NOCL0_IO0",	0x3F},
	{33,	"YUVP",		"NOCL0_IO0",	0x3F},
	{34,	"TNR0",		"NOCL0_IO0",	0x3F},
	{35,	"TNR1",		"NOCL0_IO0",	0x3F},
	{36,	"TNR2",		"NOCL0_IO0",	0x3F},
	{37,	"TNR3",		"NOCL0_IO0",	0x3F},
	{38,	"TNR4",		"NOCL0_IO0",	0x3F},
	{39,	"TNR5",		"NOCL0_IO0",	0x3F},
	{40,	"AUR0",		"NOCL0_IO1",	0x3F},
	{41,	"AUR1",		"NOCL0_IO1",	0x3F},
	{42,	"BW",		"NOCL0_IO1",	0x3F},
	{43,	"TPU0",		"NOCL0_IO1",	0x3F},
	{44,	"TPU1",		"NOCL0_IO1",	0x3F},
	{45,	"G3DMMU",	"NOCL0_IO1",	0x3F},
	{46,	"G3D2",		"NOCL0_IO0",	0x3F},
	{46,	"G3D3",		"NOCL0_IO1",	0x3F},
	{47,	"DPUF0",	"NOCL0_IO1",	0x3F},
	{48,	"DPUF1",	"NOCL0_IO1",	0x3F},
	{49,	"ISPFE0",	"NOCL0_IO1",	0x3F},
	{50,	"ISPFE1",	"NOCL0_IO1",	0x3F},
	{51,	"ISPFE2",	"NOCL0_IO1",	0x3F},
	{52,	"ISPFE3",	"NOCL0_IO1",	0x3F},
	{53,	"RGBP2",	"NOCL0_IO1",	0x3F},
	{54,	"HSI2",		"NOCL0_IO1",	0x3F},
	{55,	"MFC0",		"NOCL0_IO1",	0x3F},
	{56,	"MFC1",		"NOCL0_IO1",	0x3F},
	{57,	"RGBP0",	"NOCL0_IO1",	0x3F},
	{58,	"RGBP1",	"NOCL0_IO1",	0x3F},
	{59,	"RGBP3",	"NOCL0_IO1",	0x3F},
	{60,	"RGBP4",	"NOCL0_IO1",	0x3F},
	{61,	"RGBP5",	"NOCL0_IO1",	0x3F},
	{62,	"RGBP6",	"NOCL0_IO1",	0x3F},
	{63,	"GDC0",		"NOCL0_IO1",	0x3F},
	{64,	"GDC1",		"NOCL0_IO1",	0x3F},
	{65,	"GDC2",		"NOCL0_IO1",	0x3F},
	{66,	"G2D0",		"NOCL0_IO1",	0x3F},
	{67,	"G2D1",		"NOCL0_IO1",	0x3F},
	{68,	"G2D2",		"NOCL0_IO1",	0x3F},
	{69,	"GSE",		"NOCL0_IO1",	0x3F},
	{70,	"MCSC0",	"NOCL0_IO1",	0x3F},
	{71,	"MCSC1",	"NOCL0_IO1",	0x3F},
	{72,	"MISC",		"NOCL0_IO1",	0x3F},
	{73,	"YUVP",		"NOCL0_IO1",	0x3F},
	{74,	"TNR0",		"NOCL0_IO1",	0x3F},
	{75,	"TNR1",		"NOCL0_IO1",	0x3F},
	{76,	"TNR2",		"NOCL0_IO1",	0x3F},
	{77,	"TNR3",		"NOCL0_IO1",	0x3F},
	{78,	"TNR4",		"NOCL0_IO1",	0x3F},
	{79,	"TNR5",		"NOCL0_IO1",	0x3F},
	{80,	"ALIVE",	"NOCL0_IO",	0x3F},
	{81,	"AOC",		"NOCL0_IO",	0x3F},
	{82,	"CSSYS",	"NOCL0_IO",	0x3F},
	{83,	"GSA",		"NOCL0_IO",	0x3F},
	{84,	"HSI0",		"NOCL0_IO",	0x3F},
	{85,	"HIS1",		"NOCL0_IO",	0x3F},

	/* NOCL0_M0 / M1 / M2 / M3 CPU specific */
	{0,	"CPU0",		"NOCL0_M0",	0x3F},
	{0,	"CPU1",		"NOCL0_M1",	0x3F},
	{0,	"CPU2",		"NOCL0_M2",	0x3F},
	{0,	"CPU3",		"NOCL0_M3",	0x3F},

	/* NOCL0_M0 / M1 / M2 / M3 Common */
	{1,	"AUR0",		"NOCL0_M",	0x3F},
	{2,	"AUR1",		"NOCL0_M",	0x3F},
	{3,	"BW",		"NOCL0_M",	0x3F},
	{4,	"TPU0",		"NOCL0_M",	0x3F},
	{5,	"TPU1",		"NOCL0_M",	0x3F},
	{6,	"G3DMMU",	"NOCL0_M",	0x3F},
	{7,	"G3D0",		"NOCL0_M0",	0x3F},
	{7,	"G3D1",		"NOCL0_M1",	0x3F},
	{7,	"G3D2",		"NOCL0_M2",	0x3F},
	{7,	"G3D3",		"NOCL0_M3",	0x3F},
	{8,	"DPUF0",	"NOCL0_M",	0x3F},
	{9,	"DPUF1",	"NOCL0_M",	0x3F},
	{10,	"ISPFE0",	"NOCL0_M",	0x3F},
	{11,	"ISPFE1",	"NOCL0_M",	0x3F},
	{12,	"ISPFE2",	"NOCL0_M",	0x3F},
	{13,	"ISPFE3",	"NOCL0_M",	0x3F},
	{14,	"RGBP2",	"NOCL0_M",	0x3F},
	{15,	"HSI2",		"NOCL0_M",	0x3F},
	{16,	"MFC0",		"NOCL0_M",	0x3F},
	{17,	"MFC1",		"NOCL0_M",	0x3F},
	{18,	"RGBP0",	"NOCL0_M",	0x3F},
	{19,	"RGBP1",	"NOCL0_M",	0x3F},
	{20,	"RGBP3",	"NOCL0_M",	0x3F},
	{21,	"RGBP4",	"NOCL0_M",	0x3F},
	{22,	"RGBP5",	"NOCL0_M",	0x3F},
	{23,	"RGBP6",	"NOCL0_M",	0x3F},
	{24,	"GDC0",		"NOCL0_M",	0x3F},
	{25,	"GDC1",		"NOCL0_M",	0x3F},
	{26,	"GDC2",		"NOCL0_M",	0x3F},
	{27,	"G2D0",		"NOCL0_M",	0x3F},
	{28,	"G2D1",		"NOCL0_M",	0x3F},
	{29,	"G2D2",		"NOCL0_M",	0x3F},
	{30,	"GSE",		"NOCL0_M",	0x3F},
	{31,	"MCSC0",	"NOCL0_M",	0x3F},
	{32,	"MCSC1",	"NOCL0_M",	0x3F},
	{33,	"MISC",		"NOCL0_M",	0x3F},
	{34,	"YUVP",		"NOCL0_M",	0x3F},
	{35,	"TNR0",		"NOCL0_M",	0x3F},
	{36,	"TNR1",		"NOCL0_M",	0x3F},
	{37,	"TNR2",		"NOCL0_M",	0x3F},
	{38,	"TNR3",		"NOCL0_M",	0x3F},
	{39,	"TNR4",		"NOCL0_M",	0x3F},
	{40,	"TNR5",		"NOCL0_M",	0x3F},
	{41,	"ALIVE",	"NOCL0_M",	0x3F},
	{42,	"AOC",		"NOCL0_M",	0x3F},
	{43,	"CSSYS",	"NOCL0_M",	0x3F},
	{44,	"GSA",		"NOCL0_M",	0x3F},
	{45,	"HSI0",		"NOCL0_M",	0x3F},
	{46,	"HIS1",		"NOCL0_M",	0x3F},

	/* 0x0000_0000 - 0x7fff_ffff */
	{0,	"AUR0",		"NOCL0_DP",	0x3F},
	{1,	"AUR1",		"NOCL0_DP",	0x3F},
	{2,	"BW",		"NOCL0_DP",	0x3F},
	{3,	"TPU0",		"NOCL0_DP",	0x3F},
	{4,	"TPU1",		"NOCL0_DP",	0x3F},
	{5,	"G3DMMU",	"NOCL0_DP",	0x3F},
	{6,	"G3D0",		"NOCL0_DP",	0x3F},
	{7,	"DPUF0",	"NOCL0_DP",	0x3F},
	{8,	"DPUF1",	"NOCL0_DP",	0x3F},
	{9,	"ISPFE0",	"NOCL0_DP",	0x3F},
	{10,	"ISPFE1",	"NOCL0_DP",	0x3F},
	{11,	"ISPFE2",	"NOCL0_DP",	0x3F},
	{12,	"ISPFE3",	"NOCL0_DP",	0x3F},
	{13,	"RGBP2",	"NOCL0_DP",	0x3F},
	{14,	"HSI2",		"NOCL0_DP",	0x3F},
	{15,	"MFC0",		"NOCL0_DP",	0x3F},
	{16,	"MFC1",		"NOCL0_DP",	0x3F},
	{17,	"RGBP0",	"NOCL0_DP",	0x3F},
	{18,	"RGBP1",	"NOCL0_DP",	0x3F},
	{19,	"RGBP3",	"NOCL0_DP",	0x3F},
	{20,	"RGBP4",	"NOCL0_DP",	0x3F},
	{21,	"RGBP5",	"NOCL0_DP",	0x3F},
	{22,	"RGBP6",	"NOCL0_DP",	0x3F},
	{23,	"GDC0",		"NOCL0_DP",	0x3F},
	{24,	"GDC1",		"NOCL0_DP",	0x3F},
	{25,	"GDC2",		"NOCL0_DP",	0x3F},
	{26,	"G2D0",		"NOCL0_DP",	0x3F},
	{27,	"G2D1",		"NOCL0_DP",	0x3F},
	{28,	"G2D2",		"NOCL0_DP",	0x3F},
	{29,	"GSE",		"NOCL0_DP",	0x3F},
	{30,	"MCSC0",	"NOCL0_DP",	0x3F},
	{31,	"MCSC1",	"NOCL0_DP",	0x3F},
	{32,	"MISC",		"NOCL0_DP",	0x3F},
	{33,	"YUVP",		"NOCL0_DP",	0x3F},
	{34,	"TNR0",		"NOCL0_DP",	0x3F},
	{35,	"TNR1",		"NOCL0_DP",	0x3F},
	{36,	"TNR2",		"NOCL0_DP",	0x3F},
	{37,	"TNR3",		"NOCL0_DP",	0x3F},
	{38,	"TNR4",		"NOCL0_DP",	0x3F},
	{39,	"TNR5",		"NOCL0_DP",	0x3F},
	{40,	"ALIVE",	"NOCL0_DP",	0x3F},
	{41,	"AOC",		"NOCL0_DP",	0x3F},
	{42,	"CSSYS",	"NOCL0_DP",	0x3F},
	{43,	"GSA",		"NOCL0_DP",	0x3F},
	{44,	"HSI0",		"NOCL0_DP",	0x3F},
	{45,	"HIS1",		"NOCL0_DP",	0x3F},

	/* Configuration Interconnect */
	{0,	"BOOKER",	"CONFIGURATION",	0x3},
	{1,	"CSSYS",	"CONFIGURATION",	0x3},
	{2,	"NOCL0_DP",	"CONFIGURATION",	0x3},
};

static struct itmon_clientinfo clientinfo[] = {
	{"AUR0",	0x0,	/*XXXXXX*/	"AUR_M0",	0x0},
	{"AUR1",	0x0,	/*XXXXX0*/	"AUR_M1",	0x1},
	{"AUR1",	0x1,	/*XXXXX1*/	"SYSMMU_S0_AUR",	0x1},
	{"BW",		0x0,	/*XXXXX0*/	"BW M0",	0x1},
	{"BW",		0x1,	/*XXXXX1*/	"SYSMMU_S0_BW",	0x1},
	{"G3D0",	0x0,	/*XXXXXX*/	"GPU M0",	0x0},
	{"G3D1",	0x0,	/*XXXXXX*/	"GPU M1",	0x0},
	{"G3D2",	0x0,	/*XXXXXX*/	"GPU M2",	0x0},
	{"G3D3",	0x0,	/*XXXXXX*/	"GPU M3",	0x0},
	{"G3DMMU",	0x0,	/*XXXXXX*/	"SYSMMU_S0_G3D",	0x0},
	{"TPU0",	0x0,	/*XXXXXX*/	"TPU_M0",	0x0},
	{"TPU1",	0x0,	/*XXXXX0*/	"TPU_M1",	0x1},
	{"TPU1",	0x1,	/*XXXXX1*/	"SYSMMU_S0_TPU",	0x1},
	{"GSA",		0x1,	/*XXXXX1*/	"SYSMMU_S0_GSA_ZM",	0x1},
	{"GSA",		0x6,	/*XXX110*/	"uGME",	0x7},
	{"GSA",		0x2,	/*X00010*/	"CA32_GSACORE",	0x1F},
	{"GSA",		0xA,	/*X01010*/	"SC_GSACORE",	0x1F},
	{"GSA",		0x12,	/*X10010*/	"DMA_GSACORE",	0x1F},
	{"GSA",		0x1A,	/*X11010*/	"DAP",	0x1F},
	{"GSA",		0x0,	/*XX0000*/	"CA32_GSACORE_BAAW",	0xF},
	{"GSA",		0x4,	/*XX0100*/	"SC_GSACORE_BAAW",	0xF},
	{"GSA",		0x8,	/*XX1000*/	"DMA_GSACORE_BAAW",	0xF},
	{"GSA",		0xC,	/*XX1100*/	"DAP_BAAW",	0xF},
	{"ALIVE",	0x1,	/*XXXXX1*/	"SYSMMU_S0_ALIVE (S2)",	0x1},
	{"ALIVE",	0x0,	/*XXX000*/	"APM",	0x7},
	{"ALIVE",	0x2,	/*XXX010*/	"GREBE_DBGCORE",	0x7},
	{"ALIVE",	0x4,	/*XXX100*/	"APM_DMA",	0x7},
	{"ALIVE",	0x6,	/*XXX110*/	"PMU",	0x7},
	{"HSI0",	0x1,	/*XXXXX1*/	"SYSMMU_S0_HSI0",	0x1},
	{"HSI0",	0x0,	/*XXXXX0*/	"USB32DRD",	0x1},
	{"HSI1",	0x1,	/*XXXXX1*/	"SYSMMU_S0_HSI1",	0x1},
	{"HSI1",	0x0,	/*XXXXX0*/	"PCIE_GEN3_0",	0x1},
	{"CS",		0x2,	/*XXXX10*/	"SYSMMU_S0_CPUCL0",	0x3},
	{"CS",		0x0,	/*XXXX00*/	"CORESIGHT (AXI-M1)",	0x3},
	{"CS",		0x1,	/*XXXX01*/	"CORESIGHT (AXI-M0)",	0x3},
	{"AOC",		0x1,	/*XXXXX1*/	"SYSMMU_S0_AOC",	0x1},
	{"AOC",		0x0,	/*XXXXX0*/	"AOC",	0x1},
	{"DPUF0",	0x1,	/*XXXXX1*/	"DPUF1_DMA_TOP (M0)",	0x1},
	{"DPUF0",	0x0,	/*XXXXX0*/	"DPUF0_DMA_TOP (M0)",	0x1},
	{"DPUF1",	0x0,	/*XXXX00*/	"DPUF0_DMA_TOP (M1)",	0x3},
	{"DPUF1",	0x1,	/*XXXX01*/	"SYSMMU_S0_DPUF0",	0x3},
	{"DPUF1",	0x2,	/*XXX010*/	"DPUF1_DMA_TOP (M1)",	0x7},
	{"DPUF1",	0x6,	/*XXX110*/	"SYSMMU_S0_DPUF1",	0x7},
	{"HSI2",	0x0,	/*XXXXX0*/	"SYSMMU_S0_HSI2",	0x1},
	{"HSI2",	0x1,	/*XXX001*/	"PCIE_GEN3A_1",	0x7},
	{"HSI2",	0x3,	/*XXX011*/	"PCIE_GEN3B_1",	0x7},
	{"HSI2",	0x5,	/*XXX101*/	"UFS_EMBD",	0x7},
	{"HSI2",	0x7,	/*XXX111*/	"MMC_CARD",	0x7},
	{"ISPFE0",	0x0,	/*XXXXXX*/	"ISPFE (M0)",	0x0},
	{"ISPFE1",	0x0,	/*XXXXX0*/	"ISPFE (M1)",	0x1},
	{"ISPFE1",	0x1,	/*XXXXX1*/	"SYSMMU_S0_ISPFE",	0x1},
	{"ISPFE2",	0x0,	/*XXXXX0*/	"ISPFE (M2)",	0x1},
	{"ISPFE2",	0x1,	/*XXXXX1*/	"SYSMMU_S1_ISPFE",	0x1},
	{"ISPFE3",	0x0,	/*XXXXX0*/	"ISPFE (M3)",	0x1},
	{"ISPFE3",	0x1,	/*XXXXX1*/	"SYSMMU_S2_ISPFE",	0x1},
	{"MFC0",	0x0,	/*XXXXXX*/	"MFC (M0)",	0x0},
	{"MFC1",	0x0,	/*XXXXX0*/	"MFC (M1)",	0x1},
	{"MFC1",	0x1,	/*XXXXX1*/	"SYSMMU_S0_MFC",	0x1},
	{"RGBP0",	0x0,	/*XXXXX0*/	"RGBP (M2)",	0x1},
	{"RGBP0",	0x1,	/*XXXXX1*/	"RGBP (M4)",	0x1},
	{"RGBP1",	0x1,	/*XXXXX1*/	"SYSMMU_S0_RGBP",	0x1},
	{"RGBP1",	0x0,	/*XXX000*/	"RGBP (M1)",	0x7},
	{"RGBP1",	0x2,	/*XXX010*/	"RGBP (M3)",	0x7},
	{"RGBP1",	0x4,	/*XXX100*/	"RGBP (M5)",	0x7},
	{"RGBP1",	0x6,	/*XXX110*/	"RGBP (M7)",	0x7},
	{"RGBP2",	0x0,	/*XXXXXX*/	"MCFP (M0/1)",	0x0},
	{"RGBP3",	0x0,	/*XXXXXX*/	"MCFP (M2)",	0x0},
	{"RGBP4",	0x0,	/*XXXXXX*/	"MCFP (M3)",	0x0},
	{"RGBP5",	0x0,	/*XXXX00*/	"MCFP (M4)",	0x3},
	{"RGBP5",	0x1,	/*XXXX01*/	"MCFP (M5)",	0x3},
	{"RGBP5",	0x2,	/*XXXX10*/	"MCFP (M11)",	0x3},
	{"RGBP6",	0x1,	/*XXXXX1*/	"SYSMMU_S1_RGBP",	0x1},
	{"RGBP6",	0x0,	/*XX0000*/	"MCFP (M6)",	0xF},
	{"RGBP6",	0x2,	/*XX0010*/	"MCFP (M7)",	0xF},
	{"RGBP6",	0x4,	/*XX0100*/	"MCFP (M8)",	0xF},
	{"RGBP6",	0x6,	/*XX0110*/	"MCFP (M9)",	0xF},
	{"RGBP6",	0x8,	/*XX1000*/	"MCFP (M10)",	0xF},
	{"GDC0",	0x0,	/*XXXX00*/	"GDC0 (M0)",	0x3},
	{"GDC0",	0x1,	/*XXXX01*/	"GDC0 (M2)",	0x3},
	{"GDC0",	0x2,	/*XXXX10*/	"GDC0 (M4)",	0x3},
	{"GDC1",	0x1,	/*XXXXX1*/	"SYSMMU_S0_GDC",	0x1},
	{"GDC1",	0x0,	/*XXX000*/	"GDC1 (M0)",	0x7},
	{"GDC1",	0x2,	/*XXX010*/	"GDC1 (M2)",	0x7},
	{"GDC1",	0x4,	/*XXX100*/	"GDC1 (M4)",	0x7},
	{"GDC1",	0x6,	/*XXX110*/	"RGBP (M0)",	0x7},
	{"GDC2",	0x0,	/*XXXXXX*/	"LME",	0x0},
	{"GSE",		0x1,	/*XXXXX1*/	"SYSMMU_S0_GSE",	0x1},
	{"GSE",		0x0,	/*XXX000*/	"GSE (M0)",	0x7},
	{"GSE",		0x2,	/*XXX010*/	"GSE (M1)",	0x7},
	{"GSE",		0x4,	/*XXX100*/	"GSE (M2)",	0x7},
	{"G2D0",	0x0,	/*XXXXXX*/	"SYSMMU_TNR0 (S2)",	0x0},
	{"G2D1",	0x0,	/*XXXXXX*/	"SYSMMU_TNR0 (S1)",	0x0},
	{"G2D2",	0x1,	/*XXXXX1*/	"SYSMMU_S0_G2D",	0x1},
	{"G2D2",	0x0,	/*XXXXX0*/	"JPEG",	0x1},
	{"MCSC0",	0x1,	/*XXXX01*/	"MCSC (M0)",	0x3},
	{"MCSC0",	0x2,	/*XXXX10*/	"MCSC (M2)",	0x3},
	{"MCSC0",	0x0,	/*XXXX00*/	"MCSC (M3)",	0x3},
	{"MCSC1",	0x1,	/*XXXXX1*/	"SYSMMU_S0_MCSC",	0x1},
	{"MCSC1",	0x0,	/*XXX000*/	"MCSC (M1)",	0x7},
	{"MCSC1",	0x2,	/*XXX010*/	"MCSC (M4)",	0x7},
	{"MCSC1",	0x4,	/*XXX100*/	"MCSC (M5)",	0x7},
	{"MCSC1",	0x6,	/*XXX110*/	"MCSC (M6)",	0x7},
	{"MISC",	0x1,	/*XXXXX1*/	"SYSMMU_S0_MISC",	0x1},
	{"MISC",	0x0,	/*XX0000*/	"SC",	0xF},
	{"MISC",	0x2,	/*XX0010*/	"RTIC",	0xF},
	{"MISC",	0x4,	/*XX0100*/	"SPDMA0",	0xF},
	{"MISC",	0x6,	/*XX0110*/	"PDMA0",	0xF},
	{"MISC",	0x8,	/*XX1000*/	"DIT",	0xF},
	{"MISC",	0xA,	/*XX1010*/	"SPDMA1",	0xF},
	{"MISC",	0xC,	/*XX1100*/	"PDMA1",	0xF},
	{"TNR0",	0x0,	/*XXXXX0*/	"GTNR_MERGE (M0)",	0x1},
	{"TNR0",	0x1,	/*XXXXX1*/	"GTNR_MERGE (M1)",	0x1},
	{"TNR1",	0x1,	/*XXXXX1*/	"SYSMMU_S0_TNR",	0x1},
	{"TNR1",	0x0,	/*XXXX00*/	"GTNR_MERGE (M2)",	0x3},
	{"TNR1",	0x2,	/*XXXX10*/	"GTNR_MERGE (M8)",	0x3},
	{"TNR2",	0x1,	/*XXXXX1*/	"SYSMMU_S1_TNR",	0x1},
	{"TNR2",	0x0,	/*XXXX00*/	"GTNR_MERGE (M3)",	0x3},
	{"TNR2",	0x2,	/*XXXX10*/	"GTNR_MERGE (M9)",	0x3},
	{"TMR3",	0x0,	/*XXXXX0*/	"GTNR_MERGE (M4)",	0x1},
	{"TMR3",	0x1,	/*XXXXX1*/	"GTNR_MERGE (M5)",	0x1},
	{"TNR4",	0x0,	/*XXXXX0*/	"GTNR_MERGE (M6)",	0x1},
	{"TNR4",	0x1,	/*XXXXX1*/	"GTNR_MERGE (M7)",	0x1},
	{"TMR5",	0x1,	/*XXXXX1*/	"SYSMMU_S2_TNR",	0x1},
	{"TMR5",	0x0,	/*XXXX00*/	"GTNR_ALIGN_M0",	0x3},
	{"TMR5",	0x2,	/*XXXX10*/	"GTNR_ALIGN_M1",	0x3},
	{"YUVP",	0x1,	/*XXXXX1*/	"SYSMMU_S0_YUVP",	0x1},
	{"YUVP",	0x0,	/*XXX000*/	"YUVP (M0)",	0x7},
	{"YUVP",	0x2,	/*XXX010*/	"YUVP (M1)",	0x7},
	{"YUVP",	0x4,	/*XXX100*/	"YUVP (M4)",	0x7},
};

static struct itmon_nodeinfo nocl1a_d[] = {
	{"AUR0",	M_NODE,  0, 1, 1, 0, 0, 0, 0, 0,   4,	NOT_SUPPORT,  0, NULL},
	{"AUR1",	M_NODE,  1, 1, 1, 0, 0, 0, 0, 0,   5,	NOT_SUPPORT,  1, NULL},
	{"BW",		M_NODE,  2, 1, 1, 0, 0, 0, 0, 0,   6,	NOT_SUPPORT,  2, NULL},
	{"G3DMMU",	M_NODE,  3, 1, 1, 0, 0, 0, 0, 0,   7,	NOT_SUPPORT,  3, NULL},
	{"G3D0",	M_NODE,  4, 1, 1, 0, 0, 0, 0, 0,   8,	NOT_SUPPORT,  4, NULL},
	{"G3D1",	M_NODE,  5, 1, 1, 0, 0, 0, 0, 0,   9,	NOT_SUPPORT,  5, NULL},
	{"G3D2",	M_NODE,  6, 1, 1, 0, 0, 0, 0, 0,  10,	NOT_SUPPORT,  6, NULL},
	{"G3D3",	M_NODE,  7, 1, 1, 0, 0, 0, 0, 0,  11,	NOT_SUPPORT,  7, NULL},
	{"TPU0",	M_NODE,  8, 1, 1, 0, 0, 0, 0, 0,  12,	NOT_SUPPORT,  8, NULL},
	{"TPU1",	M_NODE,  9, 1, 1, 0, 0, 0, 0, 0,  13,	NOT_SUPPORT,  9, NULL},
	{"NOCL1A_S0",	M_NODE, 10, 1, 1, 0, 0, 0, 0, 0,  69,	NOT_SUPPORT, 10, NULL},
	{"NOCL1A_S1",	M_NODE, 11, 1, 1, 0, 0, 0, 0, 0,  70,	NOT_SUPPORT, 11, NULL},
	{"NOCL1A_S2",	M_NODE, 12, 1, 1, 0, 0, 0, 0, 0,  71,	NOT_SUPPORT, 12, NULL},
	{"NOCL1A_S3",	M_NODE, 13, 1, 1, 0, 0, 0, 0, 0,  72,	NOT_SUPPORT, 13, NULL},
	{"NOCL1A_M0",	S_NODE, 14, 1, 1, 0, 0, 0, 0, 0, 193,	NOT_SUPPORT,  0, NULL},
	{"NOCL1A_M1",	S_NODE, 15, 1, 1, 0, 0, 0, 0, 0, 194,	NOT_SUPPORT,  1, NULL},
	{"NOCL1A_M2",	S_NODE, 16, 1, 1, 0, 0, 0, 0, 0, 195,	NOT_SUPPORT,  2, NULL},
	{"NOCL1A_M3",	S_NODE, 17, 1, 1, 0, 0, 0, 0, 0, 196,	NOT_SUPPORT,  3, NULL},
};

static struct itmon_nodeinfo nocl1b_d[] = {
	{"ALIVE",	M_NODE, 0, 1, 1, 0, 0, 0, 0, 0,   47,	NOT_SUPPORT,  0, NULL},
	{"AOC",		M_NODE, 1, 1, 1, 0, 0, 0, 0, 0,   48,	NOT_SUPPORT,  1, NULL},
	{"CSSYS",	M_NODE, 2, 1, 1, 0, 0, 0, 0, 0,   49,	NOT_SUPPORT,  2, NULL},
	{"GSA",		M_NODE, 3, 1, 1, 0, 0, 0, 0, 0,   50,	NOT_SUPPORT,  3, NULL},
	{"HSI0",	M_NODE, 4, 1, 1, 0, 0, 0, 0, 0,   51,	NOT_SUPPORT,  4, NULL},
	{"HSI1",	M_NODE, 5, 1, 1, 0, 0, 0, 0, 0,   52,	NOT_SUPPORT,  5, NULL},
	{"NOCL1B_M0",	S_NODE, 6, 1, 1, 0, 0, 0, 0, 0,  192,	NOT_SUPPORT,  0, NULL},
};

static struct itmon_nodeinfo nocl2aa_d[] = {
	{"DPUF0",	M_NODE,  0, 1, 1, 0, 0, 0, 0, 0,  14,	NOT_SUPPORT,  0, NULL},
	{"DPUF1",	M_NODE,  1, 1, 1, 0, 0, 0, 0, 0,  15,	NOT_SUPPORT,  1, NULL},
	{"HSI2",	M_NODE,  2, 1, 1, 0, 0, 0, 0, 0,  16,	NOT_SUPPORT,  2, NULL},
	{"ISPFE0",	M_NODE,  3, 1, 1, 0, 0, 0, 0, 0,  17,	NOT_SUPPORT,  3, NULL},
	{"ISPFE1",	M_NODE,  4, 1, 1, 0, 0, 0, 0, 0,  18,	NOT_SUPPORT,  4, NULL},
	{"ISPFE2",	M_NODE,  5, 1, 1, 0, 0, 0, 0, 0,  19,	NOT_SUPPORT,  5, NULL},
	{"ISPFE3",	M_NODE,  6, 1, 1, 0, 0, 0, 0, 0,  20,	NOT_SUPPORT,  6, NULL},
	{"MFC0",	M_NODE,  7, 1, 1, 0, 0, 0, 0, 0,  21,	NOT_SUPPORT,  7, NULL},
	{"MFC1",	M_NODE,  8, 1, 1, 0, 0, 0, 0, 0,  22,	NOT_SUPPORT,  8, NULL},
	{"RGBP0",	M_NODE,  9, 1, 1, 0, 0, 0, 0, 0,  23,	NOT_SUPPORT,  9, NULL},
	{"RGBP1",	M_NODE, 10, 1, 1, 0, 0, 0, 0, 0,  24,	NOT_SUPPORT, 10, NULL},
	{"RGBP2",	M_NODE, 11, 1, 1, 0, 0, 0, 0, 0,  25,	NOT_SUPPORT, 11, NULL},
	{"RGBP3",	M_NODE, 12, 1, 1, 0, 0, 0, 0, 0,  26,	NOT_SUPPORT, 12, NULL},
	{"RGBP4",	M_NODE, 13, 1, 1, 0, 0, 0, 0, 0,  27,	NOT_SUPPORT, 13, NULL},
	{"RGBP5",	M_NODE, 14, 1, 1, 0, 0, 0, 0, 0,  28,	NOT_SUPPORT, 14, NULL},
	{"RGBP6",	M_NODE, 15, 1, 1, 0, 0, 0, 0, 0,  29,	NOT_SUPPORT, 15, NULL},
	{"NOCL2AA_M0",	S_NODE, 16, 1, 1, 0, 0, 0, 0, 0, 197,	NOT_SUPPORT,  0, NULL},
	{"NOCL2AA_M1",	S_NODE, 17, 1, 1, 0, 0, 0, 0, 0, 198,	NOT_SUPPORT,  1, NULL},
};

static struct itmon_nodeinfo nocl2ab_d[] = {
	{"GDC0",	M_NODE,  0, 1, 1, 0, 0, 0, 0, 0,  30,	NOT_SUPPORT,  0, NULL},
	{"GDC1",	M_NODE,  1, 1, 1, 0, 0, 0, 0, 0,  31,	NOT_SUPPORT,  1, NULL},
	{"GDC2",	M_NODE,  2, 1, 1, 0, 0, 0, 0, 0,  32,	NOT_SUPPORT,  2, NULL},
	{"GSE",		M_NODE,  3, 1, 1, 0, 0, 0, 0, 0,  33,	NOT_SUPPORT,  3, NULL},
	{"G2D0",	M_NODE,  4, 1, 1, 0, 0, 0, 0, 0,  34,	NOT_SUPPORT,  4, NULL},
	{"G2D1",	M_NODE,  5, 1, 1, 0, 0, 0, 0, 0,  35,	NOT_SUPPORT,  5, NULL},
	{"G2D2",	M_NODE,  6, 1, 1, 0, 0, 0, 0, 0,  36,	NOT_SUPPORT,  6, NULL},
	{"MCSC0",	M_NODE,  7, 1, 1, 0, 0, 0, 0, 0,  37,	NOT_SUPPORT,  7, NULL},
	{"MCSC1",	M_NODE,  8, 1, 1, 0, 0, 0, 0, 0,  38,	NOT_SUPPORT,  8, NULL},
	{"MISC",	M_NODE,  9, 1, 1, 0, 0, 0, 0, 0,  39,	NOT_SUPPORT,  9, NULL},
	{"TNR0",	M_NODE, 10, 1, 1, 0, 0, 0, 0, 0,  40,	NOT_SUPPORT, 10, NULL},
	{"TNR1",	M_NODE, 11, 1, 1, 0, 0, 0, 0, 0,  41,	NOT_SUPPORT, 11, NULL},
	{"TNR2",	M_NODE, 12, 1, 1, 0, 0, 0, 0, 0,  42,	NOT_SUPPORT, 12, NULL},
	{"TNR3",	M_NODE, 13, 1, 1, 0, 0, 0, 0, 0,  43,	NOT_SUPPORT, 13, NULL},
	{"TNR4",	M_NODE, 14, 1, 1, 0, 0, 0, 0, 0,  44,	NOT_SUPPORT, 14, NULL},
	{"TNR5",	M_NODE, 15, 1, 1, 0, 0, 0, 0, 0,  45,	NOT_SUPPORT, 15, NULL},
	{"YUVP",	M_NODE, 16, 1, 1, 0, 0, 0, 0, 0,  46,	NOT_SUPPORT, 16, NULL},
	{"NOCL2AB_M0",	S_NODE, 17, 1, 1, 0, 0, 0, 0, 0, 199,	NOT_SUPPORT,  0, NULL},
	{"NOCL2AB_M1",	S_NODE, 18, 1, 1, 0, 0, 0, 0, 0, 200,	NOT_SUPPORT,  1, NULL},
};

static struct itmon_nodeinfo nocl0_d[] = {
	{"CPU0",	M_NODE,  0, 1, 1, 0, 0, 0, 0, 0,   0,	NOT_SUPPORT,  0, NULL},
	{"CPU1",	M_NODE,  1, 1, 1, 0, 0, 0, 0, 0,   1,	NOT_SUPPORT,  1, NULL},
	{"CPU2",	M_NODE,  2, 1, 1, 0, 0, 0, 0, 0,   2,	NOT_SUPPORT,  2, NULL},
	{"CPU3",	M_NODE,  3, 1, 1, 0, 0, 0, 0, 0,   3,	NOT_SUPPORT,  3, NULL},
	{"NOCL0_S0",	M_NODE,  4, 1, 1, 0, 0, 0, 0, 0,  65,	NOT_SUPPORT,  4, NULL},
	{"NOCL0_S1",	M_NODE,  5, 1, 1, 0, 0, 0, 0, 0,  66,	NOT_SUPPORT,  5, NULL},
	{"NOCL0_S2",	M_NODE,  6, 1, 1, 0, 0, 0, 0, 0,  67,	NOT_SUPPORT,  6, NULL},
	{"NOCL0_S3",	M_NODE,  7, 1, 1, 0, 0, 0, 0, 0,  68,	NOT_SUPPORT,  7, NULL},
	{"NOCL0_S4",	M_NODE,  8, 1, 1, 0, 0, 0, 0, 0,  64,	NOT_SUPPORT,  8, NULL},
	{"NOCL0_IO0",	S_NODE,  9, 1, 1, 0, 0, 0, 0, 0, 128,	NOT_SUPPORT,  0, NULL},
	{"NOCL0_IO1",	S_NODE, 10, 1, 1, 0, 0, 0, 0, 0, 129,	NOT_SUPPORT,  1, NULL},
	{"NOCL0_DP",	S_NODE, 11, 1, 1, 0, 0, 0, 0, 0, 130,	0,  2, NULL},
	{"NOCL0_M0",	S_NODE, 12, 1, 1, 0, 0, 0, 0, 0, 131,	1,  3, NULL},
	{"NOCL0_M1",	S_NODE, 13, 1, 1, 0, 0, 0, 0, 0, 132,	2,  4, NULL},
	{"NOCL0_M2",	S_NODE, 14, 1, 1, 0, 0, 0, 0, 0, 133,	3,  5, NULL},
	{"NOCL0_M3",	S_NODE, 15, 1, 1, 0, 0, 0, 0, 0, 134,	4,  6, NULL},
};

static struct itmon_nodeinfo nocl1a_p[] = {
	{"P_NOCL1A_S0",	M_NODE,  0, 1, 1, 0, 0, 0, 0, 0,  64,	NOT_SUPPORT,  0, NULL},
	{"AUR",		S_NODE,  1, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 140,	0,  0, NULL},
	{"BW",		S_NODE,  2, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 141,	1,  1, NULL},
	{"G3D",		S_NODE,  3, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 142,	2,  2, NULL},
	{"TPU",		S_NODE,  4, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 143,	3,  3, NULL},
	{"P_NOCL1A_M0",	S_NODE,  5, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 144,	4,  4, NULL},
};

static struct itmon_nodeinfo nocl1b_p[] = {
	{"P_NOCL1B_S0",	M_NODE,  0, 1, 1, 0, 0, 0, 0, 0,  65,	NOT_SUPPORT,  0, NULL},
	{"AOC",		S_NODE,  1, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 145,	0,  0, NULL},
	{"GSA",		S_NODE,  2, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 146,	1,  1, NULL},
	{"HSI0",	S_NODE,  3, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 147,	2,  2, NULL},
	{"HSI1",	S_NODE,  4, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 148,	3,  3, NULL},
	{"P_NOCL1B_M0",	S_NODE,  5, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 149,	4,  4, NULL},
};

static struct itmon_nodeinfo nocl2aa_p[] = {
	{"P_NOCL2AA_S0", M_NODE, 0, 1, 1, 0, 0, 0, 0, 0,  66,	NOT_SUPPORT,  0, NULL},
	{"DPUB",	S_NODE,  1, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 150,	0,  0, NULL},
	{"DPUF0",	S_NODE,  2, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 151,	1,  1, NULL},
	{"DPUF1",	S_NODE,  3, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 152,	2,  2, NULL},
	{"HSI2",	S_NODE,  4, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 165,	3,  3, NULL},
	{"ISPFE",	S_NODE,  5, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 153,	4,  4, NULL},
	{"MFC",		S_NODE,  6, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 154,	5,  5, NULL},
	{"RGBP",	S_NODE,  7, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 155,	6,  6, NULL},
	{"P_NOCL2AA_M0", S_NODE, 8, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 156,	7,  7, NULL},
};

static struct itmon_nodeinfo nocl2ab_p[] = {
	{"P_NOCL2AB_S0", M_NODE, 0, 1, 1, 0, 0, 0, 0, 0,  67,	NOT_SUPPORT,  0, NULL},
	{"GDC",		S_NODE,  1, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 157,	0,  0, NULL},
	{"GSE",		S_NODE,  2, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 158,	1,  1, NULL},
	{"G2D",		S_NODE,  3, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 159,	2,  2, NULL},
	{"MCSC",	S_NODE,  4, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 160,	3,  3, NULL},
	{"TNR",		S_NODE,  5, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 161,	4,  4, NULL},
	{"YUVP",	S_NODE,  6, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 162,	5,  5, NULL},
	{"P_NOCL2AB_M0", S_NODE, 7, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 163,	6,  6, NULL},
};

static struct itmon_nodeinfo nocl0_p[] = {
	{"BOOKER",	M_NODE,  0, 1, 1, 0, 0, 0, 0, 0,   0,	NOT_SUPPORT,  0, NULL},
	{"CSSYS",	M_NODE,  1, 1, 1, 0, 0, 0, 0, 0,   1,	NOT_SUPPORT,  1, NULL},
	{"NOCL0_DP",	M_NODE,  2, 1, 1, 0, 0, 0, 0, 0,   2,	NOT_SUPPORT,  2, NULL},
	{"ALIVE",	S_NODE,  3, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 164,	0,  0, NULL},
	{"CPUCL0",	S_NODE,  4, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 128,	1,  1, NULL},
	{"EH",		S_NODE,  5, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 129,	2,  2, NULL},
	{"GIC",		S_NODE,  6, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 130,	3,  3, NULL},
	{"MIF0",	S_NODE,  7, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 131,	4,  4, NULL},
	{"MIF1",	S_NODE,  8, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 132,	5,  5, NULL},
	{"MIF2",	S_NODE,  9, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 133,	6,  6, NULL},
	{"MIF3",	S_NODE, 10, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 134,	7,  7, NULL},
	{"MISC",	S_NODE, 11, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 135,	8,  8, NULL},
	{"PERIC0",	S_NODE, 12, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 136,	9,  9, NULL},
	{"PERIC1",	S_NODE, 13, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 137,	10, 10, NULL},
	{"P_NOCL0_M0",	S_NODE, 14, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 138,	11, 11, NULL},
	{"P_NOCL0_M1",	S_NODE, 15, 1, 1, 0, 0, 0, 0, 0, 192,	NOT_SUPPORT, 12, NULL},
	{"P_NOCL0_M2",	S_NODE, 16, 1, 1, 0, 0, 0, 0, 0, 193,	NOT_SUPPORT, 13, NULL},
	{"P_NOCL0_M3",	S_NODE, 17, 1, 1, 0, 0, 0, 0, 0, 194,	NOT_SUPPORT, 14, NULL},
	{"P_NOCL0_M4",	S_NODE, 18, 1, 1, 0, 0, 0, 0, 0, 195,	NOT_SUPPORT, 15, NULL},
	{"SLC",		S_NODE, 19, 1, 1, 0, 0, 1, 0, TMOUT_VAL, 139,	12, 16, NULL},
};

#define NMI_SRC_IN_NOCL0	BIT(1)
#define NMI_SRC_IN_NOCL1A	BIT(2)
#define NMI_SRC_IN_NOCL1B	BIT(3)
#define NMI_SRC_IN_NOCL2AA	BIT(4)
#define NMI_SRC_IN_NOCL2AB	BIT(5)

static struct itmon_nodegroup nodegroup[] = {
	{"NOCL1A_D",	0x26440000, 0, 0, nocl1a_d, ARRAY_SIZE(nocl1a_d),
		DATA, NULL, 0,	0, 0, "pd_nocl1a", NMI_SRC_IN_NOCL1A},
	{"NOCL1B_D",	0x26840000, 0, 0, nocl1b_d, ARRAY_SIZE(nocl1b_d),
		DATA, NULL, 0,	0, 0, "pd_nocl1b", NMI_SRC_IN_NOCL1B},
	{"NOCL2AA_D",	0x26C40000, 0, 0, nocl2aa_d, ARRAY_SIZE(nocl2aa_d),
		DATA, NULL, 0, 0, 0, "pd_nocl2aa", NMI_SRC_IN_NOCL2AA},
	{"NOCL2AB_D",	0x27040000, 0, 0, nocl2ab_d, ARRAY_SIZE(nocl2ab_d),
		DATA, NULL, 0, 0, 0, "pd_nocl2ab", NMI_SRC_IN_NOCL2AB},
	{"NOCL0_D",	0x26050000, 0, 0, nocl0_d, ARRAY_SIZE(nocl0_d),
		DATA, NULL, 0,	0, 0, "pd_nocl0", NMI_SRC_IN_NOCL0},
	{"NOCL1A_P",	0x26450000, 0, 0, nocl1a_p, ARRAY_SIZE(nocl1a_p),
		CONFIG, NULL, 0, 0, 0, "pd_nocl1a", NMI_SRC_IN_NOCL1A},
	{"NOCL1B_P",	0x26850000, 0, 0, nocl1b_p, ARRAY_SIZE(nocl1b_p),
		CONFIG, NULL, 0, 0, 0, "pd_nocl1b", NMI_SRC_IN_NOCL1B},
	{"NOCL2AA_P",	0x26C50000, 0, 0, nocl2aa_p, ARRAY_SIZE(nocl2aa_p),
		CONFIG, NULL, 0, 0, 0, "pd_nocl1c", NMI_SRC_IN_NOCL2AA},
	{"NOCL2AB_P",	0x27050000, 0, 0, nocl2ab_p, ARRAY_SIZE(nocl2ab_p),
		CONFIG, NULL, 0, 0, 0, "pd_nocl1c", NMI_SRC_IN_NOCL2AB},
	{"NOCL0_P",	0x26060000, 0, 0, nocl0_p, ARRAY_SIZE(nocl0_p),
		CONFIG, NULL, 0, 0, 0, "pd_nocl0", NMI_SRC_IN_NOCL0},
};

static const char *itmon_pathtype[2] = {
	"DATA Path transaction",
	"Configuration(SFR) Path transaction",
};

/* Error Code Description */
static const char *itmon_errcode[16] = {
	"Error Detect by the Target(SLVERR)",
	"Decode error(DECERR)",
	"Unsupported transaction error",
	"Power Down access error",
	"Inteded Access Violation",
	"Unknown error",
	"Timeout error - response timeout in timeout value",
	"Unknown error",
	"ARID-RID mismatch",
	"AWID-BID mismatch",
	"ARLEN-RLAST mismatch (Early Last)",
	"ARLEN-RLAST mismatch (No Last)",
	"AWLEN-WLAST mismatch (Early Last)",
	"AWLEN-WLAST mismatch (No Last)",
	"Unknown error",
	"Intended AccessM_node) or Timeout Freeze(S_node)",
};

static const char *itmon_node_string[4] = {
	"M_NODE",
	"TAXI_S_NODE",
	"TAXI_M_NODE",
	"S_NODE",
};

static const char *itmon_cpu_node_string[5] = {
	"BOOKER",
	"CPU0",
	"CPU1",
	"CPU2",
	"CPU3",
};

static const char *itmon_cpu_attr_type[4] = {
	"Inst",	/* Instruction */
	"Data",	/* Data */
	"PTW",	/* Page table walk */
	NO_NAME,
};

static const char *itmon_cpu_attr_accs[2] = {
	"Demand",
	"Prefetch",
};

static const unsigned int itmon_invalid_addr[2] = {
	0x03000000,
	0x04000000,
};

static struct itmon_dev *g_itmon;

/* declare notifier_list */
static ATOMIC_NOTIFIER_HEAD(itmon_notifier_list);


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

static const struct of_device_id itmon_dt_match[] = {
	{.compatible = "samsung,exynos-itmon-v2",
	 .data = NULL,},
	{},
};
MODULE_DEVICE_TABLE(of, itmon_dt_match);

struct itmon_nodeinfo *itmon_get_nodeinfo_by_group(struct itmon_dev *itmon,
						   struct itmon_nodegroup *group,
						   const char *name)
{
	struct itmon_nodeinfo *node = group->nodeinfo;
	int i;

	for (i = 0; i < group->nodesize; i++)
		if (!strncmp(node[i].name, name, strlen(name)))
			return &node[i];

	return NULL;
}

struct itmon_nodeinfo *itmon_get_nodeinfo(struct itmon_dev *itmon,
					  struct itmon_nodegroup *group,
					  const char *name)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *node = NULL;
	int i;

	if (!name)
		return NULL;

	if (group)
		return itmon_get_nodeinfo_by_group(itmon, group, name);

	for (i = 0; i < pdata->num_nodegroup; i++) {
		group = &pdata->nodegroup[i];
		node = itmon_get_nodeinfo_by_group(itmon, group, name);

		if (node)
			return node;
	}

	return NULL;
}

struct itmon_nodeinfo *itmon_get_nodeinfo_group_by_eid(struct itmon_dev *itmon,
						       struct itmon_nodegroup *group,
						       u32 err_id)
{
	struct itmon_nodeinfo *node = group->nodeinfo;
	int i;

	for (i = 0; i < group->nodesize; i++) {
		if (node[i].err_id == err_id)
			return &node[i];
	}

	return NULL;
}

struct itmon_nodeinfo *itmon_get_nodeinfo_by_eid(struct itmon_dev *itmon,
						 struct itmon_nodegroup *group,
						 int path_type, u32 err_id)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *node = NULL;
	int i;

	if (group)
		return itmon_get_nodeinfo_group_by_eid(itmon, group, err_id);

	for (i = 0; i < pdata->num_nodegroup; i++) {
		group = &pdata->nodegroup[i];
		if (group->path_type != path_type)
			continue;
		node = itmon_get_nodeinfo_group_by_eid(itmon, group, err_id);
		if (node)
			return node;
	}

	return NULL;
}

struct itmon_nodeinfo *itmon_get_nodeinfo_group_by_tmout_offset(struct itmon_dev *itmon,
								struct itmon_nodegroup *group,
								u32 tmout_offset)
{
	struct itmon_nodeinfo *node = group->nodeinfo;
	int i;

	for (i = 0; i < group->nodesize; i++) {
		if (node[i].tmout_offset == tmout_offset)
			return &node[i];
	}

	return NULL;
}

struct itmon_nodeinfo *itmon_get_nodeinfo_by_tmout_offset(struct itmon_dev *itmon,
							  struct itmon_nodegroup *group,
							  u32 tmout_offset)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *node = NULL;
	int i;

	if (group)
		return itmon_get_nodeinfo_group_by_tmout_offset(itmon, group, tmout_offset);

	for (i = 0; i < pdata->num_nodegroup; i++) {
		group = &pdata->nodegroup[i];
		node = itmon_get_nodeinfo_group_by_tmout_offset(itmon, group, tmout_offset);
		if (node)
			return node;
	}

	return NULL;
}

static const struct itmon_rpathinfo *itmon_get_rpathinfo(struct itmon_dev *itmon,
						   unsigned int id,
						   char *dest_name)
{
	struct itmon_platdata *pdata = itmon->pdata;
	int i;

	if (!dest_name)
		return NULL;

	for (i = 0; i < pdata->num_rpathinfo; i++) {
		if (pdata->rpathinfo[i].id == (id & pdata->rpathinfo[i].bits)) {
			if (!strncmp(pdata->rpathinfo[i].dest_name, dest_name,
				     strlen(pdata->rpathinfo[i].dest_name)))
				return &pdata->rpathinfo[i];
		}
	}

	return NULL;
}

static char *itmon_get_clientinfo(struct itmon_dev *itmon,
				  const char *port_name,
				  u32 user)
{
	struct itmon_platdata *pdata = itmon->pdata;
	int i;

	if (!port_name)
		return NULL;

	for (i = 0; i < pdata->num_clientinfo; i++) {
		if ((user & pdata->clientinfo[i].bits) == pdata->clientinfo[i].user) {
			if (!strncmp(pdata->clientinfo[i].port_name, port_name, strlen(port_name)))
				return pdata->clientinfo[i].client_name;
		}
	}

	return NULL;
}

static int itmon_parse_cpuinfo_by_name(struct itmon_dev *itmon,
				       const char *name,
				       unsigned int userbit,
				       char *cpu_name)
{
	struct itmon_platdata *pdata = itmon->pdata;
	int core_num = 0, el2 = 0, strong = 0, i;

	if (!cpu_name)
		return 0;

	for (i = 0; i < (int)ARRAY_SIZE(itmon_cpu_node_string); i++) {
		if (!strncmp(name, itmon_cpu_node_string[i],
			     strlen(itmon_cpu_node_string[i]))) {
			if (userbit & BIT(0))
				el2 = 1;
			if (nr_cpu_ids > 8) {
				if (!(userbit & BIT(1)))
					strong = 1;
				core_num = ((userbit & (0xF << 2)) >> 2);
			} else {
				core_num = ((userbit & (0x7 << 1)) >> 1);
				strong = 0;
			}
			if (pdata->cpu_parsing) {
				scnprintf(cpu_name, CPU_NAME_LENGTH - 1,
					  "CPU%d%s%s", core_num, el2 == 1 ? "EL2" : "",
					  strong == 1 ? "Strng" : "");
			} else {
				strscpy(cpu_name, "CPU", CPU_NAME_LENGTH);
			}
			return 1;
		}
	};

	return 0;
}

static void _itmon_report_timeout(struct itmon_dev *itmon,
				  struct itmon_nodegroup *group,
				  struct itmon_nodeinfo *node,
				  unsigned int trans_type,
				  unsigned int bit,
				  int num,
				  int rw_offset)
{
	unsigned int id, payload0, payload1, payload2, payload3, payload4, payload5;
	unsigned int axid, valid, timeout;
	unsigned long addr, user;
	u32 axburst, axprot, axlen, axsize, domain;
	const struct itmon_rpathinfo *port = NULL;
	char *client_name = NULL;
	char cpu_name[CPU_NAME_LENGTH] = {0, };

	/*
	 * PAYLOAD0 -> PAYLOAD_FROM_BUFFER
	 * PAYLOAD1 -> PAYLOAD_FROM_SRAM_1
	 * PAYLOAD2 -> PAYLOAD_FROM_SRAM_2
	 * PAYLOAD3 -> PAYLOAD_FROM_SRAM_3
	 * PAYLOAD4 -> USER_FROM_BUFFER_0
	 * PAYLOAD5 -> USER_FROM_BUFFER_1
	 */

	writel(num | (bit << 16), group->regs + OFFSET_TMOUT_REG + TMOUT_POINT_ADDR + rw_offset);
	id = readl(group->regs + OFFSET_TMOUT_REG + TMOUT_ID + rw_offset);
	payload0 = readl(group->regs + OFFSET_TMOUT_REG +
			 TMOUT_PAYLOAD_0 + rw_offset);
	payload1 = readl(group->regs + OFFSET_TMOUT_REG +
			 TMOUT_PAYLOAD_1 + rw_offset);
	payload2 = readl(group->regs + OFFSET_TMOUT_REG +
			 TMOUT_PAYLOAD_2 + rw_offset);
	payload3 = readl(group->regs + OFFSET_TMOUT_REG +
			 TMOUT_PAYLOAD_3 + rw_offset);
	payload4 = readl(group->regs + OFFSET_TMOUT_REG +
			 TMOUT_PAYLOAD_4 + rw_offset);
	payload5 = readl(group->regs + OFFSET_TMOUT_REG +
			 TMOUT_PAYLOAD_5 + rw_offset);

	timeout = (payload0 & (unsigned int)(GENMASK(7, 4))) >> 4;
	user = ((unsigned long)payload4 << 32ULL) | payload5;
	addr = (((unsigned long)payload1 & GENMASK(15, 0)) << 32ULL) | payload2;
	axid = id & GENMASK(5, 0); /* ID[5:0] 6bit : R-PATH */
	valid = payload0 & BIT(0); /* PAYLOAD[0] : Valid or Not valid */
	axburst = TMOUT_BIT_AXBURST(payload3);
	axprot = TMOUT_BIT_AXPROT(payload3);
	axlen = (payload3 & (unsigned int)GENMASK(7, 4)) >> 4;
	axsize = (payload3 & (unsigned int)GENMASK(18, 16)) >> 16;
	domain = (payload3 & BIT(19));

	if (group->path_type == CONFIG) {
		port = itmon_get_rpathinfo(itmon, axid, "CONFIGURATION");
		if (port)
			itmon_parse_cpuinfo_by_name(itmon, port->port_name, user, cpu_name);
		client_name = cpu_name;
	} else {
		port = itmon_get_rpathinfo(itmon, axid, node->name);
		if (port)
			client_name = itmon_get_clientinfo(itmon, port->port_name, user);
	}

	if (valid)
		log_dev_err(itmon->dev, "> %03d|%5s|%16s|%16s|%5u|%5x|%06x|%6s|%4llu|%4s|%016zx|%08x|%08x|%08x|%08x|%08x|%08x\n",
			    num, (trans_type == TRANS_TYPE_READ) ? "READ" : "WRITE",
			    port ? port->port_name : NO_NAME,
			    client_name ? client_name : NO_NAME,
			    valid, timeout, id, domain ? "SHARE" : "NSHARE",
			    int_pow(2, axsize * (axlen + 1)),
			    axprot & BIT(1) ? "NSEC" : "SEC",
			    addr, payload0, payload1, payload2, payload3, payload4, payload5);
}

static void itmon_report_timeout(struct itmon_dev *itmon,
				 struct itmon_nodegroup *group)
{
	int i, num = SZ_128;
	int rw_offset = 0;
	unsigned int trans_type;
	unsigned long tmout_frz_stat;
	unsigned int tmout_offset;
	struct itmon_nodeinfo *node;
	unsigned int bit = 0;

	if (group)
		printk("group %s frz_support %s\n",
			 group->name, group->frz_support ? "YES" : "NO");

	if (!group->frz_support)
		return;

	tmout_frz_stat = readl(group->regs + OFFSET_TMOUT_REG);

	if (tmout_frz_stat == 0x0)
		return;

	log_dev_err(itmon->dev, "TIMEOUT_FREEZE_STATUS 0x%08lx", tmout_frz_stat);

	for_each_set_bit(bit, &tmout_frz_stat, group->nodesize) {
		tmout_offset = (1 << bit) >> 1;
		node = itmon_get_nodeinfo_group_by_tmout_offset(itmon, group, tmout_offset);

		log_dev_err(itmon->dev, "\nITMON Report Timeout Error Occurred : Client --> %s\n\n",
			    node->name);
		log_dev_err(itmon->dev, "> NUM| TYPE|    CLIENT BLOCK|          CLIENT|VALID|TMOUT|    ID|DOMAIN|SIZE|PROT|         ADDRESS|PAYLOAD0|PAYLOAD1|PAYLOAD2|PAYLOAD3|PAYLOAD4|PAYLOAD5");

		for (trans_type = TRANS_TYPE_WRITE; trans_type < TRANS_TYPE_NUM; trans_type++) {
			num = (trans_type == TRANS_TYPE_READ ? SZ_128 : SZ_64);
			rw_offset = (trans_type == TRANS_TYPE_READ ? 0 : TMOUT_RW_OFFSET);

			for (i = 0; i < num; i++)
				_itmon_report_timeout(itmon, group, node,
						      trans_type, bit, i, rw_offset);
		}
	}
}

/* need to reconfiguable to the address */
static void itmon_en_addr_detect(struct itmon_dev *itmon,
				 struct itmon_nodegroup *group,
				 struct itmon_nodeinfo *node,
				 bool en)
{
	void __iomem *reg = group->regs;
	u32 tmp, val, offset, i;
	char *name;

	if (node) {
		if (node->type == M_NODE || node->type == T_M_NODE)
			offset = PRTCHK_M_CTL(node->prtchk_offset);
		else
			offset = PRTCHK_S_CTL(node->prtchk_offset);

		val = __raw_readl(reg + offset) | INTEND_ACCESS_INT_EN;
		__raw_writel(val, reg + offset);

		val = ((unsigned int)INTEND_ADDR_START & U32_MAX);
		__raw_writel(val, reg + PRTCHK_START_ADDR_LOW);

		val = (unsigned int)(((unsigned long)INTEND_ADDR_START >> 32) & U16_MAX);
		__raw_writel(val, reg + PRTCHK_START_END_ADDR_UPPER);

		val = ((unsigned int)INTEND_ADDR_END & 0xFFFFFFFF);
		__raw_writel(val, reg + PRTCHK_END_ADDR_LOW);
		val = ((unsigned int)(((unsigned long)INTEND_ADDR_END >> 32)
					& 0XFFFF0000) << 16);
		tmp = readl(reg + PRTCHK_START_END_ADDR_UPPER);
		__raw_writel(tmp | val, reg + PRTCHK_START_END_ADDR_UPPER);
		name = node->name;
	} else {
		node = group->nodeinfo;
		for (i = 0; i < group->nodesize; i++) {
			if (node[i].type == M_NODE || node[i].type == T_M_NODE)
				offset = PRTCHK_M_CTL(node[i].prtchk_offset);
			else
				offset = PRTCHK_S_CTL(node[i].prtchk_offset);

			val = readl(reg + offset) | INTEND_ACCESS_INT_EN;
			writel(val, reg + offset);

			val = ((unsigned int)INTEND_ADDR_START & U32_MAX);
			writel(val, reg + PRTCHK_START_ADDR_LOW);

			val = (unsigned int)(((unsigned long)INTEND_ADDR_START >> 32) & U16_MAX);
			writel(val, reg + PRTCHK_START_END_ADDR_UPPER);

			val = ((unsigned int)INTEND_ADDR_END & 0xFFFFFFFF);
			writel(val, reg + PRTCHK_END_ADDR_LOW);
			val = ((unsigned int)(((unsigned long)INTEND_ADDR_END >> 32)
						& 0XFFFF0000) << 16);
			tmp = readl(reg + PRTCHK_START_END_ADDR_UPPER);
			writel(tmp | val, reg + PRTCHK_START_END_ADDR_UPPER);
		}
		name = group->name;
	}
	log_dev_dbg(itmon->dev, "ITMON - %s addr detect %sabled\n", name, en ? "en" : "dis");
}

static void itmon_en_prt_chk(struct itmon_dev *itmon,
			     struct itmon_nodegroup *group,
			     struct itmon_nodeinfo *node,
			     bool en)
{
	void __iomem *reg = group->regs;
	u32 offset, val = 0, i;
	char *name;

	if (en)
		val = RD_RESP_INT_EN | WR_RESP_INT_EN |
		     ARLEN_RLAST_INT_EN | AWLEN_WLAST_INT_EN;

	if (node) {
		if (node->prtchk_offset == NOT_SUPPORT)
			return;
		if (node->type == M_NODE || node->type == T_M_NODE)
			offset = PRTCHK_M_CTL(node->prtchk_offset);
		else
			offset = PRTCHK_S_CTL(node->prtchk_offset);

		writel(val, reg + offset);
		name = node->name;
	} else {
		node = group->nodeinfo;
		for (i = 0; i < group->nodesize; i++) {
			if (node[i].prtchk_offset == NOT_SUPPORT)
				continue;
			if (node[i].type == M_NODE || node[i].type == T_M_NODE)
				offset = PRTCHK_M_CTL(node[i].prtchk_offset);
			else
				offset = PRTCHK_S_CTL(node[i].prtchk_offset);

			writel(val, reg + offset);
		}
		name = group->name;
	}
	log_dev_dbg(itmon->dev, "ITMON - %s hw_assert %sabled\n", name, en ? "en" : "dis");
}

static void itmon_en_err_report(struct itmon_dev *itmon,
				struct itmon_nodegroup *group,
				struct itmon_nodeinfo *node,
				bool en)
{
	struct itmon_platdata *pdata = itmon->pdata;
	void __iomem *reg = group->regs;
	u32 val = 0, i;
	char *name;

	if (!pdata->probed)
		writel(1, reg + ERR_LOG_CLR);

	val = readl(reg + ERR_LOG_EN_NODE);

	if (node) {
		if (en)
			val |= BIT(node->id);
		else
			val &= ~BIT(node->id);
		writel(val, reg + ERR_LOG_EN_NODE);
		name = node->name;
	} else {
		node = group->nodeinfo;
		for (i = 0; i < group->nodesize; i++) {
			if (en)
				val |= (1 << node[i].id);
			else
				val &= ~(1 << node[i].id);
		}
		writel(val, reg + ERR_LOG_EN_NODE);
		name = group->name;
	}
	log_dev_dbg(itmon->dev, "ITMON - %s error reporting %sabled\n", name, en ? "en" : "dis");
}

static void itmon_en_timeout(struct itmon_dev *itmon,
			     struct itmon_nodegroup *group,
			     struct itmon_nodeinfo *node,
			     bool en)
{
	void __iomem *reg = group->regs;
	u32 offset, val = 0, i;
	char *name;

	if (node) {
		if (node->tmout_offset == NOT_SUPPORT)
			return;
		offset = TMOUT_CTL(node->tmout_offset);

		/* Enabled */
		val = (en << 0);
		val |= (node->tmout_frz_en << 1);
		val |= (node->time_val << 4);

		writel(val, reg + offset);
		name = node->name;
	} else {
		node = group->nodeinfo;
		for (i = 0; i < group->nodesize; i++) {
			if (node[i].type != S_NODE || node[i].tmout_offset == NOT_SUPPORT)
				continue;
			offset = TMOUT_CTL(node[i].tmout_offset);

			/* en */
			val = (en << 0);
			val |= (node[i].tmout_frz_en << 1);
			val |= (node[i].time_val << 4);

			writel(val, reg + offset);
		}
		name = group->name;
	}
	log_dev_dbg(itmon->dev, "ITMON - %s timeout %sabled\n", name, en ? "en" : "dis");
}

static void itmon_enable_nodepolicy(struct itmon_dev *itmon,
				    struct itmon_nodeinfo *node)
{
	struct itmon_nodepolicy *policy = &node->policy;
	struct itmon_nodegroup *group = node->group;

	if (!policy->chk_set)
		return;

	if (group->pd_support && !group->pd_status) {
		dev_err(g_itmon->dev, "%p group - %p node NOT pd on\n",
			group, node);
	}

	if (policy->chk_errrpt)
		itmon_en_err_report(itmon, group, node, policy->en_errrpt);
	if (policy->chk_tmout || policy->chk_tmout_val)
		itmon_en_timeout(itmon, group, node, policy->en_tmout);
	if (policy->chk_prtchk)
		itmon_en_prt_chk(itmon, group, node, policy->en_prtchk);
}

static void itmon_set_nodepolicy(struct itmon_dev *itmon,
				 struct itmon_nodeinfo *node,
				 struct itmon_nodepolicy policy,
				 bool now)
{
	node->policy.en = policy.en;

	if (policy.chk_errrpt)
		node->err_en = policy.en_errrpt;
	if (policy.chk_tmout)
		node->tmout_en = policy.en_tmout;
	if (policy.chk_prtchk)
		node->prt_chk_en = policy.en_prtchk;
	if (policy.chk_tmout_val)
		node->time_val = policy.en_tmout_val;
	if (policy.chk_freeze)
		node->tmout_frz_en = policy.en_freeze;

	if (policy.chk_errrpt || policy.chk_tmout ||
	    policy.chk_prtchk || policy.chk_tmout_val ||
	    policy.chk_errrpt || policy.chk_prtchk_job ||
	    policy.chk_tmout || policy.chk_freeze ||
	    policy.chk_decerr_job || policy.chk_slverr_job ||
	    policy.chk_tmout_job)
		node->policy.chk_set = 1;

	if (now)
		itmon_enable_nodepolicy(itmon, node);
}

int itmon_en_by_name(const char *name, bool en)
{
	struct itmon_nodeinfo *node;
	struct itmon_nodegroup *group;

	if (!g_itmon)
		return -ENODEV;

	node = itmon_get_nodeinfo(g_itmon, NULL, name);
	if (!node) {
		log_dev_err(g_itmon->dev, "%s node is not found\n", name);
		return -ENODEV;
	}

	group = node->group;
	if (!group) {
		log_dev_err(g_itmon->dev, "%s node's group is  not found\n", name);
		return -ENODEV;
	}

	if (group->pd_support && !group->pd_status) {
		log_dev_err(g_itmon->dev, "%p group - %p node NOT pd on\n",
			group, node);
		return -EIO;
	}

	itmon_en_err_report(g_itmon, group, node, en);
	itmon_en_prt_chk(g_itmon, group, node, en);
	if (node->type == S_NODE)
		itmon_en_timeout(g_itmon, group, node, en);

	return 0;
}
EXPORT_SYMBOL(itmon_en_by_name);

static void itmon_en_global(struct itmon_dev *itmon,
			    struct itmon_nodegroup *group,
			    int int_en,
			    int err_log_en,
			    int tmout_en,
			    int prtchk_en,
			    int fixed_det_en)
{
	u32 val = (int_en) | (err_log_en << 1) | (tmout_en << 2) |
		(prtchk_en << 3) | (fixed_det_en << 5);

	writel(val, group->regs + DBG_CTL);
}

static void itmon_init_by_group(struct itmon_dev *itmon,
				struct itmon_nodegroup *group,
				bool en)
{
	struct itmon_nodeinfo *node;
	int i;

	node = group->nodeinfo;
	for (i = 0; i < group->nodesize; i++) {
		if (en) {
			if (node[i].err_en)
				itmon_en_err_report(itmon, group, &node[i], en);
			if (node[i].prt_chk_en)
				itmon_en_prt_chk(itmon, group, &node[i], en);
		} else {
			/* as default en */
			itmon_en_err_report(itmon, group, &node[i], en);
			itmon_en_prt_chk(itmon, group, &node[i], en);
		}
		if (node[i].type == S_NODE && node[i].tmout_en)
			itmon_en_timeout(itmon, group, &node[i], en);
		if (node[i].addr_detect_en && node[i].prtchk_offset != NOT_SUPPORT)
			itmon_en_addr_detect(itmon, group, &node[i], en);
	}
}

static void itmon_init(struct itmon_dev *itmon, bool en)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodegroup *group;
	int i;

	for (i = 0; i < pdata->num_nodegroup; i++) {
		group = &pdata->nodegroup[i];
		if (group->pd_support && !group->pd_status)
			continue;
		itmon_en_global(itmon, group, en, en, en, en, en);
		if (pdata->def_en)
			itmon_init_by_group(itmon, group, en);
	}

	pdata->en = en;
	log_dev_info(itmon->dev, "itmon %sabled\n", pdata->en ? "en" : "dis");
}

void itmon_pd_sync(const char *pd_name, bool en)
{
	struct itmon_dev *itmon = g_itmon;
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodegroup *group;
	int i;

	if (!itmon || !pdata->probed)
		return;

	for (i = 0; i < pdata->num_nodegroup; i++) {
		group = &pdata->nodegroup[i];
		if (group->pd_support && !strncmp(pd_name, group->pd_name, strlen(pd_name))) {
			log_dev_dbg(itmon->dev, "%s: pd_name:%s enabled:%x, pd_status:%x\n",
				    __func__, pd_name, en, group->pd_status);
			if (group->pd_status != en) {
				if (en) {
					itmon_en_global(itmon, group, en, en, en, en, en);
					if (!pdata->def_en)
						itmon_init_by_group(itmon, group, en);
				}
				group->pd_status = en;
			}
		}
	}
}
EXPORT_SYMBOL(itmon_pd_sync);

void itmon_en(bool en)
{
	if (g_itmon)
		itmon_init(g_itmon, en);
}
EXPORT_SYMBOL(itmon_en);

int itmon_get_dpm_policy(struct itmon_dev *itmon)
{
	struct itmon_platdata *pdata = itmon->pdata;
	int i, policy = -1;

	for (i = 0; i < TYPE_MAX; i++) {
		if (pdata->policy[i].error && pdata->policy[i].policy > policy)
			policy = pdata->policy[i].policy;
	}

	return policy;
}

static void itmon_dump_dpm_policy(struct itmon_dev *itmon)
{
	struct itmon_platdata *pdata = itmon->pdata;
	int i;

	log_dev_info(itmon->dev, "\t\tError Policy Information\n\n\t\t> NAME           |Error    |Policy-def |Policy-now\n");
	for (i = 0; i < TYPE_MAX; i++) {
		log_dev_info(itmon->dev, "\t\t> %15s%10s%10s%10s\n",
			     pdata->policy[i].name,
			     pdata->policy[i].error ? "TRUE" : "FALSE",
			     itmon_dpm_action[pdata->policy[i].policy_def],
			     itmon_dpm_action[pdata->policy[i].policy]);
	}
	log_dev_info(itmon->dev, "\n");
}

static void itmon_do_dpm_policy(struct itmon_dev *itmon, bool clear)
{
	struct itmon_platdata *pdata = itmon->pdata;
	int i, sel, policy = -1;
	char buf[SZ_64];

	for (i = 0; i < TYPE_MAX; i++) {
		if (pdata->policy[i].error && pdata->policy[i].policy > policy) {
			policy = pdata->policy[i].policy;
			sel = i;
		}
		if (clear) {
			pdata->policy[i].policy = pdata->policy[i].policy_def;
			pdata->policy[i].prio = 0;
		}
	}

	if (policy >= GO_DEFAULT_ID) {
		log_dev_err(itmon->dev, "%s: %s: policy:%s\n", __func__,
			    pdata->policy[sel].name, itmon_dpm_action[policy]);
		scnprintf(buf, sizeof(buf), "itmon triggering %s %s",
			  pdata->policy[sel].name, itmon->itmon_pattern);
		dbg_snapshot_do_dpm_policy(policy, buf);
	}
}

static void itmon_reflect_policy_by_notifier(struct itmon_dev *itmon,
					     struct itmon_traceinfo *info,
					     int ret)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_policy *pl_policy;
	int sig, num;

	if ((ret & ITMON_NOTIFY_MASK) != ITMON_NOTIFY_MASK)
		return;

	sig = ret & ~ITMON_NOTIFY_MASK;

	switch (info->err_code) {
	case ERR_DECERR:
		num = DECERR;
		break;
	case ERR_SLVERR:
		num = SLVERR;
		break;
	case ERR_TMOUT:
		num = TMOUT;
		break;
	default:
		log_dev_err(itmon->dev, "%s: Invalid error code(%u)\n",
			    __func__, info->err_code);
		return;
	}
	pl_policy = &pdata->policy[num];

	log_dev_info(itmon->dev, "\nPolicy Changes: %d -> %d in %s by notifier-call\n\n",
		     pl_policy->policy, sig, itmon_errcode[info->err_code]);

	pl_policy->policy = sig;
	pl_policy->prio = CUSTOM_MAX_PRIO;
}

static int itmon_notifier_handler(struct itmon_dev *itmon,
				  struct itmon_traceinfo *info,
				  unsigned int trans_type)
{
	int ret = 0;

	/* After treatment by port */
	if ((!info->src || strlen(info->src) < 1) ||
	    (!info->dest || strlen(info->dest) < 1))
		return -1;

	itmon->notifier_info.port = info->src;
	itmon->notifier_info.client = info->client;
	itmon->notifier_info.dest = info->dest;
	itmon->notifier_info.read = info->read;
	itmon->notifier_info.target_addr = info->target_addr;
	itmon->notifier_info.errcode = info->err_code;
	itmon->notifier_info.onoff = info->onoff;

	log_dev_err(itmon->dev, "----------------------------------------------------------------------------------\n\t\t+ITMON Notifier Call Information\n");

	/* call notifier_call_chain of itmon */
	ret = atomic_notifier_call_chain(&itmon_notifier_list, 0, &itmon->notifier_info);

	log_dev_err(itmon->dev, "\t\t-ITMON Notifier Call Information\n----------------------------------------------------------------------------------\n");

	return ret;
}

static void itmon_reflect_policy(struct itmon_dev *itmon,
				 struct itmon_traceinfo *info)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_policy *pl_policy = NULL;
	struct itmon_nodepolicy nd_policy1 = {0,}, nd_policy2 = {0,}, nd_policy = {0,};
	u64 chk_job1 = 0, chk_job2 = 0, en_job1 = 0, en_job2 = 0, en_job = 0;

	if (info->s_node) {
		nd_policy1.en = info->s_node->policy.en;
		if (nd_policy1.en_irq_mask) {
			if (info->err_code == ERR_TMOUT)
				itmon_en_timeout(itmon, info->s_node->group,
						 info->s_node, false);
			if (info->err_code == ERR_DECERR ||
			    info->err_code == ERR_SLVERR)
				itmon_en_err_report(itmon, info->s_node->group,
						    info->s_node, false);
		}
	}
	if (info->m_node) {
		nd_policy2.en = info->m_node->policy.en;
		if (nd_policy2.en_irq_mask) {
			if (info->err_code == ERR_DECERR ||
			    info->err_code == ERR_SLVERR)
				itmon_en_err_report(itmon, info->m_node->group,
						    info->m_node, false);
		}
	}

	switch (info->err_code) {
	case ERR_DECERR:
		pl_policy = &pdata->policy[DECERR];
		pl_policy->error = true;
		if (nd_policy1.chk_set) {
			chk_job1 = nd_policy1.chk_decerr_job;
			en_job1 = nd_policy1.en_decerr_job;
		}
		if (nd_policy2.chk_set) {
			chk_job2 = nd_policy2.chk_decerr_job;
			en_job2 = nd_policy2.en_decerr_job;
		}
		break;
	case ERR_SLVERR:
		pl_policy = &pdata->policy[SLVERR];
		pl_policy->error = true;
		if (nd_policy1.chk_set) {
			chk_job1 = nd_policy1.chk_slverr_job;
			en_job1 = nd_policy1.en_slverr_job;
		}
		if (nd_policy2.chk_set) {
			chk_job2 = nd_policy2.chk_slverr_job;
			en_job2 = nd_policy2.en_slverr_job;
		}
		break;
	case ERR_TMOUT:
		pl_policy = &pdata->policy[TMOUT];
		pl_policy->error = true;
		if (nd_policy1.chk_set) {
			chk_job1 = nd_policy1.chk_tmout_job;
			en_job1 = nd_policy1.en_tmout_job;
		}
		if (nd_policy2.chk_set) {
			chk_job2 = nd_policy2.chk_tmout_job;
			en_job2 = nd_policy2.en_tmout_job;
		}
		break;
	default:
		dev_warn(itmon->dev, "%s: Invalid error code(%u)\n",
			 __func__, info->err_code);
		return;
	}

	if (chk_job1 && chk_job2) {
		if (nd_policy1.prio >= nd_policy2.prio) {
			nd_policy.en = nd_policy1.en;
			en_job = en_job1;
		} else {
			nd_policy.en = nd_policy2.en;
			en_job = en_job2;
		}
	} else if (chk_job1) {
		nd_policy = nd_policy1;
		en_job = en_job1;
	} else if (chk_job2) {
		nd_policy = nd_policy2;
		en_job = en_job2;
	} /* nd_policy = NULL */
	if (nd_policy.chk_set && pl_policy->prio <= nd_policy.prio) {
		pl_policy->policy = en_job;
		pl_policy->prio = nd_policy.prio;
	}
}

static void itmon_post_handler(struct itmon_dev *itmon, bool err)
{
	struct itmon_platdata *pdata = itmon->pdata;
	unsigned long ts = pdata->last_time;
	unsigned long rem_nsec = do_div(ts, 1000000000);
	unsigned long cur_time = local_clock();
	unsigned long delta;

	delta = pdata->last_time == 0 ? 0 : cur_time - pdata->last_time;

	log_dev_err(itmon->dev, "Before ITMON: [%5lu.%06lu], delta: %lu, last_errcnt: %d\n",
		    (unsigned long)ts, rem_nsec / 1000, delta, pdata->last_errcnt);

	if (err)
		itmon_do_dpm_policy(itmon, true);

	/* delta < 1s */
	if (delta > 0 && delta < 1000000000UL) {
		char buf[SZ_64];

		pdata->last_errcnt++;
		if (pdata->last_errcnt > ERR_THRESHOLD) {
			strscpy(buf, "itmon triggering s2d start", sizeof(buf));
			dbg_snapshot_do_dpm_policy(GO_S2D_ID, buf);
		}
	} else {
		pdata->last_errcnt = 0;
	}

	pdata->last_time = cur_time;
}

static void itmon_report_rawdata(struct itmon_dev *itmon,
				 struct itmon_tracedata *data)
{
	struct itmon_nodeinfo *m_node = data->m_node;
	struct itmon_nodeinfo *det_node = data->det_node;
	struct itmon_nodegroup *group = data->group;

	/* Output Raw register information */
	log_dev_err(itmon->dev, "\tRaw Register Information ---------------------------------------------------\n\n");

	if (m_node && det_node) {
		log_dev_err(itmon->dev,
			    "\t> M_NODE     : %s(%s, id: %03u)\n"
			    "\t> DETECT_NODE: %s(%s, id: %03u)\n",
			    m_node->name, itmon_node_string[m_node->type], m_node->err_id,
			    det_node->name, itmon_node_string[det_node->type], det_node->err_id);
	}

	log_dev_err(itmon->dev, "\t> BASE       : %s(0x%08llx)\n"
	       "\t> INFO_0     : 0x%08X\n"
	       "\t> INFO_1     : 0x%08X\n"
	       "\t> INFO_2     : 0x%08X\n"
	       "\t> INFO_3     : 0x%08X\n"
	       "\t> INFO_4     : 0x%08X\n"
	       "\t> INFO_5     : 0x%08X\n"
	       "\t> INFO_6     : 0x%08X\n",
	       group->name, group->phy_regs,
	       data->info_0,
	       data->info_1,
	       data->info_2,
	       data->info_3,
	       data->info_4,
	       data->info_5,
	       data->info_6);
}

static void itmon_report_traceinfo(struct itmon_dev *itmon,
				   struct itmon_traceinfo *info,
				   unsigned int trans_type)
{
	if (!info->dirty)
		return;

	log_dev_err(itmon->dev, "\n----------------------------------------------------------------------------------\n"
	       "\tTransaction Information\n\n"
	       "\t> Client (User)  : %s %s (0x%X)\n"
	       "\t> Target         : %s\n"
	       "\t> Target Address : 0x%llX %s\n"
	       "\t> Type           : %s\n"
	       "\t> Error code     : %s\n",
	       info->src, info->client ? info->client : "", info->user,
	       info->dest ? info->dest : NO_NAME,
	       info->target_addr,
	       info->baaw_prot ? "(BAAW Remapped address)" : "",
	       trans_type == TRANS_TYPE_READ ? "READ" : "WRITE",
	       itmon_errcode[info->err_code]);

	itmon_pattern_set(itmon, "from %s %s to %s",
			  info->src, info->client ? info->client : "",
			  info->dest ? info->dest : NO_NAME);

	log_dev_err(itmon->dev, "\n----------------------------------------------------------------------------------\n"
	       "\t> Size           : %llu bytes x %u burst => %llu bytes\n"
	       "\t> Burst Type     : %u (0:FIXED, 1:INCR, 2:WRAP)\n"
	       "\t> Level          : %s\n"
	       "\t> Protection     : %s\n"
	       "\t> Path Type      : %s\n\n",
	       int_pow(2, info->axsize), info->axlen + 1,
	       int_pow(2, info->axsize) * (info->axlen + 1),
	       info->axburst,
	       info->axprot & BIT(0) ? "Privileged" : "Unprivileged",
	       info->axprot & BIT(1) ? "Non-secure" : "Secure",
	       itmon_pathtype[info->path_type]);
}

static void itmon_report_pathinfo(struct itmon_dev *itmon,
				  struct itmon_tracedata *data,
				  struct itmon_traceinfo *info,
				  unsigned int trans_type)
{
	struct itmon_nodeinfo *m_node = data->m_node;
	struct itmon_nodeinfo *det_node = data->det_node;

	if (!m_node || !det_node)
		return;

	if (!info->path_dirty) {
		log_dev_err(itmon->dev,
			    "\n----------------------------------------------------------------------------------\n"
			    "\t\tITMON Report (%s)\n"
			    "----------------------------------------------------------------------------------\n"
			    "\tPATH Information\n\n",
			    trans_type == TRANS_TYPE_READ ? "READ" : "WRITE");
		info->path_dirty = true;
	}

	log_dev_err(itmon->dev, "\t> M_NODE     : %14s, %8s(id: %u)\n",
		    m_node->name, itmon_node_string[m_node->type], m_node->err_id);
	log_dev_err(itmon->dev, "\t> DETECT_NODE: %14s, %8s(id: %u)\n",
		    det_node->name, itmon_node_string[det_node->type], det_node->err_id);
}

static void itmon_parse_cpuinfo_data(struct itmon_dev *itmon,
				     struct itmon_traceinfo *info,
				     unsigned int userbit)
{
	char core_str[SZ_16];
	const char *cache_str = NO_NAME;
	int cache_dirty = -1;
	int write_evict, core_num;

	core_num = PARSE_CPU_ID_DATA(userbit);
	write_evict = PARSE_CPU_WRITE_EVIC(userbit);

	if (core_num == 0xE) {
		strscpy(core_str, "ACP request", sizeof(core_str));
	} else if (core_num == 0xF) {
		cache_dirty = (!write_evict) ? 1 : 0;
		strscpy(core_str, "Cache copy back", sizeof(core_str));
	} else {
		scnprintf(core_str, sizeof(core_str), "CPU%d", core_num);
	}

	if (cache_dirty == 1)
		cache_str = "YES";
	else if (cache_dirty == 0)
		cache_str = "NO";

	scnprintf(info->buf, sizeof(info->buf),
		  "%s, ATTR:[%s][%s], WRITE_EVICT[%d], Cache dirty[%s]",
		  core_str,
		  itmon_cpu_attr_type[PARSE_CPU_SRCATTR_TYPE(userbit)],
		  itmon_cpu_attr_accs[PARSE_CPU_SRCATTR_ACCS(userbit)],
		  write_evict,
		  cache_str);
}

static void itmon_parse_cpuinfo(struct itmon_dev *itmon,
				struct itmon_tracedata *data,
				struct itmon_traceinfo *info,
				unsigned int userbit)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *m_node = data->m_node;
	int core_num = 0, i;

	for (i = 0; i < (int)ARRAY_SIZE(itmon_cpu_node_string); i++) {
		if (!strncmp(m_node->name, itmon_cpu_node_string[i], strlen(itmon_cpu_node_string[i]))) {
			if (!pdata->cpu_parsing) {
				strscpy(info->buf, "CPU", sizeof(info->buf));
				info->client = info->buf;
			} else if (info->path_type == CONFIG) {
				core_num = PARSE_CPU_ID_CONFIG(userbit);
				scnprintf(info->buf, sizeof(info->buf), "CPU%d ", core_num);
				info->client = info->buf;
			} else if (info->path_type == DATA && (userbit & BIT(25))) {
				itmon_parse_cpuinfo_data(itmon, info, userbit);
				info->client = info->buf;
			}
			return;
		}
	}
}

static void itmon_parse_traceinfo(struct itmon_dev *itmon,
				  struct itmon_tracedata *data,
				  unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodegroup *group = data->group;
	struct itmon_traceinfo *new_info = NULL;
	int i;

	if (!data->m_node || !data->det_node)
		return;

	new_info = kzalloc(sizeof(*new_info), GFP_ATOMIC);
	if (!new_info) {
		log_dev_err(itmon->dev, "failed to kmalloc for %s -> %s\n",
			    data->m_node->name, data->det_node->name);
		return;
	}

	new_info->user = PARSE_USER(data->info_5);
	new_info->m_id = data->m_id;
	new_info->s_id = data->det_id;
	new_info->m_node = data->m_node;
	new_info->s_node = data->det_node;
	new_info->src = data->m_node->name;
	new_info->dest = data->det_node->name;
	new_info->client = itmon_get_clientinfo(itmon, new_info->m_node->name, new_info->user);

	/* Common Information */
	new_info->path_type = group->path_type;

	itmon_parse_cpuinfo(itmon, data, new_info, data->info_5);

	new_info->target_addr = (((u64)data->info_3 & GENMASK(15, 0)) << 32ULL);
	new_info->target_addr |= data->info_2;
	new_info->err_code = data->err_code;
	new_info->dirty = true;
	new_info->axsize = AXSIZE(data->info_3);
	new_info->axlen = AXLEN(data->info_3);
	new_info->axburst = AXBURST(data->info_4);
	new_info->axprot = AXPROT(data->info_4);
	new_info->baaw_prot = false;

	for (i = 0; i < (int)ARRAY_SIZE(itmon_invalid_addr); i++) {
		if (new_info->target_addr == itmon_invalid_addr[i]) {
			new_info->baaw_prot = true;
			break;
		}
	}
	data->ref_info = new_info;
	list_add_tail(&new_info->list, &pdata->infolist[trans_type]);
}

static void itmon_analyze_errlog(struct itmon_dev *itmon)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_traceinfo *info, *next_info;
	struct itmon_tracedata *data, *next_data;
	unsigned int trans_type;
	int ret;

	/* Parse */
	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		list_for_each_entry(data, &pdata->datalist[trans_type], list)
			itmon_parse_traceinfo(itmon, data, trans_type);
	}

	/* Report and Clean-up traceinfo */
	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		list_for_each_entry_safe(info, next_info, &pdata->infolist[trans_type], list) {
			info->path_dirty = false;
			itmon_reflect_policy(itmon, info);
			ret = itmon_notifier_handler(itmon, info, trans_type);
			itmon_reflect_policy_by_notifier(itmon, info, ret);
			list_for_each_entry_safe(data, next_data, &pdata->datalist[trans_type], list) {
				if (data->ref_info == info)
					itmon_report_pathinfo(itmon, data, info, trans_type);
			}
			itmon_report_traceinfo(itmon, info, trans_type);
			list_del(&info->list);
			kfree(info);
		}
	}

	itmon_dump_dpm_policy(itmon);
	/* Report Raw all tracedata and Clean-up tracedata and node */
	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		list_for_each_entry_safe(data, next_data,
					 &pdata->datalist[trans_type], list) {
			itmon_report_rawdata(itmon, data);
			list_del(&data->list);
			kfree(data);
		}
	}
}

static void *itmon_collect_errlog(struct itmon_dev *itmon,
				  struct itmon_nodegroup *group)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_tracedata *new_data = NULL;
	void __iomem *reg = group->regs;

	new_data = kzalloc(sizeof(*new_data), GFP_ATOMIC);
	if (!new_data) {
		log_dev_err(itmon->dev, "kmalloc failed for %s group\n", group->name);
		return NULL;
	}

	new_data->info_0 = readl(reg + ERR_LOG_INFO_0);
	new_data->info_1 = readl(reg + ERR_LOG_INFO_1);
	new_data->info_2 = readl(reg + ERR_LOG_INFO_2);
	new_data->info_3 = readl(reg + ERR_LOG_INFO_3);
	new_data->info_4 = readl(reg + ERR_LOG_INFO_4);
	new_data->info_5 = readl(reg + ERR_LOG_INFO_5);
	new_data->info_6 = readl(reg + ERR_LOG_INFO_6);
	new_data->en_node = readl(reg + ERR_LOG_EN_NODE);

	new_data->m_id = MNODE_ID(new_data->info_0);
	new_data->det_id = DET_ID(new_data->info_0);

	new_data->m_node = itmon_get_nodeinfo_by_eid(itmon, NULL,
						     group->path_type, new_data->m_id);
	new_data->det_node = itmon_get_nodeinfo_by_eid(itmon, NULL,
						       group->path_type, new_data->det_id);

	new_data->read = !RW(new_data->info_0);
	new_data->err_code = ERR_CODE(new_data->info_0);
	new_data->ref_info = NULL;
	new_data->group = group;

	list_add_tail(&new_data->list, &pdata->datalist[new_data->read]);

	return (void *)new_data;
}

static int itmon_pop_errlog(struct itmon_dev *itmon,
			    struct itmon_nodegroup *group,
			    bool clear)
{
	struct itmon_tracedata *data;
	void __iomem *reg = group->regs;
	u32 num_log, max_log = 64;
	int ret = 0;

	num_log = readl(reg + ERR_LOG_STAT);

	while (num_log) {
		data = itmon_collect_errlog(itmon, group);
		writel(1, reg + ERR_LOG_POP);
		num_log = readl(group->regs + ERR_LOG_STAT);
		ret++;
		max_log--;

		if (max_log == 0)
			break;
	};

	if (clear)
		writel(1, reg + ERR_LOG_CLR);

	return ret;
}

static int itmon_search_errlog(struct itmon_dev *itmon,
			       struct itmon_nodegroup *group,
			       bool clear)
{
	struct itmon_platdata *pdata = itmon->pdata;
	int i, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&itmon->ctrl_lock, flags);

	if (group) {
		ret = itmon_pop_errlog(itmon, group, clear);
		itmon_report_timeout(itmon, group);
	} else {
		/* Processing all group & nodes */
		for (i = 0; i < pdata->num_nodegroup; i++) {
			group = &pdata->nodegroup[i];
			if (group->pd_support && !group->pd_status)
				continue;
			ret += itmon_pop_errlog(itmon, group, clear);
			itmon_report_timeout(itmon, group);
		}
	}
	if (ret)
		itmon_analyze_errlog(itmon);

	spin_unlock_irqrestore(&itmon->ctrl_lock, flags);
	return ret;
}

static irqreturn_t itmon_irq_handler(int irq, void *data)
{
	struct itmon_dev *itmon = (struct itmon_dev *)data;
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodegroup *group = NULL;
	bool ret;
	int i;

	itmon_pattern_reset(itmon);

	/* Search itmon group */
	for (i = 0; i < (int)pdata->num_nodegroup; i++) {
		group = &pdata->nodegroup[i];
		if (!group->pd_support) {
			log_dev_err(itmon->dev, "%d irq, %s group, pd %s, %x errors\n",
				    irq, group->name, "on",
				    readl(group->regs + ERR_LOG_STAT));
		} else {
			log_dev_err(itmon->dev, "%d irq, %s group, pd %s, %x errors\n",
				    irq, group->name,
				    group->pd_status ? "on" : "off",
				    group->pd_status ? readl(group->regs + ERR_LOG_STAT) : 0);
		}
	}

	ret = itmon_search_errlog(itmon, NULL, true);
	if (!ret)
		log_dev_err(itmon->dev, "No errors found %s\n", __func__);
	else
		log_dev_err(itmon->dev, "Error detected %s, %d\n", __func__, ret);

	itmon_post_handler(itmon, ret);

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

static int itmon_logging_panic_handler(struct notifier_block *nb,
				       unsigned long l, void *buf)
{
	struct itmon_panic_block *itmon_panic = (struct itmon_panic_block *)nb;
	struct itmon_dev *itmon = itmon_panic->pdev;
	int ret;

	if (!IS_ERR_OR_NULL(itmon)) {
		/* Check error has been logged */
		ret = itmon_search_errlog(itmon, NULL, true);
		if (!ret) {
			log_dev_info(itmon->dev, "No errors found %s\n", __func__);
		} else {
			log_dev_err(itmon->dev, "Error detected %s, %d\n", __func__, ret);
			itmon_do_dpm_policy(itmon, true);
		}
	}
	return 0;
}

#define MAX_CUSTOMIZE          64

static void itmon_parse_dt_customize(struct itmon_dev *itmon,
				     struct device_node *np)
{
	struct device_node *node_np;
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_policy *plat_policy;
	struct itmon_nodepolicy nd_policy;
	struct itmon_nodeinfo *node;
	char snode_name[16];
	const char *node_name;
	unsigned int val;
	int i;

	for (i = 0; i < MAX_CUSTOMIZE; i++) {
		nd_policy.en = 0;
		snprintf(snode_name, sizeof(snode_name), "node%d", i);
		node_np = of_get_child_by_name(np, (const char *)snode_name);

		if (!node_np) {
			dev_info(itmon->dev, "%s device node is not found\n",
				 snode_name);
			break;
		}

		if (of_property_read_string(node_np, "node", &node_name))
			continue;

		node = itmon_get_nodeinfo(itmon, NULL, node_name);
		if (!node) {
			dev_err(itmon->dev, "%s node is not found\n", node_name);
			continue;
		} else {
			dev_info(itmon->dev, "%s node found\n", node->name);
		}

		if (!of_property_read_u32(node_np, "irq_mask", &val)) {
			nd_policy.chk_irq_mask = 1;
			nd_policy.en_irq_mask = val;
			dev_info(itmon->dev, "%s node: irq mask -> [%d]\n",
				 node->name, val);
		}

		if (!of_property_read_u32(node_np, "prio", &val)) {
			nd_policy.prio = val;
			dev_info(itmon->dev, "%s node: prio -> [%d]\n",
				 node->name, val);
		}

		if (!of_property_read_u32(node_np, "errrpt", &val)) {
			nd_policy.chk_errrpt = 1;
			nd_policy.en_errrpt = val;
			dev_info(itmon->dev, "%s node: errrtp [%d] -> [%d]\n",
				 node->name, node->err_en, val);
		}

		if (!of_property_read_u32(node_np, "prtchker", &val)) {
			nd_policy.chk_prtchk = 1;
			nd_policy.en_prtchk = val;
			dev_info(itmon->dev, "%s node: prtchker [%d] -> [%d]\n",
				 node->name, node->prt_chk_en, val);
		}

		if (!of_property_read_u32(node_np, "tmout", &val)) {
			nd_policy.chk_tmout = 1;
			nd_policy.en_tmout = val;
			dev_info(itmon->dev, "%s node: tmout [%d] -> [%d]\n",
				 node->name, node->tmout_en, val);
		}

		if (!of_property_read_u32(node_np, "tmout_val", &val)) {
			nd_policy.chk_tmout_val = 1;
			nd_policy.en_tmout_val = val;
			dev_info(itmon->dev, "%s node: tmout_val [%x] -> [%x]\n",
				 node->name, node->time_val, val);
		}

		if (!of_property_read_u32(node_np, "freeze", &val)) {
			nd_policy.chk_freeze = 1;
			nd_policy.en_freeze = val;
			dev_info(itmon->dev, "%s node: freeze [%x] -> [%x]\n",
				 node->name, node->tmout_frz_en, val);
		}

		if (!of_property_read_u32(node_np, "decerr_job", &val)) {
			plat_policy = &pdata->policy[DECERR];
			nd_policy.chk_decerr_job = 1;
			nd_policy.en_decerr_job = val;
			dev_info(itmon->dev, "%s node: decerr_job [%s] -> [%d/%s]\n",
				 node->name, itmon_dpm_action[plat_policy->policy_def],
				 nd_policy.chk_decerr_job,
				 itmon_dpm_action[nd_policy.en_decerr_job]);
		}

		if (!of_property_read_u32(node_np, "slverr_job", &val)) {
			plat_policy = &pdata->policy[SLVERR];
			nd_policy.chk_slverr_job = 1;
			nd_policy.en_slverr_job = val;
			dev_info(itmon->dev, "%s node: slverr_job [%s] -> [%d/%s]\n",
				 node->name, itmon_dpm_action[plat_policy->policy_def],
				 nd_policy.chk_slverr_job,
				 itmon_dpm_action[nd_policy.en_slverr_job]);
		}

		if (!of_property_read_u32(node_np, "tmout_job", &val)) {
			plat_policy = &pdata->policy[TMOUT];
			nd_policy.chk_tmout_job = 1;
			nd_policy.en_tmout_job = val;
			dev_info(itmon->dev, "%s node: tmout_job [%s] -> [%d/%s]\n",
				 node->name, itmon_dpm_action[plat_policy->policy_def],
				 nd_policy.chk_tmout_job,
				 itmon_dpm_action[nd_policy.en_tmout_job]);
		}

		if (!of_property_read_u32(node_np, "prtchker_job", &val)) {
			plat_policy = &pdata->policy[PRTCHKER];
			nd_policy.chk_prtchk_job = 1;
			nd_policy.en_prtchk_job = val;
			dev_info(itmon->dev, "%s node: prtchker_job [%s] -> [%d/%s]\n",
				 node->name, itmon_dpm_action[plat_policy->policy_def],
				 nd_policy.chk_prtchk_job,
				 itmon_dpm_action[nd_policy.en_prtchk_job]);
		}
		itmon_set_nodepolicy(itmon, node, nd_policy, false);
	}
	itmon_dump_dpm_policy(itmon);
}

static void itmon_parse_dt(struct itmon_dev *itmon)
{
	struct device_node *np = itmon->dev->of_node;
	struct device_node *child_np;
	struct itmon_platdata *pdata = itmon->pdata;
	unsigned int val;
	int i;

	if (of_property_read_bool(np, "no-def-en")) {
		log_dev_info(itmon->dev, "No default enable support\n");
		pdata->def_en = false;
	} else {
		pdata->def_en = true;
	}

	if (of_property_read_bool(np, "no-support-cpu-parsing")) {
		log_dev_info(itmon->dev, "CPU Parser is not support\n");
		pdata->cpu_parsing = false;
	} else {
		pdata->cpu_parsing = true;
	}

	for (i = 0; i < TYPE_MAX; i++) {
		if (!of_property_read_u32(np, pdata->policy[i].name, &val)) {
			pdata->policy[i].policy_def = val;
			pdata->policy[i].policy = val;
		}
	}

	child_np = of_get_child_by_name(np, "customize");
	if (!child_np) {
		dev_info(itmon->dev, "customize is not found\n");
		return;
	}

	itmon_parse_dt_customize(itmon, child_np);

	child_np = of_get_child_by_name(np, "dbgc");
	if (!child_np) {
		log_dev_info(itmon->dev, "dbgc is not found\n");
		return;
	}
}

static void itmon_set_irq_affinity(struct itmon_dev *itmon,
				   unsigned int irq,
				   unsigned long affinity)
{
	struct cpumask affinity_mask;
	unsigned long bit;
	char *buf = (char *)__get_free_page(GFP_KERNEL);

	cpumask_clear(&affinity_mask);

	if (affinity) {
		for_each_set_bit(bit, &affinity, nr_cpu_ids)
			cpumask_set_cpu(bit, &affinity_mask);
	} else {
		cpumask_copy(&affinity_mask, cpu_online_mask);
	}
	irq_set_affinity_hint(irq, &affinity_mask);

	cpumap_print_to_pagebuf(true, buf, &affinity_mask);
	log_dev_dbg(itmon->dev, "affinity of irq%d is %s", irq, buf);
	free_page((unsigned long)buf);
}

static int itmon_probe(struct platform_device *pdev)
{
	struct itmon_dev *itmon;
	struct itmon_panic_block *itmon_panic = NULL;
	struct itmon_platdata *pdata;
	struct itmon_nodeinfo *node;
	unsigned int irq, val = 0;
	char *dev_name;
	int ret, i, j;

	itmon = devm_kzalloc(&pdev->dev, sizeof(struct itmon_dev), GFP_KERNEL);
	if (!itmon)
		return -ENOMEM;

	itmon->dev = &pdev->dev;

	spin_lock_init(&itmon->ctrl_lock);

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct itmon_platdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	itmon->pdata = pdata;

	pdata->clientinfo = clientinfo;
	pdata->num_clientinfo = ARRAY_SIZE(clientinfo);

	pdata->nodegroup = nodegroup;
	pdata->num_nodegroup = ARRAY_SIZE(nodegroup);

	pdata->rpathinfo = rpathinfo;
	pdata->num_rpathinfo = ARRAY_SIZE(rpathinfo);

	itmon->pdata->policy = err_policy;

	itmon_parse_dt(itmon);

	for (i = 0; i < (int)pdata->num_nodegroup; i++) {
		dev_name = pdata->nodegroup[i].name;
		node = pdata->nodegroup[i].nodeinfo;

		if (pdata->nodegroup[i].phy_regs) {
			pdata->nodegroup[i].regs =
				devm_ioremap(&pdev->dev, pdata->nodegroup[i].phy_regs, SZ_16K);
			if (!pdata->nodegroup[i].regs) {
				log_dev_err(&pdev->dev,
					    "failed to claim register region - %s\n",
					    dev_name);
				return -ENOENT;
			}
		}

		for (j = 0; j < pdata->nodegroup[i].nodesize; j++)
			node[j].group = &pdata->nodegroup[i];

		irq = irq_of_parse_and_map(pdev->dev.of_node, i);
		pdata->nodegroup[i].irq = SET_IRQ(irq);
		if (!irq)
			continue;

		of_property_read_u32_index(pdev->dev.of_node, "interrupt-affinity", i, &val);
		itmon_set_irq_affinity(itmon, irq, val);

		pdata->nodegroup[i].irq |= SET_AFFINITY(val);

		irq_set_status_flags(irq, IRQ_NOAUTOEN);
		disable_irq(irq);
		ret = devm_request_irq(&pdev->dev, irq,
				       itmon_irq_handler,
				       IRQF_NOBALANCING, dev_name, itmon);
		if (ret == 0) {
			log_dev_info(&pdev->dev,
				     "success to register request irq%u - %s\n",
				     irq, dev_name);
		} else {
			log_dev_err(&pdev->dev, "failed to request irq - %s\n",
				    dev_name);
			return -ENOENT;
		}
	}
	itmon_panic = devm_kzalloc(&pdev->dev, sizeof(struct itmon_panic_block), GFP_KERNEL);

	if (!itmon_panic) {
		log_dev_err(&pdev->dev, "failed to alloc for panic handler\n");
	} else {
		itmon_panic->nb_panic_block.notifier_call = itmon_logging_panic_handler;
		itmon_panic->pdev = itmon;
		atomic_notifier_chain_register(&panic_notifier_list,
					       &itmon_panic->nb_panic_block);
	}

	platform_set_drvdata(pdev, itmon);

	itmon_init(itmon, true);

	pdata->last_time = 0;
	pdata->last_errcnt = 0;
	itmon_pattern_reset(itmon);
	g_itmon = itmon;

	for (i = 0; i < TRANS_TYPE_NUM; i++) {
		INIT_LIST_HEAD(&pdata->datalist[i]);
		INIT_LIST_HEAD(&pdata->infolist[i]);
	}

	pdata->probed = true;
	log_dev_info(&pdev->dev, "success to probe Exynos ITMON driver\n");

	for (i = 0; i < (int)pdata->num_nodegroup; i++)
		if (pdata->nodegroup[i].irq)
			enable_irq(GET_IRQ(pdata->nodegroup[i].irq));

	return 0;
}

static int itmon_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int itmon_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct itmon_dev *itmon = platform_get_drvdata(pdev);
	struct itmon_platdata *pdata = itmon->pdata;
	int i;

	for (i = 0; i < (int)pdata->num_nodegroup; i++) {
		unsigned int irq = pdata->nodegroup[i].irq;

		if (irq)
			itmon_set_irq_affinity(itmon, GET_IRQ(irq), GET_AFFINITY(irq));
	}
	itmon_init(itmon, true);

	dev_dbg(dev, "%s\n", __func__);
	return 0;
}

const struct dev_pm_ops __maybe_unused itmon_pm_ops = {
	.resume_noirq = itmon_resume_noirq
};


#endif

static struct platform_driver exynos_itmon_driver = {
	.probe = itmon_probe,
	.remove = itmon_remove,
	.driver = {
		   .name = "exynos-itmon-v2",
		   .of_match_table = itmon_dt_match,
		   .pm = pm_ptr(&itmon_pm_ops),
		   },
};
module_platform_driver(exynos_itmon_driver);

MODULE_DESCRIPTION("Samsung Exynos ITMON COMMON DRIVER V2");
MODULE_AUTHOR("Hosung Kim <hosung0.kim@samsung.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos-itmon");
