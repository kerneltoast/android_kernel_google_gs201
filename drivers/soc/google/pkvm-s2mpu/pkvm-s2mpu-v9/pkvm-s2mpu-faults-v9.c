// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023 - Google LLC
 */

#include "kvm_s2mpu.h"
#include <soc/google/pkvm-s2mpu.h>
#include <linux/io-64-nonatomic-hi-lo.h>

#define S2MPU_NR_WAYS		4
#define S2MPU_STLB_LINE_SIZE	4

struct s2mpu_mptc_entry {
	bool valid;
	u32 vid;
	u32 ppn;
	u32 others;
	u32 data;
};

struct s2mpu_ptlb_entry {
	bool valid;
	bool stage1_enabled;
	u32 tag;
	u32 ap_list;
};

static const char *str_fault_direction(u32 fault_info)
{
	return (fault_info & FAULT_INFO_RW_BIT) ? "write" : "read";
}

static const char *str_fault_type(u32 fault_info)
{
	switch (FIELD_GET(FAULT_INFO_TYPE_MASK, fault_info)) {
	case FAULT_INFO_TYPE_MPTW:
		return "MPTW fault";
	case FAULT_INFO_TYPE_AP:
		return "access permission fault";
	case FAULT_INFO_TYPE_CONTEXT:
		return "context fault";
	default:
		return "unknown fault";
	}
}

static const char *str_l1entry_gran(u32 l1attr)
{
	if (!(l1attr & L1ENTRY_ATTR_L2TABLE_EN))
		return "1G";

	switch (FIELD_GET(L1ENTRY_ATTR_GRAN_MASK, l1attr)) {
	case L1ENTRY_ATTR_GRAN_4K:
		return "4K";
	case L1ENTRY_ATTR_GRAN_64K:
		return "64K";
	default:
		return "invalid";
	}
}

static const char *str_l1entry_prot(u32 l1attr)
{
	if (l1attr & L1ENTRY_ATTR_L2TABLE_EN)
		return "??";

	switch (FIELD_GET(L1ENTRY_ATTR_PROT_MASK, l1attr)) {
	case MPT_PROT_NONE:
		return "0";
	case MPT_PROT_R:
		return "R";
	case MPT_PROT_W:
		return "W";
	case MPT_PROT_RW:
		return "RW";
	default:
		return "invalid";
	}
}

static struct s2mpu_mptc_entry read_mptc(void __iomem *base, u32 set, u32 way)
{
	struct s2mpu_mptc_entry entry;
	u32 tag_ppn;

	writel_relaxed(V9_READ_MPTC(set, way), base + REG_NS_V9_READ_MPTC);

	tag_ppn = readl_relaxed(base + REG_NS_V9_READ_MPTC_TAG_PPN),
	entry.others = readl_relaxed(base + REG_NS_V9_READ_MPTC_TAG_OTHERS),
	entry.data = readl_relaxed(base + REG_NS_V9_READ_MPTC_DATA),

	entry.ppn = FIELD_GET(V9_READ_MPTC_TAG_PPN_TPN_PPN_MASK, tag_ppn);
	entry.valid = FIELD_GET(V9_READ_MPTC_TAG_PPN_VALID_MASK, tag_ppn);
	entry.vid = FIELD_GET(V9_READ_MPTC_TAG_OTHERS_VID_MASK, entry.others);
	return entry;
}

static struct  s2mpu_ptlb_entry read_ptlb(void __iomem *base, int pmmu_id,
					  int ptlb_i, int set, int way)
{
	struct s2mpu_ptlb_entry entry;

	writel_relaxed(V9_READ_PTLB(pmmu_id, ptlb_i, set, way), base + REG_NS_V9_READ_PTLB);
	entry.tag = readl_relaxed(base + REG_NS_V9_READ_PTLB_TAG);
	entry.valid = FIELD_GET(V9_READ_PTLB_TAG_VALID_MASK, entry.tag);
	entry.stage1_enabled = FIELD_GET(V9_READ_PTLB_TAG_STAGE1_ENABLED_MASK, entry.tag);

	/*
	 * Each PTLB line have one stage-1 page descriptor and corresponding access control
	 * descriptor when storing stage-1 enabled content. Otherwise, each PTLB line have 16
	 * access control descriptors when storing stage-1 disabled content.
	 */
	if (entry.stage1_enabled)
		entry.ap_list = readl_relaxed(base + REG_NS_V9_READ_PTLB_DATA_S1_EN_PPN_AP);
	else
		entry.ap_list = readl_relaxed(base + REG_NS_V9_READ_PTLB_DATA_S1_DIS_AP_LIST);
	return entry;
}

static int dump_stlb_typea(struct s2mpu_data *data, int stlb_i, int set, int way)
{
	int tag, valid, others, stlb_data, tpn, invalid_cnt = 0;
	struct device *dev = data->dev;
	int subline;

	for (subline  = 0 ; subline < S2MPU_STLB_LINE_SIZE ; ++subline) {
		writel_relaxed(V9_READ_STLB_TYPEA(stlb_i, subline, set, way),
			       data->base + REG_NS_V9_READ_STLB);
		tag = readl_relaxed(data->base + REG_NS_V9_READ_STLB_TAG_PPN);
		tpn = readl_relaxed(data->base + REG_NS_V9_READ_STLB_TPN);
		valid = FIELD_GET(V9_READ_STLB_TPN_VALID_MASK, tpn);
		others =  readl_relaxed(data->base + REG_NS_V9_READ_STLB_TAG_OTHERS);
		stlb_data =  readl_relaxed(data->base + REG_NS_V9_READ_STLB_DATA);
		invalid_cnt += !valid;
		if (valid) {
			dev_err(dev, "  STLB[set=%u, way=%u, stlb=%u, subline=%u]",
				set, way, stlb_i, subline);
			dev_err(dev, "     {TAG=%#x, OTHERS=%#x, DATA=%#x, TPN=%#x}\n",
				tag, others, stlb_data, tpn);
		}
	}

	return invalid_cnt;
}

static int dump_stlb_typeb(struct s2mpu_data *data, int stlb_i, int set, int way)
{
	int tag, valid, others, stlb_data;
	struct device *dev = data->dev;

	writel_relaxed(V9_READ_STLB_TYPEB(stlb_i, set, way), data->base + REG_NS_V9_READ_STLB);
	tag = readl_relaxed(data->base + REG_NS_V9_READ_STLB_TAG_PPN);
	valid = FIELD_GET(V9_READ_STLB_TAG_PPN_VALID_MASK_TYPEB, tag);
	others = readl_relaxed(data->base + REG_NS_V9_READ_STLB_TAG_OTHERS);
	stlb_data = readl_relaxed(data->base + REG_NS_V9_READ_STLB_DATA);
	if (valid) {
		dev_err(dev, "  STLB[set=%u, way=%u, stlb=%u]={TAG=%#x, OTHERS=%#x, DATA=%#x}\n",
			set, way, stlb_i, tag, others, stlb_data);
	}
	return !valid;
}

irqreturn_t s2mpu_fault_handler(struct s2mpu_data *data, bool print_caches)
{
	struct device *dev = data->dev;
	unsigned int vid, gb;
	u32 vid_bmap, fmpt, smpt, nr_sets, set, way, invalid;
	/* Fault variables. */
	u32 fault_info, fault_info2, axi_id, pmmu_id, stream_id;
	/* PTLB variables. */
	u32 ptlb_num, ptlb_way, ptlb_set, ptlb_i, ptlb_info;
	/* STLB variables. */
	u32 stlb_i, stlb_num, stlb_info, stlb_way, stlb_set;
	phys_addr_t fault_pa;
	struct s2mpu_mptc_entry mptc;
	struct s2mpu_ptlb_entry ptlb;

	irqreturn_t ret = IRQ_NONE;

	while ((vid_bmap = readl_relaxed(data->base + REG_NS_FAULT_STATUS))) {
		WARN_ON_ONCE(vid_bmap & (~ALL_VIDS_BITMAP));
		vid = __ffs(vid_bmap);

		fault_pa = hi_lo_readq_relaxed(data->base + REG_NS_FAULT_PA_HIGH_LOW(vid));
		fault_info = readl_relaxed(data->base + REG_NS_FAULT_INFO(vid));
		axi_id = readl_relaxed(data->base + REG_NS_FAULT_INFO1(vid));
		fault_info2 = readl_relaxed(data->base + REG_NS_FAULT_INFO2(vid));
		WARN_ON(FIELD_GET(FAULT_INFO_VID_MASK, fault_info) != vid);
		pmmu_id = FIELD_GET(FAULT2_PMMU_ID_MASK, fault_info2);
		stream_id = FIELD_GET(FAULT2_STREAM_ID_MASK, fault_info2);

		dev_err(dev, "============== S2MPU FAULT DETECTED ==============\n");
		dev_err(dev, "  PA=%pap, FAULT_INFO=0x%08x\n",
			&fault_pa, fault_info);
		dev_err(dev, "  DIRECTION: %s, TYPE: %s\n",
			str_fault_direction(fault_info),
			str_fault_type(fault_info));
		dev_err(dev, "  VID=%u, REQ_LENGTH=%lu, REQ_AXI_ID=%u\n",
			vid, FIELD_GET(FAULT_INFO_LEN_MASK, fault_info), axi_id);
		dev_err(dev, "  PMMU_ID=%u, STREAM_ID=%u\n", pmmu_id, stream_id);

		for_each_gb(gb) {
			fmpt = readl_relaxed(data->base + REG_NS_L1ENTRY_ATTR(vid, gb));
			smpt = readl_relaxed(data->base + REG_NS_L1ENTRY_L2TABLE_ADDR(vid, gb));
			dev_err(dev, "  %uG: FMPT=%#x (%s, %s), SMPT=%#x\n",
				gb, fmpt, str_l1entry_gran(fmpt),
				str_l1entry_prot(fmpt), smpt);
		}

		dev_err(dev, "==================================================\n");

		writel_relaxed(BIT(vid), data->base + REG_NS_INTERRUPT_CLEAR);
		ret = IRQ_HANDLED;
	}

	if (!print_caches)
		return ret;

	/* Instances with only stage-2, doesn't have MPTC, and STLB is used as MPTC b/269725512 */
	if (data->has_sysmmu) {
		dev_err(dev, "================== MPTC ENTRIES ==================\n");
		nr_sets = FIELD_GET(V9_READ_MPTC_INFO_NUM_MPTC_SET,
				    readl_relaxed(data->base + REG_NS_V9_MPTC_INFO));
		for (invalid = 0, set = 0; set < nr_sets; set++) {
			for (way = 0; way < S2MPU_NR_WAYS; way++) {
				mptc = read_mptc(data->base, set, way);
				if (!mptc.valid) {
					invalid++;
					continue;
				}

				dev_err(dev,
					"  MPTC[set=%u, way=%u]={VID=%u, PPN=%#x, OTHERS=%#x, DATA=%#x}\n",
					set, way, mptc.vid, mptc.ppn, mptc.others, mptc.data);
			}
		}
		dev_err(dev, "  invalid entries: %u\n", invalid);
		dev_err(dev, "==================================================\n");
	}

	dev_err(dev, "================ PRIVATE MMU =====================\n");
	ptlb_num = FIELD_GET(V9_READ_PMMU_INFO_NUM_PTLB,
			     readl_relaxed(data->base + REG_NS_V9_PMMU_INFO));
	dev_err(dev, "  Number of PTLB: %u\n", ptlb_num);
	for (invalid = 0, ptlb_i = 0; ptlb_i < ptlb_num; ++ptlb_i) {
		ptlb_info = readl_relaxed(data->base + REG_NS_V9_PMMU_PTLB_INFO(ptlb_i));
		ptlb_way = FIELD_GET(V9_READ_PMMU_PTLB_INFO_NUM_WAY, ptlb_info);
		ptlb_set = FIELD_GET(V9_READ_PMMU_PTLB_INFO_NUM_SET, ptlb_info);
		dev_err(dev, "  PTLB[%u] number of ways %u, Number of sets %u",
			ptlb_i, ptlb_way, ptlb_set);
		for (set = 0; set < ptlb_set; ++set) {
			for (way = 0; way < ptlb_way; ++way) {
				ptlb = read_ptlb(data->base, pmmu_id, ptlb_i, set, way);
				if (!ptlb.valid) {
					invalid++;
					continue;
				}
				dev_err(dev, "  PTLB[ptlb=%u, set=%u, way=%u]\n",
					ptlb_i, set, way);
				dev_err(dev, "    {TAG=%#x, STAGE1_ENABLED=%#x, DATA_AP=%#x}\n",
					ptlb.tag, ptlb.stage1_enabled, ptlb.ap_list);
			}
		}
	}
	dev_err(dev, " Invalid entries: %u\n", invalid);
	dev_err(dev, "==================================================\n");

	dev_err(dev, "============= STLB ENTRIES TYPE %c ===============\n",
		'B' - data->has_sysmmu);
	stlb_num = FIELD_GET(V9_SWALKER_INFO_NUM_STLB_MASK,
			     readl_relaxed(data->base + REG_NS_V9_SWALKER_INFO));
	dev_err(dev, "================== Num of STLBs %u ================\n", stlb_num);
	for (invalid = 0, stlb_i = 0 ; stlb_i < stlb_num ; ++stlb_i) {
		stlb_info = readl_relaxed(data->base + REG_NS_V9_STLB_INFO(stlb_i));
		stlb_set = FIELD_GET(V9_READ_SLTB_INFO_SET_MASK, stlb_info);
		stlb_way = FIELD_GET(V9_READ_SLTB_INFO_WAY_MASK, stlb_info);
		dev_err(dev, "Number of ways %u, Number of sets %u", stlb_way, stlb_set);
		for (set = 0 ; set < stlb_set ; ++set) {
			for (way = 0 ; way < stlb_way ; ++way) {
				if (data->has_sysmmu)
					invalid += dump_stlb_typea(data, stlb_i, set, way);
				else
					invalid += dump_stlb_typeb(data, stlb_i, set, way);
			}
		}
	}
	dev_err(dev, "  Invalid entries: %u\n", invalid);
	dev_err(dev, "==================================================\n");

	return ret;
}
