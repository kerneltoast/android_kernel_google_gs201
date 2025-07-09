#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <soc/google/cal-if.h>
#include <linux/module.h>

#include "fvmap.h"
#include "cmucal.h"
#include "vclk.h"
#include "ra.h"

#define FVMAP_SIZE		(SZ_8K)
#define STEP_UV			(6250)

void __iomem *fvmap_base;
void __iomem *sram_fvmap_base;

/* Making sure margin_id and acpm_dvfs_id are sync'ed */
enum margin_id {
	MARGIN_MIF,
	MARGIN_INT,
	MARGIN_LIT,
	MARGIN_MID,
	MARGIN_BIG,
	MARGIN_DSU,
	MARGIN_BCI,
	MARGIN_G3D,
	MARGIN_G3DL2,
	MARGIN_TPU,
	MARGIN_INTCAM,
	MARGIN_TNR,
	MARGIN_CAM,
	MARGIN_MFC,
	MARGIN_DISP,
	MARGIN_AOC,
	MARGIN_BW,
	MARGIN_MAX
};

static int *margin_table[MARGIN_MAX];

static int margin_mif;
static int margin_int;
static int margin_lit;
static int margin_mid;
static int margin_big;
static int margin_dsu;
static int margin_bci;
static int margin_g3d;
static int margin_g3dl2;
static int margin_tpu;
static int margin_intcam;
static int margin_tnr;
static int margin_cam;
static int margin_mfc;
static int margin_disp;
static int margin_bw;

module_param(margin_mif, int, 0);
module_param(margin_int, int, 0);
module_param(margin_lit, int, 0);
module_param(margin_mid, int, 0);
module_param(margin_big, int, 0);
module_param(margin_dsu, int, 0);
module_param(margin_bci, int, 0);
module_param(margin_g3d, int, 0);
module_param(margin_g3dl2, int, 0);
module_param(margin_tpu, int, 0);
module_param(margin_intcam, int, 0);
module_param(margin_tnr, int, 0);
module_param(margin_cam, int, 0);
module_param(margin_mfc, int, 0);
module_param(margin_disp, int, 0);
module_param(margin_bw, int, 0);

void margin_table_init(void)
{
	margin_table[MARGIN_MIF] = &margin_mif;
	margin_table[MARGIN_INT] = &margin_int;
	margin_table[MARGIN_LIT] = &margin_lit;
	margin_table[MARGIN_MID] = &margin_mid;
	margin_table[MARGIN_BIG] = &margin_big;
	margin_table[MARGIN_DSU] = &margin_dsu;
	margin_table[MARGIN_BCI] = &margin_bci;
	margin_table[MARGIN_G3D] = &margin_g3d;
	margin_table[MARGIN_G3DL2] = &margin_g3dl2;
	margin_table[MARGIN_TPU] = &margin_tpu;
	margin_table[MARGIN_INTCAM] = &margin_intcam;
	margin_table[MARGIN_TNR] = &margin_tnr;
	margin_table[MARGIN_CAM] = &margin_cam;
	margin_table[MARGIN_MFC] = &margin_mfc;
	margin_table[MARGIN_DISP] = &margin_disp;
	margin_table[MARGIN_BW] = &margin_bw;
}

int fvmap_set_raw_voltage_table(unsigned int id, int uV)
{
	struct fvmap_header *fvmap_header;
	struct rate_volt_header *fv_table;
	int num_of_lv;
	int idx, i;

	idx = GET_IDX(id);

	fvmap_header = sram_fvmap_base;
	fv_table = sram_fvmap_base + fvmap_header[idx].o_ratevolt;
	num_of_lv = fvmap_header[idx].num_of_lv;

	for (i = 0; i < num_of_lv; i++)
		fv_table->table[i].volt += uV;

	return 0;
}

int fvmap_get_voltage_table(unsigned int id, unsigned int *table)
{
	struct fvmap_header *fvmap_header = fvmap_base;
	struct rate_volt_header *fv_table;
	int idx, i;
	int num_of_lv;

	if (!IS_ACPM_VCLK(id))
		return 0;

	idx = GET_IDX(id);

	fvmap_header = fvmap_base;
	fv_table = fvmap_base + fvmap_header[idx].o_ratevolt;
	num_of_lv = fvmap_header[idx].num_of_lv;

	for (i = 0; i < num_of_lv; i++)
		table[i] = fv_table->table[i].volt;

	return num_of_lv;
}
EXPORT_SYMBOL_GPL(fvmap_get_voltage_table);

int fvmap_get_raw_voltage_table(unsigned int id)
{
	struct fvmap_header *fvmap_header;
	struct rate_volt_header *fv_table;
	int idx, i;
	int num_of_lv;
	unsigned int table[20];

	idx = GET_IDX(id);

	fvmap_header = sram_fvmap_base;
	fv_table = sram_fvmap_base + fvmap_header[idx].o_ratevolt;
	num_of_lv = fvmap_header[idx].num_of_lv;

	for (i = 0; i < num_of_lv; i++)
		table[i] = fv_table->table[i].volt;

	for (i = 0; i < num_of_lv; i++)
		printk("dvfs id : %d  %d Khz : %d uv\n", ACPM_VCLK_TYPE | id, fv_table->table[i].rate, table[i]);

	return 0;
}

static void fvmap_copy_from_sram(void *map_base, void __iomem *sram_base)
{
	struct fvmap_header *fvmap_header;
	struct rate_volt_header *old, *new;
	struct clocks __iomem *clks;
	struct pll_header __iomem *plls;
	struct vclk *vclk;
	u32 member_addr;
	unsigned int blk_idx;
	int size, margin;
	int i, j;

	fvmap_header = map_base;
	size = cmucal_get_list_size(ACPM_VCLK_TYPE);
	memcpy_fromio(fvmap_header, sram_base, sizeof(struct fvmap_header) * size);

	for (i = 0; i < size; i++) {
		/* load fvmap info */
		vclk = cmucal_get_node(ACPM_VCLK_TYPE | i);
		if (vclk == NULL)
			continue;
		pr_debug("dvfs_type : %s - id : %x\n",
			 vclk->name, fvmap_header[i].dvfs_type);
		pr_debug("  num_of_lv      : %u\n", fvmap_header[i].num_of_lv);
		pr_debug("  num_of_members : %u\n", fvmap_header[i].num_of_members);

		old = sram_base + fvmap_header[i].o_ratevolt;
		new = map_base + fvmap_header[i].o_ratevolt;

		margin = *margin_table[vclk->id - ACPM_VCLK_TYPE];
		if (margin)
			cal_dfs_set_volt_margin(i | ACPM_VCLK_TYPE, margin);

		for (j = 0; j < fvmap_header[i].num_of_members; j++) {
			clks = sram_base + fvmap_header[i].o_members;

			if (j < fvmap_header[i].num_of_pll) {
				plls = sram_base + readw_relaxed(&clks->addr[j]);
				member_addr = readl_relaxed(&plls->addr) - 0x90000000;
			} else {
				u16 clks_addr = readw_relaxed(&clks->addr[j]);

				member_addr = (clks_addr & ~0x3) & 0xffff;
				blk_idx = clks_addr & 0x3;

				member_addr |= ((fvmap_header[i].block_addr[blk_idx]) << 16) - 0x90000000;
			}

			vclk->list[j] = cmucal_get_id_by_addr(member_addr);

			if (vclk->list[j] == INVALID_CLK_ID)
				pr_err("  Invalid addr :0x%x\n", member_addr);
			else
				pr_debug("  DVFS CMU addr:0x%x\n", member_addr);
		}

		for (j = 0; j < fvmap_header[i].num_of_lv; j++) {
			int volt = new->table[j].volt = old->table[j].volt;
			new->table[j].rate = old->table[j].rate;
			if (margin) {
				if (margin <= 100 && margin >= -100) {
					volt = volt + (volt * margin / 100);
				} else {
					margin = (margin / STEP_UV) * STEP_UV;
					volt = volt + margin;
				}
			}
			pr_debug("  lv : [%7d], volt = %d uV (%+d uV)\n",
				 new->table[j].rate, new->table[j].volt,
				 volt - new->table[j].volt);
		}
	}
}

#define DEFINE_INT_ATTRIBUTE(__margin, __id)				\
static int __margin##_show(struct seq_file *buf, void *d)		\
{									\
	seq_printf(buf, "%d\n", __margin);				\
	return 0;							\
}									\
static int __margin##_open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, __margin##_show, inode->i_private);	\
}									\
static ssize_t __margin##_write_file(struct file *file,			\
				     const char __user *user_buf,	\
				     size_t count, loff_t *ppos)	\
{									\
	char buf[32];							\
	ssize_t buf_size;						\
	int ret, margin;						\
									\
	buf_size = simple_write_to_buffer(buf, sizeof(buf) - 1,		\
					  ppos, user_buf, count);	\
	if (buf_size < 0)						\
		return buf_size;					\
									\
	ret = sscanf(buf, "%d\n", &margin);				\
	if (ret != 1) {							\
		pr_err("%s: sscanf failed (%d)\n", __func__, ret);	\
		return -EINVAL;						\
	}								\
	__margin = margin;						\
	cal_dfs_set_volt_margin((__id) | ACPM_VCLK_TYPE, __margin);	\
									\
	return buf_size;						\
}									\
static const struct file_operations __margin##_fops = {			\
	.open = __margin##_open,					\
	.read = seq_read,						\
	.write = __margin##_write_file,					\
	.llseek = seq_lseek,						\
	.release = single_release,					\
}

DEFINE_INT_ATTRIBUTE(margin_mif, MARGIN_MIF);
DEFINE_INT_ATTRIBUTE(margin_int, MARGIN_INT);
DEFINE_INT_ATTRIBUTE(margin_lit, MARGIN_LIT);
DEFINE_INT_ATTRIBUTE(margin_mid, MARGIN_MID);
DEFINE_INT_ATTRIBUTE(margin_big, MARGIN_BIG);
DEFINE_INT_ATTRIBUTE(margin_dsu, MARGIN_DSU);
DEFINE_INT_ATTRIBUTE(margin_bci, MARGIN_BCI);
DEFINE_INT_ATTRIBUTE(margin_g3d, MARGIN_G3D);
DEFINE_INT_ATTRIBUTE(margin_g3dl2, MARGIN_G3DL2);
DEFINE_INT_ATTRIBUTE(margin_tpu, MARGIN_TPU);
DEFINE_INT_ATTRIBUTE(margin_intcam, MARGIN_INTCAM);
DEFINE_INT_ATTRIBUTE(margin_tnr, MARGIN_TNR);
DEFINE_INT_ATTRIBUTE(margin_cam, MARGIN_CAM);
DEFINE_INT_ATTRIBUTE(margin_mfc, MARGIN_MFC);
DEFINE_INT_ATTRIBUTE(margin_disp, MARGIN_DISP);
DEFINE_INT_ATTRIBUTE(margin_bw, MARGIN_BW);

int fvmap_init(void __iomem *sram_base)
{
	void __iomem *map_base;
	struct dentry *de = NULL;

	map_base = kzalloc(FVMAP_SIZE, GFP_KERNEL);

	fvmap_base = map_base;
	sram_fvmap_base = sram_base;
	pr_debug("%s:fvmap initialize %p\n", __func__, sram_base);

	margin_table_init();
	fvmap_copy_from_sram(map_base, sram_base);

	de = debugfs_create_dir("fvmap_margin", 0);
	if (IS_ERR_OR_NULL(de))
		return 0;

	debugfs_create_file("margin_mif", 0600, de, NULL, &margin_mif_fops);
	debugfs_create_file("margin_int", 0600, de, NULL, &margin_int_fops);
	debugfs_create_file("margin_lit", 0600, de, NULL, &margin_lit_fops);
	debugfs_create_file("margin_mid", 0600, de, NULL, &margin_mid_fops);
	debugfs_create_file("margin_big", 0600, de, NULL, &margin_big_fops);
	debugfs_create_file("margin_dsu", 0600, de, NULL, &margin_dsu_fops);
	debugfs_create_file("margin_bci", 0600, de, NULL, &margin_bci_fops);
	debugfs_create_file("margin_g3d", 0600, de, NULL, &margin_g3d_fops);
	debugfs_create_file("margin_g3dl2", 0600, de, NULL, &margin_g3dl2_fops);
	debugfs_create_file("margin_tpu", 0600, de, NULL, &margin_tpu_fops);
	debugfs_create_file("margin_intcam", 0600, de, NULL, &margin_intcam_fops);
	debugfs_create_file("margin_tnr", 0600, de, NULL, &margin_tnr_fops);
	debugfs_create_file("margin_cam", 0600, de, NULL, &margin_cam_fops);
	debugfs_create_file("margin_mfc", 0600, de, NULL, &margin_mfc_fops);
	debugfs_create_file("margin_disp", 0600, de, NULL, &margin_disp_fops);
	debugfs_create_file("margin_bw", 0600, de, NULL, &margin_bw_fops);

	return 0;
}
EXPORT_SYMBOL_GPL(fvmap_init);

MODULE_LICENSE("GPL");
