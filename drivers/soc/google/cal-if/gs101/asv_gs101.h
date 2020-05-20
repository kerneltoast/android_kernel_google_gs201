#ifndef __ASV_GS101_H__
#define __ASV_GS101_H__

#include <linux/io.h>

#define ASV_TABLE_BASE	(0x10009000)
#define ID_TABLE_BASE	(0x10000000)

struct asv_tbl_info {
	unsigned bigcpu_asv_group:4;
	int      bigcpu_modify_group:4;
	unsigned ssa_reserved1:4;
	unsigned bigcpu_ssa0:4;
	unsigned littlecpu_asv_group:4;
	int      littlecpu_modify_group:4;
	unsigned ssa_reserved2:4;
	unsigned littlecpu_ssa0:4;

	unsigned g3d_asv_group:4;
	int      g3d_modify_group:4;
	unsigned ssa_reserved3:4;
	unsigned g3d_ssa0:4;
	unsigned mif_asv_group:4;
	int      mif_modify_group:4;
	unsigned ssa_reserved4:4;
	unsigned mif_ssa0:4;

	unsigned int_asv_group:4;
	int      int_modify_group:4;
	unsigned ssa_reserved5:4;
	unsigned int_ssa0:4;
	unsigned cam_disp_asv_group:4;
	int      cam_disp_modify_group:4;
	unsigned ssa_reserved6:4;
	unsigned cam_disp_ssa0:4;

	unsigned asv_table_version:7;
	unsigned asv_group_type:1;
	unsigned reserved_0:8;
	unsigned cp_asv_group:4;
	int      cp_modify_group:4;
	unsigned ssa_reserved7:4;
	unsigned cp_ssa0:4;

	unsigned big_vthr:2;
	unsigned big_delta:2;
	unsigned little_vthr:2;
	unsigned little_delta:2;
	unsigned g3d_vthr:2;
	unsigned g3d_delta:2;
	unsigned int_vthr:2;
	unsigned int_delta:2;
	unsigned mif_vthr:2;
	unsigned mif_delta:2;
	unsigned cp_vthr :2;
	unsigned cp_delta :2;
	unsigned midcpu_vthr:2;
	unsigned midcpu_delta:2;
	unsigned ssa_reserved8:4;

	unsigned midcpu_asv_group:4;
	int      midcpu_modify_group:4;
	unsigned midcpu_reserved9:4;
	unsigned midcpu_ssa0:4;
};

struct id_tbl_info {
	unsigned int reserved_0;

	unsigned int chip_id;

	unsigned reserved_2:10;
	unsigned product_line:2;
	unsigned reserved_2_1:4;
	unsigned char ids_bigcpu:8;
	unsigned char ids_g3d:8;

	unsigned char ids_others:8;	/* int,cam */
	unsigned asb_version:8;
	unsigned char ids_midcpu:8;
	unsigned char ids_litcpu:8;
	unsigned char ids_mif:8;
	unsigned char ids_cp:8;

	unsigned short sub_rev:4;
	unsigned short main_rev:4;
	unsigned reserved_6:8;
};

#define ASV_INFO_ADDR_CNT	(sizeof(struct asv_tbl_info) / 4)
#define ID_INFO_ADDR_CNT	(sizeof(struct id_tbl_info) / 4)

static struct asv_tbl_info asv_tbl;
static struct id_tbl_info id_tbl;

int asv_get_grp(unsigned int id)
{
	int grp = -1;

	return grp;
}

int asv_get_ids_info(unsigned int id)
{
	int ids = 0;

	return ids;
}

int asv_get_table_ver(void)
{
	return asv_tbl.asv_table_version;
}

void id_get_rev(unsigned int *main_rev, unsigned int *sub_rev)
{
	*main_rev = id_tbl.main_rev;
	*sub_rev =  id_tbl.sub_rev;
}

int id_get_product_line(void)
{
	return id_tbl.product_line;
}

int id_get_asb_ver(void)
{
	return id_tbl.asb_version;
}

int asv_table_init(void)
{
	return 0;
}
#endif
