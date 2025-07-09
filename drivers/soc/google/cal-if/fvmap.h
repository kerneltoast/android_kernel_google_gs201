#ifndef __FVMAP_H__
#define __FVMAP_H__

/* FV(Frequency Voltage MAP) */
struct fvmap_header {
	u8 dvfs_type;
	u8 num_of_lv;
	u8 num_of_members;
	u8 num_of_pll;
	u8 num_of_mux;
	u8 num_of_div;
	u16 gearratio;
	u8 init_lv;
	u8 num_of_gate;
	u8 reserved[2];
	u16 block_addr[3];
	u16 o_members;
	u16 o_ratevolt;
	u16 o_tables;
};

struct clocks {
	u16 addr[0];
};

struct pll_header {
	u32 addr;
	u16 o_lock;
	u16 level;
	u32 pms[0];
};

struct rate_volt {
	u32 rate;
	u32 volt;
};

struct rate_volt_header {
	struct rate_volt table[0];
};

struct dvfs_table {
	u8 val[0];
};

#if IS_ENABLED(CONFIG_ACPM_DVFS)
extern int fvmap_init(void __iomem *sram_base);
extern int fvmap_get_voltage_table(unsigned int id, unsigned int *table);
#else
static inline int fvmap_init(void __iomem *sram_base)
{
	return 0;
}

static inline int fvmap_get_voltage_table(unsigned int id, unsigned int *table)
{
	return 0;
}
#endif
#endif
