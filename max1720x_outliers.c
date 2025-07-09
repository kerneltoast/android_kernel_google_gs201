/*
 * Fix for capacity drift for max1720x and for max77759
 *
 * Copyright (C) 2021 Google LLC
 */


#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/time.h>
#include "gbms_power_supply.h"
#include "google_bms.h"
#include "max1720x_battery.h"

/* delay between attempts after a write failure */
#define MAX17201_FIXUP_UPDATE_DELAY_MS	10

/* max 110% of design cap, max 60% of design cap for age model */
#define MAX1720x_CC_UPPER_BOUND	110
#define MAX1720x_CC_LOWER_BOUND	60

#define ALGO_VER_CHECK(algo_ver) ((algo_ver) == MAX1720X_DA_VER_MWA2 || \
				  (algo_ver) == MAX1720X_DA_VER_NONE)


/* registers accessed from the workaround */
enum {
	MAX17X0X_REPCAP		= 0x05,
	MAX17X0X_REPSOC		= 0x06,
	MAX17X0X_MIXCAP 	= 0x0F,
	MAX17X0X_FULLCAPNOM	= 0x23,
	MAX17X0X_FULLCAPREP	= 0x35,
	MAX17X0X_RCOMP0		= 0x38,	/* 16 bits in MW A1+ */
	MAX17X0X_TEMPCO		= 0x39,
	MAX17X0X_DQACC		= 0x45,
	MAX17X0X_DPACC		= 0x46,
	MAX17X0X_VFSOC		= 0xFF,
};

/* 1 = success, 0 compare error, < 0 error */
static int max1720x_update_compare(struct maxfg_regmap *map, int reg,
				   u16 data0, u16 data1)
{
	u16 data[2] = {data0, data1};
	int ret;

	ret = regmap_raw_write(map->regmap, reg, data, sizeof(data));
	if (ret < 0)
		return -EIO;

	msleep(2);

	ret = regmap_raw_read(map->regmap, reg, data, sizeof(data));
	if (ret < 0)
		return -EIO;

	return (data[0] == data0) && (data[1] == data1);
}

/* 0 not updated, 1 updated, doesn't return IO errors */
static int max1720x_update_capacity(struct maxfg_regmap *map,
				    u16 mixcap, u16 repcap,
				    u16 fullcaprep)
{
	u16 temp;
	int err;

	err = REGMAP_WRITE(map, MAX17X0X_MIXCAP, mixcap);
	if (err == 0)
		err = REGMAP_READ(map, MAX17X0X_MIXCAP, &temp);
	if (err < 0 || temp != mixcap)
		return 0;

	/* RepCap and FullCapRep must be updated together */
	err = REGMAP_WRITE(map, MAX17X0X_REPCAP, repcap);
	if (err == 0)
		err = REGMAP_READ(map, MAX17X0X_REPCAP, &temp);
	if (err < 0 || temp != repcap)
		return 0;

	err = REGMAP_WRITE(map, MAX17X0X_FULLCAPREP, fullcaprep);
	if (err == 0)
		err = REGMAP_READ(map, MAX17X0X_FULLCAPREP, &temp);
	if (err < 0 || temp != fullcaprep)
		return 0;

	return 1;
}


/* return fullcapnom if no changes */
static int max1720x_capacity_check(int fullcapnom, int cycle_count,
				   const struct max1720x_drift_data *ddata)
{
	const int fcn = fullcapnom;
	int refcap, upper_bound, lower_bound;

	if (!ddata->design_capacity || cycle_count < ddata->cycle_stable)
		return fullcapnom;

	/* apply fade model to design capacity, bound to absolute min/max */
	refcap = ddata->design_capacity;
	if (ddata->cycle_fade) {
		const int base_capacity = ddata->design_capacity;

		lower_bound = (base_capacity * MAX1720x_CC_LOWER_BOUND) / 100;
		upper_bound = (base_capacity * MAX1720x_CC_UPPER_BOUND) / 100;

		refcap -= (base_capacity * cycle_count) / ddata->cycle_fade;
		if (refcap < lower_bound)
			refcap = lower_bound;
		else if (refcap > upper_bound)
			refcap = upper_bound;

		pr_debug("refcap@%d=%d abs_min=%d abs_max=%d\n",
			cycle_count, refcap, lower_bound, upper_bound);
	}

	/* bound FCN max to the target age. Will not operate on devices
	 * that underestimate capacity.
	 * NOTE: range decrease with cycle count
	 */
	upper_bound = (refcap * (100 + ddata->cycle_band)) / 100;
	if (fullcapnom > upper_bound)
		fullcapnom = upper_bound;

	pr_debug("fullcapnom=%d->%d upper_bound=%d\n",
		 fcn, fullcapnom, upper_bound);

	return fullcapnom;
}

/*
 * dQACC @0x45 battery charge between relaxation points.
 * dPACC @0x46 change in battery state of charge between relaxation points.
 */
/* 1 changed, 0 no changes, < 0 error*/

int max1720x_fixup_dxacc(struct max1720x_drift_data *ddata,
			 struct maxfg_regmap *map,
			 int cycle_count,
			 int plugged,
			 int lsb)
{
	u16 temp, vfsoc = 0, repsoc = 0, fullcapnom, mixcap, repcap, fcrep;
	int capacity, new_capacity;
	int dpacc, dqacc;
	int err, loops;

	if (ddata->design_capacity <= 0 || ALGO_VER_CHECK(ddata->algo_ver))
		return 0;

	err = REGMAP_READ(map, MAX17X0X_FULLCAPNOM, &fullcapnom);
	if (err < 0)
		return err;

	capacity = reg_to_micro_amp_h(fullcapnom, ddata->rsense, lsb) / 1000;

	/* return the expected FCN, done if the same of th eold one */
	new_capacity = max1720x_capacity_check(capacity, cycle_count, ddata);
	if (new_capacity == capacity)
		return 0;

	/* You can use a ratio of dPAcc = 0x190 ( = 25%) with dQACC with 64 mAh
	 * LSB. Can make dPACC larger (ex 0xC80, 200%) and give dQAcc a smaller
	 * LSB (FullCapNom >> 4, LSB = 8 mAh). The equation can be written a
	 * (DesignCap * scale) >> 4 when writing the age-compensated value.
	 */

	/* TODO: fix for TaskPeriod == 351ms b/177099997 */
	fcrep = micro_amp_h_to_reg(new_capacity * 1000, ddata->rsense, lsb);
	dqacc = fcrep >> 4;
	dpacc = 0xc80;

	/* will not update if dqacc/dpacc is already in line */
	err = REGMAP_READ(map, MAX17X0X_DQACC, &temp);
	if (err == 0 && temp == dqacc) {
		err = REGMAP_READ(map, MAX17X0X_DPACC, &temp);
		if (err == 0 && temp == dpacc) {
			pr_debug("Fix capacity: same dqacc=0x%x dpacc=0x%x\n",
				 dqacc, dpacc);
			return 0;
		}
	}

	/* fast convergence, avoid ghost drain */
	err = REGMAP_READ(map, MAX17X0X_VFSOC, &vfsoc);
	if (err == 0)
		err = REGMAP_READ(map, MAX17X0X_REPSOC, &repsoc);
	if (err < 0) {
		pr_warn("Fix capacity: fcn=%d new=%d vfsoc=0x%x repsoc=0x%x (%d)\n",
			fullcapnom, new_capacity, vfsoc, repsoc, err);
		return err;
	}

	/* vfsoc/repsoc perc, lsb = 1/256 */
	mixcap = (((u32)vfsoc) * fcrep) / 25600;
	repcap = (((u32)repsoc) * fcrep) / 25600;

	for (loops = 0; loops < 3; loops++) {
		err = max1720x_update_capacity(map, mixcap, repcap, fcrep);
		if (err == -EIO || err > 0)
			break;

		/* arbitrary delay between attempts */
		msleep(MAX17201_FIXUP_UPDATE_DELAY_MS);
	}

	pr_info("Fix capacity: fixing caps retries=%d (%d)\n", loops, err);

	/* 3 loops suggested from vendor */
	for (loops = 0; loops < 3; loops++) {
		err = max1720x_update_compare(map, MAX17X0X_DQACC, dqacc, dpacc);
		if (err == -EIO || err > 0)
			break;

		/* arbitrary delay between attempts */
		msleep(MAX17201_FIXUP_UPDATE_DELAY_MS);
	}

	pr_info("Fix capacity: %d->%d, vfsoc=0x%x repsoc=0x%x fcrep=0x%x mixcap=0x%x repcap=0x%x ddqacc=0x%x dpacc=0x%x retries=%d (%d)\n",
		fullcapnom, new_capacity, vfsoc, repsoc, fcrep, mixcap, repcap,
		dqacc, dpacc, loops, err);

	/* TODO:  b/144630261 fix Google Capacity */

	return (loops == 3) ? -ETIMEDOUT : err;
}

/* Tempco and rcomp0 must remain within the following limits to avoid capacity
 * drift. Note that tempco has a hi and low limit (one byte).
 * (RCOMP0 > INI_RCOMP0 * 1.4) or (RCOMP0 < 0.7 * INI_RCOMP0)
 * (TempCoHot >INI_TempCoHot * 1.4) or (TempCoHot < 0.7 * INI_TempCoHot)
 * (TempCoCold >INI_TempCoCold * 1.4) or (TempCoCold < 0.7 * INI_TempCoCold)
 */
#define MAXIM_RCOMP0_LIM_HI	140
#define MAXIM_RCOMP0_LIM_LO	70
#define MAXIM_TEMPCO_LIM_HI	140
#define MAXIM_TEMPCO_LIM_LO	70

static u8 comp_check(int value, int scale, int lim_low, int lim_high)
{
	if ((value * scale) < lim_low) {
		value = lim_low / scale;
	} else if ((value * scale) > lim_high) {
		value = lim_high / scale;
		if (value > 0xff)
			value = 0xff;
	}

	return value;
}

/* NOT OK For MW A1+ */
static u16 max1720x_check_rcomp0(const struct max1720x_drift_data *ddata,
				      u16 rcomp0)
{
	const int ini_rcomp0_lob = ddata->ini_rcomp0 & 0xff;
	int rcomp0_lob = rcomp0 & 0xff;

	rcomp0_lob = comp_check(rcomp0_lob, 100,
				ini_rcomp0_lob * MAXIM_RCOMP0_LIM_LO,
				ini_rcomp0_lob * MAXIM_RCOMP0_LIM_HI);

	pr_debug("rcomp0=%x rcomp0_lob=%x->%x min=%x max=%x\n",
		 rcomp0, (rcomp0 & 0xff), rcomp0_lob,
		 (ini_rcomp0_lob * MAXIM_RCOMP0_LIM_LO) / 100,
		 (ini_rcomp0_lob * MAXIM_RCOMP0_LIM_HI) / 100);

	/* always write 0 to the high byte */
	return rcomp0_lob & 0xff;
}

/* MW A1+ */
static u16 max1720x_check_mw_rcomp0(const struct max1720x_drift_data *ddata,
				    u16 rcomp0)
{
	const int lim_low = ddata->ini_rcomp0 * MAXIM_RCOMP0_LIM_LO;
	const int lim_high = ddata->ini_rcomp0 * MAXIM_RCOMP0_LIM_HI;
	const int scale = 100;
	int value = rcomp0;

	if ((value * scale) < lim_low) {
		value = lim_low / scale;
	} else if ((value * scale) > lim_high) {
		value = lim_high / scale;
		if (value > 0xffff)
			value = 0xffff;
	}

	return value;
}

/* 0 no changes, >0 changes */
static bool max1720x_comp_check(u16 *new_rcomp0, u16 *new_tempco,
				const struct max1720x_drift_data *ddata)
{
	const u16 rcomp0 = *new_rcomp0;
	const u16 tempco = *new_tempco;
	const int ini_tc_lob = ddata->ini_tempco & 0xff;
	const int ini_tc_hib = (ddata->ini_tempco >> 8) & 0xff;
	int tc_hib = (tempco >> 8) & 0xff;
	int tc_lob = tempco & 0xff;
	bool fix_rcomp0 = false;

	if (ddata->algo_ver == MAX1720X_DA_VER_ORIG) {
		*new_rcomp0 = max1720x_check_rcomp0(ddata, rcomp0);

		fix_rcomp0 = (rcomp0 & 0xff) != *new_rcomp0;
	} else if (ddata->algo_ver == MAX1720X_DA_VER_MWA1) {
		if (*new_rcomp0 < 0x100)
			*new_rcomp0 = *new_rcomp0 << 4;
		*new_rcomp0 = max1720x_check_mw_rcomp0(ddata, *new_rcomp0);

		fix_rcomp0 = rcomp0 != *new_rcomp0;
	}

	tc_lob = comp_check(tc_lob, 100, ini_tc_lob * MAXIM_TEMPCO_LIM_LO,
			    ini_tc_lob * MAXIM_TEMPCO_LIM_HI);
	tc_hib = comp_check(tc_hib, 100, ini_tc_hib * MAXIM_TEMPCO_LIM_LO,
			    ini_tc_hib * MAXIM_TEMPCO_LIM_HI);

	pr_debug("tempco=%x tempco_lob=%x->%x min=%x max=%x, tempco_hib=%x->%x min=%x max=%x\n",
		 tempco, tempco & 0xff, tc_lob,
		 (ini_tc_lob * MAXIM_TEMPCO_LIM_LO) / 100,
		 (ini_tc_lob * MAXIM_TEMPCO_LIM_HI) / 100,
		 (tempco >> 8) & 0xff, tc_hib,
		 (ini_tc_hib * MAXIM_TEMPCO_LIM_LO) / 100,
		 (ini_tc_hib * MAXIM_TEMPCO_LIM_HI) / 100);

	*new_tempco = (tc_hib << 8) | (tc_lob);

	/* fix for MW A1+ */
	return fix_rcomp0 || (tempco != *new_tempco);
}


/* fix rcomp0 and tempco */
int max1720x_fixup_comp(struct max1720x_drift_data *ddata,
			struct maxfg_regmap *map,
			int plugged)
{
	u16 new_rcomp0, new_tempco, data[2] = { 0 };
	int err, loops;

	if (ddata->ini_rcomp0 == -1 || ddata->ini_tempco == -1 ||
	    ALGO_VER_CHECK(ddata->algo_ver))
		return 0;

	err = regmap_raw_read(map->regmap, MAX17X0X_RCOMP0, data,
			      sizeof(data));
	if (err < 0)
		return -EIO;

	new_rcomp0 = data[0];
	new_tempco = data[1];

	err = max1720x_comp_check(&new_rcomp0, &new_tempco, ddata);
	pr_debug("rcomp0=0x%x tempco=0x%x (%d)\n", data[0], data[1], err);
	if (err <= 0)
		return err;

	/* 3 loops suggested from vendor */
	for (loops = 0; loops < 3; loops++) {

		err = max1720x_update_compare(map, MAX17X0X_RCOMP0,
					      new_rcomp0, new_tempco);
		if (err == -EIO || err > 0)
			break;

		/* arbitrary delay between attempts */
		msleep(MAX17201_FIXUP_UPDATE_DELAY_MS);
	}

	pr_info("Fix rcomp0=0x%x->0x%x tempco:0x%x->0x%x, retries=%d, (%d)\n",
		data[0], new_rcomp0, data[1], new_tempco, loops, err);

	return (loops == 3) ? -ETIMEDOUT : err;
}
