#include <soc/google/pwrcal-env.h>
#include <soc/google/pmucal_common.h>
#include "pmucal_rae.h"

#define A_FEW_USECS 10		/* see Documentation/timers/timers-howto.rst */
#if IS_ENABLED(CONFIG_SOC_ZUMA)
#define PMU_ALIVE_BASE_ADDR	0x15460000
#endif
#if IS_ENABLED(CONFIG_SOC_GS101)
#define PMU_ALIVE_BASE_ADDR	0x17460000
#endif
#if IS_ENABLED(CONFIG_SOC_GS201)
#define PMU_ALIVE_BASE_ADDR	0x18060000
#endif

/**
 * A global index for pmucal_rae_handle_seq.
 * it should be helpful in ramdump.
 */
static unsigned int pmucal_rae_seq_idx;

/**
 *  pmucal_rae_phy2virt - converts a sequence's PA to VA described in pmucal_p2v_list.
 *			  exposed to PMUCAL common logics.(CPU/System/Local)
 *
 *  @seq: Sequence array to be converted.
 *  @seq_size: Array size of seq.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_rae_phy2virt(struct pmucal_seq *seq, unsigned int seq_size)
{
	int i, j;

	for (i = 0; i < seq_size; i++) {
		if (seq[i].access_type == PMUCAL_DELAY)
			continue;

		if (!seq[i].base_pa && !seq[i].cond_base_pa) {
			pr_err("%s %s: PA absent in seq element (idx:%d)\n",
					PMUCAL_PREFIX, __func__, i);
			return -ENOENT;
		}

		if (seq[i].base_pa) {
			for (j = 0; j < pmucal_p2v_list_size; j++)
				if (pmucal_p2v_list[j].pa == (phys_addr_t)seq[i].base_pa)
					break;

			if (j == pmucal_p2v_list_size) {
				pr_err("%s %s: there is no such PA in p2v_list (idx:%d)\n",
						PMUCAL_PREFIX, __func__, i);
				return -ENOENT;
			} else {
				seq[i].base_va = pmucal_p2v_list[j].va;
			}
		}
		if (seq[i].cond_base_pa) {
			for (j = 0; j < pmucal_p2v_list_size; j++)
				if (pmucal_p2v_list[j].pa == (phys_addr_t)seq[i].cond_base_pa)
					break;

			if (j == pmucal_p2v_list_size) {
				pr_err("%s %s: there is no such PA in p2v_list (idx:%d)\n",
						PMUCAL_PREFIX, __func__, i);
				return -ENOENT;
			} else {
				seq[i].cond_base_va = pmucal_p2v_list[j].va;
			}
		}
	}

	return 0;
}

static inline bool pmucal_rae_check_condition(struct pmucal_seq *seq)
{
	u32 reg;

	reg = readl(seq->cond_base_va + seq->cond_offset);
	reg &= seq->cond_mask;
	if (reg == seq->cond_value)
		return true;
	else
		return false;
}

static inline bool pmucal_rae_check_condition_inv(struct pmucal_seq *seq)
{
	u32 reg;

	reg = readl(seq->cond_base_va + seq->cond_offset);
	reg &= seq->cond_mask;
	if (reg == seq->cond_value)
		return false;
	else
		return true;
}

static inline bool pmucal_rae_check_value(struct pmucal_seq *seq)
{
	u32 reg;

	if (seq->access_type == PMUCAL_WRITE_WAIT)
		reg = readl(seq->base_va + seq->offset + 0x4);
	else
		reg = readl(seq->base_va + seq->offset);
	reg &= seq->mask;
	if (reg == seq->value)
		return true;
	else
		return false;
}

static int pmucal_rae_wait(struct pmucal_seq *seq, unsigned int idx)
{
	u32 timeout = 0;

	if (seq->cond_base_va && seq->cond_offset)
		if (!pmucal_rae_check_condition(seq))
			return 0;

	while (1) {
		if (pmucal_rae_check_value(seq))
			break;
		timeout++;
		udelay(1);
		if (timeout > 5000) {
			u32 reg;

			reg = readl(seq->base_va + seq->offset);
			pr_err("%s %s:timed out during wait. reg:%s (value:0x%x, seq_idx = %d)\n",
						PMUCAL_PREFIX, __func__, seq->sfr_name, reg, idx);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static inline void pmucal_rae_read(struct pmucal_seq *seq)
{
	u32 reg;

	reg = readl(seq->base_va + seq->offset);
	seq->value = (reg & seq->mask);
}

static void pmucal_write_reg(phys_addr_t base_pa, void __iomem *base_va, u32 offset, u32 val)
{
	if ((base_pa >> 16) == (PMU_ALIVE_BASE_ADDR >> 16))
		set_priv_reg(base_pa + offset, val);
	else
		writel(val, base_va + offset);
}

static inline void pmucal_rae_write(struct pmucal_seq *seq)
{
	if (seq->mask == U32_MAX)
		pmucal_write_reg(seq->base_pa, seq->base_va, seq->offset, seq->value);
	else {
		u32 reg;

		reg = readl(seq->base_va + seq->offset);
		reg = (reg & ~seq->mask) | (seq->value & seq->mask);

		pmucal_write_reg(seq->base_pa, seq->base_va, seq->offset, reg);
	}
}

/* Atomic operation for PMU_ALIVE registers. (offset 0~0x3FFF)
   When the targer register can be accessed by multiple masters,
   This functions should be used. */
static inline void pmucal_set_bit_atomic(struct pmucal_seq *seq)
{
	if (seq->offset > 0x3fff)
		return;

	pmucal_write_reg(seq->base_pa, seq->base_va, (seq->offset | 0xc000), seq->value);
}

static inline void pmucal_clr_bit_atomic(struct pmucal_seq *seq)
{
	if (seq->offset > 0x3fff)
		return;

	pmucal_write_reg(seq->base_pa, seq->base_va, (seq->offset | 0x8000), seq->value);
}

static int pmucal_rae_write_retry(struct pmucal_seq *seq, bool inversion, unsigned int idx)
{
	u32 timeout = 0, count = 0, i = 0;
	bool retry = true;

	while (1) {
		if (inversion)
			retry = pmucal_rae_check_condition_inv(seq);
		else
			retry = pmucal_rae_check_condition(seq);
		if (!retry)
			break;

		for (i = 0; i < 10; i++)
			pmucal_rae_write(seq);
		count++;
		for (i = 0; i < count; i++)
			pmucal_rae_write(seq);

		timeout++;
		udelay(1);
		if (timeout > 2000) {
			u32 reg;

			reg = readl(seq->cond_base_va + seq->cond_offset);
			pr_err("%s %s:timed out during write-retry. (value:0x%x, seq_idx = %d)\n",
					PMUCAL_PREFIX, __func__, reg, idx);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static inline void pmucal_clr_pend(struct pmucal_seq *seq)
{
	u32 reg;

	reg = readl(seq->cond_base_va + seq->cond_offset) & seq->cond_mask;
	pmucal_write_reg(seq->base_pa, seq->base_va, seq->offset, reg & seq->mask);
}

void pmucal_rae_save_seq(struct pmucal_seq *seq, unsigned int seq_size)
{
	int i;

	for (i = 0; i < seq_size; i++) {
		switch (seq[i].access_type) {
		case PMUCAL_COND_SAVE_RESTORE:
			if (!pmucal_rae_check_condition(&seq[i]))
				break;
			fallthrough;
		case PMUCAL_SAVE_RESTORE:
			pmucal_rae_read(&seq[i]);
			seq[i].need_restore = true;
			break;
		case PMUCAL_READ:
			pmucal_rae_read(&seq[i]);
			seq[i].need_restore = false;
			break;
		case PMUCAL_COND_READ:
			if (pmucal_rae_check_condition(&seq[i]))
				pmucal_rae_read(&seq[i]);
			break;
		case PMUCAL_CHECK_SKIP:
		case PMUCAL_COND_CHECK_SKIP:
		case PMUCAL_WAIT:
		case PMUCAL_WAIT_TWO:
			break;
		case PMUCAL_CLEAR_PEND:
			pmucal_clr_pend(&seq[i]);
			break;
		default:
			break;
		}
	}
}

int pmucal_rae_restore_seq(struct pmucal_seq *seq, unsigned int seq_size)
{
	int i, ret;

	for (i = 0; i < seq_size; i++) {
		if (seq[i].need_skip) {
			seq[i].need_skip = false;
			continue;
		}

		switch (seq[i].access_type) {
		case PMUCAL_COND_SAVE_RESTORE:
			if (!pmucal_rae_check_condition(&seq[i]))
				break;
			fallthrough;
		case PMUCAL_SAVE_RESTORE:
			if (seq[i].need_restore) {
				seq[i].need_restore = false;
				pmucal_rae_write(&seq[i]);
			}
			break;
		case PMUCAL_CHECK_SKIP:
			if (pmucal_rae_check_value(&seq[i])) {
				if ((i+1) < seq_size)
					seq[i+1].need_skip = true;
			} else {
				if ((i+1) < seq_size)
					seq[i+1].need_skip = false;
			}
			break;
		case PMUCAL_COND_CHECK_SKIP:
			if (pmucal_rae_check_condition(&seq[i])) {
				if (pmucal_rae_check_value(&seq[i])) {
					if ((i+1) < seq_size)
						seq[i+1].need_skip = true;
				} else {
					if ((i+1) < seq_size)
						seq[i+1].need_skip = false;
				}
			} else {
				if ((i+1) < seq_size)
					seq[i+1].need_skip = true;
			}
			break;
		case PMUCAL_WAIT:
		case PMUCAL_WAIT_TWO:
			ret = pmucal_rae_wait(&seq[i], i);
			if (ret)
				return ret;
			break;
		case PMUCAL_WRITE:
			pmucal_rae_write(&seq[i]);
			break;
		case PMUCAL_CLEAR_PEND:
			pmucal_clr_pend(&seq[i]);
			break;
		default:
			break;
		}
	}

	return 0;
}

/**
 *  pmucal_rae_handle_seq - handles a sequence array based on each element's access_type.
 *			    exposed to PMUCAL common logics.(CPU/System/Local)
 *
 *  @seq: Sequence array to be handled.
 *  @seq_size: Array size of seq.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_rae_handle_seq(struct pmucal_seq *seq, unsigned int seq_size)
{
	int ret, i;

	for (i = 0; i < seq_size; i++) {
		pmucal_rae_seq_idx = i;
		if (seq[i].need_skip) {
			seq[i].need_skip = false;
			continue;
		}

		switch (seq[i].access_type) {
		case PMUCAL_READ:
			pmucal_rae_read(&seq[i]);
			break;
		case PMUCAL_WRITE:
			pmucal_rae_write(&seq[i]);
			break;
		case PMUCAL_COND_READ:
			if (pmucal_rae_check_condition(&seq[i]))
				pmucal_rae_read(&seq[i]);
			break;
		case PMUCAL_COND_WRITE:
			if (pmucal_rae_check_condition(&seq[i]))
				pmucal_rae_write(&seq[i]);
			break;
		case PMUCAL_CHECK_SKIP:
			if (pmucal_rae_check_value(&seq[i])) {
				if ((i+1) < seq_size)
					seq[i+1].need_skip = true;
			} else {
				if ((i+1) < seq_size)
					seq[i+1].need_skip = false;
			}
			break;
		case PMUCAL_COND_CHECK_SKIP:
			if (pmucal_rae_check_condition(&seq[i])) {
				if (pmucal_rae_check_value(&seq[i])) {
					if ((i+1) < seq_size)
						seq[i+1].need_skip = true;
				} else {
					if ((i+1) < seq_size)
						seq[i+1].need_skip = false;
				}
			} else {
				if ((i+1) < seq_size)
					seq[i+1].need_skip = true;
			}
			break;
		case PMUCAL_WAIT:
		case PMUCAL_WAIT_TWO:
			ret = pmucal_rae_wait(&seq[i], i);
			if (ret)
				return ret;
			break;
		case PMUCAL_WRITE_WAIT:
			pmucal_rae_write(&seq[i]);
			ret = pmucal_rae_wait(&seq[i], i);
			if (ret)
				return ret;
			break;
		case PMUCAL_WRITE_RETRY:
			ret = pmucal_rae_write_retry(&seq[i], false, i);
			if (ret)
				return ret;
			break;
		case PMUCAL_WRITE_RETRY_INV:
			ret = pmucal_rae_write_retry(&seq[i], true, i);
			if (ret)
				return ret;
			break;
		case PMUCAL_WRITE_RETURN:
			pmucal_rae_write(&seq[i]);
			return 0;
		case PMUCAL_DELAY:
			if (seq[i].value <= A_FEW_USECS)
				udelay(seq[i].value);
			else
				usleep_range(seq[i].value, seq[i].value);
			break;
		case PMUCAL_SET_BIT_ATOMIC:
			pmucal_set_bit_atomic(&seq[i]);
			break;
		case PMUCAL_CLR_BIT_ATOMIC:
			pmucal_clr_bit_atomic(&seq[i]);
			break;
		case PMUCAL_CLEAR_PEND:
			pmucal_clr_pend(&seq[i]);
			break;
		default:
			pr_err("%s %s:invalid PMUCAL access type\n", PMUCAL_PREFIX, __func__);
			return -EINVAL;
		}
	}

	return 0;
}

/**
 *  pmucal_rae_handle_seq - handles a sequence array based on each element's access_type.
 *			    exposed to PMUCAL common logics.(CP)
 *
 *  @seq: Sequence array to be handled.
 *  @seq_size: Array size of seq.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */

#if IS_ENABLED(CONFIG_CP_PMUCAL)
static unsigned int pmucal_rae_cp_seq_idx;
int pmucal_rae_handle_cp_seq(struct pmucal_seq *seq, unsigned int seq_size)
{
	int ret, i;

	for (i = 0; i < seq_size; i++) {
		pmucal_rae_cp_seq_idx = i;

		switch (seq[i].access_type) {
		case PMUCAL_READ:
			if (unlikely(pmucal_is_cp_smc_regs(&seq[i])))
				pmucal_smc_read(&seq[i], 1);
			else {
				pmucal_rae_read(&seq[i]);
				pr_info("%s%s\t%s = 0x%08x\n", PMUCAL_PREFIX, "raw_read", seq[i].sfr_name,
						readl(seq[i].base_va + seq[i].offset));
			}
			break;
		case PMUCAL_WRITE:
			if (unlikely(pmucal_is_cp_smc_regs(&seq[i])))
				pmucal_smc_write(&seq[i]);
			else {
				u32 reg;

				reg = readl(seq[i].base_va + seq[i].offset);
				reg = (reg & ~seq[i].mask) | seq[i].value;
				pr_info("%s%s\t%s = 0x%08x\n", PMUCAL_PREFIX, "raw_write", seq[i].sfr_name, reg);
				pmucal_rae_write(&seq[i]);
				pr_info("%s%s\t%s = 0x%08x\n", PMUCAL_PREFIX, "raw_read", seq[i].sfr_name,
						readl(seq[i].base_va + seq[i].offset));
			}
			break;
		case PMUCAL_COND_READ:
			if (pmucal_rae_check_condition(&seq[i]))
				pmucal_rae_read(&seq[i]);
			break;
		case PMUCAL_COND_WRITE:
			if (pmucal_rae_check_condition(&seq[i]))
				pmucal_rae_write(&seq[i]);
			break;
		case PMUCAL_WAIT:
		case PMUCAL_WAIT_TWO:
			ret = pmucal_rae_wait(&seq[i], i);
			if (ret)
				return ret;
			pr_info("%s%s\t%s = 0x%08x(expected = 0x%08x)\n", PMUCAL_PREFIX, "raw_read", seq[i].sfr_name,
					readl(seq[i].base_va + seq[i].offset) & seq[i].mask, seq[i].value);
			break;
		case PMUCAL_DELAY:
			udelay(seq[i].value);
			break;
		default:
			pr_err("%s %s:invalid PMUCAL access type\n", PMUCAL_PREFIX, __func__);
			return -EINVAL;
		}
	}

	return 0;
}
#endif

/**
 *  pmucal_rae_init - Init function of PMUCAL Register Access Engine.
 *		      exposed to PWRCAL interface.
 *
 *  Returns 0 on success. Otherwise, negative error code.
 */
int pmucal_rae_init(void)
{
	int i;

	if (!pmucal_p2v_list_size) {
		pr_err("%s %s: there is no p2vmap. aborting init...\n",
				PMUCAL_PREFIX, __func__);
		return -ENOENT;
	}

	for (i = 0; i < pmucal_p2v_list_size; i++) {
#ifdef PWRCAL_TARGET_LINUX
		pmucal_p2v_list[i].va = ioremap(pmucal_p2v_list[i].pa, SZ_64K);
		if (pmucal_p2v_list[i].va == NULL) {
			pr_err("%s %s:ioremap failed.\n", PMUCAL_PREFIX, __func__);
			return -ENOMEM;
		}
#else
		pmucal_p2v_list[i].va = (void *)(pmucal_p2v_list[i].pa);
#endif
	}

	return 0;
}
