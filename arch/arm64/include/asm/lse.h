/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_LSE_H
#define __ASM_LSE_H

#include <asm/atomic_ll_sc.h>

#if defined(CONFIG_ARM64_LSE_ATOMICS) && !defined(BUILD_FIPS140_KO)

#define __LSE_PREAMBLE	".arch_extension lse\n"

#include <linux/compiler_types.h>
#include <linux/export.h>
#include <linux/stringify.h>
#include <asm/alternative.h>
#include <asm/alternative-macros.h>
#include <asm/atomic_lse.h>
#include <asm/cpucaps.h>

/* Always use LSE atomics */
#define system_uses_lse_atomics() true

#define __lse_ll_sc_body(op, ...)					\
({									\
	system_uses_lse_atomics() ?					\
		__lse_##op(__VA_ARGS__) :				\
		__ll_sc_##op(__VA_ARGS__);				\
})

/* Always use LSE atomics */
#define ARM64_LSE_ATOMIC_INSN(lse)		__LSE_PREAMBLE lse

#else	/* CONFIG_ARM64_LSE_ATOMICS */

static inline bool system_uses_lse_atomics(void) { return false; }

#define __lse_ll_sc_body(op, ...)		__ll_sc_##op(__VA_ARGS__)

#define ARM64_LSE_ATOMIC_INSN(llsc, lse)	llsc

#endif	/* CONFIG_ARM64_LSE_ATOMICS */
#endif	/* __ASM_LSE_H */
