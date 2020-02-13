/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#ifndef _SYSTEM_REGS_H
#define _SYSTEM_REGS_H

struct armv8_a_errselr_el1_field {
	u64 sel		:16;
	u64 res0	:48;
};

struct armv8_a_erridr_el1_field {
	u64 num		:16;
	u64 res0	:48;
};

struct armv8_a_erxstatus_el1_field {
	u64 serr	:8;
	u64 ierr	:8;
	u64 res0	:4;
	u64 uet		:2;
	u64 pn		:1;
	u64 de		:1;
	u64 ce		:2;
	u64 mv		:1;
	u64 of		:1;
	u64 er		:1;
	u64 ue		:1;
	u64 valid	:1;
	u64 av		:1;
	u64 res1	:32;
};

struct armv8_a_erxmisc0_el1_field {
	u64 reg;
};

struct armv8_a_erxmisc1_el1_field {
	u64 reg;
};

struct armv8_a_erxaddr_el1_field {
	u64 reg;
};

#define armv8_a_system_register(name)			\
struct armv8_a_##name {					\
	union {						\
		struct armv8_a_##name##_field field;	\
		u64 reg;				\
	};						\
}

armv8_a_system_register(errselr_el1);
armv8_a_system_register(erridr_el1);
armv8_a_system_register(erxstatus_el1);
armv8_a_system_register(erxmisc0_el1);
armv8_a_system_register(erxmisc1_el1);
armv8_a_system_register(erxaddr_el1);
#endif
