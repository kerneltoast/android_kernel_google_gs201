/* linux/include/soc/google/exynos-dm.h
 *
 * Copyright (C) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * EXYNOS5 - Header file for exynos DVFS Manager support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __EXYNOS_DM_H
#define __EXYNOS_DM_H

#define EXYNOS_DM_MODULE_NAME		"exynos-dm"
#define EXYNOS_DM_TYPE_NAME_LEN		16
#define EXYNOS_DM_ATTR_NAME_LEN		(EXYNOS_DM_TYPE_NAME_LEN + 12)

#define EXYNOS_DM_RELATION_L		0
#define EXYNOS_DM_RELATION_H		1

enum exynos_constraint_type {
	CONSTRAINT_MIN = 0,
	CONSTRAINT_MAX,
	CONSTRAINT_END
};

enum dvfs_direction {
	DOWN = 0,
	UP,
	DIRECTION_END
};

struct exynos_dm_freq {
	u32				driver_freq;
	u32				constraint_freq;
};

struct exynos_dm_attrs {
	struct device_attribute attr;
	char name[EXYNOS_DM_ATTR_NAME_LEN];
};

struct exynos_dm_constraint {
	int					dm_driver;
	int					dm_constraint;

	struct list_head	driver_domain;
	struct list_head	constraint_domain;

	/* check constraint table by hw guide */
	bool				guidance;
	u32				table_length;

	enum exynos_constraint_type	constraint_type;
	char				dm_type_name[EXYNOS_DM_TYPE_NAME_LEN];
	struct exynos_dm_freq		*freq_table;

	u32					const_freq;
	u32					gov_freq;

	struct exynos_dm_constraint	*sub_constraint;
};

struct exynos_dm_data {
	/* use for DVFS domain available */
	bool				available;
#if IS_ENABLED(CONFIG_GS_ACPM)
	bool				policy_use;
#endif
	int		dm_type;
	char				dm_type_name[EXYNOS_DM_TYPE_NAME_LEN];

	int			my_order;
	int			indegree;

	u32			cur_freq;
	u32			next_target_freq;
	u32			governor_freq;
	u32			gov_min;
	u32			policy_min;
	u32			policy_max;
	u32			const_min;
	u32			const_max;

	int				(*freq_scaler)(int dm_type,
							void *devdata,
							u32 target_freq,
							unsigned int relation);

	struct list_head		min_constraints;
	struct list_head		max_constraints;
	struct list_head		min_drivers;
	struct list_head		max_drivers;

#if IS_ENABLED(CONFIG_GS_ACPM)
	u32				cal_id;
#endif

	void				*devdata;

	struct exynos_dm_attrs		dm_policy_attr;
	struct exynos_dm_attrs		constraint_table_attr;
};

struct exynos_dm_device {
	struct device			*dev;
	struct mutex			lock;
	int				domain_count;
	int				constraint_domain_count;
	int				*domain_order;
	struct exynos_dm_data		*dm_data;
};

/* Returns @cpu's constraint freq for cl0 */
unsigned int exynos_dm_constraint_freq(int cpu, unsigned int freq);

/* External Function call */
#if IS_ENABLED(CONFIG_EXYNOS_DVFS_MANAGER)
extern int exynos_dm_data_init(int dm_type,
				void *data,
				u32 min_freq,
				u32 max_freq,
				u32 cur_freq);
extern int register_exynos_dm_constraint_table(int dm_type,
				struct exynos_dm_constraint *constraint);
extern int unregister_exynos_dm_constraint_table(int dm_type,
				struct exynos_dm_constraint *constraint);
extern int register_exynos_dm_freq_scaler(int dm_type,
				int (*scaler_func)(int dm_type,
				void *devdata,
				u32 target_freq,
				unsigned int relation));
extern int unregister_exynos_dm_freq_scaler(int dm_type);
extern int policy_update_call_to_DM(int dm_type,
				u32 min_freq,
				u32 max_freq);
extern int DM_CALL(int dm_type,
				unsigned long *target_freq);
#else
static inline
int exynos_dm_data_init(int dm_type,
			void *data,
			u32 min_freq,
			u32 max_freq,
			u32 cur_freq)
{
	return 0;
}
static inline
int register_exynos_dm_constraint_table(int dm_type,
				struct exynos_dm_constraint *constraint)
{
	return 0;
}
static inline
int unregister_exynos_dm_constraint_table(int dm_type,
				struct exynos_dm_constraint *constraint)
{
	return 0;
}
static inline
int register_exynos_dm_freq_scaler(int dm_type,
			int (*scaler_func)(int dm_type,
			void *devdata,
			u32 target_freq,
			unsigned int relation))
{
	return 0;
}
static inline
int unregister_exynos_dm_freq_scaler(int dm_type)
{
	return 0;
}
static inline
int policy_update_call_to_DM(int dm_type,
			u32 min_freq,
			u32 max_freq)
{
	return 0;
}
static inline
int DM_CALL(int dm_type,
			unsigned long *target_freq)
{
	return 0;
}
#endif

#endif /* __EXYNOS_DM_H */
