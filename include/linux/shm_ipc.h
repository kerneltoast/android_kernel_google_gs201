/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2014-2019, Samsung Electronics.
 *
 */

#ifndef __SHMEM_IPC_H__
#define __SHMEM_IPC_H__

#define MAX_CP_NUM	2

#if IS_ENABLED(CONFIG_SHM_IPC)
extern int cp_shmem_get_mem_map_on_cp_flag(u32 cp_num);
extern void __iomem *cp_shmem_get_region(u32 cp, u32 idx);
extern void __iomem *cp_shmem_get_nc_region(unsigned long base, u32 size);
extern void cp_shmem_release_region(u32 cp, u32 idx);
extern void cp_shmem_release_rmem(u32 cp, u32 idx);
extern unsigned long cp_shmem_get_base(u32 cp, u32 idx);
extern u32 cp_shmem_get_size(u32 cp, u32 idx);

/* Legacy functions */
extern unsigned long shm_get_msi_base(void);
extern void __iomem *shm_get_vss_region(void);
extern unsigned long shm_get_vss_base(void);
extern u32 shm_get_vss_size(void);
extern void __iomem *shm_get_vparam_region(void);
extern unsigned long shm_get_vparam_base(void);
extern u32 shm_get_vparam_size(void);

#else /* CONFIG_SHM_IPC */

static inline int cp_shmem_get_mem_map_on_cp_flag(u32 cp_num) { return 0; }
static inline void __iomem *cp_shmem_get_region(u32 cp, u32 idx) { return NULL; }
static inline void __iomem *cp_shmem_get_nc_region(unsigned long base, u32 size) { return NULL; }
static inline void cp_shmem_release_region(u32 cp, u32 idx) { return; }
static inline void cp_shmem_release_rmem(u32 cp, u32 idx) { return; }
static inline unsigned long cp_shmem_get_base(u32 cp, u32 idx) { return 0; }
static inline u32 cp_shmem_get_size(u32 cp, u32 idx) { return 0; }

/* Legacy functions */
static inline unsigned long shm_get_msi_base(void) { return 0; }
static inline void __iomem *shm_get_vss_region(void) { return NULL; }
static inline unsigned long shm_get_vss_base(void) { return 0; }
static inline u32 shm_get_vss_size(void) { return 0; }
static inline void __iomem *shm_get_vparam_region(void) { return NULL; }
static inline unsigned long shm_get_vparam_base(void) { return 0; }
static inline u32 shm_get_vparam_size(void) { return 0; }

#endif /* CONFIG_SHM_IPC */

#endif /* __SHMEM_IPC_H__ */
