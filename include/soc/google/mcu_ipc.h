/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2014-2020, Samsung Electronics.
 *
 */

#ifndef __MCU_IPC_H__
#define __MCU_IPC_H__

#if IS_ENABLED(CONFIG_MCU_IPC)
extern int cp_mbox_register_handler(u32 idx, u32 int_num, irq_handler_t handler, void *data);
extern int cp_mbox_unregister_handler(u32 idx, u32 int_num, irq_handler_t handler);
extern int cp_mbox_enable_handler(u32 idx, u32 int_num);
extern int cp_mbox_disable_handler(u32 idx, u32 int_num);
extern int cp_mbox_check_handler(u32 idx, u32 int_num);

extern void cp_mbox_set_interrupt(u32 idx, u32 int_num);

extern u32 cp_mbox_get_sr(u32 sr_num);
extern u32 cp_mbox_extract_sr(u32 sr_num, u32 mask, u32 pos);
extern void cp_mbox_set_sr(u32 sr_num, u32 msg);
extern void cp_mbox_update_sr(u32 sr_num, u32 msg, u32 mask, u32 pos);
extern void cp_mbox_dump_sr(void);

extern void cp_mbox_reset(void);
extern int cp_mbox_set_affinity(u32 idx, int affinity);
#else /* CONFIG_MCU_IPC */
static inline int cp_mbox_register_handler(u32 idx, u32 int_num, irq_handler_t handler, void *data)
{ return 0; }
static inline int cp_mbox_unregister_handler(u32 idx, u32 int_num, irq_handler_t handler)
{ return 0; }
static inline int cp_mbox_enable_handler(u32 idx, u32 int_num) { return 0; }
static inline int cp_mbox_disable_handler(u32 idx, u32 int_num) { return 0; }
static inline int cp_mbox_check_handler(u32 idx, u32 int_num) { return 0; }

static inline void cp_mbox_set_interrupt(u32 idx, u32 int_num) { return; }

static inline u32 cp_mbox_get_sr(u32 sr_num) { return 0; }
static inline u32 cp_mbox_extract_sr(u32 sr_num, u32 mask, u32 pos) { return 0; }
static inline void cp_mbox_set_sr(u32 sr_num, u32 msg) { return; }
static inline void cp_mbox_update_sr(u32 sr_num, u32 msg, u32 mask, u32 pos) { return; }
static inline void cp_mbox_dump_sr(void) { return; }

static inline void cp_mbox_reset(void) { return; }
static inline int cp_mbox_set_affinity(u32 idx, int affinity) { return 0; }
#endif /* CONFIG_MCU_IPC */
#endif /* __MCU_IPC_H__ */
