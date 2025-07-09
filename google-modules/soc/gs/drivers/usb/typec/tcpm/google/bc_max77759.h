// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2019, Google LLC
 *
 * MAX77759 BC1.2 management.
 */

#ifndef __BC_MAX77759_H
#define __BC_MAX77759_H

#define VENDOR_ALERT1           0x80
#define CHGTYPINT               BIT(0)
#define DCDTMOINT               BIT(1)
#define PRCHGTYPINT             BIT(2)
#define CHGTYPRUNRINT           BIT(3)
#define CHGTYPRUNFINT           BIT(4)
#define DNVDATREFINT            BIT(5)

#define VENDOR_ALERT_MASK1	0x82

#define VENDOR_BC_CTRL1         0x8F
#define CHGDETEN                BIT(0)
#define CHGDETMAN               BIT(1)
#define NOBCCOMP                BIT(5)
#define DCDCPL                  BIT(7)

#define VENDOR_BC_CTRL2         0x90
#define DPDNMAN                 BIT(4)
#define DPDRV                   GENMASK(3, 2)
#define DPDRV_SHIFT             2
#define DPDRV_3V0               0x2
#define DPDRV_OPEN              0x3

#define VENDOR_BC_STATUS1       0x87
#define CHGTYP                  GENMASK(1, 0)
#define DCDTMO                  BIT(2)
#define CHGTYPRUN               BIT(6)

/* CHGTYP */
#define CHGTYP_NOT_ATTACHED	0
#define CHGTYP_SDP              0x1
#define CHGTYP_CDP              0x2
#define CHGTYP_DCP              0x3

#define VENDOR_BC_STATUS2       0x88

struct bc12_status;
struct max77759_plat;
typedef void (*bc12_status_callback)(struct max77759_plat *chip, bool running);
struct bc12_status *bc12_init(struct max77759_plat *plat, bc12_status_callback callback);
void process_bc12_alert(struct bc12_status *bc12, u8 vendor_alert1_status);
void bc12_reset_retry(struct bc12_status *bc12);
void bc12_teardown(struct bc12_status *bc12);
void bc12_enable(struct bc12_status *bc12, bool enable);
bool bc12_get_status(struct bc12_status *bc12);
enum power_supply_usb_type get_usb_type(struct bc12_status *bc12);

#endif /*__BC_MAX77759_H */

