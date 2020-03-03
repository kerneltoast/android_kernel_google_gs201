// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2019 Google LLC
 *
 */

#ifndef __USB_ICL_VOTER_H
#define __USB_ICL_VOTER_H

#include "../../../power/supply/google/gvotable.h"

#define USB_ICL_THERMAL_VOTER "THERMAL_VOTER"
#define USB_ICL_PROTO_VOTER "PROTO_VOTER"
#define USB_ICL_EL "USB_ICL"
#define USB_ICL_COMBINED_EL "USB_ICL_COMBINED"
#define USB_ICL_PROTO_EL "USB_ICL_PROTO"

#define PROTO_VOTER(S)	{		\
	S(BC12_SDP),			\
	S(DEAD_BATTERY),		\
	S(USB_CONFIGURED),		\
	S(BC12_CDP_DCP),		\
	S(USB_ICL_TYPEC),		\
	S(USB_ICL_PD),			\
	S(USB_DATA_SUSPEND) }

#define GENERATE_ENUM(e)	e
#define GENERATE_STRING(s)	#s

#define ICL_VOTER(S) {			\
	S(USB_ICL_COMB),	\
	S(USB_ICL_USER) }

enum proto_voter_priority PROTO_VOTER(GENERATE_ENUM);

static const char * const proto_voter_reason[] =
	PROTO_VOTER(GENERATE_STRING);

enum icl_voter_priority ICL_VOTER(GENERATE_ENUM);

static const char * const icl_voter_reason[] = ICL_VOTER(GENERATE_STRING);

struct usb_vote {
	char reason[GVOTABLE_MAX_REASON_LEN];
	unsigned int priority;
	unsigned int val;
};

void init_vote(struct usb_vote *vote, const char *reason, unsigned
	       int priority, unsigned int val);
#endif /*__USB_ICL_VOTER_H*/
