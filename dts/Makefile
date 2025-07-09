# SPDX-License-Identifier: GPL-2.0

gs201_overlays := \
	gs201-lynx-dev1_0.dtbo \
	gs201-lynx-proto1_0.dtbo \
	gs201-lynx-proto1_1.dtbo \
	gs201-lynx-evt1_0.dtbo \
	gs201-lynx-evt1_0-wingboard.dtbo \
	gs201-lynx-evt1_1.dtbo \
	gs201-lynx-dvt1_0.dtbo \
	gs201-lynx-dvt1_0-gain1.dtbo \
	gs201-lynx-pvt1_0.dtbo \
	gs201-lynx-mp1_0.dtbo

gs201_dtbs := \
	gs201/gs201-a0.dtb \
	gs201/gs201-b0.dtb \
	gs201/gs201-b0_v2-ipop.dtb

# Usage:
#   $(call DTBS_overlay_template, overlay, base_dtb, suffix)
#
# This is a generic template to create the following construct:
#   <overlay>-<suffix>-dtbs := <base_dtb>.dtb <overlay>.dtbo
define DTBS_overlay_template
	$(1:%.dtbo=%-$(strip $3)-dtbs) := $2 $1
endef

# Usage:
#   $(call DTB_object_template, CONFIG_, overlay, suffix)
#
# This is a generic template to create the following construct:
#   dtb-$(CONFIG_) += <overlay>-<suffix>.dtb
define DTB_object_template
	dtb-$(strip $1) += $(2:%.dtbo=%-$(strip $3).dtb)
endef

# Usage:
#   $(call multi_dtbs_overlay, list_of_base_dtbs, list_of_overlays, CONFIG_)
#
# For each (base_dtb,overlay) pair, this will create the following constructs:
#   <overlay>-<base_dtb>-dtbs := <base_dtb>.dtb <overlay>.dtbo
#   dtb-$(CONFIG_) += <overlay>-<base_dtb>.dtb
define multi_dtbs_overlay
	$(foreach base, $1, \
		$(foreach overlay, $2, \
			$(eval $(call DTBS_overlay_template, $(overlay), $(base), $(base:%.dtb=%))) \
			$(eval $(call DTB_object_template, $3, $(overlay), $(base:%.dtb=%)))))
endef

# Create the -dtbs and dtb-y constructs. We can do this by looping through the
# base dtbs and apply each overlay to it. Kbuild will use these -dtbs
# constructs to determine which dtbs are the base dtbs and which are the
# overlays. For the base dtbs, kbuild will expose the symbols as needed for the
# overlays. Additionally, kbuild will statically apply the overlay to the base
# dtb which could be used to verify the overlay applies cleanly.
$(call multi_dtbs_overlay, $(gs201_dtbs), $(gs201_overlays), y)
