# SPDX-License-Identifier: GPL-2.0
# UFSHCD makefile

# The link order is important here. ufshcd-core must initialize
# before vendor drivers.
obj-$(CONFIG_SCSI_UFSHCD)		+= ufshcd-core.o
ufshcd-core-y				+= ufshcd.o ufs-sysfs.o
ufshcd-core-$(CONFIG_DEBUG_FS)		+= ufs-debugfs.o
ufshcd-core-$(CONFIG_SCSI_UFS_BSG)	+= ufs_bsg.o
ufshcd-core-$(CONFIG_SCSI_UFS_CRYPTO)	+= ufshcd-crypto.o
ufshcd-core-$(CONFIG_SCSI_UFS_HPB)	+= ufshpb.o

obj-$(CONFIG_SCSI_UFS_DWC_TC_PCI) += tc-dwc-g210-pci.o ufshcd-dwc.o tc-dwc-g210.o
obj-$(CONFIG_SCSI_UFS_DWC_TC_PLATFORM) += tc-dwc-g210-pltfrm.o ufshcd-dwc.o tc-dwc-g210.o
obj-$(CONFIG_SCSI_UFS_CDNS_PLATFORM) += cdns-pltfrm.o
obj-$(CONFIG_SCSI_UFS_QCOM) += ufs_qcom.o
ufs_qcom-y += ufs-qcom.o
ufs_qcom-$(CONFIG_SCSI_UFS_CRYPTO) += ufs-qcom-ice.o
obj-$(CONFIG_SCSI_UFSHCD) += ufshcd-core.o
ufshcd-core-y				+= ufshcd.o ufs-sysfs.o
ufshcd-core-$(CONFIG_SCSI_UFS_BSG)	+= ufs_bsg.o
ufshcd-core-$(CONFIG_SCSI_UFS_CRYPTO) += ufshcd-crypto.o
obj-$(CONFIG_SCSI_UFSHCD_PCI) += ufshcd-pci.o
obj-$(CONFIG_SCSI_UFSHCD_PLATFORM) += ufshcd-pltfrm.o
obj-$(CONFIG_SCSI_UFS_HISI) += ufs-hisi.o
obj-$(CONFIG_SCSI_UFS_MEDIATEK) += ufs-mediatek.o
obj-$(CONFIG_SCSI_UFS_TI_J721E) += ti-j721e-ufs.o
obj-$(CONFIG_SCSI_UFS_EXYNOS) += ufs-exynos-core.o
ufs-exynos-core-y += ufs-exynos.o ufs-exynos-dbg.o ufs-pixel.o
ufs-exynos-core-$(CONFIG_SOC_GS101) += gs101/ufs-cal-if.o
ufs-exynos-core-$(CONFIG_SCSI_UFS_CRYPTO) += ufs-pixel-crypto.o ufs-exynos-fmp.o

obj-$(CONFIG_SCSI_UFS_PIXEL_FIPS140) += ufs-pixel-fips140.o
ufs-pixel-fips140-y += ufs-pixel-fips.o ufs-pixel-fips_sha256.o \
		       ufs-pixel-fips140-refs.o
# Use -DBUILD_FIPS140_KO to disable dynamic code patching as that would
# invalidate the hash of the module's .text section
CFLAGS_ufs-pixel-fips140.o += -DBUILD_FIPS140_KO
CFLAGS_ufs-pixel-fips_sha256.o := -DBUILD_FIPS140_KO
hostprogs := ufs-pixel-fips_gen_hmac
HOSTLDLIBS_ufs-pixel-fips_gen_hmac := -lcrypto -lelf
$(obj)/ufs-pixel-fips140.o: $(obj)/ufs-pixel-fips_gen_hmac
