obj-$(CONFIG_MHI_BUS) += mhi.o

mhi-y := init.o main.o pm.o boot.o
mhi-$(CONFIG_MHI_BUS_MISC) += misc.o
mhi-$(CONFIG_MHI_BUS_DEBUG) += debugfs.o

ccflags-y += -DCONFIG_WCN_GOOGLE
