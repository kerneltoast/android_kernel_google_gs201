# SPDX-License-Identifier: GPL-2.0

# Set KERNEL_SRC to google-modules so that the include paths are set correctly
# for in-tree compilation, since the modules assume that `google-modules` exists
# in the same directory as the kernel tree (one level up), whereas we've got
# `google-modules` inside the kernel tree. This way, when the modules include
# with `-I$(KERNEL_SRC)/../google-modules/SOME_MODULE`, inclusion works as-is.
export KERNEL_SRC := $(srctree)/google-modules

obj-y += lynx/display/
