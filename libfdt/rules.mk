# Copyright (c) 2022, Google, Inc. All rights reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This file is not used in the Android build process! It's used only by Trusty.


LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS := \
	$(LOCAL_DIR)/fdt.c \
	$(LOCAL_DIR)/fdt_check.c \
	$(LOCAL_DIR)/fdt_ro.c \
	$(LOCAL_DIR)/fdt_wip.c \
	$(LOCAL_DIR)/fdt_sw.c \
	$(LOCAL_DIR)/fdt_rw.c \
	$(LOCAL_DIR)/fdt_strerror.c \
	$(LOCAL_DIR)/fdt_empty_tree.c \
	$(LOCAL_DIR)/fdt_addresses.c \
	$(LOCAL_DIR)/fdt_overlay.c \
	$(LOCAL_DIR)/acpi.c \


MODULE_COMPILEFLAGS += \
	-Wno-sign-compare \
	-Wno-macro-redefined \

MODULE_EXPORT_INCLUDES += \
	$(LOCAL_DIR) \

include make/library.mk
