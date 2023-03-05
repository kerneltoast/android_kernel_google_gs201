# SPDX-License-Identifier: GPL-2.0
# Copyright (C) 2018 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved.
#
# This is based on the original scripts/dtc/Makefile, but has been revised to
# work with an insane submodule situation.

hostprogs-always-y := dtc-aosp

dtc-aosp-objs	:= dtc.o flattree.o fstree.o data.o livetree.o treesource.o \
		   srcpos.o checks.o util.o
dtc-aosp-objs	+= dtc-lexer.lex.o dtc-parser.tab.o

# We're working with a submodule, so make these all relative to that.
dtc-aosp-objs	:= $(addprefix dtc/,$(dtc-aosp-objs))

HOSTCFLAGS_DTC := -I$(srctree)/$(src)/dtc -I$(srctree)/$(src)/dtc/libfdt
HOSTCFLAGS_DTC += -DNO_YAML

HOSTCFLAGS_dtc/checks.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc/data.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc/dtc.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc/flattree.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc/fstree.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc/livetree.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc/srcpos.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc/treesource.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc/util.o := $(HOSTCFLAGS_DTC)

HOSTCFLAGS_dtc/dtc-lexer.lex.o := $(HOSTCFLAGS_DTC)
HOSTCFLAGS_dtc/dtc-parser.tab.o := $(HOSTCFLAGS_DTC)

# Dependencies on generated files need to be listed explicitly.
$(obj)/dtc/dtc-lexer.lex.o: $(obj)/dtc/dtc-parser.h

# Make the header output .h instead of .tab.h for bison, because aosp is broken.
%.h: %.y
	$(call if_changed,bison)

# Generated files need to be cleaned explicitly.
clean-files	:= dtc/dtc-lexer.lex.c dtc/dtc-parser.tab.c dtc/dtc-parser.h
