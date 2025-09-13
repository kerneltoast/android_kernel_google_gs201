/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#ifndef __DW3000_DEBUGFS_H
#define __DW3000_DEBUGFS_H

#define DW3000_REGCONTENT_MAX_SZ (12) /* 4 bytes + 0x + \n + \0 */
#define DW3000_REG_MAX_SZ (16)

/**
 * enum dw3000_reg_operation - operation done on a register
 * @DW_REG_READ: read whole register. If mask is set the output is shrinked
 * @DW_REG_WRITE: write whole register. include modify operation if mask!=0
 */
enum dw3000_reg_operation {
	DW_REG_READ = 0,
	DW_REG_WRITE,
};

/** struct dw3000_debugfs - debugfs informations in device struct dw
 * @parent_dir: dw parent directory in debugfs
 * @dbgfile_list: linked list of each files in debugfs
 */
struct dw3000_debugfs {
	struct dentry *parent_dir;
	struct list_head dbgfile_list;
};

/** struct dw3000_debugfs_file - debugfs file related structure
 * @chip_reg_priv: register
 * @file: filesystem representation
 * @fileopened: different from 0 if file already opened
 * @ll: linked list for ressources release
 */
struct dw3000_debugfs_file {
	struct dw3000_chip_register_priv chip_reg_priv;
	struct dentry *file;
	atomic_t fileopened;
	struct list_head ll;
};

/**
 * struct dw3000_debugfs_dump_private - private data associated to dbgfs file
 * @dbgfile: structure associated to the dbgfs file
 * @current_reg: current fileid reg to dump through 'registers' file
 */
struct dw3000_debugfs_dump_private {
	struct dw3000_debugfs_file dbgfile;
	const struct dw3000_chip_register *current_reg;
};

int dw3000_debugsfs_init(struct dw3000 *dw);
void dw3000_debugfs_remove(struct dw3000 *dw);

#endif
