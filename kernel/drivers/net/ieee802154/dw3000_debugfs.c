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
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/string.h>

#include "dw3000.h"
#include "dw3000_core.h"
#include "dw3000_debugfs.h"
#include "dw3000_chip.h"
#include "dw3000_cir.h"

#define MAX_CHARS_DISPLAY_DEC_UINT32 10
#define MAX_CHARS_DISPLAY_DEC_INT32 11
#define MAX_CHARS_DISPLAY_HEX_UINT32 8

#define dw3000_dbgfs_regop_write(sz, _dw, _addr, _mask, data)          \
	(_mask) ? dw3000_reg_modify##sz(_dw, _addr, 0, ~_mask, data) : \
		  dw3000_reg_write##sz(_dw, _addr, 0, data)

/**
 * dw3000_dbgfs_power() - Chip agnostic function to handle poweron/off
 * @filp: debugfs file pointer associated to the virtual register
 * @write: false means dump current power status, true means: modify it
 * @buffer: in case of powerstatus modification, 0 means poweroff, 1 means
 *   poweron. Other values are rejected
 * @size: buffer size
 * @ppos: offset in opened file
 *
 * Return: a negative error code or the size written or readed from buffer
 */
static int dw3000_dbgfs_power(struct file *filp, bool write, void *buffer,
			      size_t size, loff_t *ppos)
{
	struct dw3000_debugfs_file *dbgfs_file =
		(struct dw3000_debugfs_file *)filp->private_data;
	struct dw3000_chip_register_priv *crp = &dbgfs_file->chip_reg_priv;
	struct dw3000 *dw = crp->dw;
	char cbuf[3];
	int rc;
	u8 on;

	if (*ppos > 0)
		return 0;

	if (write) {
		if (kstrtou8_from_user(buffer, size, 0, &on)) {
			dev_err(dw->dev, "no valid value provided\n");
			return -EINVAL;
		}

		if (on == 1) {
			rc = dw3000_poweron(dw);
			if (rc)
				dev_warn(dw->dev, "poweron returns %d\n", rc);
		} else if (on == 0) {
			rc = dw3000_poweroff(dw);
			if (rc)
				dev_warn(dw->dev, "poweroff returns %d\n", rc);
		} else {
			dev_err(dw->dev, "%u value not handled\n", on);
			return -EINVAL;
		}
		if (!rc)
			rc = size;
	} else {
		on = dw->current_operational_state != DW3000_OP_STATE_OFF;
		rc = scnprintf(cbuf, 3, "%u\n", on);
		if (copy_to_user(buffer, cbuf, rc)) {
			dev_err(dw->dev, "impossible to copy data to userland");
			return -EFAULT;
		}
	}
	*ppos += rc;
	return rc;
}

/**
 * dw3000_dbgfs_cir_data() - Consummer function to dump CIR data to userspace
 * @filp: debugfs file pointer associated to the virtual register
 * @write: false means dump current power status, true means: modify it
 * @buffer: in case of powerstatus modification, 0 means poweroff, 1 means
 *   poweron. Other values are rejected
 * @size: buffer size
 * @ppos: offset in opened file
 *
 * Return: a negative error code or the size written or readed from buffer
 */

static int dw3000_dbgfs_cir_data(struct file *filp, bool write, void *buffer,
				 size_t size, loff_t *ppos)
{
	struct dw3000_debugfs_file *dbgfs_file = filp->private_data;
	struct dw3000_chip_register_priv *crp = &dbgfs_file->chip_reg_priv;
	struct dw3000 *dw = crp->dw;
	struct dw3000_cir_data *cir = dw->cir_data;
	size_t bufsz;

	/* Try to read cir data but no memory have been allocated for */
	if (unlikely(!cir))
		return 0;

	/* size is memory footprint of dw->cir_data without header members */
	bufsz = sizeof(*cir) +
		sizeof(struct dw3000_cir_record) * (cir->count - 1) -
		offsetof(struct dw3000_cir_data, count);

	/* Reset before waiting. Allowing memory change detection */
	dw->cir_data_changed = false;

	/* Wait for producer */
	if (wait_for_completion_interruptible(&cir->complete)) {
		return -ERESTARTSYS;
	}

	/* During wait, cir_data can be reallocated. If there is any change
	 * current caller have to be closed gently */
	smp_rmb();
	if (dw->cir_data_changed)
		return 0;

	/* Wait for release of shared memory */
	if (mutex_lock_interruptible(&cir->mutex)) {
		return -EINTR;
	}

	if (copy_to_user(buffer, &cir->count, bufsz)) {
		mutex_unlock(&cir->mutex);
		dev_err(dw->dev, "impossible to copy data to userland");
		return -EFAULT;
	}

	mutex_unlock(&cir->mutex);
	/* Output is 16 bytes aligned */
	bufsz = ALIGN(bufsz, 16);
	*ppos += bufsz;

	return bufsz;
}

/**
 * dw3000_dbgfs_cir_config() - Runtime modification of record count and filter
 * @filp: debugfs file pointer associated to the virtual register
 * @write: false means dump current power status, true means: modify it
 * @buffer: in case of powerstatus modification, 0 means poweroff, 1 means
 *   poweron. Other values are rejected
 * @size: buffer size
 * @ppos: offset in opened file
 *
 * Return: a negative error code or the size written or readed from buffer
 */
static int dw3000_dbgfs_cir_config(struct file *filp, bool write, void *buffer,
				   size_t size, loff_t *ppos)
{
	struct dw3000_debugfs_file *dbgfs_file =
		(struct dw3000_debugfs_file *)filp->private_data;
	struct dw3000_chip_register_priv *crp = &dbgfs_file->chip_reg_priv;
	struct dw3000 *dw = crp->dw;
	struct dw3000_cir_data *cir = dw->cir_data;
	static const char fmt[] = "count %u filter 0x%x offset %d\n";
	/* fit format string with max values for each (U)32 and sign */
	char cbuf[sizeof(fmt) +
		MAX_CHARS_DISPLAY_DEC_UINT32 +
		MAX_CHARS_DISPLAY_DEC_UINT32 +
		MAX_CHARS_DISPLAY_HEX_UINT32];
	unsigned int newcount;
	int newoff;
	u32 newfilter;
	int r = 0;

	if (unlikely(!cir))
		return 0;

	if (*ppos > 0)
		return 0;

	if (!write) {
		r = scnprintf(cbuf, sizeof(cbuf), fmt, cir->count, cir->filter,
			     cir->offset);
		if (copy_to_user(buffer, cbuf, r)) {
			dev_err(dw->dev, "impossible to copy data to userland");
			*ppos += size;
			return -EFAULT;
		}
		*ppos += r;
		return r;
	}

	if (size >= sizeof(cbuf))
		return -EINVAL;
	if (copy_from_user(cbuf, buffer, size)) {
		dev_err(dw->dev, "copy failed\n");
		return -EFAULT;
	}

	r = sscanf(cbuf, fmt, &newcount, &newfilter, &newoff);
	if (r != 3) {
		dev_err(dw->dev, "input format error (%d)\n", r);
		return -EINVAL;
	}

	if (cir->count != newcount && newcount >= 1) {
		r = dw3000_cir_data_alloc_count(dw, newcount);
		if (r < 0)
			return -ENOMEM;
	} else if (newcount < 1) {
		return -EINVAL;
	}

	dw->cir_data->filter = newfilter;
	dw->cir_data->offset = newoff;

	*ppos += size;
	return size;
}

static const struct dw3000_chip_register virtual_registers[] = {
	{ "power", 0x0, 0x0, 0x0, DW3000_CHIPREG_PERM, dw3000_dbgfs_power },
	{ "cir_data", 0x0, 0x0, 0x0,
	  DW3000_CHIPREG_RO | DW3000_CHIPREG_OPENONCE, dw3000_dbgfs_cir_data },
	{ "cir_config", 0x0, 0x0, 0x0, DW3000_CHIPREG_PERM,
	  dw3000_dbgfs_cir_config },
};

/** struct do_reg_xfer_params - parameters for spi register access
 * @reg: pointer to the register to reach
 * @operation: set operation to do on reg according to enum dw3000_reg_operation
 * @value: pointer on the data for write and modify operations
 */
struct do_reg_xfer_params {
	const struct dw3000_chip_register *reg;
	int operation;
	void *value;
};

static int do_dump_xfer(struct dw3000 *dw, const void *in, void *out)
{
	const struct dw3000_debugfs_dump_private *priv = in;
	int rc;

	rc = dw3000_xfer(dw, priv->current_reg->address << 16, 0,
			 priv->current_reg->size, out, DW3000_SPI_RD_BIT);

	return rc;
}

static int do_reg_write(struct dw3000 *dw,
			const struct dw3000_chip_register *reg, u64 *value)
{
	u8 shift = (reg->mask) ? ffs(reg->mask) - 1 : 0;
	int rc;

	*value <<= shift;

	switch (reg->size) {
	case 1:
		rc = dw3000_dbgfs_regop_write(8, dw, reg->address, reg->mask,
					      *value);
		break;
	case 2:
		rc = dw3000_dbgfs_regop_write(16, dw, reg->address, reg->mask,
					      *value);
		break;
	case 4:
		rc = dw3000_dbgfs_regop_write(32, dw, reg->address, reg->mask,
					      *value);
		break;
	default:
		if (unlikely(reg->mask)) {
			dev_warn(
				dw->dev,
				"mask defined but modify not supported for this size\n");
			rc = -ENOSYS;
			break;
		}
		rc = dw3000_reg_write_fast(dw, reg->address, 0, reg->size,
					   value, DW3000_SPI_WR_BIT);
	}
	return rc;
}

static int do_reg_read(struct dw3000 *dw,
		       const struct dw3000_chip_register *reg, void *value)
{
	u8 shift = (reg->mask) ? ffs(reg->mask) - 1 : 0;
	int rc;

	rc = dw3000_reg_read_fast(dw, reg->address, 0, reg->size, value);

	if (reg->mask && !rc) {
		*(u32 *)value &= reg->mask;
		*(u32 *)value >>= shift;
	}

	return rc;
}

static int do_reg_xfer(struct dw3000 *dw, const void *in, void *out)
{
	const struct do_reg_xfer_params *xfer = in;
	int rc = 0;

	if (xfer->operation == DW_REG_READ)
		rc = do_reg_read(dw, xfer->reg, out);
	else
		rc = do_reg_write(dw, xfer->reg, (u64 *)xfer->value);

	return rc;
}

static int dw3000_file_release(struct inode *inode, struct file *file)
{
	if (file->private_data) {
		kfree(file->private_data);
		file->private_data = NULL;
	}
	return 0;
}

static int dw3000_dump_open(struct inode *inode, struct file *file)
{
	struct dw3000_debugfs_dump_private *dev_priv;
	struct dw3000 *dw;
	size_t sz = sizeof(struct dw3000_debugfs_dump_private);

	if (unlikely(!inode->i_private))
		return -ENXIO;

	file->private_data = kzalloc(sz, GFP_KERNEL);
	if (!file->private_data) {
		dev_priv =
			(struct dw3000_debugfs_dump_private *)inode->i_private;
		dw = dev_priv->dbgfile.chip_reg_priv.dw;
		dev_err(dw->dev, "Allocation failed. Cannot open file.\n");
		return -ENOMEM;
	}
	memcpy(file->private_data, inode->i_private, sz);
	return 0;
}

static ssize_t format_reg_output(struct dw3000 *dw, void *binbuf,
				 ssize_t reg_sz, char __user *userbuf)
{
	char cbuf[36]; /* 0x + 16 bytes + \n \0 */
	ssize_t r = 0;

	if (reg_sz > 8) {
		if (copy_to_user(userbuf, binbuf, reg_sz)) {
			dev_err(dw->dev, "copy failed\n");
			return -EFAULT;
		}
		return reg_sz;
	} else {
		r = scnprintf(cbuf, 36, "0x%llx\n", *(u64 *)binbuf);
		if (copy_to_user(userbuf, cbuf, r)) {
			dev_err(dw->dev, "impossible to copy data to userland");
			return -EFAULT;
		}
	}
	return r;
}

static ssize_t format_dump_output(struct dw3000 *dw, void *buffer,
				  char *reg_name, ssize_t buffer_sz,
				  char __user *userbuf)
{
	char *cbuf;
	char *cbufpos;
	size_t cbuflen;
	char *buffer_idx;
	ssize_t line_len, remain;
	int r = 0;
	size_t bufpos = 0;

	/* Buffer for all registers of the fileid
	 * Size is computed for a byte display, this is more than
	 * required for word (4bytes) thus we keep this formula to be
	 * able to securely print data whatever the format selected */
	cbuflen = buffer_sz * 3 + 14 + 1; /* N regs max ; reg name */
	cbuf = (char *)kzalloc(cbuflen * sizeof(char), GFP_KERNEL);
	if (!cbuf) {
		dev_err(dw->dev, "dbgfs: impossible to alloc buffers mem\n");
		return -ENOMEM;
	}

	/* Output generation */
	buffer_idx = buffer;
	bufpos += scnprintf(cbuf, cbuflen, "%.12s:\n", reg_name);

	remain = cbuflen - bufpos;
	cbufpos = cbuf + bufpos;

	while (buffer_sz) {
		line_len = hex_dump_to_buffer(buffer_idx, buffer_sz, 16, 4,
					      cbufpos, remain, false);
		if (line_len >= remain) {
			dev_err(dw->dev, "buffer too small");
			break;
		}
		cbufpos[line_len++] = '\n';
		cbufpos += line_len;
		remain -= line_len;
		buffer_idx += 16;
		buffer_sz -= min(16, (int)buffer_sz);
		buffer_sz = (buffer_sz + 3) & ~3;
	};

	r = cbufpos - cbuf;
	if (copy_to_user(userbuf, cbuf, r)) {
		dev_err(dw->dev, "impossible to copy data to userland");
		r = -EFAULT;
		goto format_dump_output_err;
	}

format_dump_output_err:
	if (cbuf)
		kfree(cbuf);
	return r;
}

static ssize_t dw3000_dump_read(struct file *filp, char __user *userbuf,
				size_t count, loff_t *ppos)
{
	struct dw3000_debugfs_dump_private *priv = filp->private_data;
	struct dw3000_stm_command cmd = { do_dump_xfer, priv, NULL };
	struct dw3000_chip_register_priv *crp = &priv->dbgfile.chip_reg_priv;
	struct dw3000 *dw;
	void *buffer;
	int r = 0;

	dw = priv->dbgfile.chip_reg_priv.dw;

	/* A runlevel under DW3000_OP_STATE_IDLE_RC doesn't allow SPI xfer */
	if (dw->current_operational_state < DW3000_OP_STATE_IDLE_RC) {
		dev_err(dw->dev,
			"unable to dump registers: device not ready\n");
		return -EIO;
	}

	/* Reset feature */
	if (*ppos == 0)
		priv->current_reg = crp->reg;

	/* If table's end reached reset current reg */
	if ((priv->current_reg == (crp->reg + crp->count)) ||
	    !(priv->current_reg->flags & DW3000_CHIPREG_DUMP))
		return 0;

	/* Buffer for all registers of the fileid */
	buffer = kzalloc(priv->current_reg->size, GFP_KERNEL);
	if (!buffer) {
		dev_err(dw->dev, "dbgfs: impossible to alloc buffers mem\n");
		r = -ENOMEM;
		goto dw3000_dump_read_err;
	}

	/* Retrieve all DW's registers content */
	cmd.out = (void *)buffer;
	r = dw3000_enqueue_generic(dw, &cmd);
	if (r) {
		dev_err(dw->dev, "Fail to read registers in fileId %d (%d)\n",
			priv->current_reg->address, r);
		goto dw3000_dump_read_err;
	}

	/* Output generation */
	r = format_dump_output(dw, buffer, (char *)priv->current_reg->name,
			       priv->current_reg->size, userbuf);

	/* Update current_reg: jump to the next file_id */
	priv->current_reg++;

	/* Update position for sequential read */
	*ppos = *ppos + r;

dw3000_dump_read_err:
	if (buffer)
		kfree(buffer);
	return r;
}

static ssize_t dw3000_dbgfs_reg_op(struct file *filp, char __user *userbuf,
				   size_t count, loff_t *ppos, bool write)
{
	struct dw3000_debugfs_file *dbgfs_file =
		(struct dw3000_debugfs_file *)filp->private_data;
	struct dw3000_chip_register_priv *crp = &dbgfs_file->chip_reg_priv;
	struct do_reg_xfer_params xfer;
	struct dw3000 *dw = crp->dw;
	unsigned long long binbuf[DW3000_REG_MAX_SZ / sizeof(long long)];
	struct dw3000_stm_command cmd = { do_reg_xfer, &xfer, binbuf };
	const struct dw3000_chip_register *reg = crp->reg;
	ssize_t outsize, buflen;
	ssize_t r = 0;
	xfer.operation = DW_REG_READ;
	memset(binbuf, 0, DW3000_REG_MAX_SZ);

	/* A runlevel under DW3000_OP_STATE_IDLE_RC doesn't allow SPI xfer
	 * except if permanent access is specially granted to this register */
	if (dw->current_operational_state < DW3000_OP_STATE_IDLE_RC &&
	    !(reg->flags & DW3000_CHIPREG_PERM)) {
		dev_err(dw->dev,
			"unable to reach registers: device not ready\n");
		return -EIO;
	}

	if (reg->callback)
		return reg->callback(filp, write, userbuf, count, ppos);

	if (*ppos > 0)
		return 0;

	if (write) {
		/* Check write-protection of the register */
		if ((reg->flags & DW3000_CHIPREG_WP) && dw3000_is_active(dw)) {
			dev_err(dw->dev,
				"register %s is protected. Write refused "
				"because device is active\n",
				reg->name);
			r = -EACCES;
			goto dw3000_reg_op_err;
		}

		/* Get value from userspace */
		if (reg->size <= 8) {
			if (kstrtoull_from_user(userbuf, count, 0, binbuf)) {
				r = -EINVAL;
				dev_err(dw->dev, "%s is not valid value\n",
					userbuf);
				goto dw3000_reg_op_err;
			}
		} else {
			/* input is binary stream. truncated to reg.size or
			 * filled with 0 if provided is shorter */
			buflen = min(count, sizeof(binbuf));
			if (copy_from_user(binbuf, userbuf, buflen)) {
				r = -EFAULT;
				dev_err(dw->dev, "copy failed\n");
				goto dw3000_reg_op_err;
			}
		}
		xfer.operation = DW_REG_WRITE;
		xfer.value = binbuf;
	}

	/* do spi operation */
	xfer.reg = reg;
	r = dw3000_enqueue_generic(dw, &cmd);
	if (r) {
		dev_err(dw->dev, "fail to access register %s (%ld)\n",
			reg->name, r);
		goto dw3000_reg_op_err;
	}

	if (!write) {
		if (reg->mask) {
			/* size of mask, 0 inside it included */
			outsize = fls(reg->mask >> (ffs(reg->mask) - 1)) - 1;
			outsize = (outsize >> 3) + 1;
		} else
			outsize = reg->size;

		r = format_reg_output(dw, binbuf, outsize, userbuf);

		if (r < 0)
			goto dw3000_reg_op_err;
		else
			count = r;
	}
	*ppos += count;
	r = count;

dw3000_reg_op_err:
	return r;
}

static int dw3000_dbgfs_reg_release(struct inode *inode, struct file *file)
{
	struct dw3000_debugfs_file *dbgfs_file = file->private_data;

	atomic_dec(&dbgfs_file->fileopened);

	return 0;
}

static int dw3000_dbgfs_reg_open(struct inode *inode, struct file *file)
{
	struct dw3000_debugfs_file *dbgfs_file =
		(struct dw3000_debugfs_file *)inode->i_private;
	struct dw3000_chip_register_priv *crp;

	if (unlikely(!dbgfs_file))
		return -ENXIO;

	file->private_data = dbgfs_file;
	crp = &dbgfs_file->chip_reg_priv;

	if (crp->reg->flags & DW3000_CHIPREG_OPENONCE) {
		if (!atomic_add_unless(&dbgfs_file->fileopened, 1, 1))
			return -EBUSY;
	}

	return 0;
}

static ssize_t dw3000_dbgfs_reg_read(struct file *filp, char __user *userbuf,
				     size_t count, loff_t *ppos)
{
	return dw3000_dbgfs_reg_op(filp, userbuf, count, ppos, false);
}

static ssize_t dw3000_dbgfs_reg_write(struct file *filp,
				      const char __user *userbuf, size_t count,
				      loff_t *ppos)
{
	return dw3000_dbgfs_reg_op(filp, (char *)userbuf, count, ppos, true);
}

static struct file_operations dw3000_dump_fops = {
	.read = dw3000_dump_read,
	.open = dw3000_dump_open,
	.release = dw3000_file_release,
};

static struct file_operations dw3000_reg_fops = {
	.read = dw3000_dbgfs_reg_read,
	.write = dw3000_dbgfs_reg_write,
	.open = dw3000_dbgfs_reg_open,
	.release = dw3000_dbgfs_reg_release,
};

static int dw3000_debugsfs_regs_init(struct dw3000 *dw,
				     const struct dw3000_chip_register *regs,
				     size_t reg_count)
{
	struct dw3000_debugfs_file *cur;
	int i;

	for (i = 0; i < reg_count; i++) {
		if (regs[i].flags & DW3000_CHIPREG_DUMP)
			continue;

		cur = kzalloc(sizeof(struct dw3000_debugfs_file), GFP_KERNEL);
		if (!cur) {
			dw3000_debugfs_remove(dw);
			return -ENOMEM;
		}

		cur->chip_reg_priv.reg = &regs[i];
		cur->chip_reg_priv.dw = dw;
		INIT_LIST_HEAD(&cur->ll);
		list_add_tail(&cur->ll, &dw->debugfs.dbgfile_list);

		cur->file = debugfs_create_file(
			regs[i].name,
			S_IRUGO |
				((regs[i].flags & DW3000_CHIPREG_RO) ? 0 :
								       S_IWUGO),
			dw->debugfs.parent_dir, &cur->chip_reg_priv,
			&dw3000_reg_fops);
	}

	return 0;
}

/**
 * dw3000_debugsfs_init() - Debugfs interface for DW's registers
 * @dw: The DW device.
 *
 * Debugfs exposition of DW's registers
 *
 * Return: 0 if success, -EINVAL if dw have no registers configured and
 * ENODEV if files are impossible to create (resulting of a debugfs
 * missing support)
 */
int dw3000_debugsfs_init(struct dw3000 *dw)
{
	struct dw3000_debugfs_dump_private *dump;
	size_t reg_count;
	const struct dw3000_chip_register *regs =
		dw->chip_ops->get_registers(dw, &reg_count);
	int rc;

	if (!regs)
		return -EINVAL;
	if (!reg_count) {
		dev_dbg(dw->dev, "empty registers list returned. init abort\n");
		return -EINVAL;
	}

	INIT_LIST_HEAD(&dw->debugfs.dbgfile_list);
	dw->debugfs.parent_dir = debugfs_create_dir("dw3000", NULL);
	if (IS_ERR(dw->debugfs.parent_dir)) {
		dw->debugfs.parent_dir = NULL;
		dev_err(dw->dev, "unable to create parent dir %ld\n",
			PTR_ERR(dw->debugfs.parent_dir));
		return -EINVAL;
	}
	if (!dw->debugfs.parent_dir) { /* err even if debugfs activated */
		dev_err(dw->dev, "debugfs directory creation failed\n");
		return -EINVAL;
	}

	/* Creation of "registers" file to dump whole file ids */
	dump = kzalloc(sizeof(struct dw3000_debugfs_dump_private), GFP_KERNEL);
	if (!dump) {
		dw3000_debugfs_remove(dw);
		return -ENOMEM;
	}

	dump->dbgfile.file = debugfs_create_file("registers", S_IRUGO,
						 dw->debugfs.parent_dir, dump,
						 &dw3000_dump_fops);

	dump->dbgfile.chip_reg_priv.count = reg_count;
	dump->dbgfile.chip_reg_priv.reg = regs;
	dump->dbgfile.chip_reg_priv.dw = dw;
	INIT_LIST_HEAD(&dump->dbgfile.ll);
	list_add_tail(&dump->dbgfile.ll, &dw->debugfs.dbgfile_list);

	/* Creation of a file for each non-dumpable register */
	rc = dw3000_debugsfs_regs_init(dw, regs, reg_count);
	if (rc) {
		dev_err(dw->dev, "error when init of chip's registers\n");
		return rc;
	}
	rc = dw3000_debugsfs_regs_init(dw, virtual_registers,
				       ARRAY_SIZE(virtual_registers));
	if (rc) {
		dev_err(dw->dev, "error when init of virtual registers\n");
		return rc;
	}

	return 0;
}

/**
 * dw3000_debugfs_remove() - Remove all files and directory created by _init
 * @dw: The DW Device
 *
 * Return: nothing
 */
void dw3000_debugfs_remove(struct dw3000 *dw)
{
	while (!list_empty(&dw->debugfs.dbgfile_list)) {
		struct dw3000_debugfs_file *cur =
			list_first_entry(&dw->debugfs.dbgfile_list,
					struct dw3000_debugfs_file, ll);
		debugfs_remove(cur->file);
		list_del(&cur->ll);
		kfree(cur);
	}

	debugfs_remove_recursive(dw->debugfs.parent_dir);
}
