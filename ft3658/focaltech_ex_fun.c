/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*****************************************************************************
*
* File Name: Focaltech_ex_fun.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define PROC_UPGRADE                            0
#define PROC_READ_REGISTER                      1
#define PROC_WRITE_REGISTER                     2
#define PROC_AUTOCLB                            4
#define PROC_UPGRADE_INFO                       5
#define PROC_WRITE_DATA                         6
#define PROC_READ_DATA                          7
#define PROC_SET_TEST_FLAG                      8
#define PROC_SET_SLAVE_ADDR                     10
#define PROC_HW_RESET                           11
#define PROC_READ_STATUS                        12
#define PROC_SET_BOOT_MODE                      13
#define PROC_ENTER_TEST_ENVIRONMENT             14
#define PROC_WRITE_DATA_DIRECT                  16
#define PROC_READ_DATA_DIRECT                   17
#define PROC_CONFIGURE                          18
#define PROC_CONFIGURE_INTR                     20
#define PROC_NAME                               "ftxxxx-debug"
#define PROC_BUF_SIZE                           512

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
enum {
    RWREG_OP_READ = 0,
    RWREG_OP_WRITE = 1,
};

/*****************************************************************************
* Static variables
*****************************************************************************/
static struct rwreg_operation_t {
    int type;           /*  0: read, 1: write */
    int reg;            /*  register */
    int len;            /*  read/write length */
    int val;            /*  length = 1; read: return value, write: op return */
    int res;            /*  0: success, otherwise: fail */
    char *opbuf;        /*  length >= 1, read return value, write: op return */
} rw_op;

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
static ssize_t fts_debug_write(
    struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    u8 *writebuf = NULL;
    u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
    int buflen = count;
    int writelen = 0;
    int ret = 0;
    char tmp[PROC_BUF_SIZE];
    struct fts_ts_data *ts_data = fts_data;
    struct ftxxxx_proc *proc = &ts_data->proc;

    if (buflen <= 1) {
        FTS_ERROR("apk proc count(%d) fail", buflen);
        return -EINVAL;
    }

    if (buflen > PROC_BUF_SIZE) {
        writebuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
        if (NULL == writebuf) {
            FTS_ERROR("apk proc write buf zalloc fail");
            return -ENOMEM;
        }
    } else {
        writebuf = tmpbuf;
    }

    if (copy_from_user(writebuf, buff, buflen)) {
        FTS_ERROR("[APK]: copy from user error!!");
        ret = -EFAULT;
        goto proc_write_err;
    }

    proc->opmode = writebuf[0];
    switch (proc->opmode) {
    case PROC_SET_TEST_FLAG:
        FTS_DEBUG("[APK]: PROC_SET_TEST_FLAG = %x", writebuf[1]);
        if (writebuf[1] == 0) {
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(ENABLE);
#endif
        } else {
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(DISABLE);
#endif
        }
        break;

    case PROC_READ_REGISTER:
        proc->cmd[0] = writebuf[1];
        break;

    case PROC_WRITE_REGISTER:
        ret = fts_write_reg(writebuf[1], writebuf[2]);
        if (ret < 0) {
            FTS_ERROR("PROC_WRITE_REGISTER write error");
            goto proc_write_err;
        }
        break;

    case PROC_READ_DATA:
        writelen = buflen - 1;
        if (writelen >= FTS_MAX_COMMMAND_LENGTH) {
            FTS_ERROR("cmd(PROC_READ_DATA) length(%d) fail", writelen);
            goto proc_write_err;
        }
        memcpy(proc->cmd, writebuf + 1, writelen);
        proc->cmd_len = writelen;
        break;

    case PROC_WRITE_DATA:
        writelen = buflen - 1;
        ret = fts_write(writebuf + 1, writelen);
        if (ret < 0) {
            FTS_ERROR("PROC_WRITE_DATA write error");
            goto proc_write_err;
        }
        break;

    case PROC_SET_SLAVE_ADDR:
        break;

    case PROC_HW_RESET:
        if (buflen < PROC_BUF_SIZE) {
            snprintf(tmp, PROC_BUF_SIZE, "%s", writebuf + 1);
            tmp[buflen - 1] = '\0';
            if (strncmp(tmp, "focal_driver", 12) == 0) {
                FTS_INFO("APK execute HW Reset");
                fts_reset_proc(0);
            }
        }
        break;

    case PROC_SET_BOOT_MODE:
        FTS_DEBUG("[APK]: PROC_SET_BOOT_MODE = %x", writebuf[1]);
        if (0 == writebuf[1]) {
            ts_data->fw_is_running = true;
        } else {
            ts_data->fw_is_running = false;
        }
        break;
    case PROC_ENTER_TEST_ENVIRONMENT:
        FTS_DEBUG("[APK]: PROC_ENTER_TEST_ENVIRONMENT = %x", writebuf[1]);
        if (0 == writebuf[1]) {
            fts_enter_test_environment(0);
        } else {
            fts_enter_test_environment(1);
        }
        break;

    case PROC_READ_DATA_DIRECT:
        writelen = buflen - 1;
        if (writelen >= FTS_MAX_COMMMAND_LENGTH) {
            FTS_ERROR("cmd(PROC_READ_DATA_DIRECT) length(%d) fail", writelen);
            goto proc_write_err;
        }
        memcpy(proc->cmd, writebuf + 1, writelen);
        proc->cmd_len = writelen;
        break;

    case PROC_WRITE_DATA_DIRECT:
        writelen = buflen - 1;
        ret = fts_spi_transfer_direct(writebuf + 1, writelen, NULL, 0);
        if (ret < 0) {
            FTS_ERROR("PROC_WRITE_DATA_DIRECT write error");
            goto proc_write_err;
        }
        break;

    case PROC_CONFIGURE:
        ts_data->spi->mode = writebuf[1];
        ts_data->spi->bits_per_word = writebuf[2];
        ts_data->spi->max_speed_hz = *(u32 *)(writebuf + 4);
        FTS_INFO("spi,mode=%d,bits=%d,speed=%d", ts_data->spi->mode,
                 ts_data->spi->bits_per_word, ts_data->spi->max_speed_hz);
        ret = spi_setup(ts_data->spi);
        if (ret) {
            FTS_ERROR("spi setup fail");
            goto proc_write_err;
        }
        break;

    case PROC_CONFIGURE_INTR:
        if (writebuf[1] == 0)
            fts_irq_disable();
        else
            fts_irq_enable();
        break;

    default:
        break;
    }

    ret = buflen;
proc_write_err:
    if ((buflen > PROC_BUF_SIZE) && writebuf) {
        kfree(writebuf);
        writebuf = NULL;
    }
    return ret;
}

static ssize_t fts_debug_read(
    struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    int num_read_chars = 0;
    int buflen = count;
    u8 *readbuf = NULL;
    u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
    struct fts_ts_data *ts_data = fts_data;
    struct ftxxxx_proc *proc = &ts_data->proc;

    if (buflen <= 0) {
        FTS_ERROR("apk proc read count(%d) fail", buflen);
        return -EINVAL;
    }

    if (buflen > PROC_BUF_SIZE) {
        readbuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
        if (NULL == readbuf) {
            FTS_ERROR("apk proc buf zalloc fail");
            return -ENOMEM;
        }
    } else {
        readbuf = tmpbuf;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif

    switch (proc->opmode) {
    case PROC_READ_REGISTER:
        num_read_chars = 1;
        ret = fts_read_reg(proc->cmd[0], &readbuf[0]);
        if (ret < 0) {
            FTS_ERROR("PROC_READ_REGISTER read error");
            goto proc_read_err;
        }
        break;
    case PROC_WRITE_REGISTER:
        break;

    case PROC_READ_DATA:
        num_read_chars = buflen;
        ret = fts_read(proc->cmd, proc->cmd_len, readbuf, num_read_chars);
        if (ret < 0) {
            FTS_ERROR("PROC_READ_DATA read error");
            goto proc_read_err;
        }
        break;

    case PROC_READ_DATA_DIRECT:
        num_read_chars = buflen;
        ret = fts_spi_transfer_direct(proc->cmd, proc->cmd_len, readbuf, num_read_chars);
        if (ret < 0) {
            FTS_ERROR("PROC_READ_DATA_DIRECT read error");
            goto proc_read_err;
        }
        break;

    case PROC_WRITE_DATA:
        break;

    default:
        break;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif

    ret = num_read_chars;
proc_read_err:
    if (copy_to_user(buff, readbuf, num_read_chars)) {
        FTS_ERROR("copy to user error");
        ret = -EFAULT;
    }

    if ((buflen > PROC_BUF_SIZE) && readbuf) {
        kfree(readbuf);
        readbuf = NULL;
    }
    return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops fts_proc_fops = {
    .proc_read   = fts_debug_read,
    .proc_write  = fts_debug_write,
};
#else
static const struct file_operations fts_proc_fops = {
    .owner  = THIS_MODULE,
    .read   = fts_debug_read,
    .write  = fts_debug_write,
};
#endif

#else
static int fts_debug_write(
    struct file *filp, const char __user *buff, unsigned long len, void *data)
{
    u8 *writebuf = NULL;
    u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
    int buflen = count;
    int writelen = 0;
    int ret = 0;
    char tmp[PROC_BUF_SIZE];
    struct fts_ts_data *ts_data = fts_data;
    struct ftxxxx_proc *proc = &ts_data->proc;

    if (buflen <= 1) {
        FTS_ERROR("apk proc write count(%d) fail", buflen);
        return -EINVAL;
    }

    if (buflen > PROC_BUF_SIZE) {
        writebuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
        if (NULL == writebuf) {
            FTS_ERROR("apk proc write buf zalloc fail");
            return -ENOMEM;
        }
    } else {
        writebuf = tmpbuf;
    }

    if (copy_from_user(writebuf, buff, buflen)) {
        FTS_ERROR("[APK]: copy from user error!!");
        ret = -EFAULT;
        goto proc_write_err;
    }

    proc->opmode = writebuf[0];
    switch (proc->opmode) {
    case PROC_SET_TEST_FLAG:
        FTS_DEBUG("[APK]: PROC_SET_TEST_FLAG = %x", writebuf[1]);
        if (writebuf[1] == 0) {
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(ENABLE);
#endif
        } else {
#if FTS_ESDCHECK_EN
            fts_esdcheck_switch(DISABLE);
#endif
        }
        break;

    case PROC_READ_REGISTER:
        proc->cmd[0] = writebuf[1];
        break;

    case PROC_WRITE_REGISTER:
        ret = fts_write_reg(writebuf[1], writebuf[2]);
        if (ret < 0) {
            FTS_ERROR("PROC_WRITE_REGISTER write error");
            goto proc_write_err;
        }
        break;

    case PROC_READ_DATA:
        writelen = buflen - 1;
        if (writelen >= FTS_MAX_COMMMAND_LENGTH) {
            FTS_ERROR("cmd(PROC_READ_DATA) length(%d) fail", writelen);
            goto proc_write_err;
        }
        memcpy(proc->cmd, writebuf + 1, writelen);
        proc->cmd_len = writelen;
        break;

    case PROC_WRITE_DATA:
        writelen = buflen - 1;
        ret = fts_write(writebuf + 1, writelen);
        if (ret < 0) {
            FTS_ERROR("PROC_WRITE_DATA write error");
            goto proc_write_err;
        }
        break;

    case PROC_SET_SLAVE_ADDR:
        break;

    case PROC_HW_RESET:
        if (buflen < PROC_BUF_SIZE) {
            snprintf(tmp, PROC_BUF_SIZE, "%s", writebuf + 1);
            tmp[buflen - 1] = '\0';
            if (strncmp(tmp, "focal_driver", 12) == 0) {
                FTS_INFO("APK execute HW Reset");
                fts_reset_proc(0);
            }
        }
        break;

    case PROC_SET_BOOT_MODE:
        FTS_DEBUG("[APK]: PROC_SET_BOOT_MODE = %x", writebuf[1]);
        if (0 == writebuf[1]) {
            ts_data->fw_is_running = true;
        } else {
            ts_data->fw_is_running = false;
        }
        break;
    case PROC_ENTER_TEST_ENVIRONMENT:
        FTS_DEBUG("[APK]: PROC_ENTER_TEST_ENVIRONMENT = %x", writebuf[1]);
        if (0 == writebuf[1]) {
            fts_enter_test_environment(0);
        } else {
            fts_enter_test_environment(1);
        }
        break;

    case PROC_READ_DATA_DIRECT:
        writelen = buflen - 1;
        if (writelen >= FTS_MAX_COMMMAND_LENGTH) {
            FTS_ERROR("cmd(PROC_READ_DATA_DIRECT) length(%d) fail", writelen);
            goto proc_write_err;
        }
        memcpy(proc->cmd, writebuf + 1, writelen);
        proc->cmd_len = writelen;
        break;

    case PROC_WRITE_DATA_DIRECT:
        writelen = buflen - 1;
        ret = fts_spi_transfer_direct(writebuf + 1, writelen, NULL, 0);
        if (ret < 0) {
            FTS_ERROR("PROC_WRITE_DATA_DIRECT write error");
            goto proc_write_err;
        }
        break;

    case PROC_CONFIGURE:
        ts_data->spi->mode = writebuf[1];
        ts_data->spi->bits_per_word = writebuf[2];
        ts_data->spi->max_speed_hz = *(u32 *)(writebuf + 4);
        FTS_INFO("spi,mode=%d,bits=%d,speed=%d", ts_data->spi->mode,
                 ts_data->spi->bits_per_word, ts_data->spi->max_speed_hz);
        ret = spi_setup(ts_data->spi);
        if (ret) {
            FTS_ERROR("spi setup fail");
            goto proc_write_err;
        }
        break;

    case PROC_CONFIGURE_INTR:
        if (writebuf[1] == 0)
            fts_irq_disable();
        else
            fts_irq_enable();
        break;

    default:
        break;
    }

    ret = buflen;
proc_write_err:
    if ((buflen > PROC_BUF_SIZE) && writebuf) {
        kfree(writebuf);
        writebuf = NULL;
    }
    return ret;
}

static int fts_debug_read(
    char *page, char **start, off_t off, int count, int *eof, void *data )
{
    int ret = 0;
    int num_read_chars = 0;
    int buflen = count;
    u8 *readbuf = NULL;
    u8 tmpbuf[PROC_BUF_SIZE] = { 0 };
    struct fts_ts_data *ts_data = fts_data;
    struct ftxxxx_proc *proc = &ts_data->proc;

    if (buflen <= 0) {
        FTS_ERROR("apk proc read count(%d) fail", buflen);
        return -EINVAL;
    }

    if (buflen > PROC_BUF_SIZE) {
        readbuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
        if (NULL == readbuf) {
            FTS_ERROR("apk proc buf zalloc fail");
            return -ENOMEM;
        }
    } else {
        readbuf = tmpbuf;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif

    switch (proc->opmode) {
    case PROC_READ_REGISTER:
        num_read_chars = 1;
        ret = fts_read_reg(proc->cmd[0], &readbuf[0]);
        if (ret < 0) {
            FTS_ERROR("PROC_READ_REGISTER read error");
            goto proc_read_err;
        }
        break;
    case PROC_WRITE_REGISTER:
        break;

    case PROC_READ_DATA:
        num_read_chars = buflen;
        ret = fts_read(proc->cmd, proc->cmd_len, readbuf, num_read_chars);
        if (ret < 0) {
            FTS_ERROR("PROC_READ_DATA read error");
            goto proc_read_err;
        }
        break;

    case PROC_READ_DATA_DIRECT:
        num_read_chars = buflen;
        ret = fts_spi_transfer_direct(proc->cmd, proc->cmd_len, readbuf, num_read_chars);
        if (ret < 0) {
            FTS_ERROR("PROC_READ_DATA_DIRECT read error");
            goto proc_read_err;
        }
        break;

    case PROC_WRITE_DATA:
        break;

    default:
        break;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif

    ret = num_read_chars;
proc_read_err:
    if (copy_to_user(buff, readbuf, num_read_chars)) {
        FTS_ERROR("copy to user error");
        ret = -EFAULT;
    }

    if ((buflen > PROC_BUF_SIZE) && readbuf) {
        kfree(readbuf);
        readbuf = NULL;
    }
    return ret;
}
#endif

int fts_create_apk_debug_channel(struct fts_ts_data *ts_data)
{
    struct ftxxxx_proc *proc = &ts_data->proc;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
    proc->proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);
    if (NULL == proc->proc_entry) {
        FTS_ERROR("create proc entry fail");
        return -ENOMEM;
    }
#else
    proc->proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
    if (NULL == proc->proc_entry) {
        FTS_ERROR("create proc entry fail");
        return -ENOMEM;
    }
    proc->proc_entry->write_proc = fts_debug_write;
    proc->proc_entry->read_proc = fts_debug_read;
#endif

    FTS_INFO("Create proc entry success!");
    return 0;
}

void fts_release_apk_debug_channel(struct fts_ts_data *ts_data)
{
    struct ftxxxx_proc *proc = &ts_data->proc;

    if (proc->proc_entry) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
        proc_remove(proc->proc_entry);
#else
        remove_proc_entry(PROC_NAME, NULL);
#endif
    }
}

/************************************************************************
 * sysfs interface
 ***********************************************************************/
/* fts_hw_reset interface */
static ssize_t fts_hw_reset_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    struct input_dev *input_dev = fts_data->input_dev;
    ssize_t count = 0;

    mutex_lock(&input_dev->mutex);
    fts_reset_proc(0);
    count = snprintf(buf, PAGE_SIZE, "hw reset executed\n");
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_hw_reset_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

/* fts_irq interface */
static ssize_t fts_irq_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t count = 0;

    count = snprintf(buf, PAGE_SIZE, "irq_enable:%d\n",
        !fts_data->irq_disabled);

    return count;
}

static ssize_t fts_irq_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        FTS_INFO("enable irq");
        fts_irq_enable();
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        FTS_INFO("disable irq");
        fts_irq_disable();
    }
    mutex_unlock(&input_dev->mutex);
    return count;
}

/* fts_boot_mode interface */
static ssize_t fts_bootmode_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    if (FTS_SYSFS_ECHO_ON(buf)) {
        FTS_INFO("[EX-FUN]set to boot mode");
        fts_data->fw_is_running = false;
    } else if (FTS_SYSFS_ECHO_OFF(buf)) {
        FTS_INFO("[EX-FUN]set to fw mode");
        fts_data->fw_is_running = true;
    }
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

static ssize_t fts_bootmode_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    ssize_t count = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    if (true == fts_data->fw_is_running) {
        count = snprintf(buf, PAGE_SIZE, "tp is in fw mode\n");
    } else {
        count = snprintf(buf, PAGE_SIZE, "tp is in boot mode\n");
    }
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

/* fts_tpfwver interface */
static ssize_t fts_tpfwver_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;
    ssize_t num_read_chars = 0;
    u8 fw_major_ver = 0;
    u8 fw_minor_ver = 0;

    mutex_lock(&input_dev->mutex);

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    ret = fts_read_reg(FTS_REG_FW_MAJOR_VER, &fw_major_ver);
    if ((ret < 0) || (fw_major_ver == 0xFF) || (fw_major_ver == 0x00)) {
        num_read_chars = snprintf(buf, PAGE_SIZE,
                                  "get tp fw major version fail!\n");
        mutex_unlock(&input_dev->mutex);
        return num_read_chars;
    }
    ret = fts_read_reg(FTS_REG_FW_MINOR_VER, &fw_minor_ver);
    if (ret < 0) {
        num_read_chars = snprintf(buf, PAGE_SIZE,
                                  "get tp fw minor version fail!\n");
        mutex_unlock(&input_dev->mutex);
        return num_read_chars;
    }
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif
    num_read_chars = snprintf(buf, PAGE_SIZE, "V%02x_D%02x\n", fw_major_ver,
        fw_minor_ver);

    mutex_unlock(&input_dev->mutex);
    return num_read_chars;
}

static ssize_t fts_tpfwver_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

/* fts_rw_reg */
static ssize_t fts_tprwreg_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    int i;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);

    if (rw_op.len < 0) {
        count = snprintf(buf, PAGE_SIZE, "Invalid cmd line\n");
    } else if (rw_op.len == 1) {
        if (RWREG_OP_READ == rw_op.type) {
            if (rw_op.res == 0) {
                count = snprintf(buf, PAGE_SIZE, "Read %02X: %02X\n", rw_op.reg, rw_op.val);
            } else {
                count = snprintf(buf, PAGE_SIZE, "Read %02X failed, ret: %d\n", rw_op.reg,  rw_op.res);
            }
        } else {
            if (rw_op.res == 0) {
                count = snprintf(buf, PAGE_SIZE, "Write %02X, %02X success\n", rw_op.reg,  rw_op.val);
            } else {
                count = snprintf(buf, PAGE_SIZE, "Write %02X failed, ret: %d\n", rw_op.reg,  rw_op.res);
            }
        }
    } else {
        if (RWREG_OP_READ == rw_op.type) {
            count = snprintf(buf, PAGE_SIZE, "Read Reg: [%02X]-[%02X]\n", rw_op.reg, rw_op.reg + rw_op.len);
            count += snprintf(buf + count, PAGE_SIZE, "Result: ");
            if (rw_op.res) {
                count += snprintf(buf + count, PAGE_SIZE, "failed, ret: %d\n", rw_op.res);
            } else {
                if (rw_op.opbuf) {
                    for (i = 0; i < rw_op.len; i++) {
                        count += snprintf(buf + count, PAGE_SIZE, "%02X ", rw_op.opbuf[i]);
                    }
                    count += snprintf(buf + count, PAGE_SIZE, "\n");
                }
            }
        } else {
            ;
            count = snprintf(buf, PAGE_SIZE, "Write Reg: [%02X]-[%02X]\n", rw_op.reg, rw_op.reg + rw_op.len - 1);
            count += snprintf(buf + count, PAGE_SIZE, "Write Data: ");
            if (rw_op.opbuf) {
                for (i = 1; i < rw_op.len; i++) {
                    count += snprintf(buf + count, PAGE_SIZE, "%02X ", rw_op.opbuf[i]);
                }
                count += snprintf(buf + count, PAGE_SIZE, "\n");
            }
            if (rw_op.res) {
                count += snprintf(buf + count, PAGE_SIZE, "Result: failed, ret: %d\n", rw_op.res);
            } else {
                count += snprintf(buf + count, PAGE_SIZE, "Result: success\n");
            }
        }
        /*if (rw_op.opbuf) {
            kfree(rw_op.opbuf);
            rw_op.opbuf = NULL;
        }*/
    }
    mutex_unlock(&input_dev->mutex);

    return count;
}

static int shex_to_int(const char *hex_buf, int size)
{
    int i;
    int base = 1;
    int value = 0;
    char single;

    for (i = size - 1; i >= 0; i--) {
        single = hex_buf[i];

        if ((single >= '0') && (single <= '9')) {
            value += (single - '0') * base;
        } else if ((single >= 'a') && (single <= 'z')) {
            value += (single - 'a' + 10) * base;
        } else if ((single >= 'A') && (single <= 'Z')) {
            value += (single - 'A' + 10) * base;
        } else {
            return -EINVAL;
        }

        base *= 16;
    }

    return value;
}


static u8 shex_to_u8(const char *hex_buf, int size)
{
    return (u8)shex_to_int(hex_buf, size);
}
/*
 * Format buf:
 * [0]: '0' write, '1' read(reserved)
 * [1-2]: addr, hex
 * [3-4]: length, hex
 * [5-6]...[n-(n+1)]: data, hex
 */
static int fts_parse_buf(const char *buf, size_t cmd_len)
{
    int length;
    int i;
    char *tmpbuf;

    rw_op.reg = shex_to_u8(buf + 1, 2);
    length = shex_to_int(buf + 3, 2);

    if (buf[0] == '1') {
        rw_op.len = length;
        rw_op.type = RWREG_OP_READ;
        FTS_DEBUG("read %02X, %d bytes", rw_op.reg, rw_op.len);
    } else {
        if (cmd_len < (length * 2 + 5)) {
            pr_err("data invalided!\n");
            return -EINVAL;
        }
        FTS_DEBUG("write %02X, %d bytes", rw_op.reg, length);

        /* first byte is the register addr */
        rw_op.type = RWREG_OP_WRITE;
        rw_op.len = length + 1;
    }

    if (rw_op.len > 0) {
        tmpbuf = (char *)kzalloc(rw_op.len, GFP_KERNEL);
        if (!tmpbuf) {
            FTS_ERROR("allocate memory failed!\n");
            return -ENOMEM;
        }

        if (RWREG_OP_WRITE == rw_op.type) {
            tmpbuf[0] = rw_op.reg & 0xFF;
            FTS_DEBUG("write buffer: ");
            for (i = 1; i < rw_op.len; i++) {
                tmpbuf[i] = shex_to_u8(buf + 5 + i * 2 - 2, 2);
                FTS_DEBUG("buf[%d]: %02X", i, tmpbuf[i] & 0xFF);
            }
        }
        rw_op.opbuf = tmpbuf;
    }

    return rw_op.len;
}

static ssize_t fts_tprwreg_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input_dev = fts_data->input_dev;
    ssize_t cmd_length = 0;

    mutex_lock(&input_dev->mutex);
    cmd_length = count - 1;

    if (rw_op.opbuf) {
        kfree(rw_op.opbuf);
        rw_op.opbuf = NULL;
    }

    FTS_DEBUG("cmd len: %d, buf: %s", (int)cmd_length, buf);
    /* compatible old ops */
    if (2 == cmd_length) {
        rw_op.type = RWREG_OP_READ;
        rw_op.len = 1;
        rw_op.reg = shex_to_int(buf, 2);
    } else if (4 == cmd_length) {
        rw_op.type = RWREG_OP_WRITE;
        rw_op.len = 1;
        rw_op.reg = shex_to_int(buf, 2);
        rw_op.val = shex_to_int(buf + 2, 2);
    } else if (cmd_length < 5) {
        FTS_ERROR("Invalid cmd buffer");
        mutex_unlock(&input_dev->mutex);
        return -EINVAL;
    } else {
        rw_op.len = fts_parse_buf(buf, cmd_length);
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    if (rw_op.len < 0) {
        FTS_ERROR("cmd buffer error!");

    } else {
        if (RWREG_OP_READ == rw_op.type) {
            if (rw_op.len == 1) {
                u8 reg, val;
                reg = rw_op.reg & 0xFF;
                rw_op.res = fts_read_reg(reg, &val);
                rw_op.val = val;
            } else {
                char reg;
                reg = rw_op.reg & 0xFF;

                rw_op.res = fts_read(&reg, 1, rw_op.opbuf, rw_op.len);
            }

            if (rw_op.res < 0) {
                FTS_ERROR("Could not read 0x%02x", rw_op.reg);
            } else {
                FTS_INFO("read 0x%02x, %d bytes successful", rw_op.reg, rw_op.len);
                rw_op.res = 0;
            }

        } else {
            if (rw_op.len == 1) {
                u8 reg, val;
                reg = rw_op.reg & 0xFF;
                val = rw_op.val & 0xFF;
                rw_op.res = fts_write_reg(reg, val);
            } else {
                rw_op.res = fts_write(rw_op.opbuf, rw_op.len);
            }
            if (rw_op.res < 0) {
                FTS_ERROR("Could not write 0x%02x", rw_op.reg);

            } else {
                FTS_INFO("Write 0x%02x, %d bytes successful", rw_op.val, rw_op.len);
                rw_op.res = 0;
            }
        }
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif
    mutex_unlock(&input_dev->mutex);

    return count;
}

/* fts_upgrade_bin interface */
static ssize_t fts_fwupgradebin_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

static ssize_t fts_fwupgradebin_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    char fwname[FILE_NAME_LENGTH] = { 0 };
    struct input_dev *input_dev = fts_data->input_dev;

    if ((count <= 1) || (count >= FILE_NAME_LENGTH - 32)) {
        FTS_ERROR("fw bin name's length(%d) fail", (int)count);
        return -EINVAL;
    }
    memset(fwname, 0, sizeof(fwname));
    snprintf(fwname, FILE_NAME_LENGTH, "%s", buf);
    fwname[count - 1] = '\0';

    FTS_INFO("upgrade with bin file through sysfs node");
    mutex_lock(&input_dev->mutex);
    fts_upgrade_bin(fwname, 0);
    mutex_unlock(&input_dev->mutex);

    return count;
}

/* fts_force_upgrade interface */
static ssize_t fts_fwforceupg_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return -EPERM;
}

static ssize_t fts_fwforceupg_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    char fwname[FILE_NAME_LENGTH];
    struct input_dev *input_dev = fts_data->input_dev;

    if ((count <= 1) || (count >= FILE_NAME_LENGTH - 32)) {
        FTS_ERROR("fw bin name's length(%d) fail", (int)count);
        return -EINVAL;
    }
    memset(fwname, 0, sizeof(fwname));
    snprintf(fwname, FILE_NAME_LENGTH, "%s", buf);
    fwname[count - 1] = '\0';

    FTS_INFO("force upgrade through sysfs node");
    mutex_lock(&input_dev->mutex);
    fts_upgrade_bin(fwname, 1);
    mutex_unlock(&input_dev->mutex);

    return count;
}

/* fts_driver_info interface */
static ssize_t fts_driverinfo_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct fts_ts_data *ts_data = fts_data;
    struct fts_ts_platform_data *pdata = ts_data->pdata;
    struct input_dev *input_dev = ts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    count += snprintf(buf + count, PAGE_SIZE, "Driver Ver:%s\n",
                      FTS_DRIVER_VERSION);

    count += snprintf(buf + count, PAGE_SIZE, "Resolution:(%d,%d)~(%d,%d)\n",
                      pdata->x_min, pdata->y_min, pdata->x_max, pdata->y_max);

    count += snprintf(buf + count, PAGE_SIZE, "Max Touchs:%d\n",
                      pdata->max_touch_number);

    count += snprintf(buf + count, PAGE_SIZE,
                      "reset gpio:%d,int gpio:%d,irq:%d\n",
                      pdata->reset_gpio, pdata->irq_gpio, ts_data->irq);

    count += snprintf(buf + count, PAGE_SIZE, "IC ID:0x%02x%02x\n",
                      ts_data->ic_info.ids.chip_idh,
                      ts_data->ic_info.ids.chip_idl);

    if (ts_data->bus_type == FTS_BUS_TYPE_I2C) {
        count += snprintf(buf + count, PAGE_SIZE, "BUS:%s,addr:0x%x\n",
                          "I2C", ts_data->client->addr);
    } else {
        count += snprintf(buf + count, PAGE_SIZE,
                          "BUS:%s,mode:%d,max_freq:%d\n", "SPI",
                          ts_data->spi->mode, ts_data->spi->max_speed_hz);
    }
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_driverinfo_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

/* fts_dump_reg interface */
static ssize_t fts_dumpreg_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    u8 val = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(1);
#endif
    fts_read_reg(FTS_REG_POWER_MODE, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Power Mode:0x%02x\n", val);

    fts_read_reg(FTS_REG_FW_MAJOR_VER, &val);
    count += snprintf(buf + count, PAGE_SIZE, "FW Major Ver:0x%02x\n", val);

    fts_read_reg(FTS_REG_FW_MINOR_VER, &val);
    count += snprintf(buf + count, PAGE_SIZE, "FW Minor Ver:0x%02x\n", val);

    fts_read_reg(FTS_REG_LIC_VER, &val);
    count += snprintf(buf + count, PAGE_SIZE, "LCD Initcode Ver:0x%02x\n", val);

    fts_read_reg(FTS_REG_IDE_PARA_VER_ID, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Param Ver:0x%02x\n", val);

    fts_read_reg(FTS_REG_IDE_PARA_STATUS, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Param status:0x%02x\n", val);

    fts_read_reg(FTS_REG_VENDOR_ID, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Vendor ID:0x%02x\n", val);

    fts_read_reg(FTS_REG_GESTURE_EN, &val);
    count += snprintf(buf + count, PAGE_SIZE, "Gesture Mode:0x%02x\n", val);

    fts_read_reg(FTS_REG_CHARGER_MODE_EN, &val);
    count += snprintf(buf + count, PAGE_SIZE, "charge stat:0x%02x\n", val);

    fts_read_reg(FTS_REG_INT_CNT, &val);
    count += snprintf(buf + count, PAGE_SIZE, "INT count:0x%02x\n", val);

    fts_read_reg(FTS_REG_FLOW_WORK_CNT, &val);
    count += snprintf(buf + count, PAGE_SIZE, "ESD count:0x%02x\n", val);
#if FTS_ESDCHECK_EN
    fts_esdcheck_proc_busy(0);
#endif

    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_dumpreg_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

/* fts_dump_reg interface */
static ssize_t fts_tpbuf_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    int i = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    count += snprintf(buf + count, PAGE_SIZE, "touch point buffer:\n");
    for (i = 0; i < fts_data->pnt_buf_size; i++) {
        count += snprintf(buf + count, PAGE_SIZE, "%02x ", fts_data->point_buf[i]);
    }
    count += snprintf(buf + count, PAGE_SIZE, "\n");
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_tpbuf_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}

/* fts_log_level interface */
static ssize_t fts_log_level_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    mutex_lock(&input_dev->mutex);
    count += snprintf(buf + count, PAGE_SIZE, "log level:%d\n",
                      fts_data->log_level);
    mutex_unlock(&input_dev->mutex);

    return count;
}

static ssize_t fts_log_level_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int value = 0;
    struct input_dev *input_dev = fts_data->input_dev;

    FTS_FUNC_ENTER();
    mutex_lock(&input_dev->mutex);
    sscanf(buf, "%d", &value);
    FTS_DEBUG("log level:%d->%d", fts_data->log_level, value);
    fts_data->log_level = value;
    mutex_unlock(&input_dev->mutex);
    FTS_FUNC_EXIT();

    return count;
}

/* get the fw version  example:cat fw_version */
static DEVICE_ATTR(fts_fw_version, S_IRUGO | S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

/* read and write register(s)
*   All data type is **HEX**
*   Single Byte:
*       read:   echo 88 > rw_reg ---read register 0x88
*       write:  echo 8807 > rw_reg ---write 0x07 into register 0x88
*   Multi-bytes:
*       [0:rw-flag][1-2: reg addr, hex][3-4: length, hex][5-6...n-n+1: write data, hex]
*       rw-flag: 0, write; 1, read
*       read:  echo 10005           > rw_reg ---read reg 0x00-0x05
*       write: echo 000050102030405 > rw_reg ---write reg 0x00-0x05 as 01,02,03,04,05
*  Get result:
*       cat rw_reg
*/
static DEVICE_ATTR(fts_rw_reg, S_IRUGO | S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*  upgrade from fw bin file   example:echo "*.bin" > fts_upgrade_bin */
static DEVICE_ATTR(fts_upgrade_bin, S_IRUGO | S_IWUSR, fts_fwupgradebin_show, fts_fwupgradebin_store);
static DEVICE_ATTR(fts_force_upgrade, S_IRUGO | S_IWUSR, fts_fwforceupg_show, fts_fwforceupg_store);
static DEVICE_ATTR(fts_driver_info, S_IRUGO | S_IWUSR, fts_driverinfo_show, fts_driverinfo_store);
static DEVICE_ATTR(fts_dump_reg, S_IRUGO | S_IWUSR, fts_dumpreg_show, fts_dumpreg_store);
static DEVICE_ATTR(fts_hw_reset, S_IRUGO | S_IWUSR, fts_hw_reset_show, fts_hw_reset_store);
static DEVICE_ATTR(fts_irq, S_IRUGO | S_IWUSR, fts_irq_show, fts_irq_store);
static DEVICE_ATTR(fts_boot_mode, S_IRUGO | S_IWUSR, fts_bootmode_show, fts_bootmode_store);
static DEVICE_ATTR(fts_touch_point, S_IRUGO | S_IWUSR, fts_tpbuf_show, fts_tpbuf_store);
static DEVICE_ATTR(fts_log_level, S_IRUGO | S_IWUSR, fts_log_level_show, fts_log_level_store);

/* add your attr in here*/
static struct attribute *fts_attributes[] = {
    &dev_attr_fts_fw_version.attr,
    &dev_attr_fts_rw_reg.attr,
    &dev_attr_fts_dump_reg.attr,
    &dev_attr_fts_upgrade_bin.attr,
    &dev_attr_fts_force_upgrade.attr,
    &dev_attr_fts_driver_info.attr,
    &dev_attr_fts_hw_reset.attr,
    &dev_attr_fts_irq.attr,
    &dev_attr_fts_boot_mode.attr,
    &dev_attr_fts_touch_point.attr,
    &dev_attr_fts_log_level.attr,
    NULL
};

static struct attribute_group fts_attribute_group = {
    .attrs = fts_attributes
};

static ssize_t proc_fw_update_write(struct file *filp, const char __user *buff,
    size_t count, loff_t *ppos)
{
    struct fts_ts_data *ts_data = pde_data(file_inode(filp));
    char fwname[FILE_NAME_LENGTH] = { 0 };
    int buflen = count;

    FTS_INFO("upgrade with bin file through proc node");
    if (!ts_data) {
        FTS_ERROR("ts_data is null");
        return -EINVAL;
    }

    if ((buflen <= 0) || (buflen >= FILE_NAME_LENGTH)) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(fwname, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }
    fwname[buflen - 1] = '\0';

    mutex_lock(&ts_data->input_dev->mutex);
    fts_upgrade_bin(fwname, 0);
    mutex_unlock(&ts_data->input_dev->mutex);

    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_fw_update_fops = {
    .proc_write  = proc_fw_update_write,
};
#else
static const struct file_operations proc_fw_update_fops = {
    .owner  = THIS_MODULE,
    .write = proc_fw_update_write,
};
#endif

/* scan modes */
static ssize_t proc_scan_modes_read(struct file *filp, char __user *buff,
    size_t count, loff_t *ppos)
{
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int cnt = 0;
    loff_t pos = *ppos;

    if (pos)
        return 0;

    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "%s\n",
        "scan_modes=0,1,2,3,4");
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "%s\n",
        "0:Auto mode");
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "%s\n",
         "1:Normal Active");
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "%s\n",
        "2:Normal Idle");
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "%s\n",
        "3:Low Power Active");
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "%s\n",
        "4:Low Power Idle");

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_scan_modes_fops = {
    .proc_read   = proc_scan_modes_read,
};
#else
static const struct file_operations proc_scan_modes_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_scan_modes_read,
};
#endif

/* touch mode */
static ssize_t proc_touch_mode_read(struct file *filp, char __user *buff,
    size_t count, loff_t *ppos)
{
    int cnt = 0;
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    u8 gesture_mode = 0;
    u8 power_mode = 0;
    u8 monitor_ctrl = 0;
    loff_t pos = *ppos;

    if (pos)
        return 0;

    ret = fts_read_reg(FTS_REG_GESTURE_EN, &gesture_mode);
    if (ret < 0) {
        FTS_ERROR("read reg0xD0 fails");
        return ret;
    }

    ret = fts_read_reg(FTS_REG_POWER_MODE, &power_mode);
    if (ret < 0) {
        FTS_ERROR("read reg0xA5 fails");
        return ret;
    }

    ret = fts_read_reg(FTS_REG_MONITOR_CTRL, &monitor_ctrl);
    if (ret < 0) {
        FTS_ERROR("read reg0x86 fails");
        return ret;
    }

    if (gesture_mode) {
        if (power_mode == 0)
            cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
                "touch_mode:%d-%s\n", MODE_LOW_POWER_ACTIVE, "Low Power Active");
        else if (power_mode == 1)
            cnt += snprintf(tmpbuf + cnt,  PROC_BUF_SIZE - cnt,
                "touch_mode:%d-%s\n", MODE_LOW_POWER_IDLE, "Low Power Idle");
    } else if (monitor_ctrl) {
        cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
            "touch_mode:%d-%s\n", MODE_AUTO, "Auto mode");
    } else {
        if (power_mode == 0)
            cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
                "touch_mode:%d-%s\n", MODE_NORMAL_ACTIVE, "Normal Active");
        else if (power_mode == 1)
            cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
                "touch_mode:%d-%s\n", MODE_NORMAL_IDLE, "Normal Idle");
    }

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

static ssize_t proc_touch_mode_write(struct file *filp, const char __user *buff,
    size_t count, loff_t *ppos)
{
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int touch_mode = 0xFF;
    int buflen = count;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%d", &touch_mode);
    if ((ret != 1) || (touch_mode < 0) || (touch_mode >= MODE_CNT)) {
        FTS_ERROR("get mode(%d) fails,ret=%d", touch_mode, ret);
        return -EINVAL;
    }

    FTS_INFO("switch touch_mode to %d", touch_mode);
    switch (touch_mode) {
    case MODE_AUTO:
        ret = fts_write_reg(FTS_REG_MONITOR_CTRL, 1);
        if (ret < 0) {
            FTS_ERROR("write reg0x86 fails");
            return ret;
        }

        ret = fts_write_reg(FTS_REG_GESTURE_EN, 0);
        if (ret < 0) {
            FTS_ERROR("write reg0xd0 fails");
            return ret;
        }
        break;
    case MODE_NORMAL_ACTIVE:
        ret = fts_write_reg(FTS_REG_MONITOR_CTRL, 0);
        if (ret < 0) {
            FTS_ERROR("write reg0x86 fails");
            return ret;
        }

        ret = fts_write_reg(FTS_REG_POWER_MODE, 0);
        if (ret < 0) {
            FTS_ERROR("write reg0xA5 fails");
            return ret;
        }

        ret = fts_write_reg(FTS_REG_GESTURE_EN, 0);
        if (ret < 0) {
            FTS_ERROR("write reg0xD0 fails");
            return ret;
        }
        break;
    case MODE_NORMAL_IDLE:
        ret = fts_write_reg(FTS_REG_MONITOR_CTRL, 0);
        if (ret < 0) {
            FTS_ERROR("write reg0x86 fails");
            return ret;
        }

        ret = fts_write_reg(FTS_REG_POWER_MODE, 1);
        if (ret < 0) {
            FTS_ERROR("write reg0xA5 fails");
            return ret;
        }

        ret = fts_write_reg(FTS_REG_GESTURE_EN, 0);
        if (ret < 0) {
            FTS_ERROR("write reg0xD0 fails");
            return ret;
        }
        break;
    case MODE_LOW_POWER_ACTIVE:
        ret = fts_write_reg(FTS_REG_GESTURE_EN, 1);
        if (ret < 0) {
            FTS_ERROR("write reg0xD0 fails");
            return ret;
        }

        ret = fts_write_reg(FTS_REG_POWER_MODE, 0);
        if (ret < 0) {
            FTS_ERROR("write reg0xA5 fails");
            return ret;
        }
        break;
    case MODE_LOW_POWER_IDLE:
        ret = fts_write_reg(FTS_REG_GESTURE_EN, 1);
        if (ret < 0) {
            FTS_ERROR("write reg0xD0 fails");
            return ret;
        }

        ret = fts_write_reg(FTS_REG_POWER_MODE, 1);
        if (ret < 0) {
            FTS_ERROR("write reg0xA5 fails");
            return ret;
        }
        break;
    default:
        FTS_ERROR("Input index of mode is out of range!");
        break;
    }

    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_touch_mode_fops = {
    .proc_read   = proc_touch_mode_read,
    .proc_write  = proc_touch_mode_write,
};
#else
static const struct file_operations proc_touch_mode_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_touch_mode_read,
    .write  = proc_touch_mode_write,
};
#endif

/* lpwg */
static ssize_t proc_lpwg_read(struct file *filp, char __user *buff,
    size_t count, loff_t *ppos)
{
    int cnt = 0;
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    u8 gesture_mode = 0;
    u8 gesture_function = 0;
    loff_t pos = *ppos;

    if (pos)
        return 0;

    ret = fts_read_reg(FTS_REG_GESTURE_EN, &gesture_mode);
    if (ret < 0) {
        FTS_ERROR("read reg0xD0 fails");
        return ret;
    }

    ret = fts_read_reg(FTS_REG_GESTURE_SWITCH, &gesture_function);
    if (ret < 0) {
        FTS_ERROR("read reg0xCF fails");
        return ret;
    }

    if (gesture_function == 1)
        cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "Gesture_mode: STTW\n");
    else if (gesture_function == 2)
        cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "Gesture_mode: LPTW\n");
    else if (gesture_function == 3)
        cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "Gesture_mode: STTW + LPTW\n");
    else if (gesture_function == 0)
        cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "Disable STTW and LPTW\n");

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

static ssize_t proc_lpwg_write(struct file *filp, const char __user *buff,
    size_t count, loff_t *ppos)
{
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int gesture_mode = 0xFF;
    int buflen = count;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%d", &gesture_mode);
    if (ret != 1) {
        FTS_ERROR("get mode fails,ret=%d", ret);
        return -EINVAL;
    }

    FTS_INFO("switch gesture mode to %d", gesture_mode);

    switch (gesture_mode) {
    case 0:
         ret = fts_write_reg(FTS_REG_GESTURE_SWITCH, 0);
         if (ret < 0) {
            FTS_ERROR("write reg 0xCF fails");
            return ret;
         }
         break;

    case 1:    //Single tap
         ret = fts_write_reg(FTS_REG_GESTURE_SWITCH, 1);
         if (ret < 0) {
             FTS_ERROR("write reg 0xCF fails");
             return ret;
         }
         FTS_INFO("switch gesture function to STTW");
         break;

    case 2:    //Long press
         ret = fts_write_reg(FTS_REG_GESTURE_SWITCH, 2);
         if (ret < 0) {
             FTS_ERROR("write reg 0xCF fails");
             return ret;
         }
         FTS_INFO("switch gesture function to LPTW");
         break;

    case 3:    //Single tap + Long press
         ret = fts_write_reg(FTS_REG_GESTURE_SWITCH, 3);
         if (ret < 0) {
             FTS_ERROR("write reg 0xCF fails");
             return ret;
          }
         FTS_INFO("switch gesture function to STTW + LPTW");
         break;

    default:
         break;
   }
    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_lpwg_fops = {
    .proc_read   = proc_lpwg_read,
    .proc_write  = proc_lpwg_write,
};
#else
static const struct file_operations proc_lpwg_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_lpwg_read,
    .write  = proc_lpwg_write,
};
#endif

/* high sensitivity */
static ssize_t proc_hs_read(struct file *filp, char __user *buff, size_t count,
    loff_t *ppos)
{
    int cnt = 0;
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    u8 hs_mode = 0;
    loff_t pos = *ppos;

    if (pos)
        return 0;

    ret = fts_read_reg(FTS_REG_GLOVE_MODE_EN, &hs_mode);
    if (ret < 0) {
        FTS_ERROR("read reg0xC0 fails");
        return ret;
    }

    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "high_sensitivity mode:%s\n", hs_mode ? "Enable" : "Disable");

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

static ssize_t proc_hs_write(struct file *filp, const char __user *buff,
    size_t count, loff_t *ppos)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int hs_mode = 0xFF;
    int buflen = count;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%d", &hs_mode);
    if (ret != 1) {
        FTS_ERROR("get mode fails,ret=%d", ret);
        return -EINVAL;
    }

    ret = fts_set_glove_mode(ts_data, !!hs_mode);
    if (ret < 0)
      return ret;

    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_hs_fops = {
    .proc_read   = proc_hs_read,
    .proc_write  = proc_hs_write,
};
#else
static const struct file_operations proc_hs_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_hs_read,
    .write  = proc_hs_write,
};
#endif

/* palm */
static ssize_t proc_palm_read(struct file *filp, char __user *buff,
    size_t count, loff_t *ppos)
{
    int cnt = 0;
    struct fts_ts_data *ts_data = fts_data;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    loff_t pos = *ppos;

    if (pos)
        return 0;

    FTS_DEBUG("fw_palm = %d", ts_data->enable_fw_palm);
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "%u\n", ts_data->enable_fw_palm);

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

/* Set palm rejection mode.
 * 0 - Disable fw palm rejection.
 * 1 - Enable fw palm rejection.
 * 2 - Force disable fw palm rejection.
 * 3 - Force enable fw palm rejection.
 */
static ssize_t proc_palm_write(struct file *filp, const char __user *buff,
    size_t count, loff_t *ppos)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int palm_mode = 0xFF;
    int buflen = count;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%d", &palm_mode);
    if (ret != 1) {
        FTS_ERROR("get mode fails,ret=%d", ret);
        return -EINVAL;
    }

    if (palm_mode < 0 || palm_mode > 3) {
        FTS_ERROR("get palm mode fails, fw_palm should be in [0,1,2,3].");
        return -EINVAL;
    }

    ts_data->enable_fw_palm = palm_mode;
    FTS_INFO("switch fw_aplm to %u\n", ts_data->enable_fw_palm);

    ret = fts_set_palm_mode(ts_data, palm_mode);
    if (ret < 0) {
        return ret;
    }

    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_palm_fops = {
    .proc_read   = proc_palm_read,
    .proc_write  = proc_palm_write,
};
#else
static const struct file_operations proc_palm_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_palm_read,
    .write  = proc_palm_write,
};
#endif

/* grip */
static ssize_t proc_grip_read(struct file *filp, char __user *buff,
    size_t count, loff_t *ppos)
{
    int cnt = 0;
    struct fts_ts_data *ts_data = fts_data;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    loff_t pos = *ppos;

    if (pos)
        return 0;

    FTS_DEBUG("fw_grip = %u", ts_data->enable_fw_grip);
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "%u\n", ts_data->enable_fw_grip);

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

/* Set Grip suppression mode.
 * 0 - Disable fw grip suppression.
 * 1 - Enable fw grip suppression.
 * 2 - Force disable fw grip suppression.
 * 3 - Force enable fw grip suppression.
 */
static ssize_t proc_grip_write(struct file *filp, const char __user *buff,
    size_t count, loff_t *ppos)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int grip_mode = 0xFF;
    int buflen = count;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%d", &grip_mode);
    if (ret != 1) {
        FTS_ERROR("get mode fails,ret=%d", ret);
        return -EINVAL;
    }
    if (grip_mode < 0 || grip_mode > 3) {
        FTS_ERROR("get mode fails, grip_mode should be in [0,1,2,3].");
        return -EINVAL;
    }

    ts_data->enable_fw_grip = grip_mode;
    FTS_INFO("switch fw_grip to %u\n", ts_data->enable_fw_grip);

    ret = fts_set_grip_mode(ts_data, grip_mode);
    if (ret < 0) {
        return ret;
    }

    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_grip_fops = {
    .proc_read   = proc_grip_read,
    .proc_write  = proc_grip_write,
};
#else
static const struct file_operations proc_grip_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_grip_read,
    .write  = proc_grip_write,
};
#endif

/* sense on and off */
static ssize_t proc_sense_onoff_read(struct file *filp, char __user *buff,
    size_t count, loff_t *ppos)
{
    int cnt = 0;
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    u8 mode = 0;
    loff_t pos = *ppos;

    if (pos)
        return 0;

    ret = fts_read_reg(FTS_REG_SENSE_ONOFF, &mode);
    if (ret < 0) {
        FTS_ERROR("read reg0xEA fails");
        return ret;
    }

    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "Sensing mode:%s\n",
        mode ? "Enable" : "Disable");

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

static ssize_t proc_sense_onoff_write(struct file *filp,
    const char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int mode = 0xFF;
    int buflen = count;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%d", &mode);
    if (ret != 1) {
        FTS_ERROR("get mode fails,ret=%d", ret);
        return -EINVAL;
    }

    FTS_INFO("switch touch sense on/off to %d", mode);
    ret = fts_write_reg(FTS_REG_SENSE_ONOFF, !!mode);
    if (ret < 0) {
        FTS_ERROR("write reg0xEA fails");
        return ret;
    }

    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_sense_onoff_fops = {
    .proc_read   = proc_sense_onoff_read,
    .proc_write  = proc_sense_onoff_write,
};
#else
static const struct file_operations proc_sense_onoff_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_sense_onoff_read,
    .write  = proc_sense_onoff_write,
};
#endif

/* IRQ on and off */
static ssize_t proc_irq_onoff_read(struct file *filp,
    char __user *buff, size_t count, loff_t *ppos)
{
    int cnt = 0;
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    u8 mode = 0;
    loff_t pos = *ppos;

    if (pos)
        return 0;

    ret = fts_read_reg(FTS_REG_IRQ_ONOFF, &mode);
    if (ret < 0) {
        FTS_ERROR("read reg_0xEB fails");
        return ret;
    }

    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "touch IRQ:%s\n",
        mode ? "Enable" : "Disable");

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

static ssize_t proc_irq_onoff_write(struct file *filp,
    const char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int mode = 0xFF;
    int buflen = count;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%d", &mode);
    if (ret != 1) {
        FTS_ERROR("get mode fails,ret=%d", ret);
        return -EINVAL;
    }

    FTS_INFO("switch touch IRQ on/off to %d", mode);
    ret = fts_write_reg(FTS_REG_IRQ_ONOFF, !!mode);
    if (ret < 0) {
        FTS_ERROR("write reg_0xEB fails");
        return ret;
    }

    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_irq_onoff_fops = {
    .proc_read   = proc_irq_onoff_read,
    .proc_write  = proc_irq_onoff_write,
};
#else
static const struct file_operations proc_irq_onoff_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_irq_onoff_read,
    .write  = proc_irq_onoff_write,
};
#endif

/* heatmap on and off */
static ssize_t proc_heatmap_onoff_read(struct file *filp,
    char __user *buff, size_t count, loff_t *ppos)
{
    int cnt = 0;
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    u8 mode = 0;
    u8 compressed = 0;
    loff_t pos = *ppos;

    if (pos)
        return 0;

    ret = fts_read_reg(FTS_REG_HEATMAP_9E, &mode);
    if (ret < 0) {
        FTS_ERROR("read reg_0x%X fails", FTS_REG_HEATMAP_9E);
        return ret;
    }

    if (mode) {
        fts_read_reg(FTS_REG_HEATMAP_ED, &compressed);
        if (ret < 0) {
            FTS_ERROR("read reg_0x%X fails", FTS_REG_HEATMAP_ED);
            return ret;
        }
        cnt += snprintf(tmpbuf + cnt,  PROC_BUF_SIZE - cnt, "%s ",
            compressed ? "Compressed" : "Uncompressed");
    }

    cnt += snprintf(tmpbuf + cnt,  PROC_BUF_SIZE - cnt, "heatmap is %s\n",
                    mode ? "Enable" : "Disable");

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

static ssize_t proc_heatmap_onoff_write(struct file *filp,
    const char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int mode = 0xFF;
    int buflen = count;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%d", &mode);
    if (ret != 1) {
        FTS_ERROR("get mode fails,ret=%d", ret);
        return -EINVAL;
    }

    if (mode < FW_HEATMAP_MODE_DISABLE || mode > FW_HEATMAP_MODE_UNCOMPRESSED) {
        FTS_ERROR("Please input the parameters in \n \
             0: Disable firmware heatmap. \n \
             1: Enable firmware compressed heatmap. \n \
             2: Enable firmware uncompressed heatmap.");
        return -EINVAL;
    }
    FTS_INFO("switch heatmap on/off to %d", mode);
    fts_set_heatmap_mode(ts_data, mode);
    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_heatmap_onoff_fops = {
    .proc_read   = proc_heatmap_onoff_read,
    .proc_write  = proc_heatmap_onoff_write,
};
#else
static const struct file_operations proc_heatmap_onoff_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_heatmap_onoff_read,
    .write  = proc_heatmap_onoff_write,
};
#endif

static ssize_t proc_LPTW_setting_write(
    struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = {0};

    int buflen = count;
    int lptw_write_data[FTS_LPTW_BUF_LEN] = {0};
    u8  write_data[FTS_LPTW_BUF_LEN] = {0};

    u8 cmd[2] = {0};
    u32 data_length = 0;
    int i;

    cmd[0] = FTS_LPTW_REG_SET_E1;
    cmd[1] = FTS_LPTW_REG_SET_E2;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%x%x%x%x%x%x%x%x%x%x%x%x%x", &lptw_write_data[0],
        &lptw_write_data[1], &lptw_write_data[2], &lptw_write_data[3],
        &lptw_write_data[4], &lptw_write_data[5], &lptw_write_data[6],
        &lptw_write_data[7], &lptw_write_data[8], &lptw_write_data[9],
        &lptw_write_data[10], &lptw_write_data[11], &lptw_write_data[12]);

    if(lptw_write_data[0] == FTS_LPTW_REG_SET_E1)
        data_length = FTS_LPTW_E1_BUF_LEN;
    else if (lptw_write_data[0] == FTS_LPTW_REG_SET_E2)
        data_length = FTS_LPTW_E2_BUF_LEN;
    else
        data_length = 0;

    for (i = 0; i < data_length; i++)
        write_data[i] = (char)lptw_write_data[i];

    if (data_length != 0){
        ret=fts_write(write_data, data_length);
        if (ret < 0) {
            FTS_ERROR("write data to register E1/E2 fail");
            return ret;
        }
    }

 return count;
}

/*LPTW setting read*/
static ssize_t proc_LPTW_setting_read(
    struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
    int cnt = 0;
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = {0};
    u8 cmd[2] = {0};
    int num_read_chars = 0;
    loff_t pos = *ppos;
    int buflen = count;
    u8 *readbuf = NULL;
    u8 read_tmpbuf[20] = {0};

    if (pos)
        return 0;

    if (buflen <= 0) {
        FTS_ERROR("apk proc read count(%d) fail", buflen);
        return -EINVAL;
    }

    if (buflen > PROC_BUF_SIZE) {
        readbuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
        if (NULL == readbuf) {
            FTS_ERROR("apk proc buf zalloc fail");
            return -ENOMEM;
        }
    } else {
        readbuf = read_tmpbuf;
    }

    cmd[0] = FTS_LPTW_REG_SET_E1;
    cmd[1] = FTS_LPTW_REG_SET_E2;
    ret = fts_read(&cmd[0], 1, readbuf, FTS_LPTW_E1_BUF_LEN - 1);
    if (ret < 0) {
        FTS_ERROR("read reg_0xE1 fails");
        goto proc_read_err;
    }
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "==LPTW Gesture setting(E1)==\n");
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "min_x :%4d\n",
        ((readbuf[0] & 0x0F) << 8) + (readbuf[1] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "min_y :%4d\n",
        ((readbuf[2] & 0x0F) << 8) + (readbuf[3] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "max_x :%4d\n",
        ((readbuf[4] & 0x0F) << 8) + (readbuf[5] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt, "max_y :%4d\n",
        ((readbuf[6] & 0x0F) << 8) + (readbuf[7] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "min_frame_count :%3d\n",(readbuf[8] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "jitter :%3d\n" ,(readbuf[9] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "max_touch_size :%3d\n\n", (readbuf[10] & 0xFF));

    ret = fts_read(&cmd[1], 1, readbuf, FTS_LPTW_E2_BUF_LEN - 1);
    if (ret < 0) {
        FTS_ERROR("read reg_0xE2 fails");
        goto proc_read_err;
    }
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "==LPTW Gesture setting(E2)==\n");
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "marginal_min_x :%2d\n", (readbuf[0] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "marginal_max_x :%2d\n", (readbuf[1] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "marginal_min_y :%2d\n", (readbuf[2] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "marginal_max_y :%2d\n", (readbuf[3] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "monitor_channel_min_tx :%2d\n", (readbuf[4] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "monitor_channel_max_tx :%2d\n", (readbuf[5] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "monitor_channel_min_rx :%2d\n", (readbuf[6] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "monitor_channel_max_rx :%2d\n", (readbuf[7] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "min_node_count :%2d\n", (readbuf[8] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "motion_boundary :%4d\n\n",((readbuf[10] & 0x0F) << 8) +
        (readbuf[11] & 0xFF));

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;

proc_read_err:
    if (copy_to_user(buff, readbuf, num_read_chars)) {
        FTS_ERROR("copy to user error");
        ret = -EFAULT;
    }

    if ((buflen > PROC_BUF_SIZE) && readbuf) {
        kfree(readbuf);
        readbuf = NULL;
    }
    return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops LPTW_setting_fops = {
    .proc_read   = proc_LPTW_setting_read,
    .proc_write  = proc_LPTW_setting_write,
};
#else
static const struct file_operations LPTW_setting_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_LPTW_setting_read,
    .write  = proc_LPTW_setting_write,
};
#endif

static ssize_t proc_STTW_setting_write(
    struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = {0};
    int buflen = count;
    int sttw_write_data[FTS_STTW_E3_BUF_LEN] = {0};
    u8  write_data[FTS_STTW_E3_BUF_LEN] = {0};

    u8 cmd[2] = {0};
    u8 data_length = 0;
    int i = 0;
    cmd[0] = FTS_STTW_REG_SET_E3;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%x%x%x%x%x%x%x%x%x%x%x%x%x", &sttw_write_data[0],
        &sttw_write_data[1], &sttw_write_data[2], &sttw_write_data[3],
        &sttw_write_data[4], &sttw_write_data[5], &sttw_write_data[6],
        &sttw_write_data[7], &sttw_write_data[8], &sttw_write_data[9],
        &sttw_write_data[10], &sttw_write_data[11], &sttw_write_data[12]);

    if (sttw_write_data[0] == FTS_STTW_REG_SET_E3) {
        data_length = FTS_STTW_E3_BUF_LEN;
        for (i = 0; i < data_length; i++)
            write_data[i] = (char)sttw_write_data[i];

        ret = fts_write(write_data,data_length);
        if (ret < 0) {
            FTS_ERROR("write data to register E3 fail");
            return ret;
        }
    }

 return count;
}

/*STTW setting read*/
static ssize_t proc_STTW_setting_read(
    struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
    int cnt = 0;
    int ret = 0;
    char tmpbuf[PROC_BUF_SIZE] = {0};
    u8 cmd[2] = {0};
    int num_read_chars = 0;
    loff_t pos = *ppos;
    int buflen = count;
    u8 *readbuf = NULL;
    u8 read_tmpbuf[20] = {0};

    if (pos)
        return 0;

    if (buflen <= 0) {
        FTS_ERROR("apk proc read count(%d) fail", buflen);
        return -EINVAL;
    }

    if (buflen > PROC_BUF_SIZE) {
        readbuf = (u8 *)kzalloc(buflen * sizeof(u8), GFP_KERNEL);
        if (NULL == readbuf) {
            FTS_ERROR("apk proc buf zalloc fail");
            return -ENOMEM;
        }
    } else {
        readbuf = read_tmpbuf;
    }

    cmd[0] = FTS_STTW_REG_SET_E3;

    ret = fts_read(&cmd[0], 1, readbuf, FTS_STTW_E3_BUF_LEN - 1);
    if (ret < 0) {
        FTS_ERROR("read reg_0xE3 fails");
        goto proc_read_err;
    }

    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "==STTW Gesture setting(E3)==\n");
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "min_x :%4d\n",((readbuf[0] & 0x0F) << 8) +(readbuf[1] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "min_y :%4d\n",((readbuf[2] & 0x0F) << 8) +(readbuf[3] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "max_x :%4d\n",((readbuf[4] & 0x0F) << 8) +(readbuf[5] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "max_y :%4d\n",((readbuf[6] & 0x0F) << 8) +(readbuf[7] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "min_frame_count :%3d\n", (readbuf[8] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "max_frame_count :%3d\n", (readbuf[9] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "jitter :%3d\n",(readbuf[10] & 0xFF));
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "tap_max_touch_size :%3d\n", (readbuf[11] & 0xFF));

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;

proc_read_err:
    if (copy_to_user(buff, readbuf, num_read_chars)) {
        FTS_ERROR("copy to user error");
        ret = -EFAULT;
    }

    if ((buflen > PROC_BUF_SIZE) && readbuf) {
        kfree(readbuf);
        readbuf = NULL;
    }
    return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops STTW_setting_fops = {
    .proc_read   = proc_STTW_setting_read,
    .proc_write  = proc_STTW_setting_write,
};
#else
static const struct file_operations STTW_setting_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_STTW_setting_read,
    .write  = proc_STTW_setting_write,
};
#endif

/* motion filter mode */
static ssize_t proc_mf_mode_read(struct file *filp,
    char __user *buff, size_t count, loff_t *ppos)
{
    int cnt = 0;
    struct fts_ts_data *ts_data = fts_data;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    loff_t pos = *ppos;

    if (pos)
        return 0;

    FTS_DEBUG("mf_mode = %u", ts_data->mf_mode);
    cnt += snprintf(tmpbuf + cnt, PROC_BUF_SIZE - cnt,
        "%u\n", ts_data->mf_mode);

    if (copy_to_user(buff, tmpbuf, cnt)) {
        FTS_ERROR("copy to user error");
        return -EFAULT;
    }

    *ppos = pos + cnt;
    return cnt;
}

/**
 * Attribute to set motion filter mode.
 *  0 = Always unfilter.
 *  1 = Dynamic change motion filter.
 *  2 = Always filter by touch FW.
 */
static ssize_t proc_mf_mode_write(struct file *filp,
    const char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    int mf_mode = 0xFF;
    int buflen = count;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        return -EINVAL;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        return -EFAULT;
    }

    ret = sscanf(tmpbuf, "%d", &mf_mode);
    if (ret != 1) {
        FTS_ERROR("get mode fails,ret=%d", ret);
        return -EINVAL;
    }
    if (mf_mode < 0 || mf_mode > 2) {
        FTS_ERROR("get mode fails, mf_mode should be in [0,1,2].");
        return -EINVAL;
    }

    ts_data->mf_mode = mf_mode;
    FTS_INFO("switch fw_mode to %u\n", ts_data->mf_mode);

    return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_mf_mode_fops = {
    .proc_read   = proc_mf_mode_read,
    .proc_write  = proc_mf_mode_write,
};
#else
static const struct file_operations proc_mf_mode_fops = {
    .owner  = THIS_MODULE,
    .read   = proc_mf_mode_read,
    .write  = proc_mf_mode_write,
};
#endif

/**
 * proc_force_active_write()
 *
 * Attribute to set different scan mode.
 * 0x10 - Set FTS_TS_BUS_REF_FORCE_ACTIVE bit 0.
 * 0x11 - Set FTS_TS_BUS_REF_FORCE_ACTIVE bit 1.
 * 0x20 - Set FTS_TS_BUS_REF_BUGREPORT bit 0.
 * 0x21 - Set FTS_TS_BUS_REF_BUGREPORT bit 1.
 *
 * @return
 *    on success, return count; otherwise, return error code
 */
static ssize_t proc_force_active_write(struct file *filp,
    const char __user *buff, size_t count, loff_t *ppos)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;
    char tmpbuf[PROC_BUF_SIZE] = { 0 };
    unsigned char input;
    int buflen = count;
    bool active;
    u32 ref = 0;

    if (buflen >= PROC_BUF_SIZE) {
        FTS_ERROR("proc write length(%d) fails", buflen);
        ret = -EINVAL;
        goto exit;
    }

    if (copy_from_user(tmpbuf, buff, buflen)) {
        FTS_ERROR("copy from user error");
        ret = -EFAULT;
        goto exit;
    }

    ret = kstrtou8(tmpbuf, 16, &input);
    if (ret != 0) {
        FTS_ERROR("get mode fails, ret=%d", ret);
        ret = -EINVAL;
        goto exit;
    }
    switch (input) {
    case 0x10:
        ref = FTS_TS_BUS_REF_FORCE_ACTIVE;
        active = false;
        break;
    case 0x11:
        ref = FTS_TS_BUS_REF_FORCE_ACTIVE;
        active = true;
        break;
    case 0x20:
        ref = FTS_TS_BUS_REF_BUGREPORT;
        active = false;
        ts_data->bugreport_ktime_start = 0;
        break;
    case 0x21:
        ref = FTS_TS_BUS_REF_BUGREPORT;
        active = true;
        ts_data->bugreport_ktime_start = ktime_get();
        break;
    default:
        FTS_ERROR("Invalid input %#x.\n", input);
        ret = -EINVAL;
        goto exit;
    }

    FTS_INFO("Set bus reference bit %#x %s.", ref,
        active ? "enable" : "disable");

    if (active)
        pm_stay_awake(ts_data->dev);
    else
        pm_relax(ts_data->dev);

    ret = fts_ts_set_bus_ref(ts_data, ref, active);
    if (ret < 0) {
        FTS_ERROR("Set bus reference bit %#x %s failed.", ref,
            active ? "enable" : "disable");
      goto exit;
    }

    ret = count;

exit:
    return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_force_active_fops = {
    .proc_write  = proc_force_active_write,
};
#else
static const struct file_operations proc_force_active_fops = {
    .owner  = THIS_MODULE,
    .write  = proc_force_active_write,
};
#endif


struct proc_dir_entry *proc_fw_update;
struct proc_dir_entry *proc_scan_modes;
struct proc_dir_entry *proc_touch_mode;
struct proc_dir_entry *proc_lpwg;
struct proc_dir_entry *proc_high_sensitivity;
struct proc_dir_entry *proc_palm;
struct proc_dir_entry *proc_grip;
struct proc_dir_entry *proc_sense_onoff;
struct proc_dir_entry *proc_irq_onoff;
struct proc_dir_entry *proc_heatmap_onoff;
struct proc_dir_entry *proc_LPTW_setting;
struct proc_dir_entry *proc_STTW_setting;
struct proc_dir_entry *proc_mf_mode;
struct proc_dir_entry *proc_force_active;

static int fts_create_ctrl_procs(struct fts_ts_data *ts_data)
{
    int ret = 0;

    proc_fw_update = proc_create_data("fw_update", S_IWUSR,
        ts_data->proc_touch_entry, &proc_fw_update_fops, ts_data);
    if (!proc_fw_update) {
        FTS_ERROR("create proc_fw_update entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_scan_modes = proc_create_data("scan_modes", S_IRUSR,
        ts_data->proc_touch_entry, &proc_scan_modes_fops, ts_data);
    if (!proc_scan_modes) {
        FTS_ERROR("create proc_scan_modes entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_touch_mode = proc_create_data("touch_mode", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &proc_touch_mode_fops, ts_data);
    if (!proc_touch_mode) {
        FTS_ERROR("create proc_touch_mode entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_lpwg = proc_create_data("lpwg", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &proc_lpwg_fops, ts_data);
    if (!proc_lpwg) {
        FTS_ERROR("create proc_lpwg entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_high_sensitivity = proc_create_data("high_sensitivity",
        S_IRUSR|S_IWUSR, ts_data->proc_touch_entry, &proc_hs_fops, ts_data);
    if (!proc_lpwg) {
        FTS_ERROR("create proc_high_sensitivity entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_palm = proc_create_data("fw_palm", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &proc_palm_fops, ts_data);
    if (!proc_palm) {
        FTS_ERROR("create proc_palm entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_grip = proc_create_data("fw_grip", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &proc_grip_fops, ts_data);
    if (!proc_grip) {
        FTS_ERROR("create proc_grip entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_sense_onoff = proc_create_data("sense_onoff", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &proc_sense_onoff_fops, ts_data);
    if (!proc_sense_onoff) {
        FTS_ERROR("create proc_sense_onoff entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_irq_onoff = proc_create_data("irq_onoff", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &proc_irq_onoff_fops, ts_data);
    if (!proc_irq_onoff) {
        FTS_ERROR("create proc_irq_onoff entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_heatmap_onoff = proc_create_data("heatmap_onoff", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &proc_heatmap_onoff_fops, ts_data);
    if (!proc_heatmap_onoff) {
        FTS_ERROR("create proc_heatmap_onoff entry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_LPTW_setting = proc_create_data("LPTW_setting", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &LPTW_setting_fops, ts_data);
    if (!proc_LPTW_setting) {
        FTS_ERROR("create proc_LPTW_settingentry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_STTW_setting = proc_create_data("STTW_setting", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &STTW_setting_fops, ts_data);
    if (!proc_STTW_setting) {
        FTS_ERROR("create proc_STTW_settingentry fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_mf_mode = proc_create_data("mf_mode", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &proc_mf_mode_fops, ts_data);
    if (!proc_mf_mode) {
        FTS_ERROR("create proc_mf_mode fail");
        ret = -ENOMEM;
        return ret;
    }

    proc_force_active = proc_create_data("force_active", S_IRUSR|S_IWUSR,
        ts_data->proc_touch_entry, &proc_force_active_fops, ts_data);
    if (!proc_force_active) {
        FTS_ERROR("create proc_force_active fail");
        ret = -ENOMEM;
        return ret;
    }

    FTS_INFO("create control procs succeeds");
    return 0;
}

static void fts_free_ctrl_procs(void)
{
    if (proc_fw_update)
        proc_remove(proc_fw_update);

    if (proc_scan_modes)
        proc_remove(proc_scan_modes);

    if (proc_touch_mode)
        proc_remove(proc_touch_mode);

    if (proc_lpwg)
        proc_remove(proc_lpwg);

    if (proc_high_sensitivity)
        proc_remove(proc_high_sensitivity);

    if (proc_palm)
        proc_remove(proc_palm);

    if (proc_grip)
        proc_remove(proc_grip);

    if (proc_sense_onoff)
        proc_remove(proc_sense_onoff);

    if (proc_irq_onoff)
        proc_remove(proc_irq_onoff);

    if (proc_heatmap_onoff)
        proc_remove(proc_heatmap_onoff);

    if (proc_LPTW_setting)
        proc_remove(proc_LPTW_setting);

    if (proc_STTW_setting)
        proc_remove(proc_STTW_setting);

    if (proc_mf_mode)
        proc_remove(proc_mf_mode);

    if (proc_force_active)
        proc_remove(proc_force_active);
}

int fts_create_sysfs(struct fts_ts_data *ts_data)
{
    int ret = 0;

    ret = sysfs_create_group(&ts_data->dev->kobj, &fts_attribute_group);
    if (ret) {
        FTS_ERROR("[EX]: sysfs_create_group() failed!!");
        sysfs_remove_group(&ts_data->dev->kobj, &fts_attribute_group);
        return -ENOMEM;
    } else {
        FTS_INFO("[EX]: sysfs_create_group() succeeded!!");
    }

    ts_data->proc_touch_entry = proc_mkdir("focaltech_touch", NULL);
    if (!ts_data->proc_touch_entry) {
        FTS_ERROR("create proc/focaltech_touch fails");
    }

    ret = fts_create_ctrl_procs(ts_data);
    if (ret) {
        FTS_ERROR("Create ctrl procs fails");
    }

    return ret;
}

int fts_remove_sysfs(struct fts_ts_data *ts_data)
{
    sysfs_remove_group(&ts_data->dev->kobj, &fts_attribute_group);
    fts_free_ctrl_procs();
    if (ts_data->proc_touch_entry)
        proc_remove(fts_data->proc_touch_entry);
    return 0;
}
