/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
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

/************************************************************************
*
* File Name: focaltech_test.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-01
*
* Modify:
*
* Abstract: create char device and proc node for  the comm between APK and TP
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "focaltech_test.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_test *fts_ftest;

struct test_funcs *test_func_list[] = {
    &test_func_ft5672,
};

/*****************************************************************************
* Static function prototypes
*****************************************************************************/

/*****************************************************************************
* functions body
*****************************************************************************/
void sys_delay(int ms)
{
    msleep(ms);
}

int fts_abs(int value)
{
    if (value < 0)
        value = 0 - value;

    return value;
}

void *fts_malloc(size_t size)
{
    return kzalloc(size, GFP_KERNEL);
}

void fts_free_proc(void *p)
{
    return kfree(p);
}

void print_buffer(int *buffer, int length, int line_num)
{
    int i = 0;
    int j = 0;
    int tmpline = 0;
    char *tmpbuf = NULL;
    int tmplen = 0;
    int cnt = 0;
    struct fts_test *tdata = fts_ftest;

    if (tdata && tdata->ts_data && (tdata->ts_data->log_level < 3)) {
        return;
    }

    if ((NULL == buffer) || (length <= 0)) {
        FTS_TEST_DBG("buffer/length(%d) fail", length);
        return;
    }

    tmpline = line_num ? line_num : length;
    tmplen = tmpline * 6 + 128;
    tmpbuf = kzalloc(tmplen, GFP_KERNEL);

    for (i = 0; i < length; i = i + tmpline) {
        cnt = 0;
        for (j = 0; j < tmpline; j++) {
            cnt += snprintf(tmpbuf + cnt, tmplen - cnt, "%5d ", buffer[i + j]);
            if ((cnt >= tmplen) || ((i + j + 1) >= length))
                break;
        }
        FTS_TEST_DBG("%s", tmpbuf);
    }

    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }
}

/********************************************************************
 * test read/write interface
 *******************************************************************/
static int fts_test_bus_read(
    u8 *cmd, int cmdlen, u8 *data, int datalen)
{
    int ret = 0;

    ret = fts_read(cmd, cmdlen, data, datalen);
    if (ret < 0)
        return ret;
    else
        return 0;
}

static int fts_test_bus_write(u8 *writebuf, int writelen)
{
    int ret = 0;

    ret = fts_write(writebuf, writelen);
    if (ret < 0)
        return ret;
    else
        return 0;
}

int fts_test_read_reg(u8 addr, u8 *val)
{
    return fts_test_bus_read(&addr, 1, val, 1);
}

int fts_test_write_reg(u8 addr, u8 val)
{
    int ret;
    u8 cmd[2] = {0};

    cmd[0] = addr;
    cmd[1] = val;
    ret = fts_test_bus_write(cmd, 2);

    return ret;
}

int fts_test_read(u8 addr, u8 *readbuf, int readlen)
{
    int ret = 0;
    int i = 0;
    int packet_length = 0;
    int packet_num = 0;
    int packet_remainder = 0;
    int offset = 0;
    int byte_num = readlen;

    packet_num = byte_num / BYTES_PER_TIME;
    packet_remainder = byte_num % BYTES_PER_TIME;
    if (packet_remainder)
        packet_num++;

    if (byte_num < BYTES_PER_TIME) {
        packet_length = byte_num;
    } else {
        packet_length = BYTES_PER_TIME;
    }
    /* FTS_TEST_DBG("packet num:%d, remainder:%d", packet_num, packet_remainder); */

    ret = fts_test_bus_read(&addr, 1, &readbuf[offset], packet_length);
    if (ret < 0) {
        FTS_TEST_ERROR("read buffer fail");
        return ret;
    }
    for (i = 1; i < packet_num; i++) {
        offset += packet_length;
        if ((i == (packet_num - 1)) && packet_remainder) {
            packet_length = packet_remainder;
        }

        ret = fts_test_bus_read(&addr, 1, &readbuf[offset],
                                packet_length);

        if (ret < 0) {
            FTS_TEST_ERROR("read buffer fail");
            return ret;
        }
    }

    return 0;
}

int fts_test_write(u8 addr, u8 *writebuf, int writelen)
{
    int ret = 0;
    int i = 0;
    u8 *data = NULL;
    int packet_length = 0;
    int packet_num = 0;
    int packet_remainder = 0;
    int offset = 0;
    int byte_num = writelen;

    data = fts_malloc(BYTES_PER_TIME + 1);
    if (!data) {
        FTS_TEST_ERROR("malloc memory for bus write data fail");
        return -ENOMEM;
    }

    packet_num = byte_num / BYTES_PER_TIME;
    packet_remainder = byte_num % BYTES_PER_TIME;
    if (packet_remainder)
        packet_num++;

    if (byte_num < BYTES_PER_TIME) {
        packet_length = byte_num;
    } else {
        packet_length = BYTES_PER_TIME;
    }
    /* FTS_TEST_DBG("packet num:%d, remainder:%d", packet_num, packet_remainder); */

    data[0] = addr;
    for (i = 0; i < packet_num; i++) {
        if (i != 0) {
            data[0] = addr + 1;
        }
        if ((i == (packet_num - 1)) && packet_remainder) {
            packet_length = packet_remainder;
        }
        memcpy(&data[1], &writebuf[offset], packet_length);

        ret = fts_test_bus_write(data, packet_length + 1);
        if (ret < 0) {
            FTS_TEST_ERROR("write buffer fail");
            fts_free(data);
            return ret;
        }

        offset += packet_length;
    }

    fts_free(data);
    return 0;
}

/********************************************************************
 * test global function enter work/factory mode
 *******************************************************************/
int enter_work_mode(void)
{
    struct fts_ts_data *ts_data = fts_data;
    int ret = 0;
    u8 mode = 0;
    int i = 0;
    int j = 0;

    FTS_TEST_FUNC_ENTER();

    ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &mode);
    if ((ret >= 0) && (0x00 == mode))
        return 0;

    for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
        ret = fts_test_write_reg(DIVIDE_MODE_ADDR, 0x00);
        if (ret >= 0) {
            sys_delay(FACTORY_TEST_DELAY);
            for (j = 0; j < 20; j++) {
                ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &mode);
                if ((ret >= 0) && (mode == FTS_REG_WORKMODE_WORK_VALUE)) {
                    FTS_TEST_INFO("enter work mode success");
                    ts_data->work_mode = mode;
                    return 0;
                } else {
                    sys_delay(FACTORY_TEST_DELAY);
                }
            }
        }

        sys_delay(50);
    }

    if (i >= ENTER_WORK_FACTORY_RETRIES) {
        FTS_TEST_ERROR("Enter work mode fail");
        return -EIO;
    }

    FTS_TEST_FUNC_EXIT();
    return 0;
}

int enter_factory_mode(void)
{
    struct fts_ts_data *ts_data = fts_data;
    int ret = 0;
    u8 mode = 0;
    int i = 0;
    int j = 0;

    ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &mode);
    if ((ret >= 0) && (0x40 == mode))
        return 0;

    for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
        ret = fts_test_write_reg(DIVIDE_MODE_ADDR, 0x40);
        if (ret >= 0) {
            sys_delay(FACTORY_TEST_DELAY);
            for (j = 0; j < 20; j++) {
                ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &mode);
                if ((ret >= 0) && (mode == FTS_REG_WORKMODE_FACTORY_VALUE)) {
                    FTS_TEST_INFO("enter factory mode success");
                    sys_delay(10);
                    ts_data->work_mode = mode;
                    return 0;
                } else {
                    sys_delay(FACTORY_TEST_DELAY);
                }
            }
        }

        sys_delay(50);
    }

    if (i >= ENTER_WORK_FACTORY_RETRIES) {
        FTS_TEST_ERROR("Enter factory mode fail");
        return -EIO;
    }

    return 0;
}

/*
 * read_mass_data - read rawdata/short test data
 * addr - register addr which read data from
 * byte_num - read data length, unit:byte
 * buf - save data
 *
 * return 0 if read data succuss, otherwise return error code
 */
int read_mass_data(u8 addr, int byte_num, int *buf)
{
    int ret = 0;
    int i = 0;
    u8 *data = NULL;

    data = (u8 *)fts_malloc(byte_num * sizeof(u8));
    if (NULL == data) {
        FTS_TEST_SAVE_ERR("mass data buffer malloc fail\n");
        return -ENOMEM;
    }

    /* read rawdata buffer */
    FTS_TEST_INFO("mass data len:%d", byte_num);
    ret = fts_test_read(addr, data, byte_num);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read mass data fail\n");
        goto read_massdata_err;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        buf[i >> 1] = (int)(short)((data[i] << 8) + data[i + 1]);
    }

    ret = 0;
read_massdata_err:
    fts_free(data);
    return ret;
}

int read_mass_data_u16(u8 addr, int byte_num, int *buf)
{
    int ret = 0;
    int i = 0;
    u8 *data = NULL;

    data = (u8 *)fts_malloc(byte_num * sizeof(u8));
    if (NULL == data) {
        FTS_TEST_SAVE_ERR("mass data buffer malloc fail\n");
        return -ENOMEM;
    }

    /* read rawdata buffer */
    FTS_TEST_INFO("mass data len:%d", byte_num);
    ret = fts_test_read(addr, data, byte_num);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read mass data fail\n");
        goto read_massdata_err;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        buf[i >> 1] = (int)(u16)((data[i] << 8) + data[i + 1]);
    }

    ret = 0;
read_massdata_err:
    fts_free(data);
    return ret;
}

int short_get_adcdata_incell(u8 retval, u8 ch_num, int byte_num, int *adc_buf)
{
    int ret = 0;
    int times = 0;
    u8 short_state = 0;

    FTS_TEST_FUNC_ENTER();

    /* Start ADC sample */
    ret = fts_test_write_reg(FACTORY_REG_SHORT_TEST_EN, 0x01);
    if (ret) {
        FTS_TEST_SAVE_ERR("start short test fail\n");
        goto adc_err;
    }

    sys_delay(ch_num * FACTORY_TEST_DELAY);
    for (times = 0; times < FACTORY_TEST_RETRY; times++) {
        ret = fts_test_read_reg(FACTORY_REG_SHORT_TEST_STATE, &short_state);
        if ((ret >= 0) && (retval == short_state))
            break;
        else
            FTS_TEST_DBG("reg%x=%x,retry:%d",
                         FACTORY_REG_SHORT_TEST_STATE, short_state, times);

        sys_delay(FACTORY_TEST_RETRY_DELAY);
    }
    if (times >= FACTORY_TEST_RETRY) {
        FTS_TEST_SAVE_ERR("short test timeout, ADC data not OK\n");
        ret = -EIO;
        goto adc_err;
    }

    ret = read_mass_data(FACTORY_REG_SHORT_ADDR, byte_num, adc_buf);
    if (ret) {
        FTS_TEST_SAVE_ERR("get short(adc) data fail\n");
    }

adc_err:
    FTS_TEST_FUNC_EXIT();
    return ret;
}

/*
 * wait_state_update - wait fw status update
 */
int wait_state_update(u8 retval)
{
    int ret = 0;
    int times = 0;
    u8 addr = 0;
    u8 state = 0xFF;
    struct fts_test *tdata = fts_ftest;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    if (IC_HW_INCELL == tdata->func->hwtype) {
        addr = FACTORY_REG_PARAM_UPDATE_STATE;
    } else {
        addr = FACTORY_REG_PARAM_UPDATE_STATE_TOUCH;
    }

    while (times++ < FACTORY_TEST_RETRY) {
        sys_delay(FACTORY_TEST_DELAY);
        /* Wait register status update */
        state = 0xFF;
        ret = fts_test_read_reg(addr, &state);
        if ((ret >= 0) && (retval == state))
            break;
        else
            FTS_TEST_DBG("reg%x=%x,retry:%d", addr, state, times);
    }

    if (times >= FACTORY_TEST_RETRY) {
        FTS_TEST_SAVE_ERR("Wait State Update fail,reg%x=%x\n", addr, state);
        return -EIO;
    }

    return 0;
}

/*
 * start_scan - start to scan a frame
 */
int start_scan(void)
{
    int ret = 0;
    u8 addr = 0;
    u8 val = 0;
    u8 finish_val = 0;
    int times = 0;
    struct fts_test *tdata = fts_ftest;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    if (SCAN_SC == tdata->func->startscan_mode) {
        /* sc ic */
        addr = FACTORY_REG_SCAN_ADDR2;
        val = 0x01;
        finish_val = 0x00;
    } else {
        addr = DIVIDE_MODE_ADDR;
        val = 0xC0;
        finish_val = 0x40;
    }

    /* write register to start scan */
    ret = fts_test_write_reg(addr, val);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write start scan mode fail\n");
        return ret;
    }

    /* Wait for the scan to complete */
    while (times++ < FACTORY_TEST_RETRY) {
        sys_delay(FACTORY_TEST_DELAY);

        ret = fts_test_read_reg(addr, &val);
        if ((ret >= 0) && (val == finish_val)) {
            break;
        } else
            FTS_TEST_DBG("reg%x=%x,retry:%d", addr, val, times);
    }

    if (times >= FACTORY_TEST_RETRY) {
        FTS_TEST_SAVE_ERR("scan timeout\n");
        return -EIO;
    }

    return 0;
}

static int read_rawdata(
    struct fts_test *tdata,
    u8 off_addr,
    u8 off_val,
    u8 rawdata_addr,
    int byte_num,
    int *data)
{
    int ret = 0;

    /* set line addr or rawdata start addr */
    ret = fts_test_write_reg(off_addr, off_val);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write line/start addr fail\n");
        return ret;
    }

    if (tdata->func->raw_u16)
        ret = read_mass_data_u16(rawdata_addr, byte_num, data);
    else
        ret = read_mass_data(rawdata_addr, byte_num, data);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read rawdata fail\n");
        return ret;
    }

    return 0;
}

int get_rawdata(int *data)
{
    int ret = 0;
    u8 val = 0;
    u8 addr = 0;
    u8 rawdata_addr = 0;
    int byte_num = 0;
    struct fts_test *tdata = fts_ftest;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    /* enter factory mode */
    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        return ret;
    }

    /* start scanning */
    ret = start_scan();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("scan fail\n");
        return ret;
    }

    /* read rawdata */
    if (IC_HW_INCELL == tdata->func->hwtype) {
        val = 0xAD;
        addr = FACTORY_REG_LINE_ADDR;
        rawdata_addr = FACTORY_REG_RAWDATA_ADDR;
    } else if (IC_HW_MC_SC == tdata->func->hwtype) {
        val = 0xAA;
        addr = FACTORY_REG_LINE_ADDR;
        rawdata_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;
    } else {
        val = 0x0;
        addr = FACTORY_REG_RAWDATA_SADDR_SC;
        rawdata_addr = FACTORY_REG_RAWDATA_ADDR_SC;
    }

    byte_num = tdata->node.node_num * 2;
    ret = read_rawdata(tdata, addr, val, rawdata_addr, byte_num, data);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read rawdata fail\n");
        return ret;
    }

    return 0;
}

/*
 * chip_clb - auto clb
 */
int chip_clb(void)
{
    int ret = 0;
    u8 val = 0;
    int times = 0;

    /* start clb */
    ret = fts_test_write_reg(FACTORY_REG_CLB, 0x04);
    if (ret) {
        FTS_TEST_SAVE_ERR("write start clb fail\n");
        return ret;
    }

    while (times++ < FACTORY_TEST_RETRY) {
        sys_delay(FACTORY_TEST_RETRY_DELAY);
        ret = fts_test_read_reg(FACTORY_REG_CLB, &val);
        if ((0 == ret) && (0x02 == val)) {
            /* clb ok */
            break;
        } else
            FTS_TEST_DBG("reg%x=%x,retry:%d", FACTORY_REG_CLB, val, times);
    }

    if (times >= FACTORY_TEST_RETRY) {
        FTS_TEST_SAVE_ERR("chip clb timeout\n");
        return -EIO;
    }

    return 0;
}

/*
 * get_cb_incell - get cb data for incell IC
 */
int get_cb_incell(u16 saddr, int byte_num, int *cb_buf)
{
    int ret = 0;
    int i = 0;
    u8 cb_addr = 0;
    u8 addr_h = 0;
    u8 addr_l = 0;
    int read_num = 0;
    int packet_num = 0;
    int packet_remainder = 0;
    int offset = 0;
    int addr = 0;
    u8 *data = NULL;

    data = (u8 *)fts_malloc(byte_num * sizeof(u8));
    if (NULL == data) {
        FTS_TEST_SAVE_ERR("cb buffer malloc fail\n");
        return -ENOMEM;
    }

    packet_num = byte_num / BYTES_PER_TIME;
    packet_remainder = byte_num % BYTES_PER_TIME;
    if (packet_remainder)
        packet_num++;
    read_num = BYTES_PER_TIME;

    FTS_TEST_INFO("cb packet:%d,remainder:%d", packet_num, packet_remainder);
    cb_addr = FACTORY_REG_CB_ADDR;
    for (i = 0; i < packet_num; i++) {
        offset = read_num * i;
        addr = saddr + offset;
        addr_h = (addr >> 8) & 0xFF;
        addr_l = addr & 0xFF;
        if ((i == (packet_num - 1)) && packet_remainder) {
            read_num = packet_remainder;
        }

        ret = fts_test_write_reg(FACTORY_REG_CB_ADDR_H, addr_h);
        if (ret) {
            FTS_TEST_SAVE_ERR("write cb addr high fail\n");
            goto TEST_CB_ERR;
        }
        ret = fts_test_write_reg(FACTORY_REG_CB_ADDR_L, addr_l);
        if (ret) {
            FTS_TEST_SAVE_ERR("write cb addr low fail\n");
            goto TEST_CB_ERR;
        }

        ret = fts_test_read(cb_addr, data + offset, read_num);
        if (ret) {
            FTS_TEST_SAVE_ERR("read cb fail\n");
            goto TEST_CB_ERR;
        }
    }

    for (i = 0; i < byte_num; i++) {
        cb_buf[i] = data[i];
    }

TEST_CB_ERR:
    fts_free(data);
    return ret;
}

int get_cb_sc(int byte_num, int *cb_buf, enum byte_mode mode)
{
    int ret = 0;
    int i = 0;
    int read_num = 0;
    int packet_num = 0;
    int packet_remainder = 0;
    int offset = 0;
    u8 cb_addr = 0;
    u8 off_addr = 0;
    u8 off_h_addr = 0;
    struct fts_test *tdata = fts_ftest;
    u8 *cb = NULL;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    cb = (u8 *)fts_malloc(byte_num * sizeof(u8));
    if (NULL == cb) {
        FTS_TEST_SAVE_ERR("malloc memory for cb buffer fail\n");
        return -ENOMEM;
    }

    if (IC_HW_MC_SC == tdata->func->hwtype) {
        cb_addr = FACTORY_REG_MC_SC_CB_ADDR;
        off_addr = FACTORY_REG_MC_SC_CB_ADDR_OFF;
        off_h_addr = FACTORY_REG_MC_SC_CB_H_ADDR_OFF;
    } else if (IC_HW_SC == tdata->func->hwtype) {
        cb_addr = FACTORY_REG_SC_CB_ADDR;
        off_addr = FACTORY_REG_SC_CB_ADDR_OFF;
    }

    packet_num = byte_num / BYTES_PER_TIME;
    packet_remainder = byte_num % BYTES_PER_TIME;
    if (packet_remainder)
        packet_num++;
    read_num = BYTES_PER_TIME;
    offset = 0;

    FTS_TEST_INFO("cb packet:%d,remainder:%d", packet_num, packet_remainder);
    for (i = 0; i < packet_num; i++) {
        if ((i == (packet_num - 1)) && packet_remainder) {
            read_num = packet_remainder;
        }

        ret = fts_test_write_reg(off_addr, offset);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("write cb addr offset fail\n");
            goto cb_err;
        }

        if (tdata->func->cb_high_support) {
            ret = fts_test_write_reg(off_h_addr, offset >> 8);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("write cb_h addr offset fail\n");
                goto cb_err;
            }
        }

        ret = fts_test_read(cb_addr, cb + offset, read_num);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("read cb fail\n");
            goto cb_err;
        }

        offset += read_num;
    }

    if (DATA_ONE_BYTE == mode) {
        for (i = 0; i < byte_num; i++) {
            cb_buf[i] = cb[i];
        }
    } else if (DATA_TWO_BYTE == mode) {
        for (i = 0; i < byte_num; i = i + 2) {
            cb_buf[i >> 1] = (int)(((int)(cb[i]) << 8) + cb[i + 1]);
        }
    }

    ret = 0;
cb_err:
    fts_free(cb);
    return ret;
}

bool compare_data(int *data, int min, int max, int min_vk, int max_vk, bool key)
{
    int i = 0;
    bool result = true;
    struct fts_test *tdata = fts_ftest;
    int rx = tdata->node.rx_num;
    int node_va = tdata->node.node_num - tdata->node.key_num;

    if (!data || !tdata->node_valid) {
        FTS_TEST_SAVE_ERR("data/node_valid is null\n");
        return false;
    }

    for (i = 0; i < node_va; i++) {
        if (0 == tdata->node_valid[i])
            continue;

        if ((data[i] < min) || (data[i] > max)) {
            FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
                              i / rx + 1, i % rx + 1, data[i], min, max);
            result = false;
        }
    }

    if (key) {
        for (i = node_va; i < tdata->node.node_num; i++) {
            if (0 == tdata->node_valid[i])
                continue;

            if ((data[i] < min_vk) || (data[i] > max_vk)) {
                FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
                                  i / rx + 1, i % rx + 1,
                                  data[i], min_vk, max_vk);
                result = false;
            }
        }
    }

    return result;
}

bool compare_array(int *data, int *min, int *max, bool key)
{
    int i = 0;
    bool result = true;
    struct fts_test *tdata = fts_ftest;
    int rx = tdata->node.rx_num;
    int node_num = tdata->node.node_num;

    if (!data || !min || !max || !tdata->node_valid) {
        FTS_TEST_SAVE_ERR("data/min/max/node_valid is null\n");
        return false;
    }

    if (!key) {
        node_num -= tdata->node.key_num;
    }
    for (i = 0; i < node_num; i++) {
        if (0 == tdata->node_valid[i])
            continue;

        if ((data[i] < min[i]) || (data[i] > max[i])) {
            FTS_TEST_SAVE_ERR("test fail,node(%4d,%4d)=%5d,range=(%5d,%5d)\n",
                              i / rx + 1, i % rx + 1, data[i], min[i], max[i]);
            result = false;
        }
    }

    return result;
}

/*
 * show_data - show and save test data to testresult.txt
 */
void show_data(int *data, bool key)
{
#if TXT_SUPPORT
    int i = 0;
    int j = 0;
    struct fts_test *tdata = fts_ftest;
    int node_num = tdata->node.node_num;
    int tx_num = tdata->node.tx_num;
    int rx_num = tdata->node.rx_num;

    FTS_TEST_FUNC_ENTER();
    for (i = 0; i < tx_num; i++) {
        FTS_TEST_SAVE_INFO("Ch/Tx_%02d:  ", i + 1);
        for (j = 0; j < rx_num; j++) {
            FTS_TEST_SAVE_INFO("%5d, ", data[i * rx_num + j]);
        }
        FTS_TEST_SAVE_INFO("\n");
    }

    if (key) {
        FTS_TEST_SAVE_INFO("Ch/Tx_%02d:  ", tx_num + 1);
        for (i = tx_num * rx_num; i < node_num; i++) {
            FTS_TEST_SAVE_INFO("%5d, ",  data[i]);
        }
        FTS_TEST_SAVE_INFO("\n");
    }
    FTS_TEST_FUNC_EXIT();
#endif
}

/* mc_sc only */
/* Only V3 Pattern has mapping & no-mapping */
int mapping_switch(u8 mapping)
{
    int ret = 0;
    u8 val = 0xFF;
    struct fts_test *tdata = fts_ftest;

    if (tdata->v3_pattern) {
        ret = fts_test_read_reg(FACTORY_REG_NOMAPPING, &val);
        if (ret < 0) {
            FTS_TEST_ERROR("read 0x54 register fail");
            return ret;
        }

        if (val != mapping) {
            ret = fts_test_write_reg(FACTORY_REG_NOMAPPING, mapping);
            if (ret < 0) {
                FTS_TEST_ERROR("write 0x54 register fail");
                return ret;
            }
            sys_delay(FACTORY_TEST_DELAY);
        }
    }

    return 0;
}

bool get_fw_wp(u8 wp_ch_sel, enum wp_type water_proof_type)
{
    bool fw_wp_state = false;

    switch (water_proof_type) {
    case WATER_PROOF_ON:
        /* bit5: 0-check in wp on, 1-not check */
        fw_wp_state = !(wp_ch_sel & 0x20);
        break;
    case WATER_PROOF_ON_TX:
        /* Bit6:  0-check Rx+Tx in wp mode  1-check one channel
           Bit2:  0-check Tx in wp mode;  1-check Rx in wp mode
        */
        fw_wp_state = (!(wp_ch_sel & 0x40) || !(wp_ch_sel & 0x04));
        break;
    case WATER_PROOF_ON_RX:
        fw_wp_state = (!(wp_ch_sel & 0x40) || (wp_ch_sel & 0x04));
        break;
    case WATER_PROOF_OFF:
        /* bit7: 0-check in wp off, 1-not check */
        fw_wp_state = !(wp_ch_sel & 0x80);
        break;
    case WATER_PROOF_OFF_TX:
        /* Bit1-0:  00-check Tx in non-wp mode
                    01-check Rx in non-wp mode
                    10:check Rx+Tx in non-wp mode
        */
        fw_wp_state = ((0x0 == (wp_ch_sel & 0x03)) || (0x02 == (wp_ch_sel & 0x03)));
        break;
    case WATER_PROOF_OFF_RX:
        fw_wp_state = ((0x01 == (wp_ch_sel & 0x03)) || (0x02 == (wp_ch_sel & 0x03)));
        break;
    default:
        break;
    }

    return fw_wp_state;
}

int get_cb_mc_sc(u8 wp, int byte_num, int *cb_buf, enum byte_mode mode)
{
    int ret = 0;

    /* 1:waterproof 0:non-waterproof */
    ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, wp);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set mc_sc mode fail\n");
        return ret;
    }

    if (fts_ftest->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            return ret;
        }
    }

    /* read cb */
    ret = get_cb_sc(byte_num, cb_buf, mode);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get sc cb fail\n");
        return ret;
    }

    return 0;
}

int get_rawdata_mc_sc(enum wp_type wp, int *data)
{
    int ret = 0;
    u8 val = 0;
    u8 addr = 0;
    u8 rawdata_addr = 0;
    int byte_num = 0;
    struct fts_test *tdata = fts_ftest;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    byte_num = tdata->sc_node.node_num * 2;
    addr = FACTORY_REG_LINE_ADDR;
    rawdata_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;
    if (WATER_PROOF_ON == wp) {
        val = 0xAC;
    } else if (WATER_PROOF_OFF == wp) {
        val = 0xAB;
    } else if (HIGH_SENSITIVITY == wp) {
        val = 0xA0;
    } else if (HOV == wp) {
        val = 0xA1;
        byte_num = 4 * 2;
    }

    ret = read_rawdata(tdata, addr, val, rawdata_addr, byte_num, data);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read rawdata fail\n");
        return ret;
    }

    return 0;
}

int get_rawdata_mc(u8 fre, u8 fir, int *rawdata)
{
    int ret = 0;
    int i = 0;

    if (NULL == rawdata ) {
        FTS_TEST_SAVE_ERR("rawdata buffer is null\n");
        return -EINVAL;
    }

    /* set frequency high/low */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set frequency fail,ret=%d\n", ret);
        return ret;
    }

    /* fir enable/disable */
    ret = fts_test_write_reg(FACTORY_REG_FIR, fir);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set fir fail,ret=%d\n", ret);
        return ret;
    }

    /* get rawdata */
    for (i = 0; i < 3; i++) {
        /* lost 3 frames, in order to obtain stable data */
        ret = get_rawdata(rawdata);
    }
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get rawdata fail,ret=%d\n", ret);
        return ret;
    }

    return 0;
}

int short_get_adc_data_mc(u8 retval, int byte_num, int *adc_buf, u8 mode)
{
    int ret = 0;
    int i = 0;
    u8 short_state = 0;
    u8 short_state_reg = 0;
    u8 short_en_reg = 0;
    u8 short_data_reg = 0;
    struct fts_test *tdata = fts_ftest;

    FTS_TEST_FUNC_ENTER();
    if (tdata->func->mc_sc_short_v2) {
        short_en_reg = FACTROY_REG_SHORT2_TEST_EN;
        short_state_reg = FACTROY_REG_SHORT2_TEST_STATE;
        short_data_reg = FACTORY_REG_SHORT2_ADDR_MC;
    } else {
        short_en_reg = FACTROY_REG_SHORT_TEST_EN;
        short_state_reg = FACTROY_REG_SHORT_TEST_EN;
        short_data_reg = FACTORY_REG_SHORT_ADDR_MC;
    }

    /* select short test mode & start test */
    ret = fts_test_write_reg(short_en_reg, mode);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write short test mode fail\n");
        goto test_err;
    }

    for (i = 0; i < FACTORY_TEST_RETRY; i++) {
        sys_delay(FACTORY_TEST_RETRY_DELAY);

        ret = fts_test_read_reg(short_state_reg, &short_state);
        if ((ret >= 0) && (retval == short_state))
            break;
        else
            FTS_TEST_DBG("reg%x=%x,retry:%d", short_state_reg, short_state, i);
    }
    if (i >= FACTORY_TEST_RETRY) {
        FTS_TEST_SAVE_ERR("short test timeout, ADC data not OK\n");
        ret = -EIO;
        goto test_err;
    }

    ret = read_mass_data(short_data_reg, byte_num, adc_buf);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get short(adc) data fail\n");
    }

    FTS_TEST_DBG("adc data:\n");
    print_buffer(adc_buf, byte_num / 2, 0);
test_err:
    FTS_TEST_FUNC_EXIT();
    return ret;
}

bool compare_mc_sc(bool tx_check, bool rx_check, int *data, int *min, int *max)
{
    int i = 0;
    bool result = true;
    struct fts_test *tdata = fts_ftest;

    if (rx_check) {
        for (i = 0; i < tdata->sc_node.rx_num; i++) {
            if (0 == tdata->node_valid_sc[i])
                continue;

            if ((data[i] < min[i]) || (data[i] > max[i])) {
                FTS_TEST_SAVE_ERR("test fail,rx%d=%5d,range=(%5d,%5d)\n",
                                  i + 1, data[i], min[i], max[i]);
                result = false;
            }
        }
    }

    if (tx_check) {
        for (i = tdata->sc_node.rx_num; i < tdata->sc_node.node_num; i++) {
            if (0 == tdata->node_valid_sc[i])
                continue;

            if ((data[i] < min[i]) || (data[i] > max[i])) {
                FTS_TEST_SAVE_INFO("test fail,tx%d=%5d,range=(%5d,%5d)\n",
                                   i - tdata->sc_node.rx_num + 1,
                                   data[i], min[i], max[i]);
                result = false;
            }
        }
    }

    return result;
}

void show_data_mc_sc(int *data)
{
    int i = 0;
    struct fts_test *tdata = fts_ftest;

    FTS_TEST_SAVE_INFO("SCap Rx: ");
    for (i = 0; i < tdata->sc_node.rx_num; i++) {
        FTS_TEST_SAVE_INFO( "%5d, ", data[i]);
    }
    FTS_TEST_SAVE_INFO("\n");

    FTS_TEST_SAVE_INFO("SCap Tx: ");
    for (i = tdata->sc_node.rx_num; i < tdata->sc_node.node_num; i++) {
        FTS_TEST_SAVE_INFO( "%5d, ", data[i]);
    }
    FTS_TEST_SAVE_INFO("\n");
}
/* mc_sc end*/

#if CSV_SUPPORT || TXT_SUPPORT
static int fts_test_save_test_data(char *file_name, char *data_buf, int len)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
    struct file *pfile = NULL;
    char filepath[FILE_NAME_LENGTH] = { 0 };
    loff_t pos;
    mm_segment_t old_fs;

    FTS_TEST_FUNC_ENTER();
    memset(filepath, 0, sizeof(filepath));
    snprintf(filepath, FILE_NAME_LENGTH, "%s%s", FTS_INI_FILE_PATH, file_name);
    FTS_INFO("save test data to %s", filepath);
    if (NULL == pfile) {
        pfile = filp_open(filepath, O_TRUNC | O_CREAT | O_RDWR, 0);
    }
    if (IS_ERR(pfile)) {
        FTS_TEST_ERROR("error occured while opening file %s.",  filepath);
        return -EIO;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(pfile, data_buf, len, &pos);
    filp_close(pfile, NULL);
    set_fs(old_fs);

    FTS_TEST_FUNC_EXIT();
#endif
    return 0;
}
#endif

#if CSV_SUPPORT
static int fts_test_malloc_csv(struct fts_test *tdata)
{
    if (!tdata->csv_file_buf) {
        FTS_TEST_INFO("csv_file_buf is null, need malloc memory");
        tdata->csv_file_buf = vmalloc(CSV_BUFFER_LEN);
        if (!tdata->csv_file_buf) {
            FTS_TEST_ERROR("csv_file_buf malloc fail");
            return -ENOMEM;
        }
    }

    memset(tdata->csv_file_buf, 0, CSV_BUFFER_LEN);
    return 0;
}

static int fts_test_get_item_count_scap_csv(int index)
{
    int ret = 0;
    int i = 0;
    int select = 0;
    u8 wc_sel = 0;
    u8 hc_sel = 0;
    u8 scap_select[4] = { 0 };

    /* get waterproof channel select */
    ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read water_channel_sel fail,ret=%d\n", ret);
        return index;
    }

    ret = fts_test_read_reg(FACTORY_REG_HC_SEL, &hc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read high_channel_sel fail,ret=%d\n", ret);
        return index;
    }

    scap_select[0] = get_fw_wp(wc_sel, WATER_PROOF_ON);
    scap_select[1] = get_fw_wp(wc_sel, WATER_PROOF_OFF);
    scap_select[2] = (hc_sel & 0x03) ? 1 : 0;
    scap_select[3] = (hc_sel & 0x04) ? 1 : 0;

    for (i = 0; i < 4; i++) {
        if (scap_select[i])
            select++;
        if (select == index)
            break;
    }

    return (i + 1);
}

static void fts_test_save_data_csv(struct fts_test *tdata)
{
    int i = 0;
    int j = 0;
    int index = 0;
    int k = 0;
    int tx = 0;
    int rx = 0;
    int node_num = 0;
    int offset = 0;
    int start_line = 11;
    int data_count = 0;
    char *csv_buffer = tdata->csv_file_buf;
    char *line2_buffer = NULL;
    int csv_length = 0;
    int line2_length = 0;
    int csv_item_count = 0;
    struct fts_test_data *td = &tdata->testdata;
    struct item_info *info = NULL;

    FTS_TEST_INFO("save data in csv format");
    if (!tdata || !tdata->csv_file_buf) {
        FTS_TEST_ERROR("tdata/csv_file_buf is null");
        return ;
    }
    memset(csv_buffer, 0, CSV_BUFFER_LEN);

    line2_buffer = vmalloc(CSV_LINE2_BUFFER_LEN);
    if (!line2_buffer) {
        FTS_TEST_ERROR("line2_buffer malloc fail\n");
        goto csv_save_err;
    }

    FTS_TEST_INFO("test item count:%d", td->item_count);
    /* line 1 */
    csv_length += snprintf(csv_buffer + csv_length, \
                           CSV_BUFFER_LEN - csv_length, \
                           "ECC, 85, 170, IC Name, %s, IC Code, %x\n", \
                           tdata->ini.ic_name, \
                           (tdata->ini.ic_code >> IC_CODE_OFFSET));

    /* line 2 */
    for (i = 0; i < td->item_count; i++) {
        info = &td->info[i];
        if (info->mc_sc) {
            node_num = tdata->sc_node.node_num;
            /* set max len of tx/rx to column */
            rx = (tdata->sc_node.tx_num > tdata->sc_node.rx_num)
                 ? tdata->sc_node.tx_num : tdata->sc_node.rx_num;
        } else {
            if (info->key_support && (tdata->node.key_num > 0))
                node_num = (tdata->node.tx_num + 1) * tdata->node.rx_num;
            else
                node_num = tdata->node.tx_num * tdata->node.rx_num;
            rx = tdata->node.rx_num;
        }

        if (info->datalen > node_num) {
            data_count = (info->datalen - 1 ) / node_num + 1;
            tx = (node_num - 1 ) / rx + 1;
        } else {
            data_count = 1;
            tx = ((info->datalen - 1) / rx) + 1;
        }

        for (j = 1; j <= data_count; j++) {
            index = j;

            if (tdata->func->hwtype == IC_HW_MC_SC) {
                /*MC_SC, rawdata index will be 2*/
                if ((info->code == CODE_M_RAWDATA_TEST) && (data_count == 1)) {
                    index = 2;
                }

                /*MC_SC, SCAP index will be 1~4*/
                if ((info->code == CODE_M_SCAP_CB_TEST)
                    || (info->code == CODE_M_SCAP_RAWDATA_TEST)) {
                    index = fts_test_get_item_count_scap_csv(j);
                }
            }

            line2_length += snprintf(line2_buffer + line2_length, \
                                     CSV_LINE2_BUFFER_LEN - line2_length, \
                                     "%s, %d, %d, %d, %d, %d, ", \
                                     info->name, info->code, tx, rx,
                                     start_line, index);
            start_line += tx;
            csv_item_count++;
        }
    }

    csv_length += snprintf(csv_buffer + csv_length, \
                           CSV_BUFFER_LEN - csv_length, \
                           "TestItem Num, %d, ", \
                           csv_item_count);

    if (line2_length > 0) {
        csv_length += snprintf(csv_buffer + csv_length, \
                               CSV_BUFFER_LEN - csv_length, \
                               "%s", line2_buffer);
    }

    /* line 3 ~ 10  "\n" */
    csv_length += snprintf(csv_buffer + csv_length, \
                           CSV_BUFFER_LEN - csv_length, \
                           "\n\n\n\n\n\n\n\n\n");

    /* line 11 ~ data area */
    for (i = 0; i < td->item_count; i++) {
        info = &td->info[i];
        if (!info->data) {
            FTS_TEST_ERROR("test item data is null");
            goto csv_save_err;
        }

        if (info->mc_sc) {
            offset = 0;
            for (j = 0; j < info->datalen;) {
                for (k = 0; k < tdata->sc_node.node_num; k++) {
                    csv_length += snprintf(csv_buffer + csv_length, \
                                           CSV_BUFFER_LEN - csv_length, \
                                           "%d, ", info->data[offset + k]);
                    if ((k + 1) == tdata->sc_node.rx_num) {
                        csv_length += snprintf(csv_buffer + csv_length, \
                                               CSV_BUFFER_LEN - csv_length, \
                                               "\n");
                    }
                }
                csv_length += snprintf(csv_buffer + csv_length, \
                                       CSV_BUFFER_LEN - csv_length, \
                                       "\n");
                offset += k;
                j += k;
            }
        } else {
            for (j = 0; j < info->datalen; j++) {
                csv_length += snprintf(csv_buffer + csv_length, \
                                       CSV_BUFFER_LEN - csv_length, \
                                       "%d, ", info->data[j]);
                if (((j + 1) % tdata->node.rx_num) == 0) {
                    csv_length += snprintf(csv_buffer + csv_length, \
                                           CSV_BUFFER_LEN - csv_length,
                                           "\n");
                }
            }
        }
    }
    FTS_TEST_INFO("csv length:%d", csv_length);
    fts_test_save_test_data(FTS_CSV_FILE_NAME, csv_buffer, csv_length);

csv_save_err:
    if (line2_buffer) {
        vfree(line2_buffer);
        line2_buffer = NULL;
    }
}

static void fts_test_save_data_csv_private(struct fts_test *tdata)
{
    char *csv_buffer = tdata->csv_file_buf;
    int csv_length = 0;

    FTS_TEST_INFO("save data in csv format");
    if (!tdata || !tdata->csv_file_buf) {
        FTS_TEST_ERROR("tdata/csv_file_buf is null");
        return ;
    }
    memset(csv_buffer, 0, CSV_BUFFER_LEN);

    if (tdata->func && tdata->func->save_data_private)
        tdata->func->save_data_private(csv_buffer, &csv_length);

    FTS_TEST_INFO("csv length:%d", csv_length);
    fts_test_save_test_data(FTS_CSV_FILE_NAME, csv_buffer, csv_length);
}
#endif

#if TXT_SUPPORT
static int fts_test_malloc_txt(struct fts_test *tdata)
{
    if (!tdata->testresult) {
        FTS_TEST_INFO("testresult is null, need malloc memory");
        tdata->testresult = vmalloc(TXT_BUFFER_LEN);
        if (!tdata->testresult) {
            FTS_TEST_ERROR("tdata->testresult malloc fail\n");
            return -ENOMEM;
        }
    }
    memset(tdata->testresult, 0, TXT_BUFFER_LEN);
    tdata->testresult_len = 0;
    FTS_TEST_SAVE_INFO("FW version:0x%02x\n", tdata->fw_major_ver);
    FTS_TEST_SAVE_INFO("tx_num:%d, rx_num:%d, key_num:%d\n", tdata->node.tx_num,
                       tdata->node.rx_num, tdata->node.key_num);

    return 0;
}

static void fts_test_save_result_txt(struct fts_test *tdata)
{
    if (!tdata || !tdata->testresult) {
        FTS_TEST_ERROR("test result is null");
        return;
    }

    FTS_TEST_INFO("test result length in txt:%d", tdata->testresult_len);
    fts_test_save_test_data(FTS_TXT_FILE_NAME, tdata->testresult,
                            tdata->testresult_len);
}
#endif


/*****************************************************************************
* Name: fts_test_save_data
* Brief: Save test data.
*        If multi-data of MC, length of data package must be tx*rx,(tx+1)*rx
*        If multi-data of MC-SC, length of data package should be (tx+rx)*2
*        Need fill 0 when no actual data
* Input:
* Output:
* Return:
*****************************************************************************/
void fts_test_save_data(char *name, int code, int *data, int datacnt,
                        bool mc_sc, bool key, bool result)
{
    int datalen = datacnt;
    struct fts_test *tdata = fts_ftest;
    struct fts_test_data *td = &tdata->testdata;
    struct item_info *info = &td->info[td->item_count];

    if (!name || !data) {
        FTS_TEST_ERROR("name/data is null");
        return ;
    }

    strlcpy(info->name, name, TEST_ITEM_NAME_MAX - 1);
    info->code = code;
    info->mc_sc = mc_sc;
    info->key_support = key;
    info->result = result;
    if (datalen <= 0) {
        if (mc_sc) {
            datalen = tdata->sc_node.node_num * 2;
        } else {
            if (key && (tdata->node.key_num > 0))
                datalen = (tdata->node.tx_num + 1) * tdata->node.rx_num;
            else
                datalen = tdata->node.tx_num * tdata->node.rx_num;

        }
    }

    FTS_TEST_DBG("name:%s,len:%d", name, datalen);
    info->data = fts_malloc(datalen * sizeof(int));
    if (!info->data) {
        FTS_TEST_ERROR("malloc memory for item(%d) data fail", td->item_count);
        info->datalen = 0;
        return ;
    }
    memcpy(info->data, data, datalen * sizeof(int));
    info->datalen = datalen;

    td->item_count++;
}

static void fts_test_free_data(struct fts_test *tdata)
{
    int i = 0;
    struct fts_test_data *td = &tdata->testdata;

    for (i = 0; i < td->item_count; i++) {
        if (td->info[i].data) {
            fts_free(td->info[i].data);
        }
    }
}

static int fts_test_malloc_free_incell(struct fts_test *tdata, bool allocate)
{
    struct incell_threshold *thr = &tdata->ic.incell.thr;
    int buflen = tdata->node.node_num * sizeof(int);

    if (true == allocate) {
        FTS_TEST_INFO("buflen:%d", buflen);
        fts_malloc_r(thr->rawdata_min, buflen);
        fts_malloc_r(thr->rawdata_max, buflen);
        if (tdata->func->rawdata2_support) {
            fts_malloc_r(thr->rawdata2_min, buflen);
            fts_malloc_r(thr->rawdata2_max, buflen);
        }
        fts_malloc_r(thr->cb_min, buflen);
        fts_malloc_r(thr->cb_max, buflen);
    } else {
        fts_free(thr->rawdata_min);
        fts_free(thr->rawdata_max);
        if (tdata->func->rawdata2_support) {
            fts_free(thr->rawdata2_min);
            fts_free(thr->rawdata2_max);
        }
        fts_free(thr->cb_min);
        fts_free(thr->cb_max);
    }

    return 0;
}

static int fts_test_malloc_free_mc_sc(struct fts_test *tdata, bool allocate)
{
    struct mc_sc_threshold *thr = &tdata->ic.mc_sc.thr;
    int buflen = tdata->node.node_num * sizeof(int);
    int buflen_sc = tdata->sc_node.node_num * sizeof(int);

    if (true == allocate) {
        fts_malloc_r(thr->rawdata_h_min, buflen);
        fts_malloc_r(thr->rawdata_h_max, buflen);
        if (tdata->func->rawdata2_support) {
            fts_malloc_r(thr->rawdata_l_min, buflen);
            fts_malloc_r(thr->rawdata_l_max, buflen);
        }
        fts_malloc_r(thr->tx_linearity_max, buflen);
        fts_malloc_r(thr->tx_linearity_min, buflen);
        fts_malloc_r(thr->rx_linearity_max, buflen);
        fts_malloc_r(thr->rx_linearity_min, buflen);

        fts_malloc_r(thr->scap_cb_off_min, buflen_sc);
        fts_malloc_r(thr->scap_cb_off_max, buflen_sc);
        fts_malloc_r(thr->scap_cb_on_min, buflen_sc);
        fts_malloc_r(thr->scap_cb_on_max, buflen_sc);
        fts_malloc_r(thr->scap_cb_hi_min, buflen_sc);
        fts_malloc_r(thr->scap_cb_hi_max, buflen_sc);
        fts_malloc_r(thr->scap_cb_hov_min, buflen_sc);
        fts_malloc_r(thr->scap_cb_hov_max, buflen_sc);

        fts_malloc_r(thr->scap_rawdata_off_min, buflen_sc);
        fts_malloc_r(thr->scap_rawdata_off_max, buflen_sc);
        fts_malloc_r(thr->scap_rawdata_on_min, buflen_sc);
        fts_malloc_r(thr->scap_rawdata_on_max, buflen_sc);
        fts_malloc_r(thr->scap_rawdata_hi_min, buflen_sc);
        fts_malloc_r(thr->scap_rawdata_hi_max, buflen_sc);
        fts_malloc_r(thr->scap_rawdata_hov_min, buflen_sc);
        fts_malloc_r(thr->scap_rawdata_hov_max, buflen_sc);

        fts_malloc_r(thr->panel_differ_min, buflen);
        fts_malloc_r(thr->panel_differ_max, buflen);
        fts_malloc_r(thr->noise_min, buflen);
        fts_malloc_r(thr->noise_max, buflen);

        fts_malloc_r(thr->low_freq_rawdata_min, buflen);
        fts_malloc_r(thr->low_freq_rawdata_max, buflen);
        fts_malloc_r(thr->high_freq_rawdata_min, buflen);
        fts_malloc_r(thr->high_freq_rawdata_max, buflen);

        fts_malloc_r(thr->low_freq_rawdata_tx_linearity_max, buflen);
        fts_malloc_r(thr->low_freq_rawdata_rx_linearity_max, buflen);
        fts_malloc_r(thr->low_freq_rawdata_tx_linearity_min, buflen);
        fts_malloc_r(thr->low_freq_rawdata_rx_linearity_min, buflen);

        fts_malloc_r(thr->high_freq_rawdata_tx_linearity_max, buflen);
        fts_malloc_r(thr->high_freq_rawdata_rx_linearity_max, buflen);
        fts_malloc_r(thr->high_freq_rawdata_tx_linearity_min, buflen);
        fts_malloc_r(thr->high_freq_rawdata_rx_linearity_min, buflen);
    } else {
        fts_free(thr->rawdata_h_min);
        fts_free(thr->rawdata_h_max);
        if (tdata->func->rawdata2_support) {
            fts_free(thr->rawdata_l_min);
            fts_free(thr->rawdata_l_max);
        }
        fts_free(thr->tx_linearity_max);
        fts_free(thr->tx_linearity_min);
        fts_free(thr->rx_linearity_max);
        fts_free(thr->rx_linearity_min);

        fts_free(thr->scap_cb_off_min);
        fts_free(thr->scap_cb_off_max);
        fts_free(thr->scap_cb_on_min);
        fts_free(thr->scap_cb_on_max);
        fts_free(thr->scap_cb_hi_min);
        fts_free(thr->scap_cb_hi_max);
        fts_free(thr->scap_cb_hov_min);
        fts_free(thr->scap_cb_hov_max);

        fts_free(thr->scap_rawdata_off_min);
        fts_free(thr->scap_rawdata_off_max);
        fts_free(thr->scap_rawdata_on_min);
        fts_free(thr->scap_rawdata_on_max);
        fts_free(thr->scap_rawdata_hi_min);
        fts_free(thr->scap_rawdata_hi_max);
        fts_free(thr->scap_rawdata_hov_min);
        fts_free(thr->scap_rawdata_hov_max);

        fts_free(thr->panel_differ_min);
        fts_free(thr->panel_differ_max);

        fts_free(thr->noise_min);
        fts_free(thr->noise_max);

        fts_free(thr->low_freq_rawdata_min);
        fts_free(thr->low_freq_rawdata_max);
        fts_free(thr->high_freq_rawdata_min);
        fts_free(thr->high_freq_rawdata_max);

        fts_free(thr->low_freq_rawdata_tx_linearity_max);
        fts_free(thr->low_freq_rawdata_rx_linearity_max);
        fts_free(thr->low_freq_rawdata_tx_linearity_min);
        fts_free(thr->low_freq_rawdata_rx_linearity_min);

        fts_free(thr->high_freq_rawdata_tx_linearity_max);
        fts_free(thr->high_freq_rawdata_rx_linearity_max);
        fts_free(thr->high_freq_rawdata_tx_linearity_min);
        fts_free(thr->high_freq_rawdata_rx_linearity_min);
    }

    return 0;
}

static int fts_test_malloc_free_sc(struct fts_test *tdata, bool allocate)
{
    struct sc_threshold *thr = &tdata->ic.sc.thr;
    int buflen = tdata->node.node_num * sizeof(int);

    if (true == allocate) {
        fts_malloc_r(thr->rawdata_min, buflen);
        fts_malloc_r(thr->rawdata_max, buflen);
        fts_malloc_r(thr->cb_min, buflen);
        fts_malloc_r(thr->cb_max, buflen);
        fts_malloc_r(thr->dcb_sort, buflen);
        fts_malloc_r(thr->dcb_base, buflen);
    } else {
        fts_free(thr->rawdata_min);
        fts_free(thr->rawdata_max);
        fts_free(thr->cb_min);
        fts_free(thr->cb_max);
        fts_free(thr->dcb_sort);
        fts_free(thr->dcb_base);
    }

    return 0;
}

static int fts_test_malloc_free_thr(struct fts_test *tdata, bool allocate)
{
    int ret = 0;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("tdata/func is NULL\n");
        return -EINVAL;
    }

    if (true == allocate) {
        fts_malloc_r(tdata->node_valid, tdata->node.node_num * sizeof(int));
        fts_malloc_r(tdata->node_valid_sc, tdata->sc_node.node_num * sizeof(int));
    } else {
        fts_free(tdata->node_valid);
        fts_free(tdata->node_valid_sc);
    }

    switch (tdata->func->hwtype) {
    case IC_HW_INCELL:
        ret = fts_test_malloc_free_incell(tdata, allocate);
        break;
    case IC_HW_MC_SC:
        ret = fts_test_malloc_free_mc_sc(tdata, allocate);
        break;
    case IC_HW_SC:
        ret = fts_test_malloc_free_sc(tdata, allocate);
        break;
    default:
        FTS_TEST_SAVE_ERR("test ic type(%d) fail\n", tdata->func->hwtype);
        ret = -EINVAL;
        break;
    }

    return ret;
}

/* default enable all test item */
static void fts_test_init_item(struct fts_test *tdata)
{
    switch (tdata->func->hwtype) {
    case IC_HW_INCELL:
        tdata->ic.incell.u.tmp = 0xFFFFFFFF;
        break;
    case IC_HW_MC_SC:
        tdata->ic.mc_sc.u.tmp = 0xFFFFFFFF;
        break;
    case IC_HW_SC:
        tdata->ic.sc.u.tmp = 0xFFFFFFFF;
        break;
    }
}

static int get_tx_rx_num(u8 tx_rx_reg, u8 *ch_num, u8 ch_num_max)
{
    int ret = 0;
    int i = 0;

    for (i = 0; i < 3; i++) {
        ret = fts_test_read_reg(tx_rx_reg, ch_num);
        if ((ret < 0) || (*ch_num > ch_num_max)) {
            sys_delay(50);
        } else
            break;
    }

    if (i >= 3) {
        FTS_TEST_ERROR("get channel num fail");
        return -EIO;
    }

    return 0;
}
static int get_key_num(int *key_num_en, int max_key_num)
{
    int ret = 0;
    u8 key_en = 0;

    if (!max_key_num) {
        FTS_TEST_DBG("not support key, don't read key num register");
        return 0;
    }

    ret = fts_test_read_reg(FACTORY_REG_LEFT_KEY, &key_en);
    if (ret >= 0) {
        if (key_en & 0x01) {
            (*key_num_en)++;
        }

        if (key_en & 0x02) {
            (*key_num_en)++;
        }

        if (key_en & 0x04) {
            (*key_num_en)++;
        }
    }

    ret = fts_test_read_reg(FACTORY_REG_RIGHT_KEY, &key_en);
    if (ret >= 0) {
        if (key_en & 0x01) {
            (*key_num_en)++;
        }

        if (key_en & 0x02) {
            (*key_num_en)++;
        }

        if (key_en & 0x04) {
            (*key_num_en)++;
        }
    }

    if (*key_num_en > max_key_num) {
        FTS_TEST_ERROR("get key num, fw:%d > max:%d", *key_num_en, max_key_num);
        return -EIO;
    }

    return ret;
}

static int get_channel_num(struct fts_test *tdata)
{
    int ret = 0;
    u8 tx_num = 0;
    u8 rx_num = 0;
    int key_num = 0;
    ktime_t start_time = ktime_get();

    /* node structure */
    if (IC_HW_SC == tdata->func->hwtype) {
        ret = get_tx_rx_num(FACTORY_REG_CH_NUM_SC, &tx_num, NUM_MAX_SC);
        if (ret < 0) {
            FTS_TEST_ERROR("get channel number fail");
            return ret;
        }

        ret = get_tx_rx_num(FACTORY_REG_KEY_NUM_SC, &rx_num, KEY_NUM_MAX);
        if (ret < 0) {
            FTS_TEST_ERROR("get key number fail");
            return ret;
        }

        tdata->node.tx_num = 1;
        tdata->node.rx_num = tx_num;
        tdata->node.channel_num = tx_num;
        tdata->node.node_num = tx_num;
        key_num = rx_num;
    } else {
        ret = get_tx_rx_num(FACTORY_REG_CHX_NUM, &tx_num, TX_NUM_MAX);
        if (ret < 0) {
            FTS_TEST_ERROR("get tx_num fail");
            return ret;
        }

        ret = get_tx_rx_num(FACTORY_REG_CHY_NUM, &rx_num, RX_NUM_MAX);
        if (ret < 0) {
            FTS_TEST_ERROR("get rx_num fail");
            return ret;
        }

        if (IC_HW_INCELL == tdata->func->hwtype) {
            ret = get_key_num(&key_num, tdata->func->key_num_total);
            if (ret < 0) {
                FTS_TEST_ERROR("get key_num fail");
                return ret;
            }
        } else if (IC_HW_MC_SC == tdata->func->hwtype) {
            key_num = tdata->func->key_num_total;
        }
        tdata->node.tx_num = tx_num;
        tdata->node.rx_num = rx_num;
        if (IC_HW_INCELL == tdata->func->hwtype)
            tdata->node.channel_num = tx_num * rx_num;
        else if (IC_HW_MC_SC == tdata->func->hwtype)
            tdata->node.channel_num = tx_num + rx_num;
        tdata->node.node_num = tx_num * rx_num;
    }

    /* key */
    tdata->node.key_num = key_num;
    tdata->node.node_num += tdata->node.key_num;

    /* sc node structure */
    tdata->sc_node = tdata->node;
    if (IC_HW_MC_SC == tdata->func->hwtype) {
        if (tdata->v3_pattern) {
            ret = get_tx_rx_num(FACTORY_REG_CHX_NUM_NOMAP, &tx_num, TX_NUM_MAX);
            if (ret < 0) {
                FTS_TEST_ERROR("get no-mappint tx_num fail");
                return ret;
            }

            ret = get_tx_rx_num(FACTORY_REG_CHY_NUM_NOMAP, &rx_num, TX_NUM_MAX);
            if (ret < 0) {
                FTS_TEST_ERROR("get no-mapping rx_num fail");
                return ret;
            }

            tdata->sc_node.tx_num = tx_num;
            tdata->sc_node.rx_num = rx_num;
        }
        tdata->sc_node.channel_num = tx_num + rx_num;
        tdata->sc_node.node_num = tx_num + rx_num;
    }

    if (tdata->node.tx_num > TX_NUM_MAX) {
        FTS_TEST_ERROR("tx num(%d) fail", tdata->node.tx_num);
        return -EIO;
    }

    if (tdata->node.rx_num > RX_NUM_MAX) {
        FTS_TEST_ERROR("rx num(%d) fail", tdata->node.rx_num);
        return -EIO;
    }

    FTS_TEST_INFO("node_num:%d, tx:%d, rx:%d, key:%d",
                  tdata->node.node_num, tdata->node.tx_num,
                  tdata->node.rx_num, tdata->node.key_num);

    FTS_INFO("channel Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return 0;
}

static int fts_test_init_basicinfo(struct fts_test *tdata)
{
    int ret = 0;
    u8 val = 0;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("tdata/func is NULL\n");
        return -EINVAL;
    }

    fts_test_read_reg(REG_FW_MAJOR_VER, &val);
    if (ret < 0) {
        FTS_ERROR("read fw major version fail,ret=%d\n", ret);
        return ret;
    }
    tdata->fw_major_ver = val;

    fts_test_read_reg(REG_FW_MINOR_VER, &val);
    if (ret < 0) {
        FTS_ERROR("read fw minor version fail,ret=%d\n", ret);
        return ret;
    }
    tdata->fw_minor_ver = val;

    if (IC_HW_INCELL == tdata->func->hwtype) {
        fts_test_read_reg(REG_VA_TOUCH_THR, &val);
        tdata->va_touch_thr = val;
        fts_test_read_reg(REG_VKEY_TOUCH_THR, &val);
        tdata->vk_touch_thr = val;
    }

    /* enter factory mode */
    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("enter factory mode fail\n");
        return ret;
    }

    if (IC_HW_MC_SC == tdata->func->hwtype) {
        fts_test_read_reg(FACTORY_REG_PATTERN, &val);
        tdata->v3_pattern = (1 == val) ? true : false;
        fts_test_read_reg(FACTORY_REG_NOMAPPING, &val);
        tdata->mapping = val;
    }

    /* enter into factory mode and read tx/rx num */
    ret = get_channel_num(tdata);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get channel number fail\n");
        return ret;
    }

    return ret;
}

static int fts_test_main_init(void)
{
    int ret = 0;
    struct fts_test *tdata = fts_ftest;

    FTS_TEST_FUNC_ENTER();
    /* Init fts_test_data to 0 before test,  */
    memset(&tdata->testdata, 0, sizeof(struct fts_test_data));

    /* get basic information: tx/rx num ... */
    ret = fts_test_init_basicinfo(tdata);
    if (ret < 0) {
        FTS_TEST_ERROR("test init basicinfo fail");
        return ret;
    }

    /* allocate memory for test threshold */
    ret = fts_test_malloc_free_thr(tdata, true);
    if (ret < 0) {
        FTS_TEST_ERROR("test malloc for threshold fail");
        return ret;
    }

    /* default enable all test item */
    fts_test_init_item(tdata);
    
#if CSV_SUPPORT
        ret = fts_test_malloc_csv(tdata);
        if (ret < 0) {
            FTS_TEST_ERROR("allocate memory for test data(csv) fail");
            return ret;
        }
#endif
    
#if TXT_SUPPORT
        ret = fts_test_malloc_txt(tdata);
        if (ret < 0) {
            FTS_TEST_ERROR("allocate memory for test data(txt) fail");
            return ret;
        }
#endif

    /* allocate test data buffer */
    tdata->buffer_length = (tdata->node.tx_num + 1) * tdata->node.rx_num;
    tdata->buffer_length *= sizeof(int) * 2;
    FTS_TEST_INFO("test buffer length:%d", tdata->buffer_length);
    tdata->buffer = (int *)fts_malloc(tdata->buffer_length);
    if (NULL == tdata->buffer) {
        FTS_TEST_ERROR("test buffer(%d) malloc fail", tdata->buffer_length);
        return -ENOMEM;
    }
    memset(tdata->buffer, 0, tdata->buffer_length);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int fts_test_main_exit(void)
{
    struct fts_test *tdata = fts_ftest;

    FTS_TEST_FUNC_ENTER();
#if CSV_SUPPORT
        if (tdata->func && tdata->func->save_data_private)
            fts_test_save_data_csv_private(tdata);
        else
            fts_test_save_data_csv(tdata);
#endif
#if TXT_SUPPORT
    fts_test_save_result_txt(tdata);
#endif

    /* free memory */
    fts_test_malloc_free_thr(tdata, false);

    /* free test data */
    fts_test_free_data(tdata);

    /*free test data buffer*/
    fts_free(tdata->buffer);

    FTS_TEST_FUNC_EXIT();
    return 0;
}


/*
 * fts_test_get_testparams - get test parameter from ini
 */
static int fts_test_get_testparams(char *config_name)
{
    int ret = 0;

    ret = fts_test_get_testparam_from_ini(config_name);

    return ret;
}

static int fts_test_start(void)
{
    int testresult = 0;
    struct fts_test *tdata = fts_ftest;

    if (tdata && tdata->func && tdata->func->start_test) {
        tdata->testdata.item_count = 0;
        testresult = tdata->func->start_test();
    } else {
        FTS_TEST_ERROR("test func/start_test func is null");
    }

    return testresult;
}

/*
 * fts_test_entry - test main entry
 *
 * warning - need disable irq & esdcheck before call this function
 *
 */
static int fts_test_entry(char *ini_file_name)
{
    int ret = 0;

    /* test initialize */
    ret = fts_test_main_init();
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto test_err;
    }

    /*Read parse configuration file*/
    FTS_TEST_SAVE_INFO("ini_file_name:%s\n", ini_file_name);
    ret = fts_test_get_testparams(ini_file_name);
    if (ret < 0) {
        FTS_TEST_ERROR("get testparam fail");
        goto test_err;
    }

    /* Start testing according to the test configuration */
    if (true == fts_test_start()) {
        FTS_TEST_SAVE_INFO("=======Tp test pass.");
        if (fts_ftest->s) seq_printf(fts_ftest->s, "=======Tp test pass.\n");
        fts_ftest->result = true;
    } else {
        FTS_TEST_SAVE_INFO("=======Tp test failure.");
        if (fts_ftest->s) seq_printf(fts_ftest->s, "=======Tp test failure.\n");
        fts_ftest->result = false;
#if defined(TEST_SAVE_FAIL_RESULT) && TEST_SAVE_FAIL_RESULT
        do_gettimeofday(&(fts_ftest->tv));
#endif
    }

test_err:
    fts_test_main_exit();
    enter_work_mode();
    return ret;
}


/*
 * fts_test_entry - test main entry
 *
 * warning - need disable irq & esdcheck before call this function
 *
 */
int fts_proc_test_entry(char *ini_file_name)
{
    int ret = 0;

    /* test initialize */
    ret = fts_test_main_init();
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        return ret;
    }

    /*Read parse configuration file*/
    FTS_TEST_SAVE_INFO("ini_file_name:%s\n", ini_file_name);
    ret = fts_test_get_testparams(ini_file_name);
    if (ret < 0) {
        FTS_TEST_ERROR("get testparam fail");
        return ret;
    }

    return 0;
}

int fts_proc_test_exit(void)
{
    struct fts_test *tdata = fts_ftest;

    FTS_TEST_FUNC_ENTER();

    /* free memory */
    fts_test_malloc_free_thr(tdata, false);

    /* free test data */
    fts_test_free_data(tdata);

    /*free test data buffer*/
    fts_free(tdata->buffer);

    FTS_TEST_FUNC_EXIT();
    return 0;
}



static ssize_t fts_test_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;
    ssize_t size = 0;

    mutex_lock(&input_dev->mutex);
    size += snprintf(buf + size, PAGE_SIZE, "FTS_INI_FILE_PATH:%s\n",
                     FTS_INI_FILE_PATH);
    size += snprintf(buf + size, PAGE_SIZE, "FTS_CSV_FILE_NAME:%s\n",
                     FTS_CSV_FILE_NAME);
    size += snprintf(buf + size, PAGE_SIZE, "FTS_TXT_FILE_NAME:%s\n",
                     FTS_TXT_FILE_NAME);
    mutex_unlock(&input_dev->mutex);

    return size;
}

static ssize_t fts_test_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    char fwname[FILE_NAME_LENGTH] = { 0 };
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev;

    if (ts_data->suspended) {
        FTS_INFO("In suspend, no test, return now");
        return -EINVAL;
    }

    input_dev = ts_data->input_dev;
    memset(fwname, 0, sizeof(fwname));
    snprintf(fwname, FILE_NAME_LENGTH, "%s", buf);
    fwname[count - 1] = '\0';
    FTS_TEST_DBG("fwname:%s.", fwname);

    mutex_lock(&input_dev->mutex);
    fts_irq_disable();

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
    fts_esdcheck_switch(DISABLE);
#endif

    fts_ftest->s = NULL;
    ret = fts_enter_test_environment(1);
    if (ret < 0) {
        FTS_ERROR("enter test environment fail");
    } else {
        fts_test_entry(fwname);
    }
    ret = fts_enter_test_environment(0);
    if (ret < 0) {
        FTS_ERROR("enter normal environment fail");
    }

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
    fts_esdcheck_switch(ENABLE);
#endif

    fts_irq_enable();
    mutex_unlock(&input_dev->mutex);

    return count;
}

/*  test from test.ini
 *  example:echo "***.ini" > fts_test
 */
static DEVICE_ATTR(fts_test, S_IRUGO | S_IWUSR, fts_test_show, fts_test_store);

static struct attribute *fts_test_attributes[] = {
    &dev_attr_fts_test.attr,
    NULL
};

static struct attribute_group fts_test_attribute_group = {
    .attrs = fts_test_attributes
};

#if CSV_SUPPORT
/* /proc/fts_test_csv */
static int fts_csv_show(struct seq_file *s, void *v)
{
    struct fts_test *tdata = s->private;
    int csv_size = 0;

    if (!tdata || !tdata->csv_file_buf) {
        FTS_TEST_ERROR("tdata/csv_file_buf is null");
        return -ENODATA;
    }

    csv_size = (int)strlen(tdata->csv_file_buf);
    FTS_TEST_INFO("csv_show, size=%d,%d", (int)s->size, csv_size);
    if (csv_size <= 0) {
        seq_printf(s, "no csv data, please do test first\n");
        return 0;
    }

    if (s->size < csv_size) {
        s->count = s->size;
        return 0;
    }

    seq_printf(s, "%s", tdata->csv_file_buf);
    return 0;
}

static int fts_csv_open(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
    return single_open(file, fts_csv_show, pde_data(inode));
#else
    return single_open(file, fts_csv_show, PDE_DATA(inode));
#endif
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops fts_proccsv_fops = {
    .proc_open = fts_csv_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations fts_proccsv_fops = {
    .open = fts_csv_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif
#endif

#if TXT_SUPPORT
/* /proc/fts_test_txt */
static int fts_txt_show(struct seq_file *s, void *v)
{
    struct fts_test *tdata = s->private;
    int txt_size = 0;

    if (!tdata || !tdata->testresult) {
        FTS_TEST_ERROR("tdata/testresult is null");
        return -ENODATA;
    }

    txt_size = (int)strlen(tdata->testresult);
    FTS_TEST_INFO("csv_show, size=%d,%d", (int)s->size, txt_size);
    if (txt_size <= 0) {
        seq_printf(s, "no csv data, please do test first\n");
        return 0;
    }

    if (s->size < txt_size) {
        s->count = s->size;
        return 0;
    }

    seq_printf(s, "%s", tdata->testresult);
    return 0;
}

static int fts_txt_open(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
    return single_open(file, fts_txt_show, pde_data(inode));
#else
    return single_open(file, fts_txt_show, PDE_DATA(inode));
#endif
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops fts_proctxt_fops = {
    .proc_open = fts_txt_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations fts_proctxt_fops = {
    .open = fts_txt_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif
#endif


static int fts_test_func_init(struct fts_ts_data *ts_data)
{
    int i = 0;
    int j = 0;
    u16 ic_stype = ts_data->ic_info.ids.type;
    struct test_funcs *func = test_func_list[0];
    int func_count = sizeof(test_func_list) / sizeof(test_func_list[0]);

    FTS_TEST_FUNC_ENTER();
    if (0 == func_count) {
        FTS_TEST_SAVE_ERR("test functions list is NULL, fail\n");
        return -ENODATA;
    }

    fts_ftest = (struct fts_test *)kzalloc(sizeof(*fts_ftest), GFP_KERNEL);
    if (NULL == fts_ftest) {
        FTS_TEST_ERROR("malloc memory for test fail");
        return -ENOMEM;
    }

    for (i = 0; i < func_count; i++) {
        func = test_func_list[i];
        for (j = 0; j < FTS_MAX_COMPATIBLE_TYPE; j++) {
            if (0 == func->ctype[j])
                break;
            else if (func->ctype[j] == ic_stype) {
                FTS_TEST_INFO("match test function,type:%x",
                    (int)func->ctype[j]);
                fts_ftest->func = func;
            }
        }
    }
    if (NULL == fts_ftest->func) {
        FTS_TEST_ERROR("no test function match, can't test");
        return -ENODATA;
    }

    fts_ftest->ts_data = ts_data;
    FTS_TEST_FUNC_EXIT();
    return 0;
}

/*run_os_test*/
#define RUN_OS_TEST_INI_FILE        "focaltech_testconf.ini"
char *goog_get_test_limit_name(void)
{
    struct fts_ts_data *ts_data = fts_data;
    if (!ts_data || !ts_data->pdata ||
        ts_data->pdata->test_limits_name[0] == '\0')
      return RUN_OS_TEST_INI_FILE;

    return ts_data->pdata->test_limits_name;
}

static int proc_run_os_test_show(struct seq_file *s, void *v)
{
    int ret = 0;
    struct fts_test *tdata = (struct fts_test *)s->private;
    struct fts_ts_data *ts_data = NULL;
    struct input_dev *input_dev = NULL;

    if (s->size <= (PAGE_SIZE * 4)) {
        s->count = s->size;
        FTS_TEST_ERROR("Buffer size:%d, return ", (int)s->size);
        return 0;
    }

    if (!tdata) {
        FTS_TEST_ERROR("test data is null, return");
        return -EINVAL;
    }

    ts_data = tdata->ts_data;
    input_dev = ts_data->input_dev;
    if (ts_data->suspended) {
        FTS_TEST_ERROR("In suspend, no test, return");
        return -EINVAL;
    }

    mutex_lock(&input_dev->mutex);
    fts_irq_disable();

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
    fts_esdcheck_switch(DISABLE);
#endif

    tdata->s = s;
    ret = fts_enter_test_environment(1);
    if (ret < 0) {
        FTS_ERROR("enter test environment fail");
    } else {
        fts_test_entry(goog_get_test_limit_name());
    }
    ret = fts_enter_test_environment(0);
    if (ret < 0) {
        FTS_ERROR("enter normal environment fail");
    }

#if defined(FTS_ESDCHECK_EN) && (FTS_ESDCHECK_EN)
    fts_esdcheck_switch(ENABLE);
#endif

    fts_irq_enable();
    mutex_unlock(&input_dev->mutex);

    return 0;
}

static int proc_run_os_test_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_run_os_test_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_run_os_test_fops = {
    .proc_open   = proc_run_os_test_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_run_os_test_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_run_os_test_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

static void seq_print_and_log_array(struct seq_file *s, int tx, int rx, int* data)
{
    if (s == NULL || data == NULL) {
        FTS_ERROR("Error: NULL ptr");
        return ;
    }

    int i, j, count = 0;
    char print_buf[512];

    for (i = 0; i < tx; i++) {
      for (j = 0; j < rx; j++) {
          seq_printf(s, "%d,", data[i*rx + j]);
          count += scnprintf(print_buf + count, sizeof(print_buf) - count, "%d,",
                             data[i*rx + j]);
      }
      seq_printf(s, "\n");
      FTS_INFO("%s", print_buf);
      count = 0;
    }
}


#define SEQ_PRINT_AND_LOG(fmt, ...) \
    do { \
        seq_printf(s, fmt, ##__VA_ARGS__); \
        FTS_INFO(fmt, ##__VA_ARGS__); \
    } while (0)

/*FW Version test*/
static int proc_test_fwver_show(struct seq_file *s, void *v)
{
    int ret = 0;
    u8 fw_major_ver = 0;
    u8 fw_minor_ver = 0;
    u8 vendor_id = 0;

    ret = fts_read_reg(REG_FW_MAJOR_VER, &fw_major_ver);
    if (ret < 0) {
        FTS_ERROR("FWVER read major version fail,ret=%d\n", ret);
        goto exit;
    }

    ret = fts_read_reg(REG_FW_MINOR_VER, &fw_minor_ver);
    if (ret < 0) {
        FTS_ERROR("FWVER read minor version fail,ret=%d\n", ret);
        goto exit;
    }

    ret = fts_read_reg(FTS_REG_VENDOR_ID, &vendor_id);
    if (ret < 0) {
        FTS_ERROR("FWVER read vendor id fail,ret=%d\n", ret);
        goto exit;
    }

    SEQ_PRINT_AND_LOG("Vendor ID:%02x, FWVER:V%02x_D%02x\n", vendor_id, fw_major_ver, fw_minor_ver);

exit:
    return ret;
}

static int proc_test_fwver_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_fwver_show, inode->i_private);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_fwver_fops = {
    .proc_open   = proc_test_fwver_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_fwver_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_fwver_open,
    .read  = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/*Channel Num test*/
static int proc_test_chnum_show(struct seq_file *s, void *v)
{
    int ret = 0;
    u8 tx = 0;
    u8 rx = 0;
    ktime_t start_time = ktime_get();

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    SEQ_PRINT_AND_LOG("TX:%02d, RX:%02d\n", tx, rx);

exit:
    enter_work_mode();

    FTS_INFO("chnum Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return ret;
}

static int proc_test_chnum_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_chnum_show, inode->i_private);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_chnum_fops = {
    .proc_open   = proc_test_chnum_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_chnum_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_chnum_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* HW Reset_Pin Test */
static int proc_test_hw_reset_show(struct seq_file *s, void *v)
{
    int ret = 0;
    u8 reg88_val = 0xFF;
    u8 tmp_val = 0;

    ret = fts_read_reg(FTS_TMP_REG_AD, &reg88_val);
    if (ret < 0) {
        FTS_ERROR("read reg88 fails");
        goto exit;
    }

    tmp_val = reg88_val + 1;
    ret = fts_write_reg(FTS_TMP_REG_AD, tmp_val);
    if (ret < 0) {
        FTS_ERROR("write reg88 fails");
        goto exit;
    }

    fts_reset_proc(200);

    ret = fts_read_reg(FTS_TMP_REG_AD, &tmp_val);
    if (ret < 0) {
        FTS_ERROR("read reg88 fails");
        goto exit;
    }

    if (tmp_val == reg88_val)
        SEQ_PRINT_AND_LOG("Reset Pin test PASS.\n");
    else
        SEQ_PRINT_AND_LOG("Reset Pin test FAIL.\n");

exit:
    return ret;
}

static int proc_test_hw_reset_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_hw_reset_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_hw_reset_fops = {
    .proc_open   = proc_test_hw_reset_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations proc_test_hw_reset_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_hw_reset_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* SW Reset Test */
static int proc_test_sw_reset_show(struct seq_file *s, void *v)
{
    int ret = 0;
    u8 reg88_val = 0;
    u8 tmp_val = 0;

    ret = fts_read_reg(FTS_TMP_REG_88, &reg88_val);
    if (ret < 0) {
        FTS_ERROR("read reg88 fails");
        goto exit;
    }

    ret = fts_write_reg(FTS_TMP_REG_88, 0x22);
    if (ret < 0) {
        FTS_ERROR("write reg88 fails for SW reset");
        goto exit;
    }

    ret = fts_write_reg(FTS_TMP_REG_SOFT_RESET, 0xAA);
    if (ret < 0) {
        FTS_ERROR("write 0xAA to reg 0xFC fails");
        goto exit;
    }

    ret = fts_write_reg(FTS_TMP_REG_SOFT_RESET, 0x66);
    if (ret < 0) {
        FTS_ERROR("write 0x66 to reg 0xFC fails");
        goto exit;
    }
    sys_delay(40);
    ret = fts_read_reg(FTS_TMP_REG_88, &tmp_val);
    if (ret < 0) {
        FTS_ERROR("read reg88 fails for SW reset");
        goto exit;
    }

    if (tmp_val == reg88_val)
        SEQ_PRINT_AND_LOG("SW Reset test PASS.\n");
    else
        SEQ_PRINT_AND_LOG("SW Reset test FAIL.\n");

exit:
    return ret;
}

static int proc_test_sw_reset_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_sw_reset_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_sw_reset_fops = {
    .proc_open   = proc_test_sw_reset_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations proc_test_sw_reset_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_sw_reset_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* Lot Code */
static int goog_get_lot_code(u8* data, size_t datalen)
{
    int ret = 0;
    u8 cmd = FACTORY_REG_LOT_CODE;

    /* enter factory mode */
    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        return ret;
    }

    ret = fts_read(&cmd, 1, data, datalen);

    enter_work_mode();
    return ret;
}

static int proc_test_lot_code_show(struct seq_file *s, void *v)
{
    int ret = 0;
    u8 data[15];

    ret = goog_get_lot_code(data, sizeof(data));
    if (ret != 0) {
        FTS_ERROR("get lot code faile %d", ret);
        return ret;
    }

    SEQ_PRINT_AND_LOG("Lot Code:");
    SEQ_PRINT_AND_LOG("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X "
                      "%02X %02X %02X %02X\n",
      data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],
      data[8],data[9],data[10],data[11],data[12],data[13],data[14]);

    return 0;
}

static int proc_test_lot_code_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_lot_code_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_lot_code_fops = {
    .proc_open   = proc_test_lot_code_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations proc_test_lot_code_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_lot_code_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* INT_Pin Test */
int int_test_has_interrupt = 0;
static int proc_test_int_show(struct seq_file *s, void *v)
{
    int ret = 0;
    int i = 0;
    ktime_t start_time = ktime_get();

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }

    fts_irq_enable();
    int_test_has_interrupt = 0;
    ret = fts_write_reg(FACTORY_REG_SCAN_ADDR2, 0x01);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }
    sys_delay(10);
    for (i = 0; i < 100; i++) {
        if (int_test_has_interrupt) {
            SEQ_PRINT_AND_LOG("INT Pin test PASS.\n");
            goto exit;
        } else {
            sys_delay(10);
        }
    }
    SEQ_PRINT_AND_LOG("INT Pin test FAIL.\n");

exit:
    enter_work_mode();

    FTS_INFO("INT Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return ret;
}

static int proc_test_int_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_int_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_int_fops = {
    .proc_open   = proc_test_int_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations proc_test_int_fops = {
    .open   = proc_test_int_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif


extern int fts_test_get_raw(int *raw, u8 tx, u8 rx);
extern int fts_test_get_baseline(int *raw,int *base_raw, u8 tx, u8 rx);
extern int fts_test_get_strength(u8 *base_raw, u16 base_raw_size);
extern int fts_test_get_uniformity_data(int *raw, int *rawdata_linearity, u8 tx, u8 rx);
extern int fts_test_get_scap_raw(int *scap_raw, u8 tx, u8 rx, int *fwcheck);
extern int fts_test_get_scap_cb(int *scap_cb, u8 tx, u8 rx, int *fwcheck);
extern int fts_test_get_short(int *short_data, u8 tx, u8 rx);
extern int fts_test_get_noise(int *noise, u8 tx, u8 rx);
extern int fts_test_get_panel_differ(int *panel_differ, u8 tx, u8 rx);
extern int fts_get_low_high_freq_rawdata(struct fts_test *tdata, int *data, bool only_high);
extern int fts_test_get_scap_noise(int *scap_noise_data, u8 tx, u8 rx, int *fwcheck);
extern int fts_test_get_short_ch_to_gnd(int *res, u8 *ab_ch, u8 tx, u8 rx);
extern int fts_test_get_short_ch_to_ch(int *res, u8 *ab_ch, u8 tx, u8 rx);

/* Rawdata test */
static int proc_test_raw_show(struct seq_file *s, void *v)
{
    int ret = 0;
    int node_num = 0;
    u8 tx = 0;
    u8 rx = 0;
    bool result = 0;
    struct mc_sc_threshold *thr = &fts_ftest->ic.mc_sc.thr;
    ktime_t start_time = ktime_get();
    struct fts_test *tdata = fts_ftest;
    int *raw = tdata->raw_data;

    if (!raw) {
        FTS_ERROR("raw mem is null,can not test rawdata");
        ret = -ENOMEM;
        goto exit;
    }

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }
    /* get Tx chanel number */
    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }
    /* get Rx chanel number */
    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx * rx;

    /* get raw data */
    fts_test_get_raw(raw, tx, rx);
    result = compare_array(raw,
                           thr->rawdata_h_min,
                           thr->rawdata_h_max,
                           false);
    tdata->pretest_raw = true;

    /* output raw data */
    seq_print_and_log_array(s, tx, rx, raw);

    SEQ_PRINT_AND_LOG("Rawdata Test %s\n", result? "PASS" : "NG");

exit:
    fts_proc_test_exit();
    enter_work_mode();

    FTS_INFO("Rawdata test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return ret;
}

static int proc_test_raw_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_raw_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_raw_fops = {
    .proc_open   = proc_test_raw_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_raw_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_raw_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* Baseline test */
static int proc_test_baseline_show(struct seq_file *s, void *v)
{
    int ret = 0;
    int i = 0;
    int node_num = 0;
    u8 tx = 0;
    u8 rx = 0;
    int *raw = NULL;
    int *base_raw = NULL;

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx * rx;
    raw = fts_malloc(node_num * sizeof(int));
    if (!raw) {
        FTS_ERROR("malloc memory for raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    base_raw = fts_malloc(node_num * sizeof(int));
    if (!base_raw) {
        FTS_ERROR("malloc memory for base_raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /* get baseline data */
    fts_test_get_baseline(raw, base_raw, tx, rx);

    /* output baseline data */
    SEQ_PRINT_AND_LOG("     ");
    for (i = 0; i < rx; i++)
        SEQ_PRINT_AND_LOG(" RX%02d ", (i + 1));

    for (i = 0; i < node_num; i++) {
        if ((i % rx) == 0)
            SEQ_PRINT_AND_LOG("\nTX%02d:%5d,", (i / rx  + 1), (raw[i]-base_raw[i]));
        else
            SEQ_PRINT_AND_LOG("%5d,", (raw[i]-base_raw[i]));
    }

    SEQ_PRINT_AND_LOG("\n\n");

exit:

    if (base_raw)
        fts_free(base_raw);

    if (raw)
        fts_free(raw);

    enter_work_mode();

    return 0;
}

static int proc_test_baseline_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_baseline_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_baseline_fops = {
    .proc_open   = proc_test_baseline_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_baseline_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_baseline_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* Strength test for full size */
/* Transpose raw */
void transpose_raw(u8 *src, u8 *dist, int tx, int rx, bool big_endian) {
    int i = 0;
    int j = 0;

    for (i = 0; i < tx; i++) {
        for (j = 0; j < rx; j++) {
            if (big_endian) {
                /* keep big_endian. */
                dist[(j * tx + i) * 2] = src[(i * rx + j) * 2];
                dist[(j * tx + i) * 2 + 1] = src[(i * rx + j) * 2 + 1];
           } else {
                /* transfer to big_endian. */
                dist[(j * tx + i) * 2] = src[(i * rx + j) * 2 + 1];
                dist[(j * tx + i) * 2 + 1] = src[(i * rx + j) * 2];
           }
        }
    }
}

static int proc_test_strength_show(struct seq_file *s, void *v)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;
    int i = 0;
    int node_num = 0;
    int self_node = 0;
    u8 tx = ts_data->pdata->tx_ch_num;
    u8 rx = ts_data->pdata->rx_ch_num;

    int ms_cap_idx = FTS_CAP_DATA_LEN + 28 + 1;
    int ss_cap_on_idx = ms_cap_idx +  tx*rx* sizeof(u16);
    int ss_cap_off_idx = ss_cap_on_idx + FTS_SELF_DATA_LEN * sizeof(u16);
    short base_result = 0;

    u8 *base_raw = NULL;
    u8 *trans_raw = NULL;
    int base_raw_size = 0;
    int base = 0;
    u8 tp_finger_cnt = 0;
    int tp_events_x = 0;
    int tp_events_y = 0;
    u8 tp_events_id = 0;

    ret = enter_work_mode();
    if (ret < 0) {
        goto exit;
    }

    node_num = tx * rx;
    self_node = tx + rx;

    base_raw_size = FTS_FULL_TOUCH_RAW_SIZE(tx, rx);
    FTS_DEBUG("base_raw size = %d", base_raw_size);
    base_raw = fts_malloc(base_raw_size);
    if (!base_raw) {
        FTS_ERROR("malloc memory for raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    trans_raw = fts_malloc(node_num * sizeof(u16));
    if (!trans_raw) {
        FTS_ERROR("malloc memory for transpose raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /* get strength data. */
    ret = fts_test_get_strength(base_raw, base_raw_size);
    if (ret < 0) {
        FTS_ERROR("get strength fails");
        goto exit;
    }

    tp_finger_cnt = base_raw[1] & 0x0F;
    if (tp_finger_cnt > FTS_MAX_POINTS_SUPPORT) {
        FTS_ERROR("The finger count(%d) is over than max fingers(%d)",
            tp_finger_cnt, FTS_MAX_POINTS_SUPPORT);
        tp_finger_cnt = FTS_MAX_POINTS_SUPPORT;
    }

    /*---------Output touch point-----------*/
    for (i = 0; i < tp_finger_cnt; i++) {
         base = FTS_ONE_TCH_LEN_V2 * i +3;
         tp_events_x = ((base_raw[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 12) \
                        + ((base_raw[FTS_TOUCH_OFF_XL + base] & 0xFF) << 4) \
                        + ((base_raw[FTS_TOUCH_OFF_PRE + base] >> 4) & 0x0F);
         tp_events_y = ((base_raw[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 12) \
                        + ((base_raw[FTS_TOUCH_OFF_YL + base] & 0xFF) << 4) \
                        + (base_raw[FTS_TOUCH_OFF_PRE + base] & 0x0F);
         tp_events_id = (base_raw[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
         seq_printf(s, "Finger ID = %d, x = %d, y = %d\n", tp_events_id,
                    FTS_TOUCH_HIRES(tp_events_x), FTS_TOUCH_HIRES(tp_events_y));
    }

    seq_printf(s, "     ");
    /* transpose data buffer. */
    FTS_DEBUG("index of MS = %d", ms_cap_idx);
    transpose_raw(base_raw + ms_cap_idx, trans_raw, tx, rx, true);
    for (i = 0; i < tx; i++)
        seq_printf(s, " TX%02d ", (i + 1));

    for (i = 0; i < node_num; i++) {
        base_result = (int)(trans_raw[(i * 2)] << 8) +
                      (int)trans_raw[(i * 2) + 1];
        if ((i % tx) == 0)
            seq_printf(s, "\nRX%02d:%5d,", (i / tx + 1), base_result);
        else
            seq_printf(s, "%5d,", base_result);
    }
    /*---------END touch point-----------*/

    /*---------output self of strength data-----------*/
    seq_printf(s, "\n");
    seq_printf(s, "Scap raw(proof on):\n");
    FTS_DEBUG("index of SS_ON = %d", ss_cap_on_idx);
    for (i = 0; i < self_node; i++) {
        base_result = (int)(base_raw[(i * 2) + ss_cap_on_idx] << 8) +
                      (int)base_raw[(i * 2) + ss_cap_on_idx + 1];

        if (i == 0)
            seq_printf(s, "RX:");

        if (i == rx) {
            FTS_DEBUG("index(tx) = %d", (ss_cap_on_idx + (i * 2)));
            seq_printf(s, "\n");
            seq_printf(s, "TX:");
        }
        seq_printf(s, "%d,", base_result);
    }
    seq_printf(s, "\n\n");
    seq_printf(s, "Scap raw(proof off):\n");
    FTS_DEBUG("index of SS_OFF = %d", ss_cap_off_idx);
    for (i = 0; i < self_node; i++) {
        base_result = (int)(base_raw[(i * 2) + ss_cap_off_idx] << 8) +
                      (int)base_raw[(i * 2) + ss_cap_off_idx + 1];

        if (i == 0)
            seq_printf(s, "RX:");

        if (i == rx){
            seq_printf(s, "\n");
            seq_printf(s, "TX:");
        }
        seq_printf(s, "%d,", base_result);
    }

    seq_printf(s, "\n\n");
    /*---------END self of strength data-----------*/

exit:
    if (trans_raw)
        fts_free(trans_raw);

    if (base_raw)
        fts_free(base_raw);

    return ret;
}

static int proc_test_strength_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_strength_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_strength_fops = {
    .proc_open    = proc_test_strength_open,
    .proc_read    = seq_read,
    .proc_lseek   = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations proc_test_strength_fops = {
    .owner   = THIS_MODULE,
    .open    = proc_test_strength_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = single_release,
};
#endif

/* Rawdata_Uniformity test */
static int proc_test_uniformity_show(struct seq_file *s, void *v)
{
    int ret = 0;
    int node_num = 0;
    u8 tx = 0;
    u8 rx = 0;
    int *uniformity = NULL;
    bool result = 0;
    struct mc_sc_threshold *thr = &fts_ftest->ic.mc_sc.thr;
    struct fts_test *tdata = fts_ftest;

    ktime_t start_time = ktime_get();

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    if (tdata->pretest_raw == false) {
        FTS_ERROR("please test raw first!");
        seq_printf(s, "please test raw first!\n");
        goto exit;
    }
    tdata->pretest_raw = false;

    node_num = tx * rx;
    uniformity = fts_malloc(node_num * 2 * sizeof(int));
    if (!uniformity) {
        FTS_ERROR("malloc memory for raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /* get raw data */
    fts_test_get_uniformity_data(tdata->raw_data, uniformity, tx, rx);

    /* output raw data */
    SEQ_PRINT_AND_LOG("Rawdata Uniformity TX:\n");

    seq_print_and_log_array(s, tx, rx, uniformity);

    result = compare_array(uniformity,
                               thr->tx_linearity_min,
                               thr->tx_linearity_max,
                               false);
    SEQ_PRINT_AND_LOG("Rawdata Uniformity TX %s\n", result? "PASS" : "NG");

    SEQ_PRINT_AND_LOG("Rawdata Uniformity RX:\n");
    seq_print_and_log_array(s, tx, rx, &uniformity[node_num]);
    result = compare_array(uniformity + node_num,
                                thr->rx_linearity_min,
                                thr->rx_linearity_max,
                                false);
    SEQ_PRINT_AND_LOG("Rawdata Uniformity RX %s\n", result? "PASS" : "NG");

exit:
    if (uniformity)
        fts_free(uniformity);
    fts_proc_test_exit();
    enter_work_mode();

    FTS_INFO("Rawdata Uniformity Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return 0;
}

static int proc_test_uniformity_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_uniformity_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_uniformity_fops = {
    .proc_open   = proc_test_uniformity_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_uniformity_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_uniformity_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* Low High test */
static int proc_test_low_high_fre_uniformity(bool only_high)
{
    int ret = 0;
    int node_num = 0;
    u8 tx = 0;
    u8 rx = 0;
    int *uniformity = NULL;
    ktime_t start_time = ktime_get();

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx * rx;
    uniformity = fts_malloc(node_num * 6 * sizeof(int));
    if (!uniformity) {
        FTS_ERROR("malloc memory for raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /* get raw data */
    fts_get_low_high_freq_rawdata(fts_ftest, uniformity, only_high);

exit:
    if (uniformity)
        fts_free(uniformity);

    fts_proc_test_exit();
    enter_work_mode();

    FTS_INFO("low high frq Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return 0;
}

static int proc_test_low_high_fre_uniformity_show(struct seq_file *s, void *v)
{
    seq_printf(s, "%s", fts_ftest->log_buf);
    return 0;
}


static int proc_test_low_high_fre_uniformity_open(struct inode *inode, struct file *file)
{
    proc_test_low_high_fre_uniformity(false);
    return single_open(file, proc_test_low_high_fre_uniformity_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_low_high_fre_uniformity_fops = {
    .proc_open   = proc_test_low_high_fre_uniformity_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_low_high_fre_uniformity_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_low_high_fre_uniformity_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

static int proc_test_high_fre_uniformity_open(struct inode *inode,
                                              struct file *file)
{
    proc_test_low_high_fre_uniformity(true);
    return single_open(file, proc_test_low_high_fre_uniformity_show,
                       pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_high_fre_uniformity_fops = {
    .proc_open   = proc_test_high_fre_uniformity_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_high_fre_uniformity_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_high_fre_uniformity_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* Scap Rawdata test */
static int proc_test_sraw_show(struct seq_file *s, void *v)
{
    int ret = 0;
    int i = 0;
    int node_num = 0;
    int offset = 0;
    int *sraw = NULL;
    int fwcheck = 0;
    u8 tx = 0;
    u8 rx = 0;
    bool result = true;
    struct mc_sc_threshold *thr = &fts_ftest->ic.mc_sc.thr;
    struct fts_test *tdata = fts_ftest;
    ktime_t start_time = ktime_get();

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx + rx;
    sraw = fts_malloc(node_num * 3 * sizeof(int));
    if (!sraw) {
        FTS_ERROR("malloc memory for sraw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /* get raw data */
    fts_test_get_scap_raw(sraw, tx, rx, &fwcheck);
    SEQ_PRINT_AND_LOG("Scap raw checked:%X\n", fwcheck);

    /* output raw data */
    if ((fwcheck & 0x01) || (fwcheck & 0x02)) {
        SEQ_PRINT_AND_LOG("Scap raw(proof on):\n");
        SEQ_PRINT_AND_LOG("RX:");
        seq_print_and_log_array(s, 1, rx, sraw);

        SEQ_PRINT_AND_LOG("TX:");

        seq_print_and_log_array(s, 1, tx, &sraw[rx]);

        result = true;
        for (i = 0; i < tdata->sc_node.node_num; i++) {
            if (0 == tdata->node_valid_sc[i])
                continue;

            if (((fwcheck & 0x01) && (i < tdata->sc_node.rx_num)) ||
                ((fwcheck & 0x02) && (i >= tdata->sc_node.rx_num))) {
                if ((sraw[i + offset] < thr->scap_rawdata_off_min[i]) ||
                    (sraw[i + offset] > thr->scap_rawdata_off_max[i])) {
                    FTS_TEST_SAVE_ERR("test fail,CH%d=%5d,range=(%5d,%5d)\n",
                                      i + 1, sraw[i],
                                      thr->scap_rawdata_off_min[i],
                                      thr->scap_rawdata_off_max[i]);
                    result = false;
                }
            }
        }
        SEQ_PRINT_AND_LOG("Scap raw(proof on) %s\n", result? "PASS" : "NG");
        offset += node_num;
    }

    if ((fwcheck & 0x04) || (fwcheck & 0x08)) {
        SEQ_PRINT_AND_LOG("Scap raw(proof off):\n");
        SEQ_PRINT_AND_LOG("RX:");
        seq_print_and_log_array(s, 1, rx, &sraw[node_num]);

        SEQ_PRINT_AND_LOG("TX:");
        seq_print_and_log_array(s, 1, tx, &sraw[node_num + rx]);

        result = true;
        for (i = 0; i < tdata->sc_node.node_num; i++) {
            if (0 == tdata->node_valid_sc[i])
                continue;

            if (((fwcheck & 0x04) && (i < tdata->sc_node.rx_num)) ||
                ((fwcheck & 0x08) && (i >= tdata->sc_node.rx_num))) {
                if ((sraw[i + offset] < thr->scap_rawdata_on_min[i]) ||
                    (sraw[i + offset] > thr->scap_rawdata_on_max[i])) {
                    FTS_TEST_SAVE_ERR("test fail,CH%d=%5d,range=(%5d,%5d)\n",
                                      i + 1, sraw[i],
                                      thr->scap_rawdata_off_min[i],
                                      thr->scap_rawdata_off_max[i]);
                    result = false;
                }
            }
        }
        SEQ_PRINT_AND_LOG("Scap raw(proof off) %s\n", result? "PASS" : "NG");
        offset += node_num;
    }

    if ((fwcheck & 0x10) || (fwcheck & 0x20)) {
        SEQ_PRINT_AND_LOG("Scap raw(high):\n");
        SEQ_PRINT_AND_LOG("RX:");
        for (i = node_num * 2; i < node_num * 2 + rx; i++) {
            SEQ_PRINT_AND_LOG("%d,", sraw[i]);
        }
        SEQ_PRINT_AND_LOG("\n");

        SEQ_PRINT_AND_LOG("TX:");
        for (i = node_num * 2 + rx; i < node_num * 3; i++) {
            SEQ_PRINT_AND_LOG("%d,", sraw[i]);
        }
        SEQ_PRINT_AND_LOG("\n");

        result = true;
        for (i = 0; i < tdata->sc_node.node_num; i++) {
            if (0 == tdata->node_valid_sc[i])
                continue;

            if (((fwcheck & 0x01) && (i < tdata->sc_node.rx_num)) ||
                ((fwcheck & 0x02) && (i >= tdata->sc_node.rx_num))) {
                if ((sraw[i + offset] < thr->scap_rawdata_hov_min[i]) ||
                    (sraw[i + offset] > thr->scap_rawdata_hov_max[i])) {
                    FTS_TEST_SAVE_ERR("test fail,CH%d=%5d,range=(%5d,%5d)\n",
                                      i + 1, sraw[i],
                                      thr->scap_rawdata_hov_min[i],
                                      thr->scap_rawdata_hov_max[i]);
                    result = false;
                }
            }
        }
        SEQ_PRINT_AND_LOG("Scap raw(high) %s\n", result? "PASS" : "NG");
    }

exit:
    if (sraw)
        fts_free(sraw);

    fts_proc_test_exit();
    enter_work_mode();

    FTS_INFO("Scap raw Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return ret;
}

static int proc_test_sraw_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_sraw_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_sraw_fops = {
    .proc_open   = proc_test_sraw_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_sraw_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_sraw_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* Scap CB test */
static int proc_test_scb_show(struct seq_file *s, void *v)
{
    int ret = 0;
    int i = 0;
    int node_num = 0;
    int *scb = NULL;
    int fwcheck = 0;
    u8 tx = 0;
    u8 rx = 0;
    bool result = true;
    struct mc_sc_threshold *thr = &fts_ftest->ic.mc_sc.thr;
    struct fts_test *tdata = fts_ftest;
    int offset = 0;

    ktime_t start_time = ktime_get();

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx + rx;
    scb = fts_malloc(node_num * 6 * sizeof(int));
    if (!scb) {
        FTS_ERROR("malloc memory for scb fails");
        ret = -ENOMEM;
        goto exit;
    }

    /* get raw data */
    fts_test_get_scap_cb(scb, tx, rx, &fwcheck);
    SEQ_PRINT_AND_LOG("Scap cb checked:%X\n", fwcheck);
    /* output raw data */
    if ((fwcheck & 0x01) || (fwcheck & 0x02)) {
        SEQ_PRINT_AND_LOG("Scap CB(proof on):\n");
        SEQ_PRINT_AND_LOG("RX:");
        seq_print_and_log_array(s, 1, rx, scb);

        SEQ_PRINT_AND_LOG("TX:");
        seq_print_and_log_array(s, 1, tx, &scb[rx]);

        result = true;
        for (i = 0; i < tdata->sc_node.node_num; i++) {
            if (0 == tdata->node_valid_sc[i])
                continue;

            if (((fwcheck & 0x01) && (i < tdata->sc_node.rx_num)) ||
                ((fwcheck & 0x02) && (i >= tdata->sc_node.rx_num))) {
                if ((scb[i + offset] < thr->scap_cb_off_min[i]) ||
                    (scb[i + offset] > thr->scap_cb_off_max[i])) {
                    FTS_TEST_SAVE_ERR("test fail,CH%d=%5d,range=(%5d,%5d)\n",
                                      i + 1, scb[i + offset],
                                      thr->scap_cb_off_min[i],
                                      thr->scap_cb_off_max[i]);
                    result = false;
                }
            }
        }
        SEQ_PRINT_AND_LOG("Scap CB(proof on) %s\n", result? "PASS" : "NG");
        offset += tdata->sc_node.node_num;
    }

    if ((fwcheck & 0x04) || (fwcheck & 0x08)) {
        SEQ_PRINT_AND_LOG("Scap CB(proof off):\n");
        SEQ_PRINT_AND_LOG("RX:");
        seq_print_and_log_array(s, 1, rx, &scb[node_num]);

        SEQ_PRINT_AND_LOG("TX:");
        seq_print_and_log_array(s, 1, tx, &scb[node_num + rx]);

        result = true;
        for (i = 0; i < tdata->sc_node.node_num; i++) {
            if (0 == tdata->node_valid_sc[i])
                continue;

            if (((fwcheck & 0x04) && (i < tdata->sc_node.rx_num)) ||
                ((fwcheck & 0x08) && (i >= tdata->sc_node.rx_num))) {
                if ((scb[i + offset] < thr->scap_cb_on_min[i]) ||
                    (scb[i + offset] > thr->scap_cb_on_max[i])) {
                    FTS_TEST_SAVE_ERR("test fail,CH%d=%5d,range=(%5d,%5d)\n",
                                      i + 1, scb[i + offset],
                                      thr->scap_cb_on_min[i],
                                      thr->scap_cb_on_max[i]);
                    result = false;
                }
            }
        }
        SEQ_PRINT_AND_LOG("Scap CB(proof off) %s\n", result? "PASS" : "NG");
        offset += tdata->sc_node.node_num;
    }

    if ((fwcheck & 0x10) || (fwcheck & 0x20)) {
        SEQ_PRINT_AND_LOG("Scap raw(high):\n");

        SEQ_PRINT_AND_LOG("RX:");
        for (i = node_num * 2; i < node_num * 2 + rx; i++) {
            SEQ_PRINT_AND_LOG("%d,", scb[i]);
        }
        SEQ_PRINT_AND_LOG("\n");

        SEQ_PRINT_AND_LOG("TX:");
        for (i = node_num * 2 + rx; i < node_num * 3; i++) {
            SEQ_PRINT_AND_LOG("%d,", scb[i]);
        }
        SEQ_PRINT_AND_LOG("\n");

        result = true;
        for (i = 0; i < tdata->sc_node.node_num; i++) {
            if (0 == tdata->node_valid_sc[i])
                continue;

            if (((fwcheck & 0x10) && (i < tdata->sc_node.rx_num)) ||
                ((fwcheck & 0x20) && (i >= tdata->sc_node.rx_num))) {
                if ((scb[i + offset] < thr->scap_cb_hov_min[i]) ||
                    (scb[i + offset] > thr->scap_cb_hov_max[i])) {
                    FTS_TEST_SAVE_ERR("test fail,CH%d=%5d,range=(%5d,%5d)\n",
                                      i + 1, scb[i + offset],
                                      thr->scap_cb_hov_min[i],
                                      thr->scap_cb_hov_max[i]);
                    result = false;
                }
            }
        }
        SEQ_PRINT_AND_LOG("Scap CB(high) %s\n", result? "PASS" : "NG");
        offset += tdata->sc_node.node_num;
    }

exit:
    if (scb)
        fts_free(scb);

    fts_proc_test_exit();
    enter_work_mode();

    FTS_INFO("Scap CB Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return ret;
}

static int proc_test_scb_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_scb_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_scb_fops = {
    .proc_open   = proc_test_scb_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_scb_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_scb_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* Noise test */
static int proc_noise_test(void)
{
    int ret = 0;
    int i = 0;
    int node_num = 0;
    int scap_node_num = 0;
    u8 tx = 0;
    u8 rx = 0;
    int *noise = NULL;
    int *snoise = NULL;
    bool result = 0;
    int fwcheck = 0;
    int offset = 0;
    struct mc_sc_threshold *thr = &fts_ftest->ic.mc_sc.thr;
    struct fts_test *tdata = fts_ftest;
    char *log_buf = tdata->log_buf;
    int count = 0;
    int row_start = 0;

    ktime_t start_time = ktime_get();

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx * rx;
    noise = fts_malloc(node_num * sizeof(int));
    if (!noise) {
        FTS_ERROR("malloc memory for raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /*get raw data*/
    fts_test_get_noise(noise, tx, rx);

    /*output raw data*/

    FTS_INFO("Noise Test:\n");
    count += scnprintf(log_buf + count, PAGE_SIZE, "Noise Test:\n");
    row_start = count;

    for (i = 0; i < node_num; i++) {
        if ((i + 1) % rx) {
            count += scnprintf(log_buf + count, PAGE_SIZE, "%d,", noise[i]);
        } else {
            count += scnprintf(log_buf + count, PAGE_SIZE, "%d,\n", noise[i]);

            FTS_INFO("%s", &log_buf[row_start]);
            row_start = count;
        }
    }

    count += scnprintf(log_buf + count, PAGE_SIZE, "\n\n");

    result = compare_array(noise, thr->noise_min, thr->noise_max, false);
    FTS_INFO("Noise Test %s\n", result? "PASS" : "NG");
    count += scnprintf(log_buf + count, PAGE_SIZE, "Noise Test %s\n", result? "PASS" : "NG");

    /**************scap noise*******************/
    scap_node_num = tx + rx;
    snoise = fts_malloc(scap_node_num * 3 * sizeof(int));
    if (!snoise) {
        FTS_ERROR("malloc memory for snoise fails");
        ret = -ENOMEM;
        goto exit;
    }
    fts_test_get_scap_noise(snoise, tx, rx, &fwcheck);
    FTS_INFO("Scap noise checked:%X\n", fwcheck);
    count += scnprintf(log_buf + count, PAGE_SIZE, "Scap noise checked:%X\n", fwcheck);

        /* output raw data */
        if ((fwcheck & 0x01) || (fwcheck & 0x02)) {
            FTS_INFO("Scap noise(proof on):\n");
            count += scnprintf(log_buf + count, PAGE_SIZE, "Scap noise(proof on):\n");
            FTS_INFO("RX:");
            count += scnprintf(log_buf + count, PAGE_SIZE, "RX:");

            row_start = count;
            for (i = 0; i < rx; i++) {
                count += scnprintf(log_buf + count, PAGE_SIZE, "%d,", snoise[i]);
            }
            FTS_INFO("%s", &log_buf[row_start]);

            count += scnprintf(log_buf + count, PAGE_SIZE, "\n");
            FTS_INFO("TX:");
            count += scnprintf(log_buf + count, PAGE_SIZE, "TX:");

            row_start = count;
            for (i = rx; i < scap_node_num; i++) {
                count += scnprintf(log_buf + count, PAGE_SIZE, "%d,", snoise[i]);
            }
            FTS_INFO("%s", &log_buf[row_start]);
            count += scnprintf(log_buf + count, PAGE_SIZE, "\n");

            result = true;
            for (i = 0; i < tdata->sc_node.node_num; i++) {
                if (0 == tdata->node_valid_sc[i])
                    continue;

                if (((fwcheck & 0x01) && (i < tdata->sc_node.rx_num)) ||
                    ((fwcheck & 0x02) && (i >= tdata->sc_node.rx_num))) {
                    if ((snoise[i + offset] < thr->basic.scap_noise_on_min) ||
                        (snoise[i + offset] > thr->basic.scap_noise_on_max)) {
                        FTS_TEST_SAVE_ERR("test fail,CH%d=%5d,range=(%5d,%5d)\n",
                                          i + 1, snoise[i],
                                          thr->basic.scap_noise_on_min,
                                          thr->basic.scap_noise_on_max);
                        result = false;
                    }
                }
            }

            FTS_INFO("Scap noise(proof on) %s\n", result? "PASS" : "NG");
            count += scnprintf(log_buf + count, PAGE_SIZE, "Scap noise(proof on) %s\n", result? "PASS" : "NG");
            offset += scap_node_num;
        }

        if ((fwcheck & 0x04) || (fwcheck & 0x08)) {
            FTS_INFO("Scap noise(proof off):\n");
            count += scnprintf(log_buf + count, PAGE_SIZE, "Scap noise(proof off):\n");
            FTS_INFO("RX:");
            count += scnprintf(log_buf + count, PAGE_SIZE, "RX:");

            row_start = count;
            for (i = scap_node_num; i < scap_node_num + rx; i++) {
                count += scnprintf(log_buf + count, PAGE_SIZE, "%d,", snoise[i]);
            }
            FTS_INFO("%s", &log_buf[row_start]);

            count += scnprintf(log_buf + count, PAGE_SIZE, "\n");

            FTS_INFO("TX:");
            count += scnprintf(log_buf + count, PAGE_SIZE, "TX:");
            row_start = count;
            for (i = scap_node_num + rx; i < scap_node_num * 2; i++) {
                count += scnprintf(log_buf + count, PAGE_SIZE, "%d,", snoise[i]);
            }
            FTS_INFO("%s", &log_buf[row_start]);
            count += scnprintf(log_buf + count, PAGE_SIZE, "\n");

            result = true;
            for (i = 0; i < tdata->sc_node.node_num; i++) {
                if (0 == tdata->node_valid_sc[i])
                    continue;

                if (((fwcheck & 0x04) && (i < tdata->sc_node.rx_num)) ||
                    ((fwcheck & 0x08) && (i >= tdata->sc_node.rx_num))) {
                    if ((snoise[i + offset] < thr->basic.scap_noise_off_min) ||
                        (snoise[i + offset] > thr->basic.scap_noise_off_max)) {
                        FTS_TEST_SAVE_ERR("test fail,CH%d=%5d,range=(%5d,%5d)\n",
                                          i + 1, snoise[i],
                                          thr->basic.scap_noise_off_min,
                                          thr->basic.scap_noise_off_max);
                        result = false;
                    }
                }
            }

            FTS_INFO("Scap noise(proof off) %s\n", result? "PASS" : "NG");
            count += scnprintf(log_buf + count, PAGE_SIZE,
            "Scap noise(proof off) %s\n", result? "PASS" : "NG");
            offset += scap_node_num;
        }

        if ((fwcheck & 0x10) || (fwcheck & 0x20)) {
            count += scnprintf(log_buf + count, PAGE_SIZE, "Scap noise(high):\n");
            count += scnprintf(log_buf + count, PAGE_SIZE, "RX:");
            for (i = scap_node_num * 2; i < scap_node_num * 2 + rx; i++) {
                count += scnprintf(log_buf + count, PAGE_SIZE, "%d,", snoise[i]);
            }
            count += scnprintf(log_buf + count, PAGE_SIZE, "\n");

            count += scnprintf(log_buf + count, PAGE_SIZE, "TX:");
            for (i = scap_node_num * 2 + rx; i < scap_node_num * 3; i++) {
                count += scnprintf(log_buf + count, PAGE_SIZE, "%d,", snoise[i]);
            }

            count += scnprintf(log_buf + count, PAGE_SIZE, "\n");

            result = true;
            for (i = 0; i < tdata->sc_node.node_num; i++) {
                if (0 == tdata->node_valid_sc[i])
                    continue;

                if (((fwcheck & 0x01) && (i < tdata->sc_node.rx_num)) ||
                    ((fwcheck & 0x02) && (i >= tdata->sc_node.rx_num))) {
                    if ((snoise[i + offset] < thr->basic.scap_noise_off_min) ||
                        (snoise[i + offset] > thr->basic.scap_noise_off_max)) {
                        FTS_TEST_SAVE_ERR("test fail,CH%d=%5d,range=(%5d,%5d)\n",
                                          i + 1, snoise[i],
                                          thr->basic.scap_noise_off_min,
                                          thr->basic.scap_noise_off_max);
                        result = false;
                    }
                }
            }

            count += scnprintf(log_buf + count, PAGE_SIZE,
                "Scap noise(high) %s\n", result? "PASS" : "NG");
        }
exit:
    if (noise)
        fts_free(noise);
    if (snoise)
        fts_free(snoise);

    fts_proc_test_exit();
    enter_work_mode();

    FTS_INFO("Scap noise Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return ret;
}

static int proc_test_noise_show(struct seq_file *s, void *v)
{
    seq_printf(s, "%s", fts_ftest->log_buf);
    return 0;
}


static int proc_test_noise_open(struct inode *inode, struct file *file)
{
    proc_noise_test();
    return single_open(file, proc_test_noise_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_noise_fops = {
    .proc_open   = proc_test_noise_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release  = single_release,
};
#else
static const struct file_operations proc_test_noise_fops = {
    .owner  = THIS_MODULE,
    .open   = proc_test_noise_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* Short test */
static int proc_test_short_show(struct seq_file *s, void *v)
{
    int ret = 0;
    int i = 0;
    int node_num = 0;
    u8 tx = 0;
    u8 rx = 0;
    int *short_data = NULL;
    int *short_data_cg = NULL;
    int *short_data_cc = NULL;
    bool result = 1;
    bool cg_result = 1;
    bool cc_result = 1;
    int code = 0;
    struct fts_test *tdata = fts_ftest;
    u8 ab_ch[SC_NUM_MAX + 1] = { 0 };
    u8 ab_ch_num = 0;
    int temp = 0;
    int j = 0;
    int adc_cnt = 0;
    bool is_cc_short = false;
    bool is_cg_short = false;
    int tmp_num = 0;
    ktime_t start_time = ktime_get();

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }
    /* get Tx chanel number */
    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }
    /* get Rx chanel number */
    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx + rx;
    short_data = fts_malloc(node_num * sizeof(int));
    short_data_cg = fts_malloc(node_num * sizeof(int));
    short_data_cc = fts_malloc(node_num * sizeof(int));
    if (!short_data || !short_data_cg || !short_data_cc) {
        FTS_ERROR("malloc memory for raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /* get short all data */
    fts_test_get_short(short_data, tx, rx);

    for (i = 0; i < node_num; i++) {
        code = short_data[i];

        if (code > 1500) {
            SEQ_PRINT_AND_LOG("adc(%d) > 1500 fail", code);
            result = false;
            continue;
        }

        if ((212 - ((code * 250 / 2047) + 40)) == 0) {
            short_data[i] = 50000;
            continue;
        }
        short_data[i] = fts_abs(((code * 25 / 2047 + 4) * 2005) /
                                (212 - ((code * 250 / 2047) + 40)));
        if (short_data[i] < tdata->ic.mc_sc.thr.basic.short_cg) {
            ab_ch_num++;
            ab_ch[ab_ch_num] = i;
            result = false;
        }
    }
    /* output short data */
    SEQ_PRINT_AND_LOG("TX:");
    seq_print_and_log_array(s, 1, tx, short_data);

    SEQ_PRINT_AND_LOG("RX:");
    seq_print_and_log_array(s, 1, rx, &short_data[tx]);

    if (result == true) goto short_end;

    ab_ch[0] = ab_ch_num;
    if (ab_ch_num) {
        SEQ_PRINT_AND_LOG("\nabnormal ch:[%*ph]\n", ab_ch_num, ab_ch);
    }
    /********************get short cg********************/
    fts_test_get_short_ch_to_gnd(short_data_cg, ab_ch, tx, rx);
    for (i = 0; i < ab_ch_num; i++) {
        temp = short_data_cg[i];
        if ((212 - ((temp * 250 / 2047) + 40)) == 0) {
            short_data_cg[i] = 50000;
            continue;
        }
        short_data_cg[i] = fts_abs(((temp * 25 / 2047 + 4) * 2005) /
                                   (212 - ((temp * 250 / 2047) + 40)));
        if (short_data_cg[i] < tdata->ic.mc_sc.thr.basic.short_cg) {
            cg_result = false;
            if (!is_cg_short) {
                SEQ_PRINT_AND_LOG("\nGND Short:\n");
                is_cg_short = true;
            }

            if (ab_ch[i + 1] <= tx) {
                SEQ_PRINT_AND_LOG("Tx%d with GND:", ab_ch[i + 1]);
            } else {
                SEQ_PRINT_AND_LOG( "Rx%d with GND:", (ab_ch[i + 1] - tx));
            }
            SEQ_PRINT_AND_LOG("%d(K)\n", short_data_cg[i]);
        }
    }


    /********************get short cc********************/
    tmp_num = ab_ch_num * (ab_ch_num - 1) / 2;
    tmp_num = (tmp_num > node_num) ? node_num : tmp_num;
    fts_test_get_short_ch_to_ch(short_data_cc, ab_ch, tx, rx);

    for (i = 0; i < ab_ch_num; i++) {
        for (j = i + 1; j < ab_ch_num; j++) {
            if (adc_cnt >= tmp_num)
                break;

            temp = short_data_cc[adc_cnt];
            if ((212 - ((temp * 250 / 2047) + 40)) == 0) {
                short_data_cc[adc_cnt] = 50000;
                continue;
            }
            short_data_cc[adc_cnt] = fts_abs(((temp * 25 / 2047 + 4) * 2005) /
                                        (212 - ((temp * 250 / 2047) + 40)));
            if (short_data_cc[adc_cnt] < tdata->ic.mc_sc.thr.basic.short_cc) {
                cc_result = false;
                if (!is_cc_short) {
                    SEQ_PRINT_AND_LOG("\nMutual Short:\n");
                    is_cc_short = true;
                }

                if (ab_ch[i + 1] <= tx) {
                    SEQ_PRINT_AND_LOG("Tx%d with", (ab_ch[i + 1]));
                } else {
                    SEQ_PRINT_AND_LOG("Rx%d with", (ab_ch[i + 1] - tx));
                }

                if (ab_ch[j + 1] <= tx) {
                    SEQ_PRINT_AND_LOG(" Tx%d", (ab_ch[j + 1] ) );
                } else {
                    SEQ_PRINT_AND_LOG(" Rx%d", (ab_ch[j + 1] - tx));
                }
                SEQ_PRINT_AND_LOG(":%d(K)\n", short_data_cc[adc_cnt]);
            }
            adc_cnt++;
        }
    }

short_end:
    SEQ_PRINT_AND_LOG("Short Test %s\n", result? "PASS" : "NG");

exit:
    if (short_data)
        fts_free(short_data);
    fts_free(short_data_cg);
    fts_free(short_data_cc);

    fts_proc_test_exit();
    enter_work_mode();

    FTS_INFO("Short Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return ret;
}

static int proc_test_short_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_short_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_short_fops = {
    .proc_open   = proc_test_short_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations proc_test_short_fops = {
    .open   = proc_test_short_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

/* Panel_Differ test */
static int proc_test_panel_differ_show(struct seq_file *s, void *v)
{
    int ret = 0;
    int node_num = 0;
    u8 tx = 0;
    u8 rx = 0;
    int *panel_differ = NULL;
    bool result = 0;
    struct mc_sc_threshold *thr = &fts_ftest->ic.mc_sc.thr;
    ktime_t start_time = ktime_get();

    ret = fts_proc_test_entry(goog_get_test_limit_name());
    if (ret < 0) {
        FTS_TEST_ERROR("fts_test_main_init fail");
        goto exit;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_ERROR("enter factory mode fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHX_NUM, &tx);
    if (ret < 0) {
        FTS_ERROR("read tx fails");
        goto exit;
    }

    ret = fts_read_reg(FACTORY_REG_CHY_NUM, &rx);
    if (ret < 0) {
        FTS_ERROR("read rx fails");
        goto exit;
    }

    node_num = tx * rx;
    panel_differ = fts_malloc(node_num * sizeof(int));
    if (!panel_differ) {
        FTS_ERROR("malloc memory for raw fails");
        ret = -ENOMEM;
        goto exit;
    }

    /*get panel_differ data*/
    fts_test_get_panel_differ(panel_differ, tx, rx);

    /*output panel_differ data*/

    seq_print_and_log_array(s, tx, rx, panel_differ);

    result = compare_array(panel_differ,
                               thr->panel_differ_min,
                               thr->panel_differ_max,
                               false);


exit:
    SEQ_PRINT_AND_LOG("Panel Differ Test %s\n", result? "PASS" : "NG");

    if (panel_differ)
        fts_free(panel_differ);

    fts_proc_test_exit();
    enter_work_mode();

    FTS_INFO("Panel Differ Test %lldms taken",ktime_ms_delta(ktime_get(), start_time));
    return ret;
}

static int proc_test_panel_differ_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_test_panel_differ_show, pde_data(inode));
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops proc_test_panel_differ_fops = {
    .proc_open   = proc_test_panel_differ_open,
    .proc_read   = seq_read,
    .proc_lseek  = seq_lseek,
    .proc_release = single_release,
};
#else
static const struct file_operations proc_test_panel_differ_fops = {
    .open   = proc_test_panel_differ_open,
    .read   = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};
#endif

#define FTS_PROC_TEST_DIR       "selftest"

struct proc_dir_entry *fts_proc_test_dir;
struct proc_dir_entry *proc_run_os_test;
struct proc_dir_entry *proc_test_fwver;
struct proc_dir_entry *proc_test_chnum;
struct proc_dir_entry *proc_test_reset_pin;
struct proc_dir_entry *proc_test_sw_reset;
struct proc_dir_entry *proc_test_lot_code;

struct proc_dir_entry *proc_test_int_pin;
struct proc_dir_entry *proc_test_raw;
struct proc_dir_entry *proc_test_baseline;
struct proc_dir_entry *proc_test_strength;
struct proc_dir_entry *proc_test_uniformity;
struct proc_dir_entry *proc_test_low_high_rawdata;
struct proc_dir_entry *proc_test_high_rawdata;
struct proc_dir_entry *proc_test_sraw;
struct proc_dir_entry *proc_test_scb;
struct proc_dir_entry *proc_test_noise;
struct proc_dir_entry *proc_test_short;
struct proc_dir_entry *proc_test_panel_differ;


static int fts_create_test_procs(struct fts_ts_data *ts_data)
{
    int ret = 0;

    proc_run_os_test = proc_create_data("run_os_test", S_IRUSR,
        fts_proc_test_dir, &proc_run_os_test_fops, fts_ftest);
    if (!proc_run_os_test) {
        FTS_ERROR("create proc_run_os_test entry fail");
        return -ENOMEM;
    }

    proc_test_fwver = proc_create("FW_Version", S_IRUSR,
        ts_data->proc_touch_entry, &proc_test_fwver_fops);
    if (!proc_test_fwver) {
        FTS_ERROR("create proc_test_fwver entry fail");
        return -ENOMEM;
    }

    proc_test_chnum = proc_create("Channel_Num", S_IRUSR,
        ts_data->proc_touch_entry, &proc_test_chnum_fops);
    if (!proc_test_chnum) {
        FTS_ERROR("create proc_test_chnum entry fail");
        return -ENOMEM;
    }

    proc_test_reset_pin = proc_create("Reset_Pin", S_IRUSR,
        ts_data->proc_touch_entry, &proc_test_hw_reset_fops);
    if (!proc_test_reset_pin) {
        FTS_ERROR("create proc_test_reset_pin entry fail");
        return -ENOMEM;
    }

    proc_test_sw_reset = proc_create("SW_Reset", S_IRUSR,
        ts_data->proc_touch_entry, &proc_test_sw_reset_fops);
    if (!proc_test_sw_reset) {
        FTS_ERROR("create proc_test_sw_reset entry fail");
        return -ENOMEM;
    }

    proc_test_lot_code = proc_create("Lot_Code", S_IRUSR,
        ts_data->proc_touch_entry, &proc_test_lot_code_fops);
    if (!proc_test_lot_code) {
        FTS_ERROR("create proc_test_lot_code entry fail");
        return -ENOMEM;
    }

    proc_test_int_pin = proc_create("INT_PIN", S_IRUSR,
        ts_data->proc_touch_entry, &proc_test_int_fops);
    if (!proc_test_int_pin) {
        FTS_ERROR("create proc_test_int_pin entry fail");
        return -ENOMEM;
    }

    proc_test_raw = proc_create_data("Rawdata", S_IRUSR,
        fts_proc_test_dir, &proc_test_raw_fops, ts_data);
    if (!proc_test_raw) {
        FTS_ERROR("create proc_test_raw entry fail");
        return -ENOMEM;
    }

    proc_test_baseline = proc_create_data("Baseline", S_IRUSR,
        fts_proc_test_dir, &proc_test_baseline_fops, ts_data);
    if (!proc_test_baseline) {
        FTS_ERROR("create proc_test_baseline entry fail");
        return -ENOMEM;
    }

    proc_test_strength = proc_create_data("Strength", S_IRUSR,
        fts_proc_test_dir, &proc_test_strength_fops, ts_data);
    if (!proc_test_strength) {
        FTS_ERROR("create proc_test_strength entry fail");
        return -ENOMEM;
    }

    proc_test_uniformity = proc_create_data("Rawdata_Uniformity", S_IRUSR,
        fts_proc_test_dir, &proc_test_uniformity_fops, ts_data);
    if (!proc_test_uniformity) {
        FTS_ERROR("create proc_test_uniformity entry fail");
        return -ENOMEM;
    }

    proc_test_low_high_rawdata = proc_create_data("Low_High_Freq_Rawdata_Uniformity", S_IRUSR,
        fts_proc_test_dir, &proc_test_low_high_fre_uniformity_fops, ts_data);
    if (!proc_test_low_high_rawdata) {
        FTS_ERROR("create Low_High_Freq_Rawdata_Uniformity entry fail");
        return -ENOMEM;
    }

    proc_test_high_rawdata = proc_create_data("High_Freq_Rawdata_Uniformity", S_IRUSR,
        fts_proc_test_dir, &proc_test_high_fre_uniformity_fops, ts_data);
    if (!proc_test_high_rawdata) {
        FTS_ERROR("create High_Freq_Rawdata_Uniformity entry fail");
        return -ENOMEM;
    }

    proc_test_sraw = proc_create_data("Scap_Rawdata", S_IRUSR,
        fts_proc_test_dir, &proc_test_sraw_fops, ts_data);
    if (!proc_test_sraw) {
        FTS_ERROR("create proc_test_sraw entry fail");
        return -ENOMEM;
    }

    proc_test_scb = proc_create_data("Scap_CB", S_IRUSR,
        fts_proc_test_dir, &proc_test_scb_fops, ts_data);
    if (!proc_test_scb) {
        FTS_ERROR("create proc_test_scb entry fail");
        return -ENOMEM;
    }

    proc_test_noise = proc_create_data("Noise", S_IRUSR,
        fts_proc_test_dir, &proc_test_noise_fops, ts_data);
    if (!proc_test_noise) {
        FTS_ERROR("create proc_test_noise entry fail");
        return -ENOMEM;
    }

    proc_test_short = proc_create_data("Short", S_IRUSR,
        fts_proc_test_dir, &proc_test_short_fops, ts_data);
    if (!proc_test_short) {
        FTS_ERROR("create proc_test_short entry fail");
        return -ENOMEM;
    }

    proc_test_panel_differ = proc_create_data("Panel_Differ", S_IRUSR,
        fts_proc_test_dir, &proc_test_panel_differ_fops, ts_data);
    if (!proc_test_panel_differ) {
        FTS_ERROR("create proc_test_panel_differ entry fail");
        return -ENOMEM;
    }

    FTS_INFO("create test procs succeeds");
    return ret;
}

#define FTS_LOG_SIZE (20 * 1024)
#define FTS_TX_NUM 16
#define FTS_RX_NUM 36

int fts_test_init(struct fts_ts_data *ts_data)
{
    int ret = 0;

    FTS_TEST_FUNC_ENTER();
    /* get test function, must be the first step */
    ret = fts_test_func_init(ts_data);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("test functions init fail");
        return ret;
    }

    ret = sysfs_create_group(&ts_data->dev->kobj, &fts_test_attribute_group);
    if (0 != ret) {
        FTS_TEST_ERROR("sysfs(test) create fail");
        sysfs_remove_group(&ts_data->dev->kobj, &fts_test_attribute_group);
    } else {
        FTS_TEST_DBG("sysfs(test) create successfully");
    }

    fts_proc_test_dir = proc_mkdir(FTS_PROC_TEST_DIR,
        ts_data->proc_touch_entry);
    if (!fts_proc_test_dir) {
        FTS_ERROR("create %s fails", FTS_PROC_TEST_DIR);
        return -ENOMEM;
    }

    ret = fts_create_test_procs(ts_data);
    if (ret) {
        FTS_TEST_ERROR("create test procs fail");
    }

#if CSV_SUPPORT
    fts_ftest->proc_csv = proc_create_data("fts_test_csv", 0777, NULL, &fts_proccsv_fops, fts_ftest);
    if (NULL == fts_ftest->proc_csv) {
        FTS_ERROR("create proc_csv entry fail");
    }
#endif
#if TXT_SUPPORT
    fts_ftest->proc_txt = proc_create_data("fts_test_txt", 0777, NULL, &fts_proctxt_fops, fts_ftest);
    if (NULL == fts_ftest->proc_txt) {
        FTS_ERROR("create proc_txt entry fail");
    }
#endif
    fts_ftest->raw_data = fts_malloc(FTS_TX_NUM * FTS_RX_NUM * sizeof(int));
    if (!fts_ftest->raw_data) {
        FTS_ERROR("malloc memory for raw fails");
    }

    fts_ftest->log_buf = fts_malloc(FTS_LOG_SIZE);
    if (!fts_ftest->log_buf) {
        FTS_ERROR("malloc memory for log buf fails");
    }

    FTS_TEST_FUNC_EXIT();

    return ret;
}

int fts_test_exit(struct fts_ts_data *ts_data)
{
    struct fts_test *tdata = fts_ftest;
    
    FTS_TEST_FUNC_ENTER();

    if (fts_proc_test_dir)
        proc_remove(fts_proc_test_dir);
    sysfs_remove_group(&ts_data->dev->kobj, &fts_test_attribute_group);
#if CSV_SUPPORT
        proc_remove(tdata->proc_csv);
        if (tdata->csv_file_buf) {
            vfree(tdata->csv_file_buf);
            tdata->csv_file_buf = NULL;
        }
#endif
#if TXT_SUPPORT
        proc_remove(tdata->proc_txt);
        if (tdata->testresult) {
            vfree(tdata->testresult);
            tdata->testresult = NULL;
        }
#endif
    fts_free(fts_ftest->log_buf);
    fts_free(fts_ftest->raw_data);
    fts_free(tdata);

    FTS_TEST_FUNC_EXIT();
    return 0;
}
