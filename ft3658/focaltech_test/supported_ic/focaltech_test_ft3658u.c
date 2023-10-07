/************************************************************************
* Copyright (c) 2012-2020, Focaltech Systems (R), All Rights Reserved.
*
* File Name: Focaltech_test_ft5652.c
*
* Author: Focaltech Driver Team
*
* Created: 2018-03-08
*
* Abstract:
*
************************************************************************/

/*****************************************************************************
* included header files
*****************************************************************************/
#include "../focaltech_test.h"

/*****************************************************************************
* private constant and macro definitions using #define
*****************************************************************************/
static int short_test_ch_to_all(
    struct fts_test *tdata, int *adc, u8 *ab_ch, bool *result)
{
    int ret = 0;
    int i = 0;
    int short_res[SC_NUM_MAX + 1] = { 0 };
    int min_cc = tdata->ic.mc_sc.thr.basic.short_cc;
    int ch_num = tdata->sc_node.tx_num + tdata->sc_node.rx_num;
    int byte_num = 0;
    int code = 0;
    int code1 = 0;
    int offset = 0;
    int denominator = 0;
    int numerator = 0;
    u8 ab_ch_num = 0;

    FTS_TEST_DBG("short test:channel to all other\n");
    /* choose resistor_level */
    ret = fts_test_write_reg(FACTROY_REG_SHORT2_RES_LEVEL, 1);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write short resistor level fail\n");
        return ret;
    }

    /*get adc data*/
    byte_num = (ch_num + 1) * 2;
    ret = short_get_adc_data_mc(TEST_RETVAL_AA, byte_num, &adc[0], \
                                FACTROY_REG_SHORT2_CA);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get weak short data fail,ret:%d\n", ret);
        return ret;
    }
    //tdata->code1 = adc[ch_num];
    tdata->code1 = 1407;

    /*get resistor*/
    code1 = tdata->code1;
    offset = tdata->offset;
    for (i = 0; i < ch_num; i++) {
        code = adc[i];
        denominator = code1 - code + offset;
        if (denominator == 0) {
            short_res[i] = min_cc;
        } else {
            numerator = (code - offset + 395) * 112;
            short_res[i] = fts_abs(numerator / denominator - 3);
        }

        if (short_res[i] < min_cc) {
            ab_ch_num++;
            ab_ch[ab_ch_num] = i + 1;
        }
    }

    if (ab_ch_num) {
        FTS_TEST_SAVE_INFO("Offset:%d, Code1:%d\n", offset, code1);
        print_buffer(adc, ch_num + 1, ch_num + 1);
        print_buffer(short_res, ch_num, ch_num);
        ab_ch[0] = ab_ch_num;
        printk("[FTS_TS]ab_ch:");
        for (i = 0; i < ab_ch_num + 1; i++) {
            printk("%2d ", ab_ch[i]);
        }
        printk("\n");
        *result = false;
    } else {
        *result = true;
    }

    return 0;
}

static int short_test_ch_to_gnd(
    struct fts_test *tdata, int *adc, u8 *ab_ch, bool *result)
{
    int ret = 0;
    int i = 0;
    int short_res[SC_NUM_MAX + 1] = { 0 };
    int min_cg = tdata->ic.mc_sc.thr.basic.short_cg;
    int tx_num = tdata->sc_node.tx_num;
    int byte_num = 0;
    int code = 0;
    int code1 = 0;
    int offset = 0;
    int denominator = 0;
    int numerator = 0;
    u8 ab_ch_num = 0;
    bool is_cg_short = false;

    FTS_TEST_DBG("short test:channel to gnd\n");
    ab_ch_num = ab_ch[0];
    ret = fts_test_write(FACTROY_REG_SHORT2_AB_CH, ab_ch, ab_ch_num + 1);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write abnormal channel fail\n");
        return ret;
    }

    /* choose resistor_level */
    ret = fts_test_write_reg(FACTROY_REG_SHORT2_RES_LEVEL, 1);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write short resistor level fail\n");
        return ret;
    }

    /*get adc data*/
    byte_num = ab_ch_num * 2;
    ret = short_get_adc_data_mc(TEST_RETVAL_AA, byte_num, &adc[0], \
                                FACTROY_REG_SHORT2_CG);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get weak short data fail,ret:%d\n", ret);
        return ret;
    }

    /*get resistor*/
    *result = true;
    code1 = tdata->code1;
    offset = tdata->offset;
    for (i = 0; i < ab_ch_num; i++) {
        code = adc[i];
        denominator = code1 - code + offset;
        if (denominator == 0) {
            short_res[i] = min_cg;
        } else {
            numerator = (code - offset + 395) * 112;
            short_res[i] = fts_abs(numerator / denominator - 3);
        }

        if (short_res[i] < min_cg) {
            *result = false;
            if (!is_cg_short) {
                FTS_TEST_SAVE_INFO("GND Short:");
                is_cg_short = true;
            }

            if (ab_ch[i + 1] <= tx_num) {
                FTS_TEST_SAVE_INFO("Tx%d with GND:", ab_ch[i + 1]);
            } else {
                FTS_TEST_SAVE_INFO( "Rx%d with GND:", (ab_ch[i + 1] - tx_num));
            }
            FTS_TEST_SAVE_INFO("%d(K), ADC:%d\n", short_res[i], code);
        }
    }

    return 0;
}

static int short_test_ch_to_ch(
    struct fts_test *tdata, int *adc, u8 *ab_ch, bool *result)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    int adc_cnt = 0;
    int short_res[SC_NUM_MAX + 1] = { 0 };
    int min_cc = tdata->ic.mc_sc.thr.basic.short_cc;
    int tx_num = tdata->sc_node.tx_num;
    int ch_num = tdata->sc_node.tx_num + tdata->sc_node.rx_num;
    int byte_num = 0;
    int tmp_num = 0;
    int code = 0;
    int code1 = 0;
    int offset = 0;
    int denominator = 0;
    int numerator = 0;
    u8 ab_ch_num = 0;
    bool is_cc_short = false;

    FTS_TEST_DBG("short test:channel to channel\n");
    ab_ch_num = ab_ch[0];
    if (ab_ch_num < 2) {
        FTS_TEST_DBG("abnormal channel number<2, not run ch_ch test");
        return ret;
    }

    ret = fts_test_write(FACTROY_REG_SHORT2_AB_CH, ab_ch, ab_ch_num + 1);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write abnormal channel fail\n");
        return ret;
    }

    /* choose resistor_level */
    ret = fts_test_write_reg(FACTROY_REG_SHORT2_RES_LEVEL, 1);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write short resistor level fail\n");
        return ret;
    }

    /*get adc data*/
    /*channel to channel: num * (num - 1) / 2, max. node_num*/
    tmp_num = ab_ch_num * (ab_ch_num - 1) / 2;
    tmp_num = (tmp_num > ch_num) ? ch_num : tmp_num;
    byte_num = tmp_num * 2;
    ret = short_get_adc_data_mc(TEST_RETVAL_AA, byte_num, &adc[0], \
                                FACTROY_REG_SHORT2_CC);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get weak short data fail,ret:%d\n", ret);
        return ret;
    }

    /*get resistor*/
    *result = true;
    code1 = tdata->code1;
    offset = tdata->offset;
    for (i = 0; i < ab_ch_num; i++) {
        for (j = i + 1; j < ab_ch_num; j++) {
            if (adc_cnt >= tmp_num)
                break;
            code = adc[adc_cnt];
            denominator = code1 - code + offset;
            if (denominator == 0) {
                short_res[adc_cnt] = min_cc;
            } else {
                numerator = (code - offset + 395) * 112;
                short_res[adc_cnt] = fts_abs(numerator / denominator - 3);
            }

            if (short_res[adc_cnt] < min_cc) {
                *result = false;
                if (!is_cc_short) {
                    FTS_TEST_SAVE_INFO("Mutual Short:");
                    is_cc_short = true;
                }

                if (ab_ch[i + 1] <= tx_num) {
                    FTS_TEST_SAVE_INFO("Tx%d with", (ab_ch[i + 1]));
                } else {
                    FTS_TEST_SAVE_INFO("Rx%d with", (ab_ch[i + 1] - tx_num));
                }

                if (ab_ch[j + 1] <= tx_num) {
                    FTS_TEST_SAVE_INFO(" Tx%d", (ab_ch[j + 1] ) );
                } else {
                    FTS_TEST_SAVE_INFO(" Rx%d", (ab_ch[j + 1] - tx_num));
                }
                FTS_TEST_SAVE_INFO(":%d(K), ADC:%d\n",
                                   short_res[adc_cnt], code);
            }
            adc_cnt++;
        }
    }

    return 0;
}

/*
 * start_scan - start to scan a frame
 */
int ft5652_start_scan(int frame_num)
{
    int ret = 0;
    u8 addr = 0;
    u8 val = 0;
    u8 finish_val = 0;
    int times = 0;
    int max_retry_cnt = 100;
    struct fts_test *tdata = fts_ftest;

    if ((tdata == NULL) || (tdata->func == NULL)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    addr = DIVIDE_MODE_ADDR;
    val = 0xC0;
    finish_val = 0x40;

    /* write register to start scan */
    ret = fts_test_write_reg(addr, val);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write start scan mode fail\n");
        return ret;
    }

    sys_delay(frame_num * FACTORY_TEST_DELAY / 2);
    /* Wait for the scan to complete */
    while (times++ < max_retry_cnt) {
        sys_delay(FACTORY_TEST_DELAY);

        ret = fts_test_read_reg(addr, &val);
        if ((ret >= 0) && (val == finish_val)) {
            break;
        } else
            FTS_TEST_DBG("reg%x=%x,retry:%d", addr, val, times);
    }

    if (times >= max_retry_cnt) {
        FTS_TEST_SAVE_ERR("scan timeout\n");
        return -EIO;
    }

    return 0;
}

static int ft5652_rawdata_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    int i = 0;
    int *rawdata = NULL;
    u8 fre = 0;
    u8 data_sel = 0;
    u8 data_type = 0;
    bool result = false;
    struct mc_sc_threshold *thr = &tdata->ic.mc_sc.thr;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("====== Test Item: rawdata test");
    memset(tdata->buffer, 0, tdata->buffer_length);
    rawdata = tdata->buffer;

    if (!thr->rawdata_h_min || !thr->rawdata_h_max) {
        FTS_TEST_SAVE_ERR("rawdata_h_min/max is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        goto test_err;
    }

    /* rawdata test in mapping mode */
    ret = mapping_switch(MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* save origin value */
    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_TYPE, &data_type);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x5B fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &data_sel);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x06 error,ret=%d\n", ret);
        goto test_err;
    }

    /* set frequency high */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set frequency fail,ret=%d\n", ret);
        goto restore_reg;
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto restore_reg;
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, 0x01);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set raw type fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* select rawdata */
    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x00);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set fir fail,ret=%d\n", ret);
        goto restore_reg;
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto restore_reg;
        }
    }

    /*********************GET RAWDATA*********************/
    for (i = 0; i < 3; i++) {
        /* lost 3 frames, in order to obtain stable data */
        ret = get_rawdata(rawdata);
    }
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get rawdata fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* show test data */
    show_data(rawdata, false);

    /* compare */
    result = compare_array(rawdata,
                           thr->rawdata_h_min,
                           thr->rawdata_h_max,
                           false);

restore_reg:
    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, data_type);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore raw type fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, data_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x06 fail,ret=%d\n", ret);
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

test_err:
    if (result) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("====== rawdata test PASS");
        if (tdata->s) seq_printf(tdata->s, "------ rawdata test PASS\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_INFO("====== rawdata test NG");
        if (tdata->s) seq_printf(tdata->s, "------ rawdata test NG\n");
    }

    /* save test data */
    fts_test_save_data("Rawdata Test", CODE_M_RAWDATA_TEST,
                       rawdata, 0, false, false, *test_result);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft5652_uniformity_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    int index = 0;
    int row = 0;
    int col = 1;
    int i = 0;
    int deviation = 0;
    int max = 0;
    int min = 0;
    int uniform = 0;
    int *rawdata = NULL;
    int *rawdata_linearity = NULL;
    int *rl_tmp = NULL;
    int rl_cnt = 0;
    int offset = 0;
    int offset2 = 0;
    int tx_num = 0;
    int rx_num = 0;
    u8 fre = 0;
    u8 data_sel = 0;
    u8 data_type = 0;
    struct mc_sc_threshold *thr = &fts_ftest->ic.mc_sc.thr;
    bool result = false;
    bool result2 = false;
    bool result3 = false;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("====== Test Item: rawdata unfiormity test");
    memset(tdata->buffer, 0, tdata->buffer_length);
    rawdata = tdata->buffer;
    tx_num = tdata->node.tx_num;
    rx_num = tdata->node.rx_num;

    if (!thr->tx_linearity_max || !thr->rx_linearity_max
        || !tdata->node_valid) {
        FTS_TEST_SAVE_ERR("tx/rx_lmax/node_valid is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        goto test_err;
    }

    rawdata_linearity = fts_malloc(tdata->node.node_num * 2 * sizeof(int));
    if (!rawdata_linearity) {
        FTS_TEST_SAVE_ERR("rawdata_linearity buffer malloc fail");
        ret = -ENOMEM;
        goto test_err;
    }

    /* rawdata unfiormity test in mapping mode */
    ret = mapping_switch(MAPPING);
    if (ret) {
        FTS_TEST_SAVE_ERR("failed to switch_to_mapping,ret=%d", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_TYPE, &data_type);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x5B fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &data_sel);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x06 fail,ret=%d\n", ret);
        goto test_err;
    }

    /* set frequency high */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set frequency fail,ret=%d\n", ret);
        goto restore_reg;
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto restore_reg;
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, 0x01);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set raw type fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* select rawdata */
    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x00);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set fir fail,ret=%d\n", ret);
        goto restore_reg;
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto restore_reg;
        }
    }

    /* change register value before,need to lose 3 frame data */
    for (index = 0; index < 3; ++index) {
        ret = get_rawdata(rawdata);
    }
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get rawdata fail,ret=%d\n", ret);
        goto restore_reg;
    }
    print_buffer(rawdata, tdata->node.node_num, tdata->node.rx_num);

    result = true;
    if (thr->basic.uniformity_check_tx) {
        FTS_TEST_SAVE_INFO("Check Tx Linearity\n");
        rl_tmp = rawdata_linearity + rl_cnt;
        for (row = 0; row < tx_num; row++) {
            for (col = 1; col <  rx_num; col++) {
                offset = row * rx_num + col;
                offset2 = row * rx_num + col - 1;
                deviation = abs( rawdata[offset] - rawdata[offset2]);
                max = max(rawdata[offset], rawdata[offset2]);
                max = max ? max : 1;
                rl_tmp[offset] = 100 * deviation / max;
            }
        }
        /*show data in result.txt*/
        FTS_TEST_SAVE_INFO(" Tx Linearity:\n");
        show_data(rl_tmp, false);
        FTS_TEST_SAVE_INFO("\n" );

        /* compare */
        result = compare_array(rl_tmp,
                               thr->tx_linearity_min,
                               thr->tx_linearity_max,
                               false);

        rl_cnt += tdata->node.node_num;
    }

    result2 = true;
    if (thr->basic.uniformity_check_rx) {
        FTS_TEST_SAVE_INFO("Check Rx Linearity\n");
        rl_tmp = rawdata_linearity + rl_cnt;
        for (row = 1; row < tx_num; row++) {
            for (col = 0; col < rx_num; col++) {
                offset = row * rx_num + col;
                offset2 = (row - 1) * rx_num + col;
                deviation = abs(rawdata[offset] - rawdata[offset2]);
                max = max(rawdata[offset], rawdata[offset2]);
                max = max ? max : 1;
                rl_tmp[offset] = 100 * deviation / max;
            }
        }

        FTS_TEST_SAVE_INFO("Rx Linearity:\n");
        show_data(rl_tmp, false);
        FTS_TEST_SAVE_INFO("\n");

        /* compare */
        result2 = compare_array(rl_tmp,
                                thr->rx_linearity_min,
                                thr->rx_linearity_max,
                                false);
        rl_cnt += tdata->node.node_num;
    }

    result3 = true;
    if (thr->basic.uniformity_check_min_max) {
        FTS_TEST_SAVE_INFO("Check Min/Max\n") ;
        min = 100000;
        max = -100000;
        for (i = 0; i < tdata->node.node_num; i++) {
            if (0 == tdata->node_valid[i])
                continue;
            min = min(min, rawdata[i]);
            max = max(max, rawdata[i]);
        }
        max = !max ? 1 : max;
        uniform = 100 * abs(min) / abs(max);

        FTS_TEST_SAVE_INFO("min:%d, max:%d, get value of min/max:%d\n",
                           min, max, uniform);
        if (uniform < thr->basic.uniformity_min_max_hole) {
            result3 = false;
            FTS_TEST_SAVE_ERR("min_max out of range, set value: %d\n",
                              thr->basic.uniformity_min_max_hole);
        }
    }

restore_reg:
    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, data_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x06 fail,ret=%d\n", ret);
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, data_type);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore raw type fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

test_err:
    if (result && result2 && result3) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("====== uniformity test PASS");
        if (tdata->s) seq_printf(tdata->s, "------ uniformity test is Pass\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_ERR("====== uniformity test NG");
        if (tdata->s) seq_printf(tdata->s, "------ uniformity test is NG\n");
    }

    fts_test_save_data("Rawdata Uniformity Test",
                       CODE_M_RAWDATA_UNIFORMITY_TEST, rawdata_linearity,
                       tdata->node.node_num * 2, false, false, *test_result);

    fts_free(rawdata_linearity);
    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft5652_scap_cb_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    int i = 0;
    u8 wc_sel = 0;
    u8 sc_mode = 0;
    u8 hc_sel = 0;
    u8 hov_high = 0;
    int byte_num = 0;
    bool tmp_result = false;
    bool tmp2_result = false;
    bool tmp3_result = false;
    bool tmp4_result = false;
    bool fw_wp_check = false;
    bool tx_check = false;
    bool rx_check = false;
    int *scap_cb = NULL;
    int *scb_tmp = NULL;
    int scb_cnt = 0;
    struct mc_sc_threshold *thr = &tdata->ic.mc_sc.thr;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("====== Test Item: Scap CB Test");
    memset(tdata->buffer, 0, tdata->buffer_length);
    scap_cb = tdata->buffer;
    byte_num = tdata->sc_node.node_num * 2;

    if (tdata->sc_node.node_num * 4 > tdata->buffer_length) {
        FTS_TEST_SAVE_ERR("scap cb num(%d) > buffer length(%d)",
                          tdata->sc_node.node_num * 4,
                          tdata->buffer_length);
        ret = -EINVAL;
        goto test_err;
    }

    if (!thr->scap_cb_on_min || !thr->scap_cb_on_max
        || !thr->scap_cb_off_min || !thr->scap_cb_off_max
        || !thr->scap_cb_hi_min || !thr->scap_cb_hi_max
        || !thr->scap_cb_hov_min || !thr->scap_cb_hov_max) {
        FTS_TEST_SAVE_ERR("scap_cb_on/off/hi/hov_min/max is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    /* SCAP CB is in no-mapping mode */
    ret = mapping_switch(NO_MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch no-mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* get waterproof channel select */
    ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read water_channel_sel fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_MC_SC_MODE, &sc_mode);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read sc_mode fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_HC_SEL, &hc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read high_channel_sel fail,ret=%d\n", ret);
        goto test_err;
    }

    /* water proof on check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);
    if (thr->basic.scap_cb_wp_on_check && fw_wp_check) {
        scb_tmp = scap_cb + scb_cnt;
        /* 1:waterproof 0:non-waterproof */
        ret = get_cb_mc_sc(WATER_PROOF_ON, byte_num, scb_tmp, DATA_TWO_BYTE);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
            goto restore_reg;
        }

        /* show Scap CB */
        FTS_TEST_SAVE_INFO("scap_cb in waterproof on mode:\n");
        show_data_mc_sc(scb_tmp);

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_RX);
        tmp_result = compare_mc_sc(tx_check, rx_check, scb_tmp,
                                   thr->scap_cb_on_min,
                                   thr->scap_cb_on_max);

        scb_cnt += tdata->sc_node.node_num;
    } else {
        tmp_result = true;
    }

    /* water proof off check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);
    if (thr->basic.scap_cb_wp_off_check && fw_wp_check) {
        scb_tmp = scap_cb + scb_cnt;
        /* 1:waterproof 0:non-waterproof */
        ret = get_cb_mc_sc(WATER_PROOF_OFF, byte_num, scb_tmp, DATA_TWO_BYTE);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
            goto restore_reg;
        }

        /* show Scap CB */
        FTS_TEST_SAVE_INFO("scap_cb in waterproof off mode:\n");
        show_data_mc_sc(scb_tmp);

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_RX);
        tmp2_result = compare_mc_sc(tx_check, rx_check, scb_tmp,
                                    thr->scap_cb_off_min,
                                    thr->scap_cb_off_max);

        scb_cnt += tdata->sc_node.node_num;
    } else {
        tmp2_result = true;
    }

    /*high mode*/
    hov_high = (hc_sel & 0x03);
    if (thr->basic.scap_cb_hi_check && hov_high) {
        scb_tmp = scap_cb + scb_cnt;
        /* 1:waterproof 0:non-waterproof */
        ret = get_cb_mc_sc(HIGH_SENSITIVITY, byte_num, scb_tmp, DATA_TWO_BYTE);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
            goto restore_reg;
        }

        /* show Scap CB */
        FTS_TEST_SAVE_INFO("scap_cb in high mode:\n");
        show_data_mc_sc(scb_tmp);

        /* compare */
        tx_check = ((hov_high == 1) || (hov_high == 3));
        rx_check = ((hov_high == 2) || (hov_high == 3));
        tmp3_result = compare_mc_sc(tx_check, rx_check, scb_tmp,
                                    thr->scap_cb_hi_min,
                                    thr->scap_cb_hi_max);

        scb_cnt += tdata->sc_node.node_num;
    } else {
        tmp3_result = true;
    }

    /*hov mode*/
    hov_high = (hc_sel & 0x04);
    if (thr->basic.scap_cb_hov_check && hov_high) {
        scb_tmp = scap_cb + scb_cnt;
        byte_num = 4 * 2;
        /* 1:waterproof 0:non-waterproof */
        ret = get_cb_mc_sc(HOV, byte_num, scb_tmp, DATA_TWO_BYTE);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
            goto restore_reg;
        }

        /* show Scap CB */
        FTS_TEST_SAVE_INFO("scap_cb in hov mode:\n");
        show_data_mc_sc(scb_tmp);

        /* compare */
        tmp4_result = true;
        for (i = 0; i < 4; i++) {
            if ((scb_tmp[i] < thr->scap_cb_hov_min[i])
                || (scb_tmp[i] > thr->scap_cb_hov_max[i])) {
                FTS_TEST_SAVE_ERR("test fail,hov%d=%5d,range=(%5d,%5d)\n",
                                  i + 1, scb_tmp[i],
                                  thr->scap_cb_hov_min[i],
                                  thr->scap_cb_hov_max[i]);
                tmp4_result = false;
            }
        }

        scb_cnt += tdata->sc_node.node_num;
    } else {
        tmp4_result = true;
    }

restore_reg:
    ret = fts_test_write_reg(FACTORY_REG_WC_SEL, wc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore water_channel_sel fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, sc_mode);/* set the origin value */
    if (ret) {
        FTS_TEST_SAVE_ERR("restore sc mode fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_HC_SEL, hc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore high_channel_sel fail,ret=%d\n", ret);
    }

test_err:
    if (tmp_result && tmp2_result && tmp3_result && tmp4_result) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("====== scap cb test PASS");
        if (tdata->s) seq_printf(tdata->s, "------ scap cb test PASS\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_ERR("====== scap cb test NG");
        if (tdata->s) seq_printf(tdata->s, "------ scap cb test NG\n");
    }

    /* save test data */
    fts_test_save_data("SCAP CB Test", CODE_M_SCAP_CB_TEST,
                       scap_cb, scb_cnt, true, false, *test_result);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft5652_scap_rawdata_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    int i = 0;
    bool tmp_result = false;
    bool tmp2_result = false;
    bool tmp3_result = false;
    bool tmp4_result = false;
    bool fw_wp_check = false;
    bool tx_check = false;
    bool rx_check = false;
    int *scap_rawdata = NULL;
    int *srawdata_tmp = NULL;
    int srawdata_cnt = 0;
    u8 wc_sel = 0;
    u8 hc_sel = 0;
    u8 hov_high = 0;
    u8 data_type = 0;
    struct mc_sc_threshold *thr = &tdata->ic.mc_sc.thr;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("====== Test Item: Scap Rawdata Test");
    memset(tdata->buffer, 0, tdata->buffer_length);
    scap_rawdata = tdata->buffer;

    if ((tdata->sc_node.node_num * 4) > tdata->buffer_length) {
        FTS_TEST_SAVE_ERR("scap rawdata num(%d) > buffer length(%d)",
                          tdata->sc_node.node_num * 4,
                          tdata->buffer_length);
        ret = -EINVAL;
        goto test_err;
    }

    if (!thr->scap_rawdata_on_min || !thr->scap_rawdata_on_max
        || !thr->scap_rawdata_off_min || !thr->scap_rawdata_off_max
        || !thr->scap_rawdata_hi_min || !thr->scap_rawdata_hi_max
        || !thr->scap_rawdata_hov_min || !thr->scap_rawdata_hov_max) {
        FTS_TEST_SAVE_ERR("scap_rawdata_on/off/hi/hov_min/max is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    /* SCAP RAWDATA is in no-mapping mode */
    ret = mapping_switch(NO_MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch no-mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* get waterproof channel select */
    ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read water_channel_sel fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_HC_SEL, &hc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read high_channel_sel fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_TYPE, &data_type);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x5B fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, 0x01);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set raw type fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* scan rawdata */
    ret = start_scan();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("scan scap rawdata fail\n");
        goto restore_reg;
    }

    ret = start_scan();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("scan scap rawdata(2) fail\n");
        goto restore_reg;
    }

    /* water proof on check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);
    if (thr->basic.scap_rawdata_wp_on_check && fw_wp_check) {
        srawdata_tmp = scap_rawdata + srawdata_cnt;
        ret = get_rawdata_mc_sc(WATER_PROOF_ON, srawdata_tmp);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("get scap(WP_ON) rawdata fail\n");
            goto restore_reg;
        }

        FTS_TEST_SAVE_INFO("scap_rawdata in waterproof on mode:\n");
        show_data_mc_sc(srawdata_tmp);

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_RX);
        tmp_result = compare_mc_sc(tx_check, rx_check, srawdata_tmp,
                                   thr->scap_rawdata_on_min,
                                   thr->scap_rawdata_on_max);

        srawdata_cnt += tdata->sc_node.node_num;
    } else {
        tmp_result = true;
    }

    /* water proof off check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);
    if (thr->basic.scap_rawdata_wp_off_check && fw_wp_check) {
        srawdata_tmp = scap_rawdata + srawdata_cnt;
        ret = get_rawdata_mc_sc(WATER_PROOF_OFF, srawdata_tmp);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("get scap(WP_OFF) rawdata fail\n");
            goto restore_reg;
        }

        FTS_TEST_SAVE_INFO("scap_rawdata in waterproof off mode:\n");
        show_data_mc_sc(srawdata_tmp);

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_RX);
        tmp2_result = compare_mc_sc(tx_check, rx_check, srawdata_tmp,
                                    thr->scap_rawdata_off_min,
                                    thr->scap_rawdata_off_max);

        srawdata_cnt += tdata->sc_node.node_num;
    } else {
        tmp2_result = true;
    }

    /*high mode*/
    hov_high = (hc_sel & 0x03);
    if (thr->basic.scap_rawdata_hi_check && hov_high) {
        srawdata_tmp = scap_rawdata + srawdata_cnt;
        ret = get_rawdata_mc_sc(HIGH_SENSITIVITY, srawdata_tmp);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("get scap(HS) rawdata fail\n");
            goto restore_reg;
        }

        FTS_TEST_SAVE_INFO("scap_rawdata in hs mode:\n");
        show_data_mc_sc(srawdata_tmp);

        /* compare */
        tx_check = ((hov_high == 1) || (hov_high == 3));
        rx_check = ((hov_high == 2) || (hov_high == 3));
        tmp3_result = compare_mc_sc(tx_check, rx_check, srawdata_tmp,
                                    thr->scap_rawdata_hi_min,
                                    thr->scap_rawdata_hi_max);

        srawdata_cnt += tdata->sc_node.node_num;
    } else {
        tmp3_result = true;
    }

    /*hov mode*/
    hov_high = (hc_sel & 0x04);
    if (thr->basic.scap_rawdata_hov_check && hov_high) {
        srawdata_tmp = scap_rawdata + srawdata_cnt;
        ret = get_rawdata_mc_sc(HOV, srawdata_tmp);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("get scap(HOV) rawdata fail\n");
            goto restore_reg;
        }

        FTS_TEST_SAVE_INFO("scap_rawdata in hov mode:\n");
        show_data_mc_sc(srawdata_tmp);

        /* compare */
        tmp4_result = true;
        for (i = 0; i < 4; i++) {
            if ((srawdata_tmp[i] < thr->scap_rawdata_hov_min[i])
                || (srawdata_tmp[i] > thr->scap_rawdata_hov_max[i])) {
                FTS_TEST_SAVE_ERR("test fail,hov%d=%5d,range=(%5d,%5d)\n",
                                  i + 1, srawdata_tmp[i],
                                  thr->scap_rawdata_hov_min[i],
                                  thr->scap_rawdata_hov_max[i]);
                tmp4_result = false;
            }
        }

        srawdata_cnt += tdata->sc_node.node_num;
    } else {
        tmp4_result = true;
    }

restore_reg:
    ret = fts_test_write_reg(FACTORY_REG_WC_SEL, wc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore water_channel_sel fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_HC_SEL, hc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore high_channel_sel fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, data_type);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore raw type fail,ret=%d\n", ret);
    }

test_err:
    if (tmp_result && tmp2_result && tmp3_result && tmp4_result) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("====== scap rawdata test PASS");
        if (tdata->s) seq_printf(tdata->s, "------ scap rawdata test PASS\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_INFO("====== scap rawdata test NG");
        if (tdata->s) seq_printf(tdata->s, "------ scap rawdata test NG\n");
    }

    /* save data */
    fts_test_save_data("SCAP Rawdata Test", CODE_M_SCAP_RAWDATA_TEST,
                       scap_rawdata, srawdata_cnt, true, false, *test_result);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft5652_short_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    int ch_num = 0;
    int adc[SC_NUM_MAX + 1] = { 0 };
    u8 ab_ch[SC_NUM_MAX + 1] = { 0 };
    u8 res_level = 0;
    bool ca_result = false;
    bool cg_result = false;
    bool cc_result = false;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("====== Test Item: Short Test");
    ch_num = tdata->sc_node.tx_num + tdata->sc_node.rx_num;

    if (ch_num >= SC_NUM_MAX) {
        FTS_TEST_SAVE_ERR("sc_node ch_num(%d)>max(%d)", ch_num, SC_NUM_MAX);
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    /* short is in no-mapping mode */
    ret = mapping_switch(NO_MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch no-mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTROY_REG_SHORT2_RES_LEVEL, &res_level);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read short level fails\n");
        goto test_err;
    }

    /* get offset = readdata - 1024 */
    ret = short_get_adc_data_mc(TEST_RETVAL_AA, 1 * 2, &tdata->offset, \
                                FACTROY_REG_SHORT2_OFFSET);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get weak short data fail,ret:%d\n", ret);
        goto test_err;
    }
    tdata->offset -= 1024;
    FTS_TEST_DBG("short offset:%d", tdata->offset);

    /* get short resistance and exceptional channel */
    ret = short_test_ch_to_all(tdata, adc, ab_ch, &ca_result);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("short test of channel to all fails\n");
        goto restore_reg;
    }

    if (!ca_result) {
        /*weak short fail, get short values*/
        ret = short_test_ch_to_gnd(tdata, adc, ab_ch, &cg_result);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("short test of channel to gnd fails\n");
            goto restore_reg;
        }

        ret = short_test_ch_to_ch(tdata, adc, ab_ch, &cc_result);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("short test of channel to channel fails\n");
            goto restore_reg;
        }

    }

restore_reg:
    ret = fts_test_write_reg(FACTROY_REG_SHORT2_RES_LEVEL, res_level);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore short level fails");
    }

test_err:
    if (ca_result) {
        FTS_TEST_SAVE_INFO("====== short test PASS");
        if (tdata->s) seq_printf(tdata->s, "------ short test PASS\n");
        *test_result = true;
    } else {
        FTS_TEST_SAVE_ERR("====== short test NG");
        if (tdata->s) seq_printf(tdata->s, "------ short test NG\n");
        *test_result = false;
    }

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft5652_panel_differ_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    int i = 0;
    u8 fre = 0;
    u8 fir = 0;
    u8 normalize = 0;
    u8 data_type = 0;
    u8 data_sel = 0;
    int *panel_differ = NULL;
    struct mc_sc_threshold *thr = &tdata->ic.mc_sc.thr;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("====== Test Item: Panel Differ Test");
    memset(tdata->buffer, 0, tdata->buffer_length);
    panel_differ = tdata->buffer;

    if (!thr->panel_differ_min || !thr->panel_differ_max) {
        FTS_TEST_SAVE_ERR("panel_differ_h_min/max is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        goto test_err;
    }

    /* panel differ test in mapping mode */
    ret = mapping_switch(MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* save origin value */
    ret = fts_test_read_reg(FACTORY_REG_NORMALIZE, &normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read normalize fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_TYPE, &data_type);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x5B fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_FIR, &fir);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0xFB fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &data_sel);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x06 fail,ret=%d\n", ret);
        goto test_err;
    }

    /* set to overall normalize */
    ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, 0x00);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write normalize fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* set frequency high */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set frequency fail,ret=%d\n", ret);
        goto restore_reg;
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto restore_reg;
        }
    }

    /* fir disable */
    ret = fts_test_write_reg(FACTORY_REG_FIR, 0);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set fir fail,ret=%d\n", ret);
        goto restore_reg;
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto restore_reg;
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, 0x01);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set raw type fail,ret=%d\n", ret);
        goto restore_reg;
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x00);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set data sel fail,ret=%d\n", ret);
        goto restore_reg;
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto restore_reg;
        }
    }

    /* get rawdata */
    for (i = 0; i < 3; i++) {
        ret = get_rawdata(panel_differ);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("get rawdata fail\n");
            goto restore_reg;
        }
    }

    for (i = 0; i < tdata->node.node_num; i++) {
        panel_differ[i] = panel_differ[i] / 10;
    }

    /* show test data */
    show_data(panel_differ, false);

    /* compare */
    tmp_result = compare_array(panel_differ,
                               thr->panel_differ_min,
                               thr->panel_differ_max,
                               false);

restore_reg:
    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore normalize fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, data_type);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore raw type fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FIR, fir);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0xFB fail,ret=%d\n", ret);
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, data_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set data sel fail,ret=%d\n", ret);
    }

    if (tdata->func->param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

test_err:
    /* result */
    if (tmp_result) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("====== panel differ test PASS");
        if (tdata->s) seq_printf(tdata->s, "------ panel differ test PASS\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_ERR("====== panel differ test NG");
        if (tdata->s) seq_printf(tdata->s, "------ panel differ test NG\n");
    }

    /* save test data */
    fts_test_save_data("Panel Differ Test", CODE_M_PANELDIFFER_TEST,
                       panel_differ, 0, false, false, *test_result);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int start_test_ft5652(void)
{
    int ret = 0;
    struct fts_test *tdata = fts_ftest;
    struct mc_sc_testitem *test_item = &tdata->ic.mc_sc.u.item;
    bool temp_result = false;
    bool test_result = true;
    u8 state = 0xFF;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_INFO("test item:0x%x", tdata->ic.mc_sc.u.tmp);

    fts_test_read_reg(FACTORY_REG_PARAM_UPDATE_STATE_TOUCH, &state);
    tdata->func->param_update_support = (state == 0xAA);
    FTS_TEST_INFO("Param update:%d", tdata->func->param_update_support);


    /* rawdata test */
    if (test_item->rawdata_test == true) {
        ret = ft5652_rawdata_test(tdata, &temp_result);
        if ((ret < 0) || (temp_result == false)) {
            test_result = false;
        }
    }

    if (test_item->rawdata_uniformity_test == true) {
        ret = ft5652_uniformity_test(tdata, &temp_result);
        if ((ret < 0) || (temp_result == false)) {
            test_result = false;
        }
    }

    /* scap_cb test */
    if (test_item->scap_cb_test == true) {
        ret = ft5652_scap_cb_test(tdata, &temp_result);
        if ((ret < 0) || (temp_result == false)) {
            test_result = false;
        }
    }

    /* scap_rawdata test */
    if (test_item->scap_rawdata_test == true) {
        ret = ft5652_scap_rawdata_test(tdata, &temp_result);
        if ((ret < 0) || (temp_result == false)) {
            test_result = false;
        }
    }

    /* short test */
    if (test_item->short_test == true) {
        ret = ft5652_short_test(tdata, &temp_result);
        if ((ret < 0) || (temp_result == false)) {
            test_result = false;
        }
    }
    /* panel differ test */
    if (test_item->panel_differ_test == true) {
        ret = ft5652_panel_differ_test(tdata, &temp_result);
        if ((ret < 0) || (temp_result == false)) {
            test_result = false;
        }
    }

    /* restore mapping state */
    fts_test_write_reg(FACTORY_REG_NOMAPPING, tdata->mapping);

    FTS_TEST_FUNC_EXIT();
    return test_result;
}

struct test_funcs test_func_ft5652 = {
    .ctype = {0x88},
    .hwtype = IC_HW_MC_SC,
    .key_num_total = 0,
    .mc_sc_short_v2 = true,
    .raw_u16 = true,
    .cb_high_support = true,
    .param_update_support = false,
    .start_test = start_test_ft5652,
};

static int get_cb(int *cb_buf, int byte_num)
{
    int ret = 0;
    int i = 0;
    int read_num = 0;
    int packet_num = 0;
    int packet_remainder = 0;
    int offset = 0;
    u8 *cb = NULL;

    cb = (u8 *)fts_malloc(byte_num * sizeof(u8));
    if (cb == NULL) {
        FTS_TEST_SAVE_ERR("malloc memory for cb buffer fail\n");
        return -ENOMEM;
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

        ret = fts_test_write_reg(FACTORY_REG_MC_SC_CB_ADDR_OFF, offset);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("write cb addr offset fail\n");
            fts_free(cb);
            return ret;
        }

        ret = fts_test_write_reg(FACTORY_REG_MC_SC_CB_H_ADDR_OFF, offset >> 8);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("write cb_h addr offset fail\n");
            fts_free(cb);
            return ret;
        }

        ret = fts_test_read(FACTORY_REG_MC_SC_CB_ADDR, cb + offset, read_num);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("read cb fail\n");
            fts_free(cb);
            return ret;
        }

        offset += read_num;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        cb_buf[i >> 1] = (int)(short)((cb[i] << 8) + cb[i + 1]);
    }

    fts_free(cb);
    return ret;
}

static int get_short_adc(int *adc_buf, int byte_num, u8 mode)
{
    int ret = 0;
    int i = 0;
    u8 short_state = 0;

    FTS_TEST_FUNC_ENTER();
    /* select short test mode & start test */
    ret = fts_test_write_reg(FACTROY_REG_SHORT2_TEST_EN, mode);
    if (ret < 0) {
        FTS_TEST_ERROR("write short test mode fail\n");
        return ret;
    }

    for (i = 0; i < FACTORY_TEST_RETRY; i++) {
        sys_delay(FACTORY_TEST_RETRY_DELAY);

        ret = fts_test_read_reg(FACTROY_REG_SHORT2_TEST_STATE, &short_state);
        if ((ret >= 0) && (TEST_RETVAL_AA == short_state))
            break;
        else
            FTS_TEST_DBG("reg%x=%x,retry:%d", FACTROY_REG_SHORT2_TEST_STATE,
                short_state, i);
    }
    if (i >= FACTORY_TEST_RETRY) {
        FTS_TEST_ERROR("short test timeout, ADC data not OK\n");
        ret = -EIO;
        return ret;
    }

    ret = read_mass_data(FACTORY_REG_SHORT2_ADDR_MC, byte_num, adc_buf);
    if (ret < 0) {
        FTS_TEST_ERROR("get short(adc) data fail\n");
    }

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int fts_test_get_raw_restore_reg(u8 fre, u8 data_sel, u8 data_type) {
    int ret = 0;
    u8 state = 0;
    bool param_update_support = false;

    FTS_TEST_FUNC_ENTER();

    fts_test_read_reg(FACTORY_REG_PARAM_UPDATE_STATE_TOUCH, &state);
    param_update_support = (state == 0xAA);

    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_ERROR("restore 0x0A fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, data_type);
    if (ret < 0) {
        FTS_TEST_ERROR("restore 0x5B fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, data_sel);
    if (ret < 0) {
        FTS_TEST_ERROR("restore 0x06 fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }
    FTS_TEST_FUNC_EXIT();
    return ret;
}

int fts_test_get_raw(int *raw, u8 tx, u8 rx)
{
    int ret = 0;
    int i = 0;
    int times = 0;
    int node_num = tx * rx;
    u8 fre = 0;
    u8 data_sel = 0;
    u8 data_type = 0;
    u8 val = 0;
    u8 state = 0;
    bool param_update_support = false;

    FTS_TEST_INFO("====== Test Item: rawdata test start\n");

    fts_test_read_reg(FACTORY_REG_PARAM_UPDATE_STATE_TOUCH, &state);
    param_update_support = (state == 0xAA);
    FTS_TEST_INFO("Param update:%d", param_update_support);

    /* save origin value */
    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_ERROR("read 0x0A fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_TYPE, &data_type);
    if (ret) {
        FTS_TEST_ERROR("read 0x5B fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &data_sel);
    if (ret) {
        FTS_TEST_ERROR("read 0x06 fail,ret=%d\n", ret);
        return ret;
    }

    /* set frequency high */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
    if (ret < 0) {
        FTS_TEST_ERROR("set frequency fail,ret=%d\n", ret);
        fts_test_get_raw_restore_reg(fre, data_sel, data_type);
        return ret;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            fts_test_get_raw_restore_reg(fre, data_sel, data_type);
            return ret;
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, 0x01);
    if (ret < 0) {
        FTS_TEST_ERROR("set raw type fail,ret=%d\n", ret);
        fts_test_get_raw_restore_reg(fre, data_sel, data_type);
        return ret;
    }

    /* select rawdata */
    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x00);
    if (ret < 0) {
        FTS_TEST_ERROR("set fir fail,ret=%d\n", ret);
        fts_test_get_raw_restore_reg(fre, data_sel, data_type);
        return ret;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            fts_test_get_raw_restore_reg(fre, data_sel, data_type);
            return ret;
        }
    }

    /*********************GET RAWDATA*********************/
    for (i = 0; i < 3; i++) {
        FTS_TEST_INFO("get rawdata,i=%d", i);
        ret = fts_test_write_reg(DIVIDE_MODE_ADDR, 0xC0);
        if (ret < 0) {
            FTS_TEST_ERROR("write start scan mode fail\n");
            continue;
        }

        while (times++ < FACTORY_TEST_RETRY) {
            sys_delay(FACTORY_TEST_DELAY);

            ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &val);
            if ((ret >= 0) && (val == 0x40)) {
                break;
            } else {
                FTS_TEST_DBG("reg%x=%x,retry:%d", DIVIDE_MODE_ADDR, val, times);
            }
        }

        if (times >= FACTORY_TEST_RETRY) {
            FTS_TEST_ERROR("scan timeout\n");
            continue;
        }

        ret = fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAA);
        if (ret < 0) {
            FTS_TEST_ERROR("write line/start addr fail\n");
            continue;
        }

        ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR_MC_SC, (node_num * 2),
              raw);
    }
    if (ret < 0) {
        FTS_TEST_ERROR("get rawdata fail,ret=%d\n", ret);
        fts_test_get_raw_restore_reg(fre, data_sel, data_type);
        return ret;
    }

    fts_test_get_raw_restore_reg(fre, data_sel, data_type);
    FTS_TEST_INFO("====== Test Item: rawdata test end\n");
    return ret;
}

int fts_test_get_baseline(int *raw,int *base_raw, u8 tx, u8 rx)
{
    int ret = 0;
    int i = 0;
    int times = 0;
    int node_num = tx * rx;
    u8 fre = 0;
    u8 data_sel = 0;
    u8 data_type = 0;
    u8 val = 0;
    u8 state = 0;
    bool param_update_support = false;

    FTS_TEST_INFO("====== Test Item: baseline test start\n");

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_ERROR("failed to enter factory mode,ret=%d\n", ret);
        goto test_err;
    }

    fts_test_read_reg(FACTORY_REG_PARAM_UPDATE_STATE_TOUCH, &state);
    param_update_support = (0xAA == state);
    FTS_TEST_INFO("Param update:%d", param_update_support);

    /* save origin value */
    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_ERROR("read 0x0A fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_TYPE, &data_type);
    if (ret) {
        FTS_ERROR("read 0x5B fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &data_sel);
    if (ret) {
        FTS_TEST_ERROR("read 0x06 error,ret=%d\n", ret);
        goto test_err;
    }

    /* set frequency high */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
    if (ret < 0) {
        FTS_TEST_ERROR("set frequency fail,ret=%d\n", ret);
        goto restore_reg;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto restore_reg;
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, 0x01);
    if (ret < 0) {
        FTS_TEST_ERROR("set raw type fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* select rawdata */
    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x00);
    if (ret < 0) {
        FTS_TEST_ERROR("set fir fail,ret=%d\n", ret);
        goto restore_reg;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto restore_reg;
        }
    }

    /*********************GET RAWDATA*********************/
    FTS_TEST_INFO("get rawdata,i=%d", i);
    ret = fts_test_write_reg(DIVIDE_MODE_ADDR, 0xC0);
    if (ret < 0) {
        FTS_TEST_ERROR("write start scan mode fail\n");
    }

    while (times++ < FACTORY_TEST_RETRY) {
        sys_delay(FACTORY_TEST_DELAY);
        ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &val);
        if ((ret >= 0) && (val == 0x40)) {
            break;
        } else {
            FTS_TEST_DBG("reg%x=%x,retry:%d", DIVIDE_MODE_ADDR, val, times);
        }
    }

    if (times >= FACTORY_TEST_RETRY) {
        FTS_TEST_ERROR("scan timeout\n");
    }

    ret = fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAA);
    if (ret < 0) {
        FTS_TEST_ERROR("write line/start addr fail\n");
    }

    ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR_MC_SC, (node_num * 2),raw);

    if (ret < 0) {
        FTS_TEST_ERROR("get rawdata fail,ret=%d\n", ret);
        goto restore_reg;
    }

    /* select rawdata */
    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x01);
    if (ret < 0) {
        goto restore_reg;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            goto restore_reg;
        }
    }

    /*********************GET DATA*********************/
    ret = fts_test_write_reg(DIVIDE_MODE_ADDR, 0xC0);
    while (times++ < FACTORY_TEST_RETRY) {
        sys_delay(FACTORY_TEST_DELAY);

        ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &val);
        if ((ret >= 0) && (val == 0x40)) {
            break;
        } else {
            FTS_TEST_DBG("reg%x=%x,retry:%d", DIVIDE_MODE_ADDR, val, times);
        }
    }

    if (times >= FACTORY_TEST_RETRY) {
        FTS_TEST_ERROR("scan timeout\n");
    }

    ret = fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAA);
    if (ret < 0) {
        FTS_TEST_ERROR("write line/start addr fail\n");
    }
    ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR_MC_SC, (node_num * 2), base_raw);
    if (ret < 0) {
        FTS_TEST_ERROR("get rawdata fail,ret=%d\n", ret);
        goto restore_reg;
    }

restore_reg:
    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_ERROR("restore 0x0A fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, data_type);
    if (ret < 0) {
        FTS_TEST_ERROR("set raw type fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, data_sel);
    if (ret < 0) {
        FTS_TEST_ERROR("restore 0x06 fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

test_err:
    FTS_TEST_INFO("====== Test Item: baseline test end\n");
    return ret;
}

int fts_test_get_strength(u8 *base_raw, u16 base_raw_size)
{
    int ret = 0;
    u8 id_cmd[1] = {0};

    FTS_TEST_INFO("====== Test Item: strength test start\n");
    id_cmd[0] = FTS_CMD_READ_TOUCH_DATA;
    sys_delay(500);
    ret = fts_read(id_cmd, 1, base_raw, base_raw_size);
    if (ret < 0) {
        FTS_TEST_ERROR("get strength fail,ret=%d\n", ret);
    }

    FTS_TEST_INFO("====== Test Item: strength test end\n");
    return ret;
}

int fts_test_get_uniformity_data(int *rawdata_linearity, u8 tx, u8 rx)
{
    int ret = 0;
    int row = 0;
    int col = 1;
    int i = 0;
    int deviation = 0;
    int max = 0;
    int *raw = NULL;
    int *rl_tmp = NULL;
    int offset = 0;
    int offset2 = 0;
    int node_num = tx * rx;
    int times = 0;
    u8 fre = 0;
    u8 data_sel = 0;
    u8 data_type = 0;
    u8 val = 0;
    u8 state = 0;
    bool param_update_support = false;

    FTS_TEST_INFO("====== Test Item: rawdata unfiormity test start\n");

    raw = fts_malloc(node_num * sizeof(int));
    if (!raw) {
        FTS_TEST_ERROR("raw buffer malloc fail");
        return -ENOMEM;
    }

    fts_test_read_reg(FACTORY_REG_PARAM_UPDATE_STATE_TOUCH, &state);
    param_update_support = (state == 0xAA);
    FTS_TEST_INFO("Param update:%d", param_update_support);

    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_ERROR("read FACTORY_REG_FRE_LIST fail,ret=%d\n", ret);
        fts_free(raw);
        return -ENOMEM;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_TYPE, &data_type);
    if (ret) {
        FTS_TEST_ERROR("read FACTORY_REG_DATA_TYPE fail,ret=%d\n", ret);
        fts_free(raw);
        return -ENOMEM;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &data_sel);
    if (ret) {
        FTS_TEST_ERROR("read FACTORY_REG_DATA_SELECT fail,ret=%d\n", ret);
        fts_free(raw);
        return -ENOMEM;
    }

    /* set frequency high */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
    if (ret < 0) {
        FTS_TEST_ERROR("set frequency fail,ret=%d\n", ret);
        fts_free(raw);
        goto exit;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            fts_free(raw);
            goto exit;
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, 0x01);
    if (ret < 0) {
        FTS_TEST_ERROR("set raw type fail,ret=%d\n", ret);
        goto exit;
    }

    /* select rawdata */
    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x00);
    if (ret < 0) {
        FTS_TEST_ERROR("set data select fail,ret=%d\n", ret);
        goto exit;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto exit;
        }
    }

    for (i = 0; i < 3; i++) {
        FTS_TEST_INFO("get rawdata,i=%d", i);
        ret = fts_test_write_reg(DIVIDE_MODE_ADDR, 0xC0);
        if (ret < 0) {
            FTS_TEST_ERROR("write start scan mode fail\n");
            continue;
        }

        while (times++ < FACTORY_TEST_RETRY) {
            sys_delay(FACTORY_TEST_DELAY);

            ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &val);
            if ((ret >= 0) && (val == 0x40)) {
                break;
            } else {
                FTS_TEST_DBG("reg%x=%x,retry:%d", DIVIDE_MODE_ADDR, val, times);
            }
        }

        if (times >= FACTORY_TEST_RETRY) {
            FTS_TEST_ERROR("scan timeout\n");
            continue;
        }

        ret = fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAA);
        if (ret < 0) {
            FTS_TEST_ERROR("write line/start addr fail\n");
            continue;
        }

        ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR_MC_SC, (node_num * 2), raw);
    }
    if (ret < 0) {
        FTS_TEST_ERROR("get rawdata fail,ret=%d\n", ret);
        goto exit;
    }

    FTS_TEST_INFO("Check Tx Linearity\n");
    rl_tmp = rawdata_linearity;
    for (row = 0; row < tx; row++) {
        for (col = 1; col <  rx; col++) {
            offset = row * rx + col;
            offset2 = row * rx + col - 1;
            deviation = abs( raw[offset] - raw[offset2]);
            max = max(raw[offset], raw[offset2]);
            max = max ? max : 1;
            rl_tmp[offset] = 100 * deviation / max;
        }
    }

    FTS_TEST_INFO("Check Rx Linearity\n");
    rl_tmp = rawdata_linearity + node_num;
    for (row = 1; row < tx; row++) {
        for (col = 0; col < rx; col++) {
            offset = row * rx + col;
            offset2 = (row - 1) * rx + col;
            deviation = abs(raw[offset] - raw[offset2]);
            max = max(raw[offset], raw[offset2]);
            max = max ? max : 1;
            rl_tmp[offset] = 100 * deviation / max;
        }
    }

exit:
    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, data_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x06 fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, data_type);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set raw type fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    fts_free(raw);
    FTS_TEST_INFO("====== Test Item: rawdata unfiormity test end\n");
    return ret;
}

int fts_test_get_scap_cb(int *scap_cb, u8 tx, u8 rx, int *fwcheck)
{
    int ret = 0;
    u8 wc_sel = 0;
    u8 sc_mode = 0;
    u8 hc_sel = 0;
    u8 hov_high = 0;
    int node_num = (tx + rx);
    bool fw_wp_check = false;
    bool tx_check = false;
    bool rx_check = false;
    int *scb_tmp = NULL;
    u8 state = 0;
    bool param_update_support = false;

    FTS_TEST_INFO("====== Test Item: Scap CB Test start\n");
    *fwcheck = 0;

    fts_test_read_reg(FACTORY_REG_PARAM_UPDATE_STATE_TOUCH, &state);
    param_update_support = (state == 0xAA);
    FTS_TEST_INFO("Param update:%d", param_update_support);

    /* get waterproof channel select */
    ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);
    if (ret < 0) {
        FTS_TEST_ERROR("read water_channel_sel fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_MC_SC_MODE, &sc_mode);
    if (ret < 0) {
        FTS_TEST_ERROR("read sc_mode fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_HC_SEL, &hc_sel);
    if (ret < 0) {
        FTS_TEST_ERROR("read high_channel_sel fail,ret=%d\n", ret);
        return ret;
    }

    /* water proof on check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);
    if (fw_wp_check) {
        scb_tmp = scap_cb;
        ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, WATER_PROOF_ON);
        if (ret < 0) {
            FTS_TEST_ERROR("set mc_sc mode fail\n");
            goto exit;
        }

        if (param_update_support) {
            ret = wait_state_update(TEST_RETVAL_AA);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("wait state update fail\n");
                goto exit;
            }
        }

        ret = get_cb(scb_tmp, node_num * 2);
        if (ret < 0) {
            FTS_TEST_ERROR("get sc cb fail\n");
            goto exit;
        }

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_RX);
        *fwcheck |= (rx_check ? 0x01 : 0x00);
        *fwcheck |= (tx_check ? 0x02 : 0x00);
    }

    /* water proof off check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);
    if (fw_wp_check) {
        scb_tmp = scap_cb + node_num;
        ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, WATER_PROOF_OFF);
        if (ret < 0) {
            FTS_TEST_ERROR("set mc_sc mode fail\n");
            goto exit;
        }

        if (param_update_support) {
            ret = wait_state_update(TEST_RETVAL_AA);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("wait state update fail\n");
                goto exit;
            }
        }

        ret = get_cb(scb_tmp, node_num * 2);
        if (ret < 0) {
            FTS_TEST_ERROR("get sc cb fail\n");
            goto exit;
        }

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_RX);
        *fwcheck |= (rx_check ? 0x04 : 0x00);
        *fwcheck |= (tx_check ? 0x08 : 0x00);
    }

    /*high mode*/
    hov_high = (hc_sel & 0x03);
    if (hov_high) {
        scb_tmp = scap_cb + node_num * 2;
        ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, HIGH_SENSITIVITY);
        if (ret < 0) {
            FTS_TEST_ERROR("set mc_sc mode fail\n");
            goto exit;
        }

        if (param_update_support) {
            ret = wait_state_update(TEST_RETVAL_AA);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("wait state update fail\n");
                goto exit;
            }
        }

        ret = get_cb(scb_tmp, node_num * 2);
        if (ret < 0) {
            FTS_TEST_ERROR("get sc cb fail\n");
            goto exit;
        }

        /* compare */
        tx_check = ((hov_high == 1) || (hov_high == 3));
        rx_check = ((hov_high == 2) || (hov_high == 3));
        *fwcheck |= (rx_check ? 0x10 : 0x00);
        *fwcheck |= (tx_check ? 0x20 : 0x00);
    }

exit:
    ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, sc_mode);/* set the origin value */
    if (ret) {
        FTS_TEST_ERROR("restore sc mode fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    FTS_TEST_INFO("====== Test Item: Scap CB Test end\n");
    return ret;
}

int fts_test_get_scap_raw(int *scap_raw, u8 tx, u8 rx, int *fwcheck)
{
    int ret = 0;
    int i = 0;
    int times = 0;
    int node_num = tx + rx;
    bool fw_wp_check = false;
    bool tx_check = false;
    bool rx_check = false;
    int *srawdata_tmp = NULL;
    u8 wc_sel = 0;
    u8 hc_sel = 0;
    u8 hov_high = 0;
    u8 data_type = 0;
    u8 val = 0;

    FTS_TEST_INFO("====== Test Item: Scap Rawdata Test start\n");
    *fwcheck = 0;

    /* get waterproof channel select */
    ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);
    if (ret < 0) {
        FTS_TEST_ERROR("read water_channel_sel fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_HC_SEL, &hc_sel);
    if (ret < 0) {
        FTS_TEST_ERROR("read high_channel_sel fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_TYPE, &data_type);
    if (ret) {
        FTS_TEST_ERROR("read 0x5B fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, 0x01);
    if (ret < 0) {
        FTS_TEST_ERROR("set raw type fail,ret=%d\n", ret);
        goto exit;
    }

    /* scan rawdata 2 times*/
    for (i = 0; i < 2; i++) {
        FTS_TEST_INFO("get rawdata,i=%d", i);
        ret = fts_test_write_reg(DIVIDE_MODE_ADDR, 0xC0);
        if (ret < 0) {
            FTS_TEST_ERROR("write start scan mode fail\n");
            continue;
        }

        while (times++ < FACTORY_TEST_RETRY) {
            sys_delay(FACTORY_TEST_DELAY);

            ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &val);
            if ((ret >= 0) && (val == 0x40)) {
                break;
            } else {
                FTS_TEST_DBG("reg%x=%x,retry:%d", DIVIDE_MODE_ADDR, val, times);
            }
        }

        if (times >= FACTORY_TEST_RETRY) {
            FTS_TEST_ERROR("scan timeout\n");
            continue;
        }
    }
    if (ret < 0) {
        FTS_TEST_ERROR("scan scap rawdata fail\n");
        goto exit;
    }

    /* water proof on check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);
    if (fw_wp_check) {
        srawdata_tmp = scap_raw;
        ret = fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAC);
        if (ret < 0) {
            FTS_TEST_ERROR("write line/start addr fail\n");
            goto exit;
        }

        ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR_MC_SC, (node_num * 2), srawdata_tmp);
        if (ret < 0) {
            FTS_TEST_ERROR("get scap(WP_ON) rawdata fail\n");
            goto exit;
        }

        tx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_RX);
        *fwcheck |= (rx_check ? 0x01 : 0x00);
        *fwcheck |= (tx_check ? 0x02 : 0x00);
    }

    /* water proof off check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);
    if (fw_wp_check) {
        srawdata_tmp = scap_raw + node_num;
        ret = fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAB);
        if (ret < 0) {
            FTS_TEST_ERROR("write line/start addr fail\n");
            goto exit;
        }

        ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR_MC_SC, (node_num * 2), srawdata_tmp);
        if (ret < 0) {
            FTS_TEST_ERROR("get scap(WP_OFF) rawdata fail\n");
            goto exit;
        }

        tx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_RX);
        *fwcheck |= (rx_check ? 0x04 : 0x00);
        *fwcheck |= (tx_check ? 0x08 : 0x00);
    }

    /*high mode*/
    hov_high = (hc_sel & 0x03);
    if (hov_high) {
        srawdata_tmp = scap_raw + node_num * 2;
        ret = fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xA0);
        if (ret < 0) {
            FTS_TEST_ERROR("write line/start addr fail\n");
            goto exit;
        }

        ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR_MC_SC, (node_num * 2), srawdata_tmp);
        if (ret < 0) {
            FTS_TEST_ERROR("get scap(HIGH) rawdata fail\n");
            goto exit;
        }

        tx_check = ((hov_high == 1) || (hov_high == 3));
        rx_check = ((hov_high == 2) || (hov_high == 3));
        *fwcheck |= (rx_check ? 0x10 : 0x00);
        *fwcheck |= (tx_check ? 0x20 : 0x00);
    }

exit:
    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, data_type);
    if (ret < 0) {
        FTS_TEST_ERROR("set raw type fail,ret=%d\n", ret);
    }

    FTS_TEST_INFO("====== Test Item: Scap Rawdata Test end\n");
    return ret;
}

static int fts_test_get_short_restore_reg(u8 res_level) {
    int ret = 0;

    FTS_TEST_FUNC_ENTER();
    ret = fts_test_write_reg(FACTROY_REG_SHORT2_RES_LEVEL, res_level);
    if (ret < 0) {
        FTS_TEST_ERROR("restore FACTROY_REG_SHORT2_RES_LEVEL level fails");
    }

    FTS_TEST_FUNC_EXIT();
    return ret;
}

int fts_test_get_short(int *short_data, u8 tx, u8 rx)
{
    int ret = 0;
    int i = 0;
    int ch_num = (tx + rx);
    int offset = 0;
    int code = 0;
    int denominator = 0;
    int numerator = 0;
    u8 res_level = 0;

    FTS_TEST_INFO("====== Test Item: Short Test start");

    ret = fts_test_read_reg(FACTROY_REG_SHORT2_RES_LEVEL, &res_level);
    if (ret < 0) {
        FTS_TEST_ERROR("read short level fails\n");
        return ret;
    }

    /* get offset = readdata - 1024 */
    ret = get_short_adc(&offset, 1 * 2, FACTROY_REG_SHORT2_OFFSET);
    if (ret < 0) {
        FTS_TEST_ERROR("get weak short data fail,ret:%d\n", ret);
        fts_test_get_short_restore_reg(res_level);
        return ret;
    }
    offset -= 1024;
    FTS_TEST_INFO("short offset:%d", offset);

    /* get short resistance and exceptional channel */
    /* choose resistor_level */
    ret = fts_test_write_reg(FACTROY_REG_SHORT2_RES_LEVEL, 1);
    if (ret < 0) {
        FTS_TEST_ERROR("write short resistor level fail\n");
        fts_test_get_short_restore_reg(res_level);
        return ret;
    }

    /* get adc data */
    ret = get_short_adc(short_data, ch_num * 2, FACTROY_REG_SHORT2_CA);
    if (ret < 0) {
        FTS_TEST_ERROR("get weak short data fail,ret:%d\n", ret);
        fts_test_get_short_restore_reg(res_level);
        return ret;
    }

    for (i = 0; i < ch_num; i++) {
        code = short_data[i];
        denominator = 1407 - code + offset;
        if (denominator == 0) {
            short_data[i] = 2000;
        } else {
            numerator = (code - offset + 395) * 112;
            short_data[i] = fts_abs(numerator / denominator - 3);
        }
    }

    ret = fts_test_get_short_restore_reg(res_level);

    FTS_TEST_INFO("====== Test Item: Short Test end");
    return ret;
}

int fts_test_get_noise(int *noise, u8 tx, u8 rx)
{
    int ret = 0;
    int node_num = (tx * rx);
    u8 fre = 0;
    u8 data_sel = 0;
    u16 noise_frame = 20;
    u8 noise_mode = 0;
    u8 state = 0;
    bool param_update_support = false;

    FTS_TEST_INFO("====== Test Item: Noise test start");

    fts_test_read_reg(FACTORY_REG_PARAM_UPDATE_STATE_TOUCH, &state);
    param_update_support = (state == 0xAA);
    FTS_TEST_INFO("Param update:%d", param_update_support);

    ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &data_sel);
    if (ret) {
        FTS_TEST_SAVE_ERR("read FACTORY_REG_DATA_SELECT error,ret=%d\n", ret);
        return ret;
    }

    /* save origin value */
    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_SAVE_ERR("read FACTORY_REG_FRE_LIST fail,ret=%d\n", ret);
        return ret;
    }

    /* select rawdata */
    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x01);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set fir fail,ret=%d\n", ret);
        goto exit;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto exit;
        }
    }

    /* set frequency high */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x0);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set frequency fail,ret=%d\n", ret);
        goto exit;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto exit;
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_MAXDIFF_EN, 0x01);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write 0x1A fail,ret=%d\n", ret);
        goto exit;
    }

    ret = fts_test_write_reg(FACTORY_REG_FRAME_NUM_H, (noise_frame >> 8));
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write 0x1C fail,ret=%d\n", ret);
        goto exit;
    }

    ret = fts_test_write_reg(FACTORY_REG_FRAME_NUM_L, noise_frame);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write 0x1D fail,ret=%d\n", ret);
        goto exit;
    }

    noise_mode = 1;
    FTS_TEST_INFO("noise_mode = %x\n", noise_mode);
    ret = fts_test_write_reg(FACTORY_REG_MAXDIFF_FLAG, noise_mode);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write 0x1B fail,ret=%d\n", ret);
        goto exit;
    }

    ret = ft5652_start_scan(noise_frame);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("ft5652_start_scan fail,ret=%d\n", ret);
        goto exit;
    }

    ret = fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAA);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write 0x01 fail,ret=%d\n", ret);
        goto exit;
    }

    ret = read_mass_data(FACTORY_REG_NOISE_ADDR, (node_num * 2), noise);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read 0xCE fail\n");
        return ret;
    }

exit:
    ret = fts_test_write_reg(FACTORY_REG_MAXDIFF_FLAG, 0x0);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x1B fail,ret=%d\n", ret);
    }

    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, data_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x06 fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    FTS_TEST_INFO("====== Test Item: Noise test end");
    return ret;
}

int fts_test_get_panel_differ(int *panel_differ, u8 tx, u8 rx)
{
    int ret = 0;
    int i = 0;
    int node_num = tx * rx;
    int times = 0;
    u8 val = 0;
    u8 fre = 0;
    u8 fir = 0;
    u8 normalize = 0;
    u8 data_type = 0;
    u8 data_sel = 0;
    u8 state = 0;
    bool param_update_support = false;

    FTS_TEST_INFO("====== Test Item: Panel Differ Test start");

    fts_test_read_reg(FACTORY_REG_PARAM_UPDATE_STATE_TOUCH, &state);
    param_update_support = (state == 0xAA);
    FTS_TEST_INFO("Param update:%d", param_update_support);

    /* save origin value */
    ret = fts_test_read_reg(FACTORY_REG_NORMALIZE, &normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read normalize fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_TYPE, &data_type);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x5B fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_FIR, &fir);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0xFB fail,ret=%d\n", ret);
        return ret;
    }

    ret = fts_test_read_reg(FACTORY_REG_DATA_SELECT, &data_sel);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x06 fail,ret=%d\n", ret);
        return ret;
    }

    /* set to overall normalize */
    ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, 0x00);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write normalize fail,ret=%d\n", ret);
        goto exit;
    }

    /* set frequency high */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set frequency fail,ret=%d\n", ret);
        goto exit;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto exit;
        }
    }

    /* fir disable */
    ret = fts_test_write_reg(FACTORY_REG_FIR, 0);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set fir fail,ret=%d\n", ret);
        goto exit;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto exit;
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, 0x01);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set raw type fail,ret=%d\n", ret);
        goto exit;
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, 0x00);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set data sel fail,ret=%d\n", ret);
        goto exit;
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
            goto exit;
        }
    }

    for (i = 0; i < 3; i++) {
        FTS_TEST_INFO("get rawdata,i=%d", i);
        ret = fts_test_write_reg(DIVIDE_MODE_ADDR, 0xC0);
        if (ret < 0) {
            FTS_TEST_ERROR("write start scan mode fail\n");
            continue;
        }

        while (times++ < FACTORY_TEST_RETRY) {
            sys_delay(FACTORY_TEST_DELAY);

            ret = fts_test_read_reg(DIVIDE_MODE_ADDR, &val);
            if ((ret >= 0) && (val == 0x40)) {
                break;
            } else {
                FTS_TEST_DBG("reg%x=%x,retry:%d", DIVIDE_MODE_ADDR, val, times);
            }
        }

        if (times >= FACTORY_TEST_RETRY) {
            FTS_TEST_ERROR("scan timeout\n");
            continue;
        }

        ret = fts_test_write_reg(FACTORY_REG_LINE_ADDR, 0xAA);
        if (ret < 0) {
            FTS_TEST_ERROR("write line/start addr fail\n");
            continue;
        }

        ret = read_mass_data(FACTORY_REG_RAWDATA_ADDR_MC_SC, (node_num * 2), panel_differ);
    }

    if (ret < 0) {
        FTS_TEST_ERROR("get panel_differ fail,ret=%d\n", ret);
        goto exit;
    }

    for (i = 0; i < node_num; i++) {
        panel_differ[i] = panel_differ[i] / 10;
    }

exit:
    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore normalize fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_TYPE, data_type);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set raw type fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FIR, fir);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0xFB fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    ret = fts_test_write_reg(FACTORY_REG_DATA_SELECT, data_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set data sel fail,ret=%d\n", ret);
    }

    if (param_update_support) {
        ret = wait_state_update(TEST_RETVAL_AA);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("wait state update fail\n");
        }
    }

    FTS_TEST_INFO("====== Test Item: Panel Differ Test end");
    return ret;
}
