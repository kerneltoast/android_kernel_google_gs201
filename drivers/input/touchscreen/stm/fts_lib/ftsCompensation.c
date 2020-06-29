/*
  *
  **************************************************************************
  **                        STMicroelectronics				**
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *               FTS functions for getting Initialization Data		 *
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */
/*!
  * \file ftsCompensation.c
  * \brief Contains all the function to work with Initialization Data
  */

#include "ftsCompensation.h"
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsFrame.h"
#include "ftsHardware.h"
#include "ftsIO.h"
#include "ftsSoftware.h"
#include "ftsTime.h"
#include "ftsTool.h"


#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <stdarg.h>
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/ctype.h>


/**
  * Request to the FW to load the specified Initialization Data into HDM
  * @param type type of Initialization data to load @link load_opt Load Host
  * Data Option @endlink
  * @return OK if success or an error code which specify the type of error
  */
int requestHDMDownload(u8 type)
{
	int ret = ERROR_OP_NOT_ALLOW;
	int retry = 0;

	pr_info("%s: Requesting HDM download...\n", __func__);
	while (retry < RETRY_FW_HDM_DOWNLOAD) {
		ret = writeSysCmd(SYS_CMD_LOAD_DATA,  &type, 1);
		/* send request to load in memory the Compensation Data */
		if (ret < OK) {
			pr_err("%s: failed at %d attemp!\n",
				 __func__, retry + 1);
			retry += 1;
		} else {
			pr_info("%s: Request HDM Download FINISHED!\n",
				__func__);
			return OK;
		}
	}

	ret |= ERROR_REQU_HDM_DOWNLOAD;
	pr_err("%s: Requesting HDM Download... ERROR %08X\n",
		 __func__, ret);

	return ret;
}


/**
  * Read HDM Header and check that the type loaded match
  * with the one previously requested
  * @param type type of Initialization data requested @link load_opt Load Host
  * Data Option @endlink
  * @param header pointer to DataHeader variable which will contain the header
  * @param address pointer to a variable which will contain the updated address
  * to the next data
  * @return OK if success or an error code which specify the type of error
  */
int readHDMHeader(u8 type, DataHeader *header, u64 *address)
{
	u64 offset = ADDR_FRAMEBUFFER;
	u8 data[HDM_DATA_HEADER];
	int ret;

	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, offset, data,
				HDM_DATA_HEADER, DUMMY_FRAMEBUFFER);
	if (ret < OK) {	/* i2c function have already a retry mechanism */
		pr_err("%s: error while reading HDM data header ERROR %08X\n",
			__func__, ret);
		return ret;
	}

	pr_info("Read HDM Data Header done!\n");

	if (data[0] != HEADER_SIGNATURE) {
		pr_err("%s: The Header Signature was wrong! %02X != %02X ERROR %08X\n",
			__func__, data[0], HEADER_SIGNATURE,
			ERROR_WRONG_DATA_SIGN);
		return ERROR_WRONG_DATA_SIGN;
	}


	if (data[1] != type) {
		pr_err("%s: Wrong type found! %02X!=%02X ERROR %08X\n",
			__func__, data[1], type, ERROR_DIFF_DATA_TYPE);
		return ERROR_DIFF_DATA_TYPE;
	}

	pr_info("Type = %02X of Compensation data OK!\n", type);

	header->type = type;

	*address = offset + HDM_DATA_HEADER;

	return OK;
}


/**
  * Read MS Global Initialization data from the buffer such as Cx1
  * @param address pointer to a variable which contain the address from where
  * to read the data and will contain the updated address to the next data
  * @param global pointer to MutualSenseData variable which will contain the MS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
static int readMutualSenseGlobalData(u64 *address, MutualSenseData *global)
{
	u8 data[COMP_DATA_GLOBAL];
	int ret;

	pr_info("Address for Global data= %llx\n", *address);

	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, *address, data,
				COMP_DATA_GLOBAL, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("%s: error while reading info data ERROR %08X\n",
			__func__, ret);
		return ret;
	}
	pr_info("Global data Read !\n");

	global->header.force_node = data[0];
	global->header.sense_node = data[1];
	global->cx1 = data[2];
	/* all other bytes are reserved atm */

	pr_info("force_len = %d sense_len = %d CX1 = %d\n",
		 global->header.force_node, global->header.sense_node,
		 global->cx1);

	*address += COMP_DATA_GLOBAL;
	return OK;
}


/**
  * Read MS Initialization data for each node from the buffer
  * @param address a variable which contain the address from where to read the
  * data
  * @param node pointer to MutualSenseData variable which will contain the MS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
static int readMutualSenseNodeData(u64 address, MutualSenseData *node)
{
	int ret;
	int size = node->header.force_node * node->header.sense_node;

	pr_info("Address for Node data = %llx\n", address);

	node->node_data = (i8 *)kmalloc(size * (sizeof(i8)), GFP_KERNEL);

	if (node->node_data == NULL) {
		pr_err("%s: can not allocate node_data... ERROR %08X",
			__func__, ERROR_ALLOC);
		return ERROR_ALLOC;
	}

	pr_info("Node Data to read %d bytes\n", size);
	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, address,
				node->node_data, size, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("%s: error while reading node data ERROR %08X\n",
			__func__, ret);
		kfree(node->node_data);
		return ret;
	}
	node->node_data_size = size;

	pr_info("Read node data OK!\n");

	return size;
}

/**
  * Perform all the steps to read the necessary info for MS Initialization data
  * from the buffer and store it in a MutualSenseData variable
  * @param type type of MS Initialization data to read @link load_opt Load Host
  * Data Option @endlink
  * @param data pointer to MutualSenseData variable which will contain the MS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
int readMutualSenseCompensationData(u8 type, MutualSenseData *data)
{
	int ret;
	u64 address;

	data->node_data = NULL;

	if (!(type == LOAD_CX_MS_TOUCH || type == LOAD_CX_MS_LOW_POWER ||
	      type == LOAD_CX_MS_KEY || type == LOAD_CX_MS_FORCE)) {
		pr_err("%s: Choose a MS type of compensation data ERROR %08X\n",
			__func__, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = requestHDMDownload(type);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ret);
		return ret;
	}

	ret = readHDMHeader(type, &(data->header), &address);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_HDM_DATA_HEADER);
		return ret | ERROR_HDM_DATA_HEADER;
	}

	ret = readMutualSenseGlobalData(&address, data);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_COMP_DATA_GLOBAL);
		return ret | ERROR_COMP_DATA_GLOBAL;
	}

	ret = readMutualSenseNodeData(address, data);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_COMP_DATA_NODE);
		return ret | ERROR_COMP_DATA_NODE;
	}

	return OK;
}

/**
  * Read SS Global Initialization data from the buffer such as Ix1/Cx1 for force
  * and sense
  * @param address pointer to a variable which contain the address from where
  * to read the data and will contain the updated address to the next data
  * @param global pointer to MutualSenseData variable which will contain the SS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
static int readSelfSenseGlobalData(u64 *address, SelfSenseData *global)
{
	int ret;
	u8 data[COMP_DATA_GLOBAL];

	pr_info("Address for Global data= %llx\n", *address);
	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, *address, data,
				COMP_DATA_GLOBAL, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("%s: error while reading the data... ERROR %08X\n",
			__func__, ret);
		return ret;
	}

	pr_info("Global data Read !\n");


	global->header.force_node = data[0];
	global->header.sense_node = data[1];
	global->f_ix1 = data[2];
	global->s_ix1 = data[3];
	global->f_cx1 = (i8)data[4];
	global->s_cx1 = (i8)data[5];
	global->f_max_n = data[6];
	global->s_max_n = data[7];
	global->f_ix0 = data[8];
	global->s_ix0 = data[9];

	pr_info("force_len = %d sense_len = %d  f_ix1 = %d   s_ix1 = %d   f_cx1 = %d   s_cx1 = %d\n",
		global->header.force_node, global->header.sense_node,
		global->f_ix1, global->s_ix1, global->f_cx1, global->s_cx1);
	pr_info("max_n = %d   s_max_n = %d f_ix0 = %d  s_ix0 = %d\n",
		global->f_max_n, global->s_max_n, global->f_ix0,
		global->s_ix0);


	*address += COMP_DATA_GLOBAL;

	return OK;
}

/**
  * Read SS Initialization data for each node of force and sense channels from
  * the buffer
  * @param address a variable which contain the address from where to read the
  * data
  * @param node pointer to SelfSenseData variable which will contain the SS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
static int readSelfSenseNodeData(u64 address, SelfSenseData *node)
{
	int size = node->header.force_node * 2 + node->header.sense_node * 2;
	u8 *data;
	int ret;

	if (size <= 0) {
		pr_err("%s: Invalid  SS data length!\n", __func__);
		return ERROR_OP_NOT_ALLOW;
	}

	data = kzalloc(size * sizeof(u8), GFP_KERNEL);
	if (data == NULL)
		return ERROR_ALLOC;

	node->ix2_fm = (u8 *)kmalloc(node->header.force_node * (sizeof(u8)),
				     GFP_KERNEL);
	if (node->ix2_fm == NULL) {
		pr_err("%s: can not allocate memory for ix2_fm... ERROR %08X",
			__func__, ERROR_ALLOC);
		kfree(data);
		return ERROR_ALLOC;
	}

	node->cx2_fm = (i8 *)kmalloc(node->header.force_node * (sizeof(i8)),
				     GFP_KERNEL);
	if (node->cx2_fm == NULL) {
		pr_err("%s: can not allocate memory for cx2_fm ... ERROR %08X",
			__func__, ERROR_ALLOC);
		kfree(node->ix2_fm);
		kfree(data);
		return ERROR_ALLOC;
	}
	node->ix2_sn = (u8 *)kmalloc(node->header.sense_node * (sizeof(u8)),
				     GFP_KERNEL);
	if (node->ix2_sn == NULL) {
		pr_err("%s: can not allocate memory for ix2_sn ERROR %08X",
			__func__, ERROR_ALLOC);
		kfree(node->ix2_fm);
		kfree(node->cx2_fm);
		kfree(data);
		return ERROR_ALLOC;
	}
	node->cx2_sn = (i8 *)kmalloc(node->header.sense_node * (sizeof(i8)),
				     GFP_KERNEL);
	if (node->cx2_sn == NULL) {
		pr_err("%s: can not allocate memory for cx2_sn ERROR %08X",
			__func__, ERROR_ALLOC);
		kfree(node->ix2_fm);
		kfree(node->cx2_fm);
		kfree(node->ix2_sn);
		kfree(data);
		return ERROR_ALLOC;
	}


	pr_info("Address for Node data = %llx\n", address);

	pr_info("Node Data to read %d bytes\n", size);

	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, address, data,
				size, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("%s: error while reading data... ERROR %08X\n",
			__func__, ret);
		kfree(node->ix2_fm);
		kfree(node->cx2_fm);
		kfree(node->ix2_sn);
		kfree(node->cx2_sn);
		kfree(data);
		return ret;
	}

	pr_info("Read node data ok!\n");

	memcpy(node->ix2_fm, data, node->header.force_node);
	memcpy(node->ix2_sn, &data[node->header.force_node],
	       node->header.sense_node);
	memcpy(node->cx2_fm, &data[node->header.force_node +
				   node->header.sense_node],
	       node->header.force_node);
	memcpy(node->cx2_sn, &data[node->header.force_node * 2 +
				   node->header.sense_node],
	       node->header.sense_node);

	kfree(data);
	return OK;
}

/**
  * Perform all the steps to read the necessary info for SS Initialization data
  * from the buffer and store it in a SelfSenseData variable
  * @param type type of SS Initialization data to read @link load_opt Load Host
  * Data Option @endlink
  * @param data pointer to SelfSenseData variable which will contain the SS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
int readSelfSenseCompensationData(u8 type, SelfSenseData *data)
{
	int ret;
	u64 address;

	data->ix2_fm = NULL;
	data->cx2_fm = NULL;
	data->ix2_sn = NULL;
	data->cx2_sn = NULL;

	if (!(type == LOAD_CX_SS_TOUCH || type == LOAD_CX_SS_TOUCH_IDLE ||
	      type == LOAD_CX_SS_KEY || type == LOAD_CX_SS_FORCE)) {
		pr_err("%s: Choose a SS type of compensation data ERROR %08X\n",
			__func__, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = requestHDMDownload(type);
	if (ret < 0) {
		pr_err("%s: error while requesting data... ERROR %08X\n",
			__func__, ret);
		return ret;
	}

	ret = readHDMHeader(type, &(data->header), &address);
	if (ret < 0) {
		pr_err("%s: error while reading data header... ERROR %08X\n",
			__func__, ERROR_HDM_DATA_HEADER);
		return ret | ERROR_HDM_DATA_HEADER;
	}

	ret = readSelfSenseGlobalData(&address, data);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_COMP_DATA_GLOBAL);
		return ret | ERROR_COMP_DATA_GLOBAL;
	}

	ret = readSelfSenseNodeData(address, data);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_COMP_DATA_NODE);
		return ret | ERROR_COMP_DATA_NODE;
	}

	return OK;
}

/**
  * Read TOT MS Global Initialization data from the buffer such as number of
  * force and sense channels
  * @param address pointer to a variable which contain the address from where
  * to read the data and will contain the updated address to the next data
  * @param global pointer to a variable which will contain the TOT MS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
static int readTotMutualSenseGlobalData(u64 *address, TotMutualSenseData *global)
{
	int ret;
	u8 data[COMP_DATA_GLOBAL];

	pr_info("Address for Global data= %llx\n", *address);

	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, *address, data,
				COMP_DATA_GLOBAL, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("%s: error while reading info data ERROR %08X\n",
			__func__, ret);
		return ret;
	}
	pr_info("Global data Read !\n");

	global->header.force_node = data[0];
	global->header.sense_node = data[1];
	/* all other bytes are reserved atm */

	pr_info("force_len = %d sense_len = %d\n",
		global->header.force_node, global->header.sense_node);

	*address += COMP_DATA_GLOBAL;
	return OK;
}


/**
  * Read TOT MS Initialization data for each node from the buffer
  * @param address a variable which contain the address from where to read the
  * data
  * @param node pointer to MutualSenseData variable which will contain the TOT
  * MS initialization data
  * @return OK if success or an error code which specify the type of error
  */
static int readTotMutualSenseNodeData(u64 address, TotMutualSenseData *node)
{
	int ret, i;
	int size = node->header.force_node * node->header.sense_node;
	int toRead = size * sizeof(u16);
	u8 *data;

	if (size <= 0) {
		pr_err("%s: Invalid MS data length!\n", __func__);
		return ERROR_OP_NOT_ALLOW;
	}

	data = kzalloc(toRead * sizeof(u8), GFP_KERNEL);
	if (data == NULL)
		return ERROR_ALLOC;

	pr_info("Address for Node data = %llx\n", address);

	node->node_data = (short *)kmalloc(size * (sizeof(short)), GFP_KERNEL);

	if (node->node_data == NULL) {
		pr_err("%s: can not allocate node_data... ERROR %08X",
			__func__, ERROR_ALLOC);
		kfree(data);
		return ERROR_ALLOC;
	}

	pr_info("Node Data to read %d bytes\n", size);

	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, address, data,
				toRead, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("%s: error while reading node data ERROR %08X\n",
			__func__, ret);
		kfree(node->node_data);
		kfree(data);
		return ret;
	}
	node->node_data_size = size;

	for (i = 0; i < size; i++)
		node->node_data[i] = ((short)data[i * 2 + 1]) << 8 |
				      data[i * 2];

	pr_info("Read node data OK!\n");

	kfree(data);
	return size;
}

/**
  * Perform all the steps to read the necessary info for TOT MS Initialization
  * data from the buffer and store it in a TotMutualSenseData variable
  * @param type type of TOT MS Initialization data to read @link load_opt Load
  * Host Data Option @endlink
  * @param data pointer to a variable which will contain the TOT MS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
int readTotMutualSenseCompensationData(u8 type, TotMutualSenseData *data)
{
	int ret;
	u64 address;

	data->node_data = NULL;

	if (!(type == LOAD_PANEL_CX_TOT_MS_TOUCH || type ==
	      LOAD_PANEL_CX_TOT_MS_LOW_POWER ||
	      type == LOAD_PANEL_CX_TOT_MS_KEY ||
	      type == LOAD_PANEL_CX_TOT_MS_FORCE)) {
		pr_err("%s: Choose a TOT MS type of compensation data ERROR %08X\n",
			__func__, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = requestHDMDownload(type);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ret);
		return ret;
	}

	ret = readHDMHeader(type, &(data->header), &address);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_HDM_DATA_HEADER);
		return ret | ERROR_HDM_DATA_HEADER;
	}

	ret = readTotMutualSenseGlobalData(&address, data);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_COMP_DATA_GLOBAL);
		return ret | ERROR_COMP_DATA_GLOBAL;
	}

	ret = readTotMutualSenseNodeData(address, data);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_COMP_DATA_NODE);
		return ret | ERROR_COMP_DATA_NODE;
	}

	return OK;
}

/**
  * Read TOT SS Global Initialization data from the buffer such as number of
  * force and sense channels
  * @param address pointer to a variable which contain the address from where
  * to read the data and will contain the updated address to the next data
  * @param global pointer to a variable which will contain the TOT SS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
static int readTotSelfSenseGlobalData(u64 *address, TotSelfSenseData *global)
{
	int ret;
	u8 data[COMP_DATA_GLOBAL];

	pr_info("Address for Global data= %llx\n", *address);
	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, *address, data,
				COMP_DATA_GLOBAL, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("%s: error while reading the data... ERROR %08X\n",
			__func__, ret);
		return ret;
	}

	pr_info("Global data Read !\n");


	global->header.force_node = data[0];
	global->header.sense_node = data[1];


	pr_info("force_len = %d sense_len = %d\n",
		global->header.force_node, global->header.sense_node);


	*address += COMP_DATA_GLOBAL;

	return OK;
}

/**
  * Read TOT SS Global Initialization data from the buffer such as number of
  * force and sense channels
  * @param address pointer to a variable which contain the address from where
  * to read the data and will contain the updated address to the next data
  * @param node pointer to a variable which will contain the TOT SS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
static int readTotSelfSenseNodeData(u64 address, TotSelfSenseData *node)
{
	int size = node->header.force_node * 2 + node->header.sense_node * 2;
	int toRead = size * 2;	/* *2 2 bytes each node */
	u8 *data;
	int ret, i, j = 0;

	if (size <= 0) {
		pr_err("%s: Invalid Tot SS data length!\n", __func__);
		return ERROR_OP_NOT_ALLOW;
	}

	data = kzalloc(toRead * sizeof(u8), GFP_KERNEL);
	if (data == NULL)
		return ERROR_ALLOC;

	node->ix_fm = (u16 *)kmalloc(node->header.force_node * (sizeof(u16)),
				     GFP_KERNEL);
	if (node->ix_fm == NULL) {
		pr_err("%s: can not allocate memory for ix2_fm... ERROR %08X",
			__func__, ERROR_ALLOC);
		kfree(data);
		return ERROR_ALLOC;
	}

	node->cx_fm = (short *)kmalloc(node->header.force_node *
				       (sizeof(short)), GFP_KERNEL);
	if (node->cx_fm == NULL) {
		pr_err("%s: can not allocate memory for cx2_fm ... ERROR %08X",
			__func__, ERROR_ALLOC);
		kfree(node->ix_fm);
		kfree(data);
		return ERROR_ALLOC;
	}
	node->ix_sn = (u16 *)kmalloc(node->header.sense_node * (sizeof(u16)),
				     GFP_KERNEL);
	if (node->ix_sn == NULL) {
		pr_err("%s: can not allocate memory for ix2_sn ERROR %08X",
			__func__, ERROR_ALLOC);
		kfree(node->ix_fm);
		kfree(node->cx_fm);
		kfree(data);
		return ERROR_ALLOC;
	}
	node->cx_sn = (short *)kmalloc(node->header.sense_node *
				       (sizeof(short)), GFP_KERNEL);
	if (node->cx_sn == NULL) {
		pr_err("%s: can not allocate memory for cx2_sn ERROR %08X",
			__func__, ERROR_ALLOC);
		kfree(node->ix_fm);
		kfree(node->cx_fm);
		kfree(node->ix_sn);
		kfree(data);
		return ERROR_ALLOC;
	}


	pr_info("Address for Node data = %llx\n", address);

	pr_info("Node Data to read %d bytes\n", size);

	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, address, data,
				toRead, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("%s: error while reading data... ERROR %08X\n",
			__func__, ret);
		kfree(node->ix_fm);
		kfree(node->cx_fm);
		kfree(node->ix_sn);
		kfree(node->cx_sn);
		kfree(data);
		return ret;
	}

	pr_info("Read node data ok!\n");

	j = 0;
	for (i = 0; i < node->header.force_node; i++) {
		node->ix_fm[i] = ((u16)data[j + 1]) << 8 | data[j];
		j += 2;
	}

	for (i = 0; i < node->header.sense_node; i++) {
		node->ix_sn[i] = ((u16)data[j + 1]) << 8 | data[j];
		j += 2;
	}

	for (i = 0; i < node->header.force_node; i++) {
		node->cx_fm[i] = ((short)data[j + 1]) << 8 | data[j];
		j += 2;
	}

	for (i = 0; i < node->header.sense_node; i++) {
		node->cx_sn[i] = ((short)data[j + 1]) << 8 | data[j];
		j += 2;
	}

	if (j != toRead)
		pr_err("%s: parsed a wrong number of bytes %d!=%d\n",
			__func__, j, toRead);

	kfree(data);
	return OK;
}

/**
  * Perform all the steps to read the necessary info for TOT SS Initialization
  * data from the buffer and store it in a TotSelfSenseData variable
  * @param type type of TOT MS Initialization data to read @link load_opt Load
  * Host Data Option @endlink
  * @param data pointer to a variable which will contain the TOT MS
  * initialization data
  * @return OK if success or an error code which specify the type of error
  */
int readTotSelfSenseCompensationData(u8 type, TotSelfSenseData *data)
{
	int ret;
	u64 address;

	data->ix_fm = NULL;
	data->cx_fm = NULL;
	data->ix_sn = NULL;
	data->cx_sn = NULL;

	if (!(type == LOAD_PANEL_CX_TOT_SS_TOUCH || type ==
	      LOAD_PANEL_CX_TOT_SS_TOUCH_IDLE || type ==
	      LOAD_PANEL_CX_TOT_SS_KEY ||
	      type == LOAD_PANEL_CX_TOT_SS_FORCE)) {
		pr_err("%s: Choose a TOT SS type of compensation data ERROR %08X\n",
			__func__, ERROR_OP_NOT_ALLOW);
		return ERROR_OP_NOT_ALLOW;
	}

	ret = requestHDMDownload(type);
	if (ret < 0) {
		pr_err("%s: error while requesting data... ERROR %08X\n",
			__func__, ret);
		return ret;
	}

	ret = readHDMHeader(type, &(data->header), &address);
	if (ret < 0) {
		pr_err("%s: error while reading data header... ERROR %08X\n",
			__func__, ERROR_HDM_DATA_HEADER);
		return ret | ERROR_HDM_DATA_HEADER;
	}

	ret = readTotSelfSenseGlobalData(&address, data);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_COMP_DATA_GLOBAL);
		return ret | ERROR_COMP_DATA_GLOBAL;
	}

	ret = readTotSelfSenseNodeData(address, data);
	if (ret < 0) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_COMP_DATA_NODE);
		return ret | ERROR_COMP_DATA_NODE;
	}

	return OK;
}


/**
  * Read Initialization Data Header for the Coefficients and check that the type
  *  loaded match with the one previously requested
  * @param type type of Coefficients data requested @link load_opt Load Host
  * Data Option @endlink
  * @param msHeader pointer to DataHeader variable for the MS Coefficients
  * @param ssHeader pointer to DataHeader variable for the SS Coefficients
  * @param address pointer to a variable which will contain the updated address
  * to the next data
  * @return OK if success or an error code which specify the type of error
  */
static int readSensitivityCoeffHeader(u8 type, DataHeader *msHeader,
			       DataHeader *ssHeader, u64 *address)
{
	u64 offset = ADDR_FRAMEBUFFER;
	u8 data[SYNCFRAME_DATA_HEADER];
	int ret;

	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, offset, data,
				SYNCFRAME_DATA_HEADER, DUMMY_FRAMEBUFFER);
	if (ret < OK) {	/* i2c function have already a retry mechanism */
		pr_err("%s: error while reading data header ERROR %08X\n",
			__func__, ret);
		return ret;
	}

	pr_info("Read Data Header done!\n");

	if (data[0] != HEADER_SIGNATURE) {
		pr_err("%s: The Header Signature was wrong! %02X != %02X ERROR %08X\n",
			__func__, data[0], HEADER_SIGNATURE,
			ERROR_WRONG_DATA_SIGN);
		return ERROR_WRONG_DATA_SIGN;
	}


	if (data[1] != type) {
		pr_err("%s: Wrong type found! %02X!=%02X ERROR %08X\n",
			__func__, data[1], type, ERROR_DIFF_DATA_TYPE);
		return ERROR_DIFF_DATA_TYPE;
	}

	pr_info("Type = %02X of Compensation data OK!\n", type);

	msHeader->type = type;
	ssHeader->type = type;

	msHeader->force_node = data[5];
	msHeader->sense_node = data[6];
	pr_info("MS Force Len = %d Sense Len = %d\n",
		msHeader->force_node, msHeader->sense_node);

	ssHeader->force_node = data[7];
	ssHeader->sense_node = data[8];
	pr_info("SS Force Len = %d Sense Len = %d\n",
		ssHeader->force_node, ssHeader->sense_node);

	*address = offset + SYNCFRAME_DATA_HEADER;

	return OK;
}


/**
  * Read MS and SS Sensitivity Coefficients for from the IC
  * @param address a variable which contain the address from where to read the
  * data
  * @param msCoeff pointer to MutualSenseCoeff variable which will contain the
  * MS Coefficient data
  * @param ssCoeff pointer to SelfSenseCoeff variable which will contain the SS
  * Coefficient data
  * @return OK if success or an error code which specify the type of error
  */
static int readSensitivityCoeffNodeData(u64 address, MutualSenseCoeff *msCoeff,
				 SelfSenseCoeff *ssCoeff)
{
	int size = msCoeff->header.force_node * msCoeff->header.sense_node +
		   (ssCoeff->header.force_node + ssCoeff->header.sense_node);
	u8 *data;
	int ret;

	if (size <= 0) {
		pr_err("%s:Invalid SS coeff. length!\n", __func__);
		return ERROR_OP_NOT_ALLOW;
	}

	data = kzalloc(size * sizeof(u8), GFP_KERNEL);
	if (data == NULL)
		return ERROR_ALLOC;

	msCoeff->node_data_size = msCoeff->header.force_node *
				  msCoeff->header.sense_node;

	msCoeff->ms_coeff = (u8 *)kmalloc(msCoeff->node_data_size *
					  (sizeof(u8)), GFP_KERNEL);

	ssCoeff->ss_force_coeff = (u8 *)kmalloc(ssCoeff->header.force_node *
						(sizeof(u8)), GFP_KERNEL);

	ssCoeff->ss_sense_coeff = (u8 *)kmalloc(ssCoeff->header.sense_node *
						(sizeof(u8)), GFP_KERNEL);
	if (msCoeff->ms_coeff == NULL ||
	    ssCoeff->ss_force_coeff == NULL ||
	    ssCoeff->ss_sense_coeff == NULL) {

		pr_err("%s: can not allocate memory for coeff ERROR %08X",
			__func__, ERROR_ALLOC);

		kfree(msCoeff->ms_coeff);
		msCoeff->ms_coeff = NULL;

		kfree(ssCoeff->ss_force_coeff);
		ssCoeff->ss_force_coeff = NULL;

		kfree(ssCoeff->ss_sense_coeff);
		ssCoeff->ss_sense_coeff = NULL;

		kfree(data);
		return ERROR_ALLOC;
	}

	pr_info("Address for Node data = %llx\n", address);

	pr_info("Node Data to read %d bytes\n", size);

	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, address, data,
				size, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("%s: error while reading data... ERROR %08X\n",
			__func__, ret);
		kfree(msCoeff->ms_coeff);
		msCoeff->ms_coeff = NULL;
		kfree(ssCoeff->ss_force_coeff);
		ssCoeff->ss_force_coeff = NULL;
		kfree(ssCoeff->ss_sense_coeff);
		ssCoeff->ss_sense_coeff = NULL;
		kfree(data);
		return ret;
	}

	pr_info("Read node data ok!\n");

	memcpy(msCoeff->ms_coeff, data, msCoeff->node_data_size);
	memcpy(ssCoeff->ss_force_coeff, &data[msCoeff->node_data_size],
	       ssCoeff->header.force_node);
	memcpy(ssCoeff->ss_sense_coeff, &data[msCoeff->node_data_size +
					      ssCoeff->header.force_node],
	       ssCoeff->header.sense_node);

	kfree(data);
	return OK;
}


/**
  * Perform all the steps to read Sensitivity Coefficients and store into the
  * corresponding variables
  * @param msCoeff pointer to a variable which will contain the MS Sensitivity
  * Coefficients
  * @param ssCoeff pointer to a variable which will contain the SS Sensitivity
  * Coefficients
  * @return OK if success or an error code which specify the type of error
  */
int readSensitivityCoefficientsData(MutualSenseCoeff *msCoeff,
				    SelfSenseCoeff *ssCoeff)
{
	int ret;
	u64 address;

	msCoeff->ms_coeff = NULL;
	ssCoeff->ss_force_coeff = NULL;
	ssCoeff->ss_sense_coeff = NULL;


	ret = requestHDMDownload(LOAD_SENS_CAL_COEFF);
	if (ret < OK) {
		pr_err("%s: error while requesting data... ERROR %08X\n",
			__func__, ret);
		return ret;
	}

	ret = readSensitivityCoeffHeader(LOAD_SENS_CAL_COEFF,
					 &(msCoeff->header), &(ssCoeff->header),
					 &address);
	if (ret < OK) {
		pr_err("%s: error while reading data header... ERROR %08X\n",
			__func__, ERROR_HDM_DATA_HEADER);
		return ret | ERROR_HDM_DATA_HEADER;
	}

	ret = readSensitivityCoeffNodeData(address, msCoeff, ssCoeff);
	if (ret < OK) {
		pr_err("%s: ERROR %08X\n", __func__, ERROR_COMP_DATA_NODE);
		return ret | ERROR_COMP_DATA_NODE;
	}

	return OK;
}

/**
  * Read Golden Mutual Raw data from FW Host Data Memory.
  * @param address a variable which contain the address from where to read the
  * data
  * @param node pointer to GoldenMutualRawData variable.
  * @return OK if success or an error code which specify the type of error
  */
static int readGoldenMutualData(GoldenMutualRawData *pgmData, u64 address)
{
	u32 size, i;
	int ret;

	pgmData->data_size = 0;
	pgmData->data 	   = NULL;

	pr_info("Address for Golden Mutual hdr = %llx\n", address);

	/* read 12 byte Golden Mutual header */
	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16,
				address, (u8 *)&(pgmData->hdr),
				GM_DATA_HEADER, DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("error while reading Golden Mutual hdr... ERROR %08X\n",
			ret);
		goto out;
	}

	pr_info("ms_force_len = %u ms_sense_len = %u\n",
		 pgmData->hdr.ms_f_len, pgmData->hdr.ms_s_len);
	pr_info("ss_force_len = %u ss_sense_len = %u\n",
		 pgmData->hdr.ss_f_len, pgmData->hdr.ss_s_len);
 	pr_info("ms_key_len = %u \n", pgmData->hdr.ms_k_len);

	size = pgmData->hdr.ms_f_len * pgmData->hdr.ms_s_len;

	pgmData->data = kzalloc(size * sizeof(s16), GFP_KERNEL);
	if (pgmData->data == NULL) {
		ret = ERROR_ALLOC;
		pr_err("Unable to allocate memory for GM raw data. ERR %08X",
			ret);
		goto out;
	}

	/* go past both HDM and GM header to read the data */
	address += GM_DATA_HEADER;
	pr_info("Address for Golden Mutual data = %llx\n", address);

	//read the data buffer.
	ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, address,
				(u8 *)pgmData->data, size * sizeof(s16),
				DUMMY_FRAMEBUFFER);
	if (ret < OK) {
		pr_err("error while reading Golden Mutual data... ERROR %08X\n", ret);
		kfree(pgmData->data);
		pgmData->data = NULL;
		goto out;
	}

	pgmData->data_size = size;

	pr_info("Read data ok!\n");

	for (i = 0; i < size; i++)
		le16_to_cpus(&pgmData->data[i]);

	ret = OK;
out:
	return ret;
}

/**
  * Perform all the steps to read the necessary info for Golden Mutual raw
  * data from the buffer and store it in a GoldemMutualRawData object.
  * @param pointer to GoldemMutualRawData variable which will contain
  * the raw data.
  * @return OK if success or an error code which specify the type of error
  */
int readGoldenMutualRawData(GoldenMutualRawData *pgmData)
{
	int ret;
	u64 address;

	ret = requestHDMDownload(LOAD_GOLDEN_MUTUAL_RAW);
	if (ret < 0) {
		pr_err("error while requesting HDM Download... ERROR %08X\n",
			ret);
		goto out;
	}

	ret = readHDMHeader(LOAD_GOLDEN_MUTUAL_RAW,
			&pgmData->hdm_hdr, &address);
	if (ret < 0) {
		pr_err("error reading HDM header... ERROR %08X\n",
			ERROR_HDM_DATA_HEADER);
		ret |= ERROR_HDM_DATA_HEADER;
		goto out;
	}

	ret = readGoldenMutualData(pgmData, address);
	if (ret < 0) {
		pr_err("error reading Golden Mutual data... ERROR %08X\n",
			ERROR_GOLDEN_MUTUAL_DATA);
		ret |= ERROR_GOLDEN_MUTUAL_DATA;
		goto out;
	}

	ret = OK;
out:
	return ret;
}
