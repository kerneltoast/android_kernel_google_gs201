/*
  *
  **************************************************************************
  **                        STMicroelectronics				  **
  **************************************************************************
  **                        marco.cali@st.com				**
  **************************************************************************
  *                                                                        *
  *               FTS functions for getting Initialization Data		  **
  *                                                                        *
  **************************************************************************
  **************************************************************************
  *
  */

/*!
  * \file ftsCompensation.h
  * \brief Contains all the definitions and structs to work with Initialization
  * Data
  */

#ifndef FTS_COMPENSATION_H
#define FTS_COMPENSATION_H

#include "ftsCore.h"
#include "ftsSoftware.h"



#define RETRY_FW_HDM_DOWNLOAD 2	/* /< max number of attempts to
				 * request HDM download */


/* Bytes dimension of HDM content */

#define HDM_DATA_HEADER	DATA_HEADER	/* /< size in bytes of
						 * initialization data header */
#define COMP_DATA_GLOBAL	(16 - HDM_DATA_HEADER)	/* /< size in bytes
							 * of initialization
							 * data general info */
#define GM_DATA_HEADER		(16 - HDM_DATA_HEADER)	/* /< size in bytes of
 						* Golden Mutual data header */


#define HEADER_SIGNATURE	0xA5	/* /< signature used as starting byte of
					 * data loaded in memory */



/**
  * Struct which contains the general info about Frames and Initialization Data
  */
typedef struct {
	int force_node;	/* /< Number of Force Channels in the
			 * frame/Initialization data */
	int sense_node;	/* /< Number of Sense Channels in the
			 * frame/Initialization data */
	int type;	/* /< Type of frame/Initialization data */
} DataHeader;

/**
  * Struct which contains the MS Initialization data
  */
typedef struct {
	DataHeader header;	/* /< Header */
	i8 cx1;			/* /< Cx1 value (can be negative)) */
	i8 *node_data;	/* /< Pointer to an array of bytes which contains the
			 * CX2 data (can be negative) */
	int node_data_size;	/* /< size of the data */
} MutualSenseData;


/**
  * Struct which contains the SS Initialization data
  */
typedef struct {
	DataHeader header;	/* /< Header */
	u8 f_ix1;	/* /< IX1 Force */
	u8 s_ix1;	/* /< IX1 Sense */
	i8 f_cx1;	/* /< CX1 Force (can be negative) */
	i8 s_cx1;	/* /< CX1 Sense (can be negative) */
	u8 f_max_n;	/* /< Force MaxN */
	u8 s_max_n;	/* /< Sense MaxN */
	u8 f_ix0;	/* /< IX0 Force */
	u8 s_ix0;	/* /< IX0 Sense */

	u8 *ix2_fm;	/* /< pointer to an array of bytes which contains Force
			 * Ix2 data node */
	u8 *ix2_sn;	/* /< pointer to an array of bytes which contains Sense
			 * Ix2 data node */
	i8 *cx2_fm;	/* /< pointer to an array of bytes which contains Force
			 * Cx2 data node
			 * (can be negative) */
	i8 *cx2_sn;	/* /< pointer to an array of bytes which contains Sense
			 * Cx2 data node
			 * (can be negative)) */
} SelfSenseData;

/**
  * Struct which contains the TOT MS Initialization data
  */
typedef struct {
	DataHeader header;	/* /< Header */
	short *node_data;	/* /< pointer to an array of ushort which
				 * contains TOT MS Initialization data */
	int node_data_size;	/* /< size of data */
} TotMutualSenseData;

/**
  * Struct which contains the TOT SS Initialization data
  */
typedef struct {
	DataHeader header;	/* /< Header */

	u16 *ix_fm;	/* /< pointer to an array of ushort which contains TOT
			 * SS IX Force data */
	u16 *ix_sn;	/* /< pointer to an array of ushort which contains TOT
			 * SS IX Sense data */
	short *cx_fm;	/* /< pointer to an array of ushort which contains TOT
			 * SS CX Force data
			 * (can be negative) */
	short *cx_sn;	/* /< pointer to an array of ushort which contains TOT
			 * SS CX Sense data
			 * (can be negative) */
} TotSelfSenseData;

/**
  * Struct which contains the Mutual Sense Sensitivity Calibration Coefficients
  */
typedef struct {
	DataHeader header;	/* /< Header */
	u8 *ms_coeff;	/* /< Pointer to an array of bytes which contains the MS
			 * Sens coeff */
	int node_data_size;	/* /< size of coefficients */
} MutualSenseCoeff;

/**
  * Struct which contains the Self Sense Sensitivity Calibration Coefficients
  */
typedef struct {
	DataHeader header;	/* /< Header */
	u8 *ss_force_coeff;	/* /< Pointer to an array of bytes which
				 * contains the SS Sens Force coeff */
	u8 *ss_sense_coeff;	/* /< Pointer to an array of bytes which
				 * contains the SS Sens Sense coeff */
} SelfSenseCoeff;

/**
  * Struct which contains the Golden Mutual Header
  */
typedef struct {
	u8 ms_f_len;		/* /< ms force length */
	u8 ms_s_len;		/* /< ms sense length */
	u8 ss_f_len;		/* /< ss force length */
	u8 ss_s_len;		/* /< ss sense length */
	u8 ms_k_len;		/* /< ms key length   */
	u8 reserved_0[3];
	u32 reserved_1;
}GoldenMutualHdr;

/**
  * Struct which contains the Golden Mutual Raw data
  */
typedef struct {
	DataHeader hdm_hdr;	/* /< HDM Header */
	GoldenMutualHdr hdr; 	/* /< Golden Mutual Data Hdr */
	s16 *data; 	        /* /< pointer to the raw data */
	u32 data_size; 	        /* /< size of raw data buffer */
} GoldenMutualRawData;

int requestHDMDownload(u8 type);
int readHDMHeader(u8 type, DataHeader *header, u64 *address);
int readMutualSenseCompensationData(u8 type, MutualSenseData *data);
int readSelfSenseCompensationData(u8 type, SelfSenseData *data);
int readTotMutualSenseCompensationData(u8 type, TotMutualSenseData *data);
int readTotSelfSenseCompensationData(u8 type, TotSelfSenseData *data);
int readSensitivityCoefficientsData(MutualSenseCoeff *msData,
				    SelfSenseCoeff *ssData);
int readGoldenMutualRawData(GoldenMutualRawData *pgmData);

#endif
