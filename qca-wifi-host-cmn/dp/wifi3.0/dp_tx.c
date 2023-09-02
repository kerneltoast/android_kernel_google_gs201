/*
 * Copyright (c) 2016-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#include "htt.h"
#include "dp_htt.h"
#include "hal_hw_headers.h"
#include "dp_tx.h"
#include "dp_tx_desc.h"
#include "dp_peer.h"
#include "dp_types.h"
#include "hal_tx.h"
#include "qdf_mem.h"
#include "qdf_nbuf.h"
#include "qdf_net_types.h"
#include "qdf_module.h"
#include <wlan_cfg.h>
#include "dp_ipa.h"
#if defined(MESH_MODE_SUPPORT) || defined(FEATURE_PERPKT_INFO)
#include "if_meta_hdr.h"
#endif
#include "enet.h"
#include "dp_internal.h"
#ifdef ATH_SUPPORT_IQUE
#include "dp_txrx_me.h"
#endif
#include "dp_hist.h"
#ifdef WLAN_DP_FEATURE_SW_LATENCY_MGR
#include <dp_swlm.h>
#endif
#ifdef WIFI_MONITOR_SUPPORT
#include <dp_mon.h>
#endif
#ifdef FEATURE_WDS
#include "dp_txrx_wds.h"
#endif
#include "cdp_txrx_cmn_reg.h"

/* Flag to skip CCE classify when mesh or tid override enabled */
#define DP_TX_SKIP_CCE_CLASSIFY \
	(DP_TXRX_HLOS_TID_OVERRIDE_ENABLED | DP_TX_MESH_ENABLED)

/* TODO Add support in TSO */
#define DP_DESC_NUM_FRAG(x) 0

/* disable TQM_BYPASS */
#define TQM_BYPASS_WAR 0

/* invalid peer id for reinject*/
#define DP_INVALID_PEER 0XFFFE

#define DP_RETRY_COUNT 7

/*mapping between hal encrypt type and cdp_sec_type*/
uint8_t sec_type_map[MAX_CDP_SEC_TYPE] = {HAL_TX_ENCRYPT_TYPE_NO_CIPHER,
					  HAL_TX_ENCRYPT_TYPE_WEP_128,
					  HAL_TX_ENCRYPT_TYPE_WEP_104,
					  HAL_TX_ENCRYPT_TYPE_WEP_40,
					  HAL_TX_ENCRYPT_TYPE_TKIP_WITH_MIC,
					  HAL_TX_ENCRYPT_TYPE_TKIP_NO_MIC,
					  HAL_TX_ENCRYPT_TYPE_AES_CCMP_128,
					  HAL_TX_ENCRYPT_TYPE_WAPI,
					  HAL_TX_ENCRYPT_TYPE_AES_CCMP_256,
					  HAL_TX_ENCRYPT_TYPE_AES_GCMP_128,
					  HAL_TX_ENCRYPT_TYPE_AES_GCMP_256,
					  HAL_TX_ENCRYPT_TYPE_WAPI_GCM_SM4};
qdf_export_symbol(sec_type_map);

#ifdef CONFIG_WLAN_SYSFS_MEM_STATS
/**
 * dp_update_tx_desc_stats - Update the increase or decrease in
 * outstanding tx desc count
 * values on pdev and soc
 * @vdev: DP pdev handle
 *
 * Return: void
 */
static inline void
dp_update_tx_desc_stats(struct dp_pdev *pdev)
{
	int32_t tx_descs_cnt =
		qdf_atomic_read(&pdev->num_tx_outstanding);
	if (pdev->tx_descs_max < tx_descs_cnt)
		pdev->tx_descs_max = tx_descs_cnt;
	qdf_mem_tx_desc_cnt_update(pdev->num_tx_outstanding,
				   pdev->tx_descs_max);
}

#else /* CONFIG_WLAN_SYSFS_MEM_STATS */

static inline void
dp_update_tx_desc_stats(struct dp_pdev *pdev)
{
}
#endif /* CONFIG_WLAN_SYSFS_MEM_STATS */

#if defined(WLAN_TX_PKT_CAPTURE_ENH) || defined(FEATURE_PERPKT_INFO)
static inline
void dp_tx_enh_unmap(struct dp_soc *soc, struct dp_tx_desc_s *desc)
{
	qdf_nbuf_unmap_nbytes_single(soc->osdev, desc->nbuf,
				     QDF_DMA_TO_DEVICE,
				     desc->nbuf->len);
	desc->flags |= DP_TX_DESC_FLAG_UNMAP_DONE;
}

static inline void dp_tx_unmap(struct dp_soc *soc, struct dp_tx_desc_s *desc)
{
	if (qdf_likely(!(desc->flags & DP_TX_DESC_FLAG_UNMAP_DONE)))
		qdf_nbuf_unmap_nbytes_single(soc->osdev, desc->nbuf,
					     QDF_DMA_TO_DEVICE,
					     desc->nbuf->len);
}
#else
static inline
void dp_tx_enh_unmap(struct dp_soc *soc, struct dp_tx_desc_s *desc)
{
}

static inline void dp_tx_unmap(struct dp_soc *soc, struct dp_tx_desc_s *desc)
{
	qdf_nbuf_unmap_nbytes_single(soc->osdev, desc->nbuf,
				     QDF_DMA_TO_DEVICE, desc->nbuf->len);
}
#endif

#ifdef QCA_TX_LIMIT_CHECK
/**
 * dp_tx_limit_check - Check if allocated tx descriptors reached
 * soc max limit and pdev max limit
 * @vdev: DP vdev handle
 *
 * Return: true if allocated tx descriptors reached max configured value, else
 * false
 */
static inline bool
dp_tx_limit_check(struct dp_vdev *vdev)
{
	struct dp_pdev *pdev = vdev->pdev;
	struct dp_soc *soc = pdev->soc;

	if (qdf_atomic_read(&soc->num_tx_outstanding) >=
			soc->num_tx_allowed) {
		dp_tx_info("queued packets are more than max tx, drop the frame");
		DP_STATS_INC(vdev, tx_i.dropped.desc_na.num, 1);
		return true;
	}

	if (qdf_atomic_read(&pdev->num_tx_outstanding) >=
			pdev->num_tx_allowed) {
		dp_tx_info("queued packets are more than max tx, drop the frame");
		DP_STATS_INC(vdev, tx_i.dropped.desc_na.num, 1);
		DP_STATS_INC(vdev, tx_i.dropped.desc_na_exc_outstand.num, 1);
		return true;
	}
	return false;
}

/**
 * dp_tx_exception_limit_check - Check if allocated tx exception descriptors
 * reached soc max limit
 * @vdev: DP vdev handle
 *
 * Return: true if allocated tx descriptors reached max configured value, else
 * false
 */
static inline bool
dp_tx_exception_limit_check(struct dp_vdev *vdev)
{
	struct dp_pdev *pdev = vdev->pdev;
	struct dp_soc *soc = pdev->soc;

	if (qdf_atomic_read(&soc->num_tx_exception) >=
			soc->num_msdu_exception_desc) {
		dp_info("exc packets are more than max drop the exc pkt");
		DP_STATS_INC(vdev, tx_i.dropped.exc_desc_na.num, 1);
		return true;
	}

	return false;
}

/**
 * dp_tx_outstanding_inc - Increment outstanding tx desc values on pdev and soc
 * @vdev: DP pdev handle
 *
 * Return: void
 */
static inline void
dp_tx_outstanding_inc(struct dp_pdev *pdev)
{
	struct dp_soc *soc = pdev->soc;

	qdf_atomic_inc(&pdev->num_tx_outstanding);
	qdf_atomic_inc(&soc->num_tx_outstanding);
	dp_update_tx_desc_stats(pdev);
}

/**
 * dp_tx_outstanding__dec - Decrement outstanding tx desc values on pdev and soc
 * @vdev: DP pdev handle
 *
 * Return: void
 */
static inline void
dp_tx_outstanding_dec(struct dp_pdev *pdev)
{
	struct dp_soc *soc = pdev->soc;

	qdf_atomic_dec(&pdev->num_tx_outstanding);
	qdf_atomic_dec(&soc->num_tx_outstanding);
	dp_update_tx_desc_stats(pdev);
}

#else //QCA_TX_LIMIT_CHECK
static inline bool
dp_tx_limit_check(struct dp_vdev *vdev)
{
	return false;
}

static inline bool
dp_tx_exception_limit_check(struct dp_vdev *vdev)
{
	return false;
}

static inline void
dp_tx_outstanding_inc(struct dp_pdev *pdev)
{
	qdf_atomic_inc(&pdev->num_tx_outstanding);
	dp_update_tx_desc_stats(pdev);
}

static inline void
dp_tx_outstanding_dec(struct dp_pdev *pdev)
{
	qdf_atomic_dec(&pdev->num_tx_outstanding);
	dp_update_tx_desc_stats(pdev);
}
#endif //QCA_TX_LIMIT_CHECK

#ifdef WLAN_FEATURE_DP_TX_DESC_HISTORY
static inline enum dp_tx_event_type dp_tx_get_event_type(uint32_t flags)
{
	enum dp_tx_event_type type;

	if (flags & DP_TX_DESC_FLAG_FLUSH)
		type = DP_TX_DESC_FLUSH;
	else if (flags & DP_TX_DESC_FLAG_TX_COMP_ERR)
		type = DP_TX_COMP_UNMAP_ERR;
	else if (flags & DP_TX_DESC_FLAG_COMPLETED_TX)
		type = DP_TX_COMP_UNMAP;
	else
		type = DP_TX_DESC_UNMAP;

	return type;
}

static inline void
dp_tx_desc_history_add(struct dp_soc *soc, dma_addr_t paddr,
		       qdf_nbuf_t skb, uint32_t sw_cookie,
		       enum dp_tx_event_type type)
{
	struct dp_tx_desc_event *entry;
	uint32_t idx;

	if (qdf_unlikely(!soc->tx_tcl_history || !soc->tx_comp_history))
		return;

	switch (type) {
	case DP_TX_COMP_UNMAP:
	case DP_TX_COMP_UNMAP_ERR:
	case DP_TX_COMP_MSDU_EXT:
		idx = dp_history_get_next_index(&soc->tx_comp_history->index,
						DP_TX_COMP_HISTORY_SIZE);
		entry = &soc->tx_comp_history->entry[idx];
		break;
	case DP_TX_DESC_MAP:
	case DP_TX_DESC_UNMAP:
	case DP_TX_DESC_COOKIE:
	case DP_TX_DESC_FLUSH:
		idx = dp_history_get_next_index(&soc->tx_tcl_history->index,
						DP_TX_TCL_HISTORY_SIZE);
		entry = &soc->tx_tcl_history->entry[idx];
		break;
	default:
		dp_info_rl("Invalid dp_tx_event_type: %d", type);
		return;
	}

	entry->skb = skb;
	entry->paddr = paddr;
	entry->sw_cookie = sw_cookie;
	entry->type = type;
	entry->ts = qdf_get_log_timestamp();
}

static inline void
dp_tx_tso_seg_history_add(struct dp_soc *soc,
			  struct qdf_tso_seg_elem_t *tso_seg,
			  qdf_nbuf_t skb, uint32_t sw_cookie,
			  enum dp_tx_event_type type)
{
	int i;

	for (i = 1; i < tso_seg->seg.num_frags; i++) {
		dp_tx_desc_history_add(soc, tso_seg->seg.tso_frags[i].paddr,
				       skb, sw_cookie, type);
	}

	if (!tso_seg->next)
		dp_tx_desc_history_add(soc, tso_seg->seg.tso_frags[0].paddr,
				       skb, 0xFFFFFFFF, type);
}

static inline void
dp_tx_tso_history_add(struct dp_soc *soc, struct qdf_tso_info_t tso_info,
		      qdf_nbuf_t skb, uint32_t sw_cookie,
		      enum dp_tx_event_type type)
{
	struct qdf_tso_seg_elem_t *curr_seg = tso_info.tso_seg_list;
	uint32_t num_segs = tso_info.num_segs;

	while (num_segs) {
		dp_tx_tso_seg_history_add(soc, curr_seg, skb, sw_cookie, type);
		curr_seg = curr_seg->next;
		num_segs--;
	}
}

#else
static inline enum dp_tx_event_type dp_tx_get_event_type(uint32_t flags)
{
	return DP_TX_DESC_INVAL_EVT;
}

static inline void
dp_tx_desc_history_add(struct dp_soc *soc, dma_addr_t paddr,
		       qdf_nbuf_t skb, uint32_t sw_cookie,
		       enum dp_tx_event_type type)
{
}

static inline void
dp_tx_tso_seg_history_add(struct dp_soc *soc,
			  struct qdf_tso_seg_elem_t *tso_seg,
			  qdf_nbuf_t skb, uint32_t sw_cookie,
			  enum dp_tx_event_type type)
{
}

static inline void
dp_tx_tso_history_add(struct dp_soc *soc, struct qdf_tso_info_t tso_info,
		      qdf_nbuf_t skb, uint32_t sw_cookie,
		      enum dp_tx_event_type type)
{
}
#endif /* WLAN_FEATURE_DP_TX_DESC_HISTORY */

static int dp_get_rtpm_tput_policy_requirement(struct dp_soc *soc);

/**
 * dp_is_tput_high() - Check if throughput is high
 *
 * @soc - core txrx main context
 *
 * The current function is based of the RTPM tput policy variable where RTPM is
 * avoided based on throughput.
 */
static inline int dp_is_tput_high(struct dp_soc *soc)
{
	return dp_get_rtpm_tput_policy_requirement(soc);
}

#if defined(FEATURE_TSO)
/**
 * dp_tx_tso_unmap_segment() - Unmap TSO segment
 *
 * @soc - core txrx main context
 * @seg_desc - tso segment descriptor
 * @num_seg_desc - tso number segment descriptor
 */
static void dp_tx_tso_unmap_segment(
		struct dp_soc *soc,
		struct qdf_tso_seg_elem_t *seg_desc,
		struct qdf_tso_num_seg_elem_t *num_seg_desc)
{
	TSO_DEBUG("%s: Unmap the tso segment", __func__);
	if (qdf_unlikely(!seg_desc)) {
		DP_TRACE(ERROR, "%s %d TSO desc is NULL!",
			 __func__, __LINE__);
		qdf_assert(0);
	} else if (qdf_unlikely(!num_seg_desc)) {
		DP_TRACE(ERROR, "%s %d TSO num desc is NULL!",
			 __func__, __LINE__);
		qdf_assert(0);
	} else {
		bool is_last_seg;
		/* no tso segment left to do dma unmap */
		if (num_seg_desc->num_seg.tso_cmn_num_seg < 1)
			return;

		is_last_seg = (num_seg_desc->num_seg.tso_cmn_num_seg == 1) ?
					true : false;
		qdf_nbuf_unmap_tso_segment(soc->osdev,
					   seg_desc, is_last_seg);
		num_seg_desc->num_seg.tso_cmn_num_seg--;
	}
}

/**
 * dp_tx_tso_desc_release() - Release the tso segment and tso_cmn_num_seg
 *                            back to the freelist
 *
 * @soc - soc device handle
 * @tx_desc - Tx software descriptor
 */
static void dp_tx_tso_desc_release(struct dp_soc *soc,
				   struct dp_tx_desc_s *tx_desc)
{
	TSO_DEBUG("%s: Free the tso descriptor", __func__);
	if (qdf_unlikely(!tx_desc->msdu_ext_desc->tso_desc)) {
		dp_tx_err("SO desc is NULL!");
		qdf_assert(0);
	} else if (qdf_unlikely(!tx_desc->msdu_ext_desc->tso_num_desc)) {
		dp_tx_err("TSO num desc is NULL!");
		qdf_assert(0);
	} else {
		struct qdf_tso_num_seg_elem_t *tso_num_desc =
			(struct qdf_tso_num_seg_elem_t *)tx_desc->
				msdu_ext_desc->tso_num_desc;

		/* Add the tso num segment into the free list */
		if (tso_num_desc->num_seg.tso_cmn_num_seg == 0) {
			dp_tso_num_seg_free(soc, tx_desc->pool_id,
					    tx_desc->msdu_ext_desc->
					    tso_num_desc);
			tx_desc->msdu_ext_desc->tso_num_desc = NULL;
			DP_STATS_INC(tx_desc->pdev, tso_stats.tso_comp, 1);
		}

		/* Add the tso segment into the free list*/
		dp_tx_tso_desc_free(soc,
				    tx_desc->pool_id, tx_desc->msdu_ext_desc->
				    tso_desc);
		tx_desc->msdu_ext_desc->tso_desc = NULL;
	}
}
#else
static void dp_tx_tso_unmap_segment(
		struct dp_soc *soc,
		struct qdf_tso_seg_elem_t *seg_desc,
		struct qdf_tso_num_seg_elem_t *num_seg_desc)

{
}

static void dp_tx_tso_desc_release(struct dp_soc *soc,
				   struct dp_tx_desc_s *tx_desc)
{
}
#endif

/**
 * dp_tx_desc_release() - Release Tx Descriptor
 * @tx_desc : Tx Descriptor
 * @desc_pool_id: Descriptor Pool ID
 *
 * Deallocate all resources attached to Tx descriptor and free the Tx
 * descriptor.
 *
 * Return:
 */
void
dp_tx_desc_release(struct dp_tx_desc_s *tx_desc, uint8_t desc_pool_id)
{
	struct dp_pdev *pdev = tx_desc->pdev;
	struct dp_soc *soc;
	uint8_t comp_status = 0;

	qdf_assert(pdev);

	soc = pdev->soc;

	dp_tx_outstanding_dec(pdev);

	if (tx_desc->msdu_ext_desc) {
		if (tx_desc->frm_type == dp_tx_frm_tso)
			dp_tx_tso_desc_release(soc, tx_desc);

		if (tx_desc->flags & DP_TX_DESC_FLAG_ME)
			dp_tx_me_free_buf(tx_desc->pdev,
					  tx_desc->msdu_ext_desc->me_buffer);

		dp_tx_ext_desc_free(soc, tx_desc->msdu_ext_desc, desc_pool_id);
	}

	if (tx_desc->flags & DP_TX_DESC_FLAG_TO_FW)
		qdf_atomic_dec(&soc->num_tx_exception);

	if (HAL_TX_COMP_RELEASE_SOURCE_TQM ==
				tx_desc->buffer_src)
		comp_status = hal_tx_comp_get_release_reason(&tx_desc->comp,
							     soc->hal_soc);
	else
		comp_status = HAL_TX_COMP_RELEASE_REASON_FW;

	dp_tx_debug("Tx Completion Release desc %d status %d outstanding %d",
		    tx_desc->id, comp_status,
		    qdf_atomic_read(&pdev->num_tx_outstanding));

	dp_tx_desc_free(soc, tx_desc, desc_pool_id);
	return;
}

/**
 * dp_tx_htt_metadata_prepare() - Prepare HTT metadata for special frames
 * @vdev: DP vdev Handle
 * @nbuf: skb
 * @msdu_info: msdu_info required to create HTT metadata
 *
 * Prepares and fills HTT metadata in the frame pre-header for special frames
 * that should be transmitted using varying transmit parameters.
 * There are 2 VDEV modes that currently needs this special metadata -
 *  1) Mesh Mode
 *  2) DSRC Mode
 *
 * Return: HTT metadata size
 *
 */
static uint8_t dp_tx_prepare_htt_metadata(struct dp_vdev *vdev, qdf_nbuf_t nbuf,
					  struct dp_tx_msdu_info_s *msdu_info)
{
	uint32_t *meta_data = msdu_info->meta_data;
	struct htt_tx_msdu_desc_ext2_t *desc_ext =
				(struct htt_tx_msdu_desc_ext2_t *) meta_data;

	uint8_t htt_desc_size;

	/* Size rounded of multiple of 8 bytes */
	uint8_t htt_desc_size_aligned;

	uint8_t *hdr = NULL;

	/*
	 * Metadata - HTT MSDU Extension header
	 */
	htt_desc_size = sizeof(struct htt_tx_msdu_desc_ext2_t);
	htt_desc_size_aligned = (htt_desc_size + 7) & ~0x7;

	if (vdev->mesh_vdev || msdu_info->is_tx_sniffer ||
	    HTT_TX_MSDU_EXT2_DESC_FLAG_VALID_KEY_FLAGS_GET(msdu_info->
							   meta_data[0])) {
		if (qdf_unlikely(qdf_nbuf_headroom(nbuf) <
				 htt_desc_size_aligned)) {
			nbuf = qdf_nbuf_realloc_headroom(nbuf,
							 htt_desc_size_aligned);
			if (!nbuf) {
				/*
				 * qdf_nbuf_realloc_headroom won't do skb_clone
				 * as skb_realloc_headroom does. so, no free is
				 * needed here.
				 */
				DP_STATS_INC(vdev,
					     tx_i.dropped.headroom_insufficient,
					     1);
				qdf_print(" %s[%d] skb_realloc_headroom failed",
					  __func__, __LINE__);
				return 0;
			}
		}
		/* Fill and add HTT metaheader */
		hdr = qdf_nbuf_push_head(nbuf, htt_desc_size_aligned);
		if (!hdr) {
			dp_tx_err("Error in filling HTT metadata");

			return 0;
		}
		qdf_mem_copy(hdr, desc_ext, htt_desc_size);

	} else if (vdev->opmode == wlan_op_mode_ocb) {
		/* Todo - Add support for DSRC */
	}

	return htt_desc_size_aligned;
}

/**
 * dp_tx_prepare_tso_ext_desc() - Prepare MSDU extension descriptor for TSO
 * @tso_seg: TSO segment to process
 * @ext_desc: Pointer to MSDU extension descriptor
 *
 * Return: void
 */
#if defined(FEATURE_TSO)
static void dp_tx_prepare_tso_ext_desc(struct qdf_tso_seg_t *tso_seg,
		void *ext_desc)
{
	uint8_t num_frag;
	uint32_t tso_flags;

	/*
	 * Set tso_en, tcp_flags(NS, CWR, ECE, URG, ACK, PSH, RST, SYN, FIN),
	 * tcp_flag_mask
	 *
	 * Checksum enable flags are set in TCL descriptor and not in Extension
	 * Descriptor (H/W ignores checksum_en flags in MSDU ext descriptor)
	 */
	tso_flags = *(uint32_t *) &tso_seg->tso_flags;

	hal_tx_ext_desc_set_tso_flags(ext_desc, tso_flags);

	hal_tx_ext_desc_set_msdu_length(ext_desc, tso_seg->tso_flags.l2_len,
		tso_seg->tso_flags.ip_len);

	hal_tx_ext_desc_set_tcp_seq(ext_desc, tso_seg->tso_flags.tcp_seq_num);
	hal_tx_ext_desc_set_ip_id(ext_desc, tso_seg->tso_flags.ip_id);

	for (num_frag = 0; num_frag < tso_seg->num_frags; num_frag++) {
		uint32_t lo = 0;
		uint32_t hi = 0;

		qdf_assert_always((tso_seg->tso_frags[num_frag].paddr) &&
				  (tso_seg->tso_frags[num_frag].length));

		qdf_dmaaddr_to_32s(
			tso_seg->tso_frags[num_frag].paddr, &lo, &hi);
		hal_tx_ext_desc_set_buffer(ext_desc, num_frag, lo, hi,
			tso_seg->tso_frags[num_frag].length);
	}

	return;
}
#else
static void dp_tx_prepare_tso_ext_desc(struct qdf_tso_seg_t *tso_seg,
		void *ext_desc)
{
	return;
}
#endif

#if defined(FEATURE_TSO)
/**
 * dp_tx_free_tso_seg_list() - Loop through the tso segments
 *                             allocated and free them
 *
 * @soc: soc handle
 * @free_seg: list of tso segments
 * @msdu_info: msdu descriptor
 *
 * Return - void
 */
static void dp_tx_free_tso_seg_list(
		struct dp_soc *soc,
		struct qdf_tso_seg_elem_t *free_seg,
		struct dp_tx_msdu_info_s *msdu_info)
{
	struct qdf_tso_seg_elem_t *next_seg;

	while (free_seg) {
		next_seg = free_seg->next;
		dp_tx_tso_desc_free(soc,
				    msdu_info->tx_queue.desc_pool_id,
				    free_seg);
		free_seg = next_seg;
	}
}

/**
 * dp_tx_free_tso_num_seg_list() - Loop through the tso num segments
 *                                 allocated and free them
 *
 * @soc:  soc handle
 * @free_num_seg: list of tso number segments
 * @msdu_info: msdu descriptor
 * Return - void
 */
static void dp_tx_free_tso_num_seg_list(
		struct dp_soc *soc,
		struct qdf_tso_num_seg_elem_t *free_num_seg,
		struct dp_tx_msdu_info_s *msdu_info)
{
	struct qdf_tso_num_seg_elem_t *next_num_seg;

	while (free_num_seg) {
		next_num_seg = free_num_seg->next;
		dp_tso_num_seg_free(soc,
				    msdu_info->tx_queue.desc_pool_id,
				    free_num_seg);
		free_num_seg = next_num_seg;
	}
}

/**
 * dp_tx_unmap_tso_seg_list() - Loop through the tso segments
 *                              do dma unmap for each segment
 *
 * @soc: soc handle
 * @free_seg: list of tso segments
 * @num_seg_desc: tso number segment descriptor
 *
 * Return - void
 */
static void dp_tx_unmap_tso_seg_list(
		struct dp_soc *soc,
		struct qdf_tso_seg_elem_t *free_seg,
		struct qdf_tso_num_seg_elem_t *num_seg_desc)
{
	struct qdf_tso_seg_elem_t *next_seg;

	if (qdf_unlikely(!num_seg_desc)) {
		DP_TRACE(ERROR, "TSO number seg desc is NULL!");
		return;
	}

	while (free_seg) {
		next_seg = free_seg->next;
		dp_tx_tso_unmap_segment(soc, free_seg, num_seg_desc);
		free_seg = next_seg;
	}
}

#ifdef FEATURE_TSO_STATS
/**
 * dp_tso_get_stats_idx: Retrieve the tso packet id
 * @pdev - pdev handle
 *
 * Return: id
 */
static uint32_t dp_tso_get_stats_idx(struct dp_pdev *pdev)
{
	uint32_t stats_idx;

	stats_idx = (((uint32_t)qdf_atomic_inc_return(&pdev->tso_idx))
						% CDP_MAX_TSO_PACKETS);
	return stats_idx;
}
#else
static int dp_tso_get_stats_idx(struct dp_pdev *pdev)
{
	return 0;
}
#endif /* FEATURE_TSO_STATS */

/**
 * dp_tx_free_remaining_tso_desc() - do dma unmap for tso segments if any,
 *				     free the tso segments descriptor and
 *				     tso num segments descriptor
 *
 * @soc:  soc handle
 * @msdu_info: msdu descriptor
 * @tso_seg_unmap: flag to show if dma unmap is necessary
 *
 * Return - void
 */
static void dp_tx_free_remaining_tso_desc(struct dp_soc *soc,
					  struct dp_tx_msdu_info_s *msdu_info,
					  bool tso_seg_unmap)
{
	struct qdf_tso_info_t *tso_info = &msdu_info->u.tso_info;
	struct qdf_tso_seg_elem_t *free_seg = tso_info->tso_seg_list;
	struct qdf_tso_num_seg_elem_t *tso_num_desc =
					tso_info->tso_num_seg_list;

	/* do dma unmap for each segment */
	if (tso_seg_unmap)
		dp_tx_unmap_tso_seg_list(soc, free_seg, tso_num_desc);

	/* free all tso number segment descriptor though looks only have 1 */
	dp_tx_free_tso_num_seg_list(soc, tso_num_desc, msdu_info);

	/* free all tso segment descriptor */
	dp_tx_free_tso_seg_list(soc, free_seg, msdu_info);
}

/**
 * dp_tx_prepare_tso() - Given a jumbo msdu, prepare the TSO info
 * @vdev: virtual device handle
 * @msdu: network buffer
 * @msdu_info: meta data associated with the msdu
 *
 * Return: QDF_STATUS_SUCCESS success
 */
static QDF_STATUS dp_tx_prepare_tso(struct dp_vdev *vdev,
		qdf_nbuf_t msdu, struct dp_tx_msdu_info_s *msdu_info)
{
	struct qdf_tso_seg_elem_t *tso_seg;
	int num_seg = qdf_nbuf_get_tso_num_seg(msdu);
	struct dp_soc *soc = vdev->pdev->soc;
	struct dp_pdev *pdev = vdev->pdev;
	struct qdf_tso_info_t *tso_info;
	struct qdf_tso_num_seg_elem_t *tso_num_seg;
	tso_info = &msdu_info->u.tso_info;
	tso_info->curr_seg = NULL;
	tso_info->tso_seg_list = NULL;
	tso_info->num_segs = num_seg;
	msdu_info->frm_type = dp_tx_frm_tso;
	tso_info->tso_num_seg_list = NULL;

	TSO_DEBUG(" %s: num_seg: %d", __func__, num_seg);

	while (num_seg) {
		tso_seg = dp_tx_tso_desc_alloc(
				soc, msdu_info->tx_queue.desc_pool_id);
		if (tso_seg) {
			tso_seg->next = tso_info->tso_seg_list;
			tso_info->tso_seg_list = tso_seg;
			num_seg--;
		} else {
			dp_err_rl("Failed to alloc tso seg desc");
			DP_STATS_INC_PKT(vdev->pdev,
					 tso_stats.tso_no_mem_dropped, 1,
					 qdf_nbuf_len(msdu));
			dp_tx_free_remaining_tso_desc(soc, msdu_info, false);

			return QDF_STATUS_E_NOMEM;
		}
	}

	TSO_DEBUG(" %s: num_seg: %d", __func__, num_seg);

	tso_num_seg = dp_tso_num_seg_alloc(soc,
			msdu_info->tx_queue.desc_pool_id);

	if (tso_num_seg) {
		tso_num_seg->next = tso_info->tso_num_seg_list;
		tso_info->tso_num_seg_list = tso_num_seg;
	} else {
		DP_TRACE(ERROR, "%s: Failed to alloc - Number of segs desc",
			 __func__);
		dp_tx_free_remaining_tso_desc(soc, msdu_info, false);

		return QDF_STATUS_E_NOMEM;
	}

	msdu_info->num_seg =
		qdf_nbuf_get_tso_info(soc->osdev, msdu, tso_info);

	TSO_DEBUG(" %s: msdu_info->num_seg: %d", __func__,
			msdu_info->num_seg);

	if (!(msdu_info->num_seg)) {
		/*
		 * Free allocated TSO seg desc and number seg desc,
		 * do unmap for segments if dma map has done.
		 */
		DP_TRACE(ERROR, "%s: Failed to get tso info", __func__);
		dp_tx_free_remaining_tso_desc(soc, msdu_info, true);

		return QDF_STATUS_E_INVAL;
	}
	dp_tx_tso_history_add(soc, msdu_info->u.tso_info,
			      msdu, 0, DP_TX_DESC_MAP);

	tso_info->curr_seg = tso_info->tso_seg_list;

	tso_info->msdu_stats_idx = dp_tso_get_stats_idx(pdev);
	dp_tso_packet_update(pdev, tso_info->msdu_stats_idx,
			     msdu, msdu_info->num_seg);
	dp_tso_segment_stats_update(pdev, tso_info->tso_seg_list,
				    tso_info->msdu_stats_idx);
	dp_stats_tso_segment_histogram_update(pdev, msdu_info->num_seg);
	return QDF_STATUS_SUCCESS;
}
#else
static QDF_STATUS dp_tx_prepare_tso(struct dp_vdev *vdev,
		qdf_nbuf_t msdu, struct dp_tx_msdu_info_s *msdu_info)
{
	return QDF_STATUS_E_NOMEM;
}
#endif

QDF_COMPILE_TIME_ASSERT(dp_tx_htt_metadata_len_check,
			(DP_TX_MSDU_INFO_META_DATA_DWORDS * 4 >=
			 sizeof(struct htt_tx_msdu_desc_ext2_t)));

/**
 * dp_tx_prepare_ext_desc() - Allocate and prepare MSDU extension descriptor
 * @vdev: DP Vdev handle
 * @msdu_info: MSDU info to be setup in MSDU extension descriptor
 * @desc_pool_id: Descriptor Pool ID
 *
 * Return:
 */
static
struct dp_tx_ext_desc_elem_s *dp_tx_prepare_ext_desc(struct dp_vdev *vdev,
		struct dp_tx_msdu_info_s *msdu_info, uint8_t desc_pool_id)
{
	uint8_t i;
	uint8_t cached_ext_desc[HAL_TX_EXT_DESC_WITH_META_DATA];
	struct dp_tx_seg_info_s *seg_info;
	struct dp_tx_ext_desc_elem_s *msdu_ext_desc;
	struct dp_soc *soc = vdev->pdev->soc;

	/* Allocate an extension descriptor */
	msdu_ext_desc = dp_tx_ext_desc_alloc(soc, desc_pool_id);
	qdf_mem_zero(&cached_ext_desc[0], HAL_TX_EXT_DESC_WITH_META_DATA);

	if (!msdu_ext_desc) {
		DP_STATS_INC(vdev, tx_i.dropped.desc_na.num, 1);
		return NULL;
	}

	if (msdu_info->exception_fw &&
			qdf_unlikely(vdev->mesh_vdev)) {
		qdf_mem_copy(&cached_ext_desc[HAL_TX_EXTENSION_DESC_LEN_BYTES],
				&msdu_info->meta_data[0],
				sizeof(struct htt_tx_msdu_desc_ext2_t));
		qdf_atomic_inc(&soc->num_tx_exception);
		msdu_ext_desc->flags |= DP_TX_EXT_DESC_FLAG_METADATA_VALID;
	}

	switch (msdu_info->frm_type) {
	case dp_tx_frm_sg:
	case dp_tx_frm_me:
	case dp_tx_frm_raw:
		seg_info = msdu_info->u.sg_info.curr_seg;
		/* Update the buffer pointers in MSDU Extension Descriptor */
		for (i = 0; i < seg_info->frag_cnt; i++) {
			hal_tx_ext_desc_set_buffer(&cached_ext_desc[0], i,
				seg_info->frags[i].paddr_lo,
				seg_info->frags[i].paddr_hi,
				seg_info->frags[i].len);
		}

		break;

	case dp_tx_frm_tso:
		dp_tx_prepare_tso_ext_desc(&msdu_info->u.tso_info.curr_seg->seg,
				&cached_ext_desc[0]);
		break;


	default:
		break;
	}

	QDF_TRACE_HEX_DUMP(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_DEBUG,
			   cached_ext_desc, HAL_TX_EXT_DESC_WITH_META_DATA);

	hal_tx_ext_desc_sync(&cached_ext_desc[0],
			msdu_ext_desc->vaddr);

	return msdu_ext_desc;
}

/**
 * dp_tx_trace_pkt() - Trace TX packet at DP layer
 *
 * @skb: skb to be traced
 * @msdu_id: msdu_id of the packet
 * @vdev_id: vdev_id of the packet
 *
 * Return: None
 */
#ifdef DP_DISABLE_TX_PKT_TRACE
static void dp_tx_trace_pkt(struct dp_soc *soc,
			    qdf_nbuf_t skb, uint16_t msdu_id,
			    uint8_t vdev_id)
{
}
#else
static void dp_tx_trace_pkt(struct dp_soc *soc,
			    qdf_nbuf_t skb, uint16_t msdu_id,
			    uint8_t vdev_id)
{
	if (dp_is_tput_high(soc))
		return;

	QDF_NBUF_CB_TX_PACKET_TRACK(skb) = QDF_NBUF_TX_PKT_DATA_TRACK;
	QDF_NBUF_CB_TX_DP_TRACE(skb) = 1;
	DPTRACE(qdf_dp_trace_ptr(skb,
				 QDF_DP_TRACE_LI_DP_TX_PACKET_PTR_RECORD,
				 QDF_TRACE_DEFAULT_PDEV_ID,
				 qdf_nbuf_data_addr(skb),
				 sizeof(qdf_nbuf_data(skb)),
				 msdu_id, vdev_id, 0));

	qdf_dp_trace_log_pkt(vdev_id, skb, QDF_TX, QDF_TRACE_DEFAULT_PDEV_ID);

	DPTRACE(qdf_dp_trace_data_pkt(skb, QDF_TRACE_DEFAULT_PDEV_ID,
				      QDF_DP_TRACE_LI_DP_TX_PACKET_RECORD,
				      msdu_id, QDF_TX));
}
#endif

#ifdef WLAN_DP_FEATURE_MARK_ICMP_REQ_TO_FW
/**
 * dp_tx_is_nbuf_marked_exception() - Check if the packet has been marked as
 *				      exception by the upper layer (OS_IF)
 * @soc: DP soc handle
 * @nbuf: packet to be transmitted
 *
 * Returns: 1 if the packet is marked as exception,
 *	    0, if the packet is not marked as exception.
 */
static inline int dp_tx_is_nbuf_marked_exception(struct dp_soc *soc,
						 qdf_nbuf_t nbuf)
{
	return QDF_NBUF_CB_TX_PACKET_TO_FW(nbuf);
}
#else
static inline int dp_tx_is_nbuf_marked_exception(struct dp_soc *soc,
						 qdf_nbuf_t nbuf)
{
	return 0;
}
#endif

/**
 * dp_tx_desc_prepare_single - Allocate and prepare Tx descriptor
 * @vdev: DP vdev handle
 * @nbuf: skb
 * @desc_pool_id: Descriptor pool ID
 * @meta_data: Metadata to the fw
 * @tx_exc_metadata: Handle that holds exception path metadata
 * Allocate and prepare Tx descriptor with msdu information.
 *
 * Return: Pointer to Tx Descriptor on success,
 *         NULL on failure
 */
static
struct dp_tx_desc_s *dp_tx_prepare_desc_single(struct dp_vdev *vdev,
		qdf_nbuf_t nbuf, uint8_t desc_pool_id,
		struct dp_tx_msdu_info_s *msdu_info,
		struct cdp_tx_exception_metadata *tx_exc_metadata)
{
	uint8_t align_pad;
	uint8_t is_exception = 0;
	uint8_t htt_hdr_size;
	struct dp_tx_desc_s *tx_desc;
	struct dp_pdev *pdev = vdev->pdev;
	struct dp_soc *soc = pdev->soc;

	if (dp_tx_limit_check(vdev))
		return NULL;

	/* Allocate software Tx descriptor */
	tx_desc = dp_tx_desc_alloc(soc, desc_pool_id);

	if (qdf_unlikely(!tx_desc)) {
		DP_STATS_INC(vdev, tx_i.dropped.desc_na.num, 1);
		DP_STATS_INC(vdev, tx_i.dropped.desc_na_exc_alloc_fail.num, 1);
		return NULL;
	}

	dp_tx_outstanding_inc(pdev);

	/* Initialize the SW tx descriptor */
	tx_desc->nbuf = nbuf;
	tx_desc->frm_type = dp_tx_frm_std;
	tx_desc->tx_encap_type = ((tx_exc_metadata &&
		(tx_exc_metadata->tx_encap_type != CDP_INVALID_TX_ENCAP_TYPE)) ?
		tx_exc_metadata->tx_encap_type : vdev->tx_encap_type);
	tx_desc->vdev_id = vdev->vdev_id;
	tx_desc->pdev = pdev;
	tx_desc->msdu_ext_desc = NULL;
	tx_desc->pkt_offset = 0;
	tx_desc->length = qdf_nbuf_headlen(nbuf);

	dp_tx_trace_pkt(soc, nbuf, tx_desc->id, vdev->vdev_id);

	if (qdf_unlikely(vdev->multipass_en)) {
		if (!dp_tx_multipass_process(soc, vdev, nbuf, msdu_info))
			goto failure;
	}

	/* Packets marked by upper layer (OS-IF) to be sent to FW */
	if (dp_tx_is_nbuf_marked_exception(soc, nbuf))
		is_exception = 1;
	/*
	 * For special modes (vdev_type == ocb or mesh), data frames should be
	 * transmitted using varying transmit parameters (tx spec) which include
	 * transmit rate, power, priority, channel, channel bandwidth , nss etc.
	 * These are filled in HTT MSDU descriptor and sent in frame pre-header.
	 * These frames are sent as exception packets to firmware.
	 *
	 * HW requirement is that metadata should always point to a
	 * 8-byte aligned address. So we add alignment pad to start of buffer.
	 *  HTT Metadata should be ensured to be multiple of 8-bytes,
	 *  to get 8-byte aligned start address along with align_pad added
	 *
	 *  |-----------------------------|
	 *  |                             |
	 *  |-----------------------------| <-----Buffer Pointer Address given
	 *  |                             |  ^    in HW descriptor (aligned)
	 *  |       HTT Metadata          |  |
	 *  |                             |  |
	 *  |                             |  | Packet Offset given in descriptor
	 *  |                             |  |
	 *  |-----------------------------|  |
	 *  |       Alignment Pad         |  v
	 *  |-----------------------------| <----- Actual buffer start address
	 *  |        SKB Data             |           (Unaligned)
	 *  |                             |
	 *  |                             |
	 *  |                             |
	 *  |                             |
	 *  |                             |
	 *  |-----------------------------|
	 */
	if (qdf_unlikely((msdu_info->exception_fw)) ||
				(vdev->opmode == wlan_op_mode_ocb) ||
				(tx_exc_metadata &&
				tx_exc_metadata->is_tx_sniffer)) {
		align_pad = ((unsigned long) qdf_nbuf_data(nbuf)) & 0x7;

		if (qdf_unlikely(qdf_nbuf_headroom(nbuf) < align_pad)) {
			DP_STATS_INC(vdev,
				     tx_i.dropped.headroom_insufficient, 1);
			goto failure;
		}

		if (qdf_nbuf_push_head(nbuf, align_pad) == NULL) {
			dp_tx_err("qdf_nbuf_push_head failed");
			goto failure;
		}

		htt_hdr_size = dp_tx_prepare_htt_metadata(vdev, nbuf,
				msdu_info);
		if (htt_hdr_size == 0)
			goto failure;

		tx_desc->length = qdf_nbuf_headlen(nbuf);
		tx_desc->pkt_offset = align_pad + htt_hdr_size;
		tx_desc->flags |= DP_TX_DESC_FLAG_TO_FW;
		is_exception = 1;
		tx_desc->length -= tx_desc->pkt_offset;
	}

#if !TQM_BYPASS_WAR
	if (is_exception || tx_exc_metadata)
#endif
	{
		/* Temporary WAR due to TQM VP issues */
		tx_desc->flags |= DP_TX_DESC_FLAG_TO_FW;
		qdf_atomic_inc(&soc->num_tx_exception);
	}

	return tx_desc;

failure:
	dp_tx_desc_release(tx_desc, desc_pool_id);
	return NULL;
}

/**
 * dp_tx_prepare_desc() - Allocate and prepare Tx descriptor for multisegment frame
 * @vdev: DP vdev handle
 * @nbuf: skb
 * @msdu_info: Info to be setup in MSDU descriptor and MSDU extension descriptor
 * @desc_pool_id : Descriptor Pool ID
 *
 * Allocate and prepare Tx descriptor with msdu and fragment descritor
 * information. For frames wth fragments, allocate and prepare
 * an MSDU extension descriptor
 *
 * Return: Pointer to Tx Descriptor on success,
 *         NULL on failure
 */
static struct dp_tx_desc_s *dp_tx_prepare_desc(struct dp_vdev *vdev,
		qdf_nbuf_t nbuf, struct dp_tx_msdu_info_s *msdu_info,
		uint8_t desc_pool_id)
{
	struct dp_tx_desc_s *tx_desc;
	struct dp_tx_ext_desc_elem_s *msdu_ext_desc;
	struct dp_pdev *pdev = vdev->pdev;
	struct dp_soc *soc = pdev->soc;

	if (dp_tx_limit_check(vdev))
		return NULL;

	/* Allocate software Tx descriptor */
	tx_desc = dp_tx_desc_alloc(soc, desc_pool_id);
	if (!tx_desc) {
		DP_STATS_INC(vdev, tx_i.dropped.desc_na.num, 1);
		return NULL;
	}
	dp_tx_tso_seg_history_add(soc, msdu_info->u.tso_info.curr_seg,
				  nbuf, tx_desc->id, DP_TX_DESC_COOKIE);

	dp_tx_outstanding_inc(pdev);

	/* Initialize the SW tx descriptor */
	tx_desc->nbuf = nbuf;
	tx_desc->frm_type = msdu_info->frm_type;
	tx_desc->tx_encap_type = vdev->tx_encap_type;
	tx_desc->vdev_id = vdev->vdev_id;
	tx_desc->pdev = pdev;
	tx_desc->pkt_offset = 0;

	dp_tx_trace_pkt(soc, nbuf, tx_desc->id, vdev->vdev_id);

	/* Handle scattered frames - TSO/SG/ME */
	/* Allocate and prepare an extension descriptor for scattered frames */
	msdu_ext_desc = dp_tx_prepare_ext_desc(vdev, msdu_info, desc_pool_id);
	if (!msdu_ext_desc) {
		dp_tx_info("Tx Extension Descriptor Alloc Fail");
		goto failure;
	}

#if TQM_BYPASS_WAR
	/* Temporary WAR due to TQM VP issues */
	tx_desc->flags |= DP_TX_DESC_FLAG_TO_FW;
	qdf_atomic_inc(&soc->num_tx_exception);
#endif
	if (qdf_unlikely(msdu_info->exception_fw))
		tx_desc->flags |= DP_TX_DESC_FLAG_TO_FW;

	tx_desc->msdu_ext_desc = msdu_ext_desc;
	tx_desc->flags |= DP_TX_DESC_FLAG_FRAG;

	msdu_ext_desc->tso_desc = msdu_info->u.tso_info.curr_seg;
	msdu_ext_desc->tso_num_desc = msdu_info->u.tso_info.tso_num_seg_list;

	tx_desc->dma_addr = msdu_ext_desc->paddr;

	if (msdu_ext_desc->flags & DP_TX_EXT_DESC_FLAG_METADATA_VALID)
		tx_desc->length = HAL_TX_EXT_DESC_WITH_META_DATA;
	else
		tx_desc->length = HAL_TX_EXTENSION_DESC_LEN_BYTES;

	return tx_desc;
failure:
	dp_tx_desc_release(tx_desc, desc_pool_id);
	return NULL;
}

/**
 * dp_tx_prepare_raw() - Prepare RAW packet TX
 * @vdev: DP vdev handle
 * @nbuf: buffer pointer
 * @seg_info: Pointer to Segment info Descriptor to be prepared
 * @msdu_info: MSDU info to be setup in MSDU descriptor and MSDU extension
 *     descriptor
 *
 * Return:
 */
static qdf_nbuf_t dp_tx_prepare_raw(struct dp_vdev *vdev, qdf_nbuf_t nbuf,
	struct dp_tx_seg_info_s *seg_info, struct dp_tx_msdu_info_s *msdu_info)
{
	qdf_nbuf_t curr_nbuf = NULL;
	uint16_t total_len = 0;
	qdf_dma_addr_t paddr;
	int32_t i;
	int32_t mapped_buf_num = 0;

	struct dp_tx_sg_info_s *sg_info = &msdu_info->u.sg_info;
	qdf_dot3_qosframe_t *qos_wh = (qdf_dot3_qosframe_t *) nbuf->data;

	DP_STATS_INC_PKT(vdev, tx_i.raw.raw_pkt, 1, qdf_nbuf_len(nbuf));

	/* Continue only if frames are of DATA type */
	if (!DP_FRAME_IS_DATA(qos_wh)) {
		DP_STATS_INC(vdev, tx_i.raw.invalid_raw_pkt_datatype, 1);
		dp_tx_debug("Pkt. recd is of not data type");
		goto error;
	}
	/* SWAR for HW: Enable WEP bit in the AMSDU frames for RAW mode */
	if (vdev->raw_mode_war &&
	    (qos_wh->i_fc[0] & QDF_IEEE80211_FC0_SUBTYPE_QOS) &&
	    (qos_wh->i_qos[0] & IEEE80211_QOS_AMSDU))
		qos_wh->i_fc[1] |= IEEE80211_FC1_WEP;

	for (curr_nbuf = nbuf, i = 0; curr_nbuf;
			curr_nbuf = qdf_nbuf_next(curr_nbuf), i++) {
		/*
		 * Number of nbuf's must not exceed the size of the frags
		 * array in seg_info.
		 */
		if (i >= DP_TX_MAX_NUM_FRAGS) {
			dp_err_rl("nbuf cnt exceeds the max number of segs");
			DP_STATS_INC(vdev, tx_i.raw.num_frags_overflow_err, 1);
			goto error;
		}
		if (QDF_STATUS_SUCCESS !=
			qdf_nbuf_map_nbytes_single(vdev->osdev,
						   curr_nbuf,
						   QDF_DMA_TO_DEVICE,
						   curr_nbuf->len)) {
			dp_tx_err("%s dma map error ", __func__);
			DP_STATS_INC(vdev, tx_i.raw.dma_map_error, 1);
			goto error;
		}
		/* Update the count of mapped nbuf's */
		mapped_buf_num++;
		paddr = qdf_nbuf_get_frag_paddr(curr_nbuf, 0);
		seg_info->frags[i].paddr_lo = paddr;
		seg_info->frags[i].paddr_hi = ((uint64_t)paddr >> 32);
		seg_info->frags[i].len = qdf_nbuf_len(curr_nbuf);
		seg_info->frags[i].vaddr = (void *) curr_nbuf;
		total_len += qdf_nbuf_len(curr_nbuf);
	}

	seg_info->frag_cnt = i;
	seg_info->total_len = total_len;
	seg_info->next = NULL;

	sg_info->curr_seg = seg_info;

	msdu_info->frm_type = dp_tx_frm_raw;
	msdu_info->num_seg = 1;

	return nbuf;

error:
	i = 0;
	while (nbuf) {
		curr_nbuf = nbuf;
		if (i < mapped_buf_num) {
			qdf_nbuf_unmap_nbytes_single(vdev->osdev, curr_nbuf,
						     QDF_DMA_TO_DEVICE,
						     curr_nbuf->len);
			i++;
		}
		nbuf = qdf_nbuf_next(nbuf);
		qdf_nbuf_free(curr_nbuf);
	}
	return NULL;

}

/**
 * dp_tx_raw_prepare_unset() - unmap the chain of nbufs belonging to RAW frame.
 * @soc: DP soc handle
 * @nbuf: Buffer pointer
 *
 * unmap the chain of nbufs that belong to this RAW frame.
 *
 * Return: None
 */
static void dp_tx_raw_prepare_unset(struct dp_soc *soc,
				    qdf_nbuf_t nbuf)
{
	qdf_nbuf_t cur_nbuf = nbuf;

	do {
		qdf_nbuf_unmap_nbytes_single(soc->osdev, cur_nbuf,
					     QDF_DMA_TO_DEVICE,
					     cur_nbuf->len);
		cur_nbuf = qdf_nbuf_next(cur_nbuf);
	} while (cur_nbuf);
}

#ifdef VDEV_PEER_PROTOCOL_COUNT
void dp_vdev_peer_stats_update_protocol_cnt_tx(struct dp_vdev *vdev_hdl,
					       qdf_nbuf_t nbuf)
{
	qdf_nbuf_t nbuf_local;
	struct dp_vdev *vdev_local = vdev_hdl;

	do {
		if (qdf_likely(!((vdev_local)->peer_protocol_count_track)))
			break;
		nbuf_local = nbuf;
		if (qdf_unlikely(((vdev_local)->tx_encap_type) ==
			 htt_cmn_pkt_type_raw))
			break;
		else if (qdf_unlikely(qdf_nbuf_is_nonlinear((nbuf_local))))
			break;
		else if (qdf_nbuf_is_tso((nbuf_local)))
			break;
		dp_vdev_peer_stats_update_protocol_cnt((vdev_local),
						       (nbuf_local),
						       NULL, 1, 0);
	} while (0);
}
#endif

#ifdef WLAN_DP_FEATURE_SW_LATENCY_MGR
/**
 * dp_tx_update_stats() - Update soc level tx stats
 * @soc: DP soc handle
 * @tx_desc: TX descriptor reference
 * @ring_id: TCL ring id
 *
 * Returns: none
 */
void dp_tx_update_stats(struct dp_soc *soc,
			struct dp_tx_desc_s *tx_desc,
			uint8_t ring_id)
{
	uint32_t stats_len = 0;

	if (tx_desc->frm_type == dp_tx_frm_tso)
		stats_len  = tx_desc->msdu_ext_desc->tso_desc->seg.total_len;
	else
		stats_len = qdf_nbuf_len(tx_desc->nbuf);

	DP_STATS_INC_PKT(soc, tx.egress[ring_id], 1, stats_len);
}

int
dp_tx_attempt_coalescing(struct dp_soc *soc, struct dp_vdev *vdev,
			 struct dp_tx_desc_s *tx_desc,
			 uint8_t tid,
			 struct dp_tx_msdu_info_s *msdu_info,
			 uint8_t ring_id)
{
	struct dp_swlm *swlm = &soc->swlm;
	union swlm_data swlm_query_data;
	struct dp_swlm_tcl_data tcl_data;
	QDF_STATUS status;
	int ret;

	if (!swlm->is_enabled)
		return msdu_info->skip_hp_update;

	tcl_data.nbuf = tx_desc->nbuf;
	tcl_data.tid = tid;
	tcl_data.ring_id = ring_id;
	if (tx_desc->frm_type == dp_tx_frm_tso) {
		tcl_data.pkt_len  =
			tx_desc->msdu_ext_desc->tso_desc->seg.total_len;
	} else {
		tcl_data.pkt_len = qdf_nbuf_len(tx_desc->nbuf);
	}
	tcl_data.num_ll_connections = vdev->num_latency_critical_conn;
	swlm_query_data.tcl_data = &tcl_data;

	status = dp_swlm_tcl_pre_check(soc, &tcl_data);
	if (QDF_IS_STATUS_ERROR(status)) {
		dp_swlm_tcl_reset_session_data(soc, ring_id);
		DP_STATS_INC(swlm, tcl[ring_id].coalesce_fail, 1);
		return 0;
	}

	ret = dp_swlm_query_policy(soc, TCL_DATA, swlm_query_data);
	if (ret) {
		DP_STATS_INC(swlm, tcl[ring_id].coalesce_success, 1);
	} else {
		DP_STATS_INC(swlm, tcl[ring_id].coalesce_fail, 1);
	}

	return ret;
}

void
dp_tx_ring_access_end(struct dp_soc *soc, hal_ring_handle_t hal_ring_hdl,
		      int coalesce)
{
	if (coalesce)
		dp_tx_hal_ring_access_end_reap(soc, hal_ring_hdl);
	else
		dp_tx_hal_ring_access_end(soc, hal_ring_hdl);
}

static inline void
dp_tx_is_hp_update_required(uint32_t i, struct dp_tx_msdu_info_s *msdu_info)
{
	if (((i + 1) < msdu_info->num_seg))
		msdu_info->skip_hp_update = 1;
	else
		msdu_info->skip_hp_update = 0;
}

static inline void
dp_flush_tcp_hp(struct dp_soc *soc, uint8_t ring_id)
{
	hal_ring_handle_t hal_ring_hdl =
		dp_tx_get_hal_ring_hdl(soc, ring_id);

	if (dp_tx_hal_ring_access_start(soc, hal_ring_hdl)) {
		dp_err("Fillmore: SRNG access start failed");
		return;
	}

	dp_tx_ring_access_end_wrapper(soc, hal_ring_hdl, 0);
}

static inline void
dp_tx_check_and_flush_hp(struct dp_soc *soc,
			 QDF_STATUS status,
			 struct dp_tx_msdu_info_s *msdu_info)
{
	if (QDF_IS_STATUS_ERROR(status) && !msdu_info->skip_hp_update) {
		dp_flush_tcp_hp(soc,
			(msdu_info->tx_queue.ring_id & DP_TX_QUEUE_MASK));
	}
}
#else
static inline void
dp_tx_is_hp_update_required(uint32_t i, struct dp_tx_msdu_info_s *msdu_info)
{
}

static inline void
dp_tx_check_and_flush_hp(struct dp_soc *soc,
			 QDF_STATUS status,
			 struct dp_tx_msdu_info_s *msdu_info)
{
}
#endif

#ifdef FEATURE_RUNTIME_PM
static inline int dp_get_rtpm_tput_policy_requirement(struct dp_soc *soc)
{
	return qdf_atomic_read(&soc->rtpm_high_tput_flag);
}
/**
 * dp_tx_ring_access_end_wrapper() - Wrapper for ring access end
 * @soc: Datapath soc handle
 * @hal_ring_hdl: HAL ring handle
 * @coalesce: Coalesce the current write or not
 *
 * Wrapper for HAL ring access end for data transmission for
 * FEATURE_RUNTIME_PM
 *
 * Returns: none
 */
void
dp_tx_ring_access_end_wrapper(struct dp_soc *soc,
			      hal_ring_handle_t hal_ring_hdl,
			      int coalesce)
{
	int ret;

	/*
	 * Avoid runtime get and put APIs under high throughput scenarios.
	 */
	if (dp_get_rtpm_tput_policy_requirement(soc)) {
		dp_tx_ring_access_end(soc, hal_ring_hdl, coalesce);
		return;
	}

	ret = hif_pm_runtime_get(soc->hif_handle,
				 RTPM_ID_DW_TX_HW_ENQUEUE, true);
	switch (ret) {
	case 0:
		if (hif_system_pm_state_check(soc->hif_handle)) {
			dp_tx_hal_ring_access_end_reap(soc, hal_ring_hdl);
			hal_srng_set_event(hal_ring_hdl, HAL_SRNG_FLUSH_EVENT);
			hal_srng_inc_flush_cnt(hal_ring_hdl);
		} else {
			dp_tx_ring_access_end(soc, hal_ring_hdl, coalesce);
		}
		hif_pm_runtime_put(soc->hif_handle,
				   RTPM_ID_DW_TX_HW_ENQUEUE);
		break;
	/*
	 * If hif_pm_runtime_get returns -EBUSY or -EINPROGRESS,
	 * take the dp runtime refcount using dp_runtime_get,
	 * check link state,if up, write TX ring HP, else just set flush event.
	 * In dp_runtime_resume, wait until dp runtime refcount becomes
	 * zero or time out, then flush pending tx.
	 */
	case -EBUSY:
	case -EINPROGRESS:
		dp_runtime_get(soc);
		if (hif_pm_get_link_state(soc->hif_handle) ==
		    HIF_PM_LINK_STATE_UP) {
			dp_tx_ring_access_end(soc, hal_ring_hdl, coalesce);
		} else {
			dp_tx_hal_ring_access_end_reap(soc, hal_ring_hdl);
			hal_srng_set_event(hal_ring_hdl, HAL_SRNG_FLUSH_EVENT);
			qdf_atomic_inc(&soc->tx_pending_rtpm);
			hal_srng_inc_flush_cnt(hal_ring_hdl);
		}
		dp_runtime_put(soc);
		break;
	default:
		dp_runtime_get(soc);
		dp_tx_hal_ring_access_end_reap(soc, hal_ring_hdl);
		hal_srng_set_event(hal_ring_hdl, HAL_SRNG_FLUSH_EVENT);
		qdf_atomic_inc(&soc->tx_pending_rtpm);
		hal_srng_inc_flush_cnt(hal_ring_hdl);
		dp_runtime_put(soc);
	}
}
#else

#ifdef DP_POWER_SAVE
void
dp_tx_ring_access_end_wrapper(struct dp_soc *soc,
			      hal_ring_handle_t hal_ring_hdl,
			      int coalesce)
{
	if (hif_system_pm_state_check(soc->hif_handle)) {
		dp_tx_hal_ring_access_end_reap(soc, hal_ring_hdl);
		hal_srng_set_event(hal_ring_hdl, HAL_SRNG_FLUSH_EVENT);
		hal_srng_inc_flush_cnt(hal_ring_hdl);
	} else {
		dp_tx_ring_access_end(soc, hal_ring_hdl, coalesce);
	}
}
#endif

static inline int dp_get_rtpm_tput_policy_requirement(struct dp_soc *soc)
{
	return 0;
}
#endif

/**
 * dp_tx_get_tid() - Obtain TID to be used for this frame
 * @vdev: DP vdev handle
 * @nbuf: skb
 *
 * Extract the DSCP or PCP information from frame and map into TID value.
 *
 * Return: void
 */
static void dp_tx_get_tid(struct dp_vdev *vdev, qdf_nbuf_t nbuf,
			  struct dp_tx_msdu_info_s *msdu_info)
{
	uint8_t tos = 0, dscp_tid_override = 0;
	uint8_t *hdr_ptr, *L3datap;
	uint8_t is_mcast = 0;
	qdf_ether_header_t *eh = NULL;
	qdf_ethervlan_header_t *evh = NULL;
	uint16_t   ether_type;
	qdf_llc_t *llcHdr;
	struct dp_pdev *pdev = (struct dp_pdev *)vdev->pdev;

	DP_TX_TID_OVERRIDE(msdu_info, nbuf);
	if (qdf_likely(vdev->tx_encap_type != htt_cmn_pkt_type_raw)) {
		eh = (qdf_ether_header_t *)nbuf->data;
		hdr_ptr = (uint8_t *)(eh->ether_dhost);
		L3datap = hdr_ptr + sizeof(qdf_ether_header_t);
	} else {
		qdf_dot3_qosframe_t *qos_wh =
			(qdf_dot3_qosframe_t *) nbuf->data;
		msdu_info->tid = qos_wh->i_fc[0] & DP_FC0_SUBTYPE_QOS ?
			qos_wh->i_qos[0] & DP_QOS_TID : 0;
		return;
	}

	is_mcast = DP_FRAME_IS_MULTICAST(hdr_ptr);
	ether_type = eh->ether_type;

	llcHdr = (qdf_llc_t *)(nbuf->data + sizeof(qdf_ether_header_t));
	/*
	 * Check if packet is dot3 or eth2 type.
	 */
	if (DP_FRAME_IS_LLC(ether_type) && DP_FRAME_IS_SNAP(llcHdr)) {
		ether_type = (uint16_t)*(nbuf->data + 2*QDF_MAC_ADDR_SIZE +
				sizeof(*llcHdr));

		if (ether_type == htons(ETHERTYPE_VLAN)) {
			L3datap = hdr_ptr + sizeof(qdf_ethervlan_header_t) +
				sizeof(*llcHdr);
			ether_type = (uint16_t)*(nbuf->data + 2*QDF_MAC_ADDR_SIZE
					+ sizeof(*llcHdr) +
					sizeof(qdf_net_vlanhdr_t));
		} else {
			L3datap = hdr_ptr + sizeof(qdf_ether_header_t) +
				sizeof(*llcHdr);
		}
	} else {
		if (ether_type == htons(ETHERTYPE_VLAN)) {
			evh = (qdf_ethervlan_header_t *) eh;
			ether_type = evh->ether_type;
			L3datap = hdr_ptr + sizeof(qdf_ethervlan_header_t);
		}
	}

	/*
	 * Find priority from IP TOS DSCP field
	 */
	if (qdf_nbuf_is_ipv4_pkt(nbuf)) {
		qdf_net_iphdr_t *ip = (qdf_net_iphdr_t *) L3datap;
		if (qdf_nbuf_is_ipv4_dhcp_pkt(nbuf)) {
			/* Only for unicast frames */
			if (!is_mcast) {
				/* send it on VO queue */
				msdu_info->tid = DP_VO_TID;
			}
		} else {
			/*
			 * IP frame: exclude ECN bits 0-1 and map DSCP bits 2-7
			 * from TOS byte.
			 */
			tos = ip->ip_tos;
			dscp_tid_override = 1;

		}
	} else if (qdf_nbuf_is_ipv6_pkt(nbuf)) {
		/* TODO
		 * use flowlabel
		 *igmpmld cases to be handled in phase 2
		 */
		unsigned long ver_pri_flowlabel;
		unsigned long pri;
		ver_pri_flowlabel = *(unsigned long *) L3datap;
		pri = (ntohl(ver_pri_flowlabel) & IPV6_FLOWINFO_PRIORITY) >>
			DP_IPV6_PRIORITY_SHIFT;
		tos = pri;
		dscp_tid_override = 1;
	} else if (qdf_nbuf_is_ipv4_eapol_pkt(nbuf))
		msdu_info->tid = DP_VO_TID;
	else if (qdf_nbuf_is_ipv4_arp_pkt(nbuf)) {
		/* Only for unicast frames */
		if (!is_mcast) {
			/* send ucast arp on VO queue */
			msdu_info->tid = DP_VO_TID;
		}
	}

	/*
	 * Assign all MCAST packets to BE
	 */
	if (qdf_unlikely(vdev->tx_encap_type != htt_cmn_pkt_type_raw)) {
		if (is_mcast) {
			tos = 0;
			dscp_tid_override = 1;
		}
	}

	if (dscp_tid_override == 1) {
		tos = (tos >> DP_IP_DSCP_SHIFT) & DP_IP_DSCP_MASK;
		msdu_info->tid = pdev->dscp_tid_map[vdev->dscp_tid_map_id][tos];
	}

	if (msdu_info->tid >= CDP_MAX_DATA_TIDS)
		msdu_info->tid = CDP_MAX_DATA_TIDS - 1;

	return;
}

/**
 * dp_tx_classify_tid() - Obtain TID to be used for this frame
 * @vdev: DP vdev handle
 * @nbuf: skb
 *
 * Software based TID classification is required when more than 2 DSCP-TID
 * mapping tables are needed.
 * Hardware supports 2 DSCP-TID mapping tables for HKv1 and 48 for HKv2.
 *
 * Return: void
 */
static inline void dp_tx_classify_tid(struct dp_vdev *vdev, qdf_nbuf_t nbuf,
				      struct dp_tx_msdu_info_s *msdu_info)
{
	DP_TX_TID_OVERRIDE(msdu_info, nbuf);

	/*
	 * skip_sw_tid_classification flag will set in below cases-
	 * 1. vdev->dscp_tid_map_id < pdev->soc->num_hw_dscp_tid_map
	 * 2. hlos_tid_override enabled for vdev
	 * 3. mesh mode enabled for vdev
	 */
	if (qdf_likely(vdev->skip_sw_tid_classification)) {
		/* Update tid in msdu_info from skb priority */
		if (qdf_unlikely(vdev->skip_sw_tid_classification
			& DP_TXRX_HLOS_TID_OVERRIDE_ENABLED)) {
			uint32_t tid = qdf_nbuf_get_priority(nbuf);

			if (tid == DP_TX_INVALID_QOS_TAG)
				return;

			msdu_info->tid = tid;
			return;
		}
		return;
	}

	dp_tx_get_tid(vdev, nbuf, msdu_info);
}

#ifdef FEATURE_WLAN_TDLS
/**
 * dp_tx_update_tdls_flags() - Update descriptor flags for TDLS frame
 * @soc: datapath SOC
 * @vdev: datapath vdev
 * @tx_desc: TX descriptor
 *
 * Return: None
 */
static void dp_tx_update_tdls_flags(struct dp_soc *soc,
				    struct dp_vdev *vdev,
				    struct dp_tx_desc_s *tx_desc)
{
	if (vdev) {
		if (vdev->is_tdls_frame) {
			tx_desc->flags |= DP_TX_DESC_FLAG_TDLS_FRAME;
			vdev->is_tdls_frame = false;
		}
	}
}

static uint8_t dp_htt_tx_comp_get_status(struct dp_soc *soc, char *htt_desc)
{
	uint8_t tx_status = HTT_TX_FW2WBM_TX_STATUS_MAX;

	switch (soc->arch_id) {
	case CDP_ARCH_TYPE_LI:
		tx_status = HTT_TX_WBM_COMPLETION_V2_TX_STATUS_GET(htt_desc[0]);
		break;

	case CDP_ARCH_TYPE_BE:
		tx_status = HTT_TX_WBM_COMPLETION_V3_TX_STATUS_GET(htt_desc[0]);
		break;

	default:
		dp_err("Incorrect CDP_ARCH %d", soc->arch_id);
		QDF_BUG(0);
	}

	return tx_status;
}

/**
 * dp_non_std_htt_tx_comp_free_buff() - Free the non std tx packet buffer
 * @soc: dp_soc handle
 * @tx_desc: TX descriptor
 * @vdev: datapath vdev handle
 *
 * Return: None
 */
static void dp_non_std_htt_tx_comp_free_buff(struct dp_soc *soc,
					 struct dp_tx_desc_s *tx_desc)
{
	uint8_t tx_status = 0;
	uint8_t htt_tx_status[HAL_TX_COMP_HTT_STATUS_LEN];

	qdf_nbuf_t nbuf = tx_desc->nbuf;
	struct dp_vdev *vdev = dp_vdev_get_ref_by_id(soc, tx_desc->vdev_id,
						     DP_MOD_ID_TDLS);

	if (qdf_unlikely(!vdev)) {
		dp_err_rl("vdev is null!");
		goto error;
	}

	hal_tx_comp_get_htt_desc(&tx_desc->comp, htt_tx_status);
	tx_status = dp_htt_tx_comp_get_status(soc, htt_tx_status);
	dp_debug("vdev_id: %d tx_status: %d", tx_desc->vdev_id, tx_status);

	if (vdev->tx_non_std_data_callback.func) {
		qdf_nbuf_set_next(nbuf, NULL);
		vdev->tx_non_std_data_callback.func(
				vdev->tx_non_std_data_callback.ctxt,
				nbuf, tx_status);
		dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_TDLS);
		return;
	} else {
		dp_err_rl("callback func is null");
	}

	dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_TDLS);
error:
	qdf_nbuf_unmap_single(soc->osdev, nbuf, QDF_DMA_TO_DEVICE);
	qdf_nbuf_free(nbuf);
}

/**
 * dp_tx_msdu_single_map() - do nbuf map
 * @vdev: DP vdev handle
 * @tx_desc: DP TX descriptor pointer
 * @nbuf: skb pointer
 *
 * For TDLS frame, use qdf_nbuf_map_single() to align with the unmap
 * operation done in other component.
 *
 * Return: QDF_STATUS
 */
static inline QDF_STATUS dp_tx_msdu_single_map(struct dp_vdev *vdev,
					       struct dp_tx_desc_s *tx_desc,
					       qdf_nbuf_t nbuf)
{
	if (qdf_likely(!(tx_desc->flags & DP_TX_DESC_FLAG_TDLS_FRAME)))
		return qdf_nbuf_map_nbytes_single(vdev->osdev,
						  nbuf,
						  QDF_DMA_TO_DEVICE,
						  nbuf->len);
	else
		return qdf_nbuf_map_single(vdev->osdev, nbuf,
					   QDF_DMA_TO_DEVICE);
}
#else
static inline void dp_tx_update_tdls_flags(struct dp_soc *soc,
					   struct dp_vdev *vdev,
					   struct dp_tx_desc_s *tx_desc)
{
}

static inline void dp_non_std_htt_tx_comp_free_buff(struct dp_soc *soc,
						struct dp_tx_desc_s *tx_desc)
{
}

static inline QDF_STATUS dp_tx_msdu_single_map(struct dp_vdev *vdev,
					       struct dp_tx_desc_s *tx_desc,
					       qdf_nbuf_t nbuf)
{
	return qdf_nbuf_map_nbytes_single(vdev->osdev,
					  nbuf,
					  QDF_DMA_TO_DEVICE,
					  nbuf->len);
}
#endif

#ifdef MESH_MODE_SUPPORT
/**
 * dp_tx_update_mesh_flags() - Update descriptor flags for mesh VAP
 * @soc: datapath SOC
 * @vdev: datapath vdev
 * @tx_desc: TX descriptor
 *
 * Return: None
 */
static inline void dp_tx_update_mesh_flags(struct dp_soc *soc,
					   struct dp_vdev *vdev,
					   struct dp_tx_desc_s *tx_desc)
{
	if (qdf_unlikely(vdev->mesh_vdev))
		tx_desc->flags |= DP_TX_DESC_FLAG_MESH_MODE;
}

/**
 * dp_mesh_tx_comp_free_buff() - Free the mesh tx packet buffer
 * @soc: dp_soc handle
 * @tx_desc: TX descriptor
 * @vdev: datapath vdev handle
 *
 * Return: None
 */
static inline void dp_mesh_tx_comp_free_buff(struct dp_soc *soc,
					     struct dp_tx_desc_s *tx_desc)
{
	qdf_nbuf_t nbuf = tx_desc->nbuf;
	struct dp_vdev *vdev = NULL;

	if (tx_desc->flags & DP_TX_DESC_FLAG_TO_FW) {
		qdf_nbuf_free(nbuf);
		DP_STATS_INC(vdev, tx_i.mesh.completion_fw, 1);
	} else {
		vdev = dp_vdev_get_ref_by_id(soc, tx_desc->vdev_id,
					     DP_MOD_ID_MESH);
		if (vdev && vdev->osif_tx_free_ext)
			vdev->osif_tx_free_ext((nbuf));
		else
			qdf_nbuf_free(nbuf);

		if (vdev)
			dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_MESH);
	}
}
#else
static inline void dp_tx_update_mesh_flags(struct dp_soc *soc,
					   struct dp_vdev *vdev,
					   struct dp_tx_desc_s *tx_desc)
{
}

static inline void dp_mesh_tx_comp_free_buff(struct dp_soc *soc,
					     struct dp_tx_desc_s *tx_desc)
{
}
#endif

/**
 * dp_tx_frame_is_drop() - checks if the packet is loopback
 * @vdev: DP vdev handle
 * @nbuf: skb
 *
 * Return: 1 if frame needs to be dropped else 0
 */
int dp_tx_frame_is_drop(struct dp_vdev *vdev, uint8_t *srcmac, uint8_t *dstmac)
{
	struct dp_pdev *pdev = NULL;
	struct dp_ast_entry *src_ast_entry = NULL;
	struct dp_ast_entry *dst_ast_entry = NULL;
	struct dp_soc *soc = NULL;

	qdf_assert(vdev);
	pdev = vdev->pdev;
	qdf_assert(pdev);
	soc = pdev->soc;

	dst_ast_entry = dp_peer_ast_hash_find_by_pdevid
				(soc, dstmac, vdev->pdev->pdev_id);

	src_ast_entry = dp_peer_ast_hash_find_by_pdevid
				(soc, srcmac, vdev->pdev->pdev_id);
	if (dst_ast_entry && src_ast_entry) {
		if (dst_ast_entry->peer_id ==
				src_ast_entry->peer_id)
			return 1;
	}

	return 0;
}

/**
 * dp_tx_send_msdu_single() - Setup descriptor and enqueue single MSDU to TCL
 * @vdev: DP vdev handle
 * @nbuf: skb
 * @tid: TID from HLOS for overriding default DSCP-TID mapping
 * @meta_data: Metadata to the fw
 * @tx_q: Tx queue to be used for this Tx frame
 * @peer_id: peer_id of the peer in case of NAWDS frames
 * @tx_exc_metadata: Handle that holds exception path metadata
 *
 * Return: NULL on success,
 *         nbuf when it fails to send
 */
qdf_nbuf_t
dp_tx_send_msdu_single(struct dp_vdev *vdev, qdf_nbuf_t nbuf,
		       struct dp_tx_msdu_info_s *msdu_info, uint16_t peer_id,
		       struct cdp_tx_exception_metadata *tx_exc_metadata)
{
	struct dp_pdev *pdev = vdev->pdev;
	struct dp_soc *soc = pdev->soc;
	struct dp_tx_desc_s *tx_desc;
	QDF_STATUS status;
	struct dp_tx_queue *tx_q = &(msdu_info->tx_queue);
	uint16_t htt_tcl_metadata = 0;
	enum cdp_tx_sw_drop drop_code = TX_MAX_DROP;
	uint8_t tid = msdu_info->tid;
	struct cdp_tid_tx_stats *tid_stats = NULL;

	/* Setup Tx descriptor for an MSDU, and MSDU extension descriptor */
	tx_desc = dp_tx_prepare_desc_single(vdev, nbuf, tx_q->desc_pool_id,
			msdu_info, tx_exc_metadata);
	if (!tx_desc) {
		dp_err_rl("Tx_desc prepare Fail vdev %pK queue %d",
			  vdev, tx_q->desc_pool_id);
		drop_code = TX_DESC_ERR;
		goto fail_return;
	}

	dp_tx_update_tdls_flags(soc, vdev, tx_desc);

	if (qdf_unlikely(peer_id == DP_INVALID_PEER)) {
		htt_tcl_metadata = vdev->htt_tcl_metadata;
		HTT_TX_TCL_METADATA_HOST_INSPECTED_SET(htt_tcl_metadata, 1);
	} else if (qdf_unlikely(peer_id != HTT_INVALID_PEER)) {
		HTT_TX_TCL_METADATA_TYPE_SET(htt_tcl_metadata,
				HTT_TCL_METADATA_TYPE_PEER_BASED);
		HTT_TX_TCL_METADATA_PEER_ID_SET(htt_tcl_metadata,
				peer_id);
	} else
		htt_tcl_metadata = vdev->htt_tcl_metadata;

	if (msdu_info->exception_fw)
		HTT_TX_TCL_METADATA_VALID_HTT_SET(htt_tcl_metadata, 1);

	dp_tx_desc_update_fast_comp_flag(soc, tx_desc,
					 !pdev->enhanced_stats_en);

	dp_tx_update_mesh_flags(soc, vdev, tx_desc);

	if (qdf_unlikely(QDF_STATUS_SUCCESS !=
			 dp_tx_msdu_single_map(vdev, tx_desc, nbuf))) {
		/* Handle failure */
		dp_err("qdf_nbuf_map failed");
		DP_STATS_INC(vdev, tx_i.dropped.dma_error, 1);
		drop_code = TX_DMA_MAP_ERR;
		goto release_desc;
	}

	tx_desc->dma_addr = qdf_nbuf_mapped_paddr_get(tx_desc->nbuf);
	dp_tx_desc_history_add(soc, tx_desc->dma_addr, nbuf,
			       tx_desc->id, DP_TX_DESC_MAP);
	/* Enqueue the Tx MSDU descriptor to HW for transmit */
	status = soc->arch_ops.tx_hw_enqueue(soc, vdev, tx_desc,
					     htt_tcl_metadata,
					     tx_exc_metadata, msdu_info);

	if (status != QDF_STATUS_SUCCESS) {
		dp_tx_err_rl("Tx_hw_enqueue Fail tx_desc %pK queue %d",
			     tx_desc, tx_q->ring_id);
		dp_tx_desc_history_add(soc, tx_desc->dma_addr, nbuf,
				       tx_desc->id, DP_TX_DESC_UNMAP);
		qdf_nbuf_unmap_nbytes_single(vdev->osdev, nbuf,
					     QDF_DMA_TO_DEVICE,
					     nbuf->len);
		drop_code = TX_HW_ENQUEUE;
		goto release_desc;
	}

	return NULL;

release_desc:
	dp_tx_desc_release(tx_desc, tx_q->desc_pool_id);

fail_return:
	dp_tx_get_tid(vdev, nbuf, msdu_info);
	tid_stats = &pdev->stats.tid_stats.
		    tid_tx_stats[tx_q->ring_id][tid];
	tid_stats->swdrop_cnt[drop_code]++;
	return nbuf;
}

/**
 * dp_tx_comp_free_buf() - Free nbuf associated with the Tx Descriptor
 * @soc: Soc handle
 * @desc: software Tx descriptor to be processed
 *
 * Return: none
 */
void dp_tx_comp_free_buf(struct dp_soc *soc, struct dp_tx_desc_s *desc)
{
	qdf_nbuf_t nbuf = desc->nbuf;
	enum dp_tx_event_type type = dp_tx_get_event_type(desc->flags);

	/* nbuf already freed in vdev detach path */
	if (!nbuf)
		return;

	/* If it is TDLS mgmt, don't unmap or free the frame */
	if (desc->flags & DP_TX_DESC_FLAG_TDLS_FRAME)
		return dp_non_std_htt_tx_comp_free_buff(soc, desc);

	/* 0 : MSDU buffer, 1 : MLE */
	if (desc->msdu_ext_desc) {
		/* TSO free */
		if (hal_tx_ext_desc_get_tso_enable(
					desc->msdu_ext_desc->vaddr)) {
			dp_tx_desc_history_add(soc, desc->dma_addr, desc->nbuf,
					       desc->id, DP_TX_COMP_MSDU_EXT);
			dp_tx_tso_seg_history_add(soc,
						  desc->msdu_ext_desc->tso_desc,
						  desc->nbuf, desc->id, type);
			/* unmap eash TSO seg before free the nbuf */
			dp_tx_tso_unmap_segment(soc,
						desc->msdu_ext_desc->tso_desc,
						desc->msdu_ext_desc->
						tso_num_desc);
			qdf_nbuf_free(nbuf);
			return;
		}
	}
	/* If it's ME frame, dont unmap the cloned nbuf's */
	if ((desc->flags & DP_TX_DESC_FLAG_ME) && qdf_nbuf_is_cloned(nbuf))
		goto nbuf_free;

	dp_tx_desc_history_add(soc, desc->dma_addr, desc->nbuf, desc->id, type);
	dp_tx_unmap(soc, desc);

	if (desc->flags & DP_TX_DESC_FLAG_MESH_MODE)
		return dp_mesh_tx_comp_free_buff(soc, desc);
nbuf_free:
	qdf_nbuf_free(nbuf);
}

/**
 * dp_tx_send_msdu_multiple() - Enqueue multiple MSDUs
 * @vdev: DP vdev handle
 * @nbuf: skb
 * @msdu_info: MSDU info to be setup in MSDU extension descriptor
 *
 * Prepare descriptors for multiple MSDUs (TSO segments) and enqueue to TCL
 *
 * Return: NULL on success,
 *         nbuf when it fails to send
 */
#if QDF_LOCK_STATS
noinline
#else
#endif
qdf_nbuf_t dp_tx_send_msdu_multiple(struct dp_vdev *vdev, qdf_nbuf_t nbuf,
				    struct dp_tx_msdu_info_s *msdu_info)
{
	uint32_t i;
	struct dp_pdev *pdev = vdev->pdev;
	struct dp_soc *soc = pdev->soc;
	struct dp_tx_desc_s *tx_desc;
	bool is_cce_classified = false;
	QDF_STATUS status;
	uint16_t htt_tcl_metadata = 0;
	struct dp_tx_queue *tx_q = &msdu_info->tx_queue;
	struct cdp_tid_tx_stats *tid_stats = NULL;
	uint8_t prep_desc_fail = 0, hw_enq_fail = 0;

	if (msdu_info->frm_type == dp_tx_frm_me)
		nbuf = msdu_info->u.sg_info.curr_seg->nbuf;

	i = 0;
	/* Print statement to track i and num_seg */
	/*
	 * For each segment (maps to 1 MSDU) , prepare software and hardware
	 * descriptors using information in msdu_info
	 */
	while (i < msdu_info->num_seg) {
		/*
		 * Setup Tx descriptor for an MSDU, and MSDU extension
		 * descriptor
		 */
		tx_desc = dp_tx_prepare_desc(vdev, nbuf, msdu_info,
				tx_q->desc_pool_id);

		if (!tx_desc) {
			if (msdu_info->frm_type == dp_tx_frm_me) {
				prep_desc_fail++;
				dp_tx_me_free_buf(pdev,
					(void *)(msdu_info->u.sg_info
						.curr_seg->frags[0].vaddr));
				if (prep_desc_fail == msdu_info->num_seg) {
					/*
					 * Unmap is needed only if descriptor
					 * preparation failed for all segments.
					 */
					qdf_nbuf_unmap(soc->osdev,
						       msdu_info->u.sg_info.
						       curr_seg->nbuf,
						       QDF_DMA_TO_DEVICE);
				}
				/*
				 * Free the nbuf for the current segment
				 * and make it point to the next in the list.
				 * For me, there are as many segments as there
				 * are no of clients.
				 */
				qdf_nbuf_free(msdu_info->u.sg_info
					      .curr_seg->nbuf);
				if (msdu_info->u.sg_info.curr_seg->next) {
					msdu_info->u.sg_info.curr_seg =
						msdu_info->u.sg_info
						.curr_seg->next;
					nbuf = msdu_info->u.sg_info
					       .curr_seg->nbuf;
				}
				i++;
				continue;
			}

			if (msdu_info->frm_type == dp_tx_frm_tso) {
				dp_tx_tso_seg_history_add(
						soc,
						msdu_info->u.tso_info.curr_seg,
						nbuf, 0, DP_TX_DESC_UNMAP);
				dp_tx_tso_unmap_segment(soc,
							msdu_info->u.tso_info.
							curr_seg,
							msdu_info->u.tso_info.
							tso_num_seg_list);

				if (msdu_info->u.tso_info.curr_seg->next) {
					msdu_info->u.tso_info.curr_seg =
					msdu_info->u.tso_info.curr_seg->next;
					i++;
					continue;
				}
			}

			goto done;
		}

		if (msdu_info->frm_type == dp_tx_frm_me) {
			tx_desc->msdu_ext_desc->me_buffer =
				(struct dp_tx_me_buf_t *)msdu_info->
				u.sg_info.curr_seg->frags[0].vaddr;
			tx_desc->flags |= DP_TX_DESC_FLAG_ME;
		}

		if (is_cce_classified)
			tx_desc->flags |= DP_TX_DESC_FLAG_TO_FW;

		htt_tcl_metadata = vdev->htt_tcl_metadata;
		if (msdu_info->exception_fw) {
			HTT_TX_TCL_METADATA_VALID_HTT_SET(htt_tcl_metadata, 1);
		}

		dp_tx_is_hp_update_required(i, msdu_info);

		/*
		 * For frames with multiple segments (TSO, ME), jump to next
		 * segment.
		 */
		if (msdu_info->frm_type == dp_tx_frm_tso) {
			if (msdu_info->u.tso_info.curr_seg->next) {
				msdu_info->u.tso_info.curr_seg =
					msdu_info->u.tso_info.curr_seg->next;

				/*
				 * If this is a jumbo nbuf, then increment the
				 * number of nbuf users for each additional
				 * segment of the msdu. This will ensure that
				 * the skb is freed only after receiving tx
				 * completion for all segments of an nbuf
				 */
				qdf_nbuf_inc_users(nbuf);

				/* Check with MCL if this is needed */
				/* nbuf = msdu_info->u.tso_info.curr_seg->nbuf;
				 */
			}
		}

		/*
		 * Enqueue the Tx MSDU descriptor to HW for transmit
		 */
		status = soc->arch_ops.tx_hw_enqueue(soc, vdev, tx_desc,
						     htt_tcl_metadata,
						     NULL, msdu_info);

		dp_tx_check_and_flush_hp(soc, status, msdu_info);

		if (status != QDF_STATUS_SUCCESS) {
			dp_info("Tx_hw_enqueue Fail tx_desc %pK queue %d",
				tx_desc, tx_q->ring_id);

			dp_tx_get_tid(vdev, nbuf, msdu_info);
			tid_stats = &pdev->stats.tid_stats.
				    tid_tx_stats[tx_q->ring_id][msdu_info->tid];
			tid_stats->swdrop_cnt[TX_HW_ENQUEUE]++;

			if (msdu_info->frm_type == dp_tx_frm_me) {
				hw_enq_fail++;
				if (hw_enq_fail == msdu_info->num_seg) {
					/*
					 * Unmap is needed only if enqueue
					 * failed for all segments.
					 */
					qdf_nbuf_unmap(soc->osdev,
						       msdu_info->u.sg_info.
						       curr_seg->nbuf,
						       QDF_DMA_TO_DEVICE);
				}
				/*
				 * Free the nbuf for the current segment
				 * and make it point to the next in the list.
				 * For me, there are as many segments as there
				 * are no of clients.
				 */
				qdf_nbuf_free(msdu_info->u.sg_info
					      .curr_seg->nbuf);
				if (msdu_info->u.sg_info.curr_seg->next) {
					msdu_info->u.sg_info.curr_seg =
						msdu_info->u.sg_info
						.curr_seg->next;
					nbuf = msdu_info->u.sg_info
					       .curr_seg->nbuf;
				} else
					break;
				i++;
				dp_tx_desc_release(tx_desc, tx_q->desc_pool_id);
				continue;
			}

			/*
			 * For TSO frames, the nbuf users increment done for
			 * the current segment has to be reverted, since the
			 * hw enqueue for this segment failed
			 */
			if (msdu_info->frm_type == dp_tx_frm_tso &&
			    msdu_info->u.tso_info.curr_seg) {
				/*
				 * unmap and free current,
				 * retransmit remaining segments
				 */
				dp_tx_comp_free_buf(soc, tx_desc);
				i++;
				dp_tx_desc_release(tx_desc, tx_q->desc_pool_id);
				continue;
			}

			dp_tx_desc_release(tx_desc, tx_q->desc_pool_id);
			goto done;
		}

		/*
		 * TODO
		 * if tso_info structure can be modified to have curr_seg
		 * as first element, following 2 blocks of code (for TSO and SG)
		 * can be combined into 1
		 */

		/*
		 * For Multicast-Unicast converted packets,
		 * each converted frame (for a client) is represented as
		 * 1 segment
		 */
		if ((msdu_info->frm_type == dp_tx_frm_sg) ||
				(msdu_info->frm_type == dp_tx_frm_me)) {
			if (msdu_info->u.sg_info.curr_seg->next) {
				msdu_info->u.sg_info.curr_seg =
					msdu_info->u.sg_info.curr_seg->next;
				nbuf = msdu_info->u.sg_info.curr_seg->nbuf;
			} else
				break;
		}
		i++;
	}

	nbuf = NULL;

done:
	return nbuf;
}

/**
 * dp_tx_prepare_sg()- Extract SG info from NBUF and prepare msdu_info
 *                     for SG frames
 * @vdev: DP vdev handle
 * @nbuf: skb
 * @seg_info: Pointer to Segment info Descriptor to be prepared
 * @msdu_info: MSDU info to be setup in MSDU descriptor and MSDU extension desc.
 *
 * Return: NULL on success,
 *         nbuf when it fails to send
 */
static qdf_nbuf_t dp_tx_prepare_sg(struct dp_vdev *vdev, qdf_nbuf_t nbuf,
	struct dp_tx_seg_info_s *seg_info, struct dp_tx_msdu_info_s *msdu_info)
{
	uint32_t cur_frag, nr_frags, i;
	qdf_dma_addr_t paddr;
	struct dp_tx_sg_info_s *sg_info;

	sg_info = &msdu_info->u.sg_info;
	nr_frags = qdf_nbuf_get_nr_frags(nbuf);

	if (QDF_STATUS_SUCCESS !=
		qdf_nbuf_map_nbytes_single(vdev->osdev, nbuf,
					   QDF_DMA_TO_DEVICE,
					   qdf_nbuf_headlen(nbuf))) {
		dp_tx_err("dma map error");
		DP_STATS_INC(vdev, tx_i.sg.dma_map_error, 1);

		qdf_nbuf_free(nbuf);
		return NULL;
	}

	paddr = qdf_nbuf_mapped_paddr_get(nbuf);
	seg_info->frags[0].paddr_lo = paddr;
	seg_info->frags[0].paddr_hi = ((uint64_t) paddr) >> 32;
	seg_info->frags[0].len = qdf_nbuf_headlen(nbuf);
	seg_info->frags[0].vaddr = (void *) nbuf;

	for (cur_frag = 0; cur_frag < nr_frags; cur_frag++) {
		if (QDF_STATUS_E_FAILURE == qdf_nbuf_frag_map(vdev->osdev,
					nbuf, 0, QDF_DMA_TO_DEVICE, cur_frag)) {
			dp_tx_err("frag dma map error");
			DP_STATS_INC(vdev, tx_i.sg.dma_map_error, 1);
			goto map_err;
		}

		paddr = qdf_nbuf_get_tx_frag_paddr(nbuf);
		seg_info->frags[cur_frag + 1].paddr_lo = paddr;
		seg_info->frags[cur_frag + 1].paddr_hi =
			((uint64_t) paddr) >> 32;
		seg_info->frags[cur_frag + 1].len =
			qdf_nbuf_get_frag_size(nbuf, cur_frag);
	}

	seg_info->frag_cnt = (cur_frag + 1);
	seg_info->total_len = qdf_nbuf_len(nbuf);
	seg_info->next = NULL;

	sg_info->curr_seg = seg_info;

	msdu_info->frm_type = dp_tx_frm_sg;
	msdu_info->num_seg = 1;

	return nbuf;
map_err:
	/* restore paddr into nbuf before calling unmap */
	qdf_nbuf_mapped_paddr_set(nbuf,
				  (qdf_dma_addr_t)(seg_info->frags[0].paddr_lo |
				  ((uint64_t)
				  seg_info->frags[0].paddr_hi) << 32));
	qdf_nbuf_unmap_nbytes_single(vdev->osdev, nbuf,
				     QDF_DMA_TO_DEVICE,
				     seg_info->frags[0].len);
	for (i = 1; i <= cur_frag; i++) {
		qdf_mem_unmap_page(vdev->osdev, (qdf_dma_addr_t)
				   (seg_info->frags[i].paddr_lo | ((uint64_t)
				   seg_info->frags[i].paddr_hi) << 32),
				   seg_info->frags[i].len,
				   QDF_DMA_TO_DEVICE);
	}
	qdf_nbuf_free(nbuf);
	return NULL;
}

/**
 * dp_tx_add_tx_sniffer_meta_data()- Add tx_sniffer meta hdr info
 * @vdev: DP vdev handle
 * @msdu_info: MSDU info to be setup in MSDU descriptor and MSDU extension desc.
 * @ppdu_cookie: PPDU cookie that should be replayed in the ppdu completions
 *
 * Return: NULL on failure,
 *         nbuf when extracted successfully
 */
static
void dp_tx_add_tx_sniffer_meta_data(struct dp_vdev *vdev,
				    struct dp_tx_msdu_info_s *msdu_info,
				    uint16_t ppdu_cookie)
{
	struct htt_tx_msdu_desc_ext2_t *meta_data =
		(struct htt_tx_msdu_desc_ext2_t *)&msdu_info->meta_data[0];

	qdf_mem_zero(meta_data, sizeof(struct htt_tx_msdu_desc_ext2_t));

	HTT_TX_MSDU_EXT2_DESC_FLAG_SEND_AS_STANDALONE_SET
				(msdu_info->meta_data[5], 1);
	HTT_TX_MSDU_EXT2_DESC_FLAG_HOST_OPAQUE_VALID_SET
				(msdu_info->meta_data[5], 1);
	HTT_TX_MSDU_EXT2_DESC_HOST_OPAQUE_COOKIE_SET
				(msdu_info->meta_data[6], ppdu_cookie);

	msdu_info->exception_fw = 1;
	msdu_info->is_tx_sniffer = 1;
}

#ifdef MESH_MODE_SUPPORT

/**
 * dp_tx_extract_mesh_meta_data()- Extract mesh meta hdr info from nbuf
				and prepare msdu_info for mesh frames.
 * @vdev: DP vdev handle
 * @nbuf: skb
 * @msdu_info: MSDU info to be setup in MSDU descriptor and MSDU extension desc.
 *
 * Return: NULL on failure,
 *         nbuf when extracted successfully
 */
static
qdf_nbuf_t dp_tx_extract_mesh_meta_data(struct dp_vdev *vdev, qdf_nbuf_t nbuf,
				struct dp_tx_msdu_info_s *msdu_info)
{
	struct meta_hdr_s *mhdr;
	struct htt_tx_msdu_desc_ext2_t *meta_data =
				(struct htt_tx_msdu_desc_ext2_t *)&msdu_info->meta_data[0];

	mhdr = (struct meta_hdr_s *)qdf_nbuf_data(nbuf);

	if (CB_FTYPE_MESH_TX_INFO != qdf_nbuf_get_tx_ftype(nbuf)) {
		msdu_info->exception_fw = 0;
		goto remove_meta_hdr;
	}

	msdu_info->exception_fw = 1;

	qdf_mem_zero(meta_data, sizeof(struct htt_tx_msdu_desc_ext2_t));

	meta_data->host_tx_desc_pool = 1;
	meta_data->update_peer_cache = 1;
	meta_data->learning_frame = 1;

	if (!(mhdr->flags & METAHDR_FLAG_AUTO_RATE)) {
		meta_data->power = mhdr->power;

		meta_data->mcs_mask = 1 << mhdr->rate_info[0].mcs;
		meta_data->nss_mask = 1 << mhdr->rate_info[0].nss;
		meta_data->pream_type = mhdr->rate_info[0].preamble_type;
		meta_data->retry_limit = mhdr->rate_info[0].max_tries;

		meta_data->dyn_bw = 1;

		meta_data->valid_pwr = 1;
		meta_data->valid_mcs_mask = 1;
		meta_data->valid_nss_mask = 1;
		meta_data->valid_preamble_type  = 1;
		meta_data->valid_retries = 1;
		meta_data->valid_bw_info = 1;
	}

	if (mhdr->flags & METAHDR_FLAG_NOENCRYPT) {
		meta_data->encrypt_type = 0;
		meta_data->valid_encrypt_type = 1;
		meta_data->learning_frame = 0;
	}

	meta_data->valid_key_flags = 1;
	meta_data->key_flags = (mhdr->keyix & 0x3);

remove_meta_hdr:
	if (qdf_nbuf_pull_head(nbuf, sizeof(struct meta_hdr_s)) == NULL) {
		dp_tx_err("qdf_nbuf_pull_head failed");
		qdf_nbuf_free(nbuf);
		return NULL;
	}

	msdu_info->tid = qdf_nbuf_get_priority(nbuf);

	dp_tx_info("Meta hdr %0x %0x %0x %0x %0x %0x"
		   " tid %d to_fw %d",
		   msdu_info->meta_data[0],
		   msdu_info->meta_data[1],
		   msdu_info->meta_data[2],
		   msdu_info->meta_data[3],
		   msdu_info->meta_data[4],
		   msdu_info->meta_data[5],
		   msdu_info->tid, msdu_info->exception_fw);

	return nbuf;
}
#else
static
qdf_nbuf_t dp_tx_extract_mesh_meta_data(struct dp_vdev *vdev, qdf_nbuf_t nbuf,
				struct dp_tx_msdu_info_s *msdu_info)
{
	return nbuf;
}

#endif

/**
 * dp_check_exc_metadata() - Checks if parameters are valid
 * @tx_exc - holds all exception path parameters
 *
 * Returns true when all the parameters are valid else false
 *
 */
static bool dp_check_exc_metadata(struct cdp_tx_exception_metadata *tx_exc)
{
	bool invalid_tid = (tx_exc->tid >= DP_MAX_TIDS && tx_exc->tid !=
			    HTT_INVALID_TID);
	bool invalid_encap_type =
			(tx_exc->tx_encap_type > htt_cmn_pkt_num_types &&
			 tx_exc->tx_encap_type != CDP_INVALID_TX_ENCAP_TYPE);
	bool invalid_sec_type = (tx_exc->sec_type > cdp_num_sec_types &&
				 tx_exc->sec_type != CDP_INVALID_SEC_TYPE);
	bool invalid_cookie = (tx_exc->is_tx_sniffer == 1 &&
			       tx_exc->ppdu_cookie == 0);

	if (invalid_tid || invalid_encap_type || invalid_sec_type ||
	    invalid_cookie) {
		return false;
	}

	return true;
}

#ifdef ATH_SUPPORT_IQUE
/**
 * dp_tx_mcast_enhance() - Multicast enhancement on TX
 * @vdev: vdev handle
 * @nbuf: skb
 *
 * Return: true on success,
 *         false on failure
 */
static inline bool dp_tx_mcast_enhance(struct dp_vdev *vdev, qdf_nbuf_t nbuf)
{
	qdf_ether_header_t *eh;

	/* Mcast to Ucast Conversion*/
	if (qdf_likely(!vdev->mcast_enhancement_en))
		return true;

	eh = (qdf_ether_header_t *)qdf_nbuf_data(nbuf);
	if (DP_FRAME_IS_MULTICAST((eh)->ether_dhost) &&
	    !DP_FRAME_IS_BROADCAST((eh)->ether_dhost)) {
		dp_verbose_debug("Mcast frm for ME %pK", vdev);
		qdf_nbuf_set_next(nbuf, NULL);

		DP_STATS_INC_PKT(vdev, tx_i.mcast_en.mcast_pkt, 1,
				 qdf_nbuf_len(nbuf));
		if (dp_tx_prepare_send_me(vdev, nbuf) ==
				QDF_STATUS_SUCCESS) {
			return false;
		}

		if (qdf_unlikely(vdev->igmp_mcast_enhanc_en > 0)) {
			if (dp_tx_prepare_send_igmp_me(vdev, nbuf) ==
					QDF_STATUS_SUCCESS) {
				return false;
			}
		}
	}

	return true;
}
#else
static inline bool dp_tx_mcast_enhance(struct dp_vdev *vdev, qdf_nbuf_t nbuf)
{
	return true;
}
#endif

/**
 * dp_tx_per_pkt_vdev_id_check() - vdev id check for frame
 * @nbuf: qdf_nbuf_t
 * @vdev: struct dp_vdev *
 *
 * Allow packet for processing only if it is for peer client which is
 * connected with same vap. Drop packet if client is connected to
 * different vap.
 *
 * Return: QDF_STATUS
 */
static inline QDF_STATUS
dp_tx_per_pkt_vdev_id_check(qdf_nbuf_t nbuf, struct dp_vdev *vdev)
{
	struct dp_ast_entry *dst_ast_entry = NULL;
	qdf_ether_header_t *eh = (qdf_ether_header_t *)qdf_nbuf_data(nbuf);

	if (DP_FRAME_IS_MULTICAST((eh)->ether_dhost) ||
	    DP_FRAME_IS_BROADCAST((eh)->ether_dhost))
		return QDF_STATUS_SUCCESS;

	qdf_spin_lock_bh(&vdev->pdev->soc->ast_lock);
	dst_ast_entry = dp_peer_ast_hash_find_by_vdevid(vdev->pdev->soc,
							eh->ether_dhost,
							vdev->vdev_id);

	/* If there is no ast entry, return failure */
	if (qdf_unlikely(!dst_ast_entry)) {
		qdf_spin_unlock_bh(&vdev->pdev->soc->ast_lock);
		return QDF_STATUS_E_FAILURE;
	}
	qdf_spin_unlock_bh(&vdev->pdev->soc->ast_lock);

	return QDF_STATUS_SUCCESS;
}

/**
 * dp_tx_send_exception() - Transmit a frame on a given VAP in exception path
 * @soc: DP soc handle
 * @vdev_id: id of DP vdev handle
 * @nbuf: skb
 * @tx_exc_metadata: Handle that holds exception path meta data
 *
 * Entry point for Core Tx layer (DP_TX) invoked from
 * hard_start_xmit in OSIF/HDD to transmit frames through fw
 *
 * Return: NULL on success,
 *         nbuf when it fails to send
 */
qdf_nbuf_t
dp_tx_send_exception(struct cdp_soc_t *soc_hdl, uint8_t vdev_id,
		     qdf_nbuf_t nbuf,
		     struct cdp_tx_exception_metadata *tx_exc_metadata)
{
	struct dp_soc *soc = cdp_soc_t_to_dp_soc(soc_hdl);
	qdf_ether_header_t *eh = NULL;
	struct dp_tx_msdu_info_s msdu_info;
	struct dp_vdev *vdev = dp_vdev_get_ref_by_id(soc, vdev_id,
						     DP_MOD_ID_TX_EXCEPTION);

	if (qdf_unlikely(!vdev))
		goto fail;

	qdf_mem_zero(&msdu_info, sizeof(msdu_info));

	if (!tx_exc_metadata)
		goto fail;

	msdu_info.tid = tx_exc_metadata->tid;
	eh = (qdf_ether_header_t *)qdf_nbuf_data(nbuf);
	dp_verbose_debug("skb "QDF_MAC_ADDR_FMT,
			 QDF_MAC_ADDR_REF(nbuf->data));

	DP_STATS_INC_PKT(vdev, tx_i.rcvd, 1, qdf_nbuf_len(nbuf));

	if (qdf_unlikely(!dp_check_exc_metadata(tx_exc_metadata))) {
		dp_tx_err("Invalid parameters in exception path");
		goto fail;
	}

	/* Basic sanity checks for unsupported packets */

	/* MESH mode */
	if (qdf_unlikely(vdev->mesh_vdev)) {
		dp_tx_err("Mesh mode is not supported in exception path");
		goto fail;
	}

	/*
	 * Classify the frame and call corresponding
	 * "prepare" function which extracts the segment (TSO)
	 * and fragmentation information (for TSO , SG, ME, or Raw)
	 * into MSDU_INFO structure which is later used to fill
	 * SW and HW descriptors.
	 */
	if (qdf_nbuf_is_tso(nbuf)) {
		dp_verbose_debug("TSO frame %pK", vdev);
		DP_STATS_INC_PKT(vdev->pdev, tso_stats.num_tso_pkts, 1,
				 qdf_nbuf_len(nbuf));

		if (dp_tx_prepare_tso(vdev, nbuf, &msdu_info)) {
			DP_STATS_INC_PKT(vdev->pdev, tso_stats.dropped_host, 1,
					 qdf_nbuf_len(nbuf));
			goto fail;
		}

		goto send_multiple;
	}

	/* SG */
	if (qdf_unlikely(qdf_nbuf_is_nonlinear(nbuf))) {
		struct dp_tx_seg_info_s seg_info = {0};

		nbuf = dp_tx_prepare_sg(vdev, nbuf, &seg_info, &msdu_info);
		if (!nbuf)
			goto fail;

		dp_verbose_debug("non-TSO SG frame %pK", vdev);

		DP_STATS_INC_PKT(vdev, tx_i.sg.sg_pkt, 1,
				 qdf_nbuf_len(nbuf));

		goto send_multiple;
	}

	if (qdf_likely(tx_exc_metadata->is_tx_sniffer)) {
		DP_STATS_INC_PKT(vdev, tx_i.sniffer_rcvd, 1,
				 qdf_nbuf_len(nbuf));

		dp_tx_add_tx_sniffer_meta_data(vdev, &msdu_info,
					       tx_exc_metadata->ppdu_cookie);
	}

	/*
	 * Get HW Queue to use for this frame.
	 * TCL supports upto 4 DMA rings, out of which 3 rings are
	 * dedicated for data and 1 for command.
	 * "queue_id" maps to one hardware ring.
	 *  With each ring, we also associate a unique Tx descriptor pool
	 *  to minimize lock contention for these resources.
	 */
	dp_tx_get_queue(vdev, nbuf, &msdu_info.tx_queue);

	/*
	 * Check exception descriptors
	 */
	if (dp_tx_exception_limit_check(vdev))
		goto fail;

	/*  Single linear frame */
	/*
	 * If nbuf is a simple linear frame, use send_single function to
	 * prepare direct-buffer type TCL descriptor and enqueue to TCL
	 * SRNG. There is no need to setup a MSDU extension descriptor.
	 */
	nbuf = dp_tx_send_msdu_single(vdev, nbuf, &msdu_info,
			tx_exc_metadata->peer_id, tx_exc_metadata);

	dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_TX_EXCEPTION);
	return nbuf;

send_multiple:
	nbuf = dp_tx_send_msdu_multiple(vdev, nbuf, &msdu_info);

fail:
	if (vdev)
		dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_TX_EXCEPTION);
	dp_verbose_debug("pkt send failed");
	return nbuf;
}

/**
 * dp_tx_send_exception_vdev_id_check() - Transmit a frame on a given VAP
 *      in exception path in special case to avoid regular exception path chk.
 * @soc: DP soc handle
 * @vdev_id: id of DP vdev handle
 * @nbuf: skb
 * @tx_exc_metadata: Handle that holds exception path meta data
 *
 * Entry point for Core Tx layer (DP_TX) invoked from
 * hard_start_xmit in OSIF/HDD to transmit frames through fw
 *
 * Return: NULL on success,
 *         nbuf when it fails to send
 */
qdf_nbuf_t
dp_tx_send_exception_vdev_id_check(struct cdp_soc_t *soc_hdl,
				   uint8_t vdev_id, qdf_nbuf_t nbuf,
		     struct cdp_tx_exception_metadata *tx_exc_metadata)
{
	struct dp_soc *soc = cdp_soc_t_to_dp_soc(soc_hdl);
	struct dp_vdev *vdev = dp_vdev_get_ref_by_id(soc, vdev_id,
						     DP_MOD_ID_TX_EXCEPTION);

	if (qdf_unlikely(!vdev))
		goto fail;

	if (qdf_unlikely(dp_tx_per_pkt_vdev_id_check(nbuf, vdev)
			== QDF_STATUS_E_FAILURE)) {
		DP_STATS_INC(vdev, tx_i.dropped.fail_per_pkt_vdev_id_check, 1);
		goto fail;
	}

	/* Unref count as it will agin be taken inside dp_tx_exception */
	dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_TX_EXCEPTION);

	return dp_tx_send_exception(soc_hdl, vdev_id, nbuf, tx_exc_metadata);

fail:
	if (vdev)
		dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_TX_EXCEPTION);
	dp_verbose_debug("pkt send failed");
	return nbuf;
}

/**
 * dp_tx_send_mesh() - Transmit mesh frame on a given VAP
 * @soc: DP soc handle
 * @vdev_id: DP vdev handle
 * @nbuf: skb
 *
 * Entry point for Core Tx layer (DP_TX) invoked from
 * hard_start_xmit in OSIF/HDD
 *
 * Return: NULL on success,
 *         nbuf when it fails to send
 */
#ifdef MESH_MODE_SUPPORT
qdf_nbuf_t dp_tx_send_mesh(struct cdp_soc_t *soc_hdl, uint8_t vdev_id,
			   qdf_nbuf_t nbuf)
{
	struct dp_soc *soc = cdp_soc_t_to_dp_soc(soc_hdl);
	struct meta_hdr_s *mhdr;
	qdf_nbuf_t nbuf_mesh = NULL;
	qdf_nbuf_t nbuf_clone = NULL;
	struct dp_vdev *vdev;
	uint8_t no_enc_frame = 0;

	nbuf_mesh = qdf_nbuf_unshare(nbuf);
	if (!nbuf_mesh) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				"qdf_nbuf_unshare failed");
		return nbuf;
	}

	vdev = dp_vdev_get_ref_by_id(soc, vdev_id, DP_MOD_ID_MESH);
	if (!vdev) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				"vdev is NULL for vdev_id %d", vdev_id);
		return nbuf;
	}

	nbuf = nbuf_mesh;

	mhdr = (struct meta_hdr_s *)qdf_nbuf_data(nbuf);

	if ((vdev->sec_type != cdp_sec_type_none) &&
			(mhdr->flags & METAHDR_FLAG_NOENCRYPT))
		no_enc_frame = 1;

	if (mhdr->flags & METAHDR_FLAG_NOQOS)
		qdf_nbuf_set_priority(nbuf, HTT_TX_EXT_TID_NON_QOS_MCAST_BCAST);

	if ((mhdr->flags & METAHDR_FLAG_INFO_UPDATED) &&
		       !no_enc_frame) {
		nbuf_clone = qdf_nbuf_clone(nbuf);
		if (!nbuf_clone) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				"qdf_nbuf_clone failed");
			dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_MESH);
			return nbuf;
		}
		qdf_nbuf_set_tx_ftype(nbuf_clone, CB_FTYPE_MESH_TX_INFO);
	}

	if (nbuf_clone) {
		if (!dp_tx_send(soc_hdl, vdev_id, nbuf_clone)) {
			DP_STATS_INC(vdev, tx_i.mesh.exception_fw, 1);
		} else {
			qdf_nbuf_free(nbuf_clone);
		}
	}

	if (no_enc_frame)
		qdf_nbuf_set_tx_ftype(nbuf, CB_FTYPE_MESH_TX_INFO);
	else
		qdf_nbuf_set_tx_ftype(nbuf, CB_FTYPE_INVALID);

	nbuf = dp_tx_send(soc_hdl, vdev_id, nbuf);
	if ((!nbuf) && no_enc_frame) {
		DP_STATS_INC(vdev, tx_i.mesh.exception_fw, 1);
	}

	dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_MESH);
	return nbuf;
}

#else

qdf_nbuf_t dp_tx_send_mesh(struct cdp_soc_t *soc, uint8_t vdev_id,
			   qdf_nbuf_t nbuf)
{
	return dp_tx_send(soc, vdev_id, nbuf);
}

#endif

/**
 * dp_tx_nawds_handler() - NAWDS handler
 *
 * @soc: DP soc handle
 * @vdev_id: id of DP vdev handle
 * @msdu_info: msdu_info required to create HTT metadata
 * @nbuf: skb
 *
 * This API transfers the multicast frames with the peer id
 * on NAWDS enabled peer.

 * Return: none
 */

static inline
void dp_tx_nawds_handler(struct dp_soc *soc, struct dp_vdev *vdev,
			 struct dp_tx_msdu_info_s *msdu_info, qdf_nbuf_t nbuf)
{
	struct dp_peer *peer = NULL;
	qdf_nbuf_t nbuf_clone = NULL;
	uint16_t peer_id = DP_INVALID_PEER;
	uint16_t sa_peer_id = DP_INVALID_PEER;
	struct dp_ast_entry *ast_entry = NULL;
	qdf_ether_header_t *eh = (qdf_ether_header_t *)qdf_nbuf_data(nbuf);

	if (!soc->ast_offload_support) {
		qdf_spin_lock_bh(&soc->ast_lock);
		ast_entry = dp_peer_ast_hash_find_by_pdevid
					(soc,
					(uint8_t *)(eh->ether_shost),
					vdev->pdev->pdev_id);
		if (ast_entry)
			sa_peer_id = ast_entry->peer_id;
		qdf_spin_unlock_bh(&soc->ast_lock);
	}

	qdf_spin_lock_bh(&vdev->peer_list_lock);
	TAILQ_FOREACH(peer, &vdev->peer_list, peer_list_elem) {
		if (!peer->bss_peer && peer->nawds_enabled) {
			peer_id = peer->peer_id;
			/* Multicast packets needs to be
			 * dropped in case of intra bss forwarding
			 */
			if (!soc->ast_offload_support) {
				if (sa_peer_id == peer->peer_id) {
					dp_tx_debug("multicast packet");
					DP_STATS_INC(peer, tx.nawds_mcast_drop,
						     1);
					continue;
				}
			}

			nbuf_clone = qdf_nbuf_clone(nbuf);

			if (!nbuf_clone) {
				QDF_TRACE(QDF_MODULE_ID_DP,
					  QDF_TRACE_LEVEL_ERROR,
					  FL("nbuf clone failed"));
				break;
			}

			nbuf_clone = dp_tx_send_msdu_single(vdev, nbuf_clone,
							    msdu_info, peer_id,
							    NULL);

			if (nbuf_clone) {
				dp_tx_debug("pkt send failed");
				qdf_nbuf_free(nbuf_clone);
			} else {
				if (peer_id != DP_INVALID_PEER)
					DP_STATS_INC_PKT(peer, tx.nawds_mcast,
							 1, qdf_nbuf_len(nbuf));
			}
		}
	}

	qdf_spin_unlock_bh(&vdev->peer_list_lock);
}

/**
 * dp_tx_send() - Transmit a frame on a given VAP
 * @soc: DP soc handle
 * @vdev_id: id of DP vdev handle
 * @nbuf: skb
 *
 * Entry point for Core Tx layer (DP_TX) invoked from
 * hard_start_xmit in OSIF/HDD or from dp_rx_process for intravap forwarding
 * cases
 *
 * Return: NULL on success,
 *         nbuf when it fails to send
 */
qdf_nbuf_t dp_tx_send(struct cdp_soc_t *soc_hdl, uint8_t vdev_id,
		      qdf_nbuf_t nbuf)
{
	struct dp_soc *soc = cdp_soc_t_to_dp_soc(soc_hdl);
	uint16_t peer_id = HTT_INVALID_PEER;
	/*
	 * doing a memzero is causing additional function call overhead
	 * so doing static stack clearing
	 */
	struct dp_tx_msdu_info_s msdu_info = {0};
	struct dp_vdev *vdev = NULL;

	if (qdf_unlikely(vdev_id >= MAX_VDEV_CNT))
		return nbuf;

	/*
	 * dp_vdev_get_ref_by_id does does a atomic operation avoid using
	 * this in per packet path.
	 *
	 * As in this path vdev memory is already protected with netdev
	 * tx lock
	 */
	vdev = soc->vdev_id_map[vdev_id];
	if (qdf_unlikely(!vdev))
		return nbuf;

	dp_verbose_debug("skb "QDF_MAC_ADDR_FMT,
			 QDF_MAC_ADDR_REF(nbuf->data));

	/*
	 * Set Default Host TID value to invalid TID
	 * (TID override disabled)
	 */
	msdu_info.tid = HTT_TX_EXT_TID_INVALID;
	DP_STATS_INC_PKT(vdev, tx_i.rcvd, 1, qdf_nbuf_len(nbuf));

	if (qdf_unlikely(vdev->mesh_vdev)) {
		qdf_nbuf_t nbuf_mesh = dp_tx_extract_mesh_meta_data(vdev, nbuf,
								&msdu_info);
		if (!nbuf_mesh) {
			dp_verbose_debug("Extracting mesh metadata failed");
			return nbuf;
		}
		nbuf = nbuf_mesh;
	}

	/*
	 * Get HW Queue to use for this frame.
	 * TCL supports upto 4 DMA rings, out of which 3 rings are
	 * dedicated for data and 1 for command.
	 * "queue_id" maps to one hardware ring.
	 *  With each ring, we also associate a unique Tx descriptor pool
	 *  to minimize lock contention for these resources.
	 */
	dp_tx_get_queue(vdev, nbuf, &msdu_info.tx_queue);

	/*
	 * TCL H/W supports 2 DSCP-TID mapping tables.
	 *  Table 1 - Default DSCP-TID mapping table
	 *  Table 2 - 1 DSCP-TID override table
	 *
	 * If we need a different DSCP-TID mapping for this vap,
	 * call tid_classify to extract DSCP/ToS from frame and
	 * map to a TID and store in msdu_info. This is later used
	 * to fill in TCL Input descriptor (per-packet TID override).
	 */
	dp_tx_classify_tid(vdev, nbuf, &msdu_info);

	/*
	 * Classify the frame and call corresponding
	 * "prepare" function which extracts the segment (TSO)
	 * and fragmentation information (for TSO , SG, ME, or Raw)
	 * into MSDU_INFO structure which is later used to fill
	 * SW and HW descriptors.
	 */
	if (qdf_nbuf_is_tso(nbuf)) {
		dp_verbose_debug("TSO frame %pK", vdev);
		DP_STATS_INC_PKT(vdev->pdev, tso_stats.num_tso_pkts, 1,
				 qdf_nbuf_len(nbuf));

		if (dp_tx_prepare_tso(vdev, nbuf, &msdu_info)) {
			DP_STATS_INC_PKT(vdev->pdev, tso_stats.dropped_host, 1,
					 qdf_nbuf_len(nbuf));
			return nbuf;
		}

		goto send_multiple;
	}

	/* SG */
	if (qdf_unlikely(qdf_nbuf_is_nonlinear(nbuf))) {
		struct dp_tx_seg_info_s seg_info = {0};

		nbuf = dp_tx_prepare_sg(vdev, nbuf, &seg_info, &msdu_info);
		if (!nbuf)
			return NULL;

		dp_verbose_debug("non-TSO SG frame %pK", vdev);

		DP_STATS_INC_PKT(vdev, tx_i.sg.sg_pkt, 1,
				qdf_nbuf_len(nbuf));

		goto send_multiple;
	}

	if (qdf_unlikely(!dp_tx_mcast_enhance(vdev, nbuf)))
		return NULL;

	/* RAW */
	if (qdf_unlikely(vdev->tx_encap_type == htt_cmn_pkt_type_raw)) {
		struct dp_tx_seg_info_s seg_info = {0};

		nbuf = dp_tx_prepare_raw(vdev, nbuf, &seg_info, &msdu_info);
		if (!nbuf)
			return NULL;

		dp_verbose_debug("Raw frame %pK", vdev);

		goto send_multiple;

	}

	if (qdf_unlikely(vdev->nawds_enabled)) {
		qdf_ether_header_t *eh = (qdf_ether_header_t *)
					  qdf_nbuf_data(nbuf);
		if (DP_FRAME_IS_MULTICAST((eh)->ether_dhost))
			dp_tx_nawds_handler(soc, vdev, &msdu_info, nbuf);

		peer_id = DP_INVALID_PEER;
		DP_STATS_INC_PKT(vdev, tx_i.nawds_mcast,
				 1, qdf_nbuf_len(nbuf));
	}

	/*  Single linear frame */
	/*
	 * If nbuf is a simple linear frame, use send_single function to
	 * prepare direct-buffer type TCL descriptor and enqueue to TCL
	 * SRNG. There is no need to setup a MSDU extension descriptor.
	 */
	nbuf = dp_tx_send_msdu_single(vdev, nbuf, &msdu_info, peer_id, NULL);

	return nbuf;

send_multiple:
	nbuf = dp_tx_send_msdu_multiple(vdev, nbuf, &msdu_info);

	if (qdf_unlikely(nbuf && msdu_info.frm_type == dp_tx_frm_raw))
		dp_tx_raw_prepare_unset(vdev->pdev->soc, nbuf);

	return nbuf;
}

/**
 * dp_tx_send_vdev_id_check() - Transmit a frame on a given VAP in special
 *      case to vaoid check in perpkt path.
 * @soc: DP soc handle
 * @vdev_id: id of DP vdev handle
 * @nbuf: skb
 *
 * Entry point for Core Tx layer (DP_TX) invoked from
 * hard_start_xmit in OSIF/HDD to transmit packet through dp_tx_send
 * with special condition to avoid per pkt check in dp_tx_send
 *
 * Return: NULL on success,
 *         nbuf when it fails to send
 */
qdf_nbuf_t dp_tx_send_vdev_id_check(struct cdp_soc_t *soc_hdl,
				    uint8_t vdev_id, qdf_nbuf_t nbuf)
{
	struct dp_soc *soc = cdp_soc_t_to_dp_soc(soc_hdl);
	struct dp_vdev *vdev = NULL;

	if (qdf_unlikely(vdev_id >= MAX_VDEV_CNT))
		return nbuf;

	/*
	 * dp_vdev_get_ref_by_id does does a atomic operation avoid using
	 * this in per packet path.
	 *
	 * As in this path vdev memory is already protected with netdev
	 * tx lock
	 */
	vdev = soc->vdev_id_map[vdev_id];
	if (qdf_unlikely(!vdev))
		return nbuf;

	if (qdf_unlikely(dp_tx_per_pkt_vdev_id_check(nbuf, vdev)
			== QDF_STATUS_E_FAILURE)) {
		DP_STATS_INC(vdev, tx_i.dropped.fail_per_pkt_vdev_id_check, 1);
		return nbuf;
	}

	return dp_tx_send(soc_hdl, vdev_id, nbuf);
}

#ifdef UMAC_SUPPORT_PROXY_ARP
/**
 * dp_tx_proxy_arp() - Tx proxy arp handler
 * @vdev: datapath vdev handle
 * @buf: sk buffer
 *
 * Return: status
 */
static inline
int dp_tx_proxy_arp(struct dp_vdev *vdev, qdf_nbuf_t nbuf)
{
	if (vdev->osif_proxy_arp)
		return vdev->osif_proxy_arp(vdev->osif_vdev, nbuf);

	/*
	 * when UMAC_SUPPORT_PROXY_ARP is defined, we expect
	 * osif_proxy_arp has a valid function pointer assigned
	 * to it
	 */
	dp_tx_err("valid function pointer for osif_proxy_arp is expected!!\n");

	return QDF_STATUS_NOT_INITIALIZED;
}
#else
/**
 * dp_tx_proxy_arp() - Tx proxy arp handler
 * @vdev: datapath vdev handle
 * @buf: sk buffer
 *
 * This function always return 0 when UMAC_SUPPORT_PROXY_ARP
 * is not defined.
 *
 * Return: status
 */
static inline
int dp_tx_proxy_arp(struct dp_vdev *vdev, qdf_nbuf_t nbuf)
{
	return QDF_STATUS_SUCCESS;
}
#endif

/**
 * dp_tx_reinject_handler() - Tx Reinject Handler
 * @soc: datapath soc handle
 * @vdev: datapath vdev handle
 * @tx_desc: software descriptor head pointer
 * @status : Tx completion status from HTT descriptor
 *
 * This function reinjects frames back to Target.
 * Todo - Host queue needs to be added
 *
 * Return: none
 */
void dp_tx_reinject_handler(struct dp_soc *soc,
			    struct dp_vdev *vdev,
			    struct dp_tx_desc_s *tx_desc,
			    uint8_t *status)
{
	struct dp_peer *peer = NULL;
	uint32_t peer_id = HTT_INVALID_PEER;
	qdf_nbuf_t nbuf = tx_desc->nbuf;
	qdf_nbuf_t nbuf_copy = NULL;
	struct dp_tx_msdu_info_s msdu_info;
#ifdef WDS_VENDOR_EXTENSION
	int is_mcast = 0, is_ucast = 0;
	int num_peers_3addr = 0;
	qdf_ether_header_t *eth_hdr = (qdf_ether_header_t *)(qdf_nbuf_data(nbuf));
	struct ieee80211_frame_addr4 *wh = (struct ieee80211_frame_addr4 *)(qdf_nbuf_data(nbuf));
#endif

	qdf_assert(vdev);

	qdf_mem_zero(&msdu_info, sizeof(msdu_info));

	dp_tx_get_queue(vdev, nbuf, &msdu_info.tx_queue);

	dp_tx_debug("Tx reinject path");

	DP_STATS_INC_PKT(vdev, tx_i.reinject_pkts, 1,
			qdf_nbuf_len(tx_desc->nbuf));

#ifdef WDS_VENDOR_EXTENSION
	if (qdf_unlikely(vdev->tx_encap_type != htt_cmn_pkt_type_raw)) {
		is_mcast = (IS_MULTICAST(wh->i_addr1)) ? 1 : 0;
	} else {
		is_mcast = (IS_MULTICAST(eth_hdr->ether_dhost)) ? 1 : 0;
	}
	is_ucast = !is_mcast;

	qdf_spin_lock_bh(&vdev->peer_list_lock);
	TAILQ_FOREACH(peer, &vdev->peer_list, peer_list_elem) {
		if (peer->bss_peer)
			continue;

		/* Detect wds peers that use 3-addr framing for mcast.
		 * if there are any, the bss_peer is used to send the
		 * the mcast frame using 3-addr format. all wds enabled
		 * peers that use 4-addr framing for mcast frames will
		 * be duplicated and sent as 4-addr frames below.
		 */
		if (!peer->wds_enabled || !peer->wds_ecm.wds_tx_mcast_4addr) {
			num_peers_3addr = 1;
			break;
		}
	}
	qdf_spin_unlock_bh(&vdev->peer_list_lock);
#endif

	if (qdf_unlikely(vdev->mesh_vdev)) {
		DP_TX_FREE_SINGLE_BUF(vdev->pdev->soc, tx_desc->nbuf);
	} else {
		qdf_spin_lock_bh(&vdev->peer_list_lock);
		TAILQ_FOREACH(peer, &vdev->peer_list, peer_list_elem) {
			if ((peer->peer_id != HTT_INVALID_PEER) &&
#ifdef WDS_VENDOR_EXTENSION
			/*
			 * . if 3-addr STA, then send on BSS Peer
			 * . if Peer WDS enabled and accept 4-addr mcast,
			 * send mcast on that peer only
			 * . if Peer WDS enabled and accept 4-addr ucast,
			 * send ucast on that peer only
			 */
			((peer->bss_peer && num_peers_3addr && is_mcast) ||
			 (peer->wds_enabled &&
				  ((is_mcast && peer->wds_ecm.wds_tx_mcast_4addr) ||
				   (is_ucast && peer->wds_ecm.wds_tx_ucast_4addr))))) {
#else
			(peer->bss_peer &&
			 (dp_tx_proxy_arp(vdev, nbuf) == QDF_STATUS_SUCCESS))) {
#endif
				peer_id = DP_INVALID_PEER;

				nbuf_copy = qdf_nbuf_copy(nbuf);

				if (!nbuf_copy) {
					dp_tx_debug("nbuf copy failed");
					break;
				}

				nbuf_copy = dp_tx_send_msdu_single(vdev,
						nbuf_copy,
						&msdu_info,
						peer_id,
						NULL);

				if (nbuf_copy) {
					dp_tx_debug("pkt send failed");
					qdf_nbuf_free(nbuf_copy);
				}
			}
		}
		qdf_spin_unlock_bh(&vdev->peer_list_lock);
	}

	qdf_nbuf_free(nbuf);

	dp_tx_desc_release(tx_desc, tx_desc->pool_id);
}

/**
 * dp_tx_inspect_handler() - Tx Inspect Handler
 * @soc: datapath soc handle
 * @vdev: datapath vdev handle
 * @tx_desc: software descriptor head pointer
 * @status : Tx completion status from HTT descriptor
 *
 * Handles Tx frames sent back to Host for inspection
 * (ProxyARP)
 *
 * Return: none
 */
void dp_tx_inspect_handler(struct dp_soc *soc,
			   struct dp_vdev *vdev,
			   struct dp_tx_desc_s *tx_desc,
			   uint8_t *status)
{

	QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO,
			"%s Tx inspect path",
			__func__);

	DP_STATS_INC_PKT(vdev, tx_i.inspect_pkts, 1,
			 qdf_nbuf_len(tx_desc->nbuf));

	DP_TX_FREE_SINGLE_BUF(soc, tx_desc->nbuf);
	dp_tx_desc_release(tx_desc, tx_desc->pool_id);
}

#ifdef MESH_MODE_SUPPORT
/**
 * dp_tx_comp_fill_tx_completion_stats() - Fill per packet Tx completion stats
 *                                         in mesh meta header
 * @tx_desc: software descriptor head pointer
 * @ts: pointer to tx completion stats
 * Return: none
 */
static
void dp_tx_comp_fill_tx_completion_stats(struct dp_tx_desc_s *tx_desc,
		struct hal_tx_completion_status *ts)
{
	qdf_nbuf_t netbuf = tx_desc->nbuf;

	if (!tx_desc->msdu_ext_desc) {
		if (qdf_nbuf_pull_head(netbuf, tx_desc->pkt_offset) == NULL) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				"netbuf %pK offset %d",
				netbuf, tx_desc->pkt_offset);
			return;
		}
	}
}

#else
static
void dp_tx_comp_fill_tx_completion_stats(struct dp_tx_desc_s *tx_desc,
		struct hal_tx_completion_status *ts)
{
}

#endif

#ifdef QCA_PEER_EXT_STATS
/*
 * dp_tx_compute_tid_delay() - Compute per TID delay
 * @stats: Per TID delay stats
 * @tx_desc: Software Tx descriptor
 *
 * Compute the software enqueue and hw enqueue delays and
 * update the respective histograms
 *
 * Return: void
 */
static void dp_tx_compute_tid_delay(struct cdp_delay_tid_stats *stats,
				    struct dp_tx_desc_s *tx_desc)
{
	struct cdp_delay_tx_stats  *tx_delay = &stats->tx_delay;
	int64_t current_timestamp, timestamp_ingress, timestamp_hw_enqueue;
	uint32_t sw_enqueue_delay, fwhw_transmit_delay;

	current_timestamp = qdf_ktime_to_ms(qdf_ktime_real_get());
	timestamp_ingress = qdf_nbuf_get_timestamp(tx_desc->nbuf);
	timestamp_hw_enqueue = tx_desc->timestamp;
	sw_enqueue_delay = (uint32_t)(timestamp_hw_enqueue - timestamp_ingress);
	fwhw_transmit_delay = (uint32_t)(current_timestamp -
					 timestamp_hw_enqueue);

	/*
	 * Update the Tx software enqueue delay and HW enque-Completion delay.
	 */
	dp_hist_update_stats(&tx_delay->tx_swq_delay, sw_enqueue_delay);
	dp_hist_update_stats(&tx_delay->hwtx_delay, fwhw_transmit_delay);
}

/*
 * dp_tx_update_peer_ext_stats() - Update the peer extended stats
 * @peer: DP peer context
 * @tx_desc: Tx software descriptor
 * @tid: Transmission ID
 * @ring_id: Rx CPU context ID/CPU_ID
 *
 * Update the peer extended stats. These are enhanced other
 * delay stats per msdu level.
 *
 * Return: void
 */
static void dp_tx_update_peer_ext_stats(struct dp_peer *peer,
					struct dp_tx_desc_s *tx_desc,
					uint8_t tid, uint8_t ring_id)
{
	struct dp_pdev *pdev = peer->vdev->pdev;
	struct dp_soc *soc = NULL;
	struct cdp_peer_ext_stats *pext_stats = NULL;

	soc = pdev->soc;
	if (qdf_likely(!wlan_cfg_is_peer_ext_stats_enabled(soc->wlan_cfg_ctx)))
		return;

	pext_stats = peer->pext_stats;

	qdf_assert(pext_stats);
	qdf_assert(ring < CDP_MAX_TXRX_CTX);

	/*
	 * For non-TID packets use the TID 9
	 */
	if (qdf_unlikely(tid >= CDP_MAX_DATA_TIDS))
		tid = CDP_MAX_DATA_TIDS - 1;

	dp_tx_compute_tid_delay(&pext_stats->delay_stats[tid][ring_id],
				tx_desc);
}
#else
static inline void dp_tx_update_peer_ext_stats(struct dp_peer *peer,
					       struct dp_tx_desc_s *tx_desc,
					       uint8_t tid, uint8_t ring_id)
{
}
#endif

#ifdef HW_TX_DELAY_STATS_ENABLE
static inline
void dp_update_tx_delay_stats(struct dp_vdev *vdev, uint32_t delay, uint8_t tid,
			      uint8_t mode, uint8_t ring_id)
{
	struct cdp_tid_tx_stats *tstats =
		&vdev->stats.tid_tx_stats[ring_id][tid];

	dp_update_delay_stats(tstats, NULL, delay, tid, mode, ring_id);
}
#else
static inline
void dp_update_tx_delay_stats(struct dp_vdev *vdev, uint32_t delay, uint8_t tid,
			      uint8_t mode, uint8_t ring_id)
{
	struct cdp_tid_tx_stats *tstats =
		&vdev->pdev->stats.tid_stats.tid_tx_stats[ring_id][tid];

	dp_update_delay_stats(tstats, NULL, delay, tid, mode, ring_id);
}
#endif

/**
 * dp_tx_compute_delay() - Compute and fill in all timestamps
 *				to pass in correct fields
 *
 * @vdev: pdev handle
 * @tx_desc: tx descriptor
 * @tid: tid value
 * @ring_id: TCL or WBM ring number for transmit path
 * Return: none
 */
void dp_tx_compute_delay(struct dp_vdev *vdev, struct dp_tx_desc_s *tx_desc,
			 uint8_t tid, uint8_t ring_id)
{
	int64_t current_timestamp, timestamp_ingress, timestamp_hw_enqueue;
	uint32_t sw_enqueue_delay, fwhw_transmit_delay, interframe_delay;

	if (qdf_likely(!vdev->pdev->delay_stats_flag) &&
	    qdf_likely(!dp_is_vdev_tx_delay_stats_enabled(vdev)))
		return;

	current_timestamp = qdf_ktime_to_ms(qdf_ktime_real_get());
	timestamp_hw_enqueue = tx_desc->timestamp;
	fwhw_transmit_delay = (uint32_t)(current_timestamp -
					 timestamp_hw_enqueue);

	/*
	 * Delay between packet enqueued to HW and Tx completion
	 */
	dp_update_tx_delay_stats(vdev, fwhw_transmit_delay, tid,
				 CDP_DELAY_STATS_FW_HW_TRANSMIT, ring_id);

	/*
	 * For MCL, only enqueue to completion delay is required
	 * so return if the vdev flag is enabled.
	 */
	if (dp_is_vdev_tx_delay_stats_enabled(vdev))
		return;

	timestamp_ingress = qdf_nbuf_get_timestamp(tx_desc->nbuf);
	sw_enqueue_delay = (uint32_t)(timestamp_hw_enqueue - timestamp_ingress);
	interframe_delay = (uint32_t)(timestamp_ingress -
				      vdev->prev_tx_enq_tstamp);

	/*
	 * Delay in software enqueue
	 */
	dp_update_tx_delay_stats(vdev, sw_enqueue_delay, tid,
				 CDP_DELAY_STATS_SW_ENQ, ring_id);

	/*
	 * Update interframe delay stats calculated at hardstart receive point.
	 * Value of vdev->prev_tx_enq_tstamp will be 0 for 1st frame, so
	 * interframe delay will not be calculate correctly for 1st frame.
	 * On the other side, this will help in avoiding extra per packet check
	 * of !vdev->prev_tx_enq_tstamp.
	 */
	dp_update_tx_delay_stats(vdev, interframe_delay, tid,
				 CDP_DELAY_STATS_TX_INTERFRAME, ring_id);
	vdev->prev_tx_enq_tstamp = timestamp_ingress;
}

#ifdef DISABLE_DP_STATS
static
inline void dp_update_no_ack_stats(qdf_nbuf_t nbuf, struct dp_peer *peer,
				   struct hal_tx_completion_status *ts)
{
}
#else
static
inline void dp_update_no_ack_stats(qdf_nbuf_t nbuf, struct dp_peer *peer,
				   struct hal_tx_completion_status *ts)
{
	enum qdf_proto_subtype subtype = QDF_PROTO_INVALID;

	DPTRACE(qdf_dp_track_noack_check(nbuf, &subtype));
	if (subtype != QDF_PROTO_INVALID)
		DP_STATS_INC(peer, tx.no_ack_count[subtype], 1);

	switch (ts->status) {
	case HAL_TX_TQM_RR_REM_CMD_AGED:
		DP_STATS_INC(peer, tx.dropped.age_out, 1);
		break;
	case HAL_TX_TQM_RR_REM_CMD_REM:
		DP_STATS_INC_PKT(peer, tx.dropped.fw_rem, 1,
				 qdf_nbuf_len(nbuf));
		break;
	case HAL_TX_TQM_RR_REM_CMD_NOTX:
		DP_STATS_INC(peer, tx.dropped.fw_rem_tx, 1);
		break;
	case HAL_TX_TQM_RR_FW_REASON1:
		DP_STATS_INC(peer, tx.dropped.fw_reason1, 1);
		break;
	case HAL_TX_TQM_RR_FW_REASON2:
		DP_STATS_INC(peer, tx.dropped.fw_reason2, 1);
		break;
	case HAL_TX_TQM_RR_FW_REASON3:
		DP_STATS_INC(peer, tx.dropped.fw_reason3, 1);
		break;
	case HAL_TX_TQM_RR_REM_CMD_DISABLE_QUEUE:
		DP_STATS_INC(peer, tx.dropped.fw_rem_queue_disable, 1);
		break;
	case HAL_TX_TQM_RR_REM_CMD_TILL_NONMATCHING:
		DP_STATS_INC(peer, tx.dropped.fw_rem_no_match, 1);
		break;
	case HAL_TX_TQM_RR_DROP_THRESHOLD:
		DP_STATS_INC(peer, tx.dropped.drop_threshold, 1);
		break;
	case HAL_TX_TQM_RR_LINK_DESC_UNAVAILABLE:
		DP_STATS_INC(peer, tx.dropped.drop_link_desc_na, 1);
		break;
	case HAL_TX_TQM_RR_DROP_OR_INVALID_MSDU:
		DP_STATS_INC(peer, tx.dropped.invalid_drop, 1);
		break;
	case HAL_TX_TQM_RR_MULTICAST_DROP:
		DP_STATS_INC(peer, tx.dropped.mcast_vdev_drop, 1);
		break;
	default:
		DP_STATS_INC(peer, tx.dropped.invalid_rr, 1);
		break;
	}

	/*
	 * tx_failed is ideally supposed to be updated from HTT ppdu completion
	 * stats. But in IPQ807X/IPQ6018 chipsets owing to hw limitation there
	 * are no completions for failed cases. Hence updating tx_failed from
	 * data path. Please note that if tx_failed is fixed to be from ppdu,
	 * then this has to be removed
	 */
	DP_STATS_INC(peer, tx.tx_failed, 1);
}
#endif

static inline void
dp_tx_update_acked_rate_stats(struct dp_peer *peer,
			      struct hal_tx_completion_status *ts)
{
	uint8_t mcs, pkt_type;
	uint8_t retry_threshold = peer->mpdu_retry_threshold;

	mcs = ts->mcs;
	pkt_type = ts->pkt_type;

	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[MAX_MCS - 1], 1,
		      ((mcs >= MAX_MCS_11A) && (pkt_type == DOT11_A)));
	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[mcs], 1,
		      ((mcs < (MAX_MCS_11A)) && (pkt_type == DOT11_A)));
	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[MAX_MCS - 1], 1,
		      ((mcs >= MAX_MCS_11B) && (pkt_type == DOT11_B)));
	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[mcs], 1,
		      ((mcs < MAX_MCS_11B) && (pkt_type == DOT11_B)));
	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[MAX_MCS - 1], 1,
		      ((mcs >= MAX_MCS_11A) && (pkt_type == DOT11_N)));
	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[mcs], 1,
		      ((mcs < MAX_MCS_11A) && (pkt_type == DOT11_N)));
	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[MAX_MCS - 1], 1,
		      ((mcs >= MAX_MCS_11AC) && (pkt_type == DOT11_AC)));
	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[mcs], 1,
		      ((mcs < MAX_MCS_11AC) && (pkt_type == DOT11_AC)));
	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[MAX_MCS - 1], 1,
		      ((mcs >= (MAX_MCS - 1)) && (pkt_type == DOT11_AX)));
	DP_STATS_INCC(peer, tx.pkt_type[pkt_type].mcs_count[mcs], 1,
		      ((mcs < (MAX_MCS - 1)) && (pkt_type == DOT11_AX)));

	DP_STATS_INC(peer, tx.sgi_count[ts->sgi], 1);
	DP_STATS_INC(peer, tx.bw[ts->bw], 1);
	DP_STATS_UPD(peer, tx.last_ack_rssi, ts->ack_frame_rssi);
	DP_STATS_INC(peer, tx.wme_ac_type[TID_TO_WME_AC(ts->tid)], 1);
	DP_STATS_INCC(peer, tx.stbc, 1, ts->stbc);
	DP_STATS_INCC(peer, tx.ldpc, 1, ts->ldpc);
	DP_STATS_INCC(peer, tx.retries, 1, ts->transmit_cnt > 1);
	if (ts->first_msdu) {
		DP_STATS_INCC(peer, tx.retries_mpdu, 1, ts->transmit_cnt > 1);
		if (retry_threshold)
			DP_STATS_INCC(peer, tx.mpdu_success_with_retries,
				      qdf_do_div(ts->transmit_cnt,
						 retry_threshold),
				      ts->transmit_cnt > retry_threshold);
	}
	peer->stats.tx.last_tx_ts = qdf_system_ticks();
}

/**
 * dp_tx_update_peer_stats() - Update peer stats from Tx completion indications
 *				per wbm ring
 *
 * @tx_desc: software descriptor head pointer
 * @ts: Tx completion status
 * @peer: peer handle
 * @ring_id: ring number
 *
 * Return: None
 */
static inline void
dp_tx_update_peer_stats(struct dp_tx_desc_s *tx_desc,
			struct hal_tx_completion_status *ts,
			struct dp_peer *peer, uint8_t ring_id)
{
	struct dp_pdev *pdev = peer->vdev->pdev;
	struct dp_soc *soc = NULL;
	uint8_t tid = ts->tid;
	uint32_t length;
	struct cdp_tid_tx_stats *tid_stats;

	if (!pdev)
		return;

	if (qdf_unlikely(tid >= CDP_MAX_DATA_TIDS))
		tid = CDP_MAX_DATA_TIDS - 1;

	tid_stats = &pdev->stats.tid_stats.tid_tx_stats[ring_id][tid];
	soc = pdev->soc;

	if (ts->release_src != HAL_TX_COMP_RELEASE_SOURCE_TQM) {
		dp_err("Release source is not from TQM");
		return;
	}

	length = qdf_nbuf_len(tx_desc->nbuf);
	DP_STATS_INC_PKT(peer, tx.comp_pkt, 1, length);

	if (qdf_unlikely(pdev->delay_stats_flag) ||
	    qdf_unlikely(dp_is_vdev_tx_delay_stats_enabled(peer->vdev)))
		dp_tx_compute_delay(peer->vdev, tx_desc, tid, ring_id);

	if (ts->status < CDP_MAX_TX_TQM_STATUS) {
		tid_stats->tqm_status_cnt[ts->status]++;
	}

	if (qdf_likely(ts->status == HAL_TX_TQM_RR_FRAME_ACKED)) {
		DP_STATS_INCC(peer, tx.retry_count, 1,
			      ts->transmit_cnt > 1);
		DP_STATS_INCC(peer, tx.multiple_retry_count, 1,
			      ts->transmit_cnt > 2);
		DP_STATS_INCC(peer, tx.ofdma, 1, ts->ofdma);
		DP_STATS_INCC(peer, tx.amsdu_cnt, 1, ts->msdu_part_of_amsdu);
		DP_STATS_INCC(peer, tx.non_amsdu_cnt, 1,
			      !ts->msdu_part_of_amsdu);

		/*
		 * Following Rate Statistics are updated from HTT PPDU events
		 * from FW. Return from here if HTT PPDU events are enabled.
		 */
		if (soc->process_tx_status)
			dp_tx_update_acked_rate_stats(peer, ts);

	} else {
		dp_update_no_ack_stats(tx_desc->nbuf, peer, ts);
	}
}

#ifdef QCA_LL_TX_FLOW_CONTROL_V2
/**
 * dp_tx_flow_pool_lock() - take flow pool lock
 * @soc: core txrx main context
 * @tx_desc: tx desc
 *
 * Return: None
 */
static inline
void dp_tx_flow_pool_lock(struct dp_soc *soc,
			  struct dp_tx_desc_s *tx_desc)
{
	struct dp_tx_desc_pool_s *pool;
	uint8_t desc_pool_id;

	desc_pool_id = tx_desc->pool_id;
	pool = &soc->tx_desc[desc_pool_id];

	qdf_spin_lock_bh(&pool->flow_pool_lock);
}

/**
 * dp_tx_flow_pool_unlock() - release flow pool lock
 * @soc: core txrx main context
 * @tx_desc: tx desc
 *
 * Return: None
 */
static inline
void dp_tx_flow_pool_unlock(struct dp_soc *soc,
			    struct dp_tx_desc_s *tx_desc)
{
	struct dp_tx_desc_pool_s *pool;
	uint8_t desc_pool_id;

	desc_pool_id = tx_desc->pool_id;
	pool = &soc->tx_desc[desc_pool_id];

	qdf_spin_unlock_bh(&pool->flow_pool_lock);
}
#else
static inline
void dp_tx_flow_pool_lock(struct dp_soc *soc, struct dp_tx_desc_s *tx_desc)
{
}

static inline
void dp_tx_flow_pool_unlock(struct dp_soc *soc, struct dp_tx_desc_s *tx_desc)
{
}
#endif

/**
 * dp_tx_notify_completion() - Notify tx completion for this desc
 * @soc: core txrx main context
 * @vdev: datapath vdev handle
 * @tx_desc: tx desc
 * @netbuf:  buffer
 * @status: tx status
 *
 * Return: none
 */
static inline void dp_tx_notify_completion(struct dp_soc *soc,
					   struct dp_vdev *vdev,
					   struct dp_tx_desc_s *tx_desc,
					   qdf_nbuf_t netbuf,
					   uint8_t status)
{
	void *osif_dev;
	ol_txrx_completion_fp tx_compl_cbk = NULL;
	uint16_t flag = BIT(QDF_TX_RX_STATUS_DOWNLOAD_SUCC);

	qdf_assert(tx_desc);

	if (!vdev ||
	    !vdev->osif_vdev) {
		return;
	}

	osif_dev = vdev->osif_vdev;
	tx_compl_cbk = vdev->tx_comp;

	if (status == HAL_TX_TQM_RR_FRAME_ACKED)
		flag |= BIT(QDF_TX_RX_STATUS_OK);

	if (tx_compl_cbk)
		tx_compl_cbk(netbuf, osif_dev, flag);
}

/** dp_tx_sojourn_stats_process() - Collect sojourn stats
 * @pdev: pdev handle
 * @tid: tid value
 * @txdesc_ts: timestamp from txdesc
 * @ppdu_id: ppdu id
 *
 * Return: none
 */
#ifdef FEATURE_PERPKT_INFO
static inline void dp_tx_sojourn_stats_process(struct dp_pdev *pdev,
					       struct dp_peer *peer,
					       uint8_t tid,
					       uint64_t txdesc_ts,
					       uint32_t ppdu_id)
{
	uint64_t delta_ms;
	struct cdp_tx_sojourn_stats *sojourn_stats;

	if (qdf_unlikely(!pdev->enhanced_stats_en))
		return;

	if (qdf_unlikely(tid == HTT_INVALID_TID ||
			 tid >= CDP_DATA_TID_MAX))
		return;

	if (qdf_unlikely(!pdev->sojourn_buf))
		return;

	sojourn_stats = (struct cdp_tx_sojourn_stats *)
		qdf_nbuf_data(pdev->sojourn_buf);

	sojourn_stats->cookie = (void *)peer->rdkstats_ctx;

	delta_ms = qdf_ktime_to_ms(qdf_ktime_get()) -
				txdesc_ts;
	qdf_ewma_tx_lag_add(&peer->avg_sojourn_msdu[tid],
			    delta_ms);
	sojourn_stats->sum_sojourn_msdu[tid] = delta_ms;
	sojourn_stats->num_msdus[tid] = 1;
	sojourn_stats->avg_sojourn_msdu[tid].internal =
		peer->avg_sojourn_msdu[tid].internal;
	dp_wdi_event_handler(WDI_EVENT_TX_SOJOURN_STAT, pdev->soc,
			     pdev->sojourn_buf, HTT_INVALID_PEER,
			     WDI_NO_VAL, pdev->pdev_id);
	sojourn_stats->sum_sojourn_msdu[tid] = 0;
	sojourn_stats->num_msdus[tid] = 0;
	sojourn_stats->avg_sojourn_msdu[tid].internal = 0;
}
#else
static inline void dp_tx_sojourn_stats_process(struct dp_pdev *pdev,
					       struct dp_peer *peer,
					       uint8_t tid,
					       uint64_t txdesc_ts,
					       uint32_t ppdu_id)
{
}
#endif

#ifdef WLAN_FEATURE_PKT_CAPTURE_V2
/**
 * dp_send_completion_to_pkt_capture() - send tx completion to packet capture
 * @soc: dp_soc handle
 * @desc: Tx Descriptor
 * @ts: HAL Tx completion descriptor contents
 *
 * This function is used to send tx completion to packet capture
 */
void dp_send_completion_to_pkt_capture(struct dp_soc *soc,
				       struct dp_tx_desc_s *desc,
				       struct hal_tx_completion_status *ts)
{
	dp_wdi_event_handler(WDI_EVENT_PKT_CAPTURE_TX_DATA, soc,
			     desc, ts->peer_id,
			     WDI_NO_VAL, desc->pdev->pdev_id);
}
#endif

/**
 * dp_tx_comp_process_desc() - Process tx descriptor and free associated nbuf
 * @soc: DP Soc handle
 * @tx_desc: software Tx descriptor
 * @ts : Tx completion status from HAL/HTT descriptor
 *
 * Return: none
 */
void
dp_tx_comp_process_desc(struct dp_soc *soc,
			struct dp_tx_desc_s *desc,
			struct hal_tx_completion_status *ts,
			struct dp_peer *peer)
{
	uint64_t time_latency = 0;

	/*
	 * m_copy/tx_capture modes are not supported for
	 * scatter gather packets
	 */
	if (qdf_unlikely(!!desc->pdev->latency_capture_enable)) {
		time_latency = (qdf_ktime_to_ms(qdf_ktime_real_get()) -
				desc->timestamp);
	}

	dp_send_completion_to_pkt_capture(soc, desc, ts);

	if (dp_tx_pkt_tracepoints_enabled())
		qdf_trace_dp_packet(desc->nbuf, QDF_TX,
				    desc->msdu_ext_desc ?
				    desc->msdu_ext_desc->tso_desc : NULL,
				    desc->timestamp);

	if (!(desc->msdu_ext_desc)) {
		dp_tx_enh_unmap(soc, desc);

		if (QDF_STATUS_SUCCESS ==
		    dp_monitor_tx_add_to_comp_queue(soc, desc, ts, peer)) {
			return;
		}

		if (QDF_STATUS_SUCCESS ==
		    dp_get_completion_indication_for_stack(soc,
							   desc->pdev,
							   peer, ts,
							   desc->nbuf,
							   time_latency)) {
			dp_send_completion_to_stack(soc,
						    desc->pdev,
						    ts->peer_id,
						    ts->ppdu_id,
						    desc->nbuf);
			return;
		}
	}

	desc->flags |= DP_TX_DESC_FLAG_COMPLETED_TX;
	dp_tx_comp_free_buf(soc, desc);
}

#ifdef DISABLE_DP_STATS
/**
 * dp_tx_update_connectivity_stats() - update tx connectivity stats
 * @soc: core txrx main context
 * @tx_desc: tx desc
 * @status: tx status
 *
 * Return: none
 */
static inline
void dp_tx_update_connectivity_stats(struct dp_soc *soc,
				     struct dp_vdev *vdev,
				     struct dp_tx_desc_s *tx_desc,
				     uint8_t status)
{
}
#else
static inline
void dp_tx_update_connectivity_stats(struct dp_soc *soc,
				     struct dp_vdev *vdev,
				     struct dp_tx_desc_s *tx_desc,
				     uint8_t status)
{
	void *osif_dev;
	ol_txrx_stats_rx_fp stats_cbk;
	uint8_t pkt_type;

	qdf_assert(tx_desc);

	if (!vdev ||
	    !vdev->osif_vdev ||
	    !vdev->stats_cb)
		return;

	osif_dev = vdev->osif_vdev;
	stats_cbk = vdev->stats_cb;

	stats_cbk(tx_desc->nbuf, osif_dev, PKT_TYPE_TX_HOST_FW_SENT, &pkt_type);
	if (status == HAL_TX_TQM_RR_FRAME_ACKED)
		stats_cbk(tx_desc->nbuf, osif_dev, PKT_TYPE_TX_ACK_CNT,
			  &pkt_type);
}
#endif

#ifdef WLAN_FEATURE_TSF_UPLINK_DELAY
void dp_set_delta_tsf(struct cdp_soc_t *soc_hdl, uint8_t vdev_id,
		      uint32_t delta_tsf)
{
	struct dp_soc *soc = cdp_soc_t_to_dp_soc(soc_hdl);
	struct dp_vdev *vdev = dp_vdev_get_ref_by_id(soc, vdev_id,
						     DP_MOD_ID_CDP);

	if (!vdev) {
		dp_err_rl("vdev %d does not exist", vdev_id);
		return;
	}

	vdev->delta_tsf = delta_tsf;
	dp_debug("vdev id %u delta_tsf %u", vdev_id, delta_tsf);

	dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_CDP);
}

QDF_STATUS dp_set_tsf_ul_delay_report(struct cdp_soc_t *soc_hdl,
				      uint8_t vdev_id, bool enable)
{
	struct dp_soc *soc = cdp_soc_t_to_dp_soc(soc_hdl);
	struct dp_vdev *vdev = dp_vdev_get_ref_by_id(soc, vdev_id,
						     DP_MOD_ID_CDP);

	if (!vdev) {
		dp_err_rl("vdev %d does not exist", vdev_id);
		return QDF_STATUS_E_FAILURE;
	}

	qdf_atomic_set(&vdev->ul_delay_report, enable);

	dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_CDP);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS dp_get_uplink_delay(struct cdp_soc_t *soc_hdl, uint8_t vdev_id,
			       uint32_t *val)
{
	struct dp_soc *soc = cdp_soc_t_to_dp_soc(soc_hdl);
	struct dp_vdev *vdev;
	uint32_t delay_accum;
	uint32_t pkts_accum;

	vdev = dp_vdev_get_ref_by_id(soc, vdev_id, DP_MOD_ID_CDP);
	if (!vdev) {
		dp_err_rl("vdev %d does not exist", vdev_id);
		return QDF_STATUS_E_FAILURE;
	}

	if (!qdf_atomic_read(&vdev->ul_delay_report)) {
		dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_CDP);
		return QDF_STATUS_E_FAILURE;
	}

	/* Average uplink delay based on current accumulated values */
	delay_accum = qdf_atomic_read(&vdev->ul_delay_accum);
	pkts_accum = qdf_atomic_read(&vdev->ul_pkts_accum);

	*val = delay_accum / pkts_accum;
	dp_debug("uplink_delay %u delay_accum %u pkts_accum %u", *val,
		 delay_accum, pkts_accum);

	/* Reset accumulated values to 0 */
	qdf_atomic_set(&vdev->ul_delay_accum, 0);
	qdf_atomic_set(&vdev->ul_pkts_accum, 0);

	dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_CDP);

	return QDF_STATUS_SUCCESS;
}

static void dp_tx_update_uplink_delay(struct dp_soc *soc, struct dp_vdev *vdev,
				      struct hal_tx_completion_status *ts)
{
	uint32_t buffer_ts;
	uint32_t delta_tsf;
	uint32_t ul_delay;

	/* Tx_rate_stats_info_valid is 0 and tsf is invalid then */
	if (!ts->valid)
		return;

	if (qdf_unlikely(!vdev)) {
		dp_info_rl("vdev is null or delete in progrss");
		return;
	}

	if (!qdf_atomic_read(&vdev->ul_delay_report))
		return;

	delta_tsf = vdev->delta_tsf;

	/* buffer_timestamp is in units of 1024 us and is [31:13] of
	 * WBM_RELEASE_RING_4. After left shift 10 bits, it's
	 * valid up to 29 bits.
	 */
	buffer_ts = ts->buffer_timestamp << 10;

	ul_delay = ts->tsf - buffer_ts - delta_tsf;
	ul_delay &= 0x1FFFFFFF; /* mask 29 BITS */
	if (ul_delay > 0x1000000) {
		dp_info_rl("----------------------\n"
			   "Tx completion status:\n"
			   "----------------------\n"
			   "release_src = %d\n"
			   "ppdu_id = 0x%x\n"
			   "release_reason = %d\n"
			   "tsf = %u (0x%x)\n"
			   "buffer_timestamp = %u (0x%x)\n"
			   "delta_tsf = %u (0x%x)\n",
			   ts->release_src, ts->ppdu_id, ts->status,
			   ts->tsf, ts->tsf, ts->buffer_timestamp,
			   ts->buffer_timestamp, delta_tsf, delta_tsf);
		return;
	}

	ul_delay /= 1000; /* in unit of ms */

	qdf_atomic_add(ul_delay, &vdev->ul_delay_accum);
	qdf_atomic_inc(&vdev->ul_pkts_accum);
}
#else /* !WLAN_FEATURE_TSF_UPLINK_DELAY */
static inline
void dp_tx_update_uplink_delay(struct dp_soc *soc, struct dp_vdev *vdev,
			       struct hal_tx_completion_status *ts)
{
}
#endif /* WLAN_FEATURE_TSF_UPLINK_DELAY */

/**
 * dp_tx_comp_process_tx_status() - Parse and Dump Tx completion status info
 * @soc: DP soc handle
 * @tx_desc: software descriptor head pointer
 * @ts: Tx completion status
 * @peer: peer handle
 * @ring_id: ring number
 *
 * Return: none
 */
void dp_tx_comp_process_tx_status(struct dp_soc *soc,
				  struct dp_tx_desc_s *tx_desc,
				  struct hal_tx_completion_status *ts,
				  struct dp_peer *peer, uint8_t ring_id)
{
	uint32_t length;
	qdf_ether_header_t *eh;
	struct dp_vdev *vdev = NULL;
	qdf_nbuf_t nbuf = tx_desc->nbuf;
	enum qdf_dp_tx_rx_status dp_status;

	if (!nbuf) {
		dp_info_rl("invalid tx descriptor. nbuf NULL");
		goto out;
	}

	eh = (qdf_ether_header_t *)qdf_nbuf_data(nbuf);
	length = qdf_nbuf_len(nbuf);

	dp_status = dp_tx_hw_to_qdf(ts->status);
	DPTRACE(qdf_dp_trace_ptr(tx_desc->nbuf,
				 QDF_DP_TRACE_LI_DP_FREE_PACKET_PTR_RECORD,
				 QDF_TRACE_DEFAULT_PDEV_ID,
				 qdf_nbuf_data_addr(nbuf),
				 sizeof(qdf_nbuf_data(nbuf)),
				 tx_desc->id, ts->status, dp_status));

	dp_tx_comp_debug("-------------------- \n"
			 "Tx Completion Stats: \n"
			 "-------------------- \n"
			 "ack_frame_rssi = %d \n"
			 "first_msdu = %d \n"
			 "last_msdu = %d \n"
			 "msdu_part_of_amsdu = %d \n"
			 "rate_stats valid = %d \n"
			 "bw = %d \n"
			 "pkt_type = %d \n"
			 "stbc = %d \n"
			 "ldpc = %d \n"
			 "sgi = %d \n"
			 "mcs = %d \n"
			 "ofdma = %d \n"
			 "tones_in_ru = %d \n"
			 "tsf = %d \n"
			 "ppdu_id = %d \n"
			 "transmit_cnt = %d \n"
			 "tid = %d \n"
			 "peer_id = %d\n"
			 "tx_status = %d\n",
			 ts->ack_frame_rssi, ts->first_msdu,
			 ts->last_msdu, ts->msdu_part_of_amsdu,
			 ts->valid, ts->bw, ts->pkt_type, ts->stbc,
			 ts->ldpc, ts->sgi, ts->mcs, ts->ofdma,
			 ts->tones_in_ru, ts->tsf, ts->ppdu_id,
			 ts->transmit_cnt, ts->tid, ts->peer_id,
			 ts->status);

	/* Update SoC level stats */
	DP_STATS_INCC(soc, tx.dropped_fw_removed, 1,
			(ts->status == HAL_TX_TQM_RR_REM_CMD_REM));

	if (!peer) {
		dp_info_rl("peer is null or deletion in progress");
		DP_STATS_INC_PKT(soc, tx.tx_invalid_peer, 1, length);
		goto out;
	}
	vdev = peer->vdev;

	dp_tx_update_connectivity_stats(soc, vdev, tx_desc, ts->status);
	dp_tx_update_uplink_delay(soc, vdev, ts);

	/* check tx complete notification */
	if (qdf_nbuf_tx_notify_comp_get(nbuf))
		dp_tx_notify_completion(soc, vdev, tx_desc,
					nbuf, ts->status);

	/* Update per-packet stats for mesh mode */
	if (qdf_unlikely(vdev->mesh_vdev) &&
			!(tx_desc->flags & DP_TX_DESC_FLAG_TO_FW))
		dp_tx_comp_fill_tx_completion_stats(tx_desc, ts);

	/* Update peer level stats */
	if (qdf_unlikely(peer->bss_peer && vdev->opmode == wlan_op_mode_ap)) {
		if (ts->status != HAL_TX_TQM_RR_REM_CMD_REM) {
			DP_STATS_INC_PKT(peer, tx.mcast, 1, length);
			DP_STATS_INC_PKT(peer, tx.tx_success, 1, length);
			if ((peer->vdev->tx_encap_type ==
				htt_cmn_pkt_type_ethernet) &&
				QDF_IS_ADDR_BROADCAST(eh->ether_dhost)) {
				DP_STATS_INC_PKT(peer, tx.bcast, 1, length);
			}
		}
	} else {
		if (ts->status == HAL_TX_TQM_RR_FRAME_ACKED) {
			DP_STATS_INC_PKT(peer, tx.ucast, 1, length);
			DP_STATS_INC_PKT(peer, tx.tx_success, 1, length);
			if (qdf_unlikely(peer->in_twt)) {
				DP_STATS_INC_PKT(peer,
						 tx.tx_success_twt,
						 1, length);
			}
		}
	}

	dp_tx_update_peer_stats(tx_desc, ts, peer, ring_id);
	dp_tx_update_peer_ext_stats(peer, tx_desc, ts->tid, ring_id);
	dp_tx_send_pktlog(soc, vdev->pdev, tx_desc, nbuf, dp_status);

#ifdef QCA_SUPPORT_RDK_STATS
	if (soc->rdkstats_enabled)
		dp_tx_sojourn_stats_process(vdev->pdev, peer, ts->tid,
					    tx_desc->timestamp,
					    ts->ppdu_id);
#endif

out:
	return;
}

#if defined(QCA_VDEV_STATS_HW_OFFLOAD_SUPPORT) && \
	defined(QCA_ENHANCED_STATS_SUPPORT)
/*
 * dp_tx_update_peer_basic_stats(): Update peer basic stats
 * @peer: Datapath peer handle
 * @length: Length of the packet
 * @tx_status: Tx status from TQM/FW
 * @update: enhanced flag value present in dp_pdev
 *
 * Return: none
 */
void dp_tx_update_peer_basic_stats(struct dp_peer *peer, uint32_t length,
				   uint8_t tx_status, bool update)
{
	if ((!peer->hw_txrx_stats_en) || update) {
		DP_STATS_INC_PKT(peer, tx.comp_pkt, 1, length);
		DP_STATS_INCC(peer, tx.tx_failed, 1,
			      tx_status != HAL_TX_TQM_RR_FRAME_ACKED);
	}
}
#elif defined(QCA_VDEV_STATS_HW_OFFLOAD_SUPPORT)
void dp_tx_update_peer_basic_stats(struct dp_peer *peer, uint32_t length,
				   uint8_t tx_status, bool update)
{
	if (!peer->hw_txrx_stats_en) {
		DP_STATS_INC_PKT(peer, tx.comp_pkt, 1, length);
		DP_STATS_INCC(peer, tx.tx_failed, 1,
			      tx_status != HAL_TX_TQM_RR_FRAME_ACKED);
	}
}

#else
void dp_tx_update_peer_basic_stats(struct dp_peer *peer, uint32_t length,
				   uint8_t tx_status, bool update)
{
	DP_STATS_INC_PKT(peer, tx.comp_pkt, 1, length);
	DP_STATS_INCC(peer, tx.tx_failed, 1,
		      tx_status != HAL_TX_TQM_RR_FRAME_ACKED);
}
#endif

/**
 * dp_tx_comp_process_desc_list() - Tx complete software descriptor handler
 * @soc: core txrx main context
 * @comp_head: software descriptor head pointer
 * @ring_id: ring number
 *
 * This function will process batch of descriptors reaped by dp_tx_comp_handler
 * and release the software descriptors after processing is complete
 *
 * Return: none
 */
static void
dp_tx_comp_process_desc_list(struct dp_soc *soc,
			     struct dp_tx_desc_s *comp_head, uint8_t ring_id)
{
	struct dp_tx_desc_s *desc;
	struct dp_tx_desc_s *next;
	struct hal_tx_completion_status ts;
	struct dp_peer *peer = NULL;
	uint16_t peer_id = DP_INVALID_PEER;

	desc = comp_head;

	while (desc) {
		if (peer_id != desc->peer_id) {
			if (peer)
				dp_peer_unref_delete(peer,
						     DP_MOD_ID_TX_COMP);
			peer_id = desc->peer_id;
			peer = dp_peer_get_ref_by_id(soc, peer_id,
						     DP_MOD_ID_TX_COMP);
		}

		if (qdf_likely(desc->flags & DP_TX_DESC_FLAG_SIMPLE)) {
			struct dp_pdev *pdev = desc->pdev;

			if (qdf_likely(peer))
				dp_tx_update_peer_basic_stats(peer,
							      desc->length,
							      desc->tx_status,
							      false);
			qdf_assert(pdev);
			dp_tx_outstanding_dec(pdev);

			/*
			 * Calling a QDF WRAPPER here is creating signifcant
			 * performance impact so avoided the wrapper call here
			 */
			next = desc->next;
			dp_tx_desc_history_add(soc, desc->dma_addr, desc->nbuf,
					       desc->id, DP_TX_COMP_UNMAP);
			qdf_nbuf_unmap_nbytes_single_paddr(soc->osdev,
							   desc->nbuf,
							   desc->dma_addr,
							   QDF_DMA_TO_DEVICE,
							   desc->length);
			qdf_nbuf_free(desc->nbuf);
			dp_tx_desc_free(soc, desc, desc->pool_id);
			desc = next;
			continue;
		}
		hal_tx_comp_get_status(&desc->comp, &ts, soc->hal_soc);

		dp_tx_comp_process_tx_status(soc, desc, &ts, peer, ring_id);

		dp_tx_comp_process_desc(soc, desc, &ts, peer);

		next = desc->next;

		dp_tx_desc_release(desc, desc->pool_id);
		desc = next;
	}
	if (peer)
		dp_peer_unref_delete(peer, DP_MOD_ID_TX_COMP);
}

#ifdef WLAN_FEATURE_RX_SOFTIRQ_TIME_LIMIT
static inline
bool dp_tx_comp_loop_pkt_limit_hit(struct dp_soc *soc, int num_reaped,
				   int max_reap_limit)
{
	bool limit_hit = false;

	limit_hit =
		(num_reaped >= max_reap_limit) ? true : false;

	if (limit_hit)
		DP_STATS_INC(soc, tx.tx_comp_loop_pkt_limit_hit, 1);

	return limit_hit;
}

static inline bool dp_tx_comp_enable_eol_data_check(struct dp_soc *soc)
{
	return soc->wlan_cfg_ctx->tx_comp_enable_eol_data_check;
}

static inline int dp_tx_comp_get_loop_pkt_limit(struct dp_soc *soc)
{
	struct wlan_cfg_dp_soc_ctxt *cfg = soc->wlan_cfg_ctx;

	return cfg->tx_comp_loop_pkt_limit;
}
#else
static inline
bool dp_tx_comp_loop_pkt_limit_hit(struct dp_soc *soc, int num_reaped,
				   int max_reap_limit)
{
	return false;
}

static inline bool dp_tx_comp_enable_eol_data_check(struct dp_soc *soc)
{
	return false;
}

static inline int dp_tx_comp_get_loop_pkt_limit(struct dp_soc *soc)
{
	return 0;
}
#endif

#ifdef WLAN_FEATURE_NEAR_FULL_IRQ
static inline int
dp_srng_test_and_update_nf_params(struct dp_soc *soc, struct dp_srng *dp_srng,
				  int *max_reap_limit)
{
	return soc->arch_ops.dp_srng_test_and_update_nf_params(soc, dp_srng,
							       max_reap_limit);
}
#else
static inline int
dp_srng_test_and_update_nf_params(struct dp_soc *soc, struct dp_srng *dp_srng,
				  int *max_reap_limit)
{
	return 0;
}
#endif

#ifdef DP_TX_TRACKING
void dp_tx_desc_check_corruption(struct dp_tx_desc_s *tx_desc)
{
	if ((tx_desc->magic != DP_TX_MAGIC_PATTERN_INUSE) &&
	    (tx_desc->magic != DP_TX_MAGIC_PATTERN_FREE)) {
		dp_err_rl("tx_desc %u is corrupted", tx_desc->id);
		qdf_trigger_self_recovery(NULL, QDF_TX_DESC_LEAK);
	}
}
#endif

uint32_t dp_tx_comp_handler(struct dp_intr *int_ctx, struct dp_soc *soc,
			    hal_ring_handle_t hal_ring_hdl, uint8_t ring_id,
			    uint32_t quota)
{
	void *tx_comp_hal_desc;
	uint8_t buffer_src;
	struct dp_tx_desc_s *tx_desc = NULL;
	struct dp_tx_desc_s *head_desc = NULL;
	struct dp_tx_desc_s *tail_desc = NULL;
	uint32_t num_processed = 0;
	uint32_t count;
	uint32_t num_avail_for_reap = 0;
	bool force_break = false;
	struct dp_srng *tx_comp_ring = &soc->tx_comp_ring[ring_id];
	int max_reap_limit, ring_near_full;

	DP_HIST_INIT();

more_data:
	/* Re-initialize local variables to be re-used */
	head_desc = NULL;
	tail_desc = NULL;
	count = 0;
	max_reap_limit = dp_tx_comp_get_loop_pkt_limit(soc);

	ring_near_full = dp_srng_test_and_update_nf_params(soc, tx_comp_ring,
							   &max_reap_limit);

	if (qdf_unlikely(dp_srng_access_start(int_ctx, soc, hal_ring_hdl))) {
		dp_err("HAL RING Access Failed -- %pK", hal_ring_hdl);
		return 0;
	}

	num_avail_for_reap = hal_srng_dst_num_valid(soc->hal_soc, hal_ring_hdl, 0);

	if (num_avail_for_reap >= quota)
		num_avail_for_reap = quota;

	dp_srng_dst_inv_cached_descs(soc, hal_ring_hdl, num_avail_for_reap);

	/* Find head descriptor from completion ring */
	while (qdf_likely(num_avail_for_reap--)) {

		tx_comp_hal_desc =  dp_srng_dst_get_next(soc, hal_ring_hdl);
		if (qdf_unlikely(!tx_comp_hal_desc))
			break;
		buffer_src = hal_tx_comp_get_buffer_source(soc->hal_soc,
							   tx_comp_hal_desc);

		/* If this buffer was not released by TQM or FW, then it is not
		 * Tx completion indication, assert */
		if (qdf_unlikely(buffer_src !=
					HAL_TX_COMP_RELEASE_SOURCE_TQM) &&
				 (qdf_unlikely(buffer_src !=
					HAL_TX_COMP_RELEASE_SOURCE_FW))) {
			uint8_t wbm_internal_error;

			dp_err_rl(
				"Tx comp release_src != TQM | FW but from %d",
				buffer_src);
			hal_dump_comp_desc(tx_comp_hal_desc);
			DP_STATS_INC(soc, tx.invalid_release_source, 1);

			/* When WBM sees NULL buffer_addr_info in any of
			 * ingress rings it sends an error indication,
			 * with wbm_internal_error=1, to a specific ring.
			 * The WBM2SW ring used to indicate these errors is
			 * fixed in HW, and that ring is being used as Tx
			 * completion ring. These errors are not related to
			 * Tx completions, and should just be ignored
			 */
			wbm_internal_error = hal_get_wbm_internal_error(
							soc->hal_soc,
							tx_comp_hal_desc);

			if (wbm_internal_error) {
				dp_err_rl("Tx comp wbm_internal_error!!");
				DP_STATS_INC(soc, tx.wbm_internal_error[WBM_INT_ERROR_ALL], 1);

				if (HAL_TX_COMP_RELEASE_SOURCE_REO ==
								buffer_src)
					dp_handle_wbm_internal_error(
						soc,
						tx_comp_hal_desc,
						hal_tx_comp_get_buffer_type(
							tx_comp_hal_desc));

			} else {
				dp_err_rl("Tx comp wbm_internal_error false");
				DP_STATS_INC(soc, tx.non_wbm_internal_err, 1);
			}
			continue;
		}

		soc->arch_ops.tx_comp_get_params_from_hal_desc(soc,
							       tx_comp_hal_desc,
							       &tx_desc);
		if (!tx_desc) {
			dp_err("unable to retrieve tx_desc!");
			QDF_BUG(0);
			continue;
		}
		tx_desc->buffer_src = buffer_src;
		/*
		 * If the release source is FW, process the HTT status
		 */
		if (qdf_unlikely(buffer_src ==
					HAL_TX_COMP_RELEASE_SOURCE_FW)) {
			uint8_t htt_tx_status[HAL_TX_COMP_HTT_STATUS_LEN];

			hal_tx_comp_get_htt_desc(tx_comp_hal_desc,
					htt_tx_status);
			/* Collect hw completion contents */
			hal_tx_comp_desc_sync(tx_comp_hal_desc,
					      &tx_desc->comp, 1);
			soc->arch_ops.dp_tx_process_htt_completion(
							soc,
							tx_desc,
							htt_tx_status,
							ring_id);
		} else {
			tx_desc->tx_status =
				hal_tx_comp_get_tx_status(tx_comp_hal_desc);
			tx_desc->buffer_src = buffer_src;
			/*
			 * If the fast completion mode is enabled extended
			 * metadata from descriptor is not copied
			 */
			if (qdf_likely(tx_desc->flags &
						DP_TX_DESC_FLAG_SIMPLE))
				goto add_to_pool;

			/*
			 * If the descriptor is already freed in vdev_detach,
			 * continue to next descriptor
			 */
			if (qdf_unlikely
				((tx_desc->vdev_id == DP_INVALID_VDEV_ID) &&
				 !tx_desc->flags)) {
				dp_tx_comp_info_rl("Descriptor freed in vdev_detach %d",
						   tx_desc->id);
				DP_STATS_INC(soc, tx.tx_comp_exception, 1);
				dp_tx_desc_check_corruption(tx_desc);
				continue;
			}

			if (qdf_unlikely(tx_desc->pdev->is_pdev_down)) {
				dp_tx_comp_info_rl("pdev in down state %d",
						   tx_desc->id);
				tx_desc->flags |= DP_TX_DESC_FLAG_TX_COMP_ERR;
				dp_tx_comp_free_buf(soc, tx_desc);
				dp_tx_desc_release(tx_desc, tx_desc->pool_id);
				goto next_desc;
			}

			if (!(tx_desc->flags & DP_TX_DESC_FLAG_ALLOCATED) ||
				!(tx_desc->flags & DP_TX_DESC_FLAG_QUEUED_TX)) {
				dp_tx_comp_alert("Txdesc invalid, flgs = %x,id = %d",
						 tx_desc->flags, tx_desc->id);
				qdf_assert_always(0);
			}

			/* Collect hw completion contents */
			hal_tx_comp_desc_sync(tx_comp_hal_desc,
					      &tx_desc->comp, 1);
add_to_pool:
			DP_HIST_PACKET_COUNT_INC(tx_desc->pdev->pdev_id);

			/* First ring descriptor on the cycle */
			if (!head_desc) {
				head_desc = tx_desc;
				tail_desc = tx_desc;
			}

			tail_desc->next = tx_desc;
			tx_desc->next = NULL;
			tail_desc = tx_desc;
		}
next_desc:
		num_processed += !(count & DP_TX_NAPI_BUDGET_DIV_MASK);

		/*
		 * Processed packet count is more than given quota
		 * stop to processing
		 */

		count++;

		if (dp_tx_comp_loop_pkt_limit_hit(soc, count, max_reap_limit))
			break;
	}

	dp_srng_access_end(int_ctx, soc, hal_ring_hdl);

	/* Process the reaped descriptors */
	if (head_desc)
		dp_tx_comp_process_desc_list(soc, head_desc, ring_id);

	DP_STATS_INC(soc, tx.tx_comp[ring_id], count);

	/*
	 * If we are processing in near-full condition, there are 3 scenario
	 * 1) Ring entries has reached critical state
	 * 2) Ring entries are still near high threshold
	 * 3) Ring entries are below the safe level
	 *
	 * One more loop will move te state to normal processing and yield
	 */
	if (ring_near_full)
		goto more_data;

	if (dp_tx_comp_enable_eol_data_check(soc)) {

		if (num_processed >= quota)
			force_break = true;

		if (!force_break &&
		    hal_srng_dst_peek_sync_locked(soc->hal_soc,
						  hal_ring_hdl)) {
			DP_STATS_INC(soc, tx.hp_oos2, 1);
			if (!hif_exec_should_yield(soc->hif_handle,
						   int_ctx->dp_intr_id))
				goto more_data;
		}
	}
	DP_TX_HIST_STATS_PER_PDEV();

	return num_processed;
}

#ifdef FEATURE_WLAN_TDLS
qdf_nbuf_t dp_tx_non_std(struct cdp_soc_t *soc_hdl, uint8_t vdev_id,
			 enum ol_tx_spec tx_spec, qdf_nbuf_t msdu_list)
{
	struct dp_soc *soc = cdp_soc_t_to_dp_soc(soc_hdl);
	struct dp_vdev *vdev = dp_vdev_get_ref_by_id(soc, vdev_id,
						     DP_MOD_ID_TDLS);

	if (!vdev) {
		dp_err("vdev handle for id %d is NULL", vdev_id);
		return NULL;
	}

	if (tx_spec & OL_TX_SPEC_NO_FREE)
		vdev->is_tdls_frame = true;
	dp_vdev_unref_delete(soc, vdev, DP_MOD_ID_TDLS);

	return dp_tx_send(soc_hdl, vdev_id, msdu_list);
}
#endif

/**
 * dp_tx_vdev_attach() - attach vdev to dp tx
 * @vdev: virtual device instance
 *
 * Return: QDF_STATUS_SUCCESS: success
 *         QDF_STATUS_E_RESOURCES: Error return
 */
QDF_STATUS dp_tx_vdev_attach(struct dp_vdev *vdev)
{
	int pdev_id;
	/*
	 * Fill HTT TCL Metadata with Vdev ID and MAC ID
	 */
	HTT_TX_TCL_METADATA_TYPE_SET(vdev->htt_tcl_metadata,
				     HTT_TCL_METADATA_TYPE_VDEV_BASED);

	HTT_TX_TCL_METADATA_VDEV_ID_SET(vdev->htt_tcl_metadata,
					vdev->vdev_id);

	pdev_id =
		dp_get_target_pdev_id_for_host_pdev_id(vdev->pdev->soc,
						       vdev->pdev->pdev_id);
	HTT_TX_TCL_METADATA_PDEV_ID_SET(vdev->htt_tcl_metadata, pdev_id);

	/*
	 * Set HTT Extension Valid bit to 0 by default
	 */
	HTT_TX_TCL_METADATA_VALID_HTT_SET(vdev->htt_tcl_metadata, 0);

	dp_tx_vdev_update_search_flags(vdev);

	return QDF_STATUS_SUCCESS;
}

#ifndef FEATURE_WDS
static inline bool dp_tx_da_search_override(struct dp_vdev *vdev)
{
	return false;
}
#endif

/**
 * dp_tx_vdev_update_search_flags() - Update vdev flags as per opmode
 * @vdev: virtual device instance
 *
 * Return: void
 *
 */
void dp_tx_vdev_update_search_flags(struct dp_vdev *vdev)
{
	struct dp_soc *soc = vdev->pdev->soc;

	/*
	 * Enable both AddrY (SA based search) and AddrX (Da based search)
	 * for TDLS link
	 *
	 * Enable AddrY (SA based search) only for non-WDS STA and
	 * ProxySTA VAP (in HKv1) modes.
	 *
	 * In all other VAP modes, only DA based search should be
	 * enabled
	 */
	if (vdev->opmode == wlan_op_mode_sta &&
	    vdev->tdls_link_connected)
		vdev->hal_desc_addr_search_flags =
			(HAL_TX_DESC_ADDRX_EN | HAL_TX_DESC_ADDRY_EN);
	else if ((vdev->opmode == wlan_op_mode_sta) &&
		 !dp_tx_da_search_override(vdev))
		vdev->hal_desc_addr_search_flags = HAL_TX_DESC_ADDRY_EN;
	else
		vdev->hal_desc_addr_search_flags = HAL_TX_DESC_ADDRX_EN;

	if (vdev->opmode == wlan_op_mode_sta && !vdev->tdls_link_connected)
		vdev->search_type = soc->sta_mode_search_policy;
	else
		vdev->search_type = HAL_TX_ADDR_SEARCH_DEFAULT;
}

static inline bool
dp_is_tx_desc_flush_match(struct dp_pdev *pdev,
			  struct dp_vdev *vdev,
			  struct dp_tx_desc_s *tx_desc)
{
	if (!(tx_desc && (tx_desc->flags & DP_TX_DESC_FLAG_ALLOCATED)))
		return false;

	/*
	 * if vdev is given, then only check whether desc
	 * vdev match. if vdev is NULL, then check whether
	 * desc pdev match.
	 */
	return vdev ? (tx_desc->vdev_id == vdev->vdev_id) :
		(tx_desc->pdev == pdev);
}

#ifdef QCA_LL_TX_FLOW_CONTROL_V2
/**
 * dp_tx_desc_flush() - release resources associated
 *                      to TX Desc
 *
 * @dp_pdev: Handle to DP pdev structure
 * @vdev: virtual device instance
 * NULL: no specific Vdev is required and check all allcated TX desc
 * on this pdev.
 * Non-NULL: only check the allocated TX Desc associated to this Vdev.
 *
 * @force_free:
 * true: flush the TX desc.
 * false: only reset the Vdev in each allocated TX desc
 * that associated to current Vdev.
 *
 * This function will go through the TX desc pool to flush
 * the outstanding TX data or reset Vdev to NULL in associated TX
 * Desc.
 */
void dp_tx_desc_flush(struct dp_pdev *pdev, struct dp_vdev *vdev,
		      bool force_free)
{
	uint8_t i;
	uint32_t j;
	uint32_t num_desc, page_id, offset;
	uint16_t num_desc_per_page;
	struct dp_soc *soc = pdev->soc;
	struct dp_tx_desc_s *tx_desc = NULL;
	struct dp_tx_desc_pool_s *tx_desc_pool = NULL;

	if (!vdev && !force_free) {
		dp_err("Reset TX desc vdev, Vdev param is required!");
		return;
	}

	for (i = 0; i < MAX_TXDESC_POOLS; i++) {
		tx_desc_pool = &soc->tx_desc[i];
		if (!(tx_desc_pool->pool_size) ||
		    IS_TX_DESC_POOL_STATUS_INACTIVE(tx_desc_pool) ||
		    !(tx_desc_pool->desc_pages.cacheable_pages))
			continue;

		/*
		 * Add flow pool lock protection in case pool is freed
		 * due to all tx_desc is recycled when handle TX completion.
		 * this is not necessary when do force flush as:
		 * a. double lock will happen if dp_tx_desc_release is
		 *    also trying to acquire it.
		 * b. dp interrupt has been disabled before do force TX desc
		 *    flush in dp_pdev_deinit().
		 */
		if (!force_free)
			qdf_spin_lock_bh(&tx_desc_pool->flow_pool_lock);
		num_desc = tx_desc_pool->pool_size;
		num_desc_per_page =
			tx_desc_pool->desc_pages.num_element_per_page;
		for (j = 0; j < num_desc; j++) {
			page_id = j / num_desc_per_page;
			offset = j % num_desc_per_page;

			if (qdf_unlikely(!(tx_desc_pool->
					 desc_pages.cacheable_pages)))
				break;

			tx_desc = dp_tx_desc_find(soc, i, page_id, offset);

			if (dp_is_tx_desc_flush_match(pdev, vdev, tx_desc)) {
				/*
				 * Free TX desc if force free is
				 * required, otherwise only reset vdev
				 * in this TX desc.
				 */
				if (force_free) {
					tx_desc->flags |= DP_TX_DESC_FLAG_FLUSH;
					dp_tx_comp_free_buf(soc, tx_desc);
					dp_tx_desc_release(tx_desc, i);
				} else {
					tx_desc->vdev_id = DP_INVALID_VDEV_ID;
				}
			}
		}
		if (!force_free)
			qdf_spin_unlock_bh(&tx_desc_pool->flow_pool_lock);
	}
}
#else /* QCA_LL_TX_FLOW_CONTROL_V2! */
/**
 * dp_tx_desc_reset_vdev() - reset vdev to NULL in TX Desc
 *
 * @soc: Handle to DP soc structure
 * @tx_desc: pointer of one TX desc
 * @desc_pool_id: TX Desc pool id
 */
static inline void
dp_tx_desc_reset_vdev(struct dp_soc *soc, struct dp_tx_desc_s *tx_desc,
		      uint8_t desc_pool_id)
{
	TX_DESC_LOCK_LOCK(&soc->tx_desc[desc_pool_id].lock);

	tx_desc->vdev_id = DP_INVALID_VDEV_ID;

	TX_DESC_LOCK_UNLOCK(&soc->tx_desc[desc_pool_id].lock);
}

void dp_tx_desc_flush(struct dp_pdev *pdev, struct dp_vdev *vdev,
		      bool force_free)
{
	uint8_t i, num_pool;
	uint32_t j;
	uint32_t num_desc, page_id, offset;
	uint16_t num_desc_per_page;
	struct dp_soc *soc = pdev->soc;
	struct dp_tx_desc_s *tx_desc = NULL;
	struct dp_tx_desc_pool_s *tx_desc_pool = NULL;

	if (!vdev && !force_free) {
		dp_err("Reset TX desc vdev, Vdev param is required!");
		return;
	}

	num_desc = wlan_cfg_get_num_tx_desc(soc->wlan_cfg_ctx);
	num_pool = wlan_cfg_get_num_tx_desc_pool(soc->wlan_cfg_ctx);

	for (i = 0; i < num_pool; i++) {
		tx_desc_pool = &soc->tx_desc[i];
		if (!tx_desc_pool->desc_pages.cacheable_pages)
			continue;

		num_desc_per_page =
			tx_desc_pool->desc_pages.num_element_per_page;
		for (j = 0; j < num_desc; j++) {
			page_id = j / num_desc_per_page;
			offset = j % num_desc_per_page;
			tx_desc = dp_tx_desc_find(soc, i, page_id, offset);

			if (dp_is_tx_desc_flush_match(pdev, vdev, tx_desc)) {
				if (force_free) {
					tx_desc->flags |= DP_TX_DESC_FLAG_FLUSH;
					dp_tx_comp_free_buf(soc, tx_desc);
					dp_tx_desc_release(tx_desc, i);
				} else {
					dp_tx_desc_reset_vdev(soc, tx_desc,
							      i);
				}
			}
		}
	}
}
#endif /* !QCA_LL_TX_FLOW_CONTROL_V2 */

/**
 * dp_tx_vdev_detach() - detach vdev from dp tx
 * @vdev: virtual device instance
 *
 * Return: QDF_STATUS_SUCCESS: success
 *         QDF_STATUS_E_RESOURCES: Error return
 */
QDF_STATUS dp_tx_vdev_detach(struct dp_vdev *vdev)
{
	struct dp_pdev *pdev = vdev->pdev;

	/* Reset TX desc associated to this Vdev as NULL */
	dp_tx_desc_flush(pdev, vdev, false);

	return QDF_STATUS_SUCCESS;
}

#ifdef QCA_LL_TX_FLOW_CONTROL_V2
/* Pools will be allocated dynamically */
static QDF_STATUS dp_tx_alloc_static_pools(struct dp_soc *soc, int num_pool,
					   int num_desc)
{
	uint8_t i;

	for (i = 0; i < num_pool; i++) {
		qdf_spinlock_create(&soc->tx_desc[i].flow_pool_lock);
		soc->tx_desc[i].status = FLOW_POOL_INACTIVE;
	}

	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dp_tx_init_static_pools(struct dp_soc *soc, int num_pool,
					  int num_desc)
{
	return QDF_STATUS_SUCCESS;
}

static void dp_tx_deinit_static_pools(struct dp_soc *soc, int num_pool)
{
}

static void dp_tx_delete_static_pools(struct dp_soc *soc, int num_pool)
{
	uint8_t i;

	for (i = 0; i < num_pool; i++)
		qdf_spinlock_destroy(&soc->tx_desc[i].flow_pool_lock);
}
#else /* QCA_LL_TX_FLOW_CONTROL_V2! */
static QDF_STATUS dp_tx_alloc_static_pools(struct dp_soc *soc, int num_pool,
					   int num_desc)
{
	uint8_t i, count;

	/* Allocate software Tx descriptor pools */
	for (i = 0; i < num_pool; i++) {
		if (dp_tx_desc_pool_alloc(soc, i, num_desc)) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				  FL("Tx Desc Pool alloc %d failed %pK"),
				  i, soc);
			goto fail;
		}
	}
	return QDF_STATUS_SUCCESS;

fail:
	for (count = 0; count < i; count++)
		dp_tx_desc_pool_free(soc, count);

	return QDF_STATUS_E_NOMEM;
}

static QDF_STATUS dp_tx_init_static_pools(struct dp_soc *soc, int num_pool,
					  int num_desc)
{
	uint8_t i;
	for (i = 0; i < num_pool; i++) {
		if (dp_tx_desc_pool_init(soc, i, num_desc)) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				  FL("Tx Desc Pool init %d failed %pK"),
				  i, soc);
			return QDF_STATUS_E_NOMEM;
		}
	}
	return QDF_STATUS_SUCCESS;
}

static void dp_tx_deinit_static_pools(struct dp_soc *soc, int num_pool)
{
	uint8_t i;

	for (i = 0; i < num_pool; i++)
		dp_tx_desc_pool_deinit(soc, i);
}

static void dp_tx_delete_static_pools(struct dp_soc *soc, int num_pool)
{
	uint8_t i;

	for (i = 0; i < num_pool; i++)
		dp_tx_desc_pool_free(soc, i);
}

#endif /* !QCA_LL_TX_FLOW_CONTROL_V2 */

/**
 * dp_tx_tso_cmn_desc_pool_deinit() - de-initialize TSO descriptors
 * @soc: core txrx main context
 * @num_pool: number of pools
 *
 */
void dp_tx_tso_cmn_desc_pool_deinit(struct dp_soc *soc, uint8_t num_pool)
{
	dp_tx_tso_desc_pool_deinit(soc, num_pool);
	dp_tx_tso_num_seg_pool_deinit(soc, num_pool);
}

/**
 * dp_tx_tso_cmn_desc_pool_free() - free TSO descriptors
 * @soc: core txrx main context
 * @num_pool: number of pools
 *
 */
void dp_tx_tso_cmn_desc_pool_free(struct dp_soc *soc, uint8_t num_pool)
{
	dp_tx_tso_desc_pool_free(soc, num_pool);
	dp_tx_tso_num_seg_pool_free(soc, num_pool);
}

/**
 * dp_soc_tx_desc_sw_pools_free() - free all TX descriptors
 * @soc: core txrx main context
 *
 * This function frees all tx related descriptors as below
 * 1. Regular TX descriptors (static pools)
 * 2. extension TX descriptors (used for ME, RAW, TSO etc...)
 * 3. TSO descriptors
 *
 */
void dp_soc_tx_desc_sw_pools_free(struct dp_soc *soc)
{
	uint8_t num_pool;

	num_pool = wlan_cfg_get_num_tx_desc_pool(soc->wlan_cfg_ctx);

	dp_tx_tso_cmn_desc_pool_free(soc, num_pool);
	dp_tx_ext_desc_pool_free(soc, num_pool);
	dp_tx_delete_static_pools(soc, num_pool);
}

/**
 * dp_soc_tx_desc_sw_pools_deinit() - de-initialize all TX descriptors
 * @soc: core txrx main context
 *
 * This function de-initializes all tx related descriptors as below
 * 1. Regular TX descriptors (static pools)
 * 2. extension TX descriptors (used for ME, RAW, TSO etc...)
 * 3. TSO descriptors
 *
 */
void dp_soc_tx_desc_sw_pools_deinit(struct dp_soc *soc)
{
	uint8_t num_pool;

	num_pool = wlan_cfg_get_num_tx_desc_pool(soc->wlan_cfg_ctx);

	dp_tx_flow_control_deinit(soc);
	dp_tx_tso_cmn_desc_pool_deinit(soc, num_pool);
	dp_tx_ext_desc_pool_deinit(soc, num_pool);
	dp_tx_deinit_static_pools(soc, num_pool);
}

/**
 * dp_tso_attach() - TSO attach handler
 * @txrx_soc: Opaque Dp handle
 *
 * Reserve TSO descriptor buffers
 *
 * Return: QDF_STATUS_E_FAILURE on failure or
 * QDF_STATUS_SUCCESS on success
 */
QDF_STATUS dp_tx_tso_cmn_desc_pool_alloc(struct dp_soc *soc,
					 uint8_t num_pool,
					 uint16_t num_desc)
{
	if (dp_tx_tso_desc_pool_alloc(soc, num_pool, num_desc)) {
		dp_err("TSO Desc Pool alloc %d failed %pK", num_pool, soc);
		return QDF_STATUS_E_FAILURE;
	}

	if (dp_tx_tso_num_seg_pool_alloc(soc, num_pool, num_desc)) {
		dp_err("TSO Num of seg Pool alloc %d failed %pK",
		       num_pool, soc);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * dp_tx_tso_cmn_desc_pool_init() - TSO cmn desc pool init
 * @soc: DP soc handle
 * @num_pool: Number of pools
 * @num_desc: Number of descriptors
 *
 * Initialize TSO descriptor pools
 *
 * Return: QDF_STATUS_E_FAILURE on failure or
 * QDF_STATUS_SUCCESS on success
 */

QDF_STATUS dp_tx_tso_cmn_desc_pool_init(struct dp_soc *soc,
					uint8_t num_pool,
					uint16_t num_desc)
{
	if (dp_tx_tso_desc_pool_init(soc, num_pool, num_desc)) {
		dp_err("TSO Desc Pool alloc %d failed %pK", num_pool, soc);
		return QDF_STATUS_E_FAILURE;
	}

	if (dp_tx_tso_num_seg_pool_init(soc, num_pool, num_desc)) {
		dp_err("TSO Num of seg Pool alloc %d failed %pK",
		       num_pool, soc);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * dp_soc_tx_desc_sw_pools_alloc() - Allocate tx descriptor pool memory
 * @soc: core txrx main context
 *
 * This function allocates memory for following descriptor pools
 * 1. regular sw tx descriptor pools (static pools)
 * 2. TX extension descriptor pools (ME, RAW, TSO etc...)
 * 3. TSO descriptor pools
 *
 * Return: QDF_STATUS_SUCCESS: success
 *         QDF_STATUS_E_RESOURCES: Error return
 */
QDF_STATUS dp_soc_tx_desc_sw_pools_alloc(struct dp_soc *soc)
{
	uint8_t num_pool;
	uint32_t num_desc;
	uint32_t num_ext_desc;

	num_pool = wlan_cfg_get_num_tx_desc_pool(soc->wlan_cfg_ctx);
	num_desc = wlan_cfg_get_num_tx_desc(soc->wlan_cfg_ctx);
	num_ext_desc = wlan_cfg_get_num_tx_ext_desc(soc->wlan_cfg_ctx);

	QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO,
		  "%s Tx Desc Alloc num_pool = %d, descs = %d",
		  __func__, num_pool, num_desc);

	if ((num_pool > MAX_TXDESC_POOLS) ||
	    (num_desc > WLAN_CFG_NUM_TX_DESC_MAX))
		goto fail1;

	if (dp_tx_alloc_static_pools(soc, num_pool, num_desc))
		goto fail1;

	if (dp_tx_ext_desc_pool_alloc(soc, num_pool, num_ext_desc))
		goto fail2;

	if (wlan_cfg_is_tso_desc_attach_defer(soc->wlan_cfg_ctx))
		return QDF_STATUS_SUCCESS;

	if (dp_tx_tso_cmn_desc_pool_alloc(soc, num_pool, num_ext_desc))
		goto fail3;

	return QDF_STATUS_SUCCESS;

fail3:
	dp_tx_ext_desc_pool_free(soc, num_pool);
fail2:
	dp_tx_delete_static_pools(soc, num_pool);
fail1:
	return QDF_STATUS_E_RESOURCES;
}

/**
 * dp_soc_tx_desc_sw_pools_init() - Initialise TX descriptor pools
 * @soc: core txrx main context
 *
 * This function initializes the following TX descriptor pools
 * 1. regular sw tx descriptor pools (static pools)
 * 2. TX extension descriptor pools (ME, RAW, TSO etc...)
 * 3. TSO descriptor pools
 *
 * Return: QDF_STATUS_SUCCESS: success
 *	   QDF_STATUS_E_RESOURCES: Error return
 */
QDF_STATUS dp_soc_tx_desc_sw_pools_init(struct dp_soc *soc)
{
	uint8_t num_pool;
	uint32_t num_desc;
	uint32_t num_ext_desc;

	num_pool = wlan_cfg_get_num_tx_desc_pool(soc->wlan_cfg_ctx);
	num_desc = wlan_cfg_get_num_tx_desc(soc->wlan_cfg_ctx);
	num_ext_desc = wlan_cfg_get_num_tx_ext_desc(soc->wlan_cfg_ctx);

	if (dp_tx_init_static_pools(soc, num_pool, num_desc))
		goto fail1;

	if (dp_tx_ext_desc_pool_init(soc, num_pool, num_ext_desc))
		goto fail2;

	if (wlan_cfg_is_tso_desc_attach_defer(soc->wlan_cfg_ctx))
		return QDF_STATUS_SUCCESS;

	if (dp_tx_tso_cmn_desc_pool_init(soc, num_pool, num_ext_desc))
		goto fail3;

	dp_tx_flow_control_init(soc);
	soc->process_tx_status = CONFIG_PROCESS_TX_STATUS;
	return QDF_STATUS_SUCCESS;

fail3:
	dp_tx_ext_desc_pool_deinit(soc, num_pool);
fail2:
	dp_tx_deinit_static_pools(soc, num_pool);
fail1:
	return QDF_STATUS_E_RESOURCES;
}

/**
 * dp_tso_soc_attach() - Allocate and initialize TSO descriptors
 * @txrx_soc: dp soc handle
 *
 * Return: QDF_STATUS - QDF_STATUS_SUCCESS
 *			QDF_STATUS_E_FAILURE
 */
QDF_STATUS dp_tso_soc_attach(struct cdp_soc_t *txrx_soc)
{
	struct dp_soc *soc = (struct dp_soc *)txrx_soc;
	uint8_t num_pool;
	uint32_t num_desc;
	uint32_t num_ext_desc;

	num_pool = wlan_cfg_get_num_tx_desc_pool(soc->wlan_cfg_ctx);
	num_desc = wlan_cfg_get_num_tx_desc(soc->wlan_cfg_ctx);
	num_ext_desc = wlan_cfg_get_num_tx_ext_desc(soc->wlan_cfg_ctx);

	if (dp_tx_tso_cmn_desc_pool_alloc(soc, num_pool, num_ext_desc))
		return QDF_STATUS_E_FAILURE;

	if (dp_tx_tso_cmn_desc_pool_init(soc, num_pool, num_ext_desc))
		return QDF_STATUS_E_FAILURE;

	return QDF_STATUS_SUCCESS;
}

/**
 * dp_tso_soc_detach() - de-initialize and free the TSO descriptors
 * @txrx_soc: dp soc handle
 *
 * Return: QDF_STATUS - QDF_STATUS_SUCCESS
 */
QDF_STATUS dp_tso_soc_detach(struct cdp_soc_t *txrx_soc)
{
	struct dp_soc *soc = (struct dp_soc *)txrx_soc;
	uint8_t num_pool = wlan_cfg_get_num_tx_desc_pool(soc->wlan_cfg_ctx);

	dp_tx_tso_cmn_desc_pool_deinit(soc, num_pool);
	dp_tx_tso_cmn_desc_pool_free(soc, num_pool);

	return QDF_STATUS_SUCCESS;
}

