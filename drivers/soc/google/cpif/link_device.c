// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#include <linux/irq.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/kallsyms.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/shm_ipc.h>
#include <linux/mcu_ipc.h>
#include <linux/modem_notifier.h>
#include <linux/of_reserved_mem.h>
#if IS_ENABLED(CONFIG_PCI_EXYNOS)
#include <linux/exynos-pci-ctrl.h>
#endif
#if IS_ENABLED(CONFIG_ECT)
#include <soc/google/ect_parser.h>
#endif
#include <soc/google/cal-if.h>
#include <linux/soc/samsung/exynos-smc.h>
#include <trace/events/napi.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "link_device.h"
#include "modem_dump.h"
#include "link_ctrlmsg_iosm.h"
#include "modem_ctrl.h"
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
#include "s51xx_pcie.h"
#if IS_ENABLED(CONFIG_GS_S2MPU)
#include <soc/google/s2mpu.h>
#endif
#endif
#include "dit.h"

#define MIF_TX_QUOTA 64

#if !IS_ENABLED(CONFIG_CP_SECURE_BOOT)
#define CRC32_XINIT 0xFFFFFFFFL		/* initial value */
#define CRC32_XOROT 0xFFFFFFFFL		/* final xor value */

static const unsigned long CRC32_TABLE[256] = {
	0x00000000L, 0x77073096L, 0xEE0E612CL, 0x990951BAL, 0x076DC419L,
	0x706AF48FL, 0xE963A535L, 0x9E6495A3L, 0x0EDB8832L, 0x79DCB8A4L,
	0xE0D5E91EL, 0x97D2D988L, 0x09B64C2BL, 0x7EB17CBDL, 0xE7B82D07L,
	0x90BF1D91L, 0x1DB71064L, 0x6AB020F2L, 0xF3B97148L, 0x84BE41DEL,
	0x1ADAD47DL, 0x6DDDE4EBL, 0xF4D4B551L, 0x83D385C7L, 0x136C9856L,
	0x646BA8C0L, 0xFD62F97AL, 0x8A65C9ECL, 0x14015C4FL, 0x63066CD9L,
	0xFA0F3D63L, 0x8D080DF5L, 0x3B6E20C8L, 0x4C69105EL, 0xD56041E4L,
	0xA2677172L, 0x3C03E4D1L, 0x4B04D447L, 0xD20D85FDL, 0xA50AB56BL,
	0x35B5A8FAL, 0x42B2986CL, 0xDBBBC9D6L, 0xACBCF940L, 0x32D86CE3L,
	0x45DF5C75L, 0xDCD60DCFL, 0xABD13D59L, 0x26D930ACL, 0x51DE003AL,
	0xC8D75180L, 0xBFD06116L, 0x21B4F4B5L, 0x56B3C423L, 0xCFBA9599L,
	0xB8BDA50FL, 0x2802B89EL, 0x5F058808L, 0xC60CD9B2L, 0xB10BE924L,
	0x2F6F7C87L, 0x58684C11L, 0xC1611DABL, 0xB6662D3DL, 0x76DC4190L,
	0x01DB7106L, 0x98D220BCL, 0xEFD5102AL, 0x71B18589L, 0x06B6B51FL,
	0x9FBFE4A5L, 0xE8B8D433L, 0x7807C9A2L, 0x0F00F934L, 0x9609A88EL,
	0xE10E9818L, 0x7F6A0DBBL, 0x086D3D2DL, 0x91646C97L, 0xE6635C01L,
	0x6B6B51F4L, 0x1C6C6162L, 0x856530D8L, 0xF262004EL, 0x6C0695EDL,
	0x1B01A57BL, 0x8208F4C1L, 0xF50FC457L, 0x65B0D9C6L, 0x12B7E950L,
	0x8BBEB8EAL, 0xFCB9887CL, 0x62DD1DDFL, 0x15DA2D49L, 0x8CD37CF3L,
	0xFBD44C65L, 0x4DB26158L, 0x3AB551CEL, 0xA3BC0074L, 0xD4BB30E2L,
	0x4ADFA541L, 0x3DD895D7L, 0xA4D1C46DL, 0xD3D6F4FBL, 0x4369E96AL,
	0x346ED9FCL, 0xAD678846L, 0xDA60B8D0L, 0x44042D73L, 0x33031DE5L,
	0xAA0A4C5FL, 0xDD0D7CC9L, 0x5005713CL, 0x270241AAL, 0xBE0B1010L,
	0xC90C2086L, 0x5768B525L, 0x206F85B3L, 0xB966D409L, 0xCE61E49FL,
	0x5EDEF90EL, 0x29D9C998L, 0xB0D09822L, 0xC7D7A8B4L, 0x59B33D17L,
	0x2EB40D81L, 0xB7BD5C3BL, 0xC0BA6CADL, 0xEDB88320L, 0x9ABFB3B6L,
	0x03B6E20CL, 0x74B1D29AL, 0xEAD54739L, 0x9DD277AFL, 0x04DB2615L,
	0x73DC1683L, 0xE3630B12L, 0x94643B84L, 0x0D6D6A3EL, 0x7A6A5AA8L,
	0xE40ECF0BL, 0x9309FF9DL, 0x0A00AE27L, 0x7D079EB1L, 0xF00F9344L,
	0x8708A3D2L, 0x1E01F268L, 0x6906C2FEL, 0xF762575DL, 0x806567CBL,
	0x196C3671L, 0x6E6B06E7L, 0xFED41B76L, 0x89D32BE0L, 0x10DA7A5AL,
	0x67DD4ACCL, 0xF9B9DF6FL, 0x8EBEEFF9L, 0x17B7BE43L, 0x60B08ED5L,
	0xD6D6A3E8L, 0xA1D1937EL, 0x38D8C2C4L, 0x4FDFF252L, 0xD1BB67F1L,
	0xA6BC5767L, 0x3FB506DDL, 0x48B2364BL, 0xD80D2BDAL, 0xAF0A1B4CL,
	0x36034AF6L, 0x41047A60L, 0xDF60EFC3L, 0xA867DF55L, 0x316E8EEFL,
	0x4669BE79L, 0xCB61B38CL, 0xBC66831AL, 0x256FD2A0L, 0x5268E236L,
	0xCC0C7795L, 0xBB0B4703L, 0x220216B9L, 0x5505262FL, 0xC5BA3BBEL,
	0xB2BD0B28L, 0x2BB45A92L, 0x5CB36A04L, 0xC2D7FFA7L, 0xB5D0CF31L,
	0x2CD99E8BL, 0x5BDEAE1DL, 0x9B64C2B0L, 0xEC63F226L, 0x756AA39CL,
	0x026D930AL, 0x9C0906A9L, 0xEB0E363FL, 0x72076785L, 0x05005713L,
	0x95BF4A82L, 0xE2B87A14L, 0x7BB12BAEL, 0x0CB61B38L, 0x92D28E9BL,
	0xE5D5BE0DL, 0x7CDCEFB7L, 0x0BDBDF21L, 0x86D3D2D4L, 0xF1D4E242L,
	0x68DDB3F8L, 0x1FDA836EL, 0x81BE16CDL, 0xF6B9265BL, 0x6FB077E1L,
	0x18B74777L, 0x88085AE6L, 0xFF0F6A70L, 0x66063BCAL, 0x11010B5CL,
	0x8F659EFFL, 0xF862AE69L, 0x616BFFD3L, 0x166CCF45L, 0xA00AE278L,
	0xD70DD2EEL, 0x4E048354L, 0x3903B3C2L, 0xA7672661L, 0xD06016F7L,
	0x4969474DL, 0x3E6E77DBL, 0xAED16A4AL, 0xD9D65ADCL, 0x40DF0B66L,
	0x37D83BF0L, 0xA9BCAE53L, 0xDEBB9EC5L, 0x47B2CF7FL, 0x30B5FFE9L,
	0xBDBDF21CL, 0xCABAC28AL, 0x53B39330L, 0x24B4A3A6L, 0xBAD03605L,
	0xCDD70693L, 0x54DE5729L, 0x23D967BFL, 0xB3667A2EL, 0xC4614AB8L,
	0x5D681B02L, 0x2A6F2B94L, 0xB40BBE37L, 0xC30C8EA1L, 0x5A05DF1BL,
	0x2D02EF8DL
};
#endif

enum smc_error_flag {
	CP_NO_ERROR = 0,
	CP_NOT_ALIGN_64KB,
	CP_MEM_TOO_BIG,
	CP_FLAG_OUT_RANGE,
	CP_WRONG_TZASC_REGION_NUM,
	CP_WRONG_BL_SIZE = 5,
	CP_MEM_OUT_OF_RANGE,
	CP_NOT_ALIGN_16B,
	CP_MEM_IN_SECURE_DRAM,
	CP_ASP_ENABLE_FAIL,
	CP_ASP_DISABLE_FAIL = 10,
	CP_NOT_WORKING,
	CP_ALREADY_WORKING,
	CP_ALREADY_DUMP_MODE,
	CP_NOT_VALID_MAGIC,
	CP_SHOULD_BE_DISABLE = 15,
	CP_ALREADY_ENABLE_CPMEM_ON,
	CP_ALREADY_SET_WND,
	CP_FAIL_TO_SET_WND,
	CP_INVALID_CP_BASE,
	CP_CORRUPTED_CP_MEM_INFO = 20,
	CP_WHILE_CHECKING_SIGN,
	CP_NOT_WHILE_CHECKING_SIGN,
	CP_IS_IN_INVALID_STATE,
	CP_IS_IN_INVALID_STATE2,
	CP_ERR_WHILE_CP_SIGN_CHECK,

	CP_CHECK_SIGN_NOT_FINISH = 0x100
};

static inline void start_tx_timer(struct mem_link_device *mld,
				  struct hrtimer *timer);

#if IS_ENABLED(CONFIG_CP_SECURE_BOOT)
static char *smc_err_string[32] = {
	"CP_NO_ERROR",
	"CP_NOT_ALIGN_64KB",
	"CP_MEM_TOO_BIG",
	"CP_FLAG_OUT_RANGE",
	"CP_WRONG_TZASC_REGION_NUM",
	"CP_WRONG_BL_SIZE",
	"CP_MEM_OUT_OF_RANGE",
	"CP_NOT_ALIGN_16B",
	"CP_MEM_IN_SECURE_DRAM",
	"CP_ASP_ENABLE_FAIL",
	"CP_ASP_DISABLE_FAIL",
	"CP_NOT_WORKING",
	"CP_ALREADY_WORKING",
	"CP_ALREADY_DUMP_MODE",
	"CP_NOT_VALID_MAGIC",
	"CP_SHOULD_BE_DISABLE",
	"CP_ALREADY_ENABLE_CPMEM_ON",
	"CP_ALREADY_SET_WND",
	"CP_FAIL_TO_SET_WND",
	"CP_INVALID_CP_BASE",
	"CP_CORRUPTED_CP_MEM_INFO",
	"CP_WHILE_CHECKING_SIGN",
	"CP_NOT_WHILE_CHECKING_SIGN",
	"CP_IS_IN_INVALID_STATE",
	"CP_IS_IN_INVALID_STATE2",
	"CP_ERR_WHILE_CP_SIGN_CHECK",
};
#endif

/*============================================================================*/
static inline bool ipc_active(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;

	if (unlikely(!cp_online(mc))) {
		mif_err("%s<->%s: %s.state %s != ONLINE <%ps>\n",
			ld->name, mc->name, mc->name, mc_state(mc), CALLER);
		return false;
	}

	if (mld->dpram_magic) {
		unsigned int magic = ioread32(mld->legacy_link_dev.magic);
		unsigned int mem_access = ioread32(mld->legacy_link_dev.mem_access);

		if (magic != ld->magic_ipc || mem_access != 1) {
			mif_err("%s<->%s: ERR! magic:0x%X access:%d <%ps>\n",
				ld->name, mc->name, magic, mem_access, CALLER);
			return false;
		}
	}

	if (atomic_read(&mld->forced_cp_crash)) {
		mif_err("%s<->%s: ERR! forced_cp_crash:%d <%ps>\n",
			ld->name, mc->name, atomic_read(&mld->forced_cp_crash),
			CALLER);
		return false;
	}

	return true;
}

static inline void purge_txq(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	int i;

	/* Purge the skb_txq in every rb */
	if (ld->sbd_ipc) {
		struct sbd_link_device *sl = &mld->sbd_link_dev;

		for (i = 0; i < sl->num_channels; i++) {
			struct sbd_ring_buffer *rb = sbd_id2rb(sl, i, TX);

			skb_queue_purge(&rb->skb_q);
		}
	}

	/* Purge the skb_txq in every IPC device
	 * (IPC_MAP_FMT, IPC_MAP_NORM_RAW, etc.)
	 */
	for (i = 0; i < MAX_SIPC_MAP; i++) {
		struct legacy_ipc_device *dev = mld->legacy_link_dev.dev[i];

		skb_queue_purge(dev->skb_txq);
	}
}

/*============================================================================*/
static void shmem_handle_cp_crash(struct mem_link_device *mld,
		enum modem_state state)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;

	/* Disable normal IPC */
	iowrite32(ld->magic_crash, mld->legacy_link_dev.magic);
	iowrite32(0, mld->legacy_link_dev.mem_access);

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
	tpmon_stop();
#endif

	stop_net_ifaces(ld);
	purge_txq(mld);

	if (cp_online(mc)) {
		switch (state) {
		case STATE_CRASH_RESET:
			modem_notify_event(MODEM_EVENT_RESET, mc);
			break;
		case STATE_CRASH_EXIT:
			modem_notify_event(MODEM_EVENT_EXIT, mc);
			break;
		case STATE_CRASH_WATCHDOG:
			modem_notify_event(MODEM_EVENT_WATCHDOG, mc);
			break;
		default:
			mif_err("Invalid state to notify\n");
			break;
		}
	}

	if (cp_online(mc) || cp_booting(mc))
		change_modem_state(mc, state);

	atomic_set(&mld->forced_cp_crash, 0);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
static void handle_no_cp_crash_ack(struct timer_list *t)
#else
static void handle_no_cp_crash_ack(unsigned long arg)
#endif
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
	struct mem_link_device *mld = from_timer(mld, t, crash_ack_timer);
#else
	struct mem_link_device *mld = (struct mem_link_device *)arg;
#endif
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	if (ld->link_type == LINKDEV_PCIE) {
		if (mif_gpio_get_value(&mc->s5100_gpio_phone_active, true) == 0) {
			mif_info("Set s5100_cp_reset_required to FALSE\n");
			mc->s5100_cp_reset_required = false;
		} else {
			mif_info("Set s5100_cp_reset_required to TRUE\n");
			mc->s5100_cp_reset_required = true;
		}
	}
#endif

	if (cp_crashed(mc))
		mif_debug("%s: STATE_CRASH_EXIT without CRASH_ACK\n", ld->name);
	else {
		mif_err("%s: ERR! No CRASH_ACK from CP\n", ld->name);
		shmem_handle_cp_crash(mld, STATE_CRASH_EXIT);
	}
}

static void link_trigger_cp_crash(struct mem_link_device *mld, u32 crash_type,
		char *crash_reason_string)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	char string[CP_CRASH_INFO_SIZE];

	if (!cp_online(mc) && !cp_booting(mc)) {
		mif_err("%s: %s.state %s != ONLINE <%ps>\n",
				ld->name, mc->name, mc_state(mc), CALLER);
		return;
	}

	if (atomic_inc_return(&mld->forced_cp_crash) > 1) {
		mif_err("%s: ALREADY in progress <%ps>\n", ld->name, CALLER);
		return;
	}

	/* Disable normal IPC */
	iowrite32(ld->magic_crash, mld->legacy_link_dev.magic);
	iowrite32(0, mld->legacy_link_dev.mem_access);

	mif_stop_logging();

	memset(string, 0, CP_CRASH_INFO_SIZE);
	if (crash_reason_string)
		strcpy(string, crash_reason_string);
	memset(ld->crash_reason.string, 0, CP_CRASH_INFO_SIZE);

	if (ld->crash_reason.type != crash_type)
		ld->crash_reason.type = crash_type;

	switch (ld->protocol) {
	case PROTOCOL_SIPC:
		switch (crash_type) {
		case CRASH_REASON_USER:
		case CRASH_REASON_MIF_TX_ERR:
		case CRASH_REASON_MIF_RIL_BAD_CH:
		case CRASH_REASON_MIF_RX_BAD_DATA:
		case CRASH_REASON_MIF_FORCED:
		case CRASH_REASON_CLD:
			if (strlen(string))
				strlcat(ld->crash_reason.string, string,
						CP_CRASH_INFO_SIZE);
			break;

		default:
			break;
		}
		break;

	case PROTOCOL_SIT:
		switch (crash_type) {
		case CRASH_REASON_MIF_TX_ERR:
		case CRASH_REASON_MIF_RIL_BAD_CH:
		case CRASH_REASON_MIF_RX_BAD_DATA:
		case CRASH_REASON_MIF_FORCED:
		case CRASH_REASON_RIL_TRIGGER_CP_CRASH:
			if (strlen(string))
				strlcat(ld->crash_reason.string, string,
						CP_CRASH_INFO_SIZE);
			break;

		default:
			break;
		}
		break;

	default:
		mif_err("ERR - unknown protocol\n");
		break;
	}
	mif_info("CP Crash type:%d string:%s\n", crash_type,
			ld->crash_reason.string);

	stop_net_ifaces(ld);

	if (mld->debug_info)
		mld->debug_info();

	/**
	 * If there is no CRASH_ACK from CP in a timeout,
	 * handle_no_cp_crash_ack() will be executed.
	 */
	mif_add_timer(&mld->crash_ack_timer, FORCE_CRASH_ACK_TIMEOUT,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
		      handle_no_cp_crash_ack);
#else
		      handle_no_cp_crash_ack, (unsigned long)mld);
#endif

	update_ctrl_msg(&mld->ap2cp_united_status, ld->crash_reason.type,
			mc->sbi_crash_type_mask, mc->sbi_crash_type_pos);

#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM)
	if (ld->interrupt_types == INTERRUPT_MAILBOX) {
		/* Send CRASH_EXIT command to a CP */
		if (mld->ap2cp_msg.type == MAILBOX_SR)
			cp_mbox_dump_sr();

		send_ipc_irq(mld, cmd2int(CMD_CRASH_EXIT));

		if (mld->ap2cp_msg.type == MAILBOX_SR)
			cp_mbox_dump_sr();
	}
#endif

#if IS_ENABLED(CONFIG_SEC_MODEM_S5100)
	if (ld->interrupt_types == INTERRUPT_GPIO)
		/* Raise DUMP_NOTI GPIO to CP */
		s5100_force_crash_exit_ext();
#endif

	mif_err("%s->%s: CP_CRASH_REQ by %d, %s <%ps>\n",
				ld->name, mc->name, ld->crash_reason.type,
				ld->crash_reason.string, CALLER);
}

static bool rild_ready(struct link_device *ld)
{
	struct io_device *fmt_iod;
	struct io_device *rfs_iod;
	int fmt_opened;
	int rfs_opened;

	switch (ld->protocol) {
	case PROTOCOL_SIT:
		return true;
	default:
		fmt_iod = link_get_iod_with_channel(ld, ld->chid_fmt_0);
		if (!fmt_iod) {
			mif_err("%s: No FMT io_device\n", ld->name);
			return false;
		}

		rfs_iod = link_get_iod_with_channel(ld, ld->chid_rfs_0);
		if (!rfs_iod) {
			mif_err("%s: No RFS io_device\n", ld->name);
			return false;
		}

		fmt_opened = atomic_read(&fmt_iod->opened);
		rfs_opened = atomic_read(&rfs_iod->opened);
		mif_err_limited("%s: %s.opened=%d, %s.opened=%d\n", ld->name,
			fmt_iod->name, fmt_opened, rfs_iod->name, rfs_opened);
		if (fmt_opened > 0 && rfs_opened > 0)
			return true;

		return false;
	}
}

static void write_clk_table_to_shmem(struct mem_link_device *mld)
{
	struct clock_table *clk_tb;
	u32 *clk_data;
	int i, j;

	if (mld->clk_table == NULL) {
		mif_err("clk_table is not defined\n");
		return;
	}

	clk_tb = (struct clock_table *)mld->clk_table;

	strcpy(clk_tb->parser_version, "CT0");
	clk_tb->total_table_count = mld->total_freq_table_count;

	strcpy(clk_tb->table_info[0].table_name, "MIF");
	clk_tb->table_info[0].table_count = mld->mif_table.num_of_table;

	strcpy(clk_tb->table_info[1].table_name, "CP");
	clk_tb->table_info[1].table_count = mld->cp_table.num_of_table;

	strcpy(clk_tb->table_info[2].table_name, "MDM");
	clk_tb->table_info[2].table_count = mld->modem_table.num_of_table;

	clk_data = (u32 *)&(clk_tb->table_info[clk_tb->total_table_count]);

	/* MIF */
	for (i = 0; i < mld->mif_table.num_of_table; i++) {
		*clk_data = mld->mif_table.freq[i];
		clk_data++;
	}

	/* CP */
	for (i = 0; i < mld->cp_table.num_of_table; i++) {
		*clk_data = mld->cp_table.freq[i];
		clk_data++;
	}

	/* MODEM */
	for (i = 0; i < mld->modem_table.num_of_table; i++) {
		*clk_data = mld->modem_table.freq[i];
		clk_data++;
	}

	mif_info("PARSER_VERSION: %s\n", clk_tb->parser_version);
	mif_info("TOTAL_TABLE_COUNT: %d\n", clk_tb->total_table_count);

	for (i = 0; i < clk_tb->total_table_count; i++) {
		mif_info("TABLE_NAME[%d] : %s\n", i+1,
			clk_tb->table_info[i].table_name);
		mif_info("TABLE_COUNT[%d]: %d\n", i+1,
			clk_tb->table_info[i].table_count);
	}

	clk_data = (u32 *)&(clk_tb->table_info[clk_tb->total_table_count]);

	for (i = 0; i < clk_tb->total_table_count; i++) {
		for (j = 0; j < clk_tb->table_info[i].table_count; j++) {
			mif_info("CLOCK_TABLE[%d][%d] : %d\n",
				i+1, j+1, *clk_data);
			clk_data++;
		}
	}
}

static void write_ap_capabilities(struct mem_link_device *mld)
{
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	mld->ap_capability_0 = mld->ap_capability_0 | AP_CAP_PKTPROC_UL;
#endif
#if IS_ENABLED(CONFIG_CH_EXTENSION)
	mld->ap_capability_0 = mld->ap_capability_0 | AP_CAP_CH_EXTENSION;
#endif
	iowrite32(mld->ap_capability_0, mld->ap_capability_0_offset);
	iowrite32(mld->ap_capability_1, mld->ap_capability_1_offset);

	mif_info("ap_capability_0:0x%08x ap_capability_1:0x%08x\n",
			mld->ap_capability_0, mld->ap_capability_1);
}

static void init_enabled_capabilities(struct mem_link_device *mld)
{
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	int err;

	if (mld->ap_capability_0 & AP_CAP_PKTPROC_UL) {
		err = pktproc_init_ul(&mld->pktproc_ul);
		if (err < 0) {
			mif_err("pktproc_init_ul() error %d\n", err);
			return;
		}
	}
#endif
}

static void cmd_init_start_handler(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	int err;

	mif_err("%s: INIT_START <- %s (%s.state:%s cp_boot_done:%d)\n",
		ld->name, mc->name, mc->name, mc_state(mc),
		atomic_read(&mld->cp_boot_done));

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	err = pktproc_init(&mld->pktproc);
	if (err < 0) {
		mif_err("pktproc_init() error %d\n", err);
		return;
	}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
	err = dit_init(ld, DIT_INIT_NORMAL);
	if ((err < 0) && (err != -EPERM)) {
		mif_err("dit_init() error %d\n", err);
		return;
	}
#endif

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
	tpmon_init();
#endif

	if (ld->capability_check)
		write_ap_capabilities(mld);

	if (!ld->sbd_ipc) {
		mif_err("%s: LINK_ATTR_SBD_IPC is NOT set\n", ld->name);
		goto init_exit;
	}

	err = init_sbd_link(&mld->sbd_link_dev);
	if (err < 0) {
		mif_err("%s: init_sbd_link fail(%d)\n", ld->name, err);
		return;
	}

	if (mld->attrs & LINK_ATTR_IPC_ALIGNED)
		ld->aligned = true;
	else
		ld->aligned = false;

	sbd_activate(&mld->sbd_link_dev);

init_exit:
	send_ipc_irq(mld, cmd2int(CMD_PIF_INIT_DONE));

	mif_err("%s: PIF_INIT_DONE -> %s\n", ld->name, mc->name);
}

static void cmd_phone_start_handler(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	unsigned long flags;
	int err;
	static int phone_start_count;

	mif_info_limited("%s: PHONE_START <- %s (%s.state:%s cp_boot_done:%d)\n",
		ld->name, mc->name, mc->name, mc_state(mc),
		atomic_read(&mld->cp_boot_done));

	if (mld->state == LINK_STATE_OFFLINE)
		phone_start_count = 0;

	if (atomic_read(&mld->cp_boot_done)) {
		mif_err_limited("Abnormal PHONE_START from CP\n");

		if (phone_start_count < 100) {
			if (phone_start_count++ > 3) {
				phone_start_count = 101;
#if IS_ENABLED(CONFIG_MCU_IPC)
				if (mld->ap2cp_msg.type == MAILBOX_SR)
					cp_mbox_dump_sr();
#endif
				send_ipc_irq(mld,
					cmd2int(phone_start_count - 100));
#if IS_ENABLED(CONFIG_MCU_IPC)
				if (mld->ap2cp_msg.type == MAILBOX_SR)
					cp_mbox_dump_sr();
#endif
				return;
			}
		} else {
			if (phone_start_count++ < 105) {
				mif_err("%s: CMD(0x%x) -> %s\n", ld->name,
					cmd2int(phone_start_count - 100),
					mc->name);
#if IS_ENABLED(CONFIG_MCU_IPC)
				if (mld->ap2cp_msg.type == MAILBOX_SR)
					cp_mbox_dump_sr();
#endif
				send_ipc_irq(mld, cmd2int(phone_start_count - 100));
#if IS_ENABLED(CONFIG_MCU_IPC)
				if (mld->ap2cp_msg.type == MAILBOX_SR)
					cp_mbox_dump_sr();
#endif
			} else {
				link_trigger_cp_crash(mld,
					CRASH_REASON_CP_RSV_0,
					"Abnormal CP_START from CP");
			}
			return;
		}
	}

	spin_lock_irqsave(&mld->state_lock, flags);

	if (mld->state == LINK_STATE_IPC) {
		/*
		 * If there is no INIT_END command from AP, CP sends a CP_START
		 * command to AP periodically until it receives INIT_END from AP
		 * even though it has already been in ONLINE state.
		 */
		if (rild_ready(ld)) {
			mif_info("%s: INIT_END -> %s\n", ld->name, mc->name);
#if IS_ENABLED(CONFIG_MCU_IPC)
			if (mld->ap2cp_msg.type == MAILBOX_SR)
				cp_mbox_dump_sr();
#endif
			send_ipc_irq(mld, cmd2int(CMD_INIT_END));
#if IS_ENABLED(CONFIG_MCU_IPC)
			if (mld->ap2cp_msg.type == MAILBOX_SR)
				cp_mbox_dump_sr();
#endif
			atomic_set(&mld->cp_boot_done, 1);
		}
		goto exit;
	}

	if (ld->capability_check) {
		/* get cp_capability */
		mld->cp_capability_0 = ioread32(mld->cp_capability_0_offset);
		mld->cp_capability_1 = ioread32(mld->cp_capability_1_offset);

		if ((mld->ap_capability_0 ^ mld->cp_capability_0) & mld->ap_capability_0) {
			/* if at least one feature is owned by AP only, crash CP */
			mif_err("AP capability 0: 0x%08x CP capability 0: 0x%08x\n",
					mld->ap_capability_0, mld->cp_capability_0);
			goto capability_fail;
		}
		if ((mld->ap_capability_1 ^ mld->cp_capability_1) & mld->ap_capability_1) {
			/* if at least one feature is owned by AP only, crash CP */
			mif_err("AP capability 1: 0x%08x CP capability 1: 0x%08x\n",
					mld->ap_capability_1, mld->cp_capability_1);
			goto capability_fail;
		}

		mif_info("ap_capability_0:0x%08x ap_capability_1:0x%08x\n",
				mld->ap_capability_0, mld->ap_capability_1);
		mif_info("cp_capability_0:0x%08x cp_capability_1:0x%08x\n",
				mld->cp_capability_0, mld->cp_capability_1);

		init_enabled_capabilities(mld);
	}

	err = init_legacy_link(&mld->legacy_link_dev);
	if (err) {
		mif_err("%s: init_legacy_link fail(%d)\n", ld->name, err);
		goto exit;
	}
	atomic_set(&ld->netif_stopped, 0);

	if (rild_ready(ld)) {
		mif_info("%s: INIT_END -> %s\n", ld->name, mc->name);
#if IS_ENABLED(CONFIG_MCU_IPC)
		if (mld->ap2cp_msg.type == MAILBOX_SR)
			cp_mbox_dump_sr();
#endif
		send_ipc_irq(mld, cmd2int(CMD_INIT_END));
#if IS_ENABLED(CONFIG_MCU_IPC)
		if (mld->ap2cp_msg.type == MAILBOX_SR)
			cp_mbox_dump_sr();
#endif
		atomic_set(&mld->cp_boot_done, 1);
	}

#if IS_ENABLED(CONFIG_MCU_IPC)
	if (mld->ap2cp_msg.type == MAILBOX_SR)
		cp_mbox_dump_sr();
#endif

	ld->crash_reason.type = CRASH_REASON_NONE;
	memset(ld->crash_reason.string, 0, CP_CRASH_INFO_SIZE);
	mif_info("Set crash_reason type:%d\n", ld->crash_reason.type);

	if ((ld->protocol == PROTOCOL_SIT) && (ld->link_type == LINKDEV_SHMEM))
		write_clk_table_to_shmem(mld);

	mld->state = LINK_STATE_IPC;
	complete_all(&mc->init_cmpl);
	modem_notify_event(MODEM_EVENT_ONLINE, mc);

exit:
	if (ld->sbd_ipc)
		start_tx_timer(mld, &mld->sbd_print_timer);
	spin_unlock_irqrestore(&mld->state_lock, flags);
	return;

capability_fail:
	spin_unlock_irqrestore(&mld->state_lock, flags);
	link_trigger_cp_crash(mld, CRASH_REASON_MIF_FORCED,
			"CP lacks capability\n");
	return;
}

static void cmd_crash_reset_handler(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	unsigned long flags;

	spin_lock_irqsave(&mld->state_lock, flags);
	mld->state = LINK_STATE_OFFLINE;
	if (ld->crash_reason.type == CRASH_REASON_NONE)
		ld->crash_reason.type = CRASH_REASON_CP_ACT_CRASH;
	spin_unlock_irqrestore(&mld->state_lock, flags);

	mif_err("%s<-%s: ERR! CP_CRASH_RESET\n", ld->name, mc->name);

	shmem_handle_cp_crash(mld, STATE_CRASH_RESET);
}

static void cmd_crash_exit_handler(struct mem_link_device *mld)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	unsigned long flags;

	mif_stop_logging();

	spin_lock_irqsave(&mld->state_lock, flags);
	mld->state = LINK_STATE_CP_CRASH;
	if (ld->crash_reason.type == CRASH_REASON_NONE)
		ld->crash_reason.type = CRASH_REASON_CP_ACT_CRASH;
	spin_unlock_irqrestore(&mld->state_lock, flags);

	if (timer_pending(&mld->crash_ack_timer))
		del_timer(&mld->crash_ack_timer);

	if (atomic_read(&mld->forced_cp_crash))
		mif_err("%s<-%s: CP_CRASH_ACK\n", ld->name, mc->name);
	else
		mif_err("%s<-%s: ERR! CP_CRASH_EXIT\n", ld->name, mc->name);

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	if (ld->link_type == LINKDEV_PCIE) {
		if (mif_gpio_get_value(&mc->s5100_gpio_phone_active, true) == 0) {
			mif_info("Set s5100_cp_reset_required to FALSE\n");
			mc->s5100_cp_reset_required = false;
		} else {
			mif_info("Set s5100_cp_reset_required to TRUE\n");
			mc->s5100_cp_reset_required = true;
		}
	}
#endif

	shmem_handle_cp_crash(mld, STATE_CRASH_EXIT);
}

static void shmem_cmd_handler(struct mem_link_device *mld, u16 cmd)
{
	struct link_device *ld = &mld->link_dev;

	switch (cmd) {
	case CMD_INIT_START:
		cmd_init_start_handler(mld);
		break;

	case CMD_PHONE_START:
		cmd_phone_start_handler(mld);
		break;

	case CMD_CRASH_RESET:
		cmd_crash_reset_handler(mld);
		break;

	case CMD_CRASH_EXIT:
		cmd_crash_exit_handler(mld);
		break;

	default:
		mif_err("%s: Unknown command 0x%04X\n", ld->name, cmd);
		break;
	}
}

/*============================================================================*/
static inline int check_txq_space(struct mem_link_device *mld,
				  struct legacy_ipc_device *dev,
				  unsigned int qsize, unsigned int in,
				  unsigned int out, unsigned int count)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	unsigned int space;

	if (!circ_valid(qsize, in, out)) {
		mif_err_limited("%s: ERR! Invalid %s_TXQ{qsize:%d in:%d out:%d}\n",
			ld->name, dev->name, qsize, in, out);
		return -EIO;
	}

	space = circ_get_space(qsize, in, out);
	if (unlikely(space < count)) {
		if (cp_online(mc)) {
			mif_err_limited("%s: NOSPC %s_TX{qsize:%d in:%d out:%d space:%d len:%d}\n",
				ld->name, dev->name, qsize,
				in, out, space, count);
		}
		return -ENOSPC;
	}

	return space;
}

static int txq_write(struct mem_link_device *mld, struct legacy_ipc_device *dev,
		     struct sk_buff *skb)
{
	char *src = skb->data;
	unsigned int count = skb->len;
	char *dst = get_txq_buff(dev);
	unsigned int qsize = get_txq_buff_size(dev);
	unsigned int in = get_txq_head(dev);
	unsigned int out = get_txq_tail(dev);
	int space;

	space = check_txq_space(mld, dev, qsize, in, out, count);
	if (unlikely(space < 0))
		return space;

	barrier();

	circ_write(dst, src, qsize, in, count);

	barrier();

	set_txq_head(dev, circ_new_ptr(qsize, in, count));

	/* Commit the item before incrementing the head */
	smp_mb();

	return count;
}

static int tx_frames_to_dev(struct mem_link_device *mld,
			    struct legacy_ipc_device *dev)
{
	struct sk_buff_head *skb_txq = dev->skb_txq;
	int tx_bytes = 0;
	int ret = 0;

	while (1) {
		struct sk_buff *skb;

		skb = skb_dequeue(skb_txq);
		if (unlikely(!skb))
			break;

		ret = txq_write(mld, dev, skb);
		if (unlikely(ret < 0)) {
			/* Take the skb back to the skb_txq */
			skb_queue_head(skb_txq, skb);
			break;
		}

		tx_bytes += ret;

#ifdef DEBUG_MODEM_IF_LINK_TX
		mif_pkt(skbpriv(skb)->sipc_ch, "LNK-TX", skb);
#endif

		dev_consume_skb_any(skb);
	}

	return (ret < 0) ? ret : tx_bytes;
}

static enum hrtimer_restart tx_timer_func(struct hrtimer *timer)
{
	struct mem_link_device *mld;
	struct link_device *ld;
	struct modem_ctl *mc;
	int i;
	bool need_schedule;
	u16 mask;
	unsigned long flags;

	mld = container_of(timer, struct mem_link_device, tx_timer);
	ld = &mld->link_dev;
	mc = ld->mc;

	need_schedule = false;
	mask = 0;

	spin_lock_irqsave(&mc->lock, flags);
	if (unlikely(!ipc_active(mld)))
		goto exit;

	for (i = 0; i < MAX_SIPC_MAP; i++) {
		struct legacy_ipc_device *dev = mld->legacy_link_dev.dev[i];
		int ret;

		if (unlikely(under_tx_flow_ctrl(mld, dev))) {
			ret = check_tx_flow_ctrl(mld, dev);
			if (ret < 0) {
				if (ret == -EBUSY || ret == -ETIME) {
					need_schedule = true;
					continue;
				} else {
					link_trigger_cp_crash(mld, CRASH_REASON_MIF_TX_ERR,
						"check_tx_flow_ctrl error");
					need_schedule = false;
					goto exit;
				}
			}
		}

		ret = tx_frames_to_dev(mld, dev);
		if (unlikely(ret < 0)) {
			if (ret == -EBUSY || ret == -ENOSPC) {
				need_schedule = true;
				txq_stop(mld, dev);
				/* If txq has 2 or more packet and 2nd packet
				 * has -ENOSPC return, It request irq to consume
				 * the TX ring-buffer from CP
				 */
				mask |= msg_mask(dev);
				continue;
			} else {
				link_trigger_cp_crash(mld, CRASH_REASON_MIF_TX_ERR,
					"tx_frames_to_dev error");
				need_schedule = false;
				goto exit;
			}
		}

		if (ret > 0)
			mask |= msg_mask(dev);

		if (!skb_queue_empty(dev->skb_txq))
			need_schedule = true;
	}

	if (mask)
		send_ipc_irq(mld, mask2int(mask));

exit:
	if (need_schedule) {
		ktime_t ktime = ktime_set(0, mld->tx_period_ms * NSEC_PER_MSEC);

		hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&mc->lock, flags);

	return HRTIMER_NORESTART;
}

static int tx_func(struct mem_link_device *mld, struct hrtimer *timer,
					  struct legacy_ipc_device *dev, struct sk_buff *skb)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	struct sk_buff_head *skb_txq = dev->skb_txq;
	bool need_schedule = false;
	u16 mask = msg_mask(dev);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&mc->lock, flags);
	if (unlikely(!ipc_active(mld))) {
		spin_unlock_irqrestore(&mc->lock, flags);
		dev_kfree_skb_any(skb);
		goto exit;
	}
	spin_unlock_irqrestore(&mc->lock, flags);

	ret = txq_write(mld, dev, skb);
	if (unlikely(ret < 0)) {
		if (ret == -EBUSY || ret == -ENOSPC) {
			skb_queue_head(skb_txq, skb);
			need_schedule = true;
			txq_stop(mld, dev);
			/* If txq has 2 or more packet and 2nd packet
			 * has -ENOSPC return, It request irq to consume
			 * the TX ring-buffer from CP
			 */
			send_ipc_irq(mld, mask2int(mask));
		} else {
			link_trigger_cp_crash(mld, CRASH_REASON_MIF_TX_ERR,
					"tx_frames_to_dev error");
			need_schedule = false;
		}
		goto exit;
	}

#ifdef DEBUG_MODEM_IF_LINK_TX
	mif_pkt(skbpriv(skb)->sipc_ch, "LNK-TX", skb);
#endif

	dev_consume_skb_any(skb);

	send_ipc_irq(mld, mask2int(mask));

exit:
	if (need_schedule) {
		ktime_t ktime = ktime_set(0, mld->tx_period_ms * NSEC_PER_MSEC);

		hrtimer_start(timer, ktime, HRTIMER_MODE_REL);

		return -1;
	} else
		return 1;
}

static inline void start_tx_timer(struct mem_link_device *mld,
				  struct hrtimer *timer)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	unsigned long flags;

	spin_lock_irqsave(&mc->lock, flags);
	if (unlikely(cp_offline(mc))) {
		spin_unlock_irqrestore(&mc->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&mc->lock, flags);

	spin_lock_irqsave(&mc->tx_timer_lock, flags);
	if (!hrtimer_is_queued(timer)) {
		ktime_t ktime = ktime_set(0, mld->tx_period_ms * NSEC_PER_MSEC);

		hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&mc->tx_timer_lock, flags);
}

static inline void shmem_start_timers(struct mem_link_device *mld)
{
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	start_tx_timer(mld, &mld->pktproc_tx_timer);
#endif
	if (sbd_active(&mld->sbd_link_dev))
		start_tx_timer(mld, &mld->sbd_tx_timer);
	start_tx_timer(mld, &mld->tx_timer);
}

static inline void cancel_tx_timer(struct mem_link_device *mld,
				   struct hrtimer *timer)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	unsigned long flags;

	spin_lock_irqsave(&mc->tx_timer_lock, flags);
	if (hrtimer_active(timer))
		hrtimer_cancel(timer);
	spin_unlock_irqrestore(&mc->tx_timer_lock, flags);
}

static inline void shmem_stop_timers(struct mem_link_device *mld)
{
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	cancel_tx_timer(mld, &mld->pktproc_tx_timer);
#endif
	if (sbd_active(&mld->sbd_link_dev))
		cancel_tx_timer(mld, &mld->sbd_tx_timer);
	cancel_tx_timer(mld, &mld->tx_timer);
}

static inline void start_datalloc_timer(struct mem_link_device *mld,
				  struct hrtimer *timer)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	unsigned long flags;

	spin_lock_irqsave(&mc->lock, flags);

	if (unlikely(cp_offline(mc)))
		goto exit;

	if (!hrtimer_is_queued(timer)) {
		ktime_t ktime = ktime_set(0, ms2ns(DATALLOC_PERIOD_MS));

		hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
	}

exit:
	spin_unlock_irqrestore(&mc->lock, flags);
}

static inline void __cancel_datalloc_timer(struct mem_link_device *mld,
				   struct hrtimer *timer)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	unsigned long flags;

	spin_lock_irqsave(&mc->lock, flags);

	if (hrtimer_active(timer))
		hrtimer_cancel(timer);

	spin_unlock_irqrestore(&mc->lock, flags);
}

static inline void cancel_datalloc_timer(struct mem_link_device *mld)
{
	struct sbd_link_device *sl = &mld->sbd_link_dev;
	struct sbd_ipc_device *ipc_dev = sl->ipc_dev;
	int i;

	for (i = 0; i < sl->num_channels; i++) {
		struct sbd_ring_buffer *rb = &ipc_dev[i].rb[DL];

		if (rb->zerocopy) {
			struct zerocopy_adaptor *zdptr = rb->zdptr;

			__cancel_datalloc_timer(mld, &zdptr->datalloc_timer);
		}
	}

}

static int tx_frames_to_rb(struct sbd_ring_buffer *rb)
{
	struct sk_buff_head *skb_txq = &rb->skb_q;
	int tx_bytes = 0;
	int ret = 0;

	while (1) {
		struct sk_buff *skb;

		skb = skb_dequeue(skb_txq);
		if (unlikely(!skb))
			break;

		ret = sbd_pio_tx(rb, skb);
		if (unlikely(ret < 0)) {
			/* Take the skb back to the skb_txq */
			skb_queue_head(skb_txq, skb);
			break;
		}

		tx_bytes += ret;
#ifdef DEBUG_MODEM_IF_LINK_TX
		mif_pkt(rb->ch, "LNK-TX", skb);
#endif
		dev_consume_skb_any(skb);
	}

	return (ret < 0) ? ret : tx_bytes;
}

static enum hrtimer_restart sbd_tx_timer_func(struct hrtimer *timer)
{
	struct mem_link_device *mld =
		container_of(timer, struct mem_link_device, sbd_tx_timer);
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	struct sbd_link_device *sl = &mld->sbd_link_dev;
	int i;
	bool need_schedule = false;
	u16 mask = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&mc->lock, flags);
	if (unlikely(!ipc_active(mld))) {
		spin_unlock_irqrestore(&mc->lock, flags);
		goto exit;
	}
	spin_unlock_irqrestore(&mc->lock, flags);

	for (i = 0; i < sl->num_channels; i++) {
		struct sbd_ring_buffer *rb = sbd_id2rb(sl, i, TX);
		int ret;

		if (unlikely(sbd_under_tx_flow_ctrl(rb))) {
			ret = sbd_check_tx_flow_ctrl(rb);
			if (ret < 0) {
				if (ret == -EBUSY || ret == -ETIME) {
					need_schedule = true;
					continue;
				} else {
					link_trigger_cp_crash(mld,
						CRASH_REASON_MIF_TX_ERR,
						"check_sbd_tx_flow_ctrl error");
					need_schedule = false;
					goto exit;
				}
			}
		}

		ret = tx_frames_to_rb(rb);
		if (unlikely(ret < 0)) {
			if (ret == -EBUSY || ret == -ENOSPC) {
				need_schedule = true;
				sbd_txq_stop(rb);
				mask = MASK_SEND_DATA;
				continue;
			} else {
				link_trigger_cp_crash(mld, CRASH_REASON_MIF_TX_ERR,
					"tx_frames_to_rb error");
				need_schedule = false;
				goto exit;
			}
		}

		if (ret > 0)
			mask = MASK_SEND_DATA;

		if (!skb_queue_empty(&rb->skb_q))
			need_schedule = true;
	}

	if (mask) {
		spin_lock_irqsave(&mc->lock, flags);
		if (unlikely(!ipc_active(mld))) {
			spin_unlock_irqrestore(&mc->lock, flags);
			need_schedule = false;
			goto exit;
		}
		send_ipc_irq(mld, mask2int(mask));
		spin_unlock_irqrestore(&mc->lock, flags);
	}

exit:
	if (need_schedule) {
		ktime_t ktime = ktime_set(0, mld->tx_period_ms * NSEC_PER_MSEC);

		hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

static int sbd_tx_func(struct mem_link_device *mld, struct hrtimer *timer,
		    struct sbd_ring_buffer *rb, struct sk_buff *skb)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	bool need_schedule = false;
	u16 mask = MASK_SEND_DATA;
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&mc->lock, flags);
	if (unlikely(!ipc_active(mld))) {
		spin_unlock_irqrestore(&mc->lock, flags);
		dev_kfree_skb_any(skb);
		goto exit;
	}
	spin_unlock_irqrestore(&mc->lock, flags);

	ret = sbd_pio_tx(rb, skb);
	if (unlikely(ret < 0)) {
		if (ret == -EBUSY || ret == -ENOSPC) {
			skb_queue_head(&rb->skb_q, skb);
			need_schedule = true;
			send_ipc_irq(mld, mask2int(mask));
		} else {
			link_trigger_cp_crash(mld, CRASH_REASON_MIF_TX_ERR,
					"tx_frames_to_rb error");
			need_schedule = false;
		}
		goto exit;
	}

#ifdef DEBUG_MODEM_IF_LINK_TX
	mif_pkt(rb->ch, "LNK-TX", skb);
#endif
	dev_consume_skb_any(skb);

	spin_lock_irqsave(&mc->lock, flags);
	if (unlikely(!ipc_active(mld))) {
		spin_unlock_irqrestore(&mc->lock, flags);
		need_schedule = false;
		goto exit;
	}
	send_ipc_irq(mld, mask2int(mask));
	spin_unlock_irqrestore(&mc->lock, flags);

exit:
	if (need_schedule) {
		ktime_t ktime = ktime_set(0, mld->tx_period_ms * NSEC_PER_MSEC);

		hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
		return -1;
	} else
		return 1;
}

#if IS_ENABLED(CONFIG_CP_ZEROCOPY)
enum hrtimer_restart datalloc_timer_func(struct hrtimer *timer)
{
	struct zerocopy_adaptor *zdptr =
		container_of(timer, struct zerocopy_adaptor, datalloc_timer);
	struct sbd_ring_buffer *rb = zdptr->rb;
	struct link_device *ld = rb->ld;
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	struct modem_ctl *mc = ld->mc;
	bool need_schedule = false;
	unsigned long flags;

	spin_lock_irqsave(&mc->lock, flags);
	if (unlikely(!ipc_active(mld))) {
		spin_unlock_irqrestore(&mc->lock, flags);
		goto exit;
	}
	spin_unlock_irqrestore(&mc->lock, flags);

	if (-ENOMEM == allocate_data_in_advance(zdptr))
		need_schedule = true;

	if (need_schedule) {
		ktime_t ktime = ktime_set(0, ms2ns(DATALLOC_PERIOD_MS));

		hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
	}
exit:

	return HRTIMER_NORESTART;
}
#endif

#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
static enum hrtimer_restart pktproc_tx_timer_func(struct hrtimer *timer)
{
	struct mem_link_device *mld = container_of(timer, struct mem_link_device, pktproc_tx_timer);
	struct pktproc_adaptor_ul *ppa_ul = &mld->pktproc_ul;
	struct modem_ctl *mc = mld->link_dev.mc;
	bool need_schedule = false;
	bool need_irq = false;
	bool need_dit = false;
	unsigned long flags;
	unsigned int count;
	int ret, i;

	spin_lock_irqsave(&mc->lock, flags);
	if (unlikely(!ipc_active(mld))) {
		spin_unlock_irqrestore(&mc->lock, flags);
		return HRTIMER_NORESTART;
	}
	spin_unlock_irqrestore(&mc->lock, flags);

	for (i = 0; i < ppa_ul->num_queue; i++) {
		struct pktproc_queue_ul *q = ppa_ul->q[i];

		if (unlikely(pktproc_under_ul_flow_ctrl(q))) {
			ret = pktproc_check_ul_flow_ctrl(q);
			if (ret < 0) {
				need_schedule = true;
				continue;
			}
		}

		do {
			if (dit_check_dir_use_queue(DIT_DIR_TX, q->q_idx)) {
				if (circ_empty(q->done_ptr, q->q_info->fore_ptr))
					break;

				need_dit = true;
			} else {
				count = circ_get_usage(q->q_info->num_desc,
					q->done_ptr, q->q_info->fore_ptr);
				if (count == 0)
					break;

				q->update_fore_ptr(q, count);
				need_irq = true;
			}
			need_schedule = true;
		} while (0);
	}

	/* irq will be raised after dit_kick() */
	if (need_dit)
		dit_kick(DIT_DIR_TX, false);
	else if (need_irq)
		send_ipc_irq(mld, mask2int(MASK_SEND_DATA));

	if (need_schedule) {
		ktime_t ktime = ktime_set(0, mld->tx_period_ms * NSEC_PER_MSEC);

		hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

static int xmit_ipc_to_pktproc(struct mem_link_device *mld, struct sk_buff *skb)
{
	struct pktproc_adaptor_ul *ppa_ul = &mld->pktproc_ul;
	struct pktproc_queue_ul *q;
	int len;
	int ret = -EBUSY;
	unsigned long flags;

	if (skb->queue_mapping == 1)
		q = ppa_ul->q[PKTPROC_HIPRIO_UL];
	else /* normal ul queue */
		q = ppa_ul->q[PKTPROC_NORM_UL];

	if (ppa_ul->padding_required)
		len = skb->len + CP_PADDING;
	else
		len = skb->len;

	if (len > ppa_ul->max_packet_size) {
		mif_err_limited("ERR! PKTPROC UL QUEUE %d\n"
				"skb len %d too large\n",
				q->q_idx, len);
		return -EINVAL;
	}

	if (spin_trylock_irqsave(&q->lock, flags)) {
		ret = q->send_packet(q, skb);
		spin_unlock_irqrestore(&q->lock, flags);
	}

	if (unlikely(ret < 0)) {
		if ((ret != -EBUSY) && (ret != -ENOSPC)) {
			link_trigger_cp_crash(mld, CRASH_REASON_MIF_TX_ERR,
				"tx_frames_to_pktproc error");
			return ret;
		}

		pktproc_ul_q_stop(q);
		goto exit;
	}

#ifdef DEBUG_MODEM_IF_LINK_TX
	mif_pkt(skbpriv(skb)->sipc_ch, "LNK-TX", skb);
#endif

	if (!dit_check_dir_use_queue(DIT_DIR_TX, q->q_idx))
		dev_consume_skb_any(skb);

exit:
	/* start timer even on error */
	if (ret)
		start_tx_timer(mld, &mld->pktproc_tx_timer);

	return ret;
}
#endif /* CONFIG_CP_PKTPROC_UL */

static int xmit_ipc_to_rb(struct mem_link_device *mld, u8 ch,
			  struct sk_buff *skb)
{
	int ret, ret2;
	struct link_device *ld = &mld->link_dev;
	struct io_device *iod = skbpriv(skb)->iod;
	struct modem_ctl *mc = ld->mc;
	struct sbd_ring_buffer *rb = sbd_ch2rb_with_skb(&mld->sbd_link_dev, ch, TX, skb);
	struct sk_buff_head *skb_txq;
	unsigned long flags = 0;
	int quota = MIF_TX_QUOTA;

	if (!rb) {
		mif_err("%s: %s->%s: ERR! NO SBD RB {ch:%d}\n",
			ld->name, iod->name, mc->name, ch);
		return -ENODEV;
	}

	skb_txq = &rb->skb_q;

	if (unlikely(skb_txq->qlen >= MAX_SKB_TXQ_DEPTH)) {
		mif_err_limited("%s: %s->%s: ERR! {ch:%d} skb_txq.len %d >= limit %d\n",
				ld->name, iod->name, mc->name, ch,
				skb_txq->qlen, MAX_SKB_TXQ_DEPTH);
		ret = -EBUSY;
	} else {
		skb->len = min_t(int, skb->len, rb->buff_size);
		ret = skb->len;

		skb_queue_tail(skb_txq, skb);

		if (hrtimer_active(&mld->sbd_tx_timer)) {
			start_tx_timer(mld, &mld->sbd_tx_timer);
		} else if (spin_trylock_irqsave(&rb->lock, flags)) {
			do {
				skb = skb_dequeue(skb_txq);
				if (!skb)
					break;

				ret2 = sbd_tx_func(mld, &mld->sbd_tx_timer, rb, skb);
				if (ret2 < 0)
					break;
			} while (--quota);

			spin_unlock_irqrestore(&rb->lock, flags);
		}
	}

	return ret;
}

bool check_mem_link_tx_pending(struct mem_link_device *mld)
{
	struct sbd_link_device *sl = &mld->sbd_link_dev;

	if (sbd_active(sl))
		return check_sbd_tx_pending(mld);
	else
		return check_legacy_tx_pending(mld);
}

static int xmit_ipc_to_dev(struct mem_link_device *mld, u8 ch, struct sk_buff *skb,
		enum legacy_ipc_map legacy_buffer_index)
{
	int ret, ret2;
	struct link_device *ld = &mld->link_dev;
	struct io_device *iod = skbpriv(skb)->iod;
	struct modem_ctl *mc = ld->mc;
	struct legacy_ipc_device *dev = mld->legacy_link_dev.dev[legacy_buffer_index];
	struct sk_buff_head *skb_txq;
	unsigned long flags = 0;
	int quota = MIF_TX_QUOTA;

	if (!dev) {
		mif_err("%s: %s->%s: ERR! NO IPC DEV {ch:%d}\n",
			ld->name, iod->name, mc->name, ch);
		return -ENODEV;
	}

	skb_txq = dev->skb_txq;

	if (unlikely(skb_txq->qlen >= MAX_SKB_TXQ_DEPTH)) {
		mif_err_limited("%s: %s->%s: ERR! %s TXQ.qlen %d >= limit %d\n",
				ld->name, iod->name, mc->name, dev->name,
				skb_txq->qlen, MAX_SKB_TXQ_DEPTH);
		ret = -EBUSY;
	} else {
		ret = skb->len;

		skb_queue_tail(skb_txq, skb);

		if (hrtimer_active(&mld->tx_timer)) {
			start_tx_timer(mld, &mld->tx_timer);
		} else if (spin_trylock_irqsave(&dev->tx_lock, flags)) {
			do {
				skb = skb_dequeue(skb_txq);
				if (!skb)
					break;

				ret2 = tx_func(mld, &mld->tx_timer, dev, skb);
				if (ret2 < 0)
					break;
			} while (--quota);

			spin_unlock_irqrestore(&dev->tx_lock, flags);
		}
	}

	return ret;
}

static int xmit_to_cp(struct mem_link_device *mld, struct io_device *iod,
		    u8 ch, struct sk_buff *skb)
{
	struct link_device *ld = &mld->link_dev;

	/* for boot/dump
	 * 1) assume that link (ex. PCI) is ready
	 * 2) do not need send_ipc_irq()
	 */
	if (ld->is_bootdump_ch(ch))
		return xmit_to_legacy_link(mld, ch, skb, IPC_MAP_NORM_RAW);

	if (unlikely(!ipc_active(mld)))
		return -EIO;

#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	if (ld->is_ps_ch(ch)) {
		return xmit_ipc_to_pktproc(mld, skb);
	} else if (ld->sbd_ipc && iod->sbd_ipc) {
#else
	if (ld->sbd_ipc && iod->sbd_ipc) {
#endif
		if (likely(sbd_active(&mld->sbd_link_dev)))
			return xmit_ipc_to_rb(mld, ch, skb);
		else
			return -ENODEV;
	} else {
		if (ld->is_fmt_ch(ch) || (ld->is_wfs0_ch != NULL && ld->is_wfs0_ch(ch)))
			return xmit_ipc_to_dev(mld, ch, skb, IPC_MAP_FMT);
#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
		return xmit_ipc_to_dev(mld, ch, skb,
			(skb->queue_mapping == 1) ? IPC_MAP_HPRIO_RAW : IPC_MAP_NORM_RAW);
#else
		return xmit_ipc_to_dev(mld, ch, skb, IPC_MAP_NORM_RAW);
#endif
	}
}

/*============================================================================*/
static void pass_skb_to_demux(struct mem_link_device *mld, struct sk_buff *skb)
{
	struct link_device *ld = &mld->link_dev;
	struct io_device *iod = skbpriv(skb)->iod;
	int ret;
	u8 ch = skbpriv(skb)->sipc_ch;

	if (unlikely(!iod)) {
		mif_err("%s: ERR! No IOD for CH.%d\n", ld->name, ch);
		dev_kfree_skb_any(skb);
		link_trigger_cp_crash(mld, CRASH_REASON_MIF_RIL_BAD_CH,
			"ERR! No IOD for CH.XX");
		return;
	}

#ifdef DEBUG_MODEM_IF_LINK_RX
	mif_pkt(ch, "LNK-RX", skb);
#endif

	ret = iod->recv_skb_single(iod, ld, skb);
	if (unlikely(ret < 0)) {
		struct modem_ctl *mc = ld->mc;

		mif_err_limited("%s: %s<-%s: ERR! %s->recv_skb fail (%d)\n",
				ld->name, iod->name, mc->name, iod->name, ret);
		dev_kfree_skb_any(skb);
	}
}

static int pass_skb_to_net(struct mem_link_device *mld, struct sk_buff *skb)
{
	struct link_device *ld = &mld->link_dev;
	struct skbuff_private *priv;
	struct io_device *iod;
	struct modem_ctl *mc = ld->mc;
	int ret = 0;

	if (unlikely(!cp_online(mc))) {
		mif_err_limited("ERR! CP not online!, skb:%pK\n", skb);
		dev_kfree_skb_any(skb);
		return -EACCES;
	}

	priv = skbpriv(skb);
	if (unlikely(!priv)) {
		mif_err("%s: ERR! No PRIV in skb@%pK\n", ld->name, skb);
		dev_kfree_skb_any(skb);
		link_trigger_cp_crash(mld, CRASH_REASON_MIF_RX_BAD_DATA,
			"ERR! No PRIV");
		return -EFAULT;
	}

	iod = priv->iod;
	if (unlikely(!iod)) {
		mif_err("%s: ERR! No IOD in skb@%pK\n", ld->name, skb);
		dev_kfree_skb_any(skb);
		link_trigger_cp_crash(mld, CRASH_REASON_MIF_RX_BAD_DATA,
			"ERR! No IOD");
		return -EIO;
	}

#ifdef DEBUG_MODEM_IF_LINK_RX
	mif_pkt(iod->ch, "LNK-RX", skb);
#endif

	ret = iod->recv_net_skb(iod, ld, skb);
	if (unlikely(ret < 0)) {
		struct modem_ctl *mc = ld->mc;

		mif_err_limited("%s: %s<-%s: ERR! %s->recv_net_skb fail (%d)\n",
				ld->name, iod->name, mc->name, iod->name, ret);
		dev_kfree_skb_any(skb);
	}

	return ret;
}

#define FREE_RB_BUF_COUNT 200
static int rx_net_frames_from_zerocopy_adaptor(struct sbd_ring_buffer *rb,
		int budget, int *work_done)
{
	int rcvd = 0;
	struct link_device *ld = rb->ld;
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	struct zerocopy_adaptor *zdptr = rb->zdptr;
	unsigned int num_frames;
	int use_memcpy = 0;
	int ret = 0;

	num_frames = min_t(unsigned int, rb_usage(rb), budget);

	ld->mif_buff_mng->enable_sw_zerocopy ?
		(mld->force_use_memcpy = 0) : (mld->force_use_memcpy = 1);

	if (mld->force_use_memcpy || (num_frames > ld->mif_buff_mng->free_cell_count)
		|| (circ_get_space(zdptr->len, *(zdptr->rp), *(zdptr->wp)) < FREE_RB_BUF_COUNT)) {
		use_memcpy = 1;
		mld->memcpy_packet_count++;
	} else {
		use_memcpy = 0;
		mld->zeromemcpy_packet_count++;
	}

	while (rcvd < num_frames) {
		struct sk_buff *skb;

		skb = sbd_pio_rx_zerocopy_adaptor(rb, use_memcpy);
		if (!skb)
			break;

		/* The $rcvd must be accumulated here, because $skb can be freed
		 * in pass_skb_to_net().
		 */
		rcvd++;

		ret = pass_skb_to_net(mld, skb);
		if (ret < 0)
			break;
	}

	if (ret != -EBUSY && rcvd < num_frames) {
		struct io_device *iod = rb->iod;
		struct link_device *ld = rb->ld;
		struct modem_ctl *mc = ld->mc;

		mif_err_limited("%s: %s<-%s: WARN! rcvd %d < num_frames %d\n",
			ld->name, iod->name, mc->name, rcvd, num_frames);
	}

	*work_done = rcvd;
	allocate_data_in_advance(zdptr);

	return ret;
}

static int rx_net_frames_from_rb(struct sbd_ring_buffer *rb, int budget,
		int *work_done)
{
	int rcvd = 0;
	struct link_device *ld = rb->ld;
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	unsigned int num_frames;
	int ret = 0;

	num_frames = min_t(unsigned int, rb_usage(rb), budget);

	while (rcvd < num_frames) {
		struct sk_buff *skb;

		skb = sbd_pio_rx(rb);
		if (!skb)
			return -ENOMEM;

		/* The $rcvd must be accumulated here, because $skb can be freed
		 * in pass_skb_to_net().
		 */
		rcvd++;

		ret = pass_skb_to_net(mld, skb);
		if (ret < 0)
			break;
	}

	if (ret != -EBUSY && rcvd < num_frames) {
		struct io_device *iod = rb->iod;
		struct link_device *ld = rb->ld;
		struct modem_ctl *mc = ld->mc;

		mif_err("%s: %s<-%s: WARN! rcvd %d < num_frames %d\n",
			ld->name, iod->name, mc->name, rcvd, num_frames);
	}

	*work_done = rcvd;

	return ret;
}

static int rx_ipc_frames_from_rb(struct sbd_ring_buffer *rb)
{
	int rcvd = 0;
	struct link_device *ld = rb->ld;
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	unsigned int qlen = rb->len;
	unsigned int in = *rb->wp;
	unsigned int out = *rb->rp;
	unsigned int num_frames = circ_get_usage(qlen, in, out);

	while (rcvd < num_frames) {
		struct sk_buff *skb;

		skb = sbd_pio_rx(rb);
		if (!skb)
			return -ENOMEM;

		/* The $rcvd must be accumulated here, because $skb can be freed
		 * in pass_skb_to_demux().
		 */
		rcvd++;

		if (skbpriv(skb)->lnk_hdr) {
			u8 ch = rb->ch;
			u8 fch = ld->get_ch(skb->data);

			if (fch != ch) {
				mif_err("frm.ch:%d != rb.ch:%d\n", fch, ch);
				pr_skb("CRASH", skb, ld);
				dev_kfree_skb_any(skb);
				link_trigger_cp_crash(mld, CRASH_REASON_MIF_RX_BAD_DATA,
					"frm.ch is not same with rb.ch");
				continue;
			}
		}

		pass_skb_to_demux(mld, skb);
	}

	if (rcvd < num_frames) {
		struct io_device *iod = rb->iod;
		struct modem_ctl *mc = ld->mc;

		mif_err("%s: %s<-%s: WARN! rcvd %d < num_frames %d\n",
			ld->name, iod->name, mc->name, rcvd, num_frames);
	}
	return rcvd;
}

static int sbd_ipc_rx_func_napi(struct link_device *ld, struct io_device *iod,
		int budget, int *work_done)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct sbd_ring_buffer *rb = sbd_ch2rb(&mld->sbd_link_dev, iod->ch, RX);
	int rcvd = 0;
	int ret;

	if (rb->zerocopy)
		ret = rx_net_frames_from_zerocopy_adaptor(rb, budget, &rcvd);
	else
		ret = rx_net_frames_from_rb(rb, budget, &rcvd);

	if (IS_ERR_VALUE((unsigned long)ret) && (ret != -EBUSY))
		mif_err_limited("RX error (%d)\n", ret);

	*work_done = rcvd;
	return ret;
}

static int legacy_ipc_rx_func_napi(struct mem_link_device *mld, struct legacy_ipc_device *dev,
		int budget, int *work_done)
{
	struct link_device *ld = &mld->link_dev;
	unsigned int qsize = get_rxq_buff_size(dev);
	unsigned int in = get_rxq_head(dev);
	unsigned int out = get_rxq_tail(dev);
	unsigned int size = circ_get_usage(qsize, in, out);
	int rcvd = 0;
	int err = 0;

	if (unlikely(circ_empty(in, out)))
		return 0;

#if IS_ENABLED(CONFIG_CACHED_LEGACY_RAW_RX_BUFFER)
	if (dev->id == IPC_MAP_NORM_RAW) {
		char *src = get_rxq_buff(dev);

		if (!src) {
			mif_err_limited("get_rxq_buff() error\n");
			return -EINVAL;
		}

		if ((out + size) <= qsize)
			dma_sync_single_for_cpu(ld->dev, virt_to_phys(src + out), size,
					DMA_FROM_DEVICE);
		else {
			dma_sync_single_for_cpu(ld->dev, virt_to_phys(src + out), qsize - out,
					DMA_FROM_DEVICE);
			dma_sync_single_for_cpu(ld->dev, virt_to_phys(src), size - (qsize - out),
					DMA_FROM_DEVICE);
		}
	}
#endif

	while ((budget != 0) && (rcvd < size)) {
		struct sk_buff *skb;
		u8 ch;
		struct io_device *iod;

		skb = recv_from_legacy_link(mld, dev, in, &err);
		if (err)
			return err;

		ch = ld->get_ch(skb->data);
		iod = link_get_iod_with_channel(ld, ch);
		if (!iod) {
			mif_err("%s: ERR! [%s]No IOD for CH.%d(out:%u)\n",
				ld->name, dev->name, ch, get_rxq_tail(dev));
			pr_skb("CRASH", skb, ld);
			dev_kfree_skb_any(skb);
			link_trigger_cp_crash(mld, CRASH_REASON_MIF_RX_BAD_DATA,
				"ERR! No IOD from CP");
			break;
		}

		/* Record the IO device and the link device into the &skb->cb */
		skbpriv(skb)->iod = iod;
		skbpriv(skb)->ld = ld;

		skbpriv(skb)->lnk_hdr = iod->link_header;
		skbpriv(skb)->sipc_ch = ch;
		skbpriv(skb)->napi = &mld->mld_napi;
		/* The $rcvd must be accumulated here, because $skb can be freed
		 * in pass_skb_to_demux().
		 */
		rcvd += skb->len;

		if (ld->is_ps_ch(ch)) {
			budget--;
			*work_done += 1;
		}

		pass_skb_to_demux(mld, skb);
	}

	if ((budget != 0) && (rcvd < size)) {
		struct link_device *ld = &mld->link_dev;

		mif_err("%s: WARN! rcvd %d < size %d\n", ld->name, rcvd, size);
	}

	return 0;
}

static int legacy_ipc_rx_func(struct mem_link_device *mld, struct legacy_ipc_device *dev)
{
	struct link_device *ld = &mld->link_dev;
	unsigned int qsize = get_rxq_buff_size(dev);
	unsigned int in = get_rxq_head(dev);
	unsigned int out = get_rxq_tail(dev);
	unsigned int size = circ_get_usage(qsize, in, out);
	int rcvd = 0;
	int err = 0;

	if (unlikely(circ_empty(in, out)))
		return 0;

#if IS_ENABLED(CONFIG_CACHED_LEGACY_RAW_RX_BUFFER)
	if (dev->id == IPC_MAP_NORM_RAW) {
		char *src = get_rxq_buff(dev);

		if (!src) {
			mif_err_limited("get_rxq_buff() error\n");
			return -EINVAL;
		}

		if ((out + size) <= qsize)
			dma_sync_single_for_cpu(ld->dev, virt_to_phys(src + out), size,
					DMA_FROM_DEVICE);
		else {
			dma_sync_single_for_cpu(ld->dev, virt_to_phys(src + out), qsize - out,
					DMA_FROM_DEVICE);
			dma_sync_single_for_cpu(ld->dev, virt_to_phys(src), size - (qsize - out),
					DMA_FROM_DEVICE);
		}
	}
#endif

	while (rcvd < size) {
		struct sk_buff *skb;
		u8 ch;
		struct io_device *iod;

		skb = recv_from_legacy_link(mld, dev, in, &err);
		if (err)
			return err;

		ch = ld->get_ch(skb->data);
		iod = link_get_iod_with_channel(ld, ch);
		if (!iod) {
			mif_err("%s: ERR! [%s]No IOD for CH.%d(out:%u)\n",
				ld->name, dev->name, ch, get_rxq_tail(dev));
			pr_skb("CRASH", skb, ld);
			dev_kfree_skb_any(skb);
			link_trigger_cp_crash(mld, CRASH_REASON_MIF_RX_BAD_DATA,
				"ERR! No IOD from CP in rx_frames_from_dev()");
			break;
		}

		/* Record the IO device and the link device into the &skb->cb */
		skbpriv(skb)->iod = iod;
		skbpriv(skb)->ld = ld;

		skbpriv(skb)->lnk_hdr = iod->link_header;
		skbpriv(skb)->sipc_ch = ch;

		skbpriv(skb)->napi = NULL;

		/* The $rcvd must be accumulated here, because $skb can be freed
		 * in pass_skb_to_demux().
		 */
		rcvd += skb->len;
		pass_skb_to_demux(mld, skb);
	}

	if (rcvd < size) {
		struct link_device *ld = &mld->link_dev;

		mif_err("%s: WARN! rcvd %d < size %d\n", ld->name, rcvd, size);
	}

	return rcvd;
}

#if IS_ENABLED(CONFIG_MCU_IPC)
static ktime_t rx_int_enable_time;
static ktime_t rx_int_disable_time;
#endif

static int shmem_enable_rx_int(struct link_device *ld)
{
#if IS_ENABLED(CONFIG_MCU_IPC)
	struct mem_link_device *mld = to_mem_link_device(ld);

	if (ld->interrupt_types == INTERRUPT_MAILBOX) {
		mld->rx_int_enable = 1;
		if (rx_int_disable_time) {
			rx_int_enable_time = ktime_get();
			mld->rx_int_disabled_time += ktime_to_us(ktime_sub(rx_int_enable_time,
						rx_int_disable_time));
			rx_int_enable_time = 0;
			rx_int_disable_time = 0;
		}
		return cp_mbox_enable_handler(CP_MBOX_IRQ_IDX_0, mld->irq_cp2ap_msg);
	}
#endif

	return 0;
}

static int shmem_disable_rx_int(struct link_device *ld)
{
#if IS_ENABLED(CONFIG_MCU_IPC)
	struct mem_link_device *mld = to_mem_link_device(ld);

	if (ld->interrupt_types == INTERRUPT_MAILBOX) {
		mld->rx_int_enable = 0;
		rx_int_disable_time = ktime_get();

		return cp_mbox_disable_handler(CP_MBOX_IRQ_IDX_0, mld->irq_cp2ap_msg);
	}
#endif

	return 0;
}

static int update_handover_block_info(struct link_device *ld, unsigned long arg)
{
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	struct t_handover_block_info info;
	int err = 0;

	err = copy_from_user(&info, (const void __user *)arg,
			sizeof(struct t_handover_block_info));
	if (err) {
		mif_err("%s: ERR! handover_block_info copy_from_user fail\n",
				ld->name);
		return -EFAULT;
	}

	mif_info("%s: call send_handover_block_info (sku: %d, rev: %d)\n",
		ld->name, info.modem_sku, info.minor_id);

	memcpy(mld->ap2cp_handover_block_info.addr, &info,
			sizeof(struct t_handover_block_info));

	return 0;
}

static int bootdump_rx_func(struct mem_link_device *mld)
{
	int ret = 0;
	struct legacy_ipc_device *dev = mld->legacy_link_dev.dev[IPC_MAP_NORM_RAW];

	u32 qlen = mld->msb_rxq.qlen;

	while (qlen-- > 0) {
		struct mst_buff *msb;
		u16 intr;

		msb = msb_dequeue(&mld->msb_rxq);
		if (!msb)
			break;
		intr = msb->snapshot.int2ap;
		if (cmd_valid(intr))
			mld->cmd_handler(mld, int2cmd(intr));
		msb_free(msb);
	}

	/* recv frames from RAW buffer which should contain bootdump frames */
	ret = legacy_ipc_rx_func(mld, dev);
	if (ret == -ENOMEM) {
		if (!work_pending(&mld->page_reclaim_work)) {
			struct link_device *ld = &mld->link_dev;

			mif_err_limited("Rx ENOMEM, try reclaim work");
			queue_work(ld->rx_wq,
					&mld->page_reclaim_work);
		}
	}

	return ret;
}

static void bootdump_oom_handler_work(struct work_struct *ws)
{
	struct mem_link_device *mld =
		container_of(ws, struct mem_link_device, page_reclaim_work);
	struct sk_buff *skb;

	/* try to page reclaim with GFP_KERNEL */
	skb = alloc_skb(PAGE_SIZE - 512, GFP_KERNEL);
	if (skb)
		dev_kfree_skb_any(skb);

	/* need to disable the RX irq ?? */
	msleep(200);

	mif_info("trigger the rx task again\n");
	bootdump_rx_func(mld);
}

static void bootdump_rx_work(struct work_struct *ws)
{
	struct mem_link_device *mld;

	mld = container_of(ws, struct mem_link_device, bootdump_rx_dwork.work);

	bootdump_rx_func(mld);
}

/*============================================================================*/
static int shmem_init_comm(struct link_device *ld, struct io_device *iod)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct modem_ctl *mc = ld->mc;
	struct io_device *check_iod;
	int id = iod->ch;
	int fmt2rfs = (ld->chid_rfs_0 - ld->chid_fmt_0);
	int rfs2fmt = (ld->chid_fmt_0 - ld->chid_rfs_0);

	if (atomic_read(&mld->cp_boot_done))
		return 0;

#if IS_ENABLED(CONFIG_LINK_CONTROL_MSG_IOSM)
	if (mld->iosm) {
		struct sbd_link_device *sl = &mld->sbd_link_dev;
		struct sbd_ipc_device *sid = sbd_ch2dev(sl, iod->ch);

		if (!atomic_read(&sid->config_done)) {
			mif_err("%s isn't configured channel\n", iod->name);
			return -ENODEV;
		}

		tx_iosm_message(mld, IOSM_A2C_OPEN_CH, (u32 *)&id);
		return 0;
	}
#endif

	if (ld->protocol == PROTOCOL_SIT)
		return 0;

	if (ld->is_fmt_ch(id)) {
		check_iod = link_get_iod_with_channel(ld, (id + fmt2rfs));
		if (check_iod ? atomic_read(&check_iod->opened) : true) {
			if (ld->link_type == LINKDEV_SHMEM)
				write_clk_table_to_shmem(mld);

			mif_err("%s: %s->INIT_END->%s\n",
				ld->name, iod->name, mc->name);
			if (!atomic_read(&mld->cp_boot_done)) {
				send_ipc_irq(mld, cmd2int(CMD_INIT_END));
				atomic_set(&mld->cp_boot_done, 1);
			}
		} else {
			mif_err("%s is not opened yet\n", check_iod->name);
		}
	}

	if (ld->is_rfs_ch(id)) {
		check_iod = link_get_iod_with_channel(ld, (id + rfs2fmt));
		if (check_iod) {
			if (atomic_read(&check_iod->opened)) {
				if (ld->link_type == LINKDEV_SHMEM)
					write_clk_table_to_shmem(mld);

				mif_err("%s: %s->INIT_END->%s\n",
					ld->name, iod->name, mc->name);
				if (!atomic_read(&mld->cp_boot_done)) {
					send_ipc_irq(mld, cmd2int(CMD_INIT_END));
					atomic_set(&mld->cp_boot_done, 1);
				}
			} else {
				mif_err("%s not opened yet\n", check_iod->name);
			}
		}
	}

	return 0;
}

static void shmem_terminate_comm(struct link_device *ld, struct io_device *iod)
{
#if IS_ENABLED(CONFIG_LINK_CONTROL_MSG_IOSM)
	struct mem_link_device *mld = to_mem_link_device(ld);

	if (mld->iosm)
		tx_iosm_message(mld, IOSM_A2C_CLOSE_CH, (u32 *)&iod->ch);
#endif
}

static int shmem_send(struct link_device *ld, struct io_device *iod,
		    struct sk_buff *skb)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	u8 ch = iod->ch;

	return xmit_to_cp(mld, iod, ch, skb);
}

static void link_prepare_normal_boot(struct link_device *ld, struct io_device *iod)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	unsigned long flags;

	atomic_set(&mld->cp_boot_done, 0);

	spin_lock_irqsave(&mld->state_lock, flags);
	mld->state = LINK_STATE_OFFLINE;
	spin_unlock_irqrestore(&mld->state_lock, flags);

	cancel_tx_timer(mld, &mld->tx_timer);

	if (ld->sbd_ipc) {
#if IS_ENABLED(CONFIG_LTE_MODEM_XMM7260)
		sbd_deactivate(&mld->sbd_link_dev);
#endif
		cancel_tx_timer(mld, &mld->sbd_tx_timer);
		cancel_datalloc_timer(mld);

		if (mld->iosm) {
			memset(mld->base + CMD_RGN_OFFSET, 0, CMD_RGN_SIZE);
			mif_info("Control message region has been initialized\n");
		}
	}

#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	cancel_tx_timer(mld, &mld->pktproc_tx_timer);
#endif

	purge_txq(mld);
}

static int link_load_cp_image(struct link_device *ld, struct io_device *iod,
		     unsigned long arg)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
#if !IS_ENABLED(CONFIG_CP_SECURE_BOOT)
	struct resource *cpmem_info = mld->syscp_info;
#endif
	void __iomem *dst;
	void __user *src;
	int err;
	struct cp_image img;
	void __iomem *v_base;
	size_t valid_space;

	/**
	 * Get the information about the boot image
	 */
	memset(&img, 0, sizeof(struct cp_image));

	err = copy_from_user(&img, (const void __user *)arg, sizeof(img));
	if (err) {
		mif_err("%s: ERR! INFO copy_from_user fail\n", ld->name);
		return -EFAULT;
	}

	if (mld->boot_base == NULL) {
		mld->boot_base = cp_shmem_get_nc_region(cp_shmem_get_base(ld->mdm_data->cp_num,
					SHMEM_CP), mld->boot_size);
		if (!mld->boot_base) {
			mif_err("Failed to vmap boot_region\n");
			return -EINVAL;		/* TODO : need better return */
		}
	}

	/* Calculate size of valid space which BL will download */
	valid_space = (img.mode) ? mld->size : mld->boot_size;
	/* Calculate base address (0: BOOT_MODE, 1: DUMP_MODE) */
	v_base = (img.mode) ? mld->base : mld->boot_base;

	/**
	 * Check the size of the boot image
	 * fix the integer overflow of "img.m_offset + img.len" from Jose Duart
	 */
	if (img.size > valid_space || img.len > valid_space
			|| img.m_offset > valid_space - img.len) {
		mif_err("%s: ERR! Invalid args: size %x, offset %x, len %x\n",
			ld->name, img.size, img.m_offset, img.len);
		return -EINVAL;
	}

	dst = (void __iomem *)(v_base + img.m_offset);
	src = (void __user *)((unsigned long)img.binary);

#if !IS_ENABLED(CONFIG_CP_SECURE_BOOT)
	if (img.m_offset == (u32)cpmem_info->start)
		mld->cp_binary_size = img.size;
#endif

	err = copy_from_user(dst, src, img.len);
	if (err) {
		mif_err("%s: ERR! BOOT copy_from_user fail\n", ld->name);
		return err;
	}

	return 0;
}

#if !IS_ENABLED(CONFIG_CP_SECURE_BOOT)
unsigned long shmem_calculate_CRC32(const unsigned char *buf, unsigned long len)
{
	unsigned long ul_crc;

	if (buf == 0)
		return 0L;

	ul_crc = CRC32_XINIT;
	while (len--)
		ul_crc = CRC32_TABLE[(ul_crc ^ *buf++) & 0xFF] ^ (ul_crc >> 8);

	ul_crc ^= CRC32_XOROT;

	return ul_crc;
}

void shmem_check_modem_binary_crc(struct link_device *ld)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct resource *cpmem_info = mld->syscp_info;
	unsigned char *data;
	unsigned long CRC;

	data = (unsigned char *)mld->boot_base + (u32)cpmem_info->start;

	CRC = shmem_calculate_CRC32(data, mld->cp_binary_size);

	mif_info("Modem Main Binary CRC: %08X\n", (unsigned int)CRC);

}
#endif

unsigned long shm_get_security_param2(u32 cp_num, unsigned long mode, u32 bl_size)
{
	unsigned long ret;

	switch (mode) {
	case CP_BOOT_MODE_NORMAL:
	case CP_BOOT_MODE_DUMP:
		ret = bl_size;
		break;
	case CP_BOOT_RE_INIT:
		ret = 0;
		break;
	case CP_BOOT_MODE_MANUAL:
		ret = cp_shmem_get_base(cp_num, SHMEM_CP) + bl_size;
		break;
	default:
		mif_info("Invalid sec_mode(%lu)\n", mode);
		ret = 0;
		break;
	}
	return ret;
}

unsigned long shm_get_security_param3(u32 cp_num, unsigned long mode, u32 main_size)
{
	unsigned long ret;

	switch (mode) {
	case CP_BOOT_MODE_NORMAL:
		ret = main_size;
		break;
	case CP_BOOT_MODE_DUMP:
#ifdef CP_NONSECURE_BOOT
		ret = cp_shmem_get_base(cp_num, SHMEM_CP);
#else
		ret = cp_shmem_get_base(cp_num, SHMEM_IPC);
#endif
		break;
	case CP_BOOT_RE_INIT:
		ret = 0;
		break;
	case CP_BOOT_MODE_MANUAL:
		ret = main_size;
		break;
	default:
		mif_info("Invalid sec_mode(%lu)\n", mode);
		ret = 0;
		break;
	}
	return ret;
}

#define MAX_TRY_CNT			0x1000
#define MODE_CP_CHECK_CONTINUE		0x8

static int shmem_security_request(struct link_device *ld, struct io_device *iod,
				unsigned long arg)
{
	unsigned long mode, param2, param3;
	int err = 0;
	struct modem_sec_req msr;
#if IS_ENABLED(CONFIG_CP_SECURE_BOOT)
	uint32_t try_cnt = 0;
#endif
	u32 cp_num = ld->mdm_data->cp_num;
	struct mem_link_device *mld = ld->mdm_data->mld;

	err = copy_from_user(&msr, (const void __user *)arg, sizeof(msr));
	if (err) {
		mif_err("%s: ERR! copy_from_user fail\n", ld->name);
		err = -EFAULT;
		goto exit;
	}

	mode = (unsigned long)msr.mode;
	param2 = shm_get_security_param2(cp_num, mode, msr.param2);
	param3 = shm_get_security_param3(cp_num, mode, msr.param3);

#if !IS_ENABLED(CONFIG_CP_SECURE_BOOT)
	if (mode == CP_BOOT_MODE_NORMAL)
		shmem_check_modem_binary_crc(ld);
	/* boot_base should be unmapped after its usage on crc check */
	if (mld->boot_base != NULL) {
		vunmap(mld->boot_base);
		mld->boot_base = NULL;
	}
#else
	/* boot_base is in no use at this point */
	if (mld->boot_base != NULL) {
		vunmap(mld->boot_base);
		mld->boot_base = NULL;
	}
	exynos_smc(SMC_ID_CLK, SSS_CLK_ENABLE, 0, 0);
	if ((mode == CP_BOOT_MODE_NORMAL) && cp_shmem_get_mem_map_on_cp_flag(cp_num))
		mode |= cp_shmem_get_base(cp_num, SHMEM_CP);

	mif_info("mode=0x%lx, param2=0x%lx, param3=0x%lx, cp_base_addr=0x%lx\n",
			mode, param2, param3, cp_shmem_get_base(cp_num, SHMEM_CP));
	err = (int)exynos_smc(SMC_ID, mode, param2, param3);

	while (err == CP_CHECK_SIGN_NOT_FINISH && try_cnt < MAX_TRY_CNT) {
		try_cnt++;
		err = (int)exynos_smc(SMC_ID, MODE_CP_CHECK_CONTINUE, 0x0, 0x0);
	}

	exynos_smc(SMC_ID_CLK, SSS_CLK_DISABLE, 0, 0);

	if (try_cnt >= MAX_TRY_CNT) {
		mif_info("%s: it fails to check signature of main binary.\n"
								, ld->name);
	}

	mif_info("%s: return_value=0x%08x(%s)\n", ld->name, err,
			err < sizeof(smc_err_string) ? smc_err_string[err] : "NULL");
#endif

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if ((mode == CP_BOOT_RE_INIT) && mld->pktproc.use_dedicated_baaw) {
		mif_info("memaddr:0x%lx memsize:0x%08x",
				cp_shmem_get_base(cp_num, SHMEM_PKTPROC),
				cp_shmem_get_size(cp_num, SHMEM_PKTPROC));

		exynos_smc(SMC_ID_CLK, SSS_CLK_ENABLE, 0, 0);
		err = (int)exynos_smc(SMC_ID, CP_BOOT_EXT_BAAW,
			(unsigned long)cp_shmem_get_base(cp_num, SHMEM_PKTPROC),
			(unsigned long)cp_shmem_get_size(cp_num, SHMEM_PKTPROC));

		exynos_smc(SMC_ID_CLK, SSS_CLK_DISABLE, 0, 0);
		if (err)
			mif_err("ERROR: SMC call failure:%d\n", err);
	}
#endif

exit:
	return err;
}

long gro_flush_time = 10000;
module_param(gro_flush_time, long, 0644);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
static void gro_flush_timer(struct link_device *ld, struct napi_struct *napi)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct timespec64 curr, diff;

	if (!gro_flush_time) {
		napi_gro_flush(napi, false);
		return;
	}

	if (unlikely(mld->flush_time.tv_sec == 0)) {
		ktime_get_real_ts64(&mld->flush_time);
	} else {
		ktime_get_real_ts64(&(curr));
		diff = timespec64_sub(curr, mld->flush_time);
		if ((diff.tv_sec > 0) || (diff.tv_nsec > gro_flush_time)) {
			napi_gro_flush(napi, false);
			ktime_get_real_ts64(&mld->flush_time);
		}
	}
}

static struct timespec64 update_flush_time(struct timespec64 org_flush_time)
{
	struct timespec64 curr, diff, ret;

	ktime_get_real_ts64(&curr);
	if (unlikely(org_flush_time.tv_sec == 0)) {
		ktime_get_real_ts64(&ret);
		return ret;
	}
	diff = timespec64_sub(curr, org_flush_time);
	if (diff.tv_nsec > gro_flush_time)
		ktime_get_real_ts64(&ret);
	else
		ret = org_flush_time;

	return ret;
}
#else
static void gro_flush_timer(struct link_device *ld, struct napi_struct *napi)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct timespec curr, diff;

	if (!gro_flush_time) {
		napi_gro_flush(napi, false);
		return;
	}

	if (unlikely(mld->flush_time.tv_sec == 0)) {
		getnstimeofday(&mld->flush_time);
	} else {
		getnstimeofday(&(curr));
		diff = timespec_sub(curr, mld->flush_time);
		if ((diff.tv_sec > 0) || (diff.tv_nsec > gro_flush_time)) {
			napi_gro_flush(napi, false);
			getnstimeofday(&mld->flush_time);
		}
	}
}
static struct timespec update_flush_time(struct timespec org_flush_time)
{
	struct timespec curr, diff, ret;

	getnstimeofday(&curr);
	if (unlikely(org_flush_time.tv_sec == 0)) {
		getnstimeofday(&ret);
		return ret;
	}
	diff = timespec_sub(curr, org_flush_time);
	if (diff.tv_nsec > gro_flush_time)
		getnstimeofday(&ret);
	else
		ret = org_flush_time;

	return ret;
}
#endif

static int sbd_link_rx_func_napi(struct sbd_link_device *sl, struct link_device *ld, int budget,
		int *work_done)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < sl->num_channels ; i++) {
		struct sbd_ring_buffer *rb = sbd_id2rb(sl, i, RX);
		int ps_rcvd = 0;

		if (unlikely(rb_empty(rb)))
			continue;
		if ((budget <= 0) && ld->is_ps_ch(sbd_id2ch(sl, i)))
			continue;
		if (!ld->is_ps_ch(sbd_id2ch(sl, i)))
			ret = rx_ipc_frames_from_rb(rb);
		else /* ps channels */
			ret = sbd_ipc_rx_func_napi(ld, rb->iod, budget, &ps_rcvd);
		if ((ret == -EBUSY) || (ret == -ENOMEM))
			break;
		if (ld->is_ps_ch(sbd_id2ch(sl, i))) {
			/* count budget only for ps frames */
			budget -= ps_rcvd;
			*work_done += ps_rcvd;
		}
	}
	return ret;
}

static int legacy_link_rx_func_napi(struct mem_link_device *mld, int budget, int *work_done)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < MAX_SIPC_MAP; i++) {
		struct legacy_ipc_device *dev = mld->legacy_link_dev.dev[i];
		int ps_rcvd = 0;

		if (unlikely(circ_empty(get_rxq_head(dev), get_rxq_tail(dev))))
			continue; /* go to next device */
		if (budget <= 0)
			break;
		ret = legacy_ipc_rx_func_napi(mld, dev, budget, &ps_rcvd);
		if ((ret == -EBUSY) || (ret == -ENOMEM))
			break;
		/* count budget for all frames */
		budget -= ps_rcvd;
		*work_done += ps_rcvd;

	}

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
	tpmon_add_legacy_packet_count(*work_done);
#endif

	return ret;
}

static int shmem_enqueue_snapshot(struct mem_link_device *mld);

/*
 * mld_rx_int_poll
 *
 * This NAPI poll function does not handle reception of any network frames.
 * It is used for servicing CP2AP commands and FMT RX frames while the RX
 * mailbox interrupt is masked. When the mailbox interrupt is masked, CP can
 * set the interrupt but the AP will not react. However, the interrupt status
 * bit will still be set, so we can poll the status bit to handle new RX
 * interrupts.
 * If the RAW NAPI functions are no longer scheduled at the end of this poll
 * function, we can enable the mailbox interrupt and stop polling.
 */
static int mld_rx_int_poll(struct napi_struct *napi, int budget)
{
	struct mem_link_device *mld = container_of(napi, struct mem_link_device,
			mld_napi);
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	struct sbd_link_device *sl = &mld->sbd_link_dev;
	int total_ps_rcvd = 0;
	int ps_rcvd = 0;
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	int i = 0;
#endif
	int ret = 1;
	int total_budget = budget;
	u32 qlen = 0;

#if IS_ENABLED(CONFIG_MCU_IPC)
	if (ld->interrupt_types == INTERRUPT_MAILBOX)
		ret = cp_mbox_check_handler(CP_MBOX_IRQ_IDX_0, mld->irq_cp2ap_msg);
#endif
	if (IS_ERR_VALUE((unsigned long)ret)) {
		mif_err_limited("mbox check irq fails: err: %d\n", ret);
		goto dummy_poll_complete;
	}

	mld->rx_poll_count++;

	if (ret) { /* if an irq is raised, take care of commands */
		if (shmem_enqueue_snapshot(mld))
			goto dummy_poll_complete;

		qlen = mld->msb_rxq.qlen;

		if (unlikely(!cp_online(mc))) { /* for boot and dump sequences */
			queue_delayed_work(ld->rx_wq, &mld->bootdump_rx_dwork, 0);
			goto dummy_poll_complete;
		}

		while (qlen-- > 0) {
			struct mst_buff *msb;
			u16 intr;

			msb = msb_dequeue(&mld->msb_rxq);
			if (!msb)
				break;
			intr = msb->snapshot.int2ap;
			if (cmd_valid(intr))
				mld->cmd_handler(mld, int2cmd(intr));
			msb_free(msb);
		}
	}

	if (sbd_active(&mld->sbd_link_dev)) {
		ret = sbd_link_rx_func_napi(sl, ld, budget, &ps_rcvd);
		if ((ret == -EBUSY) || (ret == -ENOMEM))
			goto keep_poll;
		budget -= ps_rcvd;
		total_ps_rcvd += ps_rcvd;
		ps_rcvd = 0;
	} else { /* legacy buffer */
		ret = legacy_link_rx_func_napi(mld, budget, &ps_rcvd);
		if ((ret == -EBUSY) || (ret == -ENOMEM))
			goto keep_poll;
		budget -= ps_rcvd;
		total_ps_rcvd += ps_rcvd;
		ps_rcvd = 0;
	}

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (pktproc_check_support(&mld->pktproc) &&
			!mld->pktproc.use_exclusive_irq &&
			mld->pktproc.use_napi) {
		for (i = 0; i < mld->pktproc.num_queue; i++) {
			if (!pktproc_check_active(&mld->pktproc, i))
				continue;

			ret = mld->pktproc.q[i]->clean_rx_ring(mld->pktproc.q[i], budget, &ps_rcvd);
			if ((ret == -EBUSY) || (ret == -ENOMEM))
				goto keep_poll;

			budget -= ps_rcvd;
			total_ps_rcvd += ps_rcvd;
			ps_rcvd = 0;
		}
	}
#endif

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
	if (total_ps_rcvd)
		tpmon_start();
#endif

	if (atomic_read(&mld->stop_napi_poll)) {
		atomic_set(&mld->stop_napi_poll, 0);
		goto dummy_poll_complete;
	}

	if (total_ps_rcvd < total_budget) {
		napi_complete_done(napi, total_ps_rcvd);
		ld->enable_rx_int(ld);
		return total_ps_rcvd;
	}

keep_poll:
	return total_budget;

dummy_poll_complete:
	napi_complete(napi);
	ld->enable_rx_int(ld);

	return 0;
}

static void sync_net_dev(struct link_device *ld)
{
	struct mem_link_device *mld = to_mem_link_device(ld);

	napi_synchronize(&mld->mld_napi);
	mif_info("%s\n", netdev_name(&mld->dummy_net));
}

static int link_start_normal_boot(struct link_device *ld, struct io_device *iod)
{
	struct mem_link_device *mld = to_mem_link_device(ld);

	if (ld->sbd_ipc && mld->attrs & LINK_ATTR_MEM_DUMP)
		sbd_deactivate(&mld->sbd_link_dev);

	sync_net_dev(ld);

	init_legacy_link(&mld->legacy_link_dev);
	skb_queue_purge(&iod->sk_rx_q);

	if (mld->attrs & LINK_ATTR_BOOT_ALIGNED)
		ld->aligned = true;
	else
		ld->aligned = false;

	if (mld->dpram_magic) {
		unsigned int magic;

		iowrite32(ld->magic_boot, mld->legacy_link_dev.magic);
		magic = ioread32(mld->legacy_link_dev.magic);
		if (magic != ld->magic_boot) {
			mif_err("%s: ERR! magic 0x%08X != BOOT_MAGIC 0x%08X\n",
				ld->name, magic, ld->magic_boot);
			return -EFAULT;
		}
		mif_info("%s: magic == 0x%08X\n", ld->name, magic);
	}

	return 0;
}

static int link_start_dump_boot(struct link_device *ld, struct io_device *iod)
{
	struct mem_link_device *mld = to_mem_link_device(ld);

	if (ld->sbd_ipc && mld->attrs & LINK_ATTR_MEM_DUMP)
		sbd_deactivate(&mld->sbd_link_dev);

	sync_net_dev(ld);

	init_legacy_link(&mld->legacy_link_dev);
	skb_queue_purge(&iod->sk_rx_q);

	if (mld->attrs & LINK_ATTR_DUMP_ALIGNED)
		ld->aligned = true;
	else
		ld->aligned = false;

	if (mld->dpram_magic) {
		unsigned int magic;

		iowrite32(ld->magic_dump, mld->legacy_link_dev.magic);
		magic = ioread32(mld->legacy_link_dev.magic);
		if (magic != ld->magic_dump) {
			mif_err("%s: ERR! magic 0x%08X != DUMP_MAGIC 0x%08X\n",
				ld->name, magic, ld->magic_dump);
			return -EFAULT;
		}
		mif_err("%s: magic == 0x%08X\n", ld->name, magic);
	}

	return 0;
}

static void shmem_close_tx(struct link_device *ld)
{
	struct mem_link_device *mld = to_mem_link_device(ld);
	unsigned long flags;

	spin_lock_irqsave(&mld->state_lock, flags);
	mld->state = LINK_STATE_OFFLINE;
	spin_unlock_irqrestore(&mld->state_lock, flags);

	if (timer_pending(&mld->crash_ack_timer))
		del_timer(&mld->crash_ack_timer);

	stop_net_ifaces(ld);
	purge_txq(mld);
}

static int get_cp_crash_reason(struct link_device *ld, struct io_device *iod,
		unsigned long arg)
{
	int ret;

	ret = copy_to_user((void __user *)arg, &ld->crash_reason,
		sizeof(struct crash_reason));
	if (ret) {
		mif_err("ERR! copy_to_user fail!\n");
		return -EFAULT;
	}

	return 0;
}
/*============================================================================*/


#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM)
static u16 shmem_recv_cp2ap_irq(struct mem_link_device *mld)
{
	return get_ctrl_msg(&mld->cp2ap_msg);
}

static u16 shmem_recv_cp2ap_status(struct mem_link_device *mld)
{
	return (u16)extract_ctrl_msg(&mld->cp2ap_united_status, mld->sbi_cp_status_mask,
			mld->sbi_cp_status_pos);
}

static void shmem_send_ap2cp_irq(struct mem_link_device *mld, u16 mask)
{
	set_ctrl_msg(&mld->ap2cp_msg, mask);

	cp_mbox_set_interrupt(CP_MBOX_IRQ_IDX_0, mld->int_ap2cp_msg);
}

static inline u16 shmem_read_ap2cp_irq(struct mem_link_device *mld)
{
	return (u16)get_ctrl_msg(&mld->ap2cp_msg);
}
#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
static u16 pcie_recv_cp2ap_irq(struct mem_link_device *mld)
{
	return (u16)get_ctrl_msg(&mld->cp2ap_msg);
}

static u16 pcie_recv_cp2ap_status(struct mem_link_device *mld)
{
	return (u16)get_ctrl_msg(&mld->cp2ap_united_status);
}

static void pcie_send_ap2cp_irq(struct mem_link_device *mld, u16 mask)
{
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	unsigned long flags;
	bool force_crash = false;

	spin_lock_irqsave(&mc->pcie_tx_lock, flags);

	if (mutex_is_locked(&mc->pcie_onoff_lock)) {
		mif_err_limited("Reserve doorbell interrupt: PCI on/off working\n");
		set_ctrl_msg(&mld->ap2cp_msg, mask);
		mc->reserve_doorbell_int = true;
		goto exit;
	}

	if (!mc->pcie_powered_on) {
		mif_err_limited("Reserve doorbell interrupt: PCI not powered on\n");
		set_ctrl_msg(&mld->ap2cp_msg, mask);
		mc->reserve_doorbell_int = true;
		s5100_try_gpio_cp_wakeup(mc);
		goto exit;
	}

	set_ctrl_msg(&mld->ap2cp_msg, mask);
	mc->reserve_doorbell_int = false;
	if (s51xx_pcie_send_doorbell_int(mc->s51xx_pdev, mld->intval_ap2cp_msg) != 0)
		force_crash = true;

exit:
	spin_unlock_irqrestore(&mc->pcie_tx_lock, flags);

	if (unlikely(force_crash))
		s5100_force_crash_exit_ext();
}

static inline u16 pcie_read_ap2cp_irq(struct mem_link_device *mld)
{
	return (u16)get_ctrl_msg(&mld->ap2cp_msg);
}
#endif

struct shmem_srinfo {
	unsigned int size;
	char buf[0];
};

#if IS_ENABLED(CONFIG_SBD_BOOTLOG)
static void shmem_clr_sbdcplog(struct mem_link_device *mld)
{
	u8 __iomem *base = mld->base + mld->size - SHMEM_BOOTSBDLOG_SIZE;

	memset(base, 0, SHMEM_BOOTSBDLOG_SIZE);
}

void shmem_pr_sbdcplog(struct timer_list *t)
{
	struct link_device *ld = from_timer(ld, t, cplog_timer);
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	u8 __iomem *base = mld->base + mld->size - SHMEM_BOOTSBDLOG_SIZE;
	u8 __iomem *offset;
	int i;

	mif_info("dump cp logs: size: 0x%x, base: %pK\n", (unsigned int)mld->size, base);

	for (i = 0; i * 32 < SHMEM_BOOTSBDLOG_SIZE; i++) {
		offset = base + i * 32;
		mif_info("%6X: %*ph\n", (unsigned int)(offset - mld->base), 32, offset);
	}

	shmem_clr_sbdcplog(mld);
}
#endif

/* not in use */
static int shmem_ioctl(struct link_device *ld, struct io_device *iod,
		       unsigned int cmd, unsigned long arg)
{
	struct mem_link_device *mld = ld_to_mem_link_device(ld);

	mif_info("%s: cmd 0x%08X\n", ld->name, cmd);
	switch (cmd) {
	case IOCTL_GET_SRINFO:
	{
		struct shmem_srinfo __user *sr_arg =
			(struct shmem_srinfo __user *)arg;
		unsigned int count, size = mld->srinfo_size;

		if (copy_from_user(&count, &sr_arg->size, sizeof(unsigned int)))
			return -EFAULT;

		mif_info("get srinfo:%s, size = %d\n", iod->name, count);

		size = min(size, count);
		if (copy_to_user(&sr_arg->size, &size, sizeof(unsigned int)))
			return -EFAULT;

		if (copy_to_user(sr_arg->buf, mld->srinfo_base, size))
			return -EFAULT;
		break;
	}

	case IOCTL_SET_SRINFO:
	{
		struct shmem_srinfo __user *sr_arg =
			(struct shmem_srinfo __user *)arg;
		unsigned int count, size = mld->srinfo_size;

		if (copy_from_user(&count, &sr_arg->size, sizeof(unsigned int)))
			return -EFAULT;

		mif_info("set srinfo:%s, size = %d\n", iod->name, count);

		if (copy_from_user(mld->srinfo_base, sr_arg->buf, min(count, size)))
			return -EFAULT;
		break;
	}

	case IOCTL_GET_CP_BOOTLOG:
	{
#if !IS_ENABLED(CONFIG_SBD_BOOTLOG)
		u8 __iomem *base = mld->base + SHMEM_BOOTLOG_BASE;
		char str[SHMEM_BOOTLOG_BUFF];
		unsigned int size = base[0]        + (base[1] << 8)
				 + (base[2] << 16) + (base[3] << 24);

		if (size <= 0 || size > SHMEM_BOOTLOG_BUFF - SHMEM_BOOTLOG_OFFSET) {
			mif_info("Invalid CP boot log[%d]\n", size);
			return -EINVAL;
		}

		strncpy(str, base + SHMEM_BOOTLOG_OFFSET, size);
		mif_info("CP boot log[%d] : %s\n", size, str);
#else
		mif_add_timer(&ld->cplog_timer, 0, shmem_pr_sbdcplog);
#endif
		break;
	}

	case IOCTL_CLR_CP_BOOTLOG:
	{
#if !IS_ENABLED(CONFIG_SBD_BOOTLOG)
		u8 __iomem *base = mld->base + SHMEM_BOOTLOG_BASE;

		mif_info("Clear CP boot log\n");
		memset(base, 0, SHMEM_BOOTLOG_BUFF);
#endif
		break;
	}

	default:
		mif_err("%s: ERR! invalid cmd 0x%08X\n", ld->name, cmd);
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t shmem_tx_state_handler(int irq, void *data)
{
	struct mem_link_device *mld = (struct mem_link_device *)data;
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;
	u16 int2ap_status;

	int2ap_status = mld->recv_cp2ap_status(mld);

	/* Change SHM_FLOWCTL to MASK_TX_FLOWCTRL */
	int2ap_status = (int2ap_status & SHM_FLOWCTL_BIT) << 2;

	switch (int2ap_status & (SHM_FLOWCTL_BIT << 2)) {
	case MASK_TX_FLOWCTL_SUSPEND:
		if (!chk_same_cmd(mld, int2ap_status))
			tx_flowctrl_suspend(mld);
		break;

	case MASK_TX_FLOWCTL_RESUME:
		if (!chk_same_cmd(mld, int2ap_status))
			tx_flowctrl_resume(mld);
		break;

	default:
		break;
	}

	if (unlikely(!rx_possible(mc))) {
		mif_err("%s: ERR! %s.state == %s\n", ld->name, mc->name,
			mc_state(mc));
	}

	return IRQ_HANDLED;
}

static int shmem_enqueue_snapshot(struct mem_link_device *mld)
{
	struct mst_buff *msb;
	struct link_device *ld = &mld->link_dev;
	struct modem_ctl *mc = ld->mc;

	msb = mem_take_snapshot(mld, RX);
	if (!msb)
		return -ENOMEM;

	if (unlikely(!int_valid(msb->snapshot.int2ap))) {
		mif_err("%s: ERR! invalid intr 0x%X\n",
				ld->name, msb->snapshot.int2ap);
		msb_free(msb);
		return -EINVAL;
	}

	if (unlikely(!rx_possible(mc))) {
		mif_err("%s: ERR! %s.state == %s\n", ld->name, mc->name,
			mc_state(mc));
		msb_free(msb);
		return -EINVAL;
	}

	msb_queue_tail(&mld->msb_rxq, msb);

	return 0;
}

static irqreturn_t shmem_irq_handler(int irq, void *data)
{
	struct mem_link_device *mld = (struct mem_link_device *)data;
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct pktproc_adaptor *ppa = &mld->pktproc;
	int i;
#endif

	mld->rx_int_count++;
	if (napi_schedule_prep(&mld->mld_napi)) {
		struct link_device *ld = &mld->link_dev;

		ld->disable_rx_int(ld);
		mld->flush_time = ld->update_flush_time(mld->flush_time);
		__napi_schedule(&mld->mld_napi);
	}

/* ToDo: irq might be disabled by disable_rx_int() */
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (pktproc_check_support(&mld->pktproc) &&
			!mld->pktproc.use_exclusive_irq &&
			!mld->pktproc.use_napi) {
		for (i = 0; i < ppa->num_queue; i++) {
			struct pktproc_queue *q = ppa->q[i];

			if (!pktproc_check_active(&mld->pktproc, i))
				continue;

			q->irq_handler(irq, q);
		}
	}
#endif

	return IRQ_HANDLED;
}

#if IS_ENABLED(CONFIG_MCU_IPC)
static irqreturn_t shmem_cp2ap_wakelock_handler(int irq, void *data)
{
	struct mem_link_device *mld = (struct mem_link_device *)data;
	unsigned int req;

	mif_err("%s\n", __func__);

	req = extract_ctrl_msg(&mld->cp2ap_united_status, mld->sbi_cp2ap_wakelock_mask,
			mld->sbi_cp2ap_wakelock_pos);

	if (req == 0) {
		if (cpif_wake_lock_active(mld->ws)) {
			cpif_wake_unlock(mld->ws);
			mif_err("cp_wakelock unlocked\n");
		} else {
			mif_err("cp_wakelock already unlocked\n");
		}
	} else if (req == 1) {
		if (cpif_wake_lock_active(mld->ws)) {
			mif_err("cp_wakelock already unlocked\n");
		} else {
			cpif_wake_lock(mld->ws);
			mif_err("cp_wakelock locked\n");
		}
	} else {
		mif_err("unsupported request: cp_wakelock\n");
	}

	return IRQ_HANDLED;
}
#endif

#if IS_ENABLED(CONFIG_MCU_IPC) && IS_ENABLED(CONFIG_PCI_EXYNOS)
static irqreturn_t shmem_cp2ap_rat_mode_handler(int irq, void *data)
{
	struct mem_link_device *mld = (struct mem_link_device *)data;
	unsigned int req;

	req = extract_ctrl_msg(&mld->cp2ap_united_status, mld->sbi_cp_rat_mode_mask,
			mld->sbi_cp_rat_mode_pos);

	mif_err("value: %u\n", req);

	if (req) {
		exynos_pcie_l1ss_ctrl(0, PCIE_L1SS_CTRL_MODEM_IF);
		mif_err("cp requests pcie l1ss disable\n");
	} else {
		exynos_pcie_l1ss_ctrl(1, PCIE_L1SS_CTRL_MODEM_IF);
		mif_err("cp requests pcie l1ss enable\n");
	}

	return IRQ_HANDLED;
}
#endif

#if IS_ENABLED(CONFIG_ECT)
static int parse_ect(struct mem_link_device *mld, char *dvfs_domain_name)
{
	int i, counter = 0;
	u32 mif_max_freq, mif_max_num_of_table = 0;
	void *dvfs_block;
	struct ect_dvfs_domain *dvfs_domain;

	dvfs_block = ect_get_block(BLOCK_DVFS);
	if (dvfs_block == NULL)
		return -ENODEV;

	dvfs_domain = ect_dvfs_get_domain(dvfs_block, (char *)dvfs_domain_name);
	if (dvfs_domain == NULL)
		return -ENODEV;

	if (!strcmp(dvfs_domain_name, "MIF")) {
		mld->mif_table.num_of_table = dvfs_domain->num_of_level;
		mif_max_num_of_table = dvfs_domain->num_of_level;
		mld->total_freq_table_count++;

		if (mld->mif_table.use_dfs_max_freq) {
			mif_info("use dfs max freq\n");
			mif_max_freq = cal_dfs_get_max_freq(mld->mif_table.cal_id_mif);

			for (i = 0; i < mif_max_num_of_table; i++) {
				if (dvfs_domain->list_level[i].level == mif_max_freq) {
					mif_max_num_of_table = mif_max_num_of_table - i;
					counter = i;
					break;
				}

				mld->mif_table.freq[mif_max_num_of_table - 1 - i] = mif_max_freq;
				mif_err("MIF_LEV[%d] : %u\n",
						mif_max_num_of_table - i, mif_max_freq);
			}
		}

		for (i = mif_max_num_of_table - 1; i >= 0; i--) {
			mld->mif_table.freq[i] =
				dvfs_domain->list_level[counter++].level;
			mif_err("MIF_LEV[%d] : %u\n", i + 1,
					mld->mif_table.freq[i]);
		}
	} else if (!strcmp(dvfs_domain_name, "CP_CPU")) {
		mld->cp_table.num_of_table = dvfs_domain->num_of_level;
		mld->total_freq_table_count++;
		for (i = dvfs_domain->num_of_level - 1; i >= 0; i--) {
			mld->cp_table.freq[i] =
				dvfs_domain->list_level[counter++].level;
			mif_err("CP_LEV[%d] : %u\n", i + 1,
					mld->cp_table.freq[i]);
		}
	} else if (!strcmp(dvfs_domain_name, "CP")) {
		mld->modem_table.num_of_table = dvfs_domain->num_of_level;
		mld->total_freq_table_count++;
		for (i = dvfs_domain->num_of_level - 1; i >= 0; i--) {
			mld->modem_table.freq[i] =
				dvfs_domain->list_level[counter++].level;
			mif_err("MODEM_LEV[%d] : %u\n", i + 1,
					mld->modem_table.freq[i]);
		}
	}

	return 0;
}
#else
static int parse_ect(struct mem_link_device *mld, char *dvfs_domain_name)
{
	mif_err("ECT is not defined(%s)\n", __func__);

	mld->mif_table.num_of_table = 0;
	mld->cp_table.num_of_table = 0;
	mld->modem_table.num_of_table = 0;

	return 0;
}
#endif

static int shmem_rx_setup(struct link_device *ld)
{
	ld->rx_wq = alloc_workqueue(
			"mem_rx_work", WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!ld->rx_wq) {
		mif_err("%s: ERR! fail to create rx_wq\n", ld->name);
		return -ENOMEM;
	}

	return 0;
}

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
int request_pcie_msi_int(struct link_device *ld,
				struct platform_device *pdev)
{
	int ret, base_irq;
	struct mem_link_device *mld = to_mem_link_device(ld);
	struct device *dev = &pdev->dev;
	struct modem_ctl *mc = ld->mc;
	int irq_offset = 0;
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct pktproc_adaptor *ppa = &mld->pktproc;
	int i;
#endif

	mif_info("Request MSI interrupt.\n");

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (ppa->use_exclusive_irq)
		base_irq = s51xx_pcie_request_msi_int(mc->s51xx_pdev, 5);
	else
#endif
		base_irq = s51xx_pcie_request_msi_int(mc->s51xx_pdev, 4);

	mif_info("Request MSI interrupt. : BASE_IRQ(%d)\n", base_irq);
	mld->msi_irq_base = base_irq;

	if (base_irq <= 0) {
		mif_err("Can't get MSI IRQ!!!\n");
		return -EFAULT;
	}

	ret = devm_request_irq(dev, base_irq + irq_offset,
			shmem_irq_handler, IRQF_SHARED, "mif_cp2ap_msg", mld);
	if (ret) {
		mif_err("Can't request cp2ap_msg interrupt!!!\n");
		return -EIO;
	}
	irq_offset++;

	ret = devm_request_irq(dev, base_irq + irq_offset,
			shmem_tx_state_handler, IRQF_SHARED, "mif_cp2ap_status", mld);
	if (ret) {
		mif_err("Can't request cp2ap_status interrupt!!!\n");
		return -EIO;
	}
	irq_offset++;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (ppa->use_exclusive_irq) {
		for (i = 0; i < ppa->num_queue; i++) {
			struct pktproc_queue *q = ppa->q[i];

			q->irq = mld->msi_irq_base + irq_offset + q->irq_idx;
			ret = devm_request_irq(dev, q->irq, q->irq_handler, IRQF_SHARED,
					"pktproc", q);
			if (ret) {
				mif_err("devm_request_irq() for pktproc%d error %d\n", i, ret);
				return -EIO;
			}
			irq_offset++;
		}
	}
#endif

	mld->msi_irq_base_enabled = 1;
	mld->s51xx_pdev = mc->s51xx_pdev;

	return base_irq;
}

static int shmem_register_pcie(struct link_device *ld)
{
	struct modem_ctl *mc = ld->mc;
	struct platform_device *pdev = to_platform_device(mc->dev);
	static int is_registered;
	struct mem_link_device *mld = to_mem_link_device(ld);

#if IS_ENABLED(CONFIG_GS_S2MPU)
	u32 cp_num;
	u32 shmem_idx;
	int ret;
	struct device_node *s2mpu_dn;
#endif
	mif_err("CP EP driver initialization start.\n");

#if IS_ENABLED(CONFIG_GS_S2MPU)

	s2mpu_dn = of_parse_phandle(mc->dev->of_node, "s2mpu", 0);
	if (!s2mpu_dn) {
		mif_err("Failed to find s2mpu from device tree\n");
		return -EINVAL;
	}

	mc->s2mpu = s2mpu_fwnode_to_info(&s2mpu_dn->fwnode);
	if (!mc->s2mpu) {
		mif_err("Failed to get S2MPU\n");
		return -EPROBE_DEFER;
	}

	cp_num = ld->mdm_data->cp_num;

	for (shmem_idx = 0 ; shmem_idx < MAX_CP_SHMEM ; shmem_idx++) {
		if (shmem_idx == SHMEM_MSI)
			continue;

		if (cp_shmem_get_base(cp_num, shmem_idx)) {
			ret =  s2mpu_open(mc->s2mpu,
					  cp_shmem_get_base(cp_num, shmem_idx),
					  cp_shmem_get_size(cp_num, shmem_idx),
					  DMA_BIDIRECTIONAL);
			if (ret) {
				mif_err("S2MPU open failed error=%d\n", ret);
				return -EINVAL;
			}
		}
	}

	/* Also setup AoC window for voice calls */
	ret =  s2mpu_open(mc->s2mpu,
			  AOC_PCIE_WINDOW_START, AOC_PCIE_WINDOW_SIZE,
			  DMA_BIDIRECTIONAL);

	if (ret) {
		mif_err("S2MPU AoC open failed error=%d\n", ret);
		return -EINVAL;
	}

#endif

	msleep(200);

	s5100_poweron_pcie(mc);

	if (is_registered == 0) {
		/* initialize the pci_dev for modem_ctl */
		mif_err("s51xx_pcie_init start\n");
		s51xx_pcie_init(mc);
		if (mc->s51xx_pdev == NULL) {
			mif_err("s51xx_pdev is NULL. Check if CP wake up is done.\n");
			return -EINVAL;
		}

		/* debug: check MSI 32bit or 64bit - should be set as 32bit before this point*/
		// debug: pci_read_config_dword(s51xx_pcie.s51xx_pdev, 0x50, &msi_val);
		// debug: mif_err("MSI Control Reg : 0x%x\n", msi_val);

		request_pcie_msi_int(ld, pdev);
		first_save_s51xx_status(mc->s51xx_pdev);

		is_registered = 1;
	} else {
		if (mc->phone_state == STATE_CRASH_RESET) {
			print_msi_register(mc->s51xx_pdev);
			enable_irq(mld->msi_irq_base);
		}
	}

	print_msi_register(mc->s51xx_pdev);
	mc->pcie_registered = true;

	mif_err("CP EP driver initialization end.\n");

	return 0;
}
#endif

/* sysfs */
static ssize_t tx_period_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;

	modem = (struct modem_data *)dev->platform_data;
	return scnprintf(buf, PAGE_SIZE, "%d\n", modem->mld->tx_period_ms);
}

static ssize_t tx_period_ms_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	struct modem_data *modem;

	modem = (struct modem_data *)dev->platform_data;

	ret = kstrtouint(buf, 0, &modem->mld->tx_period_ms);
	if (ret)
		return -EINVAL;

	ret = count;
	return ret;
}

static ssize_t info_region_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;
	struct mem_link_device *mld;
	ssize_t count = 0;

	modem = (struct modem_data *)dev->platform_data;
	mld = modem->mld;

	if (modem->offset_ap_version)
		count += scnprintf(&buf[count], PAGE_SIZE - count, "ap_version:0x%08X\n",
			ioread32(mld->ap_version));

	if (modem->offset_cp_version)
		count += scnprintf(&buf[count], PAGE_SIZE - count, "cp_version:0x%08X\n",
			ioread32(mld->cp_version));

	if (modem->offset_cmsg_offset)
		count += scnprintf(&buf[count], PAGE_SIZE - count, "cmsg_offset:0x%08X\n",
			ioread32(mld->cmsg_offset));

	if (modem->offset_srinfo_offset)
		count += scnprintf(&buf[count], PAGE_SIZE - count, "srinfo_offset:0x%08X\n",
			ioread32(mld->srinfo_offset));

	if (modem->offset_clk_table_offset)
		count += scnprintf(&buf[count], PAGE_SIZE - count, "clk_table_offset:0x%08X\n",
			ioread32(mld->clk_table_offset));

	if (modem->offset_buff_desc_offset)
		count += scnprintf(&buf[count], PAGE_SIZE - count, "buff_desc_offset:0x%08X\n",
			ioread32(mld->buff_desc_offset));

	if (modem->offset_capability_offset) {
		count += scnprintf(&buf[count], PAGE_SIZE - count,
				"capability_offset:0x%08X\n",
				ioread32(mld->capability_offset));
		count += scnprintf(&buf[count], PAGE_SIZE - count,
				"ap_capability_0_offset:0x%08X\n",
				ioread32(mld->ap_capability_0_offset));
		count += scnprintf(&buf[count], PAGE_SIZE - count,
				"ap_capability_1_offset:0x%08X\n",
				ioread32(mld->ap_capability_1_offset));
		count += scnprintf(&buf[count], PAGE_SIZE - count,
				"cp_capability_0_offset:0x%08X\n",
				ioread32(mld->cp_capability_0_offset));
		count += scnprintf(&buf[count], PAGE_SIZE - count,
				"cp_capability_1_offset:0x%08X\n",
				ioread32(mld->cp_capability_1_offset));
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count, "ap2cp_msg:0x%08X\n",
		get_ctrl_msg(&mld->ap2cp_msg));
	count += scnprintf(&buf[count], PAGE_SIZE - count, "cp2ap_msg:0x%08X\n",
		get_ctrl_msg(&mld->cp2ap_msg));
	count += scnprintf(&buf[count], PAGE_SIZE - count, "ap2cp_united_status:0x%08X\n",
		get_ctrl_msg(&mld->ap2cp_united_status));
	count += scnprintf(&buf[count], PAGE_SIZE - count, "cp2ap_united_status:0x%08X\n",
		get_ctrl_msg(&mld->cp2ap_united_status));
	count += scnprintf(&buf[count], PAGE_SIZE - count, "ap2cp_kerneltime:0x%08X\n",
		get_ctrl_msg(&mld->ap2cp_kerneltime));
	count += scnprintf(&buf[count], PAGE_SIZE - count, "ap2cp_kerneltime_sec:0x%08X\n",
		get_ctrl_msg(&mld->ap2cp_kerneltime_sec));
	count += scnprintf(&buf[count], PAGE_SIZE - count, "ap2cp_kerneltime_usec:0x%08X\n",
		get_ctrl_msg(&mld->ap2cp_kerneltime_usec));

	return count;
}

static DEVICE_ATTR_RW(tx_period_ms);
static DEVICE_ATTR_RO(info_region);

static struct attribute *link_device_attrs[] = {
	&dev_attr_tx_period_ms.attr,
	&dev_attr_info_region.attr,
	NULL,
};

static const struct attribute_group link_device_group = {
	.attrs = link_device_attrs,
};

/* sysfs for napi */
static ssize_t rx_napi_list_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;
	struct napi_struct *n;
	struct net_device *netdev;
	ssize_t count = 0;

	modem = (struct modem_data *)dev->platform_data;
	netdev = &modem->mld->dummy_net;

	count += scnprintf(&buf[count], PAGE_SIZE - count, "[%s's napi_list]\n",
		netdev_name(netdev));
	list_for_each_entry(n, &netdev->napi_list, dev_list)
		count += scnprintf(&buf[count], PAGE_SIZE - count,
			"state: %s(%ld), weight: %d, poll: 0x%pK\n",
			test_bit(NAPI_STATE_SCHED, &n->state) ?
			"NAPI_STATE_SCHED" : "NAPI_STATE_COMPLETE",
			n->state, n->weight, (void *)n->poll);

	return count;
}

static ssize_t rx_int_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;

	modem = (struct modem_data *)dev->platform_data;
	return scnprintf(buf, PAGE_SIZE, "%d\n", modem->mld->rx_int_enable);
}

static ssize_t rx_int_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;

	modem = (struct modem_data *)dev->platform_data;
	return scnprintf(buf, PAGE_SIZE, "%d\n", modem->mld->rx_int_count);
}

static ssize_t rx_int_count_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct modem_data *modem;
	unsigned int val = 0;
	int ret;

	modem = (struct modem_data *)dev->platform_data;
	ret = kstrtouint(buf, 0, &val);

	if (val == 0)
		modem->mld->rx_int_count = 0;
	return count;
}

static ssize_t rx_poll_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;
	struct mem_link_device *mld;

	modem = (struct modem_data *)dev->platform_data;
	mld = modem->mld;

	return scnprintf(buf, PAGE_SIZE, "%s: %d\n", netdev_name(&mld->dummy_net),
		mld->rx_poll_count);
}

static ssize_t rx_poll_count_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct modem_data *modem;
	struct mem_link_device *mld;

	modem = (struct modem_data *)dev->platform_data;
	mld = modem->mld;

	mld->rx_poll_count = 0;
	return count;
}

static ssize_t rx_int_disabled_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_data *modem;

	modem = (struct modem_data *)dev->platform_data;
	return scnprintf(buf, PAGE_SIZE, "%lld\n", modem->mld->rx_int_disabled_time);
}

static ssize_t rx_int_disabled_time_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct modem_data *modem;
	unsigned int val = 0;
	int ret;

	modem = (struct modem_data *)dev->platform_data;
	ret = kstrtouint(buf, 0, &val);

	if (val == 0)
		modem->mld->rx_int_disabled_time = 0;
	return count;
}

static DEVICE_ATTR_RO(rx_napi_list);
static DEVICE_ATTR_RO(rx_int_enable);
static DEVICE_ATTR_RW(rx_int_count);
static DEVICE_ATTR_RW(rx_poll_count);
static DEVICE_ATTR_RW(rx_int_disabled_time);

static struct attribute *napi_attrs[] = {
	&dev_attr_rx_napi_list.attr,
	&dev_attr_rx_int_enable.attr,
	&dev_attr_rx_int_count.attr,
	&dev_attr_rx_poll_count.attr,
	&dev_attr_rx_int_disabled_time.attr,
	NULL,
};

static const struct attribute_group napi_group = {
	.attrs = napi_attrs,
	.name = "napi",
};

#if IS_ENABLED(CONFIG_CP_PKTPROC)
static u32 p_pktproc[3];
static u32 c_pktproc[3];

static void pktproc_print(struct mem_link_device *mld)
{
	struct pktproc_adaptor *ppa = &mld->pktproc;
	int i;
	struct pktproc_queue *q;

	for (i = 0; i < ppa->num_queue; i++) {
		q = ppa->q[i];

		c_pktproc[0] = *q->fore_ptr;
		c_pktproc[1] = *q->rear_ptr;
		c_pktproc[2] = q->done_ptr;

		if (memcmp(p_pktproc, c_pktproc, sizeof(u32)*3)) {
			mif_err("Queue:%d fore:%d rear:%d done:%d\n",
				i, c_pktproc[0], c_pktproc[1], c_pktproc[2]);
			memcpy(p_pktproc, c_pktproc, sizeof(u32)*3);
		}
	}
}
#endif

#define BUFF_SIZE 256
static u32 p_rwpointer[4];
static u32 c_rwpointer[4];

static enum hrtimer_restart sbd_print(struct hrtimer *timer)
{
	struct mem_link_device *mld = container_of(timer, struct mem_link_device, sbd_print_timer);
	struct sbd_link_device *sl = &mld->sbd_link_dev;
	u16 id;
	struct sbd_ring_buffer *rb[ULDL];
	struct io_device *iod;
	char buf[BUFF_SIZE] = { 0, };
	int len = 0;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	pktproc_print(mld);
#endif

	if (likely(sbd_active(sl))) {
		id = sbd_ch2id(sl, QOS_HIPRIO);
		rb[TX] = &sl->ipc_dev[id].rb[TX];
		rb[RX] = &sl->ipc_dev[id].rb[RX];

		c_rwpointer[0] = *(u32 *)rb[TX]->rp;
		c_rwpointer[1] = *(u32 *)rb[TX]->wp;
		c_rwpointer[2] = *(u32 *)rb[RX]->rp;
		c_rwpointer[3] = *(u32 *)rb[RX]->wp;

		if (memcmp(p_rwpointer, c_rwpointer, sizeof(u32)*4)) {
			mif_err("TX %04d/%04d %04d/%04d RX %04d/%04d %04d/%04d\n",
				c_rwpointer[0] & 0xFFFF, c_rwpointer[1] & 0xFFFF,
				c_rwpointer[0] >> 16, c_rwpointer[1] >> 16,
				c_rwpointer[2] & 0xFFFF, c_rwpointer[3] & 0xFFFF,
				c_rwpointer[2] >> 16, c_rwpointer[3] >> 16);
			memcpy(p_rwpointer, c_rwpointer, sizeof(u32)*4);

			spin_lock(&rb[TX]->iod->msd->active_list_lock);
			list_for_each_entry(iod, &rb[TX]->iod->msd->activated_ndev_list,
					node_ndev) {
				len += snprintf(buf + len, BUFF_SIZE - len, "%s: %lu/%lu ",
						iod->name, iod->ndev->stats.tx_packets,
						iod->ndev->stats.rx_packets);
			}
			spin_unlock(&rb[TX]->iod->msd->active_list_lock);

			mif_err("%s\n", buf);
		}
	}

	hrtimer_forward_now(timer, ms_to_ktime(1000));

	return HRTIMER_RESTART;
}

struct link_device *create_link_device(struct platform_device *pdev, u32 link_type)
{
	struct modem_data *modem;
	struct mem_link_device *mld;
	struct link_device *ld;
	int err;
	u32 cp_num;
	struct device_node *np_acpm = NULL;
	u32 acpm_addr;
	u8 __iomem *cmsg_base;

	mif_err("+++\n");

	/**
	 * Get the modem (platform) data
	 */
	modem = (struct modem_data *)pdev->dev.platform_data;
	if (!modem) {
		mif_err("ERR! modem == NULL\n");
		return NULL;
	}

	if (!modem->mbx) {
		mif_err("%s: ERR! mbx == NULL\n", modem->link_name);
		return NULL;
	}

	if (modem->ipc_version < SIPC_VER_50) {
		mif_err("%s<->%s: ERR! IPC version %d < SIPC_VER_50\n",
			modem->link_name, modem->name, modem->ipc_version);
		return NULL;
	}

	mif_err("MODEM:%s LINK:%s\n", modem->name, modem->link_name);

	/*
	 * Alloc an instance of mem_link_device structure
	 */
	mld = kzalloc(sizeof(struct mem_link_device), GFP_KERNEL);
	if (!mld) {
		mif_err("%s<->%s: ERR! mld kzalloc fail\n",
			modem->link_name, modem->name);
		return NULL;
	}

	/*
	 * Retrieve modem-specific attributes value
	 */
	mld->attrs = modem->link_attrs;
	mif_info("link_attrs:0x%08x\n", mld->attrs);

	/*====================================================================
	 *	Initialize "memory snapshot buffer (MSB)" framework
	 *====================================================================
	 */
	if (msb_init() < 0) {
		mif_err("%s<->%s: ERR! msb_init() fail\n",
			modem->link_name, modem->name);
		goto error;
	}

	/*====================================================================
	 *	Set attributes as a "link_device"
	 *====================================================================
	 */
	ld = &mld->link_dev;

	ld->name = modem->link_name;

	if (mld->attrs & LINK_ATTR_SBD_IPC) {
		mif_err("%s<->%s: LINK_ATTR_SBD_IPC\n", ld->name, modem->name);
		ld->sbd_ipc = true;
	}

	if (mld->attrs & LINK_ATTR_IPC_ALIGNED) {
		mif_err("%s<->%s: LINK_ATTR_IPC_ALIGNED\n",
			ld->name, modem->name);
		ld->aligned = true;
	}

	ld->ipc_version = modem->ipc_version;
	ld->interrupt_types = modem->interrupt_types;

	ld->mdm_data = modem;

	ld->dev = &pdev->dev;

	/*
	 * Set up link device methods
	 */
	ld->ioctl = shmem_ioctl;

	if (link_type == LINKDEV_SHMEM)
		ld->init_comm = shmem_init_comm;
	ld->terminate_comm = shmem_terminate_comm;
	ld->send = shmem_send;
	ld->reset_zerocopy = NULL;

	ld->link_prepare_normal_boot = link_prepare_normal_boot;
	if (mld->attrs & LINK_ATTR_MEM_BOOT) {
		if (mld->attrs & LINK_ATTR_XMIT_BTDLR) {
			if (mld->attrs & LINK_ATTR_XMIT_BTDLR_SPI) {
				u32 bus_num = 0;

				err = of_property_read_u32(pdev->dev.of_node, "cpboot_spi_bus_num",
						&bus_num);
				if (err) {
					mif_err("cpboot_spi_bus_num error:%d\n", err);
					goto error;
				}
				mld->boot_spi = cpboot_spi_get_device(bus_num);
				if (!mld->boot_spi) {
					mif_err("boot_spi is null\n");
					goto error;
				}
				ld->load_cp_image = cpboot_spi_load_cp_image;
			} else {
				ld->load_cp_image = link_load_cp_image;
			}
		}
		ld->link_start_normal_boot = link_start_normal_boot;
		if (link_type == LINKDEV_SHMEM)
			ld->security_req = shmem_security_request;
	}

	ld->link_trigger_cp_crash = link_trigger_cp_crash;

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	if (link_type == LINKDEV_PCIE)
		ld->register_pcie = shmem_register_pcie;
#endif

	if (mld->attrs & LINK_ATTR_MEM_DUMP)
		ld->link_start_dump_boot = link_start_dump_boot;

	ld->close_tx = shmem_close_tx;
	ld->get_cp_crash_reason = get_cp_crash_reason;

	ld->gro_flush = gro_flush_timer;
	ld->update_flush_time = update_flush_time;

	ld->protocol = modem->protocol;
	ld->capability_check = modem->capability_check;

	switch (ld->protocol) {
	case PROTOCOL_SIPC:
		ld->chid_fmt_0 = SIPC5_CH_ID_FMT_0;
		ld->chid_rfs_0 = SIPC5_CH_ID_RFS_0;
		ld->magic_boot = MEM_BOOT_MAGIC;
		ld->magic_crash = MEM_CRASH_MAGIC;
		ld->magic_dump = MEM_DUMP_MAGIC;
		ld->magic_ipc = MEM_IPC_MAGIC;

		ld->is_start_valid = sipc5_start_valid;
		ld->is_padding_exist = sipc5_padding_exist;
		ld->is_multi_frame = sipc5_multi_frame;
		ld->has_ext_len = sipc5_ext_len;
		ld->get_ch = sipc5_get_ch;
		ld->get_ctrl = sipc5_get_ctrl;
		ld->calc_padding_size = sipc5_calc_padding_size;
		ld->get_hdr_len = sipc5_get_hdr_len;
		ld->get_frame_len = sipc5_get_frame_len;
		ld->get_total_len = sipc5_get_total_len;
		ld->is_fmt_ch = sipc5_fmt_ch;
		ld->is_ps_ch = sipc_ps_ch;
		ld->is_rfs_ch = sipc5_rfs_ch;
		ld->is_boot_ch = sipc5_boot_ch;
		ld->is_dump_ch = sipc5_dump_ch;
		ld->is_bootdump_ch = sipc5_bootdump_ch;
		ld->is_ipc_ch = sipc5_ipc_ch;
		ld->is_csd_ch = sipc_csd_ch;
		ld->is_log_ch = sipc_log_ch;
		ld->is_router_ch = sipc_router_ch;
		ld->is_misc_ch = sipc_misc_ch;
		ld->is_embms_ch = NULL;
		ld->is_uts_ch = NULL;
		ld->is_wfs0_ch = NULL;
		ld->is_wfs1_ch = NULL;
		break;
	case PROTOCOL_SIT:
		ld->chid_fmt_0 = EXYNOS_CH_ID_FMT_0;
		ld->chid_rfs_0 = EXYNOS_CH_ID_RFS_0;
		ld->magic_boot = SHM_BOOT_MAGIC;
		ld->magic_crash = SHM_DUMP_MAGIC;
		ld->magic_dump = SHM_DUMP_MAGIC;
		ld->magic_ipc = SHM_IPC_MAGIC;

		ld->is_start_valid = exynos_start_valid;
		ld->is_padding_exist = exynos_padding_exist;
		ld->is_multi_frame = exynos_multi_frame;
		ld->has_ext_len = exynos_ext_len;
		ld->get_ch = exynos_get_ch;
		ld->get_ctrl = exynos_get_ctrl;
		ld->calc_padding_size = exynos_calc_padding_size;
		ld->get_hdr_len = exynos_get_hdr_len;
		ld->get_frame_len = exynos_get_frame_len;
		ld->get_total_len = exynos_get_total_len;
		ld->is_fmt_ch = exynos_fmt_ch;
		ld->is_ps_ch = exynos_ps_ch;
		ld->is_rfs_ch = exynos_rfs_ch;
		ld->is_boot_ch = exynos_boot_ch;
		ld->is_dump_ch = exynos_dump_ch;
		ld->is_bootdump_ch = exynos_bootdump_ch;
		ld->is_ipc_ch = exynos_ipc_ch;
		ld->is_csd_ch = exynos_rcs_ch;
		ld->is_log_ch = exynos_log_ch;
		ld->is_router_ch = exynos_router_ch;
		ld->is_embms_ch = exynos_embms_ch;
		ld->is_uts_ch = exynos_uts_ch;
		ld->is_wfs0_ch = exynos_wfs0_ch;
		ld->is_wfs1_ch = exynos_wfs1_ch;
		ld->is_oem_ch = exynos_oem_ch;
		break;
	default:
		mif_err("protocol error %d\n", ld->protocol);
		goto error;
	}

	ld->enable_rx_int = shmem_enable_rx_int;
	ld->disable_rx_int = shmem_disable_rx_int;

	ld->start_timers = shmem_start_timers;
	ld->stop_timers = shmem_stop_timers;

	ld->handover_block_info = update_handover_block_info;

	init_dummy_netdev(&mld->dummy_net);
	netif_napi_add(&mld->dummy_net, &mld->mld_napi, mld_rx_int_poll, NAPI_POLL_WEIGHT);
	napi_enable(&mld->mld_napi);
	atomic_set(&mld->stop_napi_poll, 0);

	INIT_LIST_HEAD(&ld->list);

	spin_lock_init(&ld->netif_lock);
	atomic_set(&ld->netif_stopped, 0);
	ld->tx_flowctrl_mask = 0;

	if (shmem_rx_setup(ld) < 0)
		goto error;

	if (mld->attrs & LINK_ATTR_DPRAM_MAGIC) {
		mif_err("%s<->%s: LINK_ATTR_DPRAM_MAGIC\n",
			ld->name, modem->name);
		mld->dpram_magic = true;
	}
#if IS_ENABLED(CONFIG_LINK_CONTROL_MSG_IOSM)
	mld->iosm = true;
	mld->cmd_handler = iosm_event_bh;
	INIT_WORK(&mld->iosm_w, iosm_event_work);
#else
	mld->cmd_handler = shmem_cmd_handler;
#endif

	spin_lock_init(&mld->state_lock);
	mld->state = LINK_STATE_OFFLINE;

	/*
	 * Initialize variables for TX & RX
	 */
	msb_queue_head_init(&mld->msb_rxq);
	msb_queue_head_init(&mld->msb_log);

	hrtimer_init(&mld->tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mld->tx_timer.function = tx_timer_func;

	INIT_WORK(&mld->page_reclaim_work, bootdump_oom_handler_work);

	/*
	 * Initialize variables for CP booting and crash dump
	 */
	INIT_DELAYED_WORK(&mld->bootdump_rx_dwork, bootdump_rx_work);

	/*
	 * Link local functions to the corresponding function pointers that are
	 * mandatory for all memory-type link devices
	 */
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	if (link_type == LINKDEV_PCIE) {
		mld->recv_cp2ap_irq = pcie_recv_cp2ap_irq;
		mld->send_ap2cp_irq = pcie_send_ap2cp_irq;
		mld->recv_cp2ap_status = pcie_recv_cp2ap_status;
	}
#endif
#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM)
	if (link_type == LINKDEV_SHMEM) {
		mld->recv_cp2ap_irq = shmem_recv_cp2ap_irq;
		mld->send_ap2cp_irq = shmem_send_ap2cp_irq;
		mld->recv_cp2ap_status = shmem_recv_cp2ap_status;
	}
#endif

	/*
	 * Link local functions to the corresponding function pointers that are
	 * optional for some memory-type link devices
	 */
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	if (link_type == LINKDEV_PCIE)
		mld->read_ap2cp_irq = pcie_read_ap2cp_irq;
#endif
#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM)
	if (link_type == LINKDEV_SHMEM)
		mld->read_ap2cp_irq = shmem_read_ap2cp_irq;
#endif

	/*
	 * Initialize SHMEM maps for BOOT (physical map -> logical map)
	 */
	cp_num = ld->mdm_data->cp_num;
#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM)
	if (link_type == LINKDEV_SHMEM) {
		mld->boot_size = cp_shmem_get_size(cp_num, SHMEM_CP) +
			cp_shmem_get_size(cp_num, SHMEM_VSS);
		mld->boot_base = NULL;
		mif_err("boot_base=NULL, boot_size=%lu\n",
			(unsigned long)mld->boot_size);
	}
#endif

	/*
	 * Initialize SHMEM maps for IPC (physical map -> logical map)
	 */
	mld->size = cp_shmem_get_size(cp_num, SHMEM_IPC);
#if IS_ENABLED(CONFIG_CACHED_LEGACY_RAW_RX_BUFFER)
	mld->base = cp_shmem_get_nc_region(cp_shmem_get_base(cp_num, SHMEM_IPC), SZ_2M);
#else
	mld->base = cp_shmem_get_region(cp_num, SHMEM_IPC);
#endif

#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	mld->hiprio_base = cp_shmem_get_nc_region(cp_shmem_get_base(cp_num, SHMEM_IPC)
			+ modem->legacy_raw_qos_buffer_offset, modem->legacy_raw_qos_txq_size
			+ modem->legacy_raw_qos_rxq_size);
#endif
	if (!mld->base) {
		mif_err("Failed to vmap ipc_region\n");
		goto error;
	}
	mif_err("ipc_base=%pK, ipc_size=%lu\n",
		mld->base, (unsigned long)mld->size);

	switch (link_type) {
	case LINKDEV_SHMEM:
		/*
		 * Initialize SHMEM maps for VSS (physical map -> logical map)
		 */
		mld->vss_base = cp_shmem_get_region(cp_num, SHMEM_VSS);
		if (!mld->vss_base) {
			mif_err("Failed to vmap vss_region\n");
			goto error;
		}
		mif_err("vss_base=%pK\n", mld->vss_base);

		/*
		 * Initialize memory maps for ACPM (physical map -> logical map)
		 */
		np_acpm = of_find_node_by_name(NULL, "acpm_ipc");
		if (!np_acpm)
			break;

		of_property_read_u32(np_acpm, "dump-size", &mld->acpm_size);
		of_property_read_u32(np_acpm, "dump-base", &acpm_addr);
		mld->acpm_base = cp_shmem_get_nc_region(acpm_addr, mld->acpm_size);
		if (!mld->acpm_base) {
			mif_err("Failed to vmap acpm_region\n");
			goto error;
		}
		mif_err("acpm_base=%pK acpm_size:0x%X\n", mld->acpm_base,
				mld->acpm_size);
		break;
	default:
		break;
	}

	ld->link_type = link_type;
	create_legacy_link_device(mld);

	if (ld->sbd_ipc) {
		hrtimer_init(&mld->sbd_tx_timer,
				CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		mld->sbd_tx_timer.function = sbd_tx_timer_func;

		hrtimer_init(&mld->sbd_print_timer,
				CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		mld->sbd_print_timer.function = sbd_print;

		err = create_sbd_link_device(ld,
				&mld->sbd_link_dev, mld->base, mld->size);
		if (err < 0)
			goto error;
	}

	/*
	 * Info region
	 */
	if (modem->offset_ap_version)
		mld->ap_version = (u32 __iomem *)(mld->base + modem->offset_ap_version);
	if (modem->offset_cp_version)
		mld->cp_version = (u32 __iomem *)(mld->base + modem->offset_cp_version);
	if (modem->offset_cmsg_offset) {
		mld->cmsg_offset = (u32 __iomem *)(mld->base + modem->offset_cmsg_offset);
		cmsg_base = mld->base + modem->cmsg_offset;
		iowrite32(modem->cmsg_offset, mld->cmsg_offset);
	} else {
		cmsg_base = mld->base;
	}

	if (modem->offset_srinfo_offset) {
		mld->srinfo_offset = (u32 __iomem *)(mld->base + modem->offset_srinfo_offset);
		iowrite32(modem->srinfo_offset, mld->srinfo_offset);
	}
	if (modem->offset_clk_table_offset) {
		mld->clk_table_offset = (u32 __iomem *)(mld->base + modem->offset_clk_table_offset);
		iowrite32(modem->clk_table_offset, mld->clk_table_offset);
	}
	if (modem->offset_buff_desc_offset) {
		mld->buff_desc_offset = (u32 __iomem *)(mld->base + modem->offset_buff_desc_offset);
		iowrite32(modem->buff_desc_offset, mld->buff_desc_offset);
	}

	mld->srinfo_base = (u32 __iomem *)(mld->base + modem->srinfo_offset);
	mld->srinfo_size = modem->srinfo_size;
	mld->clk_table = (u32 __iomem *)(mld->base + modem->clk_table_offset);

	if (ld->capability_check) {
		/* AP/CP capability */
		mld->capability_offset =
			(u32 __iomem *)(mld->base + modem->offset_capability_offset);
		iowrite32(modem->capability_offset, mld->capability_offset);

		mld->ap_capability_0_offset =
			(u32 __iomem *)(mld->base + modem->capability_offset);
		mld->cp_capability_0_offset =
			(u32 __iomem *)(mld->base + modem->capability_offset + 0x4);
		mld->ap_capability_1_offset =
			(u32 __iomem *)(mld->base + modem->capability_offset + 0x8);
		mld->cp_capability_1_offset =
			(u32 __iomem *)(mld->base + modem->capability_offset + 0xC);

		/* Initial value */
		iowrite32(0, mld->ap_capability_0_offset);
		iowrite32(0, mld->cp_capability_0_offset);
		iowrite32(0, mld->ap_capability_1_offset);
		iowrite32(0, mld->cp_capability_1_offset);
	}

	construct_ctrl_msg(&mld->cp2ap_msg, modem->cp2ap_msg, cmsg_base);
	construct_ctrl_msg(&mld->ap2cp_msg, modem->ap2cp_msg, cmsg_base);
	construct_ctrl_msg(&mld->cp2ap_united_status, modem->cp2ap_united_status, cmsg_base);
	construct_ctrl_msg(&mld->ap2cp_united_status, modem->ap2cp_united_status, cmsg_base);
	construct_ctrl_msg(&mld->ap2cp_kerneltime, modem->ap2cp_kerneltime, cmsg_base);
	construct_ctrl_msg(&mld->ap2cp_kerneltime_sec, modem->ap2cp_kerneltime_sec, cmsg_base);
	construct_ctrl_msg(&mld->ap2cp_kerneltime_usec, modem->ap2cp_kerneltime_usec, cmsg_base);

	/*
	 * Retrieve SHMEM MBOX#, IRQ#, etc.
	 */
	mld->irq_cp2ap_msg = modem->mbx->irq_cp2ap_msg;
	mld->int_ap2cp_msg = modem->mbx->int_ap2cp_msg;

	mld->sbi_cp_status_mask = modem->sbi_cp_status_mask;
	mld->sbi_cp_status_pos = modem->sbi_cp_status_pos;
	mld->sbi_cp2ap_wakelock_mask = modem->sbi_cp2ap_wakelock_mask;
	mld->sbi_cp2ap_wakelock_pos = modem->sbi_cp2ap_wakelock_pos;
	mld->sbi_cp_rat_mode_mask = modem->sbi_cp2ap_rat_mode_mask;
	mld->sbi_cp_rat_mode_pos = modem->sbi_cp2ap_rat_mode_pos;

	mld->ap_capability_0 = modem->ap_capability_0;
	mld->ap_capability_1 = modem->ap_capability_1;

	/*
	 * Register interrupt handlers
	 */
#if IS_ENABLED(CONFIG_MCU_IPC)
	if (ld->interrupt_types == INTERRUPT_MAILBOX) {
		err = cp_mbox_register_handler(CP_MBOX_IRQ_IDX_0, mld->irq_cp2ap_msg,
				shmem_irq_handler, mld);
		if (err) {
			mif_err("%s: ERR! cp_mbox_register_handler(MBOX_IRQ_IDX_0, %u) fail (%d)\n",
				ld->name, mld->irq_cp2ap_msg, err);
			goto error;
		}
	}
#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	/* Set doorbell interrupt value for separating interrupts */
	mld->intval_ap2cp_msg = modem->mbx->int_ap2cp_msg + DOORBELL_INT_ADD;
	mld->intval_ap2cp_status = modem->mbx->int_ap2cp_status
				+ DOORBELL_INT_ADD;
	mld->intval_ap2cp_active = modem->mbx->int_ap2cp_active
				+ DOORBELL_INT_ADD;
#endif

	/**
	 * Retrieve SHMEM MBOX# and IRQ# for wakelock
	 */
	mld->irq_cp2ap_wakelock = modem->mbx->irq_cp2ap_wakelock;

	mld->ws = cpif_wake_lock_register(ld->dev, ld->name);
	if (mld->ws == NULL) {
		mif_err("%s: wakeup_source_register fail\n", ld->name);
		goto error;
	}

#if IS_ENABLED(CONFIG_MCU_IPC)
	if (ld->interrupt_types == INTERRUPT_MAILBOX) {
		err = cp_mbox_register_handler(CP_MBOX_IRQ_IDX_0, mld->irq_cp2ap_wakelock,
				shmem_cp2ap_wakelock_handler, mld);
		if (err) {
			mif_err("%s: ERR! cp_mbox_register_handler(MBOX_IRQ_IDX_0, %u) fail (%d)\n",
				ld->name, mld->irq_cp2ap_wakelock, err);
			goto error;
		}
	}
#endif

	/**
	 * Retrieve SHMEM MBOX# and IRQ# for RAT_MODE
	 */
#if IS_ENABLED(CONFIG_MCU_IPC) && IS_ENABLED(CONFIG_PCI_EXYNOS)
	mld->irq_cp2ap_rat_mode = modem->mbx->irq_cp2ap_rat_mode;

	err = cp_mbox_register_handler(CP_MBOX_IRQ_IDX_0, mld->irq_cp2ap_rat_mode,
			shmem_cp2ap_rat_mode_handler, mld);
	if (err) {
		mif_err("%s: ERR! cp_mbox_register_handler(CP_MBOX_IRQ_IDX_0, %u) fail (%d)\n",
			ld->name, mld->irq_cp2ap_rat_mode, err);
		goto error;
	}
#endif

	if (ld->link_type == LINKDEV_SHMEM) {
		err = of_property_read_u32(pdev->dev.of_node, "devfreq_use_dfs_max_freq",
				&mld->mif_table.use_dfs_max_freq);
		if (err) {
			mif_err("devfreq_use_dfs_max_freq error:%d\n", err);
			goto error;
		}

		if (mld->mif_table.use_dfs_max_freq) {
			err = of_property_read_u32(pdev->dev.of_node,
				"devfreq_cal_id_mif", &mld->mif_table.cal_id_mif);
			if (err) {
				mif_err("devfreq_cal_id_mif error:%d\n", err);
				goto error;
			}
		}

		/* Parsing devfreq, cpufreq table from ECT */
		mif_info("Parsing MIF frequency table...\n");
		err = parse_ect(mld, "MIF");
		if (err < 0)
			mif_err("Can't get MIF frequency table!!!!!\n");

		mif_info("Parsing CP frequency table...\n");
		err = parse_ect(mld, "CP_CPU");
		if (err < 0)
			mif_err("Can't get CP frequency table!!!!!\n");

		mif_info("Parsing MODEM frequency table...\n");
		err = parse_ect(mld, "CP");
		if (err < 0)
			mif_err("Can't get MODEM frequency table!!!!!\n");
	}

	/**
	 * For TX Flow-control command from CP
	 */
	construct_ctrl_msg(&mld->ap2cp_handover_block_info,
			modem->ap2cp_handover_block_info, cmsg_base);

	mld->irq_cp2ap_status = modem->mbx->irq_cp2ap_status;
	mld->tx_flowctrl_cmd = 0;

#if IS_ENABLED(CONFIG_MCU_IPC)
	if (ld->interrupt_types == INTERRUPT_MAILBOX) {
		err = cp_mbox_register_handler(CP_MBOX_IRQ_IDX_0, mld->irq_cp2ap_status,
					shmem_tx_state_handler, mld);
		if (err) {
			mif_err("%s: ERR! cp_mbox_register_handler(MBOX_IRQ_IDX_0, %u) fail (%d)\n",
				ld->name, mld->irq_cp2ap_status, err);
			goto error;
		}
	}
#endif

#if !IS_ENABLED(CONFIG_CP_SECURE_BOOT)
	mld->syscp_info = platform_get_resource(pdev, IORESOURCE_MEM, 0);
#endif

	/* Link mem_link_device to modem_data */
	modem->mld = mld;

	mld->tx_period_ms = TX_PERIOD_MS;

	mld->pass_skb_to_net = pass_skb_to_net;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	err = pktproc_create(pdev, mld, cp_shmem_get_base(cp_num, SHMEM_PKTPROC),
			cp_shmem_get_size(cp_num, SHMEM_PKTPROC));
	if (err < 0) {
		mif_err("pktproc_create() error %d\n", err);
		goto error;
	}
#endif

#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	err = pktproc_create_ul(pdev, mld, cp_shmem_get_base(cp_num, SHMEM_PKTPROC_UL),
			cp_shmem_get_size(cp_num, SHMEM_PKTPROC_UL));
	if (err < 0) {
		mif_err("pktproc_create_ul() error %d\n", err);
		goto error;
	}
	hrtimer_init(&mld->pktproc_tx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mld->pktproc_tx_timer.function = pktproc_tx_timer_func;
#endif

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
	err = tpmon_create(pdev, ld);
	if (err < 0) {
		mif_err("tpmon_create() error %d\n", err);
		goto error;
	}
#endif

	/* sysfs */
	if (sysfs_create_group(&pdev->dev.kobj, &link_device_group))
		mif_err("failed to create sysfs node related link_device\n");

	if (sysfs_create_group(&pdev->dev.kobj, &napi_group))
		mif_err("failed to create sysfs node related napi\n");

	mif_err("---\n");
	return ld;

error:
	//shm_release_regions();

	kfree(mld);
	mif_err("xxx\n");
	return NULL;
}
