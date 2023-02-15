// SPDX-License-Identifier: GPL-2.0
/*
 * Driver core for Exynos SoC onboard UARTs.
 *
 * Ben Dooks, Copyright (c) 2003-2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 */

/* Hote on 2410 error handling
 *
 * The s3c2410 manual has a love/hate affair with the contents of the
 * UERSTAT register in the UART blocks, and keeps marking some of the
 * error bits as reserved. Having checked with the s3c2410x01,
 * it copes with BREAKs properly, so I am happy to ignore the RESERVED
 * feature from the latter versions of the manual.
 *
 * If it becomes aparrent that latter versions of the 2410 remove these
 * bits, then action will have to be taken to differentiate the versions
 * and change the policy on BREAK
 *
 * BJD, 04-Nov-2004
 */

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_s3c.h>

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/of.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <asm/irq.h>

#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/consumer.h>
#include <misc/logbuffer.h>


/* Debug logging */
#define DATA_BYTES_PER_LINE     (16)

#include <soc/google/exynos-cpupm.h>

#define EXYNOS_UART_PORT_LPM			0x5

#define EXYNOS_SERIAL_CTRL_NUM			0x4
#define EXYNOS_SERIAL_BAUD_NUM			0x2

#define S5PV210_UMCON_RTSTRIG_SHIFT		(5)

struct exynos_uart_info {
	char			*name;
	unsigned int		type;
	unsigned int		fifosize;
	unsigned long		rx_fifomask;
	unsigned long		rx_fifoshift;
	unsigned long		rx_fifofull;
	unsigned long		tx_fifomask;
	unsigned long		tx_fifoshift;
	unsigned long		tx_fifofull;
	unsigned int		rts_trig_shift;
	unsigned int		def_clk_sel;
	unsigned long		num_clks;
	unsigned long		clksel_mask;
	unsigned long		clksel_shift;

	/* uart port features */
	unsigned int		has_divslot:1;
};

struct exynos_serial_drv_data {
	struct exynos_uart_info	*info;
	struct s3c2410_uartcfg		*def_cfg;
	unsigned int			fifosize[CONFIG_SERIAL_EXYNOS_UARTS];
};

struct exynos_uart_dma {
	unsigned int			rx_chan_id;
	unsigned int			tx_chan_id;

	struct dma_slave_config		rx_conf;
	struct dma_slave_config		tx_conf;

	struct dma_chan			*rx_chan;
	struct dma_chan			*tx_chan;

	dma_addr_t			rx_addr;
	dma_addr_t			tx_addr;

	dma_cookie_t			rx_cookie;
	dma_cookie_t			tx_cookie;

	char				*rx_buf;

	dma_addr_t			tx_transfer_addr;

	size_t				rx_size;
	size_t				tx_size;

	struct dma_async_tx_descriptor	*tx_desc;
	struct dma_async_tx_descriptor	*rx_desc;

	int				tx_bytes_requested;
	int				rx_bytes_requested;
};

struct uart_local_buf {
	unsigned char *buffer;
	unsigned int size;
	unsigned int index;
};

struct exynos_uart_port {
	struct list_head		node;
	unsigned char			rx_claimed;
	unsigned char			tx_claimed;
	unsigned char			rx_enabled;
	unsigned char			tx_enabled;
	unsigned long			baudclk_rate;
	unsigned int			min_dma_size;

	unsigned int			rx_irq;
	unsigned int			tx_irq;

	unsigned int			tx_in_progress;
	unsigned int			tx_mode;
	unsigned int			rx_mode;

	unsigned int			src_clk_rate;
	struct exynos_uart_info	*info;
	struct clk			*clk;
	struct clk			*separated_clk;
	struct clk			*baudclk;
	struct uart_port		port;
	struct exynos_serial_drv_data	*drv_data;

	u32				uart_irq_affinity;
	u32				use_default_irq;
	unsigned int			usi_v2;
	unsigned int			uart_panic_log;
	struct pinctrl_state	*uart_pinctrl_tx_dat;
	struct pinctrl_state	*uart_pinctrl_rts;
	struct pinctrl_state	*uart_pinctrl_default;
	struct pinctrl *pinctrl;
	unsigned int		rts_control;
	unsigned int		rts_trig_level;

	struct regmap			*usi_reg;
	unsigned int			usi_offset;

	/* reference to platform data */
	struct s3c2410_uartcfg		*cfg;

	struct exynos_uart_dma		*dma;

	struct platform_device		*pdev;

	unsigned int			in_band_wakeup;
	unsigned int dbg_mode;

	unsigned int dbg_uart_ch;
	unsigned int dbg_uart_baud;
	unsigned int dbg_word_len;
	unsigned int			uart_logging;
	struct uart_local_buf		uart_local_buf;
	struct logbuffer *log;
};

/* conversion functions */

#define exynos_dev_to_port(__dev) dev_get_drvdata(__dev)

/* register access controls */

#define portaddr(port, reg) ((port)->membase + (reg))
#define portaddrl(port, reg) \
	((unsigned long *)(unsigned long)((port)->membase + (reg)))

static unsigned int rd_reg(struct uart_port *port, int reg)
{
	switch (port->iotype) {
	case UPIO_MEM:
		return readb_relaxed(portaddr(port, reg));
	case UPIO_MEM32:
		return readl_relaxed(portaddr(port, reg));
	default:
		return 0;
	}
}

#define rd_regl(port, reg) (readl_relaxed(portaddr(port, reg)))

static void wr_reg(struct uart_port *port, int reg, int val)
{
	switch (port->iotype) {
	case UPIO_MEM:
		writeb_relaxed(val, portaddr(port, reg));
		break;
	case UPIO_MEM32:
		writel_relaxed(val, portaddr(port, reg));
		break;
	}
}

#define wr_regl(port, reg, val) writel_relaxed(val, portaddr(port, reg))

/* Byte-order aware bit setting/clearing functions. */

static inline void exynos_set_bit(struct uart_port *port, int idx,
				  unsigned int reg)
{
	unsigned long flags;
	u32 val;

	local_irq_save(flags);
	val = rd_regl(port, reg);
	val |= (1 << idx);
	wr_regl(port, reg, val);
	local_irq_restore(flags);
}

static inline void exynos_clear_bit(struct uart_port *port, int idx,
				    unsigned int reg)
{
	unsigned long flags;
	u32 val;

	local_irq_save(flags);
	val = rd_regl(port, reg);
	val &= ~(1 << idx);
	wr_regl(port, reg, val);
	local_irq_restore(flags);
}

/* UART name and device definitions */

#define EXYNOS_SERIAL_NAME	"ttySAC"
#define EXYNOS_SERIAL_MAJOR	204
#define EXYNOS_SERIAL_MINOR	64

#define EXYNOS_TX_PIO			1
#define EXYNOS_TX_DMA			2
#define EXYNOS_RX_PIO			1
#define EXYNOS_RX_DMA			2

/* Baudrate definition*/
#define MAX_BAUD	4000000
#define MIN_BAUD	0

#define DEFAULT_SOURCE_CLK	200000000

/* flag to ignore all characters coming in */
#define RXSTAT_DUMMY_READ (0x10000000)

static LIST_HEAD(drvdata_list);

#define UART_LOOPBACK_MODE	BIT(0)
#define UART_DBG_MODE		BIT(1)

#define RTS_PINCTRL		1
#define DEFAULT_PINCTRL		0

#define USI_CON			0xC4
#define USI_OPTION		0xC8

#define USI_RESET		0
#define USI_SET_RESET		BIT(0)
#define USI_HWACG_CLKREQ_ON	BIT(1)
#define USI_HWACG_CLKSTOP_ON	BIT(2)
/* Allocate 800KB of buffer for UART logging */
#define LOG_BUFFER_SIZE		0xC8000

#define USI_SW_CONF_MASK	(0x7 << 0)
#define USI_UART_SW_CONF	BIT(0)

#define RTS_CTRL_SUPPORT	(0x1 << 0)
#define RTS_CTRL_CHK_MASK	(0xF << 16)
#define RTS_CTRL_CHK_EN		(0x1 << 16)
#define RTS_CTRL_CHK_DIS	(0x0 << 16)
#define RTS_CTRL_ENABLE		(RTS_CTRL_CHK_EN | RTS_CTRL_SUPPORT)
#define RTS_CTRL_DISABLE	(RTS_CTRL_CHK_DIS | RTS_CTRL_SUPPORT)

struct exynos_uart_port *panic_port;

static int exynos_uart_panic_handler(struct notifier_block *nb,
				     unsigned long l, void *p)
{
	struct uart_port *port = &panic_port->port;

	dev_err(panic_port->port.dev, " Register dump\n"
		"ULCON	0x%08x	UCON	0x%08x	UFCON	0x%08x	UMCON	0x%08x\n"
		"UTRSTAT	0x%08x	UERSTAT	0x%08x	UFSTAT	0x%08x	UMSTAT	0x%08x\n"
		"UBRDIV	0x%08x	UFRACVAL	0x%08x	UINTP	0x%08x	UINTM	0x%08x\n"
		, readl(port->membase + S3C2410_ULCON)
		, readl(port->membase + S3C2410_UCON)
		, readl(port->membase + S3C2410_UFCON)
		, readl(port->membase + S3C2410_UMCON)
		, readl(port->membase + S3C2410_UTRSTAT)
		, readl(port->membase + S3C2410_UERSTAT)
		, readl(port->membase + S3C2410_UFSTAT)
		, readl(port->membase + S3C2410_UMSTAT)
		, readl(port->membase + S3C2410_UBRDIV)
		, readl(port->membase + S3C2443_DIVSLOT)
		, readl(port->membase + S3C64XX_UINTP)
		, readl(port->membase + S3C64XX_UINTM)
	);

	return 0;
}

static struct notifier_block exynos_uart_panic_block = {
	.notifier_call = exynos_uart_panic_handler,
};

static void uart_sfr_dump(struct exynos_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;

	dev_err(ourport->port.dev, " Register dump\n"
		"ULCON	0x%08x	UCON	0x%08x	UFCON	0x%08x	UMCON	0x%08x\n"
		"UTRSTAT	0x%08x	UERSTAT	0x%08x	UFSTAT	0x%08x	UMSTAT	0x%08x\n"
		"UBRDIV	0x%08x	UFRACVAL	0x%08x	UINTP	0x%08x	UINTM	0x%08x\n"
		, readl(port->membase + S3C2410_ULCON)
		, readl(port->membase + S3C2410_UCON)
		, readl(port->membase + S3C2410_UFCON)
		, readl(port->membase + S3C2410_UMCON)
		, readl(port->membase + S3C2410_UTRSTAT)
		, readl(port->membase + S3C2410_UERSTAT)
		, readl(port->membase + S3C2410_UFSTAT)
		, readl(port->membase + S3C2410_UMSTAT)
		, readl(port->membase + S3C2410_UBRDIV)
		, readl(port->membase + S3C2443_DIVSLOT)
		, readl(port->membase + S3C64XX_UINTP)
		, readl(port->membase + S3C64XX_UINTM)
	);
}

static void change_uart_gpio(int value, struct exynos_uart_port *ourport)
{
	int status = 0;
	struct uart_port *port = &ourport->port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	if (value) {
		/* Disabled or default pin states	*/
		if (!(ourport->rts_control & RTS_CTRL_CHK_EN)) {
			if (!IS_ERR(ourport->uart_pinctrl_tx_dat)) {
				status = pinctrl_select_state(ourport->pinctrl,
						      ourport->uart_pinctrl_tx_dat);
				if (status)
					dev_err(ourport->port.dev,
						"Can't set TXD uart pins!!!\n");
			else
				udelay(10);
			}
			if (!IS_ERR(ourport->uart_pinctrl_rts)) {
				status = pinctrl_select_state(ourport->pinctrl,
						      ourport->uart_pinctrl_rts);
				if (status)
					dev_err(ourport->port.dev,
						"Can't set RTS uart pins!!!\n");
			}
			ourport->rts_control = RTS_CTRL_ENABLE;
		}
	} else {
		/* Enabled pin state	*/
		if ((ourport->rts_control & RTS_CTRL_CHK_EN)) {
			if (!IS_ERR(ourport->uart_pinctrl_default)) {
				status = pinctrl_select_state(ourport->pinctrl,
						      ourport->uart_pinctrl_default);
				if (status)
					dev_err(ourport->port.dev,
					"Can't set default uart pins!!!\n");
			}
			ourport->rts_control = RTS_CTRL_DISABLE;
		}
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

static void print_uart_mode(struct uart_port *port, struct ktermios *termios,
			    unsigned int baud)
{
	dev_err(port->dev, "UART port%d configurations\n", port->line);

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		dev_err(port->dev, " - 5bits word length\n");
		break;
	case CS6:
		dev_err(port->dev, " - 6bits word length\n");
		break;
	case CS7:
		dev_err(port->dev, " - 7bits word length\n");
		break;
	case CS8:
	default:
		dev_err(port->dev, " - 8bits word length\n");
		break;
	}

	if (termios->c_cflag & CSTOPB)
		dev_err(port->dev, " - Use TWO stop bit\n");
	else
		dev_err(port->dev, " - Use one stop bit\n");

	if (termios->c_cflag & CRTSCTS)
		dev_err(port->dev, " - Use Autoflow control\n");

	dev_err(port->dev, " - Baudrate : %u\n", baud);
}

static ssize_t
uart_dbg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"UART Debug Mode Configuration.\n");
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"0 : Change loopback & DBG mode.\n");
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"1 : Change DBG mode.\n");
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"2 : Change Normal mode.\n");

	if (ret < PAGE_SIZE - 1) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	} else {
		buf[PAGE_SIZE - 2] = '\n';
		buf[PAGE_SIZE - 1] = '\0';
		ret = PAGE_SIZE - 1;
	}

	return ret;
}

static ssize_t
uart_dbg_store(struct device *dev, struct device_attribute *attr,
	       const char *buf, size_t count)
{
	int input_cmd = 0, ret;
	struct exynos_uart_port *ourport;

	ret = kstrtoint(buf, 0, &input_cmd);

	list_for_each_entry(ourport, &drvdata_list, node) {
		if (&ourport->pdev->dev != dev)
			continue;

		switch (input_cmd) {
		case 0:
			dev_err(dev, "Change UART%d to Loopback(DBG) mode\n",
				ourport->port.line);
			ourport->dbg_mode = UART_DBG_MODE | UART_LOOPBACK_MODE;
			break;
		case 1:
			dev_err(dev, "Change UART%d to DBG mode\n",
				ourport->port.line);
			ourport->dbg_mode = UART_DBG_MODE;
			break;
		case 2:
			dev_err(dev, "Change UART%d to normal mode\n",
				ourport->port.line);
			ourport->dbg_mode = 0;
			break;
		default:
			dev_err(dev, "Wrong Command!(0/1/2)\n");
		}
	}

	return count;
}

static DEVICE_ATTR_RW(uart_dbg);

static void exynos_usi_init(struct uart_port *port);
static void exynos_usi_stop(struct uart_port *port);

static void uart_copy_to_local_buf(int dir, struct uart_local_buf *local_buf,
				   unsigned char *trace_buf, int len)
{
	unsigned long long time;
	unsigned long rem_nsec;
	int i;
	int cpu = raw_smp_processor_id();

	time = cpu_clock(cpu);
	rem_nsec = do_div(time, NSEC_PER_SEC);

	if (local_buf->index + (len * 3 + 30) >= local_buf->size)
		local_buf->index = 0;

	local_buf->index += snprintf(local_buf->buffer + local_buf->index,
				 local_buf->size - local_buf->index,
				"[%5lu.%06lu] ",
				(unsigned long)time, rem_nsec / NSEC_PER_USEC);

	if (dir == 1)
		local_buf->index += snprintf(local_buf->buffer +
					     local_buf->index, local_buf->size -
					     local_buf->index, "[RX] ");
	else
		local_buf->index += snprintf(local_buf->buffer +
					     local_buf->index, local_buf->size -
					     local_buf->index, "[TX] ");

	for (i = 0; i < len; i++) {
		local_buf->index += snprintf(local_buf->buffer +
					     local_buf->index,
					     local_buf->size - local_buf->index,
					     "%02X ", trace_buf[i]);
	}

	local_buf->index += snprintf(local_buf->buffer + local_buf->index,
				     local_buf->size - local_buf->index, "\n");
}

static void exynos_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg);
static void exynos_serial_pm(struct uart_port *port, unsigned int level,
			     unsigned int old);
static struct uart_driver exynos_uart_drv;

static inline void uart_clock_enable(struct exynos_uart_port *ourport)
{
	clk_prepare_enable(ourport->separated_clk);
	clk_prepare_enable(ourport->clk);
}

static inline void uart_clock_disable(struct exynos_uart_port *ourport)
{
	clk_disable_unprepare(ourport->clk);
	if (ourport->separated_clk)
		clk_disable_unprepare(ourport->separated_clk);
}

static inline struct exynos_uart_port *to_ourport(struct uart_port *port)
{
	return container_of(port, struct exynos_uart_port, port);
}

/* translate a port to the device name */

static inline const char *exynos_serial_portname(struct uart_port *port)
{
	return to_platform_device(port->dev)->name;
}

static int exynos_serial_txempty_nofifo(struct uart_port *port)
{
	return rd_regl(port, S3C2410_UTRSTAT) & S3C2410_UTRSTAT_TXE;
}

/*
 * s3c64xx and later SoC's include the interrupt mask and status registers in
 * the controller itself, unlike the exynos SoC's which have these registers
 * in the interrupt controller. Check if the port type is s3c64xx or higher.
 */
static int exynos_serial_has_interrupt_mask(struct uart_port *port)
{
	return to_ourport(port)->info->type == PORT_S3C6400;
}

static void exynos_serial_rx_enable(struct uart_port *port)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned long flags;
	unsigned int ucon, ufcon;
	int count = 10000;

	spin_lock_irqsave(&port->lock, flags);

	while (--count && !exynos_serial_txempty_nofifo(port))
		usleep_range(100, 200);

	ufcon = rd_regl(port, S3C2410_UFCON);
	ufcon |= S3C2410_UFCON_RESETRX;
	wr_regl(port, S3C2410_UFCON, ufcon);

	ucon = rd_regl(port, S3C2410_UCON);
	ucon |= S3C2410_UCON_RXIRQMODE;
	wr_regl(port, S3C2410_UCON, ucon);

	ourport->rx_enabled = 1;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void exynos_serial_rx_disable(struct uart_port *port)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned long flags;
	unsigned int ucon;

	spin_lock_irqsave(&port->lock, flags);

	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~S3C2410_UCON_RXIRQMODE;
	wr_regl(port, S3C2410_UCON, ucon);

	ourport->rx_enabled = 0;
	spin_unlock_irqrestore(&port->lock, flags);
}

static void exynos_serial_stop_tx(struct uart_port *port)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	struct exynos_uart_dma *dma = ourport->dma;
	struct circ_buf *xmit = &port->state->xmit;
	struct dma_tx_state state;
	int count;

	if (!ourport->tx_enabled)
		return;

	if (exynos_serial_has_interrupt_mask(port))
		exynos_set_bit(port, S3C64XX_UINTM_TXD, S3C64XX_UINTM);
	else
		disable_irq_nosync(ourport->tx_irq);

	if (dma && dma->tx_chan && ourport->tx_in_progress == EXYNOS_TX_DMA) {
		dmaengine_pause(dma->tx_chan);
		dmaengine_tx_status(dma->tx_chan, dma->tx_cookie, &state);
		dmaengine_terminate_all(dma->tx_chan);
		dma_sync_single_for_cpu(ourport->port.dev,
					dma->tx_transfer_addr, dma->tx_size,
					DMA_TO_DEVICE);
		async_tx_ack(dma->tx_desc);
		count = dma->tx_bytes_requested - state.residue;
		xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
		port->icount.tx += count;
	}

	ourport->tx_enabled = 0;
	ourport->tx_in_progress = 0;

	if (port->flags & UPF_CONS_FLOW)
		exynos_serial_rx_enable(port);

	ourport->tx_mode = 0;
}

static void exynos_serial_start_next_tx(struct exynos_uart_port *ourport);

static void exynos_serial_tx_dma_complete(void *args)
{
	struct exynos_uart_port *ourport = args;
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct exynos_uart_dma *dma = ourport->dma;
	struct dma_tx_state state;
	unsigned long flags;
	int count;

	dmaengine_tx_status(dma->tx_chan, dma->tx_cookie, &state);
	count = dma->tx_bytes_requested - state.residue;
	async_tx_ack(dma->tx_desc);

	dma_sync_single_for_cpu(ourport->port.dev, dma->tx_transfer_addr,
				dma->tx_size, DMA_TO_DEVICE);

	spin_lock_irqsave(&port->lock, flags);

	xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
	port->icount.tx += count;
	ourport->tx_in_progress = 0;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	exynos_serial_start_next_tx(ourport);
	spin_unlock_irqrestore(&port->lock, flags);
}

static void enable_tx_dma(struct exynos_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	u32 ucon;

	/* Mask Tx interrupt */
	if (exynos_serial_has_interrupt_mask(port))
		exynos_set_bit(port, S3C64XX_UINTM_TXD, S3C64XX_UINTM);
	else
		disable_irq_nosync(ourport->tx_irq);

	/* Enable tx dma mode */
	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~(S3C64XX_UCON_TXBURST_MASK | S3C64XX_UCON_TXMODE_MASK);
#if 0
	ucon |= (dma_get_cache_alignment() >= 16) ?
		S3C64XX_UCON_TXBURST_16 : S3C64XX_UCON_TXBURST_1;
#endif
	ucon |= S3C64XX_UCON_TXBURST_1;
	ucon |= S3C64XX_UCON_TXMODE_DMA;
	wr_regl(port,  S3C2410_UCON, ucon);

	ourport->tx_mode = EXYNOS_TX_DMA;
}

static void enable_tx_pio(struct exynos_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	u32 ucon, ufcon;

	/* Set ufcon txtrig */
	ourport->tx_in_progress = EXYNOS_TX_PIO;
	ufcon = rd_regl(port, S3C2410_UFCON);
	wr_regl(port,  S3C2410_UFCON, ufcon);

	/* Enable tx pio mode */
	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~(S3C64XX_UCON_TXMODE_MASK);
	ucon |= S3C64XX_UCON_TXMODE_CPU;
	wr_regl(port,  S3C2410_UCON, ucon);

	/* Unmask Tx interrupt */
	if (exynos_serial_has_interrupt_mask(port))
		exynos_clear_bit(port, S3C64XX_UINTM_TXD, S3C64XX_UINTM);

	else
		enable_irq(ourport->tx_irq);

	ourport->tx_mode = EXYNOS_TX_PIO;
}

static void exynos_serial_start_tx_pio(struct exynos_uart_port *ourport)
{
	if (ourport->tx_mode != EXYNOS_TX_PIO)
		enable_tx_pio(ourport);
}

static int exynos_serial_start_tx_dma(struct exynos_uart_port *ourport,
				      unsigned int count)
{
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;
	struct exynos_uart_dma *dma = ourport->dma;

	if (ourport->tx_mode != EXYNOS_TX_DMA)
		enable_tx_dma(ourport);

	dma->tx_size = count & ~(dma_get_cache_alignment() - 1);
	dma->tx_transfer_addr = dma->tx_addr + xmit->tail;

	if (ourport->uart_logging && dma->tx_size)
		uart_copy_to_local_buf(0, &ourport->uart_local_buf,
				       ourport->port.state->xmit.buf +
				       xmit->tail, dma->tx_size);

	dma_sync_single_for_device(ourport->port.dev, dma->tx_transfer_addr,
				   dma->tx_size, DMA_TO_DEVICE);

	dma->tx_desc = dmaengine_prep_slave_single(dma->tx_chan,
						   dma->tx_transfer_addr,
						   dma->tx_size,
						   DMA_MEM_TO_DEV,
						   DMA_PREP_INTERRUPT);
	if (!dma->tx_desc) {
		dev_err(ourport->port.dev, "Unable to get desc for Tx\n");
		return -EIO;
	}

	dma->tx_desc->callback = exynos_serial_tx_dma_complete;
	dma->tx_desc->callback_param = ourport;
	dma->tx_bytes_requested = dma->tx_size;

	ourport->tx_in_progress = EXYNOS_TX_DMA;
	dma->tx_cookie = dmaengine_submit(dma->tx_desc);
	dma_async_issue_pending(dma->tx_chan);
	return 0;
}

static void exynos_serial_start_next_tx(struct exynos_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned long count;

	/* Get data size up to the end of buffer */
	count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

	if (!count) {
		exynos_serial_stop_tx(port);
		return;
	}

	if (!ourport->dma || !ourport->dma->tx_chan ||
	    count < ourport->min_dma_size ||
	    xmit->tail & (dma_get_cache_alignment() - 1))
		exynos_serial_start_tx_pio(ourport);
	else
		exynos_serial_start_tx_dma(ourport, count);
}

static void exynos_serial_start_tx(struct uart_port *port)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	struct circ_buf *xmit = &port->state->xmit;

	if (!ourport->tx_enabled) {
		if (port->flags & UPF_CONS_FLOW)
			exynos_serial_rx_disable(port);

		ourport->tx_enabled = 1;
		if (!ourport->dma || !ourport->dma->tx_chan)
			exynos_serial_start_tx_pio(ourport);
	}

	if (ourport->dma && ourport->dma->tx_chan) {
		if (!uart_circ_empty(xmit) && !ourport->tx_in_progress)
			exynos_serial_start_next_tx(ourport);
	}
}

/* Throttle is called in n_tty_receive_buf_common */
static void exynos_serial_throttle(struct uart_port *port)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	__set_bit(S3C64XX_UINTM_RXD, portaddrl(port, S3C64XX_UINTM));
	wr_regl(port, S3C64XX_UINTP, S3C64XX_UINTM_RXD_MSK);

	spin_unlock_irqrestore(&port->lock, flags);
}

/* Unthrottle is called in n_tty_read */
static void exynos_serial_unthrottle(struct uart_port *port)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	__clear_bit(S3C64XX_UINTM_RXD, portaddrl(port, S3C64XX_UINTM));

	spin_unlock_irqrestore(&port->lock, flags);
}

static void exynos_uart_copy_rx_to_tty(struct exynos_uart_port *ourport, struct
				       tty_port * tty, int count)
{
	struct exynos_uart_dma *dma = ourport->dma;
	int copied;

	if (!count)
		return;

	dma_sync_single_for_cpu(ourport->port.dev, dma->rx_addr,
				dma->rx_size, DMA_FROM_DEVICE);

	ourport->port.icount.rx += count;
	if (!tty) {
		dev_err(ourport->port.dev, "No tty port\n");
		return;
	}

	if (ourport->uart_logging && count)
		uart_copy_to_local_buf(1, &ourport->uart_local_buf,
				       ourport->dma->rx_buf, count);

	copied = tty_insert_flip_string(tty,
					(unsigned char *)ourport->dma->rx_buf,
					count);
	if (copied != count) {
		WARN_ON(1);
		dev_err(ourport->port.dev, "RxData copy to tty layer failed\n");
	}
}

static void exynos_serial_stop_rx(struct uart_port *port)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	struct exynos_uart_dma *dma = ourport->dma;
	struct tty_port *t = &port->state->port;
	struct dma_tx_state state;
	enum dma_status dma_status;
	unsigned int received;

	if (ourport->rx_enabled) {
		pr_debug("%s: port=%p\n", __func__, port);
		if (exynos_serial_has_interrupt_mask(port))
			exynos_set_bit(port, S3C64XX_UINTM_RXD, S3C64XX_UINTM);
		else
			disable_irq_nosync(ourport->rx_irq);
		ourport->rx_enabled = 0;
	}
	if (dma && dma->rx_chan) {
		dmaengine_pause(dma->tx_chan);
		dma_status = dmaengine_tx_status(dma->rx_chan, dma->rx_cookie,
						 &state);
		if (dma_status == DMA_IN_PROGRESS || dma_status == DMA_PAUSED) {
			received = dma->rx_bytes_requested - state.residue;
			dmaengine_terminate_all(dma->rx_chan);
			exynos_uart_copy_rx_to_tty(ourport, t, received);
		}
	}
}

static inline struct exynos_uart_info
	*exynos_port_to_info(struct uart_port *port)
{
	return to_ourport(port)->info;
}

static inline struct s3c2410_uartcfg
	*exynos_port_to_cfg(struct uart_port *port)
{
	struct exynos_uart_port *ourport;

	if (!port->dev)
		return NULL;

	ourport = container_of(port, struct exynos_uart_port, port);
	return ourport->cfg;
}

static int exynos_serial_rx_fifocnt(struct exynos_uart_port *ourport,
				    unsigned long ufstat)
{
	struct exynos_uart_info *info = ourport->info;

	if (ufstat & info->rx_fifofull)
		return ourport->port.fifosize;

	return (ufstat & info->rx_fifomask) >> info->rx_fifoshift;
}

static void s3c64xx_start_rx_dma(struct exynos_uart_port *ourport);
static void exynos_serial_rx_dma_complete(void *args)
{
	struct exynos_uart_port *ourport = args;
	struct uart_port *port = &ourport->port;

	struct exynos_uart_dma *dma = ourport->dma;
	struct tty_port *t = &port->state->port;
	struct tty_struct *tty = tty_port_tty_get(&ourport->port.state->port);

	struct dma_tx_state state;
	unsigned long flags;
	int received;

	dmaengine_tx_status(dma->rx_chan,  dma->rx_cookie, &state);
	received  = dma->rx_bytes_requested - state.residue;
	async_tx_ack(dma->rx_desc);

	spin_lock_irqsave(&port->lock, flags);

	if (received)
		exynos_uart_copy_rx_to_tty(ourport, t, received);

	if (tty) {
		tty_flip_buffer_push(t);
		tty_kref_put(tty);
	}

	s3c64xx_start_rx_dma(ourport);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void s3c64xx_start_rx_dma(struct exynos_uart_port *ourport)
{
	struct exynos_uart_dma *dma = ourport->dma;

	dma_sync_single_for_device(ourport->port.dev, dma->rx_addr,
				   dma->rx_size, DMA_FROM_DEVICE);

	dma->rx_desc = dmaengine_prep_slave_single(dma->rx_chan, dma->rx_addr,
						   dma->rx_size,
						   DMA_DEV_TO_MEM,
						   DMA_PREP_INTERRUPT);
	if (!dma->rx_desc) {
		dev_err(ourport->port.dev, "Unable to get desc for Rx\n");
		return;
	}

	dma->rx_desc->callback = exynos_serial_rx_dma_complete;
	dma->rx_desc->callback_param = ourport;
	dma->rx_bytes_requested = dma->rx_size;

	dma->rx_cookie = dmaengine_submit(dma->rx_desc);
	dma_async_issue_pending(dma->rx_chan);
}

static int exynos_serial_tx_fifocnt(struct exynos_uart_port *ourport,
				    unsigned long ufstat)
{
	struct exynos_uart_info *info = ourport->info;

	if (ufstat & info->tx_fifofull)
		return ourport->port.fifosize;

	return (ufstat & info->tx_fifomask) >> info->tx_fifoshift;
}

/* ? - where has parity gone?? */
#define S3C2410_UERSTAT_PARITY (0x1000)

static void enable_rx_dma(struct exynos_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	unsigned int ucon;

	/* set rx mode to dma mode */
	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~(S3C64XX_UCON_RXBURST_MASK |
			S3C64XX_UCON_TIMEOUT_MASK |
			S3C64XX_UCON_EMPTYINT_EN |
			S3C64XX_UCON_DMASUS_EN |
			S3C64XX_UCON_TIMEOUT_EN |
			S3C64XX_UCON_RXMODE_MASK);
	ucon |= S3C64XX_UCON_RXBURST_1 |
			0xf << S3C64XX_UCON_TIMEOUT_SHIFT |
			S3C64XX_UCON_EMPTYINT_EN |
			S3C64XX_UCON_TIMEOUT_EN |
			S3C64XX_UCON_RXMODE_DMA;
	wr_regl(port, S3C2410_UCON, ucon);

	ourport->rx_mode = EXYNOS_RX_DMA;
}

static void enable_rx_pio(struct exynos_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	unsigned int ucon;

	/* set rx mode to pio mode */
	ucon = rd_regl(port, S3C2410_UCON);
	ucon &= ~(S3C64XX_UCON_TIMEOUT_MASK |
			S3C64XX_UCON_EMPTYINT_EN |
			S3C64XX_UCON_DMASUS_EN |
			S3C64XX_UCON_TIMEOUT_EN |
			S3C64XX_UCON_RXMODE_MASK);
	ucon |= 0xf << S3C64XX_UCON_TIMEOUT_SHIFT |
			S3C64XX_UCON_TIMEOUT_EN |
			S3C64XX_UCON_RXMODE_CPU;
	wr_regl(port, S3C2410_UCON, ucon);

	ourport->rx_mode = EXYNOS_RX_PIO;
}

static void exynos_serial_rx_drain_fifo(struct exynos_uart_port *ourport);

static irqreturn_t exynos_serial_rx_chars_dma(void *dev_id)
{
	unsigned int utrstat, ufstat, received;
	struct exynos_uart_port *ourport = dev_id;
	struct uart_port *port = &ourport->port;
	struct exynos_uart_dma *dma = ourport->dma;
	struct tty_port *t = &port->state->port;
	unsigned long flags;
	struct dma_tx_state state;

	utrstat = rd_regl(port, S3C2410_UTRSTAT);
	ufstat = rd_regl(port, S3C2410_UFSTAT);

	spin_lock_irqsave(&port->lock, flags);

	if (!(utrstat & S3C2410_UTRSTAT_TIMEOUT) &&
	    ourport->rx_mode == EXYNOS_RX_PIO) {
		s3c64xx_start_rx_dma(ourport);
		enable_rx_dma(ourport);
		wr_regl(port, S3C64XX_UINTP, S3C64XX_UINTM_RXD_MSK);
		goto finish;
	}

	if (ourport->rx_mode == EXYNOS_RX_DMA) {
		dmaengine_pause(dma->rx_chan);
		dmaengine_tx_status(dma->rx_chan, dma->rx_cookie, &state);
		dmaengine_terminate_all(dma->rx_chan);
		received = dma->rx_bytes_requested - state.residue;
		exynos_uart_copy_rx_to_tty(ourport, t, received);

		enable_rx_pio(ourport);
	}

	exynos_serial_rx_drain_fifo(ourport);

finish:
	wr_regl(port, S3C2410_UTRSTAT, S3C2410_UTRSTAT_TIMEOUT);

	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

static void exynos_serial_rx_drain_fifo(struct exynos_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	unsigned int ufcon, ch, flag, ufstat, uerstat;
	unsigned int fifocnt = 0;
	int max_count = port->fifosize;
	unsigned char insert_buf[256] = {0, };
	unsigned int insert_cnt = 0;
	unsigned char trace_buf[256] = {0, };
	int trace_cnt = 0;
	char buf[DATA_BYTES_PER_LINE * 3 + 1];

	exynos_set_bit(port, S3C64XX_UINTM_RXD, S3C64XX_UINTM);
	wr_regl(port, S3C64XX_UINTP, S3C64XX_UINTM_RXD_MSK);

	while (max_count-- > 0) {
		/*
		 * Receive all characters known to be in FIFO
		 * before reading FIFO level again
		 */
		if (fifocnt == 0) {
			ufstat = rd_regl(port, S3C2410_UFSTAT);
			fifocnt = exynos_serial_rx_fifocnt(ourport, ufstat);
			if (fifocnt == 0)
				break;
		}
		fifocnt--;

		uerstat = rd_regl(port, S3C2410_UERSTAT);
		ch = rd_reg(port, S3C2410_URXH);

		if (port->flags & UPF_CONS_FLOW) {
			int txe = exynos_serial_txempty_nofifo(port);

			if (ourport->rx_enabled) {
				if (!txe) {
					ourport->rx_enabled = 0;
					continue;
				}
			} else {
				if (txe) {
					ufcon = rd_regl(port, S3C2410_UFCON);
					ufcon |= S3C2410_UFCON_RESETRX;
					wr_regl(port, S3C2410_UFCON, ufcon);
					ourport->rx_enabled = 1;
					return;
				}
				continue;
			}
		}

		/* insert the character into the buffer */

		flag = TTY_NORMAL;
		port->icount.rx++;

		if (unlikely(uerstat & S3C2410_UERSTAT_ANY)) {
			pr_debug("rxerr: port ch=0x%02x, rxs=0x%08x\n",
				 ch, uerstat);
			if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
				logbuffer_log(ourport->log, "rxerr: port ch=0x%02x, rxs=0x%08x\n",
					      ch, uerstat);

			uart_sfr_dump(ourport);

			/* check for break */
			if (uerstat & S3C2410_UERSTAT_BREAK) {
				pr_debug("break!\n");
				port->icount.brk++;
				if (uart_handle_break(port))
					continue; /* Ignore character */
			}

			if (uerstat & S3C2410_UERSTAT_FRAME) {
				pr_err("[tty] uerstat & S3C2410_UERSTAT_FRAME!\n");
				port->icount.frame++;
			}
			if (uerstat & S3C2410_UERSTAT_OVERRUN) {
				pr_err("[tty] uerstat & S3C2410_UERSTAT_OVERRUN!\n");
				port->icount.overrun++;
			}
			uerstat &= port->read_status_mask;

			if (uerstat & S3C2410_UERSTAT_BREAK) {
				pr_err("[tty] uerstat & S3C2410_UERSTAT_BREAK2!\n");
				flag = TTY_BREAK;
			} else if (uerstat & S3C2410_UERSTAT_PARITY) {
				pr_err("[tty] uerstat & S3C2410_UERSTAT_PARITY!\n");
				flag = TTY_PARITY;
			} else if (uerstat & (S3C2410_UERSTAT_FRAME |
					    S3C2410_UERSTAT_OVERRUN))
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(port, ch))
			continue; /* Ignore character */

		insert_buf[insert_cnt++] = ch;
		if (ourport->uart_logging)
			trace_buf[trace_cnt++] = ch;
	}

	if (ourport->uart_logging && trace_cnt) {
		if (!IS_ERR_OR_NULL(ourport->log)) {
			hex_dump_to_buffer(trace_buf, trace_cnt, DATA_BYTES_PER_LINE,
				1, buf, sizeof(buf), false);
			logbuffer_log(ourport->log, "RX: len: %d, buf: %s", trace_cnt, buf);
		}
		uart_copy_to_local_buf(1, &ourport->uart_local_buf, trace_buf, trace_cnt);
	}

	exynos_clear_bit(port, S3C64XX_UINTM_RXD, S3C64XX_UINTM);

	tty_insert_flip_string(&port->state->port, insert_buf, insert_cnt);
	tty_flip_buffer_push(&port->state->port);
}

static irqreturn_t
exynos_serial_rx_chars_pio(void *dev_id)
{
	struct exynos_uart_port *ourport = dev_id;
	struct uart_port *port = &ourport->port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	exynos_serial_rx_drain_fifo(ourport);
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t exynos_serial_rx_chars(int irq, void *dev_id)
{
	struct exynos_uart_port *ourport = dev_id;

	if (ourport->dma && ourport->dma->rx_chan)
		return exynos_serial_rx_chars_dma(dev_id);
	return exynos_serial_rx_chars_pio(dev_id);
}

static irqreturn_t exynos_serial_tx_chars(int irq, void *id)
{
	struct exynos_uart_port *ourport = id;
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;
	unsigned long flags;
	int count = port->fifosize, dma_count = 0;
	unsigned char trace_buf[256] = {0, };
	int trace_cnt = 0;
	char buf[DATA_BYTES_PER_LINE * 3 + 1];

	spin_lock_irqsave(&port->lock, flags);

	exynos_set_bit(port, S3C64XX_UINTM_TXD, S3C64XX_UINTM);
	wr_regl(port, S3C64XX_UINTP, S3C64XX_UINTM_TXD_MSK);

	count = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

	if (ourport->dma && ourport->dma->tx_chan &&
	    count >= ourport->min_dma_size) {
		int align = dma_get_cache_alignment() -
			(xmit->tail & (dma_get_cache_alignment() - 1));
		if (count - align >= ourport->min_dma_size) {
			dma_count = count - align;
			count = align;
		}
	}

	if (port->x_char) {
		wr_reg(port, S3C2410_UTXH, port->x_char);
		if (ourport->uart_logging)
			trace_buf[trace_cnt++] = port->x_char;
		port->icount.tx++;
		port->x_char = 0;
		goto out;
	}

	/* if there isn't anything more to transmit, or the uart is now
	 * stopped, disable the uart and exit
	 */

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		exynos_serial_stop_tx(port);
		goto out;
	}

	/* try and drain the buffer... */

	if (count > port->fifosize) {
		count = port->fifosize;
		dma_count = 0;
	}

	while (!uart_circ_empty(xmit) && count-- > 0) {
		if (rd_regl(port, S3C2410_UFSTAT) & ourport->info->tx_fifofull)
			break;

		wr_reg(port, S3C2410_UTXH, xmit->buf[xmit->tail]);
		if (ourport->uart_logging)
			trace_buf[trace_cnt++] = (unsigned
						  char)xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (!count && dma_count) {
		exynos_serial_start_tx_dma(ourport, dma_count);
		goto out;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
		spin_unlock_irqrestore(&port->lock, flags);
		uart_write_wakeup(port);
		spin_lock_irqsave(&port->lock, flags);
	}

	if (uart_circ_empty(xmit))
		exynos_serial_stop_tx(port);

out:
	if (ourport->tx_enabled)
		exynos_clear_bit(port, S3C64XX_UINTM_TXD, S3C64XX_UINTM);

	if (ourport->uart_logging && trace_cnt) {
		if (!IS_ERR_OR_NULL(ourport->log)) {
			hex_dump_to_buffer(trace_buf, trace_cnt, DATA_BYTES_PER_LINE,
				1, buf, sizeof(buf), false);
			logbuffer_log(ourport->log, "TX: len: %d, buf: %s", trace_cnt, buf);
		}
		uart_copy_to_local_buf(0, &ourport->uart_local_buf, trace_buf, trace_cnt);
	}

	spin_unlock_irqrestore(&port->lock, flags);
	return IRQ_HANDLED;
}

/* interrupt handler for s3c64xx and later SoC's.*/
static irqreturn_t s3c64xx_serial_handle_irq(int irq, void *id)
{
	struct exynos_uart_port *ourport = id;
	struct uart_port *port = &ourport->port;
	irqreturn_t ret = IRQ_HANDLED;

	if (rd_regl(port, S3C64XX_UINTP) & S3C64XX_UINTM_RXD_MSK)
		ret = exynos_serial_rx_chars(irq, id);

	if (rd_regl(port, S3C64XX_UINTP) & S3C64XX_UINTM_TXD_MSK)
		ret = exynos_serial_tx_chars(irq, id);

	return ret;
}

static unsigned int exynos_serial_tx_empty(struct uart_port *port)
{
	struct exynos_uart_info *info = exynos_port_to_info(port);
	unsigned long ufstat = rd_regl(port, S3C2410_UFSTAT);
	unsigned long ufcon = rd_regl(port, S3C2410_UFCON);

	if (ufcon & S3C2410_UFCON_FIFOMODE) {
		if ((ufstat & info->tx_fifomask) != 0 ||
		    (ufstat & info->tx_fifofull))
			return 0;

		return 1;
	}

	return exynos_serial_txempty_nofifo(port);
}

/* no modem control lines */
static unsigned int exynos_serial_get_mctrl(struct uart_port *port)
{
	unsigned int umstat = rd_reg(port, S3C2410_UMSTAT);

	if (umstat & S3C2410_UMSTAT_CTS)
		return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
	else
		return TIOCM_CAR | TIOCM_DSR;
}

static void exynos_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned int umcon = rd_regl(port, S3C2410_UMCON);

	if (mctrl & TIOCM_RTS) {
		umcon |= S3C2410_UMCOM_RTS_LOW;
		if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
			logbuffer_log(ourport->log, "set_mctrl: Activate RTS, umcon:0x%08x", umcon);
	} else {
		umcon &= ~S3C2410_UMCOM_RTS_LOW;
		if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
			logbuffer_log(ourport->log, "set_mctrl: Inactivate RTS, umcon:0x%08x",
				umcon);
	}

	wr_regl(port, S3C2410_UMCON, umcon);
}

static void exynos_serial_break_ctl(struct uart_port *port, int break_state)
{
	unsigned long flags;
	unsigned int ucon;

	spin_lock_irqsave(&port->lock, flags);

	ucon = rd_regl(port, S3C2410_UCON);

	if (break_state)
		ucon |= S3C2410_UCON_SBREAK;
	else
		ucon &= ~S3C2410_UCON_SBREAK;

	wr_regl(port, S3C2410_UCON, ucon);

	spin_unlock_irqrestore(&port->lock, flags);
}

static int exynos_serial_request_dma(struct exynos_uart_port *ourport)
{
	struct exynos_uart_dma	*dma = ourport->dma;
	struct dma_slave_caps dma_caps;
	const char *reason = NULL;
	int ret;

	/* Default slave configuration parameters */
	dma->rx_conf.direction		= DMA_DEV_TO_MEM;
	dma->rx_conf.src_addr_width	= DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma->rx_conf.src_addr		= ourport->port.mapbase + S3C2410_URXH;
	dma->rx_conf.src_maxburst	= 1;

	dma->tx_conf.direction		= DMA_MEM_TO_DEV;
	dma->tx_conf.dst_addr_width	= DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma->tx_conf.dst_addr		= ourport->port.mapbase + S3C2410_UTXH;
	dma->tx_conf.dst_maxburst	= 1;

	dma->rx_chan = dma_request_chan(ourport->port.dev, "rx");

	if (IS_ERR(dma->rx_chan)) {
		reason = "DMA RX channel request failed";
		ret = PTR_ERR(dma->rx_chan);
		goto err_warn;
	}

	ret = dma_get_slave_caps(dma->rx_chan, &dma_caps);
	if (ret < 0 ||
	    dma_caps.residue_granularity < DMA_RESIDUE_GRANULARITY_BURST) {
		reason = "insufficient DMA RX engine capabilities";
		ret = -EOPNOTSUPP;
		goto err_release_rx;
	}

	dmaengine_slave_config(dma->rx_chan, &dma->rx_conf);

	dma->tx_chan = dma_request_chan(ourport->port.dev, "tx");
	if (IS_ERR(dma->tx_chan)) {
		reason = "DMA TX channel request failed";
		ret = PTR_ERR(dma->tx_chan);
		goto err_release_rx;
	}

	ret = dma_get_slave_caps(dma->tx_chan, &dma_caps);
	if (ret < 0 ||
	    dma_caps.residue_granularity < DMA_RESIDUE_GRANULARITY_BURST) {
		reason = "insufficient DMA TX engine capabilities";
		ret = -EOPNOTSUPP;
		goto err_release_tx;
	}

	dmaengine_slave_config(dma->tx_chan, &dma->tx_conf);

	/* RX buffer */
	dma->rx_size = PAGE_SIZE;

	dma->rx_buf = kmalloc(dma->rx_size, GFP_KERNEL);
	if (!dma->rx_buf) {
		ret = -ENOMEM;
		goto err_release_tx;
	}

	dma->rx_addr = dma_map_single(ourport->port.dev, dma->rx_buf,
				      dma->rx_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(ourport->port.dev, dma->rx_addr)) {
		reason = "DMA mapping error for RX buffer";
		ret = -EIO;
		goto err_free_rx;
	}

	/* TX buffer */
	dma->tx_addr = dma_map_single(ourport->port.dev,
				      ourport->port.state->xmit.buf,
				      UART_XMIT_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(ourport->port.dev, dma->tx_addr)) {
		reason = "DMA mapping error for TX buffer";
		ret = -EIO;
		goto err_unmap_rx;
	}

	return 0;

err_unmap_rx:
	dma_unmap_single(ourport->port.dev, dma->rx_addr, dma->rx_size,
			 DMA_FROM_DEVICE);
err_free_rx:
	kfree(dma->rx_buf);
err_release_tx:
	dma_release_channel(dma->tx_chan);
err_release_rx:
	dma_release_channel(dma->rx_chan);
err_warn:
	if (reason)
		dev_warn(ourport->port.dev, "%s, DMA will not be used\n",
			 reason);
	return ret;
}

static void exynos_serial_release_dma(struct exynos_uart_port *p)
{
	struct exynos_uart_dma	*dma = p->dma;

	if (dma->rx_chan) {
		dmaengine_terminate_all(dma->rx_chan);
		dma_unmap_single(p->port.dev, dma->rx_addr, dma->rx_size,
				 DMA_FROM_DEVICE);
		kfree(dma->rx_buf);
		dma_release_channel(dma->rx_chan);
		dma->rx_chan = NULL;
	}

	if (dma->tx_chan) {
		dmaengine_terminate_all(dma->tx_chan);
		dma_unmap_single(p->port.dev, dma->tx_addr, UART_XMIT_SIZE,
				 DMA_TO_DEVICE);
		dma_release_channel(dma->tx_chan);
		dma->tx_chan = NULL;
	}
}

static void exynos_serial_shutdown(struct uart_port *port)
{
	struct exynos_uart_port *ourport = to_ourport(port);

	if (ourport->tx_claimed) {
		if (!exynos_serial_has_interrupt_mask(port))
			free_irq(ourport->tx_irq, ourport);
		ourport->tx_enabled = 0;
		ourport->tx_claimed = 0;
		ourport->tx_mode = 0;
	}

	if (ourport->rx_claimed) {
		if (!exynos_serial_has_interrupt_mask(port))
			free_irq(ourport->rx_irq, ourport);
		ourport->rx_claimed = 0;
		ourport->rx_enabled = 0;
	}

	/* Clear pending interrupts and mask all interrupts */
	if (exynos_serial_has_interrupt_mask(port)) {
		free_irq(port->irq, ourport);

		wr_regl(port, S3C64XX_UINTP, 0xf);
		wr_regl(port, S3C64XX_UINTM, 0xf);
	}

	if (ourport->dma)
		exynos_serial_release_dma(ourport);

	ourport->tx_in_progress = 0;
}

static int exynos_serial_startup(struct uart_port *port)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned long flags;
	unsigned int ufcon;
	int ret;

	pr_debug("%s: port=%p (%08llx,%p)\n", __func__, port,
		 (unsigned long long)port->mapbase, port->membase);

	wr_regl(port, S3C64XX_UINTM, 0xf);
	if (ourport->dma) {
		ret = exynos_serial_request_dma(ourport);
		if (ret < 0) {
			devm_kfree(port->dev, ourport->dma);
			ourport->dma = NULL;
		}
	}

	if (ourport->use_default_irq == 1)
		ret = devm_request_irq(port->dev, port->irq,
				       s3c64xx_serial_handle_irq, IRQF_SHARED,
				       exynos_serial_portname(port), ourport);
	else
		ret = request_threaded_irq(port->irq, NULL,
					   s3c64xx_serial_handle_irq,
					   IRQF_ONESHOT,
					   exynos_serial_portname(port),
					   ourport);

	if (ret) {
		dev_err(port->dev, "cannot get irq %d\n", port->irq);
		return ret;
	}

	/* For compatibility with exynos Soc's */
	ourport->rx_enabled = 1;
	ourport->rx_claimed = 1;
	ourport->tx_enabled = 0;
	ourport->tx_claimed = 1;

	spin_lock_irqsave(&port->lock, flags);

	ufcon = rd_regl(port, S3C2410_UFCON);
	ufcon &= ~S5PV210_UFCON_RXTRIG256;
	ufcon |= S3C2410_UFCON_RESETRX | S5PV210_UFCON_RXTRIG4;
	if (!uart_console(port))
		ufcon |= S3C2410_UFCON_RESETTX;
	wr_regl(port, S3C2410_UFCON, ufcon);

	enable_rx_pio(ourport);

	spin_unlock_irqrestore(&port->lock, flags);

	/* Enable Rx Interrupt */
	exynos_clear_bit(port, S3C64XX_UINTM_RXD, S3C64XX_UINTM);

	return ret;
}

/* power power management control */
static void exynos_serial_pm(struct uart_port *port, unsigned int level,
			     unsigned int old)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned int umcon;

	switch (level) {
	case UART_PM_STATE_OFF:
		if (!ourport->in_band_wakeup) {
			/* disable auto flow control & set nRTS for High */
			umcon = rd_regl(port, S3C2410_UMCON);
			umcon &= ~(S3C2410_UMCOM_AFC | S3C2410_UMCOM_RTS_LOW);
			if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
				logbuffer_log(ourport->log,
					"pm: OFF: Disable AFC & Inactivate RTS, 0x%08x", umcon);
			wr_regl(port, S3C2410_UMCON, umcon);
		}
		if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
			logbuffer_log(ourport->log, "pm: OFF: uart_clock_disable");
		uart_clock_disable(ourport);
		break;

	case UART_PM_STATE_ON:
		if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
			logbuffer_log(ourport->log, "pm: ON: uart_clock_enable");
		uart_clock_enable(ourport);

		exynos_usi_init(port);
		exynos_serial_resetport(port, exynos_port_to_cfg(port));
		break;
	default:
		dev_err(port->dev, "exynos_serial: unknown pm %d\n", level);
	}
}

/* baud rate calculation
 *
 * The UARTs on the S3C2410/S3C2440 can take their clocks from a number
 * of different sources, including the peripheral clock ("pclk") and an
 * external clock ("uclk"). The S3C2440 also adds the core clock ("fclk")
 * with a programmable extra divisor.
 *
 * The following code goes through the clock sources, and calculates the
 * baud clocks (and the resultant actual baud rates) and then tries to
 * pick the closest one and select that.
 *
 */

#define MAX_CLK_NAME_LENGTH 20

static unsigned int exynos_serial_getclk(struct exynos_uart_port *ourport,
					 unsigned int req_baud,
					 struct clk **best_clk,
					 unsigned int *clk_num)
{
	struct exynos_uart_info *info = ourport->info;
	unsigned long rate;
	unsigned int cnt, baud, quot, clk_sel, best_quot = 0;
	int calc_deviation, deviation = (1 << 30) - 1;
	int ret;

	clk_sel = (ourport->cfg->clk_sel) ? ourport->cfg->clk_sel :
			ourport->info->def_clk_sel;
	for (cnt = 0; cnt < info->num_clks; cnt++) {
		if (!(clk_sel & (1 << cnt)))
			continue;

		rate = clk_get_rate(ourport->clk);

		if (rate == 0 && ourport->src_clk_rate) {
			ret = clk_set_rate(ourport->clk, ourport->src_clk_rate);
			if (ret < 0)
				dev_err(&ourport->pdev->dev, "UART clk set failed\n");

			rate = clk_get_rate(ourport->clk);
		}

		dev_info(&ourport->pdev->dev, " Clock rate : %ld\n", rate);

		if (!rate)
			continue;

		if (ourport->info->has_divslot) {
			unsigned long div = rate / req_baud;

			/* The UDIVSLOT register on the newer UARTs allows us to
			 * get a divisor adjustment of 1/16th on the baud clock.
			 *
			 * We don't keep the UDIVSLOT value (the 16ths we
			 * calculated by not multiplying the baud by 16) as it
			 * is easy enough to recalculate.
			 */

			quot = div / 16;
			baud = rate / div;
		} else {
			quot = (rate + (8 * req_baud)) / (16 * req_baud);
			baud = rate / (quot * 16);
		}
		quot--;

		calc_deviation = req_baud - baud;
		if (calc_deviation < 0)
			calc_deviation = -calc_deviation;

		if (calc_deviation < deviation) {
			*best_clk = ourport->clk;
			best_quot = quot;
			*clk_num = cnt;
			deviation = calc_deviation;
		}
	}

	return best_quot;
}

/* udivslot_table[]
 *
 * This table takes the fractional value of the baud divisor and gives
 * the recommended setting for the UDIVSLOT register.
 */
static u16 udivslot_table[16] = {
	[0] = 0x0000,
	[1] = 0x0080,
	[2] = 0x0808,
	[3] = 0x0888,
	[4] = 0x2222,
	[5] = 0x4924,
	[6] = 0x4A52,
	[7] = 0x54AA,
	[8] = 0x5555,
	[9] = 0xD555,
	[10] = 0xD5D5,
	[11] = 0xDDD5,
	[12] = 0xDDDD,
	[13] = 0xDFDD,
	[14] = 0xDFDF,
	[15] = 0xFFDF,
};

static void exynos_serial_set_termios(struct uart_port *port,
				      struct ktermios *termios,
				      struct ktermios *old)
{
	struct s3c2410_uartcfg *cfg = exynos_port_to_cfg(port);
	struct exynos_uart_port *ourport = to_ourport(port);
	struct clk *clk = ERR_PTR(-EINVAL);
	unsigned long flags;
	unsigned int baud, quot, clk_sel = 0;
	unsigned int ulcon;
	unsigned int umcon;
	unsigned int udivslot = 0;
	unsigned int real_baud_rd, real_baud_ru = 0;
	int calc_deviation_rd, calc_deviation_ru = 0;

	/*
	 * We don't support modem control lines.
	 */
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/*
	 * Ask the core to calculate the divisor for us.
	 */

	baud = uart_get_baud_rate(port, termios, old, MIN_BAUD, MAX_BAUD);
	if (ourport->dbg_uart_ch && baud == 9600)
		baud = ourport->dbg_uart_baud;
	quot = exynos_serial_getclk(ourport, baud, &clk, &clk_sel);
	if (baud == 38400 && (port->flags & UPF_SPD_MASK) == UPF_SPD_CUST)
		quot = port->custom_divisor;
	if (IS_ERR(clk))
		return;

	/* setting clock for baud rate */
	if (ourport->baudclk != clk) {
		ourport->baudclk = clk;
		ourport->baudclk_rate = clk ? clk_get_rate(clk) : 0;
	}

	if (ourport->info->has_divslot) {
		unsigned int div = ourport->baudclk_rate / baud;

		/*
		 * Find udivslot of the lowest error rate
		 *
		 * udivslot cannot be round-up to 0 because quot is fixed
		 */
		if ((div & 15) != 15) {
			real_baud_rd = ourport->baudclk_rate / div;
			real_baud_ru = ourport->baudclk_rate / (div + 1);

			calc_deviation_rd = baud - real_baud_rd;
			calc_deviation_ru = baud - real_baud_ru;

			if (calc_deviation_rd < 0)
				calc_deviation_rd = -calc_deviation_rd;
			if (calc_deviation_ru < 0)
				calc_deviation_ru = -calc_deviation_ru;

			if (calc_deviation_rd > calc_deviation_ru)
				div = div + 1;
		}

		if (cfg->has_fracval) {
			udivslot = (div & 15);
			dev_dbg(port->dev, "fracval = %04x\n", udivslot);
		} else {
			udivslot = udivslot_table[div & 15];
			dev_dbg(port->dev, "udivslot = %04x (div %d)\n",
				udivslot, div & 15);
		}
	}

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		dev_dbg(port->dev, "config: 5bits/char\n");
		ulcon = S3C2410_LCON_CS5;
		break;
	case CS6:
		dev_dbg(port->dev, "config: 6bits/char\n");
		ulcon = S3C2410_LCON_CS6;
		break;
	case CS7:
		dev_dbg(port->dev, "config: 7bits/char\n");
		ulcon = S3C2410_LCON_CS7;
		break;
	case CS8:
	default:
		dev_dbg(port->dev, "config: 8bits/char\n");
		ulcon = S3C2410_LCON_CS8;
		break;
	}

	if (ourport->dbg_uart_ch) {
		switch (ourport->dbg_word_len) {
		case 5:
			ulcon = S3C2410_LCON_CS5;
			break;
		case 6:
			ulcon = S3C2410_LCON_CS6;
			break;
		case 7:
			ulcon = S3C2410_LCON_CS7;
			break;
		case 8:
		default:
			ulcon = S3C2410_LCON_CS8;
			break;
		}
	}

	/* preserve original lcon IR settings */
	if (!ourport->usi_v2)
		ulcon |= (unsigned int)(cfg->ulcon & S3C2410_LCON_IRM);

	if (termios->c_cflag & CSTOPB)
		ulcon |= S3C2410_LCON_STOPB;

	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			ulcon |= S3C2410_LCON_PODD;
		else
			ulcon |= S3C2410_LCON_PEVEN;
	} else {
		ulcon |= S3C2410_LCON_PNONE;
	}

	spin_lock_irqsave(&port->lock, flags);

	dev_dbg(port->dev, "setting ulcon to %08x, brddiv to %d, udivslot %08x\n",
		ulcon, quot, udivslot);
	if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
		logbuffer_log(ourport->log, "set_termios: ulcon %08x, brddiv %d, udivslot %08x",
		ulcon, quot, udivslot);

	wr_regl(port, S3C2410_ULCON, ulcon);
	wr_regl(port, S3C2410_UBRDIV, quot);

	if (ourport->info->has_divslot)
		wr_regl(port, S3C2443_DIVSLOT, udivslot);
	port->status &= ~UPSTAT_AUTOCTS;

	umcon = rd_regl(port, S3C2410_UMCON);
	if ((termios->c_cflag & CRTSCTS) && (!ourport->dbg_uart_ch)) {
		umcon |= S3C2410_UMCOM_AFC;
		port->status = UPSTAT_AUTOCTS;
		if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
			logbuffer_log(ourport->log, "set_termios: Enable AFC, umcon:0x%08x", umcon);
	} else {
		umcon &= ~S3C2410_UMCOM_AFC;
		if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
			logbuffer_log(ourport->log, "set_termios: Disable AFC, umcon:0x%08x",
				umcon);
	}
	wr_regl(port, S3C2410_UMCON, umcon);

	dev_dbg(port->dev, "uart: ulcon = 0x%08x, ucon = 0x%08x, ufcon = 0x%08x\n",
		rd_regl(port, S3C2410_ULCON),
		rd_regl(port, S3C2410_UCON),
		rd_regl(port, S3C2410_UFCON));
	if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
		logbuffer_log(ourport->log, "set_termios: ulcon 0x%08x, ucon 0x%08x, ufcon 0x%08x",
		rd_regl(port, S3C2410_ULCON),
		rd_regl(port, S3C2410_UCON),
		rd_regl(port, S3C2410_UFCON));

	if (ourport->dbg_mode & UART_DBG_MODE)
		print_uart_mode(port, termios, baud);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Which character status flags are we interested in?
	 */
	port->read_status_mask = S3C2410_UERSTAT_OVERRUN;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= S3C2410_UERSTAT_FRAME |
			S3C2410_UERSTAT_PARITY;
	/*
	 * Which character status flags should we ignore?
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= S3C2410_UERSTAT_OVERRUN;
	if (termios->c_iflag & IGNBRK && termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= S3C2410_UERSTAT_FRAME;

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= RXSTAT_DUMMY_READ;

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *exynos_serial_type(struct uart_port *port)
{
	switch (port->type) {
	case PORT_S3C2410:
		return "S3C2410";
	case PORT_S3C2440:
		return "S3C2440";
	case PORT_S3C2412:
		return "S3C2412";
	case PORT_S3C6400:
		return "S3C6400/10";
	default:
		return NULL;
	}
}

#define MAP_SIZE (0x100)

static void exynos_serial_release_port(struct uart_port *port)
{
	release_mem_region(port->mapbase, MAP_SIZE);
}

static int exynos_serial_request_port(struct uart_port *port)
{
	const char *name = exynos_serial_portname(port);

	return request_mem_region(port->mapbase, MAP_SIZE, name) ? 0 : -EBUSY;
}

static void exynos_serial_config_port(struct uart_port *port, int flags)
{
	struct exynos_uart_info *info = exynos_port_to_info(port);

	if (flags & UART_CONFIG_TYPE &&
	    exynos_serial_request_port(port) == 0)
		port->type = info->type;
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int
exynos_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	struct exynos_uart_info *info = exynos_port_to_info(port);

	if (ser->type != PORT_UNKNOWN && ser->type != info->type)
		return -EINVAL;

	return 0;
}

static struct console exynos_serial_console;

static int exynos_serial_console_init(void)
{
	register_console(&exynos_serial_console);
	return 0;
}

#define EXYNOS_SERIAL_CONSOLE (&exynos_serial_console)

#if defined(CONFIG_SERIAL_SAMSUNG_CONSOLE) && defined(CONFIG_CONSOLE_POLL)
static int exynos_serial_get_poll_char(struct uart_port *port);
static void exynos_serial_put_poll_char(struct uart_port *port,
					unsigned char c);
#endif

static const struct uart_ops exynos_serial_ops = {
	.pm		= exynos_serial_pm,
	.tx_empty	= exynos_serial_tx_empty,
	.get_mctrl	= exynos_serial_get_mctrl,
	.set_mctrl	= exynos_serial_set_mctrl,
	.stop_tx	= exynos_serial_stop_tx,
	.start_tx	= exynos_serial_start_tx,
	.throttle       = exynos_serial_throttle,
	.unthrottle     = exynos_serial_unthrottle,
	.stop_rx	= exynos_serial_stop_rx,
	.break_ctl	= exynos_serial_break_ctl,
	.startup	= exynos_serial_startup,
	.shutdown	= exynos_serial_shutdown,
	.set_termios	= exynos_serial_set_termios,
	.type		= exynos_serial_type,
	.release_port	= exynos_serial_release_port,
	.request_port	= exynos_serial_request_port,
	.config_port	= exynos_serial_config_port,
	.verify_port	= exynos_serial_verify_port,
#if defined(CONFIG_SERIAL_SAMSUNG_CONSOLE) && defined(CONFIG_CONSOLE_POLL)
	.poll_get_char = exynos_serial_get_poll_char,
	.poll_put_char = exynos_serial_put_poll_char,
#endif
};

static struct uart_driver exynos_uart_drv = {
	.owner		= THIS_MODULE,
	.driver_name	= "s3c2410_serial",
	.nr		= CONFIG_SERIAL_EXYNOS_UARTS,
	.cons		= EXYNOS_SERIAL_CONSOLE,
	.dev_name	= EXYNOS_SERIAL_NAME,
	.major		= EXYNOS_SERIAL_MAJOR,
	.minor		= EXYNOS_SERIAL_MINOR,
};

#define __PORT_LOCK_UNLOCKED(i) \
	__SPIN_LOCK_UNLOCKED(exynos_serial_ports[i].port.lock)
static struct exynos_uart_port
exynos_serial_ports[CONFIG_SERIAL_EXYNOS_UARTS] = {
	[0] = {
		.port = {
			.lock		= __PORT_LOCK_UNLOCKED(0),
			.iotype		= UPIO_MEM,
			.uartclk	= 0,
			.fifosize	= 16,
			.ops		= &exynos_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 0,
		}
	},
	[1] = {
		.port = {
			.lock		= __PORT_LOCK_UNLOCKED(1),
			.iotype		= UPIO_MEM,
			.uartclk	= 0,
			.fifosize	= 16,
			.ops		= &exynos_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 1,
		}
	},
#if CONFIG_SERIAL_EXYNOS_UARTS > 2

	[2] = {
		.port = {
			.lock		= __PORT_LOCK_UNLOCKED(2),
			.iotype		= UPIO_MEM,
			.uartclk	= 0,
			.fifosize	= 16,
			.ops		= &exynos_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 2,
		}
	},
#endif
#if CONFIG_SERIAL_EXYNOS_UARTS > 3
	[3] = {
		.port = {
			.lock		= __PORT_LOCK_UNLOCKED(3),
			.iotype		= UPIO_MEM,
			.uartclk	= 0,
			.fifosize	= 16,
			.ops		= &exynos_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 3,
		}
	},
#endif
#if CONFIG_SERIAL_EXYNOS_UARTS > 4
	[4] = {
		.port = {
			.lock		= __PORT_LOCK_UNLOCKED(4),
			.iotype		= UPIO_MEM,
			.uartclk	= 0,
			.fifosize	= 16,
			.ops		= &exynos_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 4,
		}
	},
#endif
#if CONFIG_SERIAL_EXYNOS_UARTS > 5
	[5] = {
		.port = {
			.lock		= __PORT_LOCK_UNLOCKED(5),
			.iotype		= UPIO_MEM,
			.uartclk	= 0,
			.fifosize	= 16,
			.ops		= &exynos_serial_ops,
			.flags		= UPF_BOOT_AUTOCONF,
			.line		= 5,
		}
	},
#endif
};

static struct exynos_uart_port *exynos_serial_default_port(int port_index)
{
	exynos_serial_ports[port_index].port.lock =
		__PORT_LOCK_UNLOCKED(port_index);
	exynos_serial_ports[port_index].port.iotype = UPIO_MEM;
	exynos_serial_ports[port_index].port.uartclk = 0;
	exynos_serial_ports[port_index].port.fifosize = 0;
	exynos_serial_ports[port_index].port.ops =
		&exynos_serial_ops;
	exynos_serial_ports[port_index].port.flags = UPF_BOOT_AUTOCONF;
	exynos_serial_ports[port_index].port.line = port_index;

	return &exynos_serial_ports[port_index];
}

#undef __PORT_LOCK_UNLOCKED

static void exynos_usi_init(struct uart_port *port)
{
	struct exynos_uart_port *ourport = to_ourport(port);

	/* USI_RESET is active High signal.
	 * Reset value of USI_RESET is 'h1 to drive stable value to PAD.
	 * Due to this feature, the USI_RESET must be cleared (set as '0')
	 * before transaction starts.
	 */
	if (!ourport->dbg_uart_ch) {
		wr_regl(port, USI_CON, USI_SET_RESET);
		udelay(1);
	}

	wr_regl(port, USI_CON, USI_RESET);
	udelay(1);

	/* set the HWACG option bit in case of UART Rx mode.
	 * CLKREQ_ON = 1, CLKSTOP_ON = 0 (set USI_OPTION[2:1] = 2'h1)
	 */
	wr_regl(port, USI_OPTION, USI_HWACG_CLKREQ_ON);
}

static void exynos_usi_stop(struct uart_port *port)
{
	/* when USI CLKSTOP_ON is set high, this makes the
	 * Q-ch state enter into STOP state by driving both the
	 * IP_CLKREQ and IP_BUSACTREQ as low
	 */
	wr_regl(port, USI_OPTION, USI_HWACG_CLKSTOP_ON);
}

/* exynos_serial_resetport
 *
 * reset the fifos and other the settings.
 */

static void exynos_serial_resetport(struct uart_port *port,
				    struct s3c2410_uartcfg *cfg)
{
	struct exynos_uart_info *info = exynos_port_to_info(port);
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned long ucon = rd_regl(port, S3C2410_UCON);
	unsigned long umcon = rd_regl(port, S3C2410_UMCON);
	unsigned int ucon_mask;

	ucon_mask = info->clksel_mask;
	if (info->type == PORT_S3C2440)
		ucon_mask |= S3C2440_UCON0_DIVMASK;

	ucon &= ucon_mask;
	if (ourport->dbg_mode & UART_LOOPBACK_MODE) {
		dev_err(port->dev, "Change Loopback mode!\n");
		ucon |= S3C2443_UCON_LOOPBACK;
	}

	/* Disable RTS when RX FIFO contains 63 bytes */
	umcon &= ~S3C2412_UMCON_AFC_8;
	/* Set rts trigger level if declared */
	if (ourport->rts_trig_level && info->rts_trig_shift)
		umcon |= ourport->rts_trig_level << info->rts_trig_shift;
	wr_regl(port, S3C2410_UMCON, umcon);

	/* To prevent unexpected Interrupt before enabling the channel */
	wr_regl(port, S3C64XX_UINTM, 0xf);

	/* reset both fifos */
	wr_regl(port, S3C2410_UFCON, cfg->ufcon);

	/* some delay is required after fifo reset */
	udelay(1);

	wr_regl(port, S3C2410_UCON,  ucon | cfg->ucon);
}

/* exynos_serial_init_port
 *
 * initialise a single serial port from the platform device given
 */

static int exynos_serial_init_port(struct exynos_uart_port *ourport,
				   struct platform_device *platdev)
{
	struct uart_port *port = &ourport->port;
	struct s3c2410_uartcfg *cfg = ourport->cfg;
	struct resource *res;
	char clkname[MAX_CLK_NAME_LENGTH];
	int ret;

	dev_dbg(&platdev->dev, "%s: port=%p, platdev=%p\n", __func__, port,
		platdev);

	if (!platdev)
		return -ENODEV;

	if (port->mapbase != 0)
		return -EINVAL;

	/* setup info for port */
	port->dev	= &platdev->dev;
	ourport->pdev	= platdev;

	port->uartclk = 1;

	if (cfg->uart_flags & UPF_CONS_FLOW) {
		dev_dbg(port->dev, "%s: enabling flow control\n", __func__);
		port->flags |= UPF_CONS_FLOW;
	}

	/* sort our the physical and virtual addresses for each UART */

	res = platform_get_resource(platdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(port->dev, "failed to find memory resource for uart\n");
		return -EINVAL;
	}

	dev_dbg(port->dev, "resource %pR)\n", res);

	port->membase = devm_ioremap(port->dev, res->start, resource_size(res));
	if (!port->membase) {
		dev_err(port->dev, "failed to remap controller address\n");
		return -EBUSY;
	}

	port->mapbase = res->start;
	ret = platform_get_irq(platdev, 0);
	if (ret < 0) {
		port->irq = 0;
	} else {
		port->irq = ret;
		ourport->rx_irq = ret;
		ourport->tx_irq = ret + 1;
	}

	ret = platform_get_irq(platdev, 1);
	if (ret > 0)
		ourport->tx_irq = ret;
	/*
	 * DMA is currently supported only on DT platforms, if DMA properties
	 * are specified.
	 */
	if (platdev->dev.of_node && of_find_property(platdev->dev.of_node,
						     "dmas", NULL)) {
		ourport->dma = devm_kzalloc(port->dev,
					    sizeof(*ourport->dma),
					    GFP_KERNEL);
		if (!ourport->dma)
			return -ENOMEM;

		dev_info(port->dev, "DMA ENABLED\n");
	}

	if (of_property_read_u32(platdev->dev.of_node,
				 "samsung,source-clock-rate",
				 &ourport->src_clk_rate)) {
		dev_err(&platdev->dev, "No explicit src-clk. Use default src-clk\n");
		ourport->src_clk_rate = DEFAULT_SOURCE_CLK;
	}

	snprintf(clkname, sizeof(clkname), "ipclk_uart%d", ourport->port.line);
	ourport->clk = devm_clk_get(&platdev->dev, clkname);
	if (IS_ERR(ourport->clk)) {
		dev_err(port->dev, "%s: Controller clock not found\n",
			dev_name(&platdev->dev));
		return PTR_ERR(ourport->clk);
	}

	if (of_get_property(platdev->dev.of_node, "samsung,separate-uart-clk",
			    NULL)) {
		snprintf(clkname, sizeof(clkname), "gate_uart_clk%d",
			 ourport->port.line);
		ourport->separated_clk = devm_clk_get(&platdev->dev, clkname);
		if (IS_ERR(ourport->separated_clk)) {
			pr_err("%s: Controller clock not found\n",
			       dev_name(&platdev->dev));
			return PTR_ERR(ourport->separated_clk);
		}

		ret = clk_prepare_enable(ourport->separated_clk);
		if (ret) {
			dev_err(port->dev, "uart: clock failed to prepare+enable: %d\n",
				ret);
			return ret;
		}
	}

	ret = clk_prepare_enable(ourport->clk);
	if (ret) {
		pr_err("uart: clock failed to prepare+enable: %d\n", ret);
		return ret;
	}

	exynos_usi_init(port);

	/* Keep all interrupts masked and cleared */
	if (exynos_serial_has_interrupt_mask(port)) {
		wr_regl(port, S3C64XX_UINTM, 0xf);
		wr_regl(port, S3C64XX_UINTP, 0xf);
		wr_regl(port, S3C64XX_UINTSP, 0xf);
	}

	dev_dbg(port->dev, "port: map=%pa, mem=%p, irq=%d (%d,%d), clock=%u\n",
		&port->mapbase, port->membase, port->irq, ourport->rx_irq,
		ourport->tx_irq, port->uartclk);

	/* reset the fifos (and setup the uart) */
	exynos_serial_resetport(port, cfg);
	return 0;
}

/* Device driver serial port probe */

static const struct of_device_id exynos_uart_dt_match[];
static int probe_index;

static inline struct exynos_serial_drv_data *exynos_get_driver_data
(struct platform_device *pdev)
{
#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_node(exynos_uart_dt_match, pdev->dev.of_node);
		return (struct exynos_serial_drv_data *)match->data;
	}
#endif
	return (struct exynos_serial_drv_data *)
			platform_get_device_id(pdev)->driver_data;
}

static void exynos_serial_rx_fifo_wait(struct exynos_uart_port *ourport)
{
	struct uart_port *port = &ourport->port;
	unsigned int fifo_stat;
	unsigned long wait_time;
	unsigned int fifo_count;

	fifo_count = 0;

	fifo_stat = rd_regl(port, S3C2410_UFSTAT);
	fifo_count = exynos_serial_rx_fifocnt(ourport, fifo_stat);
	if (fifo_count) {
		exynos_clear_bit(port, S3C64XX_UINTM_RXD, S3C64XX_UINTM);
		ourport->rx_enabled = 1;

		wait_time = jiffies + HZ;
		do {
			port = &ourport->port;
			fifo_stat = rd_regl(port, S3C2410_UFSTAT);
			cpu_relax();
		} while (exynos_serial_rx_fifocnt(ourport, fifo_stat) &&
			 time_before(jiffies, wait_time));
	}

	if (ourport->rx_enabled)
		exynos_serial_stop_rx(port);
}

void exynos_serial_fifo_wait(void)
{
	struct exynos_uart_port *ourport;
	struct uart_port *port;
	unsigned int fifo_stat;
	unsigned long wait_time;

	list_for_each_entry(ourport, &drvdata_list, node) {
		wait_time = jiffies + HZ / 4;
		do {
			port = &ourport->port;
			fifo_stat = rd_regl(port, S3C2410_UFSTAT);
			cpu_relax();
		} while (exynos_serial_tx_fifocnt(ourport, fifo_stat) &&
			 time_before(jiffies, wait_time));
	}
}
EXPORT_SYMBOL_GPL(exynos_serial_fifo_wait);

static int exynos_serial_notifier(struct notifier_block *self,
				  unsigned long cmd, void *v)
{
	struct exynos_uart_port *ourport;
	struct uart_port *port;
	unsigned long flags;
	unsigned int umcon;

	switch (cmd) {
	case SICD_ENTER:
		list_for_each_entry(ourport, &drvdata_list, node) {
			port = &ourport->port;

			if (port->state->pm_state == UART_PM_STATE_OFF)
				continue;

			spin_lock_irqsave(&port->lock, flags);

			/* disable auto flow control & set nRTS for High */
			umcon = rd_regl(port, S3C2410_UMCON);
			umcon &= ~(S3C2410_UMCOM_AFC | S3C2410_UMCOM_RTS_LOW);
			wr_regl(port, S3C2410_UMCON, umcon);

			spin_unlock_irqrestore(&port->lock, flags);

			if (ourport->rts_control)
				change_uart_gpio(RTS_PINCTRL, ourport);
		}

		exynos_serial_fifo_wait();
		break;

	case SICD_EXIT:
		list_for_each_entry(ourport, &drvdata_list, node) {
			port = &ourport->port;

			if (port->state->pm_state == UART_PM_STATE_OFF)
				continue;

			spin_lock_irqsave(&port->lock, flags);

			if (!ourport->dbg_uart_ch) {
				/* enable auto flow control */
				umcon = rd_regl(port, S3C2410_UMCON);
				umcon |= S3C2410_UMCOM_AFC;
				wr_regl(port, S3C2410_UMCON, umcon);
			}

			spin_unlock_irqrestore(&port->lock, flags);

			if (ourport->rts_control)
				change_uart_gpio(DEFAULT_PINCTRL, ourport);
		}
		break;

	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block exynos_serial_notifier_block = {
	.notifier_call = exynos_serial_notifier,
};

static int exynos_serial_probe(struct platform_device *pdev)
{
	struct exynos_uart_port *ourport;
	struct device *dev = &pdev->dev;
	int index = probe_index;
	int ret, fifo_size, prop = 0;
	int port_index = probe_index;
	int rts_trig_level;
	char logbuf_port[6];

	dev_dbg(&pdev->dev, "%s %d\n", __func__, index);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		ret = of_alias_get_id(pdev->dev.of_node, "uart");
		if (ret < 0) {
			dev_err(&pdev->dev, "UART aliases are not defined(%d).\n",
				ret);
		} else {
			port_index = ret;
		}
	}
	ourport = &exynos_serial_ports[port_index];

	if (of_property_read_u32(dev->of_node, "samsung,usi-offset",
				 &ourport->usi_offset)) {
		dev_warn(dev, "usi offset is not specified\n");
	}

	ourport->usi_reg = syscon_regmap_lookup_by_phandle(dev->of_node,
							   "samsung,usi-phandle");
	if (IS_ERR(ourport->usi_reg)) {
		dev_info(dev, "no lookup for usi-phandle.\n");
	} else {
		u32 val;

		regmap_update_bits(ourport->usi_reg, ourport->usi_offset,
				   USI_SW_CONF_MASK, USI_UART_SW_CONF);
		regmap_read(ourport->usi_reg, ourport->usi_offset, &val);
		dev_info(&pdev->dev, "set usi mode - 0x%x\n", val);
	}

	if (ourport->port.line != port_index)
		ourport = exynos_serial_default_port(port_index);

	if (ourport->port.line >= CONFIG_SERIAL_EXYNOS_UARTS) {
		dev_err(&pdev->dev,
			"the port %d exceeded CONFIG_SERIAL_EXYNOS_UARTS(%d)\n"
			, ourport->port.line, CONFIG_SERIAL_EXYNOS_UARTS);
		return -EINVAL;
	}

	ourport->drv_data = exynos_get_driver_data(pdev);
	if (!ourport->drv_data) {
		dev_err(&pdev->dev, "could not find driver data\n");
		return -ENODEV;
	}

	ourport->baudclk = ERR_PTR(-EINVAL);
	ourport->info = ourport->drv_data->info;
	ourport->cfg = (dev_get_platdata(&pdev->dev)) ?
			dev_get_platdata(&pdev->dev) :
			ourport->drv_data->def_cfg;

	ourport->port.fifosize = (ourport->info->fifosize) ?
		ourport->info->fifosize :
		ourport->drv_data->fifosize[port_index];
	ourport->port.has_sysrq = IS_ENABLED(CONFIG_SERIAL_SAMSUNG_CONSOLE);

	if (of_get_property(pdev->dev.of_node, "samsung,uart-panic-log", NULL))
		ourport->uart_panic_log = 1;
	else
		ourport->uart_panic_log = 0;

	if (ourport->uart_panic_log) {
		atomic_notifier_chain_register(&panic_notifier_list,
					       &exynos_uart_panic_block);
		panic_port = ourport;
	}

	if (of_get_property(pdev->dev.of_node, "samsung,usi-serial-v2", NULL))
		ourport->usi_v2 = 1;
	else
		ourport->usi_v2 = 0;

	if (of_get_property(pdev->dev.of_node, "samsung,dbg-uart-ch", NULL))
		ourport->dbg_uart_ch = 1;
	else
		ourport->dbg_uart_ch = 0;

	if (ourport->dbg_uart_ch == 1) {
		if (of_property_read_u32(pdev->dev.of_node,
					 "samsung,dbg-uart-baud",
					 &ourport->dbg_uart_baud)) {
			ourport->dbg_uart_baud = 115200;
			dev_err(&pdev->dev, "No DBG baud rate value. Use 115200 Baud rate\n");
		}
		if (of_property_read_u32(pdev->dev.of_node,
					 "samsung,dbg-word-len",
					 &ourport->dbg_word_len)) {
			ourport->dbg_word_len = 8;
			dev_err(&pdev->dev, "No DBG word length value. Use 8 word length\n");
		}
	}

	if (of_get_property(pdev->dev.of_node, "samsung,rts-gpio-control",
			    NULL)) {
		ourport->rts_control = RTS_CTRL_SUPPORT;
		ourport->pinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(ourport->pinctrl)) {
			dev_err(&pdev->dev, "Can't get uart pinctrl!!!\n");
		} else {
			ourport->uart_pinctrl_rts =
				pinctrl_lookup_state(ourport->pinctrl, "rts");
			if (IS_ERR(ourport->uart_pinctrl_rts))
				dev_err(&pdev->dev,
					"Can't get RTS pinstate!!!\n");

			ourport->uart_pinctrl_tx_dat =
				pinctrl_lookup_state(ourport->pinctrl, "tx_dat");
			if (IS_ERR(ourport->uart_pinctrl_tx_dat))
				dev_err(&pdev->dev,
					"Can't get TX_DAT pinstate!!!\n");

			ourport->uart_pinctrl_default =
				pinctrl_lookup_state(ourport->pinctrl,
						     "default");
			if (IS_ERR(ourport->uart_pinctrl_default))
				dev_err(&pdev->dev,
					"Can't get Default pinstate!!!\n");
		}
	}
	if (!of_property_read_u32(pdev->dev.of_node, "samsung,rts-trig-level",
				  &rts_trig_level)) {
		ourport->rts_trig_level = rts_trig_level;
	}

	if (!of_property_read_u32(pdev->dev.of_node, "samsung,fifo-size",
				  &fifo_size)) {
		ourport->port.fifosize = fifo_size;
		ourport->info->fifosize = fifo_size;
	} else {
		dev_err(&pdev->dev, "Please add FIFO size in device tree!(UART%d)\n",
			port_index);
		return -EINVAL;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				 "reg-io-width", &prop) == 0) {
		switch (prop) {
		case 1:
			ourport->port.iotype = UPIO_MEM;
			break;
		case 4:
			ourport->port.iotype = UPIO_MEM32;
			break;
		default:
			dev_warn(&pdev->dev, "unsupported reg-io-width (%d)\n",
				 prop);
			ret = -EINVAL;
			break;
		}
	}

	/*
	 * DMA transfers must be aligned at least to cache line size,
	 * so find minimal transfer size suitable for DMA mode
	 */
	ourport->min_dma_size = max_t(int, ourport->port.fifosize,
				      dma_get_cache_alignment());

	probe_index++;

	pr_debug("%s: initialising port %p...\n", __func__, ourport);

	if (of_get_property(pdev->dev.of_node, "samsung,in-band-wakeup", NULL))
		ourport->in_band_wakeup = 1;
	else
		ourport->in_band_wakeup = 0;

	if (of_get_property(pdev->dev.of_node, "samsung,uart-logging", NULL))
		ourport->uart_logging = 1;
	else
		ourport->uart_logging = 0;

	if (of_find_property(pdev->dev.of_node,
			     "samsung,use-default-irq", NULL))
		ourport->use_default_irq = 1;
	else
		ourport->use_default_irq = 0;

	ret = exynos_serial_init_port(ourport, pdev);
	if (ret < 0)
		return ret;

	if (!exynos_uart_drv.state) {
		ret = uart_register_driver(&exynos_uart_drv);
		if (ret < 0) {
			pr_err("Failed to register Samsung UART driver\n");
			return ret;
		}
	}

	pr_debug("%s: adding port\n", __func__);
	uart_add_one_port(&exynos_uart_drv, &ourport->port);
	platform_set_drvdata(pdev, &ourport->port);

	/*
	 * Deactivate the clock enabled in exynos_serial_init_port here,
	 * so that a potential re-enablement through the pm-callback overlaps
	 * and keeps the clock enabled in this case.
	 */
	uart_clock_disable(ourport);

	list_add_tail(&ourport->node, &drvdata_list);

	ret = device_create_file(&pdev->dev, &dev_attr_uart_dbg);
	if (ret < 0)
		dev_err(&pdev->dev, "failed to create sysfs file.\n");

	ourport->dbg_mode = 0;

	if (ourport->uart_logging == 1) {
		/* Allocate memory for UART logging */
		ourport->uart_local_buf.buffer = kzalloc(LOG_BUFFER_SIZE,
							 GFP_KERNEL);

		if (!ourport->uart_local_buf.buffer) {
			dev_err(&pdev->dev, "could not allocate buffer for UART logging\n");
			ourport->uart_logging = 0;
		} else {
			ourport->uart_local_buf.size = LOG_BUFFER_SIZE;
			ourport->uart_local_buf.index = 0;
		}
	}

	if (ourport->uart_logging == 1) {
		snprintf(logbuf_port, sizeof(logbuf_port), "tty%d", port_index);
		ourport->log = logbuffer_register(logbuf_port);
		if (IS_ERR_OR_NULL(ourport->log)) {
			dev_err(&pdev->dev, "probe: logbuffer get failed\n");
			ourport->log = NULL;
		}
	}

	return 0;
}

static int exynos_serial_remove(struct platform_device *dev)
{
	struct uart_port *port = exynos_dev_to_port(&dev->dev);

	struct exynos_uart_port *ourport = to_ourport(port);

	if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log)) {
		logbuffer_unregister(ourport->log);
	}

	if (port) {
		if (ourport->uart_logging == 1)
			kfree(ourport->uart_local_buf.buffer);
		uart_remove_one_port(&exynos_uart_drv, port);
	}

	uart_unregister_driver(&exynos_uart_drv);

	return 0;
}

/* UART power management code */
#ifdef CONFIG_PM_SLEEP
static int exynos_serial_suspend(struct device *dev)
{
	struct uart_port *port = exynos_dev_to_port(dev);
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned int ucon;

	if (port) {
		/*
		 * If rts line must be protected while suspending
		 * we change the gpio pad as output high
		 */
		if (ourport->rts_control)
			change_uart_gpio(RTS_PINCTRL, ourport);

		usleep_range(200, 300);//delay for sfr update
		exynos_serial_rx_fifo_wait(ourport);

		if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
			logbuffer_log(ourport->log, "suspend: uart_suspend_port");
		uart_suspend_port(&exynos_uart_drv, port);
		if (!ourport->dbg_uart_ch || console_suspend_enabled) {
			uart_clock_enable(ourport);
			/* disable Tx, Rx mode bit for suspend in case of HWACG */
			ucon = rd_regl(port, S3C2410_UCON);
			ucon &= ~(S3C2410_UCON_RXIRQMODE | S3C2410_UCON_TXIRQMODE);
			wr_regl(port, S3C2410_UCON, ucon);
			exynos_usi_stop(port);
			uart_clock_disable(ourport);
			ourport->rx_enabled = 0;
			ourport->tx_enabled = 0;
		} else {
			ucon = rd_regl(port, S3C2410_UCON);
			ucon &= ~S3C2410_UCON_RXIRQMODE;
			wr_regl(port, S3C2410_UCON, ucon);

			ourport->rx_enabled = 0;
		}
		if (ourport->dbg_mode & UART_DBG_MODE)
			dev_err(dev, "UART suspend notification for tty framework.\n");
	}

	return 0;
}

static int exynos_serial_suspend_noirq(struct device *dev)
{
	struct uart_port *port = exynos_dev_to_port(dev);
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned int ucon;

	if (ourport->dbg_uart_ch && !console_suspend_enabled) {
		uart_clock_enable(ourport);
		/* disable Tx, Rx mode bit for suspend in case of HWACG */
		ucon = rd_regl(port, S3C2410_UCON);
		ucon &= ~S3C2410_UCON_TXIRQMODE;
		wr_regl(port, S3C2410_UCON, ucon);
		exynos_usi_stop(port);
		uart_clock_disable(ourport);

		ourport->tx_enabled = 0;
	}

	return 0;
}

static int exynos_serial_resume(struct device *dev)
{
	struct uart_port *port = exynos_dev_to_port(dev);
	struct exynos_uart_port *ourport = to_ourport(port);

	if (port) {
		if (!IS_ERR(ourport->usi_reg))
			regmap_update_bits(ourport->usi_reg,
					   ourport->usi_offset,
					   USI_SW_CONF_MASK, USI_UART_SW_CONF);

		if ((ourport->dbg_uart_ch) && (!console_suspend_enabled)) {
			uart_clock_enable(ourport);
			exynos_usi_init(port);
		}

		if (ourport->uart_logging && !IS_ERR_OR_NULL(ourport->log))
			logbuffer_log(ourport->log, "resume: uart_resume_port");

		uart_resume_port(&exynos_uart_drv, port);

		if ((ourport->dbg_uart_ch) && (!console_suspend_enabled)) {
			exynos_serial_resetport(port,
					exynos_port_to_cfg(port));
			__clear_bit(S3C64XX_UINTM_RXD,
					portaddrl(port, S3C64XX_UINTM));
		}

		if (ourport->rts_control)
			change_uart_gpio(DEFAULT_PINCTRL, ourport);

		if (ourport->dbg_mode & UART_DBG_MODE)
			dev_err(dev, "UART resume notification for tty framework.\n");
	}

	return 0;
}

static int exynos_serial_resume_noirq(struct device *dev)
{
	struct uart_port *port = exynos_dev_to_port(dev);
	struct exynos_uart_port *ourport = to_ourport(port);

	if (port) {
		/* restore IRQ mask */
		if (exynos_serial_has_interrupt_mask(port)) {
			unsigned int uintm = 0xf;

			if (ourport->tx_enabled)
				uintm &= ~S3C64XX_UINTM_TXD_MSK;
			if (ourport->rx_enabled)
				uintm &= ~S3C64XX_UINTM_RXD_MSK;
			uart_clock_enable(ourport);
			wr_regl(port, S3C64XX_UINTM, uintm);
			uart_clock_disable(ourport);
		}
	}

	return 0;
}

static const struct dev_pm_ops exynos_serial_pm_ops = {
	.suspend = exynos_serial_suspend,
	.suspend_noirq = exynos_serial_suspend_noirq,
	.resume = exynos_serial_resume,
	.resume_noirq = exynos_serial_resume_noirq,
};

#define SERIAL_EXYNOS_PM_OPS	(&exynos_serial_pm_ops)

#else /* !CONFIG_PM_SLEEP */

#define SERIAL_EXYNOS_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

/* Console code */

static struct uart_port *cons_uart;

static int
exynos_serial_console_txrdy(struct uart_port *port, unsigned int ufcon)
{
	struct exynos_uart_info *info = exynos_port_to_info(port);
	unsigned long ufstat, utrstat;

	if (ufcon & S3C2410_UFCON_FIFOMODE) {
		/* fifo mode - check amount of data in fifo registers... */

		ufstat = rd_regl(port, S3C2410_UFSTAT);
		return (ufstat & info->tx_fifofull) ? 0 : 1;
	}

	/* in non-fifo mode, we go and use the tx buffer empty */

	utrstat = rd_regl(port, S3C2410_UTRSTAT);
	return (utrstat & S3C2410_UTRSTAT_TXE) ? 1 : 0;
}

static bool
exynos_port_configured(unsigned int ucon)
{
	/* consider the serial port configured if the tx/rx mode set */
	return (ucon & 0xf) != 0;
}

#ifdef CONFIG_CONSOLE_POLL
/*
 * Console polling routines for writing and reading from the uart while
 * in an interrupt or debug context.
 */

static int exynos_serial_get_poll_char(struct uart_port *port)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned int ufstat;

	ufstat = rd_regl(port, S3C2410_UFSTAT);
	if (exynos_serial_rx_fifocnt(ourport, ufstat) == 0)
		return NO_POLL_CHAR;

	return rd_reg(port, S3C2410_URXH);
}

static void exynos_serial_put_poll_char(struct uart_port *port,
					unsigned char c)
{
	unsigned int ufcon = rd_regl(port, S3C2410_UFCON);
	unsigned int ucon = rd_regl(port, S3C2410_UCON);

	/* not possible to xmit on unconfigured port */
	if (!exynos_port_configured(ucon))
		return;

	while (!exynos_serial_console_txrdy(port, ufcon))
		cpu_relax();
	wr_reg(port, S3C2410_UTXH, c);
}

#endif /* CONFIG_CONSOLE_POLL */

static void
exynos_serial_console_putchar(struct uart_port *port, int ch)
{
	unsigned int ufcon = rd_regl(port, S3C2410_UFCON);
	unsigned int ucon;

	while (1) {
		ucon = rd_regl(port, S3C2410_UCON);
		/* not possible to xmit on unconfigured port */
		if (!exynos_port_configured(ucon))
			return;

		if (exynos_serial_console_txrdy(port, ufcon))
			break;

		cpu_relax();
	}
	wr_reg(port, S3C2410_UTXH, ch);
}

static void
exynos_serial_console_write(struct console *co, const char *s,
			    unsigned int count)
{
	unsigned int ucon = rd_regl(cons_uart, S3C2410_UCON);

	/* not possible to xmit on unconfigured port */
	if (!exynos_port_configured(ucon))
		return;

	uart_console_write(cons_uart, s, count, exynos_serial_console_putchar);
}

static void __init
exynos_serial_get_options(struct uart_port *port, int *baud, int *parity,
			  int *bits)
{
	struct exynos_uart_port *ourport = to_ourport(port);
	unsigned int ulcon;
	unsigned int ucon;
	unsigned int ubrdiv;
	unsigned long rate;

	ulcon  = rd_regl(port, S3C2410_ULCON);
	ucon   = rd_regl(port, S3C2410_UCON);
	ubrdiv = rd_regl(port, S3C2410_UBRDIV);

	dev_dbg(&ourport->pdev->dev, "%s: port=%p\n"
		"registers: ulcon=%08x, ucon=%08x, ubdriv=%08x\n",
		__func__, port, ulcon, ucon, ubrdiv);

	if (exynos_port_configured(ucon)) {
		switch (ulcon & S3C2410_LCON_CSMASK) {
		case S3C2410_LCON_CS5:
			*bits = 5;
			break;
		case S3C2410_LCON_CS6:
			*bits = 6;
			break;
		case S3C2410_LCON_CS7:
			*bits = 7;
			break;
		case S3C2410_LCON_CS8:
		default:
			*bits = 8;
			break;
		}

		switch (ulcon & S3C2410_LCON_PMASK) {
		case S3C2410_LCON_PEVEN:
			*parity = 'e';
			break;

		case S3C2410_LCON_PODD:
			*parity = 'o';
			break;

		case S3C2410_LCON_PNONE:
		default:
			*parity = 'n';
		}

		/* now calculate the baud rate */
		rate = clk_get_rate(ourport->clk);

		*baud = rate / (16 * (ubrdiv + 1));
		dev_dbg(&ourport->pdev->dev, "calculated baud %d\n", *baud);
	}
}

static int __init
exynos_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	pr_debug("%s: co=%p (%d), %s\n", __func__, co, co->index, options);

	/* is this a valid port */

	if (co->index == -1 || co->index >= CONFIG_SERIAL_EXYNOS_UARTS)
		co->index = 0;

	port = &exynos_serial_ports[co->index].port;

	/* is the port configured? */

	if (port->mapbase == 0x0)
		return -ENODEV;

	cons_uart = port;

	pr_debug("%s: port=%p (%d)\n", __func__, port, co->index);

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		exynos_serial_get_options(port, &baud, &parity, &bits);

	pr_debug("%s: baud %d\n", __func__, baud);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console exynos_serial_console = {
	.name		= EXYNOS_SERIAL_NAME,
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.write		= exynos_serial_console_write,
	.setup		= exynos_serial_console_setup,
	.data		= &exynos_uart_drv,
};

static struct exynos_serial_drv_data exynos_serial_drv_data = {
	.info = &(struct exynos_uart_info) {
		.name		= "Samsung Exynos UART",
		.type		= PORT_S3C6400,
		.has_divslot	= 1,
		.rx_fifomask	= S5PV210_UFSTAT_RXMASK,
		.rx_fifoshift	= S5PV210_UFSTAT_RXSHIFT,
		.rx_fifofull	= S5PV210_UFSTAT_RXFULL,
		.tx_fifofull	= S5PV210_UFSTAT_TXFULL,
		.tx_fifomask	= S5PV210_UFSTAT_TXMASK,
		.tx_fifoshift	= S5PV210_UFSTAT_TXSHIFT,
		.rts_trig_shift = S5PV210_UMCON_RTSTRIG_SHIFT,
		.def_clk_sel	= S3C2410_UCON_CLKSEL0,
		.num_clks	= 1,
		.clksel_mask	= 0,
		.clksel_shift	= 0,
	},
	.def_cfg = &(struct s3c2410_uartcfg) {
		.ucon		= S5PV210_UCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
		.has_fracval	= 1,
	},
	.fifosize = { 0, },
};

#define EXYNOS_SERIAL_DRV_DATA ((kernel_ulong_t)&exynos_serial_drv_data)

static const struct platform_device_id exynos_serial_driver_ids[] = {
	{
		.name		= "exynos-uart",
		.driver_data	= EXYNOS_SERIAL_DRV_DATA,
	},
	{ },
};
MODULE_DEVICE_TABLE(platform, exynos_serial_driver_ids);

#ifdef CONFIG_OF
static const struct of_device_id exynos_uart_dt_match[] = {
	{ .compatible = "samsung,exynos-uart",
		.data = (void *)EXYNOS_SERIAL_DRV_DATA },
	{},
};
MODULE_DEVICE_TABLE(of, exynos_uart_dt_match);
#endif

static struct platform_driver exynos_serial_driver = {
	.probe		= exynos_serial_probe,
	.remove		= exynos_serial_remove,
	.id_table	= exynos_serial_driver_ids,
	.driver		= {
		.name	= "exynos-uart",
		.pm	= SERIAL_EXYNOS_PM_OPS,
		.of_match_table	= of_match_ptr(exynos_uart_dt_match),
	},
};

/* module initialisation code */

static int __init exynos_serial_modinit(void)
{
	int ret;

	ret = exynos_serial_console_init();
	if (ret < 0) {
		pr_err("Failed to init exynos console\n");
		return ret;
	}

	ret = uart_register_driver(&exynos_uart_drv);
	if (ret < 0) {
		pr_err("Failed to register Samsung UART driver\n");
		return ret;
	}

	exynos_cpupm_notifier_register(&exynos_serial_notifier_block);

	return platform_driver_register(&exynos_serial_driver);
}

static void __exit exynos_serial_modexit(void)
{
	platform_driver_unregister(&exynos_serial_driver);
	uart_unregister_driver(&exynos_uart_drv);
}

module_init(exynos_serial_modinit);
module_exit(exynos_serial_modexit);

MODULE_ALIAS("platform:exynos-uart");
MODULE_DESCRIPTION("Exynos SoC Serial port driver");
MODULE_LICENSE("GPL v2");
