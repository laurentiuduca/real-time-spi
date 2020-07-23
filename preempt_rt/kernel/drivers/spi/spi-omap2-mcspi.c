// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * OMAP2 McSPI controller driver
 *
 * Copyright (C) 2020, Laurentiu-Cristian Duca
 *		laurentiu [dot] duca [at] gmail [dot] com
 *		real-time driver using interrupts
 *
 * Copyright (C) 2005, 2006 Nokia Corporation
 * Author:	Samuel Ortiz <samuel.ortiz@nokia.com> and
 *		Juha Yrj�l� <juha.yrjola@nokia.com>
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gcd.h>
#include <linux/iopoll.h>
#include <linux/swait.h>
#include <linux/spinlock_types.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>

#include <linux/platform_data/spi-omap2-mcspi.h>

#define OMAP2_MCSPI_MAX_FREQ		48000000
#define OMAP2_MCSPI_MAX_DIVIDER		4096
#define OMAP2_MCSPI_MAX_FIFODEPTH	64
#define OMAP2_MCSPI_MAX_FIFOWCNT	0xFFFF
#define SPI_AUTOSUSPEND_TIMEOUT		2000

#define OMAP2_MCSPI_REVISION		0x00
#define OMAP2_MCSPI_SYSCONFIG		0x10
#define OMAP2_MCSPI_SYSSTATUS		0x14
#define OMAP2_MCSPI_IRQSTATUS		0x18
#define OMAP2_MCSPI_IRQENABLE		0x1c
#define OMAP2_MCSPI_WAKEUPENABLE	0x20
#define OMAP2_MCSPI_SYST		0x24
#define OMAP2_MCSPI_MODULCTRL		0x28
#define OMAP2_MCSPI_XFERLEVEL		0x7c

/* per-channel banks, 0x14 bytes each, first is: */
#define OMAP2_MCSPI_CHCONF0		0x2c
#define OMAP2_MCSPI_CHSTAT0		0x30
#define OMAP2_MCSPI_CHCTRL0		0x34
#define OMAP2_MCSPI_TX0			0x38
#define OMAP2_MCSPI_RX0			0x3c

/* per-register bitmasks: */
#define OMAP2_MCSPI_IRQSTATUS_EOW		BIT(17)
#define OMAP2_MCSPI_IRQSTATUS_RX1_FULL  BIT(6)
#define OMAP2_MCSPI_IRQSTATUS_TX1_EMPTY	BIT(4)
#define OMAP2_MCSPI_IRQSTATUS_RX0_FULL  BIT(2)
#define OMAP2_MCSPI_IRQSTATUS_TX0_EMPTY	BIT(0)

#define OMAP2_MCSPI_IRQENABLE_EOW		BIT(17)
#define OMAP2_MCSPI_IRQENABLE_RX1_FULL  BIT(6)
#define OMAP2_MCSPI_IRQENABLE_TX1_EMPTY	BIT(4)
#define OMAP2_MCSPI_IRQENABLE_RX0_FULL  BIT(2)
#define OMAP2_MCSPI_IRQENABLE_TX0_UNDERFLOW BIT(1)
#define OMAP2_MCSPI_IRQENABLE_TX0_EMPTY	BIT(0)

#define OMAP2_MCSPI_MODULCTRL_SINGLE	BIT(0)
#define OMAP2_MCSPI_MODULCTRL_MS	BIT(2)
#define OMAP2_MCSPI_MODULCTRL_STEST	BIT(3)

#define OMAP2_MCSPI_CHCONF_PHA		BIT(0)
#define OMAP2_MCSPI_CHCONF_POL		BIT(1)
#define OMAP2_MCSPI_CHCONF_CLKD_MASK	(0x0f << 2)
#define OMAP2_MCSPI_CHCONF_EPOL		BIT(6)
#define OMAP2_MCSPI_CHCONF_WL_MASK	(0x1f << 7)
#define OMAP2_MCSPI_CHCONF_TRM_RX_ONLY	BIT(12)
#define OMAP2_MCSPI_CHCONF_TRM_TX_ONLY	BIT(13)
#define OMAP2_MCSPI_CHCONF_TRM_MASK	(0x03 << 12)
#define OMAP2_MCSPI_CHCONF_DMAW		BIT(14)
#define OMAP2_MCSPI_CHCONF_DMAR		BIT(15)
#define OMAP2_MCSPI_CHCONF_DPE0		BIT(16)
#define OMAP2_MCSPI_CHCONF_DPE1		BIT(17)
#define OMAP2_MCSPI_CHCONF_IS		BIT(18)
#define OMAP2_MCSPI_CHCONF_TURBO	BIT(19)
#define OMAP2_MCSPI_CHCONF_FORCE	BIT(20)
#define OMAP2_MCSPI_CHCONF_FFET		BIT(27)
#define OMAP2_MCSPI_CHCONF_FFER		BIT(28)
#define OMAP2_MCSPI_CHCONF_CLKG		BIT(29)

#define OMAP2_MCSPI_CHSTAT_RXS		BIT(0)
#define OMAP2_MCSPI_CHSTAT_TXS		BIT(1)
#define OMAP2_MCSPI_CHSTAT_EOT		BIT(2)
#define OMAP2_MCSPI_CHSTAT_TXFFE	BIT(3)

#define OMAP2_MCSPI_CHCTRL_EN		BIT(0)
#define OMAP2_MCSPI_CHCTRL_EXTCLK_MASK	(0xff << 8)

#define OMAP2_MCSPI_WAKEUPENABLE_WKEN	BIT(0)

#define OMAP2_MCSPI_SYSCONFIG_CLOCKACTIVITY_MASK	(0x3 << 8)
#define OMAP2_MCSPI_SYSCONFIG_SIDLEMODE_MASK		(0x3 << 3)
#define OMAP2_MCSPI_SYSCONFIG_SOFTRESET				BIT(1)
#define OMAP2_MCSPI_SYSCONFIG_AUTOIDLE				BIT(0)

#define OMAP2_MCSPI_SYSSTATUS_RESETDONE BIT(0)

#define PM_NEGATIVE_DELAY	-2000

/* We have 2 DMA channels per CS, one for RX and one for TX */
struct omap2_mcspi_dma {
	struct dma_chan *dma_tx;
	struct dma_chan *dma_rx;

	struct completion dma_tx_completion;
	struct completion dma_rx_completion;

	char dma_rx_ch_name[14];
	char dma_tx_ch_name[14];
};

/* use PIO for small transfers, avoiding DMA setup/teardown overhead and
 * cache operations; better heuristics consider wordsize and bitrate.
 */
#define DMA_MIN_BYTES			160


/*
 * Used for context save and restore, structure members to be updated whenever
 * corresponding registers are modified.
 */
struct omap2_mcspi_regs {
	u32 modulctrl;
	u32 wakeupenable;
	struct list_head cs;
};

struct omap2_mcspi {
	struct completion	txdone;
	struct spi_master	*master;
	/* Virtual base address of the controller */
	void __iomem		*base;
	unsigned long		phys;
	/* SPI1 has 4 channels, while SPI2 has 2 */
	struct omap2_mcspi_dma	*dma_channels;
	struct device		*dev;
	struct omap2_mcspi_regs ctx;
	int			fifo_depth;
	bool			slave_aborted;
	unsigned int		pin_dir:1;
	struct swait_queue_head swait;
	raw_spinlock_t lock;
	int interrupt_done;
	unsigned char *rx_buf;
	const unsigned char *tx_buf;
	int rx_len, tx_len;
	/* statistics */
	int n_interrupts, n_rx_full, n_tx_empty;
	/* access to the current SPI device address
	 * which may be spidev0.0, spidev1.0 or spidev1.1
	 */
	struct spi_device *spi;
};

struct omap2_mcspi_cs {
	void __iomem		*base;
	unsigned long		phys;
	int			word_len;
	u16			mode;
	struct list_head	node;
	/* Context save and restore shadow register */
	u32			chconf0, chctrl0;
};

static inline void mcspi_write_reg(struct spi_master *master,
		int idx, u32 val)
{
	struct omap2_mcspi *mcspi = spi_master_get_devdata(master);

	writel_relaxed(val, mcspi->base + idx);
}

static inline u32 mcspi_read_reg(struct spi_master *master, int idx)
{
	struct omap2_mcspi *mcspi = spi_master_get_devdata(master);

	return readl_relaxed(mcspi->base + idx);
}

static inline void mcspi_write_cs_reg(const struct spi_device *spi,
		int idx, u32 val)
{
	struct omap2_mcspi_cs	*cs = spi->controller_state;

	writel_relaxed(val, cs->base +  idx);
}

static inline u32 mcspi_read_cs_reg(const struct spi_device *spi, int idx)
{
	struct omap2_mcspi_cs	*cs = spi->controller_state;

	return readl_relaxed(cs->base + idx);
}

static inline u32 mcspi_cached_chconf0(const struct spi_device *spi)
{
	struct omap2_mcspi_cs *cs = spi->controller_state;

	return cs->chconf0;
}

static inline void mcspi_write_chconf0(const struct spi_device *spi, u32 val)
{
	struct omap2_mcspi_cs *cs = spi->controller_state;

	cs->chconf0 = val;
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCONF0, val);
	mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCONF0);
}

static inline int mcspi_bytes_per_word(int word_len)
{
	if (word_len <= 8)
		return 1;
	else if (word_len <= 16)
		return 2;
	else /* word_len <= 32 */
		return 4;
}

static void omap2_mcspi_set_dma_req(const struct spi_device *spi,
		int is_read, int enable)
{
	u32 l, rw;

	l = mcspi_cached_chconf0(spi);

	if (is_read) /* 1 is read, 0 write */
		rw = OMAP2_MCSPI_CHCONF_DMAR;
	else
		rw = OMAP2_MCSPI_CHCONF_DMAW;

	if (enable)
		l |= rw;
	else
		l &= ~rw;

	mcspi_write_chconf0(spi, l);
}

static void omap2_mcspi_set_enable(const struct spi_device *spi, int enable)
{
	struct omap2_mcspi_cs *cs = spi->controller_state;
	u32 l;

	l = cs->chctrl0;
	if (enable)
		l |= OMAP2_MCSPI_CHCTRL_EN;
	else
		l &= ~OMAP2_MCSPI_CHCTRL_EN;
	cs->chctrl0 = l;
	mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCTRL0, cs->chctrl0);
	/* Flash post-writes */
	mcspi_read_cs_reg(spi, OMAP2_MCSPI_CHCTRL0);
}

static void omap2_mcspi_set_cs(struct spi_device *spi, bool enable)
{
	u32 l;

	/* The controller handles the inverted chip selects
	 * using the OMAP2_MCSPI_CHCONF_EPOL bit so revert
	 * the inversion from the core spi_set_cs function.
	 */
	if (spi->mode & SPI_CS_HIGH)
		enable = !enable;

	if (spi->controller_state) {
		l = mcspi_cached_chconf0(spi);

		if (enable)
			l &= ~OMAP2_MCSPI_CHCONF_FORCE;
		else
			l |= OMAP2_MCSPI_CHCONF_FORCE;

		mcspi_write_chconf0(spi, l);

	}
}

static void omap2_mcspi_set_mode(struct spi_master *master)
{
	struct omap2_mcspi	*mcspi = spi_master_get_devdata(master);
	struct omap2_mcspi_regs	*ctx = &mcspi->ctx;
	u32 l;

	/*
	 * Choose master or slave mode
	 */
	l = mcspi_read_reg(master, OMAP2_MCSPI_MODULCTRL);
	l &= ~(OMAP2_MCSPI_MODULCTRL_STEST);
	if (spi_controller_is_slave(master)) {
		l |= (OMAP2_MCSPI_MODULCTRL_MS);
	} else {
		l &= ~(OMAP2_MCSPI_MODULCTRL_MS);
		l |= OMAP2_MCSPI_MODULCTRL_SINGLE;
	}
	mcspi_write_reg(master, OMAP2_MCSPI_MODULCTRL, l);

	ctx->modulctrl = l;
}

static void omap2_mcspi_set_fifo(const struct spi_device *spi,
				struct spi_transfer *t, int enable)
{
	struct spi_master *master = spi->master;
	struct omap2_mcspi_cs *cs = spi->controller_state;
	struct omap2_mcspi *mcspi;
	unsigned int wcnt;
	int max_fifo_depth, bytes_per_word;
	u32 chconf, xferlevel;

	mcspi = spi_master_get_devdata(master);

	chconf = mcspi_cached_chconf0(spi);
	if (enable) {
		bytes_per_word = mcspi_bytes_per_word(cs->word_len);
		if (t->len % bytes_per_word != 0)
			goto disable_fifo;

		if (t->rx_buf != NULL && t->tx_buf != NULL)
			max_fifo_depth = OMAP2_MCSPI_MAX_FIFODEPTH / 2;
		else
			max_fifo_depth = OMAP2_MCSPI_MAX_FIFODEPTH;

		wcnt = t->len / bytes_per_word;
		if (wcnt > OMAP2_MCSPI_MAX_FIFOWCNT)
			goto disable_fifo;

		xferlevel = wcnt << 16;
		if (t->rx_buf != NULL) {
			chconf |= OMAP2_MCSPI_CHCONF_FFER;
			xferlevel |= (bytes_per_word - 1) << 8;
		}

		if (t->tx_buf != NULL) {
			chconf |= OMAP2_MCSPI_CHCONF_FFET;
			xferlevel |= bytes_per_word - 1;
		}

		mcspi_write_reg(master, OMAP2_MCSPI_XFERLEVEL, xferlevel);
		mcspi_write_chconf0(spi, chconf);
		mcspi->fifo_depth = max_fifo_depth;

		return;
	}

disable_fifo:
	if (t->rx_buf != NULL)
		chconf &= ~OMAP2_MCSPI_CHCONF_FFER;

	if (t->tx_buf != NULL)
		chconf &= ~OMAP2_MCSPI_CHCONF_FFET;

	mcspi_write_chconf0(spi, chconf);
	mcspi->fifo_depth = 0;
}

static int mcspi_wait_for_reg_bit(void __iomem *reg, unsigned long bit)
{
	u32 val;

	return readl_poll_timeout(reg, val, val & bit, 1, MSEC_PER_SEC);
}

static int mcspi_wait_for_completion(struct  omap2_mcspi *mcspi,
				     struct completion *x)
{
	if (spi_controller_is_slave(mcspi->master)) {
		if (wait_for_completion_interruptible(x) ||
		    mcspi->slave_aborted)
			return -EINTR;
	} else {
		wait_for_completion(x);
	}

	return 0;
}

static void omap2_mcspi_rx_callback(void *data)
{
	struct spi_device *spi = data;
	struct omap2_mcspi *mcspi = spi_master_get_devdata(spi->master);
	struct omap2_mcspi_dma *mcspi_dma = &mcspi->dma_channels[spi->chip_select];

	/* We must disable the DMA RX request */
	omap2_mcspi_set_dma_req(spi, 1, 0);

	complete(&mcspi_dma->dma_rx_completion);
}

static void omap2_mcspi_tx_callback(void *data)
{
	struct spi_device *spi = data;
	struct omap2_mcspi *mcspi = spi_master_get_devdata(spi->master);
	struct omap2_mcspi_dma *mcspi_dma = &mcspi->dma_channels[spi->chip_select];

	/* We must disable the DMA TX request */
	omap2_mcspi_set_dma_req(spi, 0, 0);

	complete(&mcspi_dma->dma_tx_completion);
}

static void omap2_mcspi_tx_dma(struct spi_device *spi,
				struct spi_transfer *xfer,
				struct dma_slave_config cfg)
{
	struct omap2_mcspi	*mcspi;
	struct omap2_mcspi_dma  *mcspi_dma;

	mcspi = spi_master_get_devdata(spi->master);
	mcspi_dma = &mcspi->dma_channels[spi->chip_select];

	if (mcspi_dma->dma_tx) {
		struct dma_async_tx_descriptor *tx;

		dmaengine_slave_config(mcspi_dma->dma_tx, &cfg);

		tx = dmaengine_prep_slave_sg(mcspi_dma->dma_tx, xfer->tx_sg.sgl,
					     xfer->tx_sg.nents,
					     DMA_MEM_TO_DEV,
					     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (tx) {
			tx->callback = omap2_mcspi_tx_callback;
			tx->callback_param = spi;
			dmaengine_submit(tx);
		} else {
			/* FIXME: fall back to PIO? */
		}
	}
	dma_async_issue_pending(mcspi_dma->dma_tx);
	omap2_mcspi_set_dma_req(spi, 0, 1);

}

static unsigned
omap2_mcspi_rx_dma(struct spi_device *spi, struct spi_transfer *xfer,
				struct dma_slave_config cfg,
				unsigned es)
{
	struct omap2_mcspi	*mcspi;
	struct omap2_mcspi_dma  *mcspi_dma;
	unsigned int		count, transfer_reduction = 0;
	struct scatterlist	*sg_out[2];
	int			nb_sizes = 0, out_mapped_nents[2], ret, x;
	size_t			sizes[2];
	u32			l;
	int			elements = 0;
	int			word_len, element_count;
	struct omap2_mcspi_cs	*cs = spi->controller_state;
	void __iomem		*chstat_reg = cs->base + OMAP2_MCSPI_CHSTAT0;

	mcspi = spi_master_get_devdata(spi->master);
	mcspi_dma = &mcspi->dma_channels[spi->chip_select];
	count = xfer->len;

	/*
	 *  In the "End-of-Transfer Procedure" section for DMA RX in OMAP35x TRM
	 *  it mentions reducing DMA transfer length by one element in master
	 *  normal mode.
	 */
	if (mcspi->fifo_depth == 0)
		transfer_reduction = es;

	word_len = cs->word_len;
	l = mcspi_cached_chconf0(spi);

	if (word_len <= 8)
		element_count = count;
	else if (word_len <= 16)
		element_count = count >> 1;
	else /* word_len <= 32 */
		element_count = count >> 2;

	if (mcspi_dma->dma_rx) {
		struct dma_async_tx_descriptor *tx;

		dmaengine_slave_config(mcspi_dma->dma_rx, &cfg);

		/*
		 *  Reduce DMA transfer length by one more if McSPI is
		 *  configured in turbo mode.
		 */
		if ((l & OMAP2_MCSPI_CHCONF_TURBO) && mcspi->fifo_depth == 0)
			transfer_reduction += es;

		if (transfer_reduction) {
			/* Split sgl into two. The second sgl won't be used. */
			sizes[0] = count - transfer_reduction;
			sizes[1] = transfer_reduction;
			nb_sizes = 2;
		} else {
			/*
			 * Don't bother splitting the sgl. This essentially
			 * clones the original sgl.
			 */
			sizes[0] = count;
			nb_sizes = 1;
		}

		ret = sg_split(xfer->rx_sg.sgl, xfer->rx_sg.nents,
			       0, nb_sizes,
			       sizes,
			       sg_out, out_mapped_nents,
			       GFP_KERNEL);

		if (ret < 0) {
			dev_err(&spi->dev, "sg_split failed\n");
			return 0;
		}

		tx = dmaengine_prep_slave_sg(mcspi_dma->dma_rx,
					     sg_out[0],
					     out_mapped_nents[0],
					     DMA_DEV_TO_MEM,
					     DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (tx) {
			tx->callback = omap2_mcspi_rx_callback;
			tx->callback_param = spi;
			dmaengine_submit(tx);
		} else {
				/* FIXME: fall back to PIO? */
		}
	}

	dma_async_issue_pending(mcspi_dma->dma_rx);
	omap2_mcspi_set_dma_req(spi, 1, 1);

	ret = mcspi_wait_for_completion(mcspi, &mcspi_dma->dma_rx_completion);
	if (ret || mcspi->slave_aborted) {
		dmaengine_terminate_sync(mcspi_dma->dma_rx);
		omap2_mcspi_set_dma_req(spi, 1, 0);
		return 0;
	}

	for (x = 0; x < nb_sizes; x++)
		kfree(sg_out[x]);

	if (mcspi->fifo_depth > 0)
		return count;

	/*
	 *  Due to the DMA transfer length reduction the missing bytes must
	 *  be read manually to receive all of the expected data.
	 */
	omap2_mcspi_set_enable(spi, 0);

	elements = element_count - 1;

	if (l & OMAP2_MCSPI_CHCONF_TURBO) {
		elements--;

		if (!mcspi_wait_for_reg_bit(chstat_reg,
					    OMAP2_MCSPI_CHSTAT_RXS)) {
			u32 w;

			w = mcspi_read_cs_reg(spi, OMAP2_MCSPI_RX0);
			if (word_len <= 8)
				((u8 *)xfer->rx_buf)[elements++] = w;
			else if (word_len <= 16)
				((u16 *)xfer->rx_buf)[elements++] = w;
			else /* word_len <= 32 */
				((u32 *)xfer->rx_buf)[elements++] = w;
		} else {
			int bytes_per_word = mcspi_bytes_per_word(word_len);
			dev_err(&spi->dev, "DMA RX penultimate word empty\n");
			count -= (bytes_per_word << 1);
			omap2_mcspi_set_enable(spi, 1);
			return count;
		}
	}
	if (!mcspi_wait_for_reg_bit(chstat_reg, OMAP2_MCSPI_CHSTAT_RXS)) {
		u32 w;

		w = mcspi_read_cs_reg(spi, OMAP2_MCSPI_RX0);
		if (word_len <= 8)
			((u8 *)xfer->rx_buf)[elements] = w;
		else if (word_len <= 16)
			((u16 *)xfer->rx_buf)[elements] = w;
		else /* word_len <= 32 */
			((u32 *)xfer->rx_buf)[elements] = w;
	} else {
		dev_err(&spi->dev, "DMA RX last word empty\n");
		count -= mcspi_bytes_per_word(word_len);
	}
	omap2_mcspi_set_enable(spi, 1);
	return count;
}

static unsigned
omap2_mcspi_txrx_dma(struct spi_device *spi, struct spi_transfer *xfer)
{
	struct omap2_mcspi	*mcspi;
	struct omap2_mcspi_cs	*cs = spi->controller_state;
	struct omap2_mcspi_dma  *mcspi_dma;
	unsigned int		count;
	u8			*rx;
	const u8		*tx;
	struct dma_slave_config	cfg;
	enum dma_slave_buswidth width;
	unsigned es;
	void __iomem		*chstat_reg;
	void __iomem            *irqstat_reg;
	int			wait_res;

	mcspi = spi_master_get_devdata(spi->master);
	mcspi_dma = &mcspi->dma_channels[spi->chip_select];

	if (cs->word_len <= 8) {
		width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		es = 1;
	} else if (cs->word_len <= 16) {
		width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		es = 2;
	} else {
		width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		es = 4;
	}

	count = xfer->len;

	memset(&cfg, 0, sizeof(cfg));
	cfg.src_addr = cs->phys + OMAP2_MCSPI_RX0;
	cfg.dst_addr = cs->phys + OMAP2_MCSPI_TX0;
	cfg.src_addr_width = width;
	cfg.dst_addr_width = width;
	cfg.src_maxburst = 1;
	cfg.dst_maxburst = 1;

	rx = xfer->rx_buf;
	tx = xfer->tx_buf;

	mcspi->slave_aborted = false;
	reinit_completion(&mcspi_dma->dma_tx_completion);
	reinit_completion(&mcspi_dma->dma_rx_completion);
	reinit_completion(&mcspi->txdone);
	if (tx) {
		/* Enable EOW IRQ to know end of tx in slave mode */
		if (spi_controller_is_slave(spi->master))
			mcspi_write_reg(spi->master,
					OMAP2_MCSPI_IRQENABLE,
					OMAP2_MCSPI_IRQSTATUS_EOW);
		omap2_mcspi_tx_dma(spi, xfer, cfg);
	}

	if (rx != NULL)
		count = omap2_mcspi_rx_dma(spi, xfer, cfg, es);

	if (tx != NULL) {
		int ret;

		ret = mcspi_wait_for_completion(mcspi, &mcspi_dma->dma_tx_completion);
		if (ret || mcspi->slave_aborted) {
			dmaengine_terminate_sync(mcspi_dma->dma_tx);
			omap2_mcspi_set_dma_req(spi, 0, 0);
			return 0;
		}

		if (spi_controller_is_slave(mcspi->master)) {
			ret = mcspi_wait_for_completion(mcspi, &mcspi->txdone);
			if (ret || mcspi->slave_aborted)
				return 0;
		}

		if (mcspi->fifo_depth > 0) {
			irqstat_reg = mcspi->base + OMAP2_MCSPI_IRQSTATUS;

			if (mcspi_wait_for_reg_bit(irqstat_reg,
						OMAP2_MCSPI_IRQSTATUS_EOW) < 0)
				dev_err(&spi->dev, "EOW timed out\n");

			mcspi_write_reg(mcspi->master, OMAP2_MCSPI_IRQSTATUS,
					OMAP2_MCSPI_IRQSTATUS_EOW);
		}

		/* for TX_ONLY mode, be sure all words have shifted out */
		if (rx == NULL) {
			chstat_reg = cs->base + OMAP2_MCSPI_CHSTAT0;
			if (mcspi->fifo_depth > 0) {
				wait_res = mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_TXFFE);
				if (wait_res < 0)
					dev_err(&spi->dev, "TXFFE timed out\n");
			} else {
				wait_res = mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_TXS);
				if (wait_res < 0)
					dev_err(&spi->dev, "TXS timed out\n");
			}
			if (wait_res >= 0 &&
				(mcspi_wait_for_reg_bit(chstat_reg,
					OMAP2_MCSPI_CHSTAT_EOT) < 0))
				dev_err(&spi->dev, "EOT timed out\n");
		}
	}
	return count;
}

static void mcspi_rd_fifo(struct omap2_mcspi *mcspi)
{
	u8 byte;
	int i;

	/* Receiver register must be read to remove source of interrupt */
	for (i = 0; i < mcspi->fifo_depth; i++) {
		byte = mcspi_read_cs_reg(mcspi->spi, OMAP2_MCSPI_RX0);
		if (mcspi->rx_buf && (mcspi->rx_len > 0))
			*mcspi->rx_buf++ = byte;
		mcspi->rx_len--;
	}
}

static void mcspi_wr_fifo(struct omap2_mcspi *mcspi)
{
	u8 byte;
	int i;

	/* load transmitter register to remove the source of the interrupt */
	for (i = 0; i < mcspi->fifo_depth; i++) {
		if (mcspi->tx_len <= 0)
			byte = 0;
		else
			byte = mcspi->tx_buf ? *mcspi->tx_buf++ : 0;
		mcspi_write_cs_reg(mcspi->spi, OMAP2_MCSPI_TX0, byte);
		mcspi->tx_len--;
	}
}

static void mcspi_wr_fifo_bh(struct omap2_mcspi *mcspi)
{
	u8 byte;
	int i;
	unsigned long flags;

	raw_spin_lock_irqsave(&mcspi->lock, flags);
	/* load transmitter register to remove the source of the interrupt */
	for (i = 0; i < mcspi->fifo_depth; i++) {
		if (mcspi->tx_len <= 0)
			byte = 0;
		else
			byte = mcspi->tx_buf ? *mcspi->tx_buf++ : 0;
		mcspi_write_cs_reg(mcspi->spi, OMAP2_MCSPI_TX0, byte);
		mcspi->tx_len--;
	}
	raw_spin_unlock_irqrestore(&mcspi->lock, flags);
}

static irqreturn_t omap2_mcspi_irq_handler_rt(int irq, void *dev_id) 
{    
	struct omap2_mcspi	*mcspi = dev_id;
	u32 l;

	raw_spin_lock(&mcspi->lock);

	mcspi->n_interrupts++;
	l = mcspi_read_reg(mcspi->master, OMAP2_MCSPI_IRQSTATUS);

	if ((l & OMAP2_MCSPI_IRQSTATUS_RX0_FULL) ||
	   (l & OMAP2_MCSPI_IRQSTATUS_RX1_FULL)) {
		mcspi_rd_fifo(mcspi);
		mcspi->n_rx_full++;
	}
	if ((l & OMAP2_MCSPI_IRQSTATUS_TX0_EMPTY) ||
		(l & OMAP2_MCSPI_IRQSTATUS_TX1_EMPTY)) {
		if (mcspi->tx_len > 0)
			mcspi_wr_fifo(mcspi);
		mcspi->n_tx_empty++;
	}

	/* write 1 to OMAP2_MCSPI_IRQSTATUS field to reset it */
	mcspi_write_reg(mcspi->master, OMAP2_MCSPI_IRQSTATUS, l);
		
	if ((mcspi->tx_len <= 0) && (mcspi->rx_len <= 0)) {
		/* disable interrupts */
		mcspi_write_reg(mcspi->master, OMAP2_MCSPI_IRQENABLE, 0);

 		/* using swait.h model */
		mcspi->interrupt_done = 1;
		smp_mb();
		if (swait_active(&mcspi->swait))
			swake_up_one(&mcspi->swait);		
	}

	raw_spin_unlock(&mcspi->lock);

	return IRQ_HANDLED;
}

static int omap2_mcspi_disable_fifo(struct spi_device *spi)
{
	u32 chconf;

	chconf = mcspi_cached_chconf0(spi);
	chconf &= ~(OMAP2_MCSPI_CHCONF_FFER | OMAP2_MCSPI_CHCONF_FFET);
	mcspi_write_chconf0(spi, chconf);
	return 0;
}

static int omap2_mcspi_set_fifo_rt(struct spi_device *spi)
{
	struct omap2_mcspi *mcspi;
	unsigned int wcnt;
	int max_fifo_depth, fifo_depth, bytes_per_word;
	u32 chconf, xferlevel;

	mcspi = spi_master_get_devdata(spi->master);

	chconf = mcspi_cached_chconf0(spi);
	bytes_per_word = 1; /* support only this mode */

	max_fifo_depth = (OMAP2_MCSPI_MAX_FIFODEPTH / 2);
	if (mcspi->tx_len < max_fifo_depth) {
		fifo_depth = mcspi->tx_len;
		wcnt = mcspi->tx_len / bytes_per_word;
	} else {
		fifo_depth = max_fifo_depth;
		wcnt = max_fifo_depth * (mcspi->tx_len / max_fifo_depth)
			/ bytes_per_word;
	}
	if (wcnt > OMAP2_MCSPI_MAX_FIFOWCNT) {
		dev_err(&spi->dev,
			"%s: wcnt=%d: too many bytes in a transfer.\n",
			__func__, wcnt);
		return -EINVAL;
	}
	
	chconf |= OMAP2_MCSPI_CHCONF_FFER;
	chconf |= OMAP2_MCSPI_CHCONF_FFET;

	mcspi_write_chconf0(spi, chconf);	
	mcspi->fifo_depth = fifo_depth;

	xferlevel = wcnt << 16;
	xferlevel |= (fifo_depth - 1) << 8;
	xferlevel |= fifo_depth - 1;
	mcspi_write_reg(mcspi->master, OMAP2_MCSPI_XFERLEVEL, xferlevel);
	xferlevel = mcspi_read_reg(mcspi->master, OMAP2_MCSPI_XFERLEVEL);

	return 0;
}

static int do_transfer_irq_bh(struct spi_device *spi)
{
	struct omap2_mcspi	*mcspi;
	u32 chconf, l;
	int ret;
	DECLARE_SWAITQUEUE(swait);
	
	mcspi = spi_master_get_devdata(spi->master);
	/* configure to send and receive */
	chconf = mcspi_cached_chconf0(spi);
	chconf &= ~OMAP2_MCSPI_CHCONF_TRM_MASK;
	chconf &= ~OMAP2_MCSPI_CHCONF_TURBO;
	mcspi_write_chconf0(spi, chconf);

	/* fifo can be enabled on a single channel */
	ret = omap2_mcspi_set_fifo_rt(spi);
	if (ret)
		return ret;

	/* CS is active and channel is enabled by omap2_mcspi_transfer_one() */
	/* No need to set chip_select as chosen. */

	/* The interrupt status bit should always be reset
	 * after the channel is enabled
	 * and before the event is enabled as an interrupt source.
	 */
	/* write 1 to OMAP2_MCSPI_IRQSTATUS field to reset it */
	l = mcspi_read_reg(mcspi->master, OMAP2_MCSPI_IRQSTATUS);
	mcspi_write_reg(mcspi->master, OMAP2_MCSPI_IRQSTATUS, l);

	mcspi->n_interrupts = 0;
	mcspi->n_rx_full = 0;
	mcspi->n_tx_empty = 0;

	/* Enable interrupts last. */
	mcspi->interrupt_done = 0;
	/* support only two channels */
	if (spi->chip_select == 0)
		l = OMAP2_MCSPI_IRQENABLE_TX0_EMPTY |
			OMAP2_MCSPI_IRQENABLE_RX0_FULL;
	else
		l = OMAP2_MCSPI_IRQENABLE_TX1_EMPTY |
			OMAP2_MCSPI_IRQENABLE_RX1_FULL;
	mcspi_write_reg(mcspi->master, OMAP2_MCSPI_IRQENABLE, l);

	/* TX_EMPTY will be raised only after SPI data is sent */
	mcspi_wr_fifo_bh(mcspi);

	/* wait for transfer completion using swait.h model */
	for (;;) {
		prepare_to_swait_exclusive(&mcspi->swait, &swait, TASK_INTERRUPTIBLE);
		/* smp_mb() from set_current_state() */
	 	if (mcspi->interrupt_done)
 			break;
 		schedule();
 	}
	finish_swait(&mcspi->swait, &swait);

#if 0
	dev_warn(&spi->dev, 
			"%s: tx_len=%d rx_len=%d n_interrupts=%d n_rx_full=%d n_tx_empty=%d\n",
			__FUNCTION__,
			mcspi->tx_len, mcspi->rx_len,
		 	mcspi->n_interrupts, mcspi->n_rx_full, mcspi->n_tx_empty);
#endif
	
	/* fifo can be enabled on a single channel */
	omap2_mcspi_disable_fifo(spi);

	/* mcspi->tx_len and mcspi->rx_len should be 0 */
	if (mcspi->tx_len || mcspi->rx_len)
		return -EIO;
	return 0;
}

static int do_transfer_irq(struct spi_device *spi, struct spi_transfer *xfer)
{
	struct omap2_mcspi	*mcspi;
	int len, first_size, last_size, ret;
	int max_fifo_depth = (OMAP2_MCSPI_MAX_FIFODEPTH / 2);
	
	mcspi = spi_master_get_devdata(spi->master);

	if (xfer->len < 1)
		return -EINVAL;
	
	mcspi->rx_len = mcspi->tx_len = xfer->len;
	mcspi->rx_buf = xfer->rx_buf;
	mcspi->tx_buf = xfer->tx_buf;	
	
	len = mcspi->tx_len;

	if (len < max_fifo_depth)
		goto label_last;

	first_size = max_fifo_depth *
		(len / max_fifo_depth);
	mcspi->tx_len = first_size;
	mcspi->rx_len = first_size;
	ret = do_transfer_irq_bh(spi);
	if (ret)	
		return -EIO;

label_last:
	last_size = len % max_fifo_depth;
	if (last_size == 0)
		return xfer->len;
	mcspi->tx_len = last_size;
	mcspi->rx_len = last_size;
	ret = do_transfer_irq_bh(spi);
	if (ret)
		return -EIO;
	
	return xfer->len;
}

static unsigned
omap2_mcspi_txrx_pio(struct spi_device *spi, struct spi_transfer *xfer)
{
	struct omap2_mcspi_cs	*cs = spi->controller_state;
	unsigned int		count, c;
	u32			l;
	void __iomem		*base = cs->base;
	void __iomem		*tx_reg;
	void __iomem		*rx_reg;
	void __iomem		*chstat_reg;
	int			word_len;

	count = xfer->len;
	c = count;
	word_len = cs->word_len;

	l = mcspi_cached_chconf0(spi);

	/* We store the pre-calculated register addresses on stack to speed
	 * up the transfer loop. */
	tx_reg		= base + OMAP2_MCSPI_TX0;
	rx_reg		= base + OMAP2_MCSPI_RX0;
	chstat_reg	= base + OMAP2_MCSPI_CHSTAT0;

	if (c < (word_len>>3))
		return 0;

	if (word_len <= 8) {
		u8		*rx;
		const u8	*tx;

		rx = xfer->rx_buf;
		tx = xfer->tx_buf;

		do {
			c -= 1;
			if (tx != NULL) {
				if (mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_TXS) < 0) {
					dev_err(&spi->dev, "TXS timed out\n");
					goto out;
				}
				dev_vdbg(&spi->dev, "write-%d %02x\n",
						word_len, *tx);
				writel_relaxed(*tx++, tx_reg);
			}
			if (rx != NULL) {
				if (mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_RXS) < 0) {
					dev_err(&spi->dev, "RXS timed out\n");
					goto out;
				}

				if (c == 1 && tx == NULL &&
				    (l & OMAP2_MCSPI_CHCONF_TURBO)) {
					omap2_mcspi_set_enable(spi, 0);
					*rx++ = readl_relaxed(rx_reg);
					dev_vdbg(&spi->dev, "read-%d %02x\n",
						    word_len, *(rx - 1));
					if (mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_RXS) < 0) {
						dev_err(&spi->dev,
							"RXS timed out\n");
						goto out;
					}
					c = 0;
				} else if (c == 0 && tx == NULL) {
					omap2_mcspi_set_enable(spi, 0);
				}

				*rx++ = readl_relaxed(rx_reg);
				dev_vdbg(&spi->dev, "read-%d %02x\n",
						word_len, *(rx - 1));
			}
		} while (c);
	} else if (word_len <= 16) {
		u16		*rx;
		const u16	*tx;

		rx = xfer->rx_buf;
		tx = xfer->tx_buf;
		do {
			c -= 2;
			if (tx != NULL) {
				if (mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_TXS) < 0) {
					dev_err(&spi->dev, "TXS timed out\n");
					goto out;
				}
				dev_vdbg(&spi->dev, "write-%d %04x\n",
						word_len, *tx);
				writel_relaxed(*tx++, tx_reg);
			}
			if (rx != NULL) {
				if (mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_RXS) < 0) {
					dev_err(&spi->dev, "RXS timed out\n");
					goto out;
				}

				if (c == 2 && tx == NULL &&
				    (l & OMAP2_MCSPI_CHCONF_TURBO)) {
					omap2_mcspi_set_enable(spi, 0);
					*rx++ = readl_relaxed(rx_reg);
					dev_vdbg(&spi->dev, "read-%d %04x\n",
						    word_len, *(rx - 1));
					if (mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_RXS) < 0) {
						dev_err(&spi->dev,
							"RXS timed out\n");
						goto out;
					}
					c = 0;
				} else if (c == 0 && tx == NULL) {
					omap2_mcspi_set_enable(spi, 0);
				}

				*rx++ = readl_relaxed(rx_reg);
				dev_vdbg(&spi->dev, "read-%d %04x\n",
						word_len, *(rx - 1));
			}
		} while (c >= 2);
	} else if (word_len <= 32) {
		u32		*rx;
		const u32	*tx;

		rx = xfer->rx_buf;
		tx = xfer->tx_buf;
		do {
			c -= 4;
			if (tx != NULL) {
				if (mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_TXS) < 0) {
					dev_err(&spi->dev, "TXS timed out\n");
					goto out;
				}
				dev_vdbg(&spi->dev, "write-%d %08x\n",
						word_len, *tx);
				writel_relaxed(*tx++, tx_reg);
			}
			if (rx != NULL) {
				if (mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_RXS) < 0) {
					dev_err(&spi->dev, "RXS timed out\n");
					goto out;
				}

				if (c == 4 && tx == NULL &&
				    (l & OMAP2_MCSPI_CHCONF_TURBO)) {
					omap2_mcspi_set_enable(spi, 0);
					*rx++ = readl_relaxed(rx_reg);
					dev_vdbg(&spi->dev, "read-%d %08x\n",
						    word_len, *(rx - 1));
					if (mcspi_wait_for_reg_bit(chstat_reg,
						OMAP2_MCSPI_CHSTAT_RXS) < 0) {
						dev_err(&spi->dev,
							"RXS timed out\n");
						goto out;
					}
					c = 0;
				} else if (c == 0 && tx == NULL) {
					omap2_mcspi_set_enable(spi, 0);
				}

				*rx++ = readl_relaxed(rx_reg);
				dev_vdbg(&spi->dev, "read-%d %08x\n",
						word_len, *(rx - 1));
			}
		} while (c >= 4);
	}

	/* for TX_ONLY mode, be sure all words have shifted out */
	if (xfer->rx_buf == NULL) {
		if (mcspi_wait_for_reg_bit(chstat_reg,
				OMAP2_MCSPI_CHSTAT_TXS) < 0) {
			dev_err(&spi->dev, "TXS timed out\n");
		} else if (mcspi_wait_for_reg_bit(chstat_reg,
				OMAP2_MCSPI_CHSTAT_EOT) < 0)
			dev_err(&spi->dev, "EOT timed out\n");

		/* disable chan to purge rx datas received in TX_ONLY transfer,
		 * otherwise these rx datas will affect the direct following
		 * RX_ONLY transfer.
		 */
		omap2_mcspi_set_enable(spi, 0);
	}
out:
	omap2_mcspi_set_enable(spi, 1);
	return count - c;
}

static u32 omap2_mcspi_calc_divisor(u32 speed_hz)
{
	u32 div;

	for (div = 0; div < 15; div++)
		if (speed_hz >= (OMAP2_MCSPI_MAX_FREQ >> div))
			return div;

	return 15;
}

/* called only when no transfer is active to this device */
static int omap2_mcspi_setup_transfer(struct spi_device *spi,
		struct spi_transfer *t)
{
	struct omap2_mcspi_cs *cs = spi->controller_state;
	struct omap2_mcspi *mcspi;
	u32 l = 0, clkd = 0, div, extclk = 0, clkg = 0;
	u8 word_len = spi->bits_per_word;
	u32 speed_hz = spi->max_speed_hz;

	mcspi = spi_master_get_devdata(spi->master);

	if (t != NULL && t->bits_per_word)
		word_len = t->bits_per_word;

	cs->word_len = word_len;

	if (t && t->speed_hz)
		speed_hz = t->speed_hz;

	speed_hz = min_t(u32, speed_hz, OMAP2_MCSPI_MAX_FREQ);
	if (speed_hz < (OMAP2_MCSPI_MAX_FREQ / OMAP2_MCSPI_MAX_DIVIDER)) {
		clkd = omap2_mcspi_calc_divisor(speed_hz);
		speed_hz = OMAP2_MCSPI_MAX_FREQ >> clkd;
		clkg = 0;
	} else {
		div = (OMAP2_MCSPI_MAX_FREQ + speed_hz - 1) / speed_hz;
		speed_hz = OMAP2_MCSPI_MAX_FREQ / div;
		clkd = (div - 1) & 0xf;
		extclk = (div - 1) >> 4;
		clkg = OMAP2_MCSPI_CHCONF_CLKG;
	}

	l = mcspi_cached_chconf0(spi);

	/* standard 4-wire master mode:  SCK, MOSI/out, MISO/in, nCS
	 * REVISIT: this controller could support SPI_3WIRE mode.
	 */
	if (mcspi->pin_dir == MCSPI_PINDIR_D0_IN_D1_OUT) {
		l &= ~OMAP2_MCSPI_CHCONF_IS;
		l &= ~OMAP2_MCSPI_CHCONF_DPE1;
		l |= OMAP2_MCSPI_CHCONF_DPE0;
	} else {
		l |= OMAP2_MCSPI_CHCONF_IS;
		l |= OMAP2_MCSPI_CHCONF_DPE1;
		l &= ~OMAP2_MCSPI_CHCONF_DPE0;
	}

	/* wordlength */
	l &= ~OMAP2_MCSPI_CHCONF_WL_MASK;
	l |= (word_len - 1) << 7;

	/* set chipselect polarity; manage with FORCE */
	if (!(spi->mode & SPI_CS_HIGH))
		l |= OMAP2_MCSPI_CHCONF_EPOL;	/* active-low; normal */
	else
		l &= ~OMAP2_MCSPI_CHCONF_EPOL;

	/* set clock divisor */
	l &= ~OMAP2_MCSPI_CHCONF_CLKD_MASK;
	l |= clkd << 2;

	/* set clock granularity */
	l &= ~OMAP2_MCSPI_CHCONF_CLKG;
	l |= clkg;
	if (clkg) {
		cs->chctrl0 &= ~OMAP2_MCSPI_CHCTRL_EXTCLK_MASK;
		cs->chctrl0 |= extclk << 8;
		mcspi_write_cs_reg(spi, OMAP2_MCSPI_CHCTRL0, cs->chctrl0);
	}

	/* set SPI mode 0..3 */
	if (spi->mode & SPI_CPOL)
		l |= OMAP2_MCSPI_CHCONF_POL;
	else
		l &= ~OMAP2_MCSPI_CHCONF_POL;
	if (spi->mode & SPI_CPHA)
		l |= OMAP2_MCSPI_CHCONF_PHA;
	else
		l &= ~OMAP2_MCSPI_CHCONF_PHA;

	mcspi_write_chconf0(spi, l);

	cs->mode = spi->mode;

	dev_dbg(&spi->dev, "setup: speed %d, sample %s edge, clk %s\n",
			speed_hz,
			(spi->mode & SPI_CPHA) ? "trailing" : "leading",
			(spi->mode & SPI_CPOL) ? "inverted" : "normal");

	return 0;
}

/*
 * Note that we currently allow DMA only if we get a channel
 * for both rx and tx. Otherwise we'll do PIO for both rx and tx.
 */
static int omap2_mcspi_request_dma(struct spi_device *spi)
{
	struct spi_master	*master = spi->master;
	struct omap2_mcspi	*mcspi;
	struct omap2_mcspi_dma	*mcspi_dma;
	int ret = 0;

	mcspi = spi_master_get_devdata(master);
	mcspi_dma = mcspi->dma_channels + spi->chip_select;

	init_completion(&mcspi_dma->dma_rx_completion);
	init_completion(&mcspi_dma->dma_tx_completion);

	mcspi_dma->dma_rx = dma_request_chan(&master->dev,
					     mcspi_dma->dma_rx_ch_name);
	if (IS_ERR(mcspi_dma->dma_rx)) {
		ret = PTR_ERR(mcspi_dma->dma_rx);
		mcspi_dma->dma_rx = NULL;
		goto no_dma;
	}

	mcspi_dma->dma_tx = dma_request_chan(&master->dev,
					     mcspi_dma->dma_tx_ch_name);
	if (IS_ERR(mcspi_dma->dma_tx)) {
		ret = PTR_ERR(mcspi_dma->dma_tx);
		mcspi_dma->dma_tx = NULL;
		dma_release_channel(mcspi_dma->dma_rx);
		mcspi_dma->dma_rx = NULL;
	}

no_dma:
	return ret;
}

static int omap2_mcspi_setup(struct spi_device *spi)
{
	int			ret;
	struct omap2_mcspi	*mcspi = spi_master_get_devdata(spi->master);
	struct omap2_mcspi_regs	*ctx = &mcspi->ctx;
	struct omap2_mcspi_dma	*mcspi_dma;
	struct omap2_mcspi_cs	*cs = spi->controller_state;

	mcspi_dma = &mcspi->dma_channels[spi->chip_select];

	if (!cs) {
		cs = kzalloc(sizeof *cs, GFP_KERNEL);
		if (!cs)
			return -ENOMEM;
		cs->base = mcspi->base + spi->chip_select * 0x14;
		cs->phys = mcspi->phys + spi->chip_select * 0x14;
		cs->mode = 0;
		cs->chconf0 = 0;
		cs->chctrl0 = 0;
		spi->controller_state = cs;
		/* Link this to context save list */
		list_add_tail(&cs->node, &ctx->cs);

		if (gpio_is_valid(spi->cs_gpio)) {
			ret = gpio_request(spi->cs_gpio, dev_name(&spi->dev));
			if (ret) {
				dev_err(&spi->dev, "failed to request gpio\n");
				return ret;
			}
			gpio_direction_output(spi->cs_gpio,
					 !(spi->mode & SPI_CS_HIGH));
		}
	}

	if (!mcspi_dma->dma_rx || !mcspi_dma->dma_tx) {
		ret = omap2_mcspi_request_dma(spi);
		if (ret)
			dev_warn(&spi->dev, "not using DMA for McSPI (%d)\n",
				 ret);
	}
	
	ret = omap2_mcspi_setup_transfer(spi, NULL);

	return ret;
}

static void omap2_mcspi_cleanup(struct spi_device *spi)
{
	struct omap2_mcspi	*mcspi;
	struct omap2_mcspi_dma	*mcspi_dma;
	struct omap2_mcspi_cs	*cs;

	mcspi = spi_master_get_devdata(spi->master);

	if (spi->controller_state) {
		/* Unlink controller state from context save list */
		cs = spi->controller_state;
		list_del(&cs->node);

		kfree(cs);
	}

	if (spi->chip_select < spi->master->num_chipselect) {
		mcspi_dma = &mcspi->dma_channels[spi->chip_select];

		if (mcspi_dma->dma_rx) {
			dma_release_channel(mcspi_dma->dma_rx);
			mcspi_dma->dma_rx = NULL;
		}
		if (mcspi_dma->dma_tx) {
			dma_release_channel(mcspi_dma->dma_tx);
			mcspi_dma->dma_tx = NULL;
		}
	}

	if (gpio_is_valid(spi->cs_gpio))
		gpio_free(spi->cs_gpio);
}

#if 0
static irqreturn_t omap2_mcspi_irq_handler(int irq, void *data)
{
	struct omap2_mcspi *mcspi = data;
	u32 irqstat;

	irqstat	= mcspi_read_reg(mcspi->master, OMAP2_MCSPI_IRQSTATUS);
	if (!irqstat)
		return IRQ_NONE;

	/* Disable IRQ and wakeup slave xfer task */
	mcspi_write_reg(mcspi->master, OMAP2_MCSPI_IRQENABLE, 0);
	if (irqstat & OMAP2_MCSPI_IRQSTATUS_EOW)
		complete(&mcspi->txdone);

	return IRQ_HANDLED;
}
#endif


static int omap2_mcspi_slave_abort(struct spi_master *master)
{
	struct omap2_mcspi *mcspi = spi_master_get_devdata(master);
	struct omap2_mcspi_dma *mcspi_dma = mcspi->dma_channels;

	mcspi->slave_aborted = true;
	complete(&mcspi_dma->dma_rx_completion);
	complete(&mcspi_dma->dma_tx_completion);
	complete(&mcspi->txdone);

	return 0;
}

static int omap2_mcspi_transfer_one(struct spi_master *master,
				    struct spi_device *spi,
				    struct spi_transfer *t)
{

	/* We only enable one channel at a time -- the one whose message is
	 * -- although this controller would gladly
	 * arbitrate among multiple channels.  This corresponds to "single
	 * channel" master mode.  As a side effect, we need to manage the
	 * chipselect with the FORCE bit ... CS != channel enable.
	 */

	struct omap2_mcspi		*mcspi;
	struct omap2_mcspi_dma		*mcspi_dma;
	struct omap2_mcspi_cs		*cs;
	struct omap2_mcspi_device_config *cd;
	int				par_override = 0;
	int				status = 0, status_st = 0;
	u32				chconf;

	mcspi = spi_master_get_devdata(master);
	mcspi_dma = mcspi->dma_channels + spi->chip_select;
	cs = spi->controller_state;
	cd = spi->controller_data;

	/*
	 * The slave driver could have changed spi->mode in which case
	 * it will be different from cs->mode (the current hardware setup).
	 * If so, set par_override (even though its not a parity issue) so
	 * omap2_mcspi_setup_transfer will be called to configure the hardware
	 * with the correct mode on the first iteration of the loop below.
	 */
	if (spi->mode != cs->mode)
		par_override = 1;

	omap2_mcspi_set_enable(spi, 0);

	/* we know cs is valid pin */
	omap2_mcspi_set_cs(spi, spi->mode & SPI_CS_HIGH);

	if (par_override ||
	    (t->speed_hz != spi->max_speed_hz) ||
	    (t->bits_per_word != spi->bits_per_word)) {
		par_override = 1;
		status = omap2_mcspi_setup_transfer(spi, t);
		if (status < 0)
			goto out;
		if (t->speed_hz == spi->max_speed_hz &&
		    t->bits_per_word == spi->bits_per_word)
			par_override = 0;
	}
	if (cd && cd->cs_per_word) {
		chconf = mcspi->ctx.modulctrl;
		chconf &= ~OMAP2_MCSPI_MODULCTRL_SINGLE;
		mcspi_write_reg(master, OMAP2_MCSPI_MODULCTRL, chconf);
		mcspi->ctx.modulctrl =
			mcspi_read_cs_reg(spi, OMAP2_MCSPI_MODULCTRL);
	}

	chconf = mcspi_cached_chconf0(spi);
	chconf &= ~OMAP2_MCSPI_CHCONF_TRM_MASK;
	chconf &= ~OMAP2_MCSPI_CHCONF_TURBO;

	if (t->tx_buf == NULL)
		chconf |= OMAP2_MCSPI_CHCONF_TRM_RX_ONLY;
	else if (t->rx_buf == NULL)
		chconf |= OMAP2_MCSPI_CHCONF_TRM_TX_ONLY;

	if (cd && cd->turbo_mode && t->tx_buf == NULL) {
		/* Turbo mode is for more than one word */
		if (t->len > ((cs->word_len + 7) >> 3))
			chconf |= OMAP2_MCSPI_CHCONF_TURBO;
	}

	mcspi_write_chconf0(spi, chconf);

	if (t->len) {
		unsigned	count;

#if 0
		if ((mcspi_dma->dma_rx && mcspi_dma->dma_tx) &&
		    master->cur_msg_mapped &&
		    master->can_dma(master, spi, t))
			omap2_mcspi_set_fifo(spi, t, 1);

		omap2_mcspi_set_enable(spi, 1);

		/* RX_ONLY mode needs dummy data in TX reg */
		if (t->tx_buf == NULL)
			writel_relaxed(0, cs->base
					+ OMAP2_MCSPI_TX0);

		if ((mcspi_dma->dma_rx && mcspi_dma->dma_tx) &&
		    master->cur_msg_mapped &&
		    master->can_dma(master, spi, t))
			count = omap2_mcspi_txrx_dma(spi, t);
		else
			count = omap2_mcspi_txrx_pio(spi, t);
#endif
		/* do real-time transfer */
		/* give mcspi access to current spi device */
		mcspi->spi = spi;
		omap2_mcspi_set_enable(spi, 1);
		count = do_transfer_irq(spi, t);

		if (count != t->len) {
			status = -EIO;
			goto out;
		}
	}

	omap2_mcspi_set_enable(spi, 0);

	if (mcspi->fifo_depth > 0)
		omap2_mcspi_set_fifo(spi, t, 0);

out:
	/* Restore defaults if they were overriden */
	if (par_override) {
		par_override = 0;
		status_st = omap2_mcspi_setup_transfer(spi, NULL);
		if(status == 0)
			status = status_st;
	}

	if (cd && cd->cs_per_word) {
		chconf = mcspi->ctx.modulctrl;
		chconf |= OMAP2_MCSPI_MODULCTRL_SINGLE;
		mcspi_write_reg(master, OMAP2_MCSPI_MODULCTRL, chconf);
		mcspi->ctx.modulctrl =
			mcspi_read_cs_reg(spi, OMAP2_MCSPI_MODULCTRL);
	}

	omap2_mcspi_set_enable(spi, 0);

	/* we know cs is valid pin */
	omap2_mcspi_set_cs(spi, !(spi->mode & SPI_CS_HIGH));
	
	if (mcspi->fifo_depth > 0 && t)
		omap2_mcspi_set_fifo(spi, t, 0);

	return status;
}

static int omap2_mcspi_prepare_message(struct spi_master *master,
				       struct spi_message *msg)
{
	struct omap2_mcspi	*mcspi = spi_master_get_devdata(master);
	struct omap2_mcspi_regs	*ctx = &mcspi->ctx;
	struct omap2_mcspi_cs	*cs;

	/* Only a single channel can have the FORCE bit enabled
	 * in its chconf0 register.
	 * Scan all channels and disable them except the current one.
	 * A FORCE can remain from a last transfer having cs_change enabled
	 */
	list_for_each_entry(cs, &ctx->cs, node) {
		if (msg->spi->controller_state == cs)
			continue;

		if ((cs->chconf0 & OMAP2_MCSPI_CHCONF_FORCE)) {
			cs->chconf0 &= ~OMAP2_MCSPI_CHCONF_FORCE;
			writel_relaxed(cs->chconf0,
					cs->base + OMAP2_MCSPI_CHCONF0);
			readl_relaxed(cs->base + OMAP2_MCSPI_CHCONF0);
		}
	}

	return 0;
}

static bool omap2_mcspi_can_dma(struct spi_master *master,
				struct spi_device *spi,
				struct spi_transfer *xfer)
{
	struct omap2_mcspi *mcspi = spi_master_get_devdata(spi->master);
	struct omap2_mcspi_dma *mcspi_dma =
		&mcspi->dma_channels[spi->chip_select];

	if (!mcspi_dma->dma_rx || !mcspi_dma->dma_tx)
		return false;

	if (spi_controller_is_slave(master))
		return true;

	return (xfer->len >= DMA_MIN_BYTES);
}

static int omap2_mcspi_controller_setup(struct omap2_mcspi *mcspi)
{
	struct spi_master	*master = mcspi->master;
	struct omap2_mcspi_regs	*ctx = &mcspi->ctx;
	u32 l;
	
	l = mcspi_read_reg(master, OMAP2_MCSPI_SYSCONFIG);
	/* CLOCKACTIVITY = 3h: OCP and Functional clocks are maintained */
	l |= OMAP2_MCSPI_SYSCONFIG_CLOCKACTIVITY_MASK;
	/* SIDLEMODE = 1h: ignore idle requests */
	l &= ~OMAP2_MCSPI_SYSCONFIG_SIDLEMODE_MASK;
	l |= 0x1 << 3;
	/* AUTOIDLE=0: OCP clock is free-running */
	l &= ~OMAP2_MCSPI_SYSCONFIG_AUTOIDLE;
	mcspi_write_reg(master, OMAP2_MCSPI_SYSCONFIG, l);
	
	mcspi_write_reg(master, OMAP2_MCSPI_WAKEUPENABLE,
			OMAP2_MCSPI_WAKEUPENABLE_WKEN);
	ctx->wakeupenable = OMAP2_MCSPI_WAKEUPENABLE_WKEN;

	omap2_mcspi_set_mode(master);
	return 0;
}

static void omap2_mcspi_reset_hw(struct omap2_mcspi *mcspi)
{
	u32 l;
	struct spi_master	*master = mcspi->master;

	l = mcspi_read_reg(master, OMAP2_MCSPI_SYSCONFIG);
	l |= OMAP2_MCSPI_SYSCONFIG_SOFTRESET;
	mcspi_write_reg(master, OMAP2_MCSPI_SYSCONFIG, l);
	/* wait until reset is done */
	do {
		l = mcspi_read_reg(master, OMAP2_MCSPI_SYSSTATUS);
		cpu_relax();
	} while (!(l & OMAP2_MCSPI_SYSSTATUS_RESETDONE));
}

/*
 * When SPI wake up from off-mode, CS is in activate state. If it was in
 * inactive state when driver was suspend, then force it to inactive state at
 * wake up.
 */
static int omap_mcspi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct omap2_mcspi *mcspi = spi_master_get_devdata(master);
	struct omap2_mcspi_regs *ctx = &mcspi->ctx;
	struct omap2_mcspi_cs *cs;

	/* McSPI: context restore */
	mcspi_write_reg(master, OMAP2_MCSPI_MODULCTRL, ctx->modulctrl);
	mcspi_write_reg(master, OMAP2_MCSPI_WAKEUPENABLE, ctx->wakeupenable);

	list_for_each_entry(cs, &ctx->cs, node) {
		/*
		 * We need to toggle CS state for OMAP take this
		 * change in account.
		 */
		if ((cs->chconf0 & OMAP2_MCSPI_CHCONF_FORCE) == 0) {
			cs->chconf0 |= OMAP2_MCSPI_CHCONF_FORCE;
			writel_relaxed(cs->chconf0,
				       cs->base + OMAP2_MCSPI_CHCONF0);
			cs->chconf0 &= ~OMAP2_MCSPI_CHCONF_FORCE;
			writel_relaxed(cs->chconf0,
				       cs->base + OMAP2_MCSPI_CHCONF0);
		} else {
			writel_relaxed(cs->chconf0,
				       cs->base + OMAP2_MCSPI_CHCONF0);
		}
	}

	return 0;
}

static struct omap2_mcspi_platform_config omap2_pdata = {
	.regs_offset = 0,
};

static struct omap2_mcspi_platform_config omap4_pdata = {
	.regs_offset = OMAP4_MCSPI_REG_OFFSET,
};

static const struct of_device_id omap_mcspi_of_match[] = {
	{
		.compatible = "ti,omap2-mcspi",
		.data = &omap2_pdata,
	},
	{
		.compatible = "ti,omap4-mcspi",
		.data = &omap4_pdata,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, omap_mcspi_of_match);

static int omap2_mcspi_probe(struct platform_device *pdev)
{
	struct spi_master	*master;
	const struct omap2_mcspi_platform_config *pdata;
	struct omap2_mcspi	*mcspi;
	struct resource		*r;
	int			status = 0, i;
	u32			regs_offset = 0;
	struct device_node	*node = pdev->dev.of_node;
	const struct of_device_id *match;
	
	if (of_property_read_bool(node, "spi-slave"))
		master = spi_alloc_slave(&pdev->dev, sizeof(*mcspi));
	else
		master = spi_alloc_master(&pdev->dev, sizeof(*mcspi));
	if (!master)
		return -ENOMEM;

	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(4, 32);
	master->setup = omap2_mcspi_setup;
	master->auto_runtime_pm = false;
	master->rt = 1;
	master->prepare_message = omap2_mcspi_prepare_message;
	master->can_dma = omap2_mcspi_can_dma;
	master->transfer_one = omap2_mcspi_transfer_one;
	master->set_cs = omap2_mcspi_set_cs;
	master->cleanup = omap2_mcspi_cleanup;
	master->slave_abort = omap2_mcspi_slave_abort;
	master->dev.of_node = node;
	master->max_speed_hz = OMAP2_MCSPI_MAX_FREQ;
	master->min_speed_hz = OMAP2_MCSPI_MAX_FREQ >> 15;
	init_swait_queue_head(&master->swait);
	
	platform_set_drvdata(pdev, master);

	mcspi = spi_master_get_devdata(master);
	mcspi->master = master;
	init_swait_queue_head(&mcspi->swait);
	raw_spin_lock_init(&mcspi->lock);

	match = of_match_device(omap_mcspi_of_match, &pdev->dev);
	if (match) {
		u32 num_cs = 1; /* default number of chipselect */
		pdata = match->data;

		of_property_read_u32(node, "ti,spi-num-cs", &num_cs);
		master->num_chipselect = num_cs;
		if (of_get_property(node, "ti,pindir-d0-out-d1-in", NULL))
			mcspi->pin_dir = MCSPI_PINDIR_D0_OUT_D1_IN;
	} else {
		pdata = dev_get_platdata(&pdev->dev);
		master->num_chipselect = pdata->num_cs;
		mcspi->pin_dir = pdata->pin_dir;
	}
	regs_offset = pdata->regs_offset;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mcspi->base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(mcspi->base)) {
		status = PTR_ERR(mcspi->base);
		goto free_master;
	}
	mcspi->phys = r->start + regs_offset;
	mcspi->base += regs_offset;

	mcspi->dev = &pdev->dev;

	INIT_LIST_HEAD(&mcspi->ctx.cs);

	mcspi->dma_channels = devm_kcalloc(&pdev->dev, master->num_chipselect,
					   sizeof(struct omap2_mcspi_dma),
					   GFP_KERNEL);
	if (mcspi->dma_channels == NULL) {
		status = -ENOMEM;
		goto free_master;
	}

	for (i = 0; i < master->num_chipselect; i++) {
		sprintf(mcspi->dma_channels[i].dma_rx_ch_name, "rx%d", i);
		sprintf(mcspi->dma_channels[i].dma_tx_ch_name, "tx%d", i);
	}

	status = platform_get_irq(pdev, 0);
	if (status == -EPROBE_DEFER)
		goto free_master;
	if (status < 0) {
		dev_err(&pdev->dev, "no irq resource found\n");
		goto free_master;
	}
	init_completion(&mcspi->txdone);
	status = devm_request_irq(&pdev->dev, status,
				omap2_mcspi_irq_handler_rt, IRQF_NO_THREAD, 
				dev_name(&pdev->dev), mcspi);
	if (status) {
		dev_err(&pdev->dev, "Cannot request IRQ");
		goto free_master;
	}

	pm_runtime_use_autosuspend(&pdev->dev);
	/* if delay is negative and the use_autosuspend flag is set
	 * then runtime suspends are prevented.
	 */
	pm_runtime_set_autosuspend_delay(&pdev->dev, PM_NEGATIVE_DELAY);
	pm_runtime_enable(&pdev->dev);
	status = pm_runtime_get_sync(&pdev->dev);
	if (status < 0) {
		dev_err(&pdev->dev, "%s: pm_runtime_get_sync error %d\n",
				__func__, status);
		return status;
	}
	
	status = omap2_mcspi_controller_setup(mcspi);
	if (status < 0)
		goto disable_pm;

	status = devm_spi_register_controller(&pdev->dev, master);
	if (status < 0)
		goto disable_pm;

	return status;

disable_pm:
free_master:
	spi_master_put(master);
	return status;
}

static int omap2_mcspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct omap2_mcspi *mcspi = spi_master_get_devdata(master);

	omap2_mcspi_reset_hw(mcspi);
	
	pm_runtime_dont_use_autosuspend(mcspi->dev);
	pm_runtime_put_sync(mcspi->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:omap2_mcspi");

static int __maybe_unused omap2_mcspi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct omap2_mcspi *mcspi = spi_master_get_devdata(master);
	int error;

	error = pinctrl_pm_select_sleep_state(dev);
	if (error)
		dev_warn(mcspi->dev, "%s: failed to set pins: %i\n",
			 __func__, error);

	error = spi_master_suspend(master);
	if (error)
		dev_warn(mcspi->dev, "%s: master suspend failed: %i\n",
			 __func__, error);

	return pm_runtime_force_suspend(dev);
}

static int __maybe_unused omap2_mcspi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct omap2_mcspi *mcspi = spi_master_get_devdata(master);
	int error;

	error = pinctrl_pm_select_default_state(dev);
	if (error)
		dev_warn(mcspi->dev, "%s: failed to set pins: %i\n",
			 __func__, error);

	error = spi_master_resume(master);
	if (error)
		dev_warn(mcspi->dev, "%s: master resume failed: %i\n",
			 __func__, error);

	return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops omap2_mcspi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(omap2_mcspi_suspend,
				omap2_mcspi_resume)
	.runtime_resume	= omap_mcspi_runtime_resume,
};

static struct platform_driver omap2_mcspi_driver = {
	.driver = {
		.name =		"omap2_mcspi",
		.pm =		&omap2_mcspi_pm_ops,
		.of_match_table = omap_mcspi_of_match,
	},
	.probe =	omap2_mcspi_probe,
	.remove =	omap2_mcspi_remove,
};

module_platform_driver(omap2_mcspi_driver);
MODULE_LICENSE("GPL");
