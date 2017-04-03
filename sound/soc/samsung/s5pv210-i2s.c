/* sound/soc/samsung/i2s.c
 *
 * ALSA SoC Audio Layer - Samsung I2S Controller driver
 *
 * Copyright (c) 2010 Samsung Electronics Co. Ltd.
 *	Jaswinder Singh <jassisinghbrar@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <linux/platform_data/asoc-s3c.h>

#include "dma.h"
#include "s5pv210-i2s.h"
#include "i2s-regs.h"

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

struct i2s_dai {
	/* Platform device for this DAI */
	struct platform_device *pdev;
	/* IOREMAP'd SFRs */
	void __iomem	*addr;
	/* Physical base address of SFRs */
	u32	base;
	/*
	 * Specifically requested RCLK,BCLK,PSR by MACHINE Driver.
	 */
	unsigned rfs, bfs, psr;
	/* I2S Controller's core clock */
	struct clk *clk;
	/* Clock for generating I2S signals */
	struct clk *op_clk;
	/* Pointer to the Primary_Fifo */
	struct i2s_dai *pri_dai;
	/* Driver for this DAI */
	struct snd_soc_dai_driver i2s_dai_drv;
	/* DMA parameters */
	struct s3c_dma_params dma_playback;
	struct s3c_dma_params dma_capture;
	u32	quirks;
};

/* Lock for cross i/f checks */
static DEFINE_SPINLOCK(lock);

/* If this interface of the controller is transmitting data */
static inline bool tx_active(struct i2s_dai *i2s)
{
	u32 active;

	if (!i2s)
		return false;

	active = readl(i2s->addr + I2SCON) & CON_TXDMA_ACTIVE;

	return active ? true : false;
}

/* If this interface of the controller is receiving data */
static inline bool rx_active(struct i2s_dai *i2s)
{
	u32 active;

	if (!i2s)
		return false;

	active = readl(i2s->addr + I2SCON) & CON_RXDMA_ACTIVE;

	return active ? true : false;
}

/* If the controller is active anyway */
static inline bool any_active(struct i2s_dai *i2s)
{
	return tx_active(i2s) || rx_active(i2s);
}

static inline struct i2s_dai *to_info(struct snd_soc_dai *dai)
{
	return snd_soc_dai_get_drvdata(dai);
}

/* TX Channel Control */
static void i2s_txctrl(struct i2s_dai *i2s, int on)
{
	void __iomem *addr = i2s->addr;
	u32 con = readl(addr + I2SCON);
	u32 mod = readl(addr + I2SMOD) & ~MOD_MASK;

	printk("+%s()\n", __FUNCTION__ );

	if (on) {
		con |= CON_ACTIVE;
		con &= ~CON_TXCH_PAUSE;

		con |= CON_TXDMA_ACTIVE;
		con &= ~CON_TXDMA_PAUSE;

		if (rx_active(i2s))
			mod |= MOD_TXRX;
		else
			mod |= MOD_TXONLY;
	} else {
		con |=  CON_TXDMA_PAUSE;
		con &= ~CON_TXDMA_ACTIVE;

		con |=  CON_TXCH_PAUSE;

		if (rx_active(i2s))
			mod |= MOD_RXONLY;
		else
			con &= ~CON_ACTIVE;
	}

	writel(mod, addr + I2SMOD);
	writel(con, addr + I2SCON);
}

/* RX Channel Control */
static void i2s_rxctrl(struct i2s_dai *i2s, int on)
{
	void __iomem *addr = i2s->addr;
	u32 con = readl(addr + I2SCON);
	u32 mod = readl(addr + I2SMOD) & ~MOD_MASK;

	printk("+%s()\n", __FUNCTION__ );

	if (on) {
		con |= CON_RXDMA_ACTIVE | CON_ACTIVE;
		con &= ~(CON_RXDMA_PAUSE | CON_RXCH_PAUSE);

		if (tx_active(i2s))
			mod |= MOD_TXRX;
		else
			mod |= MOD_RXONLY;
	} else {
		con |=  CON_RXDMA_PAUSE | CON_RXCH_PAUSE;
		con &= ~CON_RXDMA_ACTIVE;

		if (tx_active(i2s))
			mod |= MOD_TXONLY;
		else
			con &= ~CON_ACTIVE;
	}

	writel(mod, addr + I2SMOD);
	writel(con, addr + I2SCON);
}

/* Flush FIFO of an interface */
static inline void i2s_fifo(struct i2s_dai *i2s, u32 flush)
{
	void __iomem *fic;
	u32 val;

	printk("+%s()\n", __FUNCTION__ );

	if (!i2s)
		return;

	fic = i2s->addr + I2SFIC;

	/* Flush the FIFO */
	writel(readl(fic) | flush, fic);

	/* Be patient */
	val = msecs_to_loops(1) / 1000; /* 1 usec */
	while (--val)
		cpu_relax();

	writel(readl(fic) & ~flush, fic);
}

static void config_setup(struct i2s_dai *i2s)
{
	u32 mod = readl(i2s->addr + I2SMOD);

	printk("+%s()\n", __FUNCTION__ );
	printk("PSR=%d, RCLK=%dfs, BCLK=%dfs\n", i2s->psr, i2s->rfs, i2s->bfs);

	mod &= ~MOD_BCLK_MASK;
	switch (i2s->bfs) {
	case 48:
		mod |= MOD_BCLK_48FS;
		break;
	case 24:
		mod |= MOD_BCLK_24FS;
		break;
	case 16:
		mod |= MOD_BCLK_16FS;
		break;
	default:
		mod |= MOD_BCLK_32FS;
		break;
	}

	mod &= ~MOD_RCLK_MASK;
	switch (i2s->rfs) {
	case 768:
		mod |= MOD_RCLK_768FS;
		break;
	case 512:
		mod |= MOD_RCLK_512FS;
		break;
	case 384:
		mod |= MOD_RCLK_384FS;
		break;
	default:
		mod |= MOD_RCLK_256FS;
		break;
	}

	writel(mod, i2s->addr + I2SMOD);

	writel(((i2s->psr - 1) << 8) | PSR_PSREN, i2s->addr + I2SPSR);
}

static int i2s_trigger(struct snd_pcm_substream *substream,
	int cmd, struct snd_soc_dai *dai)
{
	int capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct i2s_dai *i2s = to_info(rtd->cpu_dai);
	unsigned long flags;

	printk("+%s(): %d\n", __FUNCTION__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		local_irq_save(flags);

		config_setup(i2s);

		if (capture)
			i2s_rxctrl(i2s, 1);
		else
			i2s_txctrl(i2s, 1);

		local_irq_restore(flags);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		local_irq_save(flags);

		if (capture) {
			i2s_rxctrl(i2s, 0);
			i2s_fifo(i2s, FIC_RXFLUSH);
		} else {
			i2s_txctrl(i2s, 0);
			i2s_fifo(i2s, FIC_TXFLUSH);
		}

		local_irq_restore(flags);
		break;
	}

	return 0;
}

static int i2s_set_clkdiv(struct snd_soc_dai *dai,
	int div_id, int div)
{
	struct i2s_dai *i2s = to_info(dai);

	printk("+%s()\n", __FUNCTION__ );

	switch (div_id) {
	case SAMSUNG_I2S_DIV_BCLK:
		i2s->bfs = div;
		break;
	case SAMSUNG_I2S_DIV_RCLK:
		i2s->rfs = div;
		break;
	case SAMSUNG_I2S_DIV_PRESCALER:
		i2s->psr = div;
		break;
	default:
		dev_err(&i2s->pdev->dev,
			"Invalid clock divider(%d)\n", div_id);
		return -EINVAL;
	}

	return 0;
}

static int i2s_set_sysclk(struct snd_soc_dai *dai,
	  int clk_id, unsigned int freq, int dir)
{
	struct i2s_dai *i2s = to_info(dai);
	u32 mod = readl(i2s->addr + I2SMOD);

	printk("+%s()\n", __FUNCTION__ );

	switch (clk_id) {
	case SAMSUNG_I2S_OPCLK:
		mod &= ~MOD_OPCLK_MASK;
		mod |= MOD_OPCLK_PCLK;
		break;
	case SAMSUNG_I2S_CDCLK:
		if (dir == SND_SOC_CLOCK_IN)
			mod |= MOD_CDCLKCON;
		else
			mod &= ~MOD_CDCLKCON;
		break;
	case SAMSUNG_I2S_RCLKSRC:
		mod |= MOD_IMS_SYSMUX;
		clk_disable(i2s->op_clk);
		clk_set_rate(i2s->op_clk, freq);
		clk_enable(i2s->op_clk);
		break;
	default:
		dev_err(&i2s->pdev->dev, "We don't serve that!\n");
		return -EINVAL;
	}

	writel(mod, i2s->addr + I2SMOD);

	return 0;
}

static int i2s_set_fmt(struct snd_soc_dai *dai,
	unsigned int fmt)
{
	struct i2s_dai *i2s = to_info(dai);
	u32 mod = readl(i2s->addr + I2SMOD);
	u32 tmp = 0;

	printk("+%s()\n", __FUNCTION__ );

	/* Format is priority */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
		tmp |= MOD_LR_RLOW;
		tmp |= MOD_SDF_MSB;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		tmp |= MOD_LR_RLOW;
		tmp |= MOD_SDF_LSB;
		break;
	case SND_SOC_DAIFMT_I2S:
		tmp |= MOD_SDF_IIS;
		break;
	default:
		dev_err(&i2s->pdev->dev, "Format not supported\n");
		return -EINVAL;
	}

	/*
	 * INV flag is relative to the FORMAT flag - if set it simply
	 * flips the polarity specified by the Standard
	 */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		if (tmp & MOD_LR_RLOW)
			tmp &= ~MOD_LR_RLOW;
		else
			tmp |= MOD_LR_RLOW;
		break;
	default:
		dev_err(&i2s->pdev->dev, "Polarity not supported\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		tmp |= MOD_SLAVE;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		/* Set default source clock in Master mode */
		break;
	default:
		dev_err(&i2s->pdev->dev, "master/slave format not supported\n");
		return -EINVAL;
	}

	mod &= ~(MOD_SDF_MASK | MOD_LR_RLOW | MOD_SLAVE);
	mod |= tmp;
	writel(mod, i2s->addr + I2SMOD);

	return 0;
}

static int i2s_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct i2s_dai *i2s = to_info(dai);
	u32 mod = readl(i2s->addr + I2SMOD);

	printk("+%s()\n", __FUNCTION__ );

	mod &= ~(MOD_DC2_EN | MOD_DC1_EN);
	switch (params_channels(params)) {
	case 2:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			i2s->dma_playback.dma_size = 4;
		else
			i2s->dma_capture.dma_size = 4;
		break;
	case 1:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			i2s->dma_playback.dma_size = 2;
		else
			i2s->dma_capture.dma_size = 2;
		break;
	default:
		dev_err(&i2s->pdev->dev, "%d channels not supported\n",
				params_channels(params));
		return -EINVAL;
	}

	mod &= ~MOD_BLCP_MASK;
	mod &= ~MOD_BLC_MASK;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		mod |= MOD_BLCP_8BIT;
		mod |= MOD_BLC_8BIT;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		mod |= MOD_BLCP_16BIT;
		mod |= MOD_BLC_16BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		mod |= MOD_BLCP_24BIT;
		mod |= MOD_BLC_24BIT;
		break;
	default:
		dev_err(&i2s->pdev->dev, "Format(%d) not supported\n",
				params_format(params));
		return -EINVAL;
	}
	writel(mod, i2s->addr + I2SMOD);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_dai_set_dma_data(dai, substream,
			(void *)&i2s->dma_playback);
	else
		snd_soc_dai_set_dma_data(dai, substream,
			(void *)&i2s->dma_capture);

	return 0;
}

/* We set constraints on the substream acc to the version of I2S */
static int i2s_startup(struct snd_pcm_substream *substream,
	  struct snd_soc_dai *dai)
{
	struct i2s_dai *i2s = to_info(dai);
	unsigned long flags;

	printk("+%s()\n", __FUNCTION__ );

	spin_lock_irqsave(&lock, flags);

	if (!any_active(i2s) && (i2s->quirks & QUIRK_NEED_RSTCLR))
		writel(CON_RSTCLR, i2s->addr + I2SCON);

	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static void i2s_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	printk("+%s()\n", __FUNCTION__ );

	i2s_set_sysclk(dai, SAMSUNG_I2S_CDCLK,
				0, SND_SOC_CLOCK_IN);
}

static snd_pcm_sframes_t
i2s_delay(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct i2s_dai *i2s = to_info(dai);
	u32 reg = readl(i2s->addr + I2SFIC);
	snd_pcm_sframes_t delay;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		delay = FIC_RXCOUNT(reg);
	else
		delay = FIC_TXCOUNT(reg);

	return delay;
}

static int samsung_i2s_dai_probe(struct snd_soc_dai *dai)
{
	void __iomem *ass_clk;
	struct i2s_dai *i2s = to_info(dai);

	ass_clk = ioremap(0xeee10000, 0x10);
	if (ass_clk == NULL) {
		dev_err(&i2s->pdev->dev, "cannot ioremap registers ass_clk\n");
		return -ENXIO;
	}
	writel(0x1, ass_clk);
	iounmap(ass_clk);
	printk("+%s(): open CLKSRC_AUDSS\n", __FUNCTION__ );

	i2s->addr = ioremap(i2s->base, 0x100);
	if (i2s->addr == NULL) {
		dev_err(&i2s->pdev->dev, "cannot ioremap registers\n");
		return -ENXIO;
	}

	i2s->clk = clk_get(&i2s->pdev->dev, "iis");
	if (IS_ERR(i2s->clk)) {
		dev_err(&i2s->pdev->dev, "failed to get i2s_clock\n");
		iounmap(i2s->addr);
		return -ENOENT;
	}
	clk_enable(i2s->clk);

	i2s->op_clk = clk_get(&i2s->pdev->dev, "fout_epll");
	if (IS_ERR(i2s->op_clk)) {
		dev_err(&i2s->pdev->dev, "failed to get fout_epll\n");
		clk_disable(i2s->clk);
		clk_put(i2s->clk);
		iounmap(i2s->addr);
		return -ENOENT;
	}
	clk_enable(i2s->op_clk);

	if (i2s->quirks & QUIRK_NEED_RSTCLR)
		writel(CON_RSTCLR, i2s->addr + I2SCON);

	i2s_txctrl(i2s, 0);
	i2s_rxctrl(i2s, 0);
	i2s_fifo(i2s, FIC_TXFLUSH);
	i2s_fifo(i2s, FIC_RXFLUSH);

	return 0;
}

static int samsung_i2s_dai_remove(struct snd_soc_dai *dai)
{
	struct i2s_dai *i2s = snd_soc_dai_get_drvdata(dai);

	printk("+%s()\n", __FUNCTION__ );

	if (i2s->quirks & QUIRK_NEED_RSTCLR)
		writel(0, i2s->addr + I2SCON);

	clk_disable(i2s->op_clk);
	clk_put(i2s->op_clk);

	clk_disable(i2s->clk);
	clk_put(i2s->clk);

	iounmap(i2s->addr);

	i2s->op_clk = NULL;
	i2s->clk = NULL;

	return 0;
}

static const struct snd_soc_dai_ops samsung_i2s_dai_ops = {
	.trigger = i2s_trigger,
	.hw_params = i2s_hw_params,
	.set_fmt = i2s_set_fmt,
	.set_clkdiv = i2s_set_clkdiv,
	.set_sysclk = i2s_set_sysclk,
	.startup = i2s_startup,
	.shutdown = i2s_shutdown,
	.delay = i2s_delay,
};

static const struct snd_soc_component_driver samsung_i2s_component = {
	.name		= "samsung-i2s",
};

#define SAMSUNG_I2S_RATES	SNDRV_PCM_RATE_8000_96000

#define SAMSUNG_I2S_FMTS	(SNDRV_PCM_FMTBIT_S8 | \
					SNDRV_PCM_FMTBIT_S16_LE | \
					SNDRV_PCM_FMTBIT_S24_LE)

static struct i2s_dai *i2s_alloc_dai(struct platform_device *pdev)
{
	struct i2s_dai *i2s;

	printk("+%s()\n", __FUNCTION__ );

	i2s = devm_kzalloc(&pdev->dev, sizeof(struct i2s_dai), GFP_KERNEL);
	if (i2s == NULL)
		return NULL;

	i2s->pdev = pdev;

	i2s->i2s_dai_drv.symmetric_rates = 1;
	i2s->i2s_dai_drv.probe = samsung_i2s_dai_probe;
	i2s->i2s_dai_drv.remove = samsung_i2s_dai_remove;
	i2s->i2s_dai_drv.ops = &samsung_i2s_dai_ops;
	i2s->i2s_dai_drv.suspend = NULL;
	i2s->i2s_dai_drv.resume = NULL;

	i2s->i2s_dai_drv.playback.channels_min = 2;
	i2s->i2s_dai_drv.playback.channels_max = 2;
	i2s->i2s_dai_drv.playback.rates = SAMSUNG_I2S_RATES;
	i2s->i2s_dai_drv.playback.formats = SAMSUNG_I2S_FMTS;
	i2s->i2s_dai_drv.capture.channels_min = 1;
	i2s->i2s_dai_drv.capture.channels_max = 2;
	i2s->i2s_dai_drv.capture.rates = SAMSUNG_I2S_RATES;
	i2s->i2s_dai_drv.capture.formats = SAMSUNG_I2S_FMTS;

	dev_set_drvdata(&i2s->pdev->dev, i2s);

	return i2s;
}

static int samsung_i2s_probe(struct platform_device *pdev)
{
	struct i2s_dai *pri_dai;
	struct s3c_audio_pdata *i2s_pdata = pdev->dev.platform_data;
	struct samsung_i2s *i2s_cfg = NULL;
	struct resource *res;
	u32 regs_base, quirks = 0;
	int ret = 0;

	printk("+%s()\n", __FUNCTION__ );

	pri_dai = i2s_alloc_dai(pdev);
	if (!pri_dai) {
		dev_err(&pdev->dev, "Unable to alloc I2S_pri\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Unable to get I2S-TX dma resource\n");
		return -ENXIO;
	}
	pri_dai->dma_playback.channel = res->start;

	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (!res) {
		dev_err(&pdev->dev,
			"Unable to get I2S-RX dma resource\n");
		return -ENXIO;
	}
	pri_dai->dma_capture.channel = res->start;

	if (i2s_pdata == NULL) {
		dev_err(&pdev->dev, "Can't work without s3c_audio_pdata\n");
		return -EINVAL;
	}

	if (&i2s_pdata->type)
		i2s_cfg = &i2s_pdata->type.i2s;

	if (i2s_cfg) {
		quirks = i2s_cfg->quirks;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get I2S SFR address\n");
		return -ENXIO;
	}

	if (!request_mem_region(res->start, resource_size(res),
							"samsung-i2s")) {
		dev_err(&pdev->dev, "Unable to request SFR region\n");
		return -EBUSY;
	}
	regs_base = res->start;

	pri_dai->dma_playback.dma_addr = regs_base + I2STXD;
	pri_dai->dma_capture.dma_addr = regs_base + I2SRXD;
	pri_dai->dma_playback.client =
		(struct s3c2410_dma_client *)&pri_dai->dma_playback;
	pri_dai->dma_playback.ch_name = "tx";
	pri_dai->dma_capture.client =
		(struct s3c2410_dma_client *)&pri_dai->dma_capture;
	pri_dai->dma_capture.ch_name = "rx";
	pri_dai->dma_playback.dma_size = 4;
	pri_dai->dma_capture.dma_size = 4;
	pri_dai->base = regs_base;
	pri_dai->quirks = quirks;

	if (i2s_pdata->cfg_gpio && i2s_pdata->cfg_gpio(pdev)) {
		dev_err(&pdev->dev, "Unable to configure gpio\n");
		ret = -EINVAL;
		goto err;
	}

	snd_soc_register_component(&pri_dai->pdev->dev, &samsung_i2s_component,
				   &pri_dai->i2s_dai_drv, 1);

	asoc_dma_platform_register(&pdev->dev);

	return 0;
err:
	release_mem_region(regs_base, resource_size(res));

	return ret;
}

static int samsung_i2s_remove(struct platform_device *pdev)
{
	struct resource *res;

	printk("+%s()\n", __FUNCTION__ );

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res)
		release_mem_region(res->start, resource_size(res));

	asoc_dma_platform_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);

	return 0;
}

static struct platform_driver samsung_i2s_driver = {
	.probe  = samsung_i2s_probe,
	.remove = samsung_i2s_remove,
	.driver = {
		.name = "samsung-i2s",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(samsung_i2s_driver);

/* Module information */
MODULE_AUTHOR("Jaswinder Singh, <jassisinghbrar@gmail.com>");
MODULE_DESCRIPTION("Samsung I2S Interface");
MODULE_ALIAS("platform:samsung-i2s");
MODULE_LICENSE("GPL");
