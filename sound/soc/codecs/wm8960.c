/*
 * wm8960.c  --  WM8960 ALSA SoC Audio driver
 *
 * Copyright 2007-11 Wolfson Microelectronics, plc
 *
 * Author: Liam Girdwood
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/wm8960.h>

#include "wm8960.h"

/* R25 - Power 1 */
#define WM8960_VMID_MASK 0x180
#define WM8960_VREF      0x40

/* R26 - Power 2 */
#define WM8960_PWR2_LOUT1	0x40
#define WM8960_PWR2_ROUT1	0x20

/* R28 - Anti-pop 1 */
#define WM8960_POBCTRL   0x80
#define WM8960_BUFDCOPEN 0x10
#define WM8960_SOFT_ST   0x04

/*
 * wm8960 register cache
 * We can't read the WM8960 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const struct reg_default wm8960_reg_defaults[] = {
	{  0x0, 0x0097 },
	{  0x1, 0x0097 },
	{  0x2, 0x0000 },
	{  0x3, 0x0000 },
	{  0x4, 0x0000 },
	{  0x5, 0x0008 },
	{  0x6, 0x0000 },
	{  0x7, 0x000a },
	{  0x8, 0x01c0 },
	{  0x9, 0x0000 },
	{  0xa, 0x00ff },
	{  0xb, 0x00ff },

	{ 0x10, 0x0000 },
	{ 0x11, 0x007b },
	{ 0x12, 0x0100 },
	{ 0x13, 0x0032 },
	{ 0x14, 0x0000 },
	{ 0x15, 0x00c3 },
	{ 0x16, 0x00c3 },
	{ 0x17, 0x01c0 },
	{ 0x18, 0x0000 },
	{ 0x19, 0x0000 },
	{ 0x1a, 0x0000 },
	{ 0x1b, 0x0000 },
	{ 0x1c, 0x0000 },
	{ 0x1d, 0x0000 },

	{ 0x20, 0x0100 },
	{ 0x21, 0x0100 },
	{ 0x22, 0x0050 },

	{ 0x25, 0x0050 },
	{ 0x26, 0x0000 },
	{ 0x27, 0x0000 },
	{ 0x28, 0x0000 },
	{ 0x29, 0x0000 },
	{ 0x2a, 0x0040 },
	{ 0x2b, 0x0000 },
	{ 0x2c, 0x0000 },
	{ 0x2d, 0x0050 },
	{ 0x2e, 0x0050 },
	{ 0x2f, 0x0000 },
	{ 0x30, 0x0002 },
	{ 0x31, 0x0037 },

	{ 0x33, 0x0080 },
	{ 0x34, 0x0008 },
	{ 0x35, 0x0031 },
	{ 0x36, 0x0026 },
	{ 0x37, 0x00e9 },
};

static bool wm8960_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case WM8960_RESET:
		return true;
	default:
		return false;
	}
}

struct wm8960_priv {
	struct regmap *regmap;
};

#define wm8960_reset(c)	snd_soc_write(c, WM8960_RESET, 0)

static int wm8960_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	printk("%s(): fmt=%d\n", __FUNCTION__, fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	/* set iface */
	snd_soc_write(codec, WM8960_IFACE1, iface);
	return 0;
}

static struct {
	int rate;
	unsigned int val;
} alc_rates[] = {
	{ 48000, 0 },
	{ 44100, 0 },
	{ 32000, 1 },
	{ 22050, 2 },
	{ 24000, 2 },
	{ 16000, 3 },
	{ 11025, 4 },
	{ 12000, 4 },
	{  8000, 5 },
};

static int wm8960_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 iface = snd_soc_read(codec, WM8960_IFACE1) & 0xfff3;
	snd_pcm_format_t format = params_format(params);
	int i;

	/* bit size */
	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_BE:
		iface |= 0x0008;
		break;
	default:
		dev_err(codec->dev, "unsupported format %i\n", format);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(alc_rates); i++)
		if (alc_rates[i].rate == params_rate(params))
			snd_soc_update_bits(codec,
					    WM8960_ADDCTL3, 0x7,
					    alc_rates[i].val);

	/* set iface */
	snd_soc_write(codec, WM8960_IFACE1, iface);
	return 0;
}

static int wm8960_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	printk("%s(): mute=%d\n", __FUNCTION__, mute);

	if (mute) {
		snd_soc_update_bits(codec, WM8960_DACCTL1, 0x8, 0x8);
	} else {
		snd_soc_update_bits(codec, WM8960_DACCTL1, 0x8, 0);
	}
	return 0;
}

typedef enum {
	NONE,
	PLAY,
	RECORD,
} tMode;

static tMode wm8960_mode = NONE;

static void wm8960_mode_play(struct snd_soc_codec *codec)
{
	if (wm8960_mode == PLAY)
		return;

	// Startup Order :- DAC --> Mixers --> Output PGA --> Digital Unmute

	/*
	Left/Right Channel DAC Enable
	WM8960_POWER2
	bit8(DACL),bit7(DACR)
	1,1
	*/
	snd_soc_update_bits(codec, WM8960_POWER2, 0x180, 0x180);	//0x1a

	/*
	DAC to Output Mixer
	WM8960_LOUTMIX
	bit8(LD2LO)
	1
	WM8960_ROUTMIX
	bit8(RD2RO)
	1
	*/
	snd_soc_update_bits(codec, WM8960_LOUTMIX, 0x100, 0x100);	//0x22
	snd_soc_update_bits(codec, WM8960_ROUTMIX, 0x100, 0x100);	//0x25

	/*
	Output Mixer Enable Control
	WM8960_POWER3
	bit3(LOMIX),bit2(ROMIX)
	1,1
	*/
	snd_soc_update_bits(codec, WM8960_POWER3, 0xc, 0xc);		//0x2f

	/*
	Headphone Volume
	WM8960_LOUT1
	bit[6:0](LOUT1VOL)
	;
	WM8960_ROUT1
	bit[6:0](ROUT1VOL)
	;
	*/
	snd_soc_update_bits(codec, WM8960_LOUT1, 0x17f, 0x158);		//0x2 45%
	snd_soc_update_bits(codec, WM8960_ROUT1, 0x17f, 0x158);		//0x3 45%

	wm8960_mode = PLAY;

	printk("%s(): ok\n", __FUNCTION__);
}

static void wm8960_mode_record(struct snd_soc_codec *codec)
{
	if (wm8960_mode == RECORD)
		return;

	// Startup Order - Input PGA --> Mixers --> ADC

	/*
	WM8960_ADDCTL4
	bit0(MBSEL)
	1/0
	*/
	snd_soc_update_bits(codec, WM8960_ADDCTL4, 0x1, 0x0);		//0x30

	/*
	WM8960_POWER1
	bit1(MICB)
	1
	*/
	snd_soc_update_bits(codec, WM8960_POWER1, 0x2, 0x2);		//0x19

	/*
	WM8960_INBMIX1
	bit[3:1](LIN2BOOST)
	111 6db
	*/
	snd_soc_update_bits(codec, WM8960_INBMIX1, 0xe, 0xe);		//0x2b

	/*
	WM8960_POWER1
	bit5(AINL)
	1
	*/
	snd_soc_update_bits(codec, WM8960_POWER1, 0x20, 0x20);		//0x19

	/*
	WM8960_POWER1
	bit3(ADCL)
	1
	*/
	snd_soc_update_bits(codec, WM8960_POWER1, 0x8, 0x8);		//0x19

	/*
	WM8960_IFACE2
	bit6(ALRCGPIO)
	1
	*/
	snd_soc_update_bits(codec, WM8960_IFACE2, 0x40, 0x40);		//0x09

	wm8960_mode = RECORD;

	printk("%s(): ok\n", __FUNCTION__);
}

#define TEST_LOOPBACK 0

#if TEST_LOOPBACK
static void wm8960_mode_loopback(struct snd_soc_codec *codec)
{
	/*
	WM8960_INBMIX1
	bit[3:1](LIN2BOOST)
	111 6db
	*/
	snd_soc_update_bits(codec, WM8960_INBMIX1, 0xe, 0xe);		//0x2b

	/*
	WM8960_POWER1
	bit5(AINL),bit1(MICB)
	1,1
	*/
	snd_soc_update_bits(codec, WM8960_POWER1, 0x22, 0x22);		//0x19

	/*
	WM8960_ADDCTL4
	bit0(MBSEL)
	1/0
	*/
	snd_soc_update_bits(codec, WM8960_ADDCTL4, 0x1, 0x0);		//0x30

	/*
	WM8960_BYPASS1
	bit7(LB2LO)
	1
	bit[6:4](LB2LOVOL)
	000 0db
	*/
	snd_soc_update_bits(codec, WM8960_BYPASS1, 0xf0, 0x80);		//0x2d

	/*
	Output Mixer Enable Control
	WM8960_POWER3
	bit3(LOMIX),bit2(ROMIX)
	1,1
	*/
	snd_soc_update_bits(codec, WM8960_POWER3, 0xc, 0xc);		//0x2f

	/*
	Headphone Volume
	WM8960_LOUT1
	bit[6:0](LOUT1VOL)
	;
	WM8960_ROUT1
	bit[6:0](ROUT1VOL)
	;
	*/
	snd_soc_update_bits(codec, WM8960_LOUT1, 0x17f, 0x17f);		//0x2 100%
	snd_soc_update_bits(codec, WM8960_ROUT1, 0x17f, 0x17f);		//0x3 100%

	wm8960_mode = NONE;

	printk("%s(): ok\n", __FUNCTION__);
}
#endif

static int wm8960_startup(struct snd_pcm_substream *substream,
	  struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	printk("%s(): substream->stream=%d\n", __FUNCTION__, substream->stream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		printk("%s(): play\n", __FUNCTION__);

		wm8960_mode_play(codec);
	} else {
		printk("%s(): record\n", __FUNCTION__);

		wm8960_mode_record(codec);
	}

	return 0;
}

static void wm8960_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	printk("%s(): substream->stream=%d\n", __FUNCTION__, substream->stream);
}

static const char *bias[] = {
"off",
"standby",
"prepare",
"on",
};

static int wm8960_set_bias_level(struct snd_soc_codec *codec,
				      enum snd_soc_bias_level level)
{
	struct wm8960_priv *wm8960 = snd_soc_codec_get_drvdata(codec);

	printk("level: %s -> %s\n", bias[codec->dapm.bias_level], bias[level]);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		regcache_sync(wm8960->regmap);

		/* Enable anti pop mode */
		snd_soc_update_bits(codec, WM8960_APOP1,
				    WM8960_POBCTRL | WM8960_SOFT_ST |
				    WM8960_BUFDCOPEN,
				    WM8960_POBCTRL | WM8960_SOFT_ST |
				    WM8960_BUFDCOPEN);

		/* Enable LOUT1, ROUT1 */
		snd_soc_update_bits(codec, WM8960_POWER2,
				    WM8960_PWR2_LOUT1 | WM8960_PWR2_ROUT1,
				    WM8960_PWR2_LOUT1 | WM8960_PWR2_ROUT1);

		/* Enable VMID at 2*50k */
		snd_soc_update_bits(codec, WM8960_POWER1,
				    WM8960_VMID_MASK, 0x80);

		/* Ramp */
		msleep(100);

		/* Enable VREF */
		snd_soc_update_bits(codec, WM8960_POWER1,
				    WM8960_VREF, WM8960_VREF);

		msleep(100);
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}

	codec->dapm.bias_level = level;

	return 0;
}

#define WM8960_RATES SNDRV_PCM_RATE_8000_48000

#define WM8960_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_dai_ops wm8960_dai_ops = {
	.hw_params = wm8960_hw_params,
	.digital_mute = wm8960_mute,
	.set_fmt = wm8960_set_dai_fmt,
	.startup = wm8960_startup,
	.shutdown = wm8960_shutdown,
};

static struct snd_soc_dai_driver wm8960_dai = {
	.name = "wm8960-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8960_RATES,
		.formats = WM8960_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8960_RATES,
		.formats = WM8960_FORMATS,},
	.ops = &wm8960_dai_ops,
	.symmetric_rates = 1,
};

static int wm8960_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int wm8960_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static int wm8960_probe(struct snd_soc_codec *codec)
{
	int ret;

	ret = snd_soc_codec_set_cache_io(codec, 7, 9, SND_SOC_REGMAP);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	ret = wm8960_reset(codec);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to issue reset\n");
		return ret;
	}

	wm8960_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

#if TEST_LOOPBACK
	wm8960_mode_loopback(codec);
#endif

	return 0;
}

/* power down chip */
static int wm8960_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_wm8960 = {
	.probe =	wm8960_probe,
	.remove =	wm8960_remove,
	.suspend =	wm8960_suspend,
	.resume =	wm8960_resume,
};

static const struct regmap_config wm8960_regmap = {
	.reg_bits = 7,
	.val_bits = 9,
	.max_register = WM8960_PLL4,

	.reg_defaults = wm8960_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(wm8960_reg_defaults),
	.cache_type = REGCACHE_RBTREE,

	.volatile_reg = wm8960_volatile,
};

static int wm8960_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct wm8960_priv *wm8960;
	int ret;

	wm8960 = devm_kzalloc(&i2c->dev, sizeof(struct wm8960_priv),
			      GFP_KERNEL);
	if (wm8960 == NULL)
		return -ENOMEM;

	wm8960->regmap = devm_regmap_init_i2c(i2c, &wm8960_regmap);
	if (IS_ERR(wm8960->regmap))
		return PTR_ERR(wm8960->regmap);

	i2c_set_clientdata(i2c, wm8960);

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_wm8960, &wm8960_dai, 1);

	return ret;
}

static int wm8960_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct i2c_device_id wm8960_i2c_id[] = {
	{ "wm8960", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wm8960_i2c_id);

static struct i2c_driver wm8960_i2c_driver = {
	.driver = {
		.name = "wm8960",
		.owner = THIS_MODULE,
	},
	.probe =    wm8960_i2c_probe,
	.remove =   wm8960_i2c_remove,
	.id_table = wm8960_i2c_id,
};

module_i2c_driver(wm8960_i2c_driver);

MODULE_DESCRIPTION("ASoC WM8960 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
