/*
 *  s5pv210_wm8960.c
 *
 *  Copyright (c) 2009 Samsung Electronics Co. Ltd
 *  Author: Jaswinder Singh <jassi.brar@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "../codecs/wm8960.h"
#include "s5pv210-i2s.h"


static int smdk_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int bfs, rfs, rclk, psr;
	int ret;

	printk("%s(): format=%d, rate=%d\n", __FUNCTION__, params_format(params), params_rate(params));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U8:
	case SNDRV_PCM_FORMAT_S8:
		bfs = 16;
		rfs = 256;
		break;

	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_S16_LE:
		bfs = 32;
		rfs = 512;
		break;

	case SNDRV_PCM_FORMAT_U24_LE:
	case SNDRV_PCM_FORMAT_S24_LE:
		bfs = 48;
		rfs = 768;
		break;

	default:
		printk(KERN_ERR "format Not yet supported!\n");
		return -EINVAL;
	}
	
	printk("%s(): bfs=%d, rfs=%d\n", __FUNCTION__, bfs, rfs);

	/* 
	case 32768000:
	case 45158400:
	case 49152000:
	case 67737600:
	case 73728000:
	*/
	rclk = params_rate(params) * rfs;
	switch (rclk) {
	case 2048000:
	case 2822400:
	case 3072000:
		psr = 16;
		break;

	case 4096000:
	case 5644800:
	case 6144000:
	case 8467200:
	case 9216000:
		psr = 8;
		break;

	case 8192000:
	case 11289600:
	case 12288000:
	case 16934400:
	case 18432000:
		psr = 4;
		break;

	case 16384000:
	case 22579200:
	case 24576000:
	case 33868800:
	case 36864000:
		psr = 2;
		break;

	case 32768000:
	case 45158400:
	case 49152000:
	case 67737600:
	case 73728000:
		psr = 1;
		break;

	default:
		printk(KERN_ERR "rclk=%d, Not yet supported!\n", rclk);
		return -EINVAL;
	}

	printk("%s(): rclk=%d, psr=%d\n", __FUNCTION__, rclk, psr);

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		printk("-%s(): Codec DAI configuration error, %d\n", __FUNCTION__, ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                                SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		printk("-%s(): AP DAI configuration error, %d\n", __FUNCTION__, ret);
		return ret;
	}

	snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_RCLKSRC, rclk*psr, 0);
	snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK, 0, 0);
	snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK, 0, SND_SOC_CLOCK_OUT);

	snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_BCLK, bfs);
	snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_RCLK, rfs);
	snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_PRESCALER, psr);

	return 0;
}

static struct snd_soc_ops smdk_ops = {
	.hw_params = smdk_hw_params,
};

static const struct snd_soc_dapm_widget smdk_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_HP("HP", NULL),
	SND_SOC_DAPM_MIC("MicIn", NULL),
};

static const struct snd_soc_dapm_route smdk_dapm_routes[] = {
	{"Speaker", NULL, "SPK_LP"},
	{"Speaker", NULL, "SPK_LN"},
	{"Speaker", NULL, "SPK_RP"},
	{"Speaker", NULL, "SPK_RN"},
	{"HP", NULL, "HP_L"},
	{"HP", NULL, "HP_R"},
#ifdef CONFIG_MACH_SATE210
	{"RINPUT2", NULL, "MicIn"},
#else
	{"LINPUT1", NULL, "MicIn"},
#endif
};

static int smdk_wm8960_init_paiftx(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* Enabling the microphone requires the fitting of a 0R
	 * resistor to connect the line from the microphone jack.
	 */
	snd_soc_dapm_disable_pin(dapm, "MicIn");

	return 0;
}

static struct snd_soc_dai_link smdk_dai[] = {
	{ /* Primary Playback i/f */
		.name = "WM8960 PAIF RX",
		.stream_name = "Playback",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm8960-hifi",
		.platform_name = "samsung-i2s.0",
#ifdef CONFIG_MACH_SATE210
		.codec_name = "wm8960.1-001a",
#else
		.codec_name = "wm8960.0-001a",
#endif
		.ops = &smdk_ops,
	},
	{ /* Primary Capture i/f */
		.name = "WM8960 PAIF TX",
		.stream_name = "Capture",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm8960-hifi",
		.platform_name = "samsung-i2s.0",
#ifdef CONFIG_MACH_SATE210
		.codec_name = "wm8960.1-001a",
#else
		.codec_name = "wm8960.0-001a",
#endif
		.init = smdk_wm8960_init_paiftx,
		.ops = &smdk_ops,
	},
};

static struct snd_soc_card smdk = {
	.name = "S5PV210-WM8960",
	.owner = THIS_MODULE,
	.dai_link = smdk_dai,
	.num_links = ARRAY_SIZE(smdk_dai),

	.dapm_widgets = smdk_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(smdk_dapm_widgets),
	.dapm_routes = smdk_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(smdk_dapm_routes),
};

static struct platform_device *smdk_snd_device;

static int __init smdk_audio_init(void)
{
	int ret;

	smdk_snd_device = platform_device_alloc("soc-audio", -1);
	if (!smdk_snd_device)
		return -ENOMEM;

	platform_set_drvdata(smdk_snd_device, &smdk);
	ret = platform_device_add(smdk_snd_device);

	if (ret)
		platform_device_put(smdk_snd_device);

	return ret;
}
module_init(smdk_audio_init);

static void __exit smdk_audio_exit(void)
{
	platform_device_unregister(smdk_snd_device);
}
module_exit(smdk_audio_exit);

MODULE_AUTHOR("Jaswinder Singh, jassi.brar@samsung.com");
MODULE_DESCRIPTION("ALSA SoC S5PV210 WM8960");
MODULE_LICENSE("GPL");
