// SPDX-License-Identifier: GPL-2.0
/*
 * Fullhan FH8626 ASoC audio controller and integrated codec driver
 *
 * The controller (ACW) moves 24-bit mono samples through 32-bit FIFOs with
 * DMAengine cyclic transfers; the integrated codec is programmed through the
 * same register block.
 *
 * Copyright (C) 2026 Difan Zhang
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <sound/core.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#define ACW_CTRL		0x00
#define  ACW_CTRL_ENABLE			(BIT(28) | BIT(29))

#define ACW_TXFIFO_CTRL		0x04
#define  ACW_TXFIFO_ENABLE		0x00010022
#define  ACW_TXFIFO_DISABLE		0x00010010

#define ACW_RXFIFO_CTRL		0x08
#define  ACW_RXFIFO_ENABLE		0x00010022
#define  ACW_RXFIFO_DISABLE		0x00010010

#define ACW_MISC_CTRL		0x0c
#define  ACW_MISC_CTRL_VAL		0x22

#define ACW_CHANNEL_CTRL	0x10

#define ACW_DIG_VOL		0x14

#define ACW_TXFIFO_DATA		0x100
#define ACW_RXFIFO_DATA		0x200

#define ACODEC_ADC_PATH		0x80
#define  ACODEC_ADC_PATH_MIC		0x01
#define  ACODEC_ADC_PATH_LINE		0x04

#define ACODEC_ADC_VOL		0x84
#define  ACODEC_ADC_VOL_GAIN_MASK	GENMASK(6, 2)
#define  ACODEC_ADC_VOL_GAIN_SHIFT	2
#define  ACODEC_ADC_VOL_BOOST_MASK	GENMASK(1, 0)
#define  ACODEC_ADC_VOL_GAIN_DEFAULT	23
#define  ACODEC_ADC_VOL_BOOST_DEFAULT	2
#define  ACODEC_ADC_VOL_DEFAULT		((ACODEC_ADC_VOL_GAIN_DEFAULT << \
					  ACODEC_ADC_VOL_GAIN_SHIFT) | \
					 ACODEC_ADC_VOL_BOOST_DEFAULT)

#define ACODEC_ADC_CFG0		0x88
#define ACODEC_ADC_PWR		0x8c
#define  ACODEC_ADC_PWR_ENABLE		0x03

#define ACODEC_BIAS_CFG		0x90
#define  ACODEC_BIAS_CFG_VAL		0x7a

#define ACODEC_DAC_OUTPUT_CTRL	0x94
/* Analog output attenuation: 3 is the quietest step, 0 the loudest */
#define  ACODEC_DAC_OUTPUT_DEFAULT	0x03
#define  ACODEC_DAC_OUTPUT_MAX_ATTEN	0x03

#define ACODEC_DAC_CFG0		0x98
#define  ACODEC_DAC_CFG0_VAL		0x03

#define ACODEC_DAC_MUTE		0x9c
#define  ACODEC_DAC_UNMUTE		0x00
#define  ACODEC_DAC_MUTE_VAL		0x80

#define ACODEC_DAC_CFG1		0xa0
#define  ACODEC_DAC_CFG1_VAL		0x10

#define ACODEC_DAC_VOL		0xa4
#define  ACODEC_DAC_VOL_MIN		1
#define  ACODEC_DAC_VOL_BASE		0x80

#define ACODEC_DAC_CFG2		0xa8
#define  ACODEC_DAC_CFG2_VAL		0x80

#define ACW_PLL_BASE		0xac
#define ACW_PLL_FAMILY		0xb0
#define ACW_ADC_RATE_DIV	0xb4
#define ACW_DAC_RATE_DIV	0xb8
#define ACW_PLL_FRAC0		0xbc
#define ACW_PLL_FRAC1		0xc0
#define ACW_PLL_FRAC2		0xc4

#define FH8626_AUDIO_RATES	(SNDRV_PCM_RATE_8000 | \
				 SNDRV_PCM_RATE_11025 | \
				 SNDRV_PCM_RATE_16000 | \
				 SNDRV_PCM_RATE_22050 | \
				 SNDRV_PCM_RATE_32000 | \
				 SNDRV_PCM_RATE_44100 | \
				 SNDRV_PCM_RATE_48000)

/* One signed 24-bit sample in each 32-bit FIFO word */
#define FH8626_AUDIO_FORMATS	SNDRV_PCM_FMTBIT_S24_LE

#define FH8626_DAC_RAMP_DELAY_MS	5

struct fh8626_audio {
	struct device *dev;
	void __iomem *base;
	resource_size_t phys_base;
	spinlock_t lock;
	struct mutex control_lock;

	struct clk_bulk_data clks[4];
	int num_clks;

	struct gpio_desc *spk_gpio;

	unsigned int capture_vol;
	unsigned int mic_boost;
	unsigned int input_source;
	unsigned int playback_vol;
	unsigned int speaker_gain;
	unsigned int playback_switch;
	bool playback_prepared;

	struct snd_soc_card card;
	struct snd_soc_dai_link dai_link;
};

static struct fh8626_audio *fh8626_component_priv(struct snd_soc_component *c)
{
	/*
	 * Both components share the card's device; its drvdata holds the card.
	 */
	return snd_soc_card_get_drvdata(c->card);
}

static int fh8626_codec_update_bits(struct fh8626_audio *priv,
				   unsigned int *cache, unsigned int value,
				   u32 reg, u32 mask, u32 hw_value)
{
	int ret = 0;
	unsigned long flags;

	mutex_lock(&priv->control_lock);
	if (*cache == value)
		goto out;
	/* Mixer controls also run while no PCM stream holds the clocks */
	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret)
		goto out;
	spin_lock_irqsave(&priv->lock, flags);
	writel((readl(priv->base + reg) & ~mask) | (hw_value & mask),
	       priv->base + reg);
	WRITE_ONCE(*cache, value);
	spin_unlock_irqrestore(&priv->lock, flags);
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
	ret = 1;
out:
	mutex_unlock(&priv->control_lock);
	return ret;
}

static int fh8626_codec_update(struct fh8626_audio *priv, unsigned int *cache,
			      unsigned int value, u32 reg, u32 hw_value)
{
	return fh8626_codec_update_bits(priv, cache, value, reg, ~0U, hw_value);
}

static const u32 pll_12m288[] = {
	0x03, 0x07, 0x0f, 0x07, 0x31,
	0x26, 0xe9, 0x21, 0x07, 0x0d
};

static int fh8626_rate_period(unsigned int rate)
{
	switch (rate) {
	case 8000:
		return 192;
	case 11025:
		return 128;
	case 16000:
		return 96;
	case 22050:
		return 64;
	case 32000:
		return 48;
	case 44100:
	case 48000:
		return 32;
	default:
		return -EINVAL;
	}
}

static void fh8626_hw_init(struct fh8626_audio *priv)
{
	unsigned int i;

	writel(ACW_CTRL_ENABLE, priv->base + ACW_CTRL);
	writel(0, priv->base + ACW_TXFIFO_CTRL);
	writel(0, priv->base + ACW_RXFIFO_CTRL);
	writel(ACW_MISC_CTRL_VAL, priv->base + ACW_MISC_CTRL);
	/* Zero selects the integrated codec. */
	writel(0, priv->base + ACW_CHANNEL_CTRL);
	writel(0x00, priv->base + ACW_DIG_VOL);

	writel(ACODEC_ADC_PATH_MIC, priv->base + ACODEC_ADC_PATH);
	writel(ACODEC_ADC_VOL_DEFAULT, priv->base + ACODEC_ADC_VOL);
	writel(0x00, priv->base + ACODEC_ADC_CFG0);
	writel(0x02, priv->base + ACODEC_ADC_PWR);
	writel(ACODEC_ADC_PWR_ENABLE, priv->base + ACODEC_ADC_PWR);
	writel(ACODEC_BIAS_CFG_VAL, priv->base + ACODEC_BIAS_CFG);
	/*
	 * Mute and minimum gain before the analog output leaves reset;
	 * full-scale output at probe is audible even with the amplifier off.
	 */
	writel(ACODEC_DAC_MUTE_VAL, priv->base + ACODEC_DAC_MUTE);
	writel(ACODEC_DAC_VOL_MIN, priv->base + ACODEC_DAC_VOL);
	writel(ACODEC_DAC_CFG0_VAL, priv->base + ACODEC_DAC_CFG0);
	writel(ACODEC_DAC_OUTPUT_DEFAULT, priv->base + ACODEC_DAC_OUTPUT_CTRL);
	writel(ACODEC_DAC_CFG1_VAL, priv->base + ACODEC_DAC_CFG1);
	writel(ACODEC_DAC_CFG2_VAL, priv->base + ACODEC_DAC_CFG2);

	priv->capture_vol = ACODEC_ADC_VOL_DEFAULT >>
			    ACODEC_ADC_VOL_GAIN_SHIFT;
	priv->mic_boost = ACODEC_ADC_VOL_DEFAULT & ACODEC_ADC_VOL_BOOST_MASK;
	priv->input_source = 0;
	priv->playback_vol = ACODEC_DAC_VOL_BASE - 1;
	priv->speaker_gain = 0;
	priv->playback_switch = 1;

	for (i = 0; i < ARRAY_SIZE(pll_12m288); i++)
		writel(pll_12m288[i], priv->base + ACW_PLL_BASE + i * 4);
}

static int fh8626_acw_dai_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct fh8626_audio *priv = fh8626_component_priv(dai->component);
	int ret;

	/* FIFO accesses require complete 32-bit DMA words */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
					SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 4);
	if (ret < 0)
		return ret;
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
					SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 4);
	if (ret < 0)
		return ret;
	return clk_bulk_prepare_enable(priv->num_clks, priv->clks);
}

static unsigned int fh8626_playback_hw_vol_locked(struct fh8626_audio *priv)
{
	unsigned int vol = priv->playback_vol + 1;

	if (vol < ACODEC_DAC_VOL_MIN)
		vol = ACODEC_DAC_VOL_MIN;
	if (vol > ACODEC_DAC_VOL_BASE)
		vol = ACODEC_DAC_VOL_BASE;
	return vol;
}

static unsigned int fh8626_speaker_hw_atten(struct fh8626_audio *priv)
{
	return ACODEC_DAC_OUTPUT_MAX_ATTEN - priv->speaker_gain;
}

static void fh8626_playback_stop(struct fh8626_audio *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->playback_prepared = false;
	if (priv->spk_gpio)
		gpiod_set_value(priv->spk_gpio, 0);
	writel(ACODEC_DAC_MUTE_VAL, priv->base + ACODEC_DAC_MUTE);
	writel(ACODEC_DAC_VOL_MIN, priv->base + ACODEC_DAC_VOL);
	writel(ACODEC_DAC_OUTPUT_DEFAULT, priv->base + ACODEC_DAC_OUTPUT_CTRL);
	writel(ACW_TXFIFO_DISABLE, priv->base + ACW_TXFIFO_CTRL);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static void fh8626_acw_dai_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct fh8626_audio *priv = fh8626_component_priv(dai->component);

	/* Closing after PREPARE may occur before START or STOP. */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		fh8626_playback_stop(priv);
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
}

static int fh8626_acw_dai_hw_free(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct fh8626_audio *priv = fh8626_component_priv(dai->component);

	/*
	 * HW_FREE may follow PREPARE without START/STOP. Cancel pending mixer
	 * enable while keeping the startup clock reference until shutdown.
	 */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		fh8626_playback_stop(priv);
	else
		writel(ACW_RXFIFO_DISABLE, priv->base + ACW_RXFIFO_CTRL);
	return 0;
}

/*
 * Caller holds control_lock and a clock reference. STOP may cancel the
 * prepared state under the lock; PREPARE cannot replace it during the waits.
 */
static void fh8626_playback_enable(struct fh8626_audio *priv)
{
	unsigned long flags;
	bool enable;
	unsigned int step, target;

	spin_lock_irqsave(&priv->lock, flags);
	enable = priv->playback_prepared && priv->playback_switch;
	if (enable) {
		if (priv->spk_gpio)
			gpiod_set_value(priv->spk_gpio, 0);
		writel(ACODEC_DAC_MUTE_VAL, priv->base + ACODEC_DAC_MUTE);
		writel(ACODEC_DAC_VOL_MIN, priv->base + ACODEC_DAC_VOL);
		writel(ACODEC_DAC_OUTPUT_DEFAULT,
		       priv->base + ACODEC_DAC_OUTPUT_CTRL);
	}
	spin_unlock_irqrestore(&priv->lock, flags);
	if (!enable)
		return;
	/*
	 * Settle the DAC bias, then the amplifier, while muted at minimum gain.
	 * Ramp up after unmuting to avoid a pop at the saved gain.
	 */
	msleep(80);
	spin_lock_irqsave(&priv->lock, flags);
	enable = priv->playback_prepared && priv->playback_switch;
	if (enable && priv->spk_gpio)
		gpiod_set_value(priv->spk_gpio, 1);
	spin_unlock_irqrestore(&priv->lock, flags);
	if (!enable)
		return;
	msleep(80);
	spin_lock_irqsave(&priv->lock, flags);
	enable = priv->playback_prepared && priv->playback_switch;
	target = fh8626_playback_hw_vol_locked(priv);
	if (enable) {
		writel(ACODEC_DAC_UNMUTE, priv->base + ACODEC_DAC_MUTE);
		writel(fh8626_speaker_hw_atten(priv),
		       priv->base + ACODEC_DAC_OUTPUT_CTRL);
	}
	spin_unlock_irqrestore(&priv->lock, flags);
	if (!enable)
		return;
	for (step = ACODEC_DAC_VOL_MIN; step < target;) {
		unsigned int next = step + (target / 8 ? target / 8 : 1);

		if (next > target)
			next = target;
		msleep(FH8626_DAC_RAMP_DELAY_MS);
		spin_lock_irqsave(&priv->lock, flags);
		enable = priv->playback_prepared && priv->playback_switch;
		if (enable) {
			target = fh8626_playback_hw_vol_locked(priv);
			if (next > target)
				next = target;
			writel(next, priv->base + ACODEC_DAC_VOL);
		}
		spin_unlock_irqrestore(&priv->lock, flags);
		if (!enable)
			return;
		step = next;
	}
}

static int fh8626_acw_dai_prepare(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct fh8626_audio *priv = fh8626_component_priv(dai->component);
	unsigned long flags;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;
	mutex_lock(&priv->control_lock);
	spin_lock_irqsave(&priv->lock, flags);
	priv->playback_prepared = true;
	writel(ACW_TXFIFO_DISABLE, priv->base + ACW_TXFIFO_CTRL);
	if (priv->spk_gpio)
		gpiod_set_value(priv->spk_gpio, 0);
	writel(ACODEC_DAC_MUTE_VAL, priv->base + ACODEC_DAC_MUTE);
	writel(ACODEC_DAC_OUTPUT_DEFAULT, priv->base + ACODEC_DAC_OUTPUT_CTRL);
	spin_unlock_irqrestore(&priv->lock, flags);
	fh8626_playback_enable(priv);
	mutex_unlock(&priv->control_lock);
	return 0;
}

static int fh8626_acw_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct fh8626_audio *priv = fh8626_component_priv(dai->component);
	unsigned int rate = params_rate(params);
	bool family_44k = rate == 11025 || rate == 22050 || rate == 44100;
	int period;
	unsigned long flags;

	period = fh8626_rate_period(rate);
	if (period < 0)
		return period;
	if (params_channels(params) != 1)
		return -EINVAL;

	/*
	 * The PLL family is shared by both directions; the ADC and DAC each
	 * have their own sample-rate divider.
	 */
	spin_lock_irqsave(&priv->lock, flags);
	writel(family_44k ? 6 : 7, priv->base + ACW_PLL_FAMILY);
	writel(family_44k ? 0x86 : 0x31, priv->base + ACW_PLL_FRAC0);
	writel(family_44k ? 0xc2 : 0x26, priv->base + ACW_PLL_FRAC1);
	writel(family_44k ? 0x26 : 0xe9, priv->base + ACW_PLL_FRAC2);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		writel(period / 8 - 1, priv->base + ACW_DAC_RATE_DIV);
	else
		writel(period / 4 - 1, priv->base + ACW_ADC_RATE_DIV);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int fh8626_acw_dai_trigger(struct snd_pcm_substream *substream,
				  int cmd,
				  struct snd_soc_dai *dai)
{
	struct fh8626_audio *priv = fh8626_component_priv(dai->component);
	unsigned long flags;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			spin_lock_irqsave(&priv->lock, flags);
			writel(ACW_TXFIFO_ENABLE,
			       priv->base + ACW_TXFIFO_CTRL);
			spin_unlock_irqrestore(&priv->lock, flags);
		} else {
			writel(ACW_RXFIFO_ENABLE,
			       priv->base + ACW_RXFIFO_CTRL);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			fh8626_playback_stop(priv);
		else
			writel(ACW_RXFIFO_DISABLE,
			       priv->base + ACW_RXFIFO_CTRL);
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* Keep the settled analog path powered across a pause */
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			spin_lock_irqsave(&priv->lock, flags);
			writel(ACW_TXFIFO_DISABLE,
			       priv->base + ACW_TXFIFO_CTRL);
			spin_unlock_irqrestore(&priv->lock, flags);
		} else {
			writel(ACW_RXFIFO_DISABLE,
			       priv->base + ACW_RXFIFO_CTRL);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops fh8626_acw_dai_ops = {
	.startup	= fh8626_acw_dai_startup,
	.shutdown	= fh8626_acw_dai_shutdown,
	.hw_params	= fh8626_acw_dai_hw_params,
	.hw_free	= fh8626_acw_dai_hw_free,
	.prepare	= fh8626_acw_dai_prepare,
	.trigger	= fh8626_acw_dai_trigger,
};

static struct snd_soc_dai_driver fh8626_acw_dai = {
	.name = "fh8626-acw-dai",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = FH8626_AUDIO_RATES,
		.formats = FH8626_AUDIO_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = FH8626_AUDIO_RATES,
		.formats = FH8626_AUDIO_FORMATS,
	},
	.ops = &fh8626_acw_dai_ops,
	.symmetric_rates = 1,
};

static struct snd_soc_dai_driver fh8626_codec_dai = {
	.name = "fh8626-codec-dai",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = FH8626_AUDIO_RATES,
		.formats = FH8626_AUDIO_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = FH8626_AUDIO_RATES,
		.formats = FH8626_AUDIO_FORMATS,
	},
};

static int fh8626_capture_vol_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);

	ucontrol->value.integer.value[0] = READ_ONCE(priv->capture_vol);
	return 0;
}

static int fh8626_capture_vol_put(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);
	unsigned long val = ucontrol->value.integer.value[0];

	/* Ordinary gain codes are 0..30. */
	if (val > 30)
		return -EINVAL;
	return fh8626_codec_update_bits(priv, &priv->capture_vol, val,
					ACODEC_ADC_VOL,
					ACODEC_ADC_VOL_GAIN_MASK,
					val << ACODEC_ADC_VOL_GAIN_SHIFT);
}

static int fh8626_mic_boost_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);

	ucontrol->value.integer.value[0] = READ_ONCE(priv->mic_boost);
	return 0;
}

static int fh8626_mic_boost_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);
	unsigned long val = ucontrol->value.integer.value[0];

	if (val > 3)
		return -EINVAL;
	return fh8626_codec_update_bits(priv, &priv->mic_boost, val,
					ACODEC_ADC_VOL,
					ACODEC_ADC_VOL_BOOST_MASK, val);
}

static const char * const fh8626_input_sources[] = {
	"Mic", "Line"
};

static const struct soc_enum fh8626_input_source_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fh8626_input_sources),
			    fh8626_input_sources);

static int fh8626_input_source_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);

	ucontrol->value.enumerated.item[0] = READ_ONCE(priv->input_source);
	return 0;
}

static int fh8626_input_source_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);
	unsigned int val = ucontrol->value.enumerated.item[0];

	if (val >= ARRAY_SIZE(fh8626_input_sources))
		return -EINVAL;

	return fh8626_codec_update(priv, &priv->input_source, val,
				   ACODEC_ADC_PATH,
				   val ? ACODEC_ADC_PATH_LINE :
					 ACODEC_ADC_PATH_MIC);
}

static int fh8626_playback_vol_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);

	ucontrol->value.integer.value[0] = READ_ONCE(priv->playback_vol);
	return 0;
}

static int fh8626_playback_vol_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);
	unsigned long val = ucontrol->value.integer.value[0];

	if (val > 127)
		return -EINVAL;
	/* DAC gain codes are 1..128 and have no enable bit */
	return fh8626_codec_update(priv, &priv->playback_vol, val,
				   ACODEC_DAC_VOL, val + 1);
}

static int fh8626_speaker_gain_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);

	ucontrol->value.integer.value[0] = READ_ONCE(priv->speaker_gain);
	return 0;
}

static int fh8626_speaker_gain_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);
	unsigned long val = ucontrol->value.integer.value[0];
	unsigned long flags;
	int ret = 0;

	if (val > ACODEC_DAC_OUTPUT_MAX_ATTEN)
		return -EINVAL;
	mutex_lock(&priv->control_lock);
	if (priv->speaker_gain == val)
		goto out;
	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret)
		goto out;
	spin_lock_irqsave(&priv->lock, flags);
	WRITE_ONCE(priv->speaker_gain, val);
	/*
	 * While muted or idle the output holds maximum attenuation; the
	 * stored step is applied when playback unmutes.
	 */
	if (priv->playback_prepared && priv->playback_switch)
		writel(fh8626_speaker_hw_atten(priv),
		       priv->base + ACODEC_DAC_OUTPUT_CTRL);
	spin_unlock_irqrestore(&priv->lock, flags);
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
	ret = 1;
out:
	mutex_unlock(&priv->control_lock);
	return ret;
}

static int fh8626_playback_switch_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);

	ucontrol->value.integer.value[0] = READ_ONCE(priv->playback_switch);
	return 0;
}

static int fh8626_playback_switch_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct fh8626_audio *priv = fh8626_component_priv(c);
	unsigned long val = ucontrol->value.integer.value[0];
	unsigned long flags;
	int ret = 0;

	if (val > 1)
		return -EINVAL;
	mutex_lock(&priv->control_lock);
	if (priv->playback_switch == val)
		goto out;
	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret)
		goto out;
	spin_lock_irqsave(&priv->lock, flags);
	WRITE_ONCE(priv->playback_switch, val);
	if (priv->spk_gpio)
		gpiod_set_value(priv->spk_gpio, 0);
	writel(ACODEC_DAC_MUTE_VAL, priv->base + ACODEC_DAC_MUTE);
	writel(ACODEC_DAC_VOL_MIN, priv->base + ACODEC_DAC_VOL);
	writel(ACODEC_DAC_OUTPUT_DEFAULT, priv->base + ACODEC_DAC_OUTPUT_CTRL);
	spin_unlock_irqrestore(&priv->lock, flags);
	if (val)
		fh8626_playback_enable(priv);
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
	ret = 1;
out:
	mutex_unlock(&priv->control_lock);
	return ret;
}

static const struct snd_kcontrol_new fh8626_codec_controls[] = {
	SOC_SINGLE_EXT("Capture Volume", SND_SOC_NOPM, 0, 30, 0,
		       fh8626_capture_vol_get, fh8626_capture_vol_put),
	SOC_SINGLE_EXT("Mic Boost Volume", SND_SOC_NOPM, 0, 3, 0,
		       fh8626_mic_boost_get, fh8626_mic_boost_put),
	SOC_ENUM_EXT("Input Source", fh8626_input_source_enum,
		     fh8626_input_source_get, fh8626_input_source_put),
	SOC_SINGLE_EXT("Playback Volume", SND_SOC_NOPM, 0, 127, 0,
		       fh8626_playback_vol_get, fh8626_playback_vol_put),
	SOC_SINGLE_EXT("Speaker Analog Playback Volume", SND_SOC_NOPM, 0,
		       ACODEC_DAC_OUTPUT_MAX_ATTEN, 0,
		       fh8626_speaker_gain_get, fh8626_speaker_gain_put),
	SOC_SINGLE_EXT("Playback Switch", SND_SOC_NOPM, 0, 1, 0,
		       fh8626_playback_switch_get, fh8626_playback_switch_put),
};

static const struct snd_soc_dapm_widget fh8626_dapm_widgets[] = {
	SND_SOC_DAPM_MICBIAS("MICBIAS", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_SPK("SPK", NULL),
	SND_SOC_DAPM_INPUT("Mic"),
	SND_SOC_DAPM_INPUT("Line"),
};

static const struct snd_soc_dapm_route fh8626_dapm_routes[] = {
	{ "MICBIAS", NULL, "Mic" },
	{ "ADC", NULL, "MICBIAS" },
	{ "ADC", NULL, "Line" },
	{ "Capture", NULL, "ADC" },
	{ "DAC", NULL, "Playback" },
	{ "SPK", NULL, "DAC" },
};

static const struct snd_soc_component_driver fh8626_codec_component = {
	.name			= "fh8626-codec",
	.controls		= fh8626_codec_controls,
	.num_controls		= ARRAY_SIZE(fh8626_codec_controls),
	.dapm_widgets		= fh8626_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(fh8626_dapm_widgets),
	.dapm_routes		= fh8626_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(fh8626_dapm_routes),
	.non_legacy_dai_naming	= 1,
};

static const struct snd_soc_component_driver fh8626_cpu_dai_component = {
	.name			= "fh8626-cpu-dai",
	.non_legacy_dai_naming	= 1,
};

static int
fh8626_dmaengine_prepare_slave_config(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params,
				      struct dma_slave_config *slave_config)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct fh8626_audio *priv =
		fh8626_component_priv(rtd->cpu_dai->component);
	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);
	int ret;

	if (!chan || !dma_has_cap(DMA_CYCLIC, chan->device->cap_mask) ||
	    !chan->device->device_prep_dma_cyclic)
		return -EOPNOTSUPP;

	ret = snd_hwparams_to_dma_slave_config(substream, params, slave_config);
	if (ret)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		slave_config->dst_addr = priv->phys_base + ACW_TXFIFO_DATA;
	else
		slave_config->src_addr = priv->phys_base + ACW_RXFIFO_DATA;
	slave_config->src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config->dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config->src_maxburst = 8;
	slave_config->dst_maxburst = 8;

	return 0;
}

static const struct snd_pcm_hardware fh8626_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE,
	.formats		= FH8626_AUDIO_FORMATS,
	.rates			= FH8626_AUDIO_RATES,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 1,
	.channels_max		= 1,
	.period_bytes_min	= 64,
	/*
	 * One cyclic DMA descriptor per period; stay below the controller's
	 * 4095-transfer block limit for 32-bit FIFO accesses.
	 */
	.period_bytes_max	= 4096,
	.periods_min		= 2,
	.periods_max		= 128,
	.buffer_bytes_max	= 64 * 1024,
};

static const struct snd_dmaengine_pcm_config fh8626_dmaengine_pcm_config = {
	.prepare_slave_config = fh8626_dmaengine_prepare_slave_config,
	.pcm_hardware = &fh8626_pcm_hardware,
	.prealloc_buffer_size = 64 * 1024,
};

static void fh8626_audio_quiesce(void *data)
{
	struct fh8626_audio *priv = data;
	int ret;

	if (priv->spk_gpio)
		gpiod_set_value_cansleep(priv->spk_gpio, 0);
	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret) {
		dev_warn(priv->dev, "failed to enable clocks for shutdown: %d\n",
			 ret);
		return;
	}
	writel(0, priv->base + ACW_CTRL);
	writel(ACW_TXFIFO_DISABLE, priv->base + ACW_TXFIFO_CTRL);
	writel(ACW_RXFIFO_DISABLE, priv->base + ACW_RXFIFO_CTRL);
	writel(ACODEC_DAC_MUTE_VAL, priv->base + ACODEC_DAC_MUTE);
	writel(ACODEC_DAC_VOL_MIN, priv->base + ACODEC_DAC_VOL);
	writel(ACODEC_DAC_OUTPUT_DEFAULT, priv->base + ACODEC_DAC_OUTPUT_CTRL);
	readl(priv->base + ACW_CTRL);
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);
}

static int fh8626_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fh8626_audio *priv;
	struct snd_soc_card *card;
	struct snd_soc_dai_link *link;
	struct resource *res;
#ifdef CONFIG_DEBUG_FS
	struct snd_soc_component *component;
#endif
	static const char * const fh8626_clk_names[] = {
		"ac", "i2s", "mclk", "pclk"
	};
	int ret;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	spin_lock_init(&priv->lock);
	mutex_init(&priv->control_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < ACW_RXFIFO_DATA + sizeof(u32))
		return -EINVAL;
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);
	priv->phys_base = res->start;

	for (i = 0; i < ARRAY_SIZE(fh8626_clk_names); i++)
		priv->clks[i].id = fh8626_clk_names[i];
	priv->num_clks = ARRAY_SIZE(fh8626_clk_names);

	ret = devm_clk_bulk_get(dev, priv->num_clks, priv->clks);
	if (ret) {
		dev_err(dev, "failed to get audio clocks: %d\n", ret);
		return ret;
	}

	priv->spk_gpio = devm_gpiod_get_optional(dev, "spk", GPIOD_OUT_LOW);
	if (IS_ERR(priv->spk_gpio))
		return PTR_ERR(priv->spk_gpio);
	/* The amplifier is switched from the PCM trigger, so it cannot sleep */
	if (priv->spk_gpio && gpiod_cansleep(priv->spk_gpio))
		return -EINVAL;

	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret) {
		dev_err(dev, "failed to enable clocks: %d\n", ret);
		return ret;
	}
	fh8626_hw_init(priv);
	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);

	/*
	 * Registered before the ASoC resources so that teardown first removes
	 * the card and releases PCM DMA, then silences the hardware.
	 */
	ret = devm_add_action_or_reset(dev, fh8626_audio_quiesce, priv);
	if (ret)
		return ret;

	ret = devm_snd_soc_register_component(dev, &fh8626_cpu_dai_component,
					      &fh8626_acw_dai, 1);
	if (ret) {
		dev_err(dev, "failed to register CPU DAI component: %d\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_component(dev, &fh8626_codec_component,
					      &fh8626_codec_dai, 1);
	if (ret) {
		dev_err(dev, "failed to register codec component: %d\n", ret);
		return ret;
	}

#ifdef CONFIG_DEBUG_FS
	/*
	 * ASoC names components after their device, so the CPU DAI and codec
	 * would share a debugfs directory; give the CPU one its own prefix.
	 */
	component = snd_soc_lookup_component(dev,
					     fh8626_cpu_dai_component.name);
	if (!component)
		return -ENODEV;
	component->debugfs_prefix = "cpu";
#endif

	ret = devm_snd_dmaengine_pcm_register(dev,
					      &fh8626_dmaengine_pcm_config, 0);
	if (ret) {
		dev_err(dev, "failed to register DMAengine PCM: %d\n", ret);
		return ret;
	}

	card = &priv->card;
	link = &priv->dai_link;

	link->name = "fh8626-audio";
	link->stream_name = "fh8626-pcm";
	link->cpu_dai_name = "fh8626-acw-dai";
	link->codec_dai_name = "fh8626-codec-dai";
	link->cpu_name = dev_name(dev);
	link->codec_name = dev_name(dev);
	link->platform_name = dev_name(dev);
	link->dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS;

	card->name = "fh8626-audio";
	card->dev = dev;
	card->owner = THIS_MODULE;
	card->dai_link = link;
	card->num_links = 1;

	snd_soc_card_set_drvdata(card, priv);

	ret = devm_snd_soc_register_card(dev, card);
	if (ret) {
		dev_err(dev, "failed to register sound card: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id fh8626_audio_of_match[] = {
	{ .compatible = "fullhan,fh8626-audio" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fh8626_audio_of_match);

static struct platform_driver fh8626_audio_driver = {
	.probe = fh8626_audio_probe,
	.driver = {
		.name = "fh8626-audio",
		.of_match_table = fh8626_audio_of_match,
	},
};
module_platform_driver(fh8626_audio_driver);

MODULE_DESCRIPTION("Fullhan FH8626 ASoC Audio Driver");
MODULE_AUTHOR("Difan Zhang");
MODULE_LICENSE("GPL v2");
