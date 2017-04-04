#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/dm9000.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/w1-gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/leds.h>
#include <linux/fb.h>
#include <linux/pwm_backlight.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <mach/map.h>
#include <mach/regs-clock.h>

#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/samsung-time.h>
#include <plat/clock.h>
#include <plat/mfc.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-srom.h>
#include <plat/sdhci.h>
#include <plat/fb.h>
#include <plat/backlight.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <linux/platform_data/usb-ehci-s5p.h>

#include <video/platform_lcd.h>

#include "common.h"

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define TQ210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define TQ210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define TQ210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg tq210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= TQ210_UCON_DEFAULT,
		.ulcon		= TQ210_ULCON_DEFAULT,
		.ufcon		= TQ210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= TQ210_UCON_DEFAULT,
		.ulcon		= TQ210_ULCON_DEFAULT,
		.ufcon		= TQ210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= TQ210_UCON_DEFAULT,
		.ulcon		= TQ210_ULCON_DEFAULT,
		.ufcon		= TQ210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= TQ210_UCON_DEFAULT,
		.ulcon		= TQ210_ULCON_DEFAULT,
		.ufcon		= TQ210_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_DM9000
static struct resource tq210_dm9000_resources[] = {
	[0] = DEFINE_RES_MEM(S5PV210_PA_SROM_BANK1, 1),
	[1] = DEFINE_RES_MEM(S5PV210_PA_SROM_BANK1 + 4, 1),
	[2] = DEFINE_RES_NAMED(IRQ_EINT(10), 1, NULL, IORESOURCE_IRQ \
				| IORESOURCE_IRQ_HIGHLEVEL),
};

static struct dm9000_plat_data tq210_dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
	.dev_addr	= { 0x00, 0x09, 0xc0, 0xff, 0xec, 0x48 },
};

static struct platform_device tq210_dm9000 = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tq210_dm9000_resources),
	.resource	= tq210_dm9000_resources,
	.dev		= {
		.platform_data	= &tq210_dm9000_platdata,
	},
};

static void __init tq210_dm9000_init(void)
{
	unsigned int tmp;

	gpio_request(S5PV210_MP01(1), "nCS1");
	s3c_gpio_cfgpin(S5PV210_MP01(1), S3C_GPIO_SFN(2));
	gpio_free(S5PV210_MP01(1));

	tmp = (5 << S5P_SROM_BCX__TACC__SHIFT);
	__raw_writel(tmp, S5P_SROM_BC1);

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= (S5P_SROM_BW__CS_MASK << S5P_SROM_BW__NCS1__SHIFT);
	tmp |= (3 << S5P_SROM_BW__NCS1__SHIFT);
	__raw_writel(tmp, S5P_SROM_BW);
}
#endif

#ifdef CONFIG_S3C_DEV_HSMMC
static struct s3c_sdhci_platdata tq210_hsmmc_data __initdata = {
	.cd_type	= S3C_SDHCI_CD_INTERNAL,
};
#endif

static struct i2c_board_info tq210_i2c_devs0[] __initdata = {
#ifdef CONFIG_EEPROM_AT24
	{ I2C_BOARD_INFO("24c02", 0x50), },
#endif
#ifdef CONFIG_SND_SOC_WM8960
	{ I2C_BOARD_INFO("wm8960", 0x1a), },
#endif
};

#ifdef CONFIG_S5P_DEV_USB_EHCI
static struct s5p_ehci_platdata tq210_ehci_pdata;
static void __init tq210_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &tq210_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}
#endif

#ifdef CONFIG_W1_SLAVE_THERM
static struct w1_gpio_platform_data ds18b20_pdata = {
	.pin = S5PV210_GPH1(0),
	.is_open_drain = 0,
	.ext_pullup_enable_pin = -1,
};

static struct platform_device tq210_ds18b20_device = {
        .name     = "w1-gpio",
        .id       = -1,
        .dev      = {
			.platform_data = &ds18b20_pdata,
        },
};
#endif

#ifdef CONFIG_KEYBOARD_GPIO
static struct gpio_keys_button gpio_buttons[] = {
	{
		.gpio		= S5PV210_GPH0(0),
		.code		= KEY_UP,
		.desc		= "up",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH0(1),
		.code		= KEY_DOWN,
		.desc		= "down",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH0(2),
		.code		= KEY_LEFT,
		.desc		= "left",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH0(3),
		.code		= KEY_RIGHT,
		.desc		= "right",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH0(4),
		.code		= KEY_ENTER,
		.desc		= "enter",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH0(5),
		.code		= KEY_ESC,
		.desc		= "esc",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH2(6),
		.code		= KEY_HOME,
		.desc		= "home",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH2(7),
		.code		= KEY_POWER,
		.desc		= "power",
		.active_low	= 1,
		.debounce_interval = 50,
	},
};
static struct gpio_keys_platform_data gpio_button_data = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};
static struct platform_device s3c_device_gpio_button = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &gpio_button_data,
	}
};
#endif

#ifdef CONFIG_LEDS_GPIO
static struct gpio_led tq210_leds[] = {
	[0] = {
		.name			= "led1",
		.default_trigger	= "heartbeat",
		.gpio			= S5PV210_GPC0(3),
		.active_low		= 0,
		.default_state	= LEDS_GPIO_DEFSTATE_OFF,
	},
	[1] = {
		.name			= "led2",
		.default_trigger	= "timer",
		.gpio			= S5PV210_GPC0(4),
		.active_low		= 0,
		.default_state	= LEDS_GPIO_DEFSTATE_OFF,
	},
};
static struct gpio_led_platform_data tq210_gpio_led_data = {
	.leds		= tq210_leds,
	.num_leds	= ARRAY_SIZE(tq210_leds),
};
static struct platform_device tq210_device_led= {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &tq210_gpio_led_data,
	},
};
#endif

#ifdef CONFIG_FB_S3C
static void tq210_tn92_data_set_power(struct plat_lcd_data *pd,
					unsigned int power)
{
#if !defined(CONFIG_BACKLIGHT_PWM)
	if (power) {
		gpio_request_one(S5PV210_GPD0(0), GPIOF_OUT_INIT_HIGH, "GPD0");
		gpio_free(S5PV210_GPD0(0));
	} else {
		gpio_request_one(S5PV210_GPD0(0), GPIOF_OUT_INIT_LOW, "GPD0");
		gpio_free(S5PV210_GPD0(0));
	}
#endif
}

static struct plat_lcd_data tq210_lcd_tn92_data = {
	.set_power	= tq210_tn92_data_set_power,
};

static struct platform_device tq210_lcd_tn92 = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &tq210_lcd_tn92_data,
};

static struct s3c_fb_pd_win tq210_fb_win = {
	.max_bpp	= 32,
	.default_bpp	= 24,
	.xres		= 800,
	.yres		= 480,
};

static struct fb_videomode tq210_lcd_timing = {
	.left_margin	= 26,	//h_bp
	.right_margin	= 210,	//h_fp
	.upper_margin	= 13,	//v_bp
	.lower_margin	= 22,	//v_fp
	.hsync_len	= 20,
	.vsync_len	= 10,
	.xres		= 800,
	.yres		= 480,
};

static struct s3c_fb_platdata tq210_lcd0_pdata __initdata = {
	.win[0]		= &tq210_fb_win,
	.win[1]		= &tq210_fb_win,
	.win[2]		= &tq210_fb_win,
	.win[3]		= &tq210_fb_win,
	.win[4]		= &tq210_fb_win,
	.vtiming	= &tq210_lcd_timing,
	.vidcon0	= (4<<6)|(1<<4),//33.35MHz
	.vidcon1	= (1<<6)|(1<<5),
	.setup_gpio	= s5pv210_fb_gpio_setup_24bpp,
};
#endif

#ifdef CONFIG_BACKLIGHT_PWM
static struct samsung_bl_gpio_info tq210_bl_gpio_info = {
	.no = S5PV210_GPD0(0),
	.func = S3C_GPIO_SFN(2),
};
static struct platform_pwm_backlight_data tq210_bl_data = {
	.pwm_id = 0,
	.pwm_period_ns = 50000,
};
#endif

#ifdef CONFIG_INPUT_PWM_BEEPER
static struct platform_device tq210_pwm_beeper = {
	.name = "pwm-beeper",
	.id = -1,
	.dev = {
		.platform_data = (void *)1,
	},
};
static int __init tq210_pwm_beeper_init(void)
{
	int ret;

	ret = gpio_request(S5PV210_GPD0(1), "beeper");
	if (ret) {
		printk(KERN_ERR "failed to request GPD for PWM-OUT 1\n");
		return ret;
	}

	s3c_gpio_cfgpin(S5PV210_GPD0(1), S3C_GPIO_SFN(2));

	return 0;
}
#endif

static struct platform_device *tq210_devices[] __initdata = {
#ifdef CONFIG_DM9000
	&tq210_dm9000,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
	&s3c_device_i2c0,
#ifdef CONFIG_SND_S5PV210_I2S
	&s5pv210_device_iis0,
#endif
#ifdef CONFIG_S5P_DEV_USB_EHCI
	&s5p_device_ehci,
#endif
#ifdef CONFIG_W1_SLAVE_THERM
	&tq210_ds18b20_device,
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	&s3c_device_gpio_button,
#endif
#ifdef CONFIG_LEDS_GPIO
	&tq210_device_led,
#endif
#ifdef CONFIG_FB_S3C
	&s3c_device_fb,
	&tq210_lcd_tn92,
#endif
#ifdef CONFIG_INPUT_PWM_BEEPER
	&s3c_device_timer[1],
	&tq210_pwm_beeper,
#endif
};

static void __init tq210_machine_init(void)
{
#ifdef CONFIG_DM9000
	tq210_dm9000_init();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&tq210_hsmmc_data);
#endif
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, tq210_i2c_devs0,
			ARRAY_SIZE(tq210_i2c_devs0));
#ifdef CONFIG_S5P_DEV_USB_EHCI
	tq210_ehci_init();
#endif
#ifdef CONFIG_FB_S3C
	s3c_fb_set_platdata(&tq210_lcd0_pdata);
#endif
#ifdef CONFIG_BACKLIGHT_PWM
	samsung_bl_set(&tq210_bl_gpio_info, &tq210_bl_data);
#endif
#ifdef CONFIG_INPUT_PWM_BEEPER
	tq210_pwm_beeper_init();
#endif
	platform_add_devices(tq210_devices, ARRAY_SIZE(tq210_devices));
}

static void __init tq210_map_io(void)
{
	s5pv210_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(tq210_uartcfgs, ARRAY_SIZE(tq210_uartcfgs));
	samsung_set_timer_source(SAMSUNG_PWM2, SAMSUNG_PWM4);
}

static void __init tq210_reserve(void)
{
	s5p_mfc_reserve_mem(0x43000000, 8 << 20, 0x51000000, 8 << 20);
}

MACHINE_START(TQ210, "TQ210")
	/* Maintainer: gq213 <gaoqiang1211@gmail.com> */
	.atag_offset	= 0x100,
	.init_irq	= s5pv210_init_irq,
	.map_io		= tq210_map_io,
	.init_machine	= tq210_machine_init,
	.init_time	= samsung_timer_init,
	.restart	= s5pv210_restart,
	.reserve	= &tq210_reserve,
MACHINE_END
