#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/dm9000.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/leds.h>

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
#include <linux/platform_data/i2c-s3c2410.h>
#include <linux/platform_data/usb-ehci-s5p.h>

#include "common.h"

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SATE210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SATE210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SATE210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg sate210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SATE210_UCON_DEFAULT,
		.ulcon		= SATE210_ULCON_DEFAULT,
		.ufcon		= SATE210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SATE210_UCON_DEFAULT,
		.ulcon		= SATE210_ULCON_DEFAULT,
		.ufcon		= SATE210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SATE210_UCON_DEFAULT,
		.ulcon		= SATE210_ULCON_DEFAULT,
		.ufcon		= SATE210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SATE210_UCON_DEFAULT,
		.ulcon		= SATE210_ULCON_DEFAULT,
		.ufcon		= SATE210_UFCON_DEFAULT,
	},
};

#ifdef CONFIG_DM9000
static struct resource sate210_dm9000_resources[] = {
	[0] = DEFINE_RES_MEM(S5PV210_PA_SROM_BANK1, 1),
	[1] = DEFINE_RES_MEM(S5PV210_PA_SROM_BANK1 + 4, 1),
	[2] = DEFINE_RES_NAMED(IRQ_EINT(0), 1, NULL, IORESOURCE_IRQ \
				| IORESOURCE_IRQ_HIGHLEVEL),
};

static struct dm9000_plat_data sate210_dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
	.dev_addr	= { 0x00, 0x09, 0xc0, 0xff, 0xec, 0x48 },
};

static struct platform_device sate210_dm9000 = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sate210_dm9000_resources),
	.resource	= sate210_dm9000_resources,
	.dev		= {
		.platform_data	= &sate210_dm9000_platdata,
	},
};

static void __init sate210_dm9000_init(void)
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

#if (defined(CONFIG_S3C_DEV_HSMMC2) || defined(CONFIG_S3C_DEV_HSMMC3))
static struct s3c_sdhci_platdata sate210_hsmmc23_data __initdata = {
	.cd_type	= S3C_SDHCI_CD_INTERNAL,
};
#endif

#ifdef CONFIG_S3C_DEV_I2C1
static struct i2c_board_info sate210_i2c_devs1[] __initdata = {
#ifdef CONFIG_SND_SOC_WM8960
	{ I2C_BOARD_INFO("wm8960", 0x1a), },
#endif
};
#endif

#ifdef CONFIG_S5P_DEV_USB_EHCI
static struct s5p_ehci_platdata sate210_ehci_pdata;
static void __init sate210_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &sate210_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}
#endif

#ifdef CONFIG_KEYBOARD_GPIO
static struct gpio_keys_button gpio_buttons[] = {
	{
		.gpio		= S5PV210_GPH0(1),
		.code		= KEY_UP,
		.desc		= "up",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH0(2),
		.code		= KEY_DOWN,
		.desc		= "down",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH0(3),
		.code		= KEY_ENTER,
		.desc		= "enter",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH0(4),
		.code		= KEY_ESC,
		.desc		= "esc",
		.active_low	= 1,
		.debounce_interval = 50,
	},
	{
		.gpio		= S5PV210_GPH0(6),
		.code		= KEY_HOME,
		.desc		= "home",
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

static void __init gpio_button_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gpio_buttons); i++) {
		s3c_gpio_setpull(gpio_buttons[i].gpio, S3C_GPIO_PULL_UP);
	}
}
#endif

#ifdef CONFIG_LEDS_GPIO
static struct gpio_led sate210_leds[] = {
	[0] = {
		.name			= "led1",
		.default_trigger	= "heartbeat",
		.gpio			= S5PV210_GPH0(7),
		.active_low		= 0,
		.default_state	= LEDS_GPIO_DEFSTATE_OFF,
	},
	[1] = {
		.name			= "led2",
		.default_trigger	= "timer",
		.gpio			= S5PV210_GPH1(0),
		.active_low		= 0,
		.default_state	= LEDS_GPIO_DEFSTATE_OFF,
	},
};
static struct gpio_led_platform_data sate210_gpio_led_data = {
	.leds		= sate210_leds,
	.num_leds	= ARRAY_SIZE(sate210_leds),
};
static struct platform_device sate210_device_led= {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &sate210_gpio_led_data,
	},
};
#endif

static struct platform_device *sate210_devices[] __initdata = {
#ifdef CONFIG_DM9000
	&sate210_dm9000,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif
#ifdef CONFIG_S3C_DEV_I2C1
	&s3c_device_i2c1,
#endif
#ifdef CONFIG_SND_S5PV210_I2S
	&s5pv210_device_iis0,
#endif
#ifdef CONFIG_S5P_DEV_USB_EHCI
	&s5p_device_ehci,
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	&s3c_device_gpio_button,
#endif
#ifdef CONFIG_LEDS_GPIO
	&sate210_device_led,
#endif
#ifdef CONFIG_S3C_DEV_RTC
	&s3c_device_rtc,
#endif
};

static void __init sate210_machine_init(void)
{
#ifdef CONFIG_DM9000
	sate210_dm9000_init();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&sate210_hsmmc23_data);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s3c_sdhci3_set_platdata(&sate210_hsmmc23_data);
#endif
#ifdef CONFIG_S3C_DEV_I2C1
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, sate210_i2c_devs1,
			ARRAY_SIZE(sate210_i2c_devs1));
#endif
#ifdef CONFIG_S5P_DEV_USB_EHCI
	sate210_ehci_init();
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	gpio_button_init();
#endif

	platform_add_devices(sate210_devices, ARRAY_SIZE(sate210_devices));
}

static void __init sate210_map_io(void)
{
	s5pv210_init_io(NULL, 0);
	s3c24xx_init_clocks(clk_xusbxti.rate);
	s3c24xx_init_uarts(sate210_uartcfgs, ARRAY_SIZE(sate210_uartcfgs));
	samsung_set_timer_source(SAMSUNG_PWM2, SAMSUNG_PWM4);
}

static void __init sate210_reserve(void)
{
	s5p_mfc_reserve_mem(0x43000000, 8 << 20, 0x51000000, 8 << 20);
}

MACHINE_START(SATE210, "SATE210")
	/* Maintainer: gq213 <gaoqiang1211@gmail.com> */
	.atag_offset	= 0x100,
	.init_irq	= s5pv210_init_irq,
	.map_io		= sate210_map_io,
	.init_machine	= sate210_machine_init,
	.init_time	= samsung_timer_init,
	.restart	= s5pv210_restart,
	.reserve	= &sate210_reserve,
MACHINE_END
