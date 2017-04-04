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
