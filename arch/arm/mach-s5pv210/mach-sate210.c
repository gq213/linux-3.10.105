#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/dm9000.h>
#include <linux/gpio.h>

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

static struct platform_device *sate210_devices[] __initdata = {
#ifdef CONFIG_DM9000
	&sate210_dm9000,
#endif
};

static void __init sate210_machine_init(void)
{
#ifdef CONFIG_DM9000
	sate210_dm9000_init();
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
