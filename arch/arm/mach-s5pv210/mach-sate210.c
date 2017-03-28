#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>

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

static struct platform_device *sate210_devices[] __initdata = {

};

static void __init sate210_machine_init(void)
{
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
