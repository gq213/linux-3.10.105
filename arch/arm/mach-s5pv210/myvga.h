#ifndef __ARCH_ARM_MACH_S5PV210_MYVGA_H
#define __ARCH_ARM_MACH_S5PV210_MYVGA_H


#define VGA 3

#if (VGA == 1)
	#define BPP		24
	#define H_PIXELS	800
	#define V_PIXELS	600
	#define REFRESH		60
	#define PIXEL_CLK	80000000
	#define PIXEL_DIV	2
	#define SYNC_POLAR	0
	#define H_FRONT		40
	#define H_SYNC		128
	#define H_BACK		88
	#define V_FRONT		1
	#define V_SYNC		4
	#define V_BACK		23
#elif (VGA == 2)
	#define BPP		24
	#define H_PIXELS	1024
	#define V_PIXELS	768
	#define REFRESH		60
	#define PIXEL_CLK	65000000
	#define PIXEL_DIV	1
	#define SYNC_POLAR	(VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC)
	#define H_FRONT		24
	#define H_SYNC		136
	#define H_BACK		160
	#define V_FRONT		3
	#define V_SYNC		6
	#define V_BACK		29
#elif (VGA == 3)
	#define BPP		24
	#define H_PIXELS	1280
	#define V_PIXELS	720
	#define REFRESH		60
	#define PIXEL_CLK	74250000
	#define PIXEL_DIV	1
	#define SYNC_POLAR	0
	#define H_FRONT		110
	#define H_SYNC		40
	#define H_BACK		220
	#define V_FRONT		5
	#define V_SYNC		5
	#define V_BACK		20
#elif (VGA == 4)
	#define BPP		16
	#define H_PIXELS	1280
	#define V_PIXELS	800
	#define REFRESH		60
	#define PIXEL_CLK	71000000
	#define PIXEL_DIV	1
	#define SYNC_POLAR	VIDCON1_INV_VSYNC
	#define H_FRONT		48
	#define H_SYNC		32
	#define H_BACK		80
	#define V_FRONT		3
	#define V_SYNC		6
	#define V_BACK		14
#else
	#error please select VGA type !
#endif


#endif /* __ARCH_ARM_MACH_S5PV210_MYVGA_H */
