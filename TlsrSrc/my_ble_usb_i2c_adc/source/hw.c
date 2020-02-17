/*
 * hw.c
 *
 *  Created on: 17.02.2020
 *      Author: pvvx
 */
#include "proj/tl_common.h"

/* get_fhs_clock_mhz
 * reg_fhs_sel
 * 0 192
 * 1 48 192/4
 * 2 32 192/6
 * 3 12 | 16
 *
 */
unsigned int get_fhs_clock_mhz(void) {
	unsigned int fhs_clock_mhz;
	switch (reg_fhs_sel & FLD_FHS_SELECT) {
	case FHS_SEL_PAD:
		fhs_clock_mhz = CLK_QUARTZ / 1000000;
		break;
	case FHS_SEL_48M:
		fhs_clock_mhz = FLD_I2S_MOD_48M;
		break;
	case FHS_SEL_OSC:
		fhs_clock_mhz = FLD_I2S_MOD_OSC;
		break;
		//		case FHS_SEL_192M_PLL:
	default:
		fhs_clock_mhz = FLD_I2S_MOD_PLL;
	}
	return fhs_clock_mhz;
}

/*
 * get_sys_clock_khz
 */
unsigned int get_sys_clock_khz(void) {
	unsigned int sys_clock_khz;
	unsigned int r = reg_clk_sel;
	switch(r & 0x60) {
//	case 0: // 32m clock from rc
	default:
		sys_clock_khz = 32000000;
		break;
	case 0x20: // hs divider clk
		r &= 31;
		if(!r) r = 32;
		sys_clock_khz = (get_fhs_clock_mhz() * 1000)/r;
		break;
	case 0x40: // 12/16M clock from pad
		sys_clock_khz = CLK_QUARTZ / 1000;
		break;
	case 0x60: // 32k clk from pad
		sys_clock_khz = 32;
		break;
	}
	return sys_clock_khz; // [0x68] = 0xC0
}
