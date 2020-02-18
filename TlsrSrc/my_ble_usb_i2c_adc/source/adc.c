/*
 * adc.c test
 *
 *  Created on: 19.11.2019
 *      Author: pvvx
 */
#include "proj/tl_common.h"
//#include "proj/drivers/adc.h"
#if (MCU_CORE_TYPE == MCU_CORE_8266)
#include "proj/mcu_spec/adc_8266.h"
#else
#include "proj/mcu_spec/adc_8267.h"
#endif
//#include "proj/mcu_spec/anareg_8886.h"

// analog regs PGA
#define rega_aud_ctrl			0x86
enum {
	FLDA_AUD_PWDN_LEFT = 		BIT(0),
	FLDA_AUD_PWDN_RIGHT = 		BIT(1),
	FLDA_AUD_MUTE_RIGHT = 		BIT(2),
	FLDA_AUD_MUTE_LEFT = 		BIT(3),
	FLDA_AUD_PRE_GAIN_RIGHT = 	BIT_RNG(4,5),
	FLDA_AUD_PRE_GAIN_LEFT = 	BIT_RNG(6,7),
};

#define rega_aud_ctrl2			0x87
enum {
	FLDA_AUD_PST_GAIN_RIGHT = 	BIT_RNG(0,3),
	FLDA_AUD_PST_GAIN_LEFT = 	BIT_RNG(4,7),
};

#define ADC_AUTO_MODE_EN	1
#define ADC_PGA_EN			0 	// PC0
/* PC0: 20 mV p-p rega_aud_ctrl = 0x40, rega_aud_ctrl2 = 0x70, reg_adc_chn_m_sel = 0x0101006D */
#define PGA_GAIN_PREAMP		0	//(0/1) 0/20 dB, step 20 dB
#define PGA_GAIN_POSTAMP	0	//(0..9) 0..22.5 dB, step 2.5 dB

#if ADC_AUTO_MODE_EN
#define ADC_DFIFO_EN		1
#define ADC_DFIFO_SIZE		512	// dfifo size: 64,128,256,512...
#else
#define ADC_DFIFO_EN		0
#endif

#if ADC_DFIFO_EN
u16	dfifo_rd_ptr;
s16 dfifo[ADC_DFIFO_SIZE];
#define SOFT_OUT_SCALE		1	// 1, 2, 4
#endif

#if (MCU_CORE_TYPE == MCU_CORE_8266)
// 8266 Hardware parameters :
#define ADC_DFIFO_FCLK	16000000 // CLK FIFO / 2
#define MAX_DFIFO_SCALE	8 // DFIFO decimation max
#define MIN_DFIFO_SCALE	1 // DFIFO decimation min
#define MAX_PERIOD_CH0	4095 // Max ADC auto channel 0 (Misc) period max (adc_period_chn0 * system clocks)
#define MIN_PERIOD_CH0	77 	// channel ADC period (adc_period_chn0 * system clocks)
#define MAX_PERIOD_CH1	255 // ADC auto channel 1 (L)& 2 period max (adc_period_chn12 * 16 system clocks)
#define MIN_PERIOD_CH1	15 // ADC auto channel 1 (L)& 2 period min (adc_period_chn12 * 16 system clocks)
// ---MIN SPS--- DFIFO decimation 8, SOFT_OUT_SCALE (x1,x2,x4):
// x1 : 16000000/(8*1)/(4095+16*255) = 244.648318 sps
// x2 : 16000000/(8*2)/(4095+16*255) = 122.324159 sps
// x4 : 16000000/(8*4)/(4095+16*255) = 61.16208 sps
#define MIN_OUT_SPS (ADC_DFIFO_FCLK/(MAX_DFIFO_SCALE*SOFT_OUT_SCALE)/(MAX_PERIOD_CH0 + MAX_PERIOD_CH1*16))
// ---MAX HW SPS ? --- DFIFO decimation 1, SOFT_OUT_SCALE (x1,x2,x4):
// x1 : 16000000/1/(77+16*8) = 78048.780488 sps
// x2 : 16000000/2/(77+16*8) = 39024.390244 sps
// x4 : 16000000/4/(77+16*8) = 19512.195122 sps
// ---MAX SPS--- DFIFO decimation 1, SOFT_OUT_SCALE (x1,x2,x4):
// x1 : 16000000/1/(77+16*15) = 50473.18612 sps
// x2 : 16000000/2/(77+16*15) = 25236.59306 sps
// x4 : 16000000/4/(77+16*15) = 12618.29653 sps
// ---MAX SPS--- DFIFO decimation 2, SOFT_OUT_SCALE (x1,x2,x4):
// x1 : 16000000/2/(77+16*15) = 25236.59306 sps
// x2 : 16000000/4/(77+16*15) = 12618.29653 sps
// x4 : 16000000/8/(77+16*15) = 6309.148265 sps
// ---MAX SPS--- DFIFO decimation 4, SOFT_OUT_SCALE (x1,x2,x4):
// x1 : 16000000/4/(77+16*15) = 12618.29653 sps
// x2 : 16000000/8/(77+16*15) = 6309.148265 sps
// x2 : 16000000/16/(77+16*15) = 3154.574132 sps
// ---MAX SPS--- DFIFO decimation 8, SOFT_OUT_SCALE (x1,x2,x4):
// x1 : 16000000/8/(77+16*15) = 6309.148265 sps
// x2 : 16000000/16/(77+16*15) = 3154.574132 sps
// x4 : 16000000/32/(77+16*15) = 1577.287066 sps
#define MAX_OUT_SPS_D1 (ADC_DFIFO_FCLK/(1*SOFT_OUT_SCALE)/(MIN_PERIOD_CH0 + MIN_PERIOD_CH1*16))
#define MAX_OUT_SPS_D2 (ADC_DFIFO_FCLK/(2*SOFT_OUT_SCALE)/(MIN_PERIOD_CH0 + MIN_PERIOD_CH1*16))
#define MAX_OUT_SPS_D4 (ADC_DFIFO_FCLK/(4*SOFT_OUT_SCALE)/(MIN_PERIOD_CH0 + MIN_PERIOD_CH1*16))
#define MAX_OUT_SPS_D8 (ADC_DFIFO_FCLK/(8*SOFT_OUT_SCALE)/(MIN_PERIOD_CH0 + MIN_PERIOD_CH1*16))
// ---SWITCH: DFIFO decimation n * SOFT_OUT_SCALE (x1,x2,x4):
// 1  : 16000000/1/(77+16*15) = 50473.18612 sps
// 2  : 16000000/2/(77+16*15) = 25236.59306 sps
// 4  : 16000000/4/(77+16*15) = 12618.29653 sps
// 8  : 16000000/8/(77+16*15) = 6309.148265 sps
// 16 : 16000000/16/(77+16*15) = 3154.574132 sps
// 32 : 16000000/32/(77+16*15) = 1577.287066 sps
/**
 * @brief This function initializes ADC in sps
 * @param  sps = (245..50473)/SOFT_OUT_SCALE, x1 -> 245..50473 sps
 * @return <= 0 - error
 */
int init_adc_dfifo(dev_adc_cfg_t * p) {
u32 sps = p->sps;
u8 chnl = p->chnl;
u32 per, per12;
	if(sps < MIN_OUT_SPS
		|| sps > MAX_OUT_SPS_D1)
		return 0;
	/** adc stop**/
 	reg_adc_ctrl = 0;
	/** dfifo stop**/
 	// 	reg_dfifo_ana_in = 0;
	reg_dfifo_ana_in = 0;
	/**enable adc clock**/
	reg_rst_clk0 |= FLD_CLK_ADC_EN;
	/**set adc clock **/
	adc_SetClkFreq(ADC_CLK_4M);

	WriteAnalogReg(0x88,0x0f);	// select 192M clk output
	WriteAnalogReg(0x05,0x60);	// power on pll
	WriteAnalogReg(0x06,0xfe);	// bit0 - power on sar

	// set sample rate
	if(sps > MAX_OUT_SPS_D2) {
		per = ADC_DFIFO_FCLK/SOFT_OUT_SCALE;
		reg_dfifo_scale = 0x00; // DFIFO decimation 1
		reg_aud_alc_vol = 40;	// DFIFO volume 40
	}
	else if(sps > MAX_OUT_SPS_D4) {
		per = (ADC_DFIFO_FCLK/SOFT_OUT_SCALE)>>1;
		reg_dfifo_scale = 0x11; // DFIFO decimation 2
		reg_aud_alc_vol = 28; // DFIFO volume 28
	}
	else if(sps > MAX_OUT_SPS_D8) {
		per = (ADC_DFIFO_FCLK/SOFT_OUT_SCALE)>>2;
		reg_dfifo_scale = 0x33; // DFIFO decimation 4
		reg_aud_alc_vol = 16; // DFIFO volume 16
	}
	else {
		per = (ADC_DFIFO_FCLK/SOFT_OUT_SCALE)>>3;
		reg_dfifo_scale = 0x77; // DFIFO decimation 8
		reg_aud_alc_vol = 4; // DFIFO volume 4
	}
	per /= sps;
	per12 = (per - MIN_PERIOD_CH0) >> 4;
	if(per12 > 255)
		per12 = 255;
 	reg_adc_period_chn12 = per12;
 	per -= per12<<4;
 	reg_adc_period_chn0 = per;

 	// dfifo mode
 	reg_aud_hpf_alc = 11 | FLD_AUD_IN_HPF_BYPASS; // | FLD_AUD_IN_ALC_BYPASS;

 	// channel PGA? PGA On/Off?
	if((chnl&0x1f) == FLD_ADC_CHN_PGA_R || (chnl&0x1f) == FLD_ADC_CHN_PGA_L) {
		// PGA on
		reg_clk_en2 |= FLD_CLK2_DIFIO_EN | FLD_CLK2_AUD_EN;
		WriteAnalogReg(rega_aud_ctrl, cfg_adc.pga20db); //(PGA_GAIN_PREAMP << 6) | (PGA_GAIN_PREAMP << 5));
		WriteAnalogReg(rega_aud_ctrl2,cfg_adc.pga2db5);// (PGA_GAIN_POSTAMP << 4) | PGA_GAIN_POSTAMP);
		// select the input channel
		// PGA set left channel ANA_C<1> and right channel ANA_C<2>
		reg_adc_pga_sel_l = 0x21;
	} else {
		// PGA off
		reg_clk_en2 |= FLD_CLK2_DIFIO_EN;
		reg_adc_pga_sel_l = 0;
	}
	reg_adc_pga_sel_m = 0;
//	reg_adc_pga_sel_r = 0; // none in 8266?

	/// set the adc's mode && channels
	//	adc_AnaChSet(chnl); // reg_adc_chn_m_sel
	//	adc_AnaModeSet(SINGLEEND); // reg_adc_chn_m_sel
	REG_ADDR32(0x002c) = 0 // chnl GPIO_PC4
		 | (ADC_CHN_GND) 		// reg_adc_chn_m_sel
		 | ((chnl | 0x80) << 8)    // reg_adc_chn_l_sel
		 | (ADC_CHN_GND << 16)   // reg_adc_chn_r_sel
		 | ADC_SAMPLING_RES_14BIT << 24; 			// reg_adc_res_lr
	/// set the reference voltage
	//	adc_RefVoltageSet(ADC_REF_VOL_1V3); // ADC_REF_VOL_1V3, ADC_REF_VOL_AVDD
	reg_adc_ref = MASK_VAL(FLD_ADC_REF_M, ADC_REF_VOL_AVDD) // adc_RefVoltageSet(RV_1P428);
		| MASK_VAL(FLD_ADC_REF_L, ADC_REF_VOL_1V3) // !!! // RV_1P428, RV_AVDD, RV_1P224
		| MASK_VAL(FLD_ADC_REF_R, ADC_REF_VOL_AVDD);
	/// set resolution, sample cycle
	//	adc_ResSet(ADC_SAMPLING_RES_14BIT); // reg_adc_samp_res
	//	adc_SampleTimeSet(ADC_SAMPLING_CYCLE_12);// ADC_SAMPLING_CYCLE_6); reg_adc_samp_res
	reg_adc_samp_res = MASK_VAL(FLD_ADC_CHNM_SAMP_RESOL, ADC_SAMPLING_RES_14BIT)
		| MASK_VAL(FLD_ADC_CHNM_SAMP_CYCLE, ADC_SAMPLING_CYCLE_12)
		| MASK_VAL(FLD_ADC_CHNLR_SAMP_CYCLE, 0);

	// set dfifo0 buf
	reg_dfifo0_addr = (u16) ((u32)&dfifo);
	reg_dfifo0_size = (ADC_DFIFO_SIZE>>3)-1;
//	reg_dfifo0_size = (ADC_DFIFO_SIZE>>3)-1;

	// start dfifo
	// Bit[3]: disable D-MIC channel
	reg_dfifo_ana_in = BIT(3) | FLD_DFIFO_MIC_ADC_IN | FLD_DFIFO_AUD_INPUT_MONO | FLD_DFIFO_WPTR_CLR;
	dfifo_rd_ptr = 0;
//	reg_dfifo_ana_in = BIT(3) | FLD_DFIFO_MIC_ADC_IN | FLD_DFIFO_AUD_INPUT_MONO;
//	dfifo_rd_ptr = 0;

	// start adc
	reg_adc_ctrl = FLD_ADC_CHNL_AUTO_EN | FLD_ADC_AUD_DATAPATH_EN | MASK_VAL(FLD_ADC_AUD_MODE, 1); //= 0x15; 0: no audio; 1: mono; 2: stereo;
//	while(reg_audio_wr_ptr);
	return 1;
}

void init_adc(u16 per0, u8 per1, u8 vol, u8 scale, u8 chnl) {
	/**enable adc clock**/
	reg_rst_clk0 |= FLD_CLK_ADC_EN;
	/**set adc clock **/
	adc_SetClkFreq(ADC_CLK_4M);

	WriteAnalogReg(0x88,0x0f);	// select 192M clk output
	WriteAnalogReg(0x05,0x60);	// power on pll
	WriteAnalogReg(0x06,0xfe);	// power on sar

	if((chnl&0x1f) == FLD_ADC_CHN_PGA_R || (chnl&0x1f) == FLD_ADC_CHN_PGA_L) {
		// ADC_PGA_EN
		reg_clk_en2 |= FLD_CLK2_DIFIO_EN | FLD_CLK2_AUD_EN;
		WriteAnalogReg(rega_aud_ctrl, (PGA_GAIN_PREAMP << 6) | (PGA_GAIN_PREAMP << 5));
		WriteAnalogReg(rega_aud_ctrl2,(PGA_GAIN_POSTAMP << 4) | PGA_GAIN_POSTAMP);
		// select the input channel
		reg_adc_pga_sel_m = 0;
		// PGA set left channel ANA_C<1> and right channel ANA_C<2>
		reg_adc_pga_sel_l = 0x21;
	}
	else {
		reg_clk_en2 |= FLD_CLK2_DIFIO_EN;
		reg_adc_pga_sel_m = 0;
		// PGA off ?
		reg_adc_pga_sel_l = 0;
		//	adc_AnaChSet(chnl); // reg_adc_chn_m_sel // ADC_CHN_C4
		//	adc_AnaModeSet(SINGLEEND); // reg_adc_chn_m_sel
	}
	// set the adc's mode && channels
	REG_ADDR32(0x002c) = 0 // chnl GPIO_PC4
		 | (ADC_CHN_GND) 		// reg_adc_chn_m_sel
		 | ((chnl | 0x80) << 8)    // reg_adc_chn_l_sel
		 | (ADC_CHN_GND << 16)   // reg_adc_chn_r_sel
		 | ADC_SAMPLING_RES_14BIT << 24; 			// reg_adc_res_lr
	/**set the reference voltage**/
	//	adc_RefVoltageSet(ADC_REF_VOL_1V3); // ADC_REF_VOL_1V3, ADC_REF_VOL_AVDD
	reg_adc_ref = MASK_VAL(FLD_ADC_REF_M, ADC_REF_VOL_AVDD) // adc_RefVoltageSet(RV_1P428);
		| MASK_VAL(FLD_ADC_REF_L, ADC_REF_VOL_1V3) // !!! // RV_1P428, RV_AVDD, RV_1P224
		| MASK_VAL(FLD_ADC_REF_R, ADC_REF_VOL_AVDD);
	/**set resolution**/
//	adc_ResSet(ADC_SAMPLING_RES_14BIT); // reg_adc_samp_res
	/**set sample cycle**/
//	adc_SampleTimeSet(ADC_SAMPLING_CYCLE_12);// ADC_SAMPLING_CYCLE_6); reg_adc_samp_res
	reg_adc_samp_res = MASK_VAL(FLD_ADC_CHNM_SAMP_RESOL, ADC_SAMPLING_RES_14BIT)
			| MASK_VAL(FLD_ADC_CHNM_SAMP_CYCLE, ADC_SAMPLING_CYCLE_12)
			| MASK_VAL(FLD_ADC_CHNLR_SAMP_CYCLE, 0);
#if 1
	// 16000000/(264 + 16*46) = 16000 -> /8 = 2000 sps
	// Итого: 2000 sps
	reg_adc_period_chn0 = 264; // CLK CPU / (period_chn0 + period_chn12*16) / dfifo_scale = x Hz?
	reg_adc_period_chn12 = 46; //
#else
	// 16000000/(112 + 16*18) = 40000 -> /4 = 10000 sps
	// Итого: 10000 sps
	reg_adc_period_chn0 = 0x3ff;//144; // CLK CPU / (period_chn0 + period_chn12*16) / dfifo_scale = x Hz?
//	reg_adc_period_chn12 = 18;//16; //
#endif
#if ADC_AUTO_MODE_EN
#if ADC_DFIFO_EN
	reg_dfifo0_addr = (u16) ((u32)&dfifo);
	reg_dfifo0_size = (ADC_DFIFO_SIZE>>3)-1;
	reg_dfifo0_size = 0;


	reg_adc_period_chn0 = per0; reg_adc_period_chn12 = per1; reg_aud_alc_vol = vol; reg_dfifo_scale = scale;
	reg_aud_hpf_alc = 11 | FLD_AUD_IN_HPF_BYPASS; // | FLD_AUD_IN_ALC_BYPASS;

	// Bit[3]: disable D-MIC channel
	reg_dfifo_ana_in = BIT(3) | FLD_DFIFO_MIC_ADC_IN | FLD_DFIFO_AUD_INPUT_MONO | FLD_DFIFO_WPTR_CLR;
	dfifo_rd_ptr = 0;
	reg_dfifo_ana_in = BIT(3) | FLD_DFIFO_MIC_ADC_IN | FLD_DFIFO_AUD_INPUT_MONO;

	reg_adc_ctrl = FLD_ADC_CHNL_AUTO_EN | FLD_ADC_AUD_DATAPATH_EN | MASK_VAL(FLD_ADC_AUD_MODE, 1); //= 0x15; 0: no audio; 1: mono; 2: stereo;
#else
	// enable auto mode
	reg_adc_ctrl = FLD_ADC_CHNM_AUTO_EN | MASK_VAL(FLD_ADC_DONE_SIGNAL, ADC_DONE_SIGNAL_FALLING); // = 0x88
#endif
#else
	reg_adc_period_chn0 = 250; // 250;	// CLK CPU / (period_chn0 + period_chn12*16*2) / dfifo_scale = x Hz?
	// enable manual mode
	reg_adc_ctrl = MASK_VAL(FLD_ADC_AUD_MODE, 2); // = 0x20
#endif
}
void deinit_adc(void) {
	reg_adc_ctrl = 0;
#if ADC_DFIFO_EN
	reg_dfifo_ana_in = 0; // dfifo disable
//	reg_clk_en2 &= ~(FLD_CLK2_DIFIO_EN | FLD_CLK2_AUD_EN ); // в общем sdm_off
	WriteAnalogReg(rega_aud_ctrl, FLDA_AUD_PWDN_LEFT | FLDA_AUD_PWDN_RIGHT); //(PGA_GAIN_PREAMP << 6));
#endif
	REG_ADDR32(0x002c) = 0 // chnl GPIO_PC4
	 | (ADC_CHN_GND) 		// reg_adc_chn_m_sel
	 | (ADC_CHN_GND << 8)    // reg_adc_chn_l_sel
	 | (ADC_CHN_GND << 16)   // reg_adc_chn_r_sel
	 | ADC_SAMPLING_RES_14BIT << 24; 			// reg_adc_res_lr
	ADC_MODULE_CLOSED;
	reg_rst_clk0 &= ~FLD_CLK_ADC_EN;
//	reg_adc_ref = MASK_VAL(FLD_ADC_REF_M, ADC_REF_VOL_AVDD)
//		| MASK_VAL(FLD_ADC_REF_L, ADC_REF_VOL_AVDD)
//		| MASK_VAL(FLD_ADC_REF_R, ADC_REF_VOL_AVDD);
}
#else // #if (MCU_CORE_TYPE == MCU_CORE_8269)
#error Not tested!
void init_adc(void) {
#if 1
	/**set adc clock and enable adc clock**/
	adc_SetClkFreq(ADC_CLK_4M);

	WriteAnalogReg(0x88,0x0f);	// select 192M clk output
	WriteAnalogReg(0x05,0x60);	// power on pll
	WriteAnalogReg(0x06,0xfe);	// power on sar

	reg_clk_en2 |= FLD_CLK2_DIFIO_EN | FLD_CLK2_AUD_EN;
#if 0 // PGA
	//PGA Power on, output enable
	WriteAnalogReg(rega_aud_ctrl, (PGA_GAIN_PREAMP << 6));
	WriteAnalogReg(rega_aud_ctrl2, (PGA_GAIN_POSTAMP << 4));
#endif
	reg_dfifo0_addr = (u16) ((u32)&dfifo);
	reg_dfifo0_size = (ADC_DFIFO_SIZE>>3)-1;
	dfifo_rd_ptr = 0;
	config_adc(0x0c, 0x0c, SYS_16M_AMIC_16K);
	/**set the reference voltage**/
	reg_adc_ref = MASK_VAL(FLD_ADC_REF_M, RV_1P224) // adc_RefVoltageSet(RV_1P428);
		| MASK_VAL(FLD_ADC_REF_L, RV_1P224) // !!! // RV_1P428, RV_AVDD, RV_1P224
		| MASK_VAL(FLD_ADC_REF_R, RV_1P224);

	// if shl = OTVDD
	// afe3V_reg02 [5:4]
	// 01: 1/3 Vddh (i.e. AVDD3)
	// 10: 1/3 ANA_B<7>
	// WriteAnalogReg(0x02, (ReadAnalogReg(0x02) & 0xcf) | 0x20);

	REG_ADDR32(0x002c) = 0x018C8C1C; // chnl GPIO_PB7

	REG_ADDR32(0x003C) = 0xD780073F;
	REG_ADDR32(0x0030) = 0x150F0104; // 16000000/(260+15*16) = 32000 Hz
	REG_ADDR32(0x0B04) = 0x331F3B77; // 32000/8 = 4000 Hz // 0x331F3B66 -> 4571.428571 Hz

//	reg_aud_alc_vol = 0x1c;
//	reg_dfifo_ana_in = BIT(3) | FLD_DFIFO_MIC_ADC_IN | FLD_DFIFO_AUD_INPUT_MONO | FLD_DFIFO_WPTR_CLR;
//	reg_dfifo_ana_in = BIT(3) | FLD_DFIFO_MIC_ADC_IN | FLD_DFIFO_AUD_INPUT_MONO;
//	reg_adc_ctrl = FLD_ADC_CHNL_AUTO_EN | FLD_ADC_AUD_DATAPATH_EN | MASK_VAL(FLD_ADC_AUD_MODE, 1); //= 0x15; 0: no audio; 1: mono; 2: stereo;

#else
	/**set adc clock and enable adc clock**/
	adc_SetClkFreq(ADC_CLK_4M);

	WriteAnalogReg(0x88,0x0f);	// select 192M clk output
	WriteAnalogReg(0x05,0x60);	// power on pll
	WriteAnalogReg(0x06,0xfe);	// power on sar

	config_adc(0, B7, SYS_16M_AMIC_8K);

	/**select the input channel**/
#if ADC_PGA_EN
	//PGA Power on, output enable 		см. pga_enable();
	WriteAnalogReg(rega_aud_ctrl, (PGA_GAIN_PREAMP << 6));
	WriteAnalogReg(rega_aud_ctrl2, (PGA_GAIN_POSTAMP << 4));
	// PGA set left channel ANA_C<1> and right channel ANA_C<0>
	reg_adc_pga_sel_m = 0x22;
	// PGA set left channel ANA_C<1> and right channel ANA_C<2>
	// reg_adc_pga_sel_m = 0x12;
//	reg_adc_pga_sel_l &= 0xFE;
	adc_AnaChSet(ADC_CHN_PGA_R); // ADC_CHN_PGA_L ?
	/**set the adc's mode**/
	adc_AnaModeSet(CHN_PGA_L);
#else

	reg_adc_chn_m_sel = MASK_VAL(FLD_ADC_CHN_SEL, B7) 	// adc_AnaChSet(B7);
			/**set the adc's mode**/
			| MASK_VAL(FLD_ADC_DIFF_CHN_SEL, SINGLEEND); // adc_AnaModeSet(SINGLEEND);
#endif
	/**set the reference voltage**/
	reg_adc_ref = MASK_VAL(FLD_ADC_REF_M, RV_1P428) // adc_RefVoltageSet(RV_1P428); // RV_1P428, RV_AVDD, RV_1P224
		| MASK_VAL(FLD_ADC_REF_L, RV_1P428)
		| MASK_VAL(FLD_ADC_REF_R, RV_1P428);
	/**set resolution**/
	reg_adc_samp_res = MASK_VAL(FLD_ADC_CHNM_SAMP_RESOL, RES14) // adc_ResSet(RES14);
			/**set sample cycle**/
			| MASK_VAL(FLD_ADC_CHNM_SAMP_RESOL, S_24); // adc_SampleTimeSet(S_6);
	reg_adc_tsamp_lr = S_24;
	reg_adc_res_lr = 7;

	// 16000000/(244 + 8*16*2) = 32000
	reg_adc_period_chn0 = 244; // CLK CPU / (period_chn0 + period_chn12*16*2) / dfifo_scale = x Hz?
	reg_adc_period_chn12 = 8; //? *16
#if ADC_AUTO_MODE_EN
#if ADC_DFIFO_EN
	reg_clk_en2 |= FLD_CLK2_DIFIO_EN | FLD_CLK2_AUD_EN;
	reg_dfifo0_addr = (u16) ((u32)&dfifo);
	reg_dfifo0_size = (ADC_DFIFO_SIZE>>3)-1;
	dfifo_rd_ptr = 0;
	reg_dfifo_ana_in = BIT(3) | FLD_DFIFO_MIC_ADC_IN | FLD_DFIFO_AUD_INPUT_MONO | FLD_DFIFO_WPTR_CLR;
	reg_dfifo_ana_in = BIT(3) | FLD_DFIFO_MIC_ADC_IN | FLD_DFIFO_AUD_INPUT_MONO;
	reg_dfifo_scale = 12;				// = 12 down scale by 256 -> 32000/256 = 125 sps
	// volume setting
	reg_aud_hpf_alc = 11 | FLD_AUD_IN_HPF_BYPASS | FLD_AUD_IN_ALC_BYPASS | FLD_AUD_IN_LPF_BYPASS;				//=11?
	reg_aud_alc_vol = 0x3F;				//=24? volume setting ??
	reg_adc_ctrl = FLD_ADC_CHNL_AUTO_EN | FLD_ADC_AUD_DATAPATH_EN | MASK_VAL(FLD_ADC_AUD_MODE, 1); //= 0x15; 0: no audio; 1: mono; 2: stereo;
#else
	// enable auto mode
	reg_adc_ctrl = FLD_ADC_CHNM_AUTO_EN | MASK_VAL(FLD_ADC_DONE_SIGNAL, ADC_DONE_SIGNAL_FALLING); // = 0x88
#endif
#else
	reg_adc_period_chn0 = 250; // 250;	// CLK CPU / (period_chn0 + period_chn12*16*2) / dfifo_scale = x Hz?
	// enable manual mode
	reg_adc_ctrl = MASK_VAL(FLD_ADC_AUD_MODE, 2); // = 0x20
#endif
#endif
}

#if ! ADC_DFIFO_EN
// not ADC_DFIFO_EN
unsigned short get_adc(void) {
	unsigned int i;
	unsigned int asumm = 0;
	for(i=0; i < 32; i++){
#if ADC_AUTO_MODE_EN
		while(!(reg_adc_chn0_input & 1));
		while(reg_adc_chn0_input & 1);
		asumm += reg_adc_dat_byp_outp & 0x3FFF;// read data
#else
		reg_adc_chn1_outp = FLD_ADC_CHN_MANU_START;  // Set a run signal,start to run adc
		sleep_us(5);   // wait for data
		asumm += reg_adc_dat_byp_outp & 0x3FFF;// read data
#endif
	}
	return asumm >> 3;
}
#endif // ! ADC_DFIFO_EN

#endif // #if (MCU_CORE_TYPE == MCU_CORE_8269)

#if ADC_DFIFO_EN
#if SOFT_OUT_SCALE == 1
unsigned int get_adc_dfifo_len(void) {
	return((reg_audio_wr_ptr - dfifo_rd_ptr) & (ADC_DFIFO_SIZE - 1)) >> 1;
}
// В dfifo пишется по 2 семпла! (L и R канал)
// Для выборки семпла надо делать 2 шага, через один семпл.
//_attribute_ram_code_
int get_adc_dfifo(u16 * pbuf, unsigned int mincnt, unsigned int maxcnt) {
	int i = 0;
//	unsigned char r = irq_disable();
	unsigned int cur_size = ((reg_audio_wr_ptr - dfifo_rd_ptr) & (ADC_DFIFO_SIZE - 1)) >> 1;
	if (cur_size > mincnt)	{
		if(cur_size > maxcnt)
			cur_size = maxcnt;
		else
			cur_size = mincnt;
		do {
			dfifo_rd_ptr &= ADC_DFIFO_SIZE - 1;
			u16 * p = (u16 *)&dfifo[dfifo_rd_ptr];
			dfifo_rd_ptr += 2;
#if 0
			pbuf[i++] = *p + 0x8000;
#else
			s32 x = p[0] + p[1];
			pbuf[i++] = (x >> 1) + 0x8000;
#endif
		} while(i < cur_size);
	}
//	irq_restore(r);
	return i;
}
#elif SOFT_OUT_SCALE == 2
int get_adc_dfifo(u16 * pbuf, unsigned int cnt) {
	int i = 0;
	unsigned int cur_size = ((reg_audio_wr_ptr - dfifo_rd_ptr) & (ADC_DFIFO_SIZE - 1))>>2;
	if (cur_size > cnt) {
		do {
			s16 * p = &dfifo[dfifo_rd_ptr];
#if 1
			s32 x = p[0] + p[2];
			pbuf[i++] = (x>>1) + 0x8000;
#else
			s32 x = p[0] + p[1] + p[2] + p[3];
			pbuf[i++] = (x>>2) + 0x8000;
#endif
			dfifo_rd_ptr += 4;
			dfifo_rd_ptr &= ADC_DFIFO_SIZE - 1;
		} while(i < cnt);
	}
	return i;
}
#elif SOFT_OUT_SCALE == 4
int get_adc_dfifo(u16 * pbuf, unsigned int cnt) {
	int i = 0;
	unsigned int cur_size = ((reg_audio_wr_ptr - dfifo_rd_ptr) & (ADC_DFIFO_SIZE - 1))>>3;
	if (cur_size > cnt) {
		do {
#if 1
			s32 x = p[0] + p[2] + p[4] + p[6];
			pbuf[i++] = (x>>2) + 0x8000;
#else
			s32 x = p[0] + p[1] + p[2] + p[3] + p[4] + p[5] + p[6] + p[7];
			pbuf[i++] = (x>>3) + 0x8000;
#endif
			dfifo_rd_ptr += 8;
			dfifo_rd_ptr &= ADC_DFIFO_SIZE - 1;
		} while(i < cnt);
	}
	return i;
}
#endif

#else // not ADC_DFIFO_EN

unsigned short get_adc(void) {
	unsigned int i;
	unsigned int asumm = 0;
	for(i=0; i < 32; i++){
#if ADC_AUTO_MODE_EN
		while(!(reg_adc_chn0_input & 1));
		while(reg_adc_chn0_input & 1);
		asumm += reg_adc_dat_byp_outp & 0x3FFF;// read data
#else
		reg_adc_chn1_outp = FLD_ADC_CHN_MANU_START;  // Set a run signal,start to run adc
		sleep_us(5);   // wait for data
		asumm += reg_adc_dat_byp_outp & 0x3FFF;// read data
#endif
	}
	return asumm >> 3;
}

#endif // ADC_DFIFO_EN
