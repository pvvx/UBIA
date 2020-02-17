/********************************************************************************************************
 * @file     audio.c
 *
 * @brief    for TLSR chips
 *
 * @author	 BLE Group
 * @date     May. 12, 2018
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *			 The information contained herein is confidential and proprietary property of Telink
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in.
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/


#include "../../proj/tl_common.h"
#include "audio.h"
#include "pga.h"
#include "../mcu_spec/adc_8267.h"
#include "../mcu/register.h"

#if (__TL_LIB_8267__ || (MCU_CORE_TYPE == MCU_CORE_8267) || \
	__TL_LIB_8261__ || (MCU_CORE_TYPE == MCU_CORE_8261) || \
	__TL_LIB_8269__ || (MCU_CORE_TYPE == MCU_CORE_8269))

enum {
	AUD_DMIC,
	AUD_AMIC,
};

#define SET_PFM(v)     do{\
                           BM_CLR(reg_adc_period_chn0,FLD_ADC_CHNM_PERIOD);\
	              	   	   reg_adc_period_chn0 |= MASK_VAL(FLD_ADC_CHNM_PERIOD,v);\
                       }while(0)
#define SET_PFL(v)     do{\
                           BM_CLR(reg_adc_period_chn12,FLD_ADC_CHNLR_PERIOD);\
                           reg_adc_period_chn12 |= MASK_VAL(FLD_ADC_CHNLR_PERIOD,v);\
                       }while(0)
/**
 * @brief     configure the mic buffer's address and size
 * @param[in] pbuff - the first address of SRAM buffer to store MIC data.
 * @param[in] size_buff - the size of pbuff.
 * @return    none
 */
void audio_config_mic_buf(signed short* pbuff,unsigned int size_buff)
{
	reg_dfifo0_addr = (unsigned short)((u32)pbuff);
	reg_dfifo0_size = (size_buff>>4)-1;
}
/**
 * @brief     configure the SDM buffer's address and size
 * @param[in] pbuff - the first address of buffer SDM read data from.
 * @param[in] size_buff - the size of pbuff.
 * @return    none
 */
void audio_config_sdm_buf(signed short* pbuff, unsigned int size_buff)
{
	reg_aud_base_adr = (unsigned short)((u32)pbuff);
	reg_aud_buff_size = (size_buff>>4)-1;
}

/****
* brief: audio amic initial function. configure ADC corresponding parameters. set hpf,lpf and decimation ratio.
* param[in] mode_flag -- '1' differ mode ; '0' signal end mode
* param[in] misc_sys_tick -- system ticks of adc misc channel.
* param[in] l_sys_tick -- system tick of adc left channel CLOCK_SYS_TYPE
* param[in]  fhs_source - the parameter is CLOCK_SYS_TYPE. avoid CLOCK_SYS_TYPE to be modified to other word.such as SYS_TYPE etc.
*
* return none
*/
void audio_amic_init(enum audio_mode_t mode_flag,unsigned short misc_sys_tick, unsigned short left_sys_tick,enum audio_deci_t d_samp,unsigned char fhs_source)
{
	unsigned char tmp_shift;
	unsigned char adc_mode;

	reg_clk_en2 |= FLD_CLK2_DIFIO_EN; //enable dfifo clock

	/***judge which clock is the source of FHS. three selection: 1--PLL 2--RC32M 3--16Mhz crystal oscillator***/
	switch(fhs_source){
	case CLOCK_TYPE_PLL:
		adc_mode = 192; //FHS is 192Mhz
		break;
	case CLOCK_TYPE_OSC:
		adc_mode = 32;  //FHS is RC_32Mhz
		break;
//	case CLOCK_TYPE_PAD:
	default:
		adc_mode = 16;  //FHS is 16Mhz crystal oscillator
		break;
	}

	/***configure adc module. adc clock = FHS*adc_step/adc_mode ***/
	reg_adc_step_l  = 0x04;  //set the step 0x04,adc clock 4Mhz
	reg_adc_mod_l   = adc_mode;
	reg_adc_clk_en |= FLD_ADC_MOD_H_CLK; //enable adc clock

	/***set resolution,reference voltage,sample cycle***/
	BM_CLR(reg_adc_ref,FLD_ADC_REF_L);
	reg_adc_ref      |= MASK_VAL(FLD_ADC_REF_L,RV_1P428);       //1.set reference voltage
	BM_CLR(reg_adc_res_lr,FLD_ADC_RESOLUTION_SEL);
	reg_adc_res_lr   |= MASK_VAL(FLD_ADC_RESOLUTION_SEL,RES14); //2.set resolution
	BM_CLR(reg_adc_tsamp_lr,FLD_ADC_SAMPLE_TIME);
	reg_adc_tsamp_lr |= MASK_VAL(FLD_ADC_SAMPLE_TIME,S_3);      //3.set sample cycle
	
	if(mode_flag == DIFF_MODE){        //different mode 
		BM_CLR(reg_adc_chn_l_sel,FLD_ADC_CHN_SEL|FLD_ADC_DIFF_CHN_SEL|FLD_ADC_DATA_FORMAT);
		reg_adc_chn_l_sel |= MASK_VAL(FLD_ADC_CHN_SEL,AUD_PGAVOM,FLD_ADC_DIFF_CHN_SEL,AUD_PGAVOPM,FLD_ADC_DATA_FORMAT,1);
	}
	else{ //adc is single end mode
		BM_CLR(reg_adc_chn_l_sel,FLD_ADC_DIFF_CHN_SEL);
		reg_adc_chn_l_sel |= MASK_VAL(FLD_ADC_DIFF_CHN_SEL,AUD_SINGLEEND);
	}
	reg_adc_chn_l_sel |= FLD_ADC_DATA_FORMAT;                  //4.signed adc data
	BM_CLR(reg_dfifo_ana_in,FLD_DFIFO_MIC_ADC_IN|FLD_DFIFO_AUD_INPUT_MONO);
	reg_dfifo_ana_in |= MASK_VAL(FLD_DFIFO_MIC_ADC_IN,AUD_AMIC,FLD_DFIFO_AUD_INPUT_MONO,0); //select AMIC

	SET_PFM(misc_sys_tick); //set system tick of misc channel
	SET_PFL(left_sys_tick); //set system tick of left channel

	/**mono,left channel, adc done signal:falling,enable audio output**/
	reg_adc_ctrl = MASK_VAL(FLD_ADC_AUD_DATAPATH_EN,1,FLD_ADC_CHNL_AUTO_EN,1,FLD_ADC_AUD_MODE,MONO_AUDIO,\
			                FLD_ADC_DONE_SIGNAL,AUD_ADC_DONE_FALLING);
		
	/***decimation/down sample[3:0]Decimation rate [6:4]decimation shift select(0~5)***/
	switch(d_samp&0x0f){
	case R1:
		tmp_shift = 0x00;  //0x01
		break;
	case R2:
	case R3:
		tmp_shift = 0x00; //0x02
		break;
	case R4:
	case R5:
		tmp_shift = 0x00; //0x03
		break;
	case R6:
		tmp_shift = 0x00; //0x04
		break;
	default:
		tmp_shift = 0x00; //0x05
		break;
	}
	reg_dfifo_scale = MASK_VAL(FLD_DFIFO2_DEC_CIC,d_samp,FLD_DFIFO0_DEC_SCALE,tmp_shift);
	/***************HPF setting[3:0]HPF shift [4]bypass HPF [5]bypass ALC [6]bypass LPF********/
	BM_CLR(reg_aud_hpf_alc,FLD_AUD_IN_HPF_SFT);
	reg_aud_hpf_alc |= MASK_VAL(FLD_AUD_IN_HPF_SFT,AUDIO_HPF_SHIFT);//different pcb may set different value.
	/***************ALC Volume[5:0]manual volume [6]0:manual 1:auto**************************/
	reg_aud_alc_vol = MASK_VAL(FLD_AUD_MANUAL_VOLUME,MANUAL_VOLUMN_SETTINGS,FLD_AUD_VOLUME_CTRL_MODE,AUD_VOLUME_MANUAL);
}

/**
 * @brief     audio DMIC init function, config the speed of DMIC and downsample audio data to required speed.
 *            actually audio data is dmic_speed/d_samp.
 * @param[in] dmic_speed - set the DMIC speed. such as 1 indicate 1M and 2 indicate 2M.
 * @param[in] d_samp - set the decimation. ie div_speed.
 * @param[in]  fhs_source - the parameter is CLOCK_SYS_TYPE. avoid CLOCK_SYS_TYPE to be modified to other word.such as SYS_TYPE etc.
 *
 * @return    none.
 */
void audio_dmic_init(unsigned char dmic_speed, enum audio_deci_t d_samp,unsigned char fhs_source)
{
	unsigned char adc_mode = 0;
	reg_clk_en2 |= FLD_CLK2_DIFIO_EN; //enable dfifo clock.

	/***judge which clock is the source of FHS. three selection: 1--PLL 2--RC32M 3--16Mhz crystal oscillator***/
	switch(fhs_source){
	case CLOCK_TYPE_PLL:
		adc_mode = 192 - 4; //FHS is 192Mhz
		break;
	case CLOCK_TYPE_OSC:
		adc_mode = 32;  //FHS is RC_32Mhz
		break;
	case CLOCK_TYPE_PAD:
		adc_mode = 16;  //FHS is 16Mhz crystal oscillator
		break;
	}
	/***config the pin dmic_sda and dmic_scl***/
	gpio_set_func(GPIO_DMIC_DI,AS_DMIC);  //disable DI gpio function
	gpio_set_func(GPIO_DMIC_CK,AS_DMIC);  //disable CK gpio function
	gpio_set_input_en(GPIO_DMIC_DI,1);    //enable DI input
	gpio_set_input_en(GPIO_DMIC_CK,1);    //enable CK input
	reg_gpio_config_func0 |= FLD_DMIC_DI_PWM0; //enable PA0 as dmic pin

	/***configure DMIC clock ***/
	BM_CLR(reg_dmic_step,FLD_DMIC_STEP);
	reg_dmic_step |= MASK_VAL(FLD_DMIC_STEP,dmic_speed);
	reg_dmic_mod = adc_mode;
	reg_dmic_step |= FLD_DMIC_CLK_EN; // enable dmic clock
	
	BM_CLR(reg_dfifo_ana_in,FLD_DFIFO_MIC_ADC_IN|FLD_DFIFO_AUD_INPUT_MONO);
	reg_dfifo_ana_in |= MASK_VAL(FLD_DFIFO_MIC_ADC_IN,AUD_DMIC,FLD_DFIFO_AUD_INPUT_MONO,0); //enable DMIC

	/***************decimation/down sample[3:0]Decimation rate [6:4]decimation shift select(0~5)***/
	switch(d_samp){
	case R32:
		reg_dfifo_scale = MASK_VAL(FLD_DFIFO2_DEC_CIC,d_samp,FLD_DFIFO0_DEC_SCALE,0x00);
		break;
	case R64:
		reg_dfifo_scale = MASK_VAL(FLD_DFIFO2_DEC_CIC,d_samp,FLD_DFIFO0_DEC_SCALE,0x00);
		break;
	default:
		reg_dfifo_scale = MASK_VAL(FLD_DFIFO2_DEC_CIC,d_samp,FLD_DFIFO0_DEC_SCALE,0x00);
		break;
	}
	/***************HPF setting[3:0]HPF shift [4]bypass HPF [5]bypass ALC [6]bypass LPF*************/
	BM_CLR(reg_aud_hpf_alc,FLD_AUD_IN_HPF_SFT);
	reg_aud_hpf_alc |= MASK_VAL(FLD_AUD_IN_HPF_SFT,AUDIO_HPF_SHIFT);//different pcb may set different value.
	/***************ALC Volume[5:0]manual volume [6]0:manual 1:auto*********************************/
	reg_aud_alc_vol = MANUAL_VOLUMN_SETTINGS;

	/*************enable HPF ,ALC and disable LPF***/
	BM_CLR(reg_aud_hpf_alc,FLD_AUD_IN_HPF_BYPASS|FLD_AUD_IN_ALC_BYPASS|FLD_AUD_IN_LPF_BYPASS);//open hpf,alc,lpf
//	reg_aud_hpf_alc |= FLD_AUD_IN_LPF_BYPASS; //close lpf
}
/************************************************************************************
*
*	@brief	audio input set function, select analog audio input channel, start the filters
*
*	@param	adc_ch:	if audio input as signle end mode, should identify an analog audio signal input channel, enum variable of ADCINPUTCH
*
*	@return	none
*/
void audio_amic_input_set(enum audio_input_t adc_ch){

	if(adc_ch == PGA_CH){ //this selection is diff mode. the input is pga.
		pgaInit();
		preGainAdjust(PRE_DB20); //set pre pga gain to 20db
		postGainAdjust(POST_DB3); //set post pga gain to 0db
	}
	else{                 //this selection is signed end
		reg_adc_chn_l_sel &= (~FLD_ADC_CHN_SEL);
		reg_adc_chn_l_sel |= adc_ch;
	}
	//configure filter and volume control register
	BM_CLR(reg_aud_hpf_alc,FLD_AUD_IN_HPF_BYPASS|FLD_AUD_IN_ALC_BYPASS|FLD_AUD_IN_LPF_BYPASS);//open hpf,alc,lpf
//	reg_aud_hpf_alc |= FLD_AUD_IN_LPF_BYPASS; //close lpf
}
/**
*	@brief		reg0x30[1:0] 2 bits for fine tuning, divider for slow down sample rate
*	@param[in]	fine_tune - unsigned char fine_tune,range from 0 to 3
*	@return	    none
*/
void audio_finetune_sample_rate(unsigned char fine_tune)
{
    //if(fine_tune>3) return;
	BM_CLR(reg_adc_period_chn0,FLD_ADC_PHASE_TICK);
	fine_tune &= 0x03;
    reg_adc_period_chn0 |= MASK_VAL(FLD_ADC_PHASE_TICK,fine_tune);////reg0x30[1:0] 2 bits for fine tuning
}

/**
 *  @brief      tune decimation shift .i.e register 0xb04 in datasheet.
 *  @param[in]  deci_shift - range from 0 to 5.
 *  @return     none
 */
unsigned char audio_tune_deci_shift(unsigned char deci_shift)
{
	if(deci_shift > 5)
	{
		return 0;
	}
	BM_CLR(reg_dfifo_scale,FLD_DFIFO0_DEC_SCALE);
	reg_dfifo_scale |= MASK_VAL(FLD_DFIFO0_DEC_SCALE,deci_shift);
	return 1;
}
/**
 *   @brief       tune the HPF shift .i.e register 0xb05 in datasheet.
 *   @param[in]   hpf_shift - range from 0 to 0x0f
 *   @return      none
 */
 unsigned char audio_tune_hpf_shift(unsigned char hpf_shift)
 {
	if(hpf_shift > 0x0f){
		return 0;
	}
	BM_CLR(reg_aud_hpf_alc,FLD_AUD_IN_HPF_SFT);
	reg_aud_hpf_alc |= MASK_VAL(FLD_AUD_IN_HPF_SFT,hpf_shift);
	return 1;
 }
 /**
 *
 *	@brief	   sdm setting function, enable or disable the sdm output, configure SDM output paramaters
 *
 *	@param[in]	audio_out_en - audio output enable or disable set, '1' enable audio output; '0' disable output
 *	@param[in]	sample_rate - audio sampling rate, such as 16K,32k etc.
 *	@param[in]	sdm_clk -	  SDM clock, default to be 8Mhz
 *	@param[in]  fhs_source - the parameter is CLOCK_SYS_TYPE. avoid CLOCK_SYS_TYPE to be modified to other word.such as SYS_TYPE etc.
 *
 *	@return	none
 */
void audio_sdm_output_set(unsigned char audio_out_en,int sample_rate,unsigned char sdm_clk,unsigned char fhs_source)
{
	unsigned char adc_mode;
	if(audio_out_en){
		/***enable SDM pins(sdm_n,sdm_p)***/
		gpio_set_func(GPIO_SDMP,AS_SDM);  //disable gpio function
		gpio_set_func(GPIO_SDMN,AS_SDM);  //disable gpio function
		gpio_set_input_en(GPIO_SDMP,1);   //in sdk, require to enable input.because the gpio_init() function in main() function.
		gpio_set_input_en(GPIO_SDMN,1);   //in sdk, require to enable input
		reg_gpio_config_func4 &= (~(FLD_DMIC_CK_RX_CLK|FLD_I2S_DI_RX_DAT));//enable dmic function of SDMP and SDMM

		/***judge which clock is the source of FHS. three selection: 1--PLL 2--RC32M 3--16Mhz crystal oscillator***/
		switch(fhs_source){
		case CLOCK_TYPE_PLL:
			adc_mode = 192; //FHS is 192Mhz
			break;
		case CLOCK_TYPE_OSC:
			adc_mode = 32;  //FHS is RC_32Mhz
			break;
//		case CLOCK_TYPE_PAD:
		default:
			adc_mode = 16;  //FHS is 16Mhz crystal oscillator
			break;
		}

		/***configure sdm clock; ***/
		BM_CLR(reg_i2s_step,FLD_I2S_STEP);
		reg_i2s_step |= MASK_VAL(FLD_I2S_STEP,sdm_clk,FLD_I2S_CLK_EN,1);
		reg_i2s_mod   = adc_mode;
		reg_clk_en2  |= FLD_CLK2_AUD_EN;	//enable audio clock

		/***configure interpolation ratio***/
		reg_ascl_step = AUD_SDM_STEP (sample_rate, sdm_clk*1000000);
		/***2.Enable PN generator as dither control, clear bit 2, 3, 6***/
		BM_CLR(reg_aud_ctrl,FLD_AUD_ENABLE|FLD_AUD_SDM_PLAY_EN|FLD_AUD_PN_SHAPPING_BYPASS|FLD_AUD_SHAPING_EN|FLD_AUD_CONST_VAL_INPUT_EN);
		reg_aud_ctrl |= MASK_VAL(FLD_AUD_PN2_GENERATOR_EN,1,FLD_AUD_PN1_GENERATOR_EN,1);
		
		reg_aud_pn1 = 0x08;  //PN generator 1 bits used
		reg_aud_pn2 = 0x08;  //PN generator 1 bits used

		//enable audio and sdm player.
		reg_aud_ctrl |= (FLD_AUD_ENABLE|FLD_AUD_SDM_PLAY_EN);
	}
	else{
		BM_CLR(reg_aud_ctrl,FLD_AUD_SDM_PLAY_EN);  //close sdm player
	}
}

/**
*	@brief	    set audio volume level
*	@param[in]	input_out_sel - select the tune channel, '1' tune ALC volume; '0' tune SDM output volume
*	@param[in]	volume_level - volume level
*	@return	    none
*/
void audio_volume_tune(unsigned char input_out_sel, unsigned char volume_level)
{
	if(input_out_sel){
		BM_CLR(reg_aud_alc_vol,FLD_AUD_MANUAL_VOLUME);
		reg_aud_alc_vol |= MASK_VAL(FLD_AUD_MANUAL_VOLUME,volume_level);//the low six bits is volume level bits
	}
	else{
		reg_aud_vol_ctrl = volume_level;
	}
}

/*************************************************************
*
*	@brief	automatically gradual change volume
*
*	@param[in]	vol_step - volume change step, the high part is decrease step while the low part is increase step
*			    gradual_interval - volume increase interval
*
*	@return	none
*/
void audio_volume_step_adjust(unsigned char vol_step,unsigned short gradual_interval)
{
	reg_aud_vol_step = vol_step;
	BM_CLR(reg_aud_tick_interval,FLD_AUD_ALC_VOL_TICK_L|FLD_AUD_ALC_VOL_TICK_H);
	reg_aud_tick_interval |= MASK_VAL(FLD_AUD_ALC_VOL_TICK_L,(gradual_interval&0xff),FLD_AUD_ALC_VOL_TICK_H,((gradual_interval>>8)&0x3f));
}

#endif
