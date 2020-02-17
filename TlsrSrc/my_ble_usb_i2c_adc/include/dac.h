/*
 * dac.h
 *
 *  Created on: 17.02.2020
 *      Author: pvvx
 */

#ifndef DAC_H_
#define DAC_H_

/**
 * @brief     configure the SDM buffer's address and size
 * @param[in] pbuff - the first address of buffer SDM read data from.
 * @param[in] size_buff - the size of pbuff (bytes).
 * @return    none
 */
inline void sdm_set_buf(signed short* pbuff, unsigned int size_buff)
{
	reg_aud_base_adr = (unsigned short)((u32)pbuff);
	reg_aud_buff_size = (size_buff>>4)-1; // min step 16 bytes!
}

inline void sdm_off() {
	BM_CLR(reg_aud_ctrl,FLD_AUD_SDM_PLAY_EN);  //close sdm player
	reg_clk_en2  &= ~FLD_CLK2_AUD_EN;	//disable audio clock
}

unsigned int dac_cmd(dev_dac_cfg_t *p);
void sdm_init(unsigned char sdm_clk_mhz, unsigned short step);
void set_sdm_buf(signed short* pbuff, unsigned int size_buff);

#endif /* DAC_H_ */
