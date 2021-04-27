/*
 * dac.h
 *
 *  Created on: 17.02.2020
 *      Author: pvvx
 */

#ifndef DAC_H_
#define DAC_H_

extern dev_dac_cfg_t cfg_dac;
extern  const dev_dac_cfg_t def_cfg_dac;

inline void sdm_set_buf(signed short* pbuff, unsigned int size_buff)
{
	reg_aud_base_adr = (unsigned short)((u32)pbuff);
	reg_aud_buff_size = (size_buff>>4)-1; // min step 16 bytes!
}

void sdm_off(void);
unsigned int dac_cmd(dev_dac_cfg_t *p);
void sdm_init(unsigned char sdm_clk_mhz, unsigned short step, unsigned short volume);
void set_sdm_buf(signed short* pbuff, unsigned int size_buff);

#endif /* DAC_H_ */
