/*
 * adc.h
 *
 * Created on: 19.11.2019
 *      Author: pvvx
 */

#ifndef _ADC_H_
#define _ADC_H_

extern u16 dfifo_rd_ptr; // dfifo_rd_ptr = reg_audio_wr_ptr - reinit ?
void init_adc(u16 per0, u8 per1, u8 vol, u8 scale, u8 chnl);
void deinit_adc(void);
unsigned int get_adc_dfifo_len(void);
int get_adc_dfifo(u16 * pbuf, unsigned int mincnt, unsigned int maxcnt);
//int init_adc_dfifo(u32 sps, u8 channel);
int init_adc_dfifo(dev_adc_cfg_t * p);

#endif /* _ADC_H_ */
