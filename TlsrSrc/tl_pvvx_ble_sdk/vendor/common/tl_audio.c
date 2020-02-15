/********************************************************************************************************
 * @file     tl_audio.c 
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
#include "tl_audio.h"
#include "../../proj_lib/ble/trace.h"
#include "user_config.h"
#include "../../proj/mcu/register.h"
#include "../../proj/drivers/audio.h"


#ifndef		TL_MIC_32K_FIR_16K
#define		TL_MIC_32K_FIR_16K		0
#endif


int md_long =0;
int md_short =0;
int md_im =0;
int md_noise = 0;
int md_gain = 256;

static const signed char idxtbl[] = { -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};
static const unsigned short steptbl[] = {
 7,  8,  9,  10,  11,  12,  13,  14,  16,  17,
 19,  21,  23,  25,  28,  31,  34,  37,  41,  45,
 50,  55,  60,  66,  73,  80,  88,  97,  107, 118,
 130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
 337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767   };

//////////////////////////////////////////////////////////
//	for 8266: input 128-word, output 80-byte
//////////////////////////////////////////////////////////
void pcm_to_adpcm (signed short *ps, int len, signed short *pd)
{
	int i, j;
	unsigned short code=0;
	unsigned short code16=0;
	int predict_idx = 1;
	code = 0;

	for (i=0; i<8; i++) {
		*pd++ = ps[i];   //copy first 8 samples
	}
	int predict = ps[0];
	for (i=1; i<len; i++) {

		s16 di = ps[i];
		int step = steptbl[predict_idx];
		int diff = di - predict;

		if (diff >=0 ) {
			code = 0;
		}
		else {
			diff = -diff;
			code = 8;
		}

		int diffq = step >> 3;

		for (j=4; j>0; j=j>>1) {
			if( diff >= step) {
				diff = diff - step;
				diffq = diffq + step;
				code = code + j;
			}
			step = step >> 1;
		}

		code16 = (code16 >> 4) | (code << 12);
		if ( (i&3) == 3) {
			*pd++ = code16;
		}

		if(code >= 8) {
			predict = predict - diffq;
		}
		else {
			predict = predict + diffq;
		}

		if (predict > 32767) {
			predict = 32767;
		}
		else if (predict < -32767) {
			predict = -32767;
		}

		predict_idx = predict_idx + idxtbl[code];
		if(predict_idx < 0) {
			predict_idx = 0;
		}
		else if(predict_idx > 88) {
			predict_idx = 88;
		}
	}
}

#define				NUM_OF_ORIG_SAMPLE				2

void mic_to_adpcm (signed short *ps, int len, signed short *pd)
{
	int i, j;
	unsigned short code=0;
	unsigned short code16=0;
	int predict_idx = 1;
	code = 0;

	for (i=0; i<NUM_OF_ORIG_SAMPLE; i++) {
		*pd++ = ps[i];   //copy first 5 samples
	}
	int predict = ps[0];
	for (i=1; i<len; i++) {

		s16 di = ps[i];
		int step = steptbl[predict_idx];
		int diff = di - predict;

		if (diff >=0 ) {
			code = 0;
		}
		else {
			diff = -diff;
			code = 8;
		}

		int diffq = step >> 3;

		for (j=4; j>0; j=j>>1) {
			if( diff >= step) {
				diff = diff - step;
				diffq = diffq + step;
				code = code + j;
			}
			step = step >> 1;
		}

		code16 = (code16 >> 4) | (code << 12);
		if ( (i&3) == 3) {
			*pd++ = code16;
		}

		if(code >= 8) {
			predict = predict - diffq;
		}
		else {
			predict = predict + diffq;
		}

		if (predict > 32767) {
			predict = 32767;
		}
		else if (predict < -32767) {
			predict = -32767;
		}

		predict_idx = predict_idx + idxtbl[code];
		if(predict_idx < 0) {
			predict_idx = 0;
		}
		else if(predict_idx > 88) {
			predict_idx = 88;
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////
//	256-samples split into 2
/////////////////////////////////////////////////////////////////////////////////
void mic_to_adpcm_split (signed short *ps, int len, signed short *pds, int start)
{
	int i, j;
	unsigned short code=0;
	unsigned short code16=0;
	static int predict_idx = 1;
	code = 0;
	static signed short *pd;
	static int predict;

	//byte2,byte1: predict;  byte3: predict_idx; byte4:adpcm data len
	if (start)
	{
		pd = pds;
		*pd++ = predict;
		* (((signed char *)pds) + 2)= predict_idx;
		* (((unsigned char *)pds) + 3)= (ADPCM_PACKET_LEN - 4);
		pd++;
	}

	//byte5- byte128: 124 byte(62 sample) adpcm data
	for (i=0; i<len; i++) {

//		s16 di = ps[TL_MIC_32K_FIR_16K ? i * 2 : i];
		s16 di = ps[i];
		int step = steptbl[predict_idx];
		int diff = di - predict;

		if (diff >=0 ) {
			code = 0;
		}
		else {
			diff = -diff;
			code = 8;
		}

		int diffq = step >> 3;

		for (j=4; j>0; j=j>>1) {
			if( diff >= step) {
				diff = diff - step;
				diffq = diffq + step;
				code = code + j;
			}
			step = step >> 1;
		}

		code16 = (code16 >> 4) | (code << 12);
		if ( (i&3) == 3) {
			*pd++ = code16;
		}

		if(code >= 8) {
			predict = predict - diffq;
		}
		else {
			predict = predict + diffq;
		}

		if (predict > 32767) {
			predict = 32767;
		}
		else if (predict < -32767) {
			predict = -32767;
		}

		predict_idx = predict_idx + idxtbl[code];
		if(predict_idx < 0) {
			predict_idx = 0;
		}
		else if(predict_idx > 88) {
			predict_idx = 88;
		}
	}
}

////////////////////////////////////////////////////////////////////
/*  name ADPCM to pcm
    signed short *ps -> pointer to the adpcm source buffer
    signed short *pd -> pointer to the pcm destination buffer
    int len          -> decorded size
*/
void adpcm_to_pcm (signed short *ps, signed short *pd, int len){
	int i;

	//byte2,byte1: predict;  byte3: predict_idx; byte4:adpcm data len
	int predict = ps[0];
	int predict_idx = ps[1] & 0xff;
//	int adpcm_len = (ps[1]>>8) & 0xff;

	unsigned char *pcode = (unsigned char *) (ps + NUM_OF_ORIG_SAMPLE);

	unsigned char code;
	code = *pcode ++;

	//byte5- byte128: 124 byte(62 sample) adpcm data
	for (i=0; i<len; i++) {

		if (1) {
			int step = steptbl[predict_idx];

			int diffq = step >> 3;

			if (code & 4) {
				diffq = diffq + step;
			}
			step = step >> 1;
			if (code & 2) {
				diffq = diffq + step;
			}
			step = step >> 1;
			if (code & 1) {
				diffq = diffq + step;
			}

			if (code & 8) {
				predict = predict - diffq;
			}
			else {
				predict = predict + diffq;
			}

			if (predict > 32767) {
				predict = 32767;
			}
			else if (predict < -32767) {
				predict = -32767;
			}

			predict_idx = predict_idx + idxtbl[code & 15];

			if(predict_idx < 0) {
				predict_idx = 0;
			}
			else if(predict_idx > 88) {
				predict_idx = 88;
			}

			if (i&1) {
				code = *pcode ++;
			}
			else {
				code = code >> 4;
			}
		}

		if (0 && i < NUM_OF_ORIG_SAMPLE) {
			*pd++ = ps[i];
		}
		else {
			*pd++ = predict;
		}
	}
}


#if		TL_MIC_BUFFER_SIZE

#define	BUFFER_PACKET_SIZE		((ADPCM_PACKET_LEN >> 2) * TL_MIC_PACKET_BUFFER_NUM)

int		buffer_mic_enc[BUFFER_PACKET_SIZE];
u8		buffer_mic_pkt_wptr;
u8		buffer_mic_pkt_rptr;

u32		adb_t2;


#define TL_NOISE_SUPRESSION_ENABLE 0 // TODO : too much calculation can have packet drop
#if (IIR_FILTER_ENABLE)

///inner band EQ default parameter. user need to set according to actual situation.
int filter_1[10] = {995*4, 1990*4, 995*4, 849*4, 734*4};
int filter_2[10] = {3691*4, -5564*4, 2915*4, 5564*4, -2510*4};
int filter_3[10] = {2534*4, -1482*4, 955*4,  3956*4, -1866*4};

u8  filter1_shift;
u8  filter2_shift;
u8  filter3_shift;

////used for OOB processing. i.e LPF
int LPF_FILTER_1[10] = {739,87,739,2419,-1401};
int LPF_FILTER_2[10] = {4301,5262,4299,889,-3601};

u8  lpf_filter1_shift;
u8  lpf_filter2_shift;
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////


///voice data out of band need to be processed using 12 bits.
void voice_iir_OOB(signed short * ps, signed short *pd, int* coef, int nsample,u8 shift)
{
	int i = 0;
	long int s = 0;
	for (i=0; i<nsample; i++)
	{
		 //s = (*ps * coef[0])>>shift;
		s = (*ps * coef[0])>>0;          		//input 16-bit
		s += coef[5] * coef[1];
		s += coef[6] * coef[2];       		//coef 0,1,2: 12-bit
		s += coef[7] * coef[3];
		s += coef[8] * coef[4];      		//coef 4 & 5: 10-bit; coef 7 & 8: 18-bit
		//s = s >> 10;                        //18-bit
		//s = s >> 12;                        //18-bit

		s = ((s + (1 << 11)) >> 12); /////this line code indicate that process sample data with 12 bits

#if 0
		if (s >= (1<<18))
			  s = (1<<18) - 1;
		else if (s < -(1<<18))
			  s = - (1<<18);
#endif
		coef[6] = coef[5];                  //16-bit
		coef[5] = *ps++;              		//16-bit
		coef[8] = coef[7];                  //18-bit
		coef[7] = s;
		//*pd++ = s >> 3;
		//*pd++ = s >> 1;

		///EQ filter bug fixed by QW
		if(s > 32767){
			s = 32767;
		}
		else if(s < -32768){
			s = -32768;
		}

		*pd++ = s >> shift;
	}
}

///voice data inner band need to be processed using 14 bits.
void voice_iir(signed short * ps, signed short *pd, int* coef, int nsample,u8 shift)
{
	int i = 0;
	long int s = 0;
	for (i=0; i<nsample; i++)
	{
		 //s = (*ps * coef[0])>>shift;
		s = (*ps * coef[0])>>0;          		//input 16-bit
		s += coef[5] * coef[1];
		s += coef[6] * coef[2];       		//coef 0,1,2: 12-bit
		s += coef[7] * coef[3];
		s += coef[8] * coef[4];      		//coef 4 & 5: 10-bit; coef 7 & 8: 18-bit
		//s = s >> 10;                        //18-bit
		//s = s >> 12;                        //18-bit

		s = ((s + (1 << 13)) >> 14);   /////this line code indicate that process sample data with 14 bits
//		s = ((s + (1 << 11)) >> 12);
#if 0
		if (s >= (1<<18))
			  s = (1<<18) - 1;
		else if (s < -(1<<18))
			  s = - (1<<18);
#endif
		coef[6] = coef[5];                  //16-bit
		coef[5] = *ps++;              		//16-bit
		coef[8] = coef[7];                  //18-bit
		coef[7] = s;
		//*pd++ = s >> 3;
		//*pd++ = s >> 1;

		//EQ filter bug fixed by QW
		if(s > 32767){
			s = 32767;
		}
		else if(s < -32768){
			s = -32768;
		}

		*pd++ = s >> shift;
	}
}

void Audio_VolumeSet(unsigned char input_output_select,unsigned char volume_set_value){

	if(input_output_select)
		write_reg8(0xb06,volume_set_value);
	else
		write_reg8(0x561,volume_set_value);
}


#define  IIR_FILTER_ADR  0x71000
#define  CBUFFER_SIZE    20
u8 filter_step_enable = 0;
u8 vol_gain_tmp       = 0xff;
u8 filter_cmp[20];
/*
 FLASH 0x71000��ʼ���˲�����������ֲ����£�
 +----------------0x71000
  | c1(20B) �����˲�������
 +----------------0x71013
  | c2(20B) �����˲�������
 +----------------0x71027
  | c3(20B) �����˲�������
 +----------------0x7103B
  | c4(20B) �����˲�������
 +----------------0x71xxx
  | c5(20B) �����˲�������
 +----------------0x71xxx
  | f1_sft(1B) | f2_sft(1B) |f3_sft(1B) |f4_sft(1B) | f5_sft(1B) �˲���shiftϵ��
 +----------------0x7109F
  | vol_gain(1B)����������׼: 0x710A0
 +----------------0x710A1
*/
void filter_setting()
{
	/////get the cfg data in the flash
	u32 *pfilter     = (u32*)IIR_FILTER_ADR;
	u8  *p_start_iir = (u8 *)(pfilter);
	u8  vol_gain_tmp = 0;

	memset(filter_cmp, 0xff,sizeof(filter_cmp));

	if(memcmp(p_start_iir,filter_cmp,sizeof(filter_cmp)))//step 1 disaptch
	{
		memcpy((u8 *)filter_1,p_start_iir,CBUFFER_SIZE);
		filter_step_enable |=BIT(1);
	}
	if(memcmp(p_start_iir+CBUFFER_SIZE,filter_cmp,sizeof(filter_cmp)))//step 2 disaptch
	{
		memcpy((u8 *)filter_2,p_start_iir+CBUFFER_SIZE,CBUFFER_SIZE);
		filter_step_enable |=BIT(2);
	}
	if(memcmp(p_start_iir+(CBUFFER_SIZE*2),filter_cmp,sizeof(filter_cmp)))//step 3 dispatch
	{
		memcpy((u8 *)filter_3,p_start_iir+(CBUFFER_SIZE*2),CBUFFER_SIZE);
		filter_step_enable |=BIT(3);
	}

	if(memcmp(p_start_iir+(CBUFFER_SIZE*3),filter_cmp,sizeof(filter_cmp)))//step 4 dispatch
	{
		memcpy((u8 *)LPF_FILTER_1,p_start_iir+(CBUFFER_SIZE*3),CBUFFER_SIZE);
		filter_step_enable |=BIT(4);
	}
	if(memcmp(p_start_iir+(CBUFFER_SIZE*4),filter_cmp,sizeof(filter_cmp)))//step 5 dispatch
	{
		memcpy((u8 *)LPF_FILTER_2,p_start_iir+(CBUFFER_SIZE*4),CBUFFER_SIZE);
		filter_step_enable |=BIT(5);
	}

	int i;
	i = CBUFFER_SIZE*5;

	filter1_shift = (p_start_iir[i]==0xff)		?	0:p_start_iir[i];
    filter2_shift = (p_start_iir[i+1]==0xff)	?	0:p_start_iir[i+1];
	filter3_shift = (p_start_iir[i+2]==0xff)	?  	0:p_start_iir[i+2];

	lpf_filter1_shift = (p_start_iir[i+3]==0xff)?	0:p_start_iir[i+3];
	lpf_filter2_shift = (p_start_iir[i+4]==0xff)?	0:p_start_iir[i+4];

//volumn setting .position 0x710A0
	vol_gain_tmp = p_start_iir[0xA0];
	if(vol_gain_tmp!=0xff){
		if(vol_gain_tmp&0x80){
			if(MANUAL_VOLUMN_SETTINGS<(vol_gain_tmp&0x7f)){
				return;
			}
			Audio_VolumeSet(1,MANUAL_VOLUMN_SETTINGS-(vol_gain_tmp&0x7f));
		}else{
			if(MANUAL_VOLUMN_SETTINGS+vol_gain_tmp>0x3f){
				return;
			}
			Audio_VolumeSet(1,MANUAL_VOLUMN_SETTINGS+vol_gain_tmp);
		}
	}else{
		Audio_VolumeSet(1,MANUAL_VOLUMN_SETTINGS);
	}
	return ;
}

#endif


#if (BLE_AUDIO_ENABLE)
void audio_high_pass_filter(s16* ps, u32 len)
{
	u16 i = 0;
	static s16 ps_last = 0;
	static s16 pd_last = 0;
	static int pd_last_bk = 0;
	int ajst = 	AUDIO_HPF_SHIFT == 1 ? 0x1 :
				AUDIO_HPF_SHIFT == 2 ? 0x3 :
				AUDIO_HPF_SHIFT == 3 ? 0x7 :
				AUDIO_HPF_SHIFT == 4 ? 0xf :
				AUDIO_HPF_SHIFT == 5 ? 0x1f :
				AUDIO_HPF_SHIFT == 6 ? 0x3f :
				AUDIO_HPF_SHIFT == 7 ? 0x7f :
				AUDIO_HPF_SHIFT == 8 ? 0xff :
				AUDIO_HPF_SHIFT == 9 ? 0x1ff :
				AUDIO_HPF_SHIFT == 10 ? 0x3ff :
				AUDIO_HPF_SHIFT == 11 ? 0x7ff :
				AUDIO_HPF_SHIFT == 12 ? 0xfff :
				AUDIO_HPF_SHIFT == 13 ? 0x1fff :
				AUDIO_HPF_SHIFT == 14 ? 0x3fff :
				AUDIO_HPF_SHIFT == 15 ? 0x7fff :
				0x0;

	for(i=0; i< len; i++)
	{
		int y_ajst = (pd_last<0) ? ajst : 0;
		pd_last_bk = (( ((ps[i] - ps_last + pd_last) << AUDIO_HPF_SHIFT) - pd_last ) + y_ajst) >> (AUDIO_HPF_SHIFT); //256
//		pd_last = ps[i] - ps_last +( (( ( pd_last << AUDIO_HPF_SHIFT) - pd_last ) + y_ajst) >> (AUDIO_HPF_SHIFT) ); //256 now

		//high pass filter bug fixed by QW
		if(pd_last_bk > 32767){
			pd_last_bk = 32767;
		}
		else if(pd_last_bk < -32768){
			pd_last_bk = -32768;
		}

		pd_last = (s16)pd_last_bk;
		ps_last = ps[i];
		ps[i] = pd_last;

	}
}
#endif

static inline void audio_getHalfsample_func(s16*ps, u16 len){
	for(int i=0;i<(len>>1);i++){
		ps[i] = ps[2*i];
	}
}

void	proc_mic_encoder (void)
{
	static u16	buffer_mic_rptr;
	u16 mic_wptr = reg_audio_wr_ptr;
	u16 l = ((mic_wptr<<1) >= buffer_mic_rptr) ? ((mic_wptr<<1) - buffer_mic_rptr) : 0xffff;

	if ((l >=(TL_MIC_BUFFER_SIZE>>2))&& \
	   ((( (u8)(buffer_mic_pkt_wptr - buffer_mic_pkt_rptr) )&(TL_MIC_PACKET_BUFFER_NUM*2-1)) < TL_MIC_PACKET_BUFFER_NUM)) {

		s16 *ps = buffer_mic + buffer_mic_rptr;

		#if (IIR_FILTER_ENABLE)
			///out of band voice EQ process
			voice_iir_OOB(ps, ps, LPF_FILTER_1, (TL_MIC_BUFFER_SIZE>>2), lpf_filter1_shift);//12 bits
			voice_iir_OOB(ps, ps, LPF_FILTER_2, (TL_MIC_BUFFER_SIZE>>2), lpf_filter2_shift);//12 bits

			/////     32K��2��1->16K�������ݹ���        ////////
			/////step2: Ϊ���ܹ�ʹ��5��EQ filter��������Ҫ��һ���������������ʹ��32K�������֮�󣨼�����һ��֮�󣩣����ǽ�audio���ݽ���2:1��ȡ��
			audio_getHalfsample_func(ps, TL_MIC_BUFFER_SIZE>>2); //496B/2=> 248B

			/////inner band voice EQ process
			if((filter_step_enable&0x0e)!=0)
			{
				///////step3: 3��EQ filter��������������ݣ���ʱʹ��14bits EQ filter�㷨�� ////////
				if(filter_step_enable&BIT(1))
				{
					voice_iir(ps,ps,filter_1,(TL_MIC_BUFFER_SIZE>>3),filter1_shift);
				}
				if(filter_step_enable&BIT(2))
				{
					voice_iir(ps,ps,filter_2,(TL_MIC_BUFFER_SIZE>>3),filter2_shift);
				}
				if(filter_step_enable&BIT(3))
				{
					voice_iir(ps,ps,filter_3,(TL_MIC_BUFFER_SIZE>>3),filter3_shift);
				}
			}
		#else
			audio_getHalfsample_func(ps, TL_MIC_BUFFER_SIZE>>2); //���ǽ�audio���ݽ���2:1��ȡ
		#endif

		#if (SOFT_HPF_EN)
			audio_high_pass_filter(ps, (TL_MIC_BUFFER_SIZE>>3));
		#endif

		mic_to_adpcm_split (ps,	TL_MIC_ADPCM_UNIT_SIZE,\
						  (s16 *)(buffer_mic_enc + (ADPCM_PACKET_LEN>>2)*(buffer_mic_pkt_wptr & (TL_MIC_PACKET_BUFFER_NUM - 1))), 1);

		buffer_mic_rptr = buffer_mic_rptr ? 0 : (TL_MIC_BUFFER_SIZE>>2);

		buffer_mic_pkt_wptr++;

		log_task_end (TR_T_adpcm);
	}
}


int		mic_encoder_data_ready (int *pd)
{
//	if ((buffer_mic_pkt_rptr & 0x7f) == (buffer_mic_pkt_wptr >> 1)) {
//		return 0;
//	}
//
//	int *ps = buffer_mic_enc + (ADPCM_PACKET_LEN>>2) *
//			(buffer_mic_pkt_rptr & (TL_MIC_PACKET_BUFFER_NUM - 1));
//	for (int i=0; i<(ADPCM_PACKET_LEN>>2); i++) {
//		*pd++ = *ps++;
//	}
//	buffer_mic_pkt_rptr++;
//	return ADPCM_PACKET_LEN;
	return 0;
}

int	*	mic_encoder_data_buffer ()
{
	if (buffer_mic_pkt_rptr == buffer_mic_pkt_wptr) {
			return 0;
	}

	int *ps = buffer_mic_enc + (ADPCM_PACKET_LEN>>2) *
			(buffer_mic_pkt_rptr & (TL_MIC_PACKET_BUFFER_NUM - 1));

//	buffer_mic_pkt_rptr++;   ///this variable can be increased after notify successfully.

	return ps;
}

#if 0
void	proc_mic_encoder (void)
{
	u32 t = clock_time ();
	static u16	buffer_mic_rptr;
	u16 mic_wptr = reg_audio_wr_ptr;
	u16 l = ((mic_wptr<<1) - buffer_mic_rptr) & ((TL_MIC_BUFFER_SIZE>>1) - 1);
	if (l >= 128) {
		log_task_begin (TR_T_adpcm);
		s16 *ps = buffer_mic + buffer_mic_rptr;

#if 	TL_NOISE_SUPRESSION_ENABLE
		for (int i=0; i<128; i++) {
			ps[i] = noise_supression (ps[i] >> 16) << 16;
		}
#endif
		mic_to_adpcm_split (	ps,	128,
						(s16 *)(buffer_mic_enc + (ADPCM_PACKET_LEN>>2) *
						((buffer_mic_pkt_wptr>>1) & (TL_MIC_PACKET_BUFFER_NUM - 1))), !(buffer_mic_pkt_wptr&1));

		buffer_mic_rptr = (buffer_mic_rptr + 128) & ((TL_MIC_BUFFER_SIZE>>1) - 1);
		buffer_mic_pkt_wptr++;
		int pkts = ((buffer_mic_pkt_wptr>>1) - buffer_mic_pkt_rptr) & (TL_MIC_PACKET_BUFFER_NUM*2-1);
		if (pkts > TL_MIC_PACKET_BUFFER_NUM) {
			buffer_mic_pkt_rptr++;
			log_event (TR_T_adpcm_enc_overflow);
		}
		adb_t2 = clock_time() - t;
		log_task_end (TR_T_adpcm);
	}
}

int		mic_encoder_data_ready (int *pd)
{
	if ((buffer_mic_pkt_rptr & 0x7f) == (buffer_mic_pkt_wptr >> 1)) {
		return 0;
	}

	int *ps = buffer_mic_enc + (ADPCM_PACKET_LEN>>2) *
			(buffer_mic_pkt_rptr & (TL_MIC_PACKET_BUFFER_NUM - 1));
	for (int i=0; i<(ADPCM_PACKET_LEN>>2); i++) {
		*pd++ = *ps++;
	}
	buffer_mic_pkt_rptr++;
	return ADPCM_PACKET_LEN;
}

int	*	mic_encoder_data_buffer ()
{
	if ((buffer_mic_pkt_rptr & 0x7f) == (buffer_mic_pkt_wptr >> 1)) {
			return 0;
	}

	int *ps = buffer_mic_enc + (ADPCM_PACKET_LEN>>2) *
			(buffer_mic_pkt_rptr & (TL_MIC_PACKET_BUFFER_NUM - 1));

	buffer_mic_pkt_rptr++;

	return ps;
}
#endif

#endif

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
//	hardware dependent
/////////////////////////////////////////////////////////////
#if TL_SDM_BUFFER_SIZE

int		buffer_sdm_wptr;
int		buffer_sdm_dec[ADPCM_PACKET_LEN];

void adpcm_to_sdm (signed short *ps, int len){
	int i;
	int predict_idx = 1;
	int predict=0;

	unsigned char *pcode = (unsigned char *) (ps + 12);
	unsigned char code=0;

	for (i=0; i<len; i++) {

		if (i) {
			int step = steptbl[predict_idx];

			int diffq = step >> 3;

			if (code & 4) {
				diffq = diffq + step;
			}
			step = step >> 1;
			if (code & 2) {
				diffq = diffq + step;
			}
			step = step >> 1;
			if (code & 1) {
				diffq = diffq + step;
			}

			if (code & 8) {
				predict = predict - diffq;
			}
			else {
				predict = predict + diffq;
			}

			if (predict > 32767) {
				predict = 32767;
			}
			else if (predict < -32767) {
				predict = -32767;
			}

			predict_idx = predict_idx + idxtbl[code & 15];

			if(predict_idx < 0) {
				predict_idx = 0;
			}
			else if(predict_idx > 88) {
				predict_idx = 88;
			}

			if (i&1) {
				code = *pcode ++;
			}
			else {
				code = code >> 4;
			}
		}
		else {
			code = *pcode++ >> 4;
			predict = ps[0];
		}

		int t2;
		if (i < 8) {
			t2 = ps[i];
		}
		else {
			t2 = predict;
		}
		//* ((s16 *) (buffer_sdm + buffer_sdm_wptr)) = t2;
		buffer_sdm[buffer_sdm_wptr] = (t2<<0);
		buffer_sdm_wptr = (buffer_sdm_wptr + 1) & ((TL_SDM_BUFFER_SIZE>>1) - 1);
	}
}

void pcm_to_sdm (signed short *ps, int len){
	for (int i=0; i<len; i++) {
		buffer_sdm[buffer_sdm_wptr] = ps[i];
		buffer_sdm_wptr = (buffer_sdm_wptr + 1) & ((TL_SDM_BUFFER_SIZE>>1) - 1);
	}
}

void silence_to_sdm (void){
	for (int i=0; i<TL_SDM_BUFFER_SIZE>>1; i++) {
		* ((s16 *) (buffer_sdm + i)) = 0;
	}
}

int  sdm_decode_ready (int nbyte_to_decode)
{
	u16 sdm_rptr = reg_aud_rptr; //get_sdm_rd_ptr ();
	u16 num = ((buffer_sdm_wptr>>1) - sdm_rptr) & ((TL_SDM_BUFFER_SIZE>>2) - 1);
	return ((nbyte_to_decode>>2) + num) < (TL_SDM_BUFFER_SIZE >> 2);
}

int   sdm_word_in_buffer ()
{
	u16 num = ((buffer_sdm_wptr>>1) - reg_aud_rptr) & ((TL_SDM_BUFFER_SIZE>>2) - 1);
	return num;
}

void  sdm_decode_rate (int step, int adj)
{
	u16 sdm_rptr = reg_aud_rptr; //get_sdm_rd_ptr ();
	u16 num = ((buffer_sdm_wptr>>1) - sdm_rptr) & ((TL_SDM_BUFFER_SIZE>>2) - 1);

	if (num > (TL_SDM_BUFFER_SIZE*3>>5)) {
		reg_ascl_step = step + adj;
	}
	else if (num < (TL_SDM_BUFFER_SIZE>>4)) {
		reg_ascl_step = step - adj;
	}
}


void proc_sdm_decoder (void)
{

}

int  sdm_decode_data (int *ps, int nbyte)
{
	return 0;
}

#endif

