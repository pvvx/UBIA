/********************************************************************************************************
 * @file     myprintf.c
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

#include <stdarg.h>
#include "../../proj/tl_common.h"  
#include "myprintf.h"
#if(PRINT_DEBUG_INFO)

#define			DECIMAL_OUTPUT		10
#define			OCTAL_OUTPUT		8
#define			HEX_OUTPUT			16

#undef va_start
#define va_start(ap,v)    (ap = (char *)((int)&v + sizeof(v)))
#undef va_arg
#define va_arg(ap,t)      ((t *)(ap += sizeof(t)))[-1]

#ifndef		BIT_INTERVAL
#define		BIT_INTERVAL	(CLOCK_SYS_CLOCK_HZ/PRINT_BAUD_RATE)
#endif

_attribute_ram_code_ static void uart_put_char(char **str, u8 byte){
	
	if (str) {
		**str = byte;
		++(*str);
	} else{
		u8 j = 0;
		u32 t1 = 0,t2 = 0;

		REG_ADDR8(0x582+((DEBUG_INFO_TX_PIN>>8)<<3)) &= ~(DEBUG_INFO_TX_PIN & 0xff) ;//Enable output


		u32 pcTxReg = (0x583+((DEBUG_INFO_TX_PIN>>8)<<3));//register GPIO output
		u8 tmp_bit0 = read_reg8(pcTxReg) & (~(DEBUG_INFO_TX_PIN & 0xff));
		u8 tmp_bit1 = read_reg8(pcTxReg) | (DEBUG_INFO_TX_PIN & 0xff);


		u8 bit[10] = {0};
		bit[0] = tmp_bit0;
		bit[1] = (byte & 0x01)? tmp_bit1 : tmp_bit0;
		bit[2] = ((byte>>1) & 0x01)? tmp_bit1 : tmp_bit0;
		bit[3] = ((byte>>2) & 0x01)? tmp_bit1 : tmp_bit0;
		bit[4] = ((byte>>3) & 0x01)? tmp_bit1 : tmp_bit0;
		bit[5] = ((byte>>4) & 0x01)? tmp_bit1 : tmp_bit0;
		bit[6] = ((byte>>5) & 0x01)? tmp_bit1 : tmp_bit0;
		bit[7] = ((byte>>6) & 0x01)? tmp_bit1 : tmp_bit0;
		bit[8] = ((byte>>7) & 0x01)? tmp_bit1 : tmp_bit0;
		bit[9] = tmp_bit1;

		//u8 r = irq_disable();
		t1 = read_reg32(0x740);
		for(j = 0;j<10;j++)
		{
			t2 = t1;
			while(t1 - t2 < BIT_INTERVAL){
				t1  = read_reg32(0x740);
			}
			write_reg8(pcTxReg,bit[j]);        //send bit0
		}
		//irq_restore(r);
	}
}

static void _puts(char **out, char *s){
	while((*s != '\0')){
		uart_put_char(out, *s++);
	}
}

static void puti(char **out, unsigned int num, int base){
	char re[]="0123456789ABCDEF";

	char buf[50], cnt = 0;

	char *addr = &buf[49];

	*addr = '\0';

	do{
		cnt++;
		*--addr = re[num%base];
		//when num < 0x10, high bit zero fill: like this: 0x00~0x0F
		if(num < 0x10 && cnt == 1 && HEX_OUTPUT == base){
			*--addr = '0';
			break;
		}
		num/=base;
	}while(num!=0);

	_puts(out, addr);
}

static int print(char **out, const char *format, va_list arg_ptr) {
	char span;
	unsigned long j;
	char *s;
	//char *msg;

	while((span = *(format++))){
		if(span != '%'){
			uart_put_char(out, span);
		}else{
			span = *(format++);
			if(span == 'c'){
				j = va_arg(arg_ptr,int);//get value of char
				uart_put_char(out, j);
			}else if(span == 'd'){
				j = va_arg(arg_ptr,int);//get value of char
				if(j<0){
					uart_put_char(out, '-');
					j = -j;
				}
				puti(out, j,DECIMAL_OUTPUT);
			}else if(span == 's'){
				s = va_arg(arg_ptr,char *);//get string value
				_puts(out, s);
			}else if(span == 'o'){
				j = va_arg(arg_ptr,unsigned int);//get octal value
				puti(out, j,OCTAL_OUTPUT);
			}else if(span == 'x'){
					j = va_arg(arg_ptr,unsigned int);//get hex value
					puti(out, j,HEX_OUTPUT);
			}else if(span == 0){
				break;
			}else{
				uart_put_char(out, span);
			}
		}

	}
	if (out){
		**out = '\0';
	}

	va_end(arg_ptr);
	return 0;
}

int mini_printf(const char *format, ...){
	va_list arg_ptr;
	va_start(arg_ptr, format);
	return print(0, format, arg_ptr);
}

int mini_sprintf(char *out, const char *format, ...) {
	va_list args;
	va_start( args, format );
	return print(&out, format, args);
}

void PrintHex(unsigned char x){
    //u8 HexTable[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	//uart_put_char(0, '0');
	//uart_put_char(0, 'x');
	//uart_put_char(0, HexTable[x>>4]);
	//uart_put_char(0, HexTable[x&0xf]);
	//uart_put_char(0, ' ');
	mini_printf("%x ", x);
}

#endif
