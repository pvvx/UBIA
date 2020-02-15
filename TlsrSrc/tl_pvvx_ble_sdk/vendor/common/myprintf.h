/********************************************************************************************************
 * @file     myprintf.h
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

#ifndef MYPRINTF_H
#define MYPRINTF_H

#if (PRINT_DEBUG_INFO)

int mini_printf(const char *format, ...);
int mini_sprintf(char *out, const char *format, ...);
void PrintHex(unsigned char x);

#undef printf
#define printf			mini_printf
#define	printfArray		arrayPrint
#undef sprintf
#define sprintf         mini_sprintf

#define	arrayPrint(arrayAddr,len)					\
{													\
	unsigned char	i = 0;							\
	do{												\
		mini_printf("%x%s", ((unsigned char *)arrayAddr)[i++], i<len? "-":" "); \
	}while(i<len); \
	mini_printf("\n");	\
}
#else
#undef printf
#define printf
#define	printfArray
#define	PrintHex
#undef sprintf
#define sprintf
#endif

//#define debugBuffer (*(volatile unsigned char (*)[40])(0x8095d8))
#endif
