/********************************************************************************************************
 * @file     usb_hw.c
 *
 * @brief    This file provides set of functions to manage the USB interface
 *
 * @author   xiaodong.zong@telink-semi.com; jian.zhang@telink-semi.com
 * @date     Oct. 8, 2016
 *
 * @par      Copyright (c) 2016, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *           
 *           The information contained herein is confidential property of Telink 
 *           Semiconductor (Shanghai) Co., Ltd. and is available under the terms 
 *           of Commercial License Agreement between Telink Semiconductor (Shanghai) 
 *           Co., Ltd. and the licensee or the terms described here-in. This heading 
 *           MUST NOT be removed from this file.
 *
 *           Licensees are granted free, non-transferable use of the information in this 
 *           file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided. 
 *           
 *******************************************************************************************************/
#include "proj/tl_common.h"
#ifdef USE_USB_CDC
#include "usbCDC/drivers.h"

// some API:
// Endpont8 is the alias of endpoint0
void USBHW_ManualInterruptDisable(int m) {
	REG_CTRL_EP_IRQ_MODE |= m;

}

void USBHW_ManualInterruptEnable(int m) {
	REG_CTRL_EP_IRQ_MODE &= (~m);
}

void USBHW_EpWrite(unsigned long ep,unsigned char * data, int len) {
	int i;
	REG_USB_EP_PTR(ep) = 0;

	for (i = 0; i < len; i++)
		REG_USB_EP_DAT(ep) = data[i];

	REG_USB_EP_CTRL(ep) = FLD_EP_DAT_ACK;		// ACK
}

// handy help function
void USBHW_CtrlEpU16Write(unsigned short v){
	USBHW_CtrlEpDataWrite(v & 0xff);
	USBHW_CtrlEpDataWrite(v >> 8);
}

unsigned short USBHW_CtrlEpU16Read(void){
	unsigned short v = USBHW_CtrlEpDataRead();
	return (USBHW_CtrlEpDataRead() << 8) | v;
}

#endif // USE_USB_CDC



