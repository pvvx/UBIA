/********************************************************************************************************
 * @file     usbgamepad.c
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

#include "../tl_common.h"
#include "../drivers/usb.h"
#include "../drivers/usbhw.h"
#include "../drivers/usbhw_i.h"

#if(USB_GAMEPAD_ENABLE)

#define VK_GAMEPAD_RELEASE		0x00

static u32 usbgamepad_not_released;
static u32 usbgamepad_data_report_time;
static u8 const_zeros[16];

#if 0
int usbgamepad_hid_report(s8 hasId, u8 *data, int len){
	if(usb_has_suspend_irq){		//  do not report anything when in suspend. Maybe not unnecessary
		return 1;
	}
	if(usbhw_is_ep_busy(USB_EDP_GAMEPAD_IN))
		return 0;

	reg_usb_ep_ptr(USB_EDP_GAMEPAD_IN) = 0;

	if(hasId) reg_usb_ep_dat(USB_EDP_GAMEPAD_IN) = hasId;
	
	for(int i = 0; i < len; ++i){
	    reg_usb_ep_dat(USB_EDP_GAMEPAD_IN) = data[i];
	}

	usbhw_data_ep_ack(USB_EDP_GAMEPAD_IN);
    return 1;
}
#else
int usbgamepad_report(u8 *data)
{
	if(usb_has_suspend_irq){		//  do not report anything when in suspend. Maybe not unnecessary
		return 1;
	}
	if(usbhw_is_ep_busy(USB_EDP_GAMEPAD_IN))
		return 0;

	reg_usb_ep_ptr(USB_EDP_GAMEPAD_IN) = 0;

	reg_usb_ep_dat(USB_EDP_GAMEPAD_IN) = USB_HID_GAMEPAD;

	for(int i = 0; i < 6; ++i){
	    reg_usb_ep_dat(USB_EDP_GAMEPAD_IN) = data[i];
	}

	usbhw_data_ep_ack(USB_EDP_GAMEPAD_IN);
    return 1;
}
#endif

#if(0)
void usbgamepad_release_check(){
	if(usbgamepad_not_released && clock_time_exceed(usbgamepad_data_report_time, 60*1000*1000)){
	    if(usbgamepad_hid_report(USB_HID_GAMEPAD, const_zeros)){
		    usbgamepad_not_released = 0;
	    }
	}
}

void usbgamepad_report(u8* p){
    int ret = usbgamepad_hid_report(USB_HID_GAMEPAD, p);
	if(0){
		if(memcmp(p, const_zeros, 15)){	//  only buttons  matter		// 0x05 is a release key for Ò¡¸Ë
			usbgamepad_not_released = 1;
			usbgamepad_data_report_time = clock_time();
		}else{
			usbgamepad_not_released = 0;
		}
	}
}
#endif

extern void attXboxMotorFeedback(u8 *data, u8 len);
u8 usbgamppad_recvData_buf[8];
void usbgamppad_recvData(){
	u8 len = reg_usb_ep_ptr(USB_EDP_GAMEPAD_OUT & 0x07);
	usbhw_reset_ep_ptr(USB_EDP_GAMEPAD_OUT);
	usbgamppad_recvData_buf[0] = reg_usb_ep_dat(USB_EDP_GAMEPAD_OUT);
	usbgamppad_recvData_buf[1] = reg_usb_ep_dat(USB_EDP_GAMEPAD_OUT);
	usbgamppad_recvData_buf[2] = reg_usb_ep_dat(USB_EDP_GAMEPAD_OUT);
	usbgamppad_recvData_buf[3] = reg_usb_ep_dat(USB_EDP_GAMEPAD_OUT);
	usbgamppad_recvData_buf[4] = reg_usb_ep_dat(USB_EDP_GAMEPAD_OUT);
	usbgamppad_recvData_buf[5] = reg_usb_ep_dat(USB_EDP_GAMEPAD_OUT);
	usbgamppad_recvData_buf[6] = reg_usb_ep_dat(USB_EDP_GAMEPAD_OUT);
	usbgamppad_recvData_buf[7] = reg_usb_ep_dat(USB_EDP_GAMEPAD_OUT);
	if(0 == usbgamppad_recvData_buf[0] && 8 == usbgamppad_recvData_buf[1]){
//		attXboxMotorFeedback(usbgamppad_recvData_buf, 8);
	}else if(1 == usbgamppad_recvData_buf[0]){
		reg_usb_ep_ptr(USB_EDP_GAMEPAD_IN) = 0;
		reg_usb_ep_dat(USB_EDP_GAMEPAD_IN) = 0x03;
		reg_usb_ep_dat(USB_EDP_GAMEPAD_IN) = 0x00;
		reg_usb_ep_dat(USB_EDP_GAMEPAD_IN) = 0x00;
		reg_usb_ep_dat(USB_EDP_GAMEPAD_IN) = 0x00;
		usbhw_data_ep_ack(USB_EDP_GAMEPAD_IN);
	}
	
}


#endif

