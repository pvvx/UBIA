/********************************************************************************************************
 * @file     usbgamepad_i.h
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

#include "../drivers/usb.h"
#include "../drivers/usbhw.h"
#include "../drivers/usbhw_i.h"

#if 0
static const USB_Descriptor_HIDReport_Datatype_t gamepad_report_desc[] = {
	0x05, 0x01,					 // USAGE_PAGE (Generic Desktop)
	//用途，GamePad
	0x09, 0x05,					 // USAGE (Game Pad)
	0xA1, 0x01,				  // Collection (Application)
	0x85, USB_HID_GAMEPAD,	  //	 Report Id (4)

	0x05, 0x01,					 // USAGE_PAGE (Generic Desktop)
	0x09, 0x39,					 // Usage (Hat Switch)
//	0x15, 0x01,
//	0x25, 0x08,
//	0x35, 0x00,
//	0x46, 0x38, 0x10,
//	0x66, 0x0e, 0x00, 			// Unit (None)
//	0x75, 0x04,
//	0x95, 0x01,
//	0x81, 0x42,		//nput (Data,Var,Abs,NWrp,Lin,Pref,Null,Bit)

//	0x75, 0x04,
//	0x95, 0x01,
//	0x81, 0x03,		// Input (Cnst,Var,Abs,NWrp,Lin,Pref,NNul,Bit)

	//12个按键
	0x05, 0x09,					 //   USAGE_PAGE (Button)
	0x19, 0x01,					 //   USAGE_MINIMUM (Button 1)
	0x29, 0x10,					 //   USAGE_MAXIMUM (Button 10)
	0x95, 0x10,					 //   REPORT_COUNT (16)
	0x75, 0x01,					 //   REPORT_SIZE (1)
	0x81, 0x02,					 //   INPUT (Data,Var,Abs)

	//集合
	0xa1, 0x00,					 //   COLLECTION (Physical)
	//X座标跟Y座标
	0x09, 0x30,					 // 	USAGE (X), 11b: left,  00b: zero-point,  01b: right
	0x09, 0x31,					 // 	USAGE (Y), 11b: up,   00b: zero-point,  01b: down
	0x15, 0x00,					 // 	LOGICAL_MINIMUM (0)
	0x26, 0xff,0xff,			 // 	LOGICAL_MAXIMUM (2)
	0x35, 0x00,
	0x46, 0xff,0xff,
	//X/Y各用两位来表示
	0x95, 0x02,					 // 	REPORT_COUNT (2)
	0x75, 0x08,					 // 	REPORT_SIZE (8 bits)
	0x81, 0x02,					 // 	INPUT (Data,Var,Abs)
	0xc0, 						 //   END_COLLECTION


	//集合
	0xa1, 0x00,					 //   COLLECTION (Physical)
	//Z
	0x09, 0x32,					 // 	USAGE (Z), 11b: left,  00b: zero-point,  01b: right
	0x09, 0x35, 				 // 	USAGE (Rz), 11b: left,  00b: zero-point,  01b: right
	0x15, 0x00,					 // 	LOGICAL_MINIMUM (0)
	0x26, 0xff,0xff,			 // 	LOGICAL_MAXIMUM (2)
	0x35, 0x00,
	0x46, 0xff,0xff,
	//X/Y各用两位来表示
	0x95, 0x02,					 // 	REPORT_COUNT (2)
	0x75, 0x08,					 // 	REPORT_SIZE (8 bits)
	0x81, 0x02,					 // 	INPUT (Data,Var,Abs)
	0xc0, 						 //   END_COLLECTION


/*
	0x75, 0x08,
	0x95, 0x02,
	0x81, 0x03,		// Input (Cnst,Var,Abs,NWrp,Lin,Pref,NNul,Bit)
*/
0xc0,							 //   END_COLLECTION

#if 0
/*Usage Page (Consumer Devices)*/ 			0x05 ,0x0C,
/*Usage (Consumer Control)*/ 				0x09 ,0x01,	
/*Collection (Application)*/ 				0xA1 ,0x01,	
	/*Report ID (2)*/ 						0x85 ,0x02,  
	/*Report Size (16)*/ 					0x75 ,0x10,	
	/*Report Count (4)*/ 					0x95 ,0x04,	
	/*Logical Minimum (1)*/ 				0x15 ,0x01,  
	/*Logical Maximum (896)*/ 				0x26 ,0x80,0x03,
	/*Usage Minimum (Consumer Control)*/	0x19 ,0x01,
	/*Usage Maximum*/ 						0x2A ,0x80,0x03,
	/*Input (Data,Ary,Abs)*/ 				0x81 ,0x00,	
/*End Collection*/ 							0xC0  


/*Usage Page (Vendor-Defined 11)*/ 			,0x06,0x0A,0xFF  
/*Usage (Vendor-Defined 1)*/ 				,0x09,0x01	
/*Collection (Application)*/ 				,0xA1,0x01	
/*	Usage (Vendor-Defined 2)*/ 				,0x09,0x02	
/*	Collection (Physical)*/ 				,0xA1,0x00  
		/*Report ID (2)*/					,0x85,0x03
/*		Usage Page (Vendor-Defined 1)*/ 	,0x06,0x00,0xFF	
/*		Usage (Vendor-Defined 3)*/ 			,0x09,0x03	
/*		Usage (Vendor-Defined 4)*/ 			,0x09,0x04	
/*		Logical Minimum (-128)*/ 			,0x15,0x80  
/*		Logical Maximum (127)*/ 			,0x25,0x7F  
/*		Physical Minimum (0)*/ 				,0x35,0x00	
/*		Physical Maximum (255)*/ 			,0x46,0xFF,0x00  
/*		Report Size (8)*/ 					,0x75,0x08  
/*		Report Count (20)*/ 				,0x95,0x14  
/*Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)*/ ,0x81,0x02  
/*		Usage (Vendor-Defined 5)*/ 			,0x09,0x05	
/*		Usage (Vendor-Defined 6)*/ 			,0x09,0x06	
/*		Logical Minimum (-128)*/ 			,0x15,0x80  
/*		Logical Maximum (127)*/ 			,0x25,0x7F  
/*		Physical Minimum (0)*/ 				,0x35,0x00	
/*		Physical Maximum (255)*/ 			,0x46,0xFF,0x00  
/*		Report Size (8)*/ 					,0x75,0x08  
/*		Report Count (20)*/ 				,0x95,0x14  
/*Output (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit)*/ ,0x91,0x02  
/*End Collection*/ 							,0xC0  
/*End Collection*/ 							,0xC0  

#endif

};

#else

static const USB_Descriptor_HIDReport_Datatype_t gamepad_report_desc[] = {
	0x05, 0x01,					 // USAGE_PAGE (Generic Desktop)
	//用途，GamePad
	0x09, 0x05,					 // USAGE (Game Pad)
	0xA1, 0x01,				  // Collection (Application)
	0x85, USB_HID_GAMEPAD,	  //	 Report Id (4)

	//集合
	0xa1, 0x00,					 //   COLLECTION (Physical)
	//Z
	0x09, 0x32,					 // 	USAGE (Z), 11b: left,  00b: zero-point,  01b: right
	0x09, 0x35, 				 // 	USAGE (Rz), 11b: left,  00b: zero-point,  01b: right
    0x15, 0x81,         		//  LOGICAL_MINIMUM (0)
    0x25, 0x7f,        			 //  LOGICAL_MAXIMUM (4)
//	0x35, 0x00,
//	0x46, 0xff,0xff,
	//X/Y各用1Byte来表示
	0x95, 0x02,					 // 	REPORT_COUNT (2)
	0x75, 0x08,					 // 	REPORT_SIZE (16 bits)
	0x81, 0x02,					 // 	INPUT (Data,Var,Abs)
	0xc0, 						 //   END_COLLECTION

	//集合
	0xa1, 0x00,					 //   COLLECTION (Physical)
	//X座标跟Y座标
	0x09, 0x30,					 // 	USAGE (X), 11b: left,  00b: zero-point,  01b: right
	0x09, 0x31,					 // 	USAGE (Y), 11b: up,   00b: zero-point,  01b: down
    0x15, 0x81,         		//  LOGICAL_MINIMUM (0)
    0x25, 0x7f,         	//  LOGICAL_MAXIMUM (4)
//	0x35, 0x00,
//	0x46, 0xff,0xff,
	//X/Y各用1Byte来表示
	0x95, 0x02,					 // 	REPORT_COUNT (2)
	0x75, 0x08,					 // 	REPORT_SIZE (16 bits)
	0x81, 0x02,					 // 	INPUT (Data,Var,Abs)
	0xc0, 						 //   END_COLLECTION

	0x05, 0x01,					 // USAGE_PAGE (Generic Desktop)
	//	0x09, 0x39,					 // Usage (Hat Switch)
	//	0x15, 0x01,
	//	0x25, 0x08,
	//	0x35, 0x00,
	//	0x46, 0x38, 0x10,
	//	0x66, 0x0e, 0x00, 			// Unit (None)
	//	0x75, 0x04,
	//	0x95, 0x01,
	//	0x81, 0x42,		//nput (Data,Var,Abs,NWrp,Lin,Pref,Null,Bit)
	//
	//	0x75, 0x04,
	//	0x95, 0x01,
	//	0x81, 0x03,		// Input (Cnst,Var,Abs,NWrp,Lin,Pref,NNul,Bit)

	//12个按键
	0x05, 0x09,					 //   USAGE_PAGE (Button)
	0x19, 0x01,					 //   USAGE_MINIMUM (Button 1)
	0x29, 0x10,					 //   USAGE_MAXIMUM (Button 16)
	0x15, 0x00,         		//  LOGICAL_MINIMUM (0)
	0x25, 0x01,         		//  LOGICAL_MAXIMUM (1)
	0x95, 0x10,					 //   REPORT_COUNT (16)
	0x75, 0x01,					 //   REPORT_SIZE (1)
	0x81, 0x02,					 //   INPUT (Data,Var,Abs)


/*
	0x75, 0x08,
	0x95, 0x02,
	0x81, 0x03,		// Input (Cnst,Var,Abs,NWrp,Lin,Pref,NNul,Bit)
*/
0xc0,							 //   END_COLLECTION

};

#endif

static inline u8* usbgamepad_get_report_desc(void) {
	return (u8*) (gamepad_report_desc);
}

static inline u16 usbgamepad_get_report_desc_size(void) {
	return sizeof(gamepad_report_desc);
}

/////////////////////////////////   XBOX  //////////////////////////////////////////////////
static const USB_Descriptor_HIDReport_Datatype_t gamepad_report_desc_xbox[] = {
/*Usage Page (Generic Desktop) */						0x05,0x01,	
/*Usage (Game Pad) */									0x09,0x05,	
/*Collection (Application) */							0xA1,0x01,	
/*	  Collection (Physical) */							0xA1,0x00,	
/*		  Usage (X) */									0x09,0x30,	
/*		  Usage (Y) */									0x09,0x31,	
/*		  Logical Minimum (0) */						0x15,0x00,	
/*		  Logical Maximum (-1) */						0x26,0xFF,0xFF,  
/*		  Physical Minimum (0) */						0x35,0x00,	
/*		  Physical Maximum (-1) */						0x46,0xFF,0xFF,  
/*		  Report Count (2) */							0x95,0x02,	
/*		  Report Size (16) */							0x75,0x10,	
/*		  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit) */0x81,0x02,	
/*	  End Collection */ 								0xC0,  
/*	  Collection (Physical) */							0xA1,0x00,	
/*		  Usage (Rx) */ 								0x09,0x33,	
/*		  Usage (Ry) */ 								0x09,0x34,	
/*		  Logical Minimum (0) */						0x15,0x00,	
/*		  Logical Maximum (-1) */						0x26,0xFF,0xFF,  
/*		  Physical Minimum (0) */						0x35,0x00,	
/*		  Physical Maximum (-1) */						0x46,0xFF,0xFF,  
/*		  Report Count (2) */							0x95,0x02,	
/*		  Report Size (16) */							0x75,0x10,	
/*		  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit) */0x81,0x02,	
/*	  End Collection*/									0xC0,  
/*	  Collection (Physical) */							0xA1,0x00,	
/*		  Usage (Z) */									0x09,0x32,	
/*		  Logical Minimum (0) */						0x15,0x00,	
/*		  Logical Maximum (-1) */						0x26,0xFF,0xFF,  
/*		  Physical Minimum (0) */						0x35,0x00,	
/*		  Physical Maximum (-1) */						0x46,0xFF,0xFF,  
/*		  Report Count (1) */							0x95,0x01,	
/*		  Report Size (16) */							0x75,0x10,	
/*		  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit) */0x81,0x02,	
/*	  End Collection*/									0xC0,  
/*	  Usage Page (Button) */							0x05,0x09,	
/*	  Usage Minimum (Button 1) */						0x19,0x01,	
/*	  Usage Maximum (Button,0x10) */					0x29,0x0A,	
/*	  Report Count (10) */								0x95,0x0A,	
/*	  Report Size (1) */								0x75,0x01,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit) */	0x81,0x02,	
/*	  Usage Page (Generic Desktop) */					0x05,0x01,	
/*	  Usage (Hat Switch) */ 							0x09,0x39,	
/*	  Logical Minimum (1) */							0x15,0x01,	
/*	  Logical Maximum (8) */							0x25,0x08,	
/*	  Physical Minimum (0) */							0x35,0x00,	
/*	  Physical Maximum (4155) */						0x46,0x3B,0x10,  
/*	  Unit (None) */									0x66,0x0E,0x00,  
/*	  Report Size (4) */								0x75,0x04,	
/*	  Report Count (1) */								0x95,0x01,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,Null,Bit) */	0x81,0x42,	
/*	  Report Size (2) */								0x75,0x02,	
/*	  Report Count (1) */								0x95,0x01,	
/*	  Input (Cnst,Var,Abs,NWrp,Lin,Pref,NNul,Bit) */	0x81,0x03,	
/*	  Report Size (8) */								0x75,0x08,	
/*	Report Count (2) */ 								0x95,0x02,	
/*	  Input (Cnst,Var,Abs,NWrp,Lin,Pref,NNul,Bit) */	0x81,0x03,	
/*End Collection*/										0xC0,  

};

static inline u8* usbgamepad_get_report_desc_xbox(void) {
	return (u8*) (gamepad_report_desc_xbox);
}

static inline u16 usbgamepad_get_report_desc_xbox_size(void) {
	return sizeof(gamepad_report_desc_xbox);
}
/////////////////////////////////   Dinput  //////////////////////////////////////////////////
static const USB_Descriptor_HIDReport_Datatype_t gamepad_report_desc_dinput[] = {
/*Usage Page (Generic Desktop)*/						0x05,0x01,	
/*Usage (Game Pad)*/									0x09,0x05,	
/*Collection (Application)*/							0xA1,0x01,	
/*	  Report ID (1)*/									0x85,USB_HID_GAMEPAD,	
/*	  Usage Page (Generic Desktop)*/					0x05,0x01,	
/*	  Usage (Pointer)*/ 								0x09,0x01,	
/*	  Collection (Physical)*/							0xA1,0x00,	
/*		  Usage (X)*/									0x09,0x30,	
/*		  Usage (Y)*/									0x09,0x31,	
/*		  Usage (Z)*/									0x09,0x32,	
/*		  Usage (Rz)*/									0x09,0x35,	
/*		  Logical Minimum (0)*/ 						0x15,0x00,	
/*		  Logical Maximum (255)*/						0x26,0xFF,0x00,  
/*		  Report Size (8)*/ 							0x75,0x08,	
/*		  Report Count (4)*/							0x95,0x04,	
/*		  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)*/ 0x81,0x02,	
/*	  End Collection */									0xC0,
/*	  Usage (Hat Switch)*/								0x09,0x39,	
/*	  Logical Minimum (0)*/ 							0x15,0x00,	
/*	  Logical Maximum (7)*/ 							0x25,0x07,	
/*	  Physical Minimum (0)*/							0x35,0x00,	
/*	  Physical Maximum (315)*/							0x46,0x3B,0x01,  
/*	  Unit (Eng Rot: Degree)*/							0x65,0x14,	
/*	  Report Size (4)*/ 								0x75,0x04,	
/*	  Report Count (1)*/								0x95,0x01,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,Null,Bit)*/ 	0x81,0x42,	
/*	  Report Size (4)*/ 								0x75,0x04,	
/*	  Report Count (1)*/								0x95,0x01,	
/*	  Input (Cnst,Ary,Abs)*/							0x81,0x01,	
/*	  Usage Page (Button)*/ 							0x05,0x09,	
/*	  Usage Minimum (Button 1)*/						0x19,0x01,	
/*	  Usage Maximum (Button 15)*/						0x29,0x0F,	
/*	  Logical Minimum (0)*/ 							0x15,0x00,	
/*	  Logical Maximum (1)*/ 							0x25,0x01,	
/*	  Report Size (1)*/ 								0x75,0x01,	
/*	  Report Count (16)*/								0x95,0x10,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)*/ 	0x81,0x02,	
/*	  Usage Page (Simulation Controls)*/				0x05,0x02,	
/*	  Logical Minimum (0)*/ 							0x15,0x00,	
/*	  Logical Maximum (255)*/							0x26,0xFF,0x00,  
/*	  Usage (Accelerator)*/ 							0x09,0xC4,	
/*	  Usage (Brake)*/									0x09,0xC5,	
/*	  Report Count (2)*/								0x95,0x02,	
/*	  Report Size (8)*/ 								0x75,0x08,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)*/ 	0x81,0x02,	
/*End Collection */										0xC0,  
/*Usage Page (Consumer Devices)*/						0x05,0x0C,	
/*Usage (Consumer Control)*/							0x09,0x01,	
/*Collection (Application)*/							0xA1,0x01,	
/*	  Report ID (2)*/									0x85,0x02,	
/*	  Report Size (8)*/ 								0x75,0x08,	
/*	  Report Count (1)*/								0x95,0x01,	
/*	  Input (Cnst,Ary,Abs)*/							0x81,0x01,	
/*	  Report Size (16)*/								0x75,0x10,	
/*	  Report Count (4)*/								0x95,0x04,	
/*	  Logical Minimum (1)*/ 							0x15,0x01,	
/*	  Logical Maximum (896)*/							0x26,0x80,0x03,  
/*	  Usage Minimum (Consumer Control)*/				0x19,0x01,	
/*	  Usage Maximum */									0x2A,0x80,0x03,  
/*	  Input (Data,Ary,Abs)*/							0x81,0x00,	
/*End Collection */										0xC0,
	
};
static inline u8* usbgamepad_get_report_desc_dinput(void) {
	return (u8*) (gamepad_report_desc_dinput);
}

static inline u16 usbgamepad_get_report_desc_dinput_size(void) {
	return sizeof(gamepad_report_desc_dinput);
}

/////////////////////////////////   PS4  //////////////////////////////////////////////////
static const USB_Descriptor_HIDReport_Datatype_t gamepad_report_desc_ps4[] = {
/*Usage Page (Generic Desktop)*/						0x05,0x01,	
/*Usage (Game Pad)*/									0x09,0x05,	
/*Collection (Application)*/							0xA1,0x01,	
/*	  Report ID (1)*/									0x85,USB_HID_GAMEPAD,	
/*	  Usage (X)*/										0x09,0x30,	
/*	  Usage (Y)*/										0x09,0x31,	
/*	  Usage (Z)*/										0x09,0x32,	
/*	  Usage (Rz)*/										0x09,0x35,	
/*	  Logical Minimum (0)*/ 							0x15,0x00,	
/*	  Logical Maximum (255)*/							0x26,0xFF,0x00,  
/*	  Report Size (8)*/ 								0x75,0x08,	
/*	  Report Count (4)*/								0x95,0x04,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)*/ 	0x81,0x02,	
/*	  Usage (Hat Switch)*/								0x09,0x39,	
/*	  Logical Minimum (0)*/ 							0x15,0x00,	
/*	  Logical Maximum (7)*/ 							0x25,0x07,	
/*	  Physical Minimum (0)*/							0x35,0x00,	
/*	  Physical Maximum (315)*/							0x46,0x3B,0x01,  
/*	  Unit (Eng Rot: Degree)*/							0x65,0x14,	
/*	  Report Size (4)*/ 								0x75,0x04,	
/*	  Report Count (1)*/								0x95,0x01,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,Null,Bit)*/ 	0x81,0x42,	
/*	  Unit (None)*/ 									0x65,0x00,	
/*	  Usage Page (Button)*/ 							0x05,0x09,	
/*	  Usage Minimum (Button 1)*/						0x19,0x01,	
/*	  Usage Maximum (Button 14)*/						0x29,0x0E,	
/*	  Logical Minimum (0)*/ 							0x15,0x00,	
/*	  Logical Maximum (1)*/ 							0x25,0x01,	
/*	  Report Size (1)*/ 								0x75,0x01,	
/*	  Report Count (14)*/								0x95,0x0E,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)*/ 	0x81,0x02,	
/*	  Usage Page (Vendor-Defined 1)*/					0x06,0x00,0xFF,  
/*	  Usage (Vendor-Defined 32)*/						0x09,0x20,	
/*	  Report Size (6)*/ 								0x75,0x06,	
/*	  Report Count (1)*/								0x95,0x01,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)*/ 	0x81,0x02,	
/*	  Usage Page (Generic Desktop)*/					0x05,0x01,	
/*	  Usage (Rx)*/										0x09,0x33,	
/*	  Usage (Ry)*/										0x09,0x34,	
/*	  Logical Minimum (0)*/ 							0x15,0x00,	
/*	  Logical Maximum (255)*/							0x26,0xFF,0x00,  
/*	  Report Size (8)*/ 								0x75,0x08,	
/*	  Report Count (2)*/								0x95,0x02,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)*/ 	0x81,0x02,	
/*	  Usage Page (Vendor-Defined 1)*/					0x06,0x00,0xFF,  
/*	  Usage (Vendor-Defined 33)*/						0x09,0x21,	
/*	  Report Count (54)*/								0x95,0x36,	
/*	  Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)*/ 	0x81,0x02,	
/*	  Report ID (5)*/									0x85,0x05,	
/*	  Usage (Vendor-Defined 34)*/						0x09,0x22,	
/*	  Report Count (31)*/								0x95,0x1F,	
/*	  Output (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit)*/	0x91,0x02,	
/*	  Report ID (3)*/									0x85,0x03,	
/*	  Usage (Vendor-Defined 10017)*/					0x0A,0x21,0x27,  
/*	  Report Count (47)*/								0x95,0x2F,	
/*	  Feature (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit)*/	0xB1,0x02,	
/*End Collection*/										0xC0,
/*Usage Page (Vendor-Defined 241)*/ 					0x06,0xF0,0xFF,  
/*Usage (Vendor-Defined 64)*/							0x09,0x40,	
/*Collection (Application)*/							0xA1,0x01,	
/*	  Report ID (240)*/ 								0x85,0xF0,	
/*	  Usage (Vendor-Defined 71)*/						0x09,0x47,	
/*	  Report Count (63)*/								0x95,0x3F,	
/*	  Feature (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit)*/	0xB1,0x02,	
/*	  Report ID (241)*/ 								0x85,0xF1,	
/*	  Usage (Vendor-Defined 72)*/						0x09,0x48,	
/*	  Report Count (63)*/								0x95,0x3F,	
/*	  Feature (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit)*/	0xB1,0x02,	
/*	  Report ID (242)*/ 								0x85,0xF2,	
/*	  Usage (Vendor-Defined 73)*/						0x09,0x49,	
/*	  Report Count (15)*/								0x95,0x0F,	
/*	  Feature (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit)*/	0xB1,0x02,	
/*	  Report ID (243)*/ 								0x85,0xF3,	
/*	  Usage (Vendor-Defined 18177)*/					0x0A,0x01,0x47,  
/*	  Report Count (7)*/								0x95,0x07,	
/*	  Feature (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit)*/	0xB1,0x02,	
/*End Collection*/										0xC0  
};

static inline u8* usbgamepad_get_report_desc_ps4(void) {
	return (u8*) (gamepad_report_desc_ps4);
}

static inline u16 usbgamepad_get_report_desc_ps4_size(void) {
	return sizeof(gamepad_report_desc_ps4);
}



