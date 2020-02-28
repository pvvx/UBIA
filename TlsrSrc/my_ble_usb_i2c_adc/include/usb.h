/*
 * usb.h
 *
 *  Created on: 22.02.2020
 *      Author: pvvx
 */

#ifndef USB_H_
#define USB_H_

#if (USE_USB_CDC)
#include "proj_lib/pm.h"
#if (USE_BLE)
//extern u8 blt_rxfifo_b[];
//extern u8 blt_txfifo_b[];
//extern u8 wrk_enable;
//extern u8 wrk_stage;
#define USB_RESET() wrk_enable = 1 // Events: USB_SET_CTRL_UART DTR Off, USB_PWDN, USB_RESET
#define USB_PWDN() wrk_enable = 1 // Events: USB_SET_CTRL_UART DTR Off, USB_PWDN, USB_RESET
// DTR рукопожатия на линии
#define USB_SET_CTRL_UART(a) { if(a & 1) wrk_stage = 1; else wrk_enable = 1; }
#else
extern u8 usb_pwd;
extern u8 usb_pwup;
#define USB_RESET() usb_pwd = 1 // Events: USB_SET_CTRL_UART DTR Off, USB_PWDN, USB_RESET
#define USB_PWDN() usb_pwd = 1 // Events: USB_SET_CTRL_UART DTR Off, USB_PWDN, USB_RESET
// DTR рукопожатия на линии
#define USB_SET_CTRL_UART(a) { if(a & 1) usb_pwup = 1; else usb_pwd = 1; }
#endif

#define USB_CDC_MAX_RX_BLK_SIZE	64

//----------------------------- USB stack
// Define USB rx/tx buffer
#define RX_BUF_LEN    USB_CDC_MAX_RX_BLK_SIZE // in bytes
#define TX_BUF_LEN    MTU_DATA_SIZE // in bytes

typedef struct _usb_buf_t{
	unsigned char rx[RX_BUF_LEN]; // 64 bytes
	unsigned char tx[TX_BUF_LEN]; // 241 bytes
}usb_buf_t;

void USBCDC_RxCb(unsigned char *data, unsigned int len);
void usb_init(void);
void main_usb_loop(void);

#endif // USE_USB_CDC

#include "usbCDC/drivers.h"

#endif /* USB_H_ */
