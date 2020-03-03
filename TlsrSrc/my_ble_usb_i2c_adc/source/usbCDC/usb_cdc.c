/********************************************************************************************************
 * @file     usb_cdc.c
 *
 * @brief    This file provides set of functions to manage the USB cdc device
 *
 * @author   jian.zhang@telink-semi.com
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
#include "usb.h"

static void USBCDC_Write32(unsigned int value);
static void USBCDC_Read32(unsigned int *value);

USBCDC_Ctrl_t cdc_vs;

#if (USB_CDC_MAX_RX_BLK_SIZE > 64)
unsigned char usb_flg_rx_timeot;
#endif

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
    {
        0, //ControlInterfaceNumber

        CDC_TX_EPNUM, // DataINEndpointNumber
        CDC_TXRX_EPSIZE, // DataINEndpointSize
        0, // DataINEndpointDoubleBank

        CDC_RX_EPNUM,  // DataOUTEndpointNumber
        CDC_TXRX_EPSIZE, // DataOUTEndpointSize
        0, // DataOUTEndpointDoubleBank

        CDC_NOTIFICATION_EPNUM, // NotificationEndpointNumber
        CDC_NOTIFICATION_EPSIZE, // NotificationEndpointSize
        0,  // NotificationEndpointDoubleBank
    },
};
USB_ClassInfo_CDC_Device_t *CDCInterfaceInfo = &VirtualSerial_CDC_Interface;


static void USBCDC_Write32(unsigned int value)
{
    USBHW_CtrlEpDataWrite(value & 0xff);
    USBHW_CtrlEpDataWrite((value >> 8) & 0xff);
    USBHW_CtrlEpDataWrite((value >> 16) & 0xff);
    USBHW_CtrlEpDataWrite((value >> 24) & 0xff);
}

static void USBCDC_Read32(unsigned int *value)
{
    unsigned int temp = 0;
    *value = USBHW_CtrlEpDataRead();

    temp =  USBHW_CtrlEpDataRead();
    *value = (temp << 8) | (*value);
    temp = 0;

    temp =  USBHW_CtrlEpDataRead();
    *value = (temp << 16) | (*value);
    temp = 0;

    temp =  USBHW_CtrlEpDataRead();
    *value = (temp << 24) | (*value);
}

/**
 * @brief This function handles the USB CDC class-specific requests from host
 * @param[in]   bRequest the request type
 * @param[in]   wValue word-sized field that varies according to request
 * @param[in]   wIndex word-sized field that varies according to request
 *              typically used to pass an index or offset
 * @param[in]   wLength number of bytes to transfer if there is a Data stage
 * @return none
 */
void USBCDC_ControlRequestProcesss(unsigned char bRequest, unsigned short wValue, unsigned short wIndex, unsigned short wLength)
{
    if (wIndex != CDCInterfaceInfo->Config.ControlInterfaceNumber)
        return;

    switch (bRequest) {
    case CDC_REQ_GetLineEncoding:
        USBCDC_Write32(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);
        USBHW_CtrlEpDataWrite(CDCInterfaceInfo->State.LineEncoding.CharFormat);
        USBHW_CtrlEpDataWrite(CDCInterfaceInfo->State.LineEncoding.ParityType);
        USBHW_CtrlEpDataWrite(CDCInterfaceInfo->State.LineEncoding.DataBits);
        break;

    case CDC_REQ_SetLineEncoding:
        USBCDC_Read32(&CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);
        CDCInterfaceInfo->State.LineEncoding.CharFormat  = USBHW_CtrlEpDataRead();
        CDCInterfaceInfo->State.LineEncoding.ParityType  = USBHW_CtrlEpDataRead();
        CDCInterfaceInfo->State.LineEncoding.DataBits    = USBHW_CtrlEpDataRead();
#ifdef USB_SET_CFG_UART
        USB_SET_CFG_UART(LineEncoding);
#endif
        break;

    case CDC_REQ_SetControlLineState:
        CDCInterfaceInfo->State.ControlLineStates.HostToDevice = wValue;
#ifdef USB_SET_CTRL_UART
        USB_SET_CTRL_UART(wValue);
#endif
        break;

    case CDC_REQ_SendBreak:
        break;
    }
}

/**
 * @brief This function initializes the USB CDC device
 * @param   none
 * @return none
 */
void USBCDC_Init(void)
{
    /* Init UART parameters */
    CDCInterfaceInfo->State.LineEncoding.BaudRateBPS = 115200;
    CDCInterfaceInfo->State.LineEncoding.CharFormat = CDC_LINEENCODING_OneStopBit;
    CDCInterfaceInfo->State.LineEncoding.ParityType = CDC_PARITY_None;
    CDCInterfaceInfo->State.LineEncoding.DataBits = 8;
    cdc_vs.lastRecvIndex = 0;
#ifdef USB_SET_CFG_UART
    USB_SET_CFG_UART(LineEncoding);
#endif
}


/**
 * @brief This function sets the USB CDC tx and rx callback function
 * @param[in]   rxFunc rx callback function
 * @param[in]   txCb tx callback function
 * @return none
 */
void USBCDC_CBSet(cdc_cbrxFn_t rxFunc, cdc_cbtxFn_t txCb)
{
    cdc_vs.rxCb = rxFunc;
    cdc_vs.txCb = txCb;
}

/*
static int USBCDC_SendTimeoutCb(void* arg)
{
	(void) arg;
	unsigned int r = irq_disable();
	cdc_vs.txBuf = NULL;
	cdc_vs.lenToSend = 0;
	cdc_vs.lastSendIndex = 0;
    if (cdc_vs.txCb) {
        cdc_vs.txCb(0);
    }
	irq_restore(r);
	return -1;	// больше не вызывать это событие (удалить callback)
}
*/

#if	(USB_CDC_MAX_RX_BLK_SIZE > 64)
/*
 * USB rx TimeOuts
 * Called from Irq (!)
 */
_attribute_ram_code_ void USB_RxTimeOuts(void)
{
    unsigned char* p;
	unsigned int len;

	len = cdc_vs.lastRecvIndex;
	cdc_vs.lastRecvIndex = 0;

	/* Clear the buffer */
	p = cdc_vs.rxBuf;
	cdc_vs.rxBuf = NULL;

	/* Callback */
	if (cdc_vs.rxCb) {
		cdc_vs.rxCb(p, len);
	}
}
#endif
#if	(USB_CDC_MAX_RX_BLK_SIZE > 64)
/**
 * @brief This function handles the received data
 * @param   none
 * @return none
 * Called from Irq (!)
 */
_attribute_ram_code_ void USBCDC_DataRecv(void)
{
    unsigned int i;
    unsigned int len;
    unsigned char* p;

    len = REG_USB_EP_PTR(CDC_RX_EPNUM & 0x07);	// FIFO max 256 bytes
//    if (len > CDC_TXRX_EPSIZE)
//    	len = CDC_TXRX_EPSIZE;
//    if (!len)
//    	len = USB_MAX_RX_FIFO_SIZE;
    USBHW_EpPtrReset(CDC_RX_EPNUM);
    if (cdc_vs.rxBuf) {
    	for (i = 0; i < len; i++) {
    		if (cdc_vs.lastRecvIndex < USB_CDC_MAX_RX_BLK_SIZE)
    			cdc_vs.rxBuf[cdc_vs.lastRecvIndex++] = USBHW_EpDataRead(CDC_RX_EPNUM);
    		else
    			(void)USBHW_EpDataRead(CDC_RX_EPNUM);
    	}
    }
    else {     /* No buffer ! */
#ifdef LED_DEBUG
       	reg_gpio_out(LED_DEBUG) |=  LED_DEBUG & 0xff;
#endif
    	for (i = 0; i < len; i++) {
    		(void)USBHW_EpDataRead(CDC_RX_EPNUM);
    	}
#ifdef LED_DEBUG
   		reg_gpio_out(LED_DEBUG) &=  ~(LED_DEBUG & 0xff);
#endif
    }
    if (len == 0
    	|| cdc_vs.lastRecvIndex >= USB_CDC_MAX_RX_BLK_SIZE
#if	(USB_CDC_MAX_RX_BLK_SIZE > 64)
    	|| (len & (CDC_TXRX_EPSIZE-1)) != 0
#endif
    	) {
#if	(USB_CDC_MAX_RX_BLK_SIZE > 64)
        reg_irq_mask3 &= ~BIT(0); // reg_irq_mask &= ~FLD_IRQ_USB_250US_EN
        usb_flg_rx_timeot = 0;
#endif
    	len = cdc_vs.lastRecvIndex;
        cdc_vs.lastRecvIndex = 0;

        /* Clear the buffer */
        p = cdc_vs.rxBuf;
        cdc_vs.rxBuf = NULL;

        /* Callback */
        if (cdc_vs.rxCb) {
            cdc_vs.rxCb(p, len);
        }
    }
#if	(USB_CDC_MAX_RX_BLK_SIZE > 64)
    else {
        reg_irq_src3 = BIT(0); // Clear FLD_IRQ_USB_250US_EN flag
        // rx timeout = 3 -> 500..750 us (не работает на всех USB, надо больше!)
        // 1.75 ms (ModbusRTU?) -> USBCDC_RecvTimeoutCb
        usb_flg_rx_timeot = USB_RX_LAST64BLK_TIMEOUT/250;
    	reg_irq_mask3 |= BIT(0); // reg_irq_mask |= FLD_IRQ_USB_250US_EN
    }
#endif
}
#else
_attribute_ram_code_ void USBCDC_DataRecv(void)
{
    unsigned int i;
    unsigned int len;
    unsigned char* p;

    len = REG_USB_EP_PTR(CDC_RX_EPNUM & 0x07);	// FIFO max 256 bytes
//    if (len > CDC_TXRX_EPSIZE)
//    	len = CDC_TXRX_EPSIZE;
//    if (!len)
//    	len = USB_MAX_RX_FIFO_SIZE;
    USBHW_EpPtrReset(CDC_RX_EPNUM);
    if (cdc_vs.rxBuf) {
    	for (i = 0; i < len; i++) {
    		if (cdc_vs.lastRecvIndex < USB_CDC_MAX_RX_BLK_SIZE)
    			cdc_vs.rxBuf[cdc_vs.lastRecvIndex++] = USBHW_EpDataRead(CDC_RX_EPNUM);
    		else
    			(void)USBHW_EpDataRead(CDC_RX_EPNUM);
    	}
    }
    else {     /* No buffer ! */
#ifdef LED_DEBUG
       	reg_gpio_out(LED_DEBUG) |=  LED_DEBUG & 0xff;
#endif
    	for (i = 0; i < len; i++) {
    		(void)USBHW_EpDataRead(CDC_RX_EPNUM);
    	}
#ifdef LED_DEBUG
   		reg_gpio_out(LED_DEBUG) &=  ~(LED_DEBUG & 0xff);
#endif
    }
   	len = cdc_vs.lastRecvIndex;
    cdc_vs.lastRecvIndex = 0;

    /* Clear the buffer */
    p = cdc_vs.rxBuf;
    cdc_vs.rxBuf = NULL;

    /* Callback */
    if (cdc_vs.rxCb) {
         cdc_vs.rxCb(p, len);
    }
}
#endif

unsigned char usbWriteByte(u8 byte){
	if (USBHW_IsEpBusy(CDC_TX_EPNUM)) {
		/* Return to wait IRQ come again */
		return 0;
	}
	REG_USB_EP_PTR(CDC_TX_EPNUM) = 0;
	REG_USB_EP_DAT(CDC_TX_EPNUM) = byte;
	/* Write ACK */
	REG_USB_EP_CTRL(CDC_TX_EPNUM) = FLD_EP_DAT_ACK;        // ACK
	while(USBHW_IsEpBusy(CDC_TX_EPNUM));
	return 1;
}
/**
 * @brief This function sends bulk data to host as the USB CDC device
 * @param[in]   none
 * @return the length of the data send out
 * Called from Irq (!)
 */
_attribute_ram_code_ unsigned int USBCDC_BulkDataSend(void)
{
	unsigned int i, len;

    /* Wait until not busy */
    if (USBHW_IsEpBusy(CDC_TX_EPNUM)) {
   		/* Return to wait IRQ come again */
        return 0;
    }
    if (cdc_vs.txBuf == NULL) {
    	// снять task tx-timeout 5 ms контроля передачи
//    	ev_task_off(EV_USB_SEND_TIMEOUT);
        return 0;
    }
    /* Get the length to send in this bulk transaction */
    len = (cdc_vs.lenToSend > CDC_TXRX_EPSIZE) ? CDC_TXRX_EPSIZE : cdc_vs.lenToSend;
/*
    if (len == 0) {
    	// снять task tx-timeout 5 ms контроля передачи
//    	ev_task_off(EV_USB_SEND_TIMEOUT);
        return 0;
    }
*/
    cdc_vs.lenToSend -= len;

    reg_usb_ep_ptr(CDC_TX_EPNUM) = 0;
    if (cdc_vs.txBuf)
    	/* Write data to USB fifo */
    	for (i = 0; i < len; i++) {
    		reg_usb_ep_dat(CDC_TX_EPNUM) = cdc_vs.txBuf[cdc_vs.lastSendIndex++];
    }

    /* Write ACK */
    reg_usb_ep_ctrl(CDC_TX_EPNUM) = FLD_EP_DAT_ACK;        // ACK
#if 0
    unsigned int t = 0;
    while(USBHW_IsEpBusy(CDC_TX_EPNUM)) {
        if (t++ > 10000) {
            reg_usb_ep_ctrl(CDC_TX_EPNUM) &= 0xfe; // clear bit(0)
        }
    };
#else
    unsigned int t = clock_time();
    while(USBHW_IsEpBusy(CDC_TX_EPNUM)) {
#if (USE_I2C_DEV)
    	extern void TimerIrq(void);
        TimerIrq();
#endif
        if (clock_time() - t > USB_TX_ACK_BLK_TIMEOUT * CLOCK_SYS_CLOCK_1US) {
            REG_USB_EP_CTRL(CDC_TX_EPNUM) &= 0xfe; // clear bit(0)
            // TODO Error? -> callback ?
#ifdef LED_DEBUG
            reg_gpio_out(LED_DEBUG) |=  LED_DEBUG & 0xff;
#endif
            cdc_vs.lenToSend = 0;
#ifdef LED_DEBUG
            reg_gpio_out(LED_DEBUG) &=  ~(LED_DEBUG & 0xff);
#endif
            break;
        }
    }
#endif
    /* TX transaction finish */
    if (cdc_vs.lenToSend == 0) {

    	// снять task tx-timeout 5 ms контроля передачи
//    	ev_task_off(EV_USB_SEND_TIMEOUT);

    	if(len == CDC_TXRX_EPSIZE) {
    		reg_usb_ep_ptr(CDC_TX_EPNUM) = 0;
    		reg_usb_ep_ctrl(CDC_TX_EPNUM) = FLD_EP_DAT_ACK;        // ACK
        }
   		cdc_vs.lastSendIndex = 0;
   		if (cdc_vs.txCb) {
   			cdc_vs.txCb(len);
   		}
  		cdc_vs.txBuf = NULL;
    }
    else {
  	// взвести task tx-timeout 5 ms для контроля передачи
 //   	ev_task_set(EV_USB_SEND_TIMEOUT, USBCDC_SendTimeoutCb, NULL, 5000);
    }
    return len;
}
/**
 * @brief This function sends data to host as the USB CDC device
 * @param[in]   buf pointer to the data need to send
 * @param[in]   len length in byte of the data need to send
 * @return none
 */
_attribute_ram_code_ USBCDC_Sts_t USBCDC_DataSend(unsigned char *buf, unsigned int len)
{
	USBCDC_Sts_t ret = USB_BUSY;
    unsigned int r = irq_disable();
//	if (!cdc_vs.txBuf) {
	    /* Init the bulk transfer */
	    cdc_vs.lenToSend = len;
	    cdc_vs.lastSendIndex = 0;
	    cdc_vs.txBuf = buf;

	    if (buf) {
	    	// снять task tx-timeout 5 ms контроля передачи
//	    	ev_task_off(EV_USB_SEND_TIMEOUT);

	    	/* Send first bulk */
	        if (USBCDC_BulkDataSend())
	        	USBHW_DataEpAck(USB_EDP_CDC_OUT);
	    }
	    ret = USB_OK;
//    }
    irq_restore(r);

    return ret;
}

/**
 * @brief This function used to check whether the Tx of the control endpoint is busy
 * @param   none
 * @return 1: the control endpoint is busy 0: the control endpoint is idle
 */
unsigned char USBCDC_TxBusy(void)
{
    return USBHW_IsEpBusy(CDC_TX_EPNUM);
}

#endif // USE_USB_CDC
