/*
 * usb.c
 *
 *  Created on: 22.02.2020
 *      Author: pvvx
 */
#include "proj/tl_common.h"
#include "app.h"
#if USE_USB_CDC
#include "usb.h"
#if (USE_I2C_DEV)
#include "i2c_dev.h"
#endif
#if (USE_INT_ADC)
#include "adc_dev.h"
#endif
#if (USE_HX711)
#include "hx711.h"
#endif
#if (USE_INT_DAC)
#include "dac.h"
#endif
#if	(USE_INT_UART)
#include "uart_dev.h"
#endif

#if (USE_BLE)
extern u8 blt_rxfifo_b[];
extern u8 blt_txfifo_b[];
extern u8 wrk_enable;
extern u8 wrk_stage;
#define usb_pwd wrk_enable // Events: USB_SET_CTRL_UART DTR Off, USB_PWDN, USB_RESET
#define usb_pwup wrk_stage // Events: USB_SET_CTRL_UART DTR On
#define usb_buf_rx ((unsigned char *)&blt_rxfifo_b)
#define usb_buf_tx ((unsigned char *)&blt_txfifo_b)
#else
u8 usb_pwd = 0; // Events: USB_SET_CTRL_UART DTR Off, USB_PWDN, USB_RESET
u8 usb_pwup = 1; // Events: USB_SET_CTRL_UART DTR On
usb_buf_t usb_buf;
#define usb_buf_rx ((unsigned char *)&usb_buf.rx)
#define usb_buf_tx ((unsigned char *)&usb_buf.tx)
#endif

/* rxFunc rx callback function
 * Called from Irq (!) */
_attribute_ram_code_ void USBCDC_RxCb(unsigned char *data, unsigned int len){
	if (len) { // есть данные?
#ifdef USB_LED_RX
		USB_LED_RX();
#endif
		if(rx_len == 0
			&& data
			&& len >= sizeof(blk_head_t)
			&& len <= MTU_RX_DATA_SIZE
			&& data[0]+sizeof(blk_head_t) <= len) {
			rx_len = data[0]+sizeof(blk_head_t);
			memcpy(&read_pkt, data, rx_len);
		}
		USBCDC_RxBufSet(usb_buf_rx); // назначить новый буфер (в данном приложении единственный)
	}
}
/////////////////////////////////////////////////////////////////////
void usb_init(void) {
		/* Initialize usb cdc */
		USB_Init();
		USBCDC_RxBufSet(usb_buf_rx);
		USBCDC_CBSet(USBCDC_RxCb, NULL); // CDC_TxDoneCb);
		usb_dp_pullup_en(1);
}
/////////////////////////////////////////////////////////////////////
// main usb loop flow
/////////////////////////////////////////////////////////////////////
void main_usb_loop() {
	u32 i;
#if USE_HX711
	if(hx711_mode && !gpio_read(HX711_DOUT)) {
		hx711_buf[hx711_wr++] = hx711_get_data(hx711_mode);
		hx711_wr &= HX711_BUF_CNT-1;
		all_rd_count++;
	}
#endif
	if(tx_len) { // есть данные для передачи
		if(USBCDC_IsAvailable()) {
			memcpy(usb_buf_tx, &send_pkt, tx_len);
			USBCDC_DataSend(usb_buf_tx, tx_len);
			tx_len = 0;
		}
	} else
#if USE_INT_ADC
	if(cfg_adc.pktcnt
		&& (i = get_adc_dfifo((u16 *)&send_pkt.data.ui, cfg_adc.pktcnt, SMPS_BLK_CNT)) != 0) {
		all_rd_count += i;
		if(!USBCDC_IsAvailable())
			not_send_count++;
		send_pkt.head.size = i*2;
		send_pkt.head.cmd = CMD_DEV_ADC;
		tx_len = i*2+sizeof(blk_head_t);
	} else
#endif
#if USE_I2C_DEV
	if(cfg_i2c.pktcnt
//		&& timer_flg
		&& ((i2c_buf_wr - i2c_buf_rd) & (I2C_BUF_SIZE - 1)) > cfg_i2c.pktcnt) {
		for(i = 0; i < cfg_i2c.pktcnt ; i ++) {
			send_pkt.data.si[i] = i2c_buf[i2c_buf_rd];
			i2c_buf_rd++;
#if (I2C_BUF_SIZE != 256)
			i2c_buf_rd &= I2C_BUF_SIZE - 1;
#endif
		}
		all_rd_count+=cfg_i2c.pktcnt;
		if(!USBCDC_IsAvailable())
			not_send_count++;
		send_pkt.head.size = cfg_i2c.pktcnt*2;
		send_pkt.head.cmd = CMD_DEV_I2C;
		tx_len = cfg_i2c.pktcnt*2+sizeof(blk_head_t);
	} else
#endif
#if USE_HX711
	if(((hx711_wr - hx711_rd) & (HX711_BUF_CNT-1)) > HX711_DATA_OUT) {
		for(i = 0; i < HX711_DATA_OUT; i ++) {
			send_pkt.data.hxo.data[i] = hx711_buf[hx711_rd++];
			hx711_rd &= HX711_BUF_CNT-1;
		}
		send_pkt.head.cmd = CMD_DEV_TST;
		send_pkt.head.size = sizeof(hx711_out_t);
		tx_len = sizeof(hx711_out_t)+sizeof(blk_head_t);
	} else
#endif
#if (USE_INT_UART)
	if(reg_dma_rx_rdy0 & FLD_DMA_UART_RX) {
		tx_len = uart_rx_buff[0];
		memcpy(&send_pkt.data, &uart_rx_buff[4], tx_len);
		reg_dma_irq_src = FLD_DMA_UART_RX;
		send_pkt.head.cmd = CMD_DEV_UAR;
		send_pkt.head.size = tx_len;
		tx_len += sizeof(blk_head_t);
	} else
#endif
	if(rx_len) {
		tx_len = cmd_decode(&send_pkt, &read_pkt, rx_len);
		rx_len = 0;
	} else
	if(usb_pwd) { // Events: USB_SET_CTRL_UART DTR Off, USB_PWDN, USB_RESET
#if USE_I2C_DEV
		Timer_Stop();
		I2CDevSleep();
#endif
#if USE_INT_ADC
		ADC_Stop();
#endif
#if (USE_HX711)
		hx711_gpio_go_sleep();
		hx711_mode = 0;
		hx711_wr = 0;
		hx711_rd = 0;
#endif
#if USE_INT_ADC
		uart_deinit();
#endif
		sdm_off();
		ExtDevPowerOff();
		usb_pwd = 0;
	}
	else if(usb_pwup) { // Events: USB_SET_CTRL_UART DTR On
		ExtDevPowerOn();
#if USE_I2C_DEV
		sleep_us(100);
		I2CDevWakeUp();
#endif
#if (USE_HX711)
//		hx711_gpio_wakeup();
#endif
		usb_pwup = 0;
	}
}

#endif // USE_USB_CDC
