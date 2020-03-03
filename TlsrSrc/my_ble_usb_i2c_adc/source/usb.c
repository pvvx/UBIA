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
#if (USE_BLE)
#if 1 // hw init TLSR8266/TLSR8269
		reg_rst_clk0 = 0
#if USE_SPI
				| FLD_CLK_SPI_EN
#endif
#if USE_I2C
//				| FLD_CLK_I2C_EN
#endif
#if USE_USB
				| FLD_CLK_USB_EN
				| FLD_CLK_USB_PHY_EN
#endif
				| FLD_CLK_MCU_EN
				| FLD_CLK_MAC_EN
#if USE_INT_ADC
//				| FLD_CLK_ADC_EN
#endif
//				| FLD_CLK_ZB_EN
				;
		// reg_clk_en + reg_clk_sel + reg_i2s_step = 0x002C1097
		REG_ADDR32(0x64) = 0x002C0000 // FLD_CLK_SEL_DIV(0x0c), FLD_CLK_SEL_SRC(0x02)
				| FLD_CLK_GPIO_EN
				| FLD_CLK_ALGM_EN
#if USE_USB | USE_DMA
				| FLD_CLK_DMA_EN
#endif
#if USE_UART
				| FLD_CLK_UART_EN
#endif
#if USE_PWM
				| FLD_CLK_PWM_EN
#endif
#if USE_AES
				| FLD_CLK_AES_EN
#endif
//				| FLD_CLK_32K_TIMER_EN	// clk32k for system timer
				| FLD_CLK_PLL_EN
				| FLD_CLK_SWIRE_EN
//				| FLD_CLK_32K_QDEC_EN	// 32k for qdec
#if USE_AUD
				| FLD_CLK_AUD_EN
#endif
#if USE_DFIFO
				| FLD_CLK_DIFIO_EN
#endif
#if USE_KEYSCAN
				| FLD_CLK_KEYSCAN_EN
#endif
				| FLD_CLK_MCIC_EN
#if	USE_QDEC
				| FLD_CLK_QDEC_EN
#endif
				;
		REG_ADDR32(0x70) = 0 // = 0x04000400
		/* reg_fhs_sel [0x70], After reset = 0x00 */
			| (0) // bit1 FHS sel: 192M clock from pll | 32M clock from rc osc
		/* reg_dcdc_clk [0x71], After reset [0x71] = 0x04 */
			| ((1<<2)<<8)
		/* reg_?? [0x72], After reset [0x72] = 0x00 */
			| (0<<16) // watchdog reset status bit 0x72[0] = 1, manually clear - write '1'
		/* reg_clk_mux_cel [0x73], After reset  = 0x14
		* [0] clk32k select; 0: sel 32k osc 1: 32k pad
		* [1] dmic clock select, 1: select 32k (refer to bit[0] to decide which 32k ; 0: dmic clk div
		* [2] usb phy clock select, 1 : 192M divider 0: 48M pll
		* [7:4] r_lpr_div, decide system clock speed in low power mode	 */
			| ((1<<2)<<24);
#if SET_PLL == QUARTZ_12MHZ
		reg_pll_ctrl_a = FLD_PLL_A_CAL_DONE_EN | 0x80;
		analog_write(0x099, 0xb1);
		analog_write(0x082, 0x20);
		analog_write(0x09e, 0xad);
#else // SET_PLL == CLK_QUARTZ
		reg_pll_ctrl_a = FLD_PLL_A_CAL_DONE_EN;
		analog_write(0x099, 0x31);
		analog_write(0x082, 0x34);
		analog_write(0x09e, 0x82);
#endif
#endif // hw init

#endif //  (USE_BLE)
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
#if USE_INT_UART
		uart_deinit();
#endif
#if USE_INT_DAC
		sdm_off();
#endif
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
