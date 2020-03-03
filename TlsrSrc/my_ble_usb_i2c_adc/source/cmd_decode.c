/*
 * cmd_decode.c
 *
 *  Created on: 14.01.2020
 *      Author: pvvx
 */
#include "proj/tl_common.h"

#include "app.h"
#if (USE_BLE)
#include "ble.h"
#endif
#include "flash_eep.h"
#include "eep_id.h"
#if (USE_USB_CDC)
#include "usb.h"
#endif
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

u32 all_rd_count = 0; // status
u32 not_send_count = 0; // status

blk_rx_pkt_t read_pkt; // приемный буфер
blk_tx_pkt_t send_pkt; // буфер отправки

u8 rx_len = 0; // flag - пришла команда в read_pkt
u8 tx_len = 0; // flag - есть данные для передачи в send_pkt

//--------- < Test!
static inline void test_function(void) {};
//--------- Test! >

#if (USE_USB_CDC && USE_BLE)
volatile unsigned char usb_actived; // flag =1 -> usb, =0 -> ble
int tst_usb_actived(void) {
	usb_actived = gpio_read(KEY_BLE_USB) != 0;
	return usb_actived;
}
#endif

void send_rtm_err(u16 err_id, u16 err) {
	send_pkt.head.size = sizeof(dev_err_t);
	send_pkt.head.cmd = CMD_DEV_ERR;
	send_pkt.data.err.id = err_id;
	send_pkt.data.err.err = err;
#if (USE_USB_CDC && USE_BLE)
	if(usb_actived) {
		if(USBCDC_IsAvailable()) {
			USBCDC_DataSend((u8 *) &send_pkt, sizeof(blk_head_t) + sizeof(dev_err_t));
		}
	} else {
		bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, sizeof(blk_head_t) + sizeof(dev_err_t));
	}
#elif (USE_USB_CDC)
	if(USBCDC_IsAvailable()) {
		USBCDC_DataSend((u8 *) &send_pkt, sizeof(blk_head_t) + sizeof(dev_err_t));
	}
#elif (USE_BLE)
	bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, sizeof(blk_head_t) + sizeof(dev_err_t));
#endif
}

/*******************************************************************************
 * Function Name : usb_ble_cmd_decode.
 * Description	 : Main loop routine.
 * Input		 : blk_tx_pkt_t * pbufo, blk_tx_pkt_t * pbufi, int rxlen
 * Return		 : txlen.
 *******************************************************************************/
unsigned int cmd_decode(blk_tx_pkt_t * pbufo, blk_rx_pkt_t * pbufi, unsigned int rxlen) {
	unsigned int txlen = 0;
	u8 tmp;
//	if (rxlen >= sizeof(blk_head_t)) {
//		if (rxlen >= pbufi->head.size + sizeof(blk_head_t)) {
			pbufo->head.cmd = pbufi->head.cmd;
			switch (pbufi->head.cmd) {
			case CMD_DEV_VER: // Get Ver
				pbufo->data.ui[0] = INT_DEV_ID; // DevID = 0x1021
				pbufo->data.ui[1] = INT_DEV_VER; // Ver 1.2.3.4 = 0x1234
				txlen = sizeof(u16) + sizeof(u16) + sizeof(blk_head_t);
				break;
#if (USE_INT_UART)
			case CMD_DEV_UAR: // Send UART
				//	if(!(reg_uart_status1 & FLD_UART_TX_DONE))
				txlen = pbufi->head.size;
				if (pbufi->head.size) {
					if(!uart_enabled) {
						uart_init(&cfg_uart);
#if (USE_BLE)
						sleep_mode |= 4;
#endif
					} else
					if(UartTxBusy()) {
						pbufo->head.size = sizeof(dev_err_t);
						pbufo->head.cmd = CMD_DEV_ERR;
						pbufo->data.err.id = RTERR_UART;
						pbufo->data.err.err = 0;
						txlen = sizeof(dev_err_t) + sizeof(blk_head_t);
						break;
					}
					if(txlen > UART_RX_TX_LEN)
						txlen = UART_RX_TX_LEN;
					memcpy(&uart_tx_buff[4], &pbufi->data.uc, txlen);
					uart_tx_buff[0] = txlen;
					if(!uart_send(uart_tx_buff)){
						pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
						txlen = 0 + sizeof(blk_head_t);
						break;
					}
				}
//				pbufo->data.uc[0] = txlen;
				txlen = 0; // 1 + sizeof(blk_head_t);
				break;
			case CMD_DEV_UAC: // Set UART CFG/ini
				if (pbufi->head.size) {
					memcpy(&cfg_uart, &pbufi->data.ua,
						(pbufi->head.size > sizeof(cfg_uart))? sizeof(cfg_uart) : pbufi->head.size);
				}
				uart_init(&cfg_uart);
#if (USE_BLE)
				sleep_mode |= 4;
#endif
				memcpy(&pbufo->data, &cfg_uart, sizeof(cfg_uart));
				txlen = sizeof(cfg_uart) + sizeof(blk_head_t);
				break;
#endif
#if (USE_I2C_DEV)
			case CMD_DEV_CFG: // Get/Set CFG/ini & Start measure
				if (pbufi->head.size) {
					timer_flg = 0;
					memcpy(&cfg_i2c, &pbufi->data.ci2c,
						(pbufi->head.size > sizeof(cfg_i2c))? sizeof(cfg_i2c) : pbufi->head.size);
					if (!InitI2CDevice()) {
						pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
						txlen = 0 + sizeof(blk_head_t);
						break;
					}
#if	(USE_BLE)
					if(
#if	(USE_BLE && USE_USB_CDC)
							usb_actived ||
#endif
							(sleep_mode&1))
#endif
						timer_flg = 1;
				}
				memcpy(&pbufo->data, &cfg_i2c, sizeof(cfg_i2c));
				txlen = sizeof(cfg_i2c) + sizeof(blk_head_t);
				break;
			case CMD_DEV_SCF: // Store CFG/ini in Flash
				if(pbufi->head.size < sizeof(dev_scf_t)) {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
					break;
				}
				pbufo->data.ud[0] = 0;
#if	(USE_I2C_DEV)
				if(pbufi->data.scf.i2c)
					pbufo->data.scf.i2c = flash_write_cfg(&cfg_i2c, EEP_ID_I2C_CFG, sizeof(cfg_i2c));
#endif
#if	(USE_INT_ADC)
				if(pbufi->data.scf.adc)
					pbufo->data.scf.adc = flash_write_cfg(&cfg_adc, EEP_ID_ADC_CFG, sizeof(cfg_adc));
#endif
#if	(USE_BLE)
				if(pbufi->data.scf.con)
					pbufo->data.scf.con = flash_write_cfg(&ble_con_ini, EEP_ID_CON_CFG, sizeof(ble_con_ini));
				if(pbufi->data.scf.adv)
					pbufo->data.scf.adv = flash_write_cfg(&ble_cfg_ini, EEP_ID_BLE_CFG, sizeof(ble_cfg_ini));
#endif
#if	(USE_INT_DAC)
				if(pbufi->data.scf.dac)
					pbufo->data.scf.dac = flash_write_cfg(&cfg_dac, EEP_ID_DAC_CFG, sizeof(cfg_dac));
#endif
#if	(USE_INT_UART)
				if(pbufi->data.scf.uart)
					pbufo->data.scf.uart = flash_write_cfg(&cfg_uart, EEP_ID_UART_CFG, sizeof(cfg_uart));
#endif
				txlen = sizeof(dev_scf_t) + sizeof(blk_head_t);
				break;
			//-------
			case CMD_DEV_GRG: // Get reg
				tmp = irq_disable();
				if (I2CBusReadWord(pbufi->data.reg.dev_addr, pbufi->data.reg.reg_addr,
					(uint16_t *)&pbufo->data.reg.data)) {
					pbufo->data.ui[0] = pbufi->data.ui[0];
					irq_restore(tmp);
					txlen = sizeof(reg_wr_t) + sizeof(blk_head_t);
				} else {
					irq_restore(tmp);
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
				}
				break;
			case CMD_DEV_SRG: // Set reg
				tmp = irq_disable();
				if (I2CBusWriteWord(pbufi->data.reg.dev_addr, pbufi->data.reg.reg_addr,
						pbufi->data.reg.data)) {
					pbufo->data.reg = pbufi->data.reg;
					irq_restore(tmp);
					txlen = sizeof(reg_wr_t) + sizeof(blk_head_t);
				} else {
					irq_restore(tmp);
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
				};
				break;
#endif
#if (USE_INT_ADC)
			case CMD_DEV_CAD: // Get/Set CFG/ini ADC & Start measure
				if (pbufi->head.size) {
					memcpy(&cfg_adc, &pbufi->data.cadc,
						(pbufi->head.size > sizeof(cfg_adc))? sizeof(cfg_adc) : pbufi->head.size);
					if (!InitADCDevice()) {
						pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
						txlen = 0 + sizeof(blk_head_t);
						break;
					}
				}
				memcpy(&pbufo->data, &cfg_adc, sizeof(cfg_adc));
				txlen = sizeof(cfg_adc) + sizeof(blk_head_t);
				break;
#endif
#if (USE_HX711)
			case CMD_DEV_TST: // Get/Set CFG/ini ADC & Start measure
				if (pbufi->head.size) {
					hx711_wr = 0;
					hx711_rd = 0;
					hx711_mode = pbufi->data.hxi.mode & 3;
					if(hx711_mode) {
						hx711_mode += HX711MODE_A128 - 1;
						hx711_gpio_wakeup();
#if (USE_BLE)
						if(!(sleep_mode&1))
							gpio_set_wakeup(HX711_DOUT, 0, 1);  // core(gpio) low wakeup suspend
#endif
					} else {
						hx711_gpio_go_sleep();
					}
				} else {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
				}
				txlen = 0 + sizeof(blk_head_t);
				break;
#endif
			//-------
			case CMD_DEV_STA: // Status
				pbufo->data.sta.rd_cnt = all_rd_count;
				pbufo->data.sta.to_cnt = not_send_count;
				txlen = sizeof(blk_head_t) + sizeof(dev_sta_t);
				break;
			//-------
#if (USE_BLE)
			case CMD_DEV_CPU: // Connect parameters Update
				memcpy(&pbufo->data.con, &ble_con_ini, sizeof(ble_con_ini));
				txlen = pbufi->head.size;
				if (txlen) {
					if(txlen > sizeof(ble_con_ini))
						txlen = sizeof(ble_con_ini);
					memcpy(&pbufo->data.con, &pbufi->data, txlen);
					if(pbufi->data.con.intervalMax & 0x8000)
						pbufo->data.con.intervalMax &= 0x7fff;
					else
						memcpy(&ble_con_ini, &pbufo->data.con, sizeof(ble_con_ini));
#if (USE_USB_CDC)
					if(!usb_actived)
#endif
						bls_l2cap_requestConnParamUpdate(
							pbufo->data.con.intervalMin &= 0x7fff,
							pbufo->data.con.intervalMax,
							pbufo->data.con.latency,
							pbufo->data.con.timeout);
				}
#if (USE_USB_CDC)
				if(!usb_actived) {
#endif
					cur_ble_con_ini.interval = bls_ll_getConnectionInterval();
					cur_ble_con_ini.latency = bls_ll_getConnectionLatency();
					cur_ble_con_ini.timeout = bls_ll_getConnectionTimeout();
					memcpy(&pbufo->data.uc[sizeof(ble_con_ini)], &cur_ble_con_ini, sizeof(cur_ble_con_ini));
					txlen = sizeof(blk_head_t) + sizeof(ble_con_ini) + sizeof(cur_ble_con_ini);
#if (USE_USB_CDC)
				}
				else txlen = sizeof(blk_head_t) + sizeof(ble_con_ini);
#endif
				break;
			//--------
			case CMD_DEV_BLE: // BLE parameters Update
				txlen = pbufi->head.size;
				if(txlen) {
					if(txlen > sizeof(ble_cfg_ini))
						txlen = sizeof(ble_cfg_ini);
					memcpy(&ble_cfg_ini, &pbufi->data, txlen);
#if (USE_USB_CDC)
					if(!usb_actived) {
#endif
						if(bls_ll_setAdvParam(
							ble_cfg_ini.intervalMin,
							ble_cfg_ini.intervalMax,
							ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC,
							0,  NULL, MY_APP_ADV_CHANNEL,
							ADV_FP_NONE) != BLE_SUCCESS) {
							pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
							txlen = 0 + sizeof(blk_head_t);
							break;
						}
						rf_set_power_level_index(ble_cfg_ini.rf_power);
						// use restart!
#if (USE_USB_CDC)
					}
#endif
				}
				memcpy(&pbufo->data, &ble_cfg_ini, sizeof(ble_cfg_ini));
				txlen = sizeof(ble_cfg_ini) + sizeof(blk_head_t);
				break;
#endif // USE_BLE
#if (USE_I2C_DEV)
			case CMD_DEV_UTR: // I2C read/write
				txlen = pbufi->data.utr.rdlen & 0x7f;
				if(pbufi->head.size >= sizeof(i2c_utr_t)
					&&  txlen <= sizeof(pbufo->data.wr.data)
					&&	I2CBusUtr(&pbufo->data.wr.data,
							&pbufi->data.utr,
							pbufi->head.size - sizeof(i2c_utr_t)) // wrlen:  addr len - 1
							) {
					pbufo->data.wr.dev_addr = pbufi->data.utr.wrdata[0];
					pbufo->data.wr.rd_count = txlen;
					txlen += sizeof(i2c_rd_t) + sizeof(blk_head_t);
				} else {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
				}
				break;
#endif // USE_I2C_DEV
			case CMD_DEV_PWR: // Power On/Off, Sleep
				if(pbufi->head.size < sizeof(dev_pwr_slp_t)) {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
					break;
				}
				if(pbufi->data.pwr.ExtDevPowerOn) {
					ExtDevPowerOn();
				}
#if (USE_I2C_DEV)
				if(pbufi->data.pwr.I2CDevWakeUp) {
					I2CDevWakeUp();
				}
#endif
				if(pbufi->data.pwr.ExtDevPowerOff) {
					ExtDevPowerOff();
				}
#if (USE_INT_DAC)
				if(pbufi->data.pwr.DAC_off) {
					sdm_off();
				}
#endif
#if (USE_I2C_DEV)
				if(pbufi->data.pwr.I2CDevSleep) {
					Timer_Stop();
					I2CDevSleep();
				}
#endif
#if (USE_INT_ADC)
				if(pbufi->data.pwr.ADC_Stop) {
					ADC_Stop();
				}
#endif
#if (USE_INT_UART)
				if(pbufi->data.pwr.Uart_off) {
					uart_deinit();
				}
#endif
#if (USE_BLE)
				if(pbufi->data.pwr.Disconnect) {
					bls_ll_terminateConnection(HCI_ERR_REMOTE_USER_TERM_CONN);
				}
				if(pbufi->data.pwr.Sleep_On) {
					sleep_mode = 0;
				}
				if(pbufi->data.pwr.Sleep_off) {
					sleep_mode = 1;
				}
				if(pbufi->data.pwr.Sleep_CPU) {
					sleep_mode |= 4;
				}
#endif
				if(pbufi->data.pwr.Reset) {
					cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER, clock_time() + 5000 * CLOCK_SYS_CLOCK_1MS); // <3.5 uA
				}
				if(pbufi->data.pwr.Test) {
					test_function();
				}
				txlen = 0 + sizeof(blk_head_t);
				break;
#if (USE_INT_DAC)
			case CMD_DEV_DAC: // Dac cfg
				txlen = pbufi->head.size;
				if (txlen) {
					if(txlen > sizeof(cfg_dac))
						txlen = sizeof(cfg_dac);
					memcpy(&cfg_dac, &pbufi->data.dac, txlen);
					dac_cmd(&cfg_dac);
				}
//				pbufo->data.uc[0] = dac_cmd(&pbufi->data.dac);
				memcpy(&pbufo->data.dac, &cfg_dac, sizeof(cfg_dac));
				txlen = sizeof(cfg_dac) + sizeof(blk_head_t);
				break;
#endif
			case CMD_DEV_DBG: // Debug
				if(pbufi->head.size > sizeof(dev_dbg_t)) {
					memcpy((u8 *)0x800000 + pbufi->data.dbg.addr, &pbufi->data.ud[1], pbufi->head.size - sizeof(dev_dbg_t));
				} else if(pbufi->head.size < sizeof(dev_dbg_t)) {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
					break;
				}
				txlen = pbufi->data.dbg.rd_cnt;
				if(txlen){
					if(txlen > sizeof(pbufi->data))
						txlen = sizeof(pbufi->data);
					memcpy(&pbufo->data, (u8 *)0x800000 + pbufi->data.dbg.addr, txlen);
				}
				txlen += sizeof(blk_head_t);
				break;
			default:
				pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
				txlen = 0 + sizeof(blk_head_t);
				break;
			};
			pbufo->head.size = txlen - sizeof(blk_head_t);
//		}
//		else
//			rxlen = 0;
//	}
	return txlen;
}
