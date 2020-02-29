/*
 * uart_dev.h
 *
 *  Created on: 23.02.2020
 *      Author: pvvx
 */

#ifndef UART_DEV_H_
#define UART_DEV_H_

#if (MCU_CORE_TYPE == MCU_CORE_8269)
// MTU BLE max Telink SDK 241 bytes -> 241-2 = 239. 239 + 5 + step 16 bytes = 256
#define UART_RX_BUFF_SIZE      (256) // max dma read = x-5 = 59 bytes, step 16 bytes
#define UART_TX_BUFF_SIZE      (256)
#else
#define UART_RX_BUFF_SIZE      (64) // max dma read = x-5 = 59 bytes, step 16 bytes
#define UART_TX_BUFF_SIZE      (64)
#endif

extern unsigned char uart_rx_buff[UART_RX_BUFF_SIZE];
extern unsigned char uart_tx_buff[UART_TX_BUFF_SIZE];

extern dev_uart_cfg_t cfg_uart;
extern const dev_uart_cfg_t def_cfg_uart;

void uart_init(dev_uart_cfg_t * p);
void uart_deinit(void);
unsigned char uart_send(unsigned char* addr);


#endif /* UART_DEV_H_ */
