/********************************************************************************************************
 * @file     uart.h
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

#ifndef 	uart_H
#define 	uart_H

#include "../mcu/register.h"
#include "../common/compatibility.h"
#include "../common/utility.h"

/**
 *  @brief  Define parity type
 */
typedef enum {
    PARITY_NONE = 0,
    PARITY_EVEN,
    PARITY_ODD,
} UART_ParityTypeDef;

/**
 *  @brief  Define the length of stop bit
 */
typedef enum {
    STOP_BIT_ONE = 0,
    STOP_BIT_ONE_DOT_FIVE = BIT(12),
    STOP_BIT_TWO = BIT(13),
} UART_StopBitTypeDef;

enum UARTIRQSOURCE{
	UARTNONEIRQ = 0,
	UARTRXIRQ = BIT(0),
	UARTTXIRQ = BIT(1),
};

enum{
	UARTRXIRQ_MASK  = BIT(0),
	UARTTXIRQ_MASK  = BIT(1),
	UARTIRQ_MASK    = UARTRXIRQ_MASK | UARTTXIRQ_MASK,
};

/**
 *  @brief  Define UART RTS mode
 */
enum {
    UART_RTS_MODE_AUTO = 0,
    UART_RTS_MODE_MANUAL,
};

enum {
	UART_DMA_RX_IRQ_DIS = 0,
	UART_DMA_RX_IRQ_EN  = 1,
	UART_DMA_TX_IRQ_DIS = 0,
	UART_DMA_TX_IRQ_EN  = 1,
};

enum {
	UART_NODMA_RX_IRQ_DIS = 0,
	UART_NODMA_RX_IRQ_EN  = 1,
	UART_NODMA_TX_IRQ_DIS = 0,
	UART_NODMA_TX_IRQ_EN  = 1,
};

#define CLK32M_UART9600         do{\
									uart_Init(302,10,PARITY_NONE,STOP_BIT_ONE);\
									uart_DmaModeInit(UART_DMA_TX_IRQ_EN, UART_DMA_RX_IRQ_EN);\
								}while(0)
#define CLK32M_UART115200       do{\
									uart_Init(30,8,PARITY_NONE,STOP_BIT_ONE);\
									uart_DmaModeInit(UART_DMA_TX_IRQ_EN, UART_DMA_RX_IRQ_EN);\
								}while(0)
#define CLK16M_UART115200       do{\
									uart_Init(9,13,PARITY_NONE,STOP_BIT_ONE);\
									uart_DmaModeInit(UART_DMA_TX_IRQ_EN, UART_DMA_RX_IRQ_EN);\
								}while(0)
#define CLK16M_UART9600         do{\
									uart_Init(118,13,PARITY_NONE,STOP_BIT_ONE);\
									uart_DmaModeInit(UART_DMA_TX_IRQ_EN, UART_DMA_RX_IRQ_EN);\
								}while(0)

//UART_TX/UART_RX gpio pin config
#if ((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
#define    UART_GPIO_CFG_PA6_PA7()    do{\
										gpio_set_func(GPIO_PA6, AS_UART);\
										gpio_set_func(GPIO_PA7, AS_UART);\
                                      }while(0)
#define    UART_GPIO_CFG_PB2_PB3()    do{\
	                                    gpio_set_func(GPIO_PB2, AS_UART);\
									    gpio_set_func(GPIO_PB3, AS_UART);\
                                      }while(0)
#define    UART_GPIO_CFG_PC2_PC3()    do{\
	                                    gpio_set_func(GPIO_PC2, AS_UART);\
									    gpio_set_func(GPIO_PC3, AS_UART);\
                                      }while(0)
#elif (MCU_CORE_TYPE == MCU_CORE_8266)
#define    UART_GPIO_CFG_PC6_PC7()    do{\
										gpio_set_func(GPIO_PC6, AS_UART);\
										gpio_set_func(GPIO_PC7, AS_UART);\
                                      }while(0)

#endif

#define UART_GPIO_8267_PA6_PA7      1
#define UART_GPIO_8267_PC2_PC3      2
#define UART_GPIO_8267_PB2_PB3      3

/**
 * @brief     get the status of uart irq.
 * @param[in] none
 * @return    0: not uart irq ;
 *            not 0: indicate tx or rx irq
 */
#define GET_UART_NOT_DMA_IRQ()       ((reg_uart_status0&FLD_UART_IRQ_FLAG) ? 1:0)  //not dma mode,1: occur uart irq; 0:not uart irq
/**********************************************************
*
*	@brief	reset uart module
*
*	@param	none
*
*	@return	none
*/
extern void uart_Reset(void );


/**********************************************************
*
*	@brief	clear error state of uart rx, maybe used when application detected UART not work
*
*	@parm	none
*
*	@return	'1' RX error flag rised and cleard success; '0' RX error flag not rised
*
*/
unsigned char uart_ErrorCLR(void);


/*******************************************************
*
*	@brief	uart initiate, set uart clock divider, bitwidth and the uart work mode
*
*	@param	uartCLKdiv - uart clock divider
*			bwpc - bitwidth, should be set to larger than 2
*			en_rx_irq - '1' enable rx irq; '0' disable.
*			en_tx_irq - enable tx irq; '0' disable.
*
*	@return	'1' set success; '0' set error probably bwpc smaller than 3.
*
*		BaudRate = sclk/((uartCLKdiv+1)*(bwpc+1))
*		SYCLK = 16Mhz
		115200		9			13
		9600		103			15
*
*		SYCLK = 32Mhz
*		115200		19			13
		9600		237			13
*/
extern unsigned char uart_Init(unsigned short uartCLKdiv, unsigned char bwpc, UART_ParityTypeDef Parity,UART_StopBitTypeDef StopBit);
/**
 * @brief     enable uart DMA mode,config uart dam interrupt.
 * @param[in] dmaTxIrqEn -- whether or not enable UART TX interrupt.
 * @param[in] dmaRxIrqEn -- whether or not enable UART RX interrupt.
 * @return    none
 */
extern void uart_DmaModeInit(unsigned char dmaTxIrqEn, unsigned char dmaRxIrqEn);

/**
 * @brief     config the number level setting the irq bit of status register 0x9d
 *            ie 0x9d[3].
 *            If the cnt register value(0x9c[0,3]) larger or equal than the value of 0x99[0,3]
 *            or the cnt register value(0x9c[4,7]) less or equal than the value of 0x99[4,7],
 *            it will set the irq bit of status register 0x9d, ie 0x9d[3]
 * @param[in] rx_level - receive level value. ie 0x99[0,3]
 * @param[in] tx_level - transmit level value.ie 0x99[4,7]
 * @param[in] rx_irq_en - 1:enable rx irq. 0:disable rx irq
 * @param[in] tx_irq_en - 1:enable tx irq. 0:disable tx irq
 * @return    none
 * @notice    suggust closing tx irq.
 */
extern void uart_notDmaModeInit(unsigned char rx_level,unsigned char tx_level,unsigned char rx_irq_en,unsigned char tx_irq_en);

/********
 * @ brief   in not dma mode, receive the data.
 *           the method to read data should be like this: read receive data in the order from 0x90 to 0x93.
 *           then repeat the order.
 * @ param[in] none
 * @ return    the data received from the uart.
 */
extern unsigned char uart_notDmaModeRevData(void);

/**
 * @brief     uart send data function with not DMA method.
 *            variable uart_TxIndex,it must cycle the four registers 0x90 0x91 0x92 0x93 for the design of SOC.
 *            so we need variable to remember the index.
 * @param[in] uartData - the data to be send.
 * @return    1: send success ; 0: uart busy
 */
extern unsigned char uart_notDmaModeSendByte(unsigned char uartData);

/********************************************************************************
*	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start
*			the DMA send function
*
*	@param	sendBuff - send data buffer
*
*	@return	'1' send success; '0' DMA busy
*/
extern unsigned char uart_Send(unsigned char* addr);

extern unsigned char uart_Send_kma(unsigned char* addr);
/********************************************************************
*
*	@brief	uart receive function, call this function to get the UART data
*
*	@param	userDataBuff - data buffer to store the uart data
*
*	@return	'0' rx error; 'rxLen' received data length
*/
//extern unsigned short uart_Rec(unsigned char* addr);

/******************************************************************************
*
*	@brief		get the uart IRQ source and clear the IRQ status, need to be called in the irq process function
*
*	@return		uart_irq_src- enum variable of uart IRQ source, 'UARTRXIRQ' or 'UARTTXIRQ'
*
*/
extern enum UARTIRQSOURCE uart_IRQSourceGet(void);

extern enum UARTIRQSOURCE uart_IRQSourceGet_kma(void);
/****************************************************************************************
*
*	@brief	data receive buffer initiate function. DMA would move received uart data to the address space, uart packet length
*			needs to be no larger than (recBuffLen - 4).
*
*	@param	*recAddr:	receive buffer's address info.
*			recBuffLen:	receive buffer's length, the maximum uart packet length should be smaller than (recBuffLen - 4)
*
*	@return	none
*/

extern void uart_RecBuffInit(unsigned char *recAddr, unsigned short recBuffLen);

extern void uart_BuffInit(unsigned char *recAddr, unsigned short recBuffLen, unsigned char *txAddr);

void uart_clr_tx_busy_flag(void);

void uart_set_tx_done_delay (u32 t);		//for 8266 only

unsigned char uart_tx_is_busy(void);

void uart_io_init(unsigned char uart_io_sel);


#endif
