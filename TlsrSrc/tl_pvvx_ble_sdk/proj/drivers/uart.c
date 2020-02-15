/********************************************************************************************************
 * @file     uart.c
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

//BM_CLR
#include "uart.h"
#include "../../proj/tl_common.h"

#define     STARTTX         (reg_dma_tx_rdy0 |= BIT(1))                     //trigger dma1 channel to transfer.dma1 is the uart tx channel
#define     TXDONE          ((reg_uart_status1 & FLD_UART_TX_DONE) ? 1:0)   //1:uart module has send all data.0:still has data to send
#define     RXERRORCLR      (reg_uart_status0  |= FLD_UART_RX_ERR_CLR)      //if uart module occur error,this bit can clear error flag bit.
#define     RXERROR         ((reg_uart_status0 & FLD_UART_RX_ERR_FLAG)? 1:0)//uart module error status flag bit.

#define  _FLD_UART_BW_MUL1  0        // timeout is bit_width*1
#define  _FLD_UART_BW_MUL2  1        // timeout is bit_width*2
#define  _FLD_UART_BW_MUL3  2        // timeout is bit_width*3
#define  _FLD_UART_BW_MUL4  3        // timeout is bit_width*4


#if(MCU_CORE_TYPE == MCU_CORE_8266)
	#define UART_CONTINUE_DELAY_EN          1
#else
	#define UART_CONTINUE_DELAY_EN          0
#endif

volatile unsigned char uart_tx_busy_flag = 0;                   // must "volatile"
static unsigned char   *tx_buff = NULL;

#if(UART_CONTINUE_DELAY_EN)
	static volatile unsigned int uart_continue_delay_time = 0;      // must "volatile"
	static unsigned int uart_tx_done_delay_us = 900;
#endif

void uart_set_tx_busy_flag(){
    uart_tx_busy_flag = 1;
    #if(UART_CONTINUE_DELAY_EN)
    	uart_continue_delay_time = 0;
    #endif
}

void uart_clr_tx_busy_flag(){
    #if(UART_CONTINUE_DELAY_EN)
    	uart_continue_delay_time = clock_time() | 1; // make sure not zero
    #else
    	uart_tx_busy_flag = 0;
    #endif
}

void uart_set_tx_done_delay (u32 t)
{
#if(UART_CONTINUE_DELAY_EN)
	uart_tx_done_delay_us = t;
#endif
}

unsigned char uart_tx_is_busy(){
#if(UART_CONTINUE_DELAY_EN)
   if (uart_continue_delay_time && clock_time_exceed(uart_continue_delay_time, uart_tx_done_delay_us))
   {
    	uart_continue_delay_time = 0;
    	uart_tx_busy_flag = 0;
   }
    return uart_tx_busy_flag;
#else
    return (!TXDONE);
#endif
}

/**********************************************************
*
*	@brief	reset uart module
*
*	@param	none
*
*	@return	none
*/
void uart_Reset(void){

	BM_SET(reg_rst_clk0, FLD_RST_UART);
	BM_CLR(reg_rst_clk0, FLD_RST_UART);
}
/**********************************************************
*
*	@brief	clear error state of uart rx, maybe used when application detected UART not work
*
*	@parm	none
*
*	@return	'1' RX error flag rised and cleard success; '0' RX error flag not rised
*
*/
unsigned char uart_ErrorCLR(void){
	if(RXERROR){
		RXERRORCLR;
		return 1;
	}
	return 0;
}


/*******************************************************
*
*	@brief	uart initiate, set uart clock divider, bitwidth and the uart work mode
*
*	@param	uartCLKdiv - uart clock divider
*			bwpc - bitwidth, should be set to larger than 2
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

unsigned char uart_Init(unsigned short uartCLKdiv, unsigned char bwpc, UART_ParityTypeDef Parity,UART_StopBitTypeDef StopBit)
{
	if(bwpc<3)
		return 0;
	/*******************1.config bautrate and timeout********************************/
	//set uart clk divider and enable clock divider
	reg_uart_clk_div = MASK_VAL(FLD_UART_CLK_DIV, uartCLKdiv, FLD_UART_CLK_DIV_EN, 1);

	//set bit width
	reg_uart_ctrl0   = MASK_VAL( FLD_UART_BWPC, bwpc);
	//set timeout period
	reg_uart_rx_timeout = MASK_VAL(FLD_UART_TIMEOUT_BW, (bwpc+1)*12, FLD_UART_TIMEOUT_MUL, _FLD_UART_BW_MUL2);

	/*******************2.config parity function*************************************/
	if(Parity){                                          // if need parity, config parity function
		BM_SET(reg_uart_ctrl0, FLD_UART_PARITY_EN);      //enable parity

		if(PARITY_EVEN == Parity){
			BM_CLR(reg_uart_ctrl0, FLD_UART_PARITY_SEL); //enable even parity
		}
		else if(PARITY_ODD == Parity){
			BM_SET(reg_uart_ctrl0, FLD_UART_PARITY_SEL); //enable odd parity
		}
		else{
			return 0;
		}
	}
	else{ //if not parity,close the parity function
		BM_CLR(reg_uart_ctrl0, FLD_UART_PARITY_EN);  //close parity function
	}

	/******************3.config stop bit**********************************************/
	BM_CLR(reg_uart_ctrl0, FLD_UART_STOP_BIT);
	reg_uart_ctrl0 |= MASK_VAL(FLD_UART_STOP_BIT, StopBit);

	return 1;
}

/**
 * @brief     enable uart DMA mode,config uart dam interrupt.
 * @param[in] dmaTxIrqEn -- whether or not enable UART TX interrupt.
 * @param[in] dmaRxIrqEn -- whether or not enable UART RX interrupt.
 * @return    none
 */
void uart_DmaModeInit(unsigned char dmaTxIrqEn, unsigned char dmaRxIrqEn)
{
	//1.enable UART DMA mode
	BM_SET(reg_uart_ctrl0, FLD_UART_RX_DMA_EN | FLD_UART_TX_DMA_EN);

    //2.config DMAx mode
	BM_SET(reg_dma0_ctrl, FLD_DMA_WR_MEM); //set DMA0 mode to 0x01 for receive.write to memory
	BM_CLR(reg_dma1_ctrl, FLD_DMA_WR_MEM); //set DMA1 mode to 0x00 for send. read from memory

	//3.config dma irq
	if(dmaRxIrqEn){
		BM_SET(reg_dma_chn_irq_msk, FLD_DMA_UART_RX); //enable uart rx dma interrupt
		BM_SET(reg_irq_mask, FLD_IRQ_DMA_EN);
	}else{
		BM_CLR(reg_dma_chn_irq_msk, FLD_DMA_UART_RX); //disable uart rx dma interrupt
	}

	if(dmaTxIrqEn){
		BM_SET(reg_dma_chn_irq_msk, FLD_DMA_UART_TX);  //enable uart tx dma interrupt
		BM_SET(reg_irq_mask, FLD_IRQ_DMA_EN);
	}else{
		BM_CLR(reg_dma_chn_irq_msk, FLD_DMA_UART_TX);  //disable uart tx dma interrupt
	}
}

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
void uart_notDmaModeInit(unsigned char rx_level,unsigned char tx_level,unsigned char rx_irq_en,unsigned char tx_irq_en)
{
	//1.set the trig level.
	BM_CLR(reg_uart_ctrl2, FLD_UART_CTRL3_RX_IRQ_TRIG_LEVEL);
	reg_uart_ctrl2 |= MASK_VAL(FLD_UART_CTRL3_RX_IRQ_TRIG_LEVEL, rx_level);

	BM_CLR(reg_uart_ctrl2, FLD_UART_CTRL3_TX_IRQ_TRIG_LEVEL);
	reg_uart_ctrl2 |= MASK_VAL(FLD_UART_CTRL3_TX_IRQ_TRIG_LEVEL, tx_level);

	//2.config the irq.
	if(rx_irq_en){
		BM_SET(reg_uart_ctrl0, FLD_UART_RX_IRQ_EN); //enable uart rx irq
		BM_SET(reg_irq_mask, FLD_IRQ_UART_EN);      //enable uart irq.
	}else{
		BM_CLR(reg_uart_ctrl0, FLD_UART_RX_IRQ_EN); //disable uart rx irq
	}
	if(tx_irq_en){
		BM_SET(reg_uart_ctrl0, FLD_UART_TX_IRQ_EN);  //enable uart tx irq
		BM_SET(reg_irq_mask, FLD_IRQ_UART_EN);       //enable uart irq
	}else{
		BM_CLR(reg_uart_ctrl0, FLD_UART_TX_IRQ_EN);  //disable uart tx irq
	}
}
/********
 * @ brief   in not dma mode, receive the data.
 *           the method to read data should be like this: read received data in the order from 0x90 to 0x93.
 *           then repeat the order.
 * @ param[in] none
 * @ return    the data received from the uart.
 */
unsigned char uart_notDmaModeRevData(void)
{
	static unsigned char uart_RevIndex = 0;
	unsigned char tmpRevData = 0;

	tmpRevData = read_reg8(0x90 + uart_RevIndex);
	uart_RevIndex++;
	uart_RevIndex &= 0x03;
	return tmpRevData;
}

/**
 * @brief     uart send data function with not DMA method.
 *            variable uart_TxIndex,it must loop the four registers 0x90 0x91 0x92 0x93 for the design of SOC.
 *            so we need variable to remember the index.
 * @param[in] uartData - the data to be send.
 * @return    1: send success ; 0: uart busy
 */
unsigned char uart_notDmaModeSendByte(unsigned char uartData)
{
	static unsigned char uart_TxIndex = 0;

	while((read_reg8(0x9c)>>4) > 7 );  //t_buf_cnt < 8, data send OK
	write_reg8(0x90+uart_TxIndex,uartData);
	uart_TxIndex++;
	uart_TxIndex &= 0x03;    //cycle the four register 0x90 0x91 0x92 0x93.
	return 0;
}

/********************************************************************************
*	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start
*			the DMA send function
*
*	@param	sendBuff - send data buffer
*
*	@return	'1' send success; '0' DMA busy
*/
unsigned char uart_Send(unsigned char* addr){
	if(uart_tx_is_busy()){
		return 0;
	}
	reg_dma1_addr = (unsigned short)((u32)addr);   //packet data, start address is sendBuff+1
	STARTTX;
	return 1;
}

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

void uart_RecBuffInit(unsigned char *recAddr, unsigned short recBuffLen){
	unsigned char bufLen;
	bufLen = recBuffLen>>4;
	reg_dma0_addr = (unsigned short)((u32)recAddr);//set receive buffer address

	BM_CLR(reg_dma0_ctrl, FLD_DMA_BUF_SIZE);
	reg_dma0_ctrl |= MASK_VAL(FLD_DMA_BUF_SIZE, bufLen);  //set receive buffer size
}

void uart_txBuffInit(unsigned short txBuffLen){
	unsigned char bufLen;
	bufLen = txBuffLen >> 4;

	BM_CLR(reg_dma1_ctrl, FLD_DMA_BUF_SIZE);
	reg_dma1_ctrl |= MASK_VAL(FLD_DMA_BUF_SIZE, bufLen); //set receive buffer size
}
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

void uart_BuffInit(unsigned char *recAddr, unsigned short recBuffLen, unsigned char *txAddr){
	unsigned char bufLen;
	bufLen = recBuffLen>>4;
	reg_dma0_addr = (unsigned short)((unsigned int)(recAddr)); //set receive buffer address

	BM_CLR(reg_dma0_ctrl, FLD_DMA_BUF_SIZE);
	reg_dma0_ctrl |= MASK_VAL(FLD_DMA_BUF_SIZE, bufLen); //set receive buffer size

    tx_buff = txAddr;
}

/******************************************************************************
*
*	@brief		get the uart IRQ source and clear the IRQ status, need to be called in the irq process function
*
*	@return		uart_irq_src- enum variable of uart IRQ source, 'UARTRXIRQ' or 'UARTTXIRQ'
*
*/
enum UARTIRQSOURCE uart_IRQSourceGet(void){
	unsigned char irqS;

	irqS = reg_dma_irq_src;
	reg_dma_irq_src = irqS; //clear irq source

	if(irqS & 0x01){
		return UARTRXIRQ;
	}
	else if(irqS & 0x02){
		return UARTTXIRQ;
	}
	else{
		return UARTNONEIRQ;
	}
}


/************************Application Example******************
unsigned char recBuff[128];//Declare a receive buffer
void uart_useExample(void ){
	CLK32M_UART9600;
	uart_RecBuffInit(&recBuff,128);
	//Initial IO,UART rx & tx declare
	write_reg8(0x800596,0xC3);
	write_reg8(0x8005B2,0x3C);
}
*/

enum UARTIRQSOURCE uart_IRQSourceGet_kma(void){
	unsigned char irqS;
	irqS = reg_dma_irq_src;
	reg_dma_irq_src = irqS; //clear irq source
#if(!UART_CONTINUE_DELAY_EN)
	if(irqS & 0x01)	return UARTRXIRQ;
	if(irqS & 0x02)	return UARTTXIRQ;

	return UARTRXIRQ;
#else
	return (irqS & UARTIRQ_MASK);
#endif
}

/********************************************************************************
*	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start
*			the DMA send function
*
*	@param	sendBuff - send data buffer
*
*	@return	'1' send success; '0' DMA busy
*/
unsigned char uart_Send_kma(unsigned char* addr){
    unsigned long len = *((unsigned long *)addr);

    if(len > 252){
        return 0;
    }

    if (uart_tx_is_busy ())
    {
    	return 0;
    }

    uart_set_tx_busy_flag();

    reg_dma1_addr = (u16)(u32)addr;  //packet data, start address is sendBuff+1

	STARTTX;

	return 1;
}

//////////////////////////////////////////////////////////////////////
void uart_io_init(unsigned char uart_io_sel){
#if(MCU_CORE_TYPE == MCU_CORE_8266)
	uart_io_sel = uart_io_sel;
	gpio_set_func(GPIO_UTX, AS_UART);
	gpio_set_func(GPIO_URX, AS_UART);
#else
    if(UART_GPIO_8267_PA6_PA7 == uart_io_sel){
        UART_GPIO_CFG_PA6_PA7();
    }else if(UART_GPIO_8267_PC2_PC3 == uart_io_sel){
        UART_GPIO_CFG_PC2_PC3();
    }else if(UART_GPIO_8267_PB2_PB3 == uart_io_sel){
        UART_GPIO_CFG_PB2_PB3();
    }
#endif
}

#if(__TL_LIB_8267__ || (MCU_CORE_TYPE == MCU_CORE_8267) || \
	__TL_LIB_8261__ || (MCU_CORE_TYPE == MCU_CORE_8261) || \
	__TL_LIB_8269__ || (MCU_CORE_TYPE == MCU_CORE_8269))
/**
 * @brief UART hardware flow control configuration. Configure RTS pin.
 * @param[in]   enable: enable or disable RTS function.
 * @param[in]   mode: set the mode of RTS(auto or manual).
 * @param[in]   thrsh: threshold of trig RTS pin's level toggle(only for auto mode),
 *                     it means the number of bytes that has arrived in Rx buf.
 * @param[in]   invert: whether invert the output of RTS pin(only for auto mode)
 * @return none
 */
void uart_RTSCfg(unsigned char enable, unsigned char mode, unsigned char thrsh, unsigned char invert)
{
    if (enable) {
    	gpio_set_func(GPIO_PC4, AS_UART);   //disable GPIOC_GP4 Pin's GPIO function
        BM_SET(reg_gpio_config_func2, FLD_UART_RTS_PWM4);// enable GPIOC_GP4 Pin as RTS Pin.BIT4:FLD_UART_RTS_PWM4
        BM_SET(reg_uart_ctrl2, FLD_UART_CTRL2_RTS_EN);   //enable RTS function
    }
    else {
    	gpio_set_func(GPIO_PC4, AS_GPIO);  //enable GPIOC_GP4 Pin as GPIO function.
    	BM_CLR(reg_uart_ctrl2, FLD_UART_CTRL2_RTS_EN); //disable RTS function
    }

    if (mode) {
    	BM_SET(reg_uart_ctrl2, FLD_UART_CTRL2_RTS_MANUAL_EN); //enable manual mode
    }
    else {
    	BM_CLR(reg_uart_ctrl2, FLD_UART_CTRL2_RTS_MANUAL_EN); //enable auto mode
    }

    if (invert) {
    	BM_SET(reg_uart_ctrl2, FLD_UART_CTRL2_RTS_PARITY);    //invert RTS parity
    }
    else {
    	BM_CLR(reg_uart_ctrl2, FLD_UART_CTRL2_RTS_PARITY);
    }

    //set threshold
    BM_CLR(reg_uart_ctrl2, FLD_UART_CTRL2_RTS_TRIG_LVL);
    reg_uart_ctrl2 |= MASK_VAL(FLD_UART_CTRL2_RTS_TRIG_LVL, (thrsh&0xff));
}

/**
 * @brief This function sets the RTS pin's level manually
 * @param[in]   polarity: set the output of RTS pin(only for manual mode)
 * @return none
 */
void uart_RTSLvlSet(unsigned char polarity)
{
    if (polarity) {
    	BM_SET(reg_uart_ctrl2, FLD_UART_CTRL2_RTS_MANUAL_VAL);
    }
    else {
    	BM_CLR(reg_uart_ctrl2, FLD_UART_CTRL2_RTS_MANUAL_VAL);
    }
}

/**
 * @brief UART hardware flow control configuration. Configure CTS pin.
 * @param[in]   enable: enable or disable CTS function.
 * @param[in]   select: when CTS's input equals to select, tx will be stopped
 * @return none
 */
void uart_CTSCfg(unsigned char enable, unsigned char select)
{
    if (enable) {
    	gpio_set_func(GPIO_PC5, AS_UART);                 //disable GPIOC_GP5 Pin's GPIO function
    	BM_SET(reg_gpio_config_func2, FLD_UART_CTS_PWM5); //enable GPIOC_GP5 Pin as CTS Pin.BIT5:FLD_UART_CTS_PWM5
    	BM_SET(reg_uart_ctrl0, FLD_UART_CTS_EN);          //enable CTS function
    }
    else {
    	gpio_set_func(GPIO_PC5, AS_GPIO);        //enable GPIO_GP5 Pin's GPIO function
    	BM_CLR(reg_uart_ctrl0, FLD_UART_CTS_EN); //disable CTS function
    }

    if (select) {
    	BM_SET(reg_uart_ctrl0, FLD_UART_CTS_I_SELECT);
    }
    else {
    	BM_CLR(reg_uart_ctrl0, FLD_UART_CTS_I_SELECT); //invert CTS
    }
}
#endif
