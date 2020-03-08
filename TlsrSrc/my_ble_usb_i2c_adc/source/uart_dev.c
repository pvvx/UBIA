/*
 * uart.c
 *
 *  Created on: 23.02.2020
 *      Author: pvvx
 */
#include "proj/tl_common.h"
#if (USE_UART_DEV)
#include "proj/drivers/uart.h"
#include "uart_dev.h"

unsigned char uart_rx_irq = 0, uart_tx_irq = 0;


dev_uart_cfg_t cfg_uart;
const dev_uart_cfg_t def_cfg_uart = {
		.baud = 1152,
		.stop = 0,
		.parity = 0
};

unsigned char uart_enabled = 0;

__attribute__((aligned(4))) unsigned char uart_rx_buff[UART_TX_RX_DMA_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,}; // the first four byte is length to receive data.
__attribute__((aligned(4))) unsigned char uart_tx_buff[UART_TX_RX_DMA_BUFFER_SIZE]  = {0x00,0x00,0x00,0x00,}; // the first four byte is length to send data.

void uart_init(dev_uart_cfg_t * p) {
// uart_sclk = sclk/(uart_clk_div[14:0]+1)
// Baudrate = sclk/(uart_clk_div+1)/(bwpc+1)
// 16000000/(9+1)/(13+1) = 114285.714286
	if(p->baud < 3) return;
	unsigned int x = 0xffffffff, dv = p->baud;
	unsigned int uartCLKdiv;
	unsigned int bwpc = 15, i;

	dv = (CLOCK_SYS_CLOCK_HZ/100 + (dv/2))/dv;
	for(i = 4; i <= 16; i++) {
		uartCLKdiv = dv/i;
		if((dv - i*uartCLKdiv) < x) {
			x = dv - i*uartCLKdiv;
			bwpc = i;
		}
	}
	uartCLKdiv = (dv/bwpc) - 1;
	bwpc--;
	// CLK16M_UART115200;
	// uart_Init(9,13,PARITY_NONE,STOP_BIT_ONE); //set baud rate, parity bit and stop bit
	// uart_DmaModeInit(UART_DMA_TX_IRQ_EN, UART_DMA_RX_IRQ_EN);   // enable tx and rx interrupt
	/******************* config bautrate and timeout********************************/
	//set uart clk divider and enable clock divider
	reg_uart_clk_div = MASK_VAL(FLD_UART_CLK_DIV, uartCLKdiv, FLD_UART_CLK_DIV_EN, 1); // sclk/(uart_clk_div[14:0]+1)

	//set bit width
	reg_uart_ctrl0 = MASK_VAL(FLD_UART_BWPC, bwpc);
	//set timeout period
	reg_uart_rx_timeout = MASK_VAL(FLD_UART_TIMEOUT_BW, (bwpc+1)*12) | FLD_UART_BW_MUL2;

	/******************* config parity function*************************************/
	if(p->parity) {
		BM_SET(reg_uart_ctrl0, FLD_UART_PARITY_EN); //enable parity
		if(p->parity & 1)
			BM_CLR(reg_uart_ctrl0, FLD_UART_PARITY_SEL); //enable even parity
		else
			BM_SET(reg_uart_ctrl0, FLD_UART_PARITY_SEL); //enable odd parity
	} else {
		BM_CLR(reg_uart_ctrl0, FLD_UART_PARITY_EN);  //close parity function
	}
	/****************** config stop bit**********************************************/
	BM_CLR(reg_uart_ctrl0, FLD_UART_STOP_BIT);
	reg_uart_ctrl0 |= MASK_VAL(FLD_UART_STOP_BIT, p->stop); // 00: 1 bit, 01: 1.5bit 1x: 2bits

	// enable uart function and enable input
#if (MCU_CORE_TYPE == MCU_CORE_8269)
		// UART_GPIO_CFG_PC2_PC3(): 	gpio_set_func(GPIO_PC2, AS_UART);		gpio_set_func(GPIO_PC3, AS_UART);
		analog_write(0x0f, (analog_read(0x0f) & 0xf0) | PM_PIN_UP_DOWN_FLOAT | (PM_PIN_PULLUP_1M<<2));
		BM_CLR(reg_gpio_gpio_func(GPIO_PC2), (GPIO_PC2 | GPIO_PC3) & 0xFF);
		BM_SET(reg_gpio_config_func(GPIO_PC2), (GPIO_PC2 | GPIO_PC3) & 0xFF);
		BM_SET(reg_gpio_ie(GPIO_PC2), (GPIO_PC2 | GPIO_PC3) & 0xFF);  //enable input

#elif (CHIP_TYPE == CHIP_TYPE_8266)
		// UART_GPIO_CFG_PC6_PC7():	gpio_set_func(GPIO_PC6, AS_UART); gpio_set_func(GPIO_PC7, AS_UART);
		analog_write(0x10, (analog_read(0x10) & 0xf0) | PM_PIN_UP_DOWN_FLOAT | (PM_PIN_PULLUP_1M<<2));
		BM_CLR(reg_gpio_gpio_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF); // disable PC6/PC7 as gpio
		BM_CLR(reg_gpio_config_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF); // disable PC6/PC7 keyscan function
		BM_SET(reg_gpio_ie(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF);  // enable input
#else
		@error 'CHIP_TYPE? Not Code!'
#endif

	// uart_DmaModeInit(UART_DMA_TX_IRQ_EN, UART_DMA_RX_IRQ_EN);

    // config DMAx mode
	BM_SET(reg_dma0_ctrl, FLD_DMA_WR_MEM); //set DMA0 mode to 0x01 for receive.write to memory
	BM_CLR(reg_dma1_ctrl, FLD_DMA_WR_MEM); //set DMA1 mode to 0x00 for send. read from memory

	// config dma irq
//	BM_SET(reg_dma_chn_irq_msk, FLD_DMA_UART_RX); //enable uart rx dma interrupt
//	BM_SET(reg_irq_mask, FLD_IRQ_DMA_EN);

//	BM_SET(reg_dma_chn_irq_msk, FLD_DMA_UART_TX);  //enable uart tx dma interrupt
//	BM_SET(reg_irq_mask, FLD_IRQ_DMA_EN);

	// uart_RecBuffInit(uart_rec_buff, UART_RX_BUFF_SIZE);  //set uart rev buffer and buffer size
	reg_dma0_addr = (unsigned short)((u32)(&uart_rx_buff));//set receive buffer address
	BM_CLR(reg_dma0_ctrl, FLD_DMA_BUF_SIZE);
	reg_dma0_ctrl |= MASK_VAL(FLD_DMA_BUF_SIZE, UART_TX_RX_DMA_BUFFER_SIZE>>4);  //set receive buffer size
	// uart_txBuffInit(UART_TX_BUFF_SIZE);
	BM_CLR(reg_dma1_ctrl, FLD_DMA_BUF_SIZE);
	reg_dma1_ctrl |= MASK_VAL(FLD_DMA_BUF_SIZE, UART_TX_RX_DMA_BUFFER_SIZE>>4); //set receive buffer size

	// enable UART clk
	reg_clk_en1 |= FLD_CLK_UART_EN;

	// enable UART DMA mode
	BM_SET(reg_uart_ctrl0, FLD_UART_RX_DMA_EN | FLD_UART_TX_DMA_EN);

	uart_enabled = 1;
}

unsigned char uart_send(unsigned char* addr) {
	if(UartTxBusy()) //	if(reg_dma_tx_rdy0 & FLD_DMA_UART_TX)
		return 0;
	reg_dma1_addr = (unsigned short)((u32)addr);   //packet data, start address is sendBuff+1
	reg_dma_tx_rdy0 |= FLD_DMA_UART_TX;
	return 1;
}

void uart_deinit(void) {
	uart_enabled = 0;
#if (MCU_CORE_TYPE == MCU_CORE_8269)
	analog_write(0x10, (analog_read(0x10) & 0xf0) | PM_PIN_UP_DOWN_FLOAT | (PM_PIN_PULLUP_1M<<2));
	BM_SET(reg_gpio_gpio_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF);
	BM_SET(reg_gpio_config_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF);
#elif (CHIP_TYPE == CHIP_TYPE_8266)
	analog_write(0x10, (analog_read(0x10) & 0xf0) | PM_PIN_PULLUP_1M | (PM_PIN_PULLUP_1M<<2));
	BM_SET(reg_gpio_gpio_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF);
	BM_SET(reg_gpio_config_func(GPIO_PC6), (GPIO_PC7 | GPIO_PC6) & 0xFF);
#else
	@error 'CHIP_TYPE? Not Code!'
#endif
	BM_CLR(reg_uart_ctrl0, FLD_UART_RX_DMA_EN | FLD_UART_TX_DMA_EN);
	reg_uart_clk_div = 0x3fff;
	reg_clk_en1 &= ~FLD_CLK_UART_EN;
}

#if 0
void Uart_Irq(void) {
	unsigned char irqS = reg_dma_irq_src;
	reg_dma_irq_src = irqS; //clear irq source
	if(irqS & 0x01){
		uart_rx_irq = 1;
	} else if(irqS & 0x02){
		uart_tx_irq++;
	}
}
#endif

#endif //USE_UART_DEV
