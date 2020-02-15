/********************************************************************************************************
 * @file     spi.c
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

#include "../tl_common.h"
#include "spi.h"

#define SPI_BUSY_FLAG     ((reg_spi_ctrl & FLD_SPI_BUSY)?1:0)

/****
* @brief: spi bus can drive more than one spi slave. so we can use cs line to select spi slave that response master.
*         the telink's chip can use normal gpio to as cs function, not only the CN pin of spi hardware module.
*         but if device act as spi slave,the cs pin must use the CN pin of spi hardware module.
*/
#if(MCU_CORE_TYPE == MCU_CORE_8266)
void spi_master_pin_init(unsigned int cs_pin)
#elif((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
void spi_master_pin_init(enum spi_pin_t data_clk_pin, unsigned int cs_pin)
#endif
{
	#if(MCU_CORE_TYPE == MCU_CORE_8266)
		//disable the other function and the gpio will be spi.
		BM_CLR(reg_gpio_config_func4, (GPIO_PE7)&0xff);   //disable E6/E7 keyscan function   //(GPIO_PE6|GPIO_PE7)
		BM_CLR(reg_gpio_config_func6, BIT(5));            //disable E6/F0 as uart function
		gpio_set_func(GPIO_PE7, AS_SPI);         		  //disable E7 as gpio
		gpio_set_func(GPIO_PF0, AS_SPI);         		  //disable F0/F1 as gpio
		gpio_set_func(GPIO_PF1, AS_SPI);
		gpio_set_input_en(GPIO_PE7, 1);                   //enable input
		gpio_set_input_en(GPIO_PF0, 1);                   //enable input
		gpio_set_input_en(GPIO_PF1, 1);			          //enable input
	#elif((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
		if(data_clk_pin == SPI_PIN_GROUPB)
		{
			gpio_set_func(GPIO_PB5,	AS_SPI); ////enable B4/B5/B6 spi function and disable gpio function
			gpio_set_func(GPIO_PB6,	AS_SPI);
			gpio_set_func(GPIO_PB7,	AS_SPI);
			BM_CLR(reg_gpio_config_func0, FLD_SPI_DO_PWM0_N|FLD_SPI_DI_PWM1|FLD_SPI_CK_PWM1_N|FLD_SPI_CN_PWM2_N); //disable A2/A3/A4/A5 as spi function
		}
		else if(data_clk_pin == SPI_PIN_GROUPA)
		{
			gpio_set_func(GPIO_PA2, AS_SPI); //enable spi function and disable gpio function
			gpio_set_func(GPIO_PA3, AS_SPI);
			gpio_set_func(GPIO_PA4, AS_SPI);
			BM_CLR(reg_gpio_config_func1, FLD_SPI_CN_PWM4|FLD_SPI_DO_PWM4N|FLD_SPI_DI_PWM5|FLD_SPI_CK_PWM5N);//disable B4/B5/B6/B7 as spi function.

			gpio_set_func(GPIO_PB4, AS_GPIO); //enable B4/B5/B6/B7 gpio function,or they will be pwm function
			gpio_set_func(GPIO_PB5, AS_GPIO);
			gpio_set_func(GPIO_PB6, AS_GPIO);
			gpio_set_func(GPIO_PB7, AS_GPIO);
		}
	#endif

	BM_SET(reg_spi_sp, FLD_SPI_ENABLE);  //enable spi function. because i2c and spi share part of the hardware in the chip.

	gpio_set_func(cs_pin, AS_GPIO);      //cs pin as gpio function
	gpio_set_input_en(cs_pin,0);  //disable input
	gpio_write(cs_pin,1);         // output high level in idle status.
	gpio_set_output_en(cs_pin,1); //enable output
}
/**
 * @brief     This function configures the clock and working mode for SPI interface
 * @param[in] div_clk - the division factor for SPI module
 *            SPI clock = System clock / ((div_clk+1)*2); notice system clock should be at least 5x faster than spi clock.
 * @param[in] spi_mode - the selected working mode of SPI module. refer to datasheet for spi mode
 * @return    none
 */
void spi_master_init(unsigned char div_clk, enum spi_mode_t spi_mode)
{
	/***set the spi clock. spi_clk = system_clock/((div_clk+1)*2)***/
	BM_CLR(reg_spi_sp, FLD_MASTER_SPI_CLK);  //clear the spi clock division bits
	reg_spi_sp |= MASK_VAL(FLD_MASTER_SPI_CLK, div_clk&0x7f); //set the clock div bits

	BM_SET(reg_spi_ctrl, FLD_SPI_MASTER_MODE_EN);  //enable spi master mode

	/***config the spi woking mode.For spi mode spec, pls refer to datasheet***/
	BM_CLR(reg_spi_inv_clk, FLD_INVERT_SPI_CLK|FLD_DAT_DLY_HALF_CLK);//clear the mode bits

	BM_SET(reg_spi_inv_clk, spi_mode&0x03);  //set the mode
}

/*****
*   @brief: when chip act as spi slave, the cs pin must be the pin of hardware module. i.e. 8266's E6 and 8267's A5/B4
*           spi slave get the interrupt from cs pin. so can not instead of the cs pin using the other normal gpio.
*/
#if(MCU_CORE_TYPE == MCU_CORE_8266)
void spi_slave_init(enum spi_mode_t spi_mode)
#elif((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
void spi_slave_init(enum spi_pin_t spi_grp, enum spi_mode_t spi_mode)
#endif
{
	/***config the corresponding gpio as spi and enable or disable other function***/
	#if(MCU_CORE_TYPE == MCU_CORE_8266)
		//disable the other function and the gpio will be spi.
		BM_CLR(reg_gpio_config_func4, (GPIO_PE6|GPIO_PE7)&0xff);  //disable E6/E7 keyscan function
		BM_CLR(reg_gpio_config_func6, BIT(5));                    //disable E6/F0 as uart function
		gpio_set_func(GPIO_PE7, AS_SPI);                          //disable E7 as gpio
		gpio_set_func(GPIO_PF0, AS_SPI);                 		  //disable F0/F1 as gpio
		gpio_set_func(GPIO_PF1, AS_SPI);
		gpio_set_input_en(GPIO_PE6, 1);                  //enable input
		gpio_set_input_en(GPIO_PE7, 1);
		gpio_set_input_en(GPIO_PF0, 1);                  //enable input
		gpio_set_input_en(GPIO_PF1, 1);

	#elif((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
		if(spi_grp == SPI_PIN_GROUPB){
			gpio_set_func(GPIO_PB4,	AS_SPI); //enable B4/B5/B6 spi function and disable gpio function
			gpio_set_func(GPIO_PB5,	AS_SPI);
			gpio_set_func(GPIO_PB6,	AS_SPI);
			gpio_set_func(GPIO_PB7,	AS_SPI);

			BM_CLR(reg_gpio_config_func0, (GPIO_PA2|GPIO_PA3|GPIO_PA4|GPIO_PA5)&0xff);  //disable A2/A3/A4/A5 as spi function
		}
		else if(spi_grp == SPI_PIN_GROUPA){
			gpio_set_func(GPIO_PA2, AS_SPI); ////enable spi function and disable gpio function
			gpio_set_func(GPIO_PA3, AS_SPI);
			gpio_set_func(GPIO_PA4,	AS_SPI);
			gpio_set_func(GPIO_PA5,	AS_SPI);

			BM_CLR(reg_gpio_config_func1, (GPIO_PB4|GPIO_PB5|GPIO_PB6|GPIO_PB7)&0xff);  //disable B4/B5/B6 as spi function.

			gpio_set_func(GPIO_PB4,	AS_GPIO); //enable B4/B5/B6/B7 gpio function,or they will be pwm function
			gpio_set_func(GPIO_PB5,	AS_GPIO);
			gpio_set_func(GPIO_PB6,	AS_GPIO);
			gpio_set_func(GPIO_PB7,	AS_GPIO);
		}
	#endif
	/***enable slave***/
	BM_CLR(reg_spi_ctrl, FLD_SPI_MASTER_MODE_EN);  //disable spi master mode, .i.e enable spi slave mode

	BM_SET(reg_spi_sp, FLD_SPI_ENABLE); //enalbe spi function
	/***config the spi woking mode.For spi mode spec, pls refer to datasheet***/
	BM_CLR(reg_spi_inv_clk, FLD_INVERT_SPI_CLK|FLD_DAT_DLY_HALF_CLK); //clear the mode bits
	BM_SET(reg_spi_inv_clk, spi_mode);                         //set the mode
}

/**
 * @brief      This function serves to write a bulk of data to the SPI slave
 *             device specified by the CS pin
 * @param[in]  addr_cmd - pointer to the command bytes needed written into the
 *             slave device first before the writing operation of actual data
 * @param[in]  cmd_len - length in byte of the command bytes
 * @param[in]  pbuf - pointer to the data need to write
 * @param[in]  buf_len - length in byte of the data need to write
 * @param[in]  cs_pin - the CS pin specifing the slave device
 * @return     none
 * @ timing chart: addr(0or1) + cmd_0x00 + pbuf[0]+ ... + pbuf[buf_len]
 * for example : addr_cmd[3] = {0x80,0x00,0x00} or addr_cmd[3]={0x80,0x00,0x80}
 *               the first two bytes indicate slave address. the third bytes is the command that 0x00 indicate write and 0x80 read.
 */
void spi_write(unsigned char* addr_cmd, unsigned char addr_cmd_len, unsigned char* pbuf, int buf_len, unsigned int cs_pin)
{
	int i = 0;
	/***pull down cs line and enable write***/
	gpio_write(cs_pin, 0); //pull down cs line and select the slave to handle.
	BM_CLR(reg_spi_ctrl, FLD_SPI_DATA_OUT_DIS); //enable output
	BM_CLR(reg_spi_ctrl, FLD_SPI_RD);           //enable write
	/***write cmd,refer to datasheet, cmd_0x00 is write; cmd_0x80 is read***/
	for(i=0;i<addr_cmd_len;i++){
		reg_spi_data = addr_cmd[i];
		while(SPI_BUSY_FLAG);
	}
	/***write data to slave****/
	for(i=0;i<buf_len;i++){
		reg_spi_data = pbuf[i];
		while(SPI_BUSY_FLAG); //wait data sending
	}
	/***pull up cs line to release the slave***/
	gpio_write(cs_pin, 1);
}

/**
 * @brief      This function serves to read a bulk of data from the SPI slave
 *             device specified by the CS pin
 * @param[in]  addr_cmd - pointer to the command bytes needed written into the
 *             slave device first before the reading operation of actual data
 * @param[in]  addr_cmd_len - length in byte of the command bytes
 * @param[out] pbuf - pointer to the buffer that will cache the reading out data
 * @param[in]  buf_len - length in byte of the data need to read
 * @param[in]  cs_pin - the CS pin specifing the slave device
 * @return     none
 */
void spi_read(unsigned char* addr_cmd, unsigned char addr_cmd_len, unsigned char* pbuf, int buf_len, unsigned int cs_pin)
{
	int i = 0;
	unsigned char temp_spi_data = 0;
	/***pull down cs line and enable write***/
	gpio_write(cs_pin, 0); //pull down cs line and select the slave to handle.
	BM_CLR(reg_spi_ctrl, FLD_SPI_DATA_OUT_DIS); //enable output
	BM_CLR(reg_spi_ctrl, FLD_SPI_RD);           //enable write

	/***write cmd,refer to datasheet, cmd_0x00 is write; cmd_0x80 is read***/
	for(i=0;i<addr_cmd_len;i++){
		reg_spi_data = addr_cmd[i];
		while(SPI_BUSY_FLAG);
	}

	reg_spi_ctrl |= MASK_VAL(FLD_SPI_RD, 1, FLD_SPI_DATA_OUT_DIS, 1);  //enable read and disable output
	temp_spi_data = reg_spi_data;
	while(SPI_BUSY_FLAG);

	/***read the data.when read register reg_sip_data(0x08),the scl will generate 8 clock cycles to get the data from slave***/
	for(i=0;i<buf_len;i++){
		pbuf[i] = reg_spi_data;
		while(SPI_BUSY_FLAG);
	}
	/***pull up cs line to release the slave***/
	gpio_write(cs_pin, 1);

}

