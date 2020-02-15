/********************************************************************************************************
 * @file     i2c.c
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
#include "../mcu/clock.h"
#include "i2c.h"

#define I2C_CMD_BUSY_FLAG		((reg_i2c_status & FLD_I2C_CMD_BUSY)? 1:0)

#define I2C_MODULE_RESET()        do{\
									BM_SET(reg_rst_clk0,FLD_RST_I2C);\
									BM_CLR(reg_rst_clk0,FLD_RST_I2C);\
								  }while(0)

/*****
 * brief  judge what the irq source is. host write or host read.
 */
I2C_I2CIrqSrcTypeDef I2C_SlaveIrqGet(void){
	unsigned char hostStatus = reg_i2c_irq_status;
	if(hostStatus & FLD_I2C_STATUS_RD){
		return I2C_IRQ_HOST_READ_ONLY;
	}
	else if(hostStatus & FLD_I2C_STATUS_WR){
		/*the bit actually indicate read and write,but because the "return read_only"is before "read_write",so if return"read_write" indicate write only*/
		return I2C_IRQ_HOST_WRITE_ONLY;
	}
	else{
		return I2C_IRQ_NONE;
	}
}
/****
 * brief  clear the irq status bit.
 */
void I2C_SlaveIrqClr(I2C_I2CIrqSrcTypeDef src){
	if(src==I2C_IRQ_HOST_READ_ONLY){
		BM_SET(reg_i2c_clr_status, FLD_I2C_STATUS_WR|FLD_I2C_STATUS_RD);
	}
	else if(src==I2C_IRQ_HOST_WRITE_ONLY){
		BM_SET(reg_i2c_clr_status, FLD_I2C_STATUS_WR);
	}
	else{
	}
}

/*****
 * brief: the function initial the relevant gpio as i2c.
 *        when enable one group of pins as i2c, the other two groups of pin should be disable the i2c function.
 *        in addition, when disable pin as i2c, we can enable the pin as gpio.
 * param[in] gpio_sda -- the pin as data line of i2c.
 * param[in] gpio_scl -- the pin as clock line of i2c.
 * return none
 */
void i2c_pin_init(I2C_GPIO_GroupTypeDef i2c_pin_group){
	u32 gpio_sda;
	u32 gpio_scl;
#if (MCU_CORE_TYPE == MCU_CORE_8266)
	gpio_sda = GPIO_PE7;
	gpio_scl = GPIO_PF1;

	gpio_set_func(gpio_sda,AS_I2C);  //disable gpio function
	gpio_set_func(gpio_scl,AS_I2C);  //disable gpio function

	gpio_setup_up_down_resistor(gpio_sda, PM_PIN_PULLUP_10K);
	gpio_setup_up_down_resistor(gpio_scl, PM_PIN_PULLUP_10K);

	gpio_set_input_en(gpio_sda,1);
    gpio_set_input_en(gpio_scl,1);

#elif ((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
	switch(i2c_pin_group){
	case I2C_GPIO_GROUP_A3A4:
		gpio_sda = GPIO_PA3;
		gpio_scl = GPIO_PA4;

		if(BM_IS_SET(reg_gpio_config_func1, (GPIO_PB6|GPIO_PB7)&0xff)){
			BM_CLR(reg_gpio_config_func1, (GPIO_PB6|GPIO_PB7)&0xff);     //disable B6/B7 as i2c function
			gpio_set_func(GPIO_PB6, AS_GPIO);                   		 //enable B6/B7 as gpio function
			gpio_set_func(GPIO_PB7, AS_GPIO);
		}
		if(BM_IS_SET(reg_gpio_config_func2, (GPIO_PC0|GPIO_PC1)&0xff)){
			BM_CLR(reg_gpio_config_func2, (GPIO_PC0|GPIO_PC1)&0xff);    //disable C0/C1 as i2c function
			gpio_set_func(GPIO_PC0, AS_GPIO);                  			//enable C0/C1 as gpio function
			gpio_set_func(GPIO_PC1, AS_GPIO);
		}

		gpio_setup_up_down_resistor(gpio_sda, PM_PIN_PULLUP_10K);       //10k pull_up resistor
		gpio_setup_up_down_resistor(gpio_scl, PM_PIN_PULLUP_10K);       //10k pull_up resistor
		break;
	case I2C_GPIO_GROUP_B6B7:
		gpio_sda = GPIO_PB6;
		gpio_scl = GPIO_PB7;

		if(BM_IS_SET(reg_gpio_config_func0, (GPIO_PA3|GPIO_PA4)&0xff)){
			BM_CLR(reg_gpio_config_func0, (GPIO_PA3|GPIO_PA4)&0xff);     // disable A3/A4 as i2c function
			gpio_set_func(GPIO_PA3,	AS_GPIO);                    		 // enable A3/A4 as gpio function
			gpio_set_func(GPIO_PA4,	AS_GPIO);
		}
		if(BM_IS_SET(reg_gpio_config_func2, (GPIO_PC0|GPIO_PC1)&0xff)){
			BM_CLR(reg_gpio_config_func2, (GPIO_PC0|GPIO_PC1)&0xff);    //disable C0/C1 as i2c function
			gpio_set_func(GPIO_PC0,	AS_GPIO);                   		//enable C0/C1 as gpio function
			gpio_set_func(GPIO_PC1,	AS_GPIO);
		}

		gpio_setup_up_down_resistor(gpio_sda, PM_PIN_PULLUP_10K);       //10k pull_up resistor
		gpio_setup_up_down_resistor(gpio_scl, PM_PIN_PULLUP_10K);       //10k pull_up resistor
		break;
//	case I2C_GPIO_GROUP_C0C1:
	default:
		gpio_sda = GPIO_PC0;
		gpio_scl = GPIO_PC1;

		if(BM_IS_SET(reg_gpio_config_func0, (GPIO_PA3|GPIO_PA4)&0xff)){
			BM_CLR(reg_gpio_config_func0, (GPIO_PA3|GPIO_PA4)&0xff);    //disable A3/A4 as i2c function
			gpio_set_func(GPIO_PA3,	AS_GPIO);                   		// enable A3/A4 as gpio function
			gpio_set_func(GPIO_PA4,	AS_GPIO);
		}
		if(BM_IS_SET(reg_gpio_config_func1, (GPIO_PB6|GPIO_PB7)&0xff)){
			BM_CLR(reg_gpio_config_func1, (GPIO_PB6|GPIO_PB7)&0xff);    // disable B6/B7 as i2c function
			gpio_set_func(GPIO_PB6,	AS_GPIO);                   		// enable B6/B7 as gpio function
			gpio_set_func(GPIO_PB7,	AS_GPIO);
		}

		gpio_setup_up_down_resistor(gpio_sda, PM_PIN_PULLUP_10K);       //10k pull_up resistor
		gpio_setup_up_down_resistor(gpio_scl, PM_PIN_PULLUP_10K);       //10k pull_up resistor
		break;
	}

	gpio_set_func(gpio_sda,AS_I2C);  //disable gpio function
	gpio_set_func(gpio_scl,AS_I2C);  //disable gpio function
#endif

}

/**
 * @brief      This function set the id of slave device and the speed of I2C interface
 *             note: the param ID contain the bit of writting or reading.
 *             eg:the parameter 0x5C. the reading will be 0x5D and writting 0x5C.
 * @param[in]  slave_id - the id of slave device. it don't contain write or read bit.
 * @param[in]  div_clock - the division factor of I2C clock, 
 *             I2C clock = System clock / (4*div_clock);if the datasheet you look at is 2*,pls modify it.
 * @return     none
 */
void i2c_master_init_div(unsigned char slave_id, unsigned char div_clock)
{
	reg_i2c_speed = div_clock;                 //configure the i2c's clock

	reg_i2c_id = MASK_VAL(FLD_I2C_ID,slave_id);//set the id of i2c module.

	BM_SET(reg_i2c_mode,FLD_I2C_MODE_MASTER|FLD_I2C_HOLD_MASTER);  //enable master mode.
//	BM_SET(reg_i2c_mode,FLD_I2C_MODE_MASTER);

	BM_SET(reg_rst_clk0,FLD_CLK_I2C_EN);       //enable i2c clock

	BM_CLR(reg_spi_sp,FLD_SPI_ENABLE);         //force PADs act as I2C; i2c and spi share the hardware of IC
}

/**
 * @brief      This function set the id of slave device and the speed of I2C interface
*
 * @param[in]  slave_id - the id of slave device.it don't contains write or read bit,the lsb is write or read bit.
 * @param[in]  i2c_speed is in Khz. for example: i2c_speed is 200, indicate 200k          
 * @return     none
 */
void i2c_master_init_khz(unsigned char slave_id, unsigned int i2c_speed)
{
	reg_i2c_speed = (CLOCK_SYS_CLOCK_1MS/(4*i2c_speed)); //set i2c clock

	reg_i2c_id = MASK_VAL(FLD_I2C_ID,slave_id);//set the id of i2c module.

	BM_SET(reg_i2c_mode,FLD_I2C_MODE_MASTER);  //enable master mode.

	BM_SET(reg_rst_clk0,FLD_CLK_I2C_EN);       //enable i2c clock

	BM_CLR(reg_spi_sp,FLD_SPI_ENABLE);         //force PADs act as I2C; i2c and spi share the hardware of IC
}
/**
 *  @brief      the function config the ID of slave and mode of slave.
 *  @param[in]  device_id - it don't contains write or read bit,the lsb is write or read bit.
 *  @param[in]  i2c_mode - set slave mode. slave has two modes, one is DMA mode, the other is MAPPING mode.
 *  @param[in]  pbuf - if slave mode is MAPPING, set the first address of buffer master write or read slave.
 *              notice: the buffer must align 128 bytes. the write address is pbuf while the read address is pbuf+64.
 *  @return     none
 */
void i2c_slave_init(unsigned char device_id,enum I2C_SLAVE_MODE i2c_mode,unsigned char* pbuf)
{
	reg_i2c_id = MASK_VAL(FLD_I2C_ID,device_id); //configure the id of i2c module.
	if(i2c_mode == I2C_SLAVE_MAP){
		reg_i2c_mode = MASK_VAL(FLD_I2C_ADDR_AUTO, 1, FLD_I2C_MEM_MAP, 1); //enable i2c address auto increase and enable mapping mode.
		reg_i2c_mem_map = (unsigned short)((u32)pbuf);
	}

	BM_CLR(reg_i2c_mode, FLD_I2C_MODE_MASTER); //disable master mode .i.e enable slave mode.

	BM_CLR(reg_spi_sp,FLD_SPI_ENABLE);        //force PADs act as I2C; i2c and spi share the hardware of IC
}

/**
 * @brief      This function writes one byte to the slave device at the specified address
 * @param[in]  addr - pointer to the address where the one byte data will be written
 * @param[in]  addr_len - length in byte of the address, which makes this function is  
 *             compatible for slave device with both one-byte address and two-byte address
 * @param[in]  data - the one byte data will be written via I2C interface
 * @return     none
 * @ timing chart : start + ID(w) + addr(1or2) + data + stop
 */
void i2c_write_byte(unsigned char* addr, int addr_len, unsigned char data)
{
	BM_CLR(reg_i2c_id, FLD_I2C_WRITE_READ_BIT); //ID|rw_bit; if rw_bit=1,read data; if rw_bit=0,write data

	if(addr_len == 1){
		reg_i2c_adr = addr[0];
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1, FLD_I2C_CMD_ADR, 1); //send start, ID, addr
	}
	else if(addr_len == 2){
		reg_i2c_adr = addr[1];
		reg_i2c_do  = addr[0];
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1, FLD_I2C_CMD_ADR, 1, FLD_I2C_CMD_DO, 1);//send start,ID,addrL,addH
	}
	else{
		while(1);
	}
	while(I2C_CMD_BUSY_FLAG);

	reg_i2c_di = data;
	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_DI, 1);   //send data
	while(I2C_CMD_BUSY_FLAG);
	
	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_STOP, 1); //send stop bit
	while(I2C_CMD_BUSY_FLAG);
}
/**
 * @brief      This function reads one byte from the slave device at the specified address
 * @param[in]  Addr - pointer to the address where the one byte data will be read
 * @param[in]  AddrLen - length in byte of the address, which makes this function is  
 *             compatible for slave device with both one-byte address and two-byte address
 * @return     the one byte data read from the slave device via I2C interface
 * @timing chart: start+ID(w)+addr(1or2)+ restart + ID(r) + data + stop
 */
unsigned char i2c_read_byte(unsigned char* addr, unsigned char addr_len)
{
	unsigned char read_data = 0;
	/***start+ID(w)+addr(1or2bytes)***/
	BM_CLR(reg_i2c_id, FLD_I2C_WRITE_READ_BIT);  //ID|rw_bit; if rw_bit=1,read data; if rw_bit=0,write data

	if(addr_len == 1){
		reg_i2c_adr = addr[0];
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1, FLD_I2C_CMD_ADR, 1);
	}
	else if(addr_len == 2){
		reg_i2c_adr = addr[1];
		reg_i2c_do  = addr[0];
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1, FLD_I2C_CMD_ADR, 1, FLD_I2C_CMD_DO, 1);
	}
	else{
		while(1);
	}
	while(I2C_CMD_BUSY_FLAG);
	
	/***restart+ID(r)***/
	BM_SET(reg_i2c_id,FLD_I2C_WRITE_READ_BIT); //ID|rw_bit; if rw_bit=1,read data; if rw_bit=0,write data
	
	/***simulate restart. when chip don't send stop bit, chip will not occur start bit. so reset module to occur restart***/
	I2C_MODULE_RESET();    //reset i2c module and occur "restart"

	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1);
	while(I2C_CMD_BUSY_FLAG);

	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_DI, 1, FLD_I2C_CMD_READ_ID, 1, FLD_I2C_CMD_NAK, 1);//enable read and send 8 clock
	while(I2C_CMD_BUSY_FLAG);
	read_data = reg_i2c_di;  //get the data
	/***stop bit***/
	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_STOP, 1);  //send stop bit
	while(I2C_CMD_BUSY_FLAG);

	return read_data;
}
/**
 * @brief      This function writes a bulk of data to the slave device at the specified address
 * @param[in]  addr - pointer to the address where the data will be written
 * @param[in]  addr_len - length in byte of the address, which makes this function is  
 *             compatible for slave device with both one-byte address and two-byte address
 * @param[in]  pbuf - pointer to the data will be written via I2C interface
 * @param[in]  len - length in byte of the data will be written via I2C interface
 * @return     none
 */
void i2c_burst_write(unsigned char *addr, int addr_len, unsigned char * pbuf, int len)
{
    int i = 0;
    unsigned short tmp_addr = 0;

    memcpy(&tmp_addr, addr, addr_len);

    for (i = 0; i < len; i++) {
        i2c_write_byte((u8*)&tmp_addr, addr_len, pbuf[i]);
        tmp_addr++;
    }
}
/**
 * @brief      This function reads a bulk of data from the slave device at the specified address
 * @param[in]  addr - pointer to the address where the data will be read
 * @param[in]  addr_len - length in byte of the address, which makes this function is  
 *             compatible for slave device with both one-byte address and two-byte address
 * @param[out] pbuf - pointer to the buffer will cache the data read via I2C interface
 * @param[in]  len - length in byte of the data will be read via I2C interface
 * @return     none
 */
void i2c_burst_read(unsigned char* addr, int addr_len, unsigned char * pbuf, int len)
{
    int i = 0;
    unsigned short tmp_addr = 0;

    memcpy(&tmp_addr, addr, addr_len);

    for (i = 0; i < len; i++) {
        pbuf[i] = i2c_read_byte((u8*)&tmp_addr, addr_len);
        tmp_addr++;
    }
}

/**
 *  @brief      the write format in dma mode telink design. pls refer to datasheet.
 *  @param[in]  addr - the register that master write data to slave in. support one byte and two bytes. i.e param2 AddrLen may be 1 or 2.
 *  @param[in]  addr_len - the length of register. enum 1 or 2. based on the spec of i2c slave.
 *  @param[in]  pbuf - the first SRAM buffer address to write data to slave in.
 *  @param[in]  len - the length of data master write to slave.
 *  @return     none
 *  @ timing chart: start + ID(w) + addr(1B or 2B) + pbuf[0] +...+ pbuf[len] + stop
 */
void i2c_write_dma(unsigned short addr, unsigned char addr_len, unsigned char* pbuf, int len)
{
	int idx = 0;	
	/***start + ID(w) + addr(1or2)***/
	BM_CLR(reg_i2c_id, FLD_I2C_WRITE_READ_BIT);  //ID|rw_bit; if rw_bit=1,read data; if rw_bit=0,write data

	if(addr_len == 1){
		reg_i2c_adr = (unsigned char)addr;
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1, FLD_I2C_CMD_ADR, 1);
	}
	else if(addr_len == 2){
		reg_i2c_adr = (unsigned char)(addr>>8);
		reg_i2c_do  = (unsigned char)addr;
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1, FLD_I2C_CMD_ADR, 1, FLD_I2C_CMD_DO, 1);
	}
	else{
		while(1);
	}
	while(I2C_CMD_BUSY_FLAG);

	/***write data to slave***/
	for(idx=0;idx<len;idx++){
		reg_i2c_di = pbuf[idx];
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_DI, 1);
		while(I2C_CMD_BUSY_FLAG);
	}
	/***stop bit***/
	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_STOP , 1);
	while(I2C_CMD_BUSY_FLAG);
 }
/**
 * @brief      the read format in dma mode telink design. pls refer to datasheet.
 * @param[in]  addr - the register master read data from slave in. support one byte and two bytes.
 * @param[in]  addr_len - the length of register. two data: 1 or 2.
 * @param[in]  pbuf - the first address of SRAM buffer master store data in.
 * @param[in]  len - the length of data master read from slave.
 * @return     none.
 * @timing chart: start + ID(w) + addr(1B or 2B) + stop + start + ID(r) + pbuf[0] + ...+ pbuf[len] + stop
 */
void i2c_read_dma(unsigned short addr, unsigned char addr_len, unsigned char* pbuf, int len)
{
	int idx = 0;
	//start + ID(w) + addr(1or2) + stop
	BM_CLR(reg_i2c_id, FLD_I2C_WRITE_READ_BIT);//ID|rw_bit; if rw_bit=1,read data; if rw_bit=0,write data
	if(addr_len == 1){
		reg_i2c_adr = (unsigned char)addr;
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1, FLD_I2C_CMD_ADR, 1,FLD_I2C_CMD_STOP, 1);
	}
	else if(addr_len == 2){
		reg_i2c_adr = (unsigned char)(addr>>8);
		reg_i2c_do  = (unsigned char)addr;
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1, FLD_I2C_CMD_ADR, 1, FLD_I2C_CMD_DO, 1, FLD_I2C_CMD_STOP, 1);
	}
	else{
		while(1);
	}
	while(I2C_CMD_BUSY_FLAG);
	/*** start + ID(r)***/
	BM_SET(reg_i2c_id, FLD_I2C_WRITE_READ_BIT);  //ID|rw_bit; if rw_bit=1,read data; if rw_bit=0,write data
	reg_i2c_ctrl =  MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1);
	while(I2C_CMD_BUSY_FLAG);
	
	len--;
	while(len){
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_DI, 1, FLD_I2C_CMD_READ_ID, 1);
		while(I2C_CMD_BUSY_FLAG);
		pbuf[idx] = reg_i2c_di;
		idx++;
		len--;
	}
	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_DI, 1, FLD_I2C_CMD_READ_ID, 1, FLD_I2C_CMD_NAK, 1);
	while(I2C_CMD_BUSY_FLAG);

	pbuf[idx] = reg_i2c_di;
	/***stop bit***/
	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_STOP, 1);
	while(I2C_CMD_BUSY_FLAG);
}
/**
 *   @brief      the write format in mapping mode telink design. pls refer to datasheet.
 *   @param[in]  pbuf - the first address of data master write to slave.
 *   @param[in]  len - the length of data to write.
 *   @return     none
 *   @timing chart: start + ID(w) + pbuf[0] + pbuf[...] + pbuf[len] + stop
 */
void i2c_write_mapping(unsigned char* pbuf, int len)
{
	int idx = 0;
	BM_CLR(reg_i2c_id, FLD_I2C_WRITE_READ_BIT); //ID|rw_bit; if rw_bit=1,read data; if rw_bit=0,write data
	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1);
	while(I2C_CMD_BUSY_FLAG);
	
	/***start to write data to slave***/
	for(idx = 0;idx < len; idx++){
		reg_i2c_di = pbuf[idx];
		reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_DI, 1);
		while(I2C_CMD_BUSY_FLAG);
	}
	/***send stop bit***/
	reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_STOP, 1);
	while(I2C_CMD_BUSY_FLAG);
}

/**
 *  @brief      read data from slave that is mapping mode.
 *  @param[in]  pbuf - the first address of SRAM buffer to store data.
 *  @param[in]  len - the length of data read from slave(byte).
 *  @return     none
 *  @timing chart: start + ID(r) + pbuf[0] + pbuf[...] + pbuf[len] + stop
 */
 void i2c_read_mapping(unsigned char* pbuf, int len)
 {
	 int idx = 0;
	 BM_SET(reg_i2c_id, FLD_I2C_WRITE_READ_BIT); //ID|rw_bit; if rw_bit=1,read data; if rw_bit=0,write data
	 reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_START, 1, FLD_I2C_CMD_ID, 1);
	 while(I2C_CMD_BUSY_FLAG);
	 
	 len--;
	 while(len){
		 reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_DI, 1, FLD_I2C_CMD_READ_ID, 1);
		 while(I2C_CMD_BUSY_FLAG);
		 pbuf[idx++] = reg_i2c_di;
		 len--;
	 }
	 reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_DI, 1, FLD_I2C_CMD_READ_ID, 1, FLD_I2C_CMD_NAK, 1);
	 while(I2C_CMD_BUSY_FLAG);
	 pbuf[idx] = reg_i2c_di;
	 
	 /***stop bit***/
	 reg_i2c_ctrl = MASK_VAL(FLD_I2C_CMD_STOP, 1);
	 while(I2C_CMD_BUSY_FLAG);
 }
 
/*******************************************************************
 ***************** the following is simulate ***********************
 *******************************************************************/

#ifndef PIN_I2C_SCL
#define PIN_I2C_SCL				0
#endif
#ifndef PIN_I2C_SDA
#define PIN_I2C_SDA				0
#endif

static inline void i2c_sim_wait(void){
}
void i2c_sim_long_wait(void){
	CLOCK_DLY_600NS;
}
/*** Pulling the line to ground is considered a logical zero
 *   while letting the line float is a logical one.
 *   http://en.wikipedia.org/wiki/I%C2%B2C
 */
static inline void i2c_sim_scl_out(int v){
	gpio_set_output_en(PIN_I2C_SCL,(!v));
}

static inline int i2c_sim_scl_in(void){
	return gpio_read(PIN_I2C_SCL);
}

/***Pulling the line to ground is considered a logical zero
 *  while letting the line float is a logical one.
 *  http://en.wikipedia.org/wiki/I%C2%B2C
 */
static inline void i2c_sim_sda_out(int v){
	gpio_set_output_en(PIN_I2C_SDA,(!v));
}

static inline int i2c_sim_sda_in(void){
	return gpio_read(PIN_I2C_SDA);
}

static inline void i2c_sim_scl_init(void){
	gpio_set_func(PIN_I2C_SCL, AS_GPIO);
}

static inline void i2c_sim_sda_init(void){
	gpio_set_func(PIN_I2C_SDA, AS_GPIO);
	gpio_set_input_en(PIN_I2C_SDA, 1);
}

static inline void i2c_sim_scl_idle(void){
	gpio_set_output_en(PIN_I2C_SCL, 0);
	gpio_write(PIN_I2C_SCL, 0);
}

static inline void i2c_sim_sda_idle(void){
	gpio_set_output_en(PIN_I2C_SDA, 0);
	gpio_write(PIN_I2C_SDA, 0);
}


void i2c_sim_init(){}

/***
 *  Sets clock high, then data high.  This will do a stop if data was low.
 *  Then sets data low, which should be a start condition.
 *  After executing, data is left low, while clock is left high
*/
void i2c_sim_start(void)
{
	i2c_sim_scl_init();
	i2c_sim_sda_init();
	i2c_sim_sda_idle();
	i2c_sim_scl_idle();
	i2c_sim_sda_out(0);		//sda: 0
	i2c_sim_wait();

}

/***
 * puts data low, then clock low,
 * then clock high, then data high.
 * This should cause a stop, which
 * should idle the bus, I.E. both clk and data are high.
*/
void i2c_sim_stop(void)
{
	i2c_sim_sda_out(0);
	i2c_sim_wait();
	i2c_sim_scl_out(0);
	i2c_sim_wait();
	i2c_sim_scl_out(1);
	i2c_sim_wait();
	i2c_sim_sda_out(1);
}

static void i2c_sim_wirte_bit(int bit)
{
	i2c_sim_scl_out(0);
	i2c_sim_sda_out(bit);
	i2c_sim_long_wait();
	i2c_sim_scl_out(1);
}

/***
 *   Read a bit from I2C bus
 */
static int i2c_sim_read_bit(void){
	i2c_sim_wirte_bit(1);
	return i2c_sim_sda_in();
}

int i2c_sim_write_byte(u8 dat){
	int i = 0x80;
	while(i){
		i2c_sim_wirte_bit((dat & i));
		i = i >> 1;
	}
	return i2c_sim_read_bit();
}

u8 i2c_sim_read_byte(int last){
	u8 dat = 0;
	foreach(i, 8){
		i2c_sim_wirte_bit(1);
		if(i2c_sim_sda_in()){
			dat =(dat << 1) | 0x01;
		}else{
			dat = dat << 1;
		}
	}
	i2c_sim_wirte_bit(last);
	return dat;
}

void i2c_sim_write(u8 id, u8 addr, u8 dat)
{
	i2c_sim_start();
	i2c_sim_write_byte(id);
	i2c_sim_write_byte(addr);
	i2c_sim_write_byte(dat);
	i2c_sim_stop();
}

u8 i2c_sim_read(u8 id, u8 addr)
{
	u8 dat;
	i2c_sim_burst_read(id, addr, &dat, 1);
	return dat;
}

void i2c_sim_burst_read(u8 id, u8 addr,u8 *p, int n)
{
	i2c_sim_start();

	i2c_sim_write_byte(id);
	i2c_sim_write_byte(addr);
	//i2c_sim_sda_out(1);
	//i2c_sim_scl_out(0);
	//i2c_sim_long_wait();
	//i2c_sim_scl_out(1);
	//i2c_sim_sda_out(0);
	i2c_sim_stop();
	i2c_sim_start();

	i2c_sim_write_byte(id | 1);

	for(int k = 0; k < n; ++k){
		*p++ = i2c_sim_read_byte( k ==(n-1) );
	}
	i2c_sim_stop();

}

void i2c_sim_burst_write(u8 id, u8 addr,u8 *p, int n)
{
	i2c_sim_start();
	i2c_sim_write_byte(id);
	i2c_sim_write_byte(addr);
	foreach(i, n){
		i2c_sim_write_byte(*p++);
	}
	i2c_sim_stop();
}
void i2c_sim_reset_e2prom(void ){
	i2c_sim_sda_out(1);
	for(u8 i=0;i<8;i++){
		i2c_sim_scl_out(0);
		i2c_sim_long_wait();
		i2c_sim_scl_out(1);
		i2c_sim_long_wait();
	}
	i2c_sim_sda_out(0);
}



