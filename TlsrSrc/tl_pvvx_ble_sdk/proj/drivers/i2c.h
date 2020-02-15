/********************************************************************************************************
 * @file     i2c.h
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

#pragma once

#include "../common/types.h"

typedef enum {
#if ((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
	I2C_GPIO_GROUP_A3A4,
	I2C_GPIO_GROUP_B6B7,
	I2C_GPIO_GROUP_C0C1,
#elif (MCU_CORE_TYPE == MCU_CORE_8266)
	I2C_GPIO_GROUP_E7F1,
#endif
}I2C_GPIO_GroupTypeDef;

//I2C irq handler
typedef enum {
	I2C_IRQ_NONE = 0,
	I2C_IRQ_HOST_WRITE_ONLY,
	I2C_IRQ_HOST_READ_ONLY,
}I2C_I2CIrqSrcTypeDef;


#define I2C_IRQ_EN()    BM_SET(reg_irq_mask, FLD_IRQ_HOST_CMD_EN)
#define I2C_IRQ_DIS()   BM_CLR(reg_irq_mask, FLD_IRQ_HOST_CMD_EN)

/**
 *  @brief when device as I2C slave
 *         select slave mode:1.DMA;2.MAPPING
 */
enum I2C_SLAVE_MODE{
	I2C_SLAVE_DMA = 0,
	I2C_SLAVE_MAP,
};

/*****
 * brief  judge what the irq source is. host write or host read.
 */
I2C_I2CIrqSrcTypeDef I2C_SlaveIrqGet(void);

/****
 * brief  clear the irq status bit.
 */
void I2C_SlaveIrqClr(I2C_I2CIrqSrcTypeDef src);

/*****
 * brief: the function initial the relevant gpio as i2c.
 *        when enable one group of pins as i2c, the other two groups of pin should be disable the i2c function.
 *        in addition, when disable pin as i2c, we can enable the pin as gpio.
 * param[in] gpio_sda -- the pin as data line of i2c.
 * param[in] gpio_scl -- the pin as clock line of i2c.
 * return none
 */
void i2c_pin_init(I2C_GPIO_GroupTypeDef i2c_pin_group);
/**
 * @brief      This function set the id of slave device and the speed of I2C interface
 *             note: the param ID contain the bit of writting or reading.
 *             eg:the parameter 0x5C. the reading will be 0x5D and writting 0x5C.
 * @param[in]  slave_id - the id of slave device.it contains write or read bit,the lsb is write or read bit.
 *                       ID|0x01 indicate read. ID&0xfe indicate write.
 * @param[in]  div_clock - the division factor of I2C clock,
 *             I2C clock = System clock / (4*DivClock);if the datasheet you look at is 2*,pls modify it.
 * @return     none
 */
void i2c_master_init_div(unsigned char slave_id, unsigned char div_clock);

/**
 * @brief      This function set the id of slave device and the speed of I2C interface
 *             note: the param ID contain the bit of writting or reading.
 *             eg:the parameter 0x5C. the reading will be 0x5D and writting 0x5C.
 * @param[in]  slave_id - the id of slave device.it contains write or read bit,the lsb is write or read bit.
 *                       ID|0x01 indicate read. ID&0xfe indicate write.
 * @param[in]  i2c_speed is in Khz. for example: i2c_speed is 200, indicate 200k
 * @return     none
 */
void i2c_master_init_khz(unsigned char slave_id, unsigned int i2c_speed);
/**
 *  @brief      the function config the ID of slave and mode of slave.
 *  @param[in]  device_id - it contains write or read bit,the lsb is write or read bit.
 *              ID|0x01 indicate read. ID&0xfe indicate write.
 *  @param[in]  i2c_mode - set slave mode. slave has two modes, one is DMA mode, the other is MAPPING mode.
 *  @param[in]  pbuf - if slave mode is MAPPING, set the first address of buffer master write or read slave.
 *              notice: the buffer must align 128 bytes. the write address is pbuf while the read address is pbuf+64.
 *  @return     none
 */
void i2c_slave_init(unsigned char device_id,enum I2C_SLAVE_MODE i2c_mode,unsigned char* pbuf);
/**
 * @brief      This function writes one byte to the slave device at the specified address
 * @param[in]  addr - pointer to the address where the one byte data will be written
 * @param[in]  addr_len - length in byte of the address, which makes this function is
 *             compatible for slave device with both one-byte address and two-byte address
 * @param[in]  data - the one byte data will be written via I2C interface
 * @return     none
 * @ timing chart : start + ID(w) + addr(1or2) + data + stop
 */
void i2c_write_byte(unsigned char* addr, int addr_len, unsigned char data);
/**
 * @brief      This function reads one byte from the slave device at the specified address
 * @param[in]  Addr - pointer to the address where the one byte data will be read
 * @param[in]  AddrLen - length in byte of the address, which makes this function is
 *             compatible for slave device with both one-byte address and two-byte address
 * @return     the one byte data read from the slave device via I2C interface
 * @timing chart: start+ID(w)+addr(1or2)+ restart + ID(r) + data + stop
 */
unsigned char i2c_read_byte(unsigned char* addr, unsigned char addr_len);
/**
 * @brief      This function writes a bulk of data to the slave device at the specified address
 * @param[in]  addr - pointer to the address where the data will be written
 * @param[in]  addr_len - length in byte of the address, which makes this function is
 *             compatible for slave device with both one-byte address and two-byte address
 * @param[in]  pbuf - pointer to the data will be written via I2C interface
 * @param[in]  len - length in byte of the data will be written via I2C interface
 * @return     none
 */
void i2c_burst_write(unsigned char *addr, int addr_len, unsigned char * pbuf, int len);
/**
 * @brief      This function reads a bulk of data from the slave device at the specified address
 * @param[in]  addr - pointer to the address where the data will be read
 * @param[in]  addr_len - length in byte of the address, which makes this function is
 *             compatible for slave device with both one-byte address and two-byte address
 * @param[out] pbuf - pointer to the buffer will cache the data read via I2C interface
 * @param[in]  len - length in byte of the data will be read via I2C interface
 * @return     none
 */
void i2c_burst_read(unsigned char* addr, int addr_len, unsigned char * pbuf, int len);
/**
 *  @brief      the write format in dma mode telink design. pls refer to datasheet.
 *  @param[in]  addr - the register that master write data to slave in. support one byte and two bytes. i.e param2 AddrLen may be 1 or 2.
 *  @param[in]  addr_len - the length of register. enum 1 or 2. based on the spec of i2c slave.
 *  @param[in]  pbuf - the first SRAM buffer address to write data to slave in.
 *  @param[in]  len - the length of data master write to slave.
 *  @return     none
 *  @ timing chart: start + ID(w) + addr(1or2) + pbuf[0] +...+ pbuf[len] + stop
 */
void i2c_write_dma(unsigned short addr, unsigned char addr_len, unsigned char* pbuf, int len);
/**
 * @brief      the read format in dma mode telink design. pls refer to datasheet.
 * @param[in]  addr - the register master read data from slave in. support one byte and two bytes.
 * @param[in]  addr_len - the length of register. two data: 1 or 2.
 * @param[in]  pbuf - the first address of SRAM buffer master store data in.
 * @param[in]  len - the length of data master read from slave.
 * @return     none.
 * @timing chart: start + ID(w) + addr(1or2) + stop + start + ID(r) + pbuf[0] + ...+ pbuf[len] + stop
 */
void i2c_read_dma(unsigned short addr, unsigned char addr_len, unsigned char* pbuf, int len);
/**
 *   @brief      the write format in mapping mode telink design. pls refer to datasheet.
 *   @param[in]  pbuf - the first address of data master write to slave.
 *   @param[in]  len - the length of data to write.
 *   @return     none
 *   @timing chart: start + ID(w) + pbuf[0] + pbuf[...] + pbuf[len] + stop
 */
void i2c_write_mapping(unsigned char* pbuf, int len);
/**
 *  @brief      read data from slave that is mapping mode.
 *  @param[in]  pbuf - the first address of SRAM buffer to store data.
 *  @param[in]  len - the length of data read from slave(byte).
 *  @return     none
 *  @timing chart: start + ID(r) + pbuf[0] + pbuf[...] + pbuf[len] + stop
 */
void i2c_read_mapping(unsigned char* pbuf, int len);

/*******************************************************************
 *            the following is simulation method
 */

void i2c_sim_init(void);
void i2c_sim_write(u8 id, u8 addr, u8 dat);
u8 i2c_sim_read(u8 id, u8 addr);
void i2c_sim_burst_read(u8 id, u8 addr, u8 *p, int n);
void i2c_sim_burst_write(u8 id, u8 addr,u8 *p,int n);






