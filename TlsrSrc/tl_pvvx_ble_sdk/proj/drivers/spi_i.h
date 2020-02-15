/********************************************************************************************************
 * @file     spi_i.h
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

#include "../config/user_config.h"
#include "../mcu/gpio.h"
#include "../mcu/register.h"

// use static inline, because, spi flash code must reside in memory..
// these code may be embedd in flash code

_attribute_ram_code_ static inline void mspi_wait(void){
	while(reg_master_spi_ctrl & FLD_MASTER_SPI_BUSY)
		;
}

_attribute_ram_code_ static inline void mspi_high(void){
	reg_master_spi_ctrl = FLD_MASTER_SPI_CS;
}

_attribute_ram_code_ static inline void mspi_low(void){
	reg_master_spi_ctrl = 0;
}

_attribute_ram_code_ static inline u8 mspi_get(void){
	return reg_master_spi_data;
}

_attribute_ram_code_ static inline void mspi_write(u8 c){
	reg_master_spi_data = c;
}

_attribute_ram_code_ static inline void mspi_ctrl_write(u8 c){
	reg_master_spi_ctrl = c;
}

_attribute_ram_code_ static inline u8 mspi_read(void){
	mspi_write(0);		// dummy, issue clock 
	mspi_wait();
	return mspi_get();
}

