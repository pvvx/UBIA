/********************************************************************************************************
 * @file     flash.c
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
#include "../drivers/spi.h"
#include "flash.h"
#include "spi_i.h"

_attribute_ram_code_ static inline int flash_is_busy(){
	return mspi_read() & 0x01;				//  the busy bit, pls check flash spec
}

_attribute_ram_code_ static void flash_send_cmd(u8 cmd){
	mspi_high();
	sleep_us(1);
	mspi_low();
	mspi_write(cmd);
	mspi_wait();
}

_attribute_ram_code_ static void flash_send_addr(u32 addr){
	mspi_write((u8)(addr>>16));
	mspi_wait();
	mspi_write((u8)(addr>>8));
	mspi_wait();
	mspi_write((u8)(addr));
	mspi_wait();
}

//  make this a asynchorous version
_attribute_ram_code_ static void flash_wait_done()
{
	sleep_us(100);
	flash_send_cmd(FLASH_READ_STATUS_CMD);

	int i;
	for(i = 0; i < 10000000; ++i){
		if(!flash_is_busy()){
			break;
		}
	}
	mspi_high();
}

_attribute_ram_code_ void flash_erase_sector(u32 addr){
	u8 r = irq_disable();

	WATCHDOG_CLEAR;  //in case of watchdog timeout

	flash_send_cmd(FLASH_WRITE_ENABLE_CMD);
	flash_send_cmd(FLASH_SECT_ERASE_CMD);
	flash_send_addr(addr);
	mspi_high();
	flash_wait_done();

	irq_restore(r);
}

_attribute_ram_code_ void flash_write_page(u32 addr, u32 len, u8 *buf){
	u8 r = irq_disable();

	// important:  buf must not reside at flash, such as constant string.  If that case, pls copy to memory first before write
	flash_send_cmd(FLASH_WRITE_ENABLE_CMD);
	flash_send_cmd(FLASH_WRITE_CMD);
	flash_send_addr(addr);

	u32 i;
	for(i = 0; i < len; ++i){
		mspi_write(buf[i]);		/* write data */
		mspi_wait();
	}
	mspi_high();
	flash_wait_done();

	irq_restore(r);
}

_attribute_ram_code_ void flash_read_page(u32 addr, u32 len, u8 *buf){
	u8 r = irq_disable();


	flash_send_cmd(FLASH_READ_CMD);
	flash_send_addr(addr);

	mspi_write(0x00);		/* dummy,  to issue clock */
	mspi_wait();
	mspi_ctrl_write(0x0a);	/* auto mode */
	mspi_wait();
	/* get data */
	for(int i = 0; i < len; ++i){
		*buf++ = mspi_get();
		mspi_wait();
	}
	mspi_high();

	irq_restore(r);
}

#if 0
_attribute_ram_code_ u32 flash_get_jedec_id(){
	u8 r = irq_disable();
	flash_send_cmd(FLASH_GET_JEDEC_ID);
	u8 manufacturer = mspi_read();
	u8 mem_type = mspi_read();
	u8 cap_id = mspi_read();
	mspi_high();
	irq_restore(r);
	return (u32)((manufacturer << 24 | mem_type << 16 | cap_id));
}
#endif
