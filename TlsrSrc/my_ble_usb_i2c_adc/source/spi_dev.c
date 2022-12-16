/*
 * spi_dev.c
 *
 *  Created on: 08.12.2022
 *      Author: pvvx
 */
#include "proj/tl_common.h"
#if (USE_SPI_DEV)
#include "spi_dev.h"
#include "proj/drivers/spi.h"

#define SPI_CS_PIN GPIO_PE6

#define SPI_BUSY_FLAG     ((reg_spi_ctrl & FLD_SPI_BUSY)?1:0)

spi_ini_t spi_ini = {
	.div = 3,
	.mode = 3
};
uint8_t spi_dev_init = 0;

void SpiInit(void) {
	spi_master_init(spi_ini.div, spi_ini.mode);
	spi_master_pin_init(SPI_CS_PIN);
	spi_dev_init = 1;
}

void SpiUtr(void * outdata, spi_utr_t *tr, unsigned int wrlen) {
	unsigned char * pwrdata = (unsigned char *) &tr->wrdata;
	unsigned char * poutdata = (unsigned char *) outdata;
	unsigned int rdlen = tr->rdlen;

	if(!spi_dev_init)
		SpiInit();
	/***pull down cs line and enable write***/
	gpio_write(SPI_CS_PIN, 0); //pull down cs line and select the slave to handle.

	//unsigned char r = irq_disable();

	BM_CLR(reg_spi_ctrl, FLD_SPI_DATA_OUT_DIS | FLD_SPI_RD); //enable output
	/***write data to slave****/
	while (wrlen--) {
		reg_spi_data = *pwrdata++;
		while (SPI_BUSY_FLAG)
			; //wait data sending
	}
	if (rdlen) {
		reg_spi_ctrl |= MASK_VAL(FLD_SPI_RD, 1, FLD_SPI_DATA_OUT_DIS, 1); //enable read and disable output
		(void)reg_spi_data;
		while (SPI_BUSY_FLAG)
			;
		/***read the data.when read register reg_sip_data(0x08),the scl will generate 8 clock cycles to get the data from slave***/
		while (rdlen--) {
			*poutdata++ = reg_spi_data;
			while (SPI_BUSY_FLAG)
				;
		}
	}
	//irq_restore(r);
	/***pull up cs line to release the slave***/
	gpio_write(SPI_CS_PIN, 1);
}

#endif // USE_SPI_DEV
