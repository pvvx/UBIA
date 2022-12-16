/*
 * spi_dev.h
 *
 *  Created on: 08.12.2022
 *      Author: pvvx
 */

#ifndef SPI_DEV_H_
#define SPI_DEV_H_

extern spi_ini_t spi_ini;

void SpiInit(void);
void SpiUtr(void * outdata, spi_utr_t *tr, unsigned int wrlen);

#endif /* SPI_DEV_H_ */
