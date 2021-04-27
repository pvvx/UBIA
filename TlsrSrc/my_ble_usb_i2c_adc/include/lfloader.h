/*
 * lfloader.h
 *
 *  Created on: 07.03.2020
 *      Author: pvvx
 */

#ifndef LFLOADER_H_
#define LFLOADER_H_

#define FLOADER_FADDR1 0x71000	// usbfloader
#define FLOADER_FADDR2 0x72800	// uartfloader
#define FLOADER_SIZE  0x01000

void load_floader(void);

#endif /* LFLOADER_H_ */
