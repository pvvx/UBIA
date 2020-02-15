/*
 * ble_ll_encrypt.h
 *
 *  Created on: 2016-9-22
 *      Author: Telink
 */

#ifndef BLE_LL_ENCRYPT_H_
#define BLE_LL_ENCRYPT_H_





typedef struct {
	u32		pkt;
	u8		dir;
	u8		iv[8];
} ble_cyrpt_nonce_t;


typedef struct {
	u32					enc_pno;
	u32					dec_pno;
	u8					sk[16];			//session key
	ble_cyrpt_nonce_t	nonce;
	u8					st;
	u8					enable;			//1: slave enable; 2: master enable
	u8					mic_fail;
} ble_crypt_para_t;

#endif /* BLE_LL_ENCRYPT_H_ */
