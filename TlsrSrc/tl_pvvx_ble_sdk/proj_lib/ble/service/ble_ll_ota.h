/*
 * ble_ll_ota.h
 *
 *  Created on: 2015-7-20
 *      Author: Administrator
 */

#ifndef BLE_LL_OTA_H_
#define BLE_LL_OTA_H_

#ifndef	BLE_OTA_ENABLE
#define BLE_OTA_ENABLE						0
#endif

#ifndef BLE_OTA_FW_CHECK_EN
#define BLE_OTA_FW_CHECK_EN					1
#endif


#define CMD_OTA_FW_VERSION					0xff00
#define CMD_OTA_START						0xff01
#define CMD_OTA_END							0xff02


#define FLAG_FW_CHECK						0x5D
#define FW_CHECK_AGTHM2						0x02



typedef struct{
	u8  ota_start_flag;
#if (BLE_OTA_FW_CHECK_EN)
	u8 	fw_check_en;
	u16 fw_crc_last_index;
	u32 fw_crc_init;
#endif
}ota_service_t;

extern _attribute_aligned_(4) ota_service_t blcOta;


extern int blt_ota_start_flag;
extern u32 blt_ota_start_tick;
extern u32 blt_ota_timeout_us;

extern u32	ota_program_offset;
extern int 	ota_firmware_size_k;
extern u32	bls_ota_bootFlagAddr;


typedef void (*ota_startCb_t)(void);
typedef void (*ota_versionCb_t)(void);

typedef void (*ota_resIndicateCb_t)(int result);

extern ota_resIndicateCb_t otaResIndicateCb;


enum{
	OTA_SUCCESS = 0,     //success
	OTA_PACKET_LOSS,     //lost one or more OTA PDU
	OTA_DATA_CRC_ERR,     //data CRC err
	OTA_WRITE_FLASH_ERR,  //write OTA data to flash ERR
 	OTA_DATA_UNCOMPLETE,  //lost last one or more OTA PDU
 	OTA_TIMEOUT, 		  //
 	OTA_FW_CHECK_ERR,
	OTA_FW_NOT_TELINK,
};

void bls_ota_procTimeout(void);

//user interface
void bls_ota_registerStartCmdCb(ota_startCb_t cb);
void bls_ota_registerVersionReqCb(ota_versionCb_t cb);
void bls_ota_registerResultIndicateCb(ota_resIndicateCb_t cb);

void bls_ota_setTimeout(u32 timeout_us);

extern int otaWrite(void * p);
extern int otaRead(void * p);

extern void start_reboot(void);

//firmware_size_k  must be 4k aligned, ota_offset will be ignored in 8267/8269, valid in 8261/8266
void bls_ota_setFirmwareSizeAndOffset(int firmware_size_k, u32 ota_offset);


//only valid for 8261/8266
void bls_ota_setBootFlagAddr(u32 bootFlag_addr);

void bls_ota_clearNewFwDataArea(void);

unsigned short crc16 (unsigned char *pD, int len);

#endif /* BLE_LL_OTA_H_ */
