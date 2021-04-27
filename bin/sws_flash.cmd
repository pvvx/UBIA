rem TlsrPgm.py -a100 -pCOM9 -u -m ea
TlsrPgm.py -a100 -pCOM9 -u we 0 8266_jdy_10.bin
TlsrPgm.py -pCOM9 -u we 0x71000 usbfloader.bin
TlsrPgm.py -pCOM9 -u we 0x72000 jdy_10_ota_72000.bin
TlsrPgm.py -pCOM9 -u -m wf 0x72800 uart_floader_72800.bin