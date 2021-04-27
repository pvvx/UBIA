rem TlsrPgm.py -pCOM30 -a100 -u -m ea
TlsrComProg.py -pCOM30 we 0 8266_jdy_10.bin
TlsrComProg.py -pCOM30 we 0x71000 usbfloader.bin
TlsrComProg.py -pCOM30 we 0x72000 jdy_10_ota_72000.bin
TlsrComProg.py -pCOM30 wf 0x72800 floader.bin
