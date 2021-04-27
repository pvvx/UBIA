rem TlsrPgm.py -pCOM12 -a100 -u -m ea
TlsrUSBProg.py -pCOM12 we 0 8266_jdy_10.bin
TlsrUSBProg.py -pCOM12 we 0x71000 usbfloader.bin
TlsrUSBProg.py -pCOM12 we 0x72000 jdy_10_ota_72000.bin
TlsrUSBProg.py -pCOM12 wf 0x72800 floader.bin
