### UBIA Firmware files

* **8266_jdy_10.bin** - the main project file.

* **usbfloader.bin** - file of project [TlsrComProg-Usbfloader](https://github.com/pvvx/TlsrComProg/tree/master/Usbfloader).
Used to download firmware via USB-COM.
It is activated if, at module startup, the SWS pin is shorted to GND.

* **floader.bin** - file of project [TlsrComProg-Uartfloader](https://github.com/pvvx/TlsrComProg/tree/master/Uartfloader).
Used to download firmware via UART.
It is activated if, at module startup, the SWS pin is shorted to GND.


* **jdy_10_ota_72000.bin** - part for firmware on BLE OTA

##### Binary files for Firmware:

| filename  | flash addr  |
| ------------ | ------------ |
| 8266_jdy_10.bin  |  0x000000 |
| usbfloader.bin  | 0x071000  |
| jdy_10_ota_72000.bin  | 0x072000  |
| floader.bin  | 0x072800  |


[TelinkOTA](https://pvvx.github.io/UBIA/TelinkOTA.html)

