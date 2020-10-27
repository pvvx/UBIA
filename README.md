## USB/BLE to I2C/ADC/DAC/UART

Universal I2C/SMBus + ADC/DAC + UART to USB / BLE adapter.
#### JDY-10 (Tlsr8266, Bt 4.2, Rf Tx +8db)
![SCH](https://github.com/pvvx/UBIA/blob/master/DOCs/img/tBLETST_JDY10_sch.gif)

### I2C/SMBus:
>* SLC clock 100..2000 kHz.
>* #### SMBus auto-read mode:
> * USB: 20000 transactions per sec
> * BLE: 1250 transactions per sec
>* #### I2C/SMBus host write-read:
> * USB: 500 transactions per sec
> * BLE: 50 transactions per sec

### ADC(SAR):
>* 15..16 bits, Ref 1.3V, PGA 0..42.5 dB
>* SNR 73..96 dB (x1 to x8 decimation, 50 ksps to 250 sps)
>* USB: 250..50000 sps 
>* BLE: 250..5000 sps

### DAC(SDM):
>* 14..15 bits, Out p-p: 0..Vcc, SDM CLK 0.5..16 MHz
>* 2..200000 sps

### UART:
>* 300..4000000 Buad rate
>* 232 bytes block
>* BLE throughput: 30 transactions (half duplex, tx-rx) per sec (4 kbytes)

### BLE Power (default config)
>* Disconnect 0.09 mA (800 ms sleep: 17 uA, 5 ms active: TX impulse 33 mA +8dB)
>* Deep-sleep 1.7 / 3 uA (KEY2 - WakeUp KEY2 / KEY1 - Timer 30 sec)
>* Connect (7.5ms..4sec - sleep 17 uA, 3..5 ms active: TX impulse 33 mA +8dB)

### Chip Programming:
>* [COM-port](https://github.com/pvvx/TlsrComProg)
>* [STM32-BluePill](https://github.com/pvvx/TlsrTools)
>* [ET104-BT10-EVK](https://github.com/pvvx/TLSR8269-EVK)

[IDE](http://wiki.telink-semi.cn/wiki/IDE-and-Tools/IDE-for-TLSR8-Chips/)

[SDK](http://wiki.telink-semi.cn/wiki/chip-series/TLSR826x-Series/)

[forum](https://esp8266.ru/forum/threads/ubia-usb-ble-to-i2c-smbus-adapter.4810/)
