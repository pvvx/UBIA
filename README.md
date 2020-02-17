## USB/BLE to I2C/ADC

Universal I2C/SMBus + ADC to USB / BLE adapter.
#### JDY-10 (Tlsr8266, Bt 4.2, Rf Tx +8db)
![SCH](https://github.com/pvvx/UBIA/blob/master/DOCs/img/tBLETST_JDY10_sch.gif)

### I2C/SMBus:
* SLC clock 100..2000 kHz.
* #### SMBus auto-read mode:
 * USB: 20000 transactions per sec
 * BLE: 1250 transactions per sec
* #### I2C/SMBus host write-read:
 * USB: 500 transactions per sec
 * BLE: 50 transactions per sec

### ADC(SAR):
* 15..16 bits, Ref 1.3V, PGA 0..42.5 dB
* SNR 73..96 dB (x1 to x8 decimation, 50 ksps to 250 sps)
* USB: 250..50000 sps 
* BLE: 250..5000 sps

### DAC(SDM):
* 14..15 bits, Out p-p: 0..Vcc, SDM CLK 0.5..16 MHz
* 2..200000 sps

### BLE Power (default config)
* Disconnect 0.09 mA (800 ms sleep: 17 uA, 5 ms active: TX impulse 33 mA +8dB)
* Deep-sleep 1.7 / 3 uA
* Connect (7.5ms..4sec - sleep 17 uA, 3..5 ms active: TX impulse 33 mA +8dB)

[IDE](http://wiki.telink-semi.cn/dokuwiki/doku.php?id=menu:tools:ide_quick_start)

[SDK](http://wiki.telink-semi.cn/dokuwiki/doku.php?id=menu:chipset:tslr826x)

[forum](https://esp8266.ru/forum/threads/ble-modul-jdy-10-na-chipe-tlsr8266.4654/)