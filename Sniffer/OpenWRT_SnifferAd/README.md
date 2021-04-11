### BLE Advertising monitor in OpenWRT on TLSR8266 chip (USB).

---

### Usage:

```
cc2540 --channel <37-39> [--mac xx:xx:xx:xx:xx:xx] [--fifo <path>] [--daemon]
```

### Build:

```bash
gcc -O2 -Wall -o cc2540 cc2540.c -s
```

### String Format:

"6072ed7d-0003959e:d6be898e001ced5e0b38c1a4151695fe50305b059ded5e0b38c1a40a10015802b80ba7515220a6\n"

| Time stamp UTC (sec-us) | Access Address | Ad length | MAC          | Ad-data                                      | CRC    | RSSI | FCS  |
| ----------------------- | -------------- | --------- | ------------ | -------------------------------------------- | ------ | ---- | ---- |
| 6072ed7d-0003959e:      | d6be898e00     | 1c        | ed5e0b38c1a4 | 151695fe50305b059ded5e0b38c1a40a10015802b80b | a75152 | 20   | a6   |

RSSI (dBm) = 94 - RSSI 





