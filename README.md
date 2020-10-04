# Python 3 command line tool/library for controlling Ebyte LoRa modules like the E32 with GPIOs (Raspberry Pi).

## reading

```bash
usage: ebyte.py read [-h] serial pin_m0 pin_m1 pin_aux

positional arguments:
  serial      Path to the serial port device.
  pin_m0      M0 GPIO pin number.
  pin_m1      M1 GPIO pin number.
  pin_aux     AUX GPIO pin number.

optional arguments:
  -h, --help  show this help message and exit


python ebyte.py read /dev/serial0 27 17 22

```

```
Version:
--------
Frequency: 868MHz
  Version: 13
 Features: 20

Address/channel:
----------------
    addh: 0x00
    addl: 0x02
 address: 0x0002
    chan: 0x02

sped:
-----
    parity_bit: 8N1 (0)
     uart_baud: 19200 (3)
 air_data_rate: 2.4k (2)

option:
-------
  transmission_mode: fixed (1)
      io_drive_mode: TXD, RXD and AUX open-collector (1)
       wake_up_time: 250ms (0)
         fec_switch: on (1)
 transmission_power: 30dBm (0)
 ```

### writing

```python ebyte.py write /dev/serial0 27 17 22 --fec_switch=1 --chan=15```

### factory reset

```python ebyte.py reset /dev/serial0 27 17 22```

### use as library

See the code in test_raspberrypi.py if you want to use the library in your own code.
If you write to the serial device, just call ebyte.wait_for_aux_pin() after every write.
You can omit the AUX_PIN parameter, a delay is then used instead of waiting for the pin to go high.
