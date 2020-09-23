# Python 3 library for controlling Ebyte LoRa modules like the E32 with GPIOs (Raspberry Pi).

Work in progress.

Parameter setting is fully working on an Raspberry PI.

See the code in test_raspberrypi.py for instructions.

If you write to the serial device, just call ebyte.wait_for_aux_pin() after every write.

You can omit the AUX_PIN parameter, a delay is then used instead of waiting for the pin to go high.


```python
import serial
from ebyte import EbyteRaspberryPi

PIN_M0 = 27
PIN_M1 = 17
PIN_AUX = 22

ser = serial.Serial('/dev/serial0')

ebyte = EbyteRaspberryPi(ser, PIN_M0, PIN_M1, PIN_AUX)

print(ebyte.read_version_number())
print(ebyte.read_parameters())

ebyte.transmission_mode = 1
ebyte.address = 2
ebyte.chan = 2

print(ebyte.transmission_mode)
print(ebyte.chan)
print(ebyte.address)

ebyte.write_parameters()

print(ebyte.read_parameters())
```

```
Frequency: 868MHz
  Version: 13
 Features: 20


Address:
--------
    addh: 0
    addl: 2
 address: 2
    chan: 2

sped:
--------
    parity_bit: 8N1 (0)
     uart_baud: 19200 (3)
 air_data_rate: 2.4k (2)

option:
--------
  transmission_mode: fixed (1)
      io_drive_mode: TXD, RXD and AUX open-collector (1)
       wake_up_time: 250ms (0)
         fec_switch: on (1)
 transmission_power: 30dBm (0)

1
2
2

Address:
--------
    addh: 0
    addl: 2
 address: 2
    chan: 2

sped:
--------
    parity_bit: 8N1 (0)
     uart_baud: 19200 (3)
 air_data_rate: 2.4k (2)

option:
--------
  transmission_mode: fixed (1)
      io_drive_mode: TXD, RXD and AUX open-collector (1)
       wake_up_time: 250ms (0)
         fec_switch: on (1)
 transmission_power: 30dBm (0)
```
