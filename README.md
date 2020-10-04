# Python 3 library for controlling Ebyte LoRa modules like the E32 with GPIOs (Raspberry Pi).

Work in progress.

Parameter setting is fully working on an Raspberry PI.

See the code in test_raspberrypi.py for instructions.

If you write to the serial device, just call ebyte.wait_for_aux_pin() after every write.

You can omit the AUX_PIN parameter, a delay is then used instead of waiting for the pin to go high.

## reading

```python ebyte.py read /dev/serial0 27 17 22```


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
