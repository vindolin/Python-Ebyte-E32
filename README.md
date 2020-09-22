# Python 3 library for controlling Ebyte LoRa modules like the E32.

Work in progress.

Parameter setting is fully working on an Raspberry PI.

See the code in test_raspberrypi.py for instructions.

If you write to the serial device, just call ebyte.wait_for_aux_pin() after every write.

You can omit the AUX_PIN parameter, a delay is then used instead of waiting for the pin to go high.
