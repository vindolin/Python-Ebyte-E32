import serial
from ebyte import EbyteRaspberryPi

PIN_M0 = 27
PIN_M1 = 17
PIN_AUX = 22

ser = serial.Serial('/dev/serial0')

ebyte = EbyteRaspberryPi(ser, PIN_M0, PIN_M1, PIN_AUX)

print(ebyte.read_parameters())

ebyte.address = 0x0000
ebyte.uart_parity = 0
ebyte.uart_baud = 0b010
ebyte.air_data_rate = 0b010
ebyte.chan = 0x06
ebyte.transmission_mode = 0b0
ebyte.io_drive_mode = 0b1
ebyte.wake_up_time = 0b000
ebyte.fec_switch = 0b1
ebyte.transmission_power = 0b00

ebyte.write_parameters(True)

print(ebyte.read_parameters())
