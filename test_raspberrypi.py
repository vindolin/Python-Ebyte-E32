import serial
from ebyte import EbyteRaspberryPi

PIN_M0 = 27
PIN_M1 = 17
PIN_AUX = 22

ser = serial.Serial('/dev/serial0')

ebyte = EbyteRaspberryPi(ser, PIN_M0, PIN_M1, PIN_AUX)

print(ebyte.read_parameters())

ebyte.transmission_mode = 1
ebyte.address = 2
ebyte.chan = 2

print(ebyte.transmission_mode)
print(ebyte.chan)
print(ebyte.address)

ebyte.write_parameters(True)

print(ebyte.read_parameters())
