import serial
from ebyte import EbyteRaspberryPi
from time import sleep

PIN_M0 = 27
PIN_M1 = 17
PIN_AUX = 22

ser = serial.Serial('/dev/serial0')

ebyte = EbyteRaspberryPi(ser, PIN_M0, PIN_M1, PIN_AUX)

print(ebyte.read_version_number())
# print(ebyte.read_parameters())

print(ebyte.read_parameters())

print(ebyte._parameters)

ebyte.transmission_mode = 1
ebyte.address = 3
ebyte.chan = 2
print(ebyte.transmission_mode)
print(ebyte.chan)
print(ebyte.address)

ebyte.write_parameters()

# print(ebyte._parameters)
print(ebyte.read_parameters())
