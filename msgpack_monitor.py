import serial
import msgpack
from ebyte import EbyteRaspberryPi

PIN_M0 = 27
PIN_M1 = 17
PIN_AUX = 22

ser = serial.Serial('/dev/serial0')

ebyte = EbyteRaspberryPi(ser, PIN_M0, PIN_M1, PIN_AUX)

unpacker = msgpack.Unpacker()

while True:
    buf = ser.read(1)
    unpacker.feed(buf)
    for obj_ in unpacker:
        if obj_:
            print(obj_)
