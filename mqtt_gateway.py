import serial
from datetime import datetime
import paho.mqtt.client as mqtt
import msgpack
from ebyte import EbyteRaspberryPi

PIN_M0 = 27
PIN_M1 = 17
PIN_AUX = 22

# ser = serial.Serial('/dev/serial0')
ser = serial.Serial('/dev/ttyAMA0')

client = mqtt.Client()
ebyte = EbyteRaspberryPi(ser, PIN_M0, PIN_M1, PIN_AUX)

# client.username_pw_set('tulpe', 'HKUabc98')


def publish(topic, payload):
    # return
    client.connect('localhost', 1884)
    client.publish(topic, payload, 0, True)
    # print(topic, payload)
    client.loop()
    client.disconnect()


# unpacker = msgpack.Unpacker(strict_map_key=False)
unpacker = msgpack.Unpacker()

last_payloads = {}

while True:
    unpacker.feed(ser.read(1))
    for arr_ in unpacker:
        if arr_:
            try:
                client_id, counter, name, payload = arr_
                topic = f'{client_id}/{name}'

                # if client_id == 'greenhouse':  # and name in ['temp_inside', 'temp_outside':
                #    try:
                #        payload = float(payload)
                #    except Exception:
                #        pass

                if True or last_payloads.get(topic) != payload:
                    last_payloads[topic] = payload
                    print(
                        datetime.now().strftime("%H:%M:%S"),
                        topic,
                        payload
                    )
                    publish(topic, payload)

            except Exception:
                print('.')
