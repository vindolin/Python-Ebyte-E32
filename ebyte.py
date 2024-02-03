import serial
from time import sleep
from textwrap import dedent
import gpiozero

__all__ = ('Ebyte', 'EbyteRaspberryPi')

MODE_NORMAL, MODE_WAKEUP, MODE_POWERSAVE, MODE_SLEEP = range(4)

FREQUENCIES = {
    0x32: '433',
    0x38: '470',
    0x44: '915',
    0x45: '868',
    0x46: '170',
}

DEFAULT_PARAMETERS = bytearray(b'\xc0\x00\x00\x1a\x17\x44')

PERMANENT = 0xc0
TEMPORARY = 0xc2

CMD_READ_PARAMETERS = b'\xc1\xc1\xc1'
CMD_READ_VERSION = b'\xc3\xc3\xc3'
CMD_RESET = b'\xc4\xc4\xc4'

BYTE_HEAD, BYTE_ADDH, BYTE_ADDL, BYTE_SPED, BYTE_CHAN, BYTE_OPTION = range(6)

BIT_LAYOUT = {
    'addh': {
        'byte': BYTE_ADDH,
        'default': 0x00,
        'max': 0xFF,
    },
    'addl': {
        'byte': BYTE_ADDL,
        'default': 0x00,
        'max': 0xFF,
    },
    'uart_parity': {
        'byte': BYTE_SPED,
        'pos': 6,
        'bits': 2,
        'default': 0b00,
        'max': 0b11,
        'doc': ['8N1', '8O1', '8E1', '8N1'],
    },
    'uart_baud': {
        'byte': BYTE_SPED,
        'pos': 3,
        'bits': 3,
        'default': 0b011,
        'max': 0b111,
        'doc': ['1200', '2400', '4800', '9600', '19200', '38400', '57600', '115200'],
    },
    'air_data_rate': {
        'byte': BYTE_SPED,
        'pos': 0,
        'bits': 3,
        'default': 0b010,
        'max': 0b111,
        'doc': ['0.3k', '1.2k', '2.4k', '4.8k', '9.6k', '19.2k', '19.2k', '19.2k'],
    },
    'chan': {
        'byte': BYTE_CHAN,
        'default': 0x06,
        'max': 0x1f,
    },
    'transmission_mode': {
        'byte': BYTE_OPTION,
        'pos': 7,
        'bits': 1,
        'default': 0b0,
        'max': 0b1,
        'doc': ['transparent', 'fixed'],
    },
    'io_drive_mode': {
        'byte': BYTE_OPTION,
        'pos': 6,
        'bits': 1,
        'default': 0b1,
        'max': 0b1,
        'doc': ['TXD and AUX push-pull, RXD pull-up', 'TXD, RXD and AUX open-collector'],
    },
    'wake_up_time': {
        'byte': BYTE_OPTION,
        'pos': 3,
        'bits': 3,
        'default': 0b000,
        'max': 0b010,
        'doc': ['250ms', '500ms', '750ms', '1000ms', '1250ms', '1500ms', '1750ms', '2000ms'],
    },
    'fec_switch': {
        'byte': BYTE_OPTION,
        'pos': 2,
        'bits': 1,
        'default': 0b1,
        'max': 0b1,
        'doc': ['off', 'on'],
    },
    'transmission_power': {
        'byte': BYTE_OPTION,
        'pos': 0,
        'bits': 2,
        'default': 0b00,
        'max': 0b11,
        'doc': ['30dBm', '27dBm', '24dBm', '21dBm'],
    },
}


class Ebyte:
    # def init_pins(self):
    #     raise NotImplementedError

    def pin_wait_delay(self):
        raise NotImplementedError

    def set_mode(self, mode):
        raise NotImplementedError

    def wait_for_aux_pin(self):
        raise NotImplementedError

    def __init__(self, serial, pin_m0, pin_m1, pin_aux):
        self._serial = serial
        self._pin_m0 = gpiozero.DigitalOutputDevice(pin_m0)
        self._pin_m1 = gpiozero.DigitalOutputDevice(pin_m1)
        self._pin_aux = gpiozero.DigitalInputDevice(pin_aux)

        self._parameters = DEFAULT_PARAMETERS

        self._version_data = self.read_version_number()

        self.set_mode(MODE_NORMAL)

    def get_bit(self, byte, position, length):
        return byte >> position & ((1 << length) - 1)

    def set_bit(self, byte, position, length, value):
        mask = (1 << length) - 1
        return (byte & ~(mask << position) | ((value & mask) << position))

    def __getattr__(self, name):
        if name == 'address':
            return self.addh << 8 | self.addl
        else:
            try:
                layout = BIT_LAYOUT[name]

                # bitmask
                if 'bits' in layout:
                    return self.get_bit(
                        self._parameters[layout['byte']],
                        layout['pos'],
                        layout['bits'],
                    )
                else:
                    return self._parameters[layout['byte']]

            except KeyError as err:
                raise UserWarning(f'not a valid property: {err}')

    def __setattr__(self, name, value):
        if name.startswith('_'):
            object.__setattr__(self, name, value)
            return

        if name == 'address':
            self.addh = (value & 0xffff) >> 8
            self.addl = value & 0xff
        else:
            try:
                layout = BIT_LAYOUT[name]

                if value > layout['max']:
                    raise UserWarning(f'value ({value}) for {name} must be <= {layout["max"]}')

                # bitmask
                if 'bits' in layout:
                    self._parameters[layout['byte']] = self.set_bit(
                        self._parameters[layout['byte']],
                        layout['pos'],
                        layout['bits'],
                        value
                    )

                # whole byte
                else:
                    self._parameters[layout['byte']] = value

            except KeyError as err:
                raise UserWarning(f'not a valid property: {err}')

    def flush(self):
        self._serial.reset_input_buffer()

    def read_version_number(self):
        self.set_mode(MODE_SLEEP)

        self._serial.write(CMD_READ_VERSION)

        self.pin_wait_delay()

        while self._serial.inWaiting():

            head, freq, version, features = self._serial.read(4)

            return {
                'freq': freq,
                'version': version,
                'features': features,
            }

        self.set_mode(MODE_NORMAL)

    def show_parameters(self):
        def format_value(name):
            value = getattr(self, name)
            return f'{BIT_LAYOUT[name]["doc"][value]} ({value})'

        frequency = FREQUENCIES[self._version_data['freq']]

        return dedent(f'''
            Version:
            --------
            Frequency: {frequency}MHz
              Version: {self._version_data['version']}
             Features: {self._version_data['features']}

            Address/channel:
            ----------------
                addh: 0x{self.addh:02x}
                addl: 0x{self.addl:02x}
             address: 0x{self.address:04x}
                chan: 0x{self.chan:02x}

            sped:
            -----
                parity_bit: {format_value('uart_parity')}
                 uart_baud: {format_value('uart_baud')}
             air_data_rate: {format_value('air_data_rate')}

            option:
            -------
              transmission_mode: {format_value('transmission_mode')}
                  io_drive_mode: {format_value('io_drive_mode')}
                   wake_up_time: {format_value('wake_up_time')}
                     fec_switch: {format_value('fec_switch')}
             transmission_power: {format_value('transmission_power')}
        ''')

    def read_parameters(self):
        self.set_mode(MODE_SLEEP)

        self._serial.write(CMD_READ_PARAMETERS)

        self.pin_wait_delay()

        while self._serial.inWaiting():

            self._parameters = bytearray(self._serial.read(6))

            return self.show_parameters()

        self.set_mode(MODE_NORMAL)

    def write_parameters(self, permanent=False):
        """Write the parameters to the module.
        This function temporary sets the serial port to 9600, 8N1 before writing
        and then restores the former settings after it's done.
        """

        # save the current serial port settings
        old_baudrate = self._serial.baudrate
        old_parity = self._serial.parity
        old_stopbits = self._serial.stopbits

        # the ebyte modules can only save the settings in this mode
        self._serial.baudrate = 9600
        self._serial.parity = serial.PARITY_NONE
        self._serial.stopbits = serial.STOPBITS_ONE

        sleep(0.1)  # necessary?

        self.set_mode(MODE_SLEEP)

        self.flush()

        self._parameters[0] = PERMANENT if permanent else TEMPORARY
        self._serial.write(self._parameters)

        self.wait_for_aux_pin()

        # restore serial settings
        self._serial.baudrate = old_baudrate
        self._serial.parity = old_parity
        self._serial.stopbits = old_stopbits

        self.set_mode(MODE_NORMAL)


class EbyteRaspberryPi(Ebyte):
    def pin_wait_delay(self):
        sleep(0.04)

    def set_mode(self, mode):
        if mode == MODE_NORMAL:
            self._pin_m0.off()
            self._pin_m1.off()

        if mode == MODE_WAKEUP:
            self._pin_m0.on()
            self._pin_m1.off()

        if mode == MODE_POWERSAVE:
            self._pin_m0.off()
            self._pin_m1.on()

        if mode == MODE_SLEEP:
            self._pin_m0.on()
            self._pin_m1.on()

        self.pin_wait_delay()

    def wait_for_aux_pin(self):
        if self._pin_aux:
            while not self._pin_aux.value:
                pass
        else:
            self.pin_wait_delay()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Configure an Ebyte E32 like LoRa module connected to the GPIO pin header on a Raspberry Pi.')

    subparsers = parser.add_subparsers(dest='command')

    parser_read = subparsers.add_parser('read', help='Read the configuration parameters.')

    parser_read.add_argument('serial', help='Path to the serial port device.')
    parser_read.add_argument('pin_m0', type=int, help='M0 GPIO pin number.')
    parser_read.add_argument('pin_m1', type=int, help='M1 GPIO pin number.')
    parser_read.add_argument('pin_aux', type=int, default=None, help='AUX GPIO pin number.')

    parser_write = subparsers.add_parser('write', help='Write parameters to module, see manual for valid values.')
    parser_write.add_argument('serial', help='Path to the serial port device.')
    parser_write.add_argument('pin_m0', type=int, help='M0 GPIO pin number.')
    parser_write.add_argument('pin_m1', type=int, help='M1 GPIO pin number.')
    parser_write.add_argument('pin_aux', type=int, default=None, help='AUX GPIO pin number.')

    parser_write.add_argument('--permanent', type=bool, default=False, help='Write parameters so they are restored after a powerdown.')
    parser_write.add_argument('--address', type=int, default=None, help='Module address 0x0000 to 0xFFFF.')

    parser_reset = subparsers.add_parser('reset', help='Factory reset the module.')
    parser_reset.add_argument('serial', help='Path to the serial port device.')
    parser_reset.add_argument('pin_m0', type=int, help='M0 GPIO pin number.')
    parser_reset.add_argument('pin_m1', type=int, help='M1 GPIO pin number.')
    parser_reset.add_argument('pin_aux', type=int, default=None, help='AUX GPIO pin number.')

    for parameter_name, parameter_data in BIT_LAYOUT.items():
        if parameter_name not in ['addh', 'addl']:
            parser_write.add_argument(f'--{parameter_name}', type=int, default=None)

    args = parser.parse_args()

    ser = serial.Serial(args.serial)
    ebyte = EbyteRaspberryPi(ser, args.pin_m0, args.pin_m1, args.pin_aux)

    if args.command == 'read':
        print(ebyte.read_parameters())

    elif args.command == 'write':
        for arg in vars(args):
            if getattr(args, arg) is not None:
                if arg in BIT_LAYOUT or arg == 'address':
                    setattr(ebyte, arg, getattr(args, arg))

        ebyte.write_parameters(args.permanent)
        print(ebyte.read_parameters())

    elif args.command == 'reset':
        ebyte.address = 0x0000
        ebyte.uart_parity = 0
        ebyte.uart_baud = 0b011
        ebyte.air_data_rate = 0b010
        ebyte.chan = 0x06
        ebyte.transmission_mode = 0b0
        ebyte.io_drive_mode = 0b1
        ebyte.wake_up_time = 0b000
        ebyte.fec_switch = 0b1
        ebyte.transmission_power = 0b00

        ebyte.write_parameters(True)
        print(ebyte.read_parameters())
