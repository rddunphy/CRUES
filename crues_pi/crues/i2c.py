import smbus


class i2c_object:

    def __init__(self, address, channel=1):
        self.address = address
        self.bus = smbus.SMBus(channel)

        def read_byte(reg):
            return self.bus.read_byte_data(address, reg)

        def read_word(reg):
            h = self.bus.read_byte_data(address, reg)
            l = self.bus.read_byte_data(address, reg + 1)
            value = (h << 8) + l
            return value

        def read_signed_word(reg):
            val = read_word(reg)
            if (val >= 0x8000):
                return -((65535 - val) + 1)
            else:
                return val
