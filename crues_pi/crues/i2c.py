#!/usr/bin/env python
import smbus


class i2c_object:

    def __init__(self, address, channel=1):
        self.address = address
        self.bus = smbus.SMBus(channel)
        
    def write_byte(self, reg, val):
        self.bus.write_byte_data(self.address, reg, val)

    def read_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def read_word(self, reg):
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg + 1)
        value = (h << 8) + l
        return value

    def read_signed_word(self, reg):
        val = self.read_word(reg)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
