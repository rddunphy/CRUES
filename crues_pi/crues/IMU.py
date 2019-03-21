import time

import RPi.GPIO as GPIO
import math
from crues import i2c


from pin_defs import Pins

IMU_ADDRESS = 0x68

SAMPLE_PERIOD = 0.5

GYRO_RANGE = 250  # deg/sec
GYRO_DIVISIONS = 32768  # 2 ^15
GYRO_UNITS = (GYRO_RANGE * math.pi * 2.0) / (GYRO_DIVISIONS * 360)  # rad/sec

ACC_RANGE = 2 * 9.81  # m/s^2
ACC_DIVISIONS = 32768  # 2 ^15
ACC_UNITS = ACC_RANGE / ACC_DIVISIONS  # m/s^2

pins = Pins()


class IMU:
    """This class represents a scan of ranges from ultrasonic sensors.
    """

    def __init__(self, channel=1):
        self.i2c = i2c.i2c_object(IMU_ADDRESS, channel)

    def read_gyro(self):
        """ reads IMU over i2c and returns tuple of x,y,z rotational velocities
        in rad per second """
        x_rot_v = self.i2c.read_signed_word(0x43) * GYRO_UNITS
        y_rot_v = self.i2c.read_signed_word(0x45) * GYRO_UNITS
        z_rot_v = self.i2c.read_signed_word(0x47) * GYRO_UNITS
        return x_rot_v, y_rot_v, z_rot_v

    def read_accel(self):
        """ reads IMU over i2c and returns tuple of x,y,z linear accelerations
        in rad per second """
        x_a = self.i2c.read_signed_word(0x3b) * ACC_UNITS
        y_a = self.i2c.read_signed_word(0x3d) * ACC_UNITS
        z_a = self.i2c.read_signed_word(0x3f) * ACC_UNITS
        return x_a, y_a, z_a

if __name__ == '__main__':
    imu = IMU()
    while(1):
        print("gyro :" + imu.read_gyro(), + "   acc:" + imu.read_accel())
