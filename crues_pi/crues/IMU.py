#!/usr/bin/env python
import math
from crues import i2c

from pin_defs import Pins
import rospy

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
    """This class represents an IMU object connected over I2C
   """

    def __init__(self, address, rate, channel=1):
        self.address = address
        self.IMU_i2c = i2c.i2c_object(self.address, channel=channel)
        self.rate = rate
        self.IMU_i2c.write_byte(0x6B, 0x00) # turns imu on
        self.speed_vect = (0.0, 0.0, 0.0)


    def _read_gyro(self):
        """ reads IMU over i2c and returns tuple of x,y,z rotational velocities
        in rad per second """
        x_rot_v = self.IMU_i2c.read_signed_word(0x43) * GYRO_UNITS
        y_rot_v = self.IMU_i2c.read_signed_word(0x45) * GYRO_UNITS
        z_rot_v = self.IMU_i2c.read_signed_word(0x47) * GYRO_UNITS
        return (x_rot_v, y_rot_v, z_rot_v)


    def _read_accel(self):
        """ reads IMU over i2c and returns tuple of x,y,z linear accelerations
        in ms^-2 """
        x_a = self.IMU_i2c.read_signed_word(0x3b) * ACC_UNITS
        y_a = self.IMU_i2c.read_signed_word(0x3d) * ACC_UNITS
        z_a = self.IMU_i2c.read_signed_word(0x3f) * ACC_UNITS
        return (x_a, y_a, z_a)


    def get_speeds(self):
        """get linear and angular velocities from the IMU device
        @:returns speed_vect, a three item tuple with linear veocities in x,y,z in m/s
        @:returns gyro_speeds, a three item tuple with xyz angular velocities in rad/s"""
        global speed_vect
        gyro_speeds = self._read_gyro()
        accel_vect = self._read_accel()
        speed_vect = tuple([(1.0/self.rate) * accel_vect[i] + speed_vect[i] for i in range(len(accel_vect))])
        return speed_vect, gyro_speeds

    def read_values(self):
        gyro_speeds_vect = self._read_gyro()
        accel_vect = self._read_accel()
        return accel_vect, gyro_speeds_vect

if __name__ == '__main__':
    imu = IMU(0x68, 5)
    while 1:
        print("gyro :" + imu._read_gyro() + "   acc:" + imu._read_accel())
        print("speeds :" + imu.get_speeds())
