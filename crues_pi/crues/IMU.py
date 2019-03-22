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

speed_vect = (0.0, 0.0, 0.0)

pins = Pins()


# class IMU:
#    """This class represents a scan of ranges from ultrasonic sensors.
#    """

# def __init__(self, channel=1):
IMU_i2c = i2c.i2c_object(IMU_ADDRESS, channel=1)
IMU_i2c.write_byte(0x6B, 0x00)


def _read_gyro():
    """ reads IMU over i2c and returns tuple of x,y,z rotational velocities
    in rad per second """
    x_rot_v = IMU_i2c.read_signed_word(0x43) * GYRO_UNITS
    y_rot_v = IMU_i2c.read_signed_word(0x45) * GYRO_UNITS
    z_rot_v = IMU_i2c.read_signed_word(0x47) * GYRO_UNITS
    out = (x_rot_v, y_rot_v, z_rot_v)
    rospy.loginfo("Gyro: %s, %s, %s" % out)
    return out


def _read_accel():
    """ reads IMU over i2c and returns tuple of x,y,z linear accelerations
    in ms^-2 """
    x_a = IMU_i2c.read_signed_word(0x3b) * ACC_UNITS
    y_a = IMU_i2c.read_signed_word(0x3d) * ACC_UNITS
    z_a = IMU_i2c.read_signed_word(0x3f) * ACC_UNITS
    return (x_a, y_a, z_a)


def get_speeds():
    global speed_vect
    gyro_speeds = _read_gyro()
    accel_vect = _read_accel()
    speed_vect = speed_vect + tuple(SAMPLE_PERIOD * x for x in accel_vect)
    speed_vect = tuple([SAMPLE_PERIOD * accel_vect[i] + speed_vect[i] for i in range(len(accel_vect))]) 

    return speed_vect, gyro_speeds


if __name__ == '__main__':
    while 1:
        print("gyro :" + _read_gyro() + "   acc:" + _read_accel())
        # print("speeds :" + get_speeds())
