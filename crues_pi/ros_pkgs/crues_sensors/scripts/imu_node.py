#!/usr/bin/env python
import math
import smbus

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3


IMU_ADDRESS = 0x68

SAMPLE_PERIOD = 0.5

GYRO_RANGE = 250  # deg/sec
GYRO_DIVISIONS = 32768  # 2 ^15
GYRO_UNITS = (GYRO_RANGE * math.pi * 2.0) / (GYRO_DIVISIONS * 360)  # rad/sec

ACC_RANGE = 2 * 9.81  # m/s^2
ACC_DIVISIONS = 32768  # 2 ^15
ACC_UNITS = ACC_RANGE / ACC_DIVISIONS  # m/s^2


class I2C:
    """Represents device on an i2c bus"""

    def __init__(self, address, channel=1):
        """create an i2c_object on channel with an i2c address"""
        self.address = address
        self.bus = smbus.SMBus(channel)

    def write_byte(self, reg, val):
        self.bus.write_byte_data(self.address, reg, val)

    def read_byte(self, reg):
        """ read one byte from the i2c_object at the register address reg"""
        return self.bus.read_byte_data(self.address, reg)

    def read_word(self, reg):
        """ read a word from the i2c_object at the register addresses reg and reg+1"""
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg + 1)
        value = (h << 8) + l
        return value

    def read_signed_word(self, reg):
        """ read a word from the i2c_object at the register addresses reg and reg+1,
        maintaining the sign of the word"""
        val = self.read_word(reg)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val


class IMU:
    """This class represents an IMU object connected over I2C"""

    def __init__(self, channel=1, address=IMU_ADDRESS):
        rospy.init_node("imu")
        self.pub = rospy.Publisher('imu_data', Imu, queue_size=10)
        self.freq = rospy.get_param('~rate', 5)
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.rate = rospy.Rate(self.freq)
        self.address = address
        self.i2c = I2C(self.address, channel=channel)
        self.i2c.write_byte(0x6B, 0x00)  # turns imu on
        self.speed_vect = (0.0, 0.0, 0.0)

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self._publish_data()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass

    def read_gyro(self):
        """ reads IMU over i2c and returns tuple of x,y,z rotational velocities
        in rad per second """
        x_rot_v = self.i2c.read_signed_word(0x43) * GYRO_UNITS
        y_rot_v = self.i2c.read_signed_word(0x45) * GYRO_UNITS
        z_rot_v = self.i2c.read_signed_word(0x47) * GYRO_UNITS
        return -x_rot_v, -y_rot_v, z_rot_v

    def read_accel(self):
        """ reads IMU over i2c and returns tuple of x,y,z linear accelerations
        in ms^-2 """
        x_a = self.i2c.read_signed_word(0x3b) * ACC_UNITS
        y_a = self.i2c.read_signed_word(0x3d) * ACC_UNITS
        z_a = self.i2c.read_signed_word(0x3f) * ACC_UNITS
        return -x_a, -y_a, z_a

    def get_speeds(self):
        """get linear and angular velocities from the IMU device
        @:returns speed_vect, a three item tuple with linear veocities in x,y,z in m/s
        @:returns gyro_speeds, a three item tuple with xyz angular velocities in rad/s"""
        gyro_speeds = self.read_gyro()
        accel_vect = self.read_accel()
        self.speed_vect = tuple([(1.0 / self.freq) * accel_vect[i] + self.speed_vect[i] for i in range(len(accel_vect))])
        return self.speed_vect, gyro_speeds

    def _publish_data(self):
        acc = self.read_accel()
        gyro = self.read_gyro()
        msg = Imu()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # indicate orientation unknown
        msg.linear_acceleration = Vector3(acc[0], acc[1], acc[2])
        msg.angular_velocity = Vector3(gyro[0], gyro[1], gyro[2])
        rospy.loginfo("Gyro: %s  linear: %s" % (acc, gyro))
        self.pub.publish(msg)


def print_imu_data():
    imu = IMU()
    while True:
        print("gyro: " + str(imu.read_gyro()) + "   acc: " + str(imu.read_accel()))
        print("speeds: " + str(imu.get_speeds()))


if __name__ == '__main__':
    imu = IMU()
    imu.spin()
