#!/usr/bin/env python
import time
import unittest
import sys

import rosunit
from std_msgs.msg import Int16
import rostest
import rospy
from crues_tools import GPIO_MOCK

PKG = 'crues_sensors'
NAME = 'enc_test.py'


class EncoderTest(unittest.TestCase):
    def __init__(self, *args):
        super(EncoderTest, self).__init__(*args)
        self.data = -1
        self.updated = False
        rospy.init_node("test_node", anonymous=True)
        rospy.Subscriber('wheel', Int16, self._callback)

    def _callback(self, msg):
        self.data = msg.data
        self.updated = True

    def test_encoder_zero(self):
        timeout = time.time() + 10
        self.updated = False
        while not rospy.is_shutdown() and not self.updated and time.time() < timeout:
            time.sleep(0.1)
        self.assertEqual(self.data, 0)

    def test_encoder_inc(self):
        GPIO_MOCK.sim_rising_edge(1)
        timeout = time.time() + 10
        self.updated = False
        while not rospy.is_shutdown() and self.data < 0 and time.time() < timeout:
            time.sleep(0.1)
        self.assertEqual(self.data, 1)



if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, EncoderTest, sys.argv)
