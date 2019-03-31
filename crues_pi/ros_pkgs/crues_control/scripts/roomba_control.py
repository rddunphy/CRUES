#!/usr/bin/env python

import random
import time

import rospy
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist


LEFT = 0
CENTRE = 1
RIGHT = 2


class Roomba:
    def __init__(self):
        rospy.init_node('roomba')
        self.turn_vel = rospy.get_param('~turn_vel', 1)
        self.fwd_vel = rospy.get_param('~fwd_vel', 0.2)
        self.obstacle_range = rospy.get_param('~obstacle_range', 100)
        self.rate = rospy.Rate(rospy.get_param('~rate', 50))
        self.time_to_stop_turning = -1
        self.turn_twist = None
        self.last_ranges = {LEFT: None, CENTRE: None, RIGHT: None}
        rospy.Subscriber('ul_range', Int32, self.range_callback, callback_args=LEFT)
        rospy.Subscriber('uc_range', Int32, self.range_callback, callback_args=CENTRE)
        rospy.Subscriber('ur_range', Int32, self.range_callback, callback_args=RIGHT)
        self.twist_pub = rospy.Publisher('twist', Twist, queue_size=10)
        self.gled_pub = rospy.Publisher('green_led', Bool, queue_size=10)
        self.rled_pub = rospy.Publisher('red_led', Bool, queue_size=10)

    def spin(self):
        while not rospy.is_shutdown():
            self.publish_cmd()
            self.rate.sleep()

    def publish_cmd(self):
        if any([r is None for r in self.last_ranges.values()]):
            # Still waiting for first reading from another sensor
            return
        if self.time_to_stop_turning > time.time():
            # Finish turning first
            self.twist_pub.publish(self.turn_twist)
            return
        else:
            if all([r > self.obstacle_range for r in self.last_ranges.values()]):
                # All clear, just go for it
                self.forward()
            elif self.last_ranges[LEFT] < self.last_ranges[RIGHT]:
                # Obstacle more on left, so turn right
                self.turn_right()
            else:
                # Obstacle more on right, so turn left
                self.turn_left()

    def forward(self):
        out = Twist()
        self.time_to_stop_turning = -1
        out.linear.x = self.fwd_vel
        self.gled_pub.publish(True)
        self.rled_pub.publish(False)
        self.twist_pub.publish(out)

    def turn_right(self):
        self.turn_twist = Twist()
        self.turn_twist.angular.z = -self.turn_vel
        self.time_to_stop_turning = time.time() + random.uniform(0.2, 0.6)
        self.gled_pub.publish(False)
        self.rled_pub.publish(True)
        self.twist_pub.publish(self.turn_twist)

    def turn_left(self):
        self.turn_twist = Twist()
        self.turn_twist.angular.z = self.turn_vel
        self.time_to_stop_turning = time.time() + random.uniform(0.2, 0.6)
        self.gled_pub.publish(False)
        self.rled_pub.publish(True)
        self.twist_pub.publish(self.turn_twist)

    def range_callback(self, msg, s):
        self.last_ranges[s] = msg.data


if __name__ == '__main__':
    try:
        roomba = Roomba()
        roomba.spin()
    except rospy.ROSInterruptException:
        pass
