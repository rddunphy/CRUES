#!/usr/bin/env python

import random
import time

import rospy
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist


LEFT = 0
CENTRE = 1
RIGHT = 2


class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.turn_vel = rospy.get_param('~turn_vel', 1.5)
        self.fwd_vel = rospy.get_param('~fwd_vel', 0.2)
        self.obstacle_range = rospy.get_param('~obstacle_range', 0.1)
        self.min_follow_threshold = 0.1
        self.max_follow_threshold = 0.3
        self.wall_side = True  # random.choice([True, False])
        self.max_arc_time = 5
        self.rate = rospy.Rate(rospy.get_param('~rate', 50))
        self.end_arc_time = -1
        self.twist = None
        self.stop = False
        self.goal_in_view = False
        self.end_arc_time = False
        self.last_ranges = {LEFT: None, CENTRE: None, RIGHT: None}
        rospy.Subscriber('ul_range', Float32, self.range_callback, callback_args=LEFT)
        rospy.Subscriber('uc_range', Float32, self.range_callback, callback_args=CENTRE)
        rospy.Subscriber('ur_range', Float32, self.range_callback, callback_args=RIGHT)
        rospy.Subscriber('/goal_detected', Bool, self.goal_callback)
        self.twist_pub = rospy.Publisher('twist', Twist, queue_size=10)
        self.motor_sleep_pub = rospy.Publisher('motor_sleep', Bool, queue_size=10)
        self.gled_pub = rospy.Publisher('green_led', Bool, queue_size=10)
        self.rled_pub = rospy.Publisher('red_led', Bool, queue_size=10)
        self.gled_flash_pub = rospy.Publisher('green_led_flash', Int32, queue_size=10)

    def spin(self):
        try:
            while not rospy.is_shutdown() and not self.termination_condition():
                self.publish_cmd()
                self.rate.sleep()
            self.terminate()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.twist_pub.publish(Twist())
            self.motor_sleep_pub.publish(True)

    def publish_cmd(self):
        if self.stop:
            return
        if any([r is None for r in self.last_ranges.values()]):
            # Still waiting for first reading from another sensor
            return
        outer = RIGHT if self.wall_side else LEFT
        inner = LEFT if self.wall_side else RIGHT
        if self.last_ranges[CENTRE] < self.obstacle_range or \
                self.last_ranges[outer] < self.obstacle_range or \
                self.last_ranges[inner] < self.min_follow_threshold:
            self.end_arc_time = -1
            if self.wall_side:
                self.turn_right()
            else:
                self.turn_left()
        elif self.last_ranges[inner] > self.max_follow_threshold and \
                (self.end_arc_time < 0 or time.time() < self.end_arc_time):
            if self.end_arc_time < 0:
                self.end_arc_time = time.time() + self.max_arc_time
            if self.wall_side:
                self.arc_left()
            else:
                self.arc_right()
        else:
            # self.wall_side = random.choice([True, False])
            self.forward()

    def forward(self):
        self.twist = Twist()
        space = min(self.last_ranges.values())
        breaking_distance = 0.2
        vel = self.fwd_vel if space > breaking_distance else self.fwd_vel * space / breaking_distance
        self.twist.linear.x = vel
        self._publish(gled=True)

    def turn_right(self):
        self.twist = Twist()
        self.twist.angular.z = -self.turn_vel
        self._publish(rled=True)

    def turn_left(self):
        self.twist = Twist()
        self.twist.angular.z = self.turn_vel
        self._publish(rled=True)

    def arc_right(self):
        self.twist = Twist()
        self.twist.angular.z = -0.6 * self.turn_vel
        self.twist.linear.x = 0.6 * self.fwd_vel
        self._publish(rled=True, gled=True)

    def arc_left(self):
        self.twist = Twist()
        self.twist.angular.z = 0.6 * self.turn_vel
        self.twist.linear.x = 0.6 * self.fwd_vel
        self._publish(rled=True, gled=True)

    def _publish(self, rled=False, gled=False, slp=False):
        self.twist_pub.publish(self.twist)
        self.motor_sleep_pub.publish(slp)
        self.gled_pub.publish(gled)
        self.rled_pub.publish(rled)

    def range_callback(self, msg, s):
        if msg.data > -1:
            self.last_ranges[s] = msg.data

    def goal_callback(self, msg):
        self.goal_in_view = msg.data

    def termination_condition(self):
        return self.goal_in_view
        # if self.last_ranges[CENTRE] is None:
        #     return False
        # return self.goal_in_view and self.last_ranges[CENTRE] < self.obstacle_range

    def terminate(self):
        # Turn on the spot for a bit, flash the LED, then shutdown
        self.gled_flash_pub.publish(5)
        self.stop = True
        twist = Twist()
        for i in range(10):
            stop_time = time.time() + 0.3
            while time.time() < stop_time:
                twist.angular.z = 1.5 * (self.turn_vel if i % 2 == 0 else -self.turn_vel)
                self.twist_pub.publish(twist)
                self.rate.sleep()
        self.gled_flash_pub.publish(0)
        self.twist_pub.publish(Twist())
        rospy.signal_shutdown("Found goal")


if __name__ == '__main__':
    controller = WallFollower()
    controller.spin()
