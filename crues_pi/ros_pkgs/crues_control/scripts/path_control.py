#!/usr/bin/env python
import math
import time

import rospy
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist


class Controller:
    def __init__(self):
        rospy.init_node('controller')
        self.turn_vel = rospy.get_param('~turn_vel', 1)
        self.fwd_vel = rospy.get_param('~fwd_vel', 0.2)
        self.rate = rospy.Rate(rospy.get_param('~rate', 50))
        path_file = rospy.get_param('~path_file', "")
        self.path = []
        if path_file.strip() != "":
            with open(path_file, 'r') as f:
                i = 1
                for line in f:
                    tokens = line.split()
                    if len(tokens) > 0:
                        if len(tokens) != 2:
                            raise IOError("Syntax error in path file: %d tokens in line %d" % (len(tokens), i))
                        if tokens[0] == "fwd":
                            self.path.append((self.fwd, float(tokens[1])))
                        elif tokens[0] == "turn":
                            self.path.append((self.turn, float(tokens[1])))
                        else:
                            raise IOError("Syntax error in path file: Unknown token %s in line %d" % (tokens[0], i))
                    i += 1
        self.startup_time = time.time()
        self.startup_delay = rospy.get_param('~startup_delay', 2)
        self.started = False
        self.time_to_end_manoeuvre = -1
        self.manoeuvre = None
        rospy.Subscriber('turn_deg_cmd', Float32, self.turn_callback)
        rospy.Subscriber('fwd_mm_cmd', Float32, self.fwd_callback)
        self.twist_pub = rospy.Publisher('twist', Twist, queue_size=10)
        self.gled_pub = rospy.Publisher('green_led', Bool, queue_size=10)
        self.rled_pub = rospy.Publisher('red_led', Bool, queue_size=10)
        self.rled_flash_pub = rospy.Publisher('red_led_flash', Int32, queue_size=10)

    def spin(self):
        self.rled_flash_pub.publish(4)
        while not rospy.is_shutdown():
            self.publish_cmd()
            self.rate.sleep()

    def publish_cmd(self):
        if time.time() < self.startup_time + self.startup_delay:
            self.rled_flash_pub.publish(4)
            self.twist_pub.publish(Twist())
            return
        if not self.started:
            self.rled_flash_pub.publish(0)
            self.started = True
        if self.time_to_end_manoeuvre > time.time():
            # Finish turning first
            self.twist_pub.publish(self.manoeuvre)
            return
        if self.path:
            cmd, f = self.path[0]
            cmd(f)
            del self.path[0]
        else:
            self.twist_pub.publish(Twist())
            self.rled_pub.publish(False)
            self.gled_pub.publish(False)

    def fwd(self, d):
        self.manoeuvre = Twist()
        self.manoeuvre.linear.x = self.fwd_vel
        duration = d / (self.fwd_vel * 1000)
        self.time_to_end_manoeuvre = time.time() + duration
        self.rled_pub.publish(False)
        self.gled_pub.publish(True)

    def turn(self, a):
        self.manoeuvre = Twist()
        self.manoeuvre.angular.z = -self.turn_vel if a > 0 else self.turn_vel
        duration = (a * math.pi) / (self.turn_vel * 180)
        self.time_to_end_manoeuvre = time.time() + duration
        self.rled_pub.publish(True)
        self.gled_pub.publish(False)

    def fwd_callback(self, msg):
        self.path.append((self.fwd, msg.data))

    def turn_callback(self, msg):
        self.path.append((self.turn, msg.data))


if __name__ == '__main__':
    try:
        controller = Controller()
        controller.spin()
    except rospy.ROSInterruptException:
        pass
