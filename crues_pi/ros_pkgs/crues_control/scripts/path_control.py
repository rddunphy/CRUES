#!/usr/bin/env python
import math
import time

import rospy
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist


class Controller:
    def __init__(self):
        commands = {
            "fwd": self.fwd,
            "turn": self.turn,
            "fwd_vel": self.set_fwd_vel,
            "turn_vel": self.set_turn_vel,
            "wait": self.wait,
            "exit": self.exit
        }
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
                        cmd = tokens[0]
                        arg = float(tokens[1])
                        if cmd in commands:
                            self.path.append((commands[cmd], arg))
                        else:
                            raise IOError("Syntax error in path file: Unknown token %s in line %d" % (tokens[0], i))
                    i += 1
        self.startup_time = time.time()
        self.startup_delay = rospy.get_param('~startup_delay', 2)
        self.started = False
        self.shutdown = False
        self.time_to_end_manoeuvre = -1
        self.manoeuvre = None
        rospy.Subscriber('turn_cmd', Float32, self.cmd_callback, callback_args=self.turn)
        rospy.Subscriber('fwd_cmd', Float32, self.cmd_callback, callback_args=self.fwd)
        rospy.Subscriber('turn_vel', Float32, self.cmd_callback, callback_args=self.set_turn_vel)
        rospy.Subscriber('fwd_vel', Float32, self.cmd_callback, callback_args=self.set_turn_vel)
        rospy.Subscriber('wait_cmd', Float32, self.cmd_callback, callback_args=self.wait)
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
            # Finish current manoeuvre first
            self.twist_pub.publish(self.manoeuvre)
            return
        if self.shutdown:
            rospy.signal_shutdown("Shutdown command in path.")
        if self.path:
            cmd, f = self.path[0]
            cmd(f)
            del self.path[0]
            if cmd == self.set_fwd_vel or cmd == self.set_turn_vel:
                # Setting a velocity takes no time, so go straight on to next command
                self.publish_cmd()
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

    def set_fwd_vel(self, vel):
        self.fwd_vel = vel

    def set_turn_vel(self, vel):
        self.turn_vel = vel

    def wait(self, duration):
        self.manoeuvre = Twist()
        self.time_to_end_manoeuvre = time.time() + duration
        self.rled_pub.publish(False)
        self.gled_pub.publish(False)

    def exit(self, duration):
        self.wait(duration)
        self.shutdown = True

    def cmd_callback(self, msg, cmd):
        self.path.append((cmd, msg.data))


if __name__ == '__main__':
    try:
        controller = Controller()
        controller.spin()
    except rospy.ROSInterruptException:
        pass
