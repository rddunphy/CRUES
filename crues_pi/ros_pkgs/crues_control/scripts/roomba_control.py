#!/usr/bin/env python

import random
import time

import rospy
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist


LEFT = 0
CENTRE = 1
RIGHT = 2


m_pub = rospy.Publisher('twist', Twist, queue_size=10)
gl_pub = rospy.Publisher('green_led', Bool, queue_size=10)
rl_pub = rospy.Publisher('red_led', Bool, queue_size=10)
time_to_stop_turning = -1
turn = None
last_ranges = {LEFT: None, CENTRE: None, RIGHT: None}
turn_speed = 1
fwd_speed = 0.2
obstacle_range = 100


def forward():
    global time_to_stop_turning
    out = Twist()
    time_to_stop_turning = -1
    out.linear.x = fwd_speed
    gl_pub.publish(True)
    rl_pub.publish(False)
    m_pub.publish(out)


def turn_right():
    global time_to_stop_turning, turn
    turn = Twist()
    turn.angular.z = -turn_speed
    time_to_stop_turning = time.time() + random.uniform(0.2, 0.6)
    gl_pub.publish(False)
    rl_pub.publish(True)
    m_pub.publish(turn)


def turn_left():
    global time_to_stop_turning, turn
    turn = Twist()
    turn.angular.z = turn_speed
    time_to_stop_turning = time.time() + random.uniform(0.2, 0.6)
    gl_pub.publish(False)
    rl_pub.publish(True)
    m_pub.publish(turn)


def handle_new_range(msg, s):
    global time_to_stop_turning, turn
    last_ranges[s] = msg.data
    if any([r is None for r in last_ranges.values()]):
        # Still waiting for first reading from another sensor
        return
    if time_to_stop_turning > time.time():
        # Finish turning first
        m_pub.publish(turn)
        return
    else:
        if all([r > obstacle_range for r in last_ranges.values()]):
            # All clear, just go for it
            forward()
        elif last_ranges[LEFT] < last_ranges[RIGHT]:
            # Obstacle more on left, so turn right
            turn_right()
        else:
            # Obstacle more on right, so turn left
            turn_left()


def main():
    try:
        rospy.init_node('roomba')
        rospy.Subscriber('ul_range', Int32, handle_new_range, callback_args=LEFT)
        rospy.Subscriber('uc_range', Int32, handle_new_range, callback_args=CENTRE)
        rospy.Subscriber('ur_range', Int32, handle_new_range, callback_args=RIGHT)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException in node 'roomba'")
        rospy.logerr(e)


if __name__ == '__main__':
    main()
