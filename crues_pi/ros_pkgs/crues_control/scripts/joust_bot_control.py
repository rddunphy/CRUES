#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32, Int32


target = 0.0


def handle_new_target(msg):
    global target
    target = msg.data


def main():
    try:
        rospy.init_node('joust_bot')
        rospy.Subscriber('vinput', Float32, handle_new_target)
        l_pub = rospy.Publisher('lmotor_cmd', Float32)
        r_pub = rospy.Publisher('rmotor_cmd', Float32)
        rate = rospy.Rate(rospy.get_param("~rate", 50))
        while not rospy.is_shutdown():
            l_pub.publish(target)
            r_pub.publish(target)
            rate.sleep()
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException in node 'vel_control'")
        rospy.logerr(e)


if __name__ == '__main__':
    main()
