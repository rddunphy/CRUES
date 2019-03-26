#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32, Int32


target = 0.0


def handle_new_target(msg):
    global target
    target = msg.data


def main():
    try:
        rospy.init_node('vel_control')
        rospy.Subscriber('wheel_vinput', Float32, handle_new_target)
        pub = rospy.Publisher('wheel_vtarget', Float32)
        rate = rospy.Rate(rospy.get_param("~rate", 50))
        while not rospy.is_shutdown():
            pub.publish(target)
            rate.sleep()
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException in node 'vel_control'")
        rospy.logerr(e)


if __name__ == '__main__':
    main()
