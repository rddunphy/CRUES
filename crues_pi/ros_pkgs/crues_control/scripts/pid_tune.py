#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist


Ku = 0
Tu = 0


def publish(kp_pub, ki_pub, kd_pub):
    kp_pub.publish(Ku / 5)
    ki_pub.publish((2 * Ku) / (5 * Tu))
    kd_pub.publish((Ku * Tu) / 15)


def handle_ku(msg, (kp_pub, ki_pub, kd_pub)):
    global Ku
    Ku = msg.data
    publish(kp_pub, ki_pub, kd_pub)


def handle_tu(msg, (kp_pub, ki_pub, kd_pub)):
    global Tu
    Tu = msg.data
    publish(kp_pub, ki_pub, kd_pub)


def main():
    try:
        rospy.init_node('pid_tune')
        kp_pub = rospy.Publisher('Kp', Float32, queue_size=10)
        ki_pub = rospy.Publisher('Ki', Float32, queue_size=10)
        kd_pub = rospy.Publisher('Kd', Float32, queue_size=10)
        rospy.Subscriber('Ku', Float32, handle_ku, callback_args=(kp_pub, ki_pub, kd_pub))
        rospy.Subscriber('Tu', Float32, handle_tu, callback_args=(kp_pub, ki_pub, kd_pub))
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException in node 'roomba'")
        rospy.logerr(e)


if __name__ == '__main__':
    main()
