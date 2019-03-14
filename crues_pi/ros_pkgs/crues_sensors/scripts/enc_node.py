#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

from crues.encoder import Encoder


def main():
    rospy.init_node('encoder')
    pin_a = rospy.get_param("~pin_a")
    pin_b = rospy.get_param("~pin_b")
    f = rospy.get_param("~rate", 10)
    pub = rospy.Publisher('wheel', Int16, queue_size=10)
    rate = rospy.Rate(f)
    enc = Encoder(pin_a, pin_b)
    try:
        while not rospy.is_shutdown():
            pub.publish(enc.count)
            rate.sleep()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        enc.cleanup()


if __name__ == '__main__':
    main()
