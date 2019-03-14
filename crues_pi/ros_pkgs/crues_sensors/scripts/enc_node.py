#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

from crues import encoder


RATE = 10


def main():
    try:
        l_pub = rospy.Publisher('lwheel', Int16, queue_size=10)
        r_pub = rospy.Publisher('rwheel', Int16, queue_size=10)
        rospy.init_node('encoders')
        rate = rospy.Rate(RATE)
        l_enc, r_enc = encoder.get_encoders()
        while not rospy.is_shutdown():
            l_pub.publish(l_enc.count)
            r_pub.publish(r_enc.count)
            rate.sleep()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        encoder.cleanup()


if __name__ == '__main__':
    main()
