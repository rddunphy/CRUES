#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

from crues import us


def main():
    try:
        print "Running ultrasonic node"
        pub = rospy.Publisher('uc_range', Int32, queue_size=10)
        rospy.init_node('uc')
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            out = "Range: %d mm" % us.get_range(us.CENTRE)
            pub.publish(out)
            rate.sleep()
    except rospy.ROSInterruptException:
        print "ROSInterruptException in node 'uc'"


if __name__ == '__main__':
    main()
