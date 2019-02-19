#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

from crues import us, pin_defs


def main():
    try:
        us.configure_gpio()
        pub = rospy.Publisher('uc_range', Int32, queue_size=10)
        rospy.init_node('uc')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Trying to get ultrasonic range")
                range = us.get_range(us.CENTRE)
            except us.UltrasonicTimout as e:
                rospy.logwarn(str(e))
            else:
                rospy.loginfo("UC Range: %d", range)
                pub.publish(range)
            rate.sleep()
            rospy.loginfo("Yawn, that was a nice sleep.")
        rospy.warn("uc thinks rospy.is_shutdown???")
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException in node 'uc'")
        rospy.logerr(e)
    finally:
        pin_defs.cleanup()


if __name__ == '__main__':
    main()
