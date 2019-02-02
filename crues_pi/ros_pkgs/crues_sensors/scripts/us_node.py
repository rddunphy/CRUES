#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

from crues import us, pin_defs


def main():
    try:
        print "Running ultrasonic node"
        us.configure_gpio()
        pub = rospy.Publisher('uc_range', Int32, queue_size=10)
        rospy.init_node('uc')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("Ping")
            try:
                range = us.get_range(us.CENTRE)
            except us.UltrasonicTimout as e:
                rospy.logwarn(str(e))
            else:
                rospy.loginfo("UC Range: %d", range)
                pub.publish(range)
            finally:
               rate.sleep()
    except rospy.ROSInterruptException:
        print "ROSInterruptException in node 'uc'"
    finally:
        pin_defs.cleanup()


if __name__ == '__main__':
    main()
