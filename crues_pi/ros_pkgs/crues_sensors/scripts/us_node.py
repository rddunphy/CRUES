#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import Int32

from crues import us, pin_defs


## VERSION FOR ONLY CENTRE ULTRASONIC SENSOR:
# def main():
#     try:
#         us.configure_gpio()
#         pub = rospy.Publisher('uc_range', Int32, queue_size=10)
#         rospy.init_node("uc")
#         rate = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             try:
#                 rospy.loginfo("Trying to get ultrasonic range from uc")
#                 range = us.get_range(us.CENTRE)
#             except us.UltrasonicTimout as e:
#                 rospy.logwarn(str(e))
#             else:
#                 rospy.loginfo("Range from uc: %d", range)
#                 pub.publish(range)
#             rate.sleep()
#         rospy.warn("uc thinks rospy.is_shutdown???")
#     except rospy.ROSInterruptException as e:
#         rospy.logerr("ROSInterruptException in node uc")
#         rospy.logerr(e)
#     finally:
#         pin_defs.cleanup()


def publish_range(s, pub):
    try:
        r = us.get_range(s)
    except us.UltrasonicTimout as e:
        rospy.logwarn(str(e))
    else:
        rospy.loginfo("%s ultrasonic node range: %d", (us.sensor_str.get(s), r))
        pub.publish(r)


def main():
    try:
        us.configure_gpio()
        pub_l = rospy.Publisher('ul_range', Int32, queue_size=10)
        pub_c = rospy.Publisher('uc_range', Int32, queue_size=10)
        pub_r = rospy.Publisher('ur_range', Int32, queue_size=10)
        rospy.init_node("ultrasonic")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            start = time.time()
            publish_range(us.LEFT, pub_l)
            time_remaining = start + us.scan_increment - time.time()
            if time_remaining > 0:
                time.sleep(time_remaining)
            publish_range(us.CENTRE, pub_c)
            time_remaining = start + 2 * us.scan_increment - time.time()
            if time_remaining > 0:
                time.sleep(time_remaining)
            publish_range(us.RIGHT, pub_r)
            # Additionally publish scan object for SLAM node?
            rate.sleep()
        rospy.logwarn("ultrasonic node thinks rospy.is_shutdown???")
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException in ultrasonic node")
        rospy.logerr(e)
    finally:
        pin_defs.cleanup()


if __name__ == '__main__':
    main()
