#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import Int32

from crues.us import Ultrasonic, UltrasonicTimeout


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


def publish_range(us, pub):
    try:
        r = us.get_range()
    except UltrasonicTimeout as e:
        rospy.logwarn(str(e))
    else:
        rospy.logdebug("%s ultrasonic node range: %d", us.name, r)
        pub.publish(r)


def main():
    rospy.init_node("ultrasonic")
    scan_increment = rospy.get_param('~scan_increment', 0.05)
    timeout = scan_increment * 0.9
    sensor_angle = rospy.get_param('~sensor_angle', 30)
    us_l = Ultrasonic("Left", rospy.get_param('pins/ult'), rospy.get_param('pins/ule'), timeout)
    us_c = Ultrasonic("Centre", rospy.get_param('pins/uct'), rospy.get_param('pins/uce'), timeout)
    us_r = Ultrasonic("Right", rospy.get_param('pins/urt'), rospy.get_param('pins/ure'), timeout)
    r = rospy.get_param('~rate', 5)
    try:
        pub_l = rospy.Publisher('ul_range', Int32, queue_size=10)
        pub_c = rospy.Publisher('uc_range', Int32, queue_size=10)
        pub_r = rospy.Publisher('ur_range', Int32, queue_size=10)
        rate = rospy.Rate(r)
        while not rospy.is_shutdown():
            start = time.time()
            publish_range(us_l, pub_l)
            time_remaining = start + scan_increment - time.time()
            if time_remaining > 0:
                time.sleep(time_remaining)
            publish_range(us_c, pub_c)
            time_remaining = start + 2 * scan_increment - time.time()
            if time_remaining > 0:
                time.sleep(time_remaining)
            publish_range(us_r, pub_r)
            # Additionally publish scan object for SLAM node?
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException in ultrasonic node")
    finally:
        us_l.cleanup()
        us_c.cleanup()
        us_r.cleanup()


if __name__ == '__main__':
    main()
