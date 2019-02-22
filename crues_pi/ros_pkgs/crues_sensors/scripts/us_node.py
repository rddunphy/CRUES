#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

from crues import us, pin_defs


def main():
    try:
        us.configure_gpio()
        pub = rospy.Publisher('uc_range', Int32, queue_size=10)
        rospy.init_node("uc")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Trying to get ultrasonic range from uc")
                range = us.get_range(us.CENTRE)
            except us.UltrasonicTimout as e:
                rospy.logwarn(str(e))
            else:
                rospy.loginfo("Range from uc: %d", range)
                pub.publish(range)
            rate.sleep()
        rospy.warn("uc thinks rospy.is_shutdown???")
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROSInterruptException in node uc")
        rospy.logerr(e)
    finally:
        pin_defs.cleanup()

#
# def get_args():
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--sid", type=str, default="centre")
#     args = parser.parse_args()
#     if (args.sid == "left"):
#         sensor = us.LEFT
#         node_name = "ul"
#     elif (args.sid == "right"):
#         sensor = us.RIGHT
#         node_name = "ur"
#     else:
#         sensor = us.CENTRE
#         node_name = "uc"
#     return sensor, node_name


if __name__ == '__main__':
    main()
