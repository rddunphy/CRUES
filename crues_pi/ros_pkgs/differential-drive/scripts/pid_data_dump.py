#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32

out = []


def callback(msg):
    global out
    out += [msg.data]


def node():
    rospy.init_node("pid_data_dump")
    left = rospy.get_param("~left")
    try:
        if left:
            rospy.Subscriber("lwheel_vel", Float32, callback)
        else:
            rospy.Subscriber("rwheel_vel", Float32, callback)
        rospy.spin()
    finally:
        timestamp = time.ctime(time.time()).replace(" ", "_").replace(":", "_")
        wheel_str = "left" if left else "right"
        with open("/home/crues/pid_dump_%s_%s.txt" % (wheel_str, timestamp), 'w') as f:
            f.write("\n".join([str(o) for o in out]))


if __name__ == "__main__":
    node()
