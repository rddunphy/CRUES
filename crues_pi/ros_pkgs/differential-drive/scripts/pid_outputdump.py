#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32

out = []

def callback(msg):
    global out
    out += [msg.data]

def node():
    try:
        rospy.init_node("pid_outputdump")
        rospy.Subscriber("lwheel_vel", Float32, callback)
        rospy.spin()
    finally:
        with open("/home/crues/pid_dump_%s.txt" % time.ctime(time.time()).replace(" ", "_"), 'w') as f:
            f.write("\n".join([str(o) for o in out]))

if __name__ == "__main__":
    node()
