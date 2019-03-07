#!/usr/bin/env python
import rospy
from crues_comms.msg import Comms
from crues.comms import send

def send_message(data):
    rospy.loginfo(rospy.get_caller_id() + "attempted to send " + data.data + " to " + data.destination)
    try:
        send(data.destination, data.data)
        rospy.loginfo(rospy.get_caller_id() + "successfully sent " + data.data  +" t o" + data.destination)
    except:
        send_message(data)

def client_node():
    rospy.init_node('client', anonymous=True)

    rospy.Subscriber("send_message", Comms, send_message)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    client_node()
