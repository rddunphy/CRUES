#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from crues.comms import listen

def server_node():
    pub = rospy.Publisher('received_message', String, queue_size=10)
    rospy.init_node('server') #, anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
	message = listen()
        rospy.loginfo(message)
        pub.publish(message)
        #rate.sleep()

if __name__ == '__main__':
    try:
        server_node()
    except rospy.ROSInterruptException:
        pass
