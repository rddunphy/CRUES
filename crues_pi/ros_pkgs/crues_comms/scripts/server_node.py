#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from crues.comms import listen, killServer

def server_node():
    try:
        pub = rospy.Publisher('received_message', String, queue_size=10)
        rospy.init_node('server') #, anonymous=True)
        #rate = rospy.Rate(10) # 10hz
        def pubHandler(self, message):
            pub.publish(message)

        #hello_str = "hello world %s" % rospy.get_time()
        sock = listen(pubHandler)
        rospy.spin()
    
    except rospy.ROSInterruptException:
        killServer(sock)
    #rospy.loginfo(message)
    #pub.publish(message)
    #rate.sleep()

if __name__ == '__main__':
    server_node()
