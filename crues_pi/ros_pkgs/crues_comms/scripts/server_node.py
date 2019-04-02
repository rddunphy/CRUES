#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from crues.comms import listen, killServer


class Server:
    def __init__(self):
        rospy.init_node('server')
        self.pub = rospy.Publisher('received_message', String, queue_size=10)  # Should this be a Comms message?
        self.sock = listen(self._msg_callback)

    def spin(self):
        try:
            rospy.spin()
        finally:
            killServer(self.sock)

    def _msg_callback(self, msg):
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        server = Server()
        server.spin()
    except rospy.ROSInterruptException:
        pass
