#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from crues_comms.msg import Comms
from crues.comms import listen, killServer




class Server:
    def __init__(self):
        rospy.init_node('server')
        self.pubs = {}
        self.sock = listen(self._msg_callback) # self._msg_callback :: msg -> void

    def spin(self):
        try:
            rospy.spin()
        finally:
            killServer(self.sock)

    def _msg_callback(self, msg):
        if msg[0] not in self.pubs:
            self.pubs[msg[0]] = rospy.Publisher("received_messages/%s" % msg[0], String,queue_size = 10)
        self.pubs[msg[0]].publish(msg[1])    


if __name__ == '__main__':
    try:
        server = Server()
        server.spin()
    except rospy.ROSInterruptException:
        pass
