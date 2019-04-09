#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from comms import listen, kill_server


class Server:
    def __init__(self):
        rospy.init_node('server')
        self.pubs = {}
        self.hostname = rospy.get_param('hostname')
        robots = rospy.get_param('robots')
        for robot in robots:
            if robot["name"] == self.hostname:
                self.ip = robot["ip"]
        self.sock = listen(self._msg_callback, self.ip)  # self._msg_callback :: msg -> void

    def spin(self):
        try:
            rospy.spin()
        finally:
            kill_server(self.sock)

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
