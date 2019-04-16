#!/usr/bin/env python
import rospy

from crues_comms.msg import Comms
from crues_sensors.msg import Vision

from comms import send


class Client:
    def __init__(self):
        rospy.init_node('client', anonymous=True)  # Why anonymous?
        rospy.Subscriber('send_message', Comms, self._send_callback)
        rospy.Subscriber('robots_detected', Vision, self._robots_detected_callback)
        self._msg_queue = []
        self._robots_in_view = []
        robots = rospy.get_param('robots')
        self.hostname = rospy.get_param('hostname')
        self._ip_addresses = {}
        for robot in robots:
            if robot["name"] != self.hostname:
                self._ip_addresses[robot["name"]] = robot["ip"]
        self.rate = rospy.Rate(rospy.get_param('~rate', 50))

    def spin(self):
        while not rospy.is_shutdown():
            self._send()
            self.rate.sleep()

    def _send(self):
        # This blocks, so should maybe be on a new thread?
        while self._msg_queue:
            self._send_next()

    def _send_next(self):
        msg = self._msg_queue[0]
        if msg.destination not in self._robots_in_view:
            rospy.logwarn("%s attempted to send %s to %s, but robot was not in view" %
                          (rospy.get_caller_id(), msg.data, msg.destination))
            del self._msg_queue[0]
        else:
            rospy.loginfo("%s attempting to send %s to %s" % (rospy.get_caller_id(), msg.data, msg.destination))
            try:
                send(msg.destination, self._ip_addresses[msg.destination], self.hostname, msg.topic, msg.data)
                del self._msg_queue[0]
                rospy.loginfo("%s successfully sent %s to %s" % (rospy.get_caller_id(), msg.data, msg.destination))
            except:  # What's this catching?
                # Possibly have a limit for attempts - we don't want to loop forever.
                pass

    def _send_callback(self, msg):
        self._msg_queue.append(msg)

    def _robots_detected_callback(self, msg):
        self._robots_in_view = [s.strip() for s in msg.robot_list.split(",")]


if __name__ == '__main__':
    try:
        client = Client()
        client.spin()
    except rospy.ROSInterruptException:
        pass
