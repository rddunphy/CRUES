#!/usr/bin/env python
import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    try:
        rospy.init_node('us_scan_tf_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        ts = TransformStamped()

        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = rospy.get_param('~base_frame_id', 'base_link')
        ts.child_frame_id = rospy.get_param('~scan_frame_id', 'us_scan_frame')

        ts.transform.translation.x = rospy.get_param('~x_offset', -44)
        ts.transform.translation.y = rospy.get_param('~y_offset', 0)
        ts.transform.translation.z = rospy.get_param('~z_offset', 0)

        ts.transform.rotation.x = 0
        ts.transform.rotation.y = 0
        ts.transform.rotation.z = 0
        ts.transform.rotation.w = 1

        rate = rospy.Rate(rospy.get_param('~rate', 50))
        while not rospy.is_shutdown():
            broadcaster.sendTransform(ts)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
