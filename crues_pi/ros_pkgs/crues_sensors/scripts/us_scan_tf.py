#!/usr/bin/env python
import rospy

import tf2_ros
import geometry_msgs.msg


def main():
    rospy.init_node('us_scan_tf_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = 'base_link'
    static_transformStamped.child_frame_id = rospy.get_param('~scan_frame_id', 'us_scan_frame')

    static_transformStamped.transform.translation.x = rospy.get_param('~x_offset', -44)
    static_transformStamped.transform.translation.y = rospy.get_param('~y_offset', 0)
    static_transformStamped.transform.translation.z = rospy.get_param('~z_offset', 0)

    static_transformStamped.transform.rotation.x = 0
    static_transformStamped.transform.rotation.y = 0
    static_transformStamped.transform.rotation.z = 0
    static_transformStamped.transform.rotation.w = 1

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()


if __name__ == '__main__':
    main()
