#!/usr/bin/env python
# -*- coding: utf-8 -*-

#http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28Python%29

import rospy
import tf, tf2_ros
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped

# marker_idx, [x,y]
target_markers = [
    [ 0.00,  0.00], # 0 blue_bot Back
    [ 0.00,  0.00], # 1 blue_bot Right
    [ 0.00,  0.00], # 2 blue_bot Left
    [ 0.00,  0.00], # 3 red_bot Back
    [ 0.00,  0.00], # 4 red_bot Left
    [ 0.00,  0.00], # 5 red_bot Right
    [ 0.65,  0.50], # 6
    [ 0.40,  0.50], # 7
    [ 0.65, -0.50], # 8
    [ 0.40, -0.50], # 9
    [-0.40,  0.50], # 10
    [-0.65,  0.50], # 11
    [-0.40, -0.50], # 12
    [-0.65, -0.50], # 13
    [ 0.25,  0.00], # 14
    [ 0,00, -0.25], # 15
    [ 0.00,  0.25], # 16
    [-0.25,  0.00]  # 17
]

def tf_static_broadcaster():
    rospy.init_node("tf_static_broadcaster")
    rospy.loginfo("[tf_static_broadcaster]マーカの座標をbroadcastします")

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    for i in range(len(target_markers)):
        if i in [0,1,2,3,4,5]:
            #1~5はロボットについてるマーカーなので、broadcastはしない
            pass
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "target_"+str(i)
        transform_stamped.child_frame_id = "map"

        transform_stamped.transform.translation.x = float(target_markers[i][0])
        transform_stamped.transform.translation.y = float(target_markers[i][1])
        transform_stamped.transform.translation.z = 0.00

        euler = tf.transformations.quaternion_from_euler(0.00, 0.00, 0.00)
        transform_stamped.transform.rotation.x = euler[0] #x
        transform_stamped.transform.rotation.y = euler[1] #y
        transform_stamped.transform.rotation.z = euler[2] #z
        transform_stamped.transform.rotation.w = euler[3] #w

        broadcaster.sendTransform(transform_stamped)
        print("[tf_static_broadcaster]target_"+str(i)+" frame is broadcasted.")

    rospy.spin()

if __name__ == "__main__":
    tf_static_broadcaster()
