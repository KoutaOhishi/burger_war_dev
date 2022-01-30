#!/usr/bin/env python
# -*- coding: utf-8 -*-

#http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20%28Python%29

import rospy
import tf, tf2_ros
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped

# --- target definition (r), refer http://localhost:5000/warstate ---
# the number means index of warstate json file.
# the target state is stored in all_field_score param (0:no one,1:mybot,2:enemy)
#
#      6                   8
#   [BLOCK]     14      [BLOCK]
#      7                   9
#          16 [BLOCK] 15
#      10                  12
#   [BLOCK]     17      [BLOCK]
#      11                  13
#
#  coordinate systemn
#            ^ X  blue bot
#            |
#            |
#     Y <----|-----
#            |
#            |
#            |    red bot
#
# ----------------------------------------
#        Back 0                  Back 3
#   R 2[enemy_bot(b)]L 1   R 5[my_bot(r)]L 4
#        Front                   Front
# ----------------------------------------
#  ↑ qx:0.00, qy:0.00, qz:0.00, qw:1.00
#  ↓ qx:0.00, qy:0.00, qz:1.00, qw:0.00
#  → qx:0.00, qy:0.00, qz:-0.707, qw:0.707
#  ← qx:0.00, qy:0.00, qz:0.707, qw:0.707
#  ↗︎ qx:0.00, qy:0.00, qz:-0.38, qw:0.92
#  ↘️ qx:0.00, qy:0.00, qz:0.92, qw:-0.38
#  ↙️ qx:0.00, qy:0.00, qz:0.92, qw:0.38
#  ↖️ qx:0.00, qy:0.00, qz:0.38, qw:0.92
# ----------------------------------------

# marker_idx, [x,y,z, qx, qy, qz, qw]
target_markers = [
    [ 0.00,  0.00, 0.00, 0.00, 0.00,  0.00,  0.00], # 0 blue_bot Back
    [ 0.00,  0.00, 0.00, 0.00, 0.00,  0.00,  0.00], # 1 blue_bot Right
    [ 0.00,  0.00, 0.00, 0.00, 0.00,  0.00,  0.00], # 2 blue_bot Left
    [ 0.00,  0.00, 0.00, 0.00, 0.00,  0.00,  0.00], # 3 red_bot Back
    [ 0.00,  0.00, 0.00, 0.00, 0.00,  0.00,  0.00], # 4 red_bot Left
    [ 0.00,  0.00, 0.00, 0.00, 0.00,  0.00,  0.00], # 5 red_bot Right
    [ 0.85,  0.50, 0.00, 0.00, 0.00,  1.00,  0.00], # 6
    [ 0.25,  0.50, 0.00, 0.00, 0.00,  0.00,  1.00], # 7
    [ 0.85, -0.50, 0.00, 0.00, 0.00,  1.00,  0.00], # 8
    [ 0.25, -0.50, 0.00, 0.00, 0.00,  0.00,  1.00], # 9
    [-0.25,  0.50, 0.00, 0.00, 0.00,  1.00,  0.00], # 10
    [-0.85,  0.50, 0.00, 0.00, 0.00,  0.00,  1.00], # 11
    [-0.25, -0.50, 0.00, 0.00, 0.00,  1.00,  0.00], # 12
    [-0.85, -0.50, 0.00, 0.00, 0.00,  0.00,  1.00], # 13
    [ 0.50,  0.00, 0.00, 0.00, 0.00,  1.00,  0.00], # 14
    [ 0.00, -0.50, 0.00, 0.00, 0.00, 0.707, 0.707], # 15
    [ 0.00,  0.50, 0.00, 0.00, 0.00,-0.707, 0.707], # 16
    [-0.50,  0.00, 0.00, 0.00, 0.00,  0.00,  1.00]  # 17
]

check_point_markers = [
    [ 1.00,  0.00, 0.00, 0.00, 0.00,  1.00,  0.00],
    [ 0.75, -0.75, 0.00, 0.00, 0.00,  0.92, -0.38],
    [ 0.00, -1.00, 0.00, 0.00, 0.00, 0.707,  0.707],
    [-0.75, -0.75, 0.00, 0.00, 0.00,  0.92,  0.38],
    [-1.00,  0.00, 0.00, 0.00, 0.00,  0.00,  1.00],
    [-0.75,  0.75, 0.00, 0.00, 0.00,  0.38,  0.92],
    [ 0.00,  1.00, 0.00, 0.00, 0.00,-0.707,  0.707],
    [ 0.75,  0.75, 0.00, 0.00, 0.00, -0.38,  0.92]
]

broadcaster = tf2_ros.StaticTransformBroadcaster()

def tf_static_broadcaster():
    rospy.init_node("tf_static_broadcaster")
    rospy.loginfo("[tf_static_broadcaster]マーカの座標をbroadcastします")

    transform_stamped_list = []

    for i in range(len(target_markers)):
        if i <= 5:
            #0~5はロボットについてるマーカーなので、static broadcastはしない
            pass
        
        else:
            transform_stamped = TransformStamped()

            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = "map"
            transform_stamped.child_frame_id = "target_"+str(i)

            transform_stamped.transform.translation.x = float(target_markers[i][0])
            transform_stamped.transform.translation.y = float(target_markers[i][1])
            transform_stamped.transform.translation.z = float(target_markers[i][2])

            transform_stamped.transform.rotation.x = float(target_markers[i][3])
            transform_stamped.transform.rotation.y = float(target_markers[i][4])
            transform_stamped.transform.rotation.z = float(target_markers[i][5])
            transform_stamped.transform.rotation.w = float(target_markers[i][6])

            transform_stamped_list.append(transform_stamped)
            #print("idx:"+str(i)+"  "+str(target_markers[i]))
            
    for i in range(len(check_point_markers)):
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "check_point_"+str(i)

        transform_stamped.transform.translation.x = float(check_point_markers[i][0])
        transform_stamped.transform.translation.y = float(check_point_markers[i][1])
        transform_stamped.transform.translation.z = float(check_point_markers[i][2])

        transform_stamped.transform.rotation.x = float(check_point_markers[i][3])
        transform_stamped.transform.rotation.y = float(check_point_markers[i][4])
        transform_stamped.transform.rotation.z = float(check_point_markers[i][5])
        transform_stamped.transform.rotation.w = float(check_point_markers[i][6])

        transform_stamped_list.append(transform_stamped)
        print("idx:"+str(i)+"  "+str(target_markers[i]))

    broadcaster.sendTransform(transform_stamped_list)
    rospy.spin()

if __name__ == "__main__":
    tf_static_broadcaster()
