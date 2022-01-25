#!/usr/bin/env python
# -*- coding: utf-8 -*-

from waypoint import Waypoints

from enum import Enum
import math
import os
import cv2
import numpy as np
import requests

import rospy
import tf
import actionlib
import angles


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

class SeigoRun3:

    def __init__(self):
        self.get_rosparams()

    def get_rosparams(self):
        self.my_side = rospy.get_param('~side')
        self.robot_namespace = rospy.get_param('~robot_namespace')
        self.enemy_time_tolerance = rospy.get_param(
            '~detect_enemy_time_tolerance', default=0.5)
        self.snipe_th = rospy.get_param('~snipe_distance_th', default=0.8)
        self.distance_to_wall_th = rospy.get_param(
            '~distance_to_wall_th', default=0.15)
        self.counter_th = rospy.get_param('enemy_count_th', default=3)
        self.approach_distance_th = rospy.get_param(
            '~approach_distance_th', default=0.5)
        self.attack_angle_th = rospy.get_param(
            '~attack_angle_th', default=45*math.pi/180)
        self.camera_range_limit = rospy.get_param(
            '~camera_range_limit', default=[0.2, 0.5])
        self.camera_angle_limit = rospy.get_param(
            '~camera_angle_limit', default=30)*math.pi/180
        self.enable_escape_approach = rospy.get_param(
            '~enable_escape_approach', default="False")
        self.escape_approach_distance_th_min = rospy.get_param(
            '~escape_approach_distance_th_min', default=0.23)
        self.escape_approach_distance_th_max = rospy.get_param(
            '~escape_approach_distance_th_max', default=0.85)
        self.escape_approach_time_interval = rospy.get_param(
            '~escape_approach_time_interval', default=6)
        self.imu_linear_acceleration_z_th = rospy.get_param('~imu_linear_acceleration_z_th', default=20)

        self.judge_server_url = rospy.get_param('/send_id_to_judge/judge_url')

    def get_war_state(self):
        req = requests.get(self.judge_server_url+"/warState")
        dic = req.json()

        print(dic)

def main():
    rospy.init_node("seigo_run3")
    node = SeigoRun3()
    loop_rate = rospy.Rate(30) #30Hz
    while not rospy.is_shutdown():
        #some processes
        node.get_war_state()

        loop_rate.sleep()

if __name__ == "__main__":
    main()