#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ast import Pass
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
from obstacle_detector.msg import Obstacles, SegmentObstacle, CircleObstacle

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

#      6(0.65, 0.5)                   8(0.65, -0.5)
#   [BLOCK]     14(0.25, 0.0)      [BLOCK]
#      7(0.4, 0.5)                   9(0.4, -0.5)
#          16(0.0, 0.25) [BLOCK] 15(0.0, -0.25)
# 10(-0.40, 0.5)                  12(-0.40, -0.5)
#   [BLOCK]     17(-0.25, 0.0)      [BLOCK]
# 11(-0.65, 0.5)                 13(-0.65, -0.5)
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



class SeigoRun3:

    def __init__(self):
        self.get_rosparams()
        
        self.game_timestamp = 0
        self.last_game_timestamp = 0
        self.my_score = 0
        self.enemy_score = 0
        self.Is_lowwer_score = False
        self.all_field_score = np.ones([18])  # field score state
        self.all_field_score_prev = np.ones(
            [18])  # field score state (previous)
        self.enemy_get_target_no = -1
        self.enemy_get_target_no_timestamp = -1
        self.my_get_target_no = -1
        self.my_get_target_no_timestamp = -1
        self.my_body_remain = 3
        self.enemy_body_remain = 3
        # self position estimation value
        self.my_pose_x = 1000              # init value
        self.my_pose_y = 1000              # init value
        self.my_direction_th = math.pi*100 # init value

        #敵検出の情報
        rospy.Subscriber("enemy_position", Odometry, self.enemy_position_callback)
        self.enemy_position = Odometry()

        #move_baseの準備
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not self.move_base_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo("move_baseサーバの待機中")
        rospy.loginfo("move_baseサーバ起動")
        self.status = self.move_base_client.get_state()
        
        #clear_costmapサーバの準備
        rospy.wait_for_service("/move_base/clear_costmaps")
        self.clear_costmap = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        #waypointファイルの読み込み
        self.waypoint = self.load_waypoint()
        
        #tfのlistenerとbroadcasterの生成
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

    def enemy_position_callback(self, msg):
        self.enemy_position = msg
        

    def process(self):
        self.send_goal_to_move_base(self.waypoint.get_current_waypoint())
        move_base_status = self.move_base_client.get_state()
        
        #ゴールしたら
        if move_base_status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[seigoRun3]Go to next waypoint")
            self.send_goal_to_move_base(self.waypoint.get_next_waypoint())

        else:
            rospy.loginfo("[seigRun3]move_base_status:"+str(move_base_status))

    def get_position_from_tf(self, target_link, base_link):
        trans = []
        rot = []
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                target_link, base_link, rospy.Time(0))
            return trans, rot, True
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('tf error')
            return trans, rot, False

    def send_goal_to_move_base(self, waypoint):
        rospy.loginfo("[seigoRun3]コストマップをクリアします")
        self.clear_costmap.call()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.robot_namespace+"map"
        goal.target_pose.pose.position.x = waypoint[0]
        goal.target_pose.pose.position.y = waypoint[1]

        q = tf.transformations.quaternion_from_euler(0, 0, waypoint[2])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        goal.target_pose.header.stamp = rospy.Time.now()
        
        self.move_base_client.send_goal(goal)
        rospy.loginfo("[seigoRun3]move_baseサーバにwaypointを送信しました")
        rospy.sleep(0.5)

    def load_waypoint(self):
        file_path = os.environ["HOME"] + \
            '/catkin_ws/src/burger_war_dev/burger_war_dev/scripts/waypoints_20211126.csv'
        return Waypoints(file_path, self.my_side)

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
        #rospy.loginfo("[seigoRun3]Get war_state")
        req = requests.get(self.judge_server_url+"/warState")
        dic = req.json()

        #scoreの取得
        self.my_score = int(dic["scores"][self.my_side])
        if self.my_side == "r":
            self.enemy_score = int(dic["scores"]["b"])
        else:
            self.enemy_score = int(dic["scores"]["r"])

        #time_stampの取得
        self.game_timestamp = int(dic["time"])
        if dic["state"] == "running":
            self.last_game_timestamp = self.game_timestamp

        #フィールドのスコアを取得する
        for idx in range(18): # フィールドターゲットの数は18個
            self.all_field_score_prev[idx] = self.all_field_score[idx] #一つ前のフレームでのスコアを格納
            target_state = dic["targets"][idx]["player"] #ターゲットを取得しているプレイヤーを取得

            if target_state == "n": #自分も敵もターゲットを取得していない
                self.all_field_score[idx] = 1
            elif target_state == self.my_side: #自分がターゲットを取得している
                self.all_field_score[idx] = 0
            else: #的がターゲットを取得している
                self.all_field_score[idx] = 2

            if self.all_field_score[idx] != self.all_field_score_prev[idx]: #ターゲット情報に更新があるかどうか
                if self.all_field_score[idx] == 2: #敵がターゲットを取得した
                    if self.all_field_score_prev[idx] == 1:
                        print("[seigoRun3]敵がID:"+str(idx)+"のターゲットを取得した")
                    elif self.all_field_score_prev[idx] == 0:
                        print("[seigoRun3]敵にID:"+str(idx)+"のターゲットを奪われた")
                    else:
                        pass
                
                elif self.all_field_score[idx] == 1: #自分がターゲットを取得した
                    if self.all_field_score_prev[idx] == 2: 
                        print("[seigoRun3]敵が取得したID："+str(idx)+"のターゲットを奪った")
                    elif self.all_field_score_prev[idx] == 1:
                        print("[seigoRun3]ID:"+str(idx)+"のターゲットを取得した")
                    else:
                        pass
        #フィールドスコアの更新終了

        #ボディーマーカーのスコアを更新
        if self.my_side == "b":
            self.my_body_remain = np.sum(self.all_field_score[0:3])
            self.enemy_body_remain = np.sum(self.all_field_score[3:6])
        elif self.my_side == "r":
            self.my_body_remain = np.sum(self.all_field_score[3:6])
            self.enemy_body_remain = np.sum(self.all_field_score[0:3])
        
        #print("----------")
        #for i in range(6,18): #6~17
        #    print(str(i), str(self.all_field_score[i])    
        #print("----------")

        #-------------------------------------------#
        # 現在のスコアや敵ロボットとの関係から戦略を決定する
        #
        # ：決められたルートを巡回
        # ：一番近くにあるマーカを取得する
        # ：敵に正対する
        # ：障害物の影に隠れる
        #-------------------------------------------#
        def patrol(self):
            pass

def main():
    rospy.init_node("seigo_run3")
    rospy.loginfo("[seigoRun3]seigoRun3 is running")
    node = SeigoRun3()
    loop_rate = rospy.Rate(30) #30Hz
    while not rospy.is_shutdown():
        #some processes
        node.get_war_state()

        #node.process()
        
        loop_rate.sleep()

if __name__ == "__main__":
    main()