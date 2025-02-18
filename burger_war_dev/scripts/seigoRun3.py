#!/usr/bin/env python
# -*- coding: utf-8 -*-

from email.mime import base
from lib2to3.pgen2.token import BACKQUOTE
from pickle import STOP
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
from geometry_msgs.msg import Twist, PointStamped
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

# ----------------------------------------
# actionlib.GoalStatus
#  uint8 PENDING         = 0  
#  uint8 ACTIVE          = 1 
#  uint8 PREEMPTED       = 2
#  uint8 SUCCEEDED       = 3
#  uint8 ABORTED         = 4
#  uint8 REJECTED        = 5
#  uint8 PREEMPTING      = 6
#  uint8 RECALLING       = 7
#  uint8 RECALLED        = 8
#  uint8 LOST            = 9
# ----------------------------------------

# ----------------------------------------
# strategy_dicisionの返り値
FIRST_MOVE = 0
PATROL     = 1
LEAVE      = 2
HIDE       = 3
FACE       = 4
ATTACK     = 5
RUN        = 6
# ----------------------------------------

class SeigoRun3:

    def __init__(self):
        self.get_rosparams()
        
        self.my_score = 0
        self.enemy_score = 0
        self.all_field_score = np.ones([18])  # field score state
        self.all_field_score_prev = self.all_field_score
        self.my_body_remain = []
        self.enemy_body_remain = []
        
        self.first_move_did = False

        #敵検出の情報
        rospy.Subscriber("enemy_position", Odometry, self.enemy_position_callback)
        self.enemy_position = Odometry()
        self.enemy_info = [0.0, 0.0]
        self.detect_counter = 0

        #コリジョン判定用にscanトピックをsubscribe
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.scan = LaserScan()

        #debug用
        rospy.Subscriber("clicked_point", PointStamped, self.clicked_point_callback)


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

        #直接車輪に制御命令を送るpublisherの生成
        self.direct_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # war_stateの定期更新する
        rospy.Timer(rospy.Duration(1), self.get_war_state)

    def enemy_position_callback(self, msg):
        self.enemy_position = msg

    def scan_callback(self, msg):
        self.scan = msg

    def clicked_point_callback(self, msg):
        unaquired_target_idx_list = []
        all_field_score = self.all_field_score #最新のフィールドスコア状況を取得
        
        for idx in range(6, 18): #全てのターゲットに対して、誰が取っているかを確認
            # idx 0~5はロボットについているマーカーなので無視

            if all_field_score[idx] == 0:
                pass #自分が取得しているのでパス

            else:
                unaquired_target_idx_list.append(idx)

        # 未取得のターゲット（ロボットについているものは除く）が無い場合
        if len(unaquired_target_idx_list) == 0:
            print("[seigoRun3:get_farthest_unaquired_target_idx]取得可能なターゲットは0個です")
            return -1 
        
        dist_between_target_list = []
        """if isEnemy == False:
            base_frame_name = "base_link"
        
        elif isEnemy == True:
            base_frame_name = self.robot_namespace + "/enemy_closest"
        """

        map_frame_name   = self.robot_namespace + "/map"
        self.tf_broadcaster.sendTransform((msg.point.x, msg.point.y, 0), (0,0,0,1), rospy.Time.now(), "clicked_point", map_frame_name)

        check_point_idx_list = [0,2,4,6]#[0,1,2,3,4,5,6,7]
        dist_list = []
        for idx in range(len(check_point_idx_list)):
            target_frame_name = "check_point_"+str(check_point_idx_list[idx])
            source_frame_name = "clicked_point"
            
            try:
                self.tf_listener.waitForTransform(source_frame_name, target_frame_name, rospy.Time(0), rospy.Duration(1.0))
                (trans,rot) = self.tf_listener.lookupTransform(source_frame_name, target_frame_name, rospy.Time(0))
                dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2)

            except Exception as e:
                rospy.logwarn("Except:[seigoRun3:leave]")
                rospy.logwarn(str(e))
                dist = 0.00
            
            dist_list.append(dist)
            print("[seigoRun3:leave]check_point_"+str(check_point_idx_list[idx])+"までの距離："+str(dist))
        
        farthest_check_point_idx = check_point_idx_list[dist_list.index(max(dist_list))]
        print("[seigoRun3:leave]敵から最も遠くにあるcheck_point_"+str(farthest_check_point_idx)+"への移動を開始します")
    
        #チェックポイントに移動開始
        self.send_goal_pose_of_checkPoint_by_idx(farthest_check_point_idx)
    
    def detect_enemy(self):
        print("[seigoRun:detect_enemy]敵検出スタート")
        base_frame_name = "base_link"
        enemy_frame_name = self.robot_namespace + "/enemy_closest"
        trans, rot, res = self.get_position_from_tf(enemy_frame_name, base_frame_name)

        if res == False:
            print("[seigoRun3:detect_enemy]enemy_closestを座標変換できません")
            return False, 0.0, 0.0
        
        enemy_dist = math.sqrt(trans[0]*trans[0]+trans[1]*trans[1])
        enemy_dire = math.atan2(trans[1], trans[0])

        self.enemy_position

        return True, enemy_dist, enemy_dire


    def get_position_from_tf(self, target_link, base_link):
        trans = []
        rot = []
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                base_link, target_link, rospy.Time(0))
            return trans, rot, True
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('tf error')
            return trans, rot, False

    def send_goal_pose_of_target_by_idx(self, target_idx):
        #ターゲットのインデックスを引数として渡すとそこまでのGoalPoseをmovebase serverに送ります
        loop_count = 0
        
        if target_idx < 6:
            print("[seigoRun3]0~5のインデックスは受け付けない")
            return False

        else:
            target_link = "target_"+str(target_idx)
            base_link = self.robot_namespace+"map"
            res = False
            
            while not rospy.is_shutdown():
                trans, rot, res = self.get_position_from_tf(target_link, base_link)
                if res == False:
                    print("ターゲット_"+str(target_idx)+"の座標変換に失敗しました")
                    loop_count += 1
                    rospy.sleep(1)

                elif loop_count >= 10:
                    print("[seigoRun3:send_goal_pose_of_tar...]10秒経っても座標変換できないので移動できません")
                    break

                else:
                    goal_pose = Pose()
                    goal_pose.position.x = trans[0]
                    goal_pose.position.y = trans[1]
                    goal_pose.position.z = trans[2]
                    goal_pose.orientation.x = rot[0]
                    goal_pose.orientation.y = rot[1]
                    goal_pose.orientation.z = rot[2]
                    goal_pose.orientation.w = rot[3]
                    #print("goal_pose "+str(goal_pose))
                    self.send_goal_to_move_base(goal_pose)
                    break
            
            return res
    
    def send_goal_pose_of_checkPoint_by_idx(self, idx):
        #チェックポイントのインデックスを引数として渡すとそこまでのGoalPoseをmovebase serverに送ります
        loop_count = 0
        
        if idx < 0 or 8 < idx:
            print("[seigoRun3]0~8以外のインデックスは受け付けない")
            return False

        else:
            check_point_link = "check_point_"+str(idx)
            base_link = self.robot_namespace+"map"
            res = False
            
            while not rospy.is_shutdown():
                trans, rot, res = self.get_position_from_tf(check_point_link, base_link)
                if res == False:
                    print("check_point_"+str(idx)+"の座標変換に失敗しました")
                    loop_count += 1
                    rospy.sleep(1)

                elif loop_count >= 10:
                    print("[seigoRun3:send_goal_pose_of_check...]10秒経っても座標変換できないので移動できません")
                    break

                else:
                    goal_pose = Pose()
                    goal_pose.position.x = trans[0]
                    goal_pose.position.y = trans[1]
                    goal_pose.position.z = trans[2]
                    goal_pose.orientation.x = rot[0]
                    goal_pose.orientation.y = rot[1]
                    goal_pose.orientation.z = rot[2]
                    goal_pose.orientation.w = rot[3]
                    #print("goal_pose "+str(goal_pose))
                    self.send_goal_to_move_base(goal_pose)
                    break
            
            return res

    def send_goal_to_move_base(self, arg):
        #rospy.loginfo("[seigoRun3]コストマップをクリアします")
        self.clear_costmap.call()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.robot_namespace+"map"
        goal.target_pose.header.stamp = rospy.Time.now()

        if type(arg) == Pose:
            goal.target_pose.pose = arg

        else: #引数の型はwaypoint
            goal.target_pose.pose.position.x = arg[0]
            goal.target_pose.pose.position.y = arg[1]

            q = tf.transformations.quaternion_from_euler(0, 0, arg[2])
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
        
        
        self.move_base_client.send_goal(goal)
        #rospy.loginfo("[seigoRun3]move_baseサーバにwaypointを送信しました")
        rospy.sleep(0.5)

    def load_waypoint(self):
        file_path = os.environ["HOME"] + \
            '/catkin_ws/src/burger_war_dev/burger_war_dev/scripts/waypoints_20211126.csv'
        return Waypoints(file_path, self.my_side)

    def get_rosparams(self):
        self.my_side = rospy.get_param('~side')
        self.robot_namespace = rospy.get_param('~robot_namespace')
        self.judge_server_url = rospy.get_param('/send_id_to_judge/judge_url')

    def get_nearest_unaquired_target_idx(self):
        #rospy.loginfo("[seigoRun3]Get nearest target")
        unaquired_target_idx_list = []
        all_field_score = self.all_field_score #最新のフィールドスコア状況を取得
        
        for idx in range(6, 18): #全てのターゲットに対して、誰が取っているかを確認
            # idx 0~5はロボットについているマーカーなので無視

            if all_field_score[idx] == 0:
                pass #自分が取得しているのでパス

            else:
                unaquired_target_idx_list.append(idx)

        #print(unaquired_target_idx_list)
        #print("未取得のターゲットは"+str(len(unaquired_target_idx_list))+"個です")
        # 未取得のターゲット（ロボットについているものは除く）が無い場合
        if len(unaquired_target_idx_list) == 0:
            print("[seigoRun3:get_nearest_unaquired_target_idx]取得可能なターゲットは0個です")
            return -1 
        
        dist_between_target_list = []
        base_frame_name = "base_link"
        # 未取得のターゲットから自機までの距離を計算し、リストに格納する
        for i, target_idx in enumerate(unaquired_target_idx_list):
            try:
                self.tf_listener.waitForTransform(base_frame_name, "target_"+str(target_idx), rospy.Time(0), rospy.Duration(1.0))
                (trans, hoge) = self.tf_listener.lookupTransform(base_frame_name, "target_"+str(target_idx), rospy.Time(0))
                dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                dist_between_target_list.append(dist)

            except Exception as e:
                rospy.logwarn("Except:[seigoRun3.get_nearest_unaquired_target_idx]")
                rospy.logwarn(str(e))
        #print("odomとの距離一覧",dist_between_target_list)
        nearest_target_idx = unaquired_target_idx_list[dist_between_target_list.index(min(dist_between_target_list))]
        rospy.loginfo("[seigoRun3]最も近いターゲットは target_"+str(nearest_target_idx)+" です")

        return nearest_target_idx
    
    def get_nearest_check_point_idx(self, isEnemy=False):
        #敵、もしくは自機から最も近いチェックポイントのインデックスを返す
        check_point_idx_list = [0,1,2,3,4,5,6,7] #[0,2,4,6]#
        dist_list = []

        if isEnemy == True:
            source_frame_name = self.robot_namespace + "/enemy_closest"
        else:
            source_frame_name = "base_link"

        for idx in range(len(check_point_idx_list)):
            target_frame_name = "check_point_"+str(check_point_idx_list[idx])
            source_frame_name = self.robot_namespace + "/enemy_closest"
            
            try:
                self.tf_listener.waitForTransform(source_frame_name, target_frame_name, rospy.Time(0), rospy.Duration(1.0))
                (trans,rot) = self.tf_listener.lookupTransform(source_frame_name, target_frame_name, rospy.Time(0))
                dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2)

            except Exception as e:
                print("[seigoRun3:get_nearest_...]"+str(e))
                dist = 1000.00
            
            dist_list.append(dist)

        nearest_check_point_idx = check_point_idx_list[dist_list.index(min(dist_list))]
        return nearest_check_point_idx
            

    def get_farthest_unaquired_target_idx(self, isEnemy=False):
        #自機or敵機から一番遠い未取得のターゲットのidxを返す
        unaquired_target_idx_list = []
        all_field_score = self.all_field_score #最新のフィールドスコア状況を取得
        
        for idx in range(6, 18): #全てのターゲットに対して、誰が取っているかを確認
            # idx 0~5はロボットについているマーカーなので無視

            if all_field_score[idx] == 0:
                pass #自分が取得しているのでパス

            else:
                unaquired_target_idx_list.append(idx)

        # 未取得のターゲット（ロボットについているものは除く）が無い場合
        if len(unaquired_target_idx_list) == 0:
            print("[seigoRun3:get_farthest_unaquired_target_idx]取得可能なターゲットは0個です")
            return -1 
        
        dist_between_target_list = []
        if isEnemy == False:
            base_frame_name = "base_link"
        
        elif isEnemy == True:
            base_frame_name = self.robot_namespace + "/enemy_closest"

        # 未取得のターゲットから自機までの距離を計算し、リストに格納する
        for i, target_idx in enumerate(unaquired_target_idx_list):
            try:
                self.tf_listener.waitForTransform(base_frame_name, "target_"+str(target_idx), rospy.Time(0), rospy.Duration(1.0))
                (trans, hoge) = self.tf_listener.lookupTransform(base_frame_name, "target_"+str(target_idx), rospy.Time(0))
                dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                dist_between_target_list.append(dist)

            except Exception as e:
                rospy.logwarn("Except:[seigoRun3.get_farthest_unaquired_target_idx]")
                rospy.logwarn(str(e))
        #print("odomとの距離一覧",dist_between_target_list)
        farthest_target_idx = unaquired_target_idx_list[dist_between_target_list.index(max(dist_between_target_list))]
        rospy.loginfo("[seigoRun3]最も遠いターゲットは target_"+str(farthest_target_idx)+" です")

        return farthest_target_idx


    def get_war_state(self, dummy):
        #rospy.loginfo("[seigoRun3]Get war_state")
        req = requests.get(self.judge_server_url+"/warState")
        dic = req.json()

        #scoreの取得
        self.my_score = int(dic["scores"][self.my_side])
        if self.my_side == "r":
            self.enemy_score = int(dic["scores"]["b"])
        else:
            self.enemy_score = int(dic["scores"]["r"])
       
        #フィールドのスコアを取得する
        for idx in range(18): # フィールドターゲットの数は18個
            self.all_field_score_prev[idx] = self.all_field_score[idx] #一つ前のフレームでのスコアを格納
            target_state = dic["targets"][idx]["player"] #ターゲットを取得しているプレイヤーを取得

            if target_state == "n": #自分も敵もターゲットを取得していない
                self.all_field_score[idx] = 1
            elif target_state == self.my_side: #自分がターゲットを取得している
                self.all_field_score[idx] = 0
            else: #敵がターゲットを取得している
                self.all_field_score[idx] = 2

            if self.all_field_score[idx] != self.all_field_score_prev[idx]: #ターゲット情報に更新があるかどうか
                if self.all_field_score[idx] == 2: #敵がターゲットを取得した
                    if self.all_field_score_prev[idx] == 1:
                        print("[seigoRun3]敵がID:"+str(idx)+"のターゲットを取得した")
                    elif self.all_field_score_prev[idx] == 0:
                        print("[seigoRun3]敵にID:"+str(idx)+"のターゲットを奪われた")
                    else:
                        pass
                
                elif self.all_field_score[idx] == 0: #自分がターゲットを取得した
                    if self.all_field_score_prev[idx] == 2: 
                        print("[seigoRun3]敵が取得したID："+str(idx)+"のターゲットを奪った")
                    elif self.all_field_score_prev[idx] == 1:
                        print("[seigoRun3]ID:"+str(idx)+"のターゲットを取得した")
                    else:
                        pass
        #フィールドスコアの更新終了

        #ボディーマーカーのスコアを更新
        if self.my_side == "b":
            self.my_body_remain = self.all_field_score[0:3]
            self.enemy_body_remain = self.all_field_score[3:6]
        elif self.my_side == "r":
            self.my_body_remain = self.all_field_score[3:6]
            self.enemy_body_remain = self.all_field_score[0:3]
        
        #print("----------")
        #for i in range(6,18): #6~17
        #    print(str(i), str(self.all_field_score[i])    
        #print("----------")

    
    #-------------------------------------------#
    # 現在のスコアや敵ロボットとの関係から戦略を決定する
    #
    # ：決められたルートを巡回 
    # ：一番近くにあるマーカを取得する PATROL
    # ：敵に正対する FACE
    # ：敵の遠くにあるマーカーを狙う　LEAVE
    # ：障害物の影に隠れる HIDE
    #-------------------------------------------#
    def strategy_decision(self):
        # scoreや敵の位置情報をもとに動作を決定する
        if self.first_move_did == False:
            return FIRST_MOVE

        if self.my_body_remain[0] == 2 and self.my_body_remain[1] == 2 and self.my_body_remain[2] == 2:
            print("[seigoRun3:strategy_decision]Bodyのマーカーを全て敵に取られています")
            if self.get_nearest_unaquired_target_idx() == -1: #フィールドターゲットを全部取った
                return RUN
            else:    
                return PATROL
        
        #敵の位置を確認
        exist, dist, dire = self.detect_enemy()

        #if exist == True: #敵発見
        if exist == True and dist < 1.0:

            #自分が負けている場合は正対しながら後ろに下がる#下がるだけではダメかも
            if self.my_score < self.enemy_score:
                print("[seigoRun3:strategy_decision]:敵との距離が1.0未満です。ATTACKします。")
                return ATTACK
            
            else:
                print("[seigoRun3:strategy_decision]:敵との距離が1.0未満です。敵の方に正対します。")
                return FACE

        if exist == True and dist < 1.25:
            print("[seigoRun3:strategy_decision]:閾値以内の範囲に敵を発見。遠くに逃げます。")
            return LEAVE
            

            #奇数番目のチェックポイントの近くで敵を見つけたら障害物の裏に隠れる
            #return HIDE

            #if dist < 2.5:
            #    return LEAVE
            #else:
            #    return PATROL
            #return HIDE
        
        else:
            if exist == True:
                print("[seigoRun3:strategy_decision]:敵を発見!しかし閾値より遠くにいるので無視します。")
            else:
                print("[seigoRun3:strategy_decision]:周りに敵はいません。")

            if self.get_nearest_unaquired_target_idx() == -1: #フィールドターゲットを全部取った
                return RUN
            else: 
                return PATROL

    def strategy_execute(self, strategy):
        self.cancel_goal()
        if strategy == FIRST_MOVE:
            print("----------------------")
            print("----- FIRST_MOVE -----")
            print("----------------------")
            self.first_move()

        elif strategy == PATROL:
            print("------------------")
            print("----- PATROL -----")
            print("------------------")
            self.patrol()
        
        elif strategy == LEAVE:
            print("-----------------")
            print("----- LEAVE -----")
            print("-----------------")
            self.leave()
        
        elif strategy == FACE:
            print("----------------")
            print("----- FACE -----")
            print("----------------")
            self.face()

        elif strategy == ATTACK:
            print("------------------")
            print("----- ATTACK -----")
            print("------------------")
            self.attack()

        elif strategy == RUN:
            print("---------------")
            print("----- RUN -----")
            print("---------------")
            self.run()

    def cancel_goal(self):
        print("[seigoRun3]movebaseによる移動を停止します")
        self.move_base_client.cancel_all_goals()
    
    def turn_to_enemy(self, direction_diff):
        cmd_vel = Twist()
        if direction_diff > 60.0/180*math.pi:
            cmd_vel.angular.z = math.copysign(1.0, direction_diff) * 2.75
        else:
            cmd_vel.angular.z = direction_diff*2.0
        return cmd_vel
    
    # RESPECT @koy_tak
    def detect_collision(self, dist_th=0.15):
        is_front_collision = False
        is_rear_collision  = False

        if self.scan.angle_increment == 0:
            deg_90 = int((math.pi/2.0))
            print("[seigRun3:detect_collision]self.scan.angle_incrementが0なので正しく計算できない可能性があります")

        else:
            deg_90 = int((math.pi/2.0)/self.scan.angle_increment)

        front_count = len([i for i in self.scan.ranges[0:int(deg_90)] if i < dist_th]) + len([i for i in self.scan.ranges[int(deg_90)*3:-1] if i < dist_th])
        rear_count = len([i for i in self.scan.ranges[int(deg_90):int(deg_90)*3] if i < dist_th])

        if front_count > 0 and rear_count == 0:
            is_front_collision = True
            print("[seigRun3:detect_collision]front collision")
        
        elif front_count == 0 and rear_count > 0:
            is_rear_collision = True
            print("[seigRun3:detect_collision]rear collision")
        
        elif front_count > 0 and rear_count > 0:
            is_front_collision = front_count > rear_count
            is_rear_collision = not is_front_collision
            print("[seigRun3:detect_collision]both side collision")

        return is_front_collision, is_rear_collision

    def first_move(self):
        # ゲーム開始直後に行う動作
        # 手前のフィールドターゲットを３つを取る⇨その後中央横のターゲットを取る
        # 赤サイド： 13⇨17→11→16
        # 青サイド： 6⇨14→8→15
        print("[seigoRun3:first_move]開始")
        foreground_target_idx_list = []
        if self.my_side == "b":
            foreground_target_idx_list = [6,14,8]

        elif self.my_side == "r":
            foreground_target_idx_list = [13,17,11]

        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            #target_17(一番最後)の座標登録が可能かどうかチェック
            trans, rot, res = self.get_position_from_tf("target_17", self.robot_namespace+"map")

            if res == True:
                break

            else:
                print("[seigoRun3:first_move]ターゲット_17の座標変換に失敗")
                rospy.sleep(1)

        #手前の３つのフィールドターゲットを巡回する
        target_idx = foreground_target_idx_list.pop(0)

        res = self.send_goal_pose_of_target_by_idx(target_idx)
        if res == True: 
            print("[seigoRun3:first_move]target_"+str(target_idx)+"に向かって移動します")
                
        while not rospy.is_shutdown():
            ### 開始直後は敵がいる可能性は低いので予期せぬ動作を起こさせないために、敵検出処理はスキップする
            exist, dist, dire = self.detect_enemy() #敵がいないか確認
            if exist == True and dist < 1.5: #敵発見
                print("[seigoRun3:first_move]!!! 敵発見 !!!")
                break

            move_base_status = self.move_base_client.get_state()
            if self.all_field_score[target_idx] == 0 or move_base_status == actionlib.GoalStatus.SUCCEEDED:
                if move_base_status == actionlib.GoalStatus.SUCCEEDED:
                    print("[seigoRun3:first_move]target_"+str(target_idx)+"に到着")
                #rospy.sleep(1)

    
                #Back開始
                print("[seigoRun3:first_move]target_"+str(target_idx)+"がうまくマーカーに入りません")
                print("[seigoRun3:first_move]バックします。")
                twist = Twist()
                twist.linear.x = -0.25
                self.direct_twist_pub.publish(twist)

                loop_rate = rospy.Rate(10)
                while not rospy.is_shutdown():
                    is_front_collision, is_rear_collision = self.detect_collision(0.15)
                    exist, dist, dire = self.detect_enemy()

                    if self.all_field_score[target_idx] == 0:
                        break

                    if is_rear_collision == True:
                        print("[seigoRun3:first_move]rearがコリジョンしそうなので後進を停止します")
                        break

                    elif exist == True and dist < 1.5:
                        print("[seigoRun3:first_move]敵が近くにいるので敵の方を向きます")
                        self.face()
                        break

                    loop_rate.sleep()
                
                #停止
                twist = Twist()
                self.direct_twist_pub.publish(twist)

                if(len(foreground_target_idx_list)==0):
                    break #手前３つの巡回完了
                
                else:
                    target_idx = foreground_target_idx_list.pop(0)
                    self.send_goal_pose_of_target_by_idx(target_idx)
            
            elif move_base_status == actionlib.GoalStatus.ACTIVE:
                print("[seigoRun3:first_move]target_"+str(target_idx)+"に向かって移動中")
                rospy.sleep(1)
            
         
        print("[seigoRun3:first_move]手前3つのフィールドターゲットの巡回完了")
        self.first_move_did = True

    def leave(self):
        # ①一番近くにあるチェックポイントに移動
        # ②今いるチェックポイントの両隣のチェックポイントの
        exist, dist, dire = self.detect_enemy()

        if exist == False: #敵を未検出
            print("[seigoRun3:leave]敵を検出することができませんでした。")

        else: #敵発見
            check_point_idx_list = [0,1,2,3,4,5,6,7] #[0,2,4,6]#
            dist_list = []
            for idx in range(len(check_point_idx_list)):
                target_frame_name = "check_point_"+str(check_point_idx_list[idx])
                source_frame_name = self.robot_namespace + "/enemy_closest"
                
                try:
                    self.tf_listener.waitForTransform(source_frame_name, target_frame_name, rospy.Time(0), rospy.Duration(1.0))
                    (trans,rot) = self.tf_listener.lookupTransform(source_frame_name, target_frame_name, rospy.Time(0))
                    dist = math.sqrt(trans[0] ** 2 + trans[1] ** 2)

                except Exception as e:
                    rospy.logwarn("[seigoRun3:leave]"+str(e))
                    dist = 0.00
                
                dist_list.append(dist)
                #print("[seigoRun3:leave]check_point_"+str(check_point_idx_list[idx])+"までの距離："+str(dist))
            
            farthest_check_point_idx = check_point_idx_list[dist_list.index(max(dist_list))]
            print("[seigoRun3:leave]敵から最も遠くにあるcheck_point_"+str(farthest_check_point_idx)+"への移動を開始します")
        
            #チェックポイントに移動開始
            loop_rate = rospy.Rate(10)
            self.send_goal_pose_of_checkPoint_by_idx(farthest_check_point_idx)
            while not rospy.is_shutdown():
                move_base_status = self.move_base_client.get_state()
                if move_base_status == actionlib.GoalStatus.SUCCEEDED:
                    print("[seigoRun3:leave]check_point_"+str(farthest_check_point_idx)+"に到着")
                    break

                elif move_base_status == actionlib.GoalStatus.ACTIVE:
                    print("[seigoRun3:leave]check_point_"+str(farthest_check_point_idx)+"に向かって移動中")
                    #rospy.sleep(1)
                
                dist_thresh = 0.75
                time_thresh = 1
                exist, dist, dire = self.detect_enemy() #敵がいないか確認
                if exist == True:
                    if dist < 1.0:
                        print("[seigoRun3:leave]1.0以内に敵を発見。敵に正対してleave関数を終了します。")
                        self.face()
                        break

                    else:
                        print("[seigoRun3:leave]敵を発見。しかし1.0以上離れているのでleave関数を継続します。")
                """
                if exist == True: #敵発見
                    if rospy.get_time() - self.enemy_detect_last_time < time_thresh:
                        print("[seigoRun3:leave]!!! 敵発見 !!! 最後に検出してから時間が経ってないので無視します")
                    
                    elif dist < dist_thresh:
                        print("[seigoRun3:leave]!!! 敵発見 !!! 敵の方を向きます")
                        self.face()
                        break

                    else:
                        print("[seigoRun3:leave]!!! 敵発見 !!! 距離が遠いので無視します")
                """
                loop_rate.sleep()

            print("[seigoRun3:leave]一番近くにある未取得のフィールドターゲットを狙います")
            target_idx = self.get_nearest_unaquired_target_idx()

            res = self.send_goal_pose_of_target_by_idx(target_idx)
            if res == True: 
                print("[seigoRun3:leave]target_"+str(target_idx)+"に向かって移動します")

            while not rospy.is_shutdown():
                exist, dist, dire = self.detect_enemy() #敵がいないか確認
                if exist == True: #敵発見
                    if dist < 1.0:
                        print("[seigoRun3:leave]1.0以内に敵発見。敵に正対します。")
                        self.face()
                        break

                move_base_status = self.move_base_client.get_state()
                if self.all_field_score[target_idx] == 0 or move_base_status == actionlib.GoalStatus.SUCCEEDED:
                    if move_base_status == actionlib.GoalStatus.SUCCEEDED:
                        print("[seigoRun3:leave]target_"+str(target_idx)+"に到着")   
                    break
                
                elif move_base_status == actionlib.GoalStatus.ACTIVE:
                    print("[seigoRun3:leave]target_"+str(target_idx)+"に向かって移動中")
                
                loop_rate.sleep()
        
        print("[seigoRun3:leave]終了")

    def hide(self):
        #障害物の影に隠れる
        #①ロボットのリアを起点に、一番近いチェックポイントを探索
        #②
        print("[seigoRun3:hide]開始")
        print("[seigoRun3:hide]終了")

    def face(self):
        #移動中であれば停止し敵の方を向く
        print("[seigoRun3:face]開始")
        self.cancel_goal()

        exist, dist, dire = self.detect_enemy() #再び敵検出
        
        if exist == False:
            print("[seigoRun3:face]敵は周りにいません")

        elif exist == True:
            degree = 180.00 * dire / math.pi

            if abs(degree) < 25:
                print("[seigoRun3:face]敵との為す角度が25°未満なので回転しません")

            else:
                print("[seigoRun3:face]敵を発見！敵の方を向きます")
                cmd_vel = Twist()
                if degree > 0:
                    cmd_vel.angular.z = math.radians(90)
                else:
                    cmd_vel.angular.z = -math.radians(90)
                wait_time = float(abs(degree) / 90)
                start_time = rospy.Time.now()

                print("[seigoRun3:face]"+str(wait_time)+"秒回転します。")
                loop_rate = rospy.Rate(10)
                self.direct_twist_pub.publish(cmd_vel)
                while (start_time + rospy.Duration(wait_time)) > rospy.Time.now():
                    #print("[seigoRun3:face]回転中...")
                    #loop_rate.sleep()
                    pass
            
                #回転停止
                cmd_vel.angular.z = 0.0
                self.direct_twist_pub.publish(cmd_vel)
                print("[seigoRun3:face]回転終了")
        
        print("[seigoRun3:face]終了")

    
    def attack(self):
        #まずは敵の方を向く
        print("[seigoRun3:attack]開始")
        
        twist = Twist()
        loop_rate = rospy.Rate(10)

        print("[seigoRun3:attack]敵の方を向きます")
        self.face()

        """
        #敵と自機のそれぞれの近くのチェックポイントを調べる
        my_nearest_check_point_idx = self.get_nearest_check_point_idx(isEnemy=False)
        enemy_nearest_check_point_idx = self.get_nearest_check_point_idx(isEnemy=True)

        #敵、自機ともに奇数番目のチェックポイントに近いとき
        #→チェックポイントに隠れて奇襲を狙う
        if my_nearest_check_point_idx == enemy_nearest_check_point_idx and my_nearest_check_point_idx%2 != 0:
            
            #再度敵検出
            self.detect_enemy()

            #敵の位置を覚えておく
            target_link = self.robot_namespace + "/enemy_closest"
            base_link = self.robot_namespace+"map"
            trans, rot, res = self.get_position_from_tf(target_link, base_link)

            if res == False:
                print("[seigoRun3:attack]敵の座標変換ができないので奇襲攻撃はあきらめます。")

            else:
                #チェックポイントまで移動
                loop_rate = rospy.Rate(10)
                check_point_idx = my_nearest_check_point_idx
                self.send_goal_pose_of_checkPoint_by_idx(check_point_idx)
                while not rospy.is_shutdown():
                    move_base_status = self.move_base_client.get_state()
                    if move_base_status == actionlib.GoalStatus.ACTIVE:
                        print("[seigoRun3:attack]check_point_"+str(check_point_idx)+"に向かって移動中")

                    elif move_base_status == actionlib.GoalStatus.SUCCEEDED:
                        print("[seigoRun3:attack]check_point_"+str(check_point_idx)+"に到着")
                        break
                    loop_rate.sleep()

                self.cancel_goal()
                #チェックポイントに到着したら、敵の位置まで移動
                goal_pose = Pose()
                goal_pose.position.x = trans[0]
                goal_pose.position.y = trans[1]
                goal_pose.orientation.z = 1
                self.send_goal_to_move_base(goal_pose)
                while not rospy.is_shutdown():
                    move_base_status = self.move_base_client.get_state()
                    if move_base_status == actionlib.GoalStatus.ACTIVE:
                        print("[seigoRun3:attack]敵に向かって移動中")

                    elif move_base_status == actionlib.GoalStatus.SUCCEEDED:
                        print("[seigoRun3:attack]敵が元いた位置に到着")
                    
                    #敵の位置を確認
                    exist, dist, dire = self.detect_enemy()
                    if exist == True:
                        print("[seigoRun3:attack]敵を発見。敵の方を向きます。")
                        self.face()
                        break
                    
                    loop_rate.sleep()
                

        else:
        """
        #detect_collisionが反応するまで前進
        print("[seigoRun3:attack]前進します")
        twist.linear.x = 0.25
        while not rospy.is_shutdown():
            is_front_collision, is_rear_collision = self.detect_collision(0.15)
            exist, dist, dire = self.detect_enemy()

            if is_front_collision == True:
                print("[seigoRun3:attack]衝突しそうなので停止します")
                break

            elif exist == False:
                print("[seigoRun3:attack]敵を見失ったので停止します")
                break

            elif exist == True:
                if dist > 1.5:
                    print("[seigoRun3:attack]敵までの距離が1.0以上になったので後進を停止します。")
                    break

                else:
                    print("[seigoRun3:attack]敵が近くにいるので敵の方を向きます")
                    self.face()

            self.direct_twist_pub.publish(twist)
            loop_rate.sleep()

        #停止
        twist.linear.x = 0
        self.direct_twist_pub.publish(twist)
        
        #detect_collisionが反応するまで後進
        print("[seigoRun3:attack]後進します")
        twist.linear.x = -0.25
        while not rospy.is_shutdown():
            is_front_collision, is_rear_collision = self.detect_collision(0.15)
            exist, dist, dire = self.detect_enemy()

            if is_rear_collision == True:
                print("[seigoRun3:attack]衝突しそうなので停止します")
                break

            elif exist == False:
                print("[seigoRun3:attack]敵を見失ったので停止します")
                break

            elif exist == True:
                if dist > 1.5:
                    print("[seigoRun3:attack]敵までの距離が1.0以上になったので後進を停止します。")
                    break

                else:
                    print("[seigoRun3:attack]敵が近くにいるので敵の方を向きます")
                    self.face()
                    
            self.direct_twist_pub.publish(twist)
            loop_rate.sleep()
        
        #停止
        twist.linear.x = 0
        self.direct_twist_pub.publish(twist)

        #敵からもっと遠くにある未取得のターゲットに移動します。
        print("[seigoRun3:attack]敵から最も遠くにある未取得のターゲットを狙います。")
        target_idx = self.get_farthest_unaquired_target_idx(isEnemy=True)

        res = self.send_goal_pose_of_target_by_idx(target_idx)
        if res == True: 
            print("[seigoRun3:attack]target_"+str(target_idx)+"に向かって移動します")

        while not rospy.is_shutdown():
            exist, dist, dire = self.detect_enemy() #敵がいないか確認
            if exist == True: #敵発見
                if dist < 1.0:
                    print("[seigoRun3:attack]1.0以内に敵を発見。敵の方に正対します。")
                    self.face()
                    break

            move_base_status = self.move_base_client.get_state()
            if self.all_field_score[target_idx] == 0 or move_base_status == actionlib.GoalStatus.SUCCEEDED:
                if move_base_status == actionlib.GoalStatus.SUCCEEDED:
                    print("[seigoRun3:attack]target_"+str(target_idx)+"に到着")
                #rospy.sleep(1)       
                break
            
            elif move_base_status == actionlib.GoalStatus.ACTIVE:
                print("[seigoRun3:attack]target_"+str(target_idx)+"に向かって移動中")
                #rospy.sleep(1)


        # 画角内にマーカーがうまく入らない場合の処理
        print("[seigoRun3:attack]target_"+str(target_idx)+"がうまくマーカーに入りません")
        print("[seigoRun3:attack]バックします。")
        twist = Twist()
        twist.linear.x = -0.25
        self.direct_twist_pub.publish(twist)

        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            is_front_collision, is_rear_collision = self.detect_collision(0.15)
            exist, dist, dire = self.detect_enemy()

            if self.all_field_score[target_idx] == 0:
                break

            if is_rear_collision == True:
                print("[seigoRun3:attack]rearがコリジョンしそうなので後進を停止します")
                break

            elif exist == True and dist < 1.5:
                print("[seigoRun3:attack]敵が近くにいるので敵の方を向きます")
                self.face()
                break

            loop_rate.sleep()
            
        #停止
        twist = Twist()
        self.direct_twist_pub.publish(twist)

        print("[seigoRun3:attack]終了")


    def patrol(self):
        #一番近くにある未取得のフィールドターゲットを探索する
        print("[seigoRun3:patrol]開始：一番近くにある未取得のフィールドターゲットを狙います")
        target_idx = self.get_nearest_unaquired_target_idx()

        res = self.send_goal_pose_of_target_by_idx(target_idx)
        if res == True: 
            print("[seigoRun3:patrol]target_"+str(target_idx)+"に向かって移動します")

        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            exist, dist, dire = self.detect_enemy() #敵がいないか確認
            if exist == True: #敵発見
                if self.my_body_remain[0] == 2 and self.my_body_remain[1] == 2 and self.my_body_remain[2] == 2:
                    print("[seigoRun3:patrol]敵発見。Bodyマーカーを全て敵に取られているので無視します")
                
                elif dist < 1.5:
                    print("[seigoRun3:patrol]1.5以内に敵を発見。敵の方に正対します。")
                    self.face()
                    break

            move_base_status = self.move_base_client.get_state()
            if self.all_field_score[target_idx] == 0 or move_base_status == actionlib.GoalStatus.SUCCEEDED:
                if move_base_status == actionlib.GoalStatus.SUCCEEDED:
                    print("[seigoRun3:patrol]target_"+str(target_idx)+"に到着")
                #rospy.sleep(1)       
                break
            
            elif move_base_status == actionlib.GoalStatus.ACTIVE:
                print("[seigoRun3:patrol]target_"+str(target_idx)+"に向かって移動中")
            
            loop_rate.sleep()


        # 画角内にマーカーがうまく入らない場合の処理
        print("[seigoRun3:patrol]target_"+str(target_idx)+"がうまくマーカーに入りません")
        print("[seigoRun3:patrol]バックします。")
        twist = Twist()
        twist.linear.x = -0.25
        self.direct_twist_pub.publish(twist)

        while not rospy.is_shutdown():
            is_front_collision, is_rear_collision = self.detect_collision(0.15)
            exist, dist, dire = self.detect_enemy()

            if self.all_field_score[target_idx] == 0:
                break

            if is_rear_collision == True:
                print("[seigoRun3:patrol]rearがコリジョンしそうなので後進を停止します")
                break

            elif exist == True and dist < 1.5:
                if self.my_body_remain[0] == 2 and self.my_body_remain[1] == 2 and self.my_body_remain[2] == 2:
                    pass
                else:
                    print("[seigoRun3:patrol]敵が近くにいるので敵の方を向きます")
                    self.face()
                    break

            loop_rate.sleep()
        
        #停止
        twist = Twist()
        self.direct_twist_pub.publish(twist)

        print("[seigoRun3:patrol]終了")
    
    def run(self):
        print("[seigoRun3:run]開始")
        #的に見つかるまで、もしくはフィールドターゲットを奪われるまでチェックポイントを回る
        check_point_idx = 0
        self.send_goal_pose_of_checkPoint_by_idx(check_point_idx)
        loop_rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            move_base_status = self.move_base_client.get_state()
            if move_base_status == actionlib.GoalStatus.ACTIVE:
                print("[seigoRun3:checkpoint]check_point_"+str(check_point_idx)+"に向かって移動中")

            elif move_base_status == actionlib.GoalStatus.SUCCEEDED:
                print("[seigoRun3:checkpoint]check_point_"+str(check_point_idx)+"に到着")
                
                
                check_point_idx += 1
                if check_point_idx > 7:
                    check_point_idx = 0
                
                print("[seigoRun3:checkpoint]check_point_"+str(check_point_idx)+"に向かって移動開始")
                self.send_goal_pose_of_checkPoint_by_idx(check_point_idx)

            #敵の位置を確認
            exist, dist, dire = self.detect_enemy()
            if exist == True: #敵発見
                if dist < 1.0:
                    print("[seigoRun3:run]敵を発見。1.0以内にいるので正対します")
                    self.face()
                    break
                
                else:
                    print("[seigoRun3:run]敵を発見。1.0より遠くにいるので無視します。")
            
            #フィールドターゲットの状況確認
            if self.get_nearest_unaquired_target_idx() != -1: #フィールドターゲットを全部取った
                print("[seigoRun3:run]フィールドターゲットを奪われた")
                break

            loop_rate.sleep()
    

    def test_move(self):
        print("[seigoRun3:test_move]開始")
        #的に見つかるまで、もしくはフィールドターゲットを奪われるまでチェックポイントを回る
        check_point_idx = 0
        self.send_goal_pose_of_checkPoint_by_idx(check_point_idx)
        loop_rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            move_base_status = self.move_base_client.get_state()
            if move_base_status == actionlib.GoalStatus.ACTIVE:
                print("[seigoRun3:test_move]check_point_"+str(check_point_idx)+"に向かって移動中")

            elif move_base_status == actionlib.GoalStatus.SUCCEEDED:
                print("[seigoRun3:test_move]check_point_"+str(check_point_idx)+"に到着")
                
                
                check_point_idx += 1
                if check_point_idx > 7:
                    check_point_idx = 0
                
                print("[seigoRun3:test_move]check_point_"+str(check_point_idx)+"に向かって移動開始")
                self.send_goal_pose_of_checkPoint_by_idx(check_point_idx)

            #敵の位置を確認
            exist, dist, dire = self.detect_enemy()
            if exist == True: #敵発見
                if dist < 1.0:
                    print("[seigoRun3:test_move]敵を発見。1.0以内にいるので正対します")
                    self.face()
                    break
                
                else:
                    print("[seigoRun3:test_move]敵を発見。1.0より遠くにいるので無視します。")
            
            #フィールドターゲットの状況確認
            #if self.get_nearest_unaquired_target_idx() != -1: #フィールドターゲットを全部取った
            #    print("[seigoRun3:]フィールドターゲットを奪われた")
            #    break

            loop_rate.sleep()


def main():
    rospy.init_node("seigo_run3")
    rospy.loginfo("[seigoRun3]seigoRun3 is running")
    node = SeigoRun3()
    loop_rate = rospy.Rate(10) #30Hz
    
    while not rospy.is_shutdown():
        strategy = node.strategy_decision()
        node.strategy_execute(strategy)
        
        loop_rate.sleep()

if __name__ == "__main__":
    main()