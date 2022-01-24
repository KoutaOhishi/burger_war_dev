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