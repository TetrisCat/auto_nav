#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, String
from scipy import signal
from PIL import Image
import tf2_ros
import cv2

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

class getCoord:
    
    def __init__(self):
        self.node = rospy.init_node('mapread',anonymous=False,disable_signals=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.sub = rospy.Subscriber('map',OccupancyGrid,self.get_occupancy,self.tfBuffer) # Subsribes to the map topic
        self.map_res = 0
        self.cur_pose = ()
        self.odata = []
        self.cur_quat = ()


        self.occ_bins = [-1, 0, 100, 101]
        self.map_res = 0.05
    
    def get_occupancy(self,msg,tfBuffer):

        # create numpy array
        occdata = np.array([msg.data])

        # compute histogram to identify percent of bins with -1
        occ_counts = np.histogram(occdata,self.occ_bins)
        # calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        self.cur_pose = (cur_pos.x,cur_pos.y)
        self.cur_quat = (cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)


    