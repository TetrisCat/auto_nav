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

class getMap:
    
    def __init__(self):
        self.node = rospy.init_node('mapread',anonymous=False,disable_signals=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.sub = rospy.Subscriber('map',OccupancyGrid,self.get_occupancy,self.tfBuffer)
        self.yaw = 0
        self.map_res = 0
        self.cur_pose = ()
        self.odata = []
        self.target = (0,0)
        self.quats = ()
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
        rospy.loginfo('Current Pose: x: %s, y: %s', str(cur_pose[0]),str(cur_pose[1]))

        map_res = msg.info.resolution
        # get map origin struct has fields of x, y, and z
        map_origin = msg.info.origin.position
        oc2 = occdata + 1
        # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
        oc3 = (oc2>1).choose(oc2,2)
        # reshape to 2D array using column order
        self.odata = np.uint8(oc3.reshape(msg.info.height,msg.info.width,order='F'))
        
        matrix = np.transpose(np.asarray(self.odata))
        adjusted = []
        for row in matrix:
            x = [num if num == 1 else 0 if num >=2 else 10 for num in row]
            adjusted.append(x)
        kernel = np.zeros((3,3)) + 1
        #convoluting the matrix with kernel to sum up neigbors
        #idea is to find boundary between 0s and 1s
        output = signal.convolve2d(adjusted, kernel, boundary='fill',mode='same')
        self.target = get_closest(matrix,output,self.cur_pose,self.map_res,(map_origin.x,map_origin.y))
        rospy.loginfo('target coord: (%s,%s)',str(self.target[0]),str(self.target[1]))

        # convert quaternion to Euler angles
        self.quats = (cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)
        # # rospy.loginfo(['Yaw: R: ' + str(yaw) + ' D: ' + str(np.degrees(yaw))])

newOne = getMap()
time.sleep(3)
print(newOne.cur_pose)