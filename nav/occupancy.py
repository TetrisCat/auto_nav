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
        rospy.loginfo('Current Pose: x: %s, y: %s', str(self.cur_pose[0]),str(self.cur_pose[1]))

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
        self.target = self.get_closest(matrix,output,self.cur_pose,self.map_res,(map_origin.x,map_origin.y))
        rospy.loginfo('target coord: (%s,%s)',str(self.target[0]),str(self.target[1]))

    def get_closest(self,original,adjusted,curpos,res,origin):
        lst = []
        for coord,val in np.ndenumerate(adjusted):
            row = coord[0]
            col = coord[1]
            if val % 10 != 0 and original[row][col] == 0 and val > 30:
                lst.append((col*res + origin[0],row*res + origin[1]))
        # rospy.loginfo(lst)    
        distance_lst = [distance(crd,curpos) for crd in lst]

        idx = len(distance_lst)//2
        return lst[idx]

    def closure(self):
        # This function checks if mapdata contains a closed contour. The function
        # assumes that the raw map data from SLAM has been modified so that
        # -1 (unmapped) is now 0, and 0 (unoccupied) is now 1, and the occupied
        # values go from 1 to 101.

        # According to: https://stackoverflow.com/questions/17479606/detect-closed-contours?rq=1
        # closed contours have larger areas than arc length, while open contours have larger
        # arc length than area. But in my experience, open contours can have areas larger than
        # the arc length, but closed contours tend to have areas much larger than the arc length
        # So, we will check for contour closure by checking if any of the contours
        # have areas that are more than 10 times larger than the arc length
        # This value may need to be adjusted with more testing.
        ALTHRESH = 5
        # We will slightly fill in the contours to make them easier to detect
        DILATE_PIXELS = 3

        # assumes mapdata is uint8 and consists of 0 (unmapped), 1 (unoccupied),
        # and other positive values up to 101 (occupied)
        # so we will apply a threshold of 2 to create a binary image with the
        # occupied pixels set to 255 and everything else is set to 0
        # we will use OpenCV's threshold function for this
        ret,img2 = cv2.threshold(self.odata,2,255,0)
        # we will perform some erosion and dilation to fill out the contours a
        # little bit
        element = cv2.getStructuringElement(cv2.MORPH_CROSS,(DILATE_PIXELS,DILATE_PIXELS))
        # img3 = cv2.erode(img2,element)
        img4 = cv2.dilate(img2,element)
        # use OpenCV's findContours function to identify contours
        # OpenCV version 3 changed the number of return arguments, so we
        # need to check the version of OpenCV installed so we know which argument
        # to grab
        fc = cv2.findContours(img4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        (major, minor, _) = cv2.__version__.split(".")
        if(major == '3'):
            contours = fc[1]
        else:
            contours = fc[0]
        # find number of contours returned
        lc = len(contours)
        # rospy.loginfo('# Contours: %s', str(lc))
        # create array to compute ratio of area to arc length
        cAL = np.zeros((lc,2))
        for i in range(lc):
            cAL[i,0] = cv2.contourArea(contours[i])
            cAL[i,1] = cv2.arcLength(contours[i], True)

        # closed contours tend to have a much higher area to arc length ratio,
        # so if there are no contours with high ratios, we can safely say
        # there are no closed contours
        cALratio = cAL[:,0]/cAL[:,1]
        # rospy.loginfo('Closure: %s', str(cALratio))
        if np.any(cALratio > ALTHRESH):
            return True
        else:
            return False