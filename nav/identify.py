#!/usr/bin/env python
import cv2
import numpy as np
import imutils
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
bridge = CvBridge()

counter = 0
cv2.ocl.setUseOpenCL(False)

def dprint(s):
    if D_PRINT:
        print(s)

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)


class Detect:


    def __init__(self):
        self.color = 'red'
        self.mapping = {
            'red': ([17,15,100],[50,56,200])
        }
        self.upper = np.array(self.mapping[self.color][1],dtype = "uint8")
        self.lower = np.array(self.mapping[self.color][0],dtype = "uint8")
        self.cv_image = []
        self.node = rospy.init_node('nav_test',anonymous=False,disable_signals=True)
        rospy.sleep(1.5)
        self.sub = rospy.Subscriber('raspicam_node/image/compressed',CompressedImage,self.getImg)
        self.imgH = 480
        self.imgW = 640
        self.minH = 60
        self.minW = 40
        self.toggle = True
        self.diff_x = 0
        self.diff_y = 0
    
    def getImg(self,msg):
        self.cv_image = bridge.compressed_imgmsg_to_cv2(msg)



    def readImg(self):
        if not self.cv_image:
            rospy.loginfo('cv_image is still blank~')
            rospy.sleep(1)
            pass
        else:
            mask=cv2.inRange(self.cv_image, self.lower, self.upper) # binary mask of white n black pixel
            #cv2.imshow('mask',mask)
            result=cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
            #cv2.imshow('images',np.hstack([image,result]))
            cv2.waitKey(0)
            result=np.array(result)  
            #cv2.imshow('result2',result)
            contours=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            dims = (0,0)

            cnt_img = (self.imgW//2,self.imgH//2)
            for c in contours:
                peri = cv2.arcLength(c,True)
                approx = cv2.approxPolyDP(c,0.04*peri,True)

                x,y,w,h=cv2.boundingRect(np.float32(approx)) #returns rectangle object with xy cor of top left corner n width n height parameter
                if w >= self.minW and h >=self.minH:
                    dims= (self.imgW-x,self.imgH-y)
                    rospy.loginfo('Found corner at %s and %s',str(dims[0]),str(dims[1]))
                    center = (dims[0] - w//2,dims[1]-h//2)
                    self.diff_x = cnt_img[0] - center[0]
                    self.diff_y = cnt_img[1] - center[1]
                    break
            



	



