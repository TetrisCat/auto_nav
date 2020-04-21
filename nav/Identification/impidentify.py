
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

#global_hsv = np.array([])

#def on_mouse_click(event, x, y, flags, param):
#    if event == cv2.EVENT_LBUTTONDOWN:
#        print(global_hsv[y, x])

#cv2.namedWindow('frame')
#cv2.setMouseCallback('frame',on_mouse_click)


def dprint(s):
    if D_PRINT:
        print(s)

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)


class Detect:
    def __init__(self):
        self.color = 'red'
        self.mapping = {
            'red': ([0,90,90],[15,205,205])
        }
        self.upper = np.array(self.mapping[self.color][1],dtype = "uint8")
        self.lower = np.array(self.mapping[self.color][0],dtype = "uint8")
        self.cv_image = []
        self.node = rospy.init_node('nav_test',anonymous=False,disable_signals=True)
        rospy.sleep(1.5)
        self.sub = rospy.Subscriber('raspicam_node/image/compressed',CompressedImage,self.getImg)
        self.imgH = 480
        self.imgW = 640
        self.minH = 15
        self.minW = 5
        self.toggle = True
        self.diff_x = 0
        self.diff_y = 0
    
    def getImg(self,msg):
        self.cv_image = bridge.compressed_imgmsg_to_cv2(msg)

    def readImg(self):
    #global global_hsv
        if self.cv_image == []:
            rospy.loginfo('cv_image is still blank~')
            rospy.sleep(1)
            pass
        else:
            hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
        #global_hsv = hsv
        #cv2.imshow("frame", global_hsv)
            #cv2.waitKey(0)
        mask=cv2.inRange(hsv, self.lower, self.upper) # binary mask of white n black pixel
            #cv2.imshow('mask',mask)
            result=cv2.bitwise_and(hsv, hsv, mask=mask)
        out_temp=cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
            cv2.imshow('image',out_temp)
        cv2.waitKey(0)
            result=np.array(result)  
            #cv2.imshow('result2',result)
            contours=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            dims = (0,0)
        print("check 3")

            cnt_img = (self.imgW//2,self.imgH//2)
            for c in contours:
                peri = cv2.arcLength(c,True)
                approx = cv2.approxPolyDP(c,0.04*peri,True)
