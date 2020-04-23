
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

global_hsv = np.array([])

def on_mouse_click(event, x, y, flags, param):
   if event == cv2.EVENT_LBUTTONDOWN:
       print(global_hsv[y, x])

find_hsv = False

if find_hsv:
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame',on_mouse_click)

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)


class Detect:
    def __init__(self,color):
        self.color = color
        self.mapping = {
            'red': ([161,155,84],[179,255,255]),
            'blue': ([100,150,0],[140,255,255]),
            'green': ([25,52,72],[102,255,255])
            # Add additional colors and their ranges for more versatility
        }
        self.upper = np.array(self.mapping[self.color][1],dtype = "uint8")
        self.lower = np.array(self.mapping[self.color][0],dtype = "uint8")
        self.cv_image = []
        self.node = rospy.init_node('nav_test',anonymous=False,disable_signals=True)
        rospy.sleep(1.5)
        self.sub = rospy.Subscriber('raspicam_node/image/compressed',CompressedImage,self.getImg)
        self.imgH = 480 # Adjust imgH and imgW according to camera resolution
        self.imgW = 640 # Rpi Camera v2 has resolution of 640 x 480p
        self.minH = 15 # These are minimum numbers for widith and height of target for detection
        self.minW = 5 # Adjust these values according to your use (size of target)
        # 15x5 works well for detecting target even from further away 
        # as long as the hsv color range
        # is strict enough to only detect the target
        self.found = False
        self.diff_x = 0
        self.diff_y = 0
    
    def getImg(self,msg):
        self.cv_image = bridge.compressed_imgmsg_to_cv2(msg)

    def readImg(self):
        global global_hsv
        if self.cv_image == []: # Checking if subscribing topic is empty/not yet in
            rospy.loginfo('cv_image is still blank~')
            rospy.sleep(1)
            pass
        else:
            hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
            #global_hsv = hsv
            #cv2.imshow("frame", global_hsv)
            #cv2.waitKey(0)
            # Uncomment above ^ to check for hsv values in case original doesnt work
            mask=cv2.inRange(hsv, self.lower, self.upper) # binary mask of white n black pixel
            result=cv2.bitwise_and(hsv, hsv, mask=mask)
            out_temp=cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
            cv2.imshow('image',out_temp) # Showing a live image feed of target spotting for debugging
            cv2.waitKey(0)
            result=np.array(result)  
            contours=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            dims = (0,0)

            cnt_img = (self.imgW//2,self.imgH//2)
            for c in contours:
                peri = cv2.arcLength(c,True)
                approx = cv2.approxPolyDP(c,0.04*peri,True)

                x,y,w,h=cv2.boundingRect(np.float32(approx)) #returns rectangle object with xy cor of top left corner n width n height parameter
                if w >= self.minW and h >=self.minH:
                    self.found = True
                    dims= (self.imgW-x,self.imgH-y) # Dimensions of the target rectangle
                    rospy.loginfo('Found corner at %s and %s',str(dims[0]),str(dims[1]))
                    center = (dims[0] - w//2,dims[1]-h//2) # Finding center point of target
                    rospy.loginfo('Found target center at %s and %s',str(center[0]),str(center[1]))
                    self.diff_x = cnt_img[0] - center[0] 
                    # horizontal pixel difference between center of image and center of target
                    rospy.loginfo('x-axis diff : %s', str(self.diff_x))
                    self.diff_y = cnt_img[1] - center[1]
                    # vertical pixel difference between center of image and center of target
                    rospy.loginfo('y-axis diff : %s', str(self.diff_y))
                    break
