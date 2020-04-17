#!/usr/bin/env python
import cv2
import numpy as np
import imutils
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
bridge = CvBridge()
imgH = 480
imgW = 640

diff_x = 0
diff_y = 0
cv_image = []

upper = np.array([50,56,200],dtype = "uint8")
lower = np.array([17,15,100],dtype = "uint8")
counter = 0
cv2.ocl.setUseOpenCL(False)

minW = 40
minH = 60


def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def getImg(self,msg):
    global imgH
    global imgW
    global cv_image 


    imgH = msg.height
    imgW = msg.width
    cv_image = bridge.imgmsg_to_cv2(msg)

def readImg(self):
    global cv_image
    global lower
    global upper
    global diff_x
    global diff_y
    global imgH
    global imgW

    mask=cv2.inRange(cv_image, lower, upper) # binary mask of white n black pixel
    #cv2.imshow('mask',mask)
    result=cv2.bitwise_and(cv_image, cv_image, mask=mask)
    #cv2.imshow('images',np.hstack([image,result]))
    cv2.waitKey(0)
    result=np.array(result)  
    #cv2.imshow('result2',result)
    contours=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    dims = (0,0)

    cnt_img = (imgW//2,imgH//2)
    for c in contours:
        peri = cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,0.04*peri,True)

        x,y,w,h=cv2.boundingRect(np.float32(approx)) #returns rectangle object with xy cor of top left corner n width n height parameter
        if w >= minW and h >= minH:
            dims= imgW-x,imgH-y)
            rospy.loginfo('Found corner at %s and %s',str(dims[0]),str(dims[1]))
            center = (dims[0] - w//2,dims[1]-h//2)
            diff_x = cnt_img[0] - center[0]
            diff_y = cnt_img[1] - center[1]
            break

def main():
    global diff_x
    global diff_y
    
    rospy.init_node('nav_test',anonymous=False,disable_signals=True)
    rospy.Subscriber('raspicam_node',Image,getImg)
    rospy.sleep(1.5)

    quatcheck = False
    toggle = True
    rate = rospy.Rate(10)
    pub1 = rospy.Publisher('cmd_rotate',String,queue_size=10)
    pub2 = rospy.Publisher('cmd_stepper',String,queue_size=10)

    while toggle:

        # rotate turtlebot3 to face the correct direction
        if not quatcheck:
            if diff_x and diff_y:
                if abs(diff_x) > 5:
                    horz = '-1' if diff_x < 0 else '1'
                    rospy.loginfo('Calibrating Left/Right Aim. Publishing to cmd_rotate %s',horz)
                    pub1.publish(horz)
                else:
                    quatcheck = True


        # activates stepper to tilt payload cannon upwards
        else:
            if diff_x and diff_y:
                if abs(diff_y) > 5:
                    vert = '-1' if diff_x < 0 else '1'
                    rospy.loginfo('Calibrating Up/Down Aim. Publishing to cmd_stepper %s',vert)
                    pub2.publish(vert)
                else:
                    rospy.loginfo('Calibration Complete. Firing!')
                    pub2.publish('10')
                    rospy.sleep(1)
                    toggle = False

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")