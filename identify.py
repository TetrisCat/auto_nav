#!/usr/bin/env python
import cv2
import numpy as np
import imutils
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
bridge = CvBridge()


cv2.ocl.setUseOpenCL(False)
#where is the image stored

#range of BGR values. cuz apparently opencv represents images as Np arrays in reverse order?
D_PRINT = True
def dprint(s):
    if D_PRINT:
        print(s)

mapping = {
        'red': ([17,15,100],[50,56,200])
        }

color2detect = 'red'
boundaries = mapping[color2detect]

image = []
data = ()

def getImg(msg):
    global image
    global data
    global cv_image
    image = msg.data

    cv_image = bridge.imgmsg_to_cv2(image,"bgr8")


def getData(msg):
    global data
    data = (msg.height,msg.width)

def readImg():
    global rearranged
    #cv2.imshow('image',image)
    lower=np.array(boundaries[0], dtype = "uint8")
    upper=np.array(boundaries[1],dtype = "uint8")
    mask=cv2.inRange(rearranged, lower, upper) # binary mask of white n black pixel
    #cv2.imshow('mask',mask)
    result=cv2.bitwise_and(rearranged, rearranged, mask=mask)
    #cv2.imshow('images',np.hstack([image,result]))
    cv2.waitKey(0)
    result=np.array(result)  
    #cv2.imshow('result2',result)
    contours=cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    dims = ()
    for c in contours:
        peri = cv2.arcLength(c,True)
        approx = cv2.approxPolyDP(c,0.04*peri,True)

        x,y,w,h=cv2.boundingRect(np.float32(approx)) #returns rectangle object with xy cor of top left corner n width n height parameter
        if w >= 100 and h >= 80:
            dims= (x,y)

    rospy.loginfo('Found corner at %s and %s',str(dims[0]),str(dims[1]))

# if w==3 and h==3:
#     al_Cor=(x+0.5*w,y+0.5*h)
#     get_loc= 3
#     get_dir= 4 
# dprint(al_Cor)
# dprint(get_loc)
# dprint(get_dir)

def main():
    rospy.init_node('nav_test',anonymous=False,disable_signals=True)
    rospy.Subscriber('raspicam_node/camera_info',CameraInfo,getData)
    rospy.sleep(1)
    rospy.Subscriber('raspicam_node/image/compressed',CompressedImage,getImg)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        readImg()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

	



