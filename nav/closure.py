#!/usr/bin/env python
import cv2
from PIL import Image
import time

def closure(mapdata):
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
    ret,img2 = cv2.threshold(mapdata,2,255,0)
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