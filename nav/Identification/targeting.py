#!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from occ import getCoord
from impidentify import Detect, distance
'''
from movebase import GoToPose

def navbot(pose,quat):
    navigator = GoToPose()

    position = {'x': pose[0], 'y' : pose[1]}
    quaternion = {'r1' : quat[0], 'r2' : quat[1], 'r3' : quat[2], 'r4' : quat[3]}
    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])

    #nav move
    navigator.goto(position, quaternion)

'''
# Uncomment above and in main() for integrated navigation integrated with identification purpose


mapdone = False

def rotatebot(signal):
    twist = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set the update rate to 1 Hz
    rate = rospy.Rate(1)

    twist.linear.x = 0.0
    # set the direction to rotate
    val = -0.05 if signal == '-1' else 0.05
    twist.angular.z = val
    # start rotation
    pub.publish(twist)
    time.sleep(1)
    twist.angular.z = 0.0
    pub.publish(twist)

def mapdone(msg):
    global mapdone
    mapdone = msg.data


def main():
    global mapdone

    detector = Detect('red') # change color according to target color
    quatcheck = False
    toggle = True
    rate = rospy.Rate(10)
    pub1 = rospy.Publisher('cmd_rotate', String, queue_size=10)
    pub2 = rospy.Publisher('cmd_stepper', String, queue_size=10)
    rospy.Subscriber('mapdone',String,mapdone)
    occ = getCoord() # To grab the current coordinates
    tolerance = 5 # tolerance val of alignment. Reduce to make it stricter
    stored_coord = False
    while toggle:

        # Reads the image stream 
        detector.readImg()

        # if no target detected, move the turtlebot
        if not detector.found:
            pass
        
        # This is code to store current coordinates if target identified before mapping done
        '''
        elif not mapdone and not stored_coord:
            stored_pose = occ.cur_pose
            stored_quat = occ.cur_quat
            stored_coord = True
        '''

        # rotate turtlebot3 to face the correct direction
        elif not quatcheck :

            # Checks if map is done, and for stored_coordinates
            # Navigates to stored_coordinates if map is done with reasonable distance
            # Uncomment if identification after navigation
            '''
            if not mapdone:
                continue
            elif stored_coord:
                while distance(occ.cur_pose,stored_pose) > 1: # calibrate this value for accuracy
                    navbot(stored_pose,stored_quat)
                stored_coord = False
                continue
            '''

            # Rotates turtlebot left and right as long as center of target is not
            # Laterally aligned with the center of image feed of rpi camera
            # works only if rpicamera is positioned on the same vertical axis as the payload
            if detector.diff_x and detector.diff_y:
                if abs(detector.diff_x) > tolerance:
                    horz = '-1' if detector.diff_x < 0 else '1'
                    rospy.loginfo('Calibrating Left/Right Aim. Publishing to cmd_rotate %s',horz)
                    rotatebot(horz)
                else:
                    quatcheck = True
                    rospy.loginfo('Calibration Complete. Firing!') # Comment this if uncommenting stepper code below
                    pub2.publish('10') # Comment this if uncommenting stepper code below
                    rospy.sleep(1)
                    toggle = False # Comment this if uncommenting stepper code below

        # activates stepper to tilt payload cannon upwards
        '''
        else:
            if detector.diff_x and detector.diff_y:
                if abs(detector.diff_y) > tolerance :
                    vert = '-1' if detector.diff_y < 0 else '1'
                    rospy.loginfo('Calibrating Up/Down Aim. Publishing to cmd_stepper %s',vert)
                    pub2.publish(vert)
                else:
                    rospy.loginfo('Calibration Complete. Firing!')
                    pub2.publish('10') # Publishes '10' for firing sequence to initiate
                    rospy.sleep(1)
                    toggle = False
        '''
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")