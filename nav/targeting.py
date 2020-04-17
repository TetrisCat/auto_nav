#!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from impidentify import Detect, distance

def movebot():
    twist = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    twist.linear.x = 0.01
    twist.angular.z = 0.0
    pub.publish(twist)
    time.sleep(1)
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

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




def main():

    detector = Detect()

    quatcheck = False
    toggle = True
    rate = rospy.Rate(10)
    pub1 = rospy.Publisher('cmd_rotate', String, queue_size=10)
    pub2 = rospy.Publisher('cmd_stepper', String, queue_size=10)

    while toggle:

        # Reads the image stream 
        detector.readImg()

        if detector.diff_x == 0:
            movebot()

        # rotate turtlebot3 to face the correct direction
        elif not quatcheck:
            if detector.diff_x and detector.diff_y:
                if abs(detector.diff_x) > 20:
                    horz = '-1' if detector.diff_x < 0 else '1'
                    rospy.loginfo('Calibrating Left/Right Aim. Publishing to cmd_rotate %s',horz)
                    rotatebot(horz)
                else:
                    quatcheck = True


        # activates stepper to tilt payload cannon upwards
        else:
            if detector.diff_x and detector.diff_y:
                if abs(detector.diff_y) > 20:
                    vert = '-1' if detector.diff_y < 0 else '1'
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