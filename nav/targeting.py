import rospy
import time
import math
from std_msgs.msg import String

from identify import Detect, distance

color = 'red'

def main():

    detector = Detect(color)

    quatcheck = False

    rate = rospy.Rate(10)
    pub1 = rospy.Publisher('cmd_rotate',String,queue_size=10)
    pub2 = rospy.Publisher('cmd_stepper',String,queue_size=10)

    while toggle:

        # Reads the image stream 
        detector.readImg()

        # rotate turtlebot3 to face the correct direction
        if not quatcheck:
            if detector.diff_x and detector.diff_y:
                if abs(detector.diff_x) > 5:
                    horz = '-1' if detector.diff_x < 0 else '1'
                    pub1.publish(horz)
                else:
                    quatcheck = True


        # activates stepper to tilt payload cannon upwards
        else:
            if diff_x and diff_y:
                if abs(diff_y) > 5:
                    vert = '-1' if diff_x < 0 else '1'
                    pub2.publish(vert)
                else:
                    pub2.publish('10')
                    rospy.sleep(1)
                    toggle = False

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")