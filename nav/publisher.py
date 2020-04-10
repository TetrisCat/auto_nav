import rospy
import time
import math
from std_msgs.msg import String

class pubRpi:

    def __init__(self):
        self.node = rospy.init_node('pub2rpi',anonymous = True)
        self.pubR = rospy.Publisher('cmd_rotate',String,queue_size = 10)
        self.pubS = rospy.Publisher('cmd_stepper',String,queue_size = 10)

    def publish_rotate(self,signal):
        valtoPub = '-1' if signal < 0 else '1'
        self.pubR.publish(valtoPub)

    def publish_stepper(self,signal):
        valtoPub = '-1' if signal < -5 else '10' if signal < 5 else '1'
        self.pubS.publish(valtoPub)