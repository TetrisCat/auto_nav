#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, String
from PIL import Image
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import cv2
import tf2_ros
import cmath
import time

from occupancy import distance, getMap
from closure import closure
from movebase import GoToPose

toggle = True
stored_pose = ()
stored_quat = ()


def stopbot():
    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)

def main():
    mind = getMap()
    rospy.sleep(2.0)
    # save start time
    start_time = time.time()
    # initialize variable to write elapsed time to file
    contourCheck = 1

    navigator = GoToPose()
    position = {'x':mind.target[0], 'y' : mind.target[1]}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    navigator.goto(position, quaternion)

    rospy.on_shutdown(stopbot)
    rate = rospy.Rate(10)

    shutdown = False

    while not shutdown:
        while not cmd_rotate:

            if contourCheck == 1:
                if closure(odata):
                    # map is complete, so save current time into file
                    with open("maptime.txt", "w") as f:
                        f.write("Elapsed Time: " + str(time.time() - start_time))
                    contourCheck = 5
                    # play a sound
                    soundhandle = SoundClient()
                    rospy.sleep(1)
                    soundhandle.stopAll()
                    soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
                    rospy.sleep(2)
                    # save the map
                    cv2.imwrite('mazemap.png',occdata)
                    pub = rospy.Publisher('mapdone',String)
                    pub.publish('Done!')
                    rospy.sleep(1)
                    shutdown = True
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
        

