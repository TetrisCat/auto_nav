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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray,MultiArrayDimension
from scipy import signal
from PIL import Image
import tf2_ros
import cmath
import time


occdata = []
odata = []
occ_bins = [-1, 0, 100, 101]
cur_pose = ()
map_res = 0.05
target = (0,0)
stop_distance = 0.25
yaw = 0.0
rotate_speed = 0.2
linear_speed = 0.1
laser_range = np.array([])
front_angle = 10
front_angles = range(-front_angle,front_angle+1,1)

def get_occupancy(msg, tfBuffer):
    global yaw
    global map_res
    global cur_pose
    global odata
    global target

    # create numpy array
    occdata = np.array([msg.data])
    # compute histogram to identify percent of bins with -1
    occ_counts = np.histogram(occdata,occ_bins)
    # calculate total number of bins
    total_bins = msg.info.width * msg.info.height
    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
    cur_pos = trans.transform.translation
    cur_rot = trans.transform.rotation
    cur_pose = (cur_pos.x,cur_pos.y)
    rospy.loginfo('Current Pose: x: %s, y: %s', str(cur_pose[0]),str(cur_pose[1]))

    # rospy.loginfo(['Trans: ' + str(cur_pos)])
    # rospy.loginfo(['Rot: ' + str(cur_rot)])

    # get map resolution
    map_res = msg.info.resolution
    # get map origin struct has fields of x, y, and z
    map_origin = msg.info.origin.position
    # get map grid positions for x, y position
    # grid_x = round((cur_pos.x - map_origin.x) / map_res)
    # grid_y = round((cur_pos.y - map_origin.y) / map_res)
    # rospy.loginfo(['Grid Y: ' + str(grid_y) + ' Grid X: ' + str(grid_x)])

    # make occdata go from 0 instead of -1, reshape into 2D
    oc2 = occdata + 1
    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
    oc3 = (oc2>1).choose(oc2,2)
    # reshape to 2D array using column order
    odata = np.uint8(oc3.reshape(msg.info.height,msg.info.width,order='F'))
    matrix = np.transpose(np.asarray(odata))
    adjusted = []
    for row in matrix:
        x = [num if num == 1 else 0 if num >=2 else 10 for num in row]
        adjusted.append(x)
    kernel = np.zeros((3,3)) + 1
    #convoluting the matrix with kernel to sum up neigbors
    #idea is to find boundary between 0s and 1s
    output = signal.convolve2d(adjusted, kernel, boundary='fill',mode='same')
    target = get_closest(matrix,output,cur_pose,map_res,(map_origin.x,map_origin.y))
    rospy.loginfo('target coord: (%s,%s)',str(target[0]),str(target[1]))


    # convert quaternion to Euler angles
    orientation_list = [cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # rospy.loginfo(['Yaw: R: ' + str(yaw) + ' D: ' + str(np.degrees(yaw))])

def stopbot():
    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)

def goToGoal(target):
    navigator = GoToPose()

    # Customize the following values so they are appropriate for your location

    position = {'x':target[0], 'y' : target[1]}
    quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    navigator.goto(position, quaternion)

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)



def get_closest(original,adjusted,curpos,res,origin):
    lst = []
    for coord,val in np.ndenumerate(adjusted):
        row = coord[0]
        col = coord[1]
        if val % 10 != 0 and original[row][col] == 0 and val > 30:
            lst.append((col*res + origin[0],row*res + origin[1]))
    # rospy.loginfo(lst)    
    distance_lst = [distance(crd,curpos) for crd in lst]

    idx = np.argmin(distance_lst)
    return lst[idx]


class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(2))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(5)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

def main():
    global target

    rospy.init_node('nav_test',anonymous=False)
    
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1)
    rospy.Subscriber('map',OccupancyGrid,get_occupancy,tfBuffer)
    rospy.sleep(3.0)

    goToGoal(target)

    rospy.on_shutdown(stopbot)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        goToGoal(target)
        rate.sleep()

        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
        

