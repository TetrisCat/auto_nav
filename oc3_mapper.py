#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray,MultiArrayDimension
import math
import tf2_ros
import cmath
import numpy as np
import time

occdata = []
odata = []
occ_bins = [-1, 0, 100, 101]
cur_pose = ()
map_res = 0.05
dest = (0,0)
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

    rospy.loginfo(['Trans: ' + str(cur_pos)])
    rospy.loginfo(['Rot: ' + str(cur_rot)])

    # get map resolution
    map_res = msg.info.resolution
    # get map origin struct has fields of x, y, and z
    map_origin = msg.info.origin.position
    # get map grid positions for x, y position
    grid_x = round((cur_pos.x - map_origin.x) / map_res)
    grid_y = round(((cur_pos.y - map_origin.y) / map_res))
    rospy.loginfo(['Grid Y: ' + str(grid_y) + ' Grid X: ' + str(grid_x)])

    # make occdata go from 0 instead of -1, reshape into 2D
    oc2 = occdata + 1
    # set all values above 1 (i.e. above 0 in the original map data, representing occupied locations)
    oc3 = (oc2>1).choose(oc2,2)
    # reshape to 2D array using column order
    odata = np.uint8(oc3.reshape(msg.info.height,msg.info.width,order='F'))
    # set current robot location to 0
    # odata[grid_x][grid_y] = 0
    # create image from 2D array using PIL
    # find center of image
    # i_centerx = msg.info.width/2
    # i_centery = msg.info.height/2
    # translate by curr_pos - centerxy to make sure the rotation is performed
    # with the robot at the center
    # using tips from:
    # https://stackabuse.com/affine-image-transformations-in-python-with-numpy-pillow-and-opencv/
    # translation_m = np.array([[1, 0, (i_centerx-grid_y)],
    #                            [0, 1, (i_centery-grid_x)],
    #                            [0, 0, 1]])
    # # Image.transform function requires the matrix to be inverted
    # tm_inv = np.linalg.inv(translation_m)
    # # translate the image so that the robot is at the center of the image
    # img_transformed = img.transform((msg.info.height, msg.info.width),
    #                                 Image.AFFINE,
    #                                 data=tm_inv.flatten()[:6],
    #                                 resample=Image.NEAREST)

    # convert quaternion to Euler angles
    orientation_list = [cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    rospy.loginfo(['Yaw: R: ' + str(yaw) + ' D: ' + str(np.degrees(yaw))])

    
def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def get_laserscan(msg):
    global laser_range

    # create numpy array
    laser_range = np.array(msg.ranges)
    # replace 0's with nan's
    # could have replaced all values below msg.range_min, but the small values
    # that are not zero appear to be useful
    laser_range[laser_range==0] = np.nan

def get_closest():
    global odata
    global cur_pose
    global map_res

    x = {coord:value for coord,value in np.ndenumerate(odata)}
    routelst = []
    distancelst = []
    for k,v in x.items():
        if v == 2:
            i = k[0]
            j = k[1]
            counter = 0
            checkers = [-1,1]
            for checker in checkers:
                if x[(i+checker,j)] == 2:
                    counter +=1
                if x[(i,j+checker)] == 2:
                    counter +=1
            if counter <2:
                routelst.append(tuple(np.array(k)*map_res))
    
    for k in routelst:
        distancelst.append(distance(k,cur_pose))
    try:
        idx = np.argmin(distancelst)
        return routelst[idx]
    except ValueError:
        return (0,0)


def rotatebot(rot_angle):
    global yaw

    # create Twist object
    twist = Twist()
    # set up Publisher to cmd_vel topic
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # set the update rate to 1 Hz
    rate = rospy.Rate(1)

    # get current yaw angle
    current_yaw = np.copy(yaw)
    # log the info
    rospy.loginfo(['Current: ' + str(math.degrees(current_yaw))])
    # we are going to use complex numbers to avoid problems when the angles go from
    # 360 to 0, or from -180 to 180
    c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
    # calculate desired yaw
    target_yaw = rot_angle
    # convert to complex notation
    c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
    rospy.loginfo(['Desired: ' + str(math.degrees(cmath.phase(c_target_yaw)))])
    # divide the two complex numbers to get the change in direction
    c_change = c_target_yaw / c_yaw
    # get the sign of the imaginary component to figure out which way we have to turn
    c_change_dir = np.sign(c_change.imag)
    # set linear speed to zero so the TurtleBot rotates on the spot
    twist.linear.x = 0.0
    # set the direction to rotate
    twist.angular.z = c_change_dir * rotate_speed
    # start rotation
    pub.publish(twist)

    # we will use the c_dir_diff variable to see if we can stop rotating
    c_dir_diff = c_change_dir
    # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
    # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
    # becomes -1.0, and vice versa
    while(c_change_dir * c_dir_diff > 0):
        # get current yaw angle
        current_yaw = np.copy(yaw)
        # get the current yaw in complex form
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        rospy.loginfo('While Yaw: %f Target Yaw: %f', math.degrees(current_yaw), math.degrees(target_yaw))
        # get difference in angle between current and target
        c_change = c_target_yaw / c_yaw
        # get the sign to see if we can stop
        c_dir_diff = np.sign(c_change.imag)
        # rospy.loginfo(['c_change_dir: ' + str(c_change_dir) + ' c_dir_diff: ' + str(c_dir_diff)])
        rate.sleep()

    rospy.loginfo(['End Yaw: ' + str(math.degrees(current_yaw))])
    # set the rotation speed to 0
    twist.angular.z = 0.0
    # stop the rotation
    time.sleep(1)
    pub.publish(twist)

def find_angle(cpose,tpose):
    vector = [tpose[0]-cpose[0],tpose[1]-cpose[1]] 
    vector = vector / np.linalg.norm(vector)
    axisvec = [1,0] 
    axisvec = axisvec / np.linalg.norm(axisvec)
    return np.arccos(np.dot(vector,axisvec))


def pick_direction():
    global cur_pose
    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    dest = get_closest()
    rospy.loginfo('Desired Coord: ' + str(dest))
    try:
        t_yaw = find_angle(cur_pose,dest)
    except IndexError:
        t_yaw = 0
    # stop moving
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)



    rospy.loginfo('Picked destination: ' + str(dest))

    # rotate to that direction
    rotatebot(float(t_yaw))

    # start moving
    rospy.loginfo(['Start moving'])
    twist.linear.x = linear_speed
    twist.angular.z = 0.0
    # not sure if this is really necessary, but things seem to work more
    # reliably with this
    time.sleep(1)
    pub.publish(twist)


def stopbot():
    # publish to cmd_vel to move TurtleBot
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    time.sleep(1)
    pub.publish(twist)

def main():
    global laser_range
    global dest
    rospy.init_node('occ_map',anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(1.0)


    rospy.Subscriber('map',OccupancyGrid,get_occupancy,tfBuffer)
    rospy.sleep(0.5)
    rospy.Subscriber('scan', LaserScan, get_laserscan)

    rospy.on_shutdown(stopbot)

    rate =rospy.Rate(5)
    time.sleep(1)
    pick_direction()

    while not rospy.is_shutdown():
        if laser_range.size != 0:
            lri = (laser_range[front_angles]<float(stop_distance)).nonzero()
            rospy.loginfo('Distances: %s', str(lri))

        else:
            lri[0] = []

        if distance(cur_pose,dest) < 0.1:
            pick_direction()
        

        # if the list is not empty
        elif(len(lri[0])>0):
            rospy.loginfo(['Stop!'])
            # find direction with the largest distance from the Lidar
            # rotate to that direction
            # start moving
            pick_direction()

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass