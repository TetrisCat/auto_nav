#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


def callback(msg):
	# create numpy array
	laser_range = np.array([msg.ranges])
	# replace 0's with nan
	lr2 = laser_range
	lr2[lr2==0] = np.nan
	# find index with minimum value
	lr2i, lr2j = np.nanargmin(lr2), np.nanargmax(lr2)
	# log the info
        rospy.loginfo('Shortest distance is %i degrees %i with a distance of %j', lr2i, lr2[lr2i])
        rospy.loginfo('Furthest distance is %i degrees %i with a distance of %j', lr2j, lr2[lr2j])


def scanner():
	# initialize node
	rospy.init_node('scanner', anonymous=True)

	# set the update rate to 1 Hz
	rate = rospy.Rate(1) # 1 Hz

	# subscribe to LaserScan data
	rospy.Subscriber('scan', LaserScan, callback)
    
	# wait until it is time to run again
	rate.sleep()

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
    try:
	    scanner()
    except  rospy.ROSInterruptException:
        pass
