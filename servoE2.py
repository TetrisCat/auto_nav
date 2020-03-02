import time
import RPi.GPIO as GPIO
import numpy as np 
from sensor_msgs.msg import LaserScan

GPIO.setmode(GPIO.BOARD)
laser_range = np.array([])
servo_point = 32
sol_point = 40
front_angle = 1
front_angles = range(-front_angle, front_angle+1, 1)

GPIO.setup(servo_point,GPIO.OUT)
GPIO.setup(sol_point,GPIO.OUT)

pwm = GPIO.PWM(servo_point,50)
pwm.start(2.5)
time.sleep(1)

def find_nearest(array,value):
    array = np.asarray(array)
    idx = (np.abs(array-value)).argmin()
    return array[idx]

def get_laserscan(msg):
    global laser_range
    laser_range = np.array([msg.range])


def scan_forServo():
    rospy.init_node('scan_forServo', anonymous = True)

    rospy.Subscriber('scan', LaserScan, get_laserscan)

    lr2 = laser_range[0,front_angles]
    nearestToOne = find_nearest(lr2,1.0)
    rospy.loginfo(nearestToOne)
    if nearestToOne - 1 <= 0.05 and nearestToOne -1 >= - 0.05:
        pwm.ChangeDutyCycle(5)
        GPIO.output(sol_point,GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(sol_point,GPIO.LOW)
        time.sleep(0.5)
        rospy.loginfo('Distance of %f located, firing plunger',nearestToOne)
        time.sleep(0.5)

    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        scan_forServo()
    except ROSInterruptException:
        pass
