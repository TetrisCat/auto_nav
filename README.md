<a href="http://github.com/TetrisCat/auto_nav"><img src="http://emanual.robotis.com/assets/images/platform/turtlebot3/overview/turtlebot3_with_logo.png" title="FVCproductions" alt="FVCproductions"></a>

# 2020 EG2310 Group 7 Autonomous Navigation and Identification

> A collection of files accessed and used to accomplish the autonmous navigation and identification

> 13 weeks of grueling fun kill me now

> Currently known issue: closure script after mapping not running


[![Build Status](http://img.shields.io/travis/badges/badgerbadgerbadger.svg?style=flat-square)](https://travis-ci.org/badges/badgerbadgerbadger) 

- Tested and ran on ROS1 Kinetic

![Turtlebot3 CAD Design](https://i.imgur.com/hSwf48f.jpg)

**Navigation**

![Navigation Demo](http://g.recordit.co/XOr6LkQzB8.gif)

**Identification**

![Identification Demo](http://g.recordit.co/2K4EPKrPUP.gif)

---

## Table of Contents 

- [Installation](#installation)
- [Usage](#usage)
- [Documentation](#documentation)
- [Team](#team)
- [Special Thanks](#special-thanks)

---

## Installation

- Clone this repo to your local machine using `git clone https://github.com/TetrisCat/auto_nav`
- Make sure this is in your catkin workspace

### Setup

- Run the following from terminal line:

```shell
$ cd ~/catkin_ws && catkin_make
```

### Dependant Packages

**Follow the installation guide from :**

- http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/

**Other Important Dependant Package:**

- http://wiki.ros.org/navigation 

- http://wiki.ros.org/sound_play 

- https://github.com/adricpjw/eg2310_nav 

***FOR RPI CODE***

- https://github.com/adricpjw/rpi_2310 

---
## Usage

- Files are located in /nav directory

### Navigation:

***Turtlebot3***

- Bring up turtlebot
```shell
pi@raspberrypi: ~ $ roslaunch turtlebot3_bringup  turtlebot3_robot.launch
```

- Launch Move_base node
```shell
pi@raspberrypi: ~ $ roslaunch eg2310 turtlebot3_nav_sim.launch
```
***Remote PC***

- Run the navigation script
```shell
$ rosrun auto_nav navigation.py
```
### Identification

***Turtlebot3***

- Bringup turtlebot
```shell
pi@raspberrypi: ~ $ roslaunch turtlebot3_bringup  turtlebot3_robot.launch
```

- Launch rpicamera
> Might have to follow steps as listed in http://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/
```shell
pi@raspberrypi: ~ $ roslaunch turtlebot3_bringup  turtlebot3_rpicamera.launch
```
- Run actuator scripts
```shell
pi@raspberrypi: /rpi_2310$ python finalfiring.py
```
***Remote PC***

- Launch Image Viewer (OPTIONAL)
```shell
$ rqt_image_view
```

- Run targeting script:
```shell
$ rosrun auto_nav targeting.py
```
---
## Documentation 

### navigation.py
- Main python script: Requires turtlebot bringup and launch of move_base script
- Imports from occupancy.py and movebase.py

### occupancy.py
- Goal - based algorithm using RSLAM gmapping
- `get_occupancy(self,msg,tbuffer)` takes data from [nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)
    - It finds the boundary lines between Unoccupied(0) and Unknown(-1)
    - It picks a new target goal using `get_closest`
- `get_closest(self,original,adjusted,curpos,res,origin)` picks a target from a list of boundary points
    - Calculates distance from current position to each boundary point and picks median
    - To change this to Find nearest boundary point, adjust `mind = getMap()` in **navigation.py** to `mind = getMap('closest')`
- `closure(self)` checks if the mapping has been completed by checking for map contours
    - Adjust **ALTHRESH** value up or down to have a higher or lower tolerance for completed mapping

### movebase.py
- Calls up move_base as that in navigation stack
- `goto(self, pos, quat)` is the main class method to moe the turtlebot to target goal
    - Adjust `self.move_base.wait_for_result(rospy.Duration(4))` value if need be to optimise performance
- More move_base parameters to adjust can be found at https://github.com/adricpjw/eg2310_nav

### targeting.py
- Main script for identification and targeting
- Imports from **impidentify.py**
- Large parts were commented due to faulty stepper and unintegrated navigation + targeting. These can be uncommented for a fully functional integrated solution
- `detector = Detect('red')` initialises the class Detect to detect red targets. Change the color if necessary

### impidentify.py
- Contains class to subscribe to image topic and process it via OpenCV
- `self.imgH` and `self.imgW` can be adjusted for cameras with different resolution
    - Rpi Camera v2 has resolution of 640 x 480
- `self.minH` and `self.minW` determines the minimum pixel height and width of a contour to be considered as the target
    - Can be adjusted accordingly for performance
- `self.mapping` includes the hsv ranges for the different colors. More colors can be added accordingly
- If script cannot detect the colored targets correctly, consider uncommenting the *global_hsv* lines in **readImg** and use mouse to click on screen to find the target's hsv range


---
## Tests

- Used [Turtlebot Gazebo](http://wiki.ros.org/turtlebot_gazebo) for testing : http://wiki.ros.org/turtlebot_gazebo

<a href="http://wiki.ros.org/turtlebot_gazebo"><img src="https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png"> </a>

---

## Team

# AWESOME TEAM OF GROUP7

| **Cyril Aoun the COSMONAUT** | **Cyril Aoun the PRINCESS LEIA** | 
| :---: | :---: |
| ![Cyril Aoun](https://i.imgur.com/8KMpmmK.jpg)   | ![Cyril Aoun](https://i.imgur.com/BQMStDN.jpg) | 

|**Adric Pang, Mohamed Faris, Tham Kai Wen, Teo Ru Min** |
| :---: |
| ![Team7](https://i.imgur.com/AMvtMr6.jpg)  |

---
## Special Thanks

# HUGE shoutout to the following people who were amazing:

- Sim Zhi Min
- Eugene EE
- Dr Yen
- Soh Eng Keng
- Ms Annie

---
## FAQ

- **How do I do this?**
    - Google!
---



