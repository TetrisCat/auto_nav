<a href="http://github.com/TetrisCat/auto_nav"><img src="http://emanual.robotis.com/assets/images/platform/turtlebot3/overview/turtlebot3_with_logo.png" title="FVCproductions" alt="FVCproductions"></a>

# 2020 EG2310 Group 7 Autonomous Navigation and Identification

> A collection of files accessed and used to accomplish the autonmous navigation and identification

> 13 weeks of grueling fun kill me now

> Currently known issue: closure script after mapping not running


[![Build Status](http://img.shields.io/travis/badges/badgerbadgerbadger.svg?style=flat-square)](https://travis-ci.org/badges/badgerbadgerbadger) 

- Tested and ran on ROS1 Kinetic

***INSERT ANOTHER GRAPHIC HERE***

[(https://imgur.com/a/fxZWLvI)]()


**Navigation**

![Navigation Demo](http://g.recordit.co/9nRlbP8baV.gif)

**Identification**

![Identification Demo](http://g.recordit.co/xX2mprxzCk.gif)

---

## Table of Contents (Optional)

> If your `README` has a lot of info, section headers might be nice.

- [Installation](#installation)
- [Team](#team)
- [Special Thanks](#special-thanks)

---

## Installation

- Clone this repo to your local machine using `https://github.com/TetrisCat/auto_nav`
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
pi@raspberrypi: ~ $ roslaunch turtlebot3_bringup  turtlebot3_rpicamera.
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

## Documentation (Optional)

> navigation.py
- Main python script: Requires turtlebot bringup and launch of move_base script
- Imports from occupancy.py and movebase.py

> occupancy.py
- Goal - based algorithm using RSLAM gmapping

> movebase.py
- Calls up move_base as that in navigation stack

> targeting.py
- Main script for identification and targeting
- Imports from impidentify.py 

> impidentify.py
- Contains class to subscribe to image topic and process it via OpenCV

---
## Tests (Optional)

- Used Turtlebot Gazebo for testing : http://wiki.ros.org/turtlebot_gazebo

<a href="http://wiki.ros.org/turtlebot_gazebo"><img src="https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png"> </a>

---

## Team

# AWESOME TEAM OF GROUP7

| <a href="https://sg.linkedin.com/in/tham-kai-wen-2679b2184" target="_blank">**Tham Kai Wen**</a> | **Teo Rumin** | **Cyril Aoun**|
| :---: |:---:| :---:|
| [![Tham Kai Wen](https://hcastro.org/temp_photos/kaiwen.JPG)](https://sg.linkedin.com/in/tham-kai-wen-2679b2184)    | ![Teo Rumin](https://www.shutterstock.com/image-vector/profile-blank-icon-empty-photo-male-535853269) | ![Cyril Aoun](https://www.shutterstock.com/image-vector/profile-blank-icon-empty-photo-male-535853269)   |

|**Adric Pang** | **Mohamed Faris** |
| :---: |:---:| 
| ![Adric Pang](https://www.shutterstock.com/image-vector/profile-blank-icon-empty-photo-male-535853269)   | ![Mohamed Faris](https://www.shutterstock.com/image-vector/profile-blank-icon-empty-photo-male-535853269) | 

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
    - No problem! Just do this.
---



