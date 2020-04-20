<a href="http://github.com/TetrisCat/auto_nav"><img src="http://emanual.robotis.com/assets/images/platform/turtlebot3/overview/turtlebot3_with_logo.png" title="FVCproductions" alt="FVCproductions"></a>

# 2020 EG2310 Group 7 Autonomous Navigation and Identification

> A collection of files accessed and used to accomplish the autonmous navigation and identification

> 13 weeks of grueling fun kill me now

> Currently known issue: closure script after mapping not running


[![Build Status](http://img.shields.io/travis/badges/badgerbadgerbadger.svg?style=flat-square)](https://travis-ci.org/badges/badgerbadgerbadger) 

- Tested and ran on ROS1 Kinetic

***INSERT ANOTHER GRAPHIC HERE***

[(https://imgur.com/a/fxZWLvI)]()

- Use <a href="http://recordit.co/" target="_blank">**Recordit**</a> to create quicks screencasts of your desktop and export them as `GIF`s.
- For terminal sessions, there's <a href="https://github.com/chjj/ttystudio" target="_blank">**ttystudio**</a> which also supports exporting `GIF`s.

**Recordit**

![Recordit GIF](http://g.recordit.co/iLN6A0vSD8.gif)

**ttystudio**

![ttystudio GIF](https://raw.githubusercontent.com/chjj/ttystudio/master/img/example.gif)

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
### Identificaiton

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

### Teo Rumin

### Tham Kai Wen
<a href = "https://sg.linkedin.com/in/tham-kai-wen-2679b2184"><img src="https://hcastro.org/temp_photos/kaiwen.JPG"></a>

### Cyril Aoun

### Adric Pang

### MAGIK MAN 



| <a href="http://fvcproductions.com" target="_blank">**FVCproductions**</a> | <a href="http://fvcproductions.com" target="_blank">**FVCproductions**</a> | <a href="http://fvcproductions.com" target="_blank">**FVCproductions**</a> |
| :---: |:---:| :---:|
| [![FVCproductions](https://avatars1.githubusercontent.com/u/4284691?v=3&s=200)](http://fvcproductions.com)    | [![FVCproductions](https://avatars1.githubusercontent.com/u/4284691?v=3&s=200)](http://fvcproductions.com) | [![FVCproductions](https://avatars1.githubusercontent.com/u/4284691?v=3&s=200)](http://fvcproductions.com)  |
| <a href="http://github.com/fvcproductions" target="_blank">`github.com/fvcproductions`</a> | <a href="http://github.com/fvcproductions" target="_blank">`github.com/fvcproductions`</a> | <a href="http://github.com/fvcproductions" target="_blank">`github.com/fvcproductions`</a> |

- You can just grab their GitHub profile image URL
- You should probably resize their picture using `?s=200` at the end of the image URL.

---

## FAQ

- **How do I do *specifically* so and so?**
    - No problem! Just do this.
---

## Support

Reach out to me at one of the following places!

- Website at <a href="http://fvcproductions.com" target="_blank">`fvcproductions.com`</a>
- Twitter at <a href="http://twitter.com/fvcproductions" target="_blank">`@fvcproductions`</a>
- Insert more social links here.

---

## Donations (Optional)

- You could include a <a href="https://cdn.rawgit.com/gratipay/gratipay-badge/2.3.0/dist/gratipay.png" target="_blank">Gratipay</a> link as well.

[![Support via Gratipay](https://cdn.rawgit.com/gratipay/gratipay-badge/2.3.0/dist/gratipay.png)](https://gratipay.com/fvcproductions/)


---

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org)

- **[MIT license](http://opensource.org/licenses/mit-license.php)**
- Copyright 2015 © <a href="http://fvcproductions.com" target="_blank">FVCproductions</a>.
