# MapMyWorld

This is the third ROS project from the Udacity [Robotics Software Engineer](https://www.udacity.com/course/robotics-software-engineer--nd209) course.

## Overview

The robot's task is to create a 2D occupancy grid and a 3D octomap from a simulated environment using the [RTAB-Map](http://wiki.ros.org/rtabmap_ros) (Real-Time Appearance-Based Mapping) ROS package.

### License

The source code is released under an [MIT license.](https://opensource.org/licenses/MIT)

**Author: Andres Campos**

The packages: ```my_robot```, ```teleop```, and ```rtabmap_ros``` packages have been tested under  [ROS](https://www.ros.org/) Kinetic on Ubuntu 16.04. This code is for personal learning and any fitness for a paticular purpose is disclaimed.

## Installation

### Installation from Packages
To install all packages from this repository as Debian packages use

```sudo apt-get install ros-kinetic-...```

Or use ```rosdep```:

```sudo rosdep install --from-paths src```

### Building from Source

#### Dependencies
* [Robotics Operating System (ROS)](https://www.ros.org/) (middleware for robotics)

```sudo rosdep install --from-paths src```

* [RTAB-Map](http://wiki.ros.org/rtabmap_ros) (creates a 2D occupancy grid map and/or a 3D octomap for navigation)

```
cd /catkin_ws/src
git clone https://github.com/introlab/rtabmap_ros.git
```

* [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) (allows the user to teleoperate a robot)

```
cd /catkin_ws/src
git clone https://github.com/ros-teleop/teleop_twist_keyboard.git
```

#### Building
To build from source, clone the latest version from this repository into your catkin workspace and compile this package using
```
cd catkin_workspace/src
git clone https://github.com/acampos074/MapMyWorld.git
cd ../
catkin_make
```

## Usage

Navigate to the ```/src/scripts``` directory, and launch the shell script:

```./map_my_world.sh```

## Launch files
* **world.launch**: Gazebo launch and world file
* **mapping.launch**: Launch file of the mapping node

## Nodes

### **teleop_twist_keyboard**

**Published Topics**
* **```/cmd_vel```** [(geometry_msgs/Twist)](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html)

    Reads the user's keystroke commands and sends the corresponding linear and angular velocities to move the robot

### **rtabmap**

**Subscribed Topics**
* **```/camera/rgb/camera_info```** [(sensor_msgs/CameraInfo)](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)

    RGB camera metadata.

* **```/camera/rgb/image_raw```** [(sensor_msgs/Image)](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)

    RGB/Mono image.

* **```/camera/depth/image_raw```** [(sensor_msgs/Image)](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html)

     Registered depth image.

* **```/scan```** [(sensor_msgs/LaserScan)](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)

    Laser scan stream.

**Published Topics**

* **```/grid_map```** [(nav_msgs/OccupancyGrid)](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)

    Mapping Occupancy grid generated with laser scans.

**Parameters**
* **```subscribe_depth```** (bool,"true")

  Subscribe to depth image.

* **```subscribe_scan```** (bool,"true")

  Subscribe to laser scan.

* **```frame_id```** (string,"robot_footprint")

  Frame attached to the mobile robot.

* **```odom_frame_id```** (string,"odom")

  Frame attached to the odometry.

* **```database_path```**  (string,"~/.ros/rtabmap_1.db")

  Path of the RTAB-Map's database.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/acampos074/MapMyWorld/issues)
