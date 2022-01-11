# RoboND - Project3: Where I Am
Project 3 of Udacity Robotics Software Engineer Nanodegree Program

![Overview](sample.gif)  

## Overview  
In this project you'll utilize ROS AMCL package to accurately localize a mobile robot inside a map in the Gazebo simulation environments. Here are the steps to learn several aspects of robotic software engineering with a focus on ROS:  
* Create a ROS package that launches a custom robot model in a custom Gazebo world  
* Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot  
* Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results  
## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. Build and run your code.  
## Project Description  
Directory Structure  
```
.Where-I-Am                                    
└── catkin_ws
    ├── build
    │   ├── atomic_configure
    │   ├── catkin
    │   │   └── catkin_generated
    │   │       └── version
    │   ├── catkin_generated
    │   │   ├── installspace
    │   │   └── stamps
    │   │       └── Project
    │   ├── CMakeFiles
    │   │   ├── 3.5.1
    │   │   │   ├── CompilerIdC
    │   │   │   └── CompilerIdCXX
    │   │   ├── clean_test_results.dir
    │   │   ├── CMakeTmp
    │   │   ├── download_extra_data.dir
    │   │   ├── doxygen.dir
    │   │   ├── run_tests.dir
    │   │   └── tests.dir
    │   ├── gtest
    │   │   ├── CMakeFiles
    │   │   │   ├── gmock.dir
    │   │   │   │   ├── src
    │   │   │   │   └── usr
    │   │   │   │       └── src
    │   │   │   │           └── gtest
    │   │   │   │               └── src
    │   │   │   └── gmock_main.dir
    │   │   │       ├── src
    │   │   │       └── usr
    │   │   │           └── src
    │   │   │               └── gtest
    │   │   │                   └── src
    │   │   └── gtest
    │   │       └── CMakeFiles
    │   │           ├── gtest.dir
    │   │           │   └── src
    │   │           └── gtest_main.dir
    │   │               └── src
    │   ├── my_robot
    │   │   ├── catkin_generated
    │   │   │   ├── installspace
    │   │   │   └── stamps
    │   │   │       └── my_robot
    │   │   └── CMakeFiles
    │   │       └── _catkin_empty_exported_target.dir
    │   ├── pgm_map_creator
    │   │   ├── catkin_generated
    │   │   │   ├── installspace
    │   │   │   └── stamps
    │   │   │       └── pgm_map_creator
    │   │   ├── CMakeFiles
    │   │   │   ├── collision_map_creator.dir
    │   │   │   │   └── src
    │   │   │   ├── request_publisher.dir
    │   │   │   │   └── src
    │   │   │   ├── roscpp_generate_messages_cpp.dir
    │   │   │   ├── roscpp_generate_messages_eus.dir
    │   │   │   ├── roscpp_generate_messages_lisp.dir
    │   │   │   ├── roscpp_generate_messages_nodejs.dir
    │   │   │   ├── roscpp_generate_messages_py.dir
    │   │   │   ├── rosgraph_msgs_generate_messages_cpp.dir
    │   │   │   ├── rosgraph_msgs_generate_messages_eus.dir
    │   │   │   ├── rosgraph_msgs_generate_messages_lisp.dir
    │   │   │   ├── rosgraph_msgs_generate_messages_nodejs.dir
    │   │   │   ├── rosgraph_msgs_generate_messages_py.dir
    │   │   │   ├── std_msgs_generate_messages_cpp.dir
    │   │   │   ├── std_msgs_generate_messages_eus.dir
    │   │   │   ├── std_msgs_generate_messages_lisp.dir
    │   │   │   ├── std_msgs_generate_messages_nodejs.dir
    │   │   │   └── std_msgs_generate_messages_py.dir
    │   │   └── msgs
    │   │       └── CMakeFiles
    │   │           └── collision_map_creator_msgs.dir
    │   └── test_results
    ├── devel
    │   ├── lib
    │   │   ├── pgm_map_creator
    │   │   └── pkgconfig
    │   └── share
    │       ├── my_robot
    │       │   └── cmake
    │       └── pgm_map_creator
    │           └── cmake
    └── src
        ├── my_robot
        │   ├── config
        │   │   └── __MACOSX
        │   ├── launch
        │   ├── maps
        │   ├── meshes
        │   ├── urdf
        │   └── worlds
        │       └── worldchasesim
        └── pgm_map_creator
            ├── launch
            ├── maps
            ├── msgs
            ├── src
            └── world


```
## Run the project 
* Clone this repository
```
git clone https://github.com/margrammas/WhereIAm.git
```
* Open the repository and make  
```
cd /home/workspace/WhereIAm/
catkin_make
source devel/setup.bash
```
* Open two different terminal windows and launch respectively my_robot in Gazebo to load both the world and plugins  
```
roslaunch my_robot world.launch
```  
* And launch amcl node  
```
roslaunch my_robot amcl.launch
```  
* Testing  
You have two options to control your robot while it localize itself here:  
  * Send navigation goal via RViz  
  * Send move command via teleop package.  
Navigate your robot, observe its performance and tune your parameters for AMCL.  

**Selected Option: Send `2D Navigation Goal`**  
Your first option would be sending a `2D Nav Goal` from RViz. The `move_base` will try to navigate your robot based on the localization. Based on the new observation and the odometry, the robot to further perform the localization.  
Click the `2D Nav Goal` button in the toolbar, then click and drag on the map to send the goal to the robot. It will start moving and localize itself in the process. If you would like to give `amcl` node a nudge, you could give the robot an initial position estimate on the map using `2D Pose Estimate`.  

You could control your robot by keyboard commands now.  

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```bash
sudo apt-get update && sudo apt-get upgrade -y
