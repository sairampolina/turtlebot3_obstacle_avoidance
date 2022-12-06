# turtlebot3_obstacle_avoidance
This project involves the development of an basic obstacle avoidance algorithm for turtlebot3 robot in ROS2 Humble.

## Installing Ros2 on Ubuntu 22.04
- ros2 can be installed from binary package or source files.
- Installation from binary(Debian Packag) is preferred.
- Use this [link](https://docs.ros.org/en/humble/Installation.html) to install Ros2 Humble.

## Video:
https://user-images.githubusercontent.com/48856345/205806810-128ff898-8780-4f3b-a4dc-a1b6715c67ec.mp4

## Following dependent packages are required to run this ros package:
### To install:

run below commands only if you have installed ROS2 Humble, from binary packages(Debian Package).

- turtlebot3 pkg:
```
sudo apt install ros-humble-turtlebot3*
```

- gazebo_ros_packages:
```
sudo apt install ros-foxy-gazebo-ros-pkgs
```

## Clone the repo in to the src folder of your ROS Workspace:
```
cd ~/ros2_ws/src
git clone https://github.com/sairampolina/turtlebot3_obstacle_avoidance.git
```
## Install dependencies for present package by:
- from the root of your ros workspace run below command.
- Any missing dependencies are notified.

```
rosdep install -i --from-path src --rosdistro humble -y
```
## Build your package
from the root of your ros workspace run below command.
```
colcon build --packages-select tb3_obstacle_avoidance_pkg
```
## Source setup files
```
. install/setup.bash
```
## To launch walker algorithm on turtlebot3:
```
ros2 launch tb3_obstacle_avoidance_pkg walker_algorithm_launch.py
```
## To launch walker algorithm and record all topics except /camera topic (using ROS bag) :
- Navigate to the folder where you want to store your bag
- ros2 bag parameters can be found in launch file

```
ros2 launch tb3_obstacle_avoidance_pkg walker_algorithm_launch.py record_topics:=True
```
## To check the content of ROS bag:
```
ros2 bag info tb3_walker_bag/
```
## To play the ROS bag:
```
ros2 bag play tb3_walker_bag/
```
## Static code analysis
### Cpplint
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp &> ../results/cpplint.txt
```
### Cppcheck
```
cppcheck --enable=all --std=c++17 src/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ../results/cppcheck.txt
```
