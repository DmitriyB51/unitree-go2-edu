#!/bin/bash
source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI="$(cat $HOME/cdds_wired.xml)"
exec taskset -c 0-5 python3 $HOME/health_log.py
