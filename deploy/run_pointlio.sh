#!/bin/bash
# Point-LIO on the robot, domain 0, WIRED-ONLY CycloneDDS.
# The Unitree bare-DDS lidar app only delivers to a single-wired-interface binding.
source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
source $HOME/dima_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

read -r -d '' CDDS <<'XMLEOF'
<CycloneDDS>
  <Domain>
    <General><Interfaces>
      <NetworkInterface name="enP8p1s0" priority="default" multicast="default"/>
    </Interfaces></General>
  </Domain>
</CycloneDDS>
XMLEOF
export CYCLONEDDS_URI="$CDDS"

exec ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false
