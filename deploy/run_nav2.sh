#!/bin/bash
# Nav2 on the robot, domain 0, WIRED-ONLY CycloneDDS.
#
# Brings up ONLY the Nav2 layer: tf_setup + map_server + planner + controller +
# lifecycle + goal bridge. It does NOT move the robot: /cmd_vel goes nowhere until
# you separately start run_vel_ctrl.sh. That separation is deliberate (safety step A).
#
# PREREQUISITES (in this order):
#   1. run_pointlio.sh   -- odometry + /registered_scan
#   2. run_matcher.sh    -- localization, gives TF map->camera_init
#   3. this script
#   4. run_nav_bridge.sh -- so RViz on the laptop can see /map and send /goal_pose
#   5. run_vel_ctrl.sh   -- ONLY when you actually want the legs to move
#
# ⛔ NEVER also run the CMU stack (system_real_robot.launch): its pathFollower
#    publishes /cmd_vel AND /api/sport/request -> a second driver fighting us.
#
# (no set -u: ROS setup.bash references unbound vars)

source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
source $HOME/dima_ws/install/setup.bash      # go2_navigation must be built here too
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

# map:= can be overridden, but the DEFAULT (building.yaml) is the one that pairs
# with loc_5_map.pcd used by the matcher. Do not mix map sessions.
echo "[run_nav2] starting Nav2 (no motion until run_vel_ctrl.sh is started)"
exec ros2 launch go2_navigation nav2_live.launch.py "$@"
