#!/bin/bash
# Domain bridge for LIVE NAVIGATION: domain 0 (wired, robot) <-> domain 1 (WiFi, laptop).
# Use this INSTEAD of run_loc_bridge.sh when navigating -- go2_nav_bridge.yaml is a
# superset (localization topics + /map, /plan, /cmd_vel, and the reverse /goal_pose).
#
# Each domain is bound to exactly ONE interface. Not a style choice: CycloneDDS
# advertises on the FIRST listed <NetworkInterface> only, and the Unitree bare-DDS
# lidar app only delivers on a single wired binding.
#
# NOTE: no taskset here. The old core-pinning scheme (Point-LIO on cores 6,7 at
# SCHED_FIFO) was measured to BREAK the estimator and was reverted, so pinning the
# bridge away from those cores no longer has any purpose.
#
# (no set -u: ROS setup.bash references unbound vars)

source /opt/ros/humble/setup.bash
source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# The bridge itself must NOT have a single fixed domain -- it joins both.
unset ROS_DOMAIN_ID

# WiFi interface name on the dog (USB dongle). Verify with `ip -brief addr`
# if the bridge reports it cannot find the interface.
WIFI_IF="wlxacf1df009552"
WIRED_IF="enP8p1s0"

read -r -d '' CDDS <<XMLEOF
<CycloneDDS>
  <Domain id="0">
    <General><Interfaces>
      <NetworkInterface name="${WIRED_IF}" priority="default" multicast="default"/>
    </Interfaces></General>
  </Domain>
  <Domain id="1">
    <General><Interfaces>
      <NetworkInterface name="${WIFI_IF}" priority="default" multicast="default"/>
    </Interfaces></General>
  </Domain>
</CycloneDDS>
XMLEOF
export CYCLONEDDS_URI="$CDDS"

echo "[run_nav_bridge] domain0=${WIRED_IF}  domain1=${WIFI_IF}"
exec ros2 run domain_bridge domain_bridge "$HOME/go2_nav_bridge.yaml"
