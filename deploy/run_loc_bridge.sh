#!/bin/bash
# Domain bridge for live localization: domain 0 (wired, robot) <-> domain 1 (WiFi, laptop).
#
# Each domain is bound to exactly ONE interface. That is not a style choice:
# CycloneDDS advertises on the FIRST listed <NetworkInterface> only, and the
# Unitree bare-DDS lidar app only delivers on a single wired binding.
#
# Pinned to cores 0-5 so it can never steal a core from Point-LIO (6,7).
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

echo "[run_loc_bridge] domain0=${WIRED_IF}  domain1=${WIFI_IF}  cores=0-5"
exec taskset -c 0-5 ros2 run domain_bridge domain_bridge "$HOME/go2_loc_bridge.yaml"
