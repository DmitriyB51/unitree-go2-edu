# ~/ros_env.sh -- source this in any NEW ssh window before running ros2 by hand.
#   source ~/ros_env.sh
#
# The run_*.sh scripts already do all of this internally; an interactive shell
# does not, which is why a fresh window says "ros2: command not found".
#
# The CycloneDDS block is NOT optional: CycloneDDS advertises on the FIRST listed
# interface only, so without it `ros2 node list` finds nothing and gives no error.
# (no set -u: ROS setup.bash reads unbound vars)

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

echo "[ros_env] ROS Humble + unitree + dima_ws | domain 0 | cyclonedds on enP8p1s0"
