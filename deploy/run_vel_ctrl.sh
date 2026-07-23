#!/bin/bash
# ⚠️⚠️  THIS IS THE SCRIPT THAT MAKES THE ROBOT ACTUALLY WALK.  ⚠️⚠️
#
# vel_ctrl_repub: subscribes /cmd_vel (Nav2) -> calls sport_req.Move(vx,vy,vyaw)
# -> publishes /api/sport/request -> Unitree onboard gait controller -> legs.
#
# SAFETY DESIGN BUILT INTO THE NODE:
#   * /cmd_vel timeout 0.5 s  -> if Nav2 dies or WiFi drops, the robot STOPS
#                                (it does not keep walking on the last command).
#   * gamepad /joy override   -> if a js0 gamepad is connected, moving a stick
#                                takes control away from Nav2 (deadzone 0.05).
#   * speeds are capped in nav2_params.yaml: 0.25 m/s forward, 0.4 rad/s turn.
#
# ⚠️ THE FACTORY UNITREE REMOTE DOES NOT PASS THROUGH THIS NODE.
#    It commands the onboard sport controller DIRECTLY (firmware level), i.e. in
#    parallel with our Move() calls. BEFORE trusting it as an E-STOP you MUST run
#    the priority test in RUN_NAV_LIVE.md ("кто кого перебивает"). Do not assume.
#
# BEFORE RUNNING THIS:
#   - run_pointlio.sh / run_matcher.sh / run_nav2.sh are up and healthy
#   - you have watched /cmd_vel and it looks sane (step A in the runbook)
#   - the robot is in an open area, nobody in front of it, hand on the remote
#
# (no set -u: ROS setup.bash references unbound vars)

source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
# go2_sport_api lives in the CMU stack workspace on the dog:
if [ -f "$HOME/autonomy_stack_go2/install/setup.bash" ]; then
  source "$HOME/autonomy_stack_go2/install/setup.bash"
else
  source "$HOME/dima_ws/install/setup.bash"
fi
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

echo "[run_vel_ctrl] ⚠️  LEGS ARE NOW LIVE — /cmd_vel will drive the robot."
echo "[run_vel_ctrl] prints [NAV2]/[JOY]/[STOP] at ~5 Hz so you can see who is driving."
exec ros2 run go2_sport_api vel_ctrl
