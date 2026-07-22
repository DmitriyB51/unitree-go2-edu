#!/bin/bash
# RViz на НОУТБУКЕ для живой локализации по WiFi.
#
# Ноутбук сидит в домене 1 (WiFi). Собака работает в домене 0 (провод, лидар).
# Их связывает domain_bridge, запущенный НА СОБАКЕ (~/run_loc_bridge.sh).
#
# ВАЖНО: не запускай это из "обычного" терминала пользователя — там bashrc
# подгружает Isaac Sim и conda, которые ломают ROS. Поэтому ниже env -i:
# стартуем с чистым окружением и вручную добавляем только то, что нужно.
#
# (без set -u: ROS setup.bash обращается к неопределённым переменным)

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI="file://${REPO}/env/cdds_laptop_wifi.xml"

echo "[run_rviz_wifi] domain=1  iface=wlo1  config=${REPO}/env/cdds_laptop_wifi.xml"

exec env -i \
  HOME="$HOME" \
  DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="$HOME/.Xauthority" \
  RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  CYCLONEDDS_URI="$CYCLONEDDS_URI" \
  bash -lc "source /opt/ros/humble/setup.bash && exec rviz2 -d ${REPO}/go2_localization/rviz/localization.rviz"
