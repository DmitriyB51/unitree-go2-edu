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

# ►► ВЫБОР КОНФИГА RViz (аргумент, по умолчанию nav):
#      nav  — навигация: есть инструмент "Nav2 Goal" + дисплеи /map, /plan, costmap
#      loc  — только локализация: облако прайор-карты, поза, "2D Pose Estimate"
#    Симптом, что взят не тот: в панели инструментов НЕТ кнопки "Nav2 Goal",
#    только "2D Pose Estimate" -> это конфиг loc, целей им не поставить.
case "${1:-nav}" in
  nav) RVIZ_CFG="${REPO}/go2_navigation/rviz/nav2.rviz" ;;
  loc) RVIZ_CFG="${REPO}/go2_localization/rviz/localization.rviz" ;;
  *)   RVIZ_CFG="$1" ;;          # можно передать свой путь к .rviz
esac

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI="file://${REPO}/env/cdds_laptop_wifi.xml"

echo "[run_rviz_wifi] domain=1  iface=wlo1  rviz=${RVIZ_CFG}"

exec env -i \
  HOME="$HOME" \
  DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="$HOME/.Xauthority" \
  RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  CYCLONEDDS_URI="$CYCLONEDDS_URI" \
  bash -lc "source /opt/ros/humble/setup.bash && exec rviz2 -d ${RVIZ_CFG}"
