cmake_minimum_required(VERSION 3.16)
project(go2_navigation)

find_package(ament_cmake REQUIRED)

# В пакете нет C++ узлов — только launch-файлы, конфиги, карты и python-скрипты.
# Поэтому здесь мы просто раскладываем всё это в share/ и lib/.

# Launch-файлы, конфигурация Nav2, карты и настройки RViz.
install(DIRECTORY
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Python-скрипты генерации/чистки карты — ставим исполняемыми, чтобы работал
# `ros2 run go2_navigation bag_to_grid.py`.
install(PROGRAMS
  scripts/bag_to_grid.py
  scripts/goal_to_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
