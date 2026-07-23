"""
nav2_live.launch.py — Nav2 НА РОБОТЕ (Этап 6). Без заезда, без RViz, без стека CMU.

ЧТО ЭТО ПОДНИМАЕТ (только слой Nav2):
  tf_setup + map_server + planner_server + controller_server + lifecycle_manager + мост цели.

ЧЕГО ЗДЕСЬ НАМЕРЕННО НЕТ (и почему):
  • Point-LIO      — запускается своим скриптом deploy/run_pointlio.sh
  • matcher        — свой скрипт deploy/run_matcher.sh (даёт TF map->camera_init)
  • RViz           — крутится на НОУТБУКЕ через domain_bridge, не на собаке
  • vel_ctrl (мост /cmd_vel -> лапы) — ⭐ ОТДЕЛЬНО, deploy/run_vel_ctrl.sh.
    Это СПЕЦИАЛЬНО: пока мост не запущен, робот НЕ МОЖЕТ поехать. Так проходится
    «ступень A» — проверить всю цепочку (план, /cmd_vel) при неподвижных лапах.
  • ⛔ стек CMU (pathFollower / local_planner / far_planner) — НЕЛЬЗЯ запускать вместе
    с Nav2: pathFollower публикует и /cmd_vel, и напрямую /api/sport/request, то есть
    станет ВТОРЫМ водителем и подерётся с нами за лапы. Не используй system_real_robot.launch.

КАРТЫ — БРАТЬ ПАРУ ИЗ ОДНОЙ СЕССИИ (иначе локализация ляжет):
  2D для Nav2   = building.yaml      (по умолчанию здесь)
  3D для matcher = loc_5_map.pcd     (уже прописан в go2_localization/config/localization.yaml)
  ⚠️ НЕ мешать с офлайн-парой (building_old.yaml + final_map_lc.pcd).

ЗАПУСК (на собаке): deploy/run_nav2.sh
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_navigation")
    # ►► ЖИВАЯ карта = building.yaml (пара к loc_5_map.pcd), НЕ building_old.
    default_map = os.path.join(pkg, "maps", "building.yaml")
    default_params = os.path.join(pkg, "config", "nav2_params.yaml")

    args = [
        DeclareLaunchArgument("map", default_value=default_map,
                              description="2D-карта для map_server (живая = building.yaml)"),
        DeclareLaunchArgument("params", default_value=default_params,
                              description="конфигурация Nav2"),
    ]

    # Недостающее звено дерева кадров: aft_mapped -> base_link (Этап 2).
    # Углы/высота крепления — параметры внутри; mount_yaw_deg ТРЕБУЕТ калибровки на роботе.
    tf_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "tf_setup.launch.py")),
    )

    # Раздаёт 2D-карту как /map (защёлкнуто).
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        parameters=[{"yaml_filename": LaunchConfiguration("map")}],
        output="screen",
    )

    # Глобальный планировщик (NavFn) + global_costmap.
    planner = Node(
        package="nav2_planner", executable="planner_server", name="planner_server",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    # Контроллер (RPP) + local_costmap. Публикует /cmd_vel — но в лапы это попадёт
    # ТОЛЬКО если отдельно запущен vel_ctrl (deploy/run_vel_ctrl.sh).
    controller = Node(
        package="nav2_controller", executable="controller_server", name="controller_server",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    # Дирижёр lifecycle. Порядок важен: карта -> планировщик -> контроллер.
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server", "planner_server", "controller_server"],
        }],
        output="screen",
    )

    # Мост: цель из RViz (/goal_pose, приходит с ноутбука через domain_bridge)
    # -> планировщик -> контроллер -> /cmd_vel.
    goal_bridge = Node(
        package="go2_navigation", executable="goal_to_controller.py", name="goal_to_controller",
        output="screen",
    )

    return LaunchDescription(args + [tf_setup, map_server, planner, controller,
                                     lifecycle, goal_bridge])
