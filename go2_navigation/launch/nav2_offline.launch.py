"""
nav2_offline.launch.py — Этап 3: планировщик Nav2 на записанном заезде, БЕЗ езды.

ЧТО ЭТО ДЕЛАЕТ:
  Поднимает всё, что нужно, чтобы кликнуть цель в RViz и увидеть маршрут по зданию.
  Робот никуда не едет — контроллера здесь нет специально (он будет на Этапе 4).
  Смысл: проверить отдельно ОДИН слой — планирование по нашей 2D-карте.

ЧТО ПОДНИМАЕТСЯ (снизу вверх по слоям):
  1. bag play          — проигрывает записанный заезд (сканы + одометрия Point-LIO)
  2. map_matcher_node  — локализация: даёт TF map -> camera_init
  3. tf_setup          — статическое звено aft_mapped -> base_link (Этап 2)
  4. map_server        — раздаёт нашу 2D-карту в топик /map
  5. planner_server    — строит маршруты по карте (плагин NavFn)
  6. lifecycle_manager — включает узлы Nav2 (они не стартуют активными сами!)
  7. goal_to_planner   — наш мостик: клик в RViz -> запрос планировщику -> /plan
  8. rviz2             — картинка

ПРО LIFECYCLE (частая засада новичка):
  Узлы Nav2 после запуска НЕ работают сразу — они проходят стадии
  unconfigured -> inactive -> active, и переводит их lifecycle_manager.
  Если «всё запустилось, но ничего не происходит» — первым делом смотри,
  дошли ли узлы до active (в логе lifecycle_manager).

ЗАПУСК:
  ros2 launch go2_navigation nav2_offline.launch.py
  ros2 launch go2_navigation nav2_offline.launch.py play_bag:=false   # без проигрывания

ПОЛЬЗОВАНИЕ:
  Дождись в логе 'Managed nodes are active', затем в RViz жми «Nav2 Goal»
  и кликай точку в белом коридоре — появится зелёный маршрут.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_navigation")
    default_map = os.path.join(pkg, "maps", "building_old.yaml")
    default_params = os.path.join(pkg, "config", "nav2_params.yaml")
    default_loc_cfg = os.path.join(pkg, "config", "localization_offline.yaml")
    rviz_cfg = os.path.join(pkg, "rviz", "nav2.rviz")

    args = [
        DeclareLaunchArgument("bag", default_value="/home/dmitriyb51/maps/loc_test_2_1_plio",
                              description="записанный заезд для проигрывания"),
        DeclareLaunchArgument("map", default_value=default_map,
                              description="2D-карта (.yaml) для map_server"),
        DeclareLaunchArgument("params", default_value=default_params,
                              description="конфигурация Nav2"),
        DeclareLaunchArgument("localization_config", default_value=default_loc_cfg,
                              description="конфиг matcher (какая 3D-карта)"),
        DeclareLaunchArgument("play_bag", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rate", default_value="1.0", description="скорость проигрывания"),
    ]

    # --- Слой 2: локализация. Даёт TF map -> camera_init (роль map->odom в Nav2). ---
    matcher = Node(
        package="go2_localization", executable="map_matcher_node", name="map_matcher_node",
        parameters=[LaunchConfiguration("localization_config")],
        output="screen",
    )

    # --- Слой 3: недостающее звено дерева кадров, aft_mapped -> base_link (Этап 2). ---
    tf_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "tf_setup.launch.py")),
    )

    # --- Слой 4: сервер карты. Раздаёт .pgm как топик /map (защёлкнуто). ---
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        parameters=[{"yaml_filename": LaunchConfiguration("map")}],
        output="screen",
    )

    # --- Слой 5: планировщик. Внутри поднимает global_costmap (стены + inflation). ---
    planner = Node(
        package="nav2_planner", executable="planner_server", name="planner_server",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    # --- Слой 6: дирижёр. Переводит узлы Nav2 в active, иначе они молчат. ---
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "autostart": True,
            # порядок важен: карта должна подняться раньше планировщика,
            # иначе costmap стартует без статического слоя.
            "node_names": ["map_server", "planner_server"],
        }],
        output="screen",
    )

    # --- Слой 7: клик в RViz -> запрос планировщику -> /plan. ---
    goal_bridge = Node(
        package="go2_navigation", executable="goal_to_planner.py", name="goal_to_planner",
        output="screen",
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # --- Слой 1: проигрывание заезда. С задержкой — matcher грузит 3D-карту
    #     (3 млн точек) несколько секунд, и до этого сканы ему слать бессмысленно.
    bag = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "bag", "play", LaunchConfiguration("bag"),
                 "--rate", LaunchConfiguration("rate")],
            condition=IfCondition(LaunchConfiguration("play_bag")),
            output="screen",
        ),
    ])

    return LaunchDescription(args + [matcher, tf_setup, map_server, planner,
                                     lifecycle, goal_bridge, rviz, bag])
