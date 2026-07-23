"""
nav2_stage4.launch.py — Этап 4: КОНТРОЛЛЕР на записанном заезде, БЕЗ реальной езды.

ЧТО ЭТО ДЕЛАЕТ:
  То же, что Этап 3, плюс ОДИН новый слой — controller_server (RPP). Кликаешь цель:
  планировщик строит путь, а контроллер превращает его в команды скорости /cmd_vel.
  Робот НЕ едет (его команды в лапы переложит Этап 5) — мы смотрим на сами команды.

ОТЛИЧИЯ ОТ ЭТАПА 3 (nav2_offline.launch.py):
  + controller_server          — "руль": путь -> /cmd_vel (в нём же local_costmap)
  ~ lifecycle_manager          — теперь включает ТРИ узла (добавлен controller_server)
  ~ мостик goal_to_controller  — вместо goal_to_planner: он ещё и отдаёт путь контроллеру
  (bt_navigator по-прежнему НЕ поднимаем — изолируем слой; он вернётся на живом роботе)

ЗАПУСК:
  ros2 launch go2_navigation nav2_stage4.launch.py
  # в другом терминале — смотреть команды:
  ros2 topic echo /cmd_vel

ПОЛЬЗОВАНИЕ:
  Дождись 'Managed nodes are active', жми 'Nav2 Goal', кликай точку в коридоре.
  В /cmd_vel побегут команды: vx вперёд, vyaw доворот. На старте виден разворот на
  месте (vx~0, vyaw!=0 — use_rotate_to_heading). Через ~10 с FollowPath завершится
  ошибкой "застрял" — ЭТО ОЖИДАЕМО офлайн (робот в bag не слушает /cmd_vel).
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

    # --- Локализация: TF map -> camera_init (роль map->odom в Nav2). ---
    matcher = Node(
        package="go2_localization", executable="map_matcher_node", name="map_matcher_node",
        parameters=[LaunchConfiguration("localization_config")],
        output="screen",
    )

    # --- Недостающее звено дерева кадров: aft_mapped -> base_link (Этап 2). ---
    tf_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "tf_setup.launch.py")),
    )

    # --- Сервер карты: раздаёт .pgm как /map (защёлкнуто). ---
    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        parameters=[{"yaml_filename": LaunchConfiguration("map")}],
        output="screen",
    )

    # --- Планировщик (NavFn): внутри global_costmap. ---
    planner = Node(
        package="nav2_planner", executable="planner_server", name="planner_server",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    # --- НОВОЕ: контроллер (RPP). Внутри поднимает local_costmap (живой лидар).
    #     Берёт путь через action FollowPath -> публикует /cmd_vel. ---
    controller = Node(
        package="nav2_controller", executable="controller_server", name="controller_server",
        parameters=[LaunchConfiguration("params")],
        output="screen",
    )

    # --- Дирижёр: теперь включает ТРИ узла. Порядок важен: карта -> планировщик ->
    #     контроллер (контроллеру нужна карта под local/global costmap). ---
    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server", "planner_server", "controller_server"],
        }],
        output="screen",
    )

    # --- Мост Этапа 4: клик -> путь -> контроллер -> /cmd_vel. ---
    goal_bridge = Node(
        package="go2_navigation", executable="goal_to_controller.py", name="goal_to_controller",
        output="screen",
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # --- Проигрывание заезда с задержкой (matcher грузит 3D-карту несколько секунд). ---
    bag = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "bag", "play", LaunchConfiguration("bag"),
                 "--rate", LaunchConfiguration("rate")],
            condition=IfCondition(LaunchConfiguration("play_bag")),
            output="screen",
        ),
    ])

    return LaunchDescription(args + [matcher, tf_setup, map_server, planner, controller,
                                     lifecycle, goal_bridge, rviz, bag])
