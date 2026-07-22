"""
ЗАПУСК:
  ros2 launch go2_navigation tf_setup.launch.py
  ros2 launch go2_navigation tf_setup.launch.py mount_yaw_deg:=-120.0 sensor_height:=0.42

ПРОВЕРКА:
  ros2 run tf2_tools view_frames          # дерево целиком, у каждого звена ОДИН издатель
  ros2 run tf2_ros tf2_echo camera_init base_link
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_static_tf(context, *args, **kwargs):
    """Считает положение base_link в системе aft_mapped и поднимает статический TF.

    Мы задаём крепление ПОНЯТНЫМИ величинами (угол установки, высота, вынос вперёд),
    а неочевидные x/y/z считаем здесь, чтобы в конфиге не было «шести магических чисел».
    """
    # Как лидар повёрнут относительно «вперёд» робота, градусы.
    # То же число, что heading_offset_deg в go2_localization.
    mount_yaw_deg = float(LaunchConfiguration("mount_yaw_deg").perform(context))
    # На какой высоте лидар над полом, метры (base_link лежит НА полу -> уйдём вниз).
    sensor_height = float(LaunchConfiguration("sensor_height").perform(context))
    # Насколько лидар вынесен ВПЕРЁД от центра корпуса, метры (sensorOffsetX у CMU = 0.3).
    sensor_forward = float(LaunchConfiguration("sensor_forward").perform(context))

    yaw = math.radians(mount_yaw_deg)

    # Где лежит base_link, если смотреть ИЗ системы сенсора:
    #  - по высоте: на sensor_height НИЖЕ сенсора  -> z отрицательный;
    #  - в плоскости пола: на sensor_forward НАЗАД вдоль «вперёд» робота.
    #    «Вперёд» робота в системе сенсора направлено под углом yaw, значит «назад»
    #    это тот же вектор со знаком минус.
    x = -sensor_forward * math.cos(yaw)
    y = -sensor_forward * math.sin(yaw)
    z = -sensor_height

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aft_mapped_to_base_link",
        output="screen",
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--yaw", str(yaw),
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "aft_mapped",        # родитель (кадр сенсора от Point-LIO)
            "--child-frame-id", "base_link",   # потомок (корпус робота на полу)
        ],
    )
    return [static_tf]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mount_yaw_deg", default_value="-117.0",
            description="угол установки лидара относительно 'вперёд' робота, градусы "
                        "(то же, что heading_offset_deg; ТРЕБУЕТ калибровки на роботе)"),
        DeclareLaunchArgument(
            "sensor_height", default_value="0.40",
            description="высота лидара над полом, м"),
        DeclareLaunchArgument(
            "sensor_forward", default_value="0.30",
            description="вынос лидара вперёд от центра корпуса, м (sensorOffsetX у CMU)"),
        OpaqueFunction(function=make_static_tf),
    ])
