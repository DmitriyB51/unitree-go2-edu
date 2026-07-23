#!/usr/bin/env python3
"""Калибровка mount_yaw_deg.

ИДЕЯ. Мы гоним собаку СТРОГО ВПЕРЁД (vx>0, без поворота). Значит направление, куда
она реально уехала на карте, и есть её "вперёд". Сравниваем его с тем, куда, по мнению
TF, смотрит base_link. Разница = ошибка mount_yaw_deg.

ЗАПУСК:
  окно A:  ~/run_vel_ctrl.sh
  окно B:  source ~/ros_env.sh && python3 ~/calib_yaw.py
  окно C:  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'
Дай проехать 2-3 м по прямой, потом Ctrl+C в окне C.
"""
import math
import time

import rclpy
from tf2_ros import Buffer, TransformListener

rclpy.init()
n = rclpy.create_node("calib_yaw")
buf = Buffer()
TransformListener(buf, n)


def pose():
    """Позиция и курс base_link в кадре map."""
    t = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
    q = t.rotation
    # рыскание (yaw) из кватерниона
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return t.translation.x, t.translation.y, yaw


t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
    try:
        x0, y0, yaw0 = pose()
        break
    except Exception:
        pass
else:
    print("нет TF map->base_link — запущены ли Point-LIO и матчер?")
    raise SystemExit

print("старт (%.2f, %.2f), курс %.1f deg" % (x0, y0, math.degrees(yaw0)))
print("ЕДЬ ПРЯМО. Замер идёт 30 с, нужно 2-3 м. Ctrl+C когда проехал.")

samples = []
try:
    t0 = time.time()
    while time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            x, y, yaw = pose()
        except Exception:
            continue
        d = math.hypot(x - x0, y - y0)
        samples.append((x, y, yaw))
        print("  проехал %.2f м" % d, end="\r")
except KeyboardInterrupt:
    pass

if len(samples) < 10:
    print("\nслишком мало данных")
    raise SystemExit

x1, y1, yaw1 = samples[-1]
dist = math.hypot(x1 - x0, y1 - y0)
print("")
if dist < 0.5:
    print("проехал всего %.2f м — мало, нужно хотя бы 0.5 м. Повтори." % dist)
    raise SystemExit

# Куда РЕАЛЬНО уехали, и куда "смотрел" base_link в среднем.
travel = math.atan2(y1 - y0, x1 - x0)
yaws = [s[2] for s in samples]
# усредняем углы через вектор, иначе ±180° ломает среднее
my = math.atan2(sum(math.sin(a) for a in yaws), sum(math.cos(a) for a in yaws))
err = math.degrees(math.atan2(math.sin(travel - my), math.cos(travel - my)))

print("проехал                 : %.2f м" % dist)
print("направление движения    : %+.1f deg" % math.degrees(travel))
print("курс base_link (средний): %+.1f deg" % math.degrees(my))
print("")
print("ОШИБКА                  : %+.1f deg" % err)
print("")
print("Текущее mount_yaw_deg = -117.0")
print("НОВОЕ значение        = %.1f" % (-117.0 + err))
print("")
print("Если |ошибка| < 5 deg — калибровка не нужна, оставь как есть.")
print("Иначе: mount_yaw_deg в go2_navigation/launch/tf_setup.launch.py")
print("       и heading_offset_deg в config матчера — на новое значение.")
rclpy.shutdown()
