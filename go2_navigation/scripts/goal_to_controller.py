#!/usr/bin/env python3
"""
goal_to_controller.py — «кликнул цель → контроллер поехал (в командах /cmd_vel)».

ЗАЧЕМ ЭТОТ УЗЕЛ (Этап 4):
  Этап 3 проверял ПЛАНИРОВЩИК: клик → путь (зелёная линия). Робот стоял.
  Этап 4 добавляет ровно ОДИН слой — КОНТРОЛЛЕР (RPP): он берёт путь и превращает
  его в команды скорости тела /cmd_vel (vx, vy, vyaw). Это ещё НЕ езда: /cmd_vel в
  реальные лапы переложит мост vel_ctrl_repub (Этап 5). Здесь мы только смотрим,
  что контроллер выдаёт ОСМЫСЛЕННЫЕ команды.

  Как и на Этапе 3, мы НЕ поднимаем bt_navigator (дерево поведения) — чтобы
  изолировать один слой. bt_navigator вернётся на живом роботе (Этап 6).

ЦЕПОЧКА (на шаг длиннее, чем в goal_to_planner):
  /goal_pose (клик) ─► поза робота из TF ─► планировщик (ComputePathToPose) ─► путь
                    ─► путь в /plan (нарисовать) ─► контроллеру (FollowPath) ─► /cmd_vel

⚠️ ВАЖНО ПРО ОФЛАЙН: робот в заезде (bag) НЕ слушает наш /cmd_vel — он едет по своей
   записанной траектории. Поэтому контроллер видит, что робот «не слушается», и через
   ~10 с progress_checker объявит «застрял» (FollowPath завершится с ошибкой). Это
   ОЖИДАЕМО. Мы смотрим на сами команды /cmd_vel, а не на то, доехал ли робот.
   Проверить команды:  ros2 topic echo /cmd_vel

ЗАПУСК: поднимается автоматически из nav2_stage4.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, FollowPath
import tf2_ros


class GoalToController(Node):
    def __init__(self):
        super().__init__("goal_to_controller")

        # TF: из цепочки map->camera_init->aft_mapped->base_link он соберёт map->base_link
        # (где робот сейчас) — стартовая точка для планировщика.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Два клиента действий: сначала спланировать путь, потом отдать его контроллеру.
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.controller = ActionClient(self, FollowPath, "follow_path")

        # /plan — чтобы видеть путь в RViz (как на Этапе 3).
        self.path_pub = self.create_publisher(Path, "/plan", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)

        self.get_logger().info(
            "goal_to_controller готов: кликай 'Nav2 Goal' в RViz, "
            "команды смотри в  ros2 topic echo /cmd_vel")

    # ---- ШАГ 1: пришла цель — узнаём позу робота и просим ПУТЬ ------------------
    def on_goal(self, goal_msg: PoseStamped):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(
                f"не могу найти робота в TF (map->base_link): {e}. "
                "Запущены ли matcher и tf_setup?")
            return

        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = tf.transform.translation.x
        start.pose.position.y = tf.transform.translation.y
        start.pose.orientation = tf.transform.rotation

        self.get_logger().info(
            f"цель ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}) — "
            "строю путь")

        if not self.planner.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("planner_server не отвечает — он 'active'?")
            return

        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = goal_msg
        request.use_start = True
        self.planner.send_goal_async(request).add_done_callback(self.on_plan_accepted)

    def on_plan_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("планировщик отклонил запрос")
            return
        handle.get_result_async().add_done_callback(self.on_plan_result)

    # ---- ШАГ 2: путь готов — рисуем его и ОТДАЁМ КОНТРОЛЛЕРУ --------------------
    def on_plan_result(self, future):
        path = future.result().result.path
        if len(path.poses) == 0:
            self.get_logger().warn("маршрут НЕ найден — цель за стеной/в неизвестном")
            return

        self.path_pub.publish(path)                 # показать в RViz
        self.get_logger().info(
            f"путь построен ({len(path.poses)} точек) — отдаю контроллеру, "
            "поехали командами /cmd_vel")

        if not self.controller.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("controller_server не отвечает — он 'active'?")
            return

        follow = FollowPath.Goal()
        follow.path = path
        follow.controller_id = "FollowPath"              # имя нашего плагина RPP
        follow.goal_checker_id = "general_goal_checker"  # чем проверять "доехал"
        self.controller.send_goal_async(
            follow, feedback_callback=self.on_feedback
        ).add_done_callback(self.on_follow_accepted)

    def on_follow_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("контроллер отклонил путь")
            return
        self.get_logger().info("контроллер принял путь — смотри /cmd_vel")
        handle.get_result_async().add_done_callback(self.on_follow_result)

    # ---- Пока контроллер ведёт: печатаем скорость и остаток до цели -----------
    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"скорость {fb.speed:.2f} м/с, до цели {fb.distance_to_goal:.2f} м",
            throttle_duration_sec=1.0)          # не чаще раза в секунду

    def on_follow_result(self, future):
        # Офлайн это почти всегда "неудача" из-за progress_checker — и это НОРМАЛЬНО
        # (робот в bag не слушает /cmd_vel). Главное — команды мы уже увидели.
        self.get_logger().info(
            "FollowPath завершился. Офлайн ошибка 'застрял' ожидаема — "
            "робот в заезде не реагирует на /cmd_vel. Смотри на сами команды.")


def main():
    rclpy.init()
    node = GoalToController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
