#!/usr/bin/env python3
"""
goal_to_planner.py — «кликнул цель в RViz → увидел маршрут».

ЗАЧЕМ ЭТОТ УЗЕЛ (Этап 3):
  В полном Nav2 цель обрабатывает bt_navigator: он крутит дерево поведения, которое
  сначала зовёт планировщик, а потом контроллер (ехать). Но на Этапе 3 мы ещё НЕ хотим
  никуда ехать — надо просто проверить, что планировщик умеет строить маршрут по нашей
  2D-карте. Тащить ради этого контроллер и дерево поведения = сразу много движущихся
  частей, и при отказе непонятно, кто виноват.

  Поэтому этот маленький узел заменяет всю ту машинерию одной прямой связкой:
      /goal_pose (клик в RViz) ──► планировщик ──► /plan (зелёная линия в RViz)

КАК ОН РАБОТАЕТ:
  1. Слушает /goal_pose — туда RViz шлёт клик инструментом «Nav2 Goal».
  2. Спрашивает у TF, где сейчас робот (трансформация map → base_link).
     Это ровно та цепочка, которую мы собрали на Этапе 2.
  3. Зовёт у planner_server действие ComputePathToPose (старт = робот, финиш = клик).
  4. Публикует полученный путь в /plan, где его рисует RViz.

ЗАПУСК: поднимается автоматически из nav2_offline.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
import tf2_ros


class GoalToPlanner(Node):
    def __init__(self):
        super().__init__("goal_to_planner")

        # Буфер TF: сюда стекаются все звенья цепочки (map->camera_init->aft_mapped->base_link),
        # и из них он сам соберёт нужную нам трансформацию map->base_link.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Клиент действия к планировщику Nav2.
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        self.path_pub = self.create_publisher(Path, "/plan", 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)

        self.get_logger().info("goal_to_planner готов: кликай 'Nav2 Goal' в RViz")

    def on_goal(self, goal_msg: PoseStamped):
        """Пришла цель из RViz — узнаём позу робота и просим построить маршрут."""
        # Где робот прямо сейчас? Спрашиваем у TF цепочку map -> base_link.
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
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
            f"цель ({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f}), "
            f"робот ({start.pose.position.x:.2f}, {start.pose.position.y:.2f}) — считаю маршрут")

        if not self.planner.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("planner_server не отвечает — он точно 'active'?")
            return

        # Просим планировщик: от start до goal. use_start=True — иначе он возьмёт
        # позу робота сам, но нам полезно задать её явно и увидеть в логе.
        request = ComputePathToPose.Goal()
        request.start = start
        request.goal = goal_msg
        request.use_start = True

        future = self.planner.send_goal_async(request)
        future.add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("планировщик отклонил запрос")
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        """Маршрут посчитан (или нет) — публикуем и говорим, что вышло."""
        path = future.result().result.path
        n = len(path.poses)
        if n == 0:
            self.get_logger().warn(
                "маршрут НЕ найден — цель за стеной, в неизвестном, "
                "или проход слишком узкий после раздувания стен")
            return
        # Длина маршрута — полезная проверка вменяемости.
        length = 0.0
        for a, b in zip(path.poses[:-1], path.poses[1:]):
            dx = b.pose.position.x - a.pose.position.x
            dy = b.pose.position.y - a.pose.position.y
            length += (dx * dx + dy * dy) ** 0.5
        self.get_logger().info(f"маршрут построен: {n} точек, длина {length:.1f} м")
        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = GoalToPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
