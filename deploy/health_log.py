#!/usr/bin/env python3
"""
Live health logger for the Go2 localization stack.

WHY THIS EXISTS
    Point-LIO diverged to kilometres during the 2026-07-20 live run and we could
    not say why, because nothing was recording the evidence. Retroactive guessing
    is worthless. This node writes one CSV row per second so that when a
    divergence happens we can point at the exact moment and see what else moved.

WHAT IT RECORDS (one row per second)
    t_wall          wall clock (epoch seconds)
    dist_m          distance of Point-LIO's estimate from its own origin.
                    THE key number: on a walk in a building this should stay
                    within tens of metres. Hundreds/thousands = divergence.
    dz_m            Z of the estimate. Point-LIO's known weak axis.
    odom_hz         /state_estimation rate over the last second. If this sags,
                    Point-LIO is being starved of CPU.
    scan_hz         /registered_scan rate (expect ~15, matching the lidar).
    scan_pts        points in the last scan. The L1 is sparse; a sudden drop
                    means the lidar is seeing nothing to match against.
    fitness         last /localization/fitness. ~0.006-0.02 = healthy lock,
                    >0.3 = matcher rejecting, huge = GICP found no correspondence.
    cpu_pct         total CPU across all cores.
    load1           1-minute load average.

It is deliberately a plain subscriber that does no heavy work, and it is meant to
be pinned to the non-realtime cores so it cannot itself become the problem.
"""

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32


class HealthLogger(Node):
    def __init__(self, out_path):
        super().__init__("health_logger")

        # Counters for the current 1-second bucket.
        self.odom_count = 0
        self.scan_count = 0
        self.last_pos = (0.0, 0.0, 0.0)
        self.last_scan_pts = 0
        self.last_fitness = float("nan")

        # Sensor data is best-effort/high-rate; matching QoS keeps us from
        # accidentally applying backpressure to the publisher.
        fast_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Odometry, "/state_estimation", self.on_odom, fast_qos)
        self.create_subscription(PointCloud2, "/registered_scan", self.on_scan, fast_qos)
        self.create_subscription(Float32, "/localization/fitness", self.on_fitness, 10)

        self.f = open(out_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "t_wall", "dist_m", "dz_m", "odom_hz", "scan_hz",
            "scan_pts", "fitness", "cpu_pct", "load1",
        ])
        self.f.flush()

        # For CPU percentage we diff /proc/stat between ticks.
        self.prev_cpu = self.read_cpu()

        self.create_timer(1.0, self.tick)
        self.get_logger().info(f"health logging to {out_path}")

    def on_odom(self, msg):
        self.odom_count += 1
        p = msg.pose.pose.position
        self.last_pos = (p.x, p.y, p.z)

    def on_scan(self, msg):
        self.scan_count += 1
        self.last_scan_pts = msg.width * msg.height

    def on_fitness(self, msg):
        self.last_fitness = msg.data

    def read_cpu(self):
        """Return (idle, total) jiffies from /proc/stat's aggregate cpu line."""
        with open("/proc/stat") as fh:
            parts = fh.readline().split()[1:]
        vals = [int(v) for v in parts]
        idle = vals[3] + vals[4]          # idle + iowait
        return idle, sum(vals)

    def tick(self):
        idle, total = self.read_cpu()
        pidle, ptotal = self.prev_cpu
        d_total = total - ptotal
        cpu_pct = 100.0 * (1.0 - (idle - pidle) / d_total) if d_total > 0 else float("nan")
        self.prev_cpu = (idle, total)

        x, y, z = self.last_pos
        dist = math.sqrt(x * x + y * y + z * z)
        load1 = os.getloadavg()[0]

        self.w.writerow([
            f"{time.time():.1f}", f"{dist:.2f}", f"{z:.2f}",
            self.odom_count, self.scan_count, self.last_scan_pts,
            f"{self.last_fitness:.4f}", f"{cpu_pct:.1f}", f"{load1:.2f}",
        ])
        self.f.flush()

        # Shout loudly the moment the estimate leaves the building, so the
        # divergence is visible in the log tail without opening the CSV.
        if dist > 200.0:
            self.get_logger().error(f"DIVERGED: Point-LIO {dist:.0f} m from origin")

        self.odom_count = 0
        self.scan_count = 0


def main():
    out = os.path.expanduser(f"~/health_{time.strftime('%H%M%S')}.csv")
    rclpy.init()
    node = HealthLogger(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
