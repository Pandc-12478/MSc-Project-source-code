#!/usr/bin/env python3

import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException


# Frozen PEDESTRIAN_TRIGGER_ZONE from RViz /clicked_point, 2026-08-10.
# Points are in map frame and follow the clicked perimeter order.
TRIGGER_ZONE: List[Tuple[float, float]] = [
    (2.246581792831421,  0.6566500067710876),
    (2.236055374145508, -0.8413320183753967),
    (1.6932276487350464, -1.0636928081512451),
    (1.6694637537002563,  0.8086005449295044),
]


class PedestrianZoneTrigger(Node):
    """
    Converts the stable pedestrian candidate point from its LaserScan frame
    into map coordinates, then checks whether it lies inside the frozen
    PEDESTRIAN_TRIGGER_ZONE.

    Subscribes:
        /pedestrian_candidate_point : geometry_msgs/PointStamped

    Publishes:
        /pedestrian_zone_trigger : std_msgs/Bool

    This node does not publish /cmd_vel and does not modify detector logic.
    """

    def __init__(self):
        super().__init__("pedestrian_zone_trigger")

        self.declare_parameter("hold_time", 0.40)
        self.hold_time = float(self.get_parameter("hold_time").value)

        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=10.0)
        )
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.last_inside_time = None
        self.last_state = False
        self.last_log_time = 0.0

        self.point_sub = self.create_subscription(
            PointStamped,
            "/pedestrian_candidate_point",
            self.point_callback,
            10,
        )

        self.trigger_pub = self.create_publisher(
            Bool,
            "/pedestrian_zone_trigger",
            10,
        )

        # Publish False continuously when no fresh in-zone point exists.
        self.timer = self.create_timer(
            0.05,
            self.publish_state,
        )

        self.get_logger().info(
            "Pedestrian zone trigger started. "
            "Input=/pedestrian_candidate_point, "
            "output=/pedestrian_zone_trigger"
        )

        for i, (x, y) in enumerate(TRIGGER_ZONE, start=1):
            self.get_logger().info(
                f"Zone P{i}: x={x:.6f}, y={y:.6f}"
            )

    @staticmethod
    def point_in_polygon(
        x: float,
        y: float,
        polygon: List[Tuple[float, float]],
    ) -> bool:
        # Standard ray-casting test.
        inside = False
        j = len(polygon) - 1

        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            intersects = (
                (yi > y) != (yj > y)
                and x
                < (
                    (xj - xi)
                    * (y - yi)
                    / ((yj - yi) if (yj != yi) else 1e-12)
                    + xi
                )
            )

            if intersects:
                inside = not inside

            j = i

        return inside

    @staticmethod
    def transform_point(
        px: float,
        py: float,
        pz: float,
        transform,
    ) -> Tuple[float, float, float]:
        # Quaternion rotation + translation without adding a dependency
        # on tf2_geometry_msgs.
        q = transform.transform.rotation
        t = transform.transform.translation

        # Quaternion-vector rotation:
        # v' = v + 2*q_vec x (q_vec x v + qw*v)
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        tx = 2.0 * (qy * pz - qz * py)
        ty = 2.0 * (qz * px - qx * pz)
        tz = 2.0 * (qx * py - qy * px)

        rx = px + qw * tx + (qy * tz - qz * ty)
        ry = py + qw * ty + (qz * tx - qx * tz)
        rz = pz + qw * tz + (qx * ty - qy * tx)

        return (
            rx + t.x,
            ry + t.y,
            rz + t.z,
        )

    def point_callback(self, msg: PointStamped) -> None:
        source_frame = msg.header.frame_id

        if not source_frame:
            return

        try:
            # Use latest available map transform. This is appropriate for
            # the low-speed experiment and avoids rejecting a candidate
            # solely because the exact scan timestamp is not in the buffer.
            transform = self.tf_buffer.lookup_transform(
                "map",
                source_frame,
                Time(),
                timeout=Duration(seconds=0.20),
            )

        except TransformException as exc:
            now = time.monotonic()

            if now - self.last_log_time > 1.0:
                self.get_logger().warn(
                    f"TF map <- {source_frame} unavailable: {exc}"
                )
                self.last_log_time = now

            return

        map_x, map_y, _ = self.transform_point(
            msg.point.x,
            msg.point.y,
            msg.point.z,
            transform,
        )

        inside = self.point_in_polygon(
            map_x,
            map_y,
            TRIGGER_ZONE,
        )

        if inside:
            self.last_inside_time = time.monotonic()

        now = time.monotonic()
        if now - self.last_log_time > 0.5:
            self.get_logger().info(
                "Pedestrian candidate map position: "
                f"x={map_x:.3f}, y={map_y:.3f}, "
                f"inside_zone={inside}"
            )
            self.last_log_time = now

    def publish_state(self) -> None:
        now = time.monotonic()

        active = (
            self.last_inside_time is not None
            and (now - self.last_inside_time) <= self.hold_time
        )

        msg = Bool()
        msg.data = active
        self.trigger_pub.publish(msg)

        if active != self.last_state:
            self.get_logger().info(
                f"/pedestrian_zone_trigger -> {active}"
            )
            self.last_state = active


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianZoneTrigger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
