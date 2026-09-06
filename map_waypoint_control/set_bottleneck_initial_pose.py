#!/usr/bin/env python3

import math
import os
from pathlib import Path
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


WAYPOINT_YAML = os.path.expanduser(os.environ.get(
    "TB3_WAYPOINT_YAML",
    str(Path(__file__).resolve().parents[1] / "config" / "bottleneck_v3_waypoints.yaml"),
))


class SetInitialPose(Node):

    def __init__(self):
        super().__init__("set_bottleneck_initial_pose")

        with open(WAYPOINT_YAML, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)

        if config.get("map_id") != "bottleneck_map_v3":
            raise RuntimeError(
                f"Unexpected map_id: {config.get('map_id')}"
            )

        start = config["points"]["A_START"]

        self.x = float(start["x"])
        self.y = float(start["y"])
        self.yaw = float(start["yaw"])

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

    def make_message(self):

        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        half_yaw = self.yaw / 2.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        # Similar intent to RViz 2D Pose Estimate:
        # moderate uncertainty in x/y/yaw, zero elsewhere.
        covariance = [0.0] * 36
        covariance[0] = 0.25       # x variance
        covariance[7] = 0.25       # y variance
        covariance[35] = 0.0685    # yaw variance

        msg.pose.covariance = covariance

        return msg

    def publish_pose(self):

        # Give ROS discovery a brief moment, then publish repeatedly.
        time.sleep(0.8)

        for i in range(5):

            msg = self.make_message()
            self.pub.publish(msg)

            self.get_logger().info(
                f"Published A_START initial pose "
                f"{i + 1}/5: "
                f"x={self.x:.6f}, "
                f"y={self.y:.6f}, "
                f"yaw={self.yaw:.4f}"
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

            time.sleep(0.20)


def main(args=None):

    rclpy.init(args=args)
    node = SetInitialPose()

    try:
        node.publish_pose()
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
