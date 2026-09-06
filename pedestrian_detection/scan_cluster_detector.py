#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32


@dataclass
class ScanPoint:
    angle: float
    distance: float
    x: float
    y: float


@dataclass
class Cluster:
    points: List[ScanPoint]

    @property
    def point_count(self) -> int:
        return len(self.points)

    @property
    def min_distance(self) -> float:
        return min(point.distance for point in self.points)

    @property
    def mean_distance(self) -> float:
        return sum(point.distance for point in self.points) / len(self.points)

    @property
    def centre_angle(self) -> float:
        return sum(point.angle for point in self.points) / len(self.points)

    @property
    def width(self) -> float:
        first = self.points[0]
        last = self.points[-1]

        return math.hypot(
            last.x - first.x,
            last.y - first.y
        )


class PedestrianDetectionNode(Node):
    """
    Detects pedestrian-like LaserScan clusters in the bottleneck entrance.

    Published topics:
        /pedestrian_detected : std_msgs/Bool
        /pedestrian_distance : std_msgs/Float32
        /pedestrian_candidate_point : geometry_msgs/PointStamped

    The node does not publish velocity commands.
    """

    def __init__(self):
        super().__init__('pedestrian_detection')

        # General scan extraction region.
        self.scan_angle_limit_deg = 55.0
        self.scan_min_distance = 0.15
        self.scan_max_distance = 2.50

        # Clustering.
        self.cluster_gap = 0.10
        self.min_cluster_points = 3

        # Experimental pedestrian candidate region.
        self.candidate_angle_limit_deg = 20.0
        self.candidate_min_distance = 0.45
        self.candidate_max_distance = 2.20
        self.candidate_min_width = 0.03
        self.candidate_max_width = 0.70

        # Temporal hysteresis.
        self.required_detection_frames = 2
        self.required_clear_frames = 5

        self.detection_count = 0
        self.clear_count = 0
        self.pedestrian_detected = False

        self.current_candidate: Optional[Cluster] = None
        self.last_candidate: Optional[Cluster] = None

        self.last_print_time = self.get_clock().now()
        self.last_scan_frame = ''
        self.last_scan_stamp = None

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.detected_publisher = self.create_publisher(
            Bool,
            '/pedestrian_detected',
            10
        )

        self.distance_publisher = self.create_publisher(
            Float32,
            '/pedestrian_distance',
            10
        )

        # Minimal interface extension for spatial zone triggering.
        # The existing detector logic is unchanged; this publishes
        # only the selected stable cluster position in the LaserScan frame.
        self.candidate_point_publisher = self.create_publisher(
            PointStamped,
            '/pedestrian_candidate_point',
            10
        )

        self.get_logger().info(
            'Pedestrian detection started. '
            'Publishing /pedestrian_detected, /pedestrian_distance, '
            'and /pedestrian_candidate_point.'
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.last_scan_frame = msg.header.frame_id
        self.last_scan_stamp = msg.header.stamp

        points = self.extract_scan_points(msg)
        clusters = self.build_clusters(points)
        candidates = self.filter_candidates(clusters)

        self.update_detection_state(candidates)
        self.publish_detection()
        self.print_status()

    def extract_scan_points(
        self,
        msg: LaserScan
    ) -> List[ScanPoint]:

        points = []

        angle_limit = math.radians(
            self.scan_angle_limit_deg
        )

        valid_min_range = max(
            self.scan_min_distance,
            msg.range_min
        )

        valid_max_range = min(
            self.scan_max_distance,
            msg.range_max
        )

        for index, distance in enumerate(msg.ranges):
            angle = (
                msg.angle_min
                + index * msg.angle_increment
            )

            if abs(angle) > angle_limit:
                continue

            if not math.isfinite(distance):
                continue

            if distance < valid_min_range:
                continue

            if distance > valid_max_range:
                continue

            points.append(
                ScanPoint(
                    angle=angle,
                    distance=distance,
                    x=distance * math.cos(angle),
                    y=distance * math.sin(angle)
                )
            )

        return points

    def build_clusters(
        self,
        points: List[ScanPoint]
    ) -> List[Cluster]:

        if not points:
            return []

        clusters = []
        current_points = [points[0]]

        for previous, current in zip(
            points,
            points[1:]
        ):
            gap = math.hypot(
                current.x - previous.x,
                current.y - previous.y
            )

            if gap <= self.cluster_gap:
                current_points.append(current)
            else:
                clusters.append(
                    Cluster(current_points)
                )
                current_points = [current]

        clusters.append(
            Cluster(current_points)
        )

        return clusters

    def filter_candidates(
        self,
        clusters: List[Cluster]
    ) -> List[Cluster]:

        candidates = []

        angle_limit = math.radians(
            self.candidate_angle_limit_deg
        )

        for cluster in clusters:
            if cluster.point_count < self.min_cluster_points:
                continue

            if abs(cluster.centre_angle) > angle_limit:
                continue

            if cluster.min_distance < self.candidate_min_distance:
                continue

            if cluster.min_distance > self.candidate_max_distance:
                continue

            if cluster.width < self.candidate_min_width:
                continue

            if cluster.width > self.candidate_max_width:
                continue

            candidates.append(cluster)

        return candidates

    def choose_candidate(
        self,
        candidates: List[Cluster]
    ) -> Optional[Cluster]:

        if not candidates:
            return None

        return min(
            candidates,
            key=lambda cluster: cluster.min_distance
        )

    def update_detection_state(
        self,
        candidates: List[Cluster]
    ) -> None:

        selected = self.choose_candidate(candidates)

        if selected is not None:
            self.current_candidate = selected
            self.last_candidate = selected

            self.detection_count = min(
                self.detection_count + 1,
                self.required_detection_frames
            )

            self.clear_count = 0

            if (
                self.detection_count
                >= self.required_detection_frames
            ):
                self.pedestrian_detected = True

            return

        self.current_candidate = None

        if self.pedestrian_detected:
            self.clear_count += 1

            if self.clear_count >= self.required_clear_frames:
                self.pedestrian_detected = False
                self.detection_count = 0
                self.clear_count = 0
                self.last_candidate = None

        else:
            self.detection_count = max(
                self.detection_count - 1,
                0
            )

    def publish_detection(self) -> None:
        detected_msg = Bool()
        detected_msg.data = self.pedestrian_detected
        self.detected_publisher.publish(detected_msg)

        distance_msg = Float32()

        candidate = (
            self.current_candidate
            if self.current_candidate is not None
            else self.last_candidate
        )

        if self.pedestrian_detected and candidate is not None:
            distance_msg.data = float(candidate.min_distance)
        else:
            distance_msg.data = float('nan')

        self.distance_publisher.publish(distance_msg)

        # Publish one stable pedestrian candidate point only when the
        # existing detector has already declared pedestrian_detected=True.
        # Use the cluster centroid in the scan frame; this does not change
        # any clustering, candidate filtering, or hysteresis thresholds.
        zone_candidate = self.current_candidate

        if (
            zone_candidate is not None
            and self.last_scan_frame
            and self.last_scan_stamp is not None
        ):
            point_msg = PointStamped()
            point_msg.header.frame_id = self.last_scan_frame
            point_msg.header.stamp = self.last_scan_stamp

            point_msg.point.x = float(
                sum(p.x for p in zone_candidate.points) / len(zone_candidate.points)
            )
            point_msg.point.y = float(
                sum(p.y for p in zone_candidate.points) / len(zone_candidate.points)
            )
            point_msg.point.z = 0.0

            self.candidate_point_publisher.publish(point_msg)

    def print_status(self) -> None:
        now = self.get_clock().now()

        elapsed = (
            now - self.last_print_time
        ).nanoseconds / 1e9

        if elapsed < 0.5:
            return

        candidate = (
            self.current_candidate
            if self.current_candidate is not None
            else self.last_candidate
        )

        if self.pedestrian_detected and candidate is not None:
            self.get_logger().info(
                'PEDESTRIAN DETECTED | '
                f'angle='
                f'{math.degrees(candidate.centre_angle):+.1f} deg, '
                f'distance={candidate.min_distance:.2f} m, '
                f'width={candidate.width:.2f} m, '
                f'points={candidate.point_count}, '
                f'clear={self.clear_count}/'
                f'{self.required_clear_frames}'
            )
        else:
            self.get_logger().info(
                'No stable pedestrian candidate | '
                f'detection={self.detection_count}/'
                f'{self.required_detection_frames}'
            )

        self.last_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
