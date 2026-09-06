#!/usr/bin/env python3
"""
Minimal event logger for the TurtleBot3 bottleneck experiment.

Design goal:
- Keep only system data that are directly useful for experimental analysis,
  video alignment, manipulation checking, and trial-quality checks.
- Do NOT infer pedestrian probe, commitment, gaze, passing order, or physical
  clearance. Those remain video-coded research variables.
- No continuous 10 Hz telemetry file: one compact events.csv + metadata.json.

Recorded event streams:
  /pedestrian_detected      -> first detector trigger
  /pedestrian_zone_trigger  -> first spatial-zone trigger
  /trial_state              -> controller state transitions
  /motion_phase             -> motion/stage transitions
  /ehmi_state               -> actual display-state transitions
  /experiment_sync_marker   -> manual video-sync marker

Latest /odom and /pedestrian_distance values are attached to every event as
context snapshots. This retains the useful robot-state evidence without
producing a large continuous telemetry log.
"""

import csv
import json
import math
import signal
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float32, String


CONDITION_MAP = {
    "SYN": ("STRAIGHT_YIELD", "NEUTRAL"),
    "SYE": ("STRAIGHT_YIELD", "EXPLICIT"),
    "SDN": ("SIDE_YIELD", "NEUTRAL"),
    "SDE": ("SIDE_YIELD", "EXPLICIT"),
    "CN": ("CLAIM", "NEUTRAL"),
    "CE": ("CLAIM", "EXPLICIT"),
}

LOGGER_VERSION = "2.0-minimal-events"


class ExperimentDataLogger(Node):
    def __init__(self):
        super().__init__("experiment_data_logger")

        # Trial identity / destination.
        self.declare_parameter("participant_id", "")
        self.declare_parameter("session_id", "")
        self.declare_parameter("trial_number", 0)
        self.declare_parameter("attempt_number", 1)
        self.declare_parameter("condition", "")
        self.declare_parameter("output_root", "")
        self.declare_parameter("map_name", "bottleneck_map_v3")

        # Kept only for backwards compatibility with the current launcher.
        # This event-only logger does not continuously sample telemetry.
        self.declare_parameter("telemetry_rate_hz", 10.0)

        self.participant_id = str(self.get_parameter("participant_id").value).strip().upper()
        self.session_id = str(self.get_parameter("session_id").value).strip()
        self.trial_number = int(self.get_parameter("trial_number").value)
        self.attempt_number = int(self.get_parameter("attempt_number").value)
        self.condition = str(self.get_parameter("condition").value).strip().upper()
        self.output_root = str(self.get_parameter("output_root").value).strip()
        self.map_name = str(self.get_parameter("map_name").value).strip()

        self._validate_parameters()
        self.motion_strategy, self.display_condition = CONDITION_MAP[self.condition]

        # Time base.
        self.start_monotonic = time.monotonic()
        self.start_wall_epoch = time.time()
        self.start_wall_iso = self._iso_now()
        self.start_ros_s = self._ros_time_s()

        # Output files.
        trial_name = (
            f"trial_{self.trial_number:02d}_{self.condition}"
            f"_attempt_{self.attempt_number:02d}"
        )
        self.trial_dir = (
            Path(self.output_root)
            / self.participant_id
            / self.session_id
            / trial_name
        )
        self.trial_dir.mkdir(parents=True, exist_ok=False)

        self.metadata_path = self.trial_dir / "metadata.json"
        self.events_path = self.trial_dir / "events.csv"
        self.events_file = self.events_path.open("w", newline="", encoding="utf-8")
        self.events_writer = csv.writer(self.events_file)

        self.events_writer.writerow([
            "event_id",
            "trial_elapsed_s",
            "wall_time_iso",
            "ros_time_s",
            "event_type",
            "event_value",
            "trial_state",
            "motion_phase",
            "ehmi_state",
            "pedestrian_distance_m",
            "odom_x_m",
            "odom_y_m",
            "odom_yaw_rad",
        ])
        self.events_file.flush()

        self.event_id = 0
        self.finalized = False
        self.completed = False

        # Latest context attached to event rows.
        self.pedestrian_distance_m = math.nan
        self.odom_x = math.nan
        self.odom_y = math.nan
        self.odom_yaw = math.nan

        self.trial_state = ""
        self.motion_phase = ""
        self.ehmi_state = ""

        self.last_trial_state = None
        self.last_motion_phase = None
        self.last_ehmi_state = None
        self.last_pedestrian_detected = False
        self.last_zone_trigger = False

        # Context subscriptions.
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Float32, "/pedestrian_distance", self.distance_callback, 10
        )

        # Event subscriptions.
        self.create_subscription(
            Bool, "/pedestrian_detected", self.pedestrian_callback, 10
        )
        self.create_subscription(
            Bool, "/pedestrian_zone_trigger", self.zone_callback, 10
        )
        self.create_subscription(
            String, "/trial_state", self.trial_state_callback, 10
        )
        self.create_subscription(
            String, "/motion_phase", self.motion_phase_callback, 10
        )
        self.create_subscription(
            String, "/ehmi_state", self.ehmi_state_callback, 10
        )
        self.create_subscription(
            Empty, "/experiment_sync_marker", self.sync_marker_callback, 10
        )

        self._write_metadata()
        self.record_event("TRIAL_START", self.condition)

        self.get_logger().info(f"Minimal event logger ready: {self.trial_dir}")
        self.get_logger().info(
            f"Participant={self.participant_id}, trial={self.trial_number:02d}, "
            f"condition={self.condition}, attempt={self.attempt_number}"
        )
        self.get_logger().info(
            "Output: metadata.json + events.csv (no continuous telemetry.csv)"
        )

    def _validate_parameters(self):
        if not self.participant_id or not self.participant_id.startswith("P"):
            raise ValueError("participant_id must be pseudonymous, e.g. P01")
        if not self.session_id:
            raise ValueError("session_id is required")
        if self.trial_number < 1:
            raise ValueError("trial_number must be >= 1")
        if self.attempt_number < 1:
            raise ValueError("attempt_number must be >= 1")
        if self.condition not in CONDITION_MAP:
            raise ValueError("condition must be one of SYN, SYE, SDN, SDE, CN, CE")
        if not self.output_root:
            raise ValueError("output_root is required")

    def _write_metadata(self):
        metadata = {
            "participant_id": self.participant_id,
            "session_id": self.session_id,
            "trial_id": f"{self.participant_id}_T{self.trial_number:02d}_{self.condition}",
            "trial_number": self.trial_number,
            "condition_order": self.trial_number,
            "attempt_number": self.attempt_number,
            "condition_code": self.condition,
            "motion_strategy": self.motion_strategy,
            "display_condition": self.display_condition,
            "map_name": self.map_name,
            "trial_start_wall_time": self.start_wall_iso,
            "trial_start_wall_epoch_s": self.start_wall_epoch,
            "trial_start_ros_time_s": self.start_ros_s,
            "logger_version": LOGGER_VERSION,
            "logger_design": "event transitions only; odom/distance snapshots attached to events",
            "video_sync_method": "phone video + manual /experiment_sync_marker once per trial",
            "video_coded_variables": [
                "probe",
                "commitment",
                "physical_clearance",
                "gaze",
                "passing_order",
                "hesitation / secondary correction",
            ],
        }
        self.metadata_path.write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    @staticmethod
    def _iso_now():
        return datetime.now(timezone.utc).isoformat(timespec="milliseconds")

    def _ros_time_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed_s(self):
        return time.monotonic() - self.start_monotonic

    @staticmethod
    def _csv_float(value):
        try:
            if math.isnan(value):
                return "NaN"
        except TypeError:
            return ""
        return f"{float(value):.6f}"

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_x = float(pose.position.x)
        self.odom_y = float(pose.position.y)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def distance_callback(self, msg: Float32):
        self.pedestrian_distance_m = float(msg.data)

    def pedestrian_callback(self, msg: Bool):
        value = bool(msg.data)
        if value and not self.last_pedestrian_detected:
            self.record_event("PEDESTRIAN_DETECTED", "true")
        self.last_pedestrian_detected = value

    def zone_callback(self, msg: Bool):
        value = bool(msg.data)
        if value and not self.last_zone_trigger:
            self.record_event("ZONE_TRIGGER", "true")
        self.last_zone_trigger = value

    def trial_state_callback(self, msg: String):
        value = msg.data.strip()
        self.trial_state = value

        if value == self.last_trial_state:
            return

        self.last_trial_state = value
        self.record_event("TRIAL_STATE", value)

        if value == "DONE":
            self.completed = True
            self.finalize("completed")

    def motion_phase_callback(self, msg: String):
        value = msg.data.strip()
        self.motion_phase = value

        if value == self.last_motion_phase:
            return

        self.last_motion_phase = value
        self.record_event("MOTION_PHASE", value)

    def ehmi_state_callback(self, msg: String):
        value = msg.data.strip()
        self.ehmi_state = value

        if value == self.last_ehmi_state:
            return

        self.last_ehmi_state = value
        self.record_event("EHMI_STATE", value)

    def sync_marker_callback(self, _msg: Empty):
        self.record_event("SYNC_MARKER", "manual_video_sync")
        self.get_logger().info("SYNC_MARKER recorded")

    def record_event(self, event_type, event_value=""):
        if self.finalized:
            return

        self.event_id += 1
        self.events_writer.writerow([
            self.event_id,
            f"{self._elapsed_s():.6f}",
            self._iso_now(),
            f"{self._ros_time_s():.9f}",
            str(event_type),
            str(event_value),
            self.trial_state,
            self.motion_phase,
            self.ehmi_state,
            self._csv_float(self.pedestrian_distance_m),
            self._csv_float(self.odom_x),
            self._csv_float(self.odom_y),
            self._csv_float(self.odom_yaw),
        ])
        self.events_file.flush()

    def finalize(self, reason):
        if self.finalized:
            return

        self.record_event("TRIAL_END", reason)
        self.events_file.flush()
        self.events_file.close()
        self.finalized = True
        self.get_logger().info(
            f"Trial data finalized: reason={reason}, dir={self.trial_dir}"
        )

    def shutdown_before_done(self, reason="manual_stop"):
        if not self.finalized:
            self.record_event("MANUAL_ABORT", reason)
            self.finalize(reason)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = None
    stop_requested = {"value": False, "reason": "manual_stop"}

    def handle_signal(signum, _frame):
        stop_requested["value"] = True
        stop_requested["reason"] = (
            "ctrl_c" if signum == signal.SIGINT else "terminated"
        )

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        node = ExperimentDataLogger()
        while rclpy.ok() and not stop_requested["value"]:
            rclpy.spin_once(node, timeout_sec=0.2)

    except Exception as exc:
        if node is not None and not node.finalized:
            node.record_event("LOGGER_ERROR", repr(exc))
            node.finalize("logger_error")
        raise

    finally:
        if node is not None:
            if stop_requested["value"] and not node.finalized:
                node.shutdown_before_done(stop_requested["reason"])
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
