#!/usr/bin/env python3

import math
import os
from pathlib import Path
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Int32


WAYPOINT_YAML = os.path.expanduser(os.environ.get(
    "TB3_WAYPOINT_YAML",
    str(Path(__file__).resolve().parents[1] / "config" / "bottleneck_v3_waypoints.yaml"),
))

# Frozen common control across all six formal conditions.
# After pedestrian trigger: briefly slow from 0.15 to 0.08 m/s,
# then stop for exactly 2.0 s before the selected action/eHMI.
POST_DETECTION_PAUSE = 2.0
DETECTION_DECEL_TIME = 0.35


class MapWaypointController(Node):

    def __init__(self):
        super().__init__("map_waypoint_controller")

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------
        self.declare_parameter("action", "APPROACH")
        self.declare_parameter("condition", "")
        self.declare_parameter("armed", False)
        self.declare_parameter("trial_mode", False)

        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.35)
        self.declare_parameter("heading_kp", 1.2)

        self.declare_parameter("goal_tolerance", 0.08)

        # APPROACH:
        # straight forward distance from A_START.
        self.declare_parameter("approach_distance", 2.28)
        self.declare_parameter("approach_yaw_kp", 1.5)
        self.declare_parameter("approach_max_angular_speed", 0.15)

        # Formal six-condition pre-detection cruising speed.
        # This is common to all six conditions and applies only
        # before the pedestrian trigger. All formal ACTION motion
        # keeps the validated linear_speed = 0.08 m/s.
        self.declare_parameter("formal_approach_speed", 0.15)

        # STRAIGHT_YIELD: straight reverse distance from P1.
        self.declare_parameter("reverse_distance", 1.48)

        # Formal STRAIGHT_YIELD dynamic reverse target:
        # reverse by the actual APPROACH distance minus 0.20 m,
        # leaving the robot approximately 0.20 m ahead of A_START.
        self.declare_parameter("reverse_safety_margin", 0.20)
        self.declare_parameter("reverse_yaw_kp", 1.2)
        self.declare_parameter("reverse_max_angular_speed", 0.12)

        # STRAIGHT_YIELD post-yield sequence:
        # wait for pedestrian, then resume straight forward.
        self.declare_parameter("straight_yield_wait_time", 10.0)
        self.declare_parameter("resume_forward_distance", 3.50)
        self.declare_parameter("resume_yaw_kp", 0.8)
        self.declare_parameter("resume_max_angular_speed", 0.08)

        # SIDE_YIELD stage 1:
        # straight reverse distance from P1 before turning.
        self.declare_parameter("side_reverse_distance", 0.25)

        # SIDE_YIELD stage 2:
        # reverse while turning until this relative yaw change.
        self.declare_parameter("side_turn_angle", 1.54)
        self.declare_parameter("side_turn_angular_speed", 0.18)

        # CLAIM:
        # straight forward distance from P1.
        self.declare_parameter("claim_distance", 2.97)
        self.declare_parameter("claim_yaw_kp", 0.8)
        self.declare_parameter("claim_max_angular_speed", 0.08)

        # AMCL may not publish continuously while stationary.
        self.declare_parameter("amcl_timeout", 15.0)

        # LaserScan is expected continuously.
        self.declare_parameter("scan_timeout", 1.0)

        # Odom is expected continuously while moving.
        self.declare_parameter("odom_timeout", 1.0)

        self.requested_action = self.get_parameter("action").value
        self.condition = str(
            self.get_parameter("condition").value
        ).strip().upper()

        self.condition_map = {
            "SYN": ("STRAIGHT_YIELD", "NEUTRAL"),
            "SYE": ("STRAIGHT_YIELD", "EXPLICIT"),
            "SDN": ("SIDE_YIELD", "NEUTRAL"),
            "SDE": ("SIDE_YIELD", "EXPLICIT"),
            "CN": ("CLAIM", "NEUTRAL"),
            "CE": ("CLAIM", "EXPLICIT"),
        }

        if self.condition:
            if self.condition not in self.condition_map:
                raise ValueError(
                    f"Unknown condition '{self.condition}'. "
                    "Available formal conditions: "
                    "SYN, SYE, SDN, SDE, CN, CE"
                )

            self.action, self.ehmi_mode = (
                self.condition_map[self.condition]
            )
        else:
            # Preserve standalone engineering action tests.
            self.action = self.requested_action
            self.ehmi_mode = "NEUTRAL"

        self.armed = bool(
            self.get_parameter("armed").value
        )

        self.trial_mode = bool(
            self.get_parameter("trial_mode").value
        )

        self.linear_speed = float(
            self.get_parameter("linear_speed").value
        )

        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )

        self.heading_kp = float(
            self.get_parameter("heading_kp").value
        )

        self.goal_tolerance = float(
            self.get_parameter("goal_tolerance").value
        )

        self.approach_distance = float(
            self.get_parameter("approach_distance").value
        )

        self.approach_yaw_kp = float(
            self.get_parameter("approach_yaw_kp").value
        )

        self.approach_max_angular_speed = float(
            self.get_parameter(
                "approach_max_angular_speed"
            ).value
        )

        self.formal_approach_speed = float(
            self.get_parameter(
                "formal_approach_speed"
            ).value
        )

        self.reverse_distance = float(
            self.get_parameter("reverse_distance").value
        )

        self.reverse_safety_margin = float(
            self.get_parameter("reverse_safety_margin").value
        )

        self.reverse_yaw_kp = float(
            self.get_parameter("reverse_yaw_kp").value
        )

        self.reverse_max_angular_speed = float(
            self.get_parameter("reverse_max_angular_speed").value
        )

        self.straight_yield_wait_time = float(
            self.get_parameter("straight_yield_wait_time").value
        )

        self.resume_forward_distance = float(
            self.get_parameter("resume_forward_distance").value
        )

        self.resume_yaw_kp = float(
            self.get_parameter("resume_yaw_kp").value
        )

        self.resume_max_angular_speed = float(
            self.get_parameter(
                "resume_max_angular_speed"
            ).value
        )

        self.side_reverse_distance = float(
            self.get_parameter("side_reverse_distance").value
        )

        self.side_turn_angle = float(
            self.get_parameter("side_turn_angle").value
        )

        self.side_turn_angular_speed = float(
            self.get_parameter("side_turn_angular_speed").value
        )

        self.claim_distance = float(
            self.get_parameter("claim_distance").value
        )

        self.claim_yaw_kp = float(
            self.get_parameter("claim_yaw_kp").value
        )

        self.claim_max_angular_speed = float(
            self.get_parameter(
                "claim_max_angular_speed"
            ).value
        )

        self.amcl_timeout = float(
            self.get_parameter("amcl_timeout").value
        )

        self.scan_timeout = float(
            self.get_parameter("scan_timeout").value
        )

        self.odom_timeout = float(
            self.get_parameter("odom_timeout").value
        )

        # --------------------------------------------------
        # Load waypoint YAML
        # --------------------------------------------------
        with open(
            WAYPOINT_YAML,
            "r",
            encoding="utf-8",
        ) as f:
            self.config = yaml.safe_load(f)

        self.points = self.config["points"]
        self.routes = self.config["routes"]

        if self.action not in self.routes:
            raise ValueError(
                f"Unknown action '{self.action}'. "
                f"Available actions: "
                f"{list(self.routes.keys())}"
            )

        self.route = self.routes[self.action]

        if self.trial_mode and not self.condition:
            raise ValueError(
                "Formal trial_mode requires condition:= "
                "SYN, SYE, SDN, SDE, CN, or CE. "
                "Use action:=... only for standalone "
                "engineering tests with trial_mode:=false."
            )

        if (
            self.trial_mode
            and self.action == "APPROACH"
        ):
            raise ValueError(
                "trial_mode requires a formal six-condition "
                "selection."
            )

        # --------------------------------------------------
        # Development-stage motion gate
        # --------------------------------------------------
        self.motion_enabled = (
            self.action in {
                "APPROACH",
                "STRAIGHT_YIELD",
                "SIDE_YIELD",
                "CLAIM",
            }
            and self.armed
        )

        if self.action == "APPROACH":

            expected_route = [
                "A_START",
                "P1_CONFLICT",
            ]

            if self.route != expected_route:
                raise ValueError(
                    "APPROACH route must be "
                    "A_START -> P1_CONFLICT"
                )

        elif self.action == "STRAIGHT_YIELD":

            expected_route = [
                "P1_CONFLICT",
                "P2_STRAIGHT_YIELD_END",
            ]

            if self.route != expected_route:
                raise ValueError(
                    "STRAIGHT_YIELD route must be "
                    "P1_CONFLICT -> "
                    "P2_STRAIGHT_YIELD_END"
                )

        elif self.action == "SIDE_YIELD":

            expected_route = [
                "P1_CONFLICT",
                "P3_SIDE_YIELD_STRAIGHT",
                "P4_SIDE_YIELD_END",
            ]

            if self.route != expected_route:
                raise ValueError(
                    "SIDE_YIELD route must be "
                    "P1_CONFLICT -> "
                    "P3_SIDE_YIELD_STRAIGHT -> "
                    "P4_SIDE_YIELD_END"
                )

        elif self.action == "CLAIM":

            expected_route = [
                "P1_CONFLICT",
                "P5_CLAIM_END",
            ]

            if self.route != expected_route:
                raise ValueError(
                    "CLAIM route must be "
                    "P1_CONFLICT -> P5_CLAIM_END"
                )

        # --------------------------------------------------
        # State
        # --------------------------------------------------
        self.latest_pose = None
        self.latest_odom_pose = None

        self.last_amcl_time = None
        self.last_scan_time = None
        self.last_odom_time = None

        self.reverse_start_xy = None
        self.reverse_start_yaw = None

        # Formal trial distance bookkeeping.
        self.actual_approach_distance = None
        self.current_reverse_target = None

        # SIDE_YIELD state:
        # 1 = straight reverse
        # 2 = reverse turning
        self.side_stage = 1
        self.side_turn_start_yaw = None

        self.claim_start_xy = None
        self.claim_start_yaw = None

        self.approach_start_xy = None
        self.approach_start_yaw = None

        # STRAIGHT_YIELD post-action state.
        self.straight_yield_wait_start = None
        self.last_countdown_value = None
        self.resume_start_xy = None
        self.resume_start_yaw = None

        self.goal_reached = False

        # Integrated trial state. In trial_mode the selected
        # action is given by the existing action parameter,
        # but every run begins with APPROACH.
        self.trial_state = (
            "APPROACH"
            if self.trial_mode
            else None
        )
        self.pedestrian_detected = False
        self.last_pedestrian_detected = False

        # Dual-trigger formal perception:
        # either the validated stable detector OR the map-zone trigger
        # may trigger the encounter. Both are latched as events.
        self.pedestrian_detector_latched = False

        # Formal experiment spatial trigger:
        # /pedestrian_zone_trigger becomes True only when a stable
        # detector candidate lies inside PEDESTRIAN_TRIGGER_ZONE.
        self.pedestrian_zone_trigger = False
        self.last_pedestrian_zone_trigger = False

        # Rising-edge latch: once the pedestrian has entered the
        # formal trigger zone during this trial, later False samples
        # must not cancel the event before the 10 Hz control loop sees it.
        self.pedestrian_zone_trigger_latched = False

        # Fixed six-condition post-detection pause.
        self.detection_pause_start = None
        self.detection_decel_start = None

        self.goal_name = self.route[-1]
        self.goal = self.points[self.goal_name]

        # --------------------------------------------------
        # Publisher
        # --------------------------------------------------
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        # eHMI display bridge topics.
        self.ehmi_state_pub = self.create_publisher(
            String,
            "/ehmi_state",
            10,
        )

        self.ehmi_countdown_pub = self.create_publisher(
            Int32,
            "/ehmi_countdown",
            10,
        )

        # Read-only experiment-state interfaces for the data logger.
        # These publishers do not participate in motion control.
        self.trial_state_pub = self.create_publisher(
            String,
            "/trial_state",
            10,
        )

        self.motion_phase_pub = self.create_publisher(
            String,
            "/motion_phase",
            10,
        )

        self.last_ehmi_state = None
        self.last_published_trial_state = None
        self.last_published_motion_phase = None

        # --------------------------------------------------
        # AMCL QoS
        #
        # Match:
        # RELIABLE
        # TRANSIENT_LOCAL
        # --------------------------------------------------
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            amcl_qos,
        )

        # --------------------------------------------------
        # LaserScan QoS
        #
        # TurtleBot3 ld08_driver uses BEST_EFFORT.
        # --------------------------------------------------
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.pedestrian_sub = self.create_subscription(
            Bool,
            "/pedestrian_detected",
            self.pedestrian_callback,
            10,
        )

        self.pedestrian_zone_sub = self.create_subscription(
            Bool,
            "/pedestrian_zone_trigger",
            self.pedestrian_zone_callback,
            10,
        )

        # 10 Hz control loop
        self.timer = self.create_timer(
            0.1,
            self.control_loop,
        )

        # --------------------------------------------------
        # Startup information
        # --------------------------------------------------
        self.get_logger().info(
            f"Loaded map: {self.config['map_id']}"
        )

        self.get_logger().info(
            f"Frame: {self.config['frame_id']}"
        )

        self.get_logger().info(
            f"Action: {self.action}"
        )

        if self.condition:
            self.get_logger().info(
                f"Condition: {self.condition}"
            )
            self.get_logger().info(
                f"eHMI mode: {self.ehmi_mode}"
            )

        self.get_logger().info(
            f"Route: {' -> '.join(self.route)}"
        )

        self.get_logger().info(
            f"Armed: {self.armed}"
        )

        self.get_logger().info(
            f"Trial mode: {self.trial_mode}"
        )

        if self.trial_mode:
            self.get_logger().info(
                "Formal integrated sequence: APPROACH "
                "(interruptible by map zone) -> "
                "DECELERATE -> DETECTION_STOP (2.0 s) -> "
                f"{self.action} -> POST_ACTION -> DONE"
            )
            self.get_logger().info(
                f"Formal pre-detection APPROACH speed: "
                f"{self.formal_approach_speed:.2f} m/s"
            )
            self.get_logger().info(
                f"Formal ACTION linear speed remains: "
                f"{self.linear_speed:.2f} m/s"
            )
            self.get_logger().info(
                "Formal trigger logic: OR"
            )
            self.get_logger().info(
                "Trigger channel A: /pedestrian_detected "
                "(validated stable detector)"
            )
            self.get_logger().info(
                "Trigger channel B: /pedestrian_zone_trigger "
                "(map-zone spatial trigger)"
            )
            self.get_logger().info(
                f"Post-detection full stop: "
                f"{POST_DETECTION_PAUSE:.1f} s "
                "(fixed for all six conditions)"
            )
            self.get_logger().info(
                "Formal display: Stuart v2 staged intention language; countdown disabled"
            )

        self.get_logger().info(
            f"AMCL timeout: "
            f"{self.amcl_timeout:.1f} s"
        )

        self.get_logger().info(
            f"Scan timeout: "
            f"{self.scan_timeout:.1f} s"
        )

        self.get_logger().info(
            f"Odom timeout: "
            f"{self.odom_timeout:.1f} s"
        )

        if self.action == "APPROACH":
            self.get_logger().info(
                f"APPROACH straight distance: "
                f"{self.approach_distance:.2f} m"
            )
            self.get_logger().info(
                f"APPROACH yaw hold: kp="
                f"{self.approach_yaw_kp:.2f}, "
                f"max_wz="
                f"{self.approach_max_angular_speed:.2f}"
            )

        elif self.action == "STRAIGHT_YIELD":
            self.get_logger().info(
                f"Standalone reverse distance: "
                f"{self.reverse_distance:.2f} m"
            )
            if self.trial_mode:
                self.get_logger().info(
                    "Formal STRAIGHT_YIELD reverse target: "
                    "actual APPROACH distance - "
                    f"{self.reverse_safety_margin:.2f} m"
                )
            self.get_logger().info(
                f"Post-yield wait: "
                f"{self.straight_yield_wait_time:.1f} s"
            )
            self.get_logger().info(
                f"Resume forward distance: "
                f"{self.resume_forward_distance:.2f} m"
            )

        elif self.action == "SIDE_YIELD":
            self.get_logger().info(
                f"SIDE_YIELD stage-1 reverse distance: "
                f"{self.side_reverse_distance:.2f} m"
            )
            self.get_logger().info(
                f"SIDE_YIELD stage-2 turn angle: "
                f"{self.side_turn_angle:.2f} rad"
            )
            self.get_logger().info(
                f"SIDE_YIELD stage-2 angular speed: "
                f"{self.side_turn_angular_speed:.2f} rad/s"
            )

        elif self.action == "CLAIM":
            self.get_logger().info(
                f"CLAIM forward distance: "
                f"{self.claim_distance:.2f} m"
            )
            self.get_logger().info(
                f"CLAIM yaw hold: kp="
                f"{self.claim_yaw_kp:.2f}, "
                f"max_wz="
                f"{self.claim_max_angular_speed:.2f}"
            )

        for point_name in self.route:

            point = self.points[point_name]

            self.get_logger().info(
                f"{point_name}: "
                f"x={point['x']:.6f}, "
                f"y={point['y']:.6f}, "
                f"yaw={point['yaw']:.4f}"
            )

        if self.motion_enabled:

            if self.action == "APPROACH":
                self.get_logger().warn(
                    "APPROACH STRAIGHT FORWARD "
                    "MOTION ENABLED."
                )

            elif self.action == "STRAIGHT_YIELD":
                self.get_logger().warn(
                    "STRAIGHT_YIELD STRAIGHT "
                    "REVERSE MOTION ENABLED."
                )

            elif self.action == "SIDE_YIELD":
                self.get_logger().warn(
                    "SIDE_YIELD MOTION ENABLED: "
                    "STAGE 1 STRAIGHT REVERSE + "
                    "STAGE 2 REVERSE TURN."
                )

            elif self.action == "CLAIM":
                self.get_logger().warn(
                    "CLAIM STRAIGHT FORWARD "
                    "MOTION ENABLED."
                )

        else:

            self.get_logger().warn(
                "MOTION DISABLED. "
                "Only action=APPROACH, "
                "STRAIGHT_YIELD, SIDE_YIELD, "
                "or CLAIM with armed:=true may send "
                "non-zero velocity."
            )

    # ------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------

    def amcl_callback(self, msg):

        pose = msg.pose.pose

        x = pose.position.x
        y = pose.position.y

        q = pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        self.latest_pose = (
            x,
            y,
            yaw,
        )

        self.last_amcl_time = (
            time.monotonic()
        )

    def scan_callback(self, msg):

        self.last_scan_time = (
            time.monotonic()
        )

    def pedestrian_callback(self, msg):

        # Validated stable detector signal. This is one of the two
        # formal trigger channels (OR with the map-zone trigger).
        self.pedestrian_detected = bool(msg.data)

        if (
            self.pedestrian_detected
            and not self.last_pedestrian_detected
        ):
            self.pedestrian_detector_latched = True

            self.get_logger().info(
                "Pedestrian detector trigger: TRUE "
                "(latched)"
            )

            if self.trial_mode:
                self.publish_detection_ehmi()

        self.last_pedestrian_detected = (
            self.pedestrian_detected
        )

    def pedestrian_zone_callback(self, msg):

        self.pedestrian_zone_trigger = bool(
            msg.data
        )

        if (
            self.pedestrian_zone_trigger
            and not self.last_pedestrian_zone_trigger
        ):
            # Formal spatial trigger is an event, not a level.
            # Once TRUE occurs, latch it for the rest of the
            # pre-action phase so a subsequent FALSE sample cannot
            # erase the trigger before the control loop handles it.
            self.pedestrian_zone_trigger_latched = True

            self.get_logger().info(
                "PEDESTRIAN_TRIGGER_ZONE entered: TRUE "
                "(latched)"
            )

            if self.trial_mode:
                self.publish_detection_ehmi()

        self.last_pedestrian_zone_trigger = (
            self.pedestrian_zone_trigger
        )

    def odom_callback(self, msg):

        pose = msg.pose.pose

        q = pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        self.latest_odom_pose = (
            pose.position.x,
            pose.position.y,
            yaw,
        )

        self.last_odom_time = (
            time.monotonic()
        )

    # ------------------------------------------------------
    # Helpers
    # ------------------------------------------------------

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def publish_zero(self):

        self.cmd_pub.publish(
            Twist()
        )

    def publish_ehmi_state(self, state):

        if state == self.last_ehmi_state:
            return

        msg = String()
        msg.data = state

        self.ehmi_state_pub.publish(msg)
        self.last_ehmi_state = state

        self.get_logger().info(
            f"eHMI state -> {state}"
        )

    def publish_detection_ehmi(self):
        """Publish the common encounter acknowledgement without leaking action.

        NEUTRAL remains status-only.
        EXPLICIT uses conversational acknowledgement, but still does not
        reveal which action will follow until the fixed 2 s stop is complete.
        """
        if not self.trial_mode:
            return

        if self.ehmi_mode == "EXPLICIT":
            self.publish_ehmi_state("EXPLICIT_DETECTED")
        else:
            self.publish_ehmi_state("NEUTRAL_DETECTED")

    def publish_ehmi_countdown(self, seconds):

        value = max(0, int(seconds))

        if value == self.last_countdown_value:
            return

        msg = Int32()
        msg.data = value

        self.ehmi_countdown_pub.publish(msg)
        self.last_countdown_value = value

        self.get_logger().info(
            f"eHMI countdown -> {value}"
        )

    def publish_formal_action_ehmi(self, phase):
        """Publish display states only; never changes robot motion.

        NEUTRAL:
          PRE     -> STOPPED
          RUNNING -> MOVING
        It intentionally does not reveal direction or right-of-way.

        EXPLICIT:
          names the upcoming action before motion and gives richer,
          action-linked stage information while the motion is occurring.
        """
        if not self.trial_mode:
            return

        phase = str(phase).upper()

        if self.ehmi_mode == "NEUTRAL":
            if phase == "PRE":
                self.publish_ehmi_state("NEUTRAL_STOPPED")
            else:
                self.publish_ehmi_state("NEUTRAL_MOVING")
            return

        if self.action == "STRAIGHT_YIELD":
            state = (
                "EXPLICIT_STRAIGHT_PRE"
                if phase == "PRE"
                else "EXPLICIT_STRAIGHT_RUNNING"
            )
            self.publish_ehmi_state(state)
            return

        if self.action == "SIDE_YIELD":
            if phase == "PRE":
                state = "EXPLICIT_SIDE_PRE"
            elif self.side_stage == 1:
                state = "EXPLICIT_SIDE_STAGE1"
            else:
                state = "EXPLICIT_SIDE_STAGE2"
            self.publish_ehmi_state(state)
            return

        if self.action == "CLAIM":
            state = (
                "EXPLICIT_CLAIM_PRE"
                if phase == "PRE"
                else "EXPLICIT_CLAIM_RUNNING"
            )
            self.publish_ehmi_state(state)
            return

        raise RuntimeError(
            f"Unsupported formal action: {self.action}"
        )

    def publish_trial_state(self, state):
        """Publish controller trial state only when it changes."""
        state = str(state)

        if state == self.last_published_trial_state:
            return

        msg = String()
        msg.data = state
        self.trial_state_pub.publish(msg)
        self.last_published_trial_state = state

        self.get_logger().info(
            f"trial_state -> {state}"
        )

    def publish_motion_phase(self, phase):
        """Publish read-only motion phase only when it changes."""
        phase = str(phase)

        if phase == self.last_published_motion_phase:
            return

        msg = String()
        msg.data = phase
        self.motion_phase_pub.publish(msg)
        self.last_published_motion_phase = phase

        self.get_logger().info(
            f"motion_phase -> {phase}"
        )

    def formal_pedestrian_triggered(self):

        # OR logic: either validated detector evidence or
        # map-zone evidence is sufficient to trigger the trial.
        return (
            self.pedestrian_detector_latched
            or self.pedestrian_zone_trigger_latched
        )

    def current_approach_distance(self):

        if (
            self.approach_start_xy is None
            or self.latest_odom_pose is None
        ):
            return 0.0

        odom_x, odom_y, _ = self.latest_odom_pose
        start_x, start_y = self.approach_start_xy

        return math.hypot(
            odom_x - start_x,
            odom_y - start_y,
        )

    def data_status(self):

        now = time.monotonic()

        if self.last_amcl_time is None:
            return False, "AMCL missing"

        if self.last_scan_time is None:
            return False, "scan missing"

        if (
            self.action in {
                "APPROACH",
                "STRAIGHT_YIELD",
                "SIDE_YIELD",
                "CLAIM",
            }
            and self.last_odom_time is None
        ):
            return False, "odom missing"

        amcl_age = (
            now - self.last_amcl_time
        )

        scan_age = (
            now - self.last_scan_time
        )

        odom_age = None
        if self.last_odom_time is not None:
            odom_age = now - self.last_odom_time

        if amcl_age > self.amcl_timeout:

            return (
                False,
                f"AMCL stale "
                f"({amcl_age:.2f}s)"
            )

        if scan_age > self.scan_timeout:

            return (
                False,
                f"scan stale "
                f"({scan_age:.2f}s)"
            )

        if (
            self.action in {
                "APPROACH",
                "STRAIGHT_YIELD",
                "SIDE_YIELD",
                "CLAIM",
            }
            and odom_age is not None
            and odom_age > self.odom_timeout
        ):

            return (
                False,
                f"odom stale "
                f"({odom_age:.2f}s)"
            )

        return True, "OK"

    # ------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------

    def control_loop(self):

        # Motion gate
        if not self.motion_enabled:

            self.publish_zero()
            return

        # Integrated trial flow. Standalone action behaviour
        # below is left unchanged when trial_mode is false.
        if self.trial_mode:

            self.control_trial_sequence()
            return

        # Standalone engineering tests: publish read-only interfaces too.
        self.publish_trial_state("STANDALONE")
        if self.action == "APPROACH":
            self.publish_motion_phase("APPROACH")
        elif self.action == "STRAIGHT_YIELD":
            self.publish_motion_phase("STRAIGHT_YIELD")
        elif self.action == "SIDE_YIELD":
            if self.side_stage == 1:
                self.publish_motion_phase("SIDE_YIELD_STAGE1")
            else:
                self.publish_motion_phase("SIDE_YIELD_STAGE2")
        elif self.action == "CLAIM":
            self.publish_motion_phase("CLAIM")
        else:
            self.publish_motion_phase("IDLE")

        # Already finished
        if self.goal_reached:

            self.publish_zero()
            return

        # Sensor / localization safety
        data_ok, reason = self.data_status()

        if not data_ok:

            self.publish_zero()

            self.get_logger().warn(
                f"Motion blocked: {reason}",
                throttle_duration_sec=2.0,
            )

            return

        if self.latest_pose is None:

            self.publish_zero()
            return

        if self.action == "APPROACH":

            self.control_approach_straight()
            return

        if self.action == "STRAIGHT_YIELD":

            self.control_straight_yield()
            return

        if self.action == "SIDE_YIELD":

            self.control_side_yield_stage1()
            return

        if self.action == "CLAIM":

            self.control_claim()
            return

        # --------------------------------------------------
        # Current pose
        # --------------------------------------------------
        x, y, yaw = self.latest_pose

        goal_x = float(
            self.goal["x"]
        )

        goal_y = float(
            self.goal["y"]
        )

        dx = goal_x - x
        dy = goal_y - y

        distance = math.hypot(
            dx,
            dy,
        )

        # --------------------------------------------------
        # Goal reached
        # --------------------------------------------------
        if distance <= self.goal_tolerance:

            self.goal_reached = True

            self.publish_zero()

            self.get_logger().info(
                f"Reached {self.goal_name}. "
                f"distance={distance:.3f} m"
            )

            return

        # --------------------------------------------------
        # Heading control
        # --------------------------------------------------
        target_heading = math.atan2(
            dy,
            dx,
        )

        heading_error = (
            self.normalize_angle(
                target_heading - yaw
            )
        )

        angular_cmd = (
            self.heading_kp
            * heading_error
        )

        angular_cmd = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                angular_cmd,
            ),
        )

        cmd = Twist()

        # Slow forward motion if heading error is large.
        if abs(heading_error) > 0.45:

            cmd.linear.x = 0.03

        else:

            cmd.linear.x = (
                self.linear_speed
            )

        cmd.angular.z = angular_cmd

        self.cmd_pub.publish(
            cmd
        )

        self.get_logger().info(
            f"APPROACH -> {self.goal_name}: "
            f"distance={distance:.3f} m, "
            f"heading_error="
            f"{heading_error:.3f} rad, "
            f"vx={cmd.linear.x:.3f}, "
            f"wz={cmd.angular.z:.3f}",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------
    # Integrated trial sequence
    #
    # APPROACH (zone-interruptible) -> DETECTION_PAUSE (1.5 s) -> ACTION -> POST_ACTION -> DONE
    #
    # The already validated motion functions below are reused
    # without changing their motion parameters.
    # ------------------------------------------------------

    def control_trial_sequence(self):

        # Publish read-only state interfaces for experiment logging.
        self.publish_trial_state(self.trial_state)

        if self.trial_state == "APPROACH":
            self.publish_motion_phase("APPROACH")
        elif self.trial_state == "WAIT_FOR_TRIGGER":
            self.publish_motion_phase("STOPPED")
        elif self.trial_state == "DECELERATE":
            self.publish_motion_phase("DECELERATE")
        elif self.trial_state == "DETECTION_STOP":
            self.publish_motion_phase("STOPPED")
        elif self.trial_state == "ACTION":
            if self.action == "STRAIGHT_YIELD":
                self.publish_motion_phase("STRAIGHT_YIELD")
            elif self.action == "SIDE_YIELD":
                if self.side_stage == 1:
                    self.publish_motion_phase("SIDE_YIELD_STAGE1")
                elif self.side_stage == 2:
                    self.publish_motion_phase("SIDE_YIELD_STAGE2")
                else:
                    self.publish_motion_phase("IDLE")
            elif self.action == "CLAIM":
                self.publish_motion_phase("CLAIM")
            else:
                self.publish_motion_phase("IDLE")
        elif self.trial_state == "DONE":
            self.publish_motion_phase("DONE")
        else:
            self.publish_motion_phase("IDLE")

        if self.trial_state == "DONE":
            self.publish_zero()
            return

        # Keep all existing stale-data safety gates active.
        data_ok, reason = self.data_status()

        if not data_ok:
            self.publish_zero()
            self.get_logger().warn(
                f"Motion blocked: {reason}",
                throttle_duration_sec=2.0,
            )
            return

        if self.latest_pose is None:
            self.publish_zero()
            return

        if self.trial_state == "APPROACH":

            self.publish_ehmi_state(
                "APPROACH"
            )

            # Spatial trigger has priority over the nominal 1.28 m
            # APPROACH maximum distance.
            if self.formal_pedestrian_triggered():
                # Record how far the robot actually travelled in APPROACH.
                # STRAIGHT_YIELD will use this to compute its reverse target.
                self.actual_approach_distance = (
                    self.current_approach_distance()
                )

                self.get_logger().info(
                    "Formal APPROACH distance captured at trigger: "
                    f"{self.actual_approach_distance:.3f} m"
                )

                # Pedestrian detection has highest priority.
                # Give a short, visible speed reduction before the full stop.
                self.publish_detection_ehmi()

                self.detection_decel_start = (
                    time.monotonic()
                )
                self.trial_state = "DECELERATE"

                self.get_logger().info(
                    "Pedestrian trigger has priority over APPROACH. "
                    f"State -> DECELERATE at {self.linear_speed:.2f} m/s, "
                    f"then stop for {POST_DETECTION_PAUSE:.1f} s."
                )
                return

            self.control_approach_straight()

            if self.goal_reached:
                # 2.28 m is only the maximum APPROACH distance.
                # Save the actual odometry displacement before waiting.
                self.actual_approach_distance = (
                    self.current_approach_distance()
                )

                # If no spatial trigger has happened, stop and wait.
                self.goal_reached = False
                self.trial_state = "WAIT_FOR_TRIGGER"
                self.publish_zero()

                self.get_logger().info(
                    "APPROACH maximum distance reached "
                    "without zone trigger. "
                    "State -> WAIT_FOR_TRIGGER"
                )
            return

        if self.trial_state == "WAIT_FOR_TRIGGER":
            self.publish_zero()

            if self.formal_pedestrian_triggered():

                if self.actual_approach_distance is None:
                    self.actual_approach_distance = (
                        self.current_approach_distance()
                    )

                self.publish_detection_ehmi()

                self.detection_pause_start = (
                    time.monotonic()
                )
                self.trial_state = "DETECTION_STOP"

                self.get_logger().info(
                    "Pedestrian trigger received while already stationary. "
                    f"State -> DETECTION_STOP for "
                    f"{POST_DETECTION_PAUSE:.1f} s."
                )
            return

        if self.trial_state == "DECELERATE":
            # Common pre-stop cue for all six conditions:
            # reduce speed from formal APPROACH 0.15 to 0.08 m/s.
            self.publish_detection_ehmi()
            self.control_detection_deceleration()

            if self.detection_decel_start is None:
                self.detection_decel_start = time.monotonic()

            elapsed = time.monotonic() - self.detection_decel_start

            if elapsed < DETECTION_DECEL_TIME:
                return

            self.publish_zero()
            self.detection_pause_start = time.monotonic()
            self.detection_decel_start = None
            self.trial_state = "DETECTION_STOP"

            self.get_logger().info(
                "Deceleration cue complete. "
                f"Robot stopped for {POST_DETECTION_PAUSE:.1f} s."
            )
            return

        if self.trial_state == "DETECTION_STOP":
            # Common stopped acknowledgement for SYN/SYE/SDN/SDE/CN/CE.
            self.publish_zero()
            self.publish_detection_ehmi()

            if self.detection_pause_start is None:
                self.detection_pause_start = time.monotonic()

            elapsed = time.monotonic() - self.detection_pause_start

            if elapsed < POST_DETECTION_PAUSE:
                return

            self.prepare_selected_action()

            # Only after the common stopped acknowledgement may
            # Explicit conditions reveal action-specific information.
            self.publish_formal_action_ehmi(
                "PRE"
            )

            self.pedestrian_zone_trigger_latched = False
            self.pedestrian_detector_latched = False
            self.detection_pause_start = None

            self.trial_state = "ACTION"

            self.get_logger().info(
                f"Stopped acknowledgement complete "
                f"({POST_DETECTION_PAUSE:.1f} s). "
                "State -> ACTION "
                f"({self.condition}: "
                f"{self.action} + {self.ehmi_mode})"
            )
            return

        if self.trial_state == "ACTION":

            self.publish_formal_action_ehmi(
                "RUNNING"
            )

            if self.action == "STRAIGHT_YIELD":
                self.control_straight_yield()

            elif self.action == "SIDE_YIELD":
                self.control_side_yield_stage1()

            elif self.action == "CLAIM":
                self.control_claim()

            else:
                self.publish_zero()
                raise RuntimeError(
                    f"Unsupported trial action: {self.action}"
                )

            if self.goal_reached:
                self.publish_zero()

                if self.ehmi_mode == "NEUTRAL":
                    self.publish_ehmi_state("NEUTRAL_STOPPED")
                elif self.action == "STRAIGHT_YIELD":
                    self.publish_ehmi_state("EXPLICIT_STRAIGHT_COMPLETE")
                elif self.action == "SIDE_YIELD":
                    self.publish_ehmi_state("EXPLICIT_SIDE_COMPLETE")
                elif self.action == "CLAIM":
                    self.publish_ehmi_state("EXPLICIT_CLAIM_COMPLETE")

                self.trial_state = "DONE"

                self.get_logger().info(
                    f"Formal trial complete: "
                    f"{self.condition} "
                    f"({self.action} + {self.ehmi_mode}). "
                    "Final social display latched; State -> DONE"
                )

            return

        self.publish_zero()
        raise RuntimeError(
            f"Unknown trial state: {self.trial_state}"
        )

    def control_detection_deceleration(self):
        """Brief 0.08 m/s forward cue before the 2 s full stop."""
        if self.latest_odom_pose is None:
            self.publish_zero()
            return

        _, _, odom_yaw = self.latest_odom_pose

        target_yaw = (
            self.approach_start_yaw
            if self.approach_start_yaw is not None
            else odom_yaw
        )

        yaw_error = self.normalize_angle(
            target_yaw - odom_yaw
        )

        angular_cmd = self.approach_yaw_kp * yaw_error
        angular_cmd = max(
            -self.approach_max_angular_speed,
            min(self.approach_max_angular_speed, angular_cmd),
        )

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular_cmd
        self.cmd_pub.publish(cmd)

    def prepare_selected_action(self):

        self.goal_reached = False

        if self.action == "STRAIGHT_YIELD":
            self.reverse_start_xy = None
            self.reverse_start_yaw = None
            self.straight_yield_wait_start = None
            self.last_countdown_value = None
            self.resume_start_xy = None
            self.resume_start_yaw = None

            if self.trial_mode:
                if self.actual_approach_distance is None:
                    raise RuntimeError(
                        "Formal STRAIGHT_YIELD cannot compute reverse target: "
                        "actual APPROACH distance was not recorded."
                    )

                self.current_reverse_target = max(
                    0.0,
                    self.actual_approach_distance
                    - self.reverse_safety_margin,
                )

                self.get_logger().info(
                    "Formal STRAIGHT_YIELD dynamic target: "
                    f"approach={self.actual_approach_distance:.3f} m, "
                    f"margin={self.reverse_safety_margin:.3f} m, "
                    f"reverse_target={self.current_reverse_target:.3f} m"
                )
            else:
                # Preserve standalone engineering test behaviour.
                self.current_reverse_target = self.reverse_distance

            return

        if self.action == "SIDE_YIELD":
            self.reverse_start_xy = None
            self.side_stage = 1
            self.side_turn_start_yaw = None
            return

        if self.action == "CLAIM":
            self.claim_start_xy = None
            self.claim_start_yaw = None
            return

        raise RuntimeError(
            f"Unsupported selected action: {self.action}"
        )

    # ------------------------------------------------------
    # APPROACH
    #
    # No P1 map-coordinate steering is used here.
    # The robot moves straight forward from A_START for
    # approach_distance metres using odometry only.
    # ------------------------------------------------------

    def control_approach_straight(self):

        if self.latest_odom_pose is None:

            self.publish_zero()
            return

        odom_x, odom_y, odom_yaw = (
            self.latest_odom_pose
        )

        if self.approach_start_xy is None:

            self.approach_start_xy = (
                odom_x,
                odom_y,
            )

            self.approach_start_yaw = (
                odom_yaw
            )

            self.get_logger().info(
                "APPROACH forward start recorded. "
                f"yaw={odom_yaw:.3f} rad"
            )

        start_x, start_y = (
            self.approach_start_xy
        )

        travelled_distance = math.hypot(
            odom_x - start_x,
            odom_y - start_y,
        )

        if travelled_distance >= self.approach_distance:

            self.goal_reached = True
            self.publish_zero()

            self.get_logger().info(
                "APPROACH complete. "
                f"travelled_distance="
                f"{travelled_distance:.3f} m"
            )

            return

        yaw_error = self.normalize_angle(
            self.approach_start_yaw
            - odom_yaw
        )

        angular_cmd = (
            self.approach_yaw_kp
            * yaw_error
        )

        angular_cmd = max(
            -self.approach_max_angular_speed,
            min(
                self.approach_max_angular_speed,
                angular_cmd,
            ),
        )

        cmd = Twist()

        # Formal experiment: use the slightly faster common
        # pre-detection cruising speed. Standalone APPROACH
        # engineering tests retain the original linear_speed.
        if self.trial_mode:
            cmd.linear.x = self.formal_approach_speed
        else:
            cmd.linear.x = self.linear_speed

        cmd.angular.z = angular_cmd

        self.cmd_pub.publish(
            cmd
        )

        self.get_logger().info(
            "APPROACH straight forward + yaw hold: "
            f"travelled_distance="
            f"{travelled_distance:.3f} m, "
            f"target={self.approach_distance:.3f} m, "
            f"yaw_error={yaw_error:.3f} rad, "
            f"vx={cmd.linear.x:.3f}, "
            f"wz={cmd.angular.z:.3f}",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------
    # STRAIGHT_YIELD
    #
    # No P2 map-coordinate steering is used here.
    # The robot reverses straight from its start position
    # for reverse_distance metres using odometry only.
    # ------------------------------------------------------

    def control_straight_yield(self):

        if self.latest_odom_pose is None:

            self.publish_zero()
            return

        odom_x, odom_y, odom_yaw = self.latest_odom_pose

        if self.reverse_start_xy is None:

            self.reverse_start_xy = (
                odom_x,
                odom_y,
            )

            self.reverse_start_yaw = (
                odom_yaw
            )

            self.get_logger().info(
                "STRAIGHT_YIELD reverse start recorded. "
                f"yaw={odom_yaw:.3f} rad"
            )

        start_x, start_y = self.reverse_start_xy

        reversed_distance = math.hypot(
            odom_x - start_x,
            odom_y - start_y,
        )

        reverse_target = (
            self.current_reverse_target
            if self.current_reverse_target is not None
            else self.reverse_distance
        )

        if reversed_distance >= reverse_target:

            self.goal_reached = True

            self.publish_zero()

            self.get_logger().info(
                "STRAIGHT_YIELD complete. "
                f"reversed_distance={reversed_distance:.3f} m, "
                f"target={reverse_target:.3f} m"
            )

            return

        yaw_error = self.normalize_angle(
            self.reverse_start_yaw
            - odom_yaw
        )

        angular_cmd = (
            self.reverse_yaw_kp
            * yaw_error
        )

        angular_cmd = max(
            -self.reverse_max_angular_speed,
            min(
                self.reverse_max_angular_speed,
                angular_cmd,
            ),
        )

        cmd = Twist()

        cmd.linear.x = -self.linear_speed
        cmd.angular.z = angular_cmd

        self.cmd_pub.publish(
            cmd
        )

        self.get_logger().info(
            "STRAIGHT_YIELD reverse + yaw hold: "
            f"reversed_distance="
            f"{reversed_distance:.3f} m, "
            f"target={reverse_target:.3f} m, "
            f"yaw_error={yaw_error:.3f} rad, "
            f"vx={cmd.linear.x:.3f}, "
            f"wz={cmd.angular.z:.3f}",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------
    # STRAIGHT_YIELD resume forward
    #
    # New post-yield phase requested for the experiment:
    # after the 10 s yield wait, move straight forward
    # resume_forward_distance metres using odometry + yaw
    # hold. Existing APPROACH / CLAIM functions are not
    # modified.
    # ------------------------------------------------------

    def control_resume_forward(self):

        if self.latest_odom_pose is None:
            self.publish_zero()
            return

        odom_x, odom_y, odom_yaw = (
            self.latest_odom_pose
        )

        if self.resume_start_xy is None:

            self.resume_start_xy = (
                odom_x,
                odom_y,
            )

            self.resume_start_yaw = (
                odom_yaw
            )

            self.get_logger().info(
                "RESUME_FORWARD start recorded. "
                f"yaw={odom_yaw:.3f} rad"
            )

        start_x, start_y = (
            self.resume_start_xy
        )

        travelled_distance = math.hypot(
            odom_x - start_x,
            odom_y - start_y,
        )

        if (
            travelled_distance
            >= self.resume_forward_distance
        ):

            self.goal_reached = True
            self.publish_zero()

            self.get_logger().info(
                "RESUME_FORWARD complete. "
                f"travelled_distance="
                f"{travelled_distance:.3f} m"
            )

            return

        yaw_error = self.normalize_angle(
            self.resume_start_yaw
            - odom_yaw
        )

        angular_cmd = (
            self.resume_yaw_kp
            * yaw_error
        )

        angular_cmd = max(
            -self.resume_max_angular_speed,
            min(
                self.resume_max_angular_speed,
                angular_cmd,
            ),
        )

        cmd = Twist()

        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular_cmd

        self.cmd_pub.publish(
            cmd
        )

        self.get_logger().info(
            "RESUME_FORWARD straight + yaw hold: "
            f"travelled_distance="
            f"{travelled_distance:.3f} m, "
            f"target="
            f"{self.resume_forward_distance:.3f} m, "
            f"yaw_error={yaw_error:.3f} rad, "
            f"vx={cmd.linear.x:.3f}, "
            f"wz={cmd.angular.z:.3f}",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------
    # SIDE_YIELD
    #
    # Stage 1:
    #   straight reverse for side_reverse_distance metres.
    #
    # Stage 2:
    #   continue reversing while turning with a fixed
    #   positive angular velocity until the relative yaw
    #   change reaches side_turn_angle.
    #
    # P3/P4 map coordinates are not used for steering.
    # ------------------------------------------------------

    def control_side_yield_stage1(self):

        if self.latest_odom_pose is None:

            self.publish_zero()
            return

        odom_x, odom_y, odom_yaw = (
            self.latest_odom_pose
        )

        # ----------------------------------------------
        # Stage 1: straight reverse
        # ----------------------------------------------
        if self.side_stage == 1:

            if self.reverse_start_xy is None:

                self.reverse_start_xy = (
                    odom_x,
                    odom_y,
                )

                self.get_logger().info(
                    "SIDE_YIELD stage-1 "
                    "reverse start recorded."
                )

            start_x, start_y = (
                self.reverse_start_xy
            )

            reversed_distance = math.hypot(
                odom_x - start_x,
                odom_y - start_y,
            )

            if (
                reversed_distance
                >= self.side_reverse_distance
            ):

                self.side_stage = 2
                self.side_turn_start_yaw = (
                    odom_yaw
                )

                self.publish_zero()

                self.get_logger().info(
                    "SIDE_YIELD stage 1 complete. "
                    f"reversed_distance="
                    f"{reversed_distance:.3f} m. "
                    "Starting stage 2 reverse turn."
                )

                return

            cmd = Twist()
            cmd.linear.x = -self.linear_speed
            cmd.angular.z = 0.0

            self.cmd_pub.publish(
                cmd
            )

            self.get_logger().info(
                "SIDE_YIELD stage-1 straight reverse: "
                f"reversed_distance="
                f"{reversed_distance:.3f} m, "
                f"target="
                f"{self.side_reverse_distance:.3f} m, "
                f"vx={cmd.linear.x:.3f}, "
                f"wz={cmd.angular.z:.3f}",
                throttle_duration_sec=1.0,
            )

            return

        # ----------------------------------------------
        # Stage 2: reverse turn
        # ----------------------------------------------
        if self.side_stage == 2:

            if self.side_turn_start_yaw is None:

                self.side_turn_start_yaw = (
                    odom_yaw
                )

            turned_angle = (
                self.normalize_angle(
                    odom_yaw
                    - self.side_turn_start_yaw
                )
            )

            if turned_angle >= self.side_turn_angle:

                self.goal_reached = True
                self.publish_zero()

                self.get_logger().info(
                    "SIDE_YIELD complete. "
                    f"turned_angle="
                    f"{turned_angle:.3f} rad"
                )

                return

            cmd = Twist()

            cmd.linear.x = -self.linear_speed
            cmd.angular.z = (
                self.side_turn_angular_speed
            )

            self.cmd_pub.publish(
                cmd
            )

            self.get_logger().info(
                "SIDE_YIELD stage-2 reverse turn: "
                f"turned_angle="
                f"{turned_angle:.3f} rad, "
                f"target="
                f"{self.side_turn_angle:.3f} rad, "
                f"vx={cmd.linear.x:.3f}, "
                f"wz={cmd.angular.z:.3f}",
                throttle_duration_sec=1.0,
            )

            return

        self.publish_zero()


    # ------------------------------------------------------
    # CLAIM
    #
    # No P5 map-coordinate steering is used here.
    # The robot moves straight forward from P1 for
    # claim_distance metres using odometry only.
    # ------------------------------------------------------

    def control_claim(self):

        if self.latest_odom_pose is None:

            self.publish_zero()
            return

        odom_x, odom_y, odom_yaw = (
            self.latest_odom_pose
        )

        if self.claim_start_xy is None:

            self.claim_start_xy = (
                odom_x,
                odom_y,
            )

            self.claim_start_yaw = (
                odom_yaw
            )

            self.get_logger().info(
                "CLAIM forward start recorded. "
                f"yaw={odom_yaw:.3f} rad"
            )

        start_x, start_y = (
            self.claim_start_xy
        )

        travelled_distance = math.hypot(
            odom_x - start_x,
            odom_y - start_y,
        )

        if travelled_distance >= self.claim_distance:

            self.goal_reached = True
            self.publish_zero()

            self.get_logger().info(
                "CLAIM complete. "
                f"travelled_distance="
                f"{travelled_distance:.3f} m"
            )

            return

        yaw_error = self.normalize_angle(
            self.claim_start_yaw
            - odom_yaw
        )

        angular_cmd = (
            self.claim_yaw_kp
            * yaw_error
        )

        angular_cmd = max(
            -self.claim_max_angular_speed,
            min(
                self.claim_max_angular_speed,
                angular_cmd,
            ),
        )

        cmd = Twist()

        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular_cmd

        self.cmd_pub.publish(
            cmd
        )

        self.get_logger().info(
            "CLAIM straight forward + yaw hold: "
            f"travelled_distance="
            f"{travelled_distance:.3f} m, "
            f"target={self.claim_distance:.3f} m, "
            f"yaw_error={yaw_error:.3f} rad, "
            f"vx={cmd.linear.x:.3f}, "
            f"wz={cmd.angular.z:.3f}",
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------

    def stop_robot(self):

        for _ in range(5):

            self.cmd_pub.publish(
                Twist()
            )

            time.sleep(0.03)


def main(args=None):

    rclpy.init(
        args=args,
        signal_handler_options=(
            SignalHandlerOptions.NO
        ),
    )

    node = None

    try:

        node = MapWaypointController()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        print(
            "Ctrl+C received. "
            "Stopping robot..."
        )

    except Exception as exc:

        print(
            "map_waypoint_controller "
            f"error: {exc}"
        )

    finally:

        if node is not None:

            node.stop_robot()

            print(
                "Zero velocity command sent."
            )

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()

        print(
            "map_waypoint_controller "
            "stopped safely."
        )


if __name__ == "__main__":
    main()
