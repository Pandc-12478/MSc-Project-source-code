# TurtleBot3 eHMI bottleneck study — source code

ROS 2 Humble scripts for a TurtleBot3 Waffle pedestrian–robot encounter study. The controller combines localization, odometry, laser-based pedestrian detection and six condition selections. The browser display receives eHMI state and countdown messages through a small HTTP bridge.

## Status and provenance

This repository was inspected at commit `a7075ca0add633e870ffd14cae0b9278e7fc8aa7` (the uploaded August 19 source snapshot). The September 6 packaging changes improve paths, startup and documentation. They do **not** establish that this snapshot is the final code used for all study trials. Confirm that against the experiment computer before citing it as the final experimental implementation.

**The user-supplied map, waypoint YAML, localization launch and AMCL parameters are now included.** A minimal localization-only ROS package has been added; no old social policy code is included. This is not yet a standalone, hardware-validated reproduction package. No replacement coordinates or AMCL tuning have been invented.

## Requirements

- Ubuntu with ROS 2 Humble and Python 3; Bash launchers.
- ROS Python packages: `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs`, `tf2_ros`; Python `yaml` (PyYAML, commonly installed as `python3-yaml`).
- A working TurtleBot3 bringup on the robot, publishing laser scan, odometry and robot TF.
- ROS Nav2 localization components (`nav2_bringup`, AMCL, map server, lifecycle manager) and RViz2. Build the bundled localization-only package as shown below.
- The supplied map and waypoint coordinates must match the physical experiment layout.
- Robot, control computer and browser device on the appropriate network. Use the same `ROS_DOMAIN_ID` on all ROS devices; the launchers default to `30`. The HTML page uses ordinary HTTP on port `8080`; Flask and rosbridge are not required.

## Recovered configuration and build

The application scripts run directly. Only `ros_packages/turtlebot3_hmi_social` is a colcon package. Its historical name is retained for startup compatibility: it includes **only localization launch/configuration**, not social policy, motion controllers or automatic navigation. The launch and YAML files are unmodified user uploads. CMake/package metadata is newly added packaging, not recovered experimental source; maintainer contact and license remain explicitly unconfirmed.

From this repository root, build into a separate overlay (do not overwrite the original experiment workspace):

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths ros_packages --build-base .localization_build --install-base .localization_install
export TB3_WS_SETUP="$PWD/.localization_install/setup.bash"
```

Repeat the `export TB3_WS_SETUP=...` in each new terminal, using the absolute path if not at the repository root. This selects the recovered package rather than a stale installed copy.

The waypoint file retains `validated_for_motion: false`. File completeness does not establish physical validation; this flag has not been changed. The startup AMCL initial pose differs from A_START by about 0.116 m and 0.037 rad. Trial scripts reset the pose to A_START; neither pose has been silently altered. Verify physical alignment at startup and after trial reset.

The supplied P5 is about 1.466 m from P1. The controller also contains odometry-based CLAIM motion; do not interpret the waypoint separation as the actual trial travel distance. Confirm this snapshot against the final experimental implementation.

`config/bottleneck_map_v3_initial_pose.txt` is a reference record, not the active full waypoint configuration.

## Quick start

Clone/download the repository to any directory. Run these commands from its root. On the robot, start the existing, verified TurtleBot3 bringup separately.

### 1. Configure and check on the control computer

```bash
source scripts/runtime_env.sh
python3 scripts/check_runtime.py
```

The shared environment sources `/opt/ros/humble/setup.bash` and, when present, `~/turtlebot3_ws/install/setup.bash`. To use different locations, set these **before** sourcing it:

```bash
export TB3_WS_SETUP=/absolute/path/to/workspace/install/setup.bash
export TB3_WAYPOINT_YAML=/absolute/path/to/bottleneck_v3_waypoints.yaml
source scripts/runtime_env.sh
python3 scripts/check_runtime.py
```

Optional overrides: `ROS_SETUP`, `ROS_DOMAIN_ID`, `TB3_LOCALIZATION_PKG`, `TB3_LOCALIZATION_LAUNCH`, `TB3_EHMI_HTML`, `TB3_DATA_ROOT`. Use absolute paths for file overrides. Preserve the calibrated map/configuration pairing.

A non-ROS computer can check bundled file completeness with:

```bash
python3 scripts/check_runtime.py --files-only
```

A missing waypoint file produces an explicit failure. None of these checks publishes a ROS message or starts motion.

### 2. Start the support system

```bash
bash scripts/start_system.sh
```

Keep this terminal open. It starts the bundled localization launch, cluster detector, map-zone trigger and eHMI web bridge. It does not start the motion controller. Open `http://<control-computer-IP>:8080/` on the display device, since the bridge now runs on the computer executing this command. Do not simultaneously run another bridge on the same port or duplicate detector/localization nodes.

Verify localization in RViz (start RViz separately if needed), the map-to-robot TF chain, laser alignment and live topic data. Topic names existing alone does not establish healthy data.

```bash
ros2 topic list
ros2 topic info /cmd_vel -v
ros2 topic echo /pedestrian_zone_trigger --once
```

Before starting a trial there must be no competing `/cmd_vel` publisher. Use the physical experimental layout and a working emergency stop. **The trial scripts publish the saved A_START initial pose automatically; place and orient the robot at that real-world start first.** They do not drive the robot back to A_START.

### 3. Run one condition, preferably with the logger

In a second terminal at the repository root:

```bash
bash scripts/run_trial_with_logger.sh P01 1 SDE
```

Arguments are participant ID, trial number, condition, and optional attempt number. Use the actual planned condition order; the script does not randomize or select it.

| Code | Motion | Display |
|---|---|---|
| SYN | Straight yield | Neutral |
| SYE | Straight yield | Explicit |
| SDN | Side yield | Neutral |
| SDE | Side yield | Explicit |
| CN | Claim | Neutral |
| CE | Claim | Explicit |

Retry the same trial with a new attempt number:

```bash
bash scripts/run_trial_with_logger.sh P01 1 SDE 2
```

For an engineering run without data logging:

```bash
bash scripts/run_trial.sh SDE
```

**These trial commands arm the controller and can move the robot.** Run one trial at a time. Keep the support terminal running. Stop the trial using Ctrl+C and verify the robot has stopped before stopping the support system or repositioning the robot. These packaging changes are not a hardware safety validation.

### 4. Find the logged files

The default output directory is `experiment_data/` inside the repository. Override it with `TB3_DATA_ROOT` if needed. For example:

```text
experiment_data/P01/P01_YYYYMMDD/trial_01_SDE_attempt_01/
    metadata.json
    events.csv
```

The bundled logger writes compact events and metadata; it does not write continuous `telemetry.csv`. Existing attempt directories are refused, not overwritten. `map_waypoint_control/run_trial_with_logger.sh` forwards to the maintained script under `scripts/`.

For the existing optional video synchronization workflow, follow the logger launcher's printed instructions and publish `/experiment_sync_marker` from another sourced terminal. Do not treat manual clap/message timing as frame-exact synchronization.

## Troubleshooting

| Symptom | Action |
|---|---|
| Missing waypoint YAML | Recover the original calibrated file; set `TB3_WAYPOINT_YAML` or place it under `config/`. |
| Localization package/launch missing | Build the bundled localization package and set TB3_WS_SETUP to its overlay setup file. |
| `/pedestrian_zone_trigger` missing | Check the base-system terminal; the zone node needs detector candidate points and valid TF. |
| No `/amcl_pose` or incorrect map alignment | Check bringup, ROS domain, network, lifecycle state and the calibrated initial pose. |
| Display cannot load | Use the IP of the computer running the bridge; check port 8080 and network reachability. |
| Display loads but state does not change | Check `/ehmi_state`, `/ehmi_countdown` and ROS discovery; opening the HTML alone is not a live bridge test. |
| Logger refuses an existing directory | Supply the next attempt number; retain earlier data. |
| Another `/cmd_vel` publisher | Stop the competing controller/teleoperation node before retrying. |

## Validation and experimental interpretation

Packaging validation covers Python/Bash syntax, repository-relative paths, the HTTP handler and dependency failure paths. ROS graph integration, AMCL launch, signal handling on the deployed system, physical trajectories and all six end-to-end conditions require testing on the robot with the recovered configuration.

The controller's motion logic and numeric defaults, detector geometry, display content and logger data format were retained. The controller file has only a configurable waypoint-path change. Do not assume older project notes describe this snapshot's exact speeds, distances or timing. Preserve the actual experimental code before later engineering changes, and cite a verified commit/release in the dissertation.
