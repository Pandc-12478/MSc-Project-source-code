# TurtleBot3 eHMI Bottleneck Study — Source Code

Source code and configuration for an MSc research project investigating pedestrian responses to a TurtleBot3 Waffle in narrow-path encounters. The study compared three robot movement strategies with neutral and explicit eHMI communication, producing six experimental conditions.

## Project overview

The system combines:

- ROS 2 Humble and TurtleBot3 Waffle;
- map-based localisation using Nav2 AMCL;
- laser-based pedestrian detection;
- waypoint and motion control for six experimental conditions;
- a browser-based external Human–Machine Interface (eHMI);
- trial-level event and metadata logging.

The repository contains the experiment controller, detector, eHMI, localisation configuration, map, waypoints, launch scripts and data logger. Participant data, videos and interview material are not included.

## Experimental conditions

The study used a 3 × 2 within-participant design:

| Code | Robot movement strategy | eHMI condition |
|---|---|---|
| `SYN` | Straight yield | Neutral |
| `SYE` | Straight yield | Explicit |
| `SDN` | Side yield | Neutral |
| `SDE` | Side yield | Explicit |
| `CN` | Claim priority | Neutral |
| `CE` | Claim priority | Explicit |

The trial launcher requires the intended condition code as an argument. It does not randomise or determine the participant's condition order.

## Repository structure

```text
MSc-Project-source-code/
├── config/                     # Map, initial-pose record and waypoint configuration
├── ehmi_web/                   # Browser display and ROS-to-HTTP bridge
├── map_waypoint_control/       # Motion controller, initial-pose node and logger
├── pedestrian_detection/       # Laser clustering and bottleneck-zone detection
├── ros_packages/
│   └── turtlebot3_hmi_social/  # Localisation-only ROS 2 package
├── scripts/                    # Runtime checks and launch scripts
└── README.md
```

The package name `turtlebot3_hmi_social` is retained for compatibility with the experiment workspace. In this repository it contains only the localisation launch and configuration; it does not contain a social-navigation policy or Nav2 motion controller.

## Requirements

- Ubuntu with ROS 2 Humble
- TurtleBot3 Waffle with working bringup, laser scan, odometry and TF
- Nav2 localisation packages, including AMCL, map server and lifecycle manager
- Python 3 and PyYAML
- ROS Python dependencies used by the nodes: `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `std_msgs` and `tf2_ros`
- A control computer and display device connected to the appropriate network

All ROS devices must use the same `ROS_DOMAIN_ID`. The supplied runtime configuration defaults to `30`.

## Build the localisation package

From the repository root:

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --base-paths ros_packages \
  --build-base .localization_build \
  --install-base .localization_install

export TB3_WS_SETUP="$PWD/.localization_install/setup.bash"
```

Run the `export TB3_WS_SETUP=...` command again in each new terminal, using an absolute path when necessary.

## Running the system

The commands below must be run from the repository root. Start the verified TurtleBot3 bringup on the robot separately.

### 1. Check the configuration

```bash
source scripts/runtime_env.sh
python3 scripts/check_runtime.py
```

A non-ROS computer can check the presence and structure of the bundled files without starting ROS:

```bash
python3 scripts/check_runtime.py --files-only
```

These checks do not publish ROS messages or command robot motion.

### 2. Start the support nodes

```bash
bash scripts/start_system.sh
```

Keep this terminal open. The script starts:

- the localisation launch;
- laser-based pedestrian detection;
- bottleneck-zone detection;
- the eHMI web bridge.

It does not start the motion controller. Open the following address on the eHMI display device:

```text
http://<control-computer-IP>:8080/
```

Before running a trial, verify localisation, TF alignment and live topic data. Also confirm that no other controller or teleoperation node is publishing to `/cmd_vel`.

Useful checks include:

```bash
ros2 topic list
ros2 topic info /cmd_vel -v
ros2 topic echo /pedestrian_zone_trigger --once
```

### 3. Run a trial with data logging

Open a second terminal at the repository root and run:

```bash
bash scripts/run_trial_with_logger.sh P01 1 SDE
```

Arguments are:

```text
PARTICIPANT_ID  TRIAL_NUMBER  CONDITION  [ATTEMPT]
```

For example, to record a second attempt of the same trial:

```bash
bash scripts/run_trial_with_logger.sh P01 1 SDE 2
```

For an engineering test without the trial logger:

```bash
bash scripts/run_trial.sh SDE
```

The trial scripts publish the saved `A_START` initial pose but do not physically return the robot to its starting position. Place and orient the robot at the calibrated start before each run.

## Logged data

By default, trial data are written under:

```text
experiment_data/P01/P01_YYYYMMDD/trial_01_SDE_attempt_01/
├── metadata.json
└── events.csv
```

The logger records compact trial events and metadata. Existing attempt directories are not overwritten. The output root can be changed by setting `TB3_DATA_ROOT` before sourcing the runtime environment.

The `experiment_data/` directory is excluded from Git so that study records are not uploaded accidentally.

## Optional configuration overrides

The runtime scripts use repository-relative paths. The following environment variables may be set before sourcing `scripts/runtime_env.sh`:

- `ROS_SETUP`
- `TB3_WS_SETUP`
- `ROS_DOMAIN_ID`
- `TB3_WAYPOINT_YAML`
- `TB3_LOCALIZATION_PKG`
- `TB3_LOCALIZATION_LAUNCH`
- `TB3_EHMI_HTML`
- `TB3_DATA_ROOT`

Example:

```bash
export TB3_WS_SETUP=/absolute/path/to/install/setup.bash
export TB3_WAYPOINT_YAML=/absolute/path/to/bottleneck_v3_waypoints.yaml
source scripts/runtime_env.sh
```

The supplied map, initial pose and waypoint coordinates correspond to the experiment configuration and should be kept together. Recalibrate them before using the software in a different physical layout.

## Troubleshooting

| Problem | Check |
|---|---|
| Missing waypoint YAML | Confirm `config/bottleneck_v3_waypoints.yaml` exists or set `TB3_WAYPOINT_YAML`. |
| Localisation package not found | Build the bundled ROS package and set `TB3_WS_SETUP` to its install overlay. |
| No `/amcl_pose` | Check ROS discovery, Nav2 lifecycle state, TF and map alignment. |
| No `/pedestrian_zone_trigger` | Check the detector nodes, laser input and required TF transforms. |
| eHMI page does not load | Use the IP address of the computer running the bridge and check port `8080`. |
| eHMI page loads but does not update | Check `/ehmi_state`, `/ehmi_countdown` and ROS discovery. |
| Trial logger refuses a directory | Use the next attempt number; existing trial data are intentionally preserved. |
| `/cmd_vel` already has a publisher | Stop the competing controller or teleoperation node before starting a trial. |

## Safety and validation

The trial commands can move the robot. Use the original experimental safety procedures, maintain the required clearance and keep the emergency stop available. Run only one motion controller at a time.

The repository provides configuration and dependency checks, but successful static checks do not replace physical validation. Before reuse, verify the map alignment, initial pose, waypoint geometry, laser detection and all six conditions on the intended robot and site.

## Academic use

This repository accompanies the MSc dissertation project *From Prediction to Behavioural Reliance: Explicit Next-Move Communication in Pedestrian–Robot Bottleneck Encounters* by Wang Zaipeng.

When referring to the implementation, cite the repository URL together with the specific commit used for the submitted dissertation.
