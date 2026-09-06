#!/usr/bin/env python3
"""Read-only dependency checks; never starts ROS nodes or publishes motion."""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--files-only', action='store_true', help='Skip ROS imports and localization package checks')
    args = parser.parse_args()
    errors = []
    required = ['ros_packages/turtlebot3_hmi_social/config/bottleneck_v3_localization.yaml', 'ros_packages/turtlebot3_hmi_social/launch/bottleneck_v3_localization.launch.py', 'config/bottleneck_map_v3.yaml', 'config/bottleneck_map_v3.pgm',
                'map_waypoint_control/map_waypoint_controller.py',
                'map_waypoint_control/set_bottleneck_initial_pose.py',
                'map_waypoint_control/experiment_data_logger.py',
                'pedestrian_detection/scan_cluster_detector.py',
                'pedestrian_detection/pedestrian_zone_trigger.py',
                'ehmi_web/ehmi_web_bridge.py']
    for name in required:
        if not (ROOT / name).is_file():
            errors.append('Missing repository file: ' + name)
    html = Path(os.environ.get('TB3_EHMI_HTML', str(ROOT / 'ehmi_web/turtlebot3_ehmi_v4_countdown.html'))).expanduser()
    if not html.is_file():
        errors.append('Missing display HTML: ' + str(html))
    waypoints = Path(os.environ.get('TB3_WAYPOINT_YAML', str(ROOT / 'config/bottleneck_v3_waypoints.yaml'))).expanduser()
    if not waypoints.is_file():
        errors.append('Missing calibrated waypoint file: ' + str(waypoints) + '\n  Recover the original file from the experiment computer; do not invent coordinates.')
    else:
        try:
            import yaml
            data = yaml.safe_load(waypoints.read_text())
            if data.get('validated_for_motion') is not True:
                print('[NOTICE] Waypoint file is not marked validated_for_motion=true; file checks do not certify physical motion.')
            assert data['map_id'] == 'bottleneck_map_v3', 'map_id must be bottleneck_map_v3'
            assert data['frame_id'] == 'map', 'frame_id must be map'
            routes = {
                'APPROACH': ['A_START', 'P1_CONFLICT'],
                'STRAIGHT_YIELD': ['P1_CONFLICT', 'P2_STRAIGHT_YIELD_END'],
                'SIDE_YIELD': ['P1_CONFLICT', 'P3_SIDE_YIELD_STRAIGHT', 'P4_SIDE_YIELD_END'],
                'CLAIM': ['P1_CONFLICT', 'P5_CLAIM_END'],
            }
            for action, points in routes.items():
                if data['routes'][action] != points:
                    raise ValueError('Unexpected route for ' + action)
                for point in points:
                    for axis in ('x', 'y', 'yaw'):
                        if not math.isfinite(float(data['points'][point][axis])):
                            raise ValueError('Non-finite coordinate: ' + point + '/' + axis)
        except Exception as exc:
            errors.append('Waypoint validation failed: ' + str(exc))
    if not args.files_only:
        for module in ('rclpy', 'yaml', 'geometry_msgs', 'sensor_msgs', 'nav_msgs', 'std_msgs', 'tf2_ros'):
            if importlib.util.find_spec(module) is None:
                errors.append('Missing Python/ROS module: ' + module)
        package = os.environ.get('TB3_LOCALIZATION_PKG', 'turtlebot3_hmi_social')
        launch = os.environ.get('TB3_LOCALIZATION_LAUNCH', 'bottleneck_v3_localization.launch.py')
        try:
            result = subprocess.run(['ros2', 'pkg', 'prefix', '--share', package], capture_output=True, text=True, timeout=10)
            if result.returncode or not (Path(result.stdout.strip()) / 'launch' / launch).is_file():
                errors.append('Missing installed localization launch: ' + package + '/launch/' + launch)
        except (OSError, subprocess.TimeoutExpired) as exc:
            errors.append('Cannot check localization package: ' + str(exc))
    for error in errors:
        print('[MISSING/INVALID] ' + error)
    if errors:
        print('Checks failed. No robot commands were sent.')
        return 1
    print('Dependency checks passed. Live localization, geometry and robot operation still require verification.')
    return 0

if __name__ == '__main__':
    sys.exit(main())
