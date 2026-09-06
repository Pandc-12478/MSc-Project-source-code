#!/usr/bin/env bash
# Sourced by launchers. ROS setup files may reference unset variables.
set +u
TB3_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${TB3_WS_SETUP:-$HOME/turtlebot3_ws/install/setup.bash}"
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
    return 1
fi
source "$ROS_SETUP" || return 1
if [[ -f "$WS_SETUP" ]]; then
    source "$WS_SETUP" || return 1
elif [[ -n "${TB3_WS_SETUP:-}" ]]; then
    echo "[ERROR] Explicit workspace setup not found: $WS_SETUP" >&2
    return 1
fi
export TB3_REPO_ROOT
export TB3_WAYPOINT_YAML="${TB3_WAYPOINT_YAML:-$TB3_REPO_ROOT/config/bottleneck_v3_waypoints.yaml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
