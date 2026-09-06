#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/runtime_env.sh" || exit 1

INITIAL_POSE="$TB3_REPO_ROOT/map_waypoint_control/set_bottleneck_initial_pose.py"
CONTROLLER="$TB3_REPO_ROOT/map_waypoint_control/map_waypoint_controller.py"
LOGGER="$TB3_REPO_ROOT/map_waypoint_control/experiment_data_logger.py"
DATA_ROOT="${TB3_DATA_ROOT:-$TB3_REPO_ROOT/experiment_data}"

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 SYN|SYE|SDN|SDE|CN|CE"
    exit 1
fi

CONDITION="$(echo "$1" | tr '[:lower:]' '[:upper:]')"

case "$CONDITION" in
    SYN|SYE|SDN|SDE|CN|CE) ;;
    *)
        echo "[ERROR] Unknown formal condition: $CONDITION"
        echo "Allowed: SYN SYE SDN SDE CN CE"
        exit 1
        ;;
esac

if [[ ! -f "$INITIAL_POSE" ]]; then
    echo "[ERROR] Initial-pose script not found:"
    echo "  $INITIAL_POSE"
    exit 1
fi

if [[ ! -f "$CONTROLLER" ]]; then
    echo "[ERROR] Controller not found:"
    echo "  $CONTROLLER"
    exit 1
fi

python3 "$SCRIPT_DIR/check_runtime.py" --files-only || exit 1

if pgrep -af "map_waypoint_controller.py" >/dev/null 2>&1; then
    echo "[ERROR] map_waypoint_controller.py already appears to be running:"
    pgrep -af "map_waypoint_controller.py" || true
    echo "Stop the old controller with Ctrl+C first."
    exit 1
fi

echo "============================================================"
echo " FORMAL TRIAL: $CONDITION"
echo "============================================================"

echo "[1/5] Checking ROS topics..."
required_topics=(/scan /odom /amcl_pose /pedestrian_detected /pedestrian_zone_trigger)
available_topics="$(ros2 topic list 2>/dev/null || true)"

missing=0
for topic in "${required_topics[@]}"; do
    if grep -qx "$topic" <<<"$available_topics"; then
        echo "  [OK] $topic"
    else
        echo "  [MISSING] $topic"
        missing=1
    fi
done

if [[ "$missing" -ne 0 ]]; then
    echo "[ERROR] Missing required ROS topic(s)."
    exit 1
fi

echo "[2/5] Publishing A_START initial pose..."
python3 "$INITIAL_POSE"

echo "Waiting 2 s for AMCL..."
sleep 2

echo "[3/5] Checking /amcl_pose..."
if timeout 5 ros2 topic echo /amcl_pose --once >/tmp/tb3_amcl_once.txt 2>/dev/null; then
    echo "  [OK] /amcl_pose received."
else
    echo "[ERROR] No /amcl_pose message received within 5 s."
    exit 1
fi

echo "[4/5] Checking /cmd_vel publishers..."
cmd_info="$(ros2 topic info /cmd_vel -v 2>/dev/null || true)"
publisher_count="$(awk -F': ' '/Publisher count:/ {print $2; exit}' <<<"$cmd_info")"

if [[ -n "$publisher_count" && "$publisher_count" != "0" ]]; then
    echo "[ERROR] /cmd_vel already has $publisher_count publisher(s)."
    echo "$cmd_info"
    exit 1
fi

if [[ -z "$publisher_count" ]]; then
    echo "[WARN] Could not parse publisher count; showing info:"
    echo "$cmd_info"
    read -r -p "Type YES to continue: " answer
    if [[ "$answer" != "YES" ]]; then
        echo "Cancelled."
        exit 1
    fi
else
    echo "  [OK] /cmd_vel Publisher count: 0"
fi

echo "[5/5] Starting controller..."
echo "Condition: $CONDITION"
echo "Ctrl+C will stop the controller safely."
echo

exec python3 "$CONTROLLER" \
    --ros-args \
    -p condition:="$CONDITION" \
    -p trial_mode:=true \
    -p armed:=true
