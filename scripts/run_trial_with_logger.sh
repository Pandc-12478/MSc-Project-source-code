#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/runtime_env.sh" || exit 1

INITIAL_POSE="$TB3_REPO_ROOT/map_waypoint_control/set_bottleneck_initial_pose.py"
CONTROLLER="$TB3_REPO_ROOT/map_waypoint_control/map_waypoint_controller.py"
LOGGER="$TB3_REPO_ROOT/map_waypoint_control/experiment_data_logger.py"
DATA_ROOT="${TB3_DATA_ROOT:-$TB3_REPO_ROOT/experiment_data}"

if [[ $# -lt 3 || $# -gt 4 ]]; then
    echo "Usage: $0 PARTICIPANT_ID TRIAL_NUMBER CONDITION [ATTEMPT]"
    echo "Example: $0 P01 1 SDE"
    echo "Retry:   $0 P01 1 SDE 2"
    exit 1
fi

PARTICIPANT_ID="$(echo "$1" | tr '[:lower:]' '[:upper:]')"
TRIAL_NUMBER="$2"
CONDITION="$(echo "$3" | tr '[:lower:]' '[:upper:]')"
ATTEMPT="${4:-1}"
SESSION_ID="${PARTICIPANT_ID}_$(date +%Y%m%d)"

if [[ ! "$PARTICIPANT_ID" =~ ^P[0-9]+$ ]]; then
    echo "[ERROR] PARTICIPANT_ID must be pseudonymous, e.g. P01"
    exit 1
fi

if [[ ! "$TRIAL_NUMBER" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] TRIAL_NUMBER must be a positive integer."
    exit 1
fi

if [[ ! "$ATTEMPT" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] ATTEMPT must be a positive integer."
    exit 1
fi

case "$CONDITION" in
    SYN|SYE|SDN|SDE|CN|CE) ;;
    *)
        echo "[ERROR] Unknown formal condition: $CONDITION"
        echo "Allowed: SYN SYE SDN SDE CN CE"
        exit 1
        ;;
esac

for f in "$INITIAL_POSE" "$CONTROLLER" "$LOGGER"; do
    if [[ ! -f "$f" ]]; then
        echo "[ERROR] Required file not found: $f"
        exit 1
    fi
done

python3 "$SCRIPT_DIR/check_runtime.py" --files-only || exit 1

if pgrep -af "map_waypoint_controller.py" >/dev/null 2>&1; then
    echo "[ERROR] map_waypoint_controller.py already appears to be running:"
    pgrep -af "map_waypoint_controller.py" || true
    exit 1
fi

if pgrep -af "experiment_data_logger.py" >/dev/null 2>&1; then
    echo "[ERROR] experiment_data_logger.py already appears to be running:"
    pgrep -af "experiment_data_logger.py" || true
    exit 1
fi

echo "============================================================"
echo " FORMAL TRIAL WITH LOGGER"
echo " Participant : $PARTICIPANT_ID"
echo " Trial       : $TRIAL_NUMBER"
echo " Condition   : $CONDITION"
echo " Attempt     : $ATTEMPT"
echo " Session     : $SESSION_ID"
echo "============================================================"

required_topics=(
    /scan
    /odom
    /amcl_pose
    /pedestrian_detected
    /pedestrian_distance
    /pedestrian_zone_trigger
)
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

echo "[1/5] Publishing A_START initial pose..."
python3 "$INITIAL_POSE" || exit 1
sleep 2

echo "[2/5] Checking /amcl_pose..."
if timeout 5 ros2 topic echo /amcl_pose --once >/tmp/tb3_amcl_once.txt 2>/dev/null; then
    echo "  [OK] /amcl_pose received."
else
    echo "[ERROR] No /amcl_pose message received within 5 s."
    exit 1
fi

echo "[3/5] Checking /cmd_vel publishers..."
cmd_info="$(ros2 topic info /cmd_vel -v 2>/dev/null || true)"
publisher_count="$(awk -F': ' '/Publisher count:/ {print $2; exit}' <<<"$cmd_info")"

if [[ -n "$publisher_count" && "$publisher_count" != "0" ]]; then
    echo "[ERROR] /cmd_vel already has $publisher_count publisher(s)."
    echo "$cmd_info"
    exit 1
fi

if [[ -z "$publisher_count" ]]; then
    echo "[ERROR] Cannot determine /cmd_vel publisher count. Check ROS discovery."
    exit 1
fi
echo "  [OK] /cmd_vel publisher count: $publisher_count"

echo "[4/5] Starting experiment logger..."
mkdir -p "$DATA_ROOT"
python3 "$LOGGER" \
    --ros-args \
    -p participant_id:="$PARTICIPANT_ID" \
    -p session_id:="$SESSION_ID" \
    -p trial_number:="$TRIAL_NUMBER" \
    -p attempt_number:="$ATTEMPT" \
    -p condition:="$CONDITION" \
    -p output_root:="$DATA_ROOT" \
    -p telemetry_rate_hz:=10.0 &
LOGGER_PID=$!

sleep 1
if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
    echo "[ERROR] Logger failed to stay running."
    wait "$LOGGER_PID" || true
    exit 1
fi

echo "  [OK] Logger PID: $LOGGER_PID"
echo "  Data root: $DATA_ROOT"
echo
echo "VIDEO SYNC FOR THIS TRIAL:"
echo "  1) In front of the phone camera, clap once."
echo "  2) Immediately run in another sourced terminal:"
echo "     ros2 topic pub --once /experiment_sync_marker std_msgs/msg/Empty '{}'"
echo

echo "[5/5] Starting controller..."
echo "Ctrl+C will stop the controller; logger will then close/flush."
echo

python3 "$CONTROLLER" \
    --ros-args \
    -p condition:="$CONDITION" \
    -p trial_mode:=true \
    -p armed:=true
CONTROLLER_RC=$?

# Give the logger a moment to receive the final controller state if the
# controller exited normally after DONE.
sleep 0.5

if kill -0 "$LOGGER_PID" 2>/dev/null; then
    kill -INT "$LOGGER_PID" 2>/dev/null || true
fi
wait "$LOGGER_PID" 2>/dev/null || true

echo
echo "Trial process finished. Controller exit code: $CONTROLLER_RC"
echo "Saved under: $DATA_ROOT/$PARTICIPANT_ID/$SESSION_ID/"

exit "$CONTROLLER_RC"
