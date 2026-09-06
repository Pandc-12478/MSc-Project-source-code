#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/runtime_env.sh"
python3 "$SCRIPT_DIR/check_runtime.py"
LOCALIZATION_PKG="${TB3_LOCALIZATION_PKG:-turtlebot3_hmi_social}"
LOCALIZATION_LAUNCH="${TB3_LOCALIZATION_LAUNCH:-bottleneck_v3_localization.launch.py}"
MAP_FILE="$TB3_REPO_ROOT/config/bottleneck_map_v3.yaml"

for node in scan_cluster_detector.py pedestrian_zone_trigger.py ehmi_web_bridge.py; do
    if pgrep -f "$node" >/dev/null 2>&1; then
        echo "[ERROR] $node is already running. Stop the old base system first."
        exit 1
    fi
done

# These are support nodes only. Trials run separately in another terminal.
pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 launch "$LOCALIZATION_PKG" "$LOCALIZATION_LAUNCH" map:="$MAP_FILE" use_sim_time:=false &
pids+=("$!")
python3 "$TB3_REPO_ROOT/pedestrian_detection/scan_cluster_detector.py" &
pids+=("$!")
python3 "$TB3_REPO_ROOT/pedestrian_detection/pedestrian_zone_trigger.py" &
pids+=("$!")
python3 "$TB3_REPO_ROOT/ehmi_web/ehmi_web_bridge.py" &
pids+=("$!")
echo "Support processes started. Keep this terminal open."
echo "Open http://<this-computer-IP>:8080/ on the display device."
echo "Verify localization, TF and live topics before starting a trial."
echo "Stop the trial FIRST, then Ctrl+C here to stop support nodes."
# A failed support process must not leave an apparently healthy launcher.
set +e
wait -n "${pids[@]}"
rc=$?
echo "[ERROR] A support process exited (status $rc); stopping support processes."
exit 1
