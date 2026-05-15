#!/bin/bash
set -e

: "${WORKSPACE_DIR:=/home/pnx/pnx/rmvision_dart}"
: "${LAUNCH_PARAMS_FILE:=$WORKSPACE_DIR/src/vision_bringup/rm_vision_bringup/config/launch_params.yaml}"
: "${RECORD_START_DELAY_SEC:=15}"
: "${ROSBAG_CACHE_SIZE_BYTES:=52428800}"

cd "$WORKSPACE_DIR"

get_launch_bool() {
    local key="$1"
    local default_value="$2"
    local value

    value=$(python3 -c '
import sys
import yaml

path, key, default_value = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    with open(path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}
    value = data.get(key, default_value.lower() == "true")
except Exception:
    value = default_value.lower() == "true"
print("true" if bool(value) else "false")
' "$LAUNCH_PARAMS_FILE" "$key" "$default_value" 2>/dev/null) || value="$default_value"

    case "$value" in
        true|True|TRUE|1|yes|Yes|YES|on|On|ON) echo "true" ;;
        *) echo "false" ;;
    esac
}

if [ -z "${ENABLE_ROSBAG_RECORDING+x}" ]; then
    ENABLE_ROSBAG_RECORDING=$(get_launch_bool "enable_rosbag_recorder" "true")
fi
case "$ENABLE_ROSBAG_RECORDING" in
    true|True|TRUE|1|yes|Yes|YES|on|On|ON) ENABLE_ROSBAG_RECORDING="true" ;;
    *) ENABLE_ROSBAG_RECORDING="false" ;;
esac

source /opt/ros/humble/setup.bash
source install/setup.bash
source "$WORKSPACE_DIR/RECORD/clean_space.sh"

recorder_pid=""
cleanup_pid=""

stop_background_jobs() {
    if [ -n "$recorder_pid" ] && kill -0 "$recorder_pid" 2>/dev/null; then
        kill "$recorder_pid" 2>/dev/null || true
        wait "$recorder_pid" 2>/dev/null || true
    fi
    if [ -n "$cleanup_pid" ] && kill -0 "$cleanup_pid" 2>/dev/null; then
        kill "$cleanup_pid" 2>/dev/null || true
        wait "$cleanup_pid" 2>/dev/null || true
    fi
}

trap stop_background_jobs EXIT INT TERM

start_rosbag_recording() {
    sleep "$RECORD_START_DELAY_SEC"

    echo "ready to record rosbag"
    echo "rosbag output dir: $ROSBAG_OUTPUT_DIR"
    echo "rosbag size limit: ${ROSBAG_MAX_SIZE_GB}GB"
    echo "rosbag cache size: ${ROSBAG_CACHE_SIZE_BYTES} bytes"

    cleanup_old_rosbags_once

    exec nice -n 19 ionice -c 3 ros2 bag record -o "$ROSBAG_OUTPUT_DIR" \
        --max-cache-size "$ROSBAG_CACHE_SIZE_BYTES" \
        /base/image_raw/compressed \
        /outpost/image_raw/compressed \
        /base/camera_info \
        /outpost/camera_info \
        /base/Send_pnp \
        /outpost/Send_pnp \
        /base/Send_fused \
        /outpost/Send_fused \
        /Send \
        /target_id \
        /current_dart_id \
        /offset \
        /competition_mode \
        /rosout
}

if [ "$ENABLE_ROSBAG_RECORDING" = "true" ]; then
    cleanup_old_rosbags_once
    monitor_rosbag_disk_usage &
    cleanup_pid=$!

    start_rosbag_recording &
    recorder_pid=$!
else
    echo "rosbag recording disabled by enable_rosbag_recorder=false"
fi

echo "ready to launch vision"
ros2 launch rm_vision_bringup vision_bringup.launch.py
