#!/bin/bash
set -e

: "${WORKSPACE_DIR:=/home/pnx/pnx/rmvision_dart}"
: "${LAUNCH_PARAMS_FILE:=$WORKSPACE_DIR/src/vision_bringup/rm_vision_bringup/config/launch_params.yaml}"
: "${RECORD_START_DELAY_SEC:=15}"
: "${ROSBAG_CACHE_SIZE_BYTES:=52428800}"
: "${ROSBAG_CLOUD_HZ:=1.0}"
: "${ROSBAG_ZSTD_COMPRESS_COMPLETED:=true}"
: "${ROSBAG_ZSTD_START_DELAY_SEC:=180}"
: "${ROSBAG_ZSTD_INTERVAL_SEC:=600}"
: "${ROSBAG_ZSTD_MIN_AGE_SEC:=120}"
: "${ROSBAG_ZSTD_QUEUE_SIZE:=1}"
: "${ROSBAG_ZSTD_THREADS:=1}"

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

get_launch_string() {
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
    value = data.get(key, default_value)
except Exception:
    value = default_value
print(str(value))
' "$LAUNCH_PARAMS_FILE" "$key" "$default_value" 2>/dev/null) || value="$default_value"

    echo "$value"
}

if [ -z "${ENABLE_ROSBAG_RECORDING+x}" ]; then
    ENABLE_ROSBAG_RECORDING=$(get_launch_bool "enable_rosbag_recorder" "true")
fi
case "$ENABLE_ROSBAG_RECORDING" in
    true|True|TRUE|1|yes|Yes|YES|on|On|ON) ENABLE_ROSBAG_RECORDING="true" ;;
    *) ENABLE_ROSBAG_RECORDING="false" ;;
esac

if [ -z "${ROSBAG_RECORD_MODE+x}" ]; then
    ROSBAG_RECORD_MODE=$(get_launch_string "rosbag_record_mode" "full")
fi
case "$ROSBAG_RECORD_MODE" in
    full|active|base_only|outpost_only) ;;
    *)
        echo "invalid rosbag_record_mode '$ROSBAG_RECORD_MODE', fallback to full"
        ROSBAG_RECORD_MODE="full"
        ;;
esac

source /opt/ros/humble/setup.bash
source install/setup.bash
source "$WORKSPACE_DIR/RECORD/clean_space.sh"

recorder_pid=""
cleanup_pid=""
compression_pid=""

stop_background_jobs() {
    if [ -n "$recorder_pid" ] && kill -0 "$recorder_pid" 2>/dev/null; then
        kill "$recorder_pid" 2>/dev/null || true
        wait "$recorder_pid" 2>/dev/null || true
    fi
    if [ -n "$cleanup_pid" ] && kill -0 "$cleanup_pid" 2>/dev/null; then
        kill "$cleanup_pid" 2>/dev/null || true
        wait "$cleanup_pid" 2>/dev/null || true
    fi
    if [ -n "$compression_pid" ] && kill -0 "$compression_pid" 2>/dev/null; then
        kill "$compression_pid" 2>/dev/null || true
        wait "$compression_pid" 2>/dev/null || true
    fi
}

trap stop_background_jobs EXIT INT TERM

case "$ROSBAG_ZSTD_COMPRESS_COMPLETED" in
    true|True|TRUE|1|yes|Yes|YES|on|On|ON) ROSBAG_ZSTD_COMPRESS_COMPLETED="true" ;;
    *) ROSBAG_ZSTD_COMPRESS_COMPLETED="false" ;;
esac

bag_is_zstd_compressed() {
    local bag_dir="$1"
    [ -f "$bag_dir/metadata.yaml" ] || return 1
    grep -Eq 'compression_format: zstd|compression_format: "zstd"|compression_format: '\''zstd'\''' \
        "$bag_dir/metadata.yaml"
}

bag_is_old_enough_for_compression() {
    local bag_dir="$1"
    local now
    local newest

    now=$(date +%s)
    newest=$(find "$bag_dir" -type f -printf '%T@\n' 2>/dev/null | sort -nr | head -n 1 | cut -d. -f1)
    [ -n "$newest" ] || return 1
    [ $((now - newest)) -ge "$ROSBAG_ZSTD_MIN_AGE_SEC" ]
}

compress_one_completed_rosbag() {
    local bag_dir="$1"
    local compressed_dir="${bag_dir}_zstd"
    local tmp_dir="${compressed_dir}.tmp"
    local options_file

    [ "$bag_dir" != "$ROSBAG_OUTPUT_DIR" ] || return 0
    [ -d "$bag_dir" ] || return 0
    case "$bag_dir" in
        *_zstd|*_zstd.tmp|*_extracting|*_extracted|*_replaced) return 0 ;;
    esac
    [ -f "$bag_dir/metadata.yaml" ] || return 0
    bag_is_zstd_compressed "$bag_dir" && return 0
    bag_is_old_enough_for_compression "$bag_dir" || return 0

    if [ -d "$compressed_dir" ] && ros2 bag info "$compressed_dir" >/dev/null 2>&1; then
        echo "zstd compressed bag already exists, deleting uncompressed source: $bag_dir"
        rm -rf -- "$bag_dir"
        return 0
    fi

    if ! ros2 bag info "$bag_dir" >/dev/null 2>&1; then
        echo "bag metadata check failed, trying reindex before zstd: $bag_dir"
        ros2 bag reindex "$bag_dir" >/dev/null 2>&1 || return 0
        ros2 bag info "$bag_dir" >/dev/null 2>&1 || return 0
    fi

    rm -rf -- "$tmp_dir"
    options_file=$(mktemp /tmp/rmvision_zstd_options.XXXXXX.yaml)
    cat > "$options_file" <<EOF
output_bags:
  - uri: $tmp_dir
    storage_id: sqlite3
    all: true
    compression_mode: file
    compression_format: zstd
    compression_queue_size: $ROSBAG_ZSTD_QUEUE_SIZE
    compression_threads: $ROSBAG_ZSTD_THREADS
EOF

    echo "compressing completed rosbag with zstd: $bag_dir -> $compressed_dir"
    if nice -n 19 ionice -c 3 ros2 bag convert -i "$bag_dir" sqlite3 -o "$options_file" &&
       ros2 bag info "$tmp_dir" >/dev/null 2>&1; then
        rm -rf -- "$compressed_dir"
        mv "$tmp_dir" "$compressed_dir"
        rm -rf -- "$bag_dir"
        echo "zstd compression complete: $compressed_dir"
    else
        echo "zstd compression failed, keeping original bag: $bag_dir"
        rm -rf -- "$tmp_dir"
    fi
    rm -f -- "$options_file"
}

compress_completed_rosbags_once() {
    local bag_dir

    [ "$ROSBAG_ZSTD_COMPRESS_COMPLETED" = "true" ] || return 0
    [ -d "$ROSBAG_BASE_DIR" ] || return 0

    while IFS= read -r bag_dir; do
        compress_one_completed_rosbag "$bag_dir"
    done < <(find "$ROSBAG_BASE_DIR" -mindepth 1 -maxdepth 1 -type d \
        ! -path "$ROSBAG_OUTPUT_DIR" -printf '%T@ %p\n' 2>/dev/null | sort -n | cut -d' ' -f2-)
}

monitor_rosbag_zstd_compression() {
    sleep "$ROSBAG_ZSTD_START_DELAY_SEC"
    while true; do
        compress_completed_rosbags_once
        sleep "$ROSBAG_ZSTD_INTERVAL_SEC"
    done
}

start_rosbag_recording() {
    sleep "$RECORD_START_DELAY_SEC"

    echo "ready to record rosbag"
    echo "rosbag output dir: $ROSBAG_OUTPUT_DIR"
    echo "rosbag size limit: ${ROSBAG_MAX_SIZE_GB}GB"
    echo "rosbag cache size: ${ROSBAG_CACHE_SIZE_BYTES} bytes"
    echo "rosbag record mode: $ROSBAG_RECORD_MODE"
    echo "rosbag cloud record hz: $ROSBAG_CLOUD_HZ"
    echo "rosbag zstd completed compression: $ROSBAG_ZSTD_COMPRESS_COMPLETED"

    cleanup_old_rosbags_once

    exec nice -n 19 ionice -c 3 python3 "$WORKSPACE_DIR/RECORD/selective_rosbag_recorder.py" \
        --output "$ROSBAG_OUTPUT_DIR" \
        --mode "$ROSBAG_RECORD_MODE" \
        --cloud-hz "$ROSBAG_CLOUD_HZ"
}

if [ "$ENABLE_ROSBAG_RECORDING" = "true" ]; then
    cleanup_old_rosbags_once
    monitor_rosbag_disk_usage &
    cleanup_pid=$!
    if [ "$ROSBAG_ZSTD_COMPRESS_COMPLETED" = "true" ]; then
        monitor_rosbag_zstd_compression &
        compression_pid=$!
    fi

    start_rosbag_recording &
    recorder_pid=$!
else
    echo "rosbag recording disabled by enable_rosbag_recorder=false"
fi

echo "ready to launch vision"
ros2 launch rm_vision_bringup vision_bringup.launch.py
