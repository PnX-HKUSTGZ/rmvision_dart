#!/bin/bash

startup_common_return_or_exit() {
    local status="${1:-0}"
    if [ "${BASH_SOURCE[0]}" != "$0" ]; then
        return "$status"
    fi
    exit "$status"
}

if [ "${PNX_STARTUP_COMMON_LOADED:-0}" = "1" ]; then
    startup_common_return_or_exit 0
fi
export PNX_STARTUP_COMMON_LOADED=1

: "${TARGET_USER:=pnx}"
: "${TARGET_GROUP:=$TARGET_USER}"
: "${TARGET_HOME:=/home/$TARGET_USER}"
: "${LOCK_FILE:=/tmp/rmvision-dart-startup.lock}"
: "${DEFAULT_ROSBAG_BASE_DIR:=$TARGET_HOME/rosbag/rmvision_dart}"

# 兼容误配成 system-level service 的情况：
# 如果脚本被 root 拉起，就先修正关键输出目录归属，再降权到目标用户继续运行。
if [ "$(id -u)" -eq 0 ] && [ "${PNX_STARTUP_DEMOTED:-0}" != "1" ]; then
    mkdir -p "$TARGET_HOME/.ros/log" "$DEFAULT_ROSBAG_BASE_DIR"
    chown -R "$TARGET_USER:$TARGET_GROUP" "$TARGET_HOME/.ros/log" "$DEFAULT_ROSBAG_BASE_DIR"

    exec /usr/sbin/runuser -u "$TARGET_USER" -- env \
        HOME="$TARGET_HOME" \
        USER="$TARGET_USER" \
        LOGNAME="$TARGET_USER" \
        PNX_STARTUP_DEMOTED=1 \
        /bin/bash "$0" "$@"
fi

# 允许 system service 和 user service 同时存在，但只保留一个活跃实例。
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "rmvision_dart startup is already running; exiting duplicate invocation."
    startup_common_return_or_exit 75
fi

# 确保HOME环境变量正确设置
export HOME="$TARGET_HOME"
# 设置ROS日志目录
export ROS_LOG_DIR="$HOME/.ros/log"

# rosbag 统一保存目录，可通过外部环境变量覆盖。
export ROSBAG_BASE_DIR="$DEFAULT_ROSBAG_BASE_DIR"

# rosbag 总占用上限，单位为 GB。
# 当 `rosbag` 目录总大小超过这个上限时，脚本会自动删除最旧的 bag 目录。
# 如果后续你希望保留更多或更少历史数据，只需要改这个数字。
: "${ROSBAG_MAX_SIZE_GB:=50}"

# 后台清理任务的检查周期，单位为秒。
# 这里默认每 5 分钟检查一次目录总大小。
: "${ROSBAG_CLEANUP_INTERVAL_SEC:=300}"

# 为本次启动生成独立的时间戳目录，避免多次开机/重启时录包互相覆盖。
: "${RUN_TIMESTAMP:=$(date +"%Y%m%d_%H%M%S")}"
export ROSBAG_OUTPUT_DIR="$ROSBAG_BASE_DIR/rmvision_dart_$RUN_TIMESTAMP"

# 转换为字节数，后续方便与 `du -sb` 的结果直接比较。
ROSBAG_MAX_SIZE_BYTES=$((ROSBAG_MAX_SIZE_GB * 1024 * 1024 * 1024))

# 创建日志目录和 rosbag 输出目录（如果不存在）
mkdir -p "$ROS_LOG_DIR" "$ROSBAG_BASE_DIR"

# 获取 rosbag 根目录当前总大小，单位为字节。
get_rosbag_dir_size_bytes() {
    if [ ! -d "$ROSBAG_BASE_DIR" ]; then
        echo 0
        return
    fi
    du -sb "$ROSBAG_BASE_DIR" 2>/dev/null | awk '{print $1}'
}

# 当 rosbag 总占用超过上限时，循环删除最旧的历史 bag 目录。
# 注意：当前正在写入的目录不会被删除，避免损坏本次录包。
cleanup_old_rosbags_once() {
    local current_size
    local oldest_dir

    current_size=$(get_rosbag_dir_size_bytes)
    [ -n "$current_size" ] || current_size=0

    while [ "$current_size" -gt "$ROSBAG_MAX_SIZE_BYTES" ]; do
        # 排除当前正在录制的目录
        oldest_dir=$(find "$ROSBAG_BASE_DIR" -mindepth 1 -maxdepth 1 -type d ! -path "$ROSBAG_OUTPUT_DIR" \
            -printf '%T@ %p\n' 2>/dev/null | sort -n | head -n 1 | cut -d' ' -f2-)

        # 如果已经没有可删的历史目录，就停止清理并打印告警。
        if [ -z "$oldest_dir" ]; then
            echo "Warning: rosbag size exceeded ${ROSBAG_MAX_SIZE_GB}GB, but no old bag can be deleted."
            break
        fi

        echo "rosbag size exceeded ${ROSBAG_MAX_SIZE_GB}GB, deleting oldest bag: $oldest_dir"
        rm -rf -- "$oldest_dir"

        current_size=$(get_rosbag_dir_size_bytes)
        [ -n "$current_size" ] || current_size=0
    done
}

# 后台循环检查 rosbag 目录总大小。
monitor_rosbag_disk_usage() {
    while true; do
        cleanup_old_rosbags_once
        sleep "$ROSBAG_CLEANUP_INTERVAL_SEC"
    done
}
