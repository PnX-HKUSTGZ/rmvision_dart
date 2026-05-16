#!/bin/bash
set -e

: "${WORKSPACE_DIR:=/home/pnx/pnx/rmvision_dart}"
: "${ROSBAG_BASE_DIR:=/home/pnx/rosbag/rmvision_dart}"

source /opt/ros/humble/setup.bash
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# Directory containing rosbags.
BAG_DIR="${1:-$ROSBAG_BASE_DIR}"

if [ ! -d "$BAG_DIR" ]; then
    echo "Error: Directory $BAG_DIR does not exist."
    exit 1
fi

# Path to the python extraction script.
EXTRACT_SCRIPT="$WORKSPACE_DIR/RECORD/extract_bag.py"

if [ ! -f "$EXTRACT_SCRIPT" ]; then
    echo "Error: Python script $EXTRACT_SCRIPT not found."
    exit 1
fi

echo "Scanning directory: $BAG_DIR for rosbag folders..."

shopt -s nullglob

repair_bag_metadata_if_needed() {
    local dir="$1"
    local backup_path

    if ros2 bag info "$dir" >/dev/null 2>&1; then
        return 0
    fi

    if ! ls "$dir"/*.db3 >/dev/null 2>&1; then
        echo "Skipping $dir: metadata is invalid and no .db3 file was found."
        return 1
    fi

    echo "Warning: metadata.yaml in $dir is missing or invalid. Attempting to reindex..."
    if [ -f "$dir/metadata.yaml" ]; then
        backup_path="$dir/metadata.yaml.bad.$(date +%Y%m%d_%H%M%S)"
        mv "$dir/metadata.yaml" "$backup_path"
        echo "Backed up invalid metadata to $backup_path"
    fi

    ros2 bag reindex "$dir"

    if ros2 bag info "$dir" >/dev/null 2>&1; then
        return 0
    fi

    echo "Skipping $dir: failed to repair metadata."
    return 1
}

is_extraction_complete() {
    local out_dir="$1"

    [ -s "$out_dir/rosout.txt" ] || return 1

    python3 - "$out_dir" <<'PY'
import sys
from pathlib import Path

import cv2

out_dir = Path(sys.argv[1])
role_pairs = [
    (out_dir / "video_base_raw.mp4", out_dir / "video_base_annotated.mp4"),
    (out_dir / "video_outpost_raw.mp4", out_dir / "video_outpost_annotated.mp4"),
]

complete_roles = 0
for videos in role_pairs:
    if not all(video.is_file() and video.stat().st_size > 0 for video in videos):
        continue
    for video in videos:
        data = video.read_bytes()
        if b"moov" not in data:
            sys.exit(1)
        cap = cv2.VideoCapture(str(video))
        ok = cap.isOpened() and cap.get(cv2.CAP_PROP_FRAME_COUNT) > 0
        cap.release()
        if not ok:
            sys.exit(1)
    complete_roles += 1

if complete_roles < 1:
    sys.exit(1)
sys.exit(0)
PY
}

# Find directories inside BAG_DIR. We only want first-level subdirectories.
for dir in "$BAG_DIR"/*/; do
    # Skip if not a directory
    [ -d "$dir" ] || continue
    
    # Remove trailing slash
    dir="${dir%/}"
    
    # Skip directories that are already extracted outputs (ending with _extracted)
    if [[ "$dir" == *_extracted ]]; then
        continue
    fi

    # Skip only when all expected extracted outputs exist.
    out_dir="${dir}_extracted"
    if [ -d "$out_dir" ] && is_extraction_complete "$out_dir"; then
        echo "Skipping ${dir} (already extracted: ${out_dir})"
        continue
    fi
    if [ -d "$out_dir" ]; then
        echo "Re-extracting ${dir} because ${out_dir} is incomplete."
    fi

    if ! repair_bag_metadata_if_needed "$dir"; then
        continue
    fi

    echo "--------------------------------------------------------"
    echo "Extracting bag: $dir"
    if ! python3 "$EXTRACT_SCRIPT" "$dir"; then
        echo "Warning: failed to extract $dir; continuing with next bag."
    fi
done

echo "--------------------------------------------------------"
echo "Batch extraction finished!"
