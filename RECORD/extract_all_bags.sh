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

    # Check if the extracted directory already exists to prevent re-processing
    out_dir="${dir}_extracted"
    if [ -d "$out_dir" ]; then
        echo "Skipping ${dir} (already extracted: ${out_dir})"
        continue
    fi

    # Check if metadata.yaml exists, if not, try to repair the broken bag (e.g., from sudden power loss)
    if [ ! -f "$dir/metadata.yaml" ]; then
        # Check if there's at least one .db3 file inside to reindex
        if ls "$dir"/*.db3 >/dev/null 2>&1; then
            echo "Warning: Missing metadata.yaml in $dir. Attempting to reindex (rebuild metadata)..."
            ros2 bag reindex "$dir"
        fi
    fi

    # Check again if metadata.yaml exists safely identify it as a valid rosbag folder
    if [ -f "$dir/metadata.yaml" ]; then
        echo "--------------------------------------------------------"
        echo "Extracting bag: $dir"
        python3 "$EXTRACT_SCRIPT" "$dir"
    else
        echo "Skipping $dir: Not a valid rosbag or failed to repair."
    fi
done

echo "--------------------------------------------------------"
echo "Batch extraction finished!"
