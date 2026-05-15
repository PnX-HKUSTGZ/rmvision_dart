#!/bin/bash
set -e

: "${WORKSPACE_DIR:=/home/pnx/pnx/rmvision_dart}"

exec "$WORKSPACE_DIR/dart.sh"
