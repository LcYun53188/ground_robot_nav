#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WITH_VENV="$WS_DIR/scripts/with_venv.sh"
DURATION_SEC="${1:-8}"

topic_hz() {
  local topic="$1"
  timeout "$DURATION_SEC" "$WITH_VENV" ros2 topic hz "$topic" 2>/dev/null \
    | awk '/average rate:/ {rate=$3} /min:/ {min=$2; max=$4; std=$6} END {
        if (rate != "") {
          printf "%s average_hz=%s min_dt=%s max_dt=%s stddev=%s\n", topic, rate, min, max, std
        } else {
          printf "%s average_hz=NA\n", topic
        }
      }'
}

echo "# OAK-D / Visual SLAM topic rates (${DURATION_SEC}s samples)"
topic_hz /oakd/left/image_raw
topic_hz /oakd/right/image_raw
topic_hz /oakd/imu/raw
topic_hz /visual_slam/tracking/odometry

echo
echo "# oakd_unified process resource snapshot"
if pgrep -f "oakd_unified_node|oakd_unified_node_py" >/dev/null; then
  ps -o pid,comm,%cpu,%mem,rss,etime,args -p "$(pgrep -f "oakd_unified_node|oakd_unified_node_py" | paste -sd, -)"
else
  echo "oakd_unified process not found"
fi
