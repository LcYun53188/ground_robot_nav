#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WITH_VENV="$WS_DIR/scripts/with_venv.sh"

TOPIC_TIMEOUT_SEC="${TOPIC_TIMEOUT_SEC:-6}"
TF_TIMEOUT_SEC="${TF_TIMEOUT_SEC:-6}"

check_topic_type() {
  local topic="$1"
  local expected_type="$2"
  local actual_type

  actual_type="$("$WITH_VENV" ros2 topic type "$topic" 2>/dev/null || true)"
  if [[ "$actual_type" != "$expected_type" ]]; then
    echo "FAIL topic type: $topic expected $expected_type, got ${actual_type:-<missing>}" >&2
    return 1
  fi

  echo "OK topic type: $topic -> $actual_type"
}

check_topic_hz() {
  local topic="$1"
  local output
  local status

  set +e
  output="$(timeout "$TOPIC_TIMEOUT_SEC" "$WITH_VENV" ros2 topic hz "$topic" 2>&1)"
  status="$?"
  set -e

  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    echo "FAIL topic hz: $topic" >&2
    echo "$output" >&2
    return 1
  fi

  if ! grep -q "average rate" <<<"$output"; then
    echo "FAIL topic hz: $topic did not report an average rate" >&2
    echo "$output" >&2
    return 1
  fi

  echo "OK topic hz: $topic"
  grep "average rate" <<<"$output" | tail -n 1
}

check_topic_once() {
  local topic="$1"
  local output
  local status

  set +e
  output="$(timeout "$TOPIC_TIMEOUT_SEC" "$WITH_VENV" ros2 topic echo --once "$topic" 2>&1)"
  status="$?"
  set -e

  if [[ "$status" -ne 0 ]]; then
    echo "FAIL topic echo: $topic" >&2
    echo "$output" >&2
    return 1
  fi

  echo "OK topic echo: $topic"
}

check_tf() {
  local parent_frame="$1"
  local child_frame="$2"
  local output
  local status

  set +e
  output="$(timeout "$TF_TIMEOUT_SEC" "$WITH_VENV" ros2 run tf2_ros tf2_echo \
    "$parent_frame" "$child_frame" 2>&1)"
  status="$?"
  set -e

  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    echo "FAIL tf: $parent_frame -> $child_frame" >&2
    echo "$output" >&2
    return 1
  fi

  if ! grep -Eq "Transform|Translation|Rotation" <<<"$output"; then
    echo "FAIL tf: $parent_frame -> $child_frame did not produce a transform" >&2
    echo "$output" >&2
    return 1
  fi

  echo "OK tf: $parent_frame -> $child_frame"
}

main() {
  check_topic_type "/oakd/left/image_raw" "sensor_msgs/msg/Image"
  check_topic_type "/oakd/right/image_raw" "sensor_msgs/msg/Image"
  check_topic_type "/oakd/left/camera_info" "sensor_msgs/msg/CameraInfo"
  check_topic_type "/oakd/right/camera_info" "sensor_msgs/msg/CameraInfo"
  check_topic_type "/oakd/imu/raw" "sensor_msgs/msg/Imu"
  check_topic_type "/oakd/depth/image" "sensor_msgs/msg/Image"
  check_topic_type "/oakd/depth/camera_info" "sensor_msgs/msg/CameraInfo"

  check_topic_hz "/oakd/left/image_raw"
  check_topic_hz "/oakd/right/image_raw"
  check_topic_hz "/oakd/imu/raw"
  check_topic_hz "/oakd/depth/image"

  check_topic_once "/oakd/left/camera_info"
  check_topic_once "/oakd/right/camera_info"
  check_topic_once "/oakd/depth/camera_info"

  check_tf "base_link" "oakd_imu_link"
  check_tf "oakd_imu_link" "oakd_camera_optical_frame"
  check_tf "base_link" "oakd_camera_optical_frame"
}

main "$@"
