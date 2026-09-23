#!/usr/bin/env bash

set -euo pipefail

echo "WARNING: this compatibility entry point is deprecated." >&2
echo "Use /home/nvidia/project/gj_ws/sdk/scripts/record_compressed_sensors_mcap.sh." >&2

duration_seconds="${1:-600}"
timestamp="$(date +%Y_%m_%d-%H_%M_%S)"
output_path="${2:-/data/rosbag/$(date +%Y-%m-%d)/compressed_sensors_${timestamp}}"
zstd_level="${ZSTD_LEVEL:-1}"

if ! [[ "${duration_seconds}" =~ ^[1-9][0-9]*$ ]]; then
  echo "duration_seconds must be a positive integer" >&2
  exit 2
fi
share_dir="$(ros2 pkg prefix --share livox_ros_driver2_core)"
if ! ros2 pkg prefix shm_msgs >/dev/null 2>&1; then
  echo "Camera package shm_msgs is not available in the sourced ROS environment." >&2
  exit 1
fi
qos_file="${share_dir}/config/compressed_record_qos.yaml"
if [[ ! -r "${qos_file}" ]]; then
  echo "QoS override file not found: ${qos_file}" >&2
  exit 1
fi
storage_plugins="$(ros2 bag list storage)"
if ! grep -Fqx "mcap" <<<"${storage_plugins}"; then
  echo "MCAP storage plugin not found. Install ros-humble-rosbag2-storage-mcap." >&2
  exit 1
fi
if [[ -e "${output_path}" ]]; then
  echo "Output already exists: ${output_path}" >&2
  exit 1
fi

camera_topics=(
  /camera/fisheye_back/h265
  /camera/pinhole_back/h265
  /camera/fisheye_right/h265
  /camera/fisheye_left/h265
  /camera/pinhole_front/h265
  /camera/fisheye_front/h265
)

lidar_topics=(
  /livox/lidar_192_168_3_184/zstd
  /livox/lidar_192_168_1_108/zstd
  /livox/lidar_192_168_2_133/zstd
  /livox/lidar_192_168_4_143/zstd
)

required_topics=("${camera_topics[@]}" "${lidar_topics[@]}")
lidar_compressor_pid=""
recorder_pid=""
stop_requested=false

request_stop() {
  stop_requested=true
}

cleanup() {
  local status=$?
  trap - INT TERM EXIT

  if [[ -n "${recorder_pid}" ]] && kill -0 "${recorder_pid}" 2>/dev/null; then
    echo "Stopping recorder and flushing MCAP..."
    kill -INT "${recorder_pid}" 2>/dev/null || true
    wait "${recorder_pid}" 2>/dev/null || true
  fi
  if [[ -n "${lidar_compressor_pid}" ]] && kill -0 "${lidar_compressor_pid}" 2>/dev/null; then
    kill -INT "${lidar_compressor_pid}" 2>/dev/null || true
    wait "${lidar_compressor_pid}" 2>/dev/null || true
  fi

  if ((status == 0)); then
    echo "Compressed recording saved to ${output_path}"
  fi
  exit "${status}"
}

trap request_stop INT TERM
trap cleanup EXIT

echo "Expecting camera topics from: ./rb_camera.sh ros2_h265"
echo "Starting LiDAR Zstd compressor from livox_ros_driver2_core..."
ros2 run livox_ros_driver2_core pointcloud_zstd_compressor --ros-args \
  -p compression_level:="${zstd_level}" &
lidar_compressor_pid=$!

echo "Waiting up to 45 seconds for ${#required_topics[@]} compressed topics..."
deadline=$((SECONDS + 45))
while ((SECONDS < deadline)) && ! "${stop_requested}"; do
  if ! kill -0 "${lidar_compressor_pid}" 2>/dev/null; then
    wait "${lidar_compressor_pid}" || true
    echo "LiDAR compression node exited before its output topics became available." >&2
    exit 1
  fi

  topic_list="$(ros2 topic list)"
  missing=0
  for topic in "${required_topics[@]}"; do
    if ! grep -Fqx "${topic}" <<<"${topic_list}"; then
      missing=$((missing + 1))
    fi
  done
  if ((missing == 0)); then
    break
  fi
  sleep 1 || true
done

topic_list="$(ros2 topic list)"
for topic in "${required_topics[@]}"; do
  if ! grep -Fqx "${topic}" <<<"${topic_list}"; then
    echo "Compressed topic did not appear: ${topic}" >&2
    exit 1
  fi
done

mkdir -p "$(dirname "${output_path}")"
echo "Recording one MCAP for up to ${duration_seconds} seconds..."
ros2 bag record \
  --storage mcap \
  --storage-preset-profile fastwrite \
  --max-cache-size 1073741824 \
  --qos-profile-overrides-path "${qos_file}" \
  --output "${output_path}" \
  "${required_topics[@]}" &
recorder_pid=$!

deadline=$((SECONDS + duration_seconds))
while ((SECONDS < deadline)) && ! "${stop_requested}"; do
  if ! kill -0 "${recorder_pid}" 2>/dev/null; then
    wait "${recorder_pid}" || true
    echo "Recorder exited unexpectedly." >&2
    exit 1
  fi
  sleep 1 || true
done

cleanup
