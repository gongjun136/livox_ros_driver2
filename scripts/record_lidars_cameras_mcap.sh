#!/usr/bin/env bash

set -euo pipefail

duration_seconds="${1:-600}"
timestamp="$(date +%Y_%m_%d-%H_%M_%S)"
output_root="${2:-/data/rosbag/$(date +%Y-%m-%d)/livox_cameras_${timestamp}}"
qos_file="$(ros2 pkg prefix --share livox_ros_driver2)/config/rosbag_record_qos.yaml"

if ! [[ "${duration_seconds}" =~ ^[1-9][0-9]*$ ]]; then
  echo "duration_seconds must be a positive integer" >&2
  exit 2
fi

if ! storage_plugins="$(ros2 bag list storage)"; then
  echo "Failed to query rosbag2 storage plugins." >&2
  exit 1
fi

if ! grep -Fqx "mcap" <<<"${storage_plugins}"; then
  echo "MCAP storage plugin not found. Install ros-humble-rosbag2-storage-mcap." >&2
  exit 1
fi

if [[ ! -r "${qos_file}" ]]; then
  echo "QoS override file not found: ${qos_file}" >&2
  exit 1
fi

lidar_names=(
  front
  left
  right
  rear
)

lidar_topics=(
  /livox/lidar_192_168_3_184
  /livox/lidar_192_168_1_108
  /livox/lidar_192_168_2_133
  /livox/lidar_192_168_4_143
)

camera_names=(
  pinhole_back
  fisheye_right
  fisheye_left
  pinhole_front
  fisheye_front
)

camera_topics=(
  /camera/pinhole_back/shm_image_6m
  /camera/fisheye_right/shm_image_6m
  /camera/fisheye_left/shm_image_6m
  /camera/pinhole_front/shm_image_6m
  /camera/fisheye_front/shm_image_6m
)

required_topics=(
  "${lidar_topics[@]}"
  "${camera_topics[@]}"
)

missing_topics=()

refresh_missing_topics() {
  local topic
  local topic_list

  topic_list="$(ros2 topic list)"
  missing_topics=()
  for topic in "${required_topics[@]}"; do
    if ! grep -Fqx "${topic}" <<<"${topic_list}"; then
      missing_topics+=("${topic}")
    fi
  done
}

echo "Waiting up to 30 seconds for all ${#required_topics[@]} required topics..."
deadline=$((SECONDS + 30))
last_missing_count=-1
while ((SECONDS < deadline)); do
  refresh_missing_topics
  if ((${#missing_topics[@]} != last_missing_count)); then
    echo "Found $((${#required_topics[@]} - ${#missing_topics[@]}))/${#required_topics[@]} required topics."
    last_missing_count="${#missing_topics[@]}"
  fi
  if ((${#missing_topics[@]} == 0)); then
    break
  fi
  sleep 1
done

refresh_missing_topics
if ((${#missing_topics[@]} != 0)); then
  echo "The following required topics did not appear within 30 seconds:" >&2
  printf '  %s\n' "${missing_topics[@]}" >&2
  exit 1
fi

echo "All required topics are available."

# Let DDS endpoint discovery settle before creating the recorder endpoints.
sleep 3
mkdir -p "${output_root}"

common_options=(
  --storage mcap
  --storage-preset-profile fastwrite
  --max-cache-size 536870912
  --qos-profile-overrides-path "${qos_file}"
)

pids=()
recorder_names=()
input_bags=()
stopped=false
stop_requested=false

request_stop() {
  echo
  echo "Stop requested. Finishing the current writes before merging..."
  stop_requested=true
}

stop_recorders() {
  local pid

  if "${stopped}"; then
    return
  fi
  stopped=true
  trap - INT TERM EXIT

  echo "Stopping recorders and flushing MCAP buffers..."
  for pid in "${pids[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  echo "Recording saved under ${output_root}"
}

trap request_stop INT TERM
trap stop_recorders EXIT

for index in "${!lidar_topics[@]}"; do
  name="${lidar_names[index]}"
  topic="${lidar_topics[index]}"
  bag_path="${output_root}/lidar_${name}"

  ros2 bag record \
    "${common_options[@]}" \
    --output "${bag_path}" \
    "${topic}" &
  pids+=("$!")
  recorder_names+=("lidar_${name}")
  input_bags+=("${bag_path}")
done

for index in "${!camera_topics[@]}"; do
  name="${camera_names[index]}"
  topic="${camera_topics[index]}"
  bag_path="${output_root}/camera_${name}"

  ros2 bag record \
    "${common_options[@]}" \
    --output "${bag_path}" \
    "${topic}" &
  pids+=("$!")
  recorder_names+=("camera_${name}")
  input_bags+=("${bag_path}")
done

sleep 2 || true
if ! "${stop_requested}"; then
  for index in "${!pids[@]}"; do
    pid="${pids[index]}"
    if ! kill -0 "${pid}" 2>/dev/null; then
      echo "Recorder ${recorder_names[index]} failed to start." >&2
      exit 1
    fi
  done
fi

recording_start="${SECONDS}"
echo "Recording 4 LiDAR bags and 5 camera bags for up to ${duration_seconds} seconds..."
recording_deadline=$((recording_start + duration_seconds))
while ((SECONDS < recording_deadline)) && ! "${stop_requested}"; do
  sleep 1 || true
done
stop_recorders

merge_options="${output_root}/merge_options.yaml"
merged_output="${output_root}/merged"
cat > "${merge_options}" <<EOF
output_bags:
  - uri: "${merged_output}"
    storage_id: mcap
    storage_preset_profile: zstd_fast
    all: true
EOF

convert_inputs=()
for input_bag in "${input_bags[@]}"; do
  convert_inputs+=(--input "${input_bag}" mcap)
done

echo "Merging ${#input_bags[@]} bag(s) into ${merged_output}..."
if ! ros2 bag convert \
  "${convert_inputs[@]}" \
  --output-options "${merge_options}"; then
  echo "Merge failed; the original bags are still intact under ${output_root}." >&2
  exit 1
fi

echo "Merged indexed MCAP saved to ${merged_output}"
