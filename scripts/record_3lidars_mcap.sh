#!/usr/bin/env bash

set -euo pipefail

duration_seconds="${1:-600}"
timestamp="$(date +%Y_%m_%d-%H_%M_%S)"
output_root="${2:-/data/rosbag/$(date +%Y-%m-%d)/livox_${timestamp}}"
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

lidar_ips=(
  192_168_3_184
  192_168_1_108
  192_168_2_133
  192_168_4_143
)

active_indices=()

refresh_active_lidars() {
  local topic_list
  local index
  local ip

  topic_list="$(ros2 topic list)"
  active_indices=()
  for index in "${!lidar_ips[@]}"; do
    ip="${lidar_ips[index]}"
    if grep -Fqx "/livox/lidar_${ip}" <<<"${topic_list}" &&
       grep -Fqx "/livox/imu_${ip}" <<<"${topic_list}"; then
      active_indices+=("${index}")
    fi
  done
}

echo "Waiting up to 30 seconds for Livox topic pairs..."
deadline=$((SECONDS + 30))
last_active_count=-1
while ((SECONDS < deadline)); do
  refresh_active_lidars
  if ((${#active_indices[@]} != last_active_count)); then
    echo "Found ${#active_indices[@]}/${#lidar_ips[@]} complete LiDAR+IMU topic pairs."
    last_active_count="${#active_indices[@]}"
  fi
  if ((${#active_indices[@]} == ${#lidar_ips[@]})); then
    break
  fi
  sleep 1
done

refresh_active_lidars
if ((${#active_indices[@]} == 0)); then
  echo "No complete Livox LiDAR+IMU topic pair appeared within 30 seconds." >&2
  exit 1
fi

echo "Recording the following ${#active_indices[@]} LiDAR(s):"
for index in "${active_indices[@]}"; do
  echo "  ${lidar_names[index]}: ${lidar_ips[index]}"
done

# Let Fast DDS endpoint discovery settle before creating the recorder endpoints.
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

for index in "${active_indices[@]}"; do
  name="${lidar_names[index]}"
  ip="${lidar_ips[index]}"
  bag_path="${output_root}/${name}_${ip}"

  ros2 bag record \
    "${common_options[@]}" \
    --output "${bag_path}" \
    "/livox/imu_${ip}" \
    "/livox/lidar_${ip}" &
  pids+=("$!")
  recorder_names+=("${name}_${ip}")
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
echo "Recording ${#pids[@]} MCAP bag(s) for up to ${duration_seconds} seconds..."
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
