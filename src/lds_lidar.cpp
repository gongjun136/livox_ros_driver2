//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>

#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32
#include "livox_lidar_api.h"
#include "comm/comm.h"
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

using namespace std;

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq)
    : Lds(publish_freq, kSourceRawLidar), 
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }



bool LdsLidar::InitLdsLidar(const std::string& path_name) {
  if (is_initialized_) {
    printf("Lds is already inited!\n");
    return false;
  }

  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }

  path_ = path_name;
  if (!InitLidars()) {
    return false;
  }
  SetLidarPubHandle();
  if (!Start()) {
    return false;
  }
  is_initialized_ = true;
  return true;
}

bool LdsLidar::InitLidars() {
  if (!ParseSummaryConfig()) {
    return false;
  }
  std::cout << "config lidar type: " << static_cast<int>(lidar_summary_info_.lidar_type) << std::endl;

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!InitLivoxLidar()) {
      return false;
    }
  }
  return true;
}


bool LdsLidar::Start() {
  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!LivoxLidarStart()) {
      return false;
    }
  }
  return true;
}

bool LdsLidar::ParseSummaryConfig() {
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

bool LdsLidar::InitLivoxLidar() {
#ifdef BUILDING_ROS2
  DisableLivoxSdkConsoleLogger();
#endif

  // parse user config
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }
  // [livox-role-single-device]
  if (diagnostic_monitor_) {
    if (user_configs.size() != 1) {
      std::cerr << "Role-specific DTC monitoring requires exactly one configured LiDAR" << std::endl;
      return false;
    }
    diagnostic_handle_ = user_configs.front().handle;
  }
  // [livox-role-single-device]

  // SDK initialization
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // fill in lidar devices
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
    p_lidar->handle = config.handle;

    LidarExtParameter lidar_param;
    lidar_param.handle = config.handle;
    lidar_param.lidar_type = kLivoxLidarType;
    if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
      // temporary resolution
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x / 10;
      lidar_param.param.y     = config.extrinsic_param.y / 10;
      lidar_param.param.z     = config.extrinsic_param.z / 10;
    } else {
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x;
      lidar_param.param.y     = config.extrinsic_param.y;
      lidar_param.param.z     = config.extrinsic_param.z;
    }
    pub_handler().AddLidarsExtParam(lidar_param);
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  if (diagnostic_monitor_) {
    SetLivoxLidarInfoCallback(DiagnosticStatusCallback, this);
    diagnostic_observer_id_ = LivoxLidarAddPointCloudObserver(DiagnosticPacketCallback, this);
  }
  return true;
}

// [livox-register-data-callbacks]
void LdsLidar::SetLidarPubHandle() {
  pub_handler().SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  pub_handler().SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

  double publish_freq = Lds::GetLdsFrequency();
  pub_handler().SetPointCloudConfig(publish_freq);
}
// [livox-register-data-callbacks]

bool LdsLidar::LivoxLidarStart() {
  return true;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (diagnostic_monitor_) {
      LivoxLidarRemovePointCloudObserver(diagnostic_observer_id_);
      SetLivoxLidarInfoCallback(nullptr, nullptr);
    }
    LivoxLidarSdkUninit();
    printf("Livox Lidar SDK Deinit completely!\n");
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

void LdsLidar::DiagnosticStatusCallback(uint32_t handle, uint8_t, const char* info, void* context) {
  auto* self = static_cast<LdsLidar*>(context);
  if (!self || !self->diagnostic_monitor_ || handle != self->diagnostic_handle_ || !info) return;
  rapidjson::Document state;
  state.Parse(info);
  if (state.HasParseError() || !state.IsObject()) return;
  if (state.HasMember("cur_work_state") && state["cur_work_state"].IsUint()) {
    self->diagnostic_monitor_->ObserveWorkState(static_cast<uint8_t>(state["cur_work_state"].GetUint()));
  }
  if (state.HasMember("time_sync_type") && state["time_sync_type"].IsUint() &&
      state["time_sync_type"].GetUint() <= 2 &&
      state.HasMember("local_time_now") && state["local_time_now"].IsUint64() &&
      state.HasMember("last_sync_time") && state["last_sync_time"].IsUint64() &&
      state.HasMember("time_offset") && state["time_offset"].IsInt64()) {
    self->diagnostic_monitor_->ObserveTimeSync(state["time_sync_type"].GetUint(),
        state["local_time_now"].GetUint64(), state["last_sync_time"].GetUint64(), state["time_offset"].GetInt64());
  }
  if (!state.HasMember("hms_code") || !state["hms_code"].IsArray() || state["hms_code"].Size() != 8) return;
  std::array<uint32_t, 8> codes{};
  for (size_t i = 0; i < codes.size(); ++i) {
    if (!state["hms_code"][i].IsUint()) return;
    codes[i] = state["hms_code"][i].GetUint();
  }
  self->diagnostic_monitor_->ObserveHms(codes);
}

void LdsLidar::DiagnosticPacketCallback(uint32_t handle, uint8_t, LivoxLidarEthernetPacket* data, void* context) {
  auto* self = static_cast<LdsLidar*>(context);
  if (self && self->diagnostic_monitor_ && handle == self->diagnostic_handle_ && data &&
      data->data_type != kLivoxLidarImuData) self->diagnostic_monitor_->ObservePacket();
}

void LdsLidar::ObserveDiagnosticCloud(PointFrame* frame) {
  if (!diagnostic_monitor_ || !frame) return;
  for (size_t i = 0; i < frame->lidar_num; ++i) {
    const auto& cloud = frame->lidar_point[i];
    if (cloud.handle != diagnostic_handle_) continue;
    size_t valid_points = 0;
    if (cloud.points) for (size_t j = 0; j < cloud.points_num; ++j) {
      const auto& p = cloud.points[j];
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
          p.x * p.x + p.y * p.y + p.z * p.z > 0.0001f) ++valid_points;
    }
    diagnostic_monitor_->ObserveCloud(frame->base_time[i], valid_points);
  }
}

void LdsLidar::QueryDiagnosticStatus() {
  if (!diagnostic_monitor_ || diagnostic_handle_ == 0 || !is_initialized_ ||
      diagnostic_query_pending_.exchange(true)) return;
  if (QueryLivoxLidarInternalInfo(diagnostic_handle_, DiagnosticQueryCallback, this) != kLivoxLidarStatusSuccess) {
    diagnostic_query_pending_ = false;
  }
}

void LdsLidar::DiagnosticQueryCallback(livox_status status, uint32_t handle,
    LivoxLidarDiagInternalInfoResponse* response, void* context) {
  auto* self = static_cast<LdsLidar*>(context);
  if (!self) return;
  self->diagnostic_query_pending_ = false;
  if (!self->diagnostic_monitor_ || handle != self->diagnostic_handle_ ||
      status != kLivoxLidarStatusSuccess || !response || response->ret_code != 0) return;
  // The SDK validates the command envelope. Each requested key is still length-checked.
  size_t offset = 0;
  if (response->param_num > 64) return;
  std::array<uint32_t, 8> codes{};
  uint8_t work_state = 0, sync_type = 0;
  uint64_t local_time = 0, last_sync = 0;
  int64_t time_offset = 0;
  bool hms_seen = false, work_seen = false;
  uint8_t sync_fields = 0;
  for (uint16_t i = 0; i < response->param_num; ++i) {
    uint16_t key = 0, length = 0;
    if (offset + 4 > 1400) return;
    std::memcpy(&key, response->data + offset, 2);
    std::memcpy(&length, response->data + offset + 2, 2);
    offset += 4;
    if (offset + length > 1400) return;
    if (key == kKeyHmsCode && length == 32) {
      std::memcpy(codes.data(), response->data + offset, 32);
      hms_seen = true;
    } else if (key == kKeyCurWorkState && length == 1) {
      work_state = response->data[offset]; work_seen = true;
    } else if (key == kKeyTimeSyncType && length == 1) {
      sync_type = response->data[offset]; sync_fields |= 1;
    } else if (key == kKeyLocalTimeNow && length == 8) {
      std::memcpy(&local_time, response->data + offset, 8); sync_fields |= 2;
    } else if (key == kKeyLastSyncTime && length == 8) {
      std::memcpy(&last_sync, response->data + offset, 8); sync_fields |= 4;
    } else if (key == kKeyTimeOffset && length == 8) {
      std::memcpy(&time_offset, response->data + offset, 8); sync_fields |= 8;
    }
    offset += length;
  }
  if (sync_fields == 15) self->diagnostic_monitor_->ObserveTimeSync(sync_type, local_time, last_sync, time_offset);
  if (work_seen) self->diagnostic_monitor_->ObserveWorkState(work_state);
  if (hms_seen) self->diagnostic_monitor_->ObserveHms(codes);
}

}  // namespace livox_ros
