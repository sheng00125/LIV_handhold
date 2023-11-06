//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
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

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"

using namespace std;

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Global function for common use -------------------------------------------*/

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(uint32_t interval_ms) : Lds(interval_ms, kSourceRawLidar) {
  auto_connect_mode_ = true;
  is_initialized_ = false;

  whitelist_count_ = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }

int LdsLidar::InitLdsLidar(std::vector<std::string> &broadcast_code_strs,
                           const char *user_config_path) {
  if (is_initialized_) {
    printf("LiDAR data source is already inited!\n");
    return -1;
  }

  if (!Init()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor,
         _sdkversion.patch);

  SetBroadcastCallback(OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(OnDeviceChange);

  /** Add commandline input broadcast code */
  for (auto input_str : broadcast_code_strs) {
    AddBroadcastCodeToWhitelist(input_str.c_str());
  }
// 这解析 config文件，并且设置一下串口
  ParseConfigFile(user_config_path);

  if (whitelist_count_) {
    DisableAutoConnectMode();
    printf("Disable auto connect mode!\n");

    printf("List all broadcast code in whiltelist:\n");
    for (uint32_t i = 0; i < whitelist_count_; i++) {
      printf("%s\n", broadcast_code_whitelist_[i]);
    }
  } else {
    EnableAutoConnectMode();
    printf(
        "No broadcast code was added to whitelist, swith to automatic "
        "connection mode!\n");
  }

  if (enable_timesync_) {
    timesync_ = TimeSync::GetInstance();
    if (timesync_->InitTimeSync(timesync_config_)) {
      printf("Timesync init fail\n");
      return -1;
    }
// 在这里设置回调函数
    if (timesync_->SetReceiveSyncTimeCb(ReceiveSyncTimeCallback, this)) {
      printf("Set Timesync callback fail\n");
      return -1;
    }

    timesync_->StartTimesync();
  }

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }
  is_initialized_ = true;
  printf("Livox-SDK init success!\n");

  return 0;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  Uninit();
  printf("Livox SDK Deinit completely!\n");

  if (timesync_) {
    timesync_->DeInitTimeSync();
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

/** Static function in LdsLidar for callback or event process ----------------*/

/** Receiving point cloud data from Livox LiDAR. */
void LdsLidar::OnLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                             uint32_t data_num, void *client_data) {
  using namespace std;

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  LivoxEthPacket *eth_packet = data;

  if (!data || !data_num || (handle >= kMaxLidarCount)) {
    return;
  }

  lds_lidar->StorageRawPacket(handle, eth_packet);
}

void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    printf("In lidar mode, couldn't connect a hub : %s\n",
           info->broadcast_code);
    return;
  }

  if (g_lds_ldiar->IsAutoConnectMode()) {
    printf("In automatic connection mode, will connect %s\n",
           info->broadcast_code);
  } else {
    if (!g_lds_ldiar->IsBroadcastCodeExistInWhitelist(info->broadcast_code)) {
      printf("Not in the whitelist, please add %s to if want to connect!\n",
             info->broadcast_code);
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, OnLidarDataCb, (void *)g_lds_ldiar);

    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;

    UserRawConfig config;
    if (g_lds_ldiar->GetRawConfig(info->broadcast_code, config)) {
      printf("Could not find raw config, set config to default!\n");
      config.enable_fan = 1;
      config.return_mode = kFirstReturn;
      config.coordinate = kCoordinateCartesian;
      config.imu_rate = kImuFreq200Hz;
      config.extrinsic_parameter_source = kNoneExtrinsicParameter;
      config.enable_high_sensitivity = false;
    }

    p_lidar->config.enable_fan = config.enable_fan;
    p_lidar->config.return_mode = config.return_mode;
    p_lidar->config.coordinate = config.coordinate;
    p_lidar->config.imu_rate = config.imu_rate;
    p_lidar->config.extrinsic_parameter_source =
        config.extrinsic_parameter_source;
    p_lidar->config.enable_high_sensitivity = config.enable_high_sensitivity;
  } else {
    printf("Add lidar to connect is failed : %d %d \n", result, handle);
  }
}

/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, DeviceInformationCb, g_lds_ldiar);
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
  } else if (type == kEventDisconnect) {
    printf("Lidar[%s] disconnect!\n", info->broadcast_code);
    ResetLidar(p_lidar, kSourceRawLidar);
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
  }

  if (p_lidar->connect_state == kConnectStateOn) {
    printf("Lidar[%s] status_code[%d] working state[%d] feature[%d]\n",
           p_lidar->info.broadcast_code,
           p_lidar->info.status.status_code.error_code, p_lidar->info.state,
           p_lidar->info.feature);
    SetErrorMessageCallback(handle, LidarErrorStatusCb);

    /** Config lidar parameter */
    if (p_lidar->info.state == kLidarStateNormal) {
      /** Ensure the thread safety for set_bits and connect_state */
      lock_guard<mutex> lock(g_lds_ldiar->config_mutex_);

      if (p_lidar->config.coordinate != 0) {
        SetSphericalCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      } else {
        SetCartesianCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      }
      p_lidar->config.set_bits |= kConfigCoordinate;

      if (kDeviceTypeLidarMid40 != info->type) {
        LidarSetPointCloudReturnMode(
            handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
            SetPointCloudReturnModeCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigReturnMode;
      }

      if ((kDeviceTypeLidarMid70 != info->type) &&
          (kDeviceTypeLidarMid40 != info->type)) {
        LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                                 SetImuRatePushFrequencyCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigImuRate;
      }

      if (p_lidar->config.extrinsic_parameter_source ==
          kExtrinsicParameterFromLidar) {
        LidarGetExtrinsicParameter(handle, GetLidarExtrinsicParameterCb,
                                   g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigGetExtrinsicParameter;
      }

      if (kDeviceTypeLidarTele == info->type) {
        if (p_lidar->config.enable_high_sensitivity) {
          LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
          printf("Enable high sensitivity\n");
        } else {
          LidarDisableHighSensitivity(handle, SetHighSensitivityCb,
                                      g_lds_ldiar);
          printf("Disable high sensitivity\n");
        }
        p_lidar->config.set_bits |= kConfigSetHighSensitivity;
      }

      p_lidar->connect_state = kConnectStateConfig;
    }
  }
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle,
                                   DeviceInformationResponse *ack,
                                   void *clent_data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed : %d\n", status);
  }
  if (ack) {
    printf("firmware version: %d.%d.%d.%d\n", ack->firmware_version[0],
           ack->firmware_version[1], ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

/** Callback function of Lidar error message. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle,
                                  ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n",
             message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

void LdsLidar::ControlFanCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data) {}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    printf("Set return mode success!\n");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetPointCloudReturnMode(
        handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
        SetPointCloudReturnModeCb, lds_lidar);
    printf("Set return mode fail, try again!\n");
  }
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle,
                               uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    printf("Set coordinate success!\n");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_lidar->config.coordinate != 0) {
      SetSphericalCoordinate(handle, SetCoordinateCb, lds_lidar);
    } else {
      SetCartesianCoordinate(handle, SetCoordinateCb, lds_lidar);
    }

    printf("Set coordinate fail, try again!\n");
  }
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    printf("Set imu rate success!\n");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                             SetImuRatePushFrequencyCb, g_lds_ldiar);
    printf("Set imu rate fail, try again!\n");
  }
}

/** Callback function of get LiDARs' extrinsic parameter. */
void LdsLidar::GetLidarExtrinsicParameterCb(
    livox_status status, uint8_t handle,
    LidarGetExtrinsicParameterResponse *response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);
  if (handle >= kMaxLidarCount) {
    return;
  }

  if (status == kStatusSuccess) {
    if (response != nullptr) {
      printf("Lidar[%d] get ExtrinsicParameter status[%d] response[%d]\n",
             handle, status, response->ret_code);
      LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
      ExtrinsicParameter *p_extrinsic = &p_lidar->extrinsic_parameter;
      p_extrinsic->euler[0] = static_cast<float>(response->roll * PI / 180.0);
      p_extrinsic->euler[1] = static_cast<float>(response->pitch * PI / 180.0);
      p_extrinsic->euler[2] = static_cast<float>(response->yaw * PI / 180.0);
      p_extrinsic->trans[0] = static_cast<float>(response->x / 1000.0);
      p_extrinsic->trans[1] = static_cast<float>(response->y / 1000.0);
      p_extrinsic->trans[2] = static_cast<float>(response->z / 1000.0);
      EulerAnglesToRotationMatrix(p_extrinsic->euler, p_extrinsic->rotation);
      if (p_lidar->config.extrinsic_parameter_source) {
        p_extrinsic->enable = true;
      }
      printf("Lidar[%d] get ExtrinsicParameter success!\n", handle);

      lock_guard<mutex> lock(lds_lidar->config_mutex_);
      p_lidar->config.set_bits &= ~((uint32_t)(kConfigGetExtrinsicParameter));
      if (!p_lidar->config.set_bits) {
        LidarStartSampling(handle, StartSampleCb, lds_lidar);
        p_lidar->connect_state = kConnectStateSampling;
      }
    } else {
      printf("Lidar[%d] get ExtrinsicParameter fail!\n", handle);
    }
  } else if (status == kStatusTimeout) {
    printf("Lidar[%d] get ExtrinsicParameter timeout!\n", handle);
  }
}

void LdsLidar::SetHighSensitivityCb(livox_status status, uint8_t handle,
                                    DeviceParameterResponse *response,
                                    void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigSetHighSensitivity));
    printf("Set high sensitivity success!\n");

    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    };
  } else {
    if (p_lidar->config.enable_high_sensitivity) {
      LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    } else {
      LidarDisableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    }
    printf("Set high sensitivity fail, try again!\n");
  }
}

/** Callback function of starting sampling. */
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle,
                             uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);

  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
  if (status == kStatusSuccess) {
    if (response != 0) {
      p_lidar->connect_state = kConnectStateOn;
      printf("Lidar start sample fail : state[%d] handle[%d] res[%d]\n", status,
             handle, response);
    } else {
      printf("Lidar start sample success\n");
    }
  } else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    printf("Lidar start sample timeout : state[%d] handle[%d] res[%d]\n",
           status, handle, response);
  }
}

/** Callback function of stopping sampling. */
void LdsLidar::StopSampleCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data) {}

void LdsLidar::SetRmcSyncTimeCb(livox_status status, uint8_t handle,
                                uint8_t response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  printf("Set lidar[%d] sync time status[%d] response[%d]\n", handle, status,
         response);
}
// 这是GRMS 的回调函数
void LdsLidar::ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length,
                                       void *client_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  // std::unique_lock<std::mutex> lock(mtx);
  LidarDevice *p_lidar = nullptr;
  for (uint8_t handle = 0; handle < kMaxLidarCount; handle++) {
    p_lidar = &(lds_lidar->lidars_[handle]);
    if (p_lidar->connect_state == kConnectStateSampling &&
        p_lidar->info.state == kLidarStateNormal) {
      // LidarSetRmcSyncTime是SDK的函数
      livox_status status = LidarSetRmcSyncTime(handle, rmc, rmc_length,
                                                SetRmcSyncTimeCb, lds_lidar);
      // status=-1
      if (status != kStatusSuccess) {
        printf("Set GPRMC synchronization time error code: %d.\n", status);
      }
    }
  }
}

/** Add broadcast code to whitelist */
int LdsLidar::AddBroadcastCodeToWhitelist(const char *broadcast_code) {
  if (!broadcast_code || (strlen(broadcast_code) > kBroadcastCodeSize) ||
      (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (LdsLidar::IsBroadcastCodeExistInWhitelist(broadcast_code)) {
    printf("%s is alrealy exist!\n", broadcast_code);
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], broadcast_code);
  ++whitelist_count_;

  return 0;
}

bool LdsLidar::IsBroadcastCodeExistInWhitelist(const char *broadcast_code) {
  if (!broadcast_code) {
    return false;
  }

  for (uint32_t i = 0; i < whitelist_count_; i++) {
    if (strncmp(broadcast_code, broadcast_code_whitelist_[i],
                kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

int LdsLidar::ParseTimesyncConfig(rapidjson::Document &doc) {
  do {
    if (!doc.HasMember("timesync_config") || !doc["timesync_config"].IsObject())
      break;

    const rapidjson::Value &object = doc["timesync_config"];
    if (!object.IsObject()) break;

    if (!object.HasMember("enable_timesync") ||
        !object["enable_timesync"].IsBool())
      break;
    enable_timesync_ = object["enable_timesync"].GetBool();

    if (!object.HasMember("device_name") || !object["device_name"].IsString())
      break;
    std::string device_name = object["device_name"].GetString();
    std::strncpy(timesync_config_.dev_config.name, device_name.c_str(),
                 sizeof(timesync_config_.dev_config.name));

    if (!object.HasMember("comm_device_type") ||
        !object["comm_device_type"].IsInt())
      break;
    timesync_config_.dev_config.type = object["comm_device_type"].GetInt();

    if (timesync_config_.dev_config.type == kCommDevUart) {
      if (!object.HasMember("baudrate_index") ||
          !object["baudrate_index"].IsInt())
        break;
      timesync_config_.dev_config.config.uart.baudrate =
          object["baudrate_index"].GetInt();

      if (!object.HasMember("parity_index") || !object["parity_index"].IsInt())
        break;
      timesync_config_.dev_config.config.uart.parity =
          object["parity_index"].GetInt();
    }

    if (enable_timesync_) {
      printf("Enable timesync : \n");
      if (timesync_config_.dev_config.type == kCommDevUart) {
        printf("Uart[%s],baudrate index[%d],parity index[%d]\n",
               timesync_config_.dev_config.name,
               timesync_config_.dev_config.config.uart.baudrate,
               timesync_config_.dev_config.config.uart.parity);
      }
    } else {
      printf("Disable timesync\n");
    }
    return 0;
  } while (0);

  return -1;
}

/** Config file process */
int LdsLidar::ParseConfigFile(const char *pathname) {
  FILE *raw_file = std::fopen(pathname, "rb");
  if (!raw_file) {
    printf("Open json config file fail!\n");
    return -1;
  }

  char read_buffer[32768];
  rapidjson::FileReadStream config_file(raw_file, read_buffer,
                                        sizeof(read_buffer));

  rapidjson::Document doc;
  if (!doc.ParseStream(config_file).HasParseError()) {
    if (doc.HasMember("lidar_config") && doc["lidar_config"].IsArray()) {
      const rapidjson::Value &array = doc["lidar_config"];
      size_t len = array.Size();
      for (size_t i = 0; i < len; i++) {
        const rapidjson::Value &object = array[i];
        if (object.IsObject()) {
          UserRawConfig config = {0};
          memset(&config, 0, sizeof(config));
          if (object.HasMember("broadcast_code") &&
              object["broadcast_code"].IsString()) {
            std::string broadcast_code = object["broadcast_code"].GetString();
            std::strncpy(config.broadcast_code, broadcast_code.c_str(),
                         sizeof(config.broadcast_code));
          } else {
            printf("User config file parse error\n");
            continue;
          }

          if (object.HasMember("enable_connect") &&
              object["enable_connect"].IsBool()) {
            config.enable_connect = object["enable_connect"].GetBool();
          }
          if (object.HasMember("enable_fan") && object["enable_fan"].IsBool()) {
            config.enable_fan = object["enable_fan"].GetBool();
          }
          if (object.HasMember("return_mode") &&
              object["return_mode"].IsInt()) {
            config.return_mode = object["return_mode"].GetInt();
          }
          if (object.HasMember("coordinate") && object["coordinate"].IsInt()) {
            config.coordinate = object["coordinate"].GetInt();
          }
          if (object.HasMember("imu_rate") && object["imu_rate"].IsInt()) {
            config.imu_rate = object["imu_rate"].GetInt();
          }
          if (object.HasMember("extrinsic_parameter_source") &&
              object["extrinsic_parameter_source"].IsInt()) {
            config.extrinsic_parameter_source =
                object["extrinsic_parameter_source"].GetInt();
          }
          if (object.HasMember("enable_high_sensitivity") &&
              object["enable_high_sensitivity"].GetBool()) {
            config.enable_high_sensitivity =
                object["enable_high_sensitivity"].GetBool();
          }

          printf("broadcast code[%s] : %d %d %d %d %d %d\n",
                 config.broadcast_code, config.enable_connect,
                 config.enable_fan, config.return_mode, config.coordinate,
                 config.imu_rate, config.extrinsic_parameter_source);
          if (config.enable_connect) {
            if (!AddBroadcastCodeToWhitelist(config.broadcast_code)) {
              if (AddRawUserConfig(config)) {
                printf("Raw config is already exist : %s \n",
                       config.broadcast_code);
              }
            }
          }
        }
      }
    }
// 在这里设置
    if (ParseTimesyncConfig(doc)) {
      printf("Parse timesync config fail\n");
      enable_timesync_ = false;
    }
  } else {
    printf("User config file parse error[%d]\n",
           doc.ParseStream(config_file).HasParseError());
  }

  std::fclose(raw_file);

  return 0;
}

int LdsLidar::AddRawUserConfig(UserRawConfig &config) {
  if (IsExistInRawConfig(config.broadcast_code)) {
    return -1;
  }

  raw_config_.push_back(config);
  printf("Add Raw user config : %s \n", config.broadcast_code);

  return 0;
}

bool LdsLidar::IsExistInRawConfig(const char *broadcast_code) {
  if (broadcast_code == nullptr) {
    return false;
  }

  for (auto ite_config : raw_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code,
                kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

int LdsLidar::GetRawConfig(const char *broadcast_code, UserRawConfig &config) {
  if (broadcast_code == nullptr) {
    return -1;
  }

  for (auto ite_config : raw_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code,
                kBroadcastCodeSize) == 0) {
      config.enable_fan = ite_config.enable_fan;
      config.return_mode = ite_config.return_mode;
      config.coordinate = ite_config.coordinate;
      config.imu_rate = ite_config.imu_rate;
      config.extrinsic_parameter_source = ite_config.extrinsic_parameter_source;
      config.enable_high_sensitivity = ite_config.enable_high_sensitivity;
      return 0;
    }
  }

  return -1;
}

}  // namespace livox_ros
