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

/** Livox LiDAR data source, data from dependent lidar */

#ifndef LIVOX_ROS_DRIVER_LDS_LIDAR_H_
#define LIVOX_ROS_DRIVER_LDS_LIDAR_H_

#include <memory>
#include <mutex>
#include <vector>

#include "lds.h"
#include "livox_sdk.h"
#include "rapidjson/document.h"
#include "timesync.h"

namespace livox_ros {

/**
 * LiDAR data source, data from dependent lidar.
 */
class LdsLidar : public Lds {
 public:
  static LdsLidar *GetInstance(uint32_t interval_ms) {
    static LdsLidar lds_lidar(interval_ms);
    return &lds_lidar;
  }

  int InitLdsLidar(std::vector<std::string> &broadcast_code_strs,
                   const char *user_config_path);
  int DeInitLdsLidar(void);

 private:
  LdsLidar(uint32_t interval_ms);
  LdsLidar(const LdsLidar &) = delete;
  ~LdsLidar();
  LdsLidar &operator=(const LdsLidar &) = delete;
  virtual void PrepareExit(void);

  static void OnLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                            uint32_t data_num, void *client_data);
  static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
  static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
  static void StartSampleCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data);
  static void StopSampleCb(livox_status status, uint8_t handle,
                           uint8_t response, void *clent_data);
  static void DeviceInformationCb(livox_status status, uint8_t handle,
                                  DeviceInformationResponse *ack,
                                  void *clent_data);
  static void LidarErrorStatusCb(livox_status status, uint8_t handle,
                                 ErrorMessage *message);
  static void ControlFanCb(livox_status status, uint8_t handle,
                           uint8_t response, void *clent_data);
  static void SetPointCloudReturnModeCb(livox_status status, uint8_t handle,
                                        uint8_t response, void *clent_data);
  static void SetCoordinateCb(livox_status status, uint8_t handle,
                              uint8_t response, void *clent_data);
  static void SetImuRatePushFrequencyCb(livox_status status, uint8_t handle,
                                        uint8_t response, void *clent_data);
  static void SetRmcSyncTimeCb(livox_status status, uint8_t handle,
                               uint8_t response, void *client_data);
  static void ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length,
                                      void *client_data);
  static void GetLidarExtrinsicParameterCb(
      livox_status status, uint8_t handle,
      LidarGetExtrinsicParameterResponse *response, void *clent_data);
  static void SetHighSensitivityCb(livox_status status, uint8_t handle,
                                   DeviceParameterResponse *response,
                                   void *clent_data);

  void ResetLdsLidar(void);
  int AddBroadcastCodeToWhitelist(const char *broadcast_code);
  bool IsBroadcastCodeExistInWhitelist(const char *broadcast_code);

  void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }
  int ParseTimesyncConfig(rapidjson::Document &doc);
  int ParseConfigFile(const char *pathname);
  int AddRawUserConfig(UserRawConfig &config);
  bool IsExistInRawConfig(const char *broadcast_code);
  int GetRawConfig(const char *broadcast_code, UserRawConfig &config);

  bool auto_connect_mode_;
  uint32_t whitelist_count_;
  volatile bool is_initialized_;
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];
  std::vector<UserRawConfig> raw_config_;

  bool enable_timesync_;
  TimeSync *timesync_;
  TimeSyncConfig timesync_config_;
  std::mutex config_mutex_;
};

}  // namespace livox_ros
#endif
