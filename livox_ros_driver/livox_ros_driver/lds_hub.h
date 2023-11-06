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

/** Livox LiDAR data source, data from hub */

#ifndef LIVOX_ROS_DRIVER_LDS_HUB_H_
#define LIVOX_ROS_DRIVER_LDS_HUB_H_

#include <memory>
#include <vector>

#include "lds.h"
#include "livox_sdk.h"

namespace livox_ros {

/**
 * LiDAR data source, data from hub.
 */
class LdsHub : public Lds {
 public:
  static LdsHub *GetInstance(uint32_t interval_ms) {
    static LdsHub lds_hub(interval_ms);
    return &lds_hub;
  }

  int InitLdsHub(std::vector<std::string> &broadcast_code_strs,
                 const char *user_config_path);
  int DeInitLdsHub(void);

 private:
  LdsHub(uint32_t interval_ms);
  LdsHub(const LdsHub &) = delete;
  ~LdsHub();
  LdsHub &operator=(const LdsHub &) = delete;
  virtual void PrepareExit(void);

  static void OnHubDataCb(uint8_t hub_handle, LivoxEthPacket *data,
                          uint32_t data_num, void *client_data);
  static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
  static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
  static void StartSampleCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data);
  static void StopSampleCb(livox_status status, uint8_t handle,
                           uint8_t response, void *clent_data);
  static void HubQueryLidarInfoCb(livox_status status, uint8_t handle,
                                  HubQueryLidarInformationResponse *response,
                                  void *client_data);
  static void ControlFanCb(livox_status status, uint8_t handle,
                           uint8_t response, void *clent_data);
  static void HubSetPointCloudReturnModeCb(
      livox_status status, uint8_t handle,
      HubSetPointCloudReturnModeResponse *response, void *clent_data);
  static void SetCoordinateCb(livox_status status, uint8_t handle,
                              uint8_t response, void *clent_data);
  static void HubSetImuRatePushFrequencyCb(
      livox_status status, uint8_t handle,
      HubSetImuPushFrequencyResponse *response, void *clent_data);
  static void HubErrorStatusCb(livox_status status, uint8_t handle,
                               ErrorMessage *message);
  static void ConfigPointCloudReturnMode(LdsHub *lds_hub);
  static void ConfigImuPushFrequency(LdsHub *lds_hub);
  static void ConfigLidarsOfHub(LdsHub *lds_hub);

  void ResetLdsHub(void);
  void StateReset(void);
  int AddBroadcastCodeToWhitelist(const char *broadcast_code);
  bool IsBroadcastCodeExistInWhitelist(const char *broadcast_code);
  void UpdateHubLidarinfo(void);

  void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }
  int ParseConfigFile(const char *pathname);
  int AddRawUserConfig(UserRawConfig &config);
  bool IsExistInRawConfig(const char *broadcast_code);
  int GetRawConfig(const char *broadcast_code, UserRawConfig &config);
  bool IsAllLidarSetBitsClear() {
    for (int i = 0; i < kMaxLidarCount; i++) {
      if (lidars_[i].config.set_bits) {
        return false;
      }
    }
    return true;
  }

  bool auto_connect_mode_;
  uint32_t whitelist_count_;
  volatile bool is_initialized_;
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];

  LidarDevice hub_;

  std::vector<UserRawConfig> lidar_raw_config_;
  UserRawConfig hub_raw_config_;
};

}  // namespace livox_ros
#endif
