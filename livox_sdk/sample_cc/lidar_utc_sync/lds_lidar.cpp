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
#include <thread>
#include <memory>
#include <mutex>

std::mutex mtx;


/** Const varible ------------------------------------------------------------------------------- */
/** User add broadcast code here */
static const char* local_broadcast_code_list[] = {
  "000000000000001",
};

/** For callback use only */
LdsLidar* g_lidars = nullptr;

/** Lds lidar function ---------------------------------------------------------------------------*/
LdsLidar::LdsLidar() {
  auto_connect_mode_ = true;
  whitelist_count_   = 0;
  is_initialized_    = false;

  lidar_count_       = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

  memset(lidars_, 0, sizeof(lidars_));
  for (uint32_t i=0; i<kMaxLidarCount; i++) {
    lidars_[i].handle = kMaxLidarCount;
    /** Unallocated state */
    lidars_[i].connect_state = kConnectStateOff;
  }
}

LdsLidar::~LdsLidar() {
}

int LdsLidar::InitLdsLidar(std::vector<std::string>& broadcast_code_strs) {

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
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  SetBroadcastCallback(LdsLidar::OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(LdsLidar::OnDeviceChange);

  /** Add commandline input broadcast code */
  for (auto input_str : broadcast_code_strs) {
    LdsLidar::AddBroadcastCodeToWhitelist(input_str.c_str());
  }

  /** Add local broadcast code */
  LdsLidar::AddLocalBroadcastCode();

  if (whitelist_count_) {
    LdsLidar::DisableAutoConnectMode();
    printf("Disable auto connect mode!\n");

    printf("List all broadcast code in whiltelist:\n");
    for (uint32_t i=0; i<whitelist_count_; i++) {
      printf("%s\n", broadcast_code_whitelist_[i]);
    }
  } else {
    LdsLidar::EnableAutoConnectMode();
    printf("No broadcast code was added to whitelist, swith to automatic connection mode!\n");
  }

  /** Start the device discovering routine. */
  if (!Start()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lidars == nullptr) {
    g_lidars = this;
  }
  is_initialized_= true;
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

  return 0;
}


/** Static function in LdsLidar for callback or event process ------------------------------------*/

void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    printf("In lidar mode, couldn't connect a hub : %s\n", info->broadcast_code);
    return;
  }
  std::unique_lock<std::mutex> lock(mtx);
  if (g_lidars->IsAutoConnectMode()) {
    printf("In automatic connection mode, will connect %s\n", info->broadcast_code);
  } else {
    if (!g_lidars->FindInWhitelist(info->broadcast_code)) {
      printf("Not in the whitelist, please add %s to if want to connect!\n",\
             info->broadcast_code);
      return;
    }
  }

  livox_status result = kStatusFailure;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    g_lidars->lidars_[handle].handle = handle;
    g_lidars->lidars_[handle].connect_state = kConnectStateOff;
  } else {
    printf("Add lidar to connect is failed : %d %d \n", result, handle);
  }
}

void LdsLidar::SetRmcSyncTime(const char* rmc, uint16_t rmc_length) {
  printf("Rmc: %s\n",rmc);
  std::unique_lock<std::mutex> lock(mtx);
  LidarDevice* p_lidar = nullptr;
  for (uint8_t handle = 0; handle < kMaxLidarCount; handle++) {
    p_lidar = &(g_lidars->lidars_[handle]);
    if (p_lidar->connect_state != kConnectStateOff && p_lidar->info.state == kLidarStateNormal) {
      livox_status status = LidarSetRmcSyncTime(handle, rmc, rmc_length, \
        LidarSetRmcSyncTimeCb, g_lidars);
      if (status != kStatusSuccess) {
        printf("Lidar set GPRMC synchronization time error code: %d.\n",status);
      }
    }
  }
}

/** Callback function of set synchronization time. */
void LdsLidar::LidarSetRmcSyncTimeCb(livox_status status, \
    uint8_t handle,uint8_t response, void* client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  printf("OnLidarSetRmcSyncTimeCallback statue %d handle %d response %d. \n", status, handle, response);
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
  std::unique_lock<std::mutex> lock(mtx);
  LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, DeviceInformationCb, g_lidars);
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    p_lidar->connect_state = kConnectStateOff;
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }
  if (p_lidar->connect_state == kConnectStateOn) {
    printf("Device Working State %d\n", p_lidar->info.state);
    if (p_lidar->info.state == kLidarStateInit) {
      printf("Device State Change Progress %u\n", p_lidar->info.status.progress);
    } else {
      printf("Device State Error Code 0X%08x\n", p_lidar->info.status.status_code.error_code);
    }
    printf("Device feature %d\n", p_lidar->info.feature);
    SetErrorMessageCallback(handle, LdsLidar::LidarErrorStatusCb);
  }
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle, \
                         DeviceInformationResponse *ack, void *clent_data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed : %d\n", status);
  }
  if (ack) {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

/** Callback function of Lidar error message. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message) {
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
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

/** Add broadcast code to whitelist */
int LdsLidar::AddBroadcastCodeToWhitelist(const char* bd_code) {
  if (!bd_code || (strlen(bd_code) > kBroadcastCodeSize) || \
      (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (LdsLidar::FindInWhitelist(bd_code)) {
    printf("%s is alrealy exist!\n", bd_code);
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], bd_code);
  ++whitelist_count_;

  return 0;
}

void LdsLidar::AddLocalBroadcastCode(void) {
  for (size_t i=0; i<sizeof(local_broadcast_code_list)/sizeof(intptr_t); ++i) {
    std::string invalid_bd = "000000000";
    printf("Local broadcast code : %s\n", local_broadcast_code_list[i]);
    if ((kBroadcastCodeSize == strlen(local_broadcast_code_list[i]) - 1) && \
        (nullptr == strstr(local_broadcast_code_list[i], invalid_bd.c_str()))) {
      LdsLidar::AddBroadcastCodeToWhitelist(local_broadcast_code_list[i]);
    } else {
      printf("Invalid local broadcast code : %s\n", local_broadcast_code_list[i]);
    }
  }
}

bool LdsLidar::FindInWhitelist(const char* bd_code) {
  if (!bd_code) {
    return false;
  }

  for (uint32_t i=0; i<whitelist_count_; i++) {
    if (strncmp(bd_code, broadcast_code_whitelist_[i], kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}


