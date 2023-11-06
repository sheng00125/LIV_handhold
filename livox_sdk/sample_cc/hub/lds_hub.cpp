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

#include "lds_hub.h"

#include <stdio.h>
#include <string.h>
#include <thread>
#include <memory>


/** Const varible ------------------------------------------------------------------------------- */
/** User add broadcast code here */
static const char* local_broadcast_code_list[] = {
  "000000000000001",
};


/** For callback use only */
static LdsHub* g_lds_hub = nullptr;


/** Global function for common use ---------------------------------------------------------------*/

/** Lds hub function -----------------------------------------------------------------------------*/
LdsHub::LdsHub() {
  auto_connect_mode_ = true;
  whitelist_count_   = 0;
  is_initialized_    = false;

  lidar_count_       = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

  memset(lidars_, 0, sizeof(lidars_));
  for (uint32_t i=0; i<kMaxLidarCount; i++) {
    lidars_[i].handle = kMaxLidarCount;         /** Unallocated state */
    lidars_[i].connect_state = kConnectStateOff;
  }
  memset(&hub_, 0, sizeof(hub_));
  hub_.handle = kMaxLidarCount;
  hub_.connect_state = kConnectStateOff;
}

LdsHub::~LdsHub() {
}

int LdsHub::InitLdsHub(std::vector<std::string>& broadcast_code_strs) {

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

  SetBroadcastCallback(LdsHub::OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(LdsHub::OnDeviceInfoChange);

  /** Add commandline input broadcast code */
  for (auto input_str : broadcast_code_strs) {
    LdsHub::AddBroadcastCodeToWhitelist(input_str.c_str());
    printf("Cmdline input broadcast code : %s\n", input_str.c_str());
  }

  /** Add local broadcast code */
  LdsHub::AddLocalBroadcastCode();

  if (whitelist_count_) {
    LdsHub::DisableAutoConnectMode();
    printf("Disable auto connect mode!\n");

    printf("List all broadcast code in whiltelist:\n");
    for (uint32_t i=0; i<whitelist_count_; i++) {
      printf("%s\n", broadcast_code_whitelist_[i]);
    }
  } else {
    LdsHub::EnableAutoConnectMode();
    printf("No broadcast code was added to whitelist, swith to automatic connection mode!\n");
  }

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lds_hub == nullptr) {
    g_lds_hub = this;
  }
  is_initialized_= true;
  printf("Livox-SDK init success!\n");

  return 0;
}

int LdsHub::DeInitLdsHub(void) {

  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  Uninit();
  printf("Livox SDK Deinit completely!\n");

  return 0;
}

/** Static function in LdsLidar for callback */
void LdsHub::OnHubDataCb(uint8_t hub_handle, LivoxEthPacket *data,
                            uint32_t data_num, void *client_data) {

  LdsHub* lds_hub = static_cast<LdsHub *>(client_data);
  LivoxEthPacket* eth_packet = data;

  if (!data || !data_num) {
    return;
  }

  /** Caculate which lidar this eth packet data belong to */
  uint8_t handle = HubGetLidarHandle(eth_packet->slot, eth_packet->id);
  if (handle >= kMaxLidarCount) {
    return;
  }

  if (data) {
    lds_hub->receive_packet_count_++;
    if (0 == (lds_hub->receive_packet_count_ % 100)) {
      printf("Receive packet handle: %d count: %d\n", handle, lds_hub->receive_packet_count_);

      /** Parsing the timestamp and the point cloud data. */
      uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
      if(data ->data_type == kCartesian) {
        LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
      }else if ( data ->data_type == kSpherical) {
        LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
      }else if ( data ->data_type == kExtendCartesian) {
        LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
      }else if ( data ->data_type == kExtendSpherical) {
        LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
      }else if ( data ->data_type == kDualExtendCartesian) {
        LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
      }else if ( data ->data_type == kDualExtendSpherical) {
        LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
      }else if ( data ->data_type == kImu) {
        LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
      }else if ( data ->data_type == kTripleExtendCartesian) {
        LivoxTripleExtendRawPoint *p_point_data = (LivoxTripleExtendRawPoint *)data->data;
      }else if ( data ->data_type == kTripleExtendSpherical) {
        LivoxTripleExtendSpherPoint *p_point_data = (LivoxTripleExtendSpherPoint *)data->data;
      }
    }
  }
}


void LdsHub::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL) {
    return;
  }

  if (info->dev_type != kDeviceTypeHub) {
    printf("It's not a hub : %s\n", info->broadcast_code);
    return;
  }

  if (g_lds_hub->IsAutoConnectMode()) {
    printf("In automatic connection mode, will connect %s\n", info->broadcast_code);
  } else {
    if (!g_lds_hub->FindInWhitelist(info->broadcast_code)) {
      printf("Not in the whitelist, please add %s to if want to connect!\n",\
             info->broadcast_code);
      return;
    }
  }

  LidarDevice* p_hub = &g_lds_hub->hub_;
  if (p_hub->connect_state == kConnectStateOff) {
    livox_status result = kStatusFailure;
    uint8_t handle = 0;
    result = AddHubToConnect(info->broadcast_code, &handle);
    if (result == kStatusSuccess && handle < kMaxLidarCount) {
      SetDataCallback(handle, LdsHub::OnHubDataCb, (void *)g_lds_hub);
      p_hub->handle = handle;
      p_hub->connect_state = kConnectStateOff;
      //config hub's parameter
      p_hub->config.coordinate = kCoordinateCartesian;
    } else {
      printf("Add Hub to connect failed, result : %d, handle: %d.\n", result, handle);
    }
  }
}

/** Callback function of changing of device state. */
void LdsHub::OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == NULL) {
    return;
  }

  if (info->handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_hub = &g_lds_hub->hub_;
  if (type == kEventHubConnectionChange) {
    if (p_hub->connect_state == kConnectStateOff) {
      p_hub->connect_state = kConnectStateOn;
      p_hub->info = *info;
    }
    printf("[WARNING] Hub sn: [%s] Connection Change!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    p_hub->connect_state = kConnectStateOff;
    printf("[WARNING] Hub sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    p_hub->info = *info;
    printf("Hub sn [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (p_hub->connect_state == kConnectStateOn) {
    printf("Hub Working State %d\n", p_hub->info.state);
    if (p_hub->info.state == kLidarStateInit) {
      printf("Hub State Change Progress %u\n", p_hub->info.status.progress);
    } else {
      printf("Hub State Error Code 0X%08x\n", p_hub->info.status.status_code.error_code);
    }
    printf("Hub feature %d\n", p_hub->info.feature);
    SetErrorMessageCallback(p_hub->handle, LdsHub::HubErrorStatusCb);
    if (p_hub->info.state == kLidarStateNormal) {
      HubQueryLidarInformation(HubQueryLidarInfoCb, g_lds_hub);
    }
  }
}

void LdsHub::HubQueryLidarInfoCb(livox_status status, uint8_t handle, \
                                 HubQueryLidarInformationResponse *response,\
                                 void *client_data) {
  LdsHub* lds_hub = static_cast<LdsHub *>(client_data);
  if (handle >= kMaxLidarCount) {
    return;
  }

  if (status == kStatusSuccess) {
    if (response->count) {
      printf("Hub have %d lidars:\n", response->count);
      for (int i = 0; i < response->count; i++) {
        uint32_t index = (response->device_info_list[i].slot - 1) * 3 +\
                          response->device_info_list[i].id - 1;
        if (index < kMaxLidarCount) {
          LidarDevice* p_lidar = &lds_hub->lidars_[index];
          p_lidar->handle = index;
          p_lidar->info.handle = index;
          p_lidar->info.slot = response->device_info_list[i].slot;
          p_lidar->info.id   = response->device_info_list[i].id;
          p_lidar->info.type = response->device_info_list[i].dev_type;
          p_lidar->connect_state = kConnectStateSampling;
          strncpy(p_lidar->info.broadcast_code, \
                  response->device_info_list[i].broadcast_code, \
                  sizeof(p_lidar->info.broadcast_code));
          //config lidar's parameter
          p_lidar->config.enable_fan = true;
          p_lidar->config.return_mode = kStrongestReturn;
          p_lidar->config.imu_rate = kImuFreq200Hz;

          printf("[%d]%s DeviceType[%d] Slot[%d] Ver[%d.%d.%d.%d]\n", index, \
                 p_lidar->info.broadcast_code,\
                 p_lidar->info.type, p_lidar->info.slot,\
                 response->device_info_list[i].version[0],\
                 response->device_info_list[i].version[1],\
                 response->device_info_list[i].version[2],\
                 response->device_info_list[i].version[3]);
        }
      }
      LdsHub::ConfigLidarsOfHub(lds_hub);
    } else {
      printf("Hub have no lidar, will not start sample!\n");
      HubQueryLidarInformation(HubQueryLidarInfoCb, lds_hub);
    }
  } else {
    printf("Device Query Informations Failed %d\n", status);
    HubQueryLidarInformation(HubQueryLidarInfoCb, lds_hub);
  }
}

/** Callback function of hub error message. */
void LdsHub::HubErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("sync_status : %u\n", message->hub_error_code.sync_status);
      printf("temp_status : %u\n", message->hub_error_code.temp_status);
      printf("lidar_status :%u\n", message->hub_error_code.lidar_status);
      printf("lidar_link_status : %u\n", message->hub_error_code.lidar_link_status);
      printf("firmware_err : %u\n", message->hub_error_code.firmware_err);
      printf("system_status : %u\n", message->hub_error_code.system_status);
    }
  }
}

void LdsHub::ControlFanCb(livox_status status, uint8_t handle, \
                          uint8_t response, void *client_data) {

}

void LdsHub::HubSetPointCloudReturnModeCb(livox_status status, uint8_t handle, \
                                          HubSetPointCloudReturnModeResponse* response,\
                                          void *client_data) {
  LdsHub* lds_hub = static_cast<LdsHub *>(client_data);
  if ((handle >= kMaxLidarCount)  || !response) {
    return;
  }

  if ((status != kStatusSuccess) || (response->ret_code)) {
    printf("Hub set return mode fail!\n");
    ConfigPointCloudReturnMode(lds_hub);
  } else {
    printf("Hub set return mode success!\n");
    lds_hub->hub_.config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    if (!lds_hub->hub_.config.set_bits) {
      HubStartSampling(LdsHub::StartSampleCb, lds_hub);
      lds_hub->hub_.connect_state = kConnectStateSampling;
    }
  }
}

void LdsHub::SetCoordinateCb(livox_status status, uint8_t handle, \
                             uint8_t response, void *client_data) {
  LdsHub* lds_hub = static_cast<LdsHub *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_hub = &(lds_hub->hub_);
  if (status == kStatusSuccess) {
    p_hub->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    printf("Set coordinate success!\n");

    if (!p_hub->config.set_bits) {
      HubStartSampling(LdsHub::StartSampleCb, lds_hub);
      p_hub->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_hub->config.coordinate != 0) {
      SetSphericalCoordinate(handle, LdsHub::SetCoordinateCb, lds_hub);
    } else {
      SetCartesianCoordinate(handle, LdsHub::SetCoordinateCb, lds_hub);
    }
    printf("Set coordinate fail, try again!\n");
  }
}

void LdsHub::HubSetImuRatePushFrequencyCb(livox_status status, uint8_t handle, \
                                          HubSetImuPushFrequencyResponse* response,\
                                          void *client_data) {
  LdsHub* lds_hub = static_cast<LdsHub *>(client_data);
  if ((handle >= kMaxLidarCount) || !response) {
    return;
  }

  if ((status != kStatusSuccess) || (response->ret_code)) {
    printf("Hub set imu freq fail [%d]!\n", response->ret_code);
    ConfigImuPushFrequency(lds_hub);
  } else {
    printf("Hub set imu frequency success!\n");
    lds_hub->hub_.config.set_bits &= ~((uint32_t)(kConfigImuRate));
    if (!lds_hub->hub_.config.set_bits) {
      HubStartSampling(LdsHub::StartSampleCb, lds_hub);
      lds_hub->hub_.connect_state = kConnectStateSampling;
    }
  }
}

/** Callback function of starting sampling. */
void LdsHub::StartSampleCb(livox_status status, uint8_t handle, \
                           uint8_t response, void *client_data) {
  LdsHub* lds_hub = static_cast<LdsHub *>(client_data);
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_hub = &lds_hub->hub_;
  if ((status != kStatusSuccess) || (response != 0)) {
    p_hub->connect_state = kConnectStateOn;
    printf("Hub start sample fail : state[%d] handle[%d] res[%d]\n", \
           status, handle, response);

    for (int i = 0; i < kMaxLidarCount; i++) {
      LidarDevice* p_lidar = &(lds_hub->lidars_[i]);
      if (p_lidar->connect_state == kConnectStateSampling) {
        p_lidar->connect_state = kConnectStateOn;
      }
    }
  } else {
    printf("Hub start sample success!\n");
  }
}

/** Callback function of stopping sampling. */
void LdsHub::StopSampleCb(livox_status status, uint8_t handle, \
                          uint8_t response, void *client_data) {
}

void LdsHub::ConfigPointCloudReturnMode(LdsHub* lds_hub) {
   uint8_t req_buf[1024];
   HubSetPointCloudReturnModeRequest* req = (HubSetPointCloudReturnModeRequest *)req_buf;
   req->count = 0;
   for (int i = 0; i < kMaxLidarCount; i++) {
     LidarDevice* p_lidar = &(lds_hub->lidars_[i]);

     if ((p_lidar->info.type != kDeviceTypeLidarMid40) && \
         (p_lidar->connect_state == kConnectStateSampling)) {
       strncpy(req->lidar_cfg_list[req->count].broadcast_code, \
               p_lidar->info.broadcast_code, \
               sizeof(req->lidar_cfg_list[req->count].broadcast_code));
       req->lidar_cfg_list[req->count].mode = p_lidar->config.return_mode;
       req->count++;
     }
   }

   if (req->count) {
     uint32_t length = sizeof(HubSetPointCloudReturnModeRequest) + sizeof(SetPointCloudReturnModeRequestItem) * (req->count - 1);
     HubSetPointCloudReturnMode(req, length,\
                                LdsHub::HubSetPointCloudReturnModeCb, lds_hub);
     lds_hub->hub_.config.set_bits |= kConfigReturnMode;
  }
}

void LdsHub::ConfigImuPushFrequency(LdsHub* lds_hub) {
  uint8_t req_buf[1024];
  HubSetImuPushFrequencyRequest* req = (HubSetImuPushFrequencyRequest *)req_buf;
  req->count = 0;
  for (int i = 0; i < kMaxLidarCount; i++) {
    LidarDevice* p_lidar = &(lds_hub->lidars_[i]);

    if ((p_lidar->info.type != kDeviceTypeLidarMid40) && \
        (p_lidar->connect_state == kConnectStateSampling)) {
      strncpy(req->lidar_cfg_list[req->count].broadcast_code, \
              p_lidar->info.broadcast_code, \
              sizeof(req->lidar_cfg_list[req->count].broadcast_code));
      req->lidar_cfg_list[req->count].freq = p_lidar->config.imu_rate;
      req->count++;
    }
  }

  if (req->count) {
    uint32_t length = sizeof(HubSetImuPushFrequencyRequest) + sizeof(SetImuPushFrequencyRequestItem) * (req->count -1);
    HubSetImuPushFrequency(req, length,\
                           LdsHub::HubSetImuRatePushFrequencyCb, lds_hub);
    lds_hub->hub_.config.set_bits |= kConfigImuRate;
  }
}

void LdsHub::ConfigLidarsOfHub(LdsHub* lds_hub) {
  ConfigPointCloudReturnMode(lds_hub);
  ConfigImuPushFrequency(lds_hub);

  if (lds_hub->hub_.config.coordinate != 0) {
    SetSphericalCoordinate(lds_hub->hub_.handle, LdsHub::SetCoordinateCb, lds_hub);
    printf("Hub set coordinate spherical\n");
  } else {
    printf("Hub set coordinate cartesian\n");
    SetCartesianCoordinate(lds_hub->hub_.handle, LdsHub::SetCoordinateCb, lds_hub);
  }
  lds_hub->hub_.config.set_bits |= kConfigCoordinate;

  lds_hub->hub_.connect_state = kConnectStateConfig;
}

/** Add broadcast code to whitelist */
int LdsHub::AddBroadcastCodeToWhitelist(const char* broadcast_code) {
  if (!broadcast_code || (strlen(broadcast_code) > kBroadcastCodeSize) || \
      (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (LdsHub::FindInWhitelist(broadcast_code)) {
    printf("%s is alrealy exist!\n", broadcast_code);
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], broadcast_code);
  ++whitelist_count_;

  return 0;
}

void LdsHub::AddLocalBroadcastCode(void) {
  for (size_t i = 0; i < sizeof(local_broadcast_code_list)/sizeof(intptr_t); ++i) {
    std::string invalid_bd = "000000000";
    printf("Local broadcast code : %s\n", local_broadcast_code_list[i]);
    if ((kBroadcastCodeSize == strlen(local_broadcast_code_list[i]) + 1) && \
        (nullptr == strstr(local_broadcast_code_list[i], invalid_bd.c_str()))) {
      AddBroadcastCodeToWhitelist(local_broadcast_code_list[i]);
    } else {
      printf("Invalid local broadcast code : %s\n", local_broadcast_code_list[i]);
    }
  }
}

bool LdsHub::FindInWhitelist(const char* broadcast_code) {
  if (!broadcast_code) {
    return false;
  }

  for (uint32_t i = 0; i < whitelist_count_; i++) {
    if (strncmp(broadcast_code, broadcast_code_whitelist_[i], kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

/** Get and update LiDAR info */
void LdsHub::UpdateHubLidarinfo(void) {
  DeviceInfo *_lidars = (DeviceInfo *) malloc(sizeof(DeviceInfo) * kMaxLidarCount);

  uint8_t count = kMaxLidarCount;
  uint8_t status = GetConnectedDevices(_lidars, &count);
  if (status == kStatusSuccess) {
    printf("Hub have lidars : \n");
    int i = 0;
    for (i = 0; i < count; ++i) {
      uint8_t handle = _lidars[i].handle;
      if (handle < kMaxLidarCount) {
        lidars_[handle].handle = handle;
        lidars_[handle].info = _lidars[i];
        lidars_[handle].connect_state = kConnectStateSampling;
        printf("[%d] : %s\r\n", _lidars[i].handle, _lidars[i].broadcast_code);
      }
    }
  }
  if (_lidars) {
    free(_lidars);
  }
}


