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

#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>
#include "livox_sdk.h"

typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
} DeviceItem;

DeviceItem devices[kMaxLidarCount];

#define BROADCAST_CODE_LIST_SIZE  1
/** Connect all the broadcast device. */
// bool is_connect_all_broadcast_device = true;
// char broadcast_code_list[BROADCAST_CODE_LIST_SIZE][kBroadcastCodeSize];

/** Connect the broadcast device in list, please input the broadcast code. */
/*bool is_connect_all_broadcast_device = false;
char broadcast_code_list[BROADCAST_CODE_LIST_SIZE][kBroadcastCodeSize] = {
  "00000000000001"
};*/
bool is_connect_all_broadcast_device = false;
char broadcast_code_list[BROADCAST_CODE_LIST_SIZE][kBroadcastCodeSize] = {
  "3JEDK1N00181497"
};

/** Receiving error message from Livox Hub. */
void OnHubErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message) {
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

/** Receiving point cloud data from Livox Hub. */
void GetHubData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
  static uint32_t receive_packet_count = 0;
  if (data) {
    ++receive_packet_count;
    if (0 == (receive_packet_count % 1000)) {
      printf("receive packet handle: %d count: %d\n", (uint16_t) HubGetLidarHandle(data->slot, data->id), receive_packet_count);

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
      printf("data_type %d data num %d\n", data->data_type, data_num);
    }
  }
}

/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
  printf("OnSampleCallback statue %d handle %d response %d \n", status, handle, response);
  if (status == kStatusSuccess) {
    if (response != 0) {
      devices[handle].device_state = kDeviceStateConnect;
    }
  } else if (status == kStatusTimeout) {
    devices[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {

}

/** Callback function of get LiDAR units' information. */
void OnHubLidarInfo(livox_status status, uint8_t handle, HubQueryLidarInformationResponse *response, void *client_data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed %d\n", status);
  }
  if (response) {
    int i = 0;
    for (i = 0; i < response->count; ++i) {
      printf("Hub Lidar Info broadcast code %s id %d slot %d \n ",
             response->device_info_list[i].broadcast_code,
             response->device_info_list[i].id,
             response->device_info_list[i].slot);
    }
  }
}

void HubDeviceDisconnect() {
  uint8_t handle = 0;
  for (handle = 0; handle < kMaxLidarCount; ++handle) {
    if (devices[handle].device_state == kDeviceStateConnect ) {
      devices[handle].device_state = kDeviceStateDisconnect;
    }
  }
}

void HubDeviceConnect(const DeviceInfo *info) {
  HubDeviceDisconnect();
  uint8_t handle = info->handle;
  DeviceInfo *_devices = (DeviceInfo *) malloc(sizeof(DeviceInfo) * kMaxLidarCount);
  uint8_t count = kMaxLidarCount;
  livox_status status = GetConnectedDevices(_devices, &count);
  if (status == kStatusSuccess) {
    int i = 0;
    for (i = 0; i < count; ++i) {
      uint8_t handle = _devices[i].handle;
      if (handle < kMaxLidarCount) {
        devices[handle].handle = handle;
        devices[handle].info = _devices[i];
        devices[handle].device_state = kDeviceStateConnect;
      }
    }
  }
  if (_devices) {
    free(_devices);
  }
  HubQueryLidarInformation(OnHubLidarInfo, NULL);
}

void HubDeviceStateChange(const DeviceInfo *info) {
  devices[info->handle].info = *info;
}

/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == NULL) {
    return;
  }
  printf("OnDeviceChange broadcast code %s update type %d\n", info->broadcast_code, type);
  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }
  if (type == kEventHubConnectionChange) {
    HubDeviceConnect(info);
    printf("[WARNING] Hub sn: [%s] Connection Change!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    HubDeviceDisconnect();
    printf("[WARNING] Hub sn [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    HubDeviceStateChange(info);
    printf("[WARNING] Hub sn [%s] StateChange!!!\n", info->broadcast_code);
  }
  if (devices[handle].device_state == kDeviceStateConnect) {
    SetErrorMessageCallback(handle, OnHubErrorStatusCallback);
    printf("Hub Working State %d\n", devices[handle].info.state);
    if (devices[handle].info.state == kLidarStateInit) {
      printf("Hub State Change Progress %u\n", devices[handle].info.status.progress);
    } else {
      printf("Hub State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
    }
    printf("Hub feature %d\n", devices[handle].info.feature);
    if (devices[handle].info.state == kLidarStateNormal) {
      HubStartSampling(OnSampleCallback, NULL);
      devices[handle].device_state = kDeviceStateSampling;
    }
  }
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL || info->dev_type != kDeviceTypeHub) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);

  if (!is_connect_all_broadcast_device) {
    bool found = false;
    int i = 0;
    for (i = 0; i < BROADCAST_CODE_LIST_SIZE; ++i) {
      if (strncmp(info->broadcast_code, broadcast_code_list[i], kBroadcastCodeSize) == 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddHubToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, GetHubData, NULL);
    devices[handle].handle = handle;
    devices[handle].device_state = kDeviceStateDisconnect;
  }
}

int main(int argc, const char *argv[]) {
  printf("Livox SDK initializing.\n");
  /** Initialize Livox-SDK. */
  if (!Init()) {
    return -1;
  }
  printf("Livox SDK has been initialized.\n");

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  memset(devices, 0, sizeof(devices));

/** Set the callback function receiving broadcast message from Livox LiDAR. */
  SetBroadcastCallback(OnDeviceBroadcast);

/** Set the callback function called when device state change,
 * which means connection/disconnection and changing of LiDAR state.
 */
  SetDeviceStateUpdateCallback(OnDeviceInfoChange);

/** Start the device discovering routine. */
  if (Start()) {
  printf("Start discovering device.\n");

#ifdef WIN32
    Sleep(20000);
#else
    sleep(20);
#endif
    HubStopSampling(OnStopSampleCallback, NULL);
    printf("Stop sample\n");
#ifdef WIN32
    Sleep(2000);
#else
    sleep(2);
#endif
  }

  Uninit();
}
