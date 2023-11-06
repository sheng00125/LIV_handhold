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

#include <thread>
#include <chrono>
#include <string.h>
#include <algorithm>
#include "lvx_file.h"
#include "cmdline.h"

DeviceItem devices[kMaxLidarCount];
LvxFileHandle lvx_file_handler;
std::list<LvxBasePackDetail> point_packet_list;
std::condition_variable condition_variable;
std::mutex mtx;
int lidar_units_index[32];
int lvx_file_save_time = 10;
bool is_finish_extrinsic_parameter = false;

#define FRAME_RATE 20

using namespace std::chrono;

/** Connect the first broadcast hub in default and connect specific device when use program options or broadcast_code_list is not empty. */
std::vector<std::string> broadcast_code_list = {
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
  if (data) {
    if (is_finish_extrinsic_parameter) {
      std::unique_lock<std::mutex> lock(mtx);
      LvxBasePackDetail packet;
      packet.device_index = lidar_units_index[HubGetLidarHandle(data->slot, data->id)];
      lvx_file_handler.BasePointsHandle(data, packet);
      point_packet_list.push_back(packet);
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

/** Callback function of get LiDAR units' extrinsic parameter. */
void OnGetLidarUnitsExtrinsicParameter(livox_status status, uint8_t handle, HubGetExtrinsicParameterResponse *response, void *data) {
  if (status == kStatusSuccess) {
    if (response != 0) {
      printf("OnGetLidarUnitsExtrinsicParameter statue %d handle %d response %d \n", status, handle, response->ret_code);
      std::unique_lock<std::mutex> lock(mtx);
      LvxDeviceInfo lidar_info;
      for (int i = 0; i < response->count; i++) {
        ExtrinsicParameterResponseItem temp;
        memcpy(&temp, (void *)(response->parameter_list + i), sizeof(ExtrinsicParameterResponseItem));
        strncpy((char *)lidar_info.lidar_broadcast_code, temp.broadcast_code, kBroadcastCodeSize);
        strncpy((char *)lidar_info.hub_broadcast_code, broadcast_code_list[0].c_str(), kBroadcastCodeSize);

        std::unique_ptr<DeviceInfo[]> device_list(new DeviceInfo[kMaxLidarCount]);
        std::unique_ptr<uint8_t> size(new uint8_t(kMaxLidarCount));
        GetConnectedDevices(device_list.get(), size.get());
        for (int j = 0; j < kMaxLidarCount; j++) {
          if (strncmp(device_list[j].broadcast_code, temp.broadcast_code, kBroadcastCodeSize) == 0) {
            lidar_units_index[device_list[j].handle] = i;
            lidar_info.device_index = i;
            break;
          }
        }

        lidar_info.device_type = devices[handle].info.type;
        lidar_info.extrinsic_enable = false;
        lidar_info.pitch = temp.pitch;
        lidar_info.roll = temp.roll;
        lidar_info.yaw = temp.yaw;
        lidar_info.x = static_cast<float>(temp.x / 1000.0);
        lidar_info.y = static_cast<float>(temp.y / 1000.0);
        lidar_info.z = static_cast<float>(temp.z / 1000.0);
        printf("index: %d\n",lidar_info.device_index);
        printf("pitch: %f\n", lidar_info.pitch);
        printf("roll: %f\n", lidar_info.roll);
        printf("yaw: %f\n", lidar_info.yaw);
        printf("x: %f\n", lidar_info.x);
        printf("y: %f\n", lidar_info.y);
        printf("z: %f\n", lidar_info.z);
        lvx_file_handler.AddDeviceInfo(lidar_info);
      }
      is_finish_extrinsic_parameter = true;
      condition_variable.notify_one();
    }
  }
  else if (status == kStatusTimeout) {
    printf("GetLidarUnitsExtrinsicParameter timeout! \n");
  }
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

void HubDeviceConnectionChange() {
  HubDeviceDisconnect();
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
  if (info == nullptr) {
    return;
  }
  printf("OnDeviceChange broadcast code %s update type %d\n", info->broadcast_code, type);
  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }
  if (type == kEventHubConnectionChange) {
    HubDeviceConnectionChange();
    printf("[WARNING] Hub sn: [%s] Connection Change!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    HubDeviceDisconnect();
    printf("[WARNING] Hub[%s] Disconnect!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    HubDeviceStateChange(info);
    printf("[WARNING] Hub sn [%s] StateChange!!!\n", info->broadcast_code);
  }
  if (devices[handle].device_state == kDeviceStateConnect) {
    printf("Hub Working State %d\n", devices[handle].info.state);
    if (devices[handle].info.state == kLidarStateInit) {
      printf("Hub State Change Progress %u\n", devices[handle].info.status.progress);
    } else {
      printf("Hub State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
    }
    printf("Hub feature %d\n", devices[handle].info.feature);
    SetErrorMessageCallback(handle, OnHubErrorStatusCallback);
    if (devices[handle].info.state == kLidarStateNormal) {
      HubGetExtrinsicParameter(OnGetLidarUnitsExtrinsicParameter, nullptr);
      HubStartSampling(OnSampleCallback, nullptr);
      devices[handle].device_state = kDeviceStateSampling;
    }
  }
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr || info->dev_type!= kDeviceTypeHub) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);

  if (broadcast_code_list.size() > 0) {
    bool found = false;
    uint8_t i = 0;
    for (i = 0; i < broadcast_code_list.size(); ++i) {
      if (strncmp(info->broadcast_code, broadcast_code_list[i].c_str(), kBroadcastCodeSize) == 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      return;
    }
  } else {
    broadcast_code_list.push_back(info->broadcast_code);
    return;
  }

  livox_status result = kStatusFailure;
  uint8_t handle = 0;
  result = AddHubToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess) {
    SetDataCallback(handle, GetHubData, nullptr);
    devices[handle].handle = handle;
    devices[handle].device_state = kDeviceStateDisconnect;
  }
}

/** Set the program options.
* You can input the registered device broadcast code and decide whether to save the log file.
*/
void SetProgramOption(int argc, const char *argv[]) {
  cmdline::parser cmd;
  cmd.add<std::string>("code", 'c', "Register device broadcast code", false);
  cmd.add("log", 'l', "Save the log file");
  cmd.add<int>("time", 't', "Time to save point cloud to the lvx file", false);
  cmd.add("help", 'h', "Show help");
  cmd.parse_check(argc, const_cast<char **>(argv));
  if (cmd.exist("code")) {
    std::string sn_list = cmd.get<std::string>("code");
    printf("Register broadcast code: %s\n", sn_list.c_str());
    broadcast_code_list.clear();
    broadcast_code_list.push_back(sn_list);
  }
  if (cmd.exist("log")) {
    printf("Save the log file.\n");
    SaveLoggerFile();
  }
  if (cmd.exist("time")) {
    printf("Time to save point cloud to the lvx file:%d.\n", cmd.get<int>("time"));
    lvx_file_save_time = cmd.get<int>("time");
  }
  return;
}

int main(int argc, const char *argv[]) {
/** Set the program options. */
  SetProgramOption(argc, argv);

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
  if (!Start()) {
    Uninit();
    return -1;
  }
  printf("Start discovering device.\n");

  {
    std::unique_lock<std::mutex> lock(mtx);
    condition_variable.wait(lock);
  }

  printf("Start initialize lvx file.\n");
  if (!lvx_file_handler.InitLvxFile()) {
    Uninit();
    return -1;
  }

  lvx_file_handler.InitLvxFileHeader();

  int i = 0;
  steady_clock::time_point last_time = steady_clock::now();
  for (i = 0; i < lvx_file_save_time * FRAME_RATE; ++i) {
    std::list<LvxBasePackDetail> point_packet_list_temp;
    {
      std::unique_lock<std::mutex> lock(mtx);
      condition_variable.wait_for(lock, milliseconds(kDefaultFrameDurationTime) - (steady_clock::now() - last_time));
      last_time = steady_clock::now();
      point_packet_list_temp.swap(point_packet_list);
    }
    if(point_packet_list_temp.empty()) {
      printf("Point cloud packet is empty.\n");
      break;
    }
    printf("Finish save %d frame to lvx file.\n", i);
    lvx_file_handler.SaveFrameToLvxFile(point_packet_list_temp);
  }

  lvx_file_handler.CloseLvxFile();

  HubStopSampling(OnStopSampleCallback, NULL);
  printf("Stop sample\n");

  Uninit();
}
