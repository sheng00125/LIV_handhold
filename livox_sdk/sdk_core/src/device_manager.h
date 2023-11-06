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

#ifndef LIVOX_DEVICE_MANAGER_H_
#define LIVOX_DEVICE_MANAGER_H_

#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <string.h>
#include "device_discovery.h"
#include "livox_sdk.h"

namespace livox {

/** Maximum number of connected devices supported. */
static const uint8_t kMaxConnectedDeviceNum = 32;
/** Default handle value of hub. */
static const uint8_t kHubDefaultHandle = kMaxConnectedDeviceNum - 1;

/**
 * Device connected mode.
 */
typedef enum {
  kDeviceModeNone = 0, /**< no devices connected. */
  kDeviceModeHub = 1,  /**< connected with hub. */
  kDeviceModeLidar = 2 /**< connected with lidar. */
} DeviceMode;

/**
 * DeviceManager manage all the devices connected, including hub and lidars.
 */
class DeviceManager : public noncopyable {
 public:
  DeviceManager() : device_mode_(kDeviceModeNone), connected_cb_(NULL), broadcast_cb_(NULL) {}

  bool Init();
  void Uninit();

  /**
   * When a new device is successfully connected, AddDevice is called to add new device to DeviceManager.
   * @param device connected device information.
   * @return true if successfully added.
   */
  bool AddDevice(const DeviceInfo &device);

  /**
   * When device is disconnected (not response to the heartbeat command), RemoveDevice is called.
   * @param handle connected device handle.
   */
  void RemoveDevice(uint8_t handle);

  /**
   * Find device via handle.
   */
  bool FindDevice(uint8_t handle, DeviceInfo &info);

  /**
   * Find device via broadcast code.
   */
  bool FindDevice(const std::string &broadcast_code, DeviceInfo &info);

  /**
   * Check whether a device is connected.
   * @param handle device handle.
   * @return true if device is connected.
   */
  bool IsDeviceConnected(uint8_t handle);

  void SetDeviceConnectedCallback(const std::function<void(const DeviceInfo *, DeviceEvent)> &cb) {
    connected_cb_ = cb;
  }
  void SetDeviceBroadcastCallback(const std::function<void(const BroadcastDeviceInfo *info)> &cb) {
    broadcast_cb_ = cb;
  }
  void HubLidarInfomationCallback(livox_status status, uint8_t handle, HubQueryLidarInformationResponse *response);
  void QueryDeviceInformationCallback(livox_status status, uint8_t handle, DeviceInformationResponse *response);
  DeviceMode device_mode() { return device_mode_; }
  void BroadcastDevices(const BroadcastDeviceInfo *info);
  void UpdateDevices(const DeviceInfo &device, DeviceEvent type);
  void UpdateDeviceState(uint8_t handle, const HeartbeatResponse &response);
  void GetConnectedDevices(std::vector<DeviceInfo> &devices);
  bool AddListeningDevice(const std::string &broadcast_code, DeviceMode mode, uint8_t &handle);
  bool IsLidarMid40(uint8_t handle);
  bool IsLidarMid70(uint8_t handle);
  bool IsLidarTele(uint8_t handle);
  bool IsLidarAvia(uint8_t handle);
  bool GetLidarFirmwareVersion(uint8_t handle, uint32_t &firmware_version);

 private:
  typedef struct _DetailDeviceInfo {
    bool connected;
    DeviceInfo info;
    _DetailDeviceInfo() { connected = false; }
    _DetailDeviceInfo(bool _connected,
                      const char *broadcast_code,
                      uint8_t handle,
                      uint8_t port,
                      uint8_t id,
                      uint32_t type,
                      uint16_t data_port,
                      uint16_t cmd_port,
                      const char *ip,
                      const uint8_t *firmware_version) {
      connected = _connected;
      strncpy(info.broadcast_code, broadcast_code, sizeof(info.broadcast_code));
      info.handle = handle;
      info.slot = port;
      info.id = id;
      info.type = type;
      info.data_port = data_port;
      info.cmd_port = cmd_port;
      strncpy(info.ip, ip, sizeof(info.ip));
      memcpy(info.firmware_version, firmware_version, sizeof(info.firmware_version));
    }
    void clear() {
      connected = false;
      memset(info.broadcast_code, 0, sizeof(info.broadcast_code));
      info.handle = 0;
      info.slot = 0;
      info.id = 0;
      info.type = 0;
      info.data_port = 0;
      info.cmd_port = 0;
      memset(info.ip, 0, sizeof(info.ip));
      memset(info.firmware_version, 0, sizeof(info.firmware_version));
    };
  } DetailDeviceInfo;
  typedef std::array<DetailDeviceInfo, kMaxConnectedDeviceNum> DeviceContainer;
  DeviceContainer devices_;
  DeviceMode device_mode_;
  std::function<void(const DeviceInfo *, DeviceEvent)> connected_cb_;
  std::function<void(const BroadcastDeviceInfo *info)> broadcast_cb_;
  std::mutex mutex_;
};

DeviceManager &device_manager();
void DeviceFound(const DeviceInfo &data);
void DeviceRemove(uint8_t handle,DeviceEvent device_event);

}  // namespace livox

#endif  // LIVOX_DEVICE_MANAGER_H_
