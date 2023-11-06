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

#include "data_handler.h"
#include <base/logging.h>
#include "hub_data_handler.h"
#include "lidar_data_handler.h"

namespace livox {

static const size_t kPrefixDataSize = 18;

DataHandler &data_handler() {
  static DataHandler handler;
  return handler;
}

bool DataHandler::AddDataListener(uint8_t handle, const DataCallback &cb, void *client_data) {
  if (handle >= callbacks_.size()) {
    return false;
  }
  client_data_[handle] = client_data;
  callbacks_[handle] = cb;
  return true;
}

bool DataHandler::AddDevice(const DeviceInfo &info) {
  if (impl_ == NULL) {
    DeviceMode mode = static_cast<DeviceMode>(device_manager().device_mode());
    if (mode == kDeviceModeHub) {
      impl_.reset(new HubDataHandlerImpl(this));
    } else if (mode == kDeviceModeLidar) {
      impl_.reset(new LidarDataHandlerImpl(this));
    }

    if (impl_ == NULL || !impl_->Init()) {
      impl_.reset(NULL);
      return false;
    }
  }
  return impl_->AddDevice(info);
}

bool DataHandler::Init() {
  return true;
}

void DataHandler::Uninit() {
  if (impl_) {
    impl_.reset(NULL);
  }
}

void DataHandler::OnDataCallback(uint8_t handle, void *data, uint16_t size) {
  LivoxEthPacket *lidar_data = (LivoxEthPacket *)data;
  if (lidar_data == NULL) {
    return;
  }
  if (handle >= callbacks_.size()) {
    return;
  }
  DataCallback cb = callbacks_[handle];
  switch (lidar_data->data_type) {
    case kCartesian:
      size = (size - kPrefixDataSize) / sizeof(LivoxRawPoint);
      break;
    case kSpherical:
      size = (size - kPrefixDataSize) / sizeof(LivoxSpherPoint);
      break;
    case kExtendCartesian:
      size = (size - kPrefixDataSize) / sizeof(LivoxExtendRawPoint);
      break;
    case kExtendSpherical:
      size = (size - kPrefixDataSize) / sizeof(LivoxExtendSpherPoint);
      break;
    case kDualExtendCartesian:
      size = (size - kPrefixDataSize) / sizeof(LivoxDualExtendRawPoint);
      break;
    case kDualExtendSpherical:
      size = (size - kPrefixDataSize) / sizeof(LivoxDualExtendSpherPoint);
      break;
    case kImu:
      size = (size - kPrefixDataSize) / sizeof(LivoxImuPoint);
      break;
    case kTripleExtendCartesian:
      size = (size - kPrefixDataSize) / sizeof(LivoxTripleExtendRawPoint);
      break;
    case kTripleExtendSpherical:
      size = (size - kPrefixDataSize) / sizeof(LivoxTripleExtendSpherPoint);
      break;
    default:
      return;
  }
  if (cb) {
    //LOG_INFO(" device_sn: {}",  device_sn);
    //LOG_INFO(" version: {}", (uint32_t)lidar_data->version);
    //LOG_INFO(" slot: {}", (uint32_t)lidar_data->slot);
    //LOG_INFO(" resverd : {}", (uint32_t)lidar_data->reserved);
    //LOG_INFO(" errorCode: {}", (uint32_t)lidar_data->err_code);
    //LOG_INFO(" lidarID: {}", (uint16_t)lidar_data->id);
    //LOG_INFO(" timeStampType: {}", (uint32_t) (lidar_data->timestamp_type));
    //LOG_INFO(" timeStamp: {}", (uint32_t) *(lidar_data->time_stamp));
    //LOG_INFO(" dataType: {}", (uint16_t) lidar_data->data_type);
    cb(handle, lidar_data, size, client_data_[handle]);
  }
}

void DataHandler::RemoveDevice(uint8_t handle) {
  if (impl_) {
    impl_->RemoveDevice(handle);
  }
}

}  // namespace livox
