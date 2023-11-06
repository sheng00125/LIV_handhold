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

#ifndef LIVOX_DATA_HANDLER_H_
#define LIVOX_DATA_HANDLER_H_

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include "base/io_thread.h"
#include "device_manager.h"

namespace livox {
class DataHandlerImpl;

class DataHandler : public noncopyable {
 public:
  typedef std::function<void(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data)> DataCallback;

 public:
  DataHandler() {}

  bool Init();
  void Uninit();

  bool AddDevice(const DeviceInfo &info);
  void RemoveDevice(uint8_t handle);

  bool AddDataListener(uint8_t handle, const DataCallback &cb, void *client_data);
  void OnDataCallback(uint8_t handle, void *data, uint16_t size);

 private:
  std::array<DataCallback, kMaxConnectedDeviceNum> callbacks_;
  std::array<void *, kMaxConnectedDeviceNum> client_data_;
  std::unique_ptr<DataHandlerImpl> impl_;
};

class DataHandlerImpl : public IOLoop::IOLoopDelegate {
 public:
  DataHandlerImpl(DataHandler *handler) : handler_(handler) {}
  virtual ~DataHandlerImpl() {}

  virtual bool Init() = 0;
  virtual void Uninit(){};

  virtual bool AddDevice(const DeviceInfo &info) = 0;
  virtual void RemoveDevice(uint8_t handle) = 0;
  void OnDataCallback(uint8_t handle, void *data, uint16_t size) {
    if (handler_) {
      handler_->OnDataCallback(handle, data, size);
    }
  }

 protected:
  static const size_t kMaxBufferSize = 8192;
  DataHandler *handler_;
};

DataHandler &data_handler();

}  // namespace livox

#endif  // LIVOX_DATA_HANDLER_H_
