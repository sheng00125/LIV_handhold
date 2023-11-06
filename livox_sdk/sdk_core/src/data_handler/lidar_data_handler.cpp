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

#include "lidar_data_handler.h"
#include <mutex>
#include "base/network/network_util.h"
#ifdef WIN32
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <unistd.h>
#endif // WIN32


using std::lock_guard;
using std::mutex;
using std::shared_ptr;
using std::list;

namespace livox {

bool LidarDataHandlerImpl::Init() {
  return true;
}

void LidarDataHandlerImpl::Uninit() {
  for (list<DeviceItem>::iterator ite = devices_.begin(); ite != devices_.end(); ++ite) {
    DeviceItem &item = *ite;
    if (item.thread) {
      item.thread->loop().lock()->RemoveDelegate(item.sock, this);
      item.thread->Quit();
      item.thread->Join();
      item.thread->Uninit();
    }

    if (item.sock > 0) {
      util::CloseSock(item.sock);
    }
  }

  devices_.clear();
}

bool LidarDataHandlerImpl::AddDevice(const DeviceInfo &info) {
  socket_t sock = util::CreateSocket(info.data_port);

  std::shared_ptr<IOThread> thread = std::make_shared<IOThread>();
  thread->Init(false, false);
  thread->loop().lock()->AddDelegate(sock, this, reinterpret_cast<void *>(info.handle));
  {
    lock_guard<mutex> lock(mutex_);
    DeviceItem item = {sock, thread, info.handle};
    devices_.push_back(item);
  }
  return thread->Start();
}

void LidarDataHandlerImpl::RemoveDevice(uint8_t handle) {
  DeviceItem item;
  bool found = false;
  {
    lock_guard<mutex> lock(mutex_);
    for (list<DeviceItem>::iterator ite = devices_.begin(); ite != devices_.end(); ++ite) {
      if (ite->handle == handle) {
        item = *ite;
        found = true;
        devices_.erase(ite);
        break;
      }
    }
  }

  if (found) {
    item.thread->loop().lock()->RemoveDelegate(item.sock, this);
    item.thread->Quit();
    item.thread->Join();
    item.thread->Uninit();
    if (item.sock > 0) {
      util::CloseSock(item.sock);
    }
  }
}

void LidarDataHandlerImpl::OnData(socket_t sock, void *client_data) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);
  uint8_t handle = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(client_data));
  if (handle > data_buffers_.size()) {
    return;
  }
  std::unique_ptr<char[]> &buf = data_buffers_[handle];
  if (buf.get() == NULL) {
    buf.reset(new char[kMaxBufferSize]);
  }

  int size = kMaxBufferSize;
  size = util::RecvFrom(sock, reinterpret_cast<char *>(buf.get()), kMaxBufferSize, 0, &addr, &addrlen);
  if (size <= 0) {
    return;
  }
  if (handler_) {
      handler_->OnDataCallback(handle, buf.get(), size);
  }

}

}  // namespace livox
