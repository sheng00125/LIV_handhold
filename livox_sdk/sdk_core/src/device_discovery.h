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

#ifndef LIVOX_DEVICE_DISCOVERY_
#define LIVOX_DEVICE_DISCOVERY_

#include <mutex>
#include <string>
#include <algorithm>
#include "base/io_thread.h"
#include "base/noncopyable.h"
#include "comm/comm_port.h"
#include "command_handler/command_channel.h"
#include <stdio.h>
#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <ws2def.h>
#pragma comment(lib,"ws2_32.lib")
#else
#include "arpa/inet.h"
#endif

namespace livox {

/**
 * DeviceDiscovery listens broadcast message from devices, and proceed handshake
 * if device is in the listening list.
 */
class DeviceDiscovery : public noncopyable, IOLoop::IOLoopDelegate {
 public:
  typedef std::chrono::steady_clock::time_point TimePoint;

  DeviceDiscovery() : comm_port_(nullptr) {}
  bool Init();
  void Uninit();

  /**
   * start the listening of broadcast UDP message.
   * @param loop IOLoop on where DeviceDiscovery runs.
   * @return true if successfully.
   */
  bool Start(std::weak_ptr<IOLoop> loop);

  /**
   * IOLoop callback delegate.
   * @param client_data client data passed in IOLoop::AddDelegate
   */
  void OnData(socket_t, void *client_data);
  void OnTimer(TimePoint now);

 private:
  void OnBroadcast(const CommPacket &packet, struct sockaddr *addr);

 private:
  /** broadcast listening port number. */
  static const uint16_t kListenPort = 55000;
  /** command port number start offset. */
  static const uint16_t kCmdPortOffset = 500;
  /** data port number start offset. */
  static const uint16_t kDataPortOffset = 1000;
    /** sensor port number start offset. */
  static const uint16_t kSensorPortOffset = 1000;


  static uint16_t port_count;
  socket_t sock_ = -1;
  std::weak_ptr<IOLoop> loop_;
  std::unique_ptr<CommPort> comm_port_;
  std::mutex mutex_;
  typedef std::map<socket_t, std::tuple<TimePoint, DeviceInfo> > ConnectingDeviceMap;

  ConnectingDeviceMap connecting_devices_;
};

DeviceDiscovery &device_discovery();

}  // namespace livox

#endif  // LIVOX_DEVICE_DISCOVERY_
