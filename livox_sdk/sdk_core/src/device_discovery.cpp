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

#include "device_discovery.h"
#include <algorithm>
#include <mutex>
#include <iostream>
#include <vector>
#include "base/logging.h"
#include "base/network/network_util.h"
#include "command_handler/command_impl.h"
#include "device_manager.h"
#include "livox_def.h"

using std::tuple;
using std::string;
using std::vector;
using std::chrono::steady_clock;


namespace livox {

uint16_t DeviceDiscovery::port_count = 0;

bool DeviceDiscovery::Init() {
  if (comm_port_ == NULL) {
    comm_port_.reset(new CommPort());
  }
  return true;
}

bool DeviceDiscovery::Start(std::weak_ptr<IOLoop> loop) {
  if (loop.expired()) {
    return false;
  }
  loop_ = loop;
  sock_ = util::CreateSocket(kListenPort);
  if (sock_ < 0) {
    return false;
  }
  loop_.lock()->AddDelegate(sock_, this);
  return true;
}

void DeviceDiscovery::OnData(socket_t sock, void *) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);
  uint32_t buf_size = 0;
  uint8_t *cache_buf = comm_port_->FetchCacheFreeSpace(&buf_size);
  int size = buf_size;
  size = util::RecvFrom(sock, reinterpret_cast<char *>(cache_buf), buf_size, 0, &addr, &addrlen);
  if (size < 0) {
    return;
  }

  comm_port_->UpdateCacheWrIdx(size);
  CommPacket packet;
  memset(&packet, 0, sizeof(packet));

  while ((kParseSuccess == comm_port_->ParseCommStream(&packet))) {
    if (packet.cmd_set == kCommandSetGeneral && packet.cmd_code == kCommandIDGeneralBroadcast) {
      OnBroadcast(packet, &addr);
    } else if (packet.cmd_set == kCommandSetGeneral && packet.cmd_code == kCommandIDGeneralHandshake) {
      if (connecting_devices_.find(sock) == connecting_devices_.end()) {
        continue;
      }
      DeviceInfo info = std::get<1>(connecting_devices_[sock]);
      if (!loop_.expired()) {
        loop_.lock()->RemoveDelegate(sock, this);
      }
      util::CloseSock(sock);
      connecting_devices_.erase(sock);

      if (packet.data == NULL) {
        continue;
      }
      if (*(uint8_t *)packet.data == 0) {
        LOG_INFO("New Device");
        LOG_INFO("Handle: {}", static_cast<uint16_t>(info.handle));
        LOG_INFO("Broadcast Code: {}", info.broadcast_code);
        LOG_INFO("Type: {}", info.type);
        LOG_INFO("IP: {}", info.ip);
        LOG_INFO("Command Port: {}", info.cmd_port);
        LOG_INFO("Data Port: {}", info.data_port);
        DeviceFound(info);
      }
    }
  }
}

void DeviceDiscovery::OnTimer(TimePoint now) {
  ConnectingDeviceMap::iterator ite = connecting_devices_.begin();
  while (ite != connecting_devices_.end()) {
    tuple<TimePoint, DeviceInfo> &device_tuple = ite->second;
    if (now - std::get<0>(device_tuple) > std::chrono::milliseconds(500)) {
      if (!loop_.expired()) {
        loop_.lock()->RemoveDelegate(ite->first, this);
      }
      util::CloseSock(ite->first);
      connecting_devices_.erase(ite++);
    } else {
      ++ite;
    }
  }
}

void DeviceDiscovery::Uninit() {
  if (sock_ > 0) {
    if (!loop_.expired()) {
      loop_.lock()->RemoveDelegate(sock_, this);
    }
    util::CloseSock(sock_);
    sock_ = -1;
  }

  if (comm_port_) {
    comm_port_.reset(NULL);
  }
}

void DeviceDiscovery::OnBroadcast(const CommPacket &packet,  struct sockaddr *addr) {
  if (packet.data == NULL) {
    return;
  }

  BroadcastDeviceInfo device_info;
  memcpy((void*)(&device_info),(void*)(packet.data),(sizeof(BroadcastDeviceInfo)-sizeof(device_info.ip)));
  string broadcast_code = device_info.broadcast_code;
  LOG_INFO(" Broadcast broadcast code: {}", broadcast_code);

  char ip[16];
  memset(&ip, 0, sizeof(ip));
  inet_ntop(AF_INET, &((struct sockaddr_in*)addr)->sin_addr, ip, INET_ADDRSTRLEN);
  strncpy(device_info.ip, ip, sizeof(device_info.ip));
  
  device_manager().BroadcastDevices(&device_info);
 
  DeviceInfo lidar_info;
  bool found = device_manager().FindDevice(broadcast_code, lidar_info);

  if (!found) {
    LOG_INFO("Broadcast code : {} not add to connect", broadcast_code);
  }

  if (!found || device_manager().IsDeviceConnected(lidar_info.handle)) {
    return;
  }

  ++port_count;
  strncpy(lidar_info.broadcast_code, broadcast_code.c_str(), sizeof(lidar_info.broadcast_code)-1);
  lidar_info.cmd_port = kListenPort + kCmdPortOffset + port_count;
  lidar_info.data_port = kListenPort + kDataPortOffset + port_count;
  lidar_info.sensor_port = kListenPort + kSensorPortOffset + port_count;
  lidar_info.type = device_info.dev_type;
  lidar_info.state = kLidarStateUnknown;
  lidar_info.feature = kLidarFeatureNone;
  lidar_info.status.progress = 0;

  strncpy(lidar_info.ip, ip, sizeof(lidar_info.ip));

  socket_t cmd_sock = util::CreateSocket(lidar_info.cmd_port);
  if (cmd_sock < 0) {
    return;
  }
  if (!loop_.expired()) {
    loop_.lock()->AddDelegate(cmd_sock, this);
  }
  OnTimer(steady_clock::now());
  std::get<1>(connecting_devices_[cmd_sock]) = lidar_info;

  bool result = false;
  do {
    HandshakeRequest handshake_req;
    uint32_t local_ip = 0;
    if (util::FindLocalIp(*(struct sockaddr_in*)addr, local_ip) == false) {
      result = false;
      LOG_INFO("LocalIp and DeviceIp are not in same subnet");
      break;
    }
    LOG_INFO("LocalIP: {}", inet_ntoa(*(struct in_addr *)&local_ip));
    LOG_INFO("DeviceIP: {}", inet_ntoa(((struct sockaddr_in *)addr)->sin_addr));
    LOG_INFO("Command Port: {}", lidar_info.cmd_port);
    LOG_INFO("Data Port: {}", lidar_info.data_port);
    CommPacket packet;
    memset(&packet, 0, sizeof(packet));
    handshake_req.ip_addr = local_ip;
    handshake_req.cmd_port = lidar_info.cmd_port;
    handshake_req.data_port = lidar_info.data_port;
    handshake_req.sensor_port = lidar_info.sensor_port;
    packet.packet_type = kCommandTypeAck;
    packet.seq_num = CommandChannel::GenerateSeq();
    packet.cmd_set = kCommandSetGeneral;
    packet.cmd_code = kCommandIDGeneralHandshake;
    packet.data_len = sizeof(handshake_req);
    packet.data = (uint8_t *)&handshake_req;

    vector<uint8_t> buf(kMaxCommandBufferSize + 1);
    int o_len = kMaxCommandBufferSize;
    comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&o_len, packet);
    int byte_send = sendto(cmd_sock, reinterpret_cast<const char *>(buf.data()), o_len, 0, addr, sizeof(*addr));
    if (byte_send < 0) {
      return;
    }
    std::get<0>(connecting_devices_[cmd_sock]) = steady_clock::now();
    result = true;
  } while (0);

  if (result == false) {
    if (!loop_.expired()) {
      loop_.lock()->RemoveDelegate(cmd_sock, this);
    }
    util::CloseSock(cmd_sock);
    connecting_devices_.erase(cmd_sock);
  }
}


DeviceDiscovery &device_discovery() {
  static DeviceDiscovery discovery;
  return discovery;
}

}  // namespace livox
