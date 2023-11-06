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

#include "command_channel.h"
#include <functional>
#include <atomic>
#include "base/logging.h"
#include "base/network/network_util.h"
#include "command_impl.h"
#include "device_manager.h"
#include "livox_def.h"
#include <stdio.h>

using std::bind;
using std::list;
using std::make_pair;
using std::map;
using std::pair;
using std::string;
using std::chrono::steady_clock;

namespace livox {

CommandChannel::CommandChannel(uint16_t port,
                               uint8_t handle,
                               const string &remote_ip,
                               CommandChannelDelegate *cb)
    : handle_(handle),
      port_(port),
      loop_(),
      callback_(cb),
      comm_port_(new CommPort),
      remote_ip_(remote_ip),
      heartbeat_time_(),
      last_heartbeat_() {}


bool CommandChannel::Bind(std::weak_ptr<IOLoop> loop) {
  if (loop.expired()) {
    return false;
  }
  loop_ = loop;
  sock_ = util::CreateSocket(port_);
  if (sock_ == -1) {
    return false;
  }

  loop_.lock()->AddDelegate(sock_, this);
  last_heartbeat_ = steady_clock::now();
  return true;
}

void CommandChannel::OnData(socket_t , void *) {
  struct sockaddr addr;
  int addrlen = sizeof(addr);
  uint32_t buf_size = 0;
  uint8_t *cache_buf = comm_port_->FetchCacheFreeSpace(&buf_size);
  int size = buf_size;
  size = util::RecvFrom(sock_, reinterpret_cast<char *>(cache_buf), buf_size, 0, &addr, &addrlen);
  if (size <= 0) {
    return;
  }
  comm_port_->UpdateCacheWrIdx(size);
  CommPacket packet;
  memset(&packet, 0, sizeof(packet));

  while ((kParseSuccess == comm_port_->ParseCommStream(&packet))) {
    if (packet.packet_type == kCommandTypeAck) {
      uint16_t seq = packet.seq_num;
      if (commands_.find(seq) != commands_.end()) {
        Command command = commands_[seq].first;
        command.packet = packet;
        if (callback_) {
          callback_->OnCommand(handle_, command);
        }
        commands_.erase(seq);
      } else if (packet.cmd_set == kCommandSetGeneral && packet.cmd_code == kCommandIDGeneralHeartbeat) {
        OnHeartbeatAck(packet);
        if (callback_) {
          callback_->OnHeartbeatStateUpdate(handle_, *(reinterpret_cast<HeartbeatResponse *>(packet.data)));
        }
      }
    } else if (packet.packet_type == kCommandTypeMsg) {
      if (callback_) {
        Command command;
        command.packet = packet;
        callback_->OnCommand(handle_, command);
      }
    }
  }
}

void CommandChannel::SendAsync(const Command &command) {
  Command cmd = DeepCopy(command);
  if (!loop_.expired()) {
    auto w_ptr = WeakProtector(protector_);
    loop_.lock()->PostTask([this, w_ptr, cmd](){
      if(w_ptr.expired()) {
        return;
      }
      Send(cmd);
    });
  }
}

void CommandChannel::OnTimer(TimePoint now) {
  list<Command> timeout_commands;
  map<uint16_t, pair<Command, TimePoint> >::iterator ite = commands_.begin();
  while (ite != commands_.end()) {
    pair<Command, TimePoint> &command_pair = ite->second;
    if (now > command_pair.second) {
      timeout_commands.push_back(command_pair.first);
      commands_.erase(ite++);
    } else {
      ++ite;
    }
  }

  for (list<Command>::iterator ite = timeout_commands.begin(); ite != timeout_commands.end(); ++ite) {
    LOG_WARN("Command Timeout: Set {}, Id {}, Seq {}", 
        (uint16_t)ite->packet.cmd_set, ite->packet.cmd_code, ite->packet.seq_num);
    if (callback_) {
      ite->packet.packet_type = kCommandTypeAck;
      callback_->OnCommand(handle_, *ite);
    }
  }

  if (now - last_heartbeat_ > std::chrono::seconds(3)) {
    DeviceDisconnect(handle_);
  } else {
    HeartBeat(now);
  }
}

void CommandChannel::Uninit() {
  if (sock_ != -1) {
    if (!loop_.expired()) {
      loop_.lock()->RemoveDelegate(sock_, this);
    }
    util::CloseSock(sock_);
    sock_ = -1;
  }
  callback_ = NULL;
  if (comm_port_) {
    comm_port_.reset(NULL);
  }

  commands_.clear();
  last_heartbeat_ = {};
  heartbeat_time_ = {};
  remote_ip_ = "";
}

void CommandChannel::HeartBeat(TimePoint t) {
  if (heartbeat_time_ == TimePoint() || (t - heartbeat_time_) > kHeartbeatTimer) {
    heartbeat_time_ = t;

    Command command(handle_,
                    kCommandTypeCmd,
                    kCommandSetGeneral,
                    kCommandIDGeneralHeartbeat,
                    GenerateSeq(),
                    NULL,
                    0,
                    0,
                    std::shared_ptr<CommandCallback>());
    SendInternal(command);
  }
}

void CommandChannel::SendInternal(const Command &command) {
  std::vector<uint8_t> buf(kMaxCommandBufferSize + 1);
  int size = 0;
  comm_port_->Pack(buf.data(), kMaxCommandBufferSize, (uint32_t *)&size, command.packet);

  struct sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(65000);
  servaddr.sin_addr.s_addr = inet_addr(remote_ip_.c_str());

  int byte_send = sendto(sock_, (const char*)buf.data(), size, 0, (const struct sockaddr *) &servaddr,
            sizeof(servaddr));
  if (byte_send < 0) {
    if (command.cb) {
      (*command.cb)(kStatusSendFailed, handle_, NULL);
    }
  }
}

uint16_t CommandChannel::GenerateSeq() {
  static std::atomic<std::uint16_t> seq(1);
  uint16_t value = seq.load();
  uint16_t desired = 0;
  do {
    if (value == UINT16_MAX) {
      desired = 1;
    } else {
      desired = value + 1;
    }
  } while (!seq.compare_exchange_weak(value, desired));
  return desired;
}

Command CommandChannel::DeepCopy(const Command &cmd) {
  Command result_cmd(cmd);
  if (result_cmd.packet.data != NULL) {
    result_cmd.packet.data = new uint8_t[result_cmd.packet.data_len];
    memcpy(result_cmd.packet.data, cmd.packet.data, result_cmd.packet.data_len);
  }
  return result_cmd;
}

void CommandChannel::OnHeartbeatAck(const CommPacket &) {
  last_heartbeat_ = steady_clock::now();
}

void CommandChannel::DeviceDisconnect(uint8_t handle) {
  DeviceRemove(handle,kEventDisconnect);
}

void CommandChannel::Send(const Command &command) {
  LOG_INFO(" Send Command: Set {} Id {} Seq {}", (uint16_t)command.packet.cmd_set, command.packet.cmd_code, command.packet.seq_num);
  SendInternal(command);
  commands_[command.packet.seq_num] = make_pair(command, steady_clock::now() + std::chrono::milliseconds(command.time_out));
  Command &cmd = commands_[command.packet.seq_num].first;
  if (cmd.packet.data != NULL) {
    delete[] cmd.packet.data;
    cmd.packet.data = NULL;
    cmd.packet.data_len = 0;
  }
}
}  // namespace livox
