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

#include "command_handler.h"
#include <mutex>
#include "base/logging.h"
#include "command_impl.h"
#include "device_manager.h"
#include "hub_command_handler.h"
#include "lidar_command_handler.h"

using std::lock_guard;
using std::mutex;
using std::shared_ptr;
using std::list;
using std::make_pair;
using std::multimap;
using std::pair;

namespace livox {

CommandHandler &command_handler() {
  static CommandHandler handler;
  return handler;
}

uint16_t GetCommandTimeout(uint8_t command_set, uint8_t command_id) {
  switch (command_set) {
    case kCommandSetGeneral:
      assert(command_id < sizeof(GeneralCommandTimeout));
      return GeneralCommandTimeout[command_id];
    case kCommandSetLidar:
      assert(command_id < sizeof(LidarCommandTimeout));
      return LidarCommandTimeout[command_id];
    case kCommandSetHub:
      assert(command_id < sizeof(HubCommandTimeout));
      return HubCommandTimeout[command_id];
  }
  return KDefaultTimeOut;
}

bool IsSubLidarException(const Command &command) {
  if (device_manager().device_mode() == kDeviceModeLidar) {
    return false;
  }
  if (!(command.packet.cmd_set == kCommandSetGeneral &&
      command.packet.cmd_code == kCommandIDGeneralPushAbnormalState)) {
    return false;
  }

  HubErrorCode* hub_error_code = static_cast<HubErrorCode *>((void *)command.packet.data);
  if (hub_error_code->lidar_link_status == 0x01) {
    return true;
  }
  return false;
}

bool IsMidDeviceIpResponse(const Command &command) {
  return command.packet.cmd_set == kCommandSetGeneral &&
         command.packet.cmd_code == kCommandIDGeneralGetDeviceIpInformation &&
         command.packet.data_len != sizeof(GetDeviceIpModeResponse);
}

void OnSubLidarDisconnect() {
  device_manager().UpdateDevices(DeviceInfo(), kEventHubConnectionChange);
}

bool CommandHandler::AddDevice(const DeviceInfo &info) {
  if (impl_ == NULL) {
    DeviceMode mode = static_cast<DeviceMode>(device_manager().device_mode());
    if (mode == kDeviceModeHub) {
      impl_.reset(new HubCommandHandlerImpl(this, loop_));
    } else if (mode == kDeviceModeLidar) {
      impl_.reset(new LidarCommandHandlerImpl(this, loop_));
    }
  }
  if (impl_ == NULL) {
    return false;
  }

  return impl_->AddDevice(info);
}

bool CommandHandler::Init(std::weak_ptr<IOLoop> loop) {
  loop_ = loop;
  return true;
}

void CommandHandler::Uninit() {
  if (impl_) {
    impl_.reset(NULL);
  }
}

void CommandHandler::OnCommand(uint8_t handle, const Command &command) {
  if (command.packet.packet_type == kCommandTypeAck) {
    LOG_INFO(" Recieve Ack: Set {} Id {} Seq {}", (uint16_t)command.packet.cmd_set, command.packet.cmd_code, command.packet.seq_num);
    OnCommandAck(handle, command);
  } else if (command.packet.packet_type == kCommandTypeMsg) {
    LOG_INFO(" Recieve Message: Set {} Id {} Seq {}", (uint16_t)command.packet.cmd_set, command.packet.cmd_code, command.packet.seq_num);
    OnCommandMsg(handle, command);
  }
}

livox_status CommandHandler::SendCommand(uint8_t handle,
                                         uint8_t command_set,
                                         uint8_t command_id,
                                         uint8_t *data,
                                         uint16_t length,
                                         const shared_ptr<CommandCallback> &cb) {
  if (impl_ == NULL) {
    return kStatusHandlerImplNotExist;
  }

  Command cmd(handle,
              kCommandTypeCmd,
              command_set,
              command_id,
              CommandChannel::GenerateSeq(),
              data,
              length,
              GetCommandTimeout(command_set, command_id),
              cb);
  livox_status result = impl_->SendCommand(handle, cmd);
  return result;
}

livox_status CommandHandler::RegisterPush(uint8_t handle,
                                          uint8_t command_set,
                                          uint8_t command_id,
                                          const shared_ptr<CommandCallback> &cb) {
  lock_guard<mutex> lock(mutex_);
  Command req(handle, kCommandTypeMsg, command_set, command_id, 0, NULL, 0, 0, cb);
  message_registers_.insert(make_pair(MakeKey(command_set, command_id), req));
  return kStatusSuccess;
}

void CommandHandler::OnCommandAck(uint8_t handle, const Command &command) {
  if (command.cb == NULL) {
    return;
  }
  if (command.packet.data == NULL) {
    (*command.cb)(kStatusTimeout, handle, command.packet.data);
    return;
  }

  if (IsMidDeviceIpResponse(command)) {
    GetDeviceIpModeResponse  response;
    memcpy(&response, command.packet.data, command.packet.data_len);
    uint8_t net_mask[4] = {255, 255, 255, 0};
    memcpy(&response.net_mask, net_mask, 4);
    uint8_t gw_addr[4] = {192, 168, 1, 1};
    memcpy(&response.gw_addr, gw_addr, 4);

    (*command.cb)(kStatusSuccess, handle, (uint8_t *)&response);
    return;
  }
  (*command.cb)(kStatusSuccess, handle, command.packet.data);
}

void CommandHandler::OnCommandMsg(uint8_t handle, const Command &command) {
  list<Command> commands;
  {
    lock_guard<mutex> lock(mutex_);
    pair<multimap<uint16_t, Command>::iterator, multimap<uint16_t, Command>::iterator> ret;
    ret = message_registers_.equal_range(MakeKey(command.packet.cmd_set, command.packet.cmd_code));

    for (multimap<uint16_t, Command>::iterator ite = ret.first; ite != ret.second; ++ite) {
      if (ite->second.handle == handle) {
        commands.push_back(ite->second);
      }
    }
  }
  if (IsSubLidarException(command)) {
    OnSubLidarDisconnect();
  }

  for (list<Command>::iterator ite = commands.begin(); ite != commands.end(); ++ite) {
    if (ite->cb) {
      (*ite->cb)(kStatusSuccess, handle, command.packet.data);
    }
  }
}

void CommandHandler::RemoveDevice(uint8_t handle) {
  if (impl_) {
    impl_->RemoveDevice(handle);
  };
}

void CommandHandler::OnHeartbeatStateUpdate(uint8_t handle, const HeartbeatResponse &state) {
  device_manager().UpdateDeviceState(handle, state);
}

}  // namespace livox
