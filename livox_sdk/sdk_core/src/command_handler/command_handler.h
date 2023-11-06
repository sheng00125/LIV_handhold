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

#ifndef LIVOX_COMMAND_HANDLER_H_
#define LIVOX_COMMAND_HANDLER_H_

#include <memory>
#include <map>
#include <mutex>
#include "base/command_callback.h"
#include "command_channel.h"
#include "device_discovery.h"
#include "livox_sdk.h"

namespace livox {
class CommandHandlerImpl;

class CommandHandler : public noncopyable {
 public:
  explicit CommandHandler() {}

  bool Init(std::weak_ptr<IOLoop> loop);
  void Uninit();

  bool AddDevice(const DeviceInfo &info);
  void RemoveDevice(uint8_t handle);

  livox_status SendCommand(uint8_t handle,
                           uint8_t command_set,
                           uint8_t command_id,
                           uint8_t *data,
                           uint16_t length,
                           const std::shared_ptr<CommandCallback> &cb);

  livox_status RegisterPush(uint8_t handle,
                            uint8_t command_set,
                            uint8_t command_id,
                            const std::shared_ptr<CommandCallback> &cb);

  void OnCommand(uint8_t handle, const Command &command);

  void OnHeartbeatStateUpdate(uint8_t handle, const HeartbeatResponse &state);

 private:
  inline uint16_t MakeKey(uint8_t command_set, uint8_t command_id) { return (command_set << 8) | command_id; }

  void OnCommandAck(uint8_t handle, const Command &command);
  void OnCommandMsg(uint8_t handle, const Command &command);

 private:
  std::multimap<uint16_t, Command> message_registers_;
  std::unique_ptr<CommandHandlerImpl> impl_;
  std::mutex mutex_;
  std::weak_ptr<IOLoop> loop_;
};

class CommandHandlerImpl : public CommandChannelDelegate {
 public:
  CommandHandlerImpl(CommandHandler *handler) : handler_(handler) {}
  virtual ~CommandHandlerImpl() { Uninit(); }
  virtual void Uninit(){};
  virtual bool AddDevice(const DeviceInfo &info) = 0;
  virtual bool RemoveDevice(uint8_t handle) = 0;

  virtual livox_status SendCommand(uint8_t handle, const Command &command) = 0;

  virtual void OnCommand(uint8_t handle, const Command &command) {
    if (handler_) {
      handler_->OnCommand(handle, command);
    }
  }

  void OnHeartbeatStateUpdate(uint8_t handle, const HeartbeatResponse &state) {
    if (handler_) {
      handler_->OnHeartbeatStateUpdate(handle, state);
    }
  }

 private:
  CommandHandler *handler_;
};

CommandHandler &command_handler();

}  // namespace livox

#endif  // LIVOX_COMMAND_HANDLER_H_
