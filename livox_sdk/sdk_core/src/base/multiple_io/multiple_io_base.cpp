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

#include "multiple_io_base.h"

namespace livox {

void MultipleIOBase::CheckTimer() {
  TimePoint t = std::chrono::steady_clock::now();
  if (t - last_timeout_ > std::chrono::milliseconds(50)) {
    last_timeout_ = t;
    for (auto & descriptor : descriptors_) {
      PollFd pollfd =  descriptor.second;
      if (pollfd.timer_callback) {
        pollfd.timer_callback(t);
      }
    }
  }
}

void MultipleIOBase::WakeUpInit() {
  //Initialize wake up pipe
  wake_up_pipe_.reset(new WakeUpPipe());
  wake_up_pipe_->PipeCreate();
  //register wake_fd to multiple io
  PollFd wake_fd = {};
  wake_fd.fd = wake_up_pipe_->GetPipeOut();
  wake_fd.event = READBLE_EVENT;
  wake_fd.event_callback = [this](FdEvent event) {
    if (event & READBLE_EVENT) {
      if (wake_up_pipe_) {
        wake_up_pipe_->Drain();
      }
      for (auto & descriptor : descriptors_) {
        PollFd pollfd =  descriptor.second;
        if (pollfd.wake_callback) {
          pollfd.wake_callback();
        }
      }
    }
  };
  PollSetAdd(wake_fd);
}

void MultipleIOBase::WakeUpUninit() {
  PollFd wake_fd = {};
  wake_fd.fd = wake_up_pipe_->GetPipeOut();
  wake_fd.event = READBLE_EVENT | WRITABLE_EVENT;
  PollSetRemove(wake_fd);
  if (wake_up_pipe_) {
    wake_up_pipe_->PipeDestroy();
    wake_up_pipe_ = nullptr;
  }
}

void MultipleIOBase::PollWakeUp() {
  if (wake_up_pipe_) {
    wake_up_pipe_->WakeUp();
  }
  return;
}

}  // namespace livox
