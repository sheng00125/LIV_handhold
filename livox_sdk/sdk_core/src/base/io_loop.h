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

#ifndef LIVOX_IO_LOOP_H_
#define LIVOX_IO_LOOP_H_

#include <functional>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>
#include "command_callback.h"
#include "noncopyable.h"
#include "thread_base.h"
#include "multiple_io/multiple_io_base.h"
#include "multiple_io/multiple_io_factory.h"

namespace livox {

#define OPEN_MAX_POLL 48
#define POLL_TIMEOUT 50 //ms
typedef int socket_t;

class IOLoop : public noncopyable {
 public:
 typedef std::function<void(void)> IOLoopTask;

  class IOLoopDelegate {
   public:
    virtual void OnData(socket_t, void *) {}
    virtual void OnTimer(std::chrono::steady_clock::time_point) {}
    virtual void OnWake() {}
  };

 public:
  explicit IOLoop(bool enable_timer = true, bool enable_wake = true)
      : enable_timer_(enable_timer), enable_wake_(enable_wake){};


  bool Init();
  void Uninit();
  void AddDelegate(socket_t sock, IOLoopDelegate *delegate, void *data = NULL);
  void RemoveDelegate(socket_t sock, IOLoopDelegate *delegate);
  void Loop();
  bool Wakeup();
  void PostTask(const IOLoopTask &task);

 private:
  void AddDelegateAsync(socket_t sock, IOLoopDelegate *delegate, void *data);
  void RemoveDelegateAsync(socket_t sock);

 private:
  std::mutex mutex_;
  bool enable_timer_;
  bool enable_wake_;
  std::vector<IOLoopTask> pending_tasks_;
  std::unique_ptr<MultipleIOBase> multiple_io_base_;
};

}  // namespace livox

#endif  // LIVOX_IO_LOOP_H_
