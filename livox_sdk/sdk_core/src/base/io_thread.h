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

#ifndef LIVOX_IO_THREAD_H_
#define LIVOX_IO_THREAD_H_
#include <memory>
#include "io_loop.h"
#include "thread_base.h"

namespace livox {

class IOThread : public ThreadBase {
 public:
  IOThread() : loop_(nullptr) {}
  bool Init(bool enable_timer = true, bool enable_wake = true);
  void Join();
  void Uninit();

  std::weak_ptr<IOLoop> loop() { return loop_; }
  void ThreadFunc();

 private:
  std::shared_ptr<IOLoop> loop_;
};
}  // namespace livox
#endif  // LIVOX_IO_THREAD_H_
