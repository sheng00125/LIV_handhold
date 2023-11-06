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

#ifndef LIVOX_THREAD_BASE_H_
#define LIVOX_THREAD_BASE_H_
#include <atomic>
#include <thread>
#include "noncopyable.h"

namespace livox {

class ThreadBase : public noncopyable {
 public:
  ThreadBase();
  virtual ~ThreadBase() {}
  virtual void ThreadFunc() = 0;
  bool Init();
  virtual void Uninit();
  virtual bool Start();
  virtual void Join();
  void Quit() { quit_ = true; }
  bool IsQuit() { return quit_; }

 protected:
  std::shared_ptr<std::thread> thread_;
  std::atomic_bool quit_;
  
 private:
  std::atomic_bool is_thread_valid_;
};

}  // namespace livox

#endif  // LIVOX_THREAD_BASE_H_
