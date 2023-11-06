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

#ifndef MULTIPLE_IO_KQUEUE_H_
#define MULTIPLE_IO_KQUEUE_H_

#include "multiple_io_base.h"
#include "config.h"
#ifdef HAVE_KQUEUE

namespace livox {

class MultipleIOKqueue : public MultipleIOBase {
 public:
  bool PollCreate(int size);
  bool PollSetAdd(PollFd poll_fd);
  bool PollSetRemove(PollFd poll_fd);
  void Poll(int timeout);
  void PollDestroy();
 private:
  int kqueue_fd_ = -1;
  struct kevent kevent_ = {};
  std::unique_ptr<struct kevent[]> kevent_set_;
  int max_poll_size_ = 0;
  int set_size_ = 0;
};

}  // namespace livox

#endif  // HAVE_KQUEUE
#endif  // MULTIPLE_IO_KQUEUE_H_