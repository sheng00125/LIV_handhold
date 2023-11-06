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

#ifndef WIN32
#include "base/wake_up/wake_up_pipe.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

namespace livox {

WakeUpPipe::~WakeUpPipe() {
  PipeDestroy();
}

bool WakeUpPipe::WakeUp() {
  char ch = '1';
  ssize_t nbytes = sizeof(ch);
  if (pipe_in_ > 0) {
    if (nbytes != write(pipe_in_, &ch, nbytes)) {
      return false;
    }
  }
  return true;
}

bool WakeUpPipe::Drain() {
  char ch[512];
  size_t size = sizeof(ch);
  if (pipe_out_ > 0) {
    if (read(pipe_out_, ch, size) < 0) {
      return false;
    }
  }
  return true;
}

bool WakeUpPipe::PipeDestroy() {
  if (pipe_in_ > 0) {
    close(pipe_in_);
  }
  if (pipe_out_ > 0) {
    close(pipe_out_);
  }
  return true;
}

bool WakeUpPipe::PipeCreate() {
  bool status = false;
  //in filedes[0]
  //out filedes[1]
  int filedes[2]= {};
  if (pipe(filedes) == -1) {
    return false;
  }
  do {
    int flags = 0;
    if ((flags = fcntl(filedes[0], F_GETFD)) == -1) {
      break;
    }

    flags |= FD_CLOEXEC;
    if (fcntl(filedes[0], F_SETFD, flags) == -1) {
      break;
    }

    flags = 0;
    if ((flags = fcntl(filedes[1], F_GETFD)) == -1) {
      break;
    }

    flags |= FD_CLOEXEC;
    if (fcntl(filedes[1], F_SETFD, flags) == -1) {
      break;
    }
    status = true;
  } while(0);

  if (!status) {
    if (filedes[0] > 0) {
      close(filedes[0]);
    }
    if (filedes[1] > 0) {
      close(filedes[1]);
    }
    return false;
  }
  pipe_out_ = filedes[0];
  pipe_in_ = filedes[1];
  return true;
}

}  // namespace livox

#endif  // WIN32
