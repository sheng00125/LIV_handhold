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

#include "multiple_io_poll.h"

#ifdef HAVE_POLL

namespace livox {

int GetEvent (FdEvent event) {
  int rv = 0;
  if (event & READBLE_EVENT)
      rv |= POLLIN;
  if (event & WRITABLE_EVENT)
      rv |= POLLOUT;
  return rv;
}

bool MultipleIOPoll:: PollCreate(int size) {
  max_poll_size_ = size + 1;
  pollset_.reset(new struct pollfd[max_poll_size_]);
  WakeUpInit();
  return true;
}

bool MultipleIOPoll:: PollSetAdd(PollFd poll_fd) {
  if (max_poll_size_ <= (int)descriptors_.size()) {
    return false;
  }

  int fd = poll_fd.fd;
  descriptors_[fd] = poll_fd;

  struct pollfd fds;
  fds.fd = fd;
  fds.events = GetEvent(poll_fd.event);
  pollset_[pollset_num_] = fds;
  pollset_num_++;
  return true;
}

void MultipleIOPoll::PollDestroy() {
  WakeUpUninit();
  return;
}

bool MultipleIOPoll:: PollSetRemove(PollFd poll_fd) {
  int fd = poll_fd.fd;
  if (descriptors_.find(fd) != descriptors_.end()) {
    descriptors_.erase(fd);
  }

  for (int i = 0; i< pollset_num_; i++) {
    if (pollset_[i].fd == fd) {
      int dst = i;
      for (i++; i < pollset_num_ - 1; i++) {
        if (pollset_[i].fd == fd) {
          pollset_num_--;
        } else {
          pollset_[dst] = pollset_[i];
          dst++;
        }
      }
    }
  }
  return true;
}

void MultipleIOPoll:: Poll(int time_out) {
  int rv = poll(pollset_.get(), pollset_num_, time_out);
  if (rv > 0) {
    for (int i = 0; i < pollset_num_; i++) {
      FdEvent fd_event = NONE_EVENT;
      if (pollset_[i].revents & POLLIN) {
        fd_event |= READBLE_EVENT;
      }
      if (pollset_[i].revents & POLLOUT) {
        fd_event |= WRITABLE_EVENT;
      }
      int fd = pollset_[i].fd;
      if (descriptors_.find(fd) != descriptors_.end()) {
          PollFd pollfd =  descriptors_[fd];
          pollfd.event_callback(fd_event);
      }
      pollset_[i].revents = NONE_EVENT;
    }
  }
  CheckTimer();
}

}  // namespace livox

#endif // HAVE_POLL
