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

#ifdef WIN32
#include "base/wake_up/wake_up_pipe.h"
#include <fcntl.h>
#include <winsock2.h>
#include <stdio.h>
#pragma comment(lib,"iphlpapi.lib")
#pragma comment(lib,"ws2_32.lib")

namespace livox {

WakeUpPipe::~WakeUpPipe() {
  PipeDestroy();
}

bool WakeUpPipe::WakeUp() {
  char ch = '1';
  size_t nbytes = sizeof(ch);
  if (pipe_in_ > 0) {
    if (nbytes != send(pipe_in_,&ch, nbytes, 0)) {
      return false;
    }
  }
  return true;
}

bool WakeUpPipe::Drain() {
  char ch[512];
  size_t size = sizeof(ch);
  if (pipe_out_ > 0) {
    recv(pipe_out_, ch, size, 0);
  }
  return true;
}

bool WakeUpPipe::PipeDestroy() {
  if (pipe_in_ > 0) {
    closesocket(pipe_in_);
  }
  if (pipe_out_ > 0) {
    closesocket(pipe_out_);
  }
  return true;
}

bool WakeUpPipe::PipeCreate() {
  int listen_sock = -1;
  unsigned long on = 1;
  bool status = false;
  if ((listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET) {
    return false;
  }

  struct sockaddr_in servaddr;
  int servaddr_len = sizeof(servaddr);
  servaddr.sin_family = AF_INET;
  servaddr.sin_port   = 0;
  servaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  do {
    if (bind(listen_sock, (const struct sockaddr *)&servaddr, sizeof(servaddr)) == SOCKET_ERROR) {
      break;
    }
    if (getsockname(listen_sock, (struct sockaddr *)&servaddr, &servaddr_len) == SOCKET_ERROR) {
      break;
    }
    if (listen(listen_sock, 1) == SOCKET_ERROR) {
      break;
    }
    if ((pipe_in_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET) {
      break;
    }
    if (connect(pipe_in_, (const struct sockaddr *)&servaddr, sizeof(servaddr)) == SOCKET_ERROR) {
      break;
    }
    if (ioctlsocket(listen_sock, FIONBIO, &on) == SOCKET_ERROR) {
      break;
    }

    struct sockaddr_in clientaddr;
    int clientaddr_len = sizeof(clientaddr);
    fd_set poll_set;
    FD_ZERO(&poll_set);
    FD_SET(listen_sock, &poll_set);
    // timeout 2s
    struct timeval timeout = {2, 0};
    int rv = select(0, &poll_set, nullptr, nullptr, &timeout);
    if (rv <= 0) {
      break;
    }
    if ((pipe_out_ = accept(listen_sock, (struct sockaddr *)&clientaddr, &clientaddr_len)) == INVALID_SOCKET) {
      break;
    }
    status = true;
  } while(0);

  closesocket(listen_sock);
  if (!status) {
    if (pipe_in_ > 0) {
      closesocket(pipe_in_);
    }
    return false;
  }
  return true;
}

}  // namespace livox

#endif  // WIN32