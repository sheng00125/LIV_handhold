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

#ifndef LIVOX_NETWORK_UTIL_H_
#define LIVOX_NETWORK_UTIL_H_

#include <stdint.h>
#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif // WIN32
#include <stdio.h>
#include <stdio.h>
#include <fcntl.h>


namespace livox {
namespace util {

typedef int socket_t;

socket_t CreateSocket(uint16_t port, bool nonblock = true, bool reuse_port = true);

void CloseSock(socket_t sock);

bool FindLocalIp(const struct sockaddr_in &client_addr, uint32_t &local_ip);

size_t RecvFrom(socket_t &sock, void *buff,  size_t buf_size, int flag, struct sockaddr *addr, int* addrlen);

}  // namespace util
}  // namespace livox

#endif  // LIVOX_NETWORK_UTIL_H_
