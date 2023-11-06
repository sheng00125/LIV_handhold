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
#include "base/network/network_util.h"
#include <ifaddrs.h>
#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <netdb.h>

namespace livox {
namespace util {

socket_t CreateSocket(uint16_t port, bool nonblock, bool reuse_port) {
  int status = -1;
  int on = -1;
  int sock = -1;
  struct sockaddr_in servaddr;

  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    return -1;
  }

  if (nonblock) {
    status = ioctl(sock, FIONBIO, (char*)&on);
    if (status != 0) {
      close(sock);
      return -1;
    }
  }

  if (reuse_port) {
    status = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
                        (char *) &on, sizeof (on));
    if (status != 0) {
      close(sock);
      return -1;
    }
  }

  memset(&servaddr, 0, sizeof(servaddr));
 
  // Filling server information
  servaddr.sin_family    = AF_INET; // IPv4
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(port);

  status = bind(sock, (const struct sockaddr *)&servaddr, sizeof(servaddr));
  if (status != 0) {
    close(sock);
    return -1;
  }

  return sock;
}

void CloseSock(int sock) {
  if (sock > 0) {
    close(sock);
  }
}

bool FindLocalIp(const struct sockaddr_in &client_addr, uint32_t &local_ip) {
  struct ifaddrs *if_addrs = NULL, *addrs = NULL;
  if (getifaddrs(&if_addrs) == -1) {
    return false;
  }
  addrs = if_addrs;
  bool found = false;
  while (if_addrs != NULL) {
    if ((if_addrs->ifa_addr != NULL) && (if_addrs->ifa_addr->sa_family == AF_INET))  // check it is IP4
    {
      // is a connected IP4 Address
      struct sockaddr_in *ifu_localaddr = (struct sockaddr_in *)if_addrs->ifa_addr;
      struct sockaddr_in *ifu_netmask = (struct sockaddr_in *)if_addrs->ifa_netmask;
      if (ifu_localaddr->sin_addr.s_addr != htonl(INADDR_ANY)) {
        if ((ifu_localaddr->sin_addr.s_addr & ifu_netmask->sin_addr.s_addr) ==
            (client_addr.sin_addr.s_addr & ifu_netmask->sin_addr.s_addr)) {
          local_ip = ifu_localaddr->sin_addr.s_addr;
          found = true;
          break;
        }
      }
    }
    if_addrs = if_addrs->ifa_next;
  }

  if (addrs) {
    freeifaddrs(addrs);
  }
  return found;
}

size_t RecvFrom(socket_t &sock, void *buff,  size_t buf_size, int flag, struct sockaddr *addr, int *addrlen) {
  return recvfrom(sock, buff, buf_size, 0, addr, (socklen_t *)addrlen);
}

}  // namespace util
}  // namespace livox

#endif  // WIN32
