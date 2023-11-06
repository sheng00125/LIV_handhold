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
#include "base/network/network_util.h"
#include <memory>
#include <iphlpapi.h>
#include <string>
#pragma comment(lib,"iphlpapi.lib")
#pragma comment(lib,"ws2_32.lib")

namespace livox {
namespace util {

void CloseSock(socket_t sock) {
  closesocket(sock);
}

socket_t CreateSocket(uint16_t port, bool nonblock, bool reuse_port) {
  int status = -1;
  int on = -1;
  int sock = -1;
  struct sockaddr_in servaddr;
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock == INVALID_SOCKET) {
    return -1;
  }

  if (nonblock) {
    status = ioctlsocket(sock, FIONBIO, (u_long *)&on);
    if (status != NO_ERROR) {
      closesocket(sock);
      return -1;
    }
  }

  if (reuse_port) {
    status = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
                        (char *) &on, sizeof (on));
    if (status != 0) {
      closesocket(sock);
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
    closesocket(sock);
    return -1;
  }

  return sock;
}

bool GetAdapterState(const IP_ADAPTER_INFO *pAdapter) {
  if(pAdapter == NULL) {
    return false;
  }
  MIB_IFROW info;
  memset(&info ,0 ,sizeof(MIB_IFROW));
  info.dwIndex = pAdapter->Index;
  if (GetIfEntry(&info) != NOERROR) {
    return false;
  }
  if (info.dwOperStatus == IF_OPER_STATUS_NON_OPERATIONAL
    || info.dwOperStatus == IF_OPER_STATUS_UNREACHABLE
    || info.dwOperStatus == IF_OPER_STATUS_DISCONNECTED
    || info.dwOperStatus == IF_OPER_STATUS_CONNECTING)
    return false;
  return true;
}

bool FindLocalIp(const struct sockaddr_in &client_addr, uint32_t &local_ip) {
  bool found = false;
  ULONG ulOutbufLen = sizeof(IP_ADAPTER_INFO);
  std::unique_ptr<uint8_t[]> pAdapterInfo(new uint8_t[ulOutbufLen]);
  DWORD dlRetVal = GetAdaptersInfo(reinterpret_cast<IP_ADAPTER_INFO *>(pAdapterInfo.get()), &ulOutbufLen);
  if (dlRetVal == ERROR_BUFFER_OVERFLOW) {
    pAdapterInfo.reset(new uint8_t[ulOutbufLen]);
    dlRetVal = GetAdaptersInfo(reinterpret_cast<IP_ADAPTER_INFO *>(pAdapterInfo.get()), &ulOutbufLen);
  }

  IP_ADAPTER_INFO *pAdapter = reinterpret_cast<IP_ADAPTER_INFO *>(pAdapterInfo.get());
  if (NO_ERROR == dlRetVal && pAdapter != NULL) {
    while (pAdapter != NULL) {
      if (GetAdapterState(pAdapter)) {
        std::string str_ip = pAdapter->IpAddressList.IpAddress.String;
        std::string str_mask = pAdapter->IpAddressList.IpMask.String;
        ULONG host_ip = inet_addr(const_cast<char *>(str_ip.c_str()));
        ULONG host_mask = inet_addr(const_cast<char *>(str_mask.c_str()));

        if ((host_ip & host_mask) ==
          (client_addr.sin_addr.S_un.S_addr & host_mask)) {
          local_ip = host_ip;
          found = true;
          break;
        }
      }
      pAdapter = pAdapter->Next;
    }
  }
  return found;
}

size_t RecvFrom(socket_t &sock, void *buff,  size_t buf_size, int flag, struct sockaddr *addr, int* addrlen) {
  return recvfrom(sock, (char *)buff, buf_size, 0, addr, addrlen);
}

}  // namespace util
}  // namespace livox

#endif  // WIN32
