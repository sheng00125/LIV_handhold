//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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

#include "comm/comm.h"
#include <string.h>
#include <arpa/inet.h>

namespace livox_ros {

/** Common function --------------------------------------------------------- */
bool IsFilePathValid(const char *path_str) {
  int str_len = strlen(path_str);

  if ((str_len > kPathStrMinSize) && (str_len < kPathStrMaxSize)) {
    return true;
  } else {
    return false;
  }
}

uint32_t CalculatePacketQueueSize(const double publish_freq) {
  uint32_t queue_size = 10;
  if (publish_freq > 10.0) {
    queue_size = static_cast<uint32_t>(publish_freq) + 1;
  }
  return queue_size;
}

std::string IpNumToString(uint32_t ip_num) {
  struct in_addr ip;
  ip.s_addr = ip_num;
  return std::string(inet_ntoa(ip));
}

uint32_t IpStringToNum(std::string ip_string) {
  return static_cast<uint32_t>(inet_addr(ip_string.c_str()));
}

std::string ReplacePeriodByUnderline(std::string str) {
  std::size_t pos = str.find(".");
  while (pos != std::string::npos) {
    str.replace(pos, 1, "_");
    pos = str.find(".");
  }
  return str;
}

} // namespace livox_ros

