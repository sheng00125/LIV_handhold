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

#ifndef COMM_COMM_DEVICE_H_
#define COMM_COMM_DEVICE_H_

#include <stdint.h>

namespace livox_ros {

const uint32_t kDevNameLengthMax = 256;

/** Communication device type define */
enum CommDeviceType {
  kCommDevUart,
  kCommDevUsb,
  kCommDevCan,
  kCommDevEthernet,
  kCommDevUndef
};

/** Communication device uart config */
struct CommDevUartConfig {
  uint8_t baudrate;
  uint8_t parity;
};

/** Communication device usb config */
struct CommDevUsbConfig {
  void *data;
};

/** Communication device can config */
struct CommDevCanConfig {
  void *data;
};

/** Communication device config */
typedef struct {
  uint8_t type;
  char name[kDevNameLengthMax];
  union {
    CommDevUartConfig uart;
    CommDevUsbConfig usb;
    CommDevCanConfig can;
  } config;
} CommDevConfig;

}  // namespace livox_ros
#endif  // COMM_COMM_DEVICE_H_
