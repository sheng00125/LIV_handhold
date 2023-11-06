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

#ifndef LIVOX_GPS_PROTOCOL_H_
#define LIVOX_GPS_PROTOCOL_H_

#include <stdint.h>
#include "protocol.h"

namespace livox_ros {

#pragma pack(1)

typedef struct {
  uint8_t sof;
  uint8_t cmd_str[1];
} GpsPreamble;

typedef struct {
  uint8_t sof;
  uint8_t data[1];
} GpsPacket;

#pragma pack()

uint8_t AscciiToHex(const uint8_t *TwoChar);

class GpsProtocol : public Protocol {
 public:
  GpsProtocol();
  ~GpsProtocol() = default;

  int32_t ParsePacket(const uint8_t *i_buf, uint32_t i_len,
                      CommPacket *o_packet) override;

  int32_t Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len,
               const CommPacket &i_packet) override;

  uint32_t GetPreambleLen() override;

  uint32_t GetPacketWrapperLen() override;

  uint32_t FindPacketLen(const uint8_t *buf, uint32_t buf_length) override;

  uint32_t GetPacketLen(const uint8_t *buf) override;

  int32_t CheckPreamble(const uint8_t *buf) override;

  int32_t CheckPacket(const uint8_t *buf) override;

 private:
  uint32_t found_length_;

  uint8_t CalcGpsPacketChecksum(const uint8_t *buf, uint32_t length);
};

}  // namespace livox_ros
#endif  // LIVOX_GPS_PROTOCOL_H_
