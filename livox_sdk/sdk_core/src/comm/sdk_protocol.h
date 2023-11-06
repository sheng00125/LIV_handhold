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

#ifndef LIVOX_SDK_PROTOCOL_H_
#define LIVOX_SDK_PROTOCOL_H_

#include <stdint.h>
#include "../include/comm/protocol.h"
#include "../include/third_party/FastCRC/FastCRC.h"
namespace livox {
typedef enum { kSdkVerNone, kSdkVer0, kSdkVer1 } SdkVersion;

#pragma pack(1)

typedef struct {
  uint8_t sof;
  uint8_t version;
  uint16_t length;
  uint8_t packet_type;
  uint16_t seq_num;
  uint16_t preamble_crc;
} SdkPreamble;

typedef struct {
  uint8_t sof;
  uint8_t version;
  uint16_t length;
  uint8_t packet_type;
  uint16_t seq_num;
  uint16_t preamble_crc;
  uint8_t cmd_set;
  uint8_t cmd_id;
  uint8_t data[1];
} SdkPacket;

#pragma pack()

class SdkProtocol : public Protocol {
 public:
  SdkProtocol(uint16_t seed16, uint32_t seed32);

  ~SdkProtocol();

  int32_t ParsePacket(uint8_t *i_buf, uint32_t i_len, CommPacket *o_packet);

  int32_t Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet);

  uint32_t GetPreambleLen();

  uint32_t GetPacketWrapperLen();

  uint32_t GetPacketLen(uint8_t *buf);

  int32_t CheckPreamble(uint8_t *buf);

  int32_t CheckPacket(uint8_t *buf);

 private:
  FastCRC16 crc16_;
  FastCRC32 crc32_;
};
}  // namespace livox
#endif  // LIVOX_SDK_PROTOCOL_H_
