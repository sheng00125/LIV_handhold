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

#include "sdk_protocol.h"

#include <stdio.h>
#include <string.h>
namespace livox {
const uint8_t kSdkProtocolSof = 0xAA;
const uint32_t kSdkPacketCrcSize = 4;          // crc32
const uint32_t kSdkPacketPreambleCrcSize = 2;  // crc16

SdkProtocol::SdkProtocol(uint16_t seed16, uint32_t seed32) : crc16_(seed16), crc32_(seed32) {}

SdkProtocol::~SdkProtocol() {}

int32_t SdkProtocol::Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet) {
  SdkPacket *sdk_packet = (SdkPacket *)o_buf;

  if (kLidarSdk != i_packet.protocol) {
    return -1;
  }

  sdk_packet->sof = kSdkProtocolSof;
  sdk_packet->length = i_packet.data_len + GetPacketWrapperLen();

  if (sdk_packet->length > o_buf_size) {
    return -1;
  }

  sdk_packet->version = kSdkVer0;
  sdk_packet->packet_type = i_packet.packet_type;
  sdk_packet->seq_num = i_packet.seq_num & 0xFFFF;
  sdk_packet->preamble_crc = crc16_.mcrf4xx_calc(o_buf, GetPreambleLen() - kSdkPacketPreambleCrcSize);
  sdk_packet->cmd_set = i_packet.cmd_set;
  sdk_packet->cmd_id = i_packet.cmd_code;

  memcpy(sdk_packet->data, i_packet.data, i_packet.data_len);

  uint32_t crc = crc32_.crc32_calc(o_buf, sdk_packet->length - kSdkPacketCrcSize);
  o_buf[sdk_packet->length - 4] = crc & 0xFF;
  o_buf[sdk_packet->length - 3] = (crc >> 8) & 0xFF;
  o_buf[sdk_packet->length - 2] = (crc >> 16) & 0xFF;
  o_buf[sdk_packet->length - 1] = (crc >> 24) & 0xFF;

  *o_len = sdk_packet->length;

  return 0;
}

int32_t SdkProtocol::ParsePacket(uint8_t *i_buf, uint32_t i_len, CommPacket *o_packet) {
  SdkPacket *sdk_packet = (SdkPacket *)i_buf;

  if (i_len < GetPacketWrapperLen()) {
    return -1;  // packet lenth error
  }

  memset((void *)o_packet, 0, sizeof(CommPacket));

  o_packet->packet_type = sdk_packet->packet_type;
  o_packet->protocol = kLidarSdk;
  o_packet->protocol_version = sdk_packet->version;

  o_packet->seq_num = sdk_packet->seq_num;
  o_packet->cmd_set = sdk_packet->cmd_set;
  o_packet->cmd_code = sdk_packet->cmd_id;
  o_packet->data_len = sdk_packet->length - GetPacketWrapperLen();
  o_packet->data = sdk_packet->data;

  return 0;
}

uint32_t SdkProtocol::GetPreambleLen() {
  return sizeof(SdkPreamble);
}

uint32_t SdkProtocol::GetPacketWrapperLen() {
  return sizeof(SdkPacket) - 1 + kSdkPacketCrcSize;
}

uint32_t SdkProtocol::GetPacketLen(uint8_t *buf) {
  SdkPreamble *preamble = (SdkPreamble *)buf;
  return preamble->length;
}

int32_t SdkProtocol::CheckPreamble(uint8_t *buf) {
  SdkPreamble *preamble = (SdkPreamble *)buf;

  if ((preamble->sof == kSdkProtocolSof) && (crc16_.mcrf4xx_calc(buf, GetPreambleLen()) == 0)) {
    return 0;
  } else {
    return -1;
  }
}

int32_t SdkProtocol::CheckPacket(uint8_t *buf) {
  SdkPacket *sdk_packet = (SdkPacket *)buf;
  uint32_t crc = ((uint32_t)(buf[sdk_packet->length - 4])) | (((uint32_t)(buf[sdk_packet->length - 3])) << 8) |
                 (((uint32_t)(buf[sdk_packet->length - 2])) << 16) | (((uint32_t)(buf[sdk_packet->length - 1])) << 24);

  if (crc32_.crc32_calc(buf, sdk_packet->length - kSdkPacketCrcSize) == crc) {
    return 0;
  } else {
    return -1;
  }
}
}  // namespace livox
