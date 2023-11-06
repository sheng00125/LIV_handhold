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

#include "gps_protocol.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

namespace livox_ros {

const uint8_t kGpsProtocolSof = '$';
const uint8_t kGpsProtocolEof = '*';
const uint32_t kPacketLengthLmit = 200;
const uint32_t kPreambleLen = 1;
const uint32_t kWrapperLen = 4; /** '$' + '*' + '2 checksum byte' */

GpsProtocol::GpsProtocol() { found_length_ = 0; }

int32_t GpsProtocol::Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len,
                          const CommPacket &i_packet) {
  // GpsPacket* gps_packet = (GpsPacket*)o_buf;
  return 0;
}

int32_t GpsProtocol::ParsePacket(const uint8_t *i_buf, uint32_t i_len,
                                 CommPacket *o_packet) {
  // GpsPacket *gps_packet = (GpsPacket *)i_buf;

  if (i_len < GetPacketWrapperLen()) {
    return -1;  // packet length error
  }
  memset((void *)o_packet, 0, sizeof(CommPacket));
  o_packet->protocol = kGps;
  o_packet->data = (uint8_t *)i_buf;
  o_packet->data_len = i_len;

  return 0;
}

uint32_t GpsProtocol::GetPreambleLen() { return kPreambleLen; /** '$' */ }

uint32_t GpsProtocol::GetPacketWrapperLen() {
  return kWrapperLen; /** '$' + '*' + '2 checksum bytes' */
}

uint32_t GpsProtocol::FindPacketLen(const uint8_t *buf, uint32_t buf_length) {
  uint32_t i = 0;
  for (; (i < buf_length) && (i < kPacketLengthLmit); i++) {
    if ((buf[i] == kGpsProtocolEof) && (buf[0] == kGpsProtocolSof)) {
      found_length_ = i + 1 + 2; /* num = index + 1 + two bytes checksum */
      return kFindLengthSuccess;
    }
  }
  if (i < kPacketLengthLmit) {
    return kFindLengthContinue;
  } else {
    return kFindLengthError;
  }
}

uint32_t GpsProtocol::GetPacketLen(const uint8_t *buf) { return found_length_; }

int32_t GpsProtocol::CheckPreamble(const uint8_t *buf) {
  GpsPreamble *preamble = (GpsPreamble *)buf;

  if (preamble->sof == kGpsProtocolSof) {
    return 0;
  } else {
    return -1;
  }
}

int32_t GpsProtocol::CheckPacket(const uint8_t *buf) {
  uint8_t checksum =
      CalcGpsPacketChecksum(&buf[1], found_length_ - kWrapperLen);
  uint8_t raw_checksum = AscciiToHex(&buf[found_length_ - 2]);
  if (checksum == raw_checksum) {
    return 0;
  } else {
    return 0;
  }
}

uint8_t GpsProtocol::CalcGpsPacketChecksum(const uint8_t *buf,
                                           uint32_t length) {
  uint8_t result = buf[0];
  for (uint32_t i = 1; i < length; i++) {
    result ^= buf[i];
  }
  return result;
}

uint8_t AscciiToHex(const uint8_t *TwoChar) {
  uint8_t h = toupper(TwoChar[0]) - 0x30;
  if (h > 9) h -= 7;

  uint8_t l = toupper(TwoChar[1]) - 0x30;
  if (l > 9) l -= 7;

  return h * 16 + l;
}

}  // namespace livox_ros
