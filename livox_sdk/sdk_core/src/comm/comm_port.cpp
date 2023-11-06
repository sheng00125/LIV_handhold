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

#include "comm/comm_port.h"
#include <stdio.h>
#include <string.h>
#include <iostream>

#include "sdk_protocol.h"

namespace livox {
const uint32_t kSearchPacketPreamble = 0;
const uint32_t kGetPacketData = 1;

CommPort::CommPort() {
  protocol_ = new SdkProtocol(0x4c49, 0x564f580a);
  cache_.wr_idx = 0;
  cache_.rd_idx = 0;
  cache_.size = kCacheSize;
  parse_step_ = kSearchPacketPreamble;
  seq_num_ = 0;
}

CommPort::~CommPort() {
  if (protocol_) {
    delete protocol_;
  }
}

uint8_t *CommPort::FetchCacheFreeSpace(uint32_t *o_len) {
  UpdateCache();

  if (cache_.wr_idx < cache_.size) {
    *o_len = cache_.size - cache_.wr_idx;
    return &cache_.buf[cache_.wr_idx];
  } else {
    *o_len = 0;
    return NULL;
  }
}

int32_t CommPort::UpdateCacheWrIdx(uint32_t used_size) {
  if ((cache_.wr_idx + used_size) < cache_.size) {
    cache_.wr_idx += used_size;
    return 0;
  } else {
    return -1;
  }
}

uint32_t CommPort::GetCacheTailSize() {
  if (cache_.wr_idx < cache_.size) {
    return cache_.size - cache_.wr_idx;
  } else {
    return 0;
  }
}

uint32_t CommPort::GetValidDataSize() {
  if (cache_.wr_idx > cache_.rd_idx) {
    return cache_.wr_idx - cache_.rd_idx;
  } else {
    return 0;
  }
}

void CommPort::UpdateCache(void) {
  if (GetCacheTailSize() < kMoveCacheLimit) {  // move unused data to cache head
    uint32_t valid_data_size = GetValidDataSize();

    if (valid_data_size) {
      memmove(cache_.buf, &cache_.buf[cache_.rd_idx], valid_data_size);
      cache_.rd_idx = 0;
      cache_.wr_idx = valid_data_size;
    } else if (cache_.rd_idx) {
      cache_.rd_idx = 0;
      cache_.wr_idx = 0;
    }
  }
}

int32_t CommPort::Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet) {
  return protocol_->Pack(o_buf, o_buf_size, o_len, i_packet);
}

int32_t CommPort::ParseCommStream(CommPacket *o_pack) {
#define CUR_PACK_LEN (protocol_->GetPacketLen(&cache_.buf[cache_.rd_idx]))
#define CUR_CACHE_POS (&cache_.buf[cache_.rd_idx])

  while (GetValidDataSize() > protocol_->GetPreambleLen()) {
    // printf("unpack step : %d %d %d\r\n", parse_step_, cache_.wr_idx, cache_.rd_idx);
    if (kSearchPacketPreamble == parse_step_) {
      if (!protocol_->CheckPreamble(CUR_CACHE_POS)) {
        parse_step_ = kGetPacketData;
      } else {
        ++cache_.rd_idx;
      }
    } else if (kGetPacketData == parse_step_) {
      if (GetValidDataSize() >= CUR_PACK_LEN && CUR_PACK_LEN) {
        parse_step_ = kSearchPacketPreamble;
        if (!protocol_->CheckPacket(CUR_CACHE_POS)) {
          int32_t ret = protocol_->ParsePacket(CUR_CACHE_POS, GetValidDataSize(), o_pack);
          cache_.rd_idx += CUR_PACK_LEN;
          if (kParseSuccess == ret) {
            return ret;
          }
        } else {
          cache_.rd_idx += protocol_->GetPreambleLen();
        }
      } else {
        if (CUR_PACK_LEN > cache_.size) {  // always > cache_.size,
          cache_.rd_idx = 0;
          cache_.wr_idx = 0;
          parse_step_ = kSearchPacketPreamble;
        }

        break;
      }
    } else {
      parse_step_ = kSearchPacketPreamble;
    }
  }

  return kParseFail;
}

uint16_t CommPort::GetAndUpdateSeqNum() {
  // add lock here for thread safe
  uint16_t seq = seq_num_;
  seq_num_++;

  return seq;
}

// only for pack function test use
// uint8_t test_buf[1024];
// uint32_t o_len = 0;
// comm_port_->Pack(test_buf, sizeof(test_buf), &o_len, packet);
// for (uint32_t print_i = 0; print_i < o_len; print_i++) {
//  printf("%.2x_", test_buf[print_i]);
//}
// printf("\r\n");
}  // namespace livox
