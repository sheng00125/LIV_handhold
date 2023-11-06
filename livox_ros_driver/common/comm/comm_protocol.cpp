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

#include "comm_protocol.h"
#include <stdio.h>
#include <string.h>
#include <iostream>

namespace livox_ros {

CommProtocol::CommProtocol(ProtocolConfig &config) : config_(config) {
  ResetParser();
  ResetCache();

  offset_to_read_index_ = 0;
  packet_length_ = 0;

  if (kGps == config.type) {
    is_length_known = false;
    protocol_ = new GpsProtocol();
  } else {
    is_length_known = true;
    protocol_ = nullptr;
  }
}

CommProtocol::~CommProtocol() {
  if (protocol_) {
    delete protocol_;
  }
}

uint8_t *CommProtocol::FetchCacheFreeSpace(uint32_t *o_len) {
  UpdateCache();

  if (cache_.wr_idx < cache_.size) {
    *o_len = cache_.size - cache_.wr_idx;
    return &cache_.buf[cache_.wr_idx];
  } else {
    *o_len = 0;
    return nullptr;
  }
}

int32_t CommProtocol::UpdateCacheWrIdx(uint32_t used_size) {
  if ((cache_.wr_idx + used_size) < cache_.size) {
    cache_.wr_idx += used_size;
    return 0;
  } else {
    return -1;
  }
}

uint32_t CommProtocol::GetCacheTailSize() {
  if (cache_.wr_idx < cache_.size) {
    return cache_.size - cache_.wr_idx;
  } else {
    return 0;
  }
}

uint32_t CommProtocol::GetValidDataSize() {
  if (cache_.wr_idx > cache_.rd_idx) {
    return cache_.wr_idx - cache_.rd_idx;
  } else {
    return 0;
  }
}

void CommProtocol::UpdateCache(void) {
  if (GetCacheTailSize() <
      kMoveCacheLimit) { /* move unused data to cache head */
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

int32_t CommProtocol::Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len,
                           const CommPacket &i_packet) {
  return protocol_->Pack(o_buf, o_buf_size, o_len, i_packet);
}

void CommProtocol::ResetParser() { fsm_parse_step_ = kSearchPacketPreamble; }

int32_t CommProtocol::ParseCommStream(CommPacket *o_pack) {
  int32_t ret = kParseFail;
  while ((GetValidDataSize() > protocol_->GetPreambleLen()) &&
         (GetValidDataSize() > offset_to_read_index_)) {
    switch (fsm_parse_step_) {
      case kSearchPacketPreamble: {
        FsmSearchPacketPreamble();
        break;
      }
      case kFindPacketLength: {
        FsmFindPacketLength();
        break;
      }
      case kGetPacketData: {
        ret = FsmGetPacketData(o_pack);
        break;
      }
      default: { FsmParserStateTransfer(kSearchPacketPreamble); }
    }

    /* Must exit when in the below case */
    if ((ret == kParseSuccess) || (ret == kParseNeedMoreData)) break;
  }

  return ret;
}

uint16_t CommProtocol::GetAndUpdateSeqNum() {
  /* add lock here for thread safe */
  uint16_t seq = seq_num_;
  seq_num_++;

  return seq;
}

int32_t CommProtocol::FsmSearchPacketPreamble() {
  do {
    if (!protocol_->CheckPreamble(GetCacheReadPos())) {
      if (!is_length_known) {
        offset_to_read_index_ = 0;
        packet_length_ = 0;
        FsmParserStateTransfer(kFindPacketLength);
        break;
      } else {
        packet_length_ = protocol_->GetPacketLen(GetCacheReadPos());
        if ((packet_length_ < cache_.size) &&
            (packet_length_ >
             protocol_
                 ->GetPacketWrapperLen())) { /* check the legality of length */
          FsmParserStateTransfer(kGetPacketData);
          break;
        }
      }
    }
    // printf("|%2x",  cache_.buf[cache_.rd_idx]);
    ++cache_.rd_idx; /* skip one byte */
  } while (0);

  return 0;
}

int32_t CommProtocol::FsmFindPacketLength() {
  uint32_t valid_data_size = GetValidDataSize();
  uint32_t ret = protocol_->FindPacketLen(GetCacheReadPos(), valid_data_size);
  if (ret == kFindLengthSuccess) {
    packet_length_ = protocol_->GetPacketLen(GetCacheReadPos());
    FsmParserStateTransfer(kGetPacketData);
    offset_to_read_index_ = 0;
  } else if (ret == kFindLengthContinue) { /* continue to find next time */
    offset_to_read_index_ = valid_data_size;
  } else { /* find length error */
    offset_to_read_index_ = 0;
    cache_.rd_idx += valid_data_size;
    FsmParserStateTransfer(kSearchPacketPreamble);
    printf("Continue to find length error\n");
  }

  return 0;
}

int32_t CommProtocol::FsmGetPacketData(CommPacket *o_pack) {
  int32_t ret = kParseFail;
  do {
    if (GetValidDataSize() < packet_length_) {
      ret = kParseNeedMoreData;
      break;
    }

    if (protocol_->CheckPacket(GetCacheReadPos())) {
      cache_.rd_idx += protocol_->GetPreambleLen();
      FsmParserStateTransfer(kSearchPacketPreamble);
      printf("Check packet error\n");
      break;
    }

    if (protocol_->ParsePacket(GetCacheReadPos(), packet_length_, o_pack)) {
      cache_.rd_idx += protocol_->GetPacketLen(GetCacheReadPos());
      FsmParserStateTransfer(kSearchPacketPreamble);
      printf("Parse packet error\n");
      break;
    }

    cache_.rd_idx += protocol_->GetPacketLen(GetCacheReadPos());
    FsmParserStateTransfer(kSearchPacketPreamble);
    return kParseSuccess;
  } while (0);

  return ret;
}

}  // namespace livox_ros
