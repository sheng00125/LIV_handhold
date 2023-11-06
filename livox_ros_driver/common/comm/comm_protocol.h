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

#ifndef COMM_COMM_PROTOCOL_H_
#define COMM_COMM_PROTOCOL_H_

#include <stdint.h>
#include "gps_protocol.h"
#include "protocol.h"
#include "sdk_protocol.h"

namespace livox_ros {
const uint32_t kCacheSize = 8192;
const uint32_t kMoveCacheLimit = 1536;

enum FsmParseState {
  kSearchPacketPreamble = 0,
  kFindPacketLength = 1,
  kGetPacketData = 2,
  kParseStepUndef
};

/** Communication data cache define */
typedef struct {
  uint8_t buf[kCacheSize];
  uint32_t rd_idx;
  uint32_t wr_idx;
  uint32_t size;
} CommCache;

class CommProtocol {
 public:
  CommProtocol(ProtocolConfig &config);
  ~CommProtocol();

  int32_t Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len,
               const CommPacket &i_packet);

  int32_t ParseCommStream(CommPacket *o_pack);

  uint8_t *FetchCacheFreeSpace(uint32_t *o_len);

  int32_t UpdateCacheWrIdx(uint32_t used_size);

  uint16_t GetAndUpdateSeqNum();

  void ResetParser();

 private:
  uint32_t GetCacheTailSize();
  uint32_t GetValidDataSize();
  void UpdateCache(void);
  uint8_t *GetCacheReadPos() { return &cache_.buf[cache_.rd_idx]; }
  void ResetCache() {
    cache_.wr_idx = 0;
    cache_.rd_idx = 0;
    cache_.size = kCacheSize;
  }

  ProtocolConfig config_;
  Protocol *protocol_;
  CommCache cache_;
  uint16_t seq_num_;

  bool is_length_known;
  bool IsLengthKnown() { return is_length_known; }

  volatile uint32_t offset_to_read_index_;
  uint32_t packet_length_;
  volatile uint32_t fsm_parse_step_;
  int32_t FsmSearchPacketPreamble();
  int32_t FsmFindPacketLength();
  int32_t FsmGetPacketData(CommPacket *o_pack);
  void FsmParserStateTransfer(uint32_t new_state) {
    if (new_state < kParseStepUndef) {
      fsm_parse_step_ = new_state;
    } else {
      fsm_parse_step_ = kSearchPacketPreamble;
    }
  }
};

}  // namespace livox_ros
#endif  // COMM_COMM_PROTOCOL_H_
