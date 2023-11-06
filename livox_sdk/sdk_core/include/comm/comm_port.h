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

#ifndef COMM_COMM_PORT_H_
#define COMM_COMM_PORT_H_

#include <stdint.h>
#include "protocol.h"

namespace livox {
const uint32_t kCacheSize = 8192;
const uint32_t kMoveCacheLimit = 1536;

typedef struct {
  uint8_t buf[kCacheSize];
  uint32_t rd_idx;
  uint32_t wr_idx;
  uint32_t size;
} PortCache;

class CommPort {
 public:
  CommPort();

  ~CommPort();

  int32_t Pack(uint8_t *o_buf, uint32_t o_buf_size, uint32_t *o_len, const CommPacket &i_packet);

  int32_t ParseCommStream(CommPacket *o_pack);

  uint8_t *FetchCacheFreeSpace(uint32_t *o_len);

  int32_t UpdateCacheWrIdx(uint32_t used_size);

  uint16_t GetAndUpdateSeqNum();

 private:
  uint32_t GetCacheTailSize();

  uint32_t GetValidDataSize();

  void UpdateCache(void);

  PortCache cache_;
  Protocol *protocol_;
  uint32_t parse_step_;
  uint16_t seq_num_;
};

}  // namespace livox
#endif  // COMM_COMM_PORT_H_
