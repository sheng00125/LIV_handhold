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

// livox lidar data queue

#ifndef LIVOX_ROS_DRIVER_LDQ_H_
#define LIVOX_ROS_DRIVER_LDQ_H_

#include <stdint.h>

namespace livox_ros {

const uint32_t KEthPacketMaxLength = 1500;

#pragma pack(1)

typedef struct {
  uint64_t time_rcv; /**< receive time when data arrive */
  uint32_t point_num;
  uint8_t raw_data[KEthPacketMaxLength];
} StoragePacket;

#pragma pack()

typedef struct {
  StoragePacket *storage_packet;
  volatile uint32_t rd_idx;
  volatile uint32_t wr_idx;
  uint32_t mask;
  uint32_t size; /**< must be power of 2. */
} LidarDataQueue;

inline static bool IsPowerOf2(uint32_t size) {
  return (size != 0) && ((size & (size - 1)) == 0);
}

inline static uint32_t RoundupPowerOf2(uint32_t size) {
  uint32_t power2_val = 0;
  for (int i = 0; i < 32; i++) {
    power2_val = ((uint32_t)1) << i;
    if (size <= power2_val) {
      break;
    }
  }

  return power2_val;
}

/** queue operate function */
int InitQueue(LidarDataQueue *queue, uint32_t queue_size);
int DeInitQueue(LidarDataQueue *queue);
void ResetQueue(LidarDataQueue *queue);
void QueuePrePop(LidarDataQueue *queue, StoragePacket *storage_packet);
void QueuePopUpdate(LidarDataQueue *queue);
uint32_t QueuePop(LidarDataQueue *queue, StoragePacket *storage_packet);
uint32_t QueueUsedSize(LidarDataQueue *queue);
uint32_t QueueUnusedSize(LidarDataQueue *queue);
uint32_t QueueIsFull(LidarDataQueue *queue);
uint32_t QueueIsEmpty(LidarDataQueue *queue);
uint32_t QueuePush(LidarDataQueue *queue, StoragePacket *storage_packet);
uint32_t QueuePushAny(LidarDataQueue *queue, uint8_t *data, uint32_t length,
                      uint64_t time_rcv, uint32_t point_num);

}  // namespace livox_ros
#endif
