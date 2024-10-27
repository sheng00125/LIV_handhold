//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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

#include <stdio.h>
#include <string.h>

#include "ldq.h"

namespace livox_ros {

/* for pointcloud queue process */
bool InitQueue(LidarDataQueue *queue, uint32_t queue_size) {
  if (queue == nullptr) {
    // ROS_WARN("RosDriver Queue: Initialization failed - invalid queue.");
    return false;
  }

  if (!IsPowerOf2(queue_size)) {
    queue_size = RoundupPowerOf2(queue_size);
    printf("Init queue, real query size:%u.\n", queue_size);
  }

  if (queue->storage_packet) {
    delete[] queue->storage_packet;
    queue->storage_packet = nullptr;
  }

  queue->storage_packet = new StoragePacket[queue_size];
  if (queue->storage_packet == nullptr) {
    // ROS_WARN("RosDriver Queue: Initialization failed - failed to allocate memory.");
    return false;
  }

  queue->rd_idx = 0;
  queue->wr_idx = 0;
  queue->size = queue_size;
  queue->mask = queue_size - 1;

  return true;
}

bool DeInitQueue(LidarDataQueue *queue) {
  if (queue == nullptr) {
    // ROS_WARN("RosDriver Queue: Deinitialization failed - invalid queue.");
    return false;
  }

  if (queue->storage_packet) {
    delete[] queue->storage_packet;
  }

  queue->rd_idx = 0;
  queue->wr_idx = 0;
  queue->size = 0;
  queue->mask = 0;

  return true;
}

void ResetQueue(LidarDataQueue *queue) {
  queue->rd_idx = 0;
  queue->wr_idx = 0;
}

bool QueuePrePop(LidarDataQueue *queue, StoragePacket *storage_packet) {
  if (queue == nullptr || storage_packet == nullptr) {
    // ROS_WARN("RosDriver Queue: Invalid pointer parameters.");
    return false;
  }

  if (QueueIsEmpty(queue)) {
    // ROS_WARN("RosDriver Queue: Pop failed, since the queue is empty.");
    return false;
  }

  uint32_t rd_idx = queue->rd_idx & queue->mask;

  storage_packet->base_time = queue->storage_packet[rd_idx].base_time;
  storage_packet->points_num = queue->storage_packet[rd_idx].points_num;
  storage_packet->points.resize(queue->storage_packet[rd_idx].points_num);

  memcpy(storage_packet->points.data(), queue->storage_packet[rd_idx].points.data(), (storage_packet->points_num) * sizeof(PointXyzlt));
  return true;
}

void QueuePopUpdate(LidarDataQueue *queue) {
  queue->rd_idx++;
}

bool QueuePop(LidarDataQueue *queue, StoragePacket *storage_packet) {
  if (!QueuePrePop(queue, storage_packet)) {
    return false;
  }
  QueuePopUpdate(queue);

  return true;
}

uint32_t QueueUsedSize(LidarDataQueue *queue) {
  return queue->wr_idx - queue->rd_idx;
}

uint32_t QueueUnusedSize(LidarDataQueue *queue) {
  return (queue->size - QueueUsedSize(queue));
}

bool QueueIsFull(LidarDataQueue *queue) {
  return ((queue->wr_idx - queue->rd_idx) > queue->mask);
}

bool QueueIsEmpty(LidarDataQueue *queue) {
  return (queue->rd_idx == queue->wr_idx);
}

uint32_t QueuePushAny(LidarDataQueue *queue, uint8_t *data, const uint64_t base_time) {
  uint32_t wr_idx = queue->wr_idx & queue->mask;
  PointPacket* lidar_point_data = reinterpret_cast<PointPacket*>(data);
  queue->storage_packet[wr_idx].base_time = base_time;
  queue->storage_packet[wr_idx].points_num = lidar_point_data->points_num;

  queue->storage_packet[wr_idx].points.clear();
  queue->storage_packet[wr_idx].points.resize(lidar_point_data->points_num);
  memcpy(queue->storage_packet[wr_idx].points.data(), lidar_point_data->points, sizeof(PointXyzlt) * (lidar_point_data->points_num));

  queue->wr_idx++;
  return 1;
}

}  // namespace livox_ros
