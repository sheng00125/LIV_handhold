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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <algorithm>

#include "lds.h"
#include "comm/ldq.h"

namespace livox_ros {

CacheIndex Lds::cache_index_;

/* Member function --------------------------------------------------------- */
Lds::Lds(const double publish_freq, const uint8_t data_src)
    : lidar_count_(kMaxSourceLidar),
      pcd_semaphore_(0),
      imu_semaphore_(0),
      publish_freq_(publish_freq),
      data_src_(data_src),
      request_exit_(false) {
  ResetLds(data_src_);
}

Lds::~Lds() {
  lidar_count_ = 0;
  ResetLds(0);
  printf("lds destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

void Lds::ResetLidar(LidarDevice *lidar, uint8_t data_src) {
  //cache_index_.ResetIndex(lidar);
  DeInitQueue(&lidar->data);
  lidar->imu_data.Clear();

  lidar->data_src = data_src;
  lidar->connect_state = kConnectStateOff;
}

void Lds::SetLidarDataSrc(LidarDevice *lidar, uint8_t data_src) {
  lidar->data_src = data_src;
}

void Lds::ResetLds(uint8_t data_src) {
  lidar_count_ = kMaxSourceLidar;
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    ResetLidar(&lidars_[i], data_src);
  }
}

void Lds::RequestExit() {
  request_exit_ = true;
}

bool Lds::IsAllQueueEmpty() {
  for (int i = 0; i < lidar_count_; i++) {
    if (!QueueIsEmpty(&lidars_[i].data)) {
      return false;
    }
  }
  return true;
}

bool Lds::IsAllQueueReadStop() {
  for (int i = 0; i < lidar_count_; i++) {
    uint32_t data_size = QueueUsedSize(&lidars_[i].data);
    if (data_size) {
      return false;
    }
  }
  return true;
}

void Lds::StorageImuData(ImuData* imu_data) {
  uint32_t device_num = 0;
  if (imu_data->lidar_type == kLivoxLidarType) {
    device_num = imu_data->handle;
  } else {
    printf("Storage imu data failed, unknown lidar type:%u.\n", imu_data->lidar_type);
    return;
  }

  uint8_t index = 0;
  int ret = cache_index_.GetIndex(imu_data->lidar_type, device_num, index);
  if (ret != 0) {
    printf("Storage point data failed, can not get index, lidar type:%u, device_num:%u.\n", imu_data->lidar_type, device_num);
    return;
  }

  LidarDevice *p_lidar = &lidars_[index];
  LidarImuDataQueue* imu_queue = &p_lidar->imu_data;
  imu_queue->Push(imu_data);
  if (!imu_queue->Empty()) {
    if (imu_semaphore_.GetCount() <= 0) {
      imu_semaphore_.Signal();
    }
  }
}

void Lds::StorageLvxPointData(PointFrame* frame) {
  if (frame == nullptr) {
    return;
  }

  uint8_t lidar_number = frame->lidar_num;
  for (uint i = 0; i < lidar_number; ++i) {
    PointPacket& lidar_point = frame->lidar_point[i];

    uint64_t base_time = frame->base_time[i];
    uint8_t index = 0;
    int8_t ret = cache_index_.LvxGetIndex(lidar_point.lidar_type, lidar_point.handle, index);
    if (ret != 0) {
      printf("Storage lvx point data failed, lidar type:%u, device num:%u.\n", lidar_point.lidar_type, lidar_point.handle);
      continue;
    }

    lidars_[index].connect_state = kConnectStateSampling;

    PushLidarData(&lidar_point, index, base_time);
  }
}

void Lds::StoragePointData(PointFrame* frame) {
  if (frame == nullptr) {
    return;
  }

  uint8_t lidar_number = frame->lidar_num;
  for (uint i = 0; i < lidar_number; ++i) {
    PointPacket& lidar_point = frame->lidar_point[i];
    //printf("StoragePointData, lidar_type:%u, point_num:%lu.\n", lidar_point.lidar_type, lidar_point.points_num);

    uint64_t base_time = frame->base_time[i];

    uint8_t index = 0;
    int8_t ret = cache_index_.GetIndex(lidar_point.lidar_type, lidar_point.handle, index);
    if (ret != 0) {
      printf("Storage point data failed, lidar type:%u, handle:%u.\n", lidar_point.lidar_type, lidar_point.handle);
      continue;
    }
    PushLidarData(&lidar_point, index, base_time);
  }
}

void Lds::PushLidarData(PointPacket* lidar_data, const uint8_t index, const uint64_t base_time) {
  if (lidar_data == nullptr) {
    return;
  }

  LidarDevice *p_lidar = &lidars_[index];
  LidarDataQueue *queue = &p_lidar->data;

  if (nullptr == queue->storage_packet) {
    uint32_t queue_size = CalculatePacketQueueSize(publish_freq_);
    InitQueue(queue, queue_size);
    printf("Lidar[%u] storage queue size: %u\n", index, queue_size);
  }

  if (!QueueIsFull(queue)) {
    QueuePushAny(queue, (uint8_t *)lidar_data, base_time);
    if (!QueueIsEmpty(queue)) {
      if (pcd_semaphore_.GetCount() <= 0) {
        pcd_semaphore_.Signal();
      }
    }
  } else {
    if (pcd_semaphore_.GetCount() <= 0) {
        pcd_semaphore_.Signal();
    }
  }
}

void Lds::PrepareExit(void) {}

}  // namespace livox_ros
