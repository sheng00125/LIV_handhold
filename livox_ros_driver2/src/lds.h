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

// livox lidar data source

#ifndef LIVOX_ROS_DRIVER_LDS_H_
#define LIVOX_ROS_DRIVER_LDS_H_

#include <map>

#include "comm/semaphore.h"
#include "comm/comm.h"
#include "comm/cache_index.h"


namespace livox_ros {



/**
 * Lidar data source abstract.
 */
class Lds {
 public:
  Lds(const double publish_freq, const uint8_t data_src);
  virtual ~Lds();

  void StorageImuData(ImuData* imu_data);
  void StoragePointData(PointFrame* frame);
  void StorageLvxPointData(PointFrame* frame);

  int8_t GetHandle(const uint8_t lidar_type, const PointPacket* lidar_point);
  void PushLidarData(PointPacket* lidar_data, const uint8_t index, const uint64_t base_time);

  static void ResetLidar(LidarDevice *lidar, uint8_t data_src);
  static void SetLidarDataSrc(LidarDevice *lidar, uint8_t data_src);
  void ResetLds(uint8_t data_src);

  void RequestExit();

  bool IsAllQueueEmpty();
  bool IsAllQueueReadStop();

  void CleanRequestExit() { request_exit_ = false; }
  bool IsRequestExit() { return request_exit_; }
  virtual void PrepareExit(void);

  // get publishing frequency
  double GetLdsFrequency() { return publish_freq_; }

 public:
  uint8_t lidar_count_;                 /**< Lidar access handle. */
  LidarDevice lidars_[kMaxSourceLidar]; /**< The index is the handle */
  Semaphore pcd_semaphore_;
  Semaphore imu_semaphore_;
  static CacheIndex cache_index_;
 protected:
  double publish_freq_;
  uint8_t data_src_;
 private:
  volatile bool request_exit_;
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LDS_H_
