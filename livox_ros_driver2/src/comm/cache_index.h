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

#ifndef LIVOX_ROS_DRIVER_CACHE_INDEX_H_
#define LIVOX_ROS_DRIVER_CACHE_INDEX_H_

#include <mutex>
#include <array>
#include <map>
#include <string>

#include "comm/comm.h"

namespace livox_ros {

class CacheIndex {
 public:
  CacheIndex();
  int8_t GetFreeIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index);
  int8_t GetIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index);
  int8_t GenerateIndexKey(const uint8_t livox_lidar_type, const uint32_t handle, std::string& key);
  int8_t LvxGetIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index);
  void ResetIndex(LidarDevice *lidar);

 private:
  std::mutex index_mutex_;
  std::map<std::string, uint8_t> map_index_; /* key:handle/slot, val:index */
  std::array<bool, kMaxSourceLidar> index_cache_;
};

} // namespace livox_ros

# endif // LIVOX_ROS_DRIVER_CACHE_INDEX_H_
