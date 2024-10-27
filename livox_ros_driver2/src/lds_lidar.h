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

/** Livox LiDAR data source, data from dependent lidar */

#ifndef LIVOX_ROS_DRIVER_LDS_LIDAR_H_
#define LIVOX_ROS_DRIVER_LDS_LIDAR_H_

#include <memory>
#include <mutex>
#include <vector>

#include "lds.h"
#include "comm/comm.h"

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include "rapidjson/document.h"

namespace livox_ros {

class LdsLidar final : public Lds {
 public:
  static LdsLidar *GetInstance(double publish_freq) {
    printf("LdsLidar *GetInstance\n");
    static LdsLidar lds_lidar(publish_freq);
    return &lds_lidar;
  }

  bool InitLdsLidar(const std::string& path_name);
  bool Start();

  int DeInitLdsLidar(void);
 private:
  LdsLidar(double publish_freq);
  LdsLidar(const LdsLidar &) = delete;
  ~LdsLidar();
  LdsLidar &operator=(const LdsLidar &) = delete;

  bool ParseSummaryConfig();

  bool InitLidars();
  bool InitLivoxLidar();    // for new SDK

  bool LivoxLidarStart();

  void ResetLdsLidar(void);

  void SetLidarPubHandle();

	// auto connect mode
	void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }

  virtual void PrepareExit(void);

 public:
  std::mutex config_mutex_;

 private:
  std::string path_;
  LidarSummaryInfo lidar_summary_info_;


  bool auto_connect_mode_;
  uint32_t whitelist_count_;
  volatile bool is_initialized_;
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LDS_LIDAR_H_
