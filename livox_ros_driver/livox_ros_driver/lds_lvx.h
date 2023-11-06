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

// livox lidar lvx data source

#ifndef LIVOX_ROS_DRIVER_LDS_LVX_H_
#define LIVOX_ROS_DRIVER_LDS_LVX_H_

#include <memory>
#include <thread>

#include "lds.h"
#include "lvx_file.h"

namespace livox_ros {

/**
 * Lidar data source abstract.
 */
class LdsLvx : public Lds {
 public:
  static LdsLvx *GetInstance(uint32_t interval_ms) {
    static LdsLvx lds_lvx(interval_ms);
    return &lds_lvx;
  }

  int InitLdsLvx(const char *lvx_path);
  int DeInitLdsLvx(void);
  void PrepareExit(void);

 private:
  LdsLvx(uint32_t interval_ms);
  LdsLvx(const LdsLvx &) = delete;
  ~LdsLvx();
  LdsLvx &operator=(const LdsLvx &) = delete;

  void StartRead() { start_read_lvx_ = true; }
  void StopRead() { start_read_lvx_ = false; }
  bool IsStarted() { return start_read_lvx_; }

  void ReadLvxFile();

  volatile bool is_initialized_;
  OutPacketBuffer packets_of_frame_;
  std::shared_ptr<LvxFileHandle> lvx_file_;
  std::shared_ptr<std::thread> t_read_lvx_;
  volatile bool start_read_lvx_;
};

}  // namespace livox_ros
#endif
