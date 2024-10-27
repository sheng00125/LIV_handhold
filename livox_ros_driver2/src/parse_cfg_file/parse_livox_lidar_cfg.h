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

#ifndef LIVOX_ROS_DRIVER_LIVOX_LIDAR_CFG_PARSER_H_
#define LIVOX_ROS_DRIVER_LIVOX_LIDAR_CFG_PARSER_H_

#include "comm/comm.h"
#include "livox_lidar_def.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/stringbuffer.h"

#include <iostream>
#include <string>
#include <vector>

namespace livox_ros {
  
class LivoxLidarConfigParser {
 public:
  explicit LivoxLidarConfigParser(const std::string& path)  : path_(path) {}
  ~LivoxLidarConfigParser() {}

  bool Parse(std::vector<UserLivoxLidarConfig> &lidar_configs);

 private:
  bool ParseUserConfigs(const rapidjson::Document &doc,
                         std::vector<UserLivoxLidarConfig> &user_configs);
  bool ParseExtrinsics(const rapidjson::Value &value, ExtParameter &param);

  const std::string path_;
};

} // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LIVOX_LIDAR_CFG_PARSER_H_
