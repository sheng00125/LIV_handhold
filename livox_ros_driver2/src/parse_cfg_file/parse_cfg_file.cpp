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

#include "parse_cfg_file.h"

#include <iostream>
#include <cstdio>
#include <arpa/inet.h>

namespace livox_ros {

ParseCfgFile::ParseCfgFile(const std::string& path) : path_(path) {}

bool ParseCfgFile::ParseSummaryInfo(LidarSummaryInfo& lidar_summary_info) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb");
  if (!raw_file) {
    std::cout << "parse summary info failed, can not open file: " << path_ << std::endl;
    return false;
  }

  char read_buffer[kMaxBufferSize];
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer));
  rapidjson::Document doc;
  do {
    if (doc.ParseStream(config_file).HasParseError()) {
      break;
    }
    if (!doc.HasMember("lidar_summary_info") || !doc["lidar_summary_info"].IsObject()) {
      break;
    }
    const rapidjson::Value &object = doc["lidar_summary_info"];
    if (!object.HasMember("lidar_type") || !object["lidar_type"].IsUint()) {
      break;
    }
    lidar_summary_info.lidar_type = static_cast<uint8_t>(object["lidar_type"].GetUint());
    std::fclose(raw_file);
    return true;
  } while (false);

  std::cout << "parse lidar type failed." << std::endl;
  std::fclose(raw_file);
  return false;
}

} // namespace livox_ros

