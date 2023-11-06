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

#include <condition_variable>
#include <memory>
#include <fstream>
#include <list>
#include <vector>
#include <mutex>
#include "livox_sdk.h"


#define kMaxPointSize 1500
#define kDefaultFrameDurationTime 50

typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
} DeviceItem;

#pragma pack(1)

typedef struct {
  uint8_t signature[16];
  uint8_t version[4];
  uint32_t magic_code;
} LvxFilePublicHeader;

typedef struct {
  uint32_t frame_duration;
  uint8_t device_count;
} LvxFilePrivateHeader;


typedef struct {
  uint8_t lidar_broadcast_code[16];
  uint8_t hub_broadcast_code[16];
  uint8_t device_index;
  uint8_t device_type;
  uint8_t extrinsic_enable;
  float roll;
  float pitch;
  float yaw;
  float x;
  float y;
  float z;
} LvxDeviceInfo;

typedef struct {
  uint8_t device_index;
  uint8_t version;
  uint8_t port_id;
  uint8_t lidar_index;
  uint8_t rsvd;
  uint32_t error_code;
  uint8_t timestamp_type;
  uint8_t data_type;
  uint8_t timestamp[8];
  uint8_t raw_point[kMaxPointSize];
  uint32_t pack_size;
} LvxBasePackDetail;

typedef struct {
  uint64_t current_offset;
  uint64_t next_offset;
  uint64_t frame_index;
} FrameHeader;

#pragma pack()

class LvxFileHandle {
public:
  LvxFileHandle();

  bool InitLvxFile();
  void InitLvxFileHeader();
  void SaveFrameToLvxFile(std::list<LvxBasePackDetail> &point_packet_list_temp);
  void CloseLvxFile();

  void AddDeviceInfo(LvxDeviceInfo &info) { device_info_list_.push_back(info); };
  int GetDeviceInfoListSize() { return device_info_list_.size(); }

  void BasePointsHandle(LivoxEthPacket *data, LvxBasePackDetail &packet);

private:
  std::ofstream lvx_file_;
  std::vector<LvxDeviceInfo> device_info_list_;
  uint32_t cur_frame_index_;
  uint64_t cur_offset_;
  uint32_t frame_duration_;
};