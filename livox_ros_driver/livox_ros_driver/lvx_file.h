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
#ifndef LIVOX_FILE_H_
#define LIVOX_FILE_H_

#include <fstream>
#include <ios>
#include <list>
#include <memory>
#include <mutex>
#include <vector>
#include "livox_sdk.h"

namespace livox_ros {

#define kMaxPointSize 1500
#define kDefaultFrameDurationTime 50
const uint32_t kMaxFrameSize = 2048 * 1024;

typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef enum {
  kLvxFileOk = 0,
  kLvxFileNotExist,
  kLvxFileSizeFault,
  kLvxFileHeaderFault,
  kLvxFileDeviceInfoFault,
  kLvxFileDataInfoFault,
  kLvxFileAtEnd,
  kLvxFileReadFail,
  kLvxFileFrameHeaderError,
  kLvxFileUndefFault
} LvxFileState;

typedef enum {
  kLvxFileV0 = 0,
  kLvxFileV1 = 1,
  kLvxFileVersionUndef = 2,
} LvxFileVersion;

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
} LvxFileDeviceInfo;

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
} LvxFilePacket;

typedef struct {
  uint64_t current_offset;
  uint64_t next_offset;
  uint64_t frame_index;
} FrameHeader;

typedef struct {
  FrameHeader header;
  LvxFilePacket *packet;
} LvxFileFrame;

typedef struct { uint8_t device_count; } LvxFilePrivateHeaderV0;

typedef struct {
  uint8_t lidar_broadcast_code[16];
  uint8_t hub_broadcast_code[16];
  uint8_t device_index;
  uint8_t device_type;
  float roll;
  float pitch;
  float yaw;
  float x;
  float y;
  float z;
} LvxFileDeviceInfoV0;

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
  LivoxPoint raw_point[100];
} LvxFilePacketV0;

typedef struct {
  uint64_t current_offset;
  uint64_t next_offset;
  uint64_t frame_index;
  uint64_t packet_count;
} FrameHeaderV0;

typedef struct {
  FrameHeaderV0 header;
  LvxFilePacketV0 *packet;
} LvxFileFrameV0;

typedef struct {
  uint32_t buffer_capacity; /* max buffer size */
  uint32_t data_size;       /* frame data erea size */
  uint8_t *packet;          /* packet data erea */
} OutPacketBuffer;

#pragma pack()

class LvxFileHandle {
 public:
  LvxFileHandle();
  ~LvxFileHandle() = default;

  int Open(const char *filename, std::ios_base::openmode mode);
  bool Eof();

  int InitLvxFile();
  void InitLvxFileHeader();
  void SaveFrameToLvxFile(std::list<LvxFilePacket> &point_packet_list_temp);
  void BasePointsHandle(LivoxEthPacket *data, LvxFilePacket &packet);
  void CloseLvxFile();

  void AddDeviceInfo(LvxFileDeviceInfo &info) {
    device_info_list_.push_back(info);
  }
  int GetDeviceInfoListSize() { return device_info_list_.size(); }
  int GetDeviceCount() { return device_count_; }
  int GetDeviceInfo(uint8_t idx, LvxFileDeviceInfo *info);
  int GetFileState(void) { return state_; };
  int GetPacketsOfFrame(OutPacketBuffer *PacketsOfFrame);
  int GetLvxFileReadProgress();
  int GetFileVersion() { return file_ver_; }

 private:
  std::fstream lvx_file_;
  std::vector<LvxFileDeviceInfo> device_info_list_;
  uint8_t file_ver_;
  uint8_t device_count_;
  LvxFilePublicHeader public_header_;
  LvxFilePrivateHeader private_header_;
  LvxFilePrivateHeaderV0 private_header_v0_;
  uint32_t cur_frame_index_;
  uint64_t cur_offset_;
  uint32_t frame_duration_;
  uint64_t data_start_offset_;
  uint64_t size_;
  int mode_;
  int state_;

  uint64_t MiniFileSize();
  uint64_t PrivateHeaderOffset();
  uint64_t DataStartOffset();
  uint32_t PacketNumOfFrame();

  bool ReadAndCheckHeader();
  bool AddAndCheckDeviceInfo();
  bool PrepareDataRead();

  uint64_t DataSizeOfFrame(FrameHeader &frame_header) {
    return (frame_header.next_offset - frame_header.current_offset -
            sizeof(frame_header));
  }

  uint64_t DataSizeOfFrame(FrameHeaderV0 &frame_header_v0) {
    return (frame_header_v0.next_offset - frame_header_v0.current_offset -
            sizeof(frame_header_v0));
  }
};

}  // namespace livox_ros
#endif
