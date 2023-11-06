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

#include "lvx_file.h"
#include <string.h>
#include <time.h>
#include <cmath>

#include "lds.h"
#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_utils.hpp"

namespace livox_ros {

#define M_PI 3.14159265358979323846

const uint32_t kMaxLvxFileHeaderLength = 16 * 1024;
const char *kLvxHeaderSigStr = "livox_tech";
const uint32_t kLvxHeaderMagicCode = 0xac0ea767;

LvxFileHandle::LvxFileHandle()
    : file_ver_(kLvxFileV1),
      device_count_(0),
      cur_frame_index_(0),
      cur_offset_(0),
      data_start_offset_(0),
      size_(0),
      mode_(0),
      state_(0) {
  memset((void *)&public_header_, 0, sizeof(public_header_));
  memset((void *)&private_header_, 0, sizeof(private_header_));
  memset((void *)&private_header_v0_, 0, sizeof(private_header_v0_));
}

bool LvxFileHandle::ReadAndCheckHeader() {
  lvx_file_.seekg(0, std::ios::beg);
  lvx_file_.read((char *)(&public_header_), sizeof(public_header_));

  if (strcmp((const char *)public_header_.signature, kLvxHeaderSigStr)) {
    return false;
  }

  /**
  if (public_header_.magic_code != kLvxHeaderMagicCode) {
    return false;
  }
  */
  if (public_header_.version[1] > kLvxFileV1) {
    printf("Unkown lvx file version[%d.%d.%d.%d]\n", public_header_.version[0],
           public_header_.version[1], public_header_.version[2],
           public_header_.version[3]);
    return false;
  }

  file_ver_ = public_header_.version[1];
  printf("Livox file version[%d]\n", file_ver_);

  return true;
}

uint64_t LvxFileHandle::MiniFileSize() {
  if (file_ver_ == kLvxFileV1) {
    return (sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeader) +
            sizeof(LvxFileDeviceInfo) + sizeof(FrameHeader) +
            sizeof(LvxFilePacket));
  } else {
    return (sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeaderV0) +
            sizeof(LvxFileDeviceInfoV0) + sizeof(FrameHeaderV0) +
            sizeof(LvxFilePacketV0));
  }
}

uint64_t LvxFileHandle::PrivateHeaderOffset() {
  return sizeof(LvxFilePublicHeader);
}

uint64_t LvxFileHandle::DataStartOffset() {
  if (file_ver_ == kLvxFileV1) {
    return (sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeader) +
            sizeof(LvxFileDeviceInfo) * private_header_.device_count);
  } else {
    return (sizeof(LvxFilePublicHeader) + sizeof(LvxFilePrivateHeaderV0) +
            sizeof(LvxFileDeviceInfoV0) * private_header_v0_.device_count);
  }
}

bool LvxFileHandle::AddAndCheckDeviceInfo() {
  lvx_file_.seekg(PrivateHeaderOffset(), std::ios::beg);

  if (file_ver_ == kLvxFileV1) {
    lvx_file_.read((char *)(&private_header_), sizeof(private_header_));
    device_count_ = private_header_.device_count;
  } else {
    lvx_file_.read((char *)(&private_header_v0_), sizeof(private_header_v0_));
    device_count_ = private_header_v0_.device_count;
  }

  if (!device_count_) {
    return false;
  }

  for (int i = 0; i < device_count_; i++) {
    LvxFileDeviceInfo device_info;
    if (file_ver_ == kLvxFileV1) {
      lvx_file_.read((char *)(&device_info), sizeof(LvxFileDeviceInfo));
    } else { /* device info v0 to v1 */
      LvxFileDeviceInfoV0 device_info_v0;
      lvx_file_.read((char *)(&device_info_v0), sizeof(LvxFileDeviceInfoV0));
      memcpy((void *)&device_info, (void *)&device_info_v0,
             &device_info.extrinsic_enable - device_info.lidar_broadcast_code);
      memcpy((void *)&device_info.roll, (void *)&device_info_v0.roll,
             sizeof(float) * 6);
      device_info.extrinsic_enable = 0;
    }
    AddDeviceInfo(device_info);
  }

  return true;
}

bool LvxFileHandle::PrepareDataRead() {
  lvx_file_.seekg(DataStartOffset(), std::ios::beg);

  FrameHeader frame_header; /* v0&v1 compatible */
  lvx_file_.read((char *)(&frame_header), sizeof(frame_header));

  if ((frame_header.current_offset != DataStartOffset()) ||
      (frame_header.frame_index != 0)) {
    return false;
  }

  /** reset the read position to the start offset of data erea */
  lvx_file_.seekg(DataStartOffset(), std::ios::beg);

  return true;
}

int LvxFileHandle::Open(const char *filename, std::ios_base::openmode mode) {
  if ((mode & std::ios::in) == std::ios::in) {
    state_ = kLvxFileOk;
    lvx_file_.open(filename, mode | std::ios_base::binary | std::ios_base::ate);

    if (!lvx_file_.is_open()) {
      state_ = kLvxFileNotExist;
      return state_;
    }

    size_ = lvx_file_.tellg();
    lvx_file_.seekg(0, std::ios::beg);
    printf("Filesize %lu\n", size_);

    if (size_ < MiniFileSize()) {
      state_ = kLvxFileSizeFault;

      return state_;
    }

    if (!ReadAndCheckHeader()) {
      state_ = kLvxFileHeaderFault;
      return state_;
    }

    if (!AddAndCheckDeviceInfo()) {
      state_ = kLvxFileDeviceInfoFault;
      return state_;
    }

    if (!PrepareDataRead()) {
      state_ = kLvxFileDataInfoFault;
      return state_;
    }
  } else {
    lvx_file_.open(filename, mode | std::ios_base::binary);

    if (!lvx_file_.is_open()) {
      state_ = kLvxFileNotExist;
      return state_;
    }
  }

  return state_;
}

bool LvxFileHandle::Eof() { return lvx_file_.eof(); }

int LvxFileHandle::InitLvxFile() {
  time_t curtime = time(nullptr);
  char filename[30] = {0};

  tm *local_time = localtime(&curtime);
  strftime(filename, sizeof(filename), "%Y%m%d%H%M%S", local_time);

  return Open(filename, std::ios::out | std::ios::binary);
}

void LvxFileHandle::InitLvxFileHeader() {
  char write_buffer[kMaxLvxFileHeaderLength];
  cur_offset_ = 0;

  std::string signature = kLvxHeaderSigStr;
  memcpy(public_header_.signature, signature.c_str(), signature.size());
  public_header_.version[0] = 1;
  public_header_.version[1] = file_ver_; /* default version 1 */
  public_header_.version[2] = 0;
  public_header_.version[3] = 0;
  public_header_.magic_code = kLvxHeaderMagicCode;
  memcpy(&write_buffer[cur_offset_], (void *)&public_header_,
         sizeof(public_header_));
  cur_offset_ += sizeof(public_header_);

  if (file_ver_ == kLvxFileV1) {
    private_header_.device_count =
        static_cast<uint8_t>(device_info_list_.size());
    private_header_.frame_duration = frame_duration_;
    device_count_ = private_header_.device_count;
    memcpy(&write_buffer[cur_offset_], (void *)&private_header_,
           sizeof(private_header_));
    cur_offset_ += sizeof(private_header_);
  } else {
    private_header_v0_.device_count =
        static_cast<uint8_t>(device_info_list_.size());
    device_count_ = private_header_v0_.device_count;
    memcpy(&write_buffer[cur_offset_], (void *)&private_header_v0_,
           sizeof(private_header_v0_));
    cur_offset_ += sizeof(private_header_v0_);
  }

  for (int i = 0; i < device_count_; i++) {
    if (file_ver_ == kLvxFileV1) {
      memcpy(&write_buffer[cur_offset_], (void *)&device_info_list_[i],
             sizeof(LvxFileDeviceInfo));
      cur_offset_ += sizeof(LvxFileDeviceInfo);
    } else {
      LvxFileDeviceInfoV0 device_info_v0;
      memcpy((void *)&device_info_v0, (void *)&device_info_list_[i],
             &device_info_list_[i].extrinsic_enable -
                 device_info_list_[i].lidar_broadcast_code);
      memcpy((void *)&device_info_v0.roll, (void *)&device_info_list_[i].roll,
             sizeof(float) * 6);
      memcpy(&write_buffer[cur_offset_], (void *)&device_info_v0,
             sizeof(device_info_v0));
      cur_offset_ += sizeof(device_info_v0);
    }
  }

  lvx_file_.write(&write_buffer[cur_offset_], cur_offset_);
}

void LvxFileHandle::SaveFrameToLvxFile(
    std::list<LvxFilePacket> &point_packet_list_temp) {
  uint64_t cur_pos = 0;
  FrameHeader frame_header = {0};
  std::unique_ptr<char[]> write_buffer(new char[kMaxFrameSize]);

  frame_header.current_offset = cur_offset_;
  frame_header.next_offset = cur_offset_ + sizeof(FrameHeader);
  for (auto iter : point_packet_list_temp) {
    frame_header.next_offset += iter.pack_size;
  }

  frame_header.frame_index = cur_frame_index_;

  memcpy(write_buffer.get() + cur_pos, (void *)&frame_header,
         sizeof(FrameHeader));
  cur_pos += sizeof(FrameHeader);

  for (auto iter : point_packet_list_temp) {
    if (cur_pos + iter.pack_size >= kMaxFrameSize) {
      lvx_file_.write((char *)write_buffer.get(), cur_pos);
      cur_pos = 0;
      memcpy(write_buffer.get() + cur_pos, (void *)&(iter), iter.pack_size);
      cur_pos += iter.pack_size;
    } else {
      memcpy(write_buffer.get() + cur_pos, (void *)&(iter), iter.pack_size);
      cur_pos += iter.pack_size;
    }
  }
  lvx_file_.write((char *)write_buffer.get(), cur_pos);

  cur_offset_ = frame_header.next_offset;
  cur_frame_index_++;
}

void LvxFileHandle::CloseLvxFile() {
  if (lvx_file_ && lvx_file_.is_open()) lvx_file_.close();
}

void LvxFileHandle::BasePointsHandle(LivoxEthPacket *data,
                                     LvxFilePacket &packet) {
  memcpy((void *)&packet, (void *)data, GetEthPacketLen(data->data_type));
}

int LvxFileHandle::GetDeviceInfo(uint8_t idx, LvxFileDeviceInfo *info) {
  if (idx < device_info_list_.size()) {
    *info = device_info_list_[idx];
    return 0;
  }

  return -1;
}

int LvxFileHandle::GetPacketsOfFrame(OutPacketBuffer *packets_of_frame) {
  if (!lvx_file_ || lvx_file_.eof()) {
    state_ = kLvxFileAtEnd;
    return kLvxFileAtEnd;
  }

  uint64_t tmp_size = lvx_file_.tellg();
  if (tmp_size >= size_) {
    printf("At the file end %lu\n", tmp_size);
    state_ = kLvxFileAtEnd;
    return kLvxFileAtEnd;
  }

  FrameHeader frame_header;
  FrameHeaderV0 frame_header_v0;
  uint64_t read_length;
  if (file_ver_ == kLvxFileV1) {
    lvx_file_.read((char *)&frame_header, sizeof(frame_header));
    if (!lvx_file_) {
      return kLvxFileReadFail;
    }
    if ((size_ < frame_header.current_offset) ||
        (frame_header.next_offset < frame_header.current_offset)) {
      return kLvxFileFrameHeaderError;
    }
    packets_of_frame->data_size = DataSizeOfFrame(frame_header);
    read_length = packets_of_frame->data_size;
  } else {
    lvx_file_.read((char *)&frame_header_v0, sizeof(frame_header_v0));
    if (!lvx_file_) {
      return kLvxFileReadFail;
    }
    if ((size_ < frame_header_v0.current_offset) ||
        (frame_header_v0.next_offset < frame_header_v0.current_offset)) {
      return kLvxFileFrameHeaderError;
    }
    packets_of_frame->data_size = DataSizeOfFrame(frame_header_v0);
    read_length = packets_of_frame->data_size;
  }
  lvx_file_.read((char *)(packets_of_frame->packet), read_length);
  if (lvx_file_) {
    return kLvxFileOk;
  } else {
    return kLvxFileReadFail;
  }
}

int LvxFileHandle::GetLvxFileReadProgress() {
  if (!size_) {
    return 0;
  }

  if (!lvx_file_.eof()) {
    return (lvx_file_.tellg() * 100ULL) / size_;
  } else {
    return 100;
  }
}

void ParseExtrinsicXml(DeviceItem &item, LvxFileDeviceInfo &info) {
  rapidxml::file<> extrinsic_param("extrinsic.xml");
  rapidxml::xml_document<> doc;
  doc.parse<0>(extrinsic_param.data());
  rapidxml::xml_node<> *root = doc.first_node();
  if ("Livox" == (std::string)root->name()) {
    for (rapidxml::xml_node<> *device = root->first_node(); device;
         device = device->next_sibling()) {
      if ("Device" == (std::string)device->name() &&
          (strncmp(item.info.broadcast_code, device->value(),
                   kBroadcastCodeSize) == 0)) {
        memcpy(info.lidar_broadcast_code, device->value(), kBroadcastCodeSize);
        memset(info.hub_broadcast_code, 0, kBroadcastCodeSize);
        info.device_type = item.info.type;
        info.device_index = item.handle;
        for (rapidxml::xml_attribute<> *param = device->first_attribute();
             param; param = param->next_attribute()) {
          if ("roll" == (std::string)param->name()) {
            info.roll = static_cast<float>(atof(param->value()));
          }
          if ("pitch" == (std::string)param->name()) {
            info.pitch = static_cast<float>(atof(param->value()));
          }
          if ("yaw" == (std::string)param->name()) {
            info.yaw = static_cast<float>(atof(param->value()));
          }
          if ("x" == (std::string)param->name()) {
            info.x = static_cast<float>(atof(param->value()));
          }
          if ("y" == (std::string)param->name()) {
            info.y = static_cast<float>(atof(param->value()));
          }
          if ("z" == (std::string)param->name()) {
            info.z = static_cast<float>(atof(param->value()));
          }
        }
      }
    }
  }
}

}  // namespace livox_ros
