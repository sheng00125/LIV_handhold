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

#include "lds_lvx.h"

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <functional>
#include <memory>
#include <thread>

#include "lvx_file.h"

namespace livox_ros {

/** Const varible ------------------------------------------------------------*/
const uint32_t kMaxPacketsNumOfFrame = 8192;

/** For device connect use ---------------------------------------------------*/
LdsLvx::LdsLvx(uint32_t interval_ms) : Lds(interval_ms, kSourceLvxFile) {
  start_read_lvx_ = false;
  is_initialized_ = false;
  lvx_file_ = std::make_shared<LvxFileHandle>();
  packets_of_frame_.buffer_capacity =
      kMaxPacketsNumOfFrame * sizeof(LvxFilePacket);
  packets_of_frame_.packet =
      new uint8_t[kMaxPacketsNumOfFrame * sizeof(LvxFilePacket)];
}

LdsLvx::~LdsLvx() {
  if (packets_of_frame_.packet != nullptr) {
    delete[] packets_of_frame_.packet;
  }
  if (t_read_lvx_->joinable()) {
    t_read_lvx_->join();
  }
}

void LdsLvx::PrepareExit(void) {
  lvx_file_->CloseLvxFile();
  printf("Convert complete, Press [Ctrl+C] to exit!\n");
}

int LdsLvx::InitLdsLvx(const char *lvx_path) {
  if (is_initialized_) {
    printf("Livox file data source is already inited!\n");
    return -1;
  }

  int ret = lvx_file_->Open(lvx_path, std::ios::in);
  if (ret) {
    printf("Open %s file fail[%d]!\n", lvx_path, ret);
    return ret;
  }

  if (lvx_file_->GetFileVersion() == kLvxFileV1) {
    ResetLds(kSourceRawLidar);
  } else {
    ResetLds(kSourceLvxFile);
  }

  uint32_t valid_lidar_count_ = lvx_file_->GetDeviceCount();
  if (!valid_lidar_count_ || (valid_lidar_count_ >= kMaxSourceLidar)) {
    lvx_file_->CloseLvxFile();
    printf("Lidar count error in %s : %d\n", lvx_path, valid_lidar_count_);
    return -1;
  }
  printf("LvxFile[%s] have %d lidars\n", lvx_path, valid_lidar_count_);

  for (uint32_t i = 0; i < valid_lidar_count_; i++) {
    LvxFileDeviceInfo lvx_dev_info;
    lvx_file_->GetDeviceInfo(i, &lvx_dev_info);
    uint8_t handle = lvx_dev_info.device_index;
    if (handle >= kMaxSourceLidar) {
      printf("Invalid hanle from lvx file!\n");
      continue;
    }
    lidars_[handle].handle        = handle;
    lidars_[handle].connect_state = kConnectStateSampling;
    lidars_[handle].info.handle = handle;
    lidars_[handle].info.type   = lvx_dev_info.device_type;
    memcpy(lidars_[handle].info.broadcast_code, \
           lvx_dev_info.lidar_broadcast_code, \
           sizeof(lidars_[handle].info.broadcast_code));

    if (lvx_file_->GetFileVersion() == kLvxFileV1) {
      lidars_[handle].data_src = kSourceRawLidar;
    } else {
      lidars_[handle].data_src = kSourceLvxFile;
    }

    ExtrinsicParameter *p_extrinsic = &lidars_[handle].extrinsic_parameter;
    p_extrinsic->euler[0] = lvx_dev_info.roll * PI / 180.0;
    p_extrinsic->euler[1] = lvx_dev_info.pitch * PI / 180.0;
    p_extrinsic->euler[2] = lvx_dev_info.yaw * PI / 180.0;
    p_extrinsic->trans[0] = lvx_dev_info.x;
    p_extrinsic->trans[1] = lvx_dev_info.y;
    p_extrinsic->trans[2] = lvx_dev_info.z;
    EulerAnglesToRotationMatrix(p_extrinsic->euler, p_extrinsic->rotation);
    p_extrinsic->enable = lvx_dev_info.extrinsic_enable;
  }

  t_read_lvx_ =
      std::make_shared<std::thread>(std::bind(&LdsLvx::ReadLvxFile, this));
  is_initialized_ = true;

  StartRead();

  return ret;
}

/** Global function in LdsLvx for callback */
void LdsLvx::ReadLvxFile() {
  while (!start_read_lvx_);
  printf("Start to read lvx file.\n");

  int file_state = kLvxFileOk;
  int progress = 0;
  while (start_read_lvx_) {
    file_state = lvx_file_->GetPacketsOfFrame(&packets_of_frame_);
    if (!file_state) {
      uint32_t data_size = packets_of_frame_.data_size;
      uint8_t *packet_base = packets_of_frame_.packet;
      uint32_t data_offset = 0;
      while (data_offset < data_size) {
        LivoxEthPacket *eth_packet;
        int32_t handle;
        uint8_t data_type;
        if (lvx_file_->GetFileVersion()) {
          LvxFilePacket *detail_packet =
              (LvxFilePacket *)&packet_base[data_offset];
          eth_packet = (LivoxEthPacket *)(&detail_packet->version);
          handle = detail_packet->device_index;
        } else {
          LvxFilePacketV0 *detail_packet =
              (LvxFilePacketV0 *)&packet_base[data_offset];
          eth_packet = (LivoxEthPacket *)(&detail_packet->version);
          handle = detail_packet->device_index;
        }

        data_type = eth_packet->data_type;
        if (handle >= lvx_file_->GetDeviceCount()) {
          printf("Raw data handle error, error handle is %d\n", handle);
          break;
        }
        if (data_type >= kMaxPointDataType) {
          printf("Raw data type error, error data_type is %d\n", data_type);
          break;
        }
        if (eth_packet->version != 5) {
          printf("EthPacket version[%d] not supported\n", eth_packet->version);
          break;
        }
        
        /** Packet length + device index */
        data_offset += (GetEthPacketLen(data_type) + 1);
        StorageRawPacket(handle, eth_packet);

        LidarDataQueue *p_queue = &lidars_[handle].data;
        if (p_queue != nullptr) {
          while (QueueIsFull(p_queue)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        p_queue = &lidars_[handle].imu_data;
        if (p_queue != nullptr) {
          while (QueueIsFull(p_queue)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
      }
    } else {
      if (file_state != kLvxFileAtEnd) {
        printf("Exit read the lvx file, read file state[%d]!\n", file_state);
      } else {
        printf("Read the lvx file complete!\n");
      }
      break;
    }

    if (progress != lvx_file_->GetLvxFileReadProgress()) {
      progress = lvx_file_->GetLvxFileReadProgress();
      printf("Read progress : %d \n", progress);
    }
  }
  printf("Wait for file conversion to complete!\n");
  int32_t wait_cnt = 5;
  while (!IsAllQueueEmpty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    if (semaphore_.GetCount() <= 0) {
      semaphore_.Signal();
    }
    if (IsAllQueueReadStop()) {
      --wait_cnt;
      if (wait_cnt <= 0) {
        break;
      }
    }
  }

  RequestExit();
  while(semaphore_.GetCount() > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  semaphore_.Signal();
}

}  // namespace livox_ros
