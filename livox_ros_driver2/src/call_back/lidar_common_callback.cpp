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

#include "lidar_common_callback.h"

#include "../lds_lidar.h"

#include <string>

namespace livox_ros {

void LidarCommonCallback::OnLidarPointClounCb(PointFrame* frame, void* client_data) {
  if (frame == nullptr) {
    printf("LidarPointCloudCb frame is nullptr.\n");
    return;
  }

  if (client_data == nullptr) {
    printf("Lidar point cloud cb failed, client data is nullptr.\n");
    return;
  }

  if (frame->lidar_num ==0) {
    printf("LidarPointCloudCb lidar_num:%u.\n", frame->lidar_num);
    return;
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  
  //printf("Lidar point cloud, lidar_num:%u.\n", frame->lidar_num);

  lds_lidar->StoragePointData(frame);
}

void LidarCommonCallback::LidarImuDataCallback(ImuData* imu_data, void *client_data) {
  if (imu_data == nullptr) {
    printf("Imu data is nullptr.\n");
    return;
  }
  if (client_data == nullptr) {
    printf("Lidar point cloud cb failed, client data is nullptr.\n");
    return;
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  lds_lidar->StorageImuData(imu_data);
}

} // namespace livox_ros



