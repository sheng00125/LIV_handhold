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

#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string>
#include <map>
#include "livox_sdk.h"


/** Connect all the broadcast device. */
std::map<std::string,std::string> broadcast_code_list;


/** Callback function when broadcast message received. */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);
  
  if (broadcast_code_list.find(info->ip) == broadcast_code_list.end()) {
    broadcast_code_list[info->ip] = info->broadcast_code;
    return;
  }
  
  if (broadcast_code_list[info->ip] == info->broadcast_code) {
    return;
  }
  
  printf("broadcast_code: {%s} is conflicted with broadcast_code: {%s}, which ip is : {%s}\n", \
    broadcast_code_list[info->ip].c_str(), info->broadcast_code, info->ip);
}

int main(int argc, const char *argv[]) {

  printf("Livox SDK initializing.\n");
/** Initialize Livox-SDK. */
  if (!Init()) {
    return -1;
  }
  printf("Livox SDK has been initialized.\n");

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

/** Set the callback function receiving broadcast message from Livox LiDAR. */
  SetBroadcastCallback(OnDeviceBroadcast);

/** Start the device discovering routine. */
  printf("Start IP Conflict detection.\n");
  if (!Start()) {
    Uninit();
    return -1;
  }
  printf("Start discovering device.\n");

#ifdef WIN32
  Sleep(10000);
#else
  sleep(10);
#endif

/** Uninitialize Livox-SDK. */
  Uninit();
}
