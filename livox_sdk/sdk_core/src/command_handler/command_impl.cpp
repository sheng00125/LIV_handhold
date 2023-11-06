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

#include <livox_sdk.h>

#include "command_handler.h"
#include "command_impl.h"
#include "data_handler/data_handler.h"
#include "device_manager.h"
#include "livox_def.h"
#include "livox_sdk.h"

#define HMS_PARSE_BUF (128)
#define YY_PARSE_BUF (128)

using std::pair;
using std::vector;
using namespace livox;

void SetDeviceStateUpdateCallback(DeviceStateUpdateCallback cb) {
  device_manager().SetDeviceConnectedCallback(cb);
}

void SetBroadcastCallback(DeviceBroadcastCallback cb) {
  device_manager().SetDeviceBroadcastCallback(cb);
}

livox_status AddHubToConnect(const char *broadcast_code, uint8_t *handle) {
  bool result = device_manager().AddListeningDevice(broadcast_code,
                                                    kDeviceModeHub, *handle);
  if (result) {
    return kStatusSuccess;
  } else {
    return kStatusFailure;
  }
}

livox_status AddLidarToConnect(const char *broadcast_code, uint8_t *handle) {
  bool result = device_manager().AddListeningDevice(broadcast_code,
                                                    kDeviceModeLidar, *handle);
  if (result) {
    return kStatusSuccess;
  } else {
    return kStatusFailure;
  }
}

livox_status GetConnectedDevices(DeviceInfo *devices, uint8_t *size) {
  if (devices == NULL || size == NULL) {
    return kStatusFailure;
  }
  vector<DeviceInfo> device;
  device_manager().GetConnectedDevices(device);
  uint16_t final_size = (*size >= device.size()) ? device.size() : *size;
  for (int i = 0; i < final_size; i++) {
    devices[i] = device[i];
  }
  *size = final_size;
  return kStatusSuccess;
}

void SetDataCallback(uint8_t handle, DataCallback cb, void *client_data) {
  data_handler().AddDataListener(handle, cb, client_data);
}

livox_status DeviceSampleControl(uint8_t handle, bool enable,
                                 CommonCommandCallback cb, void *client_data) {
  uint8_t req = enable;
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralControlSample, &req,
      sizeof(req), MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status LidarFanControl(uint8_t handle, bool enable,
                             CommonCommandCallback cb, void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar ||
      device_manager().IsLidarMid40(handle)) {
    return kStatusNotSupported;
  }
  uint8_t req = static_cast<uint8_t>(enable);
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarControlFan, &req, sizeof(req),
      MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status SetDeviceIp(uint8_t handle, SetDeviceIpExtendModeRequest *req,
                         CommonCommandCallback cb, void *client_data) {
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralConfigureStaticDynamicIp,
      (uint8_t *)req, sizeof(*req),
      MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

bool ChecksumRmc(const char *buff_begin, const char *buff_end) {
  // checksum $GPRMC/$GNRMC format : *hh
  const uint8_t head_size = strlen("$");
  const uint8_t tail_size = strlen("hh");
  char tail[3] = {0};
  uint32_t tail_num = 0;
  uint8_t crc = 0;
  buff_begin += head_size;

  for (; buff_begin < buff_end; buff_begin++) {
    if (*buff_begin == '*') {
      if (buff_begin + tail_size <= buff_end) {
        strncpy(tail, &buff_begin[1], tail_size);
        sscanf(tail, "%x", &tail_num);
        crc ^= (uint8_t)tail_num;
        if (crc == 0) {
          return true;
        }
        break;
      }
    }
    crc ^= *buff_begin;
  }
  return false;
}

bool ParseRmcTime(const char *rmc, uint16_t rmc_len,
                  LidarSetUtcSyncTimeRequest *utc_time_req) {

  const char *rmc_begin = strstr(rmc, "$GPRMC");
  if (rmc_begin == NULL) {
    // 返回字符串中首次出现子串的地址
    rmc_begin = strstr(rmc, "$GNRMC");
  }
  if (rmc_begin == NULL) {
    printf("rmc_begin null");
    // return false;
  }
  // 校验一下
  // if (!ChecksumRmc(rmc_begin, rmc + rmc_len)) {
  //   return false;
  // }

  char utc_hms_buff[HMS_PARSE_BUF] = {0};
  char utc_yy_buff[YY_PARSE_BUF] = {0};
  int hour = 0, minute = 0, second = 0;

  memset(utc_time_req, 0, sizeof(LidarSetUtcSyncTimeRequest));
  // 自己加

  sscanf(rmc_begin,
         "$G%*[NP]RMC,%[^,],%*C,%*f,%*C,%*f,%*C,%*f,%*f,%2hhu%2hhu%[^,],%*f,",
         &utc_hms_buff[0], &(utc_time_req->day), &(utc_time_req->month),
         &utc_yy_buff[0]);

  sscanf(utc_hms_buff, "%2d%2d%2d", &hour, &minute, &second);
  sscanf(utc_yy_buff, "%*C%*C%2hhu", &(utc_time_req->year));
  printf("year:%d,month:%d,day:%d,h,m,s:%d,%d,%d\n", utc_time_req->year,
         utc_time_req->month, utc_time_req->day, hour, minute, second);
  // utc_time_req->year = 0;
  // utc_time_req->month = 0;
  // utc_time_req->day = 0;
  utc_time_req->hour = hour;
  utc_time_req->microsecond = (minute * 60 * 1000 + second * 1000) * 1000;
  printf("\n!!!!!!!!!!!!!!%d\n", utc_time_req->microsecond);
  return true;
  //从字符串读取格式化输入
  int num = sscanf(
      rmc_begin,
      "$G%*[NP]RMC,%[^,],%*C,%*f,%*C,%*f,%*C,%*f,%*f,%2hhu%2hhu%[^,],%*f,",
      &utc_hms_buff[0], &(utc_time_req->day), &(utc_time_req->month),
      &utc_yy_buff[0]);
  // hms day month year
  if (num != 4) {
    num = sscanf(
        rmc_begin,
        "$G%*[NP]RMC,%[^,],%*C,%*f,%*C,%*f,%*C,%*f,,%2hhu%2hhu%[^,],%*f,",
        &utc_hms_buff[0], &(utc_time_req->day), &(utc_time_req->month),
        &utc_yy_buff[0]);
  }

  if (num != 4) {
    printf("num != 4");
    return false;
  }
  sscanf(utc_hms_buff, "%2d%2d%2d", &hour, &minute, &second);
  printf("%d", (hour));
  int time_buff_size = strlen(utc_yy_buff);
  uint8_t temp_time = 0;
  switch (time_buff_size) {
  case sizeof("yyyy") - 1:
    if (strncmp(utc_yy_buff, "2000", strlen("2000")) < 0) {
      //
      printf("utc_yy_buff");
      return false;
    }
    if (1 != sscanf(utc_yy_buff, "%*C%*C%2hhu", &(utc_time_req->year))) {
      printf("2");
      return false;
    }
    temp_time = utc_time_req->day;
    utc_time_req->day = utc_time_req->month;
    utc_time_req->month = temp_time;
    break;
  case sizeof("yy") - 1:
    if (1 != sscanf(utc_yy_buff, "%2hhu", &(utc_time_req->year))) {
      printf("3");
      return false;
    }
    break;
  default:
    printf("4");
    return false;
  }

  if (3 != sscanf(utc_hms_buff, "%2d%2d%2d", &hour, &minute, &second)) {
    printf("5");
    return false;
  }

  utc_time_req->hour = hour;
  utc_time_req->microsecond = (minute * 60 * 1000 + second * 1000) * 1000;
  return true;
}

livox_status SetDeviceParameters(uint8_t handle, uint8_t *req, uint16_t length,
                                 SetDeviceParametersCallback cb,
                                 void *client_data) {

  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralSetDeviceParam, req, length,
      MakeCommandCallback<DeviceParameterResponse>(cb, client_data));
  return result;
}

livox_status GetDeiveParameter(uint8_t handle, GetDeviceParameterRequest *req,
                               GetDeviceParametersCallback cb,
                               void *client_data) {
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralGetDeviceParam,
      (uint8_t *)req, sizeof(GetDeviceParameterRequest),
      MakeCommandCallback<GetDeviceParameterResponse>(cb, client_data));
  return result;
}

livox_status LidarStartSampling(uint8_t handle, CommonCommandCallback cb,
                                void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar) {
    return kStatusNotSupported;
  }
  return DeviceSampleControl(handle, true, cb, client_data);
}

livox_status LidarStopSampling(uint8_t handle, CommonCommandCallback cb,
                               void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar) {
    return kStatusNotSupported;
  }
  return DeviceSampleControl(handle, false, cb, client_data);
}

livox_status HubStartSampling(CommonCommandCallback cb, void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  return DeviceSampleControl(kHubDefaultHandle, true, cb, client_data);
}

livox_status HubStopSampling(CommonCommandCallback cb, void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  return DeviceSampleControl(kHubDefaultHandle, false, cb, client_data);
}

uint8_t HubGetLidarHandle(uint8_t slot, uint8_t id) {
  return (slot - 1) * 3 + id - 1;
}

livox_status QueryDeviceInformation(uint8_t handle,
                                    DeviceInformationCallback cb,
                                    void *client_data) {
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralDeviceInfo, NULL, 0,
      MakeCommandCallback<DeviceInformationResponse>(cb, client_data));
  return result;
}

livox_status DisconnectDevice(uint8_t handle, CommonCommandCallback cb,
                              void *client_data) {
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralDisconnect, NULL, 0,
      MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status SetCartesianCoordinate(uint8_t handle, CommonCommandCallback cb,
                                    void *client_data) {
  uint8_t req = 0;
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralCoordinateSystem, &req,
      sizeof(req), MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status SetSphericalCoordinate(uint8_t handle, CommonCommandCallback cb,
                                    void *client_data) {
  uint8_t req = 1;
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralCoordinateSystem, &req,
      sizeof(req), MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status SetErrorMessageCallback(uint8_t handle, ErrorMessageCallback cb) {
  livox_status result = command_handler().RegisterPush(
      handle, kCommandSetGeneral, kCommandIDGeneralPushAbnormalState,
      MakeMessageCallback<ErrorMessage>(cb));
  return result;
}

livox_status SetStaticDynamicIP(uint8_t handle, SetDeviceIPModeRequest *req,
                                CommonCommandCallback cb, void *client_data) {
  if (device_manager().device_mode() == kDeviceModeLidar &&
      !device_manager().IsLidarMid40(handle)) {
    return kStatusNotSupported;
  }
  SetDeviceIpExtendModeRequest ext_req;
  ext_req.ip_mode = req->ip_mode;
  ext_req.ip_addr = req->ip_addr;
  uint8_t net_mask[4] = {255, 255, 255, 0};
  memcpy(&ext_req.net_mask, net_mask, 4);
  uint8_t gw_addr[4] = {192, 168, 1, 1};
  memcpy(&ext_req.gw_addr, gw_addr, 4);
  return SetDeviceIp(handle, &ext_req, cb, client_data);
}

livox_status SetDynamicIp(uint8_t handle, CommonCommandCallback cb,
                          void *client_data) {
  SetDeviceIpExtendModeRequest req;
  req.ip_mode = kLidarDynamicIpMode;
  return SetDeviceIp(handle, &req, cb, client_data);
}

livox_status SetStaticIp(uint8_t handle, SetStaticDeviceIpModeRequest *req,
                         CommonCommandCallback cb, void *client_data) {
  SetDeviceIpExtendModeRequest static_ip_req = {
      kLidarStaticIpMode, req->ip_addr, req->net_mask, req->gw_addr};
  return SetDeviceIp(handle, &static_ip_req, cb, client_data);
}

livox_status GetDeviceIpInformation(uint8_t handle,
                                    GetDeviceIpInformationCallback cb,
                                    void *client_data) {
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralGetDeviceIpInformation, NULL,
      0, MakeCommandCallback<GetDeviceIpModeResponse>(cb, client_data));
  return result;
}

livox_status RebootDevice(uint8_t handle, uint16_t timeout,
                          CommonCommandCallback cb, void *client_data) {
  if (device_manager().IsLidarMid40(handle)) {
    uint32_t firm_ver = 0;
    // Mid40 firmware version 03.07.0000
    uint32_t min_firm_ver = (uint32_t)3 << 24 | 7 << 16 | 0 << 8 | 0;
    device_manager().GetLidarFirmwareVersion(handle, firm_ver);
    if (firm_ver < min_firm_ver) {
      return kStatusNotSupported;
    }
  }

  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralRebootDevice,
      (uint8_t *)(&timeout), sizeof(timeout),
      MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status LidarEnableHighSensitivity(uint8_t handle,
                                        SetDeviceParametersCallback cb,
                                        void *client_data) {
  if (!device_manager().IsLidarTele(handle) &&
      !device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }

  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  KeyValueParam *kv = (KeyValueParam *)&req_buff[0];
  kv->key = static_cast<uint16_t>(kKeyHighSensetivity);
  kv->length = 1;
  kv->value[0] = static_cast<uint8_t>(true);
  req_len = sizeof(KeyValueParam);

  return SetDeviceParameters(handle, req_buff, req_len, cb, client_data);
}

livox_status LidarDisableHighSensitivity(uint8_t handle,
                                         SetDeviceParametersCallback cb,
                                         void *client_data) {
  if (!device_manager().IsLidarTele(handle) &&
      !device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  KeyValueParam *kv = (KeyValueParam *)&req_buff[0];
  kv->key = static_cast<uint16_t>(kKeyHighSensetivity);
  kv->length = 1;
  kv->value[0] = static_cast<uint8_t>(false);
  req_len = sizeof(KeyValueParam);

  return SetDeviceParameters(handle, req_buff, req_len, cb, client_data);
}

livox_status LidarGetHighSensitivityState(uint8_t handle,
                                          GetDeviceParametersCallback cb,
                                          void *client_data) {
  if (!device_manager().IsLidarTele(handle) &&
      !device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }

  GetDeviceParameterRequest req;
  req.param_num = 1;
  req.key[0] = static_cast<uint16_t>(kKeyHighSensetivity);
  return GetDeiveParameter(handle, &req, cb, client_data);
}

livox_status LidarSetSlotNum(uint8_t handle, uint8_t slot,
                             SetDeviceParametersCallback cb,
                             void *client_data) {
  if (!device_manager().IsLidarMid70(handle) &&
      !device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  KeyValueParam *kv = (KeyValueParam *)&req_buff[0];
  kv->key = static_cast<uint16_t>(kKeySlotNum);
  kv->length = 1;
  kv->value[0] = slot;
  req_len = sizeof(KeyValueParam);

  return SetDeviceParameters(handle, req_buff, req_len, cb, client_data);
}

livox_status LidarGetSlotNum(uint8_t handle, GetDeviceParametersCallback cb,
                             void *client_data) {
  if (!device_manager().IsLidarMid70(handle) &&
      !device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }

  GetDeviceParameterRequest req;
  req.param_num = 1;
  req.key[0] = static_cast<uint16_t>(kKeySlotNum);
  return GetDeiveParameter(handle, &req, cb, client_data);
}

livox_status LidarGetScanPattern(uint8_t handle, GetDeviceParametersCallback cb,
                                 void *client_data) {
  if (!device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }
  GetDeviceParameterRequest req;
  req.param_num = 1;
  req.key[0] = static_cast<uint16_t>(kKeyScanPattern);
  return GetDeiveParameter(handle, &req, cb, client_data);
}

livox_status LidarSetScanPattern(uint8_t handle, LidarScanPattern pattern,
                                 SetDeviceParametersCallback cb,
                                 void *client_data) {
  if (!device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  KeyValueParam *kv = (KeyValueParam *)&req_buff[0];
  kv->key = static_cast<uint16_t>(kKeyScanPattern);
  kv->length = 1;
  kv->value[0] = static_cast<uint8_t>(pattern);
  req_len = sizeof(KeyValueParam);
  return SetDeviceParameters(handle, req_buff, req_len, cb, client_data);
}

livox_status DeviceResetAllParameters(uint8_t handle,
                                      DeviceResetParametersCallback cb,
                                      void *client_data) {
  if (!device_manager().IsLidarAvia(handle) &&
      !device_manager().IsLidarMid70(handle) &&
      !device_manager().IsLidarTele(handle)) {
    return kStatusNotSupported;
  }
  ResetDeviceParameterRequest req = {};
  req.flag = 0; // Reset all keys
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralResetDeviceParam,
      (uint8_t *)&req, sizeof(req),
      MakeCommandCallback<DeviceParameterResponse>(cb, client_data));
  return result;
}

livox_status DeviceResetParameters(uint8_t handle, DeviceParamKeyName *keys,
                                   uint8_t keys_num,
                                   DeviceResetParametersCallback cb,
                                   void *client_data) {
  if (!device_manager().IsLidarAvia(handle) &&
      !device_manager().IsLidarMid70(handle) &&
      !device_manager().IsLidarTele(handle)) {
    return kStatusNotSupported;
  }
  uint8_t req_buff[kMaxCommandBufferSize] = {0};
  uint16_t req_len = 0;
  ResetDeviceParameterRequest *req =
      (ResetDeviceParameterRequest *)&req_buff[0];
  req->flag = 1; // Reset some keys
  req->key_num = keys_num;
  for (uint8_t i = 0; i < keys_num; i++) {
    req->key[i] = static_cast<uint16_t>(keys[i]);
  }
  req_len =
      sizeof(ResetDeviceParameterRequest) + (keys_num - 1) * sizeof(uint16_t);

  livox_status result = command_handler().SendCommand(
      handle, kCommandSetGeneral, kCommandIDGeneralResetDeviceParam,
      (uint8_t *)req, req_len,
      MakeCommandCallback<DeviceParameterResponse>(cb, client_data));
  return result;
}

livox_status LidarSetMode(uint8_t handle, LidarMode mode,
                          CommonCommandCallback cb, void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar) {
    return kStatusNotSupported;
  }
  uint8_t req = static_cast<uint8_t>(mode);
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarSetMode, &req, sizeof(req),
      MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status LidarSetExtrinsicParameter(uint8_t handle,
                                        LidarSetExtrinsicParameterRequest *req,
                                        CommonCommandCallback cb,
                                        void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarSetExtrinsicParameter,
      (uint8_t *)req, sizeof(*req),
      MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status LidarGetExtrinsicParameter(uint8_t handle,
                                        LidarGetExtrinsicParameterCallback cb,
                                        void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarGetExtrinsicParameter, NULL, 0,
      MakeCommandCallback<LidarGetExtrinsicParameterResponse>(cb, client_data));
  return result;
}

livox_status LidarRainFogSuppress(uint8_t handle, bool enable,
                                  CommonCommandCallback cb, void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar ||
      !device_manager().IsLidarMid40(handle)) {
    return kStatusNotSupported;
  }
  uint8_t req = static_cast<uint8_t>(enable);
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarControlRainFogSuppression, &req,
      sizeof(req), MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status LidarTurnOnFan(uint8_t handle, CommonCommandCallback cb,
                            void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar ||
      device_manager().IsLidarMid40(handle) ||
      device_manager().IsLidarMid70(handle) ||
      device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }
  return LidarFanControl(handle, true, cb, client_data);
}

livox_status LidarTurnOffFan(uint8_t handle, CommonCommandCallback cb,
                             void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar ||
      device_manager().IsLidarMid40(handle) ||
      device_manager().IsLidarMid70(handle) ||
      device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }
  return LidarFanControl(handle, false, cb, client_data);
}

livox_status LidarGetFanState(uint8_t handle, LidarGetFanStateCallback cb,
                              void *data) {
  if (device_manager().device_mode() != kDeviceModeLidar ||
      device_manager().IsLidarMid40(handle) ||
      device_manager().IsLidarMid70(handle) ||
      device_manager().IsLidarAvia(handle)) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarGetFanState, NULL, 0,
      MakeCommandCallback<LidarGetFanStateResponse>(cb, data));
  return result;
}

livox_status LidarSetPointCloudReturnMode(uint8_t handle,
                                          PointCloudReturnMode mode,
                                          CommonCommandCallback cb,
                                          void *data) {
  if (device_manager().device_mode() != kDeviceModeLidar ||
      device_manager().IsLidarMid40(handle)) {
    return kStatusNotSupported;
  }
  uint8_t req = static_cast<uint8_t>(mode);
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarSetPointCloudReturnMode, &req,
      sizeof(req), MakeCommandCallback<uint8_t>(cb, data));
  return result;
}

livox_status
LidarGetPointCloudReturnMode(uint8_t handle,
                             LidarGetPointCloudReturnModeCallback cb,
                             void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar ||
      device_manager().IsLidarMid40(handle)) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarGetPointCloudReturnMode, NULL, 0,
      MakeCommandCallback<LidarGetPointCloudReturnModeResponse>(cb,
                                                                client_data));
  return result;
}

livox_status LidarSetImuPushFrequency(uint8_t handle, ImuFreq freq,
                                      CommonCommandCallback cb,
                                      void *client_data) {
  if (device_manager().device_mode() != kDeviceModeLidar ||
      device_manager().IsLidarMid40(handle) ||
      device_manager().IsLidarMid70(handle)) {
    return kStatusNotSupported;
  }
  uint8_t req = static_cast<uint8_t>(freq);
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarSetImuPushFrequency, &req,
      sizeof(req), MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status LidarGetImuPushFrequency(uint8_t handle,
                                      LidarGetImuPushFrequencyCallback cb,
                                      void *data) {
  if (device_manager().device_mode() != kDeviceModeLidar ||
      device_manager().IsLidarMid40(handle) ||
      device_manager().IsLidarMid70(handle)) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarGetImuPushFrequency, NULL, 0,
      MakeCommandCallback<LidarGetImuPushFrequencyResponse>(cb, data));
  return result;
}

livox_status HubQueryLidarInformation(HubQueryLidarInformationCallback cb,
                                      void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubQueryLidarInformation,
      NULL, 0,
      MakeCommandCallback<HubQueryLidarInformationResponse>(cb, client_data));
  return result;
}

livox_status HubSetMode(HubSetModeRequest *req, uint16_t length,
                        HubSetModeCallback cb, void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubSetMode, (uint8_t *)req,
      length, MakeCommandCallback<HubSetModeResponse>(cb, client_data));
  return result;
}

livox_status HubControlSlotPower(HubControlSlotPowerRequest *req,
                                 CommonCommandCallback cb, void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubControlSlotPower,
      (uint8_t *)req, sizeof(*req),
      MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status HubSetExtrinsicParameter(HubSetExtrinsicParameterRequest *req,
                                      uint16_t length,
                                      HubSetExtrinsicParameterCallback cb,
                                      void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubSetExtrinsicParameter,
      (uint8_t *)req, length,
      MakeCommandCallback<HubSetExtrinsicParameterResponse>(cb, client_data));
  return result;
}

livox_status HubGetExtrinsicParameter(HubGetExtrinsicParameterCallback cb,
                                      void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubGetExtrinsicParameter,
      NULL, 0,
      MakeCommandCallback<HubGetExtrinsicParameterResponse>(cb, client_data));
  return result;
}

livox_status HubQueryLidarStatus(HubQueryLidarStatusCallback cb,
                                 void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubQueryLidarDeviceStatus,
      NULL, 0,
      MakeCommandCallback<HubQueryLidarStatusResponse>(cb, client_data));
  return result;
}

livox_status HubExtrinsicParameterCalculation(bool enable,
                                              CommonCommandCallback cb,
                                              void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  uint8_t req = static_cast<uint8_t>(enable);
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub,
      kCommandIDHubExtrinsicParameterCalculation, &req, sizeof(req),
      MakeCommandCallback<uint8_t>(cb, client_data));
  return result;
}

livox_status HubRainFogSuppress(HubRainFogSuppressRequest *req, uint16_t length,
                                HubRainFogSuppressCallback cb,
                                void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubRainFogSuppression,
      (uint8_t *)req, length,
      MakeCommandCallback<HubRainFogSuppressResponse>(cb, client_data));
  return result;
}

livox_status HubQuerySlotPowerStatus(HubQuerySlotPowerStatusCallback cb,
                                     void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubQuerySlotPowerStatus,
      NULL, 0,
      MakeCommandCallback<HubQuerySlotPowerStatusResponse>(cb, client_data));
  return result;
}

livox_status HubFanControl(HubFanControlRequest *req, uint16_t length,
                           HubFanControlCallback cb, void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubControlFan,
      (uint8_t *)req, length,
      MakeCommandCallback<HubFanControlResponse>(cb, client_data));
  return result;
}

livox_status HubGetFanState(HubGetFanStateRequest *req, uint16_t length,
                            HubGetFanStateCallback cb, void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubGetFanState,
      (uint8_t *)req, length,
      MakeCommandCallback<HubGetFanStateResponse>(cb, client_data));
  return result;
}

livox_status HubSetPointCloudReturnMode(HubSetPointCloudReturnModeRequest *req,
                                        uint16_t length,
                                        HubSetPointCloudReturnModeCallback cb,
                                        void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubSetPointCloudReturnMode,
      (uint8_t *)req, length,
      MakeCommandCallback<HubSetPointCloudReturnModeResponse>(cb, client_data));
  return result;
}

livox_status HubGetPointCloudReturnMode(HubGetPointCloudReturnModeRequest *req,
                                        uint16_t length,
                                        HubGetPointCloudReturnModeCallback cb,
                                        void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubGetPointCloudReturnMode,
      (uint8_t *)req, length,
      MakeCommandCallback<HubGetPointCloudReturnModeResponse>(cb, client_data));
  return result;
}

livox_status HubSetImuPushFrequency(HubSetImuPushFrequencyRequest *req,
                                    uint16_t length,
                                    HubSetImuPushFrequencyCallback cb,
                                    void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubSetImuPushFrequency,
      (uint8_t *)req, length,
      MakeCommandCallback<HubSetImuPushFrequencyResponse>(cb, client_data));
  return result;
}

livox_status HubGetImuPushFrequency(HubGetImuPushFrequencyRequest *req,
                                    uint16_t length,
                                    HubGetImuPushFrequencyCallback cb,
                                    void *client_data) {
  if (device_manager().device_mode() != kDeviceModeHub) {
    return kStatusNotSupported;
  }
  livox_status result = command_handler().SendCommand(
      kHubDefaultHandle, kCommandSetHub, kCommandIDHubGetImuPushFrequency,
      (uint8_t *)req, length,
      MakeCommandCallback<HubGetImuPushFrequencyResponse>(cb, client_data));
  return result;
}

livox_status LidarSetUtcSyncTime(uint8_t handle,
                                 LidarSetUtcSyncTimeRequest *req,
                                 CommonCommandCallback cb, void *client_data) {
  livox_status result = command_handler().SendCommand(
      handle, kCommandSetLidar, kCommandIDLidarSetSyncTime, (uint8_t *)req,
      sizeof(*req), MakeCommandCallback<uint8_t>(cb, client_data));
  printf("LidarSetUtcSyncTime Failure:%d\n", result);
  return result;
}

livox_status LidarSetRmcSyncTime(uint8_t handle, const char *rmc,
                                 uint16_t rmc_length, CommonCommandCallback cb,
                                 void *client_data) {
  printf("check!");
  if (device_manager().device_mode() != kDeviceModeLidar) {
    return kStatusNotSupported;
  }

  if (device_manager().IsLidarMid40(handle)) {
    uint32_t firm_ver = 0;
    // Mid40 firmware version 03.07.0000
    uint32_t min_firm_ver = (uint32_t)3 << 24 | 7 << 16 | 0 << 8 | 0;
    device_manager().GetLidarFirmwareVersion(handle, firm_ver);
    if (firm_ver < min_firm_ver) {
      return kStatusNotSupported;
    }
  }

  LidarSetUtcSyncTimeRequest utc_time_req;
  if (!ParseRmcTime(rmc, rmc_length, &utc_time_req)) {
    printf("ParseRmcTime kStatusFailure");
    return kStatusFailure;
  }
  printf("ParseRmcTimeok");
  return LidarSetUtcSyncTime(handle, &utc_time_req, cb, client_data);
}
