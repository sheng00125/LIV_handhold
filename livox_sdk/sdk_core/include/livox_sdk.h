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

#ifndef LIVOX_SDK_H_
#define LIVOX_SDK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "livox_def.h"

/**
* Return SDK's version information in a numeric form.
* @param version Pointer to a version structure for returning the version information.
*/
void GetLivoxSdkVersion(LivoxSdkVersion *version);

/**
 * Disable console log output.
 */
void DisableConsoleLogger();

/**
 * Initialize the SDK.
 * @return true if successfully initialized, otherwise false.
 */
bool Init();

/**
 * Start the device scanning routine which runs on a separate thread.
 * @return true if successfully started, otherwise false.
 */
bool Start();

/**
 * Uninitialize the SDK.
 */
void Uninit();

/**
* Save the log file.
*/
void SaveLoggerFile();

/**
 * @c SetBroadcastCallback response callback function.
 * @param info information of the broadcast device, becomes invalid after the function returns.
 */
typedef void (*DeviceBroadcastCallback)(const BroadcastDeviceInfo *info);

/**
 * Set the callback of listening device broadcast message. When broadcast message is received from Livox Hub/LiDAR, cb
 * is called.
 * @param cb callback for device broadcast.
 */
void SetBroadcastCallback(DeviceBroadcastCallback cb);

/**
 * @c SetDeviceStateUpdateCallback response callback function.
 * @param device  information of the connected device.
 * @param type    the update type that indicates connection/disconnection of the device or change of working state.
 */
typedef void (*DeviceStateUpdateCallback)(const DeviceInfo *device, DeviceEvent type);

/**
 * @brief Add a callback for device connection or working state changing event.
 * @note Livox SDK supports two hardware connection modes. 1: Directly connecting to the LiDAR device; 2. Connecting to
 * the LiDAR device(s) via the Livox Hub. In the first mode, connection/disconnection of every LiDAR unit is reported by
 * this callback. In the second mode, only connection/disconnection of the Livox Hub is reported by this callback. If
 * you want to get information of the LiDAR unit(s) connected to hub, see \ref HubQueryLidarInformation.
 * @note 3 conditions can trigger this callback:
 *         1. Connection and disconnection of device.
 *         2. A change of device working state.
 *         3. An error occurs.
 * @param cb callback for device connection/disconnection.
 */
void SetDeviceStateUpdateCallback(DeviceStateUpdateCallback cb);

/**
 * Add a broadcast code to the connecting list and only devices with broadcast code in this list will be connected. The
 * broadcast code is unique for every device.
 * @param broadcast_code device's broadcast code.
 * @param handle device handle. For Livox Hub, the handle is always 31; for LiDAR units connected to the Livox Hub,
 * the corresponding handle is (slot-1)*3+id-1.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status AddHubToConnect(const char *broadcast_code, uint8_t *handle);

/**
 * Add a broadcast code to the connecting list and only devices with broadcast code in this list will be connected. The
 * broadcast code is unique for every device.
 * @param broadcast_code device's broadcast code.
 * @param handle device handle. The handle is the same as the order calling AddLidarToConnect starting from 0.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status AddLidarToConnect(const char *broadcast_code, uint8_t *handle);

/**
 * Get all connected devices' information.
 * @param devices list of connected devices' information.
 * @param size    number of devices connected.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status GetConnectedDevices(DeviceInfo *devices, uint8_t *size);

/**
 * Function type of callback that queries device's information.
 * @param status   kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle   device handle.
 * @param response response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*DeviceInformationCallback)(livox_status status,
                                          uint8_t handle,
                                          DeviceInformationResponse *response,
                                          void *client_data);

/**
 * Command to query device's information.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status QueryDeviceInformation(uint8_t handle, DeviceInformationCallback cb, void *client_data);

/**
 * Callback function for receiving point cloud data.
 * @param handle      device handle.
 * @param data        device's data.
 * @param data_num    number of points in data.
 * @param client_data user data associated with the command.
 */
typedef void (*DataCallback)(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data);

/**
 * Set the callback to receive point cloud data. Only one callback is supported for a specific device. Set the point
 * cloud data callback before beginning sampling.
 * @param handle      device handle.
 * @param cb callback to receive point cloud data.
 * @note 1: Don't do any blocking operations in callback function, it will affects further data's receiving;
 * 2: For different device handle, callback to receive point cloud data will run on its own thread. If you bind
 * different handle to same callback function, please make sure that operations in callback function are thread-safe;
 * 3: callback function's data pointer will be invalid after callback fuction returns. It's recommended to
 * copy all data_num of point cloud every time callback is triggered.
 * @param client_data user data associated with the command.
 */
void SetDataCallback(uint8_t handle, DataCallback cb, void *client_data);

/**
 * Function type of callback with 1 byte of response.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*CommonCommandCallback)(livox_status status, uint8_t handle, uint8_t response, void *client_data);

/**
 * Start hub sampling.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubStartSampling(CommonCommandCallback cb, void *client_data);

/**
 * Stop the Livox Hub's sampling.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubStopSampling(CommonCommandCallback cb, void *client_data);

/**
 * Get the LiDAR unit handle used in the Livox Hub data callback function from slot and id.
 * @param  slot   Livox Hub's slot.
 * @param  id     Livox Hub's id.
 * @return LiDAR unit handle.
 */
uint8_t HubGetLidarHandle(uint8_t slot, uint8_t id);

/**
 * Disconnect divice.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DisconnectDevice(uint8_t handle, CommonCommandCallback cb, void *client_data);

/**
 * Change point cloud coordinate system to cartesian coordinate.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetCartesianCoordinate(uint8_t handle, CommonCommandCallback cb, void *client_data);

/**
 * Change point cloud coordinate system to spherical coordinate.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetSphericalCoordinate(uint8_t handle, CommonCommandCallback cb, void *client_data);

/**
 * Callback of the error status message.
 * kStatusSuccess on successful return, see \ref LivoxStatus for other
 * @param handle      device handle.
 * @param response    response from the device.
 */
typedef void (*ErrorMessageCallback)(livox_status status, uint8_t handle, ErrorMessage *message);

/**
 * Add error status callback for the device.
 * error code.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetErrorMessageCallback(uint8_t handle, ErrorMessageCallback cb);

/**
 * Set device's IP mode.
 * @note \ref SetStaticDynamicIP only supports setting Hub or Mid40/100's IP mode.
 * If you want to set Horizon or Tele's IP mode, see \ref SetStaticIp and \ref SetDynamicIp.
 * @param  handle        device handle.
 * @param  req           request sent to device.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetStaticDynamicIP(uint8_t handle,
                                SetDeviceIPModeRequest *req,
                                CommonCommandCallback cb,
                                void *client_data);
/**
 * Set device's static IP mode.
 * @note Mid40/100 is not supported to set subnet mask and gateway address. 
 * \ref SetStaticDeviceIpModeRequest's setting: net_mask and gw_addr will not take effect on Mid40/100.
 * @param  handle        device handle.
 * @param  req           request sent to device.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetStaticIp(uint8_t handle,
                         SetStaticDeviceIpModeRequest *req,
                         CommonCommandCallback cb,
                         void *client_data);

/**
 * Set device's dynamic IP mode.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status SetDynamicIp(uint8_t handle,
                          CommonCommandCallback cb,
                          void *client_data);
/**
 * Callback function that gets device's IP information.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*GetDeviceIpInformationCallback)(livox_status status,
                                               uint8_t handle,
                                               GetDeviceIpModeResponse *response,
                                               void *client_data);

/**
 * Get device's IP mode.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status GetDeviceIpInformation(uint8_t handle, GetDeviceIpInformationCallback cb, void *client_data);

/**
 * Reboot device.
 * @note \ref RebootDevice is not supported for Mid40/100 firmware version < 03.07.0000.
 * @param  handle        device handle.
 * @param  timeout       reboot device after [timeout] ms.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status RebootDevice(uint8_t handle, uint16_t timeout, CommonCommandCallback cb, void * client_data);

/**
 * @c SetDeviceParameters' response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*SetDeviceParametersCallback)(livox_status status,
                                            uint8_t handle,
                                            DeviceParameterResponse *response,
                                            void *client_data);

/**
 * LiDAR Enable HighSensitivity.
 * @note \ref LidarEnableHighSensitivity only support for Tele/Avia.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarEnableHighSensitivity(uint8_t handle, SetDeviceParametersCallback cb, void *client_data);

/**
 * LiDAR Disable HighSensitivity.
 * @note \ref LidarDisableHighSensitivity only support for Tele/Avia.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarDisableHighSensitivity(uint8_t handle, SetDeviceParametersCallback cb, void *client_data);

/**
 * @c GetDeviceParameters' response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*GetDeviceParametersCallback)(livox_status status,
                                            uint8_t handle,
                                            GetDeviceParameterResponse *response,
                                            void *client_data);

/**
 * LiDAR Get HighSensitivity State.
 * @note \ref LidarGetHighSensitivityState only support for Tele/Avia.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarGetHighSensitivityState(uint8_t handle, GetDeviceParametersCallback cb, void *client_data);

/**
 * LiDAR Set Scan Pattern.
 * @note \ref LidarSetScanPattern only support for Avia.
 * @param  handle        device handle.
 * @param  pattern       scan pattern of LiDAR, see \ref LidarScanPattern for detail.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarSetScanPattern(uint8_t handle, LidarScanPattern pattern, SetDeviceParametersCallback cb, void *client_data);

/**
 * LiDAR Get Scan Pattern.
 * @note \ref LidarGetScanPattern only support for Avia.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarGetScanPattern(uint8_t handle, GetDeviceParametersCallback cb, void *client_data);

/**
 * LiDAR Set Slot Number.
 * @note \ref LidarSetSlotNum only support for Mid70/Avia.
 * @param  handle        device handle.
 * @param  slot          slot number of LiDAR, range from 1 to 9.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarSetSlotNum(uint8_t handle, uint8_t slot, SetDeviceParametersCallback cb, void *client_data);

/**
 * LiDAR Get Slot Number.
 * @note \ref LidarGetSlotNum only support for Mid70/Avia.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarGetSlotNum(uint8_t handle, GetDeviceParametersCallback cb, void *client_data);

/**
 * @c DeviceResetParameters' response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*DeviceResetParametersCallback)(livox_status status,
                                              uint8_t handle,
                                              DeviceParameterResponse *response,
                                              void *client_data);

/**
 * Reset LiDAR/Hub's All Parameters, see \ref DeviceParamKeyName for all parameters.
 * @param  handle        device handle.
 * @param  cb            callback for the command. 
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DeviceResetAllParameters(uint8_t handle, DeviceResetParametersCallback cb, void *client_data);

/**
 * Reset LiDAR/Hub's Parameters, see \ref DeviceParamKeyName for all parameters.
 * @param  handle        device handle.
 * @param  keys          keys to reset, see \ref DeviceParamKeyName for all parameters.
 * @param  num           num of keys to reset.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status DeviceResetParameters(uint8_t handle, DeviceParamKeyName * keys, uint8_t num, DeviceResetParametersCallback cb, void *client_data);


/**
 * @c HubQueryLidarInformation response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubQueryLidarInformationCallback)(livox_status status,
                                                 uint8_t handle,
                                                 HubQueryLidarInformationResponse *response,
                                                 void *client_data);

/**
 * Query the information of LiDARs connected to the hub.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubQueryLidarInformation(HubQueryLidarInformationCallback cb, void *client_data);

/**
 * @c HubSetMode response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubSetModeCallback)(livox_status status, uint8_t handle, HubSetModeResponse *response, void *client_data);

/**
 * Set the mode of LiDAR unit connected to the Livox Hub.
 * @param  req           mode configuration of LiDAR units.
 * @param  length        length of req.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubSetMode(HubSetModeRequest *req, uint16_t length, HubSetModeCallback cb, void *client_data);

/**
 * @c HubQueryLidarStatus response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubQueryLidarStatusCallback)(livox_status status, uint8_t handle, HubQueryLidarStatusResponse *response, void *client_data);

/**
 * Get the state of LiDAR units connected to the Livox Hub.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubQueryLidarStatus(HubQueryLidarStatusCallback cb, void *client_data);

/**
 * Toggle the power supply of designated slots.
 * @param  req           request whether to enable or disable the power of designated slots.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubControlSlotPower(HubControlSlotPowerRequest *req, CommonCommandCallback cb, void *client_data);

/**
 * @c HubSetExtrinsicParameter response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubSetExtrinsicParameterCallback)(livox_status status,
                                                 uint8_t handle,
                                                 HubSetExtrinsicParameterResponse *response,
                                                 void *client_data);

/**
 * Set extrinsic parameters of LiDAR units connected to the Livox Hub.
 * @param  req           the parameters to write.
 * @param  length        the request's length.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubSetExtrinsicParameter(HubSetExtrinsicParameterRequest *req,
                                      uint16_t length,
                                      HubSetExtrinsicParameterCallback cb,
                                      void *client_data);

/**
 * @c HubGetExtrinsicParameter response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubGetExtrinsicParameterCallback)(livox_status status,
                                                 uint8_t handle,
                                                 HubGetExtrinsicParameterResponse *response,
                                                 void *client_data);

/**
 * Get extrinsic parameters of LiDAR units connected to the Livox Hub.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubGetExtrinsicParameter(HubGetExtrinsicParameterCallback cb, void *client_data);

/**
 * Turn on or off the calculation of extrinsic parameters.
 * @param  enable        the request whether enable or disable calculating the extrinsic parameters.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubExtrinsicParameterCalculation(bool enable, CommonCommandCallback cb, void *client_data);

/**
 * @c HubRainFogSuppress response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubRainFogSuppressCallback)(livox_status status,
                                           uint8_t handle,
                                           HubRainFogSuppressResponse *response,
                                           void *client_data);

/**
 * Toggling the rain and fog mode for lidars connected to the hub.
 * @param  req           the request whether open or close the rain and fog mode.
 * @param  length        the request's length.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubRainFogSuppress(HubRainFogSuppressRequest *req,
                                uint16_t length,
                                HubRainFogSuppressCallback cb,
                                void *client_data);

/**
* @c HubQuerySlotPowerStatus response callback function.
* @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
* error code.
* @param handle      device handle.
* @param response    response from the device.
* @param client_data user data associated with the command.
*/
typedef void(*HubQuerySlotPowerStatusCallback)(livox_status status, uint8_t handle, HubQuerySlotPowerStatusResponse *response, void *client_data);

/**
* Get the power supply state of each hub slot.
* @param  cb            callback for the command.
* @param  client_data   user data associated with the command.
* @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
*/
livox_status HubQuerySlotPowerStatus(HubQuerySlotPowerStatusCallback cb, void *client_data);

/**
* @c HubFanControl response callback function.
* @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
* error code.
* @param handle      device handle.
* @param response    response from the device.
* @param client_data user data associated with the command.
*/
typedef void (*HubFanControlCallback)(livox_status status,
                                      uint8_t handle,
                                      HubFanControlResponse *response,
                                      void *client_data);

/**
 * Turn on or off the fan of LiDAR unit connected to the Livox Hub.
 * @param  req           Fan control of LiDAR units.
 * @param  length        length of req.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubFanControl(HubFanControlRequest *req, uint16_t length, HubFanControlCallback cb, void *client_data);

/**
* @c HubGetFanControl response callback function.
* @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
* error code.
* @param handle      device handle.
* @param response    response from the device.
* @param client_data user data associated with the command.
*/
typedef void (*HubGetFanStateCallback)(livox_status status,
                                       uint8_t handle,
                                       HubGetFanStateResponse *response,
                                       void *client_data);

/**
 * Get fan state of LiDAR unit connected to the Livox Hub.
 * @param  req           Get fan state of LiDAR units.
 * @param  length        length of req.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubGetFanState(HubGetFanStateRequest *req,
                            uint16_t length,
                            HubGetFanStateCallback cb,
                            void *client_data);

/**
 * @c HubSetPointCloudReturnMode response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubSetPointCloudReturnModeCallback)(livox_status status,
                                                   uint8_t handle,
                                                   HubSetPointCloudReturnModeResponse *response,
                                                   void *client_data);

/**
 * Set point cloud return mode of LiDAR units connected to the Livox Hub.
 * @param  req           set point cloud return mode of LiDAR units.
 * @param  length        the request's length.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubSetPointCloudReturnMode(HubSetPointCloudReturnModeRequest *req,
                                        uint16_t length,
                                        HubSetPointCloudReturnModeCallback cb,
                                        void *client_data);

/**
 * @c HubGetPointCloudReturnMode response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubGetPointCloudReturnModeCallback)(livox_status status,
                                                   uint8_t handle,
                                                   HubGetPointCloudReturnModeResponse *response,
                                                   void *client_data);

/**
 * Get point cloud return mode of LiDAR unit connected to the Livox Hub.
 * @param  req           Get point cloud return mode of LiDAR units.
 * @param  length        length of req.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubGetPointCloudReturnMode(HubGetPointCloudReturnModeRequest *req,
                                        uint16_t length,
                                        HubGetPointCloudReturnModeCallback cb,
                                        void *client_data);

/**
 * @c HubSetImuPushFrequency response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubSetImuPushFrequencyCallback)(livox_status status,
                                               uint8_t handle,
                                               HubSetImuPushFrequencyResponse *response,
                                               void *client_data);

/**
 * Set IMU push frequency of LiDAR units connected to the Livox Hub.
 * @param  req           set IMU push frequency of LiDAR units.
 * @param  length        the request's length.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubSetImuPushFrequency(HubSetImuPushFrequencyRequest *req,
                                    uint16_t length,
                                    HubSetImuPushFrequencyCallback cb,
                                    void *client_data);

/**
 * @c HubGetImuPushFrequency response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*HubGetImuPushFrequencyCallback)(livox_status status,
                                               uint8_t handle,
                                               HubGetImuPushFrequencyResponse *response,
                                               void *client_data);

/**
 * Get IMU push frequency of LiDAR units connected to the Livox Hub.
 * @param  req           get IMU push frequency of LiDAR units.
 * @param  length        the request's length.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status HubGetImuPushFrequency(HubGetImuPushFrequencyRequest *req,
                                    uint16_t length,
                                    HubGetImuPushFrequencyCallback cb,
                                    void *client_data);

/**
 * Start LiDAR sampling.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarStartSampling(uint8_t handle, CommonCommandCallback cb, void *client_data);

/**
 * Stop LiDAR sampling.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarStopSampling(uint8_t handle, CommonCommandCallback cb, void *client_data);

/**
 * Set LiDAR mode.
 * @note Successful callback function status only means LiDAR successfully starting the changing process of mode.
 * You need to observe the actually change of mode in DeviceStateUpdateCallback function.
 * @param  handle        device handle.
 * @param  mode          the mode to change.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarSetMode(uint8_t handle, LidarMode mode, CommonCommandCallback cb, void *client_data);

/**
 * Set LiDAR extrinsic parameters.
 * @param  handle        device handle.
 * @param  req         the parameters to write.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarSetExtrinsicParameter(uint8_t handle,
                                        LidarSetExtrinsicParameterRequest *req,
                                        CommonCommandCallback cb,
                                        void *client_data);

/**
 * @c LidarGetExtrinsicParameter response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*LidarGetExtrinsicParameterCallback)(livox_status status,
                                                   uint8_t handle,
                                                   LidarGetExtrinsicParameterResponse *response,
                                                   void *client_data);

/**
 * Get LiDAR extrinsic parameters.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarGetExtrinsicParameter(uint8_t handle, LidarGetExtrinsicParameterCallback cb, void *client_data);

/**
 * Enable and disable the rain/fog suppression.
 * @note \ref LidarRainFogSuppress only support for Mid40/100.
 * @param  handle        device handle.
 * @param  enable        enable and disable the rain/fog suppression.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarRainFogSuppress(uint8_t handle, bool enable, CommonCommandCallback cb, void *client_data);

/**
 * Turn off the fan.
 * @note \ref LidarTurnOffFan is not supported for Mid40/100.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarTurnOffFan(uint8_t handle, CommonCommandCallback cb, void *client_data);

/**
 * Turn on the fan.
 * @note \ref LidarTurnOnFan is not supported for Mid40/100.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarTurnOnFan(uint8_t handle, CommonCommandCallback cb, void *client_data);

/**
 * @c LidarGetFanState response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*LidarGetFanStateCallback)(livox_status status,
                                        uint8_t handle,
                                        LidarGetFanStateResponse *response,
                                        void *client_data);

/**
 * Get state of the fan.
 * @note \ref LidarGetFanState is not supported for Mid40/100.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarGetFanState(uint8_t handle, LidarGetFanStateCallback cb, void * client_data) ;

/**
 * Set point cloud return mode.
 * @note \ref LidarSetPointCloudReturnMode is not supported for Mid40/100.
 * @param  handle        device handle.
 * @param  mode          point cloud return mode.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarSetPointCloudReturnMode(uint8_t handle, PointCloudReturnMode mode,  CommonCommandCallback cb, void * client_data);

/**
 * @c LidaGetPointCloudReturnMode response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*LidarGetPointCloudReturnModeCallback)(livox_status status,
                                                     uint8_t handle,
                                                     LidarGetPointCloudReturnModeResponse *response,
                                                     void *client_data);

/**
 * Get point cloud return mode.
 * @note \ref LidarGetPointCloudReturnMode is not supported for Mid40/100.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarGetPointCloudReturnMode(uint8_t handle, LidarGetPointCloudReturnModeCallback cb, void * client_data);

/**
 * Set IMU push frequency.
 * @note \ref LidarSetImuPushFrequency is not supported for Mid40/100.
 * @param  handle        device handle.
 * @param  freq          IMU push frequency.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarSetImuPushFrequency(uint8_t handle, ImuFreq freq, CommonCommandCallback cb, void * client_data);

/**
 * @c LidaGetImuPushFrequency response callback function.
 * @param status      kStatusSuccess on successful return, kStatusTimeout on timeout, see \ref LivoxStatus for other
 * error code.
 * @param handle      device handle.
 * @param response    response from the device.
 * @param client_data user data associated with the command.
 */
typedef void (*LidarGetImuPushFrequencyCallback)(livox_status status,
                                                 uint8_t handle,
                                                 LidarGetImuPushFrequencyResponse *response,
                                                 void *client_data);

/**
 * Get IMU push frequency.
 * @note \ref LidarGetImuPushFrequency is not supported for Mid40/100.
 * @param  handle        device handle.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarGetImuPushFrequency(uint8_t handle, LidarGetImuPushFrequencyCallback cb, void * client_data);

/**
 * Set GPRMC formate synchronization time.
 * @note \ref LidarSetRmcSyncTime is not supported for Mid40/100 firmware version < 03.07.0000.
 * @param  handle        device handle.
 * @param  rmc           GPRMC/GNRMC format data.
 * @param  rmc_length    lenth of gprmc.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarSetRmcSyncTime(uint8_t handle,
                                 const char* rmc,
                                 uint16_t rmc_length,
                                 CommonCommandCallback cb,
                                 void *client_data);

/**
 * Set UTC formate synchronization time.
 * @note \ref LidarSetUtcSyncTime is not supported for Mid40/100 firmware version < 03.07.0000.
 * @param  handle        device handle.
 * @param  req           UTC format data.
 * @param  cb            callback for the command.
 * @param  client_data   user data associated with the command.
 * @return kStatusSuccess on successful return, see \ref LivoxStatus for other error code.
 */
livox_status LidarSetUtcSyncTime(uint8_t handle,
                                 LidarSetUtcSyncTimeRequest* req,
                                 CommonCommandCallback cb,
                                 void *client_data);

#ifdef __cplusplus
}
#endif

#endif  // LIVOX_SDK_H_
