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

#ifndef LIVOX_DEF_H_
#define LIVOX_DEF_H_

#include <stdint.h>

#define kMaxLidarCount 32

/** Device type. */
typedef enum {
  kDeviceTypeHub = 0,          /**< Livox Hub. */
  kDeviceTypeLidarMid40 = 1,   /**< Mid-40. */
  kDeviceTypeLidarTele = 2,    /**< Tele. */
  kDeviceTypeLidarHorizon = 3,  /**< Horizon. */
  kDeviceTypeLidarMid70 = 6,    /**< Livox Mid-70. */
  kDeviceTypeLidarAvia = 7     /**< Avia. */
} DeviceType;

/** Lidar state. */
typedef enum {
  kLidarStateInit = 0,        /**< Initialization state. */
  kLidarStateNormal = 1,      /**< Normal work state. */
  kLidarStatePowerSaving = 2, /**< Power-saving state. */
  kLidarStateStandBy = 3,     /**< Standby state. */
  kLidarStateError = 4,       /**< Error state. */
  kLidarStateUnknown = 5      /**< Unknown state. */
} LidarState;

/** Lidar mode. */
typedef enum {
  kLidarModeNormal = 1,      /**< Normal mode. */
  kLidarModePowerSaving = 2, /**< Power-saving mode. */
  kLidarModeStandby = 3      /**< Standby mode. */
} LidarMode;

/** Lidar feature. */
typedef enum {
  kLidarFeatureNone = 0,   /**< No feature. */
  kLidarFeatureRainFog = 1 /**< Rain and fog feature. */
} LidarFeature;

/** Lidar IP mode. */
typedef enum {
  kLidarDynamicIpMode = 0,   /**< Dynamic IP. */
  kLidarStaticIpMode = 1     /**< Static IP. */
} LidarIpMode;

/** Lidar Scan Pattern. */
typedef enum {
  kNoneRepetitiveScanPattern = 0,  /**< None Repetitive Scan Pattern. */
  kRepetitiveScanPattern = 1,      /**< Repetitive Scan Pattern. */
} LidarScanPattern;

/** Function return value definition. */
typedef enum {
  kStatusSendFailed = -9,           /**< Command send failed. */
  kStatusHandlerImplNotExist = -8,  /**< Handler implementation not exist. */
  kStatusInvalidHandle = -7,        /**< Device handle invalid. */
  kStatusChannelNotExist = -6,      /**< Command channel not exist. */
  kStatusNotEnoughMemory = -5,      /**< No enough memory. */
  kStatusTimeout = -4,              /**< Operation timeouts. */
  kStatusNotSupported = -3,         /**< Operation is not supported on this device. */
  kStatusNotConnected = -2,         /**< Requested device is not connected. */
  kStatusFailure = -1,              /**< Failure. */
  kStatusSuccess = 0                /**< Success. */
} LivoxStatus;

/** Fuction return value defination, refer to \ref LivoxStatus. */
typedef int32_t livox_status;

/** Device update type, indicating the change of device connection or working state. */
typedef enum {
  kEventConnect = 0,              /**< Device is connected. */
  kEventDisconnect = 1,           /**< Device is removed. */
  kEventStateChange = 2,          /**< Device working state changes or an error occurs. */
  kEventHubConnectionChange = 3   /**< Hub is connected or LiDAR unit(s) is/are removed. */
} DeviceEvent;

/** Timestamp sync mode define. */
typedef enum {
  kTimestampTypeNoSync = 0, /**< No sync signal mode. */
  kTimestampTypePtp = 1,    /**< 1588v2.0 PTP sync mode. */
  kTimestampTypeRsvd = 2,   /**< Reserved use. */
  kTimestampTypePpsGps = 3, /**< pps+gps sync mode. */
  kTimestampTypePps = 4,    /**< pps only sync mode. */
  kTimestampTypeUnknown = 5 /**< Unknown mode. */
} TimestampType;

/** Point data type. */
typedef enum {
  kCartesian,               /**< Cartesian coordinate point cloud. */
  kSpherical,               /**< Spherical coordinate point cloud. */
  kExtendCartesian,         /**< Extend cartesian coordinate point cloud. */
  kExtendSpherical,         /**< Extend spherical coordinate point cloud. */
  kDualExtendCartesian,     /**< Dual extend cartesian coordinate  point cloud. */
  kDualExtendSpherical,     /**< Dual extend spherical coordinate point cloud. */
  kImu,                     /**< IMU data. */
  kTripleExtendCartesian,   /**< Triple extend cartesian coordinate  point cloud. */
  kTripleExtendSpherical,   /**< Triple extend spherical coordinate  point cloud. */
  kMaxPointDataType         /**< Max Point Data Type. */
} PointDataType;

/** Point cloud return mode. */
typedef enum {
  kFirstReturn,             /**< First single return mode . */
  kStrongestReturn,         /**< Strongest single return mode. */
  kDualReturn,              /**< Dual return mode. */
  kTripleReturn,            /**< Triple return mode. */
} PointCloudReturnMode;

/** IMU push frequency. */
typedef enum {
  kImuFreq0Hz,              /**< IMU push closed. */
  kImuFreq200Hz,            /**< IMU push frequency 200Hz. */
} ImuFreq;

#pragma pack(1)

#define LIVOX_SDK_MAJOR_VERSION       2
#define LIVOX_SDK_MINOR_VERSION       3
#define LIVOX_SDK_PATCH_VERSION       0

#define kBroadcastCodeSize 16

/** The numeric version information struct.  */
typedef struct {
  int major;      /**< major number */
  int minor;      /**< minor number */
  int patch;      /**< patch number */
} LivoxSdkVersion;

/** Cartesian coordinate format. */
typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
} LivoxRawPoint;

/** Spherical coordinate format. */
typedef struct {
  uint32_t depth;       /**< Depth, Unit: mm */
  uint16_t theta;       /**< Zenith angle[0, 18000], Unit: 0.01 degree */
  uint16_t phi;         /**< Azimuth[0, 36000], Unit: 0.01 degree */
  uint8_t reflectivity; /**< Reflectivity */
} LivoxSpherPoint;

/** Standard point cloud format */
typedef struct {
  float x;              /**< X axis, Unit:m */
  float y;              /**< Y axis, Unit:m */
  float z;              /**< Z axis, Unit:m */
  uint8_t reflectivity; /**< Reflectivity */
} LivoxPoint;

/** Extend cartesian coordinate format. */
typedef struct {
  int32_t x;            /**< X axis, Unit:mm */
  int32_t y;            /**< Y axis, Unit:mm */
  int32_t z;            /**< Z axis, Unit:mm */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxExtendRawPoint;

/** Extend spherical coordinate format. */
typedef struct {
  uint32_t depth;       /**< Depth, Unit: mm */
  uint16_t theta;       /**< Zenith angle[0, 18000], Unit: 0.01 degree */
  uint16_t phi;         /**< Azimuth[0, 36000], Unit: 0.01 degree */
  uint8_t reflectivity; /**< Reflectivity */
  uint8_t tag;          /**< Tag */
} LivoxExtendSpherPoint;

/** Dual extend cartesian coordinate format. */
typedef struct {
  int32_t x1;            /**< X axis, Unit:mm */
  int32_t y1;            /**< Y axis, Unit:mm */
  int32_t z1;            /**< Z axis, Unit:mm */
  uint8_t reflectivity1; /**< Reflectivity */
  uint8_t tag1;          /**< Tag */
  int32_t x2;            /**< X axis, Unit:mm */
  int32_t y2;            /**< Y axis, Unit:mm */
  int32_t z2;            /**< Z axis, Unit:mm */
  uint8_t reflectivity2; /**< Reflectivity */
  uint8_t tag2;          /**< Tag */
} LivoxDualExtendRawPoint;

/** Dual extend spherical coordinate format. */
typedef struct {
  uint16_t theta;        /**< Zenith angle[0, 18000], Unit: 0.01 degree */
  uint16_t phi;          /**< Azimuth[0, 36000], Unit: 0.01 degree */
  uint32_t depth1;       /**< Depth, Unit: mm */
  uint8_t reflectivity1; /**< Reflectivity */
  uint8_t tag1;          /**< Tag */
  uint32_t depth2;       /**< Depth, Unit: mm */
  uint8_t reflectivity2; /**< Reflectivity */
  uint8_t tag2;          /**< Tag */
} LivoxDualExtendSpherPoint;

/** Triple extend cartesian coordinate format. */
typedef struct {
  int32_t x1;            /**< X axis, Unit:mm */
  int32_t y1;            /**< Y axis, Unit:mm */
  int32_t z1;            /**< Z axis, Unit:mm */
  uint8_t reflectivity1; /**< Reflectivity */
  uint8_t tag1;          /**< Tag */
  int32_t x2;            /**< X axis, Unit:mm */
  int32_t y2;            /**< Y axis, Unit:mm */
  int32_t z2;            /**< Z axis, Unit:mm */
  uint8_t reflectivity2; /**< Reflectivity */
  uint8_t tag2;          /**< Tag */
  int32_t x3;            /**< X axis, Unit:mm */
  int32_t y3;            /**< Y axis, Unit:mm */
  int32_t z3;            /**< Z axis, Unit:mm */
  uint8_t reflectivity3; /**< Reflectivity */
  uint8_t tag3;          /**< Tag */
} LivoxTripleExtendRawPoint;

/** Triple extend spherical coordinate format. */
typedef struct {
  uint16_t theta;        /**< Zenith angle[0, 18000], Unit: 0.01 degree */
  uint16_t phi;          /**< Azimuth[0, 36000], Unit: 0.01 degree */
  uint32_t depth1;       /**< Depth, Unit: mm */
  uint8_t reflectivity1; /**< Reflectivity */
  uint8_t tag1;          /**< Tag */
  uint32_t depth2;       /**< Depth, Unit: mm */
  uint8_t reflectivity2; /**< Reflectivity */
  uint8_t tag2;          /**< Tag */
  uint32_t depth3;       /**< Depth, Unit: mm */
  uint8_t reflectivity3; /**< Reflectivity */
  uint8_t tag3;          /**< Tag */
} LivoxTripleExtendSpherPoint;

/** IMU data format. */
typedef struct {
  float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  float acc_x;         /**< Accelerometer X axis, Unit:g */
  float acc_y;         /**< Accelerometer Y axis, Unit:g */
  float acc_z;         /**< Accelerometer Z axis, Unit:g */
} LivoxImuPoint;

/** LiDAR error code. */
typedef struct {
  uint32_t temp_status : 2;      /**< 0: Temperature in Normal State. 1: High or Low. 2: Extremely High or Extremely Low. */
  uint32_t volt_status : 2;      /**< 0: Voltage in Normal State. 1: High. 2: Extremely High. */
  uint32_t motor_status : 2;     /**< 0: Motor in Normal State. 1: Motor in Warning State. 2:Motor in Error State, Unable to Work. */
  uint32_t dirty_warn : 2;       /**< 0: Not Dirty or Blocked. 1: Dirty or Blocked. */
  uint32_t firmware_err : 1;     /**< 0: Firmware is OK. 1: Firmware is Abnormal, Need to be Upgraded. */
  uint32_t pps_status : 1;       /**< 0: No PPS Signal. 1: PPS Signal is OK. */
  uint32_t device_status : 1;    /**< 0: Normal. 1: Warning for Approaching the End of Service Life. */
  uint32_t fan_status : 1;       /**< 0: Fan in Normal State. 1: Fan in Warning State. */
  uint32_t self_heating : 1;     /**< 0: Normal. 1: Low Temperature Self Heating On. */
  uint32_t ptp_status : 1;       /**< 0: No 1588 Signal. 1: 1588 Signal is OK. */
  /** 0: System dose not start time synchronization.
   * 1: Using PTP 1588 synchronization.
   * 2: Using GPS synchronization.
   * 3: Using PPS synchronization.
   * 4: System time synchronization is abnormal.(The highest priority synchronization signal is abnormal)
  */
  uint32_t time_sync_status : 3;
  uint32_t rsvd : 13;            /**< Reserved. */
  uint32_t system_status : 2;    /**< 0: Normal. 1: Warning. 2: Error. */
} LidarErrorCode;

/** Hub error code. */
typedef struct {
  /** 0: No synchronization signal.
   * 1: 1588 synchronization.
   * 2: GPS synchronization.
   * 3: System time synchronization is abnormal.(The highest priority synchronization signal is abnormal)
   */
  uint32_t sync_status : 2;
  uint32_t temp_status : 2;        /**< 0: Temperature in Normal State. 1: High or Low. 2: Extremely High or Extremely Low. */
  uint32_t lidar_status : 1;       /**< 0: LiDAR State is Normal. 1: LiDAR State is Abnormal. */
  uint32_t lidar_link_status : 1;  /**< 0: LiDAR Connection is Normal. 1: LiDAR Connection is Abnormal. */
  uint32_t firmware_err : 1;       /**< 0: LiDAR Firmware is OK. 1: LiDAR Firmware is Abnormal, Need to be Upgraded. */
  uint32_t rsvd : 23;              /**< Reserved. */
  uint32_t system_status : 2;      /**< 0: Normal. 1: Warning. 2: Error. */
} HubErrorCode;

/**
 * Device error message.
 */
typedef union {
  uint32_t error_code;                /**< Error code. */
  LidarErrorCode lidar_error_code;    /**< Lidar error code. */
  HubErrorCode hub_error_code;        /**< Hub error code. */
} ErrorMessage;

/** Point cloud packet. */
typedef struct {
  uint8_t version;              /**< Packet protocol version. */
  uint8_t slot;                 /**< Slot number used for connecting LiDAR. */
  uint8_t id;                   /**< LiDAR id. */
  uint8_t rsvd;                 /**< Reserved. */
  uint32_t err_code;      /**< Device error status indicator information. */
  uint8_t timestamp_type;       /**< Timestamp type. */
  /** Point cloud coordinate format, refer to \ref PointDataType . */
  uint8_t data_type;
  uint8_t timestamp[8];         /**< Nanosecond or UTC format timestamp. */
  uint8_t data[1];              /**< Point cloud data. */
} LivoxEthPacket;

/** Information of LiDAR work state. */
typedef union {
  uint32_t progress;    /**< LiDAR work state switching progress. */
  ErrorMessage status_code; /**< LiDAR work state status code. */
} StatusUnion;

/** Information of the connected LiDAR or hub. */
typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code, null-terminated string, 15 characters at most. */
  uint8_t handle;                          /**< Device handle. */
  uint8_t slot;                            /**< Slot number used for connecting LiDAR. */
  uint8_t id;                              /**< LiDAR id. */
  uint8_t type;                            /**< Device type, refer to \ref DeviceType. */
  uint16_t data_port;                      /**< Point cloud data UDP port. */
  uint16_t cmd_port;                       /**< Control command UDP port. */
  uint16_t sensor_port;                    /**< IMU data UDP port. */
  char ip[16];                             /**< IP address. */
  LidarState state;                        /**< LiDAR state. */
  LidarFeature feature;                    /**< LiDAR feature. */
  StatusUnion status;                      /**< LiDAR work state status. */
  uint8_t firmware_version[4];             /**< Firmware version. */
} DeviceInfo;

/** The information of broadcast device. */
typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code, null-terminated string, 15 characters at most. */
  uint8_t dev_type;                        /**< Device type, refer to \ref DeviceType. */
  uint16_t reserved;                       /**< Reserved. */
  char ip[16];                             /**< Device ip. */
} BroadcastDeviceInfo;

/** The information of LiDAR units that are connected to the Livox Hub. */
typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code, null-terminated string, 15 characters at most. */
  uint8_t dev_type;                        /**< Device type, refer to \ref DeviceType. */
  uint8_t version[4];                      /**< Firmware version. */
  uint8_t slot;                            /**< Slot number used for connecting LiDAR units. */
  uint8_t id;                              /**< Device id. */
} ConnectedLidarInfo;

/** LiDAR mode configuration information. */
typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code, null-terminated string, 15 characters at most. */
  uint8_t state;                           /**< LiDAR state, refer to \ref LidarMode. */
} LidarModeRequestItem;

typedef struct {
  uint8_t ret_code;                        /**< Return code. */
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
} ReturnCode;

typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
} DeviceBroadcastCode;

typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
  uint8_t feature;                         /**< Close or open the rain and fog feature. */
} RainFogSuppressRequestItem;

typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
  uint8_t state;                           /**< Fan state: 1 for turn on fan, 0 for turn off fan. */
} FanControlRequestItem;

typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
} GetFanStateRequestItem;

typedef struct {
  uint8_t ret_code;                        /**< Return code. */
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
  uint8_t state;                           /**< Fan state: 1 for fan is on, 0 for fan is off. */
} GetFanStateResponseItem;

typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
  uint8_t mode;                            /**< Point cloud return mode, refer to \ref PointCloudReturnMode. */
} SetPointCloudReturnModeRequestItem;

typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
} GetPointCloudReturnModeRequestItem;

typedef struct {
  uint8_t ret_code;                        /**< Return code. */
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
  uint8_t mode;                            /**< Point cloud return mode, refer to \ref PointCloudReturnMode. */
} GetPointCloudReturnModeResponseItem;

typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
  uint8_t freq;                            /**< IMU push frequency, refer to \ref ImuFreq. */
} SetImuPushFrequencyRequestItem;


typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
} GetImuPushFrequencyRequestItem;

typedef struct {
  uint8_t ret_code;                        /**< Return code. */
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
  uint8_t freq;                            /**< IMU push frequency, refer to \ref ImuFreq. */
} GetImuPushFrequencyResponseItem;

/** LiDAR configuration information. */
typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Device broadcast code. */
  float roll;                              /**< Roll angle, unit: degree. */
  float pitch;                             /**< Pitch angle, unit: degree. */
  float yaw;                               /**< Yaw angle, unit: degree. */
  int32_t x;                               /**< X translation, unit: mm. */
  int32_t y;                               /**< Y translation, unit: mm. */
  int32_t z;                               /**< Z translation, unit: mm. */
} ExtrinsicParameterRequestItem;

/** LiDAR extrinsic parameters. */
typedef struct {
  uint8_t ret_code;                        /**< Return code. */
  char broadcast_code[kBroadcastCodeSize]; /**< Broadcast code. */
  float roll;                              /**< Roll angle, unit: degree. */
  float pitch;                             /**< Pitch angle, unit: degree. */
  float yaw;                               /**< Yaw angle, unit: degree. */
  int32_t x;                               /**< X translation, unit: mm. */
  int32_t y;                               /**< Y translation, unit: mm. */
  int32_t z;                               /**< Z translation, unit: mm. */
} ExtrinsicParameterResponseItem;

typedef struct {
  char broadcast_code[kBroadcastCodeSize]; /**< Broadcast code. */
  uint8_t state;                           /**< LiDAR state. */
  uint8_t feature;                         /**< LiDAR feature. */
  StatusUnion error_union;                 /**< LiDAR work state. */
} LidarStateItem;

/**
 * The request body for the command of handshake.
 */
typedef struct {
  uint32_t ip_addr;     /**< IP address of the device. */
  uint16_t data_port;   /**< UDP port of the data connection. */
  uint16_t cmd_port;    /**< UDP port of the command connection. */
  uint16_t sensor_port; /**< UDP port of the sensor connection. */
} HandshakeRequest;

/**
 * The response body of querying device information.
 */
typedef struct {
  uint8_t ret_code;            /**< Return code. */
  uint8_t firmware_version[4]; /**< Firmware version. */
} DeviceInformationResponse;

/**
 * The request body of the command for setting device's IP mode.
 */
typedef struct {
  uint8_t ip_mode;  /**< IP address mode: 0 for dynamic IP address, 1 for static IP address. */
  uint32_t ip_addr; /**< IP address. */
} SetDeviceIPModeRequest;

/**
 * The response body of getting device's IP mode.
 */
typedef struct {
  uint8_t ret_code;  /**< Return code. */
  uint8_t ip_mode;   /**< IP address mode: 0 for dynamic IP address, 1 for static IP address. */
  uint32_t ip_addr;  /**< IP address. */
  uint32_t net_mask; /**< Subnet mask. */
  uint32_t gw_addr;  /**< Gateway address. */
} GetDeviceIpModeResponse;

/**
 * The request body of the command for setting static device's IP mode.
 */
typedef struct {
  uint32_t ip_addr;  /**< IP address. */
  uint32_t net_mask; /**< Subnet mask. */
  uint32_t gw_addr;  /**< Gateway address. */
} SetStaticDeviceIpModeRequest;

/**
 * The body of heartbeat response.
 */
typedef struct {
  uint8_t ret_code;        /**< Return code. */
  uint8_t state;           /**< Working state. */
  uint8_t feature;         /**< LiDAR feature. */
  StatusUnion error_union; /**< LiDAR work state. */
} HeartbeatResponse;

/**
 * The error code of Getting/Setting Device's Parameters.
 */
typedef enum {
  kKeyNoError = 0,                  /**< No Error. */
  kKeyNotSupported = 1,             /**< The key is not supported. */
  kKeyExecFailed = 2,               /**< Execution failed. */
  kKeyNotSupportedWritingState = 3, /**< The key cannot be written. */
  kKeyValueError = 4,               /**< Wrong value. */
  kKeyValueLengthError = 5,         /**< Wrong value length. */
  kKeyNoEnoughMemory = 6,           /**< Reading parameter length limit. */
  kKeyLengthError = 7,              /**< The number of parameters does not match. */
} KeyErrorCode;

/**
 * The response body of setting device's parameter.
 */
typedef struct {
  uint8_t ret_code;              /**< Return code. */
  uint16_t error_param_key;      /**< Error Key. */
  uint8_t error_code;            /**< Error code, refer to \ref KeyErrorCode. */
} DeviceParameterResponse;

/**
 * Keys of device's parameters.
 */
typedef enum {
  kKeyDefault = 0,              /**< Default key name. */
  kKeyHighSensetivity = 1,      /**< Key to get/set LiDAR' Sensetivity. */
  kKeyScanPattern =  2,         /**< Key to get/set LiDAR' ScanPattern. */
  kKeySlotNum = 3,              /**< Key to get/set LiDAR' Slot number. */
} DeviceParamKeyName;

/**
 * Key and value of device's parameters.
 */
typedef struct {
  uint16_t key;                /*< Key, refer to \ref DeviceParamKeyName. */
  uint16_t length;             /*< Length of value */
  uint8_t value[1];            /*< Value */
} KeyValueParam;

/**
 * The response body of getting device's parameter.
 */
typedef struct {
  DeviceParameterResponse rsp;     /*< Return code. */
  KeyValueParam kv;                /*< Key and value of device's parameters. */
} GetDeviceParameterResponse;

/**
 * The request body for the command of getting device's parameters.
 */
typedef struct {
  uint8_t param_num;           /*< Number of key. */
  uint16_t key[1];             /*< Key, refer to \ref DeviceParamKeyName. */
} GetDeviceParameterRequest;

/**
 * The request body for the command of resetting device's parameters.
 */
typedef struct {
  uint8_t flag;                /*< 0: for resetting all keys, 1: for resetting part of keys. */
  uint8_t key_num;             /*< number of keys to reset. */
  uint16_t key[1];             /*< Keys to reset, refer to \ref DeviceParamKeyName. */
} ResetDeviceParameterRequest;

/**
 * The request body for the command of setting Livox LiDAR's parameters.
 */
typedef struct {
  float roll;  /**< Roll angle, unit: degree. */
  float pitch; /**< Pitch angle, unit: degree. */
  float yaw;   /**< Yaw angle, unit: degree. */
  int32_t x;   /**< X translation, unit: mm. */
  int32_t y;   /**< Y translation, unit: mm. */
  int32_t z;   /**< Z translation, unit: mm. */
} LidarSetExtrinsicParameterRequest;

/**
 * The response body of getting Livox LiDAR's parameters.
 */
typedef struct {
  uint8_t ret_code;
  float roll;  /**< Roll angle, unit: degree. */
  float pitch; /**< Pitch angle, unit: degree. */
  float yaw;   /**< Yaw angle, unit: degree. */
  int32_t x;   /**< X translation, unit: mm. */
  int32_t y;   /**< Y translation, unit: mm. */
  int32_t z;   /**< Z translation, unit: mm. */
} LidarGetExtrinsicParameterResponse;

/**
 * The response body of getting the Livox LiDAR's fan state.
 */
typedef struct {
  uint8_t ret_code;     /**< Return code. */
  uint8_t state;        /**< Fan state: 1 for fan is on, 0 for fan is off. */
} LidarGetFanStateResponse;

/**
 * The response body of getting the Livox LiDAR's point cloud return mode.
 */
typedef struct {
  uint8_t ret_code;    /**< Return code. */
  uint8_t mode;        /**< Point cloud return mode, refer to \ref PointCloudReturnMode. */
} LidarGetPointCloudReturnModeResponse;


/**
 * The response body of getting the Livox LiDAR's IMU push frequency.
 */
typedef struct {
  uint8_t ret_code;            /**< Return code. */
  uint8_t freq;                /**< IMU push frequency, refer to \ref ImuFreq. */
} LidarGetImuPushFrequencyResponse;

/**
 * The response body of setting the Livox LiDAR's Sync time.
 */
typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint32_t microsecond;
} LidarSetUtcSyncTimeRequest;

/**
 * The response body of querying the information of LiDAR units connected to the Livox Hub.
 */
typedef struct {
  uint8_t ret_code;                       /**< Return code. */
  uint8_t count;                          /**< Count of device_info_list. */
  ConnectedLidarInfo device_info_list[1]; /**< Connected lidars information list. */
} HubQueryLidarInformationResponse;

/**
 * The request body of setting Livox Hub's working mode.
 */
typedef struct {
  uint8_t count;                       /**< Count of config_list. */
  LidarModeRequestItem config_list[1]; /**< LiDAR mode configuration list. */
} HubSetModeRequest;

/**
 * The response of setting Livox Hub's working mode.
 */
typedef struct {
  uint8_t ret_code;             /**< Return code. */
  uint8_t count;                /**< Count of ret_state_list. */
  ReturnCode ret_state_list[1]; /**< Return status list. */
} HubSetModeResponse;

/**
 * The request body of toggling the power supply of the slot.
 */
typedef struct {
  uint8_t slot;  /**< Slot of the hub. */
  uint8_t state; /**< Status of toggling the power supply. */
} HubControlSlotPowerRequest;

/**
 * The request body of setting the Livox Hub's parameters.
 */
typedef struct {
  uint8_t count;                 /**< Count of cfg_param_list. */
  ExtrinsicParameterRequestItem parameter_list[1]; /**< Extrinsic parameter configuration list. */
} HubSetExtrinsicParameterRequest;

/**
 * The response body of setting the Livox Hub's parameters.
 */
typedef struct {
  uint8_t ret_code;            /**< Return code. */
  uint8_t count;               /**< Count of ret_code_list. */
  ReturnCode ret_code_list[1]; /**< Return code list. */
} HubSetExtrinsicParameterResponse;

/**
 * The request body of getting the Livox Hub's parameters.
 */
typedef struct {
  uint8_t count;                    /**< Count of code_list. */
  DeviceBroadcastCode code_list[1]; /**< Broadcast code list. */
} HubGetExtrinsicParameterRequest;

/**
 * The response body of getting the Livox Hub's parameters.
 */
typedef struct {
  uint8_t ret_code;        /**< Return code. */
  uint8_t count;           /**< Count of code_list. */
  ExtrinsicParameterResponseItem parameter_list[1]; /**< Extrinsic parameter list. */
} HubGetExtrinsicParameterResponse;

/**
 * The response body of getting sub LiDAR's state conneted to Hub.
 */
typedef struct {
  uint8_t ret_code;             /**< Return code. */
  uint8_t count;                /**< Count of state_list. */
  LidarStateItem state_list[1]; /**< LiDAR units state list. */
} HubQueryLidarStatusResponse;

/**
 * The request body of toggling the Livox Hub's rain and fog mode.
 */
typedef struct {
  uint8_t count;                                /**< Count of lidar_cfg_list. */
  RainFogSuppressRequestItem lidar_cfg_list[1]; /**< Rain fog suppress configuration list. */
} HubRainFogSuppressRequest;

/**
 * The response body of toggling the Livox Hub's rain and fog mode.
 */
typedef struct {
  uint8_t ret_code;             /**< Return code. */
  uint8_t count;                /**< Count of ret_state_list. */
  ReturnCode ret_state_list[1]; /**< Return state list */
} HubRainFogSuppressResponse;

/**
* The response body of getting Hub slots' power state.
*/
typedef struct {
  uint8_t ret_code;             /**< Return code. */
  uint16_t slot_power_state;    /**< Slot power status. */
} HubQuerySlotPowerStatusResponse;

/**
 * The request body of controlling the sub LiDAR's fan state conneted to Hub.
 */
typedef struct {
  uint8_t count;                /**< Count of lidar_cfg_list. */
  FanControlRequestItem lidar_cfg_list[1]; /**< Fan control configuration list. */
} HubFanControlRequest;

/**
 * The response body of controlling the sub LiDAR's fan state conneted to Hub.
 */
typedef struct {
  uint8_t ret_code;             /**< Return code. */
  uint8_t count;                /**< Count of return_list. */
  ReturnCode return_list[1];    /**< Return list */
} HubFanControlResponse;

/**
 * The request body of getting the sub LiDAR's fan state conneted to Hub.
 */
typedef struct {
  uint8_t count;                             /**< Count of lidar_cfg_list. */
  GetFanStateRequestItem lidar_cfg_list[1];  /**< Get Fan state list. */
} HubGetFanStateRequest;

/**
 * The response body of getting the sub LiDAR's fan state conneted to Hub.
 */
typedef struct {
  uint8_t ret_code;                         /**< Return code. */
  uint8_t count;                            /**< Count of return_list. */
  GetFanStateResponseItem return_list[1];   /**< Fan state list. */
} HubGetFanStateResponse;

/**
 * The request body of setting point cloud return mode of sub LiDAR conneted to Hub.
 */
typedef struct {
  uint8_t count;         /**< Count of lidar_cfg_list. */
  SetPointCloudReturnModeRequestItem lidar_cfg_list[1]; /**< Point cloud return mode configuration list. */
} HubSetPointCloudReturnModeRequest;

/**
 * The response body of setting point cloud return mode of sub LiDAR conneted to Hub.
 */
typedef struct {
  uint8_t ret_code;             /**< Return code. */
  uint8_t count;                /**< Count of return_list. */
  ReturnCode return_list[1];    /**< Return list. */
} HubSetPointCloudReturnModeResponse;

/**
 * The request body of getting sub LiDAR's point cloud return mode conneted to Hub.
 */
typedef struct {
  uint8_t count;                /**< Count of lidar_cfg_list. */
  GetPointCloudReturnModeRequestItem lidar_cfg_list[1]; /**< Get point cloud return mode list. */
} HubGetPointCloudReturnModeRequest;

/**
 * The response body of getting sub LiDAR's point cloud return mode conneted to Hub.
 */
typedef struct {
  uint8_t ret_code;               /**< Return code. */
  uint8_t count;                  /**< Count of return_list. */
  GetPointCloudReturnModeResponseItem return_list[1];  /**< Point cloud return mode list. */
} HubGetPointCloudReturnModeResponse;

/**
 * The request body of setting IMU push frequency of sub LiDAR conneted to Hub.
 */
typedef struct {
  uint8_t count;                 /**< Count of lidar_cfg_list. */
  SetImuPushFrequencyRequestItem lidar_cfg_list[1]; /**< IMU push frequency configuration list. */
} HubSetImuPushFrequencyRequest;

/**
 * The response body of setting IMU push frequency of sub LiDAR conneted to Hub.
 */
typedef struct {
  uint8_t ret_code;               /**< Return code. */
  uint8_t count;                  /**< Count of return_list. */
  ReturnCode return_list[1];      /**< Return list. */
} HubSetImuPushFrequencyResponse;

/**
 * The request body of getting sub LiDAR's IMU push frequency conneted to Hub.
 */
typedef struct {
  uint8_t count;                 /**< Count of lidar_cfg_list. */
  GetImuPushFrequencyRequestItem lidar_cfg_list[1];  /**< Get IMU push frequency list. */
} HubGetImuPushFrequencyRequest;

/**
 * The response body of getting sub LiDAR's IMU push frequency conneted to Hub.
 */
typedef struct {
  uint8_t ret_code;              /**< Return code. */
  uint8_t count;                 /**< Count of return_list. */
  GetImuPushFrequencyResponseItem return_list[1];    /**< IMU push frequency list. */
} HubGetImuPushFrequencyResponse;

#pragma pack()

#endif  // LIVOX_DEF_H_
