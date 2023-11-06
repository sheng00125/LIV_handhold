#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <stdio.h>

using namespace std;

bool is_undistorted = true;
unsigned int g_nPayloadSize = 0;
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  int ExposureAuto = Params["ExposureAuto"];
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int GainAuto = Params["GainAuto"];
  int GammaSelector = Params["GammaSelector"];
  float FrameRate = Params["FrameRate"];
  int ExposureTime = Params["ExposureTime"];

  int nRet;
  nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", ExposureAuto);
  if (MV_OK == nRet) {
    std::string msg = "Set Exposure Auto: " + ExposureAutoStr[ExposureAuto];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Exposure auto mode");
  }

  // nRet = MV_CC_SetExposureTime(handle, ExposureTime);
  // if (MV_OK == nRet) {
  //   std::string msg =
  //       "Set Exposure Time: " + std::to_string(ExposureTime) + "ms";
  //   ROS_INFO_STREAM(msg.c_str());
  // } else {
  //   ROS_ERROR_STREAM("Fail to set Exposure Time");
  // }

  // nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
  // if (MV_OK == nRet) {
  //   std::string msg =
  //       "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) + "ms";
  //   ROS_INFO_STREAM(msg.c_str());
  // } else {
  //   ROS_ERROR_STREAM("Fail to set Exposure Time Lower");
  // }

  // nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
  // if (MV_OK == nRet) {
  //   std::string msg =
  //       "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) + "ms";
  //   ROS_INFO_STREAM(msg.c_str());
  // } else {
  //   ROS_ERROR_STREAM("Fail to set Exposure Time Upper");
  // }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);
  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gain auto mode");
  }
  
  nRet = MV_CC_SetFrameRate(handle, FrameRate);
  if (MV_OK == nRet) {
    std::string msg = "Set Frame Rate: " + std::to_string(FrameRate) + "hz";
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Frame Rate");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grab");
  std::string params_file = std::string(argv[1]);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  int nRet = MV_OK;
  void *handle = NULL;
  ros::Rate loop_rate(10);
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  float ExposureTime_init = 5000;
  float ExposureTime_final = 50000;
  float ExposureTime = ExposureTime_init;
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];

  image_transport::Publisher pub = it.advertise(pub_topic, 1);

  while (ros::ok()) {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    // 枚举检测到的相机数量
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("Enum Devices fail!");
      break;
    }

    bool find_expect_camera = false;
    int expect_camera_index = 0;
    if (stDeviceList.nDeviceNum == 0) {
      ROS_ERROR_STREAM("No Camera.\n");
      break;
    } else {
      // 根据serial number启动指定相机
      for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        std::string serial_number =
            std::string((char *)stDeviceList.pDeviceInfo[i]
                            ->SpecialInfo.stUsb3VInfo.chSerialNumber);
        if (expect_serial_number == serial_number) {
          find_expect_camera = true;
          expect_camera_index = i;
          break;
        }
      }
    }
    if (!find_expect_camera) {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      ROS_ERROR_STREAM(msg.c_str());
      break;
    }

    nRet = MV_CC_CreateHandle(&handle,
                              stDeviceList.pDeviceInfo[expect_camera_index]);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("Create Handle fail");
      break;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("Open Device fail\n");
      break;
    } else {
      ROS_INFO_STREAM("Successfully open camera");
    }

    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("Set Trigger Mode fail\n");
      break;
    }

    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("Get PayloadSize fail\n");
      break;
    }
    g_nPayloadSize = stParam.nCurValue;

    nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
    if (nRet != MV_OK) {
      ROS_ERROR_STREAM("Pixel setting can't work.");
      break;
    }

    setParams(handle, params_file);

    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("Start Grabbing fail.\n");
      break;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    unsigned char *pData =
        (unsigned char *)malloc(sizeof(unsigned char) * (g_nPayloadSize));

    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {

      std::string msg =
          "Set Exposure Time: " + std::to_string(ExposureTime / 1000) + "ms";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time");
    }

    nRet =
        MV_CC_GetImageForBGR(handle, pData, g_nPayloadSize, &stImageInfo, 100);
    if (MV_OK != nRet) {
      ROS_ERROR_STREAM("No data");
      std::free(pData);
      pData = NULL;
      break;
    } else {
      ExposureTime += 45;
      if (ExposureTime >= ExposureTime_final)
        break;
    }

    std::string msg =
        "Open cameraSucessfully! Publish image data to " + pub_topic;
    ROS_INFO_STREAM(msg.c_str());

    cv::Size imageSize;
    imageSize.height = stImageInfo.nHeight;
    imageSize.width = stImageInfo.nWidth;

    // cv::Mat view, rview, map1, map2;
    // cv::initUndistortRectifyMap(
    //     cameraMatrix, distCoeffs, cv::Mat(),
    //     cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1,
    //                                   imageSize, 0),
    //     imageSize, CV_16SC2, map1, map2);

    while (ros::ok()) {
      memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
      if (pData == NULL) {
        ROS_ERROR_STREAM("Allocate memory failed.\n");
        break;
      }

      nRet = MV_CC_GetImageForBGR(handle, pData, g_nPayloadSize, &stImageInfo,
                                  150);
      if (MV_OK != nRet) {
        ROS_ERROR_STREAM("No data");
        std::free(pData);
        pData = NULL;
        break;
      }

      cv::Mat srcImage, calibration;
      srcImage =
          cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
      msg->header.stamp = ros::Time::now();
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      srcImage.release();
    }
    // fs.release();
    free(pData);
    nRet = MV_CC_StopGrabbing(handle);
    nRet = MV_CC_CloseDevice(handle);
    nRet = MV_CC_DestroyHandle(handle);
    break;
  }

  if (nRet != MV_OK) {
    if (handle != NULL) {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
  }

  return 0;
}