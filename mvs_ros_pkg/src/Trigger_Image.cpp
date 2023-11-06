#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using namespace std;

bool g_bExit = false;
image_transport::Publisher pub;

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void) {
  int c;
  while ((c = getchar()) != '\n' && c != EOF)
    ;
  fprintf(stderr, "\nPress enter to exit.\n");
  while (getchar() != '\n')
    ;
  g_bExit = true;
  sleep(1);
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (NULL == pstMVDevInfo) {
    printf("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
    int nIp1 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined
    // name
    printf("Device Model Name: %s\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    printf("UserDefinedName: %s\n\n",
           pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    printf("Device Model Name: %s\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("UserDefinedName: %s\n\n",
           pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
  } else
    printf("Not support.\n");

  return true;
}

static void *WorkThread(void *pUser) {
  int nRet = MV_OK;

  // ch:获取数据包大小 | en:Get payload size
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return NULL;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  unsigned char *pData =
      (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (NULL == pData)
    return NULL;

  unsigned int nDataSize = stParam.nCurValue;

  while (1) {

    nRet = MV_CC_SetCommandValue(pUser, "TriggerSoftware");
    if (MV_OK != nRet)
      printf("failed in TriggerSoftware[%x]\n", nRet);

    nRet =
        MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
             stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
      cv::Mat srcImage;
      srcImage =
          cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
      pub.publish(msg);
    }
    // else
    //     printf("Get One Frame failed![%x]\n", nRet);

    if (g_bExit)
      break;
  }

  if (pData) {
    free(pData);
    pData = NULL;
  }

  return 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "trigger");
  // std::string params_file = std::string(argv[1]);
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  pub = it.advertise("mvs_camera/image", 1);

  int nRet = MV_OK;

  void *handle = NULL;
  do {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
      printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
      break;
    }
    if (stDeviceList.nDeviceNum > 0) {
      for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
        printf("[device %d]:\n", i);
        MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (NULL == pDeviceInfo)
          break;

        PrintDeviceInfo(pDeviceInfo);
      }
    } else {
      printf("Find No Devices!\n");
      break;
    }

    printf("Please Intput camera index: ");
    unsigned int nIndex = 0;
    scanf("%d", &nIndex);

    if (nIndex >= stDeviceList.nDeviceNum) {
      printf("Intput error!\n");
      break;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet) {
      printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
      break;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
      break;
    }

    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal
    // package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) {
      int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
      if (nPacketSize > 0) {
        nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
        if (nRet != MV_OK)
          printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
      } else
        printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
    }

    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
    if (MV_OK != nRet) {
      printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
      break;
    }

    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 2);
    if (MV_OK != nRet)
      ROS_ERROR("Fail to set Exposure auto mode");

    nRet = MV_CC_SetAutoExposureTimeLower(handle, 65);
    if (MV_OK != nRet)
      ROS_ERROR("Fail to set Exposure Time Lower");

    nRet = MV_CC_SetAutoExposureTimeUpper(handle, 10000);
    if (MV_OK != nRet)
      ROS_ERROR("Fail to set Exposure Time Upper");

    nRet = MV_CC_SetEnumValue(handle, "GainAuto", 2);
    if (MV_OK != nRet)
      ROS_ERROR("Fail to set Gain auto mode");

    // nRet = MV_CC_SetFrameRate(handle, 10);
    // if (MV_OK != nRet)
    //     ROS_ERROR("Fail to set Frame Rate");

    // 设置触发模式为on
    // set trigger mode as on
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
      break;
    }

    // 设置触发源
    // set trigger source
    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
    if (MV_OK != nRet) {
      printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
      break;
    }

    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    pthread_t nThreadID;
    nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
    if (nRet != 0) {
      printf("thread create failed.ret = %d\n", nRet);
      break;
    }

    PressEnterToExit();

    // 停止取流
    // end grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
      break;
    }

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
      break;
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
      printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
      break;
    }
  } while (0);

  if (nRet != MV_OK) {
    if (handle != NULL) {
      MV_CC_DestroyHandle(handle);
      handle = NULL;
    }
  }

  printf("exit\n");
  return 0;
}
