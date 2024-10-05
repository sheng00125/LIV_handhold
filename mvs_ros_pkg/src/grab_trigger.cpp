#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <chrono>
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>

using namespace std;

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt;

unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
bool exit_flag = false;
image_transport::Publisher pub;
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
float image_scale = 0.0;
int trigger_enable = 1;

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  image_scale = Params["image_scale"];   
  if (image_scale < 0.1) image_scale = 1;
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int ExposureTime = Params["ExposureTime"];
  int ExposureAutoMode = Params["ExposureAutoMode"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];
  int nRet;

  // 设置曝光模式
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
      std::string msg = "Set ExposureAutoMode: " + ExposureAutoStr[ExposureAutoMode];

  if (MV_OK == nRet) {
    ROS_INFO_STREAM(msg.c_str());
  } else {
    if(ExposureAutoMode == 2) {
      ROS_WARN_STREAM("Fail to set Exposure Auto Mode to Continues");
    }
    else {
      ROS_INFO_STREAM(msg.c_str());
    }
  }

  // 如果是自动曝光
  if (ExposureAutoMode == 2) {
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) +
          "us";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Lower");
    }
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) +
          "us";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time: " + std::to_string(ExposureTime) + "us";
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Exposure Time");
    }
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);

  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gain auto mode");
  }

  if (GainAuto == 0) {
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      std::string msg = "Set Gain: " + std::to_string(Gain);
      ROS_INFO_STREAM(msg.c_str());
    } else {
      ROS_ERROR_STREAM("Fail to set Gain");
    }
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set GammaSlector");
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    ROS_INFO_STREAM(msg.c_str());
  } else {
    ROS_ERROR_STREAM("Fail to set Gamma");
  }
}

void SignalHandler(int signal) {
  if (signal == SIGINT) {  // 捕捉 Ctrl + C 触发的 SIGINT 信号
    fprintf(stderr, "\nReceived Ctrl+C, exiting...\n");
    exit_flag = true;    // 设置退出标志
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler; // 设置处理函数
  sigemptyset(&sigIntHandler.sa_mask);      // 清空信号屏蔽集
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
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
  unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (NULL == pData) return NULL;

  unsigned int nDataSize = stParam.nCurValue;

  while (!exit_flag && ros::ok()) {

    nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      
      ros::Time rcv_time;
      if(trigger_enable && pointt != MAP_FAILED && pointt->low != 0)
      {
        // 赋值共享内存中的时间戳给相机帧
        int64_t b = pointt->low;
        double time_pc = b / 1000000000.0;
        rcv_time = ros::Time(time_pc);
      }
      else
      {
        rcv_time = ros::Time::now();
      }
     
      std::string debug_msg;
      debug_msg = "GetOneFrame,nFrameNum[" +
                  std::to_string(stImageInfo.nFrameNum) + "], FrameTime:" +
                  std::to_string(rcv_time.toSec());
      // ROS_INFO_STREAM(debug_msg.c_str());

      cv::Mat srcImage;
      srcImage =
          cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      cv::resize(srcImage, srcImage,
                 cv::Size(srcImage.cols * image_scale,
                          srcImage.rows * image_scale),
                 CV_INTER_LINEAR);

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", srcImage).toImageMsg();
      msg->header.stamp = rcv_time;
      pub.publish(msg);
    }
  }

  if (pData) {
    free(pData);
    pData = NULL;
  }

  return 0;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "mvs_trigger");
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
  trigger_enable = Params["TriggerEnable"];
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  pub = it.advertise(pub_topic, 1);
  
  const char *user_name = getlogin();
  std::string path_for_time_stamp = "/home/" + std::string(user_name) + "/timeshare";
  const char *shared_file_name = path_for_time_stamp.c_str();

  int fd = open(shared_file_name, O_RDWR);

  pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, 0);

  SetupSignalHandler();
 
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    printf("Enum Devices fail!");
    return -1;
  }

  if (stDeviceList.nDeviceNum == 0) {
    ROS_ERROR_STREAM("No Camera.\n");
    return -1;
  } 
  else 
  {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) 
    {
      nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[i]);
      if (MV_OK != nRet) 
      {
        ROS_ERROR_STREAM("Failed to create handle for device " << i << "Error code: " << nRet);
        continue; // Continue trying other devices if available
      }
      else 
      {
        ROS_INFO_STREAM("Successfully created handle for device " << i);
        break; // Exit the loop after successfully creating a handle
      }
    }
    if (MV_OK != nRet) 
    {
      ROS_ERROR_STREAM("Failed to create handle for any device.");
      return -1; // Ensure function exits if no handle could be created
    }
  }

  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet) {
    printf("Open Device fail\n");
    return -1;
  }

  nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
  if (MV_OK != nRet) {
    printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
    return -1;
  }

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail\n");
    return -1;
  }
  g_nPayloadSize = stParam.nCurValue;

  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
  if (nRet != MV_OK) {
    printf("Pixel setting can't work.");
    return -1;
  }

  setParams(handle, params_file);

  // set trigger mode as on
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) {
    printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
    return -1;
  }

  // set trigger source
  nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
  if (MV_OK != nRet) {
    printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
    return -1;
  }

  ROS_INFO("Finish all params set! Start grabbing...");
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    printf("Start Grabbing fail.\n");
    return -1;
  }

  pthread_t nThreadID;
  nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
  if (nRet != 0) {
    printf("thread create failed.ret = %d\n", nRet);
    return -1;
  }

  while (!exit_flag && ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  if (nThreadID) {
    pthread_join(nThreadID, NULL);
    ROS_INFO_STREAM("Worker thread joined.");
  }

  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_CloseDevice(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_DestroyHandle(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    return -1;
  }

  munmap(pointt, sizeof(time_stamp));

  return 0;
}