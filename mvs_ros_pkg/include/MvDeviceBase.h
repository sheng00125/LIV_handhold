
#ifndef __MV_DEVICE_BASE_H__
#define __MV_DEVICE_BASE_H__

#include "MvInclude.h"

namespace MvCamCtrl
{

    interface   IMvDevice
    {

        // ch:打开设备
		// en:Open Device
        virtual int     Open(unsigned int nAccessMode = MV_ACCESS_Exclusive, unsigned short nSwitchoverKey = 0)    = 0;


        // ch:关闭设备
		// en:Close Device
        virtual int     Close()                                 = 0;


        // ch:判断设备的状态，仅当返回false时，可打开设备
		// en:Determines the status of the device, which can be opened only if false is returned
        virtual bool    IsOpen()                                = 0;


        // ch:开启抓图
		// en:Start Grabbing
        virtual int     StartGrabbing()                         = 0;


        // ch:停止抓图
		// en:Stop Grabbing
        virtual int     StopGrabbing()                          = 0;


        // ch:获取设备信息
		// en:Get Device Information
        virtual int     GetDeviceInfo(MV_CC_DEVICE_INFO&)       = 0;


        /** @fn     GetGenICamXML(unsigned char* pData, unsigned int nDataSize, unsigned int* pnDataLen)
         *  @brief  获取设备的XML文件
         *  @param  pData           [IN][OUT]   - 待拷入的缓存地址
                    nDataSize       [IN]        - 缓存大小
                    pnDataLen       [OUT]       - xml 文件数据长度
         *  
         *  @return 成功，返回MV_OK；失败，返回错误码
         *  @note   当pData为NULL或nDataSize比实际的xml文件小时，不拷贝数据，由pnDataLen返回xml文件大小；
         *          当pData为有效缓存地址，且缓存足够大时，拷贝完整数据，并由pnDataLen返回xml文件大小。
		 
		 * @fn     GetGenICamXML(unsigned char* pData, unsigned int nDataSize, unsigned int* pnDataLen)
         *  @brief  Get the device XML file
         *  @param  pData           [IN][OUT]   - Cache Address to copy in
                    nDataSize       [IN]        - Cache Size
                    pnDataLen       [OUT]       - xml File Data Length
         *  
         *  @return Success, return MV_OK; Fail, return error code
         *  @note   When pData is NULL or nDataSize smaller than the actual xml file, do not copy the data, return xml file size from pnDataLen;
         *          When pData is a valid cache address, and the cache is large enough, the full data is copied and the xml file size is returned by pnDataLen.
         */
        virtual int     GetGenICamXML(unsigned char* pData, unsigned int nDataSize, unsigned int* pnDataLen)        = 0;


        /** @fn     GetOneFrame(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO* pFrameInfo)
         *  @brief  获取一帧图像数据
         *  @param  pData           [IN][OUT]   - 数据指针
                    nDataLen        [IN]        - 数据长度
                    pFrameInfo      [OUT]       - 输出的帧信息
         *  
         *  @return 成功，返回MV_OK；失败，返回错误码
		 
		 * @fn     GetOneFrame(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO* pFrameInfo)
         *  @brief  Get one frame image data
         *  @param  pData           [IN][OUT]   - Data Pointer
                    nDataLen        [IN]        - Data Length
                    pFrameInfo      [OUT]       - Output Frame Information
         *  
         *  @return Success, return MV_OK; Fail, return error code
         */
        virtual int     GetOneFrame(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO* pFrameInfo)  = 0;


        // ch:获取GenICam使用的传输层代理类
		// en:Gets the transport layer agent class used by GenICam
        virtual TlProxy     GetTlProxy()                                                                            = 0;

        virtual ~IMvDevice( void ){};
    };


    interface   IDeviceFactory
    {

        // ch:枚举子网内，指定的传输协议对应的所有设备
		// en:Enumerate all devices within the subnet that correspond to the specified transport protocol
        virtual int EnumDevices( unsigned int nTLayerType , MV_CC_DEVICE_INFO_LIST& stDevList )     = 0;


        // ch:创建设备代理类
		// en:Create a device agent class
        virtual IMvDevice* CreateDevice( const MV_CC_DEVICE_INFO& device )                          = 0;


        // ch:销毁指定设备的内部资源
		// en:Destroy the internal resources of the specified device
        virtual int DestroyDevice( IMvDevice* )                                                     = 0;


        // ch:判断指定的设备是否可以访问
		// en:Determines whether the specified device is accessible
        virtual bool IsDeviceAccessible( const MV_CC_DEVICE_INFO& deviceInfo)                       = 0;
    };

    

}

#endif /* __MV_DEVICE_BASE_H__ */
