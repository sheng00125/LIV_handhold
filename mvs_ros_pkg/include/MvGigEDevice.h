
#ifndef __MV_GIGE_DEVICE_H__
#define __MV_GIGE_DEVICE_H__

#include "MvDeviceBase.h"

namespace MvCamCtrl
{
    class CMvGigEDevice : public IMvDevice
    {
    public:

        // ch:打开设备
		// en:Open Device
        virtual int     Open(unsigned int nAccessMode = MV_ACCESS_Exclusive, unsigned short nSwitchoverKey = 0);


        // ch:关闭设备
		// en:Close Device
        virtual int     Close();


        // ch:判断设备的状态，仅当返回false时，可打开设备
		// en:Determines the status of the device, only if false is returned, the device can be opened 
        virtual bool    IsOpen();


        // ch:开启抓图
		// en:Start Grabbing
        virtual int     StartGrabbing();


        // ch:停止抓图
		// en:Stop Grabbing
        virtual int     StopGrabbing();


        // ch:获取设备信息
		// en:Get Device Information
        virtual int     GetDeviceInfo(MV_CC_DEVICE_INFO&);


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
         *  @brief  Get Device XML File
         *  @param  pData           [IN][OUT]   - Cache Address to copy in
                    nDataSize       [IN]        - Cache Size
                    pnDataLen       [OUT]       - XML File Data Length
         *  
         *  @return Success, return MV_OK; Fail, return Error Code
         *  @note   When pData is NULL or nDataSize is smaller than the actual xml file, do not copy data, return xml file size from the pnDataLen;
         *          When pData is a valid cache address and the cache is large enough, the full data is copied and the xml file size is returned by pnDataLen.
         */
        virtual int     GetGenICamXML(unsigned char* pData, unsigned int nDataSize, unsigned int* pnDataLen);


        /** @fn     GetOneFrame(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO* pFrameInfo)
         *  @brief  获取一帧图像数据
         *  @param  pData           [IN][OUT]   - 数据指针
                    nDataLen        [IN]        - 数据长度
                    pFrameInfo      [OUT]       - 输出的帧信息
         *  
         *  @return 成功，返回MV_OK；失败，返回错误码
		 
		 * @fn     GetOneFrame(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO* pFrameInfo)
         *  @brief  Get One Frame Image Data
         *  @param  pData           [IN][OUT]   - Data Pointer
                    nDataLen        [IN]        - Data Length
                    pFrameInfo      [OUT]       - Output Frame Information
         *  
         *  @return Success, return MV_OK; Fail, return Error Code
         */
        virtual int     GetOneFrame(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO* pFrameInfo);


        // ch:获取GenICam使用的传输层代理类
		// en:Get the transport layer agent class used by GenICam
        virtual TlProxy     GetTlProxy();


        virtual ~CMvGigEDevice( void );


        CMvGigEDevice( const MV_CC_DEVICE_INFO* pInfo );


        // ch:获取网络信息
		// en:Get Net Information
        virtual int     GetNetTransInfo(MV_NETTRANS_INFO* pstInfo);


        // ch:强制IP
		// en:Force IP
        virtual int     ForceIp(unsigned int nIP);


        // ch:配置IP方式
		// en:Configure IP Mode
        virtual int     SetIpConfig(unsigned int nType);


        // ch:获取各种类型的信息
		// en:Get Various Types of Information
        virtual int     GetAllMatchInfo(MV_ALL_MATCH_INFO* pstInfo);

        // ch:注册消息异常回调
		// en:Register Message Exception Callback
        virtual int     RegisterExceptionCallBack(void(__stdcall* cbException)(unsigned int nMsgType, void* pUser),
                                                    void* pUser);

        virtual int     SetSingleShot(void(__stdcall* cbSingleShot)(unsigned char* pData , unsigned int nDataLen, 
                                                                    MV_FRAME_OUT_INFO* pFrameInfo, void* pUser), 
                                        void* pUser);

        // ch:设置设备采集模式
		// en:Set Device Aquisition Mode
        virtual int     SetAcquisitionMode(MV_CAM_ACQUISITION_MODE enMode);


        // ch:设备本地升级
		// en:Device Local Update
        virtual int     LocalUpgrade(const void *pFilePathName);

        // ch:获取当前升级进度
		// en:Get Current Update Process
        virtual int     GetUpgradeProcess(unsigned int* pnProcess);

        virtual int     GetOptimalPacketSize();

        // ch:显示一帧图像
		// en:Display One Frame Image
        virtual int     Display(void* hWnd);

        virtual int     SetNetTransMode(unsigned int nType);

        virtual int     ReadMemory(void *pBuffer, int64_t nAddress, int64_t nLength);

        virtual int     WriteMemory(const void *pBuffer, int64_t nAddress, int64_t nLength);

        // ch:返回值 0驱动不正常，1驱动正常工作
		// en:Return Value 0 driver exception, 1 driver work properly
        virtual int     IsDriverWorking();

        // ch:0：不丢弃；1：丢弃
		// en:0: Do not throw; 1: Throw
        virtual int     SetThrowAbnormalImage(int bThrow);

        // ch:设置SDK内部图像缓存节点个数，范围[1, 30]，在抓图前调用
		// en:Set the number of the internal image cache nodes in SDK, range [1, 30], called before capture
        virtual int     SetImageNodeNum(unsigned int nNum);

        // ch:设置gvcp超时时间
		// en:Set Gvcp Timeout
        virtual int     SetGvcpTimeout(unsigned int nTimeout);

        // ch:注册图像数据回调
		// en:Register Image Data Callback
        virtual int     RegisterImageCallBack(void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO* pFrameInfo, void* pUser),
                                                    void* pUser);
        // ch:注册EVENT消息回调，已废弃
		// en:Register Event Message Callback
        virtual int     RegisterEventCallBack(void(__stdcall* cbEvent)(unsigned int nUserDefinedId, void* pUser),
                                                void* pUser);

        // ch:注册全部EVENT消息回调
		// en:Register all event Callback, chunk
        virtual int     RegisterAllEventCallBack(void(__stdcall* cbEvent)(MV_EVENT_OUT_INFO * pEventInfo, void* pUser),void* pUser);

        // ch:注册单个EVENT消息回调
		// en:Register event Callback, chunk
        virtual int     RegisterEventCallBack(const char* pEventName,void(__stdcall* cbEvent)(MV_EVENT_OUT_INFO * pEventInfo, void* pUser),
                                                void* pUser);

        // ch:注册图像数据回调,chunk
		// en:Register Image Data Callback, chunk
        virtual int     RegisterImageCallBackEx(void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser),void* pUser);

		virtual int     RegisterImageCallBackForRGB(void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser),void* pUser);
		virtual int     RegisterImageCallBackForBGR(void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser),void* pUser);

        virtual int     GetOneFrameEx(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO_EX* pFrameInfo);

        virtual int     GetImageForRGB(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO_EX* pFrameInfo, int nMsec);
        virtual int     GetImageForBGR(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO_EX* pFrameInfo, int nMsec);

        virtual int     GetOneFrameTimeout(unsigned char * pData , unsigned int nDataSize, MV_FRAME_OUT_INFO_EX* pFrameInfo, int nMsec);

    private:
        CDevRefs*       m_pRefs;
    };


}

#endif /* __MV_GIGE_DEVICE_H__ */
