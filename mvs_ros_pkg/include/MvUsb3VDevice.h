
#ifndef __MV_USB3V_DEVICE_H__
#define __MV_USB3V_DEVICE_H__

#include "MvDeviceBase.h"

namespace MvCamCtrl
{
    class CMvUsb3VDevice : public IMvDevice
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
         *  @brief  Get Device XML Files
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


        virtual ~CMvUsb3VDevice( void );


        CMvUsb3VDevice( const MV_CC_DEVICE_INFO* pInfo );

        // ch:显示一帧图像
		// en:Display One Frame Image
        virtual int     Display(void* hWnd);


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
		// en:Device Local Upgrade
        virtual int     LocalUpgrade(const void *pFilePathName);

        // ch:获取当前升级进度
		// en:Get Current Upgrade Process
        virtual int     GetUpgradeProcess(unsigned int* pnProcess);

        virtual int     ReadMemory(void *pBuffer, int64_t nAddress, int64_t nLength);

        virtual int     WriteMemory(const void *pBuffer, int64_t nAddress, int64_t nLength);

        // ch:设置SDK内部图像缓存节点个数，范围[1, 30]，在抓图前调用
		// en:Set the number of the internal image cache nodes in SDK, range [1, 30], called before capture
        virtual int     SetImageNodeNum(unsigned int nNum);

        // ch:注册图像数据回调
		// en:Register Image Data Callback
        virtual int     RegisterImageCallBack(void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO* pFrameInfo, void* pUser),
                                                void* pUser);

    private:
        CDevRefs*       m_pRefs;
    };


}

#endif /* __MV_GIGE_DEVICE_H__ */
