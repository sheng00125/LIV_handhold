
#ifndef __MV_TLFACTORY_H__
#define __MV_TLFACTORY_H__

#include "GenApi/Synch.h"
#include "MvInclude.h"
#include "MvDeviceBase.h"

namespace MvCamCtrl
{
    class MV_CAMCTRL_API CTlFactory : public IDeviceFactory
    {
    public:

        /// Retrieve the transport layer factory singleton
        static CTlFactory& GetInstance();


        /// Retrieve all available transport layers
        unsigned int EnumerateTls();


        /** @fn     EnumDevices( unsigned int nTLayerType , MV_CC_DEVICE_INFO_LIST& stDevList )
         *  @brief  枚举子网内，指定的传输协议对应的所有设备
         *  @param  nTLayerType     [IN]    - 指定的传输协议
                    stDevList       [OUT]   - 设备信息列表
         *  
         *  @return 成功，返回MV_OK；失败，返回错误码
		 
		 * @fn     EnumDevices( unsigned int nTLayerType , MV_CC_DEVICE_INFO_LIST& stDevList )
         *  @brief  Enumerate all devices within the subnet that correspond to the specified transport protocol
         *  @param  nTLayerType     [IN]    - Specified transport protocol
                    stDevList       [OUT]   - Device information list
         *  
         *  @return Success, return MV_OK; Fail, return Error code
         */
        virtual int EnumDevices( unsigned int nTLayerType , MV_CC_DEVICE_INFO_LIST& stDevList );


        /** @fn     CreateDevice( const MV_CC_DEVICE_INFO& device )
         *  @brief  创建设备代理类
         *  @param  device          [IN]    - 设备信息（仅要求传输层类型有效即可）
         *  
         *  @return 成功，返回设备代理实例；失败，返回NULL
		 
		 * @fn     CreateDevice( const MV_CC_DEVICE_INFO& device )
         *  @brief  Create Device Agent Class
         *  @param  device          [IN]    - Device Information（Only the transport layer type valid is required）
         *  
         *  @return Success, return device agent instance; Fail, return NULL
         */
        virtual IMvDevice* CreateDevice( const MV_CC_DEVICE_INFO& device );


        /** @fn     DestroyDevice( IMvDevice* pDevice)
         *  @brief  销毁指定设备的内部资源
         *  @param  pDevice         [IN]    - 设备对象
         *  
         *  @return 成功，返回MV_OK；失败，返回错误码
		 
		 * @fn     DestroyDevice( IMvDevice* pDevice)
         *  @brief  Destroy the internal resources of the specified device
         *  @param  pDevice         [IN]    - Device Object
         *  
         *  @return Success, return MV_OK; Fail, return Error code
         */
        virtual int DestroyDevice( IMvDevice* );


        /** @fn     IsDeviceAccessible( const MV_CC_DEVICE_INFO& deviceInfo)
         *  @brief  判断指定的设备是否可以访问
         *  @param  deviceInfo      [IN]    - 指定的设备信息
         *  
         *  @return 可以访问，返回 true ；没有权限或设备已掉线，返回 false 
         *  @note   暂不支持
		 
		 * @fn     IsDeviceAccessible( const MV_CC_DEVICE_INFO& deviceInfo)
         *  @brief  Determine whether the specified device is accessible
         *  @param  deviceInfo      [IN]    - Specified device information
         *  
         *  @return Can be accessed, return true; no permissions or the device is dropped, return false
         *  @note   Not supported yet
         */
        virtual bool IsDeviceAccessible( const MV_CC_DEVICE_INFO& deviceInfo);

        ~CTlFactory( void );

        virtual unsigned int  GetSDKVersion();

    protected:

        static CTlFactory   m_sTLinstance;
        GenApi::CLock       m_cLock;
        CTlRefs*            m_pCreatedTls;

    private:

        CTlFactory( void );
        CTlFactory& operator=( const CTlFactory& );
        CTlFactory( const CTlFactory& );
    };

}

#endif /* __MV_TLFACTORY_H__ */
