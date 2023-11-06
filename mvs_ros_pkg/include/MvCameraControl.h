
#ifndef _MV_CAMERA_CTRL_H_
#define _MV_CAMERA_CTRL_H_

#include "CameraParams.h"
#include "MvErrorDefine.h"


/**
*  @brief  ��̬�⵼�뵼������
*  @brief  Import and export definition of the dynamic library
*/
#ifndef MV_CAMCTRL_API

#if (defined(_WIN32) || defined(WIN64))
#if defined(MV_CAMCTRL_EXPORTS)
#define MV_CAMCTRL_API __declspec(dllexport)
#else
#define MV_CAMCTRL_API __declspec(dllimport)
#endif
#else
#ifndef __stdcall
#define __stdcall
#endif

#if defined(MV_CAMCTRL_EXPORTS)
#define MV_CAMCTRL_API __attribute__((visibility("default")))
#else
#define MV_CAMCTRL_API
#endif
#endif

#endif

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************/
/* ����Ļ���ָ��Ͳ���                                         		*/
/* Camera basic instructions and operations                             */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CC_GetSDKVersion()
 *  @brief  ��ȡSDK�汾��
 *  @param
 *  @return ʼ�շ���4�ֽڰ汾�� |��    |��    |����  |  ����|
                                 8bits  8bits  8bits  8bits

 *  @fn     MV_CC_GetSDKVersion()
 *  @brief  Get SDK Version
 *  @param
 *  @return Always return 4 Bytes of version number |Main    |Sub    |Rev  |
 Test|
                                                     8bits  8bits  8bits  8bits
 ************************************************************************/
MV_CAMCTRL_API unsigned int __stdcall MV_CC_GetSDKVersion();

/************************************************************************
 *  @fn     MV_CC_EnumerateTls()
 *  @brief  ��ȡ֧�ֵĴ����
 *  @return ֧�ֵĴ������

 *  @fn     MV_CC_EnumerateTls()
 *  @brief  Get supported Transport Layer
 *  @return Supported Transport Layer number
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumerateTls();

/************************************************************************
 *  @fn     MV_CC_EnumDevices()
 *  @brief  ö���豸
 *  @param  nTLayerType            [IN]           ö�ٴ����
 *  @param  pstDevList             [OUT]          �豸�б�
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_EnumDevices()
 *  @brief  Enumerate Device
 *  @param  nTLayerType            [IN]           Enumerate TLs
 *  @param  pstDevList             [OUT]          Device List
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevices(
    IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST *pstDevList);

/************************************************************************
 *  @fn     MV_CC_EnumDevicesEx()
 *  @brief  ���ݳ�������ö���豸
 *  @param  nTLayerType            [IN]           ö�ٴ����
 *  @param  pstDevList             [OUT]          �豸�б�
 *  @param  pManufacturerName      [IN]           ��������
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_EnumDevicesEx()
 *  @brief  Enumerate device according to manufacture name
 *  @param  nTLayerType            [IN]           Transmission layer of
 enumeration
 *  @param  pstDevList             [OUT]          Device list
 *  @param  pManufacturerName      [IN]           Manufacture Name
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesEx(
    IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST *pstDevList,
    IN const char *pManufacturerName);

/************************************************************************
 *  @fn     MV_CC_IsDeviceAccessible()
 *  @brief  �豸�Ƿ�ɴ�
 *  @param  pstDevInfo             [IN]           �豸��Ϣ�ṹ��
 *  @param  nAccessMode            [IN]           ����Ȩ��
 *  @return �ɴ����true�����ɴ����false

 *  @fn     MV_CC_IsDeviceAccessible()
 *  @brief  Is the device accessible
 *  @param  pstDevInfo             [IN]           Device Information Structure
 *  @param  nAccessMode            [IN]           Access Right
 *  @return Access, return true. Not access, return false
 ************************************************************************/
MV_CAMCTRL_API bool __stdcall MV_CC_IsDeviceAccessible(
    IN MV_CC_DEVICE_INFO *pstDevInfo, IN unsigned int nAccessMode);

/************************************************************************
 *  @fn     MV_CC_SetSDKLogPath()
 *  @brief  ����SDK��־·���������־����MvLogServer�����ã���ýӿ���Ч��
 *  @param  pSDKLogPath             [IN]           SDK��־·��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_SetSDKLogPath()
 *  @brief  Set SDK log path
 *  @param  pSDKLogPath             [IN]           SDK log path
 *  @return Access, return true. Not access, return false
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetSDKLogPath(IN const char *pSDKLogPath);

/************************************************************************
 *  @fn     MV_CC_CreateHandle()
 *  @brief  �����豸���
 *  @param  handle                 [OUT]          �����ַ
 *  @param  pstDevInfo             [IN]           �豸��Ϣ�ṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_CreateHandle()
 *  @brief  Create Device Handle
 *  @param  handle                 [OUT]          Handle Address
 *  @param  pstDevInfo             [IN]           Device Information Structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandle(
    OUT void **handle, IN const MV_CC_DEVICE_INFO *pstDevInfo);

/************************************************************************
 *  @fn     MV_CC_CreateHandleWithoutLog
 *  @brief  �����豸�������������־
 *  @param  handle                 [OUT]          �����ַ
 *  @param  pstDevInfo             [IN]           �豸��Ϣ�ṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_CreateHandleWithoutLog
 *  @brief  Create Device Handle without log
 *  @param  handle                 [OUT]          Handle Address
 *  @param  pstDevInfo             [IN]           Device Information Structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandleWithoutLog(
    OUT void **handle, IN const MV_CC_DEVICE_INFO *pstDevInfo);

/************************************************************************
 *  @fn     MV_CC_DestroyHandle()
 *  @brief  �����豸���
 *  @param  handle                 [IN]          ���
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_DestroyHandle()
 *  @brief  Destroy Device Handle
 *  @param  handle                 [IN]          Handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DestroyHandle(IN void *handle);

/************************************************************************
 *  @fn     MV_CC_OpenDevice()
 *  @brief  ���豸
 *  @param  handle                 [IN]          ���
 *  @param  nAccessMode            [IN]          ����Ȩ��
 *  @param  nSwitchoverKey         [IN]          �л�����Ȩ��ʱ����Կ
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_OpenDevice()
 *  @brief  Open Device
 *  @param  handle                 [IN]          Handle
 *  @param  nAccessMode            [IN]          Access Right
 *  @param  nSwitchoverKey         [IN]          Switch key of access right
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
#ifndef __cplusplus
MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(IN void *handle,
                                              IN unsigned int nAccessMode,
                                              IN unsigned short nSwitchoverKey);
#else
MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(
    IN void *handle, IN unsigned int nAccessMode = MV_ACCESS_Exclusive,
    IN unsigned short nSwitchoverKey = 0);
#endif

/***********************************************************************
 *  @fn         MV_CC_CloseDevice
 *  @brief      �ر����
 *  @param       handle                 [IN]          ���
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_CloseDevice
 *  @brief      Close Device
 *  @param      handle                 [IN]          Handle
 *  @return     Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CloseDevice(IN void *handle);

/***********************************************************************
 *  @fn         MV_CC_IsDeviceConnected
 *  @brief      �ж�����Ƿ�������״̬
 *  @param      handle                 [IN]          ���
 *  @return     �豸��������״̬������true��û���ӻ�ʧȥ���ӣ�����false

 *  @fn         MV_CC_IsDeviceConnected
 *  @brief      Is The Device Connected
 *  @param      handle                 [IN]          Handle
 *  @return     Connected, return true. Not Connected or DIsconnected, return
 false
 ***********************************************************************/
MV_CAMCTRL_API bool __stdcall MV_CC_IsDeviceConnected(IN void *handle);

/***********************************************************************
 *  @fn         MV_CC_RegisterImageCallBackEx
 *  @brief      ע��ͼ�����ݻص���chunk
 *  @param       handle                 [IN]          ���
 *  @param       cbOutput               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_RegisterImageCallBackEx
 *  @brief      register image data callback, chunk
 *  @param       handle                 [IN]          Handle
 *  @param       cbOutput               [IN]          Callback function pointer
 *  @param       pUser                  [IN]          User defined variable
 *  @return      Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackEx(
    void *handle,
    void(__stdcall *cbOutput)(unsigned char *pData,
                              MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser),
    void *pUser);

/***********************************************************************
 *  @fn         MV_CC_RegisterImageCallbackForRGB
 *  @brief      ע��ͼ�����ݻص���RGB
 *  @param       handle                 [IN]          ���
 *  @param       cbOutput               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_RegisterImageCallbackForRGB
 *  @brief      register image data callback, RGB
 *  @param       handle                 [IN]          Handle
 *  @param       cbOutput               [IN]          Callback function pointer
 *  @param       pUser                  [IN]          User defined variable
 *  @return      Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackForRGB(
    void *handle,
    void(__stdcall *cbOutput)(unsigned char *pData,
                              MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser),
    void *pUser);

/***********************************************************************
 *  @fn         MV_CC_RegisterImageCallbackForBGR
 *  @brief      ע��ͼ�����ݻص���BGR
 *  @param       handle                 [IN]          ���
 *  @param       cbOutput               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_RegisterImageCallbackForBGR
 *  @brief      register image data callback, BGR
 *  @param       handle                 [IN]          Handle
 *  @param       cbOutput               [IN]          Callback function pointer
 *  @param       pUser                  [IN]          User defined variable
 *  @return      Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackForBGR(
    void *handle,
    void(__stdcall *cbOutput)(unsigned char *pData,
                              MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser),
    void *pUser);

/***********************************************************************
 *  @fn         MV_CC_StartGrabbing
 *  @brief      ��ʼȡ��
 *  @param       handle                 [IN]          ���
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_StartGrabbing
 *  @brief      Start Grabbing
 *  @param      handle                 [IN]          Handle
 *  @return     Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StartGrabbing(IN void *handle);

/***********************************************************************
 *  @fn         MV_CC_StopGrabbing
 *  @brief      ֹͣȡ��
 *  @param       handle                 [IN]          ���
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_StopGrabbing
 *  @brief      Stop Grabbing
 *  @param      handle                 [IN]          Handle
 *  @return     Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StopGrabbing(IN void *handle);

/***********************************************************************
 *  @fn         MV_CC_GetImageForRGB
 *  @brief      ��ȡһ֡RGB���ݣ��˺���Ϊ��ѯʽ��ȡ��ÿ�ε��ò�ѯ�ڲ�
                �����������ݣ���������Χ���ݣ������ݷ��ش�����
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @param       nMsec                  [IN]          �ȴ���ʱʱ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_GetImageForRGB
 *  @brief      Get one frame of RGB data, this function is using query to get
 data,
                query whether the internal cache has data, return data if there
 has, return error code if no data
 *  @param       handle                 [IN]          Handle
 *  @param       pData                  [OUT]         Recevied image data
 pointer
 *  @param       nDataSize              [IN]          Recevied buffer size
 *  @param       pFrameInfo             [OUT]         Image information
 structure
 *  @param       nMsec                  [IN]          Waiting timeout
 *  @return      Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageForRGB(
    IN void *handle, IN OUT unsigned char *pData, IN unsigned int nDataSize,
    IN OUT MV_FRAME_OUT_INFO_EX *pFrameInfo, int nMsec);

/***********************************************************************
 *  @fn         MV_CC_GetImageForBGR
 *  @brief      ��ȡһ֡BGR���ݣ��˺���Ϊ��ѯʽ��ȡ��ÿ�ε��ò�ѯ�ڲ�
                �����������ݣ���������Χ���ݣ������ݷ��ش�����
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @param       nMsec                  [IN]          �ȴ���ʱʱ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_GetImageForBGR
 *  @brief      Get one frame of BGR data, this function is using query to get
 data,
                query whether the internal cache has data, return data if there
 has, return error code if no data
 *  @param       handle                 [IN]          Handle
 *  @param       pData                  [OUT]         Recevied image data
 pointer
 *  @param       nDataSize              [IN]          Recevied buffer size
 *  @param       pFrameInfo             [OUT]         Image information
 structure
 *  @param       nMsec                  [IN]          Waiting timeout
 *  @return      Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageForBGR(
    IN void *handle, IN OUT unsigned char *pData, IN unsigned int nDataSize,
    IN OUT MV_FRAME_OUT_INFO_EX *pFrameInfo, int nMsec);

/***********************************************************************
*  @fn         MV_CC_GetImageBuffer
*  @brief      ���ó�ʱ���ƻ�ȡһ֡ͼƬ��SDK�ڲ��ȴ�ֱ��������ʱ���أ�
�˽ӿڿ�������ȡ��ƽ���ԣ��ʺ����ڶ�ƽ����Ҫ��ϸߵĳ��ϣ���MV_CC_Display����ͬʱʹ�ã�
*  @param       handle                 [IN]          ���
*  @param       pFrame                 [OUT]
ͼ�������׵�ַ��ͼ����Ϣ
*  @param       nMsec                  [IN]
�ȴ���ʱʱ�䣬����INFINITEʱ��ʾ���޵ȴ���ֱ���յ�һ֡���ݻ���ֹͣȡ��
*  @return
�ɹ�������MV_OK�����󣬷��ش�����

*  @fn         MV_CC_GetImageBuffer
*  @brief      Timeout mechanism is used to get image, and the SDK waits inside
until the data is returned,
this interface can increase the flow stability, and is suitable for occasions
with higher stability requirements
*  @param       handle                 [IN]          Handle
*  @param       pFrame                 [OUT]         Pointer of recevied image
data and info
*  @param       nMsec                  [IN]          Waiting timeout
*  @return      Success, return MV_OK. Failure, return error code
***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageBuffer(IN void *handle,
                                                  OUT MV_FRAME_OUT *pFrame,
                                                  IN unsigned int nMsec);

/***********************************************************************
*  @fn         MV_CC_FreeImageBuffer
*  @brief
�ͷ�ͼ�񻺴�(�˽ӿ������ͷŲ���ʹ�õ�ͼ�񻺴棬��MV_CC_GetImageBuffer����ʹ��)
*  @param       handle                 [IN]          ���
*  @param       pFrame                 [IN]          ͼ�����ݺ���Ϣ����ָ��
*  @return
�ɹ�������MV_OK�����󣬷��ش�����

*  @fn         MV_CC_FreeImageBuffer
*  @brief      Free image buffer(this interface can free image buffer, used with
MV_CC_GetImageBuffer)
*  @param       handle                 [IN]          Handle
*  @param       pFrame                 [IN]          Pointer of recevied image
data and info
*  @return      Success, return MV_OK. Failure, return error code
***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FreeImageBuffer(IN void *handle,
                                                   IN MV_FRAME_OUT *pFrame);

/***********************************************************************
 *  @fn         MV_CC_GetOneFrameTimeout
 *  @brief      ���ó�ʱ���ƻ�ȡһ֡ͼƬ��SDK�ڲ��ȴ�ֱ��������ʱ���أ�
                �˽ӿڿ�������ȡ��ƽ���ԣ��ʺ����ڶ�ƽ����Ҫ��ϸߵĳ���
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @param       nMsec                  [IN]          �ȴ���ʱʱ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_GetOneFrameTimeout
 *  @brief      Timeout mechanism is used to get image, and the SDK waits inside
 until the data is returned,
                this interface can increase the flow stability, and is suitable
 for occasions with higher stability requirements
 *  @param       handle                 [IN]          Handle
 *  @param       pData                  [OUT]         Recevied image data
 pointer
 *  @param       nDataSize              [IN]          Recevied buffer size
 *  @param       pFrameInfo             [OUT]         Image information
 structure
 *  @param       nMsec                  [IN]          Waiting timeout
 *  @return      Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrameTimeout(
    IN void *handle, IN OUT unsigned char *pData, IN unsigned int nDataSize,
    IN OUT MV_FRAME_OUT_INFO_EX *pFrameInfo, unsigned int nMsec);

/***********************************************************************
 *  @fn         MV_CC_Display
 *  @brief      ��ʾͼ��ע����ʾ���ڣ��ڲ��Զ���ʾ����MV_CC_GetImageBuffer����ͬʱʹ�ã�
 *  @param       handle                 [IN]          ���
 *  @param       hWnd                   [IN]          ��ʾ���ھ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_Display
 *  @brief      Display one frame image, register display window, automatic
 display internally
 *  @param      handle                 [IN]          Handle
 *  @param      hWnd                   [IN]          Display Window Handle
 *  @return     Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_Display(IN void *handle, void *hWnd);

/***********************************************************************
 *  @fn         MV_CC_DisplayOneFrame
 *  @brief      ��ʾһ֡ͼ��
 *  @param       handle                 [IN]          ���
 *  @param       pDisplayInfo           [IN]          ͼ����Ϣ
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_DisplayOneFrame
 *  @brief      Display one frame image
 *  @param      handle                 [IN]          Handle
 *  @param      pDisplayInfo           [IN]          Frame Info
 *  @return     Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DisplayOneFrame(
    IN void *handle, IN MV_DISPLAY_FRAME_INFO *pDisplayInfo);

/***********************************************************************
 *  @fn         MV_CC_SetImageNodeNum
 *  @brief      ����SDK�ڲ�ͼ�񻺴�ڵ��������Χ[1,
 30]����ץͼǰ����
 *  @param       handle                 [IN]          ���
 *  @param       nNum                   [IN]          ����ڵ����
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_SetImageNodeNum
 *  @brief      Set the number of the internal image cache nodes in SDK, in the
 range [1, 30], to be called before the capture
 *  @param      handle                 [IN]          Handle
 *  @param      nNum                   [IN]          Image Node Number
 *  @return     Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetImageNodeNum(IN void *handle,
                                                   unsigned int nNum);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetImageInfo(IN void* handle, IN
 OUT MV_IMAGE_BASIC_INFO* pstInfo);
 *  @brief  ��ȡͼ�������Ϣ
 *  @param  void* handle                     [IN]        ������
 *  @param  MV_IMAGE_BASIC_INFO* pstInfo     [IN][OUT]
 ���ظ��������й����ͼ�������Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ο� CameraParam.h �е� MV_IMAGE_BASIC_INFO ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetImageInfo(IN void* handle, IN
 OUT MV_IMAGE_BASIC_INFO* pstInfo);
 *  @brief  Get basic information of image
 *  @param  void* handle                     [IN]        Handle
 *  @param  MV_IMAGE_BASIC_INFO* pstInfo     [IN][OUT]   Structure pointer of
 image basic information
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to the definition of MV_IMAGE_BASIC_INFO in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageInfo(
    IN void *handle, IN OUT MV_IMAGE_BASIC_INFO *pstInfo);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceInfo(IN void* handle, IN
 OUT MV_CC_DEVICE_INFO* pstDevInfo);
 *  @brief  ��ȡ�豸��Ϣ
 *  @param  void* handle                     [IN]        ������
 *  @param  MV_CC_DEVICE_INFO* pstDevInfo    [IN][OUT]
 ���ظ��������й�����豸��Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ο� CameraParam.h �е� MV_CC_DEVICE_INFO ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceInfo(IN void* handle, IN
 OUT MV_CC_DEVICE_INFO* pstDevInfo);
 *  @brief  Get device information
 *  @param  void* handle                     [IN]        Handle
 *  @param  MV_CC_DEVICE_INFO* pstDevInfo    [IN][OUT]   Structure pointer of
 device information
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to the definition of MV_CC_DEVICE_INFO in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceInfo(
    IN void *handle, IN OUT MV_CC_DEVICE_INFO *pstDevInfo);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAllMatchInfo(IN void* handle,
 IN OUT MV_ALL_MATCH_INFO* pstInfo);
 *  @brief  ��ȡ�������͵���Ϣ
 *  @param  void* handle                     [IN]        ������
 *  @param  MV_ALL_MATCH_INFO* pstInfo       [IN][OUT]
 ���ظ��������й�����������͵���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ο� CameraParam.h �е� MV_ALL_MATCH_INFO ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAllMatchInfo(IN void* handle,
 IN OUT MV_ALL_MATCH_INFO* pstInfo);
 *  @brief  Get various type of information
 *  @param  void* handle                     [IN]        Handle
 *  @param  MV_ALL_MATCH_INFO* pstInfo       [IN][OUT]   Structure pointer of
 various type of information
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to the definition of MV_ALL_MATCH_INFO in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAllMatchInfo(
    IN void *handle, IN OUT MV_ALL_MATCH_INFO *pstInfo);

/************************************************************************/
/* ���úͻ�ȡ������������ܽӿ�                                 		*/
/* General interface for getting and setting camera parameters          */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetIntValue(IN void* handle,
                                                           IN const char*
 strKey,
                                                           OUT MVCC_INTVALUE
 *pIntValue);
 *  @brief  ��ȡInteger����ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ�����ȡ������Ϣ��Ϊ"Width"
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�������Խṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetIntValue(IN void* handle,
                                                           IN const char*
 strKey,
                                                           OUT MVCC_INTVALUE
 *pIntValue);
 *  @brief  Get Integer value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value, for example,
 using "Width" to get width
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of camera
 features
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetIntValue(IN void *handle,
                                               IN const char *strKey,
                                               OUT MVCC_INTVALUE *pIntValue);

/************************************************************************/
/* ���úͻ�ȡ������������ܽӿ�                                 		*/
/* General interface for getting and setting camera parameters          */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetIntValueEx(IN void* handle,
                                                           IN const char*
 strKey,
                                                           OUT MVCC_INTVALUE
 *pIntValue);
 *  @brief  ��ȡInteger����ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ�����ȡ������Ϣ��Ϊ"Width"
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�������Խṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetIntValueEx(IN void* handle,
                                                           IN const char*
 strKey,
                                                           OUT MVCC_INTVALUE
 *pIntValue);
 *  @brief  Get Integer value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value, for example,
 using "Width" to get width
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of camera
 features
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetIntValueEx(
    IN void *handle, IN const char *strKey, OUT MVCC_INTVALUE_EX *pIntValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetIntValue(IN void* handle,
                                                           IN const char*
 strKey,
                                                           IN unsigned int
 nValue);
 *  @brief  ����Integer������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ�����ȡ������Ϣ��Ϊ"Width"
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetIntValue(IN void* handle,
                                                           IN const char*
 strKey,
                                                           IN unsigned int
 nValue);
 *  @brief  Set Integer value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value, for example,
 using "Width" to set width
 *          const unsigned int nValue   [IN]        Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetIntValue(IN void *handle,
                                               IN const char *strKey,
                                               IN unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetIntValueEx(IN void* handle,
                                                           IN const char*
 strKey,
                                                           IN int64_t nValue);
 *  @brief  ����Integer������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ�����ȡ������Ϣ��Ϊ"Width"
 *          const int64_t nValue   [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetIntValueEx(IN void* handle,
                                                           IN const char*
 strKey,
                                                           IN int64_t nValue);
 *  @brief  Set Integer value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value, for example,
 using "Width" to set width
 *          const int64_t nValue   [IN]        Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetIntValueEx(IN void *handle,
                                                 IN const char *strKey,
                                                 IN int64_t nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValue(IN void* handle,
                                                            IN const char*
 strKey,
                                                            OUT MVCC_ENUMVALUE
 *pEnumValue);
 *  @brief  ��ȡEnum����ֵ
 *  @param  void* handle                   [IN]        ������
 *  @param  char* strKey                   [IN]
 ���Լ�ֵ�����ȡ���ظ�ʽ��Ϣ��Ϊ"PixelFormat"
 *  @param  MVCC_ENUMVALUE* pEnumValue     [IN][OUT] ���ظ��������й�������Խṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValue(IN void* handle,
                                                            IN const char*
 strKey,
                                                            OUT MVCC_ENUMVALUE
 *pEnumValue);
 *  @brief  Get Enum value
 *  @param  void* handle                   [IN]        Handle
 *  @param  char* strKey                   [IN]        Key value, for example,
 using "PixelFormat" to get pixel format
 *  @param  MVCC_ENUMVALUE* pEnumValue     [IN][OUT]   Structure pointer of
 camera features
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValue(IN void *handle,
                                                IN const char *strKey,
                                                OUT MVCC_ENUMVALUE *pEnumValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValue(IN void* handle,
                                                            IN const char*
 strKey,
                                                            IN unsigned int
 nValue);
 *  @brief  ����Enum������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]
 ���Լ�ֵ�����ȡ���ظ�ʽ��Ϣ��Ϊ"PixelFormat"
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValue(IN void* handle,
                                                            IN const char*
 strKey,
                                                            IN unsigned int
 nValue);
 *  @brief  Set Enum value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value, for example,
 using "PixelFormat" to set pixel format
 *          const unsigned int nValue   [IN]        Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValue(IN void *handle,
                                                IN const char *strKey,
                                                IN unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValueByString(IN void*
 handle,
                                                            IN const char*
 strKey,
                                                            IN const char*
 sValue);
 *  @brief  ����Enum������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]
 ���Լ�ֵ�����ȡ���ظ�ʽ��Ϣ��Ϊ"PixelFormat"
 *          char* sValue                [IN]        ��Ҫ���õ�����������ַ���
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValue(IN void* handle,
                                                            IN const char*
 strKey,
                                                            IN const char*
 sValue);
 *  @brief  Set Enum value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value, for example,
 using "PixelFormat" to set pixel format
 *          char* sValue                [IN]        Feature String to set
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValueByString(IN void *handle,
                                                        IN const char *strKey,
                                                        IN const char *sValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFloatValue(IN void* handle,
                                                             IN const char*
 strKey,
                                                             OUT MVCC_FLOATVALUE
 *pFloatValue);
 *  @brief  ��ȡFloat����ֵ
 *  @param  void* handle                     [IN]        ������
 *  @param  char* strKey                     [IN]        ���Լ�ֵ
 *  @param  MVCC_FLOATVALUE *pFloatValue     [IN][OUT]
 ���ظ��������й�������Խṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFloatValue(IN void* handle,
                                                             IN const char*
 strKey,
                                                             OUT MVCC_FLOATVALUE
 *pFloatValue);
 *  @brief  Get Float value
 *  @param  void* handle                     [IN]        Handle
 *  @param  char* strKey                     [IN]        Key value
 *  @param  MVCC_FLOATVALUE *pFloatValue     [IN][OUT]   Structure pointer of
 camera features
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFloatValue(
    IN void *handle, IN const char *strKey, OUT MVCC_FLOATVALUE *pFloatValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFloatValue(IN void* handle,
                                                             IN const char*
 strKey,
                                                             IN float fValue);
 *  @brief  ����float������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ
 *          float fValue                [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFloatValue(IN void* handle,
                                                             IN const char*
 strKey,
                                                             IN float fValue);
 *  @brief  Set float value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value
 *          float fValue                [IN]        Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFloatValue(IN void *handle,
                                                 IN const char *strKey,
                                                 IN float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBoolValue(IN void* handle,
                                                            IN const char*
 strKey,
                                                            OUT bool
 *pBoolValue);
 *  @brief  ��ȡBoolean����ֵ
 *  @param  void* handle                     [IN]        ������
 *  @param  char* strKey                     [IN]        ���Լ�ֵ
 *  @param  bool *pBoolValue                 [IN][OUT]   ���ظ��������й��������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBoolValue(IN void* handle,
                                                            IN const char*
 strKey,
                                                            OUT bool
 *pBoolValue);
 *  @brief  Get Boolean value
 *  @param  void* handle                     [IN]        Handle
 *  @param  char* strKey                     [IN]        Key value
 *  @param  bool *pBoolValue                 [IN][OUT]   Structure pointer of
 camera features
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBoolValue(IN void *handle,
                                                IN const char *strKey,
                                                OUT bool *pBoolValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBoolValue(IN void* handle,
                                                            IN const char*
 strKey,
                                                            IN bool bValue);
 *  @brief  ����Boolean������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ
 *          bool bValue                 [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBoolValue(IN void* handle,
                                                            IN const char*
 strKey,
                                                            IN bool bValue);
 *  @brief  Set Boolean value
 *  @param  void* handle                [IN]        Handle
 *  @param  char* strKey                [IN]        Key value
 *          bool bValue                 [IN]        Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBoolValue(IN void *handle,
                                                IN const char *strKey,
                                                IN bool bValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetStringValue(IN void* handle,
                                                              IN const char*
 strKey,
                                                              OUT
 MVCC_STRINGVALUE *pStringValue);
 *  @brief  ��ȡString����ֵ
 *  @param  void* handle                       [IN]        ������
 *  @param  char* strKey                       [IN]        ���Լ�ֵ
 *  @param  MVCC_STRINGVALUE *pStringValue     [IN][OUT]
 ���ظ��������й�������Խṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetStringValue(IN void* handle,
                                                              IN const char*
 strKey,
                                                              OUT
 MVCC_STRINGVALUE *pStringValue);
 *  @brief  Get String value
 *  @param  void* handle                       [IN]        Handle
 *  @param  char* strKey                       [IN]        Key value
 *  @param  MVCC_STRINGVALUE *pStringValue     [IN][OUT]   Structure pointer of
 camera features
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetStringValue(
    IN void *handle, IN const char *strKey, OUT MVCC_STRINGVALUE *pStringValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetStringValue(IN void* handle,
                                                              IN const char*
 strKey,
                                                              IN const char *
 sValue);
 *  @brief  ����String������ֵ
 *  @param  void* handle                  [IN]        ������
 *  @param  char* strKey                  [IN]        ���Լ�ֵ
 *          char * sValue                 [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetStringValue(IN void* handle,
                                                              IN const char*
 strKey,
                                                              IN const char *
 sValue);
 *  @brief  Set String value
 *  @param  void* handle                  [IN]        Handle
 *  @param  char* strKey                  [IN]        Key value
 *          char * sValue                 [IN]        Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetStringValue(IN void *handle,
                                                  IN const char *strKey,
                                                  IN const char *sValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetCommandValue(IN void* handle,
                                                               IN const char*
 strKey);
 *  @brief  ����Command������ֵ
 *  @param  void* handle                  [IN]        ������
 *  @param  char* strKey                  [IN]        ���Լ�ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetCommandValue(IN void* handle,
                                                               IN const char*
 strKey);
 *  @brief  Set Command value
 *  @param  void* handle                  [IN]        Handle
 *  @param  char* strKey                  [IN]        Key value
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetCommandValue(IN void *handle,
                                                   IN const char *strKey);

/************************************************************************
 *  @fn     MV_CC_InvalidateNodes
 *  @brief  ���GenICam�ڵ㻺��
 *  @param  handle                  [IN]           �����ַ
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_InvalidateNodes
 *  @brief  Invalidate GenICam Nodes
 *  @param  handle                  [IN]           Handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_InvalidateNodes(IN void *handle);

/************************************************************************/
/* ���������ȡ�����ã���ģ������нӿڣ����𲽷�������������������ܽӿڴ���
 */
/* Get and set camara parameters, all interfaces of this module will be replaced
 * by general interface*/
/************************************************************************/
/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetWidth(IN void* handle, IN OUT
 MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡͼ�����
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]
 ���ظ��������й�������ȵ���Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *                  unsigned int    nCurValue;      // ���������ǰ�Ŀ���ֵ
 *                  unsigned int    nMax;           // ��ʾ����������������õĿ���ֵ
 *                  unsigned int    nMin;           // ��ʾ�����������С�����õĿ���ֵ
 *                  unsigned int    nInc;           //
 ��ʾ������õĿ�������������nInc�ı�����������Ч
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
 *          �������ͽṹ������Ľӿڿɲ��մ˽ӿ�

 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetWidth(IN void* handle, IN OUT
 MVCC_INTVALUE* pstValue);
 *  @brief  Get image width
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_INTVALUE* pstValue     [IN][OUT]   Returns the information
 structure pointer about the camera's width for the caller
 *          The meaning of returns pstValue structure
 *                  unsigned int    nCurValue;      // Represents the current
 width value of the camera
 *                  unsigned int    nMax;           // Indicates the maximum
 settable width value allowed by the camera
 *                  unsigned int    nMin;           // Indicates the minimum
 settable width value allowed by the camera
 *                  unsigned int    nInc;           // Indicates that the width
 increment set by the camera must be a multiple of nInc, otherwise it is invalid
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
 *          Other Integer structure parameters interface can refer to this
 interface
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetWidth(IN void *handle,
                                            IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
*  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetWidth(IN void* handle, IN const
unsigned int nValue);
*  @brief  ����ͼ�����
*  @param  void* handle                [IN]        ������
*          const unsigned int nValue   [IN]
��Ҫ���õ�������ȵ�ֵ,ע��˿���ֵ������MV_CC_GetWidth�ӿڷ��ص�pstValue�е�nInc�ı����������óɹ�
*  @return �ɹ�,����MV_OK,����������Ƚ������Ϊ��Ӧֵ��ʧ��,���ش�����

* @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetWidth(IN void* handle, IN const
unsigned int nValue);
 *  @brief  Set image width
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        To set the value of the
camera width, note that the width value must be a multiple of nInc in the
pstValue returned by the MV_CC_GetWidth interface
 *  @return Success, return MV_OK, and the camera width will change to the
corresponding value. Failure, return error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetWidth(IN void *handle,
                                            IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeight(IN void* handle, IN OUT
 MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡͼ��߶�
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT] ���ظ��������й�����߶ȵ���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�����߶���Ϣ���ص��ṹ���У�ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeight(IN void* handle, IN OUT
 MVCC_INTVALUE* pstValue);
 *  @brief  Get image height
 *  @param  void* handle                [IN]        Camera handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of
 information structure related to camera height to user
 *  @return Success, return MV_OK, and return height information to the
 structure. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetHeight(IN void *handle,
                                             IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeight(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  ����ͼ��߶�
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]
 ��Ҫ���õ�������ȵ�ֵ,ע��˿���ֵ������MV_CC_GetWidth�ӿڷ��ص�pstValue�е�nInc�ı����������óɹ�
 *  @return �ɹ�,����MV_OK,��������߶Ƚ������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeight(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  Set image height
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Camera height value to set,
 note that this value must be times of nInc of pstValue returned by
 MV_CC_GetWidth
 *  @return Success, return MV_OK, and the camera height will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetHeight(IN void *handle,
                                             IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetX(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡͼ��Xƫ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]
 ���ظ��������й����Xƫ�Ƶ���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetX(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get image X offset
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of
 information structure related to camera X offset to user
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetX(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  ����ͼ��AOIƫ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����AOI��ֵ
 *  @return �ɹ�,����MV_OK,�������AOIƫ�ƽ������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  Set image X offset
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Camera X offset value to set
 *  @return Success, return MV_OK, and the camera X offset will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void *handle,
                                                 IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetY(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡͼ��Yƫ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]
 ���ظ��������й����Yƫ�Ƶ���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetY(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get image Y offset
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of
 information structure related to camera Y offset to user
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetY(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  ����ͼ��AOIƫ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����AOI��ֵ
 *  @return �ɹ�,����MV_OK,�������AOIƫ�ƽ������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetY(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  Set image Y offset
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Camera Y offset value to set
 *  @return Success, return MV_OK, and the camera Y offset will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetY(IN void *handle,
                                                 IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeLower(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ�ع�����
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]
 ���ظ��������й�����ع�ֵ���޽ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeLower(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get exposure lower limit
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of
 information structure related to camera exposure lower to user
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeLower(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeLower(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  �����ع�ֵ����
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ��ع�ֵ����
 *  @return
 �ɹ�,����MV_OK,��������ع����޽������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeLower(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set exposure lower limit
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Exposure lower to set
 *  @return Success, return MV_OK, and the camera exposure time lower limit
 value will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeLower(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeUpper(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ�ع�����
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]
 ���ظ��������й�����ع�ֵ���޽ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeUpper(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get exposure upper limit
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of
 information structure related to camera exposure upper to user
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeUpper(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeUpper(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  �����ع�ֵ����
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ��ع�ֵ����
 *  @return
 �ɹ�,����MV_OK,��������ع����޽������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeUpper(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set exposure upper limit
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Exposure upper to set
 *  @return Success, return MV_OK, and the camera exposure time upper limit
 value will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeUpper(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBrightness(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ����ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�������Ƚṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBrightness(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get brightness
 *  @param  void* handle                [IN]        Camera Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Return pointer of
 information structure related to camera brightness to user
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBrightness(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBrightness(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  ��������ֵ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ�����ֵ
 *  @return �ɹ�,����MV_OK,����������Ƚ������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBrightness(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  Set brightness
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        Brightness upper to set
 *  @return Success, return MV_OK, and the camera brightness value will change
 to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBrightness(IN void *handle,
                                                 IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameRate(IN void* handle, IN
 OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡ֡��
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT] ���ظ��������й����֡�ʵ���Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *                                      float           fCurValue;      //
 ��ʾ�����ǰ��֡��
 *                                      float           fMax;           //
 ��ʾ����������õ����֡��
 *                                      float           fMin;           //
 ��ʾ����������õ���С֡��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
 *          ���������ͽṹ������Ľӿڿɲ��մ˽ӿ�

 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameRate(IN void* handle, IN
 OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  Get Frame Rate
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Return pointer of
 information structure related to camera frame rate to user
 *          The meaning of returns pstValue structure
 *                                      float           fCurValue;      //
 Indicates the current frame rate of the camera
 *                                      float           fMax;           //
 Indicates the maximum frame rate allowed by the camera
 *                                      float           fMin;           //
 Indicates the minimum frame rate allowed by the camera
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
 *          Other interface of Float structure parameters can refer to this
 interface
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFrameRate(
    IN void *handle, IN OUT MVCC_FLOATVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameRate(IN void* handle, IN
 const float fValue);
 *  @brief  ����֡��
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����֡��
 *  @return �ɹ�,����MV_OK,�������֡�ʽ������Ϊ��Ӧֵ��ʧ��,���ش�����

 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameRate(IN void* handle, IN
 const float fValue);
 *  @brief  Set frame rate
 *  @param  void* handle                [IN]        Camera Handle
 *          const float fValue          [IN]        Camera frame rate to set
 *  @return Success, return MV_OK, and camera frame rate will be changed to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFrameRate(IN void *handle,
                                                IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGain(IN void* handle, IN OUT
 MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡ����
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]
 ���ظ��������й�����������Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *                                      float           fCurValue;      //
 ��ʾ�����ǰ��֡��
 *                                      float           fMax;           //
 ��ʾ����������õ����֡��
 *                                      float           fMin;           //
 ��ʾ����������õ���С֡��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
 *          ���������ͽṹ������Ľӿڿɲ��մ˽ӿ�

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGain(IN void* handle, IN OUT
 MVCC_FLOATVALUE* pstValue);
 *  @brief  Get Gain
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Return pointer of
 information structure related to gain to user
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *                                      float           fCurValue;      //
 Camera current gain
 *                                      float           fMax;           // The
 maximum gain camera allowed
 *                                      float           fMin;           // The
 minimum gain camera allowed
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
 *          Other interface of Float structure parameters can refer to this
 interface
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGain(IN void *handle,
                                           IN OUT MVCC_FLOATVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGain(IN void* handle, IN const
 float fValue);
 *  @brief  ����֡��
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����֡��
 *  @return �ɹ�,����MV_OK,�������֡�ʽ������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGain(IN void* handle, IN const
 float fValue);
 *  @brief  Set Gain
 *  @param  void* handle                [IN]        Camera Handle
 *          const float fValue          [IN]        Gain value to set
 *  @return Success, return MV_OK, and the camera gain value will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGain(IN void *handle,
                                           IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureTime(IN void* handle,
 IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡ�ع�ʱ��
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]
 ���ظ��������й�����ع�ʱ�����Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *                                      float           fCurValue;      //
 ��ʾ�����ǰ��֡��
 *                                      float           fMax;           //
 ��ʾ����������õ����֡��
 *                                      float           fMin;           //
 ��ʾ����������õ���С֡��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
 *          ���������ͽṹ������Ľӿڿɲ��մ˽ӿ�

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureTime(IN void* handle,
 IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  Get exposure time
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Return pointer of
 information structure related to exposure time to user
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *                                      float           fCurValue;      //
 Camera current exposure time
 *                                      float           fMax;           // The
 maximum exposure time camera allowed
 *                                      float           fMin;           // The
 minimum exposure time camera allowed
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
 *          Other interface of Float structure parameters can refer to this
 interface
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetExposureTime(
    IN void *handle, IN OUT MVCC_FLOATVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureTime(IN void* handle,
 IN const float fValue);
 *  @brief  �����ع�ʱ��
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����֡��
 *  @return �ɹ�,����MV_OK,�������֡�ʽ������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureTime(IN void* handle,
 IN const float fValue);
 *  @brief  Set exposure time
 *  @param  void* handle                [IN]        Camera Handle
 *          const float fValue          [IN]        Exposure time to set
 *  @return Success, return MV_OK, and the camera exposure time value will
 change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetExposureTime(IN void *handle,
                                                   IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetPixelFormat(IN void* handle,
 IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ���ظ�ʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]
 ���ظ������ߵ��й����ظ�ʽ����Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *          unsigned int    nCurValue;                              //
 �����ǰ�����ظ�ʽ����ö������,����˵PixelType_Gvsp_Mono8,
 �����õ���������ֵ,������ֵ����PixelType.h��MvGvspPixelTypeö������
 *          unsigned int    nSupportedNum;                          //
 ���֧�ֵ����ظ�ʽ�ĸ���
 *          unsigned int    nSupportValue[MV_MAX_XML_SYMBOLIC_NUM]; //
 �������֧�ֵ����ظ�ʽ��Ӧ������ֵ�б�������Ҫ�������ظ�ʽʱ��������������������е�һ�֣�������Ч
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
            ����ö�����Ͳ����ӿڿɲ��մ˽ӿڣ��й���Ӧ������ö�����Ͷ�Ӧ������ֵ�����PixelType.h
 �� CameraParams.h����Ӧ�Ķ���

 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetPixelFormat(IN void* handle, IN
 OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get Pixel Format
 *  @param  void* handle                [IN]        Camera Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Returns the information
 structure pointer about pixel format for the caller
 *          The meaning of returns pstValue structure
 *          unsigned int    nCurValue;                              //  The
 current pixel format of the camera, is the enumeration type, such as
 PixelType_Gvsp_Mono8, here is the integer value, the specific value please
 refer to MvGvspPixelType enumeration type in PixelType.h
 *          unsigned int    nSupportedNum;                          //  Number
 of pixel formats supported by the camera
 *          unsigned int    nSupportValue[MV_MAX_XML_SYMBOLIC_NUM]; //  The
 integer values list correspond to all supported pixel formats of the camera,
 followed by when set the pixel format, the parameter must be one of this list,
 otherwise invalid
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
            Other interface of Enumeration structure parameters can refer to
 this interface, look for the corresponding definition in PixelType.h and
 CameraParams.h for the integer values of the enum type parameter
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetPixelFormat(
    IN void *handle, IN OUT MVCC_ENUMVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetPixelFormat(IN void* handle,
 IN const unsigned int nValue);
 *  @brief  �������ظ�ʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]
 Ҫ���õ����ظ�ʽ��Ӧ������ֵ�����ô˽ӿ�ʱ����ֱ����дö��ֵ����MV_CC_SetPixelFormat(m_handle,
 PixelType_Gvsp_RGB8_Packed);
 *  @return �ɹ�,����MV_OK,����������ظ�ʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 *
 *
 Ҫ���õ�ö�����ͱ�����Get�ӿڷ��ص�nSupportValue[MV_MAX_XML_SYMBOLIC_NUM]�е�һ�֣������ʧ��

 * @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetPixelFormat(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  Set Pixel Format
 *  @param  void* handle                [IN]        Camera Handle
 *          const unsigned int nValue   [IN]        The corresponding integer
 value for pixel format to be set, when calling this interface can be directly
 filled in enumeration values, such as MV_CC_SetPixelFormat(m_handle,
 PixelType_Gvsp_RGB8_Packed);
 *  @return Success, return MV_OK, and the camera pixel format will change to
 the corresponding value. Failure, return error code
 *
 *          Other interface of Enumeration structure parameters can refer to
 this interface, the enumeration type to be set must be one of the nSupportValue
 [MV_MAX_XML_SYMBOLIC_NUM] returned by the Get interface, otherwise it will fail
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetPixelFormat(IN void *handle,
                                                  IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionMode(IN void*
 handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ�ɼ�ģʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT] ���ظ������ߵ��йزɼ�ģʽ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е�
 MV_CAM_ACQUISITION_MODE ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionMode(IN void*
 handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get acquisition mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of
 acquisition mode
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
            Refer to MV_CC_GetPixelFormat and definition of
 MV_CAM_ACQUISITION_MODE in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionMode(
    IN void *handle, IN OUT MVCC_ENUMVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionMode(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  �������ظ�ʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õĲɼ�ģʽ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,��������ɼ�ģʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionMode(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set acquisition mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set
 corresponding to acquisition mode
 *  @return Success, return MV_OK, and the camera acquisition mode will change
 to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionMode(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGainMode(IN void* handle, IN
 OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ����ģʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]
 ���ظ������ߵ��й�����ģʽ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е� MV_CAM_GAIN_MODE
 ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGainMode(IN void* handle, IN
 OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get gain mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]    Structure pointer of gain
 mode
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
            Refer to MV_CC_GetPixelFormat and definition of MV_CAM_GAIN_MODE in
 CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGainMode(IN void *handle,
                                               IN OUT MVCC_ENUMVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGainMode(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  ��������ģʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õ�����ģʽ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,�����������ģʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGainMode(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  Set gain mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set
 corresponding to gain mode
 *  @return Success, return MV_OK, and the camera gain mode will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGainMode(IN void *handle,
                                               IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureAutoMode(IN void*
 handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ�Զ��ع�ģʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]
 ���ظ������ߵ��й��Զ��ع�ģʽ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е�
 MV_CAM_EXPOSURE_AUTO_MODE ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureAutoMode(IN void*
 handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get auto exposure mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of auto
 exposure mode
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
            Refer to MV_CC_GetPixelFormat and definition of
 MV_CAM_EXPOSURE_AUTO_MODE in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetExposureAutoMode(
    IN void *handle, IN OUT MVCC_ENUMVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureAutoMode(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  �����Զ��ع�ģʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õ��Զ��ع�ģʽ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,��������Զ��ع�ģʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureAutoMode(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set auto exposure mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set
 corresponding to auto exposure mode
 *  @return Success, return MV_OK, and the camera auto exposure mode will change
 to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetExposureAutoMode(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerMode(IN void* handle,
 IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ����ģʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]
 ���ظ������ߵ��йش���ģʽ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е�
 MV_CAM_TRIGGER_MODE ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerMode(IN void* handle,
 IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get trigger mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of trigger
 mode
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
            Refer to MV_CC_GetPixelFormat and definition of MV_CAM_TRIGGER_MODE
 in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerMode(
    IN void *handle, IN OUT MVCC_ENUMVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerMode(IN void* handle,
 IN const unsigned int nValue);
 *  @brief  ���ô���ģʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õĴ���ģʽ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,�����������ģʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerMode(IN void* handle,
 IN const unsigned int nValue);
 *  @brief  Set trigger mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set
 corresponding to trigger mode
 *  @return Success, return MV_OK, and the camera trigger mode will change to
 the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerMode(IN void *handle,
                                                  IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerDelay(IN void* handle,
 IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡ������ʱ
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]
 ���ظ��������й����������ʱ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetFrameRate

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerDelay(IN void* handle,
 IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  Get tigger delay
 *  @param  void* handle                [IN]        Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Structure pointer of trigger
 delay
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
 *          Refer to MV_CC_GetFrameRate
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerDelay(
    IN void *handle, IN OUT MVCC_FLOATVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerDelay(IN void* handle,
 IN const float fValue);
 *  @brief  ���ô�����ʱ
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����������ʱ
 *  @return �ɹ�,����MV_OK,�������������ʱ�������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerDelay(IN void* handle,
 IN const float fValue);
 *  @brief  Set tigger delay
 *  @param  void* handle                [IN]        Handle
 *          const float fValue          [IN]        Trigger delay to set
 *  @return Success, return MV_OK, and the camera trigger delay will change to
 the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerDelay(IN void *handle,
                                                   IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerSource(IN void* handle,
 IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ����Դ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT] ���ظ������ߵ��йش���Դ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е�
 MV_CAM_TRIGGER_SOURCE ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerSource(IN void* handle,
 IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get trigger source
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of trigger
 source
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
            Refer to MV_CC_GetPixelFormat and definition of
 MV_CAM_TRIGGER_SOURCE in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerSource(
    IN void *handle, IN OUT MVCC_ENUMVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerSource(IN void* handle,
 IN const unsigned int nValue);
 *  @brief  ���ô���Դ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õĴ���Դ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,�����������Դ�������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerSource(IN void* handle,
 IN const unsigned int nValue);
 *  @brief  Set trigger source
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set
 corresponding to trigger source
 *  @return Success, return MV_OK, and the camera trigger source will change to
 the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerSource(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_TriggerSoftwareExecute(IN void*
 handle);
 *  @brief  ������һ�Σ��ӿڽ�����ѡ��Ĵ���ԴΪ��������ʱ��Ч��
 *  @param  void* handle                [IN]        ������
 *  @return �ɹ�,����MV_OK, ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_TriggerSoftwareExecute(IN void*
 handle);
 *  @brief  Execute software trigger once (this interface only valid when the
 trigger source is set to software)
 *  @param  void* handle                [IN]        Handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_TriggerSoftwareExecute(IN void *handle);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGammaSelector(IN void* handle,
 IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡGamma����
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]
 ���ظ������ߵ��й�Gamma���͵���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е�
 MV_CAM_GAMMA_SELECTOR ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGammaSelector(IN void* handle,
 IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get Gamma mode
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of gamma
 mode
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
            Refer to MV_CC_GetPixelFormat and definition of
 MV_CAM_GAMMA_SELECTOR in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGammaSelector(
    IN void *handle, IN OUT MVCC_ENUMVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGammaSelector(IN void* handle,
 IN const unsigned int nValue);
 *  @brief  ����Gamma����
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õ�Gamma���Ͷ�Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,�������Gamma���ͽ������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGammaSelector(IN void* handle,
 IN const unsigned int nValue);
 *  @brief  Set Gamma mode
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set
 corresponding to gamma mode
 *  @return Success, return MV_OK, and the camera gamma mode will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGammaSelector(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGamma(IN void* handle, IN OUT
 MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡGammaֵ
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]
 ���ظ��������й����Gammaֵ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetFrameRate

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGamma(IN void* handle, IN OUT
 MVCC_FLOATVALUE* pstValue);
 *  @brief  Get Gamma value
 *  @param  void* handle                [IN]        Handle
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   Structure pointer of gamma
 value
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
 *          Refer to MV_CC_GetFrameRate
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGamma(IN void *handle,
                                            IN OUT MVCC_FLOATVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGamma(IN void* handle, IN
 const float fValue);
 *  @brief  ����Gammaֵ
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����Gammaֵ
 *  @return �ɹ�,����MV_OK,�������Gammaֵ�������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGamma(IN void* handle, IN
 const float fValue);
 *  @brief  Set Gamma value
 *  @param  void* handle                [IN]        Handle
 *          const float fValue          [IN]        Gamma value to set
 *  @return Success, return MV_OK, and the camera gamma value will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGamma(IN void *handle,
                                            IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSharpness(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ���
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������Ƚṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSharpness(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get sharpness
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of
 sharpness
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetSharpness(IN void *handle,
                                                IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSharpness(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  �������
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����
 *  @return �ɹ�,����MV_OK,���������Ƚ������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSharpness(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  Set sharpness
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Sharpness to set
 *  @return Success, return MV_OK, and the camera sharpness will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetSharpness(IN void *handle,
                                                IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHue(IN void* handle, IN OUT
 MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ�Ҷ�
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�����ҶȽṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHue(IN void* handle, IN OUT
 MVCC_INTVALUE* pstValue);
 *  @brief  Get Hue
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of Hue
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetHue(IN void *handle,
                                          IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHue(IN void* handle, IN const
 unsigned int nValue);
 *  @brief  ���ûҶ�
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õĻҶ�
 *  @return �ɹ�,����MV_OK,��������ҶȽ������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHue(IN void* handle, IN const
 unsigned int nValue);
 *  @brief  Set Hue
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Hue to set
 *  @return Success, return MV_OK, and the camera Hue will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetHue(IN void *handle,
                                          IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSaturation(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ���Ͷ�
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�������ͶȽṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

  *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSaturation(IN void* handle,
 IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get Saturation
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of
 Saturation
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetSaturation(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSaturation(IN void* handle, IN
const unsigned int nValue);
 *  @brief  ���ñ��Ͷ�
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õı��Ͷ�
 *  @return �ɹ�,����MV_OK,����������ͶȽ������Ϊ��Ӧֵ��ʧ��,���ش�����

*  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSaturation(IN void* handle, IN
const unsigned int nValue);
 *  @brief  Set Saturation
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Saturation to set
 *  @return Success, return MV_OK, and the camera Saturation will change to the
corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetSaturation(IN void *handle,
                                                 IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceWhiteAuto(IN void*
 handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ�Զ���ƽ��
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]
 ���ظ������ߵ��й��Զ���ƽ�����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е�
 MV_CAM_BALANCEWHITE_AUTO ����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceWhiteAuto(IN void*
 handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  Get Auto white balance
 *  @param  void* handle                [IN]        Handle
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   Structure pointer of auto
 white balance
 *  @return Success, return MV_OK, and get the structure of the corresponding
 parameters. Failure, return error code
 *
            Refer to MV_CC_GetPixelFormat and definition of
 MV_CAM_BALANCEWHITE_AUTO in CameraParam.h
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceWhiteAuto(
    IN void *handle, IN OUT MVCC_ENUMVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceWhiteAuto(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  �����Զ���ƽ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õ��Զ���ƽ���Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,��������Զ���ƽ�⽫�����Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceWhiteAuto(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set Auto white balance
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Integer value to set
 corresponding to auto white balance
 *  @return Success, return MV_OK, and the camera auto white balance will change
 to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceWhiteAuto(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioRed(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������ƽ��
 ��ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioRed(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get white balance red
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of white
 balance red
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioRed(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioRed(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  ���ð�ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õİ�ƽ�� ��
 *  @return �ɹ�,����MV_OK,���������ƽ�� �콫�����Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioRed(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set white balance red
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        White balance red to set
 *  @return Success, return MV_OK, and the camera white balance red will change
 to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioRed(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioGreen(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������ƽ��
 �̽ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioGreen(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get white balance green
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of white
 balance green
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioGreen(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioGreen(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  ���ð�ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õİ�ƽ�� ��
 *  @return �ɹ�,����MV_OK,���������ƽ�� �̽������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioGreen(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set white balance green
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        White balance green to set
 *  @return Success, return MV_OK, and the camera white balance green will
 change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioGreen(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioBlue(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������ƽ��
 ���ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioBlue(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get white balance blue
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of white
 balance blue
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioBlue(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioBlue(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  ���ð�ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õİ�ƽ�� ��
 *  @return �ɹ�,����MV_OK,���������ƽ�� ���������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioBlue(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set white balance blue
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        White balance blue to set
 *  @return Success, return MV_OK, and the camera white balance blue will change
 to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioBlue(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameSpecInfoAbility(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡˮӡ��Ϣ�ڰ�������Ϣ����
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]
 ���ظ��������й����ˮӡ��Ϣ�ڰ�������Ϣ���ͽṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameSpecInfoAbility(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get information type included by frame stamp
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of
 information type included by frame stamp
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFrameSpecInfoAbility(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameSpecInfoAbility(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  ����ˮӡ��Ϣ�ڰ�������Ϣ����
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ�ˮӡ��Ϣ�ڰ�������Ϣ����
 *  @return �ɹ�,����MV_OK,�������ˮӡ��Ϣ�ڰ�������Ϣ���ͻ����Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameSpecInfoAbility(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set information type included by frame stamp
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Information type included by
 frame stamp to set
 *  @return Success, return MV_OK, and the camera information type included by
 frame stamp will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFrameSpecInfoAbility(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceUserID(IN void* handle,
 IN OUT MVCC_STRINGVALUE* pstValue);
 *  @brief  ��ȡ�豸�Զ�������
 *  @param  void* handle                [IN]        ������
 *          MVCC_STRINGVALUE* pstValue  [IN OUT]    ���ظ��������й�������ֽṹ��ָ��
 *  @return �ɹ�,����MV_OK,���һ�ȡ��������Զ������֣�ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceUserID(IN void* handle,
 IN OUT MVCC_STRINGVALUE* pstValue);
 *  @brief  Get device user defined name
 *  @param  void* handle                [IN]        Handle
 *          MVCC_STRINGVALUE* pstValue  [IN OUT]    Structure pointer of device
 name
 *  @return Success, return MV_OK, and get the camera user defined name.
 Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceUserID(
    IN void *handle, IN OUT MVCC_STRINGVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetDeviceUserID(IN void* handle,
 IN const char* chValue);
 *  @brief  �����豸�Զ�������
 *  @param  void* handle                [IN]        ������
 *          IN const char* chValue      [IN]        �豸����
 *  @return �ɹ�,����MV_OK,���������豸�Զ������֣�ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetDeviceUserID(IN void* handle,
 IN const char* chValue);
 *  @brief  Set device user defined name
 *  @param  void* handle                [IN]        Handle
 *          IN const char* chValue      [IN]        Device name
 *  @return Success, return MV_OK, and set the camera user defined name.
 Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetDeviceUserID(IN void *handle,
                                                   IN const char *chValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBurstFrameCount(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡһ�δ�����֡��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]
 ���ظ��������й����һ�δ�����֡���ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBurstFrameCount(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get frame number trigger by once
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of frame
 number trigger by once
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBurstFrameCount(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBurstFrameCount(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  ����һ�δ�����֡��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ�һ�δ�����֡��
 *  @return �ɹ�,����MV_OK,�������һ�δ�����֡�������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBurstFrameCount(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set frame number trigger by once
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Frame number trigger by once
 to set
 *  @return Success, return MV_OK, and the camera frame number trigger by once
 will change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBurstFrameCount(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionLineRate(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��Ƶ
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������Ƶ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionLineRate(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get line rate
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of line
 rate
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionLineRate(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionLineRate(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  ������Ƶ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ���Ƶ
 *  @return �ɹ�,����MV_OK,���������Ƶ�����Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionLineRate(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set line rate
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Line rate to set
 *  @return Success, return MV_OK, and the camera line rate will change to the
 corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionLineRate(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeartBeatTimeout(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ������Ϣ
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT] ���ظ��������й����������Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeartBeatTimeout(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get heartbeat information
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of
 heartbeat information
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetHeartBeatTimeout(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeartBeatTimeout(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  ����������Ϣ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ�������Ϣ
 *  @return �ɹ�,����MV_OK,�������������Ϣ�����Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeartBeatTimeout(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set heartbeat information
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Heartbeat information to set
 *  @return Success, return MV_OK, and the camera heartbeat information will
 change to the corresponding value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetHeartBeatTimeout(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************/
/* �豸���� �� �Ĵ�����д ���쳣���¼��ص�                            	*/
/* Device upgrade, register read and write and exception callback       */
/************************************************************************/
// �豸��������
// Device Local Upgrade
/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_LocalUpgrade(IN void* handle,
                                                            const void
 *pFilePathName);
 *  @brief  �豸��������
 *  @param  void* handle                  [IN]        ������
 *  @param  void *pFilePathName           [IN]        �ļ���
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_LocalUpgrade(IN void* handle,
                                                            const void
 *pFilePathName);
 *  @brief  Device Local Upgrade
 *  @param  void* handle                  [IN]        Camera handle
 *  @param  void *pFilePathName           [IN]        File name
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LocalUpgrade(IN void *handle,
                                                const void *pFilePathName);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetUpgradeProcess(IN void*
 handle,
                                                                 unsigned int*
 pnProcess);
 *  @brief  ��ȡ��������
 *  @param  void* handle                  [IN]        ������
 *  @param  unsigned int* pnProcess       [OUT]       ���Ƚ��յ�ַ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetUpgradeProcess(IN void*
 handle,
                                                                 unsigned int*
 pnProcess);
 *  @brief  Get Upgrade Progress
 *  @param  void* handle                  [IN]        Camera handle
 *  @param  unsigned int* pnProcess       [OUT]       Progress receiving address
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetUpgradeProcess(IN void *handle,
                                                     unsigned int *pnProcess);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetOptimalPacketSize(IN void*
 handle);
 *  @brief  ��ȡ��ѵ�packet size���ýӿ�Ŀǰֻ֧��GigE���
 *  @param  void* handle                  [IN]        ������
 *  @return ���packetsize

 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetOptimalPacketSize(IN void*
 handle);
 *  @brief  Get the optimal Packet Size, Only support GigE Camera
 *  @param  void* handle                  [IN]        Camera handle
 *  @return Optimal packetsize
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOptimalPacketSize(IN void *handle);

/************************************************************************
 *  @fn     MV_CC_ReadMemory
 *  @brief  ���ڴ�
 *  @param  handle���豸���
 *  @param  pBuffer����Ϊ����ֵʹ�ã�����������ڴ�ֵ���ڴ�ֵ�ǰ��մ��ģʽ�洢�ģ�
 *  @param
nAddress������ȡ���ڴ��ַ���õ�ַ���Դ��豸��Camera.xml�ļ��л�ȡ������xxx_RegAddr��xml�ڵ�ֵ
                  ���豸��Camera.xml�ļ������豸��֮���Զ�������Ӧ�ó���ĵ�ǰĿ¼�У�
 *  @param  nLength������ȡ���ڴ泤��
 *  @return �����ش�����

 *  @fn     MV_CC_ReadMemory
 *  @brief  Read Memory
 *  @param  handle: Device Handle
 *  @param  pBuffer: Used as a return value, save the read-in memory value (
Memory value is stored in accordance with the big end model)
 *  @param  nAddress: Memory address to be read, which can be obtained from the
Camera.xml file of the device, the form xml node value of xxx_RegAddr
                     (Camera.xml file of device is automatically generated in
the application's current directory after the device is opened)
 *  @param  nLength: Length of the memory to be read
 *  @return Refer to the return error code
*************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ReadMemory(IN void *handle, void *pBuffer,
                                              int64_t nAddress,
                                              int64_t nLength);

/************************************************************************
 *  @fn     MV_CC_WriteMemory
 *  @brief  д�ڴ�
 *  @param  handle���豸���
 *  @param  pBuffer����д����ڴ�ֵ��ע���ڴ�ֵҪ���մ��ģʽ�洢��
 *  @param
nAddress����д����ڴ��ַ���õ�ַ���Դ��豸��Camera.xml�ļ��л�ȡ������xxx_RegAddr��xml�ڵ�ֵ
                  ���豸��Camera.xml�ļ������豸��֮���Զ�������Ӧ�ó���ĵ�ǰĿ¼�У�
 *  @param  nLength����д����ڴ泤��
 *  @return �����ش�����

 *  @fn     MV_CC_WriteMemory
 *  @brief  Write Memory
 *  @param  handle: Device Handle
 *  @param  pBuffer: Memory value to be written ( Note the memory value to be
stored in accordance with the big end model)
 *  @param  nAddress: Memory address to be written, which can be obtained from
the Camera.xml file of the device, the form xml node value of xxx_RegAddr
                     (Camera.xml file of device is automatically generated in
the application's current directory after the device is opened)
 *  @param  nLength: Length of the memory to be written
 *  @return Refer to the return error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_WriteMemory(IN void *handle,
                                               const void *pBuffer,
                                               int64_t nAddress,
                                               int64_t nLength);

// ע���쳣��Ϣ�ص����ڴ��豸֮�����
// Register exception message callback, call after open device
/************************************************************************
 *  @fn     MV_CC_RegisterExceptionCallBack
 *  @brief  ע���쳣��Ϣ�ص����ڴ��豸֮�����
 *  @param  handle���豸���
 *  @param  cbException       [IN]      �쳣�ص�����ָ��
 *  @param  pUser             [IN]      �û��Զ������
 *  @return �����ش�����

 *  @fn     MV_CC_RegisterExceptionCallBack
 *  @brief  Register Exception Message CallBack, call after open device
 *  @param  handle: Device handle
 *  @param  cbException       [IN]       Exception Message CallBack Function
Pointer
 *  @param  pUser             [IN]       User defined variable
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterExceptionCallBack(
    IN void *handle,
    void(__stdcall *cbException)(unsigned int nMsgType, void *pUser),
    void *pUser);

/************************************************************************
 *  @fn     MV_CC_RegisterAllEventCallBack
 *  @brief  ע��ȫ���¼��ص����ڴ��豸֮�����,ֻ֧��GIGE
 *  @param  handle���豸���
 *  @param  cbEvent           [IN]      �쳣�ص�����ָ��
 *  @param  pUser             [IN]      �û��Զ������
 *  @return �����ش�����

 *  @fn     MV_CC_RegisterEventCallBack
 *  @brief  Register event callback, which is called after the device is opened
 *  @param  handle: Device handel
 *  @param  cbEvent           [IN]      Exception CallBack Function Pointer
 *  @param  pUser             [IN]      User defined variable
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterAllEventCallBack(
    void *handle,
    void(__stdcall *cbEvent)(MV_EVENT_OUT_INFO *pEventInfo, void *pUser),
    void *pUser);

/************************************************************************
 *  @fn     MV_CC_RegisterEventCallBackEx
 *  @brief  ע�ᵥ���¼��ص����ڴ��豸֮�����,ֻ֧��GIGE
 *  @param  handle���豸���
 *  @param  pEventName        [IN]      �¼�����
 *  @param  cbEvent           [IN]      �쳣�ص�����ָ��
 *  @param  pUser             [IN]      �û��Զ������
 *  @return �����ش�����


************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterEventCallBackEx(
    void *handle, const char *pEventName,
    void(__stdcall *cbEvent)(MV_EVENT_OUT_INFO *pEventInfo, void *pUser),
    void *pUser);

/************************************************************************/
/* GigEVision �豸���еĽӿ�                                     		*/
/* GigEVision device specific interface                                 */
/************************************************************************/
/************************************************************************
 *  @fn     MV_GIGE_ForceIpEx
 *  @brief  ǿ��IP
 *  @param  handle���豸���
 *  @param  nIP               [IN]      ���õ�IP
 *  @param  nSubNetMask       [IN]      ��������
 *  @param  nDefaultGateWay   [IN]      Ĭ������
 *  @return �����ش�����

 *  @fn     MV_GIGE_ForceIpEx
 *  @brief  Force IP
 *  @param  handle: Device handel
 *  @param  nIP               [IN]      IP to set
 *  @param  nSubNetMask       [IN]      Subnet mask
 *  @param  nDefaultGateWay   [IN]      Default gateway
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_ForceIpEx(IN void *handle,
                                               unsigned int nIP,
                                               unsigned int nSubNetMask,
                                               unsigned int nDefaultGateWay);

/************************************************************************/
/* GigEVision �豸���еĽӿ�                                     */
/************************************************************************/
/************************************************************************
 *  @fn     MV_GIGE_SetIpConfig
 *  @brief  ����IP��ʽ
 *  @param  handle���豸���
 *  @param  nType               [IN]      IP���ͣ���MV_IP_CFG_x
 *  @return �����ش�����

 *  @fn     MV_GIGE_SetIpConfig
 *  @brief  IP configuration method
 *  @param  handle: Device handel
 *  @param  nType               [IN]      IP type, refer to MV_IP_CFG_x
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetIpConfig(IN void *handle,
                                                 unsigned int nType);

/************************************************************************
 *  @fn     MV_GIGE_SetNetTransMode
 *  @brief  ���ý�ʹ��ĳ��ģʽ,type: MV_NET_TRANS_x��������ʱ��Ĭ������ʹ��driver
 *  @param  handle���豸���
 *  @param  nType               [IN]      ���紫��ģʽ����MV_NET_TRANS_x
 *  @return �����ش�����

 *  @fn     MV_GIGE_SetNetTransMode
 *  @brief  Set to use only one mode,type: MV_NET_TRANS_x. When do not set,
priority is to use driver by default
 *  @param  handle: Device handel
 *  @param  nType               [IN]      Net transmission mode, refer to
MV_NET_TRANS_x
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetNetTransMode(IN void *handle,
                                                     unsigned int nType);

/************************************************************************
 *  @fn     MV_GIGE_GetNetTransInfo
 *  @brief  ��ȡ���紫����Ϣ
 *  @param  handle���豸���
 *  @param  pstInfo             [OUT]      ��Ϣ�ṹ��
 *  @return �����ش�����

 *  @fn     MV_GIGE_GetNetTransInfo
 *  @brief  Get net transmission information
 *  @param  handle: Device handel
 *  @param  pstInfo             [OUT]      Information Structure
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetNetTransInfo(IN void *handle,
                                                     MV_NETTRANS_INFO *pstInfo);

/************************************************************************
 *  @fn     MV_GIGE_SetGvcpTimeout
 *  @brief  ����GVCP���ʱʱ��
 *  @param  handle                 [IN]           �����ַ
 *  @param  nMillisec              [IN]           ��ʱʱ�䣬�Ժ���λ��λ����Χ��0-10000
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_GIGE_SetGvcpTimeout
 *  @brief  Set GVCP cammand timeout
 *  @param  handle                 [IN]           Handle
 *  @param  nMillisec              [IN]           Timeout, ms as unit, range:
 0-10000
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGvcpTimeout(void *handle,
                                                    unsigned int nMillisec);

/************************************************************************
 *  @fn     MV_GIGE_SetResend
 *  @brief  �����Ƿ���ط���֧�֣����ط�������
 *  @param  handle                 [IN]           �����ַ
 *  @param  bEnable                [IN]           �Ƿ�֧���ط���
 *  @param  nMaxResendPercent      [IN]           ����ط���
 *  @param  nResendTimeout         [IN]           �ط���ʱʱ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_GIGE_SetResend
 *  @brief  Set whethe to enable resend, and set resend
 *  @param  handle                 [IN]           Handle
 *  @param  bEnable                [IN]           enable resend
 *  @param  nMaxResendPercent      [IN]           Max resend persent
 *  @param  nResendTimeout         [IN]           Resend timeout
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
#ifndef __cplusplus
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResend(void *handle,
                                               unsigned int bEnable,
                                               unsigned int nMaxResendPercent,
                                               unsigned int nResendTimeout);
#else
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResend(
    void *handle, unsigned int bEnable, unsigned int nMaxResendPercent = 10,
    unsigned int nResendTimeout = 50);
#endif

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPSPacketSize(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ�������С
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]
 ���ظ��������й�����������С�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPSPacketSize(IN void*
 handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get network packet size
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of network
 packet size
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPSPacketSize(
    IN void *handle, IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPSPacketSize(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  �����������С
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ��������С
 *  @return �ɹ�,����MV_OK,��������������С�����Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPSPacketSize(IN void*
 handle, IN const unsigned int nValue);
 *  @brief  Set network packet size
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Packet size to set
 *  @return Success, return MV_OK, and change packet size to setting value.
 Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPSPacketSize(
    IN void *handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPD(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��������ͼ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]
 ���ظ��������й������������ͼ���ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *
 *          �ɲ��սӿ�MV_CC_GetWidth

 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPD(IN void* handle, IN
 OUT MVCC_INTVALUE* pstValue);
 *  @brief  Get network packet sending delay
 *  @param  void* handle                [IN]        Handle
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   Structure pointer of network
 packet sending delay
 *  @return Success, return MV_OK. Failure, return error code
 *
 *          Refer to MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPD(IN void *handle,
                                                IN OUT MVCC_INTVALUE *pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPD(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  ������������ͼ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ���������ͼ��
 *  @return �ɹ�,����MV_OK,���������������ͼ�������Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPD(IN void* handle, IN
 const unsigned int nValue);
 *  @brief  Set network packet sending delay
 *  @param  void* handle                [IN]        Handle
 *          const unsigned int nValue   [IN]        Packet delay to set
 *  @return Success, return MV_OK, and change packet delay to setting value.
 Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPD(IN void *handle,
                                                IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCDA(IN void* handle,
 unsigned int* pnIP);
 *  @brief  ��ȡ���ն�IP��ַ��0xa9fe0102 ��ʾ 169.254.1.2
 *  @param  void* handle                [IN]        ������
 *  @param  unsigned int* pnIP          [IN][OUT]
 ���ظ������߽��ն�IP��ַ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCDA(IN void* handle,
 unsigned int* pnIP);
 *  @brief  Get receiver IP address, 0xa9fe0102 indicates 169.254.1.2
 *  @param  void* handle                [IN]        Handle
 *  @param  unsigned int* pnIP          [IN][OUT]   Receiver IP address
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCDA(IN void *handle,
                                                unsigned int *pnIP);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCDA(IN void* handle,
 unsigned int nIP);
 *  @brief  ���ý��ն�IP��ַ
 *  @param  void* handle                [IN]        ������
 *          unsigned int nIP            [IN]        ��Ҫ���õĽ��ն�IP��ַ
 *  @return �ɹ�,����MV_OK,����������ն�IP��ַ�����Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCDA(IN void* handle,
 unsigned int nIP);
 *  @brief  Set receiver IP address
 *  @param  void* handle                [IN]        Handel
 *          unsigned int nIP            [IN]        Receiver IP address to set
 *  @return Success, return MV_OK, and change receiver IP address to setting
 value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCDA(IN void *handle,
                                                unsigned int nIP);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCSP(IN void* handle,
 unsigned int* pnPort);
 *  @brief  ��ȡ���Ͷ˵Ķ˿ں�
 *  @param  void* handle                [IN]        ������
 *  @param  unsigned int* pnPort        [IN][OUT]   ���ظ������߷��Ͷ˵Ķ˿ں�
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCSP(IN void* handle,
 unsigned int* pnPort);
 *  @brief  Get transmitter port number
 *  @param  void* handle                [IN]        Handle
 *  @param  unsigned int* pnPort        [IN][OUT]   Transmitter port number
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCSP(IN void *handle,
                                                unsigned int *pnPort);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCSP(IN void* handle,
 unsigned int nPort);
 *  @brief  ���÷��Ͷ˵Ķ˿ں�
 *  @param  void* handle                [IN]        ������
 *          unsigned int nPort          [IN]        ��Ҫ���õķ��Ͷ˵Ķ˿ں�
 *  @return �ɹ�,����MV_OK,����������Ͷ˵Ķ˿ںŻ����Ϊ��Ӧֵ��ʧ��,���ش�����

 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCSP(IN void* handle,
 unsigned int nPort);
 *  @brief  Set transmitter port number
 *  @param  void* handle                [IN]        Handle
 *          unsigned int nPort          [IN]        Transmitter port number to
 set
 *  @return Success, return MV_OK, and change transmitter port number to setting
 value. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCSP(IN void *handle,
                                                unsigned int nPort);

/************************************************************************
 * @fn      MV_GIGE_SetTransmissionType
 * @brief   ���ô���ģʽ������Ϊ����ģʽ���鲥ģʽ��
 * @param   handle                      [IN]       ������
 * @param   stTransmissionType          [IN]       ����ģʽ�ṹ��
 * @return ���سɹ��������
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetTransmissionType(
    void *handle, MV_TRANSMISSION_TYPE *pstTransmissionType);

/************************************************************************
 * @fn      MV_GIGE_IssueActionCommand
 * @brief   ������������
 * @param   pstActionCmdInfo                   [IN]       ����������Ϣ |
 *en:Action Command
 * @param   MV_ACTION_CMD_RESULT_LIST          [OUT]      �����������Ϣ�б� |
 *en:Action Command Result List
 * @return ���سɹ��������
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_IssueActionCommand(
    IN MV_ACTION_CMD_INFO *pstActionCmdInfo,
    OUT MV_ACTION_CMD_RESULT_LIST *pstActionCmdResults);

/************************************************************************/
/* CameraLink �豸���еĽӿ�                                     */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CAML_SetDeviceBauderate
 *  @brief  �����豸������
 *  @param  handle���豸���
 *  @param  nBaudrate               [IN]
���õĲ�����ֵ����ֵ�ο�CameraParams.h�к궨�壬��#define MV_CAML_BAUDRATE_9600 0x00000001
 *  @return �����ش�����

 *  @fn     MV_CAML_SetDeviceBauderate
 *  @brief  Set device bauderate using one of the CL_BAUDRATE_XXXX value
 *  @param  handle Handle
 *  @param  nBaudrate               [IN]      baud rate to set. Refer to the
'CameraParams.h' for parameter definitions, for example, #define
MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetDeviceBauderate(IN void *handle,
                                                        unsigned int nBaudrate);

/************************************************************************
 *  @fn     MV_CAML_GetDeviceBauderate
 *  @brief  ��ȡ�豸������
 *  @param  handle���豸���
 *  @param  pnCurrentBaudrate               [OUT]
��������Ϣָ�룬��ֵ�ο�CameraParams.h�к궨�壬��#define MV_CAML_BAUDRATE_9600 0x00000001
 *  @return �����ش�����

 *  @fn     MV_CAML_GetDeviceBauderate
 *  @brief  Returns the current device bauderate, using one of the
CL_BAUDRATE_XXXX value
 *  @param  handle Handle
 *  @param  pnCurrentBaudrate               [OUT]      Return pointer of baud
rate to user. Refer to the 'CameraParams.h' for parameter definitions, for
example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetDeviceBauderate(
    IN void *handle, unsigned int *pnCurrentBaudrate);

/************************************************************************
 *  @fn     MV_CAML_GetSupportBauderates
 *  @brief  ��ȡ�豸������������֧�ֵĲ�����
 *  @param  handle���豸���
 *  @param  pnBaudrateAblity               [OUT]
֧�ֵĲ�������Ϣ��ָ�롣��֧�ֲ����ʵĻ���������������ֵ�ο�CameraParams.h�к궨�壬��#define
MV_CAML_BAUDRATE_9600  0x00000001
 *  @return �����ش�����

 *  @fn     MV_CAML_GetSupportBauderates
 *  @brief  Returns supported bauderates of the combined device and host
interface
 *  @param  handle Handle
 *  @param  pnBaudrateAblity               [OUT]      Return pointer of the
supported bauderates to user. 'OR' operation results of the supported
bauderates. Refer to the 'CameraParams.h' for single value definitions, for
example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetSupportBauderates(
    IN void *handle, unsigned int *pnBaudrateAblity);

/************************************************************************
 *  @fn     MV_CAML_SetGenCPTimeOut
 *  @brief  ���ô��ڲ����ȴ�ʱ��
 *  @param  handle���豸���
 *  @param  nMillisec               [IN]      ���ڲ����ĵȴ�ʱ��, ms
 *  @return �����ش�����

 *  @fn     MV_CAML_SetGenCPTimeOut
 *  @brief  Sets the timeout for operations on the serial port
 *  @param  handle Handle
 *  @param  nMillisec               [IN]      Timeout in [ms] for operations on
the serial port.
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetGenCPTimeOut(IN void *handle,
                                                     unsigned int nMillisec);

/************************************************************************/
/* XML������������                                                      */
/* XML parse tree generation                                            */
/************************************************************************/
/***********************************************************************
 *  @fn         MV_XML_GetGenICamXML
 *  @brief      ��ȡ���������XML
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pnDataLen              [OUT]         ʵ�����ݴ�С
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_XML_GetGenICamXML
 *  @brief      Get camera feature tree XML
 *  @param       handle                 [IN]          Handle
 *  @param       pData                  [OUT]         Recevied image buffer
 pointer
 *  @param       nDataSize              [IN]          Recevied buffer size
 *  @param       pnDataLen              [OUT]         Actual data size
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetGenICamXML(IN void *handle,
                                                  IN OUT unsigned char *pData,
                                                  IN unsigned int nDataSize,
                                                  OUT unsigned int *pnDataLen);

/***********************************************************************
 *  @fn         MV_XML_GetRootNode
 *  @brief      ��ȡ���ڵ�
 *  @param       handle                 [IN]          ���
 *  @param       pstNode                [OUT]         ���ڵ���Ϣ�ṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_XML_GetRootNode
 *  @brief      Get root node
 *  @param       handle                 [IN]          Handle
 *  @param       pstNode                [OUT]         Root node information
 structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetRootNode(
    IN void *handle, IN OUT MV_XML_NODE_FEATURE *pstNode);

/***********************************************************************
 *  @fn         MV_XML_GetChildren
 *  @brief      ��xml�л�ȡָ���ڵ�������ӽڵ㣬���ڵ�ΪRoot
 *  @param       handle                 [IN]          ���
 *  @param       pstNode                [IN]          ���ڵ���Ϣ�ṹ��
 *  @param       pstNodesList           [OUT]         �ڵ��б��ṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_XML_GetChildren
 *  @brief      Get all children node of specific node from xml, root node is
 Root
 *  @param       handle                 [IN]          Handle
 *  @param       pstNode                [IN]          Root node information
 structure
 *  @param       pstNodesList           [OUT]         Node information structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetChildren(
    IN void *handle, IN MV_XML_NODE_FEATURE *pstNode,
    IN OUT MV_XML_NODES_LIST *pstNodesList);

/***********************************************************************
 *  @fn         MV_XML_GetNodeFeature
 *  @brief      ��õ�ǰ�ڵ������
 *  @param       handle                 [IN]          ���
 *  @param       pstNode                [IN]          ���ڵ���Ϣ�ṹ��
 *  @param       pstFeature             [OUT]         ��ǰ�ڵ����Խṹ�壬
                           pstFeature ����ṹ�����ݲο� MV_XML_FEATURE_x
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_XML_GetNodeFeature
 *  @brief      Get current node feature
 *  @param       handle                 [IN]          Handle
 *  @param       pstNode                [IN]          Root node information
 structure
 *  @param       pstFeature             [OUT]         Current node feature
 structure
                           Details of pstFeature refer to MV_XML_FEATURE_x
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetNodeFeature(
    IN void *handle, IN MV_XML_NODE_FEATURE *pstNode, IN OUT void *pstFeature);

/***********************************************************************
 *  @fn         MV_XML_UpdateNodeFeature
 *  @brief      ���½ڵ�
 *  @param       handle                 [IN]          ���
 *  @param       enType                 [IN]          �ڵ�����
 *  @param       pstFeature             [OUT]         ��ǰ�ڵ����Խṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_XML_UpdateNodeFeature
 *  @brief      Update node
 *  @param       handle                 [IN]          Handle
 *  @param       enType                 [IN]          Node type
 *  @param       pstFeature             [OUT]         Current node feature
 structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_UpdateNodeFeature(
    IN void *handle, IN enum MV_XML_InterfaceType enType, IN void *pstFeature);

// �нڵ���Ҫ����ʱ�Ļص�����
// ������MV_XML_UpdateNodeFeature�ӿڸ��½ڵ�����ʱ��ע��Ļص�����cbUpdate����pstNodesList�з�����֮������Ľڵ�
/***********************************************************************
 *  @fn         MV_XML_RegisterUpdateCallBack
 *  @brief      ע����»ص�
 *  @param       handle                 [IN]          ���
 *  @param       cbUpdate               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_XML_RegisterUpdateCallBack
 *  @brief      Register update callback
 *  @param       handle                 [IN]          Handle
 *  @param       cbUpdate               [IN]          Callback function pointer
 *  @param       pUser                  [IN]          User defined variable
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_RegisterUpdateCallBack(
    IN void *handle,
    IN void(__stdcall *cbUpdate)(enum MV_XML_InterfaceType enType,
                                 void *pstFeature,
                                 MV_XML_NODES_LIST *pstNodesList, void *pUser),
    IN void *pUser);

/************************************************************************/
/* ���ӽӿ� */
/* Additional interface                                                 */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CC_SaveImageEx2
 *  @brief  ����ͼƬ��֧��Bmp��Jpeg.����������50-99֮ǰ
 *  @param  handle                 [IN]           �����ַ
 *  @param  pSaveParam             [IN][OUT]          ����ͼƬ�����ṹ��
                       pData;              // [IN]     �������ݻ���
                       nDataLen;           // [IN]     �������ݴ�С
                       enPixelType;        // [IN]     �������ݵ����ظ�ʽ
                       nWidth;             // [IN]     ͼ���
                       nHeight;            // [IN]     ͼ���
                       pImageBuffer;       // [OUT]    ���ͼƬ����
                       nImageLen;          // [OUT]    ���ͼƬ��С
                       nBufferSize;        // [IN]     �ṩ�������������С
                       enImageType;        // [IN]     ���ͼƬ��ʽ
                       nJpgQuality;        // [IN]     ��������, (50-99]
                       iMethodValue        // [IN]     Bayer��ʽתΪRGB24�Ĳ�ֵ����
                       nReserved[4];
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_SaveImageEx2
 *  @brief  Save image, support Bmp and Jpeg. Encoding quality, (50-99]
 *  @param  handle                 [IN]           Handle
 *  @param  pSaveParam             [IN][OUT]           Save image parameters
 structure
                       pData;              // [IN]     Input data buffer
                       nDataLen;           // [IN]     Input data size
                       enPixelType;        // [IN]     Pixel format of input
 data
                       nWidth;             // [IN]     Image width
                       nHeight;            // [IN]     Image height
                       pImageBuffer;       // [OUT]    Output image buffer
                       nImageLen;          // [OUT]    Output image size
                       nBufferSize;        // [IN]     Output buffer size
 provided
                       enImageType;        // [IN]     Output image format
                       nJpgQuality;        // [IN]     Encoding quality, (50-99]
                       iMethodValue        // [IN]     Interpolation method of
 convert Bayer to RGB24
                       nReserved[4];
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageEx2(
    IN void *handle, MV_SAVE_IMAGE_PARAM_EX *pSaveParam);

/************************************************************************
 *  @fn     MV_CC_ConvertPixelType
 *  @brief  ���ظ�ʽת��
 *  @param  pstCvtParam             [IN][OUT]          ����ͼƬ�����ṹ��
                     unsigned short         nWidth;             // [IN]     ͼ���
                     unsigned short         nHeight;            // [IN]     ͼ���
                     enum MvGvspPixelType   enSrcPixelType;     // [IN] Դ���ظ�ʽ
                     unsigned char*         pSrcData;           // [IN]
 �������ݻ���
                     unsigned int           nSrcDataLen;        // [IN]
 �������ݴ�С
                     enum MvGvspPixelType   enDstPixelType;     // [IN]
 Ŀ�����ظ�ʽ
                     unsigned char*         pDstBuffer;         // [OUT]
 ������ݻ���
                     unsigned int           nDstLen;            // [OUT]
 ������ݴ�С
                     unsigned int           nDstBufferSize;     // [IN]
 �ṩ�������������С
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_ConvertPixelType
 *  @brief  Pixel format conversion
 *  @param  pstCvtParam             [IN][OUT]          Save image parameter
 structure
                     unsigned short         nWidth;             // [IN] Width
                     unsigned short         nHeight;            // [IN] Height
                     enum MvGvspPixelType   enSrcPixelType;     // [IN]
 Source pixel format
                     unsigned char*         pSrcData;           // [IN]
 Input data buffer
                     unsigned int           nSrcDataLen;        // [IN]
 Input data size
                     enum MvGvspPixelType   enDstPixelType;     // [IN]
 Destination pixel format
                     unsigned char*         pDstBuffer;         // [OUT]
 Output data buffer
                     unsigned int           nDstLen;            // [OUT]
 Output data size
                     unsigned int           nDstBufferSize;     // [IN]
 Provided output buffer size
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ConvertPixelType(
    IN void *handle, IN OUT MV_CC_PIXEL_CONVERT_PARAM *pstCvtParam);

/************************************************************************
 *  @fn     MV_CC_SetBayerCvtQuality
 *  @brief  ��ֵ�㷨��������
 *  @param  BayerCvtQuality             [IN]          Bayer�Ĳ�ֵ����  0-�����
 1-˫���� 2-Hamilton
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_SetBayerCvtQuality
 *  @brief  Interpolation algorithm type setting
 *  @param  BayerCvtQuality             [IN]          Bayer interpolation method
 0-nearest neighbour 1-bilinearity 2-Hamilton
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCvtQuality(
    IN void *handle, IN unsigned int BayerCvtQuality);

/************************************************************************
 *  @fn     MV_CC_GetTlProxy
 *  @brief  ��ȡGenICam����
 *  @param  handle                 [IN]           �����ַ
 *  @return GenICam������ָ�� ����������ֵ��NULL���쳣����NULL

 *  @fn     MV_CC_GetTlProxy
 *  @brief  Get GenICam proxy
 *  @param  handle                 [IN]           Handle address
 *  @return GenICam proxy pointer, normal, return non-NULL; exception, return
 NULL
 ************************************************************************/
MV_CAMCTRL_API void *__stdcall MV_CC_GetTlProxy(IN void *handle);

/************************************************************************
 *  @fn     MV_CC_FeatureSave
 *  @brief  �����������
 *  @param  handle                [IN]           �����ַ
 *  @param  pFileName             [IN]          �����ļ���
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_FeatureSave
 *  @brief  Save camera feature
 *  @param  handle                [IN]           Handle
 *  @param  pFileName             [IN]          File name
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureSave(IN void *handle,
                                               IN const char *pFileName);

/************************************************************************
 *  @fn     MV_CC_FeatureLoad
 *  @brief  �����������
 *  @param  handle                [IN]           �����ַ
 *  @param  pFileName             [IN]          �����ļ���
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_FeatureLoad
 *  @brief  Load camera feature
 *  @param  handle                [IN]           Handle
 *  @param  pFileName             [IN]          File name
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureLoad(IN void *handle,
                                               IN const char *pFileName);

/************************************************************************
 *  @fn     MV_CC_FileAccessRead
 *  @brief  �������ȡ�ļ�
 *  @param  handle                [IN]           �����ַ
 *  @param  pstFileAccess         [IN]           �ļ���ȡ�ṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_FileAccessRead
 *  @brief  Read the file from the camera
 *  @param  handle                [IN]           Handle
 *  @param  pstFileAccess         [IN]           File access structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessRead(
    IN void *handle, IN MV_CC_FILE_ACCESS *pstFileAccess);

/************************************************************************
 *  @fn     MV_CC_FileAccessWrite
 *  @brief  ���ļ�д�����
 *  @param  handle                [IN]           �����ַ
 *  @param  pstFileAccess         [IN]           �ļ���ȡ�ṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_FileAccessWrite
 *  @brief  Write the file to camera
 *  @param  handle                [IN]           Handle
 *  @param  pstFileAccess         [IN]           File access structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessWrite(
    IN void *handle, IN MV_CC_FILE_ACCESS *pstFileAccess);

/************************************************************************
 *  @fn     MV_CC_GetFileAccessProgress
 *  @brief  ��ȡ�ļ���ȡ�Ľ���
 *  @param  handle                  [IN]           �����ַ
 *  @param  pstFileAccessProgress   [IN]           ��������
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ����ǰ�ļ���ȡ��״̬��

 *  @fn     MV_CC_GetFileAccessProgress
 *  @brief  Get File Access Progress
 *  @param  handle                  [IN]           Handle
 *  @param  pstFileAccessProgress   [IN]           File access Progress
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFileAccessProgress(
    IN void *handle, OUT MV_CC_FILE_ACCESS_PROGRESS *pstFileAccessProgress);

/************************************************************************
 *  @fn     MV_CC_StartRecord
 *  @brief  ��ʼ¼��
 *  @param  handle                [IN]           �����ַ
 *  @param  pstRecordParam         [IN]          ¼������ṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_StartRecord
 *  @brief  Start Record
 *  @param  handle                [IN]           Handle
 *  @param  pstRecordParam         [IN]          Record param structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StartRecord(
    IN void *handle, IN MV_CC_RECORD_PARAM *pstRecordParam);

/************************************************************************
 *  @fn     MV_CC_InputOneFrame
 *  @brief  ����¼������
 *  @param  handle                [IN]           �����ַ
 *  @param  pstInputFrameInfo         [IN]       ¼�����ݽṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_InputOneFrame
 *  @brief  Input RAW data to Record
 *  @param  handle                [IN]           Handle
 *  @param  pstInputFrameInfo         [IN]       Record data structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_InputOneFrame(
    IN void *handle, IN MV_CC_INPUT_FRAME_INFO *pstInputFrameInfo);

/************************************************************************
 *  @fn     MV_CC_StopRecord
 *  @brief  ֹͣ¼��
 *  @param  handle                [IN]           �����ַ
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_StopRecord
 *  @brief  Stop Record
 *  @param  handle                [IN]           Handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StopRecord(IN void *handle);

/************************************************************************/
/* ���õĽӿ� */
/* Abandoned interface                                                  */
/************************************************************************/
/***********************************************************************
 *  @fn         MV_CC_GetOneFrame
 *  @brief
 ��ȡһ֡ͼ�񣬴˺���Ϊ��ѯʽ��ȡ��ÿ�ε��ò�ѯ�ڲ�������
                �����ݣ���������Χ���ݣ������ݷ��ش�����
                ���ýӿ������ã�������� MV_CC_GetOneFrameTimeOut�ӿڣ�
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_GetOneFrame
 *  @brief      Get one frame data, this function is using query to get data,
                query whether the internal cache has data, return data if there
 has, return error code if no data
                (This interface is abandoned, it is recommended to use the
 MV_CC_GetOneFrameTimeOut)
 *  @param       handle                 [IN]          Handle
 *  @param       pData                  [OUT]         Recevied image data
 pointer
 *  @param       nDataSize              [IN]          Recevied buffer size
 *  @param       pFrameInfo             [OUT]         Image information
 structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrame(
    IN void *handle, IN OUT unsigned char *pData, IN unsigned int nDataSize,
    IN OUT MV_FRAME_OUT_INFO *pFrameInfo);

/***********************************************************************
 *  @fn         MV_CC_GetOneFrameEx
 *  @brief      ��ȡһ֡trunck���ݣ��˺���Ϊ��ѯʽ��ȡ��ÿ�ε��ò�ѯ�ڲ�
                �����������ݣ���������Χ���ݣ������ݷ��ش�����
                ���ýӿ������ã�������� MV_CC_GetOneFrameTimeOut�ӿڣ�
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_GetOneFrameEx
 *  @brief      Get one frame of trunck data, this function is using query to
 get data,
                query whether the internal cache has data, return data if there
 has, return error code if no data
                (This interface is abandoned, it is recommended to use the
 MV_CC_GetOneFrameTimeOut)
 *  @param       handle                 [IN]          Handle
 *  @param       pData                  [OUT]         Recevied image data
 pointer
 *  @param       nDataSize              [IN]          Recevied buffer size
 *  @param       pFrameInfo             [OUT]         Image information
 structure
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrameEx(
    IN void *handle, IN OUT unsigned char *pData, IN unsigned int nDataSize,
    IN OUT MV_FRAME_OUT_INFO_EX *pFrameInfo);

/***********************************************************************
 *  @fn         MV_CC_RegisterImageCallBack
 *  @brief      ע��ͼ�����ݻص����ýӿ������ã��������
 MV_CC_RegisterImageCallBackEx�ӿڣ�
 *  @param       handle                 [IN]          ���
 *  @param       cbOutput               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn         MV_CC_RegisterImageCallBack
 *  @brief      Register image data callback (This interface is abandoned, it is
 recommended to use the MV_CC_RegisterImageCallBackEx)
 *  @param       handle                 [IN]          Handle
 *  @param       cbOutput               [IN]          Callback function pointer
 *  @param       pUser                  [IN]          User defined variable
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBack(
    void *handle,
    void(__stdcall *cbOutput)(unsigned char *pData,
                              MV_FRAME_OUT_INFO *pFrameInfo, void *pUser),
    void *pUser);

/************************************************************************
 *  @fn     MV_CC_SaveImage
 *  @brief  ����ͼƬ���ýӿ������ã�������� MV_CC_SaveImageEx�ӿڣ�
 *  @param  pSaveParam             [IN][OUT]          ����ͼƬ�����ṹ��
                       pData;              // [IN]     �������ݻ���
                       nDataLen;           // [IN]     �������ݴ�С
                       enPixelType;        // [IN]     �������ݵ����ظ�ʽ
                       nWidth;             // [IN]     ͼ���
                       nHeight;            // [IN]     ͼ���
                       pImageBuffer;       // [OUT]    ���ͼƬ����
                       nImageLen;          // [OUT]    ���ͼƬ��С
                       nBufferSize;        // [IN]     �ṩ�������������С
                       enImageType;        // [IN]     ���ͼƬ��ʽ
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_SaveImage
 *  @brief  Save image (This interface is abandoned, it is recommended to use
 the MV_CC_SaveImageEx)
 *  @param  pSaveParam             [IN][OUT]          Save image parameters
 structure
                       pData;              // [IN]     Input data buffer
                       nDataLen;           // [IN]     Input data size
                       enPixelType;        // [IN]     Input data pixel format
                       nWidth;             // [IN]     Width
                       nHeight;            // [IN]     Height
                       pImageBuffer;       // [OUT]    Output image buffer
                       nImageLen;          // [OUT]    Output image size
                       nBufferSize;        // [IN]     Provided output buffer
 size
                       enImageType;        // [IN]     Output image type
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImage(
    IN OUT MV_SAVE_IMAGE_PARAM *pSaveParam);

/************************************************************************
 *  @fn     MV_GIGE_ForceIp
 *  @brief  ǿ��IP���ýӿ������ã�������� MV_GIGE_ForceIpEx�ӿڣ�
 *  @param  handle���豸���
 *  @param  nIP               [IN]      ���õ�IP
 *  @return �����ش�����

 *  @fn     MV_GIGE_ForceIp
 *  @brief  Force IP (This interface is abandoned, it is recommended to use the
MV_GIGE_ForceIpEx)
 *  @param  handle Handle
 *  @param  nIP               [IN]      IP to set
 *  @return Refer to error code
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_ForceIp(IN void *handle, unsigned int nIP);

/************************************************************************
 *  @fn     MV_CC_RegisterEventCallBack
 *  @brief  ע���¼��ص����ýӿ������ã�������� MV_CC_RegisterEventCallBackEx�ӿڣ�
 *  @param  handle���豸���
 *  @param  cbEvent           [IN]      �¼��ص�����ָ��
 *  @param  pUser             [IN]      �û��Զ������
 *  @return �����ش�����

 *  @fn     MV_CC_RegisterEventCallBack
 *  @brief  Register event callback (this interface has been deprecated and is
recommended to be converted to the MV_CC_RegisterEventCallBackEx interface)
 *  @param  handle���豸���
 *  @param  cbEvent           [IN]      event callback pointer
 *  @param  pUser             [IN]      User defined value
 *  @return �����ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterEventCallBack(
    void *handle,
    void(__stdcall *cbEvent)(unsigned int nExternalEventId, void *pUser),
    void *pUser);

/************************************************************************
 *  @fn     MV_CC_SaveImageEx
 *  @brief  ����ͼƬ��֧��Bmp��Jpeg.����������50-99֮ǰ ���ýӿ������ã��������
 MV_CC_SaveImageEx2�ӿڣ�
 *  @param  pSaveParam             [IN][OUT]          ����ͼƬ�����ṹ��
                       pData;              // [IN]     �������ݻ���
                       nDataLen;           // [IN]     �������ݴ�С
                       enPixelType;        // [IN]     �������ݵ����ظ�ʽ
                       nWidth;             // [IN]     ͼ���
                       nHeight;            // [IN]     ͼ���
                       pImageBuffer;       // [OUT]    ���ͼƬ����
                       nImageLen;          // [OUT]    ���ͼƬ��С
                       nBufferSize;        // [IN]     �ṩ�������������С
                       enImageType;        // [IN]     ���ͼƬ��ʽ
                       nJpgQuality;        // [IN]     ��������, (50-99]
                       nReserved[4];
 *  @return
 �ɹ�������MV_OK�����󣬷��ش�����

 *  @fn     MV_CC_SaveImageEx
 *  @brief  Save image, support Bmp and Jpeg. Encoding quality, (50-99]
 *  @param  pSaveParam             [IN][OUT]           Save image parameters
 structure
                       pData;              // [IN]     Input data buffer
                       nDataLen;           // [IN]     Input data size
                       enPixelType;        // [IN]     Pixel format of input
 data
                       nWidth;             // [IN]     Image width
                       nHeight;            // [IN]     Image height
                       pImageBuffer;       // [OUT]    Output image buffer
                       nImageLen;          // [OUT]    Output image size
                       nBufferSize;        // [IN]     Output buffer size
 provided
                       enImageType;        // [IN]     Output image format
                       nJpgQuality;        // [IN]     Encoding quality, (50-99]
                       nReserved[4];
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageEx(
    IN OUT MV_SAVE_IMAGE_PARAM_EX *pSaveParam);

#ifdef __cplusplus
}
#endif

#endif //_MV_CAMERA_CTRL_H_
