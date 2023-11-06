
#ifndef _MV_ERROR_DEFINE_H_
#define _MV_ERROR_DEFINE_H_

// ch:正确码定义 | en://Definition of correct code
#define MV_OK                   0x00000000  ///< 成功，无错误 | en:Successed, no error

//通用错误码定义:范围0x80000000-0x800000FF | en:
#define MV_E_HANDLE             0x80000000  ///< 错误或无效的句柄 | en:Error or invalid handle
#define MV_E_SUPPORT            0x80000001  ///< 不支持的功能 | en:Not supported function
#define MV_E_BUFOVER            0x80000002  ///< 缓存已满 | en:Cache is full
#define MV_E_CALLORDER          0x80000003  ///< 函数调用顺序错误 | en:Function calling order error
#define MV_E_PARAMETER          0x80000004  ///< 错误的参数 | en:Incorrect parameter
#define MV_E_RESOURCE           0x80000006  ///< 资源申请失败 | en:Applying resource failed
#define MV_E_NODATA             0x80000007  ///< 无数据 | en:No data
#define MV_E_PRECONDITION       0x80000008  ///< 前置条件有误，或运行环境已发生变化 | en:Precondition error, or running environment changed
#define MV_E_VERSION            0x80000009  ///< 版本不匹配 | en:Version mismatches
#define MV_E_NOENOUGH_BUF       0x8000000A  ///< 传入的内存空间不足 | en:Insufficient memory
#define MV_E_ABNORMAL_IMAGE     0x8000000B  ///< 异常图像，可能是丢包导致图像不完整 | en:Abnormal image, maybe incomplete image because of lost packet
#define MV_E_LOAD_LIBRARY       0x8000000C  ///< 动态导入DLL失败 | en:Load library failed
#define MV_E_NOOUTBUF           0x8000000D  ///< 没有可输出的缓存 | en:No Avaliable Buffer
#define MV_E_UNKNOW             0x800000FF  ///< 未知的错误 | en:Unknown error

// GenICam系列错误:范围0x80000100-0x800001FF | en:GenICam Series Error Codes: Range from 0x80000100 to 0x800001FF
#define MV_E_GC_GENERIC         0x80000100  ///< 通用错误 | en:General error
#define MV_E_GC_ARGUMENT        0x80000101  ///< 参数非法 | en:Illegal parameters
#define MV_E_GC_RANGE           0x80000102  ///< 值超出范围 | en:The value is out of range
#define MV_E_GC_PROPERTY        0x80000103  ///< 属性 | en:Property
#define MV_E_GC_RUNTIME         0x80000104  ///< 运行环境有问题 | en:Running environment error
#define MV_E_GC_LOGICAL         0x80000105  ///< 逻辑错误 | en:Logical error
#define MV_E_GC_ACCESS          0x80000106  ///< 节点访问条件有误 | en:Node accessing condition error
#define MV_E_GC_TIMEOUT         0x80000107  ///< 超时 | en:Timeout
#define MV_E_GC_DYNAMICCAST     0x80000108  ///< 转换异常 | en:Transformation exception
#define MV_E_GC_UNKNOW          0x800001FF  ///< GenICam未知错误 | en:GenICam unknown error

//GigE_STATUS对应的错误码:范围0x80000200-0x800002FF | en:GigE_STATUS Error Codes: Range from 0x80000200 to 0x800002FF
#define MV_E_NOT_IMPLEMENTED    0x80000200  ///< 命令不被设备支持 | en:The command is not supported by device
#define MV_E_INVALID_ADDRESS    0x80000201  ///< 访问的目标地址不存在 | en:The target address being accessed does not exist
#define MV_E_WRITE_PROTECT      0x80000202  ///< 目标地址不可写 | en:The target address is not writable
#define MV_E_ACCESS_DENIED      0x80000203  ///< 设备无访问权限 | en:No permission
#define MV_E_BUSY               0x80000204  ///< 设备忙，或网络断开 | en:Device is busy, or network disconnected
#define MV_E_PACKET             0x80000205  ///< 网络包数据错误 | en:Network data packet error
#define MV_E_NETER              0x80000206  ///< 网络相关错误 | en:Network error
// GigE相机特有的错误码
#define MV_E_IP_CONFLICT        0x80000221  ///< 设备IP冲突 | en:Device IP conflict

//USB_STATUS对应的错误码:范围0x80000300-0x800003FF | en:USB_STATUS Error Codes: Range from 0x80000300 to 0x800003FF
#define MV_E_USB_READ           0x80000300      ///< 读usb出错 | en:Reading USB error
#define MV_E_USB_WRITE          0x80000301      ///< 写usb出错 | en:Writing USB error
#define MV_E_USB_DEVICE         0x80000302      ///< 设备异常 | en:Device exception
#define MV_E_USB_GENICAM        0x80000303      ///< GenICam相关错误 | en:GenICam error
#define MV_E_USB_BANDWIDTH      0x80000304      ///< 带宽不足  该错误码新增 | en:Insufficient bandwidth, this error code is newly added
#define MV_E_USB_DRIVER         0x80000305      ///< 驱动不匹配或者未装驱动 | en:Driver mismatch or unmounted drive
#define MV_E_USB_UNKNOW         0x800003FF      ///< USB未知的错误 | en:USB unknown error

//升级时对应的错误码:范围0x80000400-0x800004FF | en:Upgrade Error Codes: Range from 0x80000400 to 0x800004FF
#define MV_E_UPG_FILE_MISMATCH     0x80000400 ///< 升级固件不匹配 | en:Firmware mismatches
#define MV_E_UPG_LANGUSGE_MISMATCH 0x80000401 ///< 升级固件语言不匹配 | en:Firmware language mismatches
#define MV_E_UPG_CONFLICT          0x80000402 ///< 升级冲突（设备已经在升级了再次请求升级即返回此错误） | en:Upgrading conflicted (repeated upgrading requests during device upgrade)
#define MV_E_UPG_INNER_ERR         0x80000403 ///< 升级时相机内部出现错误 | en:Camera internal error during upgrade
#define MV_E_UPG_UNKNOW            0x800004FF ///< 升级时未知错误 | en:Unknown error during upgrade





#endif //_MV_ERROR_DEFINE_H_
