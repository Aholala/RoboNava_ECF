/**
 * @file bsp_usb_vcp.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP USB 虚拟串口抽象层 —— 实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 实现 bsp_usb_vcp.h 中定义的驱动无关 USB VCP 接口。
 *       提供发送、接收、连接状态查询和 ISR 事件通知。
 *       get_connected 和 get_busy 共享一个公共的 query() 辅助函数。
 */
#include "bsp_usb_vcp.h"

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 验证 VCP 对象已初始化且非空
 * @param me VCP 对象指针
 * @return 有效则返回 BSP_STATUS_OK，为 NULL 则返回 BSP_STATUS_INVALID_ARGUMENT，
 *         未初始化则返回 BSP_STATUS_NOT_INITIALIZED
 */
static bsp_status_t validate(const bsp_usb_vcp_t *me){if(me==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized?BSP_STATUS_OK:BSP_STATUS_NOT_INITIALIZED;}

/* ======================== 生命周期 ======================== */

/**
 * @brief 使用给定配置初始化 USB VCP 实例
 * @param me VCP 对象指针（入口时需清零）
 * @param config 配置参数（设备句柄、驱动操作表、可选回调）
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 * @note 必需的驱动操作：transmit、receive。首先调用可选的驱动初始化钩子，
 *       然后装配对象字段。
 */
bsp_status_t bsp_usb_vcp_init(bsp_usb_vcp_t *me,const bsp_usb_vcp_config_t *config){bsp_status_t status;
    if((me==NULL)||(config==NULL)||(config->device_handle==NULL)||(config->driver_ops==NULL)||
       (config->driver_ops->transmit==NULL)||(config->driver_ops->receive==NULL))return BSP_STATUS_INVALID_ARGUMENT;
    *me=(bsp_usb_vcp_t){0};if(config->driver_ops->init!=NULL){status=config->driver_ops->init(config->device_handle);
        if(status!=BSP_STATUS_OK)return status;}me->device_handle=config->device_handle;
    me->driver_ops=config->driver_ops;me->callback=config->callback;me->user_context=config->user_context;
    me->is_initialized=true;return BSP_STATUS_OK;}

/**
 * @brief 反初始化 USB VCP 实例并释放硬件资源
 * @param me VCP 对象指针
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 */
bsp_status_t bsp_usb_vcp_deinit(bsp_usb_vcp_t *me){bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    s=me->driver_ops->deinit?me->driver_ops->deinit(me->device_handle):BSP_STATUS_OK;
    if(s==BSP_STATUS_OK)me->is_initialized=false;return s;}

/* ======================== 回调配置 ======================== */

/**
 * @brief 设置或替换 VCP 事件回调及其用户上下文
 * @param me VCP 对象指针
 * @param callback 回调函数指针（可为 NULL 以禁用）
 * @param context 传递给回调的不透明用户上下文
 * @return 成功返回 BSP_STATUS_OK
 */
bsp_status_t bsp_usb_vcp_set_callback(bsp_usb_vcp_t *me,bsp_event_callback_t callback,void *context){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;me->callback=callback;
    me->user_context=context;return BSP_STATUS_OK;}

/* ======================== 发送 / 接收 ======================== */

/**
 * @brief 通过 USB 虚拟串口发送数据
 * @param me VCP 对象指针
 * @param data 指向发送缓冲区的指针（不能为 NULL）
 * @param size 要发送的字节数（必须 > 0）
 * @param timeout 超时时间（毫秒）
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 */
bsp_status_t bsp_usb_vcp_transmit(bsp_usb_vcp_t *me,const uint8_t *data,size_t size,uint32_t timeout){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if((data==NULL)||(size==0U))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->transmit(me->device_handle,data,size,timeout);}

/**
 * @brief 从 USB 虚拟串口接收数据
 * @param me VCP 对象指针
 * @param data 指向接收缓冲区的指针（不能为 NULL）
 * @param capacity 最大接收字节数（必须 > 0）
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 * @note 实际接收的字节数通过事件回调通知。
 */
bsp_status_t bsp_usb_vcp_receive(bsp_usb_vcp_t *me,uint8_t *data,size_t capacity){
    const bsp_status_t s=validate(me);if(s!=BSP_STATUS_OK)return s;
    if((data==NULL)||(capacity==0U))return BSP_STATUS_INVALID_ARGUMENT;
    return me->driver_ops->receive(me->device_handle,data,capacity);}

/* ======================== 控制 ======================== */

/**
 * @brief 中止任何进行中的 USB VCP 传输
 * @param me VCP 对象指针
 * @return 成功返回 BSP_STATUS_OK，驱动未实现 abort 则返回
 *         BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_usb_vcp_abort(bsp_usb_vcp_t *me){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;return me->driver_ops->abort?me->driver_ops->abort(me->device_handle)
        :BSP_STATUS_UNSUPPORTED;}

/* ======================== 状态查询 ======================== */

/**
 * @brief 公共查询辅助函数：验证对象，然后委托给可选的驱动操作
 * @param me VCP 对象指针（const）
 * @param value 布尔结果的输出指针
 * @param operation 要调用的驱动操作（可为 NULL）
 * @return 成功返回 BSP_STATUS_OK，operation 为 NULL 则返回 BSP_STATUS_UNSUPPORTED
 */
static bsp_status_t query(const bsp_usb_vcp_t *me,bool *value,
    bsp_status_t (*operation)(const void *,bool *)){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s;if(value==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    return operation?operation(me->device_handle,value):BSP_STATUS_UNSUPPORTED;}

/**
 * @brief 查询 USB 主机是否已连接
 * @param me VCP 对象指针（const）
 * @param connected 输出指针，USB 主机已连接则设为 true
 * @return 成功返回 BSP_STATUS_OK，驱动未实现 get_connected 则返回
 *         BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_usb_vcp_get_connected(const bsp_usb_vcp_t *me,bool *connected){
    return query(me,connected,(me!=NULL&&me->driver_ops!=NULL)?me->driver_ops->get_connected:NULL);}

/**
 * @brief 查询 USB VCP 外设当前是否忙碌
 * @param me VCP 对象指针（const）
 * @param busy 输出指针，传输进行中则设为 true
 * @return 成功返回 BSP_STATUS_OK，驱动未实现 get_busy 则返回
 *         BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_usb_vcp_get_busy(const bsp_usb_vcp_t *me,bool *busy){
    return query(me,busy,(me!=NULL&&me->driver_ops!=NULL)?me->driver_ops->get_busy:NULL);}

/* ======================== 中断服务通知 ======================== */

/**
 * @brief 通知 VCP 对象发生了传输事件（由 ISR 调用）
 * @param me VCP 对象指针
 * @param event 事件类型（发送完成、接收完成等）
 * @param status 传输状态
 * @param size 传输的字节数
 * @note 由驱动层的 USB 中断处理函数调用。如果注册了用户回调则调用之。
 */
void bsp_usb_vcp_notify(bsp_usb_vcp_t *me,bsp_event_t event,bsp_status_t status,size_t size){
    if((me!=NULL)&&me->is_initialized&&(me->callback!=NULL))me->callback(event,status,size,me->user_context);}
