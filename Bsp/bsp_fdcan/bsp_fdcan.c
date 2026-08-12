/**
 * @file bsp_fdcan.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief FDCAN 抽象层实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 统一 Classic CAN 和 CAN FD 的操作接口。
 *       帧校验 (length_ok / frame_ok) 覆盖 Classic 和 FD 两种模式的
 *       合法数据长度、标识符范围及格式一致性约束。
 */

#include "bsp_fdcan.h"

/* ======================== 内部校验 ======================== */

/**
 * @brief 校验 FDCAN 对象有效性
 * @param m FDCAN 对象指针
 * @return BSP_STATUS_OK 有效，否则返回相应错误码
 */
static bsp_status_t validate(const bsp_fdcan_t *m){if(!m)return BSP_STATUS_INVALID_ARGUMENT;return m->is_initialized?BSP_STATUS_OK:BSP_STATUS_NOT_INITIALIZED;}

/**
 * @brief 校验数据长度合法性
 * @param n 数据长度（字节）
 * @return true 合法（Classic 0-8 或 FD 12/16/20/24/32/48/64）
 * @note CAN FD 仅支持特定 DLC 编码对应的长度值
 */
static bool length_ok(uint8_t n){return n<=8||n==12||n==16||n==20||n==24||n==32||n==48||n==64;}

/**
 * @brief 校验帧结构合法性
 * @param f 帧指针
 * @return true 所有字段合法
 * @note 检查：数据长度有效、标识符不超出 ID 类型范围、帧类型合法、
 *       Classic 模式下数据长度不超过 8 字节
 */
static bool frame_ok(const bsp_fdcan_frame_t*f){return f&&length_ok(f->data_length)&&
 ((f->id_type==BSP_CAN_ID_STANDARD&&f->identifier<=0x7FFU)||(f->id_type==BSP_CAN_ID_EXTENDED&&f->identifier<=0x1FFFFFFFU))&&
 (f->frame_type==BSP_CAN_FRAME_DATA||f->frame_type==BSP_CAN_FRAME_REMOTE)&&
 (f->format==BSP_FDCAN_FORMAT_CLASSIC||f->format==BSP_FDCAN_FORMAT_FD_NO_BRS||f->format==BSP_FDCAN_FORMAT_FD_BRS)&&
 (f->format!=BSP_FDCAN_FORMAT_CLASSIC||f->data_length<=8);}

/* ======================== 生命周期 ======================== */

/**
 * @brief 初始化 FDCAN 实例
 * @param m FDCAN 对象指针
 * @param c 配置参数
 * @return 执行状态
 * @note 校验所有必填字段后零初始化，调用驱动 init（若存在），
 *       填充 device_handle / driver_ops / callback / user_context
 */
bsp_status_t bsp_fdcan_init(bsp_fdcan_t*m,const bsp_fdcan_config_t*c){bsp_status_t s;if(!m||!c||!c->device_handle||!c->driver_ops||!c->driver_ops->start||!c->driver_ops->stop||!c->driver_ops->configure_filter||!c->driver_ops->transmit||!c->driver_ops->receive)return BSP_STATUS_INVALID_ARGUMENT;*m=(bsp_fdcan_t){0};if(c->driver_ops->init){s=c->driver_ops->init(c->device_handle);if(s!=BSP_STATUS_OK)return s;}m->device_handle=c->device_handle;m->driver_ops=c->driver_ops;m->callback=c->callback;m->user_context=c->user_context;m->is_initialized=true;return BSP_STATUS_OK;}

/**
 * @brief 反初始化 FDCAN 实例
 * @param m FDCAN 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_fdcan_deinit(bsp_fdcan_t*m){bsp_status_t s=validate(m);if(s!=BSP_STATUS_OK)return s;s=m->driver_ops->deinit?m->driver_ops->deinit(m->device_handle):BSP_STATUS_OK;if(s==BSP_STATUS_OK)m->is_initialized=false;return s;}

/* ======================== 回调与运行控制 ======================== */

/**
 * @brief 设置事件回调函数
 * @param m FDCAN 对象指针
 * @param cb 回调函数指针
 * @param ctx 用户上下文
 * @return 执行状态
 */
bsp_status_t bsp_fdcan_set_callback(bsp_fdcan_t*m,bsp_event_callback_t cb,void*ctx){bsp_status_t s=validate(m);if(s!=BSP_STATUS_OK)return s;m->callback=cb;m->user_context=ctx;return BSP_STATUS_OK;}

/**
 * @brief 启动 FDCAN 外设
 * @param m FDCAN 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_fdcan_start(bsp_fdcan_t*m){bsp_status_t s=validate(m);return s==BSP_STATUS_OK?m->driver_ops->start(m->device_handle):s;}

/**
 * @brief 停止 FDCAN 外设
 * @param m FDCAN 对象指针
 * @return 执行状态
 */
bsp_status_t bsp_fdcan_stop(bsp_fdcan_t*m){bsp_status_t s=validate(m);return s==BSP_STATUS_OK?m->driver_ops->stop(m->device_handle):s;}

/* ======================== 滤波器配置 ======================== */

/**
 * @brief 配置硬件接收滤波器
 * @param m FDCAN 对象指针
 * @param f 滤波器配置（ID + Mask + FIFO 选择）
 * @return 执行状态
 * @note 校验滤波器参数合法性：FIFO 选择、ID 类型与标识符/掩码范围一致性
 */
bsp_status_t bsp_fdcan_configure_filter(bsp_fdcan_t*m,const bsp_can_filter_t*f){bsp_status_t s=validate(m);if(s!=BSP_STATUS_OK)return s;if(!f||((f->receive_fifo!=BSP_CAN_RX_FIFO_0)&&(f->receive_fifo!=BSP_CAN_RX_FIFO_1))||!((f->id_type==BSP_CAN_ID_STANDARD&&f->identifier<=0x7FFU&&f->mask<=0x7FFU)||(f->id_type==BSP_CAN_ID_EXTENDED&&f->identifier<=0x1FFFFFFFU&&f->mask<=0x1FFFFFFFU)))return BSP_STATUS_INVALID_ARGUMENT;return m->driver_ops->configure_filter(m->device_handle,f);}

/* ======================== 数据收发 ======================== */

/**
 * @brief 发送 FDCAN 帧
 * @param m FDCAN 对象指针
 * @param f 待发送的帧（Classic 或 FD）
 * @param t 超时时间（ms）
 * @return 执行状态
 * @note 发送前通过 frame_ok() 校验帧合法性
 */
bsp_status_t bsp_fdcan_transmit(bsp_fdcan_t*m,const bsp_fdcan_frame_t*f,uint32_t t){bsp_status_t s=validate(m);if(s!=BSP_STATUS_OK)return s;return frame_ok(f)?m->driver_ops->transmit(m->device_handle,f,t):BSP_STATUS_OUT_OF_RANGE;}

/**
 * @brief 接收 FDCAN 帧
 * @param m FDCAN 对象指针
 * @param q 接收 FIFO 选择
 * @param f 输出参数，接收到的帧
 * @return 执行状态
 * @note 接收后通过 frame_ok() 校验帧合法性，非法帧返回 BSP_STATUS_IO_ERROR
 */
bsp_status_t bsp_fdcan_receive(bsp_fdcan_t*m,bsp_can_receive_fifo_t q,bsp_fdcan_frame_t*f){bsp_status_t s=validate(m);if(s!=BSP_STATUS_OK)return s;if(!f||(q!=BSP_CAN_RX_FIFO_0&&q!=BSP_CAN_RX_FIFO_1))return BSP_STATUS_INVALID_ARGUMENT;s=m->driver_ops->receive(m->device_handle,q,f);return s!=BSP_STATUS_OK?s:(frame_ok(f)?BSP_STATUS_OK:BSP_STATUS_IO_ERROR);}

/* ======================== 状态查询 ======================== */

/**
 * @brief 读取 FDCAN 协议状态
 * @param m FDCAN 对象指针
 * @param p 输出参数，协议状态结构体
 * @return 执行状态；驱动不支持时返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_fdcan_get_protocol_status(const bsp_fdcan_t*m,bsp_fdcan_protocol_status_t*p){bsp_status_t s=validate(m);if(s!=BSP_STATUS_OK)return s;if(!p)return BSP_STATUS_INVALID_ARGUMENT;return m->driver_ops->get_protocol_status?m->driver_ops->get_protocol_status(m->device_handle,p):BSP_STATUS_UNSUPPORTED;}

/**
 * @brief 读取发送缓冲区空闲级别
 * @param m FDCAN 对象指针
 * @param l 输出参数，空闲槽位数
 * @return 执行状态；驱动不支持时返回 BSP_STATUS_UNSUPPORTED
 */
bsp_status_t bsp_fdcan_get_transmit_free_level(const bsp_fdcan_t*m,uint32_t*l){bsp_status_t s=validate(m);if(s!=BSP_STATUS_OK)return s;if(!l)return BSP_STATUS_INVALID_ARGUMENT;return m->driver_ops->get_transmit_free_level?m->driver_ops->get_transmit_free_level(m->device_handle,l):BSP_STATUS_UNSUPPORTED;}

/* ======================== 事件通知 ======================== */

/**
 * @brief 事件通知入口 — 由驱动层触发，转发给用户回调
 * @param m FDCAN 对象指针
 * @param e 事件类型
 * @param s 事件关联状态
 * @param n 传输字节数
 * @note 内部做非空和初始化检查后安全调用用户回调
 */
void bsp_fdcan_notify(bsp_fdcan_t*m,bsp_event_t e,bsp_status_t s,size_t n){if(m&&m->is_initialized&&m->callback)m->callback(e,s,n,m->user_context);}
