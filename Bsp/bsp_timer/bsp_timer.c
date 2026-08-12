/**
 * @file bsp_timer.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief BSP 硬件定时器抽象层 —— 实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 实现 bsp_timer.h 中定义的驱动无关定时器接口。
 *       所有公共函数在委托给驱动操作表之前验证对象状态。
 *       启动/停止使用共享的 ACTION 宏以减少样板代码。
 */
#include "bsp_timer.h"

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 验证定时器对象已初始化且非空
 * @param me 定时器对象指针
 * @return 有效则返回 BSP_STATUS_OK，为 NULL 则返回 BSP_STATUS_INVALID_ARGUMENT，
 *         未初始化则返回 BSP_STATUS_NOT_INITIALIZED
 */
static bsp_status_t validate(const bsp_timer_t *me) {
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/* ======================== 生命周期 ======================== */

/**
 * @brief 使用给定配置初始化定时器实例
 * @param me 定时器对象指针（入口时需清零）
 * @param config 配置参数（设备句柄、驱动操作表、可选回调）
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 * @note 首先调用可选的驱动初始化钩子，然后装配对象字段。
 */
bsp_status_t bsp_timer_init(bsp_timer_t *me,const bsp_timer_config_t *config){
    bsp_status_t status;
    if((me==NULL)||(config==NULL)||(config->device_handle==NULL)||(config->driver_ops==NULL)||
       (config->driver_ops->start==NULL)||(config->driver_ops->stop==NULL)||
       (config->driver_ops->set_counter==NULL)||(config->driver_ops->get_counter==NULL)||
       (config->driver_ops->set_period==NULL)||(config->driver_ops->get_period==NULL)||
       (config->driver_ops->get_frequency==NULL))return BSP_STATUS_INVALID_ARGUMENT;
    *me=(bsp_timer_t){0};
    if(config->driver_ops->init!=NULL){status=config->driver_ops->init(config->device_handle);
        if(status!=BSP_STATUS_OK)return status;}
    me->device_handle=config->device_handle; me->driver_ops=config->driver_ops;
    me->callback=config->callback; me->user_context=config->user_context; me->is_initialized=true;
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化定时器实例并释放硬件资源
 * @param me 定时器对象指针
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 */
bsp_status_t bsp_timer_deinit(bsp_timer_t *me){
    bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    s=me->driver_ops->deinit?me->driver_ops->deinit(me->device_handle):BSP_STATUS_OK;
    if(s==BSP_STATUS_OK)me->is_initialized=false; return s;
}

/* ======================== 回调配置 ======================== */

/**
 * @brief 设置或替换定时器溢出回调及其用户上下文
 * @param me 定时器对象指针
 * @param callback 回调函数指针（可为 NULL 以禁用）
 * @param context 传递给回调的不透明用户上下文
 * @return 成功返回 BSP_STATUS_OK
 */
bsp_status_t bsp_timer_set_callback(bsp_timer_t *me,bsp_timer_callback_t callback,void *context){
    const bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    me->callback=callback; me->user_context=context; return BSP_STATUS_OK;
}

/* ======================== 控制（启动 / 停止） ======================== */

/** @brief 生成简单的"验证后委托"控制函数的宏 */
#define ACTION(name,member) /** @brief 详见 bsp_timer.h */ \
    bsp_status_t name(bsp_timer_t *me){const bsp_status_t s=validate(me); \
    return s==BSP_STATUS_OK?me->driver_ops->member(me->device_handle):s;}
ACTION(bsp_timer_start,start)
ACTION(bsp_timer_stop,stop)

/* ======================== 计数器访问 ======================== */

/**
 * @brief 将定时器计数器设置为指定值
 * @param me 定时器对象指针
 * @param value 要设置的计数值
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 */
bsp_status_t bsp_timer_set_counter(bsp_timer_t *me,uint32_t value){const bsp_status_t s=validate(me);
    return s==BSP_STATUS_OK?me->driver_ops->set_counter(me->device_handle,value):s;}

/**
 * @brief 将定时器计数器重置为零
 * @param me 定时器对象指针
 * @return 成功返回 BSP_STATUS_OK，否则返回驱动特定错误码
 * @note 对 bsp_timer_set_counter(me, 0) 的便捷封装。
 */
bsp_status_t bsp_timer_reset(bsp_timer_t *me){return bsp_timer_set_counter(me,0U);}

/**
 * @brief 读取当前定时器计数值
 * @param me 定时器对象指针（const）
 * @param value 计数值的输出指针
 * @return 成功返回 BSP_STATUS_OK，value 为 NULL 则返回 BSP_STATUS_INVALID_ARGUMENT
 */
bsp_status_t bsp_timer_get_counter(const bsp_timer_t *me,uint32_t *value){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s; return value?me->driver_ops->get_counter(me->device_handle,value):BSP_STATUS_INVALID_ARGUMENT;}

/* ======================== 周期配置 ======================== */

/**
 * @brief 设置定时器自动重载周期
 * @param me 定时器对象指针
 * @param value 要设置的周期值（必须 > 0）
 * @return 成功返回 BSP_STATUS_OK，value 为零则返回 BSP_STATUS_OUT_OF_RANGE
 */
bsp_status_t bsp_timer_set_period(bsp_timer_t *me,uint32_t value){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s; return value?me->driver_ops->set_period(me->device_handle,value):BSP_STATUS_OUT_OF_RANGE;}

/**
 * @brief 获取当前定时器自动重载周期
 * @param me 定时器对象指针（const）
 * @param value 周期值的输出指针
 * @return 成功返回 BSP_STATUS_OK，value 为 NULL 则返回 BSP_STATUS_INVALID_ARGUMENT
 */
bsp_status_t bsp_timer_get_period(const bsp_timer_t *me,uint32_t *value){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s; return value?me->driver_ops->get_period(me->device_handle,value):BSP_STATUS_INVALID_ARGUMENT;}

/**
 * @brief 获取定时器时钟频率（Hz）
 * @param me 定时器对象指针（const）
 * @param value 频率值（Hz）的输出指针
 * @return 成功返回 BSP_STATUS_OK，value 为 NULL 则返回 BSP_STATUS_INVALID_ARGUMENT
 */
bsp_status_t bsp_timer_get_frequency(const bsp_timer_t *me,uint32_t *value){const bsp_status_t s=validate(me);
    if(s!=BSP_STATUS_OK)return s; return value?me->driver_ops->get_frequency(me->device_handle,value):BSP_STATUS_INVALID_ARGUMENT;}

/* ======================== 中断服务通知 ======================== */

/**
 * @brief 通知定时器对象发生了溢出事件（由 ISR 调用）
 * @param me 定时器对象指针
 * @note 由驱动层的中断处理函数调用。如果注册了用户回调则调用之。
 *       可安全使用未初始化或 NULL 定时器调用（守卫检查会静默返回）。
 */
void bsp_timer_notify_elapsed(bsp_timer_t *me){if((me!=NULL)&&me->is_initialized&&(me->callback!=NULL))
    me->callback(me,me->user_context);}
