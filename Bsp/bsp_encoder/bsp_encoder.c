/**
 * @file bsp_encoder.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 编码器抽象层实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 核心设计：通过 driver_ops 将硬件驱动与上层逻辑解耦。
 *       get_delta() 支持环形计数器 — 当 counter_modulus > 0 时自动处理
 *       溢出回绕，计算最短路径增量。
 */

#include "bsp_encoder.h"
#include <limits.h>

/* ======================== 内部辅助 ======================== */

/**
 * @brief 校验编码器对象有效性
 * @param me 编码器对象指针
 * @return BSP_STATUS_OK 有效，否则返回相应错误码
 */
static bsp_status_t validate(const bsp_encoder_t *me) {
    if (me == NULL) return BSP_STATUS_INVALID_ARGUMENT;
    return me->is_initialized ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

/* ======================== 生命周期 ======================== */

/**
 * @brief 初始化编码器实例
 * @param me 编码器对象指针
 * @param config 配置参数（包含硬件句柄、驱动操作表、计数器模数）
 * @return 执行状态
 * @note 校验所有必填字段，调用驱动 init（若存在），零初始化后填充配置字段
 */
bsp_status_t bsp_encoder_init(bsp_encoder_t *me, const bsp_encoder_config_t *config) {
    bsp_status_t status;
    // ---- 参数校验：所有必填字段均不可为空 ----
    if ((me == NULL) || (config == NULL) || (config->device_handle == NULL) ||
        (config->driver_ops == NULL) || ((config->counter_modulus != 0U) &&
        (config->counter_modulus < 2U)) || (config->driver_ops->start == NULL) ||
        (config->driver_ops->stop == NULL) || (config->driver_ops->set_count == NULL) ||
        (config->driver_ops->get_count == NULL) || (config->driver_ops->get_direction == NULL))
        return BSP_STATUS_INVALID_ARGUMENT;
    // ---- 零初始化，确保 stable state ----
    *me = (bsp_encoder_t){0};
    if (config->driver_ops->init != NULL) {
        status = config->driver_ops->init(config->device_handle);
        if (status != BSP_STATUS_OK) return status;
    }
    me->device_handle = config->device_handle; me->driver_ops = config->driver_ops;
    me->counter_modulus = config->counter_modulus; me->is_initialized = true;
    return BSP_STATUS_OK;
}

/**
 * @brief 反初始化编码器实例
 * @param me 编码器对象指针
 * @return 执行状态
 * @note 调用驱动 deinit（若存在），成功后清除初始化标志
 */
bsp_status_t bsp_encoder_deinit(bsp_encoder_t *me) {
    bsp_status_t status = validate(me); if (status != BSP_STATUS_OK) return status;
    status = me->driver_ops->deinit ? me->driver_ops->deinit(me->device_handle) : BSP_STATUS_OK;
    if (status == BSP_STATUS_OK) me->is_initialized = false; return status;
}

/* ======================== 运行控制 ======================== */

/**
 * @brief 启动/停止编码器计数（宏展开生成）
 * @note 通过 ACTION 宏为 start/stop 统一生成校验-转发的样板代码
 */
#define ACTION(name, member) bsp_status_t name(bsp_encoder_t *me) { \
    const bsp_status_t s=validate(me); return s==BSP_STATUS_OK ? me->driver_ops->member(me->device_handle):s; }
ACTION(bsp_encoder_start, start)
ACTION(bsp_encoder_stop, stop)

/* ======================== 数据访问 ======================== */

/**
 * @brief 设置编码器绝对计数值
 * @param me 编码器对象指针
 * @param count 目标计数值
 * @return 执行状态
 * @note 成功后同步更新 previous_count，保证后续 get_delta 计算的连续性
 */
bsp_status_t bsp_encoder_set_count(bsp_encoder_t *me, int32_t count) {
    const bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    const bsp_status_t result=me->driver_ops->set_count(me->device_handle,count);
    if(result==BSP_STATUS_OK)me->previous_count=count; return result;
}

/**
 * @brief 重置编码器计数值为零
 * @param me 编码器对象指针
 * @return 执行状态（委托给 set_count）
 */
bsp_status_t bsp_encoder_reset(bsp_encoder_t *me) { return bsp_encoder_set_count(me,0); }

/**
 * @brief 读取编码器当前计数值
 * @param me 编码器对象指针
 * @param count 输出参数，接收计数值
 * @return 执行状态
 */
bsp_status_t bsp_encoder_get_count(const bsp_encoder_t *me, int32_t *count) {
    const bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    return count ? me->driver_ops->get_count(me->device_handle,count):BSP_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief 计算自上次读取以来的增量（delta）
 * @param me 编码器对象指针
 * @param delta 输出参数，接收增量值
 * @return 执行状态
 * @note 环形计数器模式：通过比较差值绝对值与半模数判断最短路径方向，
 *       自动处理溢出回绕（如从 MAX 翻转到 MIN 或反向）。
 *       delta 范围受限于 INT32_MAX/MIN，超出返回 BSP_STATUS_OUT_OF_RANGE。
 */
bsp_status_t bsp_encoder_get_delta(bsp_encoder_t *me, int32_t *delta) {
    int32_t current; int64_t difference; if(delta==NULL)return BSP_STATUS_INVALID_ARGUMENT;
    const bsp_status_t s=bsp_encoder_get_count(me,&current); if(s!=BSP_STATUS_OK)return s;
    difference=(int64_t)current-me->previous_count;
    // ---- 环形计数器回绕处理 ----
    if(me->counter_modulus>0U){const int64_t m=me->counter_modulus,h=m/2;
        if(difference>h)difference-=m; else if(difference < -h)difference+=m;}
    if((difference>INT32_MAX)||(difference<INT32_MIN))return BSP_STATUS_OUT_OF_RANGE;
    *delta=(int32_t)difference; me->previous_count=current; return BSP_STATUS_OK;
}

/**
 * @brief 查询编码器当前旋转方向
 * @param me 编码器对象指针
 * @param direction 输出参数，接收方向枚举值
 * @return 执行状态
 */
bsp_status_t bsp_encoder_get_direction(const bsp_encoder_t *me,bsp_encoder_direction_t *direction){
    const bsp_status_t s=validate(me); if(s!=BSP_STATUS_OK)return s;
    return direction ? me->driver_ops->get_direction(me->device_handle,direction):BSP_STATUS_INVALID_ARGUMENT;
}
