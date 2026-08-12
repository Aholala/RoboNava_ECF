/**
 * @file app_gimbal.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 云台应用模块接口 -- 俯仰/偏航双轴控制
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 定义云台配置结构体及控制接口，以角度模式驱动俯仰和偏航电机，支持编码器与 IMU 两种反馈模式。
 */

#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#include "bsp_common.h"
#include "module_board_comm.h"
#include "module_motor.h"

#include <stdbool.h>

/** @brief 云台模块的静态配置。 */
typedef struct
{
    module_motor_t *pitch_motor;        /**< 俯仰轴电机实例。 */
    module_motor_t *yaw_motor;          /**< 偏航轴电机实例。 */
    module_board_comm_t *board_comm;    /**< 可选的板间通信链路。 */
    float target_tolerance_rad;         /**< 判定目标已锁定的位置误差 [rad]。 */
} app_gimbal_config_t;

/** @brief 云台运行时实例。 */
typedef struct
{
    app_gimbal_config_t config; /**< 静态配置的副本。 */
    bool initialized;           /**< 初始化阶段已成功完成。 */
} app_gimbal_t;

/**
 * @brief  初始化云台模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK。
 */
bsp_status_t app_gimbal_init(app_gimbal_t *me, const app_gimbal_config_t *config);

/**
 * @brief  执行一个云台控制周期。
 * @param  me            已初始化的云台实例。
 * @param  delta_time_s  距上次调用的经过时间 [s]。
 */
void app_gimbal_update(app_gimbal_t *me, float delta_time_s);

#endif
