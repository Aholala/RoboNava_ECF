/**
 * @file app_chassis.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 底盘应用模块接口 -- 全向轮系（Swerve）驱动
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 定义底盘配置结构体、运行时实例及控制接口，支持普通、自旋、跟随云台三种驱动模式与停车自锁。
 */

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "alg_swerve.h"
#include "app_types.h"
#include "bsp_common.h"
#include "module_swerve.h"

#include <stdbool.h>

/** @brief 底盘模块的静态配置。 */
typedef struct
{
    alg_swerve_t *kinematics;                                         /**< 轮系运动学模型。 */
    module_swerve_t *modules[ALG_SWERVE_RECTANGULAR_MODULE_COUNT];    /**< 四个舵轮模块实例。 */
    float follow_gain;                                                /**< 跟随云台模式的增益系数。 */
    float stop_deadband;                                              /**< 判定停车的速度死区 [m/s 或 rad/s]。 */
} app_chassis_config_t;

/** @brief 底盘运行时实例。 */
typedef struct
{
    app_chassis_config_t config; /**< 静态配置的副本。 */
    app_chassis_feedback_t feedback; /**< 最近一次控制周期的反馈。 */
    bool initialized;            /**< 初始化阶段已成功完成。 */
} app_chassis_t;

/**
 * @brief  初始化底盘模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK。
 */
bsp_status_t app_chassis_init(app_chassis_t *me, const app_chassis_config_t *config);

/**
 * @brief  执行一个底盘控制周期。
 * @param  me            已初始化的底盘实例。
 * @param  delta_time_s  距上次调用的经过时间 [s]。
 */
bsp_status_t app_chassis_update(app_chassis_t *me,
                                const app_chassis_command_t *command,
                                float delta_time_s);

const app_chassis_feedback_t *app_chassis_get_feedback(const app_chassis_t *me);

#endif
