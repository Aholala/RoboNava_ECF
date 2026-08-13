/**
 * @file app_chassis.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 底盘应用模块接口 -- 麦轮/全向轮/舵轮统一底盘控制
 * @version 1.0
 * @date 2026-08-13
 * @copyright Copyright (c) 2026
 *
 * @note 定义底盘配置结构体及控制接口，同一套接口支持麦克纳姆轮、全向轮和舵轮三种底盘，类型仅在初始化时选择。
 */

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "alg_mecanum.h"
#include "alg_omni.h"
#include "alg_swerve.h"
#include "app_types.h"
#include "bsp_common.h"
#include "module_motor.h"
#include "module_swerve.h"

#include <stdbool.h>

#define APP_CHASSIS_MAX_WHEEL_COUNT (4U)

/** @brief 底盘驱动类型。 */
typedef enum
{
    APP_CHASSIS_TYPE_MECANUM = 0,  /**< 麦克纳姆轮底盘。 */
    APP_CHASSIS_TYPE_OMNI,         /**< 全向轮底盘。 */
    APP_CHASSIS_TYPE_SWERVE        /**< 舵轮底盘。 */
} app_chassis_type_t;

/** @brief 底盘模块的静态配置。 */
typedef struct
{
    app_chassis_type_t type;    /**< 底盘类型，决定 drive 联合体中哪个成员有效。 */
    float follow_gain;          /**< 跟随云台模式下的偏航角比例增益。 */
    float stop_deadband;        /**< 判定已停车的速度死区 [m/s 或 rad/s]。 */
    union
    {
        struct
        {
            alg_mecanum_t *kinematics;                              /**< 麦轮运动学实例。 */
            module_motor_t *motors[ALG_MECANUM_WHEEL_COUNT];        /**< 四轮电机，顺序：左前、右前、左后、右后。 */
        } mecanum;
        struct
        {
            alg_omni_t *kinematics;                                 /**< 全向轮运动学实例。 */
            module_motor_t *motors[APP_CHASSIS_MAX_WHEEL_COUNT];    /**< 轮电机数组，顺序与运动学配置一致。 */
            size_t wheel_count;                                     /**< 实际轮数（3 或 4）。 */
        } omni;
        struct
        {
            alg_swerve_t *kinematics;                                        /**< 舵轮运动学实例。 */
            module_swerve_t *modules[ALG_SWERVE_RECTANGULAR_MODULE_COUNT];   /**< 四模块，顺序：左前、右前、左后、右后。 */
        } swerve;
    } drive;    /**< 按 type 选择的驱动配置。 */
} app_chassis_config_t;

/** @brief 底盘运行时实例。 */
typedef struct
{
    app_chassis_config_t config;     /**< 静态配置的副本。 */
    app_chassis_feedback_t feedback; /**< 最近一次底盘反馈。 */
    bool initialized;                /**< 初始化阶段已成功完成。 */
} app_chassis_t;

/**
 * @brief  初始化底盘模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_chassis_init(app_chassis_t *me, const app_chassis_config_t *config);

/**
 * @brief  执行一个底盘控制周期。
 * @param  me            已初始化的底盘实例。
 * @param  command       命令层发布的底盘运动指令。
 * @param  delta_time_s  距上次调用的经过时间 [s]（必须 > 0）。
 * @return 成功返回 BSP_STATUS_OK，参数无效/未初始化/电机离线返回对应错误码。
 *
 * 只计算并设置电机目标；调用方需在所有 App 更新完成后对各电机总线调用
 * module_*_motor_bus_update()。
 */
bsp_status_t app_chassis_update(app_chassis_t *me,
                                const app_chassis_command_t *command,
                                float delta_time_s);

/**
 * @brief  读取最近一次底盘反馈。
 * @param  me  已初始化的底盘实例。
 * @return 只读反馈指针，实例无效时返回 NULL。
 */
const app_chassis_feedback_t *app_chassis_get_feedback(const app_chassis_t *me);

#endif
