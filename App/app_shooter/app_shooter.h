/**
 * @file app_shooter.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 射击器应用模块接口 -- 摩擦轮与拨弹控制
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 定义射击器配置结构体及控制接口，管理摩擦轮启停、单发与连发控制，并发布射击器反馈。
 */

#ifndef APP_SHOOTER_H
#define APP_SHOOTER_H

#include "bsp_common.h"
#include "app_types.h"
#include "module_shooter.h"

#include <stdbool.h>

/** @brief 射击器模块的静态配置。 */
typedef struct
{
    module_shooter_t *shooter;          /**< 射击器硬件抽象。 */
} app_shooter_config_t;

/** @brief 射击器运行时实例。 */
typedef struct
{
    app_shooter_config_t config;    /**< 静态配置的副本。 */
    app_shooter_feedback_t feedback; /**< 最近一次控制周期的反馈。 */
    bool previous_fire_request;     /**< 上一帧的开火标志（用于上升沿检测）。 */
    bool initialized;               /**< 初始化阶段已成功完成。 */
} app_shooter_t;

/**
 * @brief  初始化射击器模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK。
 */
bsp_status_t app_shooter_init(app_shooter_t *me, const app_shooter_config_t *config);

/**
 * @brief  执行一个射击器控制周期。
 * @param  me            已初始化的射击器实例。
 * @param  delta_time_s  距上次调用的经过时间 [s]。
 */
bsp_status_t app_shooter_update(app_shooter_t *me,
                                const app_shooter_command_t *command,
                                const app_gimbal_feedback_t *gimbal,
                                float delta_time_s);

const app_shooter_feedback_t *app_shooter_get_feedback(const app_shooter_t *me);

#endif
