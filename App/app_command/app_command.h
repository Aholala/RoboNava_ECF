/**
 * @file app_command.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 命令应用模块接口 -- 遥控输入到底盘/云台/发射命令的转换
 * @version 1.0
 * @date 2026-08-13
 * @copyright Copyright (c) 2026
 *
 * @note 定义命令配置与输出结构体，把与设备无关的遥控输入转换为底盘、云台和发射命令。
 */

#ifndef APP_COMMAND_H
#define APP_COMMAND_H

#include "app_types.h"
#include "bsp_common.h"

#include <stdbool.h>

/** @brief 命令模块的静态配置。 */
typedef struct
{
    int16_t channel_maximum_offset;         /**< 通道量程（减中值后的最大值），典型 660。 */
    float maximum_yaw_rate_rad_per_s;       /**< 偏航角速率上限 [rad/s]。 */
    float maximum_pitch_rate_rad_per_s;     /**< 俯仰角速率上限 [rad/s]。 */
    float minimum_pitch_rad;                /**< 俯仰角下限 [rad]。 */
    float maximum_pitch_rad;                /**< 俯仰角上限 [rad]。 */
    float maximum_chassis_velocity_m_per_s; /**< 底盘平移速度上限 [m/s]。 */
    float maximum_chassis_spin_rad_per_s;   /**< 底盘自旋角速率上限 [rad/s]。 */
    float friction_velocity_rad_per_s;      /**< 摩擦轮目标转速 [rad/s]。 */
} app_command_config_t;

/** @brief 命令模块每周期发布的输出指令。 */
typedef struct
{
    app_chassis_command_t chassis;        /**< 底盘运动指令。 */
    app_gimbal_command_t gimbal;          /**< 云台运动指令。 */
    app_shooter_command_t shooter;        /**< 射击器指令。 */
    bool automatic_vision_requested;      /**< 请求切换视觉自动跟踪模式。 */
} app_command_output_t;

/** @brief 命令模块运行时实例。 */
typedef struct
{
    app_command_config_t config;    /**< 静态配置的副本。 */
    app_command_output_t output;    /**< 最近一次发布的输出指令。 */
    float yaw_target_rad;           /**< 累积的偏航目标角 [rad]。 */
    float pitch_target_rad;         /**< 累积的俯仰目标角 [rad]。 */
    uint32_t sequence;              /**< 单调递增的帧序号。 */
    bool initialized;               /**< 初始化阶段已成功完成。 */
} app_command_t;

/**
 * @brief  初始化命令模块。
 * @param  me      指向调用方分配的实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_command_init(app_command_t *me, const app_command_config_t *config);

/**
 * @brief  执行一个命令转换周期。
 * @param  me               已初始化的命令实例。
 * @param  remote           通用遥控输入，可为 NULL（视为失联）。
 * @param  gimbal_feedback  云台反馈，可为 NULL。
 * @param  vision_target    视觉目标，可为 NULL。
 * @param  delta_time_s     距上次调用的经过时间 [s]（必须 > 0）。
 * @return 成功返回 BSP_STATUS_OK，参数无效/未初始化返回对应错误码。
 */
bsp_status_t app_command_update(app_command_t *me,
                                const app_remote_input_t *remote,
                                const app_gimbal_feedback_t *gimbal_feedback,
                                const app_vision_target_t *vision_target,
                                float delta_time_s);

/**
 * @brief  读取最近一次发布的输出指令。
 * @param  me  已初始化的命令实例。
 * @return 只读输出指针，实例无效时返回 NULL。
 */
const app_command_output_t *app_command_get_output(const app_command_t *me);

#endif
