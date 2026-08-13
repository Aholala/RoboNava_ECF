/**
 * @file app_command.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 命令应用模块接口 -- 遥控器解码与高层指令生成
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 定义命令模块配置结构体及接口，解码 DR16/板间通信遥控数据并生成底盘、云台、射击器指令。
 */

#ifndef APP_COMMAND_H
#define APP_COMMAND_H

#include "bsp_common.h"
#include "app_types.h"

#include <stdbool.h>

/** @brief 命令模块的静态配置。 */
typedef struct
{
    int16_t channel_maximum_offset;             /**< 去中值通道的绝对值上限，DR16 通常为 660。 */
    float maximum_yaw_rate_rad_per_s;           /**< 云台偏航最大角速率 [rad/s]。 */
    float maximum_pitch_rate_rad_per_s;         /**< 云台俯仰最大角速率 [rad/s]。 */
    float minimum_pitch_rad;                    /**< 俯仰角下限 [rad]。 */
    float maximum_pitch_rad;                    /**< 俯仰角上限 [rad]。 */
    float maximum_chassis_velocity_m_per_s;     /**< 底盘最大平移速度 [m/s]。 */
    float maximum_chassis_spin_rad_per_s;       /**< 自旋模式下底盘最大偏航角速率 [rad/s]。 */
} app_command_config_t;

/**
 * @brief  初始化命令模块（单例）。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK。
 */
bsp_status_t app_command_init(const app_command_config_t *config);

/**
 * @brief  执行一个命令解码周期。
 * @param  delta_time_s  距上次调用的经过时间 [s]。
 */
void app_command_update(const app_remote_input_t *remote, float delta_time_s);

#endif
