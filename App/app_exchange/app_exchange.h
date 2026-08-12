/**
 * @file app_exchange.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块间数据交换层接口（发布/读取模式）
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 为每类数据结构提供临界区保护的单元素缓冲区，生产者/消费者通过 publish/read 函数对交换数据。
 */

#ifndef APP_EXCHANGE_H
#define APP_EXCHANGE_H

#include "app_types.h"

/** @brief 将所有交换缓冲区清零初始化。 */
void app_exchange_init(void);

/** @brief 发布底盘指令（生产者侧）。 */
void app_exchange_publish_chassis_command(const app_chassis_command_t *command);
/** @brief 读取最新的底盘指令（消费者侧）。 */
void app_exchange_read_chassis_command(app_chassis_command_t *command);

/** @brief 发布云台指令。 */
void app_exchange_publish_gimbal_command(const app_gimbal_command_t *command);
/** @brief 读取最新的云台指令。 */
void app_exchange_read_gimbal_command(app_gimbal_command_t *command);

/** @brief 发布射击器指令。 */
void app_exchange_publish_shooter_command(const app_shooter_command_t *command);
/** @brief 读取最新的射击器指令。 */
void app_exchange_read_shooter_command(app_shooter_command_t *command);

/** @brief 发布 IMU 姿态快照。 */
void app_exchange_publish_imu(const app_imu_snapshot_t *snapshot);
/** @brief 读取最新的 IMU 快照。 */
void app_exchange_read_imu(app_imu_snapshot_t *snapshot);

/** @brief 发布云台反馈。 */
void app_exchange_publish_gimbal_feedback(const app_gimbal_feedback_t *feedback);
/** @brief 读取最新的云台反馈。 */
void app_exchange_read_gimbal_feedback(app_gimbal_feedback_t *feedback);

/** @brief 发布视觉跟踪目标。 */
void app_exchange_publish_vision_target(const app_vision_target_t *target);
/** @brief 读取最新的视觉目标。 */
void app_exchange_read_vision_target(app_vision_target_t *target);

#endif
