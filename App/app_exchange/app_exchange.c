/**
 * @file app_exchange.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块间数据交换层实现
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 通过 APP_EXCHANGE_DEFINE X-宏生成各类型的 publish/read 函数对，临界区内完成数据拷贝。
 */

#include "app_exchange.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ======================== 交换缓冲区 ======================== */

static app_chassis_command_t app_exchange_chassis_command;
static app_gimbal_command_t app_exchange_gimbal_command;
static app_shooter_command_t app_exchange_shooter_command;
static app_imu_snapshot_t app_exchange_imu_snapshot;
static app_gimbal_feedback_t app_exchange_gimbal_feedback;
static app_vision_target_t app_exchange_vision_target;

/* ======================== X-宏生成器 ======================== */

/**
 * @brief  为一种交换类型生成 publish / read 函数对。
 *
 * @param type     交换值的 C 类型。
 * @param suffix   函数名后缀，拼接到 `app_exchange_publish_` / `app_exchange_read_`。
 * @param storage  对应的静态缓冲区变量。
 *
 * 两个函数均在临界区内完成简单拷贝，确保对 FreeRTOS 任务而言
 * 单次读取/写入是原子的。
 */
#define APP_EXCHANGE_DEFINE(type, suffix, storage)                                           \
    void app_exchange_publish_##suffix(const type *value)                                    \
    {                                                                                         \
        if (value != NULL)                                                                    \
        {                                                                                     \
            taskENTER_CRITICAL();                                                             \
            storage = *value;                                                                 \
            taskEXIT_CRITICAL();                                                              \
        }                                                                                     \
    }                                                                                         \
    void app_exchange_read_##suffix(type *value)                                             \
    {                                                                                         \
        if (value != NULL)                                                                    \
        {                                                                                     \
            taskENTER_CRITICAL();                                                             \
            *value = storage;                                                                 \
            taskEXIT_CRITICAL();                                                              \
        }                                                                                     \
    }

/* ======================== 公共 API ======================== */

/**
 * @brief  在临界区内将所有交换缓冲区清零初始化。
 */
void app_exchange_init(void)
{
    taskENTER_CRITICAL();
    memset(&app_exchange_chassis_command, 0, sizeof(app_exchange_chassis_command));
    memset(&app_exchange_gimbal_command, 0, sizeof(app_exchange_gimbal_command));
    memset(&app_exchange_shooter_command, 0, sizeof(app_exchange_shooter_command));
    memset(&app_exchange_imu_snapshot, 0, sizeof(app_exchange_imu_snapshot));
    memset(&app_exchange_gimbal_feedback, 0, sizeof(app_exchange_gimbal_feedback));
    memset(&app_exchange_vision_target, 0, sizeof(app_exchange_vision_target));
    taskEXIT_CRITICAL();
}

/* 为每种交换类型实例化 publish / read 函数对。 */
APP_EXCHANGE_DEFINE(app_chassis_command_t, chassis_command, app_exchange_chassis_command)
APP_EXCHANGE_DEFINE(app_gimbal_command_t, gimbal_command, app_exchange_gimbal_command)
APP_EXCHANGE_DEFINE(app_shooter_command_t, shooter_command, app_exchange_shooter_command)
APP_EXCHANGE_DEFINE(app_imu_snapshot_t, imu, app_exchange_imu_snapshot)
APP_EXCHANGE_DEFINE(app_gimbal_feedback_t, gimbal_feedback, app_exchange_gimbal_feedback)
APP_EXCHANGE_DEFINE(app_vision_target_t, vision_target, app_exchange_vision_target)
