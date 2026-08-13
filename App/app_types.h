/**
 * @file app_types.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 应用层共享类型定义
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 集中定义应用层各模块间交换的命令、反馈、快照及目标结构体与配套枚举。
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 与具体遥控器和板间协议无关的三段开关状态。 */
typedef enum
{
    APP_SWITCH_INVALID = 0,
    APP_SWITCH_UP,
    APP_SWITCH_DOWN,
    APP_SWITCH_MIDDLE
} app_switch_t;

/** @brief 项目适配层提供的通用遥控输入。 */
typedef struct
{
    int16_t channel[4];      /**< 原始通道减中值后的整数，DR16 典型为 [-660, 660]。 */
    app_switch_t left_switch;
    app_switch_t right_switch;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    bool mouse_left_pressed;
    bool mouse_right_pressed;
    uint16_t keyboard;
    int16_t dial;            /**< 拨轮原始值减中值后的整数。 */
    uint32_t sequence;
    bool online;
} app_remote_input_t;

/** @brief 底盘驱动模式选择。 */
typedef enum
{
    APP_CHASSIS_MODE_NO_FORCE = 0,  /**< 无动力输出（禁用）。 */
    APP_CHASSIS_MODE_NORMAL,        /**< 标准速度控制模式。 */
    APP_CHASSIS_MODE_SPIN,          /**< 以最大速率持续旋转。 */
    APP_CHASSIS_MODE_FOLLOW_GIMBAL  /**< 偏航角跟随云台朝向。 */
} app_chassis_mode_t;

/** @brief 云台反馈源选择。 */
typedef enum
{
    APP_GIMBAL_FEEDBACK_ENCODER = 0, /**< 基于电机编码器的反馈。 */
    APP_GIMBAL_FEEDBACK_IMU          /**< IMU 姿态融合反馈。 */
} app_gimbal_feedback_mode_t;

/** @brief 命令层发布的底盘运动指令。 */
typedef struct
{
    float velocity_x_m_per_s;           /**< 期望前向速度 [m/s]。 */
    float velocity_y_m_per_s;           /**< 期望横向速度 [m/s]。 */
    float angular_velocity_rad_per_s;   /**< 期望偏航角速率 [rad/s]。 */
    float gimbal_yaw_rad;               /**< 当前云台偏航角 [rad]。 */
    app_chassis_mode_t mode;            /**< 底盘驱动模式。 */
    bool self_lock_when_stopped;        /**< 速度为零时启用自锁。 */
    bool enabled;                       /**< 指令有效标志。 */
    uint32_t sequence;                  /**< 单调递增的帧序号。 */
} app_chassis_command_t;

/** @brief 命令层发布的云台运动指令。 */
typedef struct
{
    float yaw_target_rad;                    /**< 期望偏航角 [rad]。 */
    float pitch_target_rad;                  /**< 期望俯仰角 [rad]。 */
    app_gimbal_feedback_mode_t feedback_mode; /**< 选定的反馈源。 */
    bool enabled;                            /**< 指令有效标志。 */
    uint32_t sequence;                       /**< 单调递增的帧序号。 */
} app_gimbal_command_t;

/** @brief 命令层发布的射击器指令。 */
typedef struct
{
    bool friction_enabled;                  /**< 摩擦轮正在旋转。 */
    bool fire_requested;                    /**< 单发触发（上升沿检测）。 */
    bool automatic_fire_enabled;            /**< 跟踪时连续发射。 */
    float friction_velocity_rad_per_s;      /**< 摩擦轮目标转速 [rad/s]。 */
    uint32_t sequence;                      /**< 单调递增的帧序号。 */
} app_shooter_command_t;

/** @brief 周期性发布的 IMU 姿态快照。 */
typedef struct
{
    float yaw_rad;                          /**< 欧拉偏航角 [rad]。 */
    float pitch_rad;                        /**< 欧拉俯仰角 [rad]。 */
    float roll_rad;                         /**< 欧拉横滚角 [rad]。 */
    float angular_velocity_rad_per_s[3];    /**< 陀螺仪角速率 (x, y, z) [rad/s]。 */
    uint32_t sample_count;                  /**< 累计传感器采样计数。 */
    bool valid;                             /**< 姿态估计有效标志。 */
    float continuous_yaw_rad;               /**< Continuous yaw without +/-pi wrapping [rad]. */
} app_imu_snapshot_t;

/** @brief 每控制周期发布的云台反馈。 */
typedef struct
{
    float yaw_rad;                  /**< 实测偏航角 [rad]。 */
    float pitch_rad;                /**< 实测俯仰角 [rad]。 */
    float yaw_velocity_rad_per_s;   /**< 实测偏航角速率 [rad/s]。 */
    float pitch_velocity_rad_per_s; /**< 实测俯仰角速率 [rad/s]。 */
    bool motors_online;             /**< 全部云台电机在线。 */
    bool target_locked;             /**< 位置误差在容差范围内。 */
    bool imu_valid;                 /**< 本周期 IMU 姿态数据有效。 */
} app_gimbal_feedback_t;

/** @brief 每控制周期发布的底盘反馈。 */
typedef struct
{
    float velocity_x_m_per_s;           /**< 指令前向速度 [m/s]。 */
    float velocity_y_m_per_s;           /**< 指令横向速度 [m/s]。 */
    float angular_velocity_rad_per_s;   /**< 指令偏航角速率 [rad/s]。 */
    app_chassis_mode_t mode;            /**< 当前驱动模式。 */
    bool self_lock_active;              /**< 自锁配置已激活。 */
    bool motors_online;                 /**< 全部底盘模块在线。 */
} app_chassis_feedback_t;

/** @brief 每控制周期发布的射击器反馈。 */
typedef struct
{
    uint8_t state;              /**< 射击器状态机当前状态。 */
    uint8_t jam_retry_count;    /**< 连续卡弹重试次数。 */
    bool friction_ready;        /**< 摩擦轮已达到目标转速。 */
    bool fire_permission;       /**< 裁判系统/安全开火许可。 */
} app_shooter_feedback_t;

/** @brief 通过 USB VCP 接收的视觉跟踪目标。 */
typedef struct
{
    float target_yaw_rad;       /**< 视觉系统报告的偏航角 [rad]。 */
    float target_pitch_rad;     /**< 视觉系统报告的俯仰角 [rad]。 */
    uint32_t update_count;      /**< 累计有效目标更新次数。 */
    bool target_valid;          /**< 当前目标数据有效（未超时）。 */
    bool tracking_ready;        /**< 视觉系统报告已锁定跟踪。 */
} app_vision_target_t;

#endif
