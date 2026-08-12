/**
 * @file alg_pid.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 面向车载控制的精简 PID 控制器
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note C11、静态内存、显式时间步长，不依赖 HAL 或 RTOS。
 *       支持单环 PID、位置-速度串级 PID 和角度串级 PID。
 *       所有结构体均为值类型，无动态内存分配。
 */
#ifndef ALG_PID_H
#define ALG_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 状态码与基础类型 ======================== */

/** @brief PID 算法返回状态码 */
typedef enum
{
    ALG_PID_STATUS_OK = 0,              /**< 执行成功 */
    ALG_PID_STATUS_INVALID_ARGUMENT,    /**< 无效参数（NULL 指针等） */
    ALG_PID_STATUS_OUT_OF_RANGE,        /**< 参数超出有效范围 */
    ALG_PID_STATUS_NOT_INITIALIZED,     /**< 对象未初始化 */
    ALG_PID_STATUS_NUMERICAL_ERROR      /**< 数值计算错误（NaN/Inf） */
} alg_pid_status_t;

/** @brief PID 控制器配置参数 */
typedef struct
{
    float proportional_gain;            /**< 比例增益 Kp */
    float integral_gain;                /**< 积分增益 Ki */
    float derivative_gain;              /**< 微分增益 Kd */
    float derivative_filter_cutoff_hz;  /**< 微分项低通滤波截止频率（Hz），0=无滤波 */
    float integral_min;                 /**< 积分项下限 */
    float integral_max;                 /**< 积分项上限 */
    float output_min;                   /**< 输出下限 */
    float output_max;                   /**< 输出上限 */
    bool derivative_on_measurement;     /**< true=微分作用于测量值（推荐），false=微分作用于误差 */
} alg_pid_config_t;

/** @brief PID 高级输入（含前馈分量） */
typedef struct
{
    float setpoint;                     /**< 目标值 */
    float measurement;                  /**< 当前测量值 */
    float velocity_feedforward;         /**< 速度前馈 */
    float acceleration_feedforward;     /**< 加速度前馈 */
    float additional_feedforward;       /**< 附加前馈（如摩擦力补偿） */
    float delta_time_s;                 /**< 时间步长（秒） */
} alg_pid_input_t;

/** @brief PID 各分量分解信息（用于在线调试） */
typedef struct
{
    float proportional;                 /**< 比例分量 */
    float integral;                     /**< 积分分量 */
    float derivative;                   /**< 微分分量 */
    float feedforward;                  /**< 前馈分量总和 */
    float unsaturated_output;           /**< 饱和前的总输出 */
    float output;                       /**< 限幅后的实际输出 */
} alg_pid_terms_t;

/** @brief PID 控制器实例（单环） */
typedef struct
{
    alg_pid_config_t config;            /**< 控制器配置 */
    alg_pid_terms_t terms;              /**< 当前各分量值 */
    float previous_error;               /**< 上一周期误差（用于微分） */
    float previous_measurement;         /**< 上一周期测量值（用于微分） */
    float filtered_derivative;          /**< 经低通滤波的微分信号 */
    bool has_previous_sample;           /**< 是否有历史采样数据 */
    bool is_initialized;                /**< 是否已完成初始化 */
} alg_pid_t;

/* ======================== 单环 PID API ======================== */

/**
 * @brief 初始化 PID 配置为默认值
 * @param config 配置结构体指针
 * @return 执行状态
 * @note 默认值：所有增益为 0，积分/输出限幅为 ±FLT_MAX，微分作用于测量值
 */
alg_pid_status_t alg_pid_config_init(alg_pid_config_t *config);

/**
 * @brief 初始化 PID 控制器
 * @param me PID 控制器对象
 * @param config 控制器配置
 * @return 执行状态
 */
alg_pid_status_t alg_pid_init(alg_pid_t *me, const alg_pid_config_t *config);

/**
 * @brief 重置 PID 控制器状态
 * @param me PID 控制器对象
 * @param measurement 当前测量值（用于初始化微分历史）
 * @param initial_output 初始输出值（会写入积分项以实现无扰切换）
 * @return 执行状态
 * @note 将积分项设为 initial_output（限幅后），清除微分和误差历史
 */
alg_pid_status_t alg_pid_reset(alg_pid_t *me, float measurement, float initial_output);

/**
 * @brief 简化 PID 更新（无前馈）
 * @param me PID 控制器对象
 * @param setpoint 目标值
 * @param measurement 当前测量值
 * @param delta_time_s 时间步长（秒）
 * @param output 输出控制量（输出参数）
 * @return 执行状态
 * @note 内部调用 alg_pid_update_advanced，前馈分量均为 0
 */
alg_pid_status_t alg_pid_update(alg_pid_t *me, float setpoint, float measurement,
                                float delta_time_s, float *output);

/**
 * @brief 高级 PID 更新（含前馈和反饱和积分）
 * @param me PID 控制器对象
 * @param input 包含设定值、测量值、前馈和 dt 的输入结构体
 * @param output 输出控制量（输出参数）
 * @return 执行状态
 * @note 算法流程：
 *       1. 计算误差 e = setpoint - measurement
 *       2. P 项 = Kp * e
 *       3. I 项 = clamp(I_prev + Ki * e * dt, I_min, I_max)
 *       4. D 项 = Kd * filtered(-d_measurement/dt) 或 Kd * filtered(de/dt)
 *       5. 总输出 = P + I + D + 前馈，经输出限幅后输出
 *       6. 反饱和积分：若输出饱和且积分项加剧饱和方向，则冻结积分项
 */
alg_pid_status_t alg_pid_update_advanced(alg_pid_t *me, const alg_pid_input_t *input,
                                         float *output);

/**
 * @brief 获取 PID 各分量的当前值（用于调试/遥测）
 * @param me PID 控制器对象
 * @return 指向 terms 的指针，未初始化时返回 NULL
 */
const alg_pid_terms_t *alg_pid_get_terms(const alg_pid_t *me);

/* ======================== 串级 PID 类型与 API ======================== */

/** @brief 串级 PID 配置 */
typedef struct
{
    alg_pid_config_t position_config;   /**< 位置环（外环）PID 配置 */
    alg_pid_config_t velocity_config;   /**< 速度环（内环）PID 配置 */
    uint32_t position_loop_divider;     /**< 位置环降频因子（1=每周期运行） */
    float velocity_setpoint_min;        /**< 速度设定值下限 */
    float velocity_setpoint_max;        /**< 速度设定值上限 */
} alg_pid_cascade_config_t;

/** @brief 串级 PID 输入 */
typedef struct
{
    float position_setpoint;            /**< 位置目标值 */
    float position_measurement;         /**< 位置测量值 */
    float velocity_measurement;         /**< 速度测量值 */
    float velocity_feedforward;         /**< 速度前馈（作用于位置环） */
    float actuator_feedforward;         /**< 执行器前馈（作用于速度环） */
    float delta_time_s;                 /**< 时间步长（秒） */
} alg_pid_cascade_input_t;

/** @brief 串级 PID 控制器实例 */
typedef struct
{
    alg_pid_t position_controller;      /**< 位置环 PID */
    alg_pid_t velocity_controller;      /**< 速度环 PID */
    uint32_t position_loop_divider;     /**< 位置环降频因子 */
    uint32_t position_loop_counter;     /**< 位置环降频计数器 */
    float position_elapsed_time_s;      /**< 位置环累计时间（秒） */
    float velocity_setpoint_min;        /**< 速度设定值下限 */
    float velocity_setpoint_max;        /**< 速度设定值上限 */
    float velocity_setpoint;            /**< 当前速度设定值（位置环输出） */
    bool is_initialized;                /**< 是否已完成初始化 */
} alg_pid_cascade_t;

/**
 * @brief 初始化串级 PID
 * @param me 串级 PID 对象
 * @param config 串级 PID 配置
 * @return 执行状态
 * @note 同时初始化位置环和速度环两个 PID 子控制器
 */
alg_pid_status_t alg_pid_cascade_init(alg_pid_cascade_t *me,
                                      const alg_pid_cascade_config_t *config);

/**
 * @brief 重置串级 PID
 * @param me 串级 PID 对象
 * @param position_measurement 当前位置测量值
 * @param velocity_measurement 当前速度测量值
 * @param initial_output 初始输出值
 * @return 执行状态
 * @note 同时重置位置环和速度环，清除降频计数器
 */
alg_pid_status_t alg_pid_cascade_reset(alg_pid_cascade_t *me, float position_measurement,
                                       float velocity_measurement, float initial_output);

/**
 * @brief 更新串级 PID
 * @param me 串级 PID 对象
 * @param input 包含位置/速度目标值和测量值的输入结构体
 * @param output 输出控制量（输出参数）
 * @return 执行状态
 * @note 位置环按 position_loop_divider 降频运行，速度环每周期运行。
 *       位置环输出经限幅后作为速度环的设定值。
 */
alg_pid_status_t alg_pid_cascade_update(alg_pid_cascade_t *me,
                                        const alg_pid_cascade_input_t *input, float *output);

/**
 * @brief 获取当前速度环设定值
 * @param me 串级 PID 对象
 * @return 速度设定值（未初始化时返回 0.0F）
 * @note 该值为最近一次位置环更新产出的速度指令
 */
float alg_pid_cascade_get_velocity_setpoint(const alg_pid_cascade_t *me);

/* ======================== 角度串级 PID 类型与 API ======================== */

/** @brief 角度串级 PID 配置（封装串级 PID 配置） */
typedef struct
{
    alg_pid_cascade_config_t cascade_config; /**< 串级 PID 配置 */
} alg_pid_angle_config_t;

/** @brief 角度串级 PID 输入（单位：弧度） */
typedef struct
{
    float target_position_rad;          /**< 目标角度（rad） */
    float target_velocity_rad_per_s;    /**< 目标角速度（rad/s），用于前馈 */
    float measured_position_rad;        /**< 测量角度（rad） */
    float measured_velocity_rad_per_s;  /**< 测量角速度（rad/s） */
    float actuator_feedforward;         /**< 执行器前馈 */
    float delta_time_s;                 /**< 时间步长（秒） */
} alg_pid_angle_input_t;

/** @brief 角度串级 PID 控制器实例（封装串级 PID） */
typedef struct
{
    alg_pid_cascade_t cascade;          /**< 内部串级 PID */
} alg_pid_angle_t;

/**
 * @brief 初始化角度串级 PID
 * @param me 角度串级 PID 对象
 * @param config 角度串级 PID 配置
 * @return 执行状态
 * @note 便捷封装，内部直接委托给 alg_pid_cascade_init
 */
alg_pid_status_t alg_pid_angle_init(alg_pid_angle_t *me, const alg_pid_angle_config_t *config);

/**
 * @brief 重置角度串级 PID
 * @param me 角度串级 PID 对象
 * @param measured_position_rad 当前测量角度（rad）
 * @param measured_velocity_rad_per_s 当前测量角速度（rad/s）
 * @param initial_output 初始输出值
 * @return 执行状态
 */
alg_pid_status_t alg_pid_angle_reset(alg_pid_angle_t *me, float measured_position_rad,
                                     float measured_velocity_rad_per_s, float initial_output);

/**
 * @brief 更新角度串级 PID
 * @param me 角度串级 PID 对象
 * @param input 角度输入结构体（目标位置/速度、测量位置/速度）
 * @param control_output 输出控制量（输出参数）
 * @return 执行状态
 * @note 将角度输入转换为串级 PID 输入，委托给 alg_pid_cascade_update
 */
alg_pid_status_t alg_pid_angle_update(alg_pid_angle_t *me, const alg_pid_angle_input_t *input,
                                      float *control_output);

/**
 * @brief 获取当前角速度设定值
 * @param me 角度串级 PID 对象
 * @return 角速度设定值（未初始化时返回 0.0F）
 */
float alg_pid_angle_get_velocity_setpoint(const alg_pid_angle_t *me);

#ifdef __cplusplus
}
#endif

#endif
