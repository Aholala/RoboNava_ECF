/**
 * @file alg_omni.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用全向轮底盘运动学算法实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 支持任意数量、任意位置和任意驱动方向的全向轮。
 *       内部使用几何关系将轮速约束转化为线性方程，并利用 alg_chassis_motion
 *       中的加权最小二乘求解器进行正解。
 *       逆解支持任意旋转中心和轮速饱和缩放。
 */

#include "alg_omni.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 检查单个轮配置是否合法
 * @param wheel_config  轮配置
 * @return true=合法
 * @note 检查位置、驱动方向、半径、方向符号和权重是否有限且合法。
 *       半径和权重必须为正，方向符号必须为 ±1。
 */
static bool alg_omni_wheel_config_is_valid(const alg_omni_wheel_config_t *wheel_config)
{
    return isfinite(wheel_config->position_x_m) && isfinite(wheel_config->position_y_m) &&
           isfinite(wheel_config->drive_direction_rad) && isfinite(wheel_config->wheel_radius_m) &&
           (wheel_config->wheel_radius_m > 0.0F) &&
           ((wheel_config->direction_sign == 1.0F) || (wheel_config->direction_sign == -1.0F)) &&
           isfinite(wheel_config->odometry_weight) && (wheel_config->odometry_weight > 0.0F);
}

/**
 * @brief 计算轮子约束方程的系数
 * @param wheel_config  轮配置
 * @param velocity_x_coefficient  输出：vx 的系数（Cx = cos(drive_dir)）
 * @param velocity_y_coefficient  输出：vy 的系数（Cy = sin(drive_dir)）
 * @param angular_velocity_coefficient_m  输出：wz 的系数（单位米）
 * @note 约束形式：vx*Cx + vy*Cy + wz*Cw = v_linear
 *       角速度系数 Cw = -Cx*pos_y + Cy*pos_x（驱动方向与位置向量的叉积）。
 *       为纯计算函数，无副作用，不修改轮配置。
 */
static void alg_omni_get_constraint_coefficients(const alg_omni_wheel_config_t *wheel_config,
                                                 float *velocity_x_coefficient,
                                                 float *velocity_y_coefficient,
                                                 float *angular_velocity_coefficient_m)
{
    *velocity_x_coefficient = cosf(wheel_config->drive_direction_rad);
    *velocity_y_coefficient = sinf(wheel_config->drive_direction_rad);
    // ---- 角速度系数 = (cos,sin) 与位置向量的叉积 ----
    *angular_velocity_coefficient_m = (-*velocity_x_coefficient * wheel_config->position_y_m) +
                                      (*velocity_y_coefficient * wheel_config->position_x_m);
}

/* ======================== 轮组布局生成 ======================== */

/**
 * @brief 生成均匀圆周切向布局的轮组配置（用于对称全向底盘）
 * @param wheel_configs  输出：轮组配置数组（需有 wheel_count 个元素）
 * @param wheel_count    轮子数量（≥2）
 * @param center_to_wheel_distance_m  轮心到车体原点的距离（米，>0）
 * @param wheel_radius_m  轮子半径（米，>0）
 * @param first_wheel_position_angle_rad  第一个轮子的位置角（弧度）
 * @param tangential_direction_sign  切向方向符号（+1 或 -1），决定驱动方向沿圆周顺时针或逆时针
 * @param wheel_direction_signs  各轮电机安装方向符号数组（长度 wheel_count），可为 NULL（则全为 +1）
 * @param odometry_weight  所有轮相同的正解权重（>0）
 * @return 执行状态
 * @note 轮子均匀分布在圆周上，驱动方向沿圆周切线。
 *       位置角 = first_angle + 2π * index / count
 *       驱动方向 = 位置角 + tangential_sign * π/2
 *       适用于标准三轮或四轮全向底盘。
 */
alg_chassis_status_t alg_omni_configure_tangential_layout(
    alg_omni_wheel_config_t *wheel_configs, size_t wheel_count, float center_to_wheel_distance_m,
    float wheel_radius_m, float first_wheel_position_angle_rad, float tangential_direction_sign,
    const float *wheel_direction_signs, float odometry_weight)
{
    const float full_circle_rad = 6.28318530717958647692F;
    const float quarter_turn_rad = 1.57079632679489661923F;
    size_t wheel_index;

    // ---- 参数检查 ----
    if ((wheel_configs == NULL) || (wheel_count < 2U) || !isfinite(center_to_wheel_distance_m) ||
        (center_to_wheel_distance_m <= 0.0F) || !isfinite(wheel_radius_m) ||
        (wheel_radius_m <= 0.0F) || !isfinite(first_wheel_position_angle_rad) ||
        ((tangential_direction_sign != 1.0F) && (tangential_direction_sign != -1.0F)) ||
        !isfinite(odometry_weight) || (odometry_weight <= 0.0F))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    // ---- 逐轮生成配置 ----
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
    {
        const float motor_direction_sign =
            (wheel_direction_signs == NULL) ? 1.0F : wheel_direction_signs[wheel_index];
        const float position_angle_rad = first_wheel_position_angle_rad +
                                         full_circle_rad * (float)wheel_index / (float)wheel_count;

        if ((motor_direction_sign != 1.0F) && (motor_direction_sign != -1.0F))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }

        wheel_configs[wheel_index] = (alg_omni_wheel_config_t){
            .position_x_m = center_to_wheel_distance_m * cosf(position_angle_rad),
            .position_y_m = center_to_wheel_distance_m * sinf(position_angle_rad),
            .drive_direction_rad =
                position_angle_rad + tangential_direction_sign * quarter_turn_rad,
            .wheel_radius_m = wheel_radius_m,
            .direction_sign = motor_direction_sign,
            .odometry_weight = odometry_weight,
        };
    }
    return ALG_CHASSIS_STATUS_OK;
}

/* ======================== 初始化 ======================== */

/**
 * @brief 初始化全向底盘运动学模型
 * @param me        模型对象
 * @param wheel_configs  轮组配置数组（将被对象引用，必须保持有效）
 * @param wheel_count    轮子数量（必须 >0）
 * @param maximum_wheel_angular_velocity_rad_per_s  轮速上限（rad/s，>0）
 * @return 执行状态
 * @note 逐一校验所有轮配置的合法性（位置、驱动方向、半径、符号、权重）。
 *       配置数组指针被保存为引用，调用者在对象生命周期内不能释放或修改该数组。
 *       不使用动态内存，所有数据由调用者管理。
 */
alg_chassis_status_t alg_omni_init(alg_omni_t *me, const alg_omni_wheel_config_t *wheel_configs,
                                   size_t wheel_count,
                                   float maximum_wheel_angular_velocity_rad_per_s)
{
    size_t wheel_index;

    // ---- 参数检查 ----
    if ((me == NULL) || (wheel_configs == NULL) || (wheel_count == 0U) ||
        !isfinite(maximum_wheel_angular_velocity_rad_per_s) ||
        (maximum_wheel_angular_velocity_rad_per_s <= 0.0F))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;

    // ---- 检查所有轮配置 ----
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
    {
        if (!alg_omni_wheel_config_is_valid(&wheel_configs[wheel_index]))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
    }

    // ---- 保存配置 ----
    me->wheel_configs = wheel_configs;
    me->wheel_count = wheel_count;
    me->maximum_wheel_angular_velocity_rad_per_s = maximum_wheel_angular_velocity_rad_per_s;
    me->is_initialized = true;
    return ALG_CHASSIS_STATUS_OK;
}

/* ======================== 逆运动学 ======================== */

/**
 * @brief 逆运动学：将底盘原点速度映射到各轮角速度（绕底盘中心旋转）
 * @param me        模型对象
 * @param chassis_velocity  期望的底盘原点速度（vx, vy, wz）
 * @param wheel_is_available  各轮是否可用（长度为 wheel_count 的数组），NULL 表示全部可用
 * @param wheel_angular_velocities_rad_per_s  输出：各轮角速度（rad/s），需有 wheel_count 容量
 * @param output_capacity  输出数组容量（至少为 wheel_count）
 * @param applied_scale  输出：实际应用的缩放比例（≤1.0）
 * @return 执行状态
 * @note 等价于调用 alg_omni_inverse_with_center_of_rotation 且旋转中心为(0,0)。
 *       不可用轮输出为 0。
 *       若任一可用轮超速，所有可用轮按等比例缩放以保持运动方向。
 */
alg_chassis_status_t alg_omni_inverse(const alg_omni_t *me,
                                      const alg_chassis_velocity_t *chassis_velocity,
                                      const bool *wheel_is_available,
                                      float *wheel_angular_velocities_rad_per_s,
                                      size_t output_capacity, float *applied_scale)
{
    return alg_omni_inverse_with_center_of_rotation(
        me, chassis_velocity, 0.0F, 0.0F, wheel_is_available, wheel_angular_velocities_rad_per_s,
        output_capacity, applied_scale);
}

/**
 * @brief 逆运动学：将指定旋转中心处的速度映射到各轮角速度
 * @param me        模型对象
 * @param center_velocity  旋转中心处的速度（vx, vy, wz）
 * @param center_of_rotation_x_m  旋转中心相对底盘原点的 X 坐标（米）
 * @param center_of_rotation_y_m  旋转中心相对底盘原点的 Y 坐标（米）
 * @param wheel_is_available  各轮是否可用（长度为 wheel_count 的数组）
 * @param wheel_angular_velocities_rad_per_s  输出：各轮角速度（rad/s）
 * @param output_capacity  输出数组容量（至少为 wheel_count）
 * @param applied_scale  输出：实际应用的缩放比例（≤1.0）
 * @return 执行状态
 * @note 实现步骤：先将旋转中心速度转换到底盘原点，再逐轮计算角速度。
 *       旋转中心可为任意点，支持绕云台轴、某轮接地点或车外点旋转。
 *       轮子线性速度：v_linear = Cx*vx + Cy*vy + Cw*wz
 *       轮子角速度 = v_linear / radius * direction_sign
 */
alg_chassis_status_t alg_omni_inverse_with_center_of_rotation(
    const alg_omni_t *me, const alg_chassis_velocity_t *center_velocity,
    float center_of_rotation_x_m, float center_of_rotation_y_m, const bool *wheel_is_available,
    float *wheel_angular_velocities_rad_per_s, size_t output_capacity, float *applied_scale)
{
    alg_chassis_velocity_t origin_velocity;
    alg_chassis_status_t status;
    size_t wheel_index;

    // ---- 参数检查 ----
    if ((me == NULL) || (center_velocity == NULL) || (wheel_angular_velocities_rad_per_s == NULL) ||
        !isfinite(center_velocity->velocity_x_m_per_s) ||
        !isfinite(center_velocity->velocity_y_m_per_s) ||
        !isfinite(center_velocity->angular_velocity_rad_per_s))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }
    if (output_capacity < me->wheel_count)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    // ---- 旋转中心速度 → 车体原点速度 ----
    status = alg_chassis_convert_center_velocity_to_origin(
        center_velocity, center_of_rotation_x_m, center_of_rotation_y_m, &origin_velocity);
    if (status != ALG_CHASSIS_STATUS_OK)
    {
        return status;
    }

    // ---- 逐轮计算角速度 ----
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        const alg_omni_wheel_config_t *const wc = &me->wheel_configs[wheel_index];
        float cx, cy, cw;
        float wheel_linear_v;

        // 获取约束系数
        alg_omni_get_constraint_coefficients(wc, &cx, &cy, &cw);

        // 计算轮子线性速度（沿驱动方向）
        wheel_linear_v = cx * origin_velocity.velocity_x_m_per_s +
                         cy * origin_velocity.velocity_y_m_per_s +
                         cw * origin_velocity.angular_velocity_rad_per_s;

        // 转换为角速度并乘以方向符号
        wheel_angular_velocities_rad_per_s[wheel_index] =
            wheel_linear_v / wc->wheel_radius_m * wc->direction_sign;
    }

    // ---- 轮速饱和缩放 ----
    return alg_chassis_scale_wheel_velocities(
        wheel_angular_velocities_rad_per_s, wheel_is_available, me->wheel_count,
        me->maximum_wheel_angular_velocity_rad_per_s, applied_scale);
}

/* ======================== 正运动学 ======================== */

/**
 * @brief 正运动学：从实测轮速估计底盘速度（加权最小二乘）
 * @param me        模型对象
 * @param wheel_angular_velocities_rad_per_s  各轮实测角速度（rad/s，长度 wheel_count）
 * @param wheel_is_available  各轮是否可用（NULL 表示全部可用）
 * @param known_component_mask  已知分量的掩码（bit0=vx, bit1=vy, bit2=wz），0 表示无先验
 * @param known_velocity  已知分量值（若掩码非零，必须提供）
 * @param constraint_workspace  工作区数组（用于构建约束方程，长度至少 wheel_count）
 * @param workspace_capacity  工作区容量（至少为 wheel_count）
 * @param solution  输出：估计速度及残差
 * @return 执行状态
 * @note 使用各轮的 odometry_weight 作为权重。
 *       约束形式：Cx*vx + Cy*vy + Cw*wz = measured_v
 *       其中 measured_v = omega * radius * direction_sign
 *       可用轮数不足 3 时需提供已知分量以避免奇异。
 *       残差 RMS 可用于检测异常轮速。
 *       工作区由调用者提供，避免动态内存分配。
 */
alg_chassis_status_t alg_omni_forward(const alg_omni_t *me,
                                      const float *wheel_angular_velocities_rad_per_s,
                                      const bool *wheel_is_available, uint8_t known_component_mask,
                                      const alg_chassis_velocity_t *known_velocity,
                                      alg_chassis_constraint_t *constraint_workspace,
                                      size_t workspace_capacity, alg_chassis_solution_t *solution)
{
    size_t wheel_index;

    // ---- 参数检查 ----
    if ((me == NULL) || (wheel_angular_velocities_rad_per_s == NULL) ||
        (constraint_workspace == NULL) || (solution == NULL))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }
    if (workspace_capacity < me->wheel_count)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    // ---- 逐轮构建约束 ----
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        const alg_omni_wheel_config_t *const wc = &me->wheel_configs[wheel_index];
        float cx, cy, cw;

        if (!isfinite(wheel_angular_velocities_rad_per_s[wheel_index]))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }

        alg_omni_get_constraint_coefficients(wc, &cx, &cy, &cw);

        constraint_workspace[wheel_index] = (alg_chassis_constraint_t){
            .velocity_x_coefficient = cx,
            .velocity_y_coefficient = cy,
            .angular_velocity_coefficient_m = cw,
            .measured_velocity_m_per_s = wheel_angular_velocities_rad_per_s[wheel_index] *
                                         wc->wheel_radius_m * wc->direction_sign,
            .weight = wc->odometry_weight,
            .is_available = (wheel_is_available == NULL) || wheel_is_available[wheel_index],
        };
    }

    // ---- 调用通用加权最小二乘求解器 ----
    return alg_chassis_solve_velocity(constraint_workspace, me->wheel_count, known_component_mask,
                                      known_velocity, me->wheel_count, solution);
}