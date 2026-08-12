/**
 * @file alg_imu_ekf_update.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief IMU EKF 更新实现（预测 + 校正）
 * @version 1.0
 * @date 2026-07-24
 * @copyright Copyright (c) 2026
 *
 * @note 包含过程噪声计算、零偏渐消、创新统计计算、
 *       加速度计校正和完整更新流程。
 */

#include "alg_imu_ekf_internal.h"

#include <math.h>   // sqrtf, fabsf, isfinite
#include <stddef.h> // NULL

/** @brief 最小模长阈值，用于避免除零 */
#define ALG_IMU_EKF_MINIMUM_NORM (1.0e-6F)
/** @brief 奇异阈值，用于矩阵求逆判断 */
#define ALG_IMU_EKF_SINGULAR_THRESHOLD (1.0e-12F)
#define ALG_IMU_EKF_PI (3.14159265358979323846F)
#define ALG_IMU_EKF_TWO_PI (2.0F * ALG_IMU_EKF_PI)

/* ======================== 过程噪声计算 ======================== */

/**
 * @brief 更新过程噪声矩阵 Q
 * @param me EKF 对象
 * @param delta_time_s 时间步长（秒）
 * @note Q = G * (σ²_gyro*dt) * G^T + Q_bias
 *
 *       @par G 矩阵的物理含义（4×3，四元数对陀螺仪噪声的雅可比）：
 *       陀螺仪噪声 ω_noise 各向同性（每个轴独立），通过四元数微分方程
 *       dq/dt = 0.5 * q⊗ω 传播到状态空间。
 *       对噪声项线化：Δq ≈ 0.5*dt * ∂(q⊗ω)/∂ω * ω_noise = G * ω_noise
 *       因此 G = 0.5 * ∂(q⊗ω)/∂ω，矩阵 gyrMapping 中已包含 0.5 因子。
 *
 *       @par G 矩阵的结构（4 行 3 列）：
 *       第 0 列（X 轴噪声对四元数的影响）：
 *       ∂(q⊗[ω_x,0,0])/∂ω = 0.5 * [ -q_x,  q_w,  q_z, -q_y]?
 *       实际存储：
 *       col0 = [-q1,  q0, -q3,  q2]  ← X 轴陀螺仪噪声
 *       col1 = [-q2,  q3,  q0, -q1]  ← Y 轴陀螺仪噪声
 *       col2 = [-q3, -q2,  q1,  q0]  ← Z 轴陀螺仪噪声
 *
 *       @par 计算 G * Q_gyro * G^T：
 *       由于 Q_gyro = σ²_gyro * I（各向同性假设），
 *       实际计算简化为：σ²_gyro * dt * G * G^T
 *       即 gyro_variance_factor * sum(gyrMapping[row][axis] * gyrMapping[col][axis])
 *
 *       @par Q_bias（零偏随机游走噪声）：
 *       仅 X/Y 轴（状态索引 4 和 5）有噪声。
 *       方差 = σ²_bias_rw * dt
 *       Z 轴无过程噪声（不估计零偏）。
 */
static void alg_imu_ekf_update_process_noise(alg_imu_ekf_t *me, float delta_time_s)
{
    float gyro_mapping[4U * 3U]; // 四元数对陀螺仪噪声的雅可比
    float gyro_variance_factor;  // 陀螺仪噪声方差因子
    float bias_variance;         // 零偏随机游走方差
    size_t row;
    size_t column;
    size_t axis;
    float accumulator;

    // ---- 清空过程噪声矩阵 ----
    for (row = 0U; row < (ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION); ++row) {
        me->process_noise[row] = 0.0F;
}

    // ---- 构建四元数对陀螺仪噪声的映射矩阵 G ----
    // G 将 3 维陀螺仪噪声映射到 4 维四元数状态
    // 每列对应一个轴的噪声，每行对应四元数一个分量
    gyro_mapping[0] = -me->state[1]; // 列0（X 轴噪声）→ q0
    gyro_mapping[1] = -me->state[2]; // 列0 → q1
    gyro_mapping[2] = -me->state[3]; // 列0 → q2
    gyro_mapping[3] = me->state[0];  // 列0 → q3

    gyro_mapping[4] = -me->state[3]; // 列1（Y 轴噪声）→ q0
    gyro_mapping[5] = me->state[2];  // 列1 → q1
    gyro_mapping[6] = me->state[0];  // 列1 → q2
    gyro_mapping[7] = -me->state[1]; // 列1 → q3

    gyro_mapping[8] = me->state[2];  // 列2（Z 轴噪声）→ q0
    gyro_mapping[9] = -me->state[1]; // 列2 → q1
    gyro_mapping[10] = me->state[0]; // 列2 → q2
    gyro_mapping[11] = me->state[3]; // 列2 → q3

    // ---- 计算陀螺仪噪声对四元数协方差的贡献 ----
    // Q_quat = G * Q_gyro * G^T * (0.5 * dt)^2
    gyro_variance_factor = me->config.gyro_noise_std_rad_s * delta_time_s;

    for (row = 0U; row < 4U; ++row)
    {
        for (column = 0U; column < 4U; ++column)
        {
            accumulator = 0.0F;
            // 对三个轴累加
            for (axis = 0U; axis < 3U; ++axis)
            {
                accumulator += gyro_mapping[(row * 3U) + axis] * gyro_mapping[(column * 3U) + axis];
            }
            me->process_noise[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                gyro_variance_factor * accumulator;
        }
    }

    // ---- 零偏随机游走噪声（仅 X/Y 轴） ----
    bias_variance = me->config.gyro_bias_random_walk_std_rad_s2 * delta_time_s;
    me->process_noise[(4U * ALG_IMU_EKF_STATE_DIMENSION) + 4U] = bias_variance;
    me->process_noise[(5U * ALG_IMU_EKF_STATE_DIMENSION) + 5U] = bias_variance;

    // Z 轴零偏无过程噪声（不估计）
}

/* ======================== 零偏协方差渐消 ======================== */

/**
 * @brief 应用零偏协方差渐消因子
 * @param me EKF 对象
 * @note 每次预测时对 X/Y 零偏相关协方差乘以 sqrt(fading_factor)，
 *       防止零偏估计过于自信，保持对温度/老化引起的慢漂移的跟踪能力。
 *
 *       @par 渐消因子的作用机制：
 *       标准 EKF 的协方差预测 P = F*P*F^T + Q 会使协方差收敛到稳态。
 *       当 Q 偏小（低估实际零偏漂移）时，P 会收缩到过小，导致
 *       卡尔曼增益趋于 0，滤波器不再响应新的观测来修正零偏。
 *       渐消因子在预测后人为放大零偏相关协方差，等价于增大 Q，
 *       使滤波器始终对新观测保持一定的响应能力。
 *
 *       @par 缩放策略：
 *       对协方差矩阵中与零偏状态（索引 4-5）相关的所有元素
 *       （行索引或列索引 >=4）乘以 sqrt(fading_factor) 的对应次幂：
 *       - 行仅涉及零偏：乘 sqrt(fading_factor) 一次
 *       - 列仅涉及零偏：乘 sqrt(fading_factor) 一次
 *       - 行列均涉及零偏：乘 fading_factor（即两次 sqrt）
 *       这相当于对零偏状态人为注入额外的不确定度。
 */
static void alg_imu_ekf_apply_bias_fading(alg_imu_ekf_t *me)
{
    const float bias_scale = sqrtf(me->config.gyro_bias_fading_factor);
    size_t row;
    size_t column;
    float scale;

    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            scale = 1.0F;
            // 若行或列涉及零偏状态（索引 4~5），应用渐消因子
            if (row >= 4U) {
                scale *= bias_scale;
}
            if (column >= 4U) {
                scale *= bias_scale;
}
            me->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] *= scale;
        }
    }
}

/* ======================== 3×3 对称矩阵求逆 ======================== */

/**
 * @brief 求 3×3 对称矩阵的逆
 * @param matrix 输入矩阵（3×3，行优先）
 * @param inverse 输出逆矩阵（3×3）
 * @return true=成功，false=奇异
 */
static bool alg_imu_ekf_invert_symmetric3x3(const float matrix[9], float inverse[9])
{
    // 计算余子式
    const float cofactor_00 = (matrix[4] * matrix[8]) - (matrix[5] * matrix[7]);
    const float cofactor_01 = (matrix[5] * matrix[6]) - (matrix[3] * matrix[8]);
    const float cofactor_02 = (matrix[3] * matrix[7]) - (matrix[4] * matrix[6]);

    // 行列式 = a00*C00 + a01*C01 + a02*C02
    const float determinant =
        (matrix[0] * cofactor_00) + (matrix[1] * cofactor_01) + (matrix[2] * cofactor_02);

    if (!isfinite(determinant) || (fabsf(determinant) <= ALG_IMU_EKF_SINGULAR_THRESHOLD)) {
        return false;
}

    // 使用伴随矩阵法求逆
    inverse[0] = cofactor_00 / determinant;
    inverse[1] = ((matrix[2] * matrix[7]) - (matrix[1] * matrix[8])) / determinant;
    inverse[2] = ((matrix[1] * matrix[5]) - (matrix[2] * matrix[4])) / determinant;
    inverse[3] = cofactor_01 / determinant;
    inverse[4] = ((matrix[0] * matrix[8]) - (matrix[2] * matrix[6])) / determinant;
    inverse[5] = ((matrix[2] * matrix[3]) - (matrix[0] * matrix[5])) / determinant;
    inverse[6] = cofactor_02 / determinant;
    inverse[7] = ((matrix[1] * matrix[6]) - (matrix[0] * matrix[7])) / determinant;
    inverse[8] = ((matrix[0] * matrix[4]) - (matrix[1] * matrix[3])) / determinant;

    return alg_imu_ekf_internal_is_finite_array(inverse, 9U);
}

/* ======================== 创新统计计算 ======================== */

/**
 * @brief 计算创新统计量（NIS 和自适应噪声倍率）
 * @param me EKF 对象
 * @param normalized_measurement 归一化后的加速度测量（3 维单位向量）
 * @param base_measurement_variance 基础测量噪声方差
 * @return 执行状态
 * @note 完整流程：
 *
 *       @par 1. 创新残差 y = z - h(x)：
 *       测量值 z = 归一化加速度方向（单位向量）
 *       预测值 h(x) = 将世界系重力旋转到机体系（也是单位向量）
 *       创新 y 表示预测与观测之间的角度偏差（3 维）。
 *
 *       @par 2. 创新协方差 S = H*P*H^T + R：
 *       将状态协方差 P 通过测量雅可比 H 映射到测量空间，
 *       再加上测量噪声 R 得到创新协方差 S（3×3 对称正定阵）。
 *       S 刻画了"创新 y 应该有多大"的统计不确定性。
 *
 *       @par 3. 3×3 对称矩阵求逆（伴随矩阵法）：
 *       S 为 3×3 对称矩阵，使用解析求逆（避免数值消元法）：
 *       S^-1 = adj(S) / det(S)
 *       利用对称性减少计算量，行列式 < 阈值时报告奇异。
 *
 *       @par 4. NIS = y^T * S^-1 * y（归一化创新平方）：
 *       这是将创新归一化到单位协方差空间的二次型。
 *       理论上 NIS ~ χ²(m) 卡方分布（m=3 自由度）。
 *       NIS 过大表示观测与预测不一致（违反统计假设）。
 *
 *       @par 5. 卡方检验的意义：
 *       3 自由度卡方分布的 95% 分位约为 7.8，99% 分位约为 11.3。
 *       若 NIS >> 分位值，说明加速度计观测异常（如设备在运动）：
 *       - > adaptation_threshold：逐渐增大测量噪声（软处理）
 *       - > rejection_threshold：完全拒绝（硬拒绝）
 *
 *       @par 6. 自适应测量噪声倍率：
 *       在 [adaptation_threshold, rejection_threshold] 区间内，
 *       noise_scale = 1 + (max_scale - 1) * ratio^2（二次递增）
 *       实际测量噪声 R_adapted = R_base * noise_scale
 *       二次递增使得初期温和增大，接近拒绝阈值时快速增大。
 */
static alg_imu_ekf_status_t
alg_imu_ekf_compute_innovation_statistics(alg_imu_ekf_t *me, const float normalized_measurement[3],
                                          float base_measurement_variance)
{
    float *predicted_measurement = me->innovation_workspace;            // 预测测量（3 维）
    float *measurement_jacobian = predicted_measurement + 3U;           // 测量雅可比（3×6 = 18）
    float *measurement_covariance_product = measurement_jacobian + 18U; // H*P（3×6 = 18）
    float *innovation_covariance =
        measurement_covariance_product + 18U;                          // S = H*P*H^T + R（3×3 = 9）
    float *innovation_covariance_inverse = innovation_covariance + 9U; // S^-1（3×3 = 9）
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;
    float transformed_innovation[3];
    alg_kalman_status_t kalman_status;

    // ---- 1. 计算预测测量 h(x) ----
    kalman_status = alg_imu_ekf_internal_measurement_function(
        me->state, ALG_IMU_EKF_STATE_DIMENSION, ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        predicted_measurement, me);
    if (kalman_status != ALG_KALMAN_STATUS_OK) {
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;
}

    // ---- 2. 计算测量雅可比 H ----
    kalman_status = alg_imu_ekf_internal_measurement_jacobian(
        me->state, ALG_IMU_EKF_STATE_DIMENSION, ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        measurement_jacobian, me);
    if (kalman_status != ALG_KALMAN_STATUS_OK) {
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;
}

    // ---- 3. 计算创新残差 y = z - h(x) ----
    for (row = 0U; row < 3U; ++row) {
        me->innovation[row] = normalized_measurement[row] - predicted_measurement[row];
}

    // ---- 4. 计算 H*P ----
    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    measurement_jacobian[(row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                    me->covariance[(shared_index * ALG_IMU_EKF_STATE_DIMENSION) + column];
            }
            measurement_covariance_product[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                accumulator;
        }
    }

    // ---- 5. 计算创新协方差 S = H*P*H^T + R ----
    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    measurement_covariance_product[(row * ALG_IMU_EKF_STATE_DIMENSION) +
                                                   shared_index] *
                    measurement_jacobian[(column * ALG_IMU_EKF_STATE_DIMENSION) + shared_index];
            }
            innovation_covariance[(row * 3U) + column] =
                accumulator + ((row == column) ? base_measurement_variance : 0.0F);
        }
    }

    // ---- 6. 求创新协方差逆矩阵 ----
    if (!alg_imu_ekf_invert_symmetric3x3(innovation_covariance, innovation_covariance_inverse)) {
        return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

    // ---- 7. 计算 NIS = y^T * S^-1 * y ----
    // 先计算 S^-1 * y
    for (row = 0U; row < 3U; ++row)
    {
        transformed_innovation[row] =
            (innovation_covariance_inverse[(row * 3U)] * me->innovation[0]) +
            (innovation_covariance_inverse[(row * 3U) + 1U] * me->innovation[1]) +
            (innovation_covariance_inverse[(row * 3U) + 2U] * me->innovation[2]);
    }

    // 再计算 y^T * (S^-1 * y)
    me->last_normalized_innovation_squared = (me->innovation[0] * transformed_innovation[0]) +
                                             (me->innovation[1] * transformed_innovation[1]) +
                                             (me->innovation[2] * transformed_innovation[2]);

    return isfinite(me->last_normalized_innovation_squared) ? ALG_IMU_EKF_STATUS_OK
                                                            : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

/* ======================== EKF 预测 ======================== */

/**
 * @brief EKF 预测步骤（仅陀螺仪积分）
 * @param me EKF 对象
 * @param gyroscope_rad_s 陀螺仪读数（rad/s），长度 3
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_predict(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                         float delta_time_s)
{
    alg_kalman_status_t kalman_status;
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (gyroscope_rad_s == NULL)) {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
}
    if (!alg_imu_ekf_internal_is_finite_array(gyroscope_rad_s, 3U) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F)) {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
}

    // ---- 1. 零偏协方差渐消 ----
    alg_imu_ekf_apply_bias_fading(me);

    // ---- 2. 更新过程噪声 ----
    alg_imu_ekf_update_process_noise(me, delta_time_s);

    // ---- 3. 执行通用 EKF 预测 ----
    kalman_status = alg_kalman_extended_predict(&me->kalman, gyroscope_rad_s, delta_time_s);
    if (kalman_status != ALG_KALMAN_STATUS_OK) {
        return alg_imu_ekf_internal_map_kalman_status(kalman_status);
}

    // ---- 4. 四元数归一化与协方差投影 ----
    status = alg_imu_ekf_internal_normalize_and_project(me);
    me->was_accelerometer_used = false;

    return status;
}

/* ======================== 加速度计校正 ======================== */

/**
 * @brief EKF 校正步骤（加速度计观测）
 * @param me EKF 对象
 * @param accelerometer_m_s2 加速度计读数（m/s²），长度 3
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 * @note 观测被拒绝时返回 ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED
 *       （此时状态和协方差仍为预测值，不会被校正修改）。
 *
 *       @par 完整 9 步流程：
 *
 *       **1. 模长硬拒绝检查：**
 *       计算 |accel| 并检查与 g 的偏差。偏差超过
 *       accelerometer_rejection_threshold_g 时直接拒绝。
 *       原理：加速度计只有在静止/匀速时才测量纯重力。
 *       有外部加速度时，模长严重偏离 g，方向不可信。
 *
 *       **2. 低通滤波：**
 *       对三轴加速度分别应用一阶低通滤波器。
 *       消除高频振动噪声，减小测量噪声对校正的扰动。
 *
 *       **3. 归一化（仅用方向）：**
 *       将滤波后加速度归一化为单位向量。
 *       EKF 只使用重力方向信息校正 Roll/Pitch，
 *       不使用模长信息（模长受运动影响大）。
 *       等价于假设 |a| = g。
 *
 *       **4. 计算创新统计（NIS）：**
 *       调用 compute_innovation_statistics 计算：
 *       - 预测测量 h(x)
 *       - 创新残差 y
 *       - 创新协方差 S
 *       - NIS = y^T * S^-1 * y
 *
 *       **5. 卡方检验与收敛判断：**
 *       NIS < rejection_threshold/2 时，标记 has_converged = true。
 *       NIS > rejection_threshold 且在收敛状态：
 *       - 稳定状态下累加 rejection_count
 *       - 拒绝观测但保留，最多容忍 50 次连续拒绝
 *       - 超过 50 次则重置收敛标志（认为模型已发散）
 *       NIS 正常时清零 rejection_count。
 *
 *       **6. 自适应噪声倍率计算：**
 *       NIS 在 [adaptation_threshold, rejection_threshold) 区间内：
 *       ratio = (NIS - adapt_thr) / (reject_thr - adapt_thr)
 *       noise_scale = 1 + (max_scale - 1) * ratio²
 *       NIS 低于自适应阈值：noise_scale = 1.0（使用基准噪声）
 *
 *       **7. 更新测量噪声矩阵：**
 *       R_diag = base_variance * noise_scale
 *       更新 3×3 对角测量噪声矩阵。
 *
 *       **8. 执行通用 EKF 校正：**
 *       调用 alg_kalman_extended_correct，内部完成：
 *       - 计算 h(x) 和 H = ∂h/∂x
 *       - 卡尔曼增益 K = P*H^T * (H*P*H^T + R)^-1
 *       - 状态更新 x += K*(z - h(x))
 *       - Joseph 协方差更新
 *
 *       **9. 四元数归一化与协方差投影：**
 *       校正后的状态 x 和协方差 P 需要投影回单位四元数约束面。
 *       这是确保四元数保持 ||q||=1 的关键步骤。
 *       was_accelerometer_used 标记本次校正是否成功。
 */
alg_imu_ekf_status_t alg_imu_ekf_correct_accelerometer(alg_imu_ekf_t *me,
                                                       const float accelerometer_m_s2[3],
                                                       float delta_time_s)
{
    float raw_norm;
    float filtered_norm;
    float relative_deviation;
    float normalized_measurement[3];
    float base_measurement_variance;
    float adaptation_ratio;
    float noise_scale;
    size_t index;
    alg_filter_status_t filter_status;
    alg_kalman_status_t kalman_status;
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (accelerometer_m_s2 == NULL)) {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
}
    if (!alg_imu_ekf_internal_is_finite_array(accelerometer_m_s2, 3U) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F)) {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
}

    // ---- 1. 模长检查：硬拒绝 ----
    raw_norm = sqrtf((accelerometer_m_s2[0] * accelerometer_m_s2[0]) +
                     (accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                     (accelerometer_m_s2[2] * accelerometer_m_s2[2]));
    if (!isfinite(raw_norm) || (raw_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    // 计算相对 1g 的偏差
    relative_deviation = fabsf(raw_norm - me->config.gravity_m_s2) / me->config.gravity_m_s2;
    me->last_accelerometer_norm_m_s2 = raw_norm;
    me->last_accelerometer_deviation_g = relative_deviation;

    if (relative_deviation > me->config.accelerometer_rejection_threshold_g)
    {
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    // ---- 2. 低通滤波 ----
    for (index = 0U; index < 3U; ++index)
    {
        if (me->config.accelerometer_lpf_cutoff_hz <= 0.0F)
        {
            me->filtered_accelerometer_m_s2[index] = accelerometer_m_s2[index];
        }
        else
        {
            filter_status = alg_filter_low_pass_update(&me->accelerometer_filter[index],
                                                       accelerometer_m_s2[index], delta_time_s,
                                                       &me->filtered_accelerometer_m_s2[index]);
            if (filter_status != ALG_FILTER_STATUS_OK) {
                return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
            }
        }
    }

    // ---- 3. 归一化滤波后加速度（仅使用方向） ----
    filtered_norm =
        sqrtf((me->filtered_accelerometer_m_s2[0] * me->filtered_accelerometer_m_s2[0]) +
              (me->filtered_accelerometer_m_s2[1] * me->filtered_accelerometer_m_s2[1]) +
              (me->filtered_accelerometer_m_s2[2] * me->filtered_accelerometer_m_s2[2]));
    if (!isfinite(filtered_norm) || (filtered_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    for (index = 0U; index < 3U; ++index) {
        normalized_measurement[index] = me->filtered_accelerometer_m_s2[index] / filtered_norm;
}

    // ---- 4. 计算创新统计（NIS） ----
    base_measurement_variance =
        me->config.accelerometer_direction_noise_std * me->config.accelerometer_direction_noise_std;

    status = alg_imu_ekf_compute_innovation_statistics(me, normalized_measurement,
                                                       base_measurement_variance);
    if (status != ALG_IMU_EKF_STATUS_OK) {
        return status;
}

    // ---- 5. 卡方检验：NIS 超过拒绝阈值 ----
    if (me->last_normalized_innovation_squared <
        (0.5F * me->config.chi_square_rejection_threshold))
    {
        me->has_converged = true;
    }
    if ((me->last_normalized_innovation_squared >
         me->config.chi_square_rejection_threshold) && me->has_converged)
    {
        me->rejection_count = me->is_stable ? (me->rejection_count + 1U) : 0U;
        if (me->rejection_count <= 50U)
        {
            me->last_measurement_noise_scale = me->config.maximum_measurement_noise_scale;
            me->was_accelerometer_used = false;
            return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
        }
        me->has_converged = false;
    }
    else
    {
        me->rejection_count = 0U;
    }

    // ---- 6. 自适应噪声倍率 ----
    noise_scale = 1.0F;
    if (me->last_normalized_innovation_squared > me->config.chi_square_adaptation_threshold)
    {
        // NIS 在自适应区间内：线性增加噪声倍率
        adaptation_ratio =
            (me->last_normalized_innovation_squared - me->config.chi_square_adaptation_threshold) /
            (me->config.chi_square_rejection_threshold -
             me->config.chi_square_adaptation_threshold);
        noise_scale += (me->config.maximum_measurement_noise_scale - 1.0F) * adaptation_ratio *
                       adaptation_ratio;
    }
    me->last_measurement_noise_scale = noise_scale;

    // ---- 7. 更新测量噪声矩阵 ----
    for (index = 0U; index < 9U; ++index) {
        me->measurement_noise[index] = 0.0F;
}
    for (index = 0U; index < 3U; ++index) {
        me->measurement_noise[(index * 3U) + index] = base_measurement_variance * noise_scale;
}

    // ---- 8. 执行 EKF 校正 ----
    kalman_status = alg_kalman_extended_correct(&me->kalman, normalized_measurement);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        me->was_accelerometer_used = false;
        return alg_imu_ekf_internal_map_kalman_status(kalman_status);
    }

    // ---- 9. 归一化与投影 ----
    status = alg_imu_ekf_internal_normalize_and_project(me);
    me->was_accelerometer_used = (status == ALG_IMU_EKF_STATUS_OK);
    return status;
}

/* ======================== 完整更新 ======================== */

/**
 * @brief 更新连续 yaw 角度（处理 ±π 回绕）
 * @param me EKF 对象
 * @note yaw 角的 atan2 输出范围是 [-π, π]。
 *       当机器人旋转超过 ±π 时，yaw 会从 +π 跳变到 -π（或反之），
 *       造成不连续。此函数检测跳变并累计圈数，实现连续 yaw。
 *
 *       @par 回绕检测算法：
 *       比较当前 yaw 与上一帧 yaw 的差值：
 *       - 差值 > +π：发生了 -π 方向回绕（从 π 跳到 -π），圈数 -1
 *       - 差值 < -π：发生了 +π 方向回绕（从 -π 跳到 π），圈数 +1
 *       - 差值在 ±π 内：正常变化，圈数不变
 *
 *       @par 连续 yaw 计算公式：
 *       continuous_yaw = wrapped_yaw + revolution_count * 2π
 *       例：从 π 旋转到 3π/2 时，wrapped_yaw = -π/2，rev_count = 1，
 *       则 continuous_yaw = -π/2 + 2π = 3π/2。连续可导。
 */
static void alg_imu_ekf_update_continuous_yaw(alg_imu_ekf_t *me)
{
    alg_imu_ekf_euler_t euler;
    float yaw_delta;

    if (alg_imu_ekf_get_euler(me, &euler) != ALG_IMU_EKF_STATUS_OK) {
        return;
    }
    if (me->update_count > 0U)
    {
        yaw_delta = euler.yaw_rad - me->previous_yaw_rad;
        // 检测 ±π 跳变
        if (yaw_delta > ALG_IMU_EKF_PI) {
            --me->yaw_revolution_count;
        } else if (yaw_delta < -ALG_IMU_EKF_PI) {
            ++me->yaw_revolution_count;
        }
    }
    me->previous_yaw_rad = euler.yaw_rad;
    // 连续 yaw = 环绕 yaw + 圈数 * 2π
    me->continuous_yaw_rad =
        euler.yaw_rad + ((float)me->yaw_revolution_count * ALG_IMU_EKF_TWO_PI);
    ++me->update_count;
}

/**
 * @brief 完整 EKF 更新（预测 + 校正）
 * @param me EKF 对象
 * @param gyroscope_rad_s 陀螺仪读数
 * @param accelerometer_m_s2 加速度计读数
 * @param delta_time_s 时间步长
 * @param accelerometer_used 输出是否使用加速度观测
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_update(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                        const float accelerometer_m_s2[3], float delta_time_s,
                                        bool *accelerometer_used)
{
    alg_imu_ekf_status_t status;
    float corrected_gyro_x;
    float corrected_gyro_y;
    float gyro_norm;
    float accelerometer_norm;

    if ((me == NULL) || (gyroscope_rad_s == NULL) || (accelerometer_m_s2 == NULL)) {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    }

    // ---- 稳定性判断 ----
    // 稳定条件：
    // 1. 陀螺仪模长 < 0.3 rad/s（约 17°/s）：设备角速度很小（近似静止或缓慢旋转）
    // 2. 加速度模长偏离 g < 0.5 m/s²（约 0.05g）：设备没有明显的线性加速度
    // 两条件同时满足时，is_stable = true
    // 稳定状态下加速度观测更可信，可用于更高精度的 Roll/Pitch 校正
    corrected_gyro_x = gyroscope_rad_s[0] - me->state[4];
    corrected_gyro_y = gyroscope_rad_s[1] - me->state[5];
    gyro_norm = sqrtf((corrected_gyro_x * corrected_gyro_x) +
                      (corrected_gyro_y * corrected_gyro_y) +
                      (gyroscope_rad_s[2] * gyroscope_rad_s[2]));
    accelerometer_norm = sqrtf((accelerometer_m_s2[0] * accelerometer_m_s2[0]) +
                               (accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                               (accelerometer_m_s2[2] * accelerometer_m_s2[2]));
    me->is_stable = isfinite(gyro_norm) && isfinite(accelerometer_norm) &&
                    (gyro_norm < 0.3F) &&
                    (fabsf(accelerometer_norm - me->config.gravity_m_s2) < 0.5F);

    // ---- 1. 预测 ----
    status = alg_imu_ekf_predict(me, gyroscope_rad_s, delta_time_s);
    if (status != ALG_IMU_EKF_STATUS_OK) {
        return status;
}

    // ---- 2. 校正 ----
    status = alg_imu_ekf_correct_accelerometer(me, accelerometer_m_s2, delta_time_s);

    // 加速度被拒绝时返回 OK（预测结果仍然有效）
    if (status == ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED)
    {
        if (accelerometer_used != NULL) {
            *accelerometer_used = false;
}
        alg_imu_ekf_update_continuous_yaw(me);
        return ALG_IMU_EKF_STATUS_OK;
    }

    if (status != ALG_IMU_EKF_STATUS_OK) {
        return status;
}

    if (accelerometer_used != NULL) {
        *accelerometer_used = true;
}

    alg_imu_ekf_update_continuous_yaw(me);
    return ALG_IMU_EKF_STATUS_OK;
}
