/**
 * @file alg_kalman_matrix.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 卡尔曼滤波矩阵运算实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 包含矩阵求逆（带部分主元选择的 Gauss-Jordan 消元法）、
 *       矩阵乘法、转置乘法、数组检查、对称化等工具函数。
 *       所有矩阵使用行优先存储。
 */

#include "alg_kalman_internal.h"

#include <math.h>   // fabsf, isfinite
#include <stddef.h> // NULL

/** @brief 奇异阈值，用于判断矩阵是否可逆 */
#define ALG_KALMAN_SINGULAR_THRESHOLD (1.0e-12F)

/* ======================== 矩阵求逆（Gauss-Jordan） ======================== */

/**
 * @brief 使用 Gauss-Jordan 消元法求矩阵逆（带部分主元选择）
 * @param matrix 输入矩阵（会被修改为行阶梯形，原地操作）
 * @param inverse 输出逆矩阵
 * @param dimension 矩阵维度
 * @return 执行状态
 * @note
 *       @par 部分主元选择策略：
 *       在第 k 列消元前，从第 k 行到最后一行的该列中选出绝对值最大的元素，
 *       将其所在行与第 k 行交换。这避免了小主元导致的数值不稳定。
 *       与"全主元选择"（行列都选）相比：
 *       - 只交换行，不改变未知数顺序（不需要记录列置换）
 *       - 计算量增加约 n² 次比较，但嵌入式矩阵维数不高（典型 n<=20），可忽略
 *       - 在实践中数值稳定性接近全主元选择
 *
 *       @par Gauss-Jordan 消元 vs 直接解方程：
 *       这里使用的是求逆而非解方程组的形式，因为：
 *       - 卡尔曼滤波每次校正都需要对创新协方差矩阵求逆
 *       - 之后该逆矩阵还要用于多处计算
 *       - 增广矩阵法直接求逆比分步求解效率更高
 *
 *       @par 步骤：
 *       1. 初始化逆矩阵为单位矩阵 I
 *       2. 对每一列 j：
 *          a. 选主元（第 j 列中绝对值最大的行）
 *          b. 若主元 < 阈值 -> 奇异矩阵
 *          c. 交换主元行到当前行（原矩阵和逆矩阵同时交换）
 *          d. 归一化主元行（除以主元值）-> 主元变为 1
 *          e. 消去其他所有行中的第 j 列（Gauss-Jordan 特征，非仅下方行）
 *       3. 检查逆矩阵元素有效性
 *
 *       @par 消去所有行的原因（Gauss-Jordan vs Gauss）：
 *       Gauss 消元只消去下方行，得到上三角阵，还需回代。
 *       Gauss-Jordan 消去所有行，得到单位矩阵，无需回代。
 *       增广矩阵 I 直接变为 A^-1。代码更简洁，嵌入式场景可接受额外的乘除。
 */
static alg_kalman_status_t alg_kalman_internal_invert(float *matrix, float *inverse,
                                                      size_t dimension)
{
    size_t row;
    size_t column;
    size_t pivot_row;
    size_t element;
    float pivot_magnitude;
    float candidate_magnitude;
    float pivot_value;
    float scale;
    float temporary;

    // ---- 1. 初始化逆矩阵为单位矩阵 ----
    for (row = 0U; row < dimension; ++row) {
        for (column = 0U; column < dimension; ++column) {
            inverse[(row * dimension) + column] = (row == column) ? 1.0F : 0.0F;
}
}

    // ---- 2. 对每一列进行消元 ----
    for (column = 0U; column < dimension; ++column)
    {
        // 2.1 选择主元（当前列绝对值最大的行）
        pivot_row = column;
        pivot_magnitude = fabsf(matrix[(column * dimension) + column]);

        for (row = column + 1U; row < dimension; ++row)
        {
            candidate_magnitude = fabsf(matrix[(row * dimension) + column]);
            if (candidate_magnitude > pivot_magnitude)
            {
                pivot_magnitude = candidate_magnitude;
                pivot_row = row;
            }
        }

        // 检查主元是否有效
        if (!isfinite(pivot_magnitude) || (pivot_magnitude <= ALG_KALMAN_SINGULAR_THRESHOLD)) {
            return ALG_KALMAN_STATUS_SINGULAR_MATRIX;
}

        // 2.2 交换主元行到当前行（如果需要）
        if (pivot_row != column)
        {
            for (element = 0U; element < dimension; ++element)
            {
                temporary = matrix[(column * dimension) + element];
                matrix[(column * dimension) + element] = matrix[(pivot_row * dimension) + element];
                matrix[(pivot_row * dimension) + element] = temporary;

                temporary = inverse[(column * dimension) + element];
                inverse[(column * dimension) + element] =
                    inverse[(pivot_row * dimension) + element];
                inverse[(pivot_row * dimension) + element] = temporary;
            }
        }

        // 2.3 归一化主元行
        pivot_value = matrix[(column * dimension) + column];
        for (element = 0U; element < dimension; ++element)
        {
            matrix[(column * dimension) + element] /= pivot_value;
            inverse[(column * dimension) + element] /= pivot_value;
        }

        // 2.4 消去其他行中的当前列（Gauss-Jordan）
        for (row = 0U; row < dimension; ++row)
        {
            if (row == column) {
                continue;
}

            scale = matrix[(row * dimension) + column];
            for (element = 0U; element < dimension; ++element)
            {
                matrix[(row * dimension) + element] -=
                    scale * matrix[(column * dimension) + element];
                inverse[(row * dimension) + element] -=
                    scale * inverse[(column * dimension) + element];
            }
        }
    }

    // ---- 3. 检查结果有效性 ----
    return alg_kalman_internal_is_finite_array(inverse, dimension * dimension)
               ? ALG_KALMAN_STATUS_OK
               : ALG_KALMAN_STATUS_NUMERICAL_ERROR;
}

/* ======================== 数组检查函数 ======================== */

/**
 * @brief 检查数组元素是否全部为有限数
 * @param values 数组指针
 * @param value_count 元素个数
 * @return true=全有限
 */
bool alg_kalman_internal_is_finite_array(const float *values, size_t value_count)
{
    size_t index;

    if (values == NULL) {
        return false;
}

    for (index = 0U; index < value_count; ++index)
    {
        if (!isfinite(values[index])) {
            return false;
}
    }
    return true;
}

/**
 * @brief 检查矩阵对角线是否全部为非负有限数
 * @param matrix 矩阵指针（行优先）
 * @param dimension 矩阵维度
 * @return true=对角线全为非负有限数
 */
bool alg_kalman_internal_has_nonnegative_diagonal(const float *matrix, size_t dimension)
{
    size_t index;

    if (matrix == NULL) {
        return false;
}

    for (index = 0U; index < dimension; ++index)
    {
        if (!isfinite(matrix[(index * dimension) + index]) ||
            (matrix[(index * dimension) + index] < 0.0F)) {
            return false;
}
    }
    return true;
}

/* ======================== 矩阵运算 ======================== */

/**
 * @brief 拷贝数组
 * @param destination 目标数组
 * @param source 源数组
 * @param value_count 元素个数
 */
void alg_kalman_internal_copy(float *destination, const float *source, size_t value_count)
{
    size_t index;
    for (index = 0U; index < value_count; ++index) {
        destination[index] = source[index];
}
}

/**
 * @brief 矩阵乘法：C = A * B
 * @param left 左矩阵 A（left_rows × shared_dimension）
 * @param left_rows A 的行数
 * @param shared_dimension A 的列数 = B 的行数
 * @param right 右矩阵 B（shared_dimension × right_columns）
 * @param right_columns B 的列数
 * @param output 输出矩阵 C（left_rows × right_columns）
 * @note 三重循环，累加器避免频繁写入
 */
void alg_kalman_internal_multiply(const float *left, size_t left_rows, size_t shared_dimension,
                                  const float *right, size_t right_columns, float *output)
{
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;

    for (row = 0U; row < left_rows; ++row)
    {
        for (column = 0U; column < right_columns; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < shared_dimension; ++shared_index)
            {
                accumulator += left[(row * shared_dimension) + shared_index] *
                               right[(shared_index * right_columns) + column];
            }
            output[(row * right_columns) + column] = accumulator;
        }
    }
}

/**
 * @brief 矩阵乘法：C = A * B^T
 * @param left 左矩阵 A（left_rows × shared_dimension）
 * @param left_rows A 的行数
 * @param shared_dimension A 的列数 = B 的列数
 * @param right 右矩阵 B（right_rows × shared_dimension）
 * @param right_rows B 的行数
 * @param output 输出矩阵 C（left_rows × right_rows）
 * @note 用于计算 P * H^T 等场景，避免显式转置
 */
void alg_kalman_internal_multiply_right_transpose(const float *left, size_t left_rows,
                                                  size_t shared_dimension, const float *right,
                                                  size_t right_rows, float *output)
{
    size_t left_row;
    size_t right_row;
    size_t shared_index;
    float accumulator;

    for (left_row = 0U; left_row < left_rows; ++left_row)
    {
        for (right_row = 0U; right_row < right_rows; ++right_row)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < shared_dimension; ++shared_index)
            {
                accumulator += left[(left_row * shared_dimension) + shared_index] *
                               right[(right_row * shared_dimension) + shared_index];
            }
            output[(left_row * right_rows) + right_row] = accumulator;
        }
    }
}

/**
 * @brief 对称化矩阵：P = (P + P^T) / 2
 * @param matrix 矩阵指针（会被修改）
 * @param dimension 矩阵维度
 * @note 用于消除数值误差导致的非对称性
 */
void alg_kalman_internal_symmetrize(float *matrix, size_t dimension)
{
    size_t row;
    size_t column;
    float average;

    for (row = 0U; row < dimension; ++row)
    {
        for (column = row + 1U; column < dimension; ++column)
        {
            average =
                0.5F * (matrix[(row * dimension) + column] + matrix[(column * dimension) + row]);
            matrix[(row * dimension) + column] = average;
            matrix[(column * dimension) + row] = average;
        }
    }
}

/* ======================== 卡尔曼校正核心 ======================== */

/**
 * @brief 执行卡尔曼校正（Joseph 形式协方差更新）
 * @param state 状态向量（会被原地更新）
 * @param covariance 协方差矩阵（会被原地更新）
 * @param state_dimension 状态维度 n
 * @param measurement_matrix 观测矩阵 H（m×n，行优先）
 * @param measurement_noise 测量噪声矩阵 R（m×m，行优先）
 * @param measurement 测量值向量 z（m×1）
 * @param predicted_measurement 预测测量值 h(x)（m×1）
 * @param measurement_dimension 测量维度 m
 * @param workspace 工作区（调用者预分配）
 * @param workspace_size 工作区大小（float 元素数）
 * @return 执行状态
 *
 * @par Joseph 形式协方差更新 P = (I-KH)*P*(I-KH)^T + K*R*K^T
 *       数值稳定性优于直接形式 P = (I-KH)*P。
 *       不要求 H 和 R 是常量——EKF 中 H 在每次校正时重新计算。
 *
 * @par 10 步完整流程：
 *
 * **1. 计算创新残差 y = z - h(x)**（逐元素减法，m 维）：
 * 创新 y 表示测量值与预测值的差异。
 *
 * **2. 计算 H*P**（矩阵乘法，m×n 结果）：
 * 将状态协方差通过测量雅可比映射到测量空间的第一部分。
 *
 * **3. 计算创新协方差 S = H*P*H^T + R**（m×m 对称阵）：
 * S 是创新 y 的理论协方差。两个分量：
 * - H*P*H^T：状态不确定度投射到测量空间
 * - R：测量噪声本身的协方差
 *
 * **4. 计算 P*H^T**（通过对 H*P 转置获取，避免重复乘法）：
 * 用于计算卡尔曼增益 K = P*H^T * S^-1。
 *
 * **5. 求 S 的逆矩阵 S^-1**（Gauss-Jordan 消元）：
 * 对 m×m 矩阵求逆，m 通常很小（IMU EKF 中 m=3）。
 *
 * **6. 计算卡尔曼增益 K = P*H^T * S^-1**（n×m 矩阵）：
 * K 的每个元素给出"该状态分量对该测量分量的修正敏感度"。
 * 行对应于状态，列对应于测量分量。
 *
 * **7. 状态更新 x = x + K*y**（矩阵乘向量 + 逐元素加法）：
 * K*y 将创新从测量空间映射回状态空间，叠加到原状态。
 *
 * **8. Joseph 协方差更新（6 个子步骤）：**
 * 8.1 计算 I - K*H（n×n，逐元素求反再加 1）
 * 8.2 计算 (I-KH)*P（n×n）
 * 8.3 计算 (I-KH)*P*(I-KH)^T（右转置乘法）
 * 8.4 计算 K*R（n×m）
 * 8.5 计算 (K*R)*K^T（右转置乘法，n×n）
 * 8.6 合并两项：P_new = 第 8.3 项 + 第 8.5 项
 *
 * **9. 对称化** P = (P + P^T)/2：
 * 浮点舍入误差会导致轻微不对称，强制对称化保证数值正确性。
 *
 * **10. 有效性检查与提交**：
 * 检查状态和新协方差的每个元素是否为有限数，
 * 然后拷贝回原数组完成更新。
 */
alg_kalman_status_t
alg_kalman_internal_correct(float *state, float *covariance, size_t state_dimension,
                            const float *measurement_matrix, const float *measurement_noise,
                            const float *measurement, const float *predicted_measurement,
                            size_t measurement_dimension, float *workspace, size_t workspace_size)
{
    // ---- 计算工作区大小需求 ----
    const size_t state_square = state_dimension * state_dimension;
    const size_t measurement_square = measurement_dimension * measurement_dimension;
    const size_t cross_size = state_dimension * measurement_dimension;
    const size_t required_size = state_dimension + (3U * state_square) + (3U * cross_size) +
                                 measurement_dimension + (2U * measurement_square);

    // ---- 工作区指针分配 ----
    float *new_state;                        // 更新后的状态
    float *innovation;                       // 创新残差 y = z - h(x)
    float *measurement_covariance_product;   // H*P（m×n）
    float *innovation_covariance;            // S = H*P*H^T + R（m×m）
    float *innovation_covariance_inverse;    // S^-1（m×m）
    float *covariance_measurement_transpose; // P*H^T（n×m）
    float *gain;                             // 卡尔曼增益 K（n×m）
    float *identity_minus_gain_measurement;  // I - K*H（n×n）
    float *temporary_state_square;           // 临时 n×n 矩阵
    float *new_covariance;                   // 更新后的协方差

    size_t state_index;
    size_t measurement_index;
    size_t index;
    alg_kalman_status_t status;

    // ---- 检查工作区是否足够 ----
    if (workspace_size < required_size) {
        return ALG_KALMAN_STATUS_INSUFFICIENT_WORKSPACE;
}

    // ---- 分配工作区 ----
    new_state = workspace;
    innovation = new_state + state_dimension;
    measurement_covariance_product = innovation + measurement_dimension;
    innovation_covariance = measurement_covariance_product + cross_size;
    innovation_covariance_inverse = innovation_covariance + measurement_square;
    covariance_measurement_transpose = innovation_covariance_inverse + measurement_square;
    gain = covariance_measurement_transpose + cross_size;
    identity_minus_gain_measurement = gain + cross_size;
    temporary_state_square = identity_minus_gain_measurement + state_square;
    new_covariance = temporary_state_square + state_square;

    // ---- 1. 计算创新残差 y = z - h(x) ----
    for (measurement_index = 0U; measurement_index < measurement_dimension; ++measurement_index) {
        innovation[measurement_index] =
            measurement[measurement_index] - predicted_measurement[measurement_index];
}

    // ---- 2. 计算 H*P ----
    alg_kalman_internal_multiply(measurement_matrix, measurement_dimension, state_dimension,
                                 covariance, state_dimension, measurement_covariance_product);

    // ---- 3. 计算创新协方差 S = H*P*H^T + R ----
    alg_kalman_internal_multiply_right_transpose(
        measurement_covariance_product, measurement_dimension, state_dimension, measurement_matrix,
        measurement_dimension, innovation_covariance);
    for (index = 0U; index < measurement_square; ++index) {
        innovation_covariance[index] += measurement_noise[index];
}

    // ---- 4. 计算 P*H^T（转置） ----
    // 注意：由于 H*P 已知，P*H^T 是 H*P 的转置
    alg_kalman_internal_copy(new_state, state, state_dimension);
    for (state_index = 0U; state_index < state_dimension; ++state_index)
    {
        for (measurement_index = 0U; measurement_index < measurement_dimension; ++measurement_index)
        {
            covariance_measurement_transpose[(state_index * measurement_dimension) +
                                             measurement_index] =
                measurement_covariance_product[(measurement_index * state_dimension) + state_index];
        }
    }

    // ---- 5. 求 S 的逆矩阵 ----
    status = alg_kalman_internal_invert(innovation_covariance, innovation_covariance_inverse,
                                        measurement_dimension);
    if (status != ALG_KALMAN_STATUS_OK) {
        return status;
}

    // ---- 6. 计算卡尔曼增益 K = P*H^T * S^-1 ----
    alg_kalman_internal_multiply(covariance_measurement_transpose, state_dimension,
                                 measurement_dimension, innovation_covariance_inverse,
                                 measurement_dimension, gain);

    // ---- 7. 状态更新 x = x + K*y ----
    for (state_index = 0U; state_index < state_dimension; ++state_index)
    {
        for (measurement_index = 0U; measurement_index < measurement_dimension; ++measurement_index)
        {
            new_state[state_index] +=
                gain[(state_index * measurement_dimension) + measurement_index] *
                innovation[measurement_index];
        }
    }

    // ---- 8. 协方差更新（Joseph 形式） ----
    // P = (I-KH)*P*(I-KH)^T + K*R*K^T
    // 8.1 计算 I - K*H
    alg_kalman_internal_multiply(gain, state_dimension, measurement_dimension, measurement_matrix,
                                 state_dimension, identity_minus_gain_measurement);
    for (index = 0U; index < state_square; ++index) {
        identity_minus_gain_measurement[index] = -identity_minus_gain_measurement[index];
}
    for (state_index = 0U; state_index < state_dimension; ++state_index) {
        identity_minus_gain_measurement[(state_index * state_dimension) + state_index] += 1.0F;
}

    // 8.2 (I-KH)*P
    alg_kalman_internal_multiply(identity_minus_gain_measurement, state_dimension, state_dimension,
                                 covariance, state_dimension, temporary_state_square);

    // 8.3 (I-KH)*P*(I-KH)^T
    alg_kalman_internal_multiply_right_transpose(temporary_state_square, state_dimension,
                                                 state_dimension, identity_minus_gain_measurement,
                                                 state_dimension, new_covariance);

    // 8.4 K*R*K^T
    alg_kalman_internal_multiply(gain, state_dimension, measurement_dimension, measurement_noise,
                                 measurement_dimension, covariance_measurement_transpose);
    alg_kalman_internal_multiply_right_transpose(covariance_measurement_transpose, state_dimension,
                                                 measurement_dimension, gain, state_dimension,
                                                 temporary_state_square);

    // 8.5 合并两项
    for (index = 0U; index < state_square; ++index) {
        new_covariance[index] += temporary_state_square[index];
}

    // ---- 9. 对称化并检查结果 ----
    alg_kalman_internal_symmetrize(new_covariance, state_dimension);
    if (!alg_kalman_internal_is_finite_array(new_state, state_dimension) ||
        !alg_kalman_internal_is_finite_array(new_covariance, state_square)) {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
}

    // ---- 10. 提交更新 ----
    alg_kalman_internal_copy(state, new_state, state_dimension);
    alg_kalman_internal_copy(covariance, new_covariance, state_square);

    return ALG_KALMAN_STATUS_OK;
}