/**
 * @file alg_math_matrix.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 动态矩阵运算实现（加减、乘、转置、求逆、求解、Cholesky）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 所有矩阵使用行优先存储。
 *       求逆/求解采用部分主元 Gauss-Jordan/Gauss 消元。
 *       Cholesky 要求对称正定，输入与输出数据指针不可复用。
 */

#include "alg_math.h"
#include <math.h>

/** @brief 奇异矩阵判定阈值（主元绝对值低于此值视为奇异） */
#define ALG_MATH_MATRIX_PIVOT_THRESHOLD (1.0e-12F)

/* ======================== 内部辅助 ======================== */

/**
 * @brief 检查矩阵描述符是否有效（非空、尺寸合法、不溢出）
 * @param matrix 矩阵描述符指针
 * @return true=有效
 */
static bool alg_math_matrix_is_valid(const alg_math_matrix_t *matrix)
{
    return (matrix != NULL) && (matrix->data != NULL) && (matrix->rows > 0U) &&
           (matrix->columns > 0U) && (matrix->rows <= (SIZE_MAX / matrix->columns));
}

/**
 * @brief 检查两个矩阵尺寸是否相同（行数和列数均相等）
 * @param left 矩阵 A
 * @param right 矩阵 B
 * @return true=尺寸相同
 */
static bool alg_math_matrix_has_same_size(const alg_math_matrix_t *left,
                                          const alg_math_matrix_t *right)
{
    return (left->rows == right->rows) && (left->columns == right->columns);
}

/* ======================== 初始化与基础操作 ======================== */

/**
 * @brief 初始化矩阵描述符（绑定数据缓冲区）
 * @param matrix 矩阵描述符指针
 * @param data 数据缓冲区（调用者负责分配，行优先存储）
 * @param rows 行数
 * @param columns 列数
 * @return 执行状态
 * @note 仅设置元数据，不修改数据内容；需自行清零或赋值
 */
alg_math_status_t alg_math_matrix_init(alg_math_matrix_t *matrix, float *data, size_t rows,
                                       size_t columns)
{
    if ((matrix == NULL) || (data == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if ((rows == 0U) || (columns == 0U) || (rows > (SIZE_MAX / columns))) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    matrix->rows = rows;
    matrix->columns = columns;
    matrix->data = data;
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 矩阵所有元素置零
 * @param matrix 矩阵描述符
 * @return 执行状态
 */
alg_math_status_t alg_math_matrix_zero(alg_math_matrix_t *matrix)
{
    size_t index;
    if (!alg_math_matrix_is_valid(matrix)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    for (index = 0U; index < (matrix->rows * matrix->columns); ++index) {
        matrix->data[index] = 0.0F;
}
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 设置单位矩阵（对角线为 1，其余为 0）
 * @param matrix 方阵描述符
 * @return 执行状态，非方阵返回 SIZE_MISMATCH
 */
alg_math_status_t alg_math_matrix_identity(alg_math_matrix_t *matrix)
{
    size_t index;
    if (!alg_math_matrix_is_valid(matrix)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (matrix->rows != matrix->columns) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    (void)alg_math_matrix_zero(matrix);
    for (index = 0U; index < matrix->rows; ++index) {
        matrix->data[(index * matrix->columns) + index] = 1.0F;
}
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 复制矩阵：destination = source（逐元素）
 * @param source 源矩阵
 * @param destination 目标矩阵（须与源同尺寸，不得为同一对象）
 * @return 执行状态
 * @note 需预先初始化 destination 描述符并分配缓冲区
 */
alg_math_status_t alg_math_matrix_copy(const alg_math_matrix_t *source,
                                       alg_math_matrix_t *destination)
{
    size_t index;
    if (!alg_math_matrix_is_valid(source) || !alg_math_matrix_is_valid(destination)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!alg_math_matrix_has_same_size(source, destination)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    if (!alg_math_is_finite_array(source->data, source->rows * source->columns)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    for (index = 0U; index < (source->rows * source->columns); ++index) {
        destination->data[index] = source->data[index];
}
    return ALG_MATH_STATUS_OK;
}

/* ======================== 矩阵算术 ======================== */

/**
 * @brief 矩阵逐元素运算（加法/减法内部复用）
 * @param left 左操作数矩阵
 * @param right 右操作数矩阵
 * @param right_scale 右操作数缩放因子（+1=加法，-1=减法）
 * @param result 输出矩阵（不得与输入复用）
 * @return 执行状态
 * @note result[i] = left[i] + right_scale * right[i]
 */
static alg_math_status_t alg_math_matrix_elementwise(const alg_math_matrix_t *left,
                                                     const alg_math_matrix_t *right,
                                                     float right_scale, alg_math_matrix_t *result)
{
    size_t index;
    float value;
    if (!alg_math_matrix_is_valid(left) || !alg_math_matrix_is_valid(right) ||
        !alg_math_matrix_is_valid(result)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!alg_math_matrix_has_same_size(left, right) || !alg_math_matrix_has_same_size(left, result)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    if (!alg_math_is_finite_array(left->data, left->rows * left->columns) ||
        !alg_math_is_finite_array(right->data, right->rows * right->columns)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    for (index = 0U; index < (left->rows * left->columns); ++index)
    {
        value = left->data[index] + (right_scale * right->data[index]);
        if (!isfinite(value)) {
            return ALG_MATH_STATUS_NUMERICAL_ERROR;
}
        result->data[index] = value;
    }
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 矩阵加法：result = left + right
 * @param left 左操作数矩阵
 * @param right 右操作数矩阵
 * @param result 输出矩阵（不得与输入复用）
 * @return 执行状态
 */
alg_math_status_t alg_math_matrix_add(const alg_math_matrix_t *left, const alg_math_matrix_t *right,
                                      alg_math_matrix_t *result)
{
    return alg_math_matrix_elementwise(left, right, 1.0F, result);
}

/**
 * @brief 矩阵减法：result = left - right
 * @param left 左操作数矩阵
 * @param right 右操作数矩阵
 * @param result 输出矩阵（不得与输入复用）
 * @return 执行状态
 */
alg_math_status_t alg_math_matrix_subtract(const alg_math_matrix_t *left,
                                           const alg_math_matrix_t *right,
                                           alg_math_matrix_t *result)
{
    return alg_math_matrix_elementwise(left, right, -1.0F, result);
}

/**
 * @brief 矩阵缩放：result = input * scale
 * @param input 输入矩阵
 * @param scale 缩放因子
 * @param result 输出矩阵（不得与输入复用）
 * @return 执行状态
 */
alg_math_status_t alg_math_matrix_scale(const alg_math_matrix_t *input, float scale,
                                        alg_math_matrix_t *result)
{
    size_t index;
    float value;
    if (!alg_math_matrix_is_valid(input) || !alg_math_matrix_is_valid(result)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!alg_math_matrix_has_same_size(input, result)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    if (!isfinite(scale) || !alg_math_is_finite_array(input->data, input->rows * input->columns)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    for (index = 0U; index < (input->rows * input->columns); ++index)
    {
        value = input->data[index] * scale;
        if (!isfinite(value)) {
            return ALG_MATH_STATUS_NUMERICAL_ERROR;
}
        result->data[index] = value;
    }
    return ALG_MATH_STATUS_OK;
}

/* ======================== 矩阵乘法与转置 ======================== */

/**
 * @brief 矩阵乘法：result = left * right
 * @param left 左操作数矩阵 (M x K)
 * @param right 右操作数矩阵 (K x N)
 * @param result 输出矩阵 (M x N)，不得与输入复用同一缓冲区
 * @return 执行状态
 * @note 三重循环 O(M*K*N)，result 数据指针不得等于 left 或 right
 */
alg_math_status_t alg_math_matrix_multiply(const alg_math_matrix_t *left,
                                           const alg_math_matrix_t *right,
                                           alg_math_matrix_t *result)
{
    size_t row, column, shared_index;
    float acc;
    if (!alg_math_matrix_is_valid(left) || !alg_math_matrix_is_valid(right) ||
        !alg_math_matrix_is_valid(result)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if ((left->columns != right->rows) || (result->rows != left->rows) ||
        (result->columns != right->columns)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    if ((result->data == left->data) || (result->data == right->data)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!alg_math_is_finite_array(left->data, left->rows * left->columns) ||
        !alg_math_is_finite_array(right->data, right->rows * right->columns)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    // ---- result = left * right，三重循环逐元素计算 ----
    for (row = 0U; row < result->rows; ++row)
    {
        for (column = 0U; column < result->columns; ++column)
        {
            // 计算第 row 行与第 column 列的内积（点积）
            acc = 0.0F;
            for (shared_index = 0U; shared_index < left->columns; ++shared_index) {
                acc += left->data[(row * left->columns) + shared_index] *
                       right->data[(shared_index * right->columns) + column];
}
            if (!isfinite(acc)) {
                return ALG_MATH_STATUS_NUMERICAL_ERROR;
}
            result->data[(row * result->columns) + column] = acc;
        }
    }
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 矩阵转置：result = input^T
 * @param input 输入矩阵
 * @param result 输出矩阵（可与 input 同一缓冲区，仅限方阵原地转置）
 * @return 执行状态
 * @note 原地转置通过交换上三角元素实现，仅支持方阵
 */
alg_math_status_t alg_math_matrix_transpose(const alg_math_matrix_t *input,
                                            alg_math_matrix_t *result)
{
    size_t row, column;
    float temporary_value;
    if (!alg_math_matrix_is_valid(input) || !alg_math_matrix_is_valid(result)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if ((result->rows != input->columns) || (result->columns != input->rows)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    if (!alg_math_is_finite_array(input->data, input->rows * input->columns)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    if (input->data == result->data)
    {
        // ---- 原地转置：仅限方阵 ----
        if (input->rows != input->columns) {
            return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
        for (row = 0U; row < input->rows; ++row) {
            for (column = row + 1U; column < input->columns; ++column)
            {
                temporary_value = result->data[(row * result->columns) + column];
                result->data[(row * result->columns) + column] =
                    result->data[(column * result->columns) + row];
                result->data[(column * result->columns) + row] = temporary_value;
            }
}
        return ALG_MATH_STATUS_OK;
    }

    // ---- 非原地转置 ----
    for (row = 0U; row < input->rows; ++row) {
        for (column = 0U; column < input->columns; ++column) {
            result->data[(column * result->columns) + row] =
                input->data[(row * input->columns) + column];
}
}
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 矩阵乘向量：result = matrix * vector
 * @param matrix 输入矩阵 (M x N)
 * @param vector 输入向量（长度 N）
 * @param vector_length 向量长度
 * @param result 输出向量（长度 M），不得指向 vector
 * @param result_length 结果长度
 * @return 执行状态
 */
alg_math_status_t alg_math_matrix_multiply_vector(const alg_math_matrix_t *matrix,
                                                  const float *vector, size_t vector_length,
                                                  float *result, size_t result_length)
{
    size_t row, column;
    float acc;
    if (!alg_math_matrix_is_valid(matrix) || (vector == NULL) || (result == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if ((vector_length != matrix->columns) || (result_length != matrix->rows)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    if (vector == result) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!alg_math_is_finite_array(matrix->data, matrix->rows * matrix->columns) ||
        !alg_math_is_finite_array(vector, vector_length)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    for (row = 0U; row < matrix->rows; ++row)
    {
        // 计算矩阵第 row 行与列向量的点积
        acc = 0.0F;
        for (column = 0U; column < matrix->columns; ++column) {
            acc += matrix->data[(row * matrix->columns) + column] * vector[column];
}
        if (!isfinite(acc)) {
            return ALG_MATH_STATUS_NUMERICAL_ERROR;
}
        result[row] = acc;
    }
    return ALG_MATH_STATUS_OK;
}

/* ======================== 线性代数 ======================== */

/**
 * @brief 矩阵行交换（用于求逆/求解的部分主元选择）
 * @param matrix 矩阵数据（行优先）
 * @param column_count 列数
 * @param first_row 第一行索引
 * @param second_row 第二行索引
 */
static void alg_math_matrix_swap_rows(float *matrix, size_t column_count, size_t first_row,
                                      size_t second_row)
{
    size_t col;
    float temporary_value;
    for (col = 0U; col < column_count; ++col)
    {
        temporary_value = matrix[(first_row * column_count) + col];
        matrix[(first_row * column_count) + col] = matrix[(second_row * column_count) + col];
        matrix[(second_row * column_count) + col] = temporary_value;
    }
}

/**
 * @brief 矩阵求逆（Gauss-Jordan 消元，部分主元）
 * @param input 输入方阵 (N x N)
 * @param inverse 输出逆矩阵 (N x N)，不得与 input 共用缓冲区
 * @param workspace 工作区缓冲区
 * @param workspace_size 工作区大小（字节数）
 * @return 执行状态，奇异矩阵返回 SINGULAR
 * @note 算法流程：
 *       1. 构造增广矩阵 [A | I]（大小 N x 2N）
 *       2. 对每一列选择绝对值最大的主元（部分主元）
 *       3. 归一化主元行
 *       4. 消去其他行的当前列
 *       5. 提取右侧 N x N 子矩阵作为 A^{-1}
 */
alg_math_status_t alg_math_matrix_invert(const alg_math_matrix_t *input, alg_math_matrix_t *inverse,
                                         float *workspace, size_t workspace_size)
{
    size_t order, aug_cols, row, col, pivot_row;
    float pivot_mag, cand_mag, pivot, factor;

    if (!alg_math_matrix_is_valid(input) || !alg_math_matrix_is_valid(inverse) ||
        (workspace == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if ((input->rows != input->columns) || !alg_math_matrix_has_same_size(input, inverse)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    order = input->rows;
    if (workspace_size < ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(order)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    if (!alg_math_is_finite_array(input->data, order * order)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    aug_cols = 2U * order;

    // ---- 构造增广矩阵 [A | I] ----
    for (row = 0U; row < order; ++row)
    {
        for (col = 0U; col < order; ++col) {
            workspace[(row * aug_cols) + col] = input->data[(row * order) + col];
}
        for (col = 0U; col < order; ++col) {
            workspace[(row * aug_cols) + order + col] = (row == col) ? 1.0F : 0.0F;
}
    }

    // ---- Gauss-Jordan 消元 ----
    for (col = 0U; col < order; ++col)
    {
        // 选择主元（列最大值）
        pivot_row = col;
        pivot_mag = fabsf(workspace[(col * aug_cols) + col]);
        for (row = col + 1U; row < order; ++row)
        {
            cand_mag = fabsf(workspace[(row * aug_cols) + col]);
            if (cand_mag > pivot_mag)
            {
                pivot_mag = cand_mag;
                pivot_row = row;
            }
        }
        if (pivot_mag <= ALG_MATH_MATRIX_PIVOT_THRESHOLD) {
            return ALG_MATH_STATUS_SINGULAR;
}

        // 交换行
        if (pivot_row != col) {
            alg_math_matrix_swap_rows(workspace, aug_cols, col, pivot_row);
}

        // 归一化主元行
        pivot = workspace[(col * aug_cols) + col];
        for (size_t idx = 0U; idx < aug_cols; ++idx) {
            workspace[(col * aug_cols) + idx] /= pivot;
}

        // ---- 消去其他行的当前列：row_i := row_i - factor * pivot_row ----
        for (row = 0U; row < order; ++row)
        {
            if (row == col) {
                continue;
}
            // 消去因子 = 待消行当前列的元素值
            factor = workspace[(row * aug_cols) + col];
            for (size_t idx = 0U; idx < aug_cols; ++idx) {
                workspace[(row * aug_cols) + idx] -= factor * workspace[(col * aug_cols) + idx];
}
        }
    }

    // ---- 提取逆矩阵 ----
    for (row = 0U; row < order; ++row) {
        for (col = 0U; col < order; ++col) {
            inverse->data[(row * order) + col] = workspace[(row * aug_cols) + order + col];
}
}

    return alg_math_is_finite_array(inverse->data, order * order) ? ALG_MATH_STATUS_OK
                                                                  : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 线性方程组求解 Ax = b（Gauss 消元，部分主元）
 * @param coefficients 系数矩阵 A (N x N)
 * @param right_hand_side 右端向量 b（长度 N）
 * @param solution 输出解向量 x（长度 N）
 * @param workspace 工作区缓冲区
 * @param workspace_size 工作区大小（字节数）
 * @return 执行状态，奇异矩阵返回 SINGULAR
 * @note 算法流程：
 *       1. 构造增广矩阵 [A | b]
 *       2. 前向消元（部分主元选择 + 行消除）
 *       3. 回代求解 x
 */
alg_math_status_t alg_math_matrix_solve(const alg_math_matrix_t *coefficients,
                                        const float *right_hand_side, float *solution,
                                        float *workspace, size_t workspace_size)
{
    size_t order, aug_cols, row, col, pivot_row;
    float pivot_mag, cand_mag, factor, acc;

    if (!alg_math_matrix_is_valid(coefficients) || (right_hand_side == NULL) ||
        (solution == NULL) || (workspace == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (coefficients->rows != coefficients->columns) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    order = coefficients->rows;
    if (workspace_size < ALG_MATH_MATRIX_SOLVE_WORKSPACE_SIZE(order)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    if (!alg_math_is_finite_array(coefficients->data, order * order) ||
        !alg_math_is_finite_array(right_hand_side, order)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    aug_cols = order + 1U;

    // ---- 构造增广矩阵 [A | b] ----
    for (row = 0U; row < order; ++row)
    {
        for (col = 0U; col < order; ++col) {
            workspace[(row * aug_cols) + col] = coefficients->data[(row * order) + col];
}
        workspace[(row * aug_cols) + order] = right_hand_side[row];
    }

    // ---- 前向消元（部分主元） ----
    for (col = 0U; col < order; ++col)
    {
        pivot_row = col;
        pivot_mag = fabsf(workspace[(col * aug_cols) + col]);
        for (row = col + 1U; row < order; ++row)
        {
            cand_mag = fabsf(workspace[(row * aug_cols) + col]);
            if (cand_mag > pivot_mag)
            {
                pivot_mag = cand_mag;
                pivot_row = row;
            }
        }
        if (pivot_mag <= ALG_MATH_MATRIX_PIVOT_THRESHOLD) {
            return ALG_MATH_STATUS_SINGULAR;
}
        if (pivot_row != col) {
            alg_math_matrix_swap_rows(workspace, aug_cols, col, pivot_row);
}

        // 计算消去因子：factor = A[row][col] / A[col][col]
        for (row = col + 1U; row < order; ++row)
        {
            factor = workspace[(row * aug_cols) + col] / workspace[(col * aug_cols) + col];
            // 主元下方元素置零（理论上应为 0）
            workspace[(row * aug_cols) + col] = 0.0F;
            for (size_t idx = col + 1U; idx < aug_cols; ++idx) {
                workspace[(row * aug_cols) + idx] -= factor * workspace[(col * aug_cols) + idx];
}
        }
    }

    // ---- 回代求解：自底向上计算 x[n-1] 到 x[0] ----
    for (row = order; row-- > 0U;)
    {
        // x[row] = (b[row] - sum(A[row][j] * x[j] for j > row)) / A[row][row]
        acc = workspace[(row * aug_cols) + order];
        for (col = row + 1U; col < order; ++col) {
            acc -= workspace[(row * aug_cols) + col] * solution[col];
}
        if (fabsf(workspace[(row * aug_cols) + row]) <= ALG_MATH_MATRIX_PIVOT_THRESHOLD) {
            return ALG_MATH_STATUS_SINGULAR;
}
        solution[row] = acc / workspace[(row * aug_cols) + row];
    }
    return alg_math_is_finite_array(solution, order) ? ALG_MATH_STATUS_OK
                                                     : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief Cholesky 分解 A = L * L^T（要求对称正定）
 * @param input 输入矩阵 A (N x N)
 * @param lower_triangular 输出下三角矩阵 L，不得与 input 共用缓冲区
 * @return 执行状态，非对称返回 OUT_OF_RANGE，非正定返回 SINGULAR
 * @note 算法流程：
 *       1. 检查近似对称性（容差 1e-5 相对误差）
 *       2. 按行逐列计算 L
 *       3. 对角线元素 L[i][i] = sqrt(A[i][i] - sum(L[i][k]^2))
 *       4. 非对角线元素 L[i][j] = (A[i][j] - sum(L[i][k]*L[j][k])) / L[j][j]
 */
alg_math_status_t alg_math_matrix_cholesky(const alg_math_matrix_t *input,
                                           alg_math_matrix_t *lower_triangular)
{
    size_t row, col, k;
    float acc;

    if (!alg_math_matrix_is_valid(input) || !alg_math_matrix_is_valid(lower_triangular)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if ((input->rows != input->columns) || !alg_math_matrix_has_same_size(input, lower_triangular)) {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
}
    if (input->data == lower_triangular->data) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!alg_math_is_finite_array(input->data, input->rows * input->columns)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    // ---- 检查对称性（近似） ----
    for (row = 0U; row < input->rows; ++row) {
        for (col = row + 1U; col < input->columns; ++col)
        {
            float upper = input->data[(row * input->columns) + col];
            float lower = input->data[(col * input->columns) + row];
            float tol = 1.0e-5F * (1.0F + fmaxf(fabsf(upper), fabsf(lower)));
            if (fabsf(upper - lower) > tol) {
                return ALG_MATH_STATUS_OUT_OF_RANGE;
}
        }
}

    (void)alg_math_matrix_zero(lower_triangular);

    // ---- Cholesky 分解：A = L * L^T，逐行逐列计算下三角矩阵 L ----
    for (row = 0U; row < input->rows; ++row)
    {
        for (col = 0U; col <= row; ++col)
        {
            // 取 A[row][col]，减去前面列的累积贡献 sum(L[row][k] * L[col][k])
            acc = input->data[(row * input->columns) + col];
            for (k = 0U; k < col; ++k) {
                acc -= lower_triangular->data[(row * lower_triangular->columns) + k] *
                       lower_triangular->data[(col * lower_triangular->columns) + k];
}

            if (row == col)
            {
                // 对角线元素：L[i][i] = sqrt(A[i][i] - sum(L[i][k]^2))
                if (acc <= ALG_MATH_MATRIX_PIVOT_THRESHOLD) {
                    return ALG_MATH_STATUS_SINGULAR;
}
                lower_triangular->data[(row * lower_triangular->columns) + col] = sqrtf(acc);
            }
            else
            {
                // 非对角线元素：L[i][j] = (A[i][j] - sum(L[i][k]*L[j][k])) / L[j][j]
                lower_triangular->data[(row * lower_triangular->columns) + col] =
                    acc / lower_triangular->data[(col * lower_triangular->columns) + col];
            }
        }
    }
    return alg_math_is_finite_array(lower_triangular->data,
                                    lower_triangular->rows * lower_triangular->columns)
               ? ALG_MATH_STATUS_OK
               : ALG_MATH_STATUS_NUMERICAL_ERROR;
}
