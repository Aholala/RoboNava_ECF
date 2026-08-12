/**
 * @file module_device.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模块设备统一基类头文件
 * @version 2.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 提供所有非电机模块的统一 C11 对象基类。派生对象将 super 作为第一个成员，
 *       并在各自的 .c 文件中持有 static const module_device_ops_t 虚表。
 *       基类不分配内存，也不依赖 MCU 或厂商 HAL。
 *
 *       对象和状态码保持在 Module 层。
 */

#ifndef MODULE_DEVICE_H
#define MODULE_DEVICE_H

#include <stdbool.h> // bool
#include <stddef.h>  // size_t, offsetof
#include <stdint.h>  // uint32_t

#ifdef __cplusplus
extern "C"
{
#endif

    /* ========================================================================
     * 容器宏 — 别名到 ECF 统一宏
     * ======================================================================== */

#define MODULE_CONTAINER_OF(member_pointer, parent_type, member_name)                              \
    ((parent_type *)((uint8_t *)(member_pointer) - offsetof(parent_type, member_name)))
#define MODULE_CONTAINER_OF_CONST(member_pointer, parent_type, member_name)                        \
    ((const parent_type *)((const uint8_t *)(member_pointer) - offsetof(parent_type, member_name)))
#define MODULE_STATIC_ASSERT_SUPER_FIRST(derived_type)                                             \
    _Static_assert(offsetof(derived_type, super) == 0U, #derived_type " must place super first")
#define MODULE_DEVICE_OBJECT_MAGIC (0x4D444556UL)

    /* ========================================================================
     * 状态码 — 别名到 ECF 统一状态码
     * ======================================================================== */

    typedef enum
    {
        MODULE_DEVICE_STATUS_OK = 0,
        MODULE_DEVICE_STATUS_INVALID_ARGUMENT,
        MODULE_DEVICE_STATUS_NOT_INITIALIZED,
        MODULE_DEVICE_STATUS_ALREADY_INITIALIZED,
        MODULE_DEVICE_STATUS_UNSUPPORTED,
        MODULE_DEVICE_STATUS_OPERATION_FAILED
    } module_device_status_t;

    /* ======================== 前向声明 ======================== */

    typedef struct module_device module_device_t;

    /* ======================== 虚操作表 ======================== */

    /**
     * @brief 设备虚操作表（由派生类在 .c 中静态定义）
     * @note 不支持的操作可以为 NULL。
     */
    typedef struct
    {
        module_device_status_t (*start)(module_device_t *const me);
        module_device_status_t (*stop)(module_device_t *const me);
        module_device_status_t (*update)(module_device_t *const me, uint32_t elapsed_time_ms);
    } module_device_ops_t;

    /* ======================== 基类结构体 ======================== */

    /**
     * @brief 模块设备基类
     * @note 派生类必须将 super 作为第一个成员
     */
    struct module_device
    {
        const module_device_ops_t *vptr; // 虚表指针（只读）
        const char *logical_name;        // 逻辑名称（便于日志诊断）
        uint32_t registration_key;       // 注册键值（稳定数字标识）
        uint32_t object_magic;           // 魔数（MODULE_DEVICE_OBJECT_MAGIC）
        bool is_initialized;             // 初始化完成标志
    };

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化基类（第一阶段构造）
     * @param me 设备对象
     * @param vptr 虚表指针
     * @param logical_name 逻辑名称
     * @param registration_key 注册键值
     * @return 执行状态
     * @note 只填充基类字段，不标记为已初始化。
     *       派生类完成自己的资源初始化后调用 module_device_complete_init 提交。
     */
    module_device_status_t module_device_init_base(module_device_t *const me,
                                                   const module_device_ops_t *const vptr,
                                                   const char *const logical_name,
                                                   uint32_t registration_key);

    /**
     * @brief 完成初始化（第二阶段构造）
     * @param me 设备对象
     * @return 执行状态
     * @note 设置 is_initialized = true，提交对象为有效状态。
     *       调用前必须确保派生类资源已成功初始化。
     */
    module_device_status_t module_device_complete_init(module_device_t *const me);

    /**
     * @brief 中止初始化（清理状态）
     * @param me 设备对象
     * @note 清除所有字段，留下可识别的未初始化对象。
     *       在派生类资源初始化失败时调用。
     */
    void module_device_abort_init(module_device_t *const me);

    /**
     * @brief 启动设备（调用虚表 start）
     * @param me 设备对象
     * @return 执行状态
     */
    module_device_status_t module_device_start(module_device_t *const me);

    /**
     * @brief 停止设备（调用虚表 stop）
     * @param me 设备对象
     * @return 执行状态
     */
    module_device_status_t module_device_stop(module_device_t *const me);

    /**
     * @brief 更新设备（调用虚表 update）
     * @param me 设备对象
     * @param elapsed_time_ms 距上次更新的时间（毫秒）
     * @return 执行状态
     */
    module_device_status_t module_device_update(module_device_t *const me,
                                                uint32_t elapsed_time_ms);

    /**
     * @brief 检查设备是否已初始化
     * @param me 设备对象
     * @return true=已初始化且有效
     */
    bool module_device_is_initialized(const module_device_t *const me);

    /**
     * @brief 获取逻辑名称
     * @param me 设备对象
     * @return 逻辑名称指针，若未初始化则返回 NULL
     */
    /**
     * @brief 获取注册键值
     * @param me 设备对象
     * @return 注册键值，若未初始化则返回 0
     */
#ifdef __cplusplus
}
#endif

#endif /* MODULE_DEVICE_H */
