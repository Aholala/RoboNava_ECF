/**
 * @file app_safety.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 安全应用模块接口 -- 心跳、失联与整机输出门控
 * @version 1.0
 * @date 2026-08-13
 * @copyright Copyright (c) 2026
 *
 * @note 定义安全监控器与安全实例结构体，管理心跳监控、可选硬件看门狗与全局电机输出门。
 */

#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include "bsp_common.h"
#include "bsp_watchdog.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_SAFETY_MAX_MONITOR_COUNT (16U)      /**< 最大监控器数量。 */
#define APP_SAFETY_DEFAULT_TASK_PERIOD_MS (5U)  /**< 默认安全任务周期 [ms]。 */

/** @brief 监控器在线状态。 */
typedef enum
{
    APP_SAFETY_STATE_STARTING = 0,  /**< 启动中，尚未收到首个心跳。 */
    APP_SAFETY_STATE_ONLINE,        /**< 在线。 */
    APP_SAFETY_STATE_OFFLINE        /**< 失联。 */
} app_safety_state_t;

/** @brief 监控器状态变化回调。 */
typedef void (*app_safety_callback_t)(void *user_context);

/** @brief 心跳监控器的静态配置。 */
typedef struct
{
    const char *name;                        /**< 监控器名称（用于日志）。 */
    uint32_t timeout_ms;                     /**< 心跳超时时间 [ms]。 */
    bool required;                           /**< 是否必需：失联时关闭整机输出。 */
    app_safety_callback_t offline_callback;  /**< 失联回调，可为 NULL。 */
    app_safety_callback_t online_callback;   /**< 在线回调，可为 NULL。 */
    void *user_context;                      /**< 回调用户上下文。 */
} app_safety_monitor_config_t;

/** @brief 单个心跳监控器实例。 */
typedef struct
{
    app_safety_monitor_config_t config;          /**< 静态配置的副本。 */
    volatile uint32_t last_online_time_ms;       /**< 上次收到心跳的时刻 [ms]。 */
    volatile bool heartbeat_received;            /**< 是否已收到心跳。 */
    uint32_t offline_time_ms;                    /**< 已失联时长 [ms]。 */
    app_safety_state_t state;                    /**< 当前在线状态。 */
    bool is_registered;                          /**< 已注册到安全实例。 */
} app_safety_monitor_t;

/** @brief 安全模块运行时实例。 */
typedef struct
{
    app_safety_monitor_t *monitors[APP_SAFETY_MAX_MONITOR_COUNT];  /**< 已注册监控器数组。 */
    size_t monitor_count;        /**< 已注册监控器数量。 */
    bsp_watchdog_t *watchdog;    /**< 可选硬件看门狗。 */
    bool output_enabled;         /**< 操作者已请求输出。 */
    bool output_allowed;         /**< 整机输出门（必需监控器在线且操作者已请求）。 */
    bool initialized;            /**< 初始化阶段已成功完成。 */
} app_safety_t;

/**
 * @brief  初始化安全模块，并关闭整机输出。
 * @param  me        指向调用方分配的实例。
 * @param  watchdog  可选硬件看门狗，可为 NULL。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_safety_init(app_safety_t *me, bsp_watchdog_t *watchdog);

/**
 * @brief  运行时更换硬件看门狗。
 * @param  me        已初始化的安全实例。
 * @param  watchdog  新看门狗，可为 NULL。
 * @return 成功返回 BSP_STATUS_OK，未初始化返回 BSP_STATUS_NOT_INITIALIZED。
 */
bsp_status_t app_safety_set_watchdog(app_safety_t *me, bsp_watchdog_t *watchdog);

/**
 * @brief  初始化单个心跳监控器。
 * @param  me      指向调用方分配的监控器实例。
 * @param  config  静态配置（内部拷贝）。
 * @return 成功返回 BSP_STATUS_OK，参数无效返回 BSP_STATUS_INVALID_ARGUMENT。
 */
bsp_status_t app_safety_monitor_init(app_safety_monitor_t *me,
                                     const app_safety_monitor_config_t *config);

/**
 * @brief  注册监控器到安全实例。
 * @param  me        已初始化的安全实例。
 * @param  monitor   已初始化的监控器。
 * @return 成功返回 BSP_STATUS_OK，已注册返回 BSP_STATUS_BUSY，超容量返回 BSP_STATUS_NO_RESOURCE。
 */
bsp_status_t app_safety_register(app_safety_t *me, app_safety_monitor_t *monitor);

/**
 * @brief  上报监控器在线（收到有效帧时调用）。
 * @param  monitor  监控器实例。
 * @param  now_ms   当前时刻 [ms]。
 */
void app_safety_notify_online(app_safety_monitor_t *monitor, uint32_t now_ms);

/**
 * @brief  执行一个安全周期：刷新监控状态并更新输出门。
 * @param  me      已初始化的安全实例。
 * @param  now_ms  当前时刻 [ms]。
 */
void app_safety_process(app_safety_t *me, uint32_t now_ms);

/**
 * @brief  设置操作者输出请求。
 * @param  me       已初始化的安全实例。
 * @param  enabled  是否请求输出。
 */
void app_safety_set_output_enabled(app_safety_t *me, bool enabled);

/**
 * @brief  查询整机输出门状态。
 * @param  me  安全实例。
 * @return 输出允许时为 true。
 */
bool app_safety_output_allowed(const app_safety_t *me);

/**
 * @brief  查询监控器在线状态。
 * @param  monitor  监控器实例。
 * @return 当前在线状态，实例为空时返回 APP_SAFETY_STATE_OFFLINE。
 */
app_safety_state_t app_safety_get_state(const app_safety_monitor_t *monitor);

/**
 * @brief  查询监控器已失联时长。
 * @param  monitor  监控器实例。
 * @return 失联时长 [ms]，实例为空时返回 UINT32_MAX。
 */
uint32_t app_safety_get_offline_time_ms(const app_safety_monitor_t *monitor);

/**
 * @brief  查询是否所有必需监控器均在线。
 * @param  me  已初始化的安全实例。
 * @return 存在必需监控器且全部在线时为 true。
 */
bool app_safety_all_required_online(const app_safety_t *me);

#endif
