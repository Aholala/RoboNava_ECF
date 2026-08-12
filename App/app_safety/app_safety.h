/**
 * @file app_safety.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 安全/心跳看门狗应用模块接口
 * @version 1.0
 * @date 2026-08-12
 * @copyright Copyright (c) 2026
 *
 * @note 定义安全监控器配置结构体及管理接口，管理最多16个软件心跳监控器，超时后触发离线回调并刷新硬件看门狗。
 */

#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include "bsp_common.h"
#include "bsp_watchdog.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 最大可注册的心跳监控器数量。 */
#define APP_SAFETY_MAX_MONITOR_COUNT (16U)
/** @brief 推荐的安全处理任务周期 [ms]。 */
#define APP_SAFETY_DEFAULT_TASK_PERIOD_MS (5U)

    /** @brief 监控器在线/离线状态。 */
    typedef enum
    {
        APP_SAFETY_STATE_STARTING = 0, /**< 已注册但尚未收到心跳。 */
        APP_SAFETY_STATE_ONLINE,       /**< 心跳在超时阈值内到达。 */
        APP_SAFETY_STATE_OFFLINE       /**< 超时已过期。 */
    } app_safety_state_t;

    /** @brief 状态转换回调函数签名。 */
    typedef void (*app_safety_callback_t)(void *user_context);

    /** @brief 单个安全监控器的静态配置。 */
    typedef struct
    {
        const char *name;                       /**< 人类可读的名称（用于日志）。 */
        uint32_t timeout_ms;                    /**< 心跳超时阈值 [ms]。 */
        bool required;                          /**< 是否被 app_safety_all_required_online() 统计。 */
        app_safety_callback_t offline_callback; /**< 转入离线状态时调用的回调。 */
        app_safety_callback_t online_callback;  /**< 转入在线状态时调用的回调。 */
        void *user_context;                     /**< 传递给回调的不透明指针。 */
    } app_safety_monitor_config_t;

    /** @brief 单个安全监控器的运行时状态。 */
    typedef struct
    {
        app_safety_monitor_config_t config;     /**< 静态配置的副本。 */
        volatile uint32_t last_online_tick;     /**< 最后一次心跳的 FreeRTOS 滴答值。 */
        volatile bool heartbeat_received;       /**< 至少收到过一次心跳。 */
        uint32_t offline_time_ms;               /**< 距上次在线的经过时间 [ms]。 */
        app_safety_state_t state;               /**< 当前在线/离线状态。 */
        bool is_registered;                     /**< 监控器已注册。 */
    } app_safety_monitor_t;

    /**
     * @brief  初始化安全管理器单例。
     * @param  watchdog  硬件看门狗实例（可为 NULL）。
     * @return 成功返回 BSP_STATUS_OK。
     */
    bsp_status_t app_safety_init(bsp_watchdog_t *watchdog);

    /**
     * @brief  运行时替换硬件看门狗。
     * @param  watchdog  新的硬件看门狗实例（可为 NULL）。
     * @return 成功返回 BSP_STATUS_OK。
     */
    bsp_status_t app_safety_set_watchdog(bsp_watchdog_t *watchdog);

    /**
     * @brief  注册前初始化监控器结构体。
     * @param  me      指向调用方分配的监控器。
     * @param  config  静态配置（内部拷贝）。
     * @return 成功返回 BSP_STATUS_OK。
     */
    bsp_status_t app_safety_monitor_init(app_safety_monitor_t *me,
                                       const app_safety_monitor_config_t *config);

    /**
     * @brief  注册一个已初始化的监控器。
     * @param  monitor  已初始化的监控器。
     * @return 成功返回 BSP_STATUS_OK。
     */
    bsp_status_t app_safety_register(app_safety_monitor_t *monitor);

    /**
     * @brief  为指定监控器记录一次心跳。
     * @param  monitor  已注册的监控器。
     */
    void app_safety_notify_online(app_safety_monitor_t *monitor);

    /**
     * @brief  执行一个安全处理周期（由周期性任务调用）。
     *
     * 评估所有已注册的监控器，更新状态，触发回调，
     * 并在配置了硬件看门狗时刷新它。
     */
    void app_safety_process(void);

    /**
     * @brief  查询监控器的当前状态。
     * @param  monitor  已注册的监控器（NULL 返回 OFFLINE）。
     * @return 当前状态。
     */
    app_safety_state_t app_safety_get_state(const app_safety_monitor_t *monitor);

    /**
     * @brief  查询监控器已离线时长。
     * @param  monitor  已注册的监控器（NULL 返回 UINT32_MAX）。
     * @return 离线时长 [ms]。
     */
    uint32_t app_safety_get_offline_time_ms(const app_safety_monitor_t *monitor);

    /**
     * @brief  检查所有必要监控器是否均在线。
     * @return 全部必要监控器均处于 ONLINE 状态时返回 true。
     */
    bool app_safety_all_required_online(void);

#ifdef __cplusplus
}
#endif

#endif
