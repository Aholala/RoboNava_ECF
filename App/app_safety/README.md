# app_safety

`app_safety_t` 是普通实例，不依赖 RTOS。它管理固定容量的心跳监控器、可选硬件看门狗和整机电机输出门。

```c
app_safety_t safety;
app_safety_monitor_t remote_monitor;

app_safety_init(&safety, &watchdog);
app_safety_monitor_init(&remote_monitor, &(app_safety_monitor_config_t){
    .name = "remote",
    .timeout_ms = 100,
    .required = true,
});
app_safety_register(&safety, &remote_monitor);

app_safety_notify_online(&remote_monitor, now_ms); /* 有效帧到达 */
app_safety_set_output_enabled(&safety, true);      /* 操作者请求 */
app_safety_process(&safety, now_ms);               /* 周期调用 */
```

只有存在 required 监控器、所有 required 监控器在线且操作者已请求输出时，`app_safety_output_allowed(&safety)` 才为 true。required 对象失联会关闭全局电机输出门并撤销操作者请求。
