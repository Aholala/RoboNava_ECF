# bsp_log — 日志输出

`bsp_log` 是基于 Segger RTT 实现的日志打印模块，通过调试器向上位机发送格式化日志。

## 三级日志宏

推荐使用 `bsp_log.h` 提供的三级日志：

```c
#define LOGINFO(format, ...)      // 信息
#define LOGWARNING(format, ...)   // 警告
#define LOGERROR(format, ...)     // 错误
```

## API 速查

| 函数 | 功能 |
|------|------|
| `printf_log(fmt, ...)` | 向调试器上位机发送格式化信息，格式同 printf |
| `Float2Str(str, va)` | 浮点转字符串（RTT 不支持 `%f`，需先转换再发送） |

## 使用示例

```c
printf_log("Hello World!\n");
printf_log("Motor %d error, code %d!\n", 3, 1);

float current = 114.514f;
char buf[64];
Float2Str(buf, current);
printf_log("current = %s\n", buf);
```

## 注意事项

1. **RTT 不支持 `%f`**：浮点需先用 `Float2Str()` 转成字符串再发送。
2. **必须用 `debug-jlink` 启动调试**才能启用 RTT（无论实际调试器是 cmsis-dap 还是 daplink）。
3. **先启动 J-Link 调试任务再打开 `log` 任务**（见项目 `.vscode/task.json` 注释），否则 RTT viewer 可能无法连接。
4. **Ozone 版本原因**：日志可能不换行或没有颜色。
