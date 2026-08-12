# app_imu — IMU 姿态应用

包装 BMI088 + EKF。提供姿态快照和坐标系变换。

## 用法

```c
app_imu_config_t cfg = {
    .sensor = &bmi088,
    .ekf_config = NULL, // 使用 alg_imu_ekf 默认参数
};
app_imu_t imu;
app_imu_init(&imu, &cfg);

// 周期更新（ISR 触发读取 → 任务中 EKF 更新 → 发布快照）
app_imu_update(&imu, dt);

// 读快照
const app_imu_snapshot_t *imu = app_imu_get_snapshot();
float yaw = imu->yaw_rad;
float pitch = imu->pitch_rad;
bool valid = imu->valid;
```

首次有效采样会用加速度方向初始化 Roll/Pitch，之后执行四元数 EKF。六轴 IMU 的
Yaw 仍会随 Z 轴陀螺零偏缓慢漂移，需要磁力计、视觉或机构约束提供长期航向修正。
