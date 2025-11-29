## 目标
- 两个 IMU（SPI1 主、SPI3 辅）各 6 轴简单滤波 → 同步后做减法得到噪声。
- 主 IMU（SPI1）6 轴减噪 → 与磁力计融合输出四元数与欧拉角，发布到 uORB。
- 保持调度 120 Hz（融合），采样目标 1000 Hz（驱动发布由 FIFO/ODR 控制）。

## 设计要点
- 数据订阅：`sensor_accel`、`sensor_gyro`（实例 0、1）、`sensor_mag`。
- 同步策略：按 `timestamp_sample` 对齐，两 IMU 时间差阈值（例如 ≤ 1 ms）；超出丢弃本次或插值（首版先丢弃）。
- 简单滤波：一阶低通 `y += α * (x - y)`，α 依据目标带宽（例如 α≈0.2 @ 120 Hz）。
- 差噪：`noise = imu2_filt - imu1_filt`；主 IMU降噪：`imu1_denoised = imu1_raw - k * noise`（k=1，参数化）。
- 融合：轻量互补/Mahony
  - 积分陀螺得姿态 → 用加速度重力方向校正俯仰/横滚；用磁力计水平投影校正航向。
  - 输出 `vehicle_attitude` 四元数（欧拉角在模块内计算，用于日志/调试，必要时发布辅助话题）。
- 发布速率：ScheduledWorkItem `ScheduleOnInterval(8333)`（≈120 Hz）。

## 代码改动
- `src/modules/dual_imu_fusion/DualIMUFusion.cpp`
  - 添加：
    - 滤波器结构（LPF）与参数（α）、差噪增益（k）。
    - IMU 同步逻辑（按时间戳阈值），实例选择（0、1）。
    - Mahony/互补融合（使用 `matrix` 库进行向量与四元数运算）。
    - 计算欧拉角（roll/pitch/yaw）用于调试（保留在模块内部或可选发布）。
  - 修改：在 `Run()` 内从两路 IMU 读取 → 滤波 → 差噪 → 融合 → 发布 `vehicle_attitude`。
- 参数化（可选）：在 `ModuleParams` 中增加 α、k、同步阈值，以便后续调优。

## 验证步骤
1) 构建：WSL `make st_nucleo-h743zi-fc_default -j4`。
2) 运行级：
   - `icm42688p status`（确认 -6 变体发布速率）。
   - `listener vehicle_attitude`（≈120 Hz）。
   - `listener sensor_accel`/`sensor_gyro`（实例 0、1）观察同步与滤波效果（可临时增加模块内统计日志）。
   - `mavlink status`（ATTITUDE_QUATERNION/HIGHRES_IMU 120 Hz）。

## 注意事项
- 实例映射：以订阅顺序或根据设备 ID/总线号判定主/辅（首版按实例 0→SPI1 主、实例 1→SPI3 辅，若不一致再按设备 ID修正）。
- 时间戳：使用 `timestamp_sample` 与系统 HRT，避免延迟引起的错配。
- 数值稳定性：归一化四元数；避免加速度校正在大机动时引入剧烈抖动（可门限加速度模或权重随加速度偏差调整）。

## 交付
- 更新的融合模块源代码。
- 构建产物与简要验证结果（话题速率、姿态输出）。
