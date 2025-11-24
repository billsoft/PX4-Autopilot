# EKF2 扩展卡尔曼滤波器完整教材

## 第一章：EKF2 概述与理论基础

### 1.1 什么是 EKF2？

EKF2（Extended Kalman Filter 2）是 PX4 的核心状态估计器，负责融合来自 IMU、GPS、磁力计、气压计、光流、测距仪等多种传感器的数据，输出飞行器的姿态、位置、速度、传感器偏置、风速等状态估计。

**位置**：`src/modules/ekf2/`
**核心类**：`Ekf`（继承自 `EstimatorInterface`）
**包装模块**：`EKF2`（继承自 `ModuleParams`, `ScheduledWorkItem`）

### 1.2 卡尔曼滤波基本原理

扩展卡尔曼滤波器（EKF）是卡尔曼滤波器（KF）的非线性扩展，通过泰勒展开对非线性系统进行局部线性化，从而递归估计系统状态。

#### 1.2.1 线性卡尔曼滤波器（KF）回顾

**系统模型**：
```
x_k = F_k * x_{k-1} + B_k * u_k + w_k    // 状态转移方程
z_k = H_k * x_k + v_k                     // 观测方程
```

其中：
- `x_k`：k 时刻的状态向量（n×1）
- `F_k`：状态转移矩阵（n×n）
- `B_k`：控制输入矩阵（n×m）
- `u_k`：控制输入（m×1）
- `w_k`：过程噪声，服从 N(0, Q_k)
- `z_k`：观测向量（m×1）
- `H_k`：观测矩阵（m×n）
- `v_k`：观测噪声，服从 N(0, R_k)

**预测步骤（Predict）**：
```
x̂_k|k-1 = F_k * x̂_{k-1|k-1} + B_k * u_k   // 状态预测
P_k|k-1 = F_k * P_{k-1|k-1} * F_k^T + Q_k  // 协方差预测
```

**更新步骤（Update）**：
```
y_k = z_k - H_k * x̂_k|k-1                  // 创新（Innovation）
S_k = H_k * P_k|k-1 * H_k^T + R_k          // 创新协方差
K_k = P_k|k-1 * H_k^T * S_k^{-1}           // 卡尔曼增益
x̂_k|k = x̂_k|k-1 + K_k * y_k               // 状态更新
P_k|k = (I - K_k * H_k) * P_k|k-1          // 协方差更新
```

#### 1.2.2 扩展卡尔曼滤波器（EKF）

对于非线性系统：
```
x_k = f(x_{k-1}, u_k) + w_k
z_k = h(x_k) + v_k
```

EKF 通过计算雅可比矩阵（Jacobian）对非线性函数进行线性化：
```
F_k = ∂f/∂x |_{x̂_{k-1|k-1}}
H_k = ∂h/∂x |_{x̂_k|k-1}
```

然后按照线性 KF 的流程进行预测和更新。

### 1.3 PX4 EKF2 的特点

1. **基于四元数的姿态表示**：避免万向节锁，数值稳定性好
2. **多源传感器融合**：支持 IMU、GPS、磁力计、气压计、光流、测距仪、外部视觉等
3. **模块化观测源管理**：每个传感器有独立的融合逻辑和健康检查
4. **自动降级和故障恢复**：传感器失效时自动切换到其他观测源
5. **符号数学自动生成**：使用 SymPy 生成预测和融合方程，减少手工推导错误（见 `EKF/python/ekf_derivation/`）
6. **环形缓冲区管理**：传感器数据带时间戳延迟补偿

---

## 第二章：状态向量设计

### 2.1 状态向量组成

EKF2 的状态向量定义在 `ekf_derivation/generated/state.h`，核心状态包括：

```cpp
State {
    quat_nominal,   // 四元数姿态（4维，单位约束）
    vel,            // NED 坐标系速度（3维）[m/s]
    pos,            // NED 坐标系位置（3维）[m]
    gyro_bias,      // 陀螺仪偏置（3维）[rad/s]
    accel_bias,     // 加速度计偏置（3维）[m/s²]
    mag_I,          // 惯性系地磁场（3维）[Gauss]（可选）
    mag_B,          // 机体系磁场偏置（3维）[Gauss]（可选）
    wind_vel,       // 风速（2维，NED 坐标系 XY 分量）[m/s]（可选）
    terrain,        // 地形高度（1维）[m]（可选）
};
```

**说明**：
- **quat_nominal**：采用 Hamilton 约定的单位四元数 `q = [w, x, y, z]^T`，表示从机体坐标系（FRD）到 NED 坐标系的旋转
- **vel/pos**：均为 NED 坐标系（North-East-Down）
- **gyro_bias/accel_bias**：缓慢漂移的传感器偏置，EKF 在线估计并补偿
- **mag_I/mag_B**：可选磁场状态，用于磁力计校准和航向估计
- **wind_vel**：水平风速估计，对固定翼飞行至关重要
- **terrain**：地形相对于 NED 原点的高度，用于地形辅助导航

**代码位置**：
- 状态定义：`src/modules/ekf2/EKF/ekf_derivation/generated/state.h`
- 状态初始化：`src/modules/ekf2/EKF/ekf.cpp:56-135` (`Ekf::reset()`)

### 2.2 状态向量初始化

```cpp
// src/modules/ekf2/EKF/ekf.cpp:56-78
void Ekf::reset()
{
    ECL_INFO("reset");

    _state.quat_nominal.setIdentity();  // 初始姿态为单位四元数（零旋转）
    _state.vel.setZero();               // 速度初始为零
    _state.pos.setZero();               // 位置初始为零（相对 NED 原点）
    _state.gyro_bias.setZero();         // 陀螺偏置初始为零
    _state.accel_bias.setZero();        // 加速度偏置初始为零

#if defined(CONFIG_EKF2_MAGNETOMETER)
    _state.mag_I.setZero();
    _state.mag_B.setZero();
#endif

#if defined(CONFIG_EKF2_WIND)
    _state.wind_vel.setZero();
#endif

#if defined(CONFIG_EKF2_TERRAIN)
    _state.terrain = -_gpos.altitude() + _params.ekf2_min_rng;
#endif
    // ... 更多初始化
}
```

### 2.3 误差状态（Error-State）建模

PX4 EKF2 采用**误差状态卡尔曼滤波器（Error-State Kalman Filter, ESKF）**：
- **标称状态（Nominal State）**：`_state`（即上述 State）
- **误差状态（Error State）**：`δx`，表示真实状态与标称状态的偏差
- **协方差矩阵**：`P`，对应误差状态的不确定度

**优势**：
1. 误差状态通常小量，线性化精度更高
2. 四元数标称状态保持单位约束，误差状态仅需 3 维（旋转误差）
3. 预测步骤在标称状态上进行，更新步骤在误差状态上进行，更新后重置误差状态

**误差状态维度**：
- 姿态误差：3 维（小角度近似）
- 速度误差：3 维
- 位置误差：3 维
- 陀螺偏置误差：3 维
- 加速度偏置误差：3 维
- 磁场误差（可选）：6 维
- 风速误差（可选）：2 维
- 地形误差（可选）：1 维

**总维度**：根据配置不同，约为 24 维（基础）到 30+ 维（全配置）。

---

## 第三章：协方差矩阵管理

### 3.1 协方差矩阵初始化

协方差矩阵 `P` 表示状态估计的不确定度，其对角线元素为各状态分量的方差，非对角线元素为状态间的协方差。

**初始化函数**：`src/modules/ekf2/EKF/covariance.cpp:58-117`

```cpp
// src/modules/ekf2/EKF/covariance.cpp:58-70
void Ekf::initialiseCovariance()
{
    P.zero();

    resetQuatCov(0.f);  // 姿态初始不确定度设为 0（通过零速/零位融合精细调平）

    // 速度初始方差
#if defined(CONFIG_EKF2_GNSS)
    const float vel_var = sq(fmaxf(_params.ekf2_gps_v_noise, 0.01f));
#else
    const float vel_var = sq(0.5f);
#endif
    P.uncorrelateCovarianceSetVariance<State::vel.dof>(State::vel.idx,
                                                       Vector3f(vel_var, vel_var, sq(1.5f) * vel_var));
    // ... 位置、偏置、磁场、风、地形初始化
}
```

**关键参数**：
- `EKF2_GPS_V_NOISE`：GPS 速度噪声，用于速度初始方差
- `EKF2_GPS_P_NOISE`：GPS 位置噪声，用于水平位置初始方差
- `EKF2_BARO_NOISE`：气压计噪声，用于垂向位置初始方差
- `EKF2_GYR_B_NOISE`：陀螺偏置噪声
- `EKF2_ACC_B_NOISE`：加速度偏置噪声

### 3.2 协方差预测

每次 IMU 更新时，需要根据系统动态模型预测协方差矩阵的增长（体现过程噪声累积）。

**函数**：`src/modules/ekf2/EKF/covariance.cpp:119-150`

```cpp
// src/modules/ekf2/EKF/covariance.cpp:119-147
void Ekf::predictCovariance(const imuSample &imu_delayed)
{
    const float dt = 0.5f * (imu_delayed.delta_vel_dt + imu_delayed.delta_ang_dt);

    // 陀螺噪声方差
    float gyro_noise = _params.ekf2_gyr_noise;
    const float gyro_var = sq(gyro_noise);

    // 加速度噪声方差（根据传感器健康状态调整）
    float accel_noise = _params.ekf2_acc_noise;
    Vector3f accel_var;

    for (unsigned i = 0; i < 3; i++) {
        if (_fault_status.flags.bad_acc_vertical || imu_delayed.delta_vel_clipping[i]) {
            // 加速度异常或剪切时，提高过程噪声
            accel_var(i) = sq(BADACC_BIAS_PNOISE);
        } else {
            accel_var(i) = sq(accel_noise);
        }
    }

    // 符号自动生成的协方差预测函数
    P = sym::PredictCovariance(_state.vector(), P,
                               imu_delayed.delta_vel / imu_delayed.delta_vel_dt, accel_var,
                               imu_delayed.delta_ang / imu_delayed.delta_ang_dt, gyro_var,
                               dt);
    // ... 添加偏置/风/磁场/地形的过程噪声
}
```

**说明**：
- `sym::PredictCovariance`：使用 SymPy 自动生成的预测函数（见 `ekf_derivation/generated/predict_covariance.h`），根据 IMU 测量和噪声模型推进协方差矩阵
- **过程噪声**：
  - 陀螺噪声影响姿态不确定度增长
  - 加速度噪声影响速度和位置不确定度增长
  - 偏置噪声影响偏置估计的扩散

**参数**：
- `EKF2_GYR_NOISE`：陀螺角速度白噪声 [rad/s]
- `EKF2_ACC_NOISE`：加速度白噪声 [m/s²]
- `EKF2_GYR_B_NOISE`：陀螺偏置过程噪声 [rad/s²]
- `EKF2_ACC_B_NOISE`：加速度偏置过程噪声 [m/s³]

### 3.3 协方差更新

当观测到来时（如 GPS 位置、磁力计航向），EKF 计算卡尔曼增益并更新协方差矩阵，降低相应状态的不确定度。

**通用融合函数**（简化示意）：
```cpp
// 通用标量融合（如高度、航向）
bool Ekf::fuseScalar(float innovation, float innovation_variance, float obs_variance,
                     const VectorXf &H, VectorXf &K)
{
    // 计算创新协方差：S = H * P * H^T + R
    float S = (H.transpose() * P * H)(0, 0) + obs_variance;

    // 计算卡尔曼增益：K = P * H^T * S^{-1}
    K = P * H / S;

    // 更新协方差：P = (I - K * H) * P
    // Joseph 形式保证数值稳定性：P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
    P = P - K * S * K.transpose();

    return true;
}
```

实际代码中，针对不同传感器有专门的融合函数（如 `fuseVelPosHeight`、`fuseHeading` 等），但核心逻辑一致。

---

## 第四章：IMU 数据处理与预测步骤

### 4.1 IMU 数据接收与缓冲

EKF2 订阅 `vehicle_imu` 主题，接收预处理后的 IMU 数据（已完成校准和传感器选择）。

**订阅代码**：`src/modules/ekf2/EKF2.cpp`（未完全展示，参考模块主循环）

**数据结构**：
```cpp
struct imuSample {
    uint64_t time_us;           // 时间戳 [us]
    Vector3f delta_ang;         // 角增量 [rad]
    Vector3f delta_vel;         // 速度增量 [m/s]
    float delta_ang_dt;         // 角增量时间间隔 [s]
    float delta_vel_dt;         // 速度增量时间间隔 [s]
    bool delta_ang_clipping[3]; // 角速度剪切标志
    bool delta_vel_clipping[3]; // 加速度剪切标志
};
```

**环形缓冲区**：`RingBuffer<imuSample> _imu_buffer`（见 `EKF/RingBuffer.h`）
- 存储最近一段时间的 IMU 数据，支持时间戳延迟补偿
- 其他传感器数据（GPS、Baro 等）也有对应的 RingBuffer

### 4.2 状态预测

状态预测基于 IMU 角增量和速度增量进行**积分更新**。

**函数**：`src/modules/ekf2/EKF/ekf_helper.cpp`（预测函数未完全展示，简化示意）

```cpp
void Ekf::predictState(const imuSample &imu_delayed)
{
    const Vector3f delta_ang_corrected = imu_delayed.delta_ang - _state.gyro_bias * imu_delayed.delta_ang_dt;
    const Vector3f delta_vel_corrected = imu_delayed.delta_vel - _state.accel_bias * imu_delayed.delta_vel_dt;

    // 四元数更新：q_k = q_{k-1} ⊗ exp(0.5 * Δθ)
    const Quatf dq = Quatf(AxisAnglef(delta_ang_corrected));
    _state.quat_nominal = (_state.quat_nominal * dq).normalized();

    // 速度更新：v_k = v_{k-1} + R^T * Δv + g * Δt
    const Vector3f delta_vel_NED = _R_to_earth * delta_vel_corrected;
    _state.vel += delta_vel_NED + Vector3f(0.f, 0.f, CONSTANTS_ONE_G) * imu_delayed.delta_vel_dt;

    // 位置更新：p_k = p_{k-1} + v_{k-1} * Δt + 0.5 * a * Δt^2（简化为 p += v * Δt）
    _state.pos += _state.vel * imu_delayed.delta_vel_dt;

    // 偏置预测：无控制输入，保持不变
    // gyro_bias 和 accel_bias 在融合步骤中根据观测残差更新
}
```

**说明**：
- **陀螺偏置补偿**：从角增量中减去估计的陀螺偏置
- **加速度偏置补偿**：从速度增量中减去估计的加速度偏置
- **四元数积分**：采用一阶近似（小角度假设下的四元数乘法）
- **重力补偿**：速度更新时加上重力加速度（NED 坐标系下为 `[0, 0, +9.81]^T`）
- **姿态矩阵更新**：`_R_to_earth` 从四元数计算得到，用于坐标系转换

### 4.3 IMU 数据质量检查

**剪切检测**：
- IMU 数据带有剪切标志（`delta_ang_clipping`, `delta_vel_clipping`）
- 剪切表示传感器量程饱和，EKF 会提高过程噪声并标记故障

**垂向加速度检查**：
- `src/modules/ekf2/EKF/ekf_helper.cpp` 中检测垂向加速度是否合理
- 异常时设置 `_fault_status.flags.bad_acc_vertical`，提高协方差预测的噪声

---

## 第五章：多源传感器融合

### 5.1 融合架构概览

EKF2 采用**模块化融合架构**，每个观测源有独立的：
1. **数据接收**：订阅对应 uORB 主题并存入 RingBuffer
2. **健康检查**：质量、时效性、一致性检验
3. **融合逻辑**：计算创新、卡尔曼增益、更新状态和协方差
4. **故障检测**：创新检验、门限测试

**支持的观测源**（通过 CONFIG 宏条件编译）：
- GPS (GNSS)
- 气压计（Barometer）
- 磁力计（Magnetometer）
- 测距仪（Range Finder）
- 光流（Optical Flow）
- 外部视觉（External Vision）
- 空速计（Airspeed）
- 辅助速度（Auxiliary Velocity）

### 5.2 GPS 融合

**位置**：`src/modules/ekf2/EKF/aid_sources/gnss/`

#### 5.2.1 GPS 数据接收

**订阅**：`sensor_gps` 主题

**数据结构**：
```cpp
struct gnssSample {
    uint64_t time_us;
    Vector2f pos;       // 纬度/经度 [deg] → 转换为 NED [m]
    float hgt;          // 高度 MSL/WGS84 [m]
    Vector3f vel;       // 速度 NED [m/s]
    float hacc;         // 水平位置精度估计 [m]
    float vacc;         // 垂向位置精度估计 [m]
    float sacc;         // 速度精度估计 [m/s]
    uint8_t fix_type;   // 定位类型（2D/3D/RTK）
    uint8_t nsats;      // 卫星数量
    float pdop;         // 位置精度因子
};
```

#### 5.2.2 GPS 健康检查

**函数**：`src/modules/ekf2/EKF/aid_sources/gnss/gnss_checks.hpp`

```cpp
class GnssChecks {
public:
    bool runChecks(const gnssSample &gps_sample, const parameters &params);

    // 检查项包括：
    // 1. 卫星数量 >= EKF2_REQ_NSATS
    // 2. PDOP <= EKF2_REQ_PDOP
    // 3. 水平精度 hacc <= EKF2_REQ_EPH
    // 4. 垂向精度 vacc <= EKF2_REQ_EPV
    // 5. 速度精度 sacc <= EKF2_REQ_SACC
    // 6. 水平漂移率 < EKF2_REQ_HDRIFT
    // 7. 垂向漂移率 < EKF2_REQ_VDRIFT
};
```

**参数示例**：
- `EKF2_REQ_NSATS = 6`：最少需要 6 颗卫星
- `EKF2_REQ_PDOP = 2.5`：PDOP 阈值
- `EKF2_REQ_EPH = 3.0`：水平精度阈值 [m]
- `EKF2_REQ_SACC = 0.5`：速度精度阈值 [m/s]

#### 5.2.3 GPS 位置/速度融合

**函数**：`src/modules/ekf2/EKF/velocity_fusion.cpp`, `position_fusion.cpp`

**简化流程**：
```cpp
void Ekf::fuseGpsVelocity()
{
    // 计算创新（Innovation）：观测值 - 预测值
    Vector3f vel_innov = _gps_sample_delayed.vel - _state.vel;

    // 计算观测雅可比矩阵 H（速度观测直接对应状态速度）
    // H = [0, 0, 0, I_3, 0, ...] （仅速度项为单位阵，其他为零）

    // 计算创新协方差 S = H * P * H^T + R
    Matrix3f S = P.slice<3,3>(State::vel.idx, State::vel.idx) + R_gps_vel;

    // 计算卡尔曼增益 K = P * H^T * S^{-1}
    // 更新状态：_state += K * vel_innov（误差状态更新后重置）
    // 更新协方差：P = (I - K * H) * P
}
```

**创新门限测试**：
```cpp
// 归一化创新平方（NIS）：y^T * S^{-1} * y
float innovation_test_ratio = vel_innov.transpose() * S.inverse() * vel_innov / gate_sigma;

if (innovation_test_ratio < 1.0f) {
    // 通过门限测试，融合观测
} else {
    // 未通过，拒绝观测或标记故障
}
```

**参数**：
- `EKF2_GPS_V_GATE = 5.0`：速度融合门限（卡方分布）
- `EKF2_GPS_P_GATE = 5.0`：位置融合门限

### 5.3 气压计融合

**位置**：`src/modules/ekf2/EKF/height_control.cpp`

#### 5.3.1 气压计数据接收

**订阅**：`vehicle_air_data` 主题

**数据结构**：
```cpp
struct baroSample {
    uint64_t time_us;
    float hgt;          // 气压高度 [m]（基于标准大气模型）
};
```

#### 5.3.2 气压计融合

气压计提供垂向位置观测（高度），融合到 `_state.pos(2)`（NED 坐标系下 Z 轴向下，高度为负）。

**简化流程**：
```cpp
void Ekf::fuseBaroHeight()
{
    // 创新：观测高度 - 预测高度
    float baro_innov = _baro_sample_delayed.hgt - (-_state.pos(2));

    // 观测雅可比 H：仅位置 Z 分量为 -1（NED 坐标系约定）
    // H = [0, 0, 0, 0, 0, -1, 0, ...]

    // 创新协方差 S = H * P * H^T + R_baro
    float S = P(State::pos.idx + 2, State::pos.idx + 2) + sq(_params.ekf2_baro_noise);

    // 计算增益并更新
    VectorXf K = P.col(State::pos.idx + 2) / S;
    _state.pos(2) -= K(State::pos.idx + 2) * baro_innov;
    // ... 更新协方差
}
```

**参数**：
- `EKF2_BARO_NOISE = 2.0`：气压计观测噪声 [m]
- `EKF2_BARO_GATE = 5.0`：气压计融合门限
- `EKF2_BARO_DELAY = 0`：气压计延迟补偿 [ms]

#### 5.3.3 高度源选择

EKF2 支持多种高度源，通过 `EKF2_HGT_REF` 参数选择：
- `0`：气压计高度
- `1`：GPS 高度
- `2`：测距仪高度
- `3`：外部视觉高度

**动态切换**：在气压计失效时自动切换到 GPS 或测距仪。

### 5.4 磁力计融合

**位置**：`src/modules/ekf2/EKF/yaw_fusion.cpp`, `mag_fusion.cpp`

#### 5.4.1 磁力计数据接收

**订阅**：`vehicle_magnetometer` 主题

**数据结构**：
```cpp
struct magSample {
    uint64_t time_us;
    Vector3f mag;       // 磁场强度 [Gauss]（机体坐标系）
};
```

#### 5.4.2 磁力计融合策略

PX4 支持两种磁力计融合模式（`EKF2_MAG_TYPE`）：
1. **3D 磁场融合**：融合磁场 3 个分量，估计磁场偏置和地磁场（状态 `mag_I`, `mag_B`）
2. **航向融合**：仅融合航向角（Yaw），不估计磁场状态

**航向融合**（常用模式）：
```cpp
void Ekf::fuseHeading()
{
    // 从磁场观测计算航向角
    float mag_declination = _params.ekf2_mag_decl;  // 磁偏角
    float measured_hdg = atan2f(_mag_sample_delayed.mag(1), _mag_sample_delayed.mag(0)) + mag_declination;

    // 从当前状态四元数提取航向角
    float predicted_hdg = Eulerf(_state.quat_nominal).psi();

    // 创新（注意角度环绕处理）
    float hdg_innov = wrap_pi(measured_hdg - predicted_hdg);

    // 观测雅可比 H：∂heading/∂δθ（姿态误差）
    // H 为 [∂ψ/∂δθ_x, ∂ψ/∂δθ_y, ∂ψ/∂δθ_z, 0, ...] （仅姿态误差项非零）

    // 融合更新
    // ...
}
```

**参数**：
- `EKF2_MAG_DELAY = 0`：磁力计延迟 [ms]
- `EKF2_HDG_GATE = 2.6`：航向融合门限
- `EKF2_MAG_DECL = 0.0`：磁偏角 [deg]（从世界磁场模型获取）
- `EKF2_MAG_TYPE = 0`：磁力计融合类型（0=自动, 1=航向, 5=3D）

#### 5.4.3 磁力计校准

EKF 在线估计磁场偏置 `_state.mag_B`，用于软铁/硬铁干扰补偿。离线校准（六面翻滚）提供初始偏置。

### 5.5 测距仪融合

**位置**：`src/modules/ekf2/EKF/aid_sources/range_finder/sensor_range_finder.hpp`

#### 5.5.1 测距仪数据接收

**订阅**：`distance_sensor` 主题

**数据结构**：
```cpp
struct rangeSample {
    uint64_t time_us;
    float rng;          // 距离测量 [m]
    int8_t quality;     // 信号质量 [0-100]
};
```

#### 5.5.2 测距仪融合

测距仪测量地面距离（AGL, Above Ground Level），需要结合姿态计算垂向高度。

**关键公式**：
```
h_agl = rng * cos(pitch) * cos(roll)  // 倾斜补偿
```

**融合代码**（简化）：
```cpp
void Ekf::fuseRangeHeight()
{
    // 计算倾斜补偿后的高度
    float cos_tilt = _R_to_earth(2, 2);  // 旋转矩阵 Z 轴法向分量
    float range_pred = (-_state.pos(2) + _state.terrain) / cos_tilt;

    // 创新
    float rng_innov = _range_sample_delayed.rng - range_pred;

    // 融合更新（类似气压计，但同时更新地形状态）
}
```

**参数**：
- `EKF2_RNG_NOISE = 0.1`：测距仪噪声 [m]
- `EKF2_RNG_GATE = 5.0`：测距仪融合门限
- `EKF2_RNG_PITCH = 0.0`：测距仪俯仰角偏置 [deg]

### 5.6 光流融合

**位置**：`src/modules/ekf2/EKF/aid_sources/optical_flow/`

#### 5.6.1 光流数据接收

**订阅**：`vehicle_optical_flow` 主题

**数据结构**：
```cpp
struct flowSample {
    uint64_t time_us;
    Vector2f flow_rate;     // 光流角速度 [rad/s]（X/Y 方向）
    Vector3f gyro_rate;     // 陀螺角速度（补偿用）
    float quality;          // 质量指标
};
```

#### 5.6.2 光流融合原理

光流测量像素位移速率，结合高度和姿态可反推水平速度：
```
v_x = h * ω_x   （简化，未考虑陀螺补偿）
v_y = h * ω_y
```

其中：
- `h`：距地高度（来自测距仪或气压计）
- `ω_x, ω_y`：光流角速度

**完整模型**（含陀螺补偿）：
```
observed_flow = (v_body_xy / h) + gyro_xy - body_rate_xy
```

**融合代码**（简化）：
```cpp
void Ekf::fuseOpticalFlow()
{
    // 计算预测光流（从状态速度和高度）
    float hagl = -_state.pos(2) + _state.terrain;  // 高度 AGL
    Vector2f flow_pred = (_R_to_earth.transpose() * _state.vel).xy() / hagl;

    // 陀螺补偿
    flow_pred += _flow_sample_delayed.gyro_rate.xy();

    // 创新
    Vector2f flow_innov = _flow_sample_delayed.flow_rate - flow_pred;

    // 融合（更新水平速度状态）
}
```

**参数**：
- `EKF2_OF_DELAY = 20`：光流延迟 [ms]
- `EKF2_OF_GATE = 3.0`：光流融合门限
- `EKF2_OF_QMIN = 10`：最小质量阈值

---

## 第六章：偏置估计与自校准

### 6.1 陀螺偏置估计

陀螺偏置 `_state.gyro_bias` 表示陀螺仪零点漂移，EKF 通过融合外部观测（GPS 速度、磁力计航向）间接估计。

**更新机制**：
- 当融合 GPS 速度时，速度创新会影响姿态和陀螺偏置估计
- 当飞行器静止时（零速更新），陀螺偏置估计更加准确

**约束**：
- `EKF2_GB_NOISE`：陀螺偏置过程噪声，控制偏置估计的响应速度

### 6.2 加速度偏置估计

加速度偏置 `_state.accel_bias` 影响速度和位置积分精度。

**更新机制**：
- 融合 GPS 位置/速度时，位置/速度创新会修正加速度偏置
- 飞行器静止时（零速/零位更新），加速度偏置估计最准确

**约束**：
- `EKF2_ABG_NOISE`：加速度偏置过程噪声

### 6.3 磁场偏置估计

磁场偏置 `_state.mag_B` 表示机体磁场干扰（硬铁/软铁效应）。

**更新机制**：
- 融合 3D 磁场时，EKF 同时估计地磁场 `mag_I` 和机体偏置 `mag_B`
- 通过不同姿态下的观测，分离地磁场和机体干扰

**离线校准**：
- 六面翻滚校准提供初始磁场偏置，EKF 在飞行中继续修正

---

## 第七章：高度控制与融合策略

### 7.1 高度融合模式

EKF2 支持多种高度源，通过 `EKF2_HGT_REF` 参数选择：

| 参数值 | 高度源       | 适用场景                     |
|-------|-------------|----------------------------|
| 0     | Barometer   | 常规飞行（GPS 辅助）          |
| 1     | GPS         | 气压计失效或高海拔             |
| 2     | Range       | 室内/低空飞行（需测距仪）       |
| 3     | Vision      | 视觉定位（VIO/SLAM）          |

### 7.2 高度源切换逻辑

**代码位置**：`src/modules/ekf2/EKF/height_control.cpp`

**切换条件**：
1. 当前高度源失效（如气压计超时）
2. 备用高度源可用且质量更好
3. 切换后进行高度重置，避免突变

**示例切换流程**：
```cpp
// Baro → GPS 切换
if (_control_status.flags.baro_hgt && !baro_data_available) {
    if (gps_data_available && gps_quality_good) {
        // 重置高度状态到 GPS 观测
        resetHeightToGps();
        _control_status.flags.baro_hgt = false;
        _control_status.flags.gps_hgt = true;
    }
}
```

### 7.3 地形估计

**状态**：`_state.terrain`（相对 NED 原点的地形高度）

**用途**：
- 与气压高度结合，提供 AGL 高度
- 辅助测距仪和光流融合

**更新机制**：
- 融合测距仪时，同时更新地形状态
- 融合光流时，利用地形高度计算水平速度

**参数**：
- `EKF2_TERR_NOISE = 5.0`：地形过程噪声 [m/s]
- `EKF2_TERR_GRAD = 0.5`：最大地形梯度 [m/m]

---

## 第八章：风速估计（固定翼重点）

### 8.1 风速状态

**状态**：`_state.wind_vel`（NED 坐标系 XY 分量，2 维）[m/s]

**用途**：
- 固定翼飞行控制（空速 = 地速 + 风速）
- 改善 GPS 速度融合精度
- 侧滑角（Sideslip）估计

### 8.2 风速估计方法

#### 8.2.1 基于空速计和 GPS

**原理**：
```
v_ground = v_airspeed + v_wind
```

融合空速计（IAS）和 GPS 地速，反推风速。

#### 8.2.2 基于侧滑角约束

固定翼在协调转弯时侧滑角接近零，可作为观测约束估计风速。

**代码位置**：`src/modules/ekf2/EKF/wind.cpp`

**参数**：
- `EKF2_WIND_NSD = 1e-2`：风速过程噪声 [m/s²]
- `EKF2_BETA_GATE = 5.0`：侧滑角融合门限

### 8.3 合成空速

当空速计不可用时，EKF 可从 GPS 地速和风速估计合成空速：
```
v_airspeed_synth = ||v_ground - v_wind||
```

**参数**：`EKF2_ARSP_THR`（空速计使能阈值）

---

## 第九章：故障检测与创新监控

### 9.1 创新（Innovation）检验

创新（残差）反映观测与预测的偏差，是健康状态的重要指标。

**归一化创新平方（NIS）**：
```
NIS = y^T * S^{-1} * y
```

其中：
- `y`：创新向量
- `S`：创新协方差矩阵

**门限测试**：
```cpp
if (NIS < gate_threshold) {
    // 通过，融合观测
} else {
    // 拒绝观测，可能的传感器故障
    _innov_check_fail_status.flags.reject_xxx = true;
}
```

**常见门限参数**：
- `EKF2_GPS_V_GATE = 5.0`：对应 95% 置信度（卡方分布）
- `EKF2_BARO_GATE = 5.0`
- `EKF2_MAG_GATE = 3.0`

### 9.2 传感器超时检测

**机制**：
- 每个传感器有最后更新时间戳 `_time_last_xxx`
- 如果当前时间 - 最后更新时间 > 超时阈值，标记传感器失效

**示例**：
```cpp
const bool gps_data_ready = (_time_last_gps > _time_last_imu)
                         && ((_time_last_imu - _time_last_gps) < GPS_MAX_TIMEOUT_US);

if (!gps_data_ready) {
    _control_status.flags.gps = false;  // GPS 失效
}
```

### 9.3 故障状态标志

**结构体**：`_fault_status`

**标志位**：
- `bad_mag_x/y/z`：磁力计某轴异常
- `bad_hdg`：航向估计失效
- `bad_mag_decl`：磁偏角异常
- `bad_airspeed`：空速计异常
- `bad_sideslip`：侧滑角异常
- `bad_acc_vertical`：垂向加速度异常

**用途**：
- 触发传感器切换
- 提高过程噪声
- 发布诊断消息（`estimator_status`）

---

## 第十章：EKF2 初始化与重置

### 10.1 滤波器初始化流程

**函数**：`src/modules/ekf2/EKF/ekf.cpp:198-235`

```cpp
bool Ekf::initialiseFilter()
{
    // 1. 初始化倾斜角（Tilt）
    if (!initialiseTilt()) {
        return false;
    }

    // 2. 初始化航向（Yaw）
    //    优先级：磁力计 > GNSS 航向 > 外部视觉
    if (!initialiseYaw()) {
        return false;
    }

    // 3. 初始化协方差矩阵
    initialiseCovariance();

    // 4. 重置输出预测器
    _output_predictor.reset();

    _filter_initialised = true;
    return true;
}
```

### 10.2 倾斜角初始化

**原理**：
- 利用加速度计观测重力方向，计算 Roll 和 Pitch
- 假设飞行器静止，加速度仅为重力

**代码**（简化）：
```cpp
bool Ekf::initialiseTilt()
{
    // 平均多次 IMU 测量，减少噪声
    Vector3f accel_mean = computeAccelMean();

    // 归一化重力向量
    Vector3f gravity_dir = accel_mean.normalized();

    // 计算 Roll 和 Pitch
    float roll = atan2f(gravity_dir(1), gravity_dir(2));
    float pitch = asinf(-gravity_dir(0));

    // 构造初始四元数（Yaw 暂设为 0）
    _state.quat_nominal = Quatf(Eulerf(roll, pitch, 0.f));

    return true;
}
```

### 10.3 航向初始化

**优先级**：
1. **磁力计航向**（默认）
2. **GNSS 航向**（双天线 GPS 或运动中的 GPS 速度矢量）
3. **外部视觉航向**
4. **无航向信息**（Yaw = 0，后续通过运动估计）

**代码**（简化）：
```cpp
bool Ekf::initialiseYaw()
{
#if defined(CONFIG_EKF2_MAGNETOMETER)
    if (mag_data_available) {
        float mag_yaw = atan2f(_mag_sample_delayed.mag(1), _mag_sample_delayed.mag(0));
        mag_yaw += _params.ekf2_mag_decl;  // 磁偏角修正

        Eulerf euler(_state.quat_nominal);
        euler.psi() = mag_yaw;
        _state.quat_nominal = Quatf(euler);

        return true;
    }
#endif

#if defined(CONFIG_EKF2_GNSS)
    if (gps_yaw_available) {
        // 使用 GPS 双天线航向
        // ...
    }
#endif

    // 无航向信息，使用默认 Yaw = 0
    return true;
}
```

### 10.4 状态重置

在特定情况下（如传感器切换、丢失定位），需要重置部分状态。

**示例：重置位置到 GPS**：
```cpp
void Ekf::resetPositionToGps()
{
    // 将位置状态重置为 GPS 观测
    _state.pos.xy() = _gps_sample_delayed.pos;

    // 重置位置协方差
    float gps_pos_var = sq(_params.ekf2_gps_p_noise);
    P.uncorrelateCovarianceSetVariance<2>(State::pos.idx, Vector2f(gps_pos_var, gps_pos_var));

    // 记录重置事件
    _output_predictor.resetPosition();
}
```

**常见重置场景**：
- GPS 首次可用
- 高度源切换
- 长时间无观测后恢复

---

## 第十一章：输出预测与延迟补偿

### 11.1 输出预测器

EKF 状态估计有延迟（传感器延迟 + 滤波延迟），但控制器需要实时状态。输出预测器（Output Predictor）利用最新 IMU 数据外推状态到当前时刻。

**类**：`OutputPredictor`（见 `output_predictor/output_predictor.h`）

**原理**：
```
x_now = x_delayed + ∫[t_delayed, t_now] f(x, u) dt
```

利用 IMU 角增量和速度增量，从延迟状态积分到当前状态。

### 11.2 延迟补偿机制

**环形缓冲区**：
- 所有传感器数据存入 RingBuffer，带时间戳
- EKF 从缓冲区提取与当前 IMU 时间对齐的观测

**参数**：
- `EKF2_DELAY_MAX = 100`：最大允许延迟 [ms]
- `EKF2_GPS_DELAY = 110`：GPS 延迟补偿 [ms]
- `EKF2_BARO_DELAY = 0`：气压计延迟 [ms]

---

## 第十二章：参数调优指南

### 12.1 核心噪声参数

| 参数名称              | 默认值  | 单位      | 说明                        |
|---------------------|--------|----------|----------------------------|
| EKF2_GYR_NOISE      | 0.015  | rad/s    | 陀螺角速度白噪声              |
| EKF2_ACC_NOISE      | 0.35   | m/s²     | 加速度白噪声                 |
| EKF2_GYR_B_NOISE    | 0.001  | rad/s²   | 陀螺偏置过程噪声              |
| EKF2_ACC_B_NOISE    | 0.003  | m/s³     | 加速度偏置过程噪声            |
| EKF2_GPS_V_NOISE    | 0.3    | m/s      | GPS 速度观测噪声             |
| EKF2_GPS_P_NOISE    | 0.5    | m        | GPS 位置观测噪声             |
| EKF2_BARO_NOISE     | 2.0    | m        | 气压计观测噪声               |
| EKF2_MAG_NOISE      | 0.05   | Gauss    | 磁力计观测噪声               |

**调优原则**：
- **过程噪声（Q）越大**：滤波器响应越快，但抖动增大
- **观测噪声（R）越大**：滤波器越信任预测，响应变慢
- **平衡选择**：根据传感器实际精度和动态特性调整

### 12.2 融合门限参数

| 参数名称          | 默认值 | 说明                       |
|-----------------|-------|---------------------------|
| EKF2_GPS_V_GATE | 5.0   | GPS 速度融合门限（卡方）      |
| EKF2_GPS_P_GATE | 5.0   | GPS 位置融合门限            |
| EKF2_BARO_GATE  | 5.0   | 气压计融合门限               |
| EKF2_HDG_GATE   | 2.6   | 航向融合门限                |
| EKF2_OF_GATE    | 3.0   | 光流融合门限                |

**调优建议**：
- 门限过小：观测易被拒绝，滤波器退化为纯惯导
- 门限过大：异常观测被接受，污染状态估计

### 12.3 GPS 质量要求参数

| 参数名称         | 默认值 | 单位 | 说明                 |
|----------------|-------|-----|---------------------|
| EKF2_REQ_EPH   | 3.0   | m   | 最大水平位置误差       |
| EKF2_REQ_EPV   | 5.0   | m   | 最大垂向位置误差       |
| EKF2_REQ_SACC  | 0.5   | m/s | 最大速度误差          |
| EKF2_REQ_NSATS | 6     | -   | 最小卫星数量          |
| EKF2_REQ_PDOP  | 2.5   | -   | 最大 PDOP           |

**使用场景**：
- 严格要求（城市峡谷、室内边缘）：降低 EPH/EPV 阈值
- 宽松要求（开阔地）：提高阈值，允许更多 GPS 数据

### 12.4 调优实战案例

**案例 1：GPS 位置创新过大**

**现象**：
- `estimator_innovations.gps_hvel[0/1]` 创新大于 1 m
- GPS 频繁被拒绝

**分析**：
- GPS 实际精度差（EPH > 5m），或
- EKF2_GPS_P_NOISE 设置过小，滤波器过度信任 GPS

**解决**：
1. 检查 GPS 质量（`sensor_gps.eph`）
2. 提高 `EKF2_GPS_P_NOISE` 至 1.0 或 2.0
3. 降低 `EKF2_REQ_EPH` 阈值，拒绝低质量 GPS

**案例 2：气压计高度漂移**

**现象**：
- 飞行中高度缓慢漂移（几米到十几米）
- `estimator_innovations.baro_vpos` 创新持续非零

**分析**：
- 气温/气压变化导致气压计漂移
- 或螺旋桨下洗气流影响气压读数

**解决**：
1. 启用气压计地面效应补偿（`EKF2_GND_EFF_DZ`, `EKF2_GND_MAX_HGT`）
2. 提高 `EKF2_BARO_NOISE` 至 3.0 或 5.0，减少对气压计的依赖
3. 考虑切换高度源到 GPS（`EKF2_HGT_REF = 1`）

---

## 第十三章：调试与诊断

### 13.1 EKF2 状态主题

**发布的主题**（用于监控和调试）：

| 主题名称                     | 说明                         |
|-----------------------------|----------------------------|
| `vehicle_attitude`          | 姿态估计（四元数/欧拉角）       |
| `vehicle_local_position`    | 局部位置/速度（NED）           |
| `vehicle_global_position`   | 全局位置（纬度/经度/高度）      |
| `vehicle_odometry`          | 里程计（位置/速度/姿态）        |
| `estimator_status`          | EKF 内部状态汇总              |
| `estimator_innovations`     | 创新向量（所有传感器）         |
| `estimator_innovation_variances` | 创新方差                |
| `estimator_sensor_bias`     | 传感器偏置估计                |
| `wind`                      | 风速估计                     |
| `estimator_gps_status`      | GPS 健康状态                 |

### 13.2 创新监控

**命令行监控**：
```bash
listener estimator_innovations
```

**关键字段**：
```
gps_hvel[2]         // GPS 水平速度创新 [m/s]
gps_vvel            // GPS 垂向速度创新 [m/s]
gps_hpos[2]         // GPS 水平位置创新 [m]
baro_vpos           // 气压计垂向位置创新 [m]
mag_heading         // 磁力计航向创新 [rad]
flow[2]             // 光流创新 [rad/s]
```

**健康判据**：
- 创新应为零均值、小方差的白噪声
- 持续非零创新表明模型不匹配或传感器漂移
- 突发大创新表明传感器故障或环境变化

### 13.3 协方差监控

**命令行查看**：
```bash
listener estimator_status
```

**关键字段**：
```
pos_horiz_accuracy  // 水平位置不确定度 [m]
pos_vert_accuracy   // 垂向位置不确定度 [m]
vel_horiz_accuracy  // 水平速度不确定度 [m/s]
```

**诊断**：
- 不确定度持续增长 → 缺少观测源（如 GPS 丢失）
- 不确定度振荡 → 观测质量不稳定
- 不确定度过小 → 过度自信，可能滤波器发散

### 13.4 故障标志

**命令行查看**：
```bash
listener estimator_status
```

**字段**：
```
control_mode_flags  // 融合模式标志（哪些传感器在用）
innovation_check_flags  // 创新检验失败标志
```

**示例标志位**：
- `cs_tilt_align`：倾斜角对齐完成
- `cs_yaw_align`：航向对齐完成
- `cs_gps`：GPS 融合使能
- `cs_opt_flow`：光流融合使能
- `cs_mag_hdg`：磁力计航向融合使能

### 13.5 日志分析

**Flight Review**（https://logs.px4.io）：

1. **上传日志**（.ulg 格式）
2. **查看关键图表**：
   - Estimator Innovations：创新时间序列
   - Estimator Watchdog：故障标志时间线
   - GPS：GPS 质量指标（EPH/EPV/PDOP）
   - Sensor Biases：偏置估计趋势

**典型问题识别**：
- **创新突变**：传感器故障或环境突变
- **协方差发散**：滤波器失稳
- **偏置漂移**：传感器老化或温漂

---

## 第十四章：高级主题

### 14.1 多 EKF 实例与选择器

**功能**：运行多个 EKF 实例（不同传感器组合），通过选择器选择最优估计。

**位置**：`src/modules/ekf2/EKF2Selector.hpp`

**启用**：`CONFIG_EKF2_MULTI_INSTANCE`

**用途**：
- 提高鲁棒性（某实例失效时切换）
- 传感器故障隔离

### 14.2 外部视觉融合（VIO/SLAM）

**订阅主题**：`vehicle_visual_odometry`

**数据**：
- 位置（局部坐标系）
- 速度
- 姿态

**代码位置**：`src/modules/ekf2/EKF/aid_sources/external_vision/`

**应用场景**：
- 室内定位（无 GPS）
- 视觉惯性里程计（VIO）
- 激光 SLAM

### 14.3 零速更新（Zero Velocity Update, ZUPT）

**原理**：
- 检测飞行器静止时（着陆或手持），强制速度为零
- 提高偏置估计精度

**代码位置**：`src/modules/ekf2/EKF/aid_sources/ZeroVelocityUpdate.hpp`

**触发条件**：
- 垂向加速度接近 1g
- 角速度接近 0
- 高度稳定

### 14.4 GPS-Denied 导航

**策略**：
- 光流 + 测距仪（水平速度 + 高度）
- 外部视觉（VIO/SLAM）
- 纯惯导（短时间）

**注意事项**：
- 纯惯导误差指数增长（数秒内发散）
- 需要高质量 IMU 和良好的偏置估计

---

## 第十五章：常见问题与解答

### Q1：EKF2 与 LPE（本地位置估计器）的区别？

**A**：
- **EKF2**：扩展卡尔曼滤波器，基于协方差预测和更新，支持多源融合，主流选择
- **LPE**：简化的互补滤波器，计算量小但精度较低，已逐步淘汰

**推荐**：新项目使用 EKF2。

### Q2：为什么姿态估计有延迟？

**A**：
- EKF 融合观测数据需要时间对齐（传感器延迟 + 滤波延迟）
- 输出预测器利用最新 IMU 外推状态到当前时刻，减少延迟
- 实际控制使用的姿态是预测值，非延迟估计值

### Q3：如何判断 EKF 是否健康？

**A**：检查以下指标：
1. **创新**：接近零均值，方差稳定
2. **协方差**：水平位置不确定度 < 5m，垂向 < 10m
3. **故障标志**：无持续的 `innovation_check_fail`
4. **传感器状态**：GPS、Baro、Mag 均融合使能

### Q4：GPS 丢失后 EKF 会怎样？

**A**：
1. **短期**（数秒）：纯惯导模式，位置/速度协方差快速增长
2. **中期**（数十秒）：若有光流/视觉，切换到这些观测源
3. **长期**（分钟级）：纯惯导发散，位置估计不可用

**建议**：配置备用观测源（光流、测距仪、外部视觉）。

### Q5：如何提高 EKF 收敛速度？

**A**：
1. **提高过程噪声**（`EKF2_GYR_NOISE`, `EKF2_ACC_NOISE`），加快状态更新
2. **降低观测噪声**（前提是传感器精度确实高），增加观测权重
3. **启用零速更新**（静止时快速收敛偏置）
4. **良好的初始化**（倾斜角和航向对齐精确）

---

## 第十六章：源代码导读

### 16.1 核心文件组织

```
src/modules/ekf2/
├── EKF2.hpp / EKF2.cpp           # 模块包装（uORB 订阅/发布）
├── EKF2Selector.hpp              # 多实例选择器
├── EKF/                          # 核心 EKF 算法
│   ├── ekf.h / ekf.cpp           # EKF 主类
│   ├── estimator_interface.h/.cpp # 传感器接口
│   ├── covariance.cpp            # 协方差初始化/预测/更新
│   ├── control.cpp               # 融合模式控制逻辑
│   ├── ekf_helper.cpp            # 辅助函数（状态预测、重置等）
│   ├── height_control.cpp        # 高度融合
│   ├── position_fusion.cpp       # 位置融合
│   ├── velocity_fusion.cpp       # 速度融合
│   ├── yaw_fusion.cpp            # 航向融合
│   ├── mag_fusion.cpp            # 磁力计融合
│   ├── wind.cpp                  # 风速估计
│   ├── terrain_control.cpp       # 地形估计
│   ├── aid_sources/              # 观测源模块
│   │   ├── gnss/                 # GPS
│   │   ├── range_finder/         # 测距仪
│   │   ├── optical_flow/         # 光流
│   │   ├── external_vision/      # 外部视觉
│   │   └── ...
│   ├── bias_estimator/           # 偏置估计器
│   ├── output_predictor/         # 输出预测器
│   └── ekf_derivation/           # 符号推导（Python SymPy）
│       └── generated/            # 自动生成的预测/融合函数
└── params/                       # 参数定义
```

### 16.2 关键函数调用流程

**主循环**（`EKF2.cpp:Run()`）：
```
1. 订阅 vehicle_imu，接收 IMU 数据
2. 调用 _ekf.update()
   ├─> predictCovariance()        // 协方差预测
   ├─> predictState()             // 状态预测
   └─> controlFusionModes()       // 控制融合模式
       ├─> controlGpsFusion()     // GPS 融合
       ├─> controlBaroFusion()    // 气压计融合
       ├─> controlMagFusion()     // 磁力计融合
       ├─> controlRangeFusion()   // 测距仪融合
       └─> controlOpticalFlowFusion()  // 光流融合
3. 发布估计结果
   ├─> PublishAttitude()
   ├─> PublishLocalPosition()
   ├─> PublishGlobalPosition()
   └─> PublishInnovations()
```

### 16.3 符号推导与代码生成

**位置**：`src/modules/ekf2/EKF/python/ekf_derivation/`

**流程**：
1. 使用 SymPy 定义状态向量、观测模型
2. 符号计算雅可比矩阵、协方差预测方程
3. 生成 C++ 代码（`generated/*.h`）
4. 编译到 EKF2 模块

**优势**：
- 避免手工推导错误
- 自动优化计算效率
- 易于维证和扩展

---

## 第十七章：实战演练

### 17.1 SITL 仿真调试 EKF2

**步骤**：
```bash
# 1. 启动 Gazebo 仿真
make px4_sitl gz_x500

# 2. 监控 EKF 状态
listener estimator_status
listener estimator_innovations

# 3. 注入 GPS 故障（模拟丢失）
param set SIM_GPS_BLOCK 1

# 4. 观察 EKF 响应（切换到纯惯导或其他观测源）
```

### 17.2 真机日志分析

**步骤**：
1. 飞行后下载日志（通过 QGroundControl 或 SD 卡）
2. 上传到 https://logs.px4.io
3. 查看 **Estimator** 部分：
   - 创新时间序列
   - GPS 质量
   - 传感器偏置
4. 识别异常：
   - 创新突变 → 传感器故障
   - 协方差发散 → 观测丢失
   - 偏置漂移 → 温漂或老化

### 17.3 参数调优工作流

**迭代流程**：
```
1. 默认参数飞行 → 记录日志
2. 分析创新和协方差 → 识别问题传感器
3. 调整噪声参数（Q/R）
4. 再次飞行 → 验证改进
5. 重复直到满意
```

**常用调整**：
- GPS 精度差 → 提高 `EKF2_GPS_P_NOISE`
- 气压计漂移 → 提高 `EKF2_BARO_NOISE`
- 响应慢 → 提高过程噪声（`EKF2_GYR_NOISE`, `EKF2_ACC_NOISE`）

---

## 第十八章：总结与展望

### 18.1 EKF2 核心要点回顾

1. **误差状态卡尔曼滤波器（ESKF）**：标称状态 + 误差状态建模
2. **四元数姿态表示**：避免奇异性，数值稳定
3. **多源传感器融合**：GPS、气压计、磁力计、光流、测距仪等
4. **模块化观测源管理**：独立健康检查、融合逻辑、故障检测
5. **符号自动生成**：减少人工推导错误，优化计算
6. **输出预测器**：补偿延迟，提供实时状态

### 18.2 EKF2 的局限性

1. **线性化误差**：非线性系统的一阶线性化近似，大误差时精度下降
2. **高斯噪声假设**：实际噪声可能非高斯（如磁场干扰）
3. **计算复杂度**：协方差矩阵 O(n²)，状态维度增加时计算量增大
4. **调参依赖**：需要根据传感器特性调整参数

### 18.3 未来发展方向

1. **无迹卡尔曼滤波器（UKF）**：更高阶的非线性近似
2. **粒子滤波器**：处理非高斯噪声和多模态分布
3. **机器学习辅助**：利用 NN 学习传感器噪声模型
4. **多传感器时间同步**：硬件时间戳对齐，减少延迟补偿误差

---

## 附录：参数速查表

### A.1 核心参数

| 参数               | 默认值 | 单位   | 说明                     |
|-------------------|-------|-------|-------------------------|
| EKF2_GYR_NOISE    | 0.015 | rad/s | 陀螺白噪声                |
| EKF2_ACC_NOISE    | 0.35  | m/s²  | 加速度白噪声              |
| EKF2_GYR_B_NOISE  | 0.001 | rad/s²| 陀螺偏置噪声              |
| EKF2_ACC_B_NOISE  | 0.003 | m/s³  | 加速度偏置噪声            |
| EKF2_GPS_V_NOISE  | 0.3   | m/s   | GPS 速度噪声             |
| EKF2_GPS_P_NOISE  | 0.5   | m     | GPS 位置噪声             |
| EKF2_BARO_NOISE   | 2.0   | m     | 气压计噪声               |
| EKF2_MAG_NOISE    | 0.05  | Gauss | 磁力计噪声               |
| EKF2_GPS_V_GATE   | 5.0   | -     | GPS 速度融合门限          |
| EKF2_BARO_GATE    | 5.0   | -     | 气压计融合门限            |
| EKF2_HGT_REF      | 0     | -     | 高度源（0=Baro,1=GPS等）  |

### A.2 完整参数列表

参考 PX4 参数文档：https://docs.px4.io/main/en/advanced_config/parameter_reference.html#ekf2

---

## 参考文献

1. **PX4 官方文档**：https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html
2. **ECL (Estimation & Control Library)**：https://github.com/PX4/PX4-ECL
3. **SymPy EKF Derivation**：`src/modules/ekf2/EKF/python/ekf_derivation/`
4. **Quaternion Kinematics for the Error-State Kalman Filter**：Joan Solà 论文
5. **Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems**：Paul D. Groves 教材

---

**教材结束**

本教材涵盖了 PX4 EKF2 的理论基础、实现细节、调试方法和实战案例。通过学习本教材，您应能够：
1. 理解 EKF2 的数学原理和算法流程
2. 阅读和修改 EKF2 源代码
3. 调优 EKF2 参数以适配不同传感器配置
4. 诊断和解决 EKF2 相关问题
5. 扩展 EKF2 以支持新的观测源

**下一步学习建议**：
- 实际飞行测试并分析日志
- 阅读 `src/modules/ekf2/EKF/` 源代码
- 研究 SymPy 推导脚本，理解符号计算
- 参与 PX4 社区讨论（https://discuss.px4.io）
