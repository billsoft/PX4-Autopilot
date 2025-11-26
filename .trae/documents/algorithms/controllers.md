---
文档版本: 1.0
适用PX4版本: v1.13.x - v1.15.x
最后更新: 2025-11-26
文档类型: 算法深度教程
难度等级: ⭐⭐⭐⭐ (高级)
前置要求: 控制理论, PID 控制器, 飞行力学基础
预计学习时间: 10-14 小时
代码路径: src/modules/mc_pos_control/, src/modules/mc_att_control/, src/modules/fw_att_control/
---

# PX4 飞行控制器完整教材

## 第一章：控制系统概述

### 1.1 什么是飞行控制器？

飞行控制器（Flight Controller）是飞行器控制律的实现，负责将高层任务（如"飞到某位置"）转化为底层执行器指令（如"电机转速"）。PX4 采用**层级式控制架构**：

```
任务目标（位置/速度） → 位置控制器 → 姿态期望 → 姿态控制器
→ 角速度期望 → 角速度控制器 → 力矩/推力 → 控制分配 → 执行器（电机/舵面）
```

**核心控制器模块**：
- **多旋翼（Multicopter）**：`mc_pos_control`, `mc_att_control`, `mc_rate_control`
- **固定翼（Fixed-wing）**：`fw_att_control`, `fw_rate_control`, TECS
- **垂直起降（VTOL）**：切换多旋翼/固定翼控制器
- **控制分配（Control Allocation）**：`control_allocator`

### 1.2 控制理论基础

#### 1.2.1 PID 控制器

PID（Proportional-Integral-Derivative）是最经典的控制算法，由三个部分组成：

**标准 PID 方程**（并行形式）：
```
u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de/dt
```

其中：
- `e(t) = r(t) - y(t)`：误差（期望值 - 实际值）
- `Kp`：比例增益
- `Ki`：积分增益
- `Kd`：微分增益

**理想 PID 方程**（串联形式）：
```
u(t) = K * [e(t) + (1/Ti) * ∫e(τ)dτ + Td * de/dt]
```

其中：
- `K`：总增益
- `Ti`：积分时间常数
- `Td`：微分时间常数

**转换关系**：
```
Kp = K
Ki = K / Ti
Kd = K * Td
```

PX4 的角速度控制器使用理想 PID 形式，通过参数 `MC_ROLLRATE_K`（K）、`MC_ROLLRATE_P`（1/Ti）、`MC_ROLLRATE_D`（Td）配置。

#### 1.2.2 离散 PID 实现

实际控制器以固定周期 `dt` 采样运行，需要离散化：

**离散 PID**：
```cpp
// 比例项
float P = Kp * error;

// 积分项（梯形积分）
integral += Ki * (error + error_prev) / 2.0f * dt;
integral = constrain(integral, -integral_limit, integral_limit);

// 微分项（差分近似）
float D = Kd * (error - error_prev) / dt;

// 输出
float output = P + integral + D;
```

**改进技术**：
1. **积分饱和抑制（Anti-windup）**：限制积分累积，防止积分饱和
2. **微分滤波**：对微分项低通滤波，减少高频噪声放大
3. **前馈（Feed-forward）**：直接使用期望值的导数，改善响应速度

#### 1.2.3 串级控制（Cascade Control）

PX4 采用串级控制架构，外环输出作为内环期望值：

**优势**：
- 内环快速响应，抑制扰动
- 外环处理低频任务，提高精度
- 层次清晰，易于调试

**多旋翼串级**（3 层）：
```
位置控制器（外环） → 速度期望
速度控制器（中环） → 姿态期望（倾斜角）
姿态控制器（内环） → 角速度期望
角速度控制器（最内环） → 力矩输出
```

### 1.3 坐标系与转换

**常用坐标系**：
- **NED（North-East-Down）**：导航坐标系，EKF 输出位置/速度
- **FRD（Forward-Right-Down）**：机体坐标系，传感器测量
- **FLU（Forward-Left-Up）**：部分外部接口（ROS）使用

**旋转表示**：
- **四元数（Quaternion）**：`q = [w, x, y, z]^T`，无奇异性
- **欧拉角（Euler Angles）**：Roll-Pitch-Yaw，直观但有万向节锁
- **旋转矩阵（Rotation Matrix）**：3×3 矩阵，计算开销大

**坐标转换**（NED → Body）：
```cpp
Vector3f force_body = R_body_to_ned.transpose() * force_ned;
// 或
Vector3f force_body = q_ned_to_body.rotateVectorInverse(force_ned);
```

---

## 第二章：多旋翼位置控制器

### 2.1 位置控制器架构

**位置**：`src/modules/mc_pos_control/`

**核心类**：`MulticopterPositionControl`

**控制流程**：
```
1. 订阅 vehicle_local_position（EKF 估计位置/速度）
2. 订阅 trajectory_setpoint（期望位置/速度/加速度）
3. 计算速度期望（位置 PID）
4. 计算加速度期望（速度 PID）
5. 计算推力向量（加速度 → 推力）
6. 计算姿态期望（推力方向 → Roll/Pitch）
7. 发布 vehicle_attitude_setpoint
```

### 2.2 位置到速度转换

**位置误差 → 速度期望**（P 控制）：

```cpp
// src/modules/mc_pos_control/PositionControl/PositionControl.cpp
Vector3f vel_sp = _pos_p.emult(pos_error);  // 比例控制

// 限制水平速度
float vel_sp_xy_norm = vel_sp.xy().norm();
if (vel_sp_xy_norm > _param_mpc_xy_vel_max) {
    vel_sp.xy() = vel_sp.xy() / vel_sp_xy_norm * _param_mpc_xy_vel_max;
}

// 限制垂向速度
vel_sp(2) = constrain(vel_sp(2), -_param_mpc_z_vel_max_up, _param_mpc_z_vel_max_dn);
```

**关键参数**：
| 参数名称           | 默认值 | 单位   | 说明                  |
|------------------|-------|-------|--------------------|
| MPC_XY_P         | 0.95  | -     | 水平位置比例增益         |
| MPC_Z_P          | 1.0   | -     | 垂向位置比例增益         |
| MPC_XY_VEL_MAX   | 12.0  | m/s   | 最大水平速度            |
| MPC_Z_VEL_MAX_UP | 3.0   | m/s   | 最大上升速度            |
| MPC_Z_VEL_MAX_DN | 1.5   | m/s   | 最大下降速度            |

### 2.3 速度到加速度转换（PID 控制）

**速度误差 → 加速度期望**：

```cpp
// 速度误差
Vector3f vel_error = vel_sp - vel;

// 比例项
Vector3f accel_sp = _vel_p.emult(vel_error);

// 积分项（抗积分饱和）
Vector3f accel_i = _vel_i.emult(_vel_int);
_vel_int += vel_error * dt;
_vel_int = constrain_integral(_vel_int, accel_i_limit);

// 微分项（可选，通常禁用以避免噪声）
Vector3f accel_d = _vel_d.emult((vel_error - _vel_error_prev) / dt);

// 总输出
Vector3f accel_sp_total = accel_sp + accel_i + accel_d + accel_ff;

// 限制加速度
accel_sp_total.xy() = limit_length(accel_sp_total.xy(), _param_mpc_acc_hor_max);
accel_sp_total(2) = constrain(accel_sp_total(2), -_param_mpc_acc_up_max, _param_mpc_acc_down_max);
```

**关键参数**：
| 参数名称         | 默认值 | 单位   | 说明                |
|----------------|-------|-------|-------------------|
| MPC_XY_VEL_P   | 0.09  | -     | 水平速度比例增益       |
| MPC_XY_VEL_I   | 0.02  | -     | 水平速度积分增益       |
| MPC_XY_VEL_D   | 0.01  | -     | 水平速度微分增益       |
| MPC_Z_VEL_P    | 0.2   | -     | 垂向速度比例增益       |
| MPC_Z_VEL_I    | 0.02  | -     | 垂向速度积分增益       |
| MPC_ACC_HOR_MAX| 5.0   | m/s²  | 最大水平加速度        |
| MPC_ACC_UP_MAX | 4.0   | m/s²  | 最大上升加速度        |
| MPC_ACC_DOWN_MAX| 3.0  | m/s²  | 最大下降加速度        |

### 2.4 加速度到推力转换

**加速度期望 → 推力向量**（NED 坐标系）：

```cpp
// 重力补偿
Vector3f thrust_ned = (accel_sp - Vector3f(0.f, 0.f, CONSTANTS_ONE_G)) * mass;

// 转换到机体坐标系
Vector3f thrust_body = R_ned_to_body * thrust_ned;

// 归一化（推力 → 油门）
float thrust_max = mass * CONSTANTS_ONE_G / _hover_thrust;  // 悬停推力归一化
Vector3f thrust_sp = thrust_body / thrust_max;

// 限制推力（0 到 1）
thrust_sp(2) = -constrain(-thrust_sp(2), _param_mpc_thr_min, _param_mpc_thr_max);
```

**说明**：
- 推力在 NED 坐标系计算，包含重力补偿
- 转换到机体坐标系后，Z 轴向下为负（推力向上为正）
- 归一化到 [0, 1] 范围，对应执行器输出

### 2.5 推力到姿态转换

**推力方向 → Roll/Pitch 期望**：

```cpp
// 推力方向（归一化）
Vector3f body_z = thrust_sp.normalized();

// 期望 Yaw（来自用户输入或轨迹）
float yaw_sp = _yaw_setpoint;

// 构造期望姿态矩阵
Vector3f body_x = Vector3f(cos(yaw_sp), sin(yaw_sp), 0.f);
Vector3f body_y = body_z.cross(body_x).normalized();
body_x = body_y.cross(body_z);

// 转换为四元数
Matrix3f R_sp;
R_sp.col(0) = body_x;
R_sp.col(1) = body_y;
R_sp.col(2) = body_z;
Quatf q_sp(R_sp);

// 提取 Roll/Pitch
Eulerf euler_sp(q_sp);
float roll_sp = euler_sp.phi();
float pitch_sp = euler_sp.theta();
```

**关键点**：
- 推力方向决定 Roll/Pitch
- Yaw 由用户输入或轨迹决定（位置控制器不控制 Yaw）
- 倾斜角受限于 `MPC_TILTMAX_AIR`（默认 45°）

---

## 第三章：多旋翼姿态控制器

### 3.1 姿态控制器架构

**位置**：`src/modules/mc_att_control/`

**核心类**：`MulticopterAttitudeControl`

**控制流程**：
```
1. 订阅 vehicle_attitude（EKF 估计姿态）
2. 订阅 vehicle_attitude_setpoint（期望姿态）
3. 计算姿态误差（四元数误差）
4. 计算角速度期望（姿态 P 控制）
5. 发布 vehicle_rates_setpoint
```

### 3.2 四元数姿态误差

**姿态误差计算**（四元数）：

```cpp
// src/lib/AttitudeControl/AttitudeControl.cpp
Quatf q_error = q_sp * q_current.inversed();

// 转换为轴角表示（误差旋转向量）
AxisAnglef aa_error(q_error);
Vector3f e_R = aa_error.axis() * aa_error.angle();

// 小角度近似（当角度很小时）
if (q_error(0) < 0.f) {
    e_R = -e_R;  // 确保旋转方向最短
}
```

**说明**：
- `q_error` 表示从当前姿态到期望姿态的旋转
- 轴角表示直接给出旋转轴和角度
- 小角度时，轴角向量近似为欧拉角误差

### 3.3 姿态到角速度转换（P 控制）

**姿态误差 → 角速度期望**：

```cpp
// 比例控制
Vector3f rate_sp = _gain_p.emult(e_R);

// Yaw 加权（减少 Yaw 响应）
rate_sp(2) *= _yaw_w;

// 限制角速度
rate_sp(0) = constrain(rate_sp(0), -_rate_limit(0), _rate_limit(0));  // Roll
rate_sp(1) = constrain(rate_sp(1), -_rate_limit(1), _rate_limit(1));  // Pitch
rate_sp(2) = constrain(rate_sp(2), -_rate_limit(2), _rate_limit(2));  // Yaw
```

**关键参数**：
| 参数名称           | 默认值 | 单位   | 说明               |
|------------------|-------|-------|--------------------|
| MC_ROLL_P        | 6.5   | -     | Roll 姿态比例增益    |
| MC_PITCH_P       | 6.5   | -     | Pitch 姿态比例增益   |
| MC_YAW_P         | 2.8   | -     | Yaw 姿态比例增益     |
| MC_ROLLRATE_MAX  | 220   | deg/s | 最大 Roll 角速度    |
| MC_PITCHRATE_MAX | 220   | deg/s | 最大 Pitch 角速度   |
| MC_YAWRATE_MAX   | 200   | deg/s | 最大 Yaw 角速度     |
| MC_YAW_WEIGHT    | 0.4   | -     | Yaw 权重（降低响应）|

### 3.4 手动模式的姿态生成

**手动模式**（Stabilized/Altitude/Position）：

```cpp
// 遥控器输入 → 姿态期望
float roll_sp = _manual_control_setpoint.roll * _man_tilt_max;
float pitch_sp = -_manual_control_setpoint.pitch * _man_tilt_max;

// Yaw 生成（速率模式）
float yaw_rate_sp = _manual_control_setpoint.yaw * _param_mpc_man_y_max;
_yaw_sp += yaw_rate_sp * dt;
_yaw_sp = wrap_pi(_yaw_sp);  // 角度环绕

// 构造姿态期望四元数
Quatf q_sp = Quatf(Eulerf(roll_sp, pitch_sp, _yaw_sp));
```

**参数**：
- `MPC_MAN_TILT_MAX`：手动模式最大倾斜角（默认 35°）
- `MPC_MAN_Y_MAX`：手动模式最大 Yaw 速率（默认 150 deg/s）

---

## 第四章：多旋翼角速度控制器

### 4.1 角速度控制器架构

**位置**：`src/modules/mc_rate_control/`

**核心类**：`MulticopterRateControl`

**控制流程**：
```
1. 订阅 vehicle_angular_velocity（陀螺仪角速度）
2. 订阅 vehicle_rates_setpoint（期望角速度）
3. 计算角速度误差
4. PID 控制计算力矩
5. 发布 vehicle_torque_setpoint / vehicle_thrust_setpoint
```

### 4.2 PID 角速度控制

**代码位置**：`src/lib/rate_control/rate_control.cpp`

**PID 实现**（理想形式）：

```cpp
Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp,
                              const Vector3f &angular_accel, float dt)
{
    // 角速度误差
    Vector3f rate_error = rate_sp - rate;

    // 比例项（通过 K 缩放）
    Vector3f torque_p = _gain_p.emult(rate_error);

    // 积分项（梯形积分 + 抗饱和）
    Vector3f rate_i = _rate_int + _gain_i.emult(rate_error) * dt;

    // 积分限幅
    for (int i = 0; i < 3; i++) {
        rate_i(i) = constrain(rate_i(i), -_lim_int(i), _lim_int(i));
    }
    _rate_int = rate_i;
    Vector3f torque_i = _gain.emult(rate_i);

    // 微分项（使用角加速度，避免噪声放大）
    Vector3f torque_d = _gain_d.emult(-angular_accel);  // 负号：误差微分 = -输出微分

    // 前馈项
    Vector3f torque_ff = _gain_ff.emult(rate_sp);

    // 总输出
    Vector3f torque = _gain.emult(torque_p + torque_i + torque_d + torque_ff);

    return torque;
}
```

**关键参数**（以 Roll 为例）：
| 参数名称            | 默认值 | 说明                          |
|-------------------|-------|------------------------------|
| MC_ROLLRATE_K     | 1.0   | 总增益 K                      |
| MC_ROLLRATE_P     | 0.15  | P 增益（并行：K*P）             |
| MC_ROLLRATE_I     | 0.2   | I 增益（并行：K*I）             |
| MC_ROLLRATE_D     | 0.003 | D 增益（并行：K*D）             |
| MC_RR_INT_LIM     | 0.30  | 积分限幅                      |
| MC_ROLLRATE_FF    | 0.0   | 前馈增益                      |

**并行到理想转换**：
```cpp
// 设置增益时的转换
float K = _param_mc_rollrate_k.get();
float Kp = K * _param_mc_rollrate_p.get();
float Ki = K * _param_mc_rollrate_i.get();
float Kd = K * _param_mc_rollrate_d.get();
_rate_control.setPidGains(Vector3f(Kp, ...), Vector3f(Ki, ...), Vector3f(Kd, ...));
```

### 4.3 抗积分饱和（Anti-windup）

**问题**：当执行器饱和时，积分项继续累积会导致超调。

**PX4 的解决方案**：

1. **积分限幅**：`MC_RR_INT_LIM`
2. **条件积分**：着陆时清零积分
3. **反馈饱和**：检测力矩饱和，停止积分累积

```cpp
// 着陆时清零积分
if (_landed || _maybe_landed) {
    _rate_int.zero();
}

// 力矩饱和检测（在控制分配器中实现）
if (torque_saturated) {
    // 停止积分累积或反向积分
}
```

### 4.4 Acro 模式（特技模式）

**Acro 模式**：直接控制角速度，无姿态稳定。

```cpp
// 遥控器输入 → 角速度期望
Vector3f rate_sp;
rate_sp(0) = _manual_control_setpoint.roll * _acro_rate_max(0);   // Roll rate
rate_sp(1) = -_manual_control_setpoint.pitch * _acro_rate_max(1); // Pitch rate
rate_sp(2) = _manual_control_setpoint.yaw * _acro_rate_max(2);    // Yaw rate

// 无姿态反馈，直接进入角速度控制器
```

**参数**：
- `MC_ACRO_R_MAX`：Acro 模式最大 Roll 速率（默认 720 deg/s）
- `MC_ACRO_P_MAX`：Acro 模式最大 Pitch 速率（默认 720 deg/s）
- `MC_ACRO_Y_MAX`：Acro 模式最大 Yaw 速率（默认 540 deg/s）

---

## 第五章：控制分配（Control Allocation）

### 5.1 控制分配概述

**问题**：控制器输出力矩/推力（3 维力矩 + 1 维推力），需要分配到多个电机（如 4 个或更多）。

**数学模型**：
```
[Fx]   [B11 B12 ... B1n]   [u1]
[Fy] = [B21 B22 ... B2n] * [u2]
[Fz]   [B31 B32 ... B3n]   [...]
[Mx]   [B41 B42 ... B4n]   [un]
[My]
[Mz]
```

其中：
- `[Fx, Fy, Fz, Mx, My, Mz]^T`：期望力/力矩
- `B`：混控矩阵（取决于电机布局）
- `[u1, u2, ..., un]^T`：电机推力

### 5.2 混控矩阵

**位置**：`src/modules/control_allocator/`

**四旋翼 X 型混控矩阵**（简化）：

```
        Front
         M1
          |
 M4 ------+------ M2
          |
         M3

混控矩阵：
        M1    M2    M3    M4
Fz   [  1     1     1     1  ]  // 总推力
Mx   [  L    -L    -L     L  ]  // Roll 力矩
My   [  L     L    -L    -L  ]  // Pitch 力矩
Mz   [ -D     D    -D     D  ]  // Yaw 力矩

其中：
L = 电机臂长 * 推力系数
D = 反扭矩系数
```

### 5.3 伪逆分配

**最小二乘解**（Moore-Penrose 伪逆）：

```cpp
// 伪逆矩阵：B_inv = (B^T * B)^{-1} * B^T
Matrix<float, n, 4> B_inv = (B.transpose() * B).inverse() * B.transpose();

// 分配
Vector<float, n> motor_thrust = B_inv * Vector<float, 4>(thrust, torque_x, torque_y, torque_z);

// 限制电机推力 [min, max]
for (int i = 0; i < n; i++) {
    motor_thrust(i) = constrain(motor_thrust(i), motor_min, motor_max);
}
```

### 5.4 饱和处理

**问题**：电机推力有物理限制（0 到最大），分配结果可能超出限制。

**优先级策略**（PX4）：
1. **推力优先**：保证总推力（高度控制）
2. **姿态优先**：牺牲推力保证姿态响应

**代码位置**：`src/modules/control_allocator/ControlAllocation/`

---

## 第六章：固定翼姿态控制器

### 6.1 固定翼控制特点

**与多旋翼的区别**：
- **欠驱动**：无法悬停，需要速度产生升力
- **耦合性强**：Roll-Yaw 耦合（副翼 + 方向舵协调转弯）
- **动态复杂**：空速、迎角、侧滑角影响控制效果

**控制架构**：
```
导航任务 → 期望轨迹 → L1 制导 → 期望 Roll/Pitch
→ 姿态控制器 → 期望 Roll/Pitch/Yaw 速率
→ 角速度控制器 → 副翼/升降舵/方向舵偏转
```

### 6.2 固定翼姿态控制器

**位置**：`src/modules/fw_att_control/`

**核心类**：`FixedwingAttitudeControl`

**控制器类型**：
- **Roll 控制器**：`ECL_RollController`（基于能量控制逻辑）
- **Pitch 控制器**：`ECL_PitchController`
- **Yaw 控制器**：`ECL_YawController`（协调转弯）

### 6.3 Roll 控制器

**代码位置**：`src/modules/fw_att_control/fw_roll_controller.cpp`

**控制律**（简化）：

```cpp
float ECL_RollController::control_attitude(float roll_sp, float roll)
{
    // Roll 角误差
    float roll_error = roll_sp - roll;
    roll_error = wrap_pi(roll_error);

    // Roll 速率前馈（期望 Roll 速率）
    float roll_rate_ff = roll_sp_dot / _tc;  // _tc: 时间常数

    // Roll 速率期望（P 控制 + 前馈）
    float roll_rate_sp = roll_error / _tc + roll_rate_ff;

    // 限制 Roll 速率
    roll_rate_sp = constrain(roll_rate_sp, -_max_rate, _max_rate);

    return roll_rate_sp;
}

float ECL_RollController::control_rate(float roll_rate_sp, float roll_rate, float airspeed)
{
    // Roll 速率误差
    float roll_rate_error = roll_rate_sp - roll_rate;

    // 积分项（带抗饱和）
    float integrator_input = roll_rate_error * _k_i;
    _integrator += integrator_input * dt;
    _integrator = constrain(_integrator, -_integrator_max, _integrator_max);

    // 比例项
    float proportional = roll_rate_error * _k_p;

    // 前馈项（空速相关）
    float feedforward = _k_ff * airspeed;

    // 副翼偏转指令
    float aileron = proportional + _integrator + feedforward;

    return aileron;
}
```

**关键参数**：
| 参数名称       | 默认值 | 说明                  |
|--------------|-------|--------------------|
| FW_R_TC      | 0.4   | Roll 时间常数 [s]     |
| FW_R_RMAX    | 70    | 最大 Roll 速率 [deg/s]|
| FW_RR_P      | 0.05  | Roll 速率 P 增益      |
| FW_RR_I      | 0.1   | Roll 速率 I 增益      |
| FW_RR_FF     | 0.5   | Roll 速率前馈增益     |

### 6.4 Pitch 控制器

**特殊性**：Pitch 控制需要考虑空速和载荷因子（机动时）。

```cpp
float ECL_PitchController::control_attitude(float pitch_sp, float pitch, float airspeed)
{
    // Pitch 角误差
    float pitch_error = pitch_sp - pitch;

    // 空速缩放（低空速时减小响应）
    float airspeed_scaling = constrain(airspeed / airspeed_trim, 0.5f, 2.0f);

    // Pitch 速率期望
    float pitch_rate_sp = pitch_error / (_tc * airspeed_scaling);

    // 限制 Pitch 速率（上下不对称）
    if (pitch_rate_sp > 0) {
        pitch_rate_sp = constrain(pitch_rate_sp, 0.f, _max_rate_pos);
    } else {
        pitch_rate_sp = constrain(pitch_rate_sp, -_max_rate_neg, 0.f);
    }

    return pitch_rate_sp;
}
```

**参数**：
- `FW_P_TC`：Pitch 时间常数（默认 0.4 s）
- `FW_P_RMAX_POS`：最大抬头速率（默认 60 deg/s）
- `FW_P_RMAX_NEG`：最大低头速率（默认 60 deg/s）

### 6.5 协调转弯（Coordinated Turn）

**问题**：固定翼转弯时需要 Roll 和 Yaw 配合，避免侧滑。

**协调条件**：
```
侧滑角 β ≈ 0
Yaw 速率 ψ̇ = (g / V) * tan(φ)
```

**Yaw 控制器**：

```cpp
float ECL_YawController::control_attitude(float roll, float airspeed)
{
    // 协调转弯所需的 Yaw 速率（从 Roll 角推导）
    float yaw_rate_coordinated = CONSTANTS_ONE_G / airspeed * tan(roll);

    // 实际 Yaw 速率（来自陀螺）
    float yaw_rate = _rate_gyro;

    // Yaw 速率误差
    float yaw_rate_error = yaw_rate_coordinated - yaw_rate;

    // 方向舵偏转（PI 控制）
    float rudder = yaw_rate_error * _k_p + _integrator;

    return rudder;
}
```

**参数**：
- `FW_YR_P`：Yaw 速率 P 增益（默认 0.05）
- `FW_YR_I`：Yaw 速率 I 增益（默认 0.1）

---

## 第七章：参数调优方法

### 7.1 调优流程

**通用流程**：
```
1. 默认参数测试 → 记录日志
2. 分析日志 → 识别问题（振荡/超调/响应慢）
3. 调整单个参数 → 再次测试
4. 迭代至满意
```

**调优顺序**（由内向外）：
1. **角速度控制器**（最内环）
2. **姿态控制器**（中环）
3. **位置/速度控制器**（外环）

### 7.2 多旋翼调优

#### 7.2.1 角速度 PID 调优

**步骤**：
1. **禁用 I/D 增益**：设置 `MC_ROLLRATE_I = 0`, `MC_ROLLRATE_D = 0`
2. **增加 P 增益**：从默认值开始，逐步增加 `MC_ROLLRATE_P`
   - 观察 Roll 响应是否快速跟踪期望
   - 增加到出现振荡，然后减少 20%
3. **添加 I 增益**：设置 `MC_ROLLRATE_I`，消除稳态误差
   - 从 0.1 * Kp 开始
   - 观察是否有积分饱和（超调）
4. **添加 D 增益**（可选）：设置 `MC_ROLLRATE_D`，抑制振荡
   - 从 0.01 * Kp 开始
   - 注意噪声放大

**日志分析**：
- **期望与实际角速度**：`vehicle_rates_setpoint` vs `vehicle_angular_velocity`
- **跟踪误差**：应小于 10 deg/s
- **振荡频率**：高频振荡（> 10 Hz）→ 减少 P 或增加滤波

#### 7.2.2 姿态 P 调优

**步骤**：
1. **增加姿态 P 增益**：`MC_ROLL_P`
   - 观察 Roll 角响应是否快速
   - 增加到出现角速度饱和或振荡
2. **检查角速度饱和**：
   - 如果角速度达到 `MC_ROLLRATE_MAX`，增加角速度限制或减少姿态 P

**日志分析**：
- **期望与实际姿态**：`vehicle_attitude_setpoint` vs `vehicle_attitude`
- **响应时间**：从期望变化到 63% 响应，应 < 0.2 s

#### 7.2.3 位置/速度 PID 调优

**步骤**：
1. **位置 P 调优**：`MPC_XY_P`
   - 观察位置响应是否快速
   - 过大会导致速度超调
2. **速度 P 调优**：`MPC_XY_VEL_P`
   - 观察速度响应
   - 过大会导致倾斜角振荡
3. **速度 I 调优**：`MPC_XY_VEL_I`
   - 消除风扰动导致的位置漂移
   - 过大会导致超调

**日志分析**：
- **位置跟踪**：`trajectory_setpoint` vs `vehicle_local_position`
- **速度跟踪**：期望速度 vs 实际速度

### 7.3 固定翼调优

#### 7.3.1 Roll 控制器调优

**步骤**：
1. **调整时间常数**：`FW_R_TC`
   - 时间常数越小，响应越快
   - 过小会导致振荡
2. **调整 Roll 速率 P**：`FW_RR_P`
   - 增加 P 直到出现振荡
   - 减少 20%
3. **调整 Roll 速率 I**：`FW_RR_I`
   - 消除稳态误差
   - 过大会导致超调

#### 7.3.2 Pitch 控制器调优

**类似 Roll，但注意**：
- **空速影响**：低空速时响应变慢
- **上下不对称**：抬头和低头速率限制不同

#### 7.3.3 Yaw/协调转弯调优

**步骤**：
1. **观察侧滑角**：`airspeed_validated.beta`
   - 协调转弯时侧滑角应接近 0
2. **调整 Yaw 速率 P**：`FW_YR_P`
   - 增加 P 直到侧滑角接近 0
3. **调整 Yaw 速率 I**：`FW_YR_I`
   - 消除持续侧滑

### 7.4 自动调参（Auto-tune）

**功能**：自动测试系统响应，计算最优 PID 参数。

**位置**：
- 多旋翼：`src/modules/mc_autotune_attitude_control/`
- 固定翼：`src/modules/fw_autotune_attitude_control/`

**使用方法**：
```bash
# QGroundControl 中启用
设置 → 参数 → 搜索 "autotune"
MC_AT_EN = 1  # 启用多旋翼自动调参
```

**原理**（简化）：
1. **激励输入**：施加方波角速度指令
2. **系统辨识**：测量响应，拟合传递函数
3. **PID 计算**：根据传递函数计算最优 PID 增益
4. **应用参数**：自动更新参数

---

## 第八章：高级控制技术

### 8.1 前馈控制（Feed-forward）

**原理**：利用期望轨迹的导数，提前施加控制量，改善跟踪性能。

**位置控制前馈**：

```cpp
// 期望加速度（来自轨迹规划器）
Vector3f accel_ff = trajectory_setpoint.acceleration;

// 直接加到 PID 输出
Vector3f accel_sp_total = accel_pid + accel_ff;
```

**角速度控制前馈**：

```cpp
// 期望角速度（来自姿态控制器）
Vector3f rate_sp = ...;

// 前馈项（比例于期望角速度）
Vector3f torque_ff = _gain_ff.emult(rate_sp);

// 加到 PID 输出
Vector3f torque_total = torque_pid + torque_ff;
```

**参数**：
- `MC_ROLLRATE_FF`：角速度前馈增益（默认 0.0）
- 通常设置为 0.2 ~ 0.5

### 8.2 抗振陷波滤波器（Notch Filter）

**问题**：结构振动或螺旋桨共振导致特定频率振荡。

**解决方案**：陷波滤波器（Notch Filter）滤除特定频率。

**配置**（多旋翼）：
```cpp
// 速度陷波滤波器
if (MPC_VEL_NF_FRQ > 0 && MPC_VEL_NF_BW > 0) {
    _vel_notch_filter.setParameters(sample_freq, MPC_VEL_NF_FRQ, MPC_VEL_NF_BW);
}
```

**参数**：
- `MPC_VEL_NF_FRQ`：陷波频率 [Hz]（0 表示禁用）
- `MPC_VEL_NF_BW`：陷波带宽 [Hz]

**使用场景**：
- 特定频率振荡（如 15 Hz 螺旋桨振动）
- FFT 分析日志，识别振动频率
- 设置陷波滤波器频率为振动频率

### 8.3 增益调度（Gain Scheduling）

**原理**：根据工作点（如空速）动态调整控制增益。

**固定翼空速补偿**：

```cpp
// 空速归一化
float airspeed_scaling = constrain(airspeed / airspeed_trim, 0.5f, 2.0f);

// 调整控制增益
float K_scaled = K * airspeed_scaling;
```

**多旋翼推力非线性补偿**：

```cpp
// 推力平方律补偿（电机推力 ∝ 转速²）
float thrust_linearized = sqrt(thrust_desired / thrust_max);
```

### 8.4 自适应控制（Adaptive Control）

**PX4 的简单自适应**：悬停推力估计（Hover Thrust Estimation, HTE）。

**原理**：
```
悬停推力 = 重力 / 加速度增益
```

**实现**（位置控制器）：

```cpp
// 估计悬停推力（垂向加速度 = 0 时）
if (fabs(vel_z) < 0.1f && fabs(accel_z) < 0.5f) {
    float hover_thrust_estimate = thrust_z / mass / CONSTANTS_ONE_G;
    _hover_thrust_est = hover_thrust_estimate;
}
```

**用途**：
- 自动补偿载荷变化
- 改善高度控制精度

---

## 第九章：实战案例

### 9.1 案例 1：多旋翼振荡问题

**现象**：
- Roll/Pitch 轴高频振荡（10-20 Hz）
- 电机发热，电池消耗快

**分析**：
1. 查看日志 `vehicle_angular_velocity`
2. FFT 分析，识别振荡频率（假设 15 Hz）
3. 检查 `actuator_controls_0`，确认输出振荡

**解决**：
1. **降低 P 增益**：`MC_ROLLRATE_P` 减少 20%
2. **增加 D 滤波**：降低 `IMU_GYRO_CUTOFF`（陀螺低通截止频率）
3. **检查机械**：螺旋桨是否平衡，电机是否松动

### 9.2 案例 2：位置漂移

**现象**：
- 悬停时缓慢漂移（几米）
- 风扰动后无法回到原位置

**分析**：
1. 查看日志 `vehicle_local_position` vs `trajectory_setpoint`
2. 检查速度积分项是否工作

**解决**：
1. **增加速度 I 增益**：`MPC_XY_VEL_I` 从 0.02 增加到 0.05
2. **检查 EKF**：`estimator_innovations` 是否异常
3. **检查 GPS**：`sensor_gps.eph` 是否 < 3 m

### 9.3 案例 3：固定翼转弯侧滑

**现象**：
- 转弯时飞机内侧倾斜（不协调）
- `airspeed_validated.beta` 持续非零

**分析**：
1. 查看 `vehicle_attitude` 和 `vehicle_angular_velocity`
2. 检查 Roll 和 Yaw 速率是否匹配

**解决**：
1. **增加 Yaw 速率 P**：`FW_YR_P` 从 0.05 增加到 0.08
2. **调整 Yaw 速率 I**：`FW_YR_I` 从 0.1 增加到 0.15
3. **检查方向舵行程**：确保方向舵可以充分偏转

---

## 第十章：源代码导读

### 10.1 多旋翼控制器文件组织

```
src/modules/mc_pos_control/
├── MulticopterPositionControl.hpp/.cpp   # 位置控制器主类
├── PositionControl/
│   └── PositionControl.cpp               # 位置/速度 PID
└── Takeoff/                              # 起飞逻辑

src/modules/mc_att_control/
├── mc_att_control.hpp/.cpp               # 姿态控制器主类
└── AttitudeControl/
    └── AttitudeControl.cpp               # 姿态 P 控制

src/modules/mc_rate_control/
├── MulticopterRateControl.hpp/.cpp       # 角速度控制器主类
└── rate_control/
    └── rate_control.cpp                  # PID 实现

src/lib/control_allocator/
└── ControlAllocation/                    # 控制分配算法
```

### 10.2 固定翼控制器文件组织

```
src/modules/fw_att_control/
├── FixedwingAttitudeControl.hpp/.cpp    # 姿态控制器主类
├── fw_roll_controller.cpp/.h            # Roll 控制器
├── fw_pitch_controller.cpp/.h           # Pitch 控制器
├── fw_yaw_controller.cpp/.h             # Yaw 控制器（协调转弯）
└── fw_wheel_controller.cpp/.h           # 前轮/尾轮控制

src/modules/fw_rate_control/
└── FixedwingRateControl.hpp/.cpp        # 角速度控制器

src/lib/tecs/
└── TECS.cpp/.hpp                        # 总能量控制系统（见第三个文档）
```

### 10.3 关键函数调用流程

**多旋翼控制链路**：
```
1. MulticopterPositionControl::Run()
   ├─> PositionControl::update()           // 位置/速度 PID
   ├─> PositionControl::getThrust()        // 推力向量
   └─> ControlMath::thrustToAttitude()     // 推力 → 姿态

2. MulticopterAttitudeControl::Run()
   ├─> AttitudeControl::update()           // 姿态 P 控制
   └─> 发布 vehicle_rates_setpoint

3. MulticopterRateControl::Run()
   ├─> RateControl::update()               // PID 角速度控制
   └─> 发布 vehicle_torque_setpoint

4. ControlAllocator::Run()
   ├─> ControlAllocation::allocate()       // 混控分配
   └─> 发布 actuator_motors
```

---

## 第十一章：常见问题与解答

### Q1：为什么有多个控制器（位置/姿态/角速度）？

**A**：串级控制架构的优势：
- **分离关注点**：每层处理不同频率的动态
- **抗扰动**：内环快速抑制扰动（如风）
- **易于调试**：由内向外逐层调试
- **复用**：姿态/角速度控制器可用于多种飞行模式

### Q2：PID 参数怎么选择初值？

**A**：
1. **使用默认值**：PX4 默认参数针对常规配置优化
2. **参考相似配置**：查看同型号飞行器的参数
3. **理论估算**（高级）：
   - 角速度 P ≈ 惯性矩 / (推力 * 臂长 * 时间常数)
   - 从保守值（如 Kp = 0.1）开始，逐步增加

### Q3：什么时候需要 D 增益？

**A**：
- **角速度控制**：通常不需要（噪声放大），PX4 使用角加速度代替微分
- **姿态控制**：一般不用 D（角速度已提供阻尼）
- **位置控制**：速度 PID 的 D 项通常禁用

### Q4：如何判断控制器是否调好了？

**A**：检查指标：
1. **跟踪误差**：< 10%（如角速度误差 < 10 deg/s）
2. **响应时间**：< 0.2 s（63% 响应）
3. **超调**：< 5%
4. **振荡**：无持续振荡
5. **鲁棒性**：抗风扰动能力强

### Q5：VTOL 如何切换控制器？

**A**：
- **多旋翼模式**：使用 `mc_*_control` 模块
- **固定翼模式**：使用 `fw_*_control` 模块
- **过渡模式**：`vtol_att_control` 混合控制
- **切换逻辑**：根据 `vehicle_status.is_vtol` 和 `vtol_vehicle_status.vehicle_vtol_state`

---

## 第十二章：总结与展望

### 12.1 核心要点回顾

1. **层级控制**：位置 → 姿态 → 角速度 → 执行器
2. **PID 控制**：比例-积分-微分，需逐层调优
3. **串级优势**：快速内环 + 精确外环
4. **控制分配**：力矩/推力 → 多个执行器
5. **前馈改进**：利用期望轨迹导数，提高跟踪性能

### 12.2 控制器发展方向

1. **模型预测控制（MPC）**：显式处理约束，优化轨迹
2. **自适应控制**：在线估计参数，适应载荷变化
3. **鲁棒控制**：H∞、滑模控制，抗扰动能力强
4. **学习控制**：强化学习、神经网络控制器

---

## 附录：参数速查表

### A.1 多旋翼核心参数

| 参数名称           | 默认值 | 说明                |
|------------------|-------|-------------------|
| MPC_XY_P         | 0.95  | 水平位置 P            |
| MPC_Z_P          | 1.0   | 垂向位置 P            |
| MPC_XY_VEL_P     | 0.09  | 水平速度 P            |
| MPC_XY_VEL_I     | 0.02  | 水平速度 I            |
| MPC_Z_VEL_P      | 0.2   | 垂向速度 P            |
| MC_ROLL_P        | 6.5   | Roll 姿态 P          |
| MC_ROLLRATE_K    | 1.0   | Roll 速率总增益       |
| MC_ROLLRATE_P    | 0.15  | Roll 速率 P          |
| MC_ROLLRATE_I    | 0.2   | Roll 速率 I          |
| MC_ROLLRATE_D    | 0.003 | Roll 速率 D          |

### A.2 固定翼核心参数

| 参数名称       | 默认值 | 说明               |
|--------------|-------|--------------------|
| FW_R_TC      | 0.4   | Roll 时间常数       |
| FW_RR_P      | 0.05  | Roll 速率 P        |
| FW_RR_I      | 0.1   | Roll 速率 I        |
| FW_P_TC      | 0.4   | Pitch 时间常数      |
| FW_PR_P      | 0.08  | Pitch 速率 P       |
| FW_YR_P      | 0.05  | Yaw 速率 P（协调转弯）|

---

## 参考文献

1. **PX4 开发指南**：https://docs.px4.io/main/en/flight_stack/controller_diagrams.html
2. **多旋翼控制**：Beard & McLain《Small Unmanned Aircraft》
3. **固定翼控制**：Stevens & Lewis《Aircraft Control and Simulation》
4. **PID 调优**：Åström & Hägglund《Advanced PID Control》

---

**教材结束**

本教材涵盖了 PX4 飞行控制器的理论基础、实现细节、调优方法和实战案例。通过学习，您应能够：
1. 理解串级控制架构和 PID 原理
2. 阅读和修改控制器源代码
3. 调优控制参数以适配不同飞行器
4. 诊断和解决控制问题
5. 扩展控制器以支持新功能

**下一步学习建议**：
- 实际飞行测试并分析日志
- 阅读控制器源代码
- 尝试自动调参功能
- 研究高级控制技术（MPC、自适应控制）
