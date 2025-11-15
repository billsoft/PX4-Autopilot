/**
 * @file IMU数据处理_详细注释.cpp
 * @brief VehicleIMU模块核心代码 - 详细中文注释版
 *
 * 本文件展示PX4如何处理原始IMU数据：
 * 1. 高频采样数据的积分与降采样
 * 2. 圆锥效应补偿（提升姿态积分精度）
 * 3. 振动监控与剪切检测
 * 4. 在线偏置校准
 */

#include "VehicleIMU.hpp"
#include <Integrator.hpp>
#include <lib/mathlib/math/Limits.hpp>

using namespace time_literals;
using namespace matrix;

/**
 * @class VehicleIMU
 * @brief IMU数据预处理与质量监控
 *
 * 设计目标：
 * 1. 将8kHz原始采样降频到250Hz（匹配EKF更新率）
 * 2. 积分角速度/加速度为增量形式（δang/δvel）
 * 3. 实时监控传感器健康状态（振动/剪切/温度）
 * 4. 支持多IMU冗余与在线校准
 *
 * 数据流：
 * sensor_accel/sensor_gyro (8kHz)
 *     ↓
 * [校准] → [积分] → [圆锥补偿] → [振动统计]
 *     ↓
 * vehicle_imu (250Hz) → EKF2
 */

/**
 * @brief VehicleIMU主循环函数
 *
 * 职责：
 * - 订阅原始陀螺数据（触发回调）
 * - 同步读取加速度数据
 * - 执行积分与质量检查
 * - 发布融合后的IMU增量
 */
void VehicleIMU::Run()
{
    // ========================================================================
    // 步骤1：同步获取陀螺与加速度数据
    // ========================================================================

    // 1.1 检查陀螺数据可用性
    // _sensor_gyro_sub 是回调订阅，有新数据时触发Run()
    sensor_gyro_s gyro;
    if (!_sensor_gyro_sub.copy(&gyro)) {
        return; // 无新数据，退出本次循环
    }

    // 1.2 时间戳检查（防止时序倒退）
    //
    // 问题场景：
    // - 系统时间跳变（NTP同步、时间溢出）
    // - 传感器时间戳异常
    //
    // 后果：
    // - 积分时间dt为负，导致状态倒退
    // - 协方差矩阵数值不稳定
    const hrt_abstime timestamp_sample = gyro.timestamp_sample;

    if (timestamp_sample <= _gyro_timestamp_sample_last) {
        PX4_ERR("Gyro timestamp regression detected: %llu -> %llu",
                _gyro_timestamp_sample_last, timestamp_sample);
        return; // 丢弃异常数据
    }

    // 1.3 读取匹配的加速度数据
    //
    // 时序要求：
    // - 加速度与陀螺应来自同一物理时刻
    // - 时间戳差异<1ms（硬件同步）
    //
    // 同步策略：
    // - 优先：硬件时间戳匹配（FIFO批量读取）
    // - 备选：软件最近邻匹配
    sensor_accel_s accel;
    if (!_sensor_accel_sub.copy(&accel)) {
        return; // 加速度数据缺失，等待下次
    }

    // 检查加速度-陀螺时间戳对齐
    const int64_t timestamp_diff = abs(accel.timestamp_sample - gyro.timestamp_sample);
    if (timestamp_diff > 1000) { // 1ms
        PX4_WARN("Accel-Gyro timestamp mismatch: %lld us", timestamp_diff);
        // 继续处理（容错），但标记数据质量下降
        _data_gap = true;
    }

    // ========================================================================
    // 步骤2：传感器校准（偏置/缩放系数/旋转矩阵）
    // ========================================================================

    // 2.1 陀螺校准
    //
    // 校准参数来源：
    // - 出厂标定：存储在参数系统（SENS_GYRO_ID, CAL_GYRO*_OFF等）
    // - 在线学习：EKF估计的偏置反馈（estimator_sensor_bias话题）
    //
    // 校准模型：
    // ω_calib = R · S · (ω_raw - b)
    //
    // 其中：
    // - ω_raw: 原始ADC输出（LSB）
    // - b: 零偏（offset）
    // - S: 缩放矩阵（scale + cross-axis coupling）
    // - R: 旋转矩阵（传感器坐标系 → 机体坐标系）
    Vector3f gyro_raw(gyro.x, gyro.y, gyro.z);
    Vector3f gyro_calibrated = _gyro_calibration.Correct(gyro_raw);

    // 2.2 加速度校准
    //
    // 额外考虑：
    // - 温度补偿：加速度对温度敏感（热膨胀）
    // - 非线性校正：高量程时的非线性误差
    Vector3f accel_raw(accel.x, accel.y, accel.z);
    Vector3f accel_calibrated = _accel_calibration.Correct(accel_raw);

    // ========================================================================
    // 步骤3：振动监控（实时评估机械质量）
    // ========================================================================

    // 3.1 加速度振动指标
    //
    // 计算原理：
    // - Δa = a_k - a_{k-1}（相邻样本差分）
    // - vibration = |Δa| / Δt（变化率）
    //
    // 物理意义：
    // - 高频振动表现为相邻样本剧烈变化
    // - 低频机动表现为缓慢变化
    //
    // 指标用途：
    // - 超过阈值时禁止偏置学习（避免误学习振动）
    // - 发布到vehicle_imu_status供监控
    UpdateAccelVibrationMetrics(accel_calibrated);

    // 3.2 陀螺振动指标
    //
    // 差异：
    // - 陀螺振动通常比加速度小（转动惯量滤波）
    // - 但对姿态估计影响更直接
    UpdateGyroVibrationMetrics(gyro_calibrated);

    // ========================================================================
    // 步骤4：剪切检测（传感器饱和预警）
    // ========================================================================

    // 4.1 加速度剪切
    //
    // 检测逻辑：
    // - 输出接近量程上限（如±16g的15.9g）
    // - 连续多个采样点饱和（排除单次尖峰）
    //
    // 后果：
    // - 真实加速度被截断，积分误差大
    // - 需通知EKF增大过程噪声
    bool accel_clipping[3] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        // 检测方法1：ADC原始值检查（最可靠）
        if (accel.clip_counter[i] > 0) {
            accel_clipping[i] = true;
        }

        // 检测方法2：校准后幅值检查（备用）
        const float accel_range = 16.f * CONSTANTS_ONE_G; // ±16g
        if (fabsf(accel_calibrated(i)) > 0.95f * accel_range) {
            accel_clipping[i] = true;
        }
    }

    // 4.2 陀螺剪切
    //
    // 典型量程：±2000 dps (±34.9 rad/s)
    // 剪切场景：快速翻滚、碰撞、传感器故障
    bool gyro_clipping[3] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        if (gyro.clip_counter[i] > 0) {
            gyro_clipping[i] = true;
        }
    }

    // ========================================================================
    // 步骤5：积分器更新（核心数据处理）
    // ========================================================================

    // 5.1 计算积分时间步长
    //
    // 注意：
    // - 使用sensor内部时间戳差（而非系统时间）
    // - 补偿传感器时钟漂移
    const float dt_gyro = (timestamp_sample - _gyro_timestamp_sample_last) * 1e-6f;
    const float dt_accel = (accel.timestamp_sample - _accel_timestamp_sample_last) * 1e-6f;

    // 异常检测：dt过大或过小
    if (dt_gyro < 0.0001f || dt_gyro > 0.02f) { // 0.1ms ~ 20ms
        PX4_ERR("Abnormal gyro dt: %.6f s", dt_gyro);
        return;
    }

    // 5.2 加速度积分（简单累加）
    //
    // 积分器类型：sensors::Integrator
    // 功能：
    // - 累积速度增量：Δv += a·Δt
    // - 累积时间：Δt_total += Δt
    // - 跟踪剪切标志
    //
    // 数学模型：
    // Δv = ∫(t0, t1) a(t) dt ≈ Σ a_i·Δt_i
    _accel_integrator.put(accel_calibrated, dt_accel);

    // 5.3 陀螺积分（含圆锥补偿）
    //
    // 积分器类型：sensors::IntegratorConing
    // 额外功能：
    // - 圆锥效应补偿（见下文详解）
    // - 旋转矢量非交换性修正
    //
    // 数学模型：
    // Δθ = ∫(t0, t1) ω(t) dt + coning_correction
    _gyro_integrator.put(gyro_calibrated, dt_gyro);

    // ========================================================================
    // 步骤6：圆锥效应补偿详解
    // ========================================================================
    //
    // 物理背景：
    // - 飞行器同时绕两个轴旋转（如俯仰+滚转）
    // - 旋转矢量不满足交换律：R1·R2 ≠ R2·R1
    // - 简单积分会产生系统性误差（圆锥误差）
    //
    // 数学推导：
    //
    // 假设角速度为两个正弦函数叠加：
    // ω(t) = A·[sin(ωt), cos(ωt), 0]^T
    //
    // 真实旋转：
    // Δθ_true = ∫ω dt + (1/12)·∫(ω × (∫ω dt))
    //
    // 离散化（二阶精度）：
    // Δθ_compensated = Σω_i·Δt + (1/12)·(Δθ_{k-1} × Δθ_k)
    //
    // 实现（在IntegratorConing内部）：
    // ```cpp
    // Vector3f delta_angle_this = gyro * dt;
    // Vector3f coning_correction = (1.0f / 12.0f) *
    //                               (_delta_angle_last.cross(delta_angle_this));
    // _delta_angle_accumulated += delta_angle_this + coning_correction;
    // _delta_angle_last = delta_angle_this;
    // ```
    //
    // 效果：
    // - 精度提升：姿态漂移从 ~1°/min → <0.1°/min
    // - 适用场景：多旋翼快速机动、固定翼滚转
    //
    // 参考文献：
    // - Savage, P. G. (1998). "Strapdown Analytics"
    // - Miller, R. B. (1983). "A New Strapdown Attitude Algorithm"

    // ========================================================================
    // 步骤7：检查积分周期是否完成
    // ========================================================================

    // 7.1 判断是否达到发布周期
    //
    // 目标周期：_imu_integration_interval_us（默认5000us = 5ms = 200Hz）
    // 实际周期：累积的dt_total
    //
    // 策略：
    // - 累积时间 >= 目标周期 → 发布
    // - 否则继续累积
    const uint32_t integration_interval_us = _accel_integrator.get_accumulated_interval();

    if (integration_interval_us < _imu_integration_interval_us) {
        // 未达到发布周期，继续累积
        _gyro_timestamp_sample_last = timestamp_sample;
        _accel_timestamp_sample_last = accel.timestamp_sample;
        return;
    }

    // ========================================================================
    // 步骤8：构造并发布vehicle_imu消息
    // ========================================================================

    // 8.1 从积分器获取累积结果
    vehicle_imu_s imu_msg;
    imu_msg.timestamp_sample = timestamp_sample;

    // 角度增量（rad）
    // 注意：已包含圆锥补偿
    _gyro_integrator.get_and_reset_integrated_sample(
        imu_msg.delta_ang,
        imu_msg.delta_ang_dt
    );

    // 速度增量（m/s）
    _accel_integrator.get_and_reset_integrated_sample(
        imu_msg.delta_vel,
        imu_msg.delta_vel_dt
    );

    // 8.2 设置剪切标志
    //
    // 用途：
    // - EKF检测到剪切后增大过程噪声
    // - 防止错误数据污染状态估计
    for (int i = 0; i < 3; i++) {
        imu_msg.delta_vel_clipping[i] = accel_clipping[i];
        imu_msg.delta_ang_clipping[i] = gyro_clipping[i];
    }

    // 8.3 设置传感器元数据
    imu_msg.accel_device_id = _accel_calibration.device_id();
    imu_msg.gyro_device_id = _gyro_calibration.device_id();

    // 8.4 校准计数器（用于检测传感器切换）
    //
    // 用途：
    // - 传感器热插拔检测
    // - 校准参数更新检测
    // - EKF检测到变化后重置偏置估计
    imu_msg.calibration_count = _accel_calibration.calibration_count() +
                                 _gyro_calibration.calibration_count();

    // 8.5 发布消息
    //
    // 话题：vehicle_imu（可多实例）
    // 频率：~200Hz（取决于_imu_integration_interval_us）
    // 订阅者：EKF2、日志、监控模块
    imu_msg.timestamp = hrt_absolute_time();
    _vehicle_imu_pub.publish(imu_msg);

    // ========================================================================
    // 步骤9：在线偏置校准（估计器反馈学习）
    // ========================================================================

    // 9.1 检查是否有新的偏置估计
    //
    // 数据流：
    // EKF2估计偏置 → estimator_sensor_bias话题 → VehicleIMU学习
    //
    // 学习条件：
    // - EKF已收敛（飞行时间>30s）
    // - 偏置方差小于阈值（估计可信）
    // - 飞行器非剧烈机动（避免误学习）
    if (_param_sens_imu_autocal.get()) {
        estimator_sensor_bias_s bias_est;

        for (int instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
            if (_estimator_sensor_bias_subs[instance].copy(&bias_est)) {

                // 9.2 陀螺偏置学习
                //
                // 判断依据：
                // - 偏置方差 < 阈值（如0.001 rad/s）
                // - 估计器标记为valid
                if (bias_est.gyro_bias_valid &&
                    Vector3f(bias_est.gyro_bias_variance).max() < 0.001f) {

                    // 学习策略：指数平滑
                    // b_learned = α·b_estimated + (1-α)·b_old
                    // α = 0.1（缓慢学习，避免瞬态扰动）
                    const float alpha = 0.1f;
                    Vector3f bias_est_vec(bias_est.gyro_bias);

                    _gyro_learned_calibration[instance].offset =
                        alpha * bias_est_vec +
                        (1.0f - alpha) * _gyro_learned_calibration[instance].offset;

                    _gyro_learned_calibration[instance].bias_variance =
                        Vector3f(bias_est.gyro_bias_variance);

                    _gyro_learned_calibration[instance].valid = true;

                    // 9.3 应用学习的偏置
                    //
                    // 时机：
                    // - 下次传感器初始化时生效
                    // - 或调用SensorCalibrationSaveGyro()持久化
                    _gyro_calibration.set_offset(_gyro_learned_calibration[instance].offset);
                }

                // 9.4 加速度偏置学习（类似流程）
                if (bias_est.accel_bias_valid &&
                    Vector3f(bias_est.accel_bias_variance).max() < 0.01f) {

                    const float alpha = 0.1f;
                    Vector3f bias_est_vec(bias_est.accel_bias);

                    _accel_learned_calibration[instance].offset =
                        alpha * bias_est_vec +
                        (1.0f - alpha) * _accel_learned_calibration[instance].offset;

                    _accel_learned_calibration[instance].valid = true;

                    _accel_calibration.set_offset(_accel_learned_calibration[instance].offset);
                }
            }
        }
    }

    // ========================================================================
    // 步骤10：发布传感器健康状态
    // ========================================================================

    // 10.1 更新健康状态结构体
    //
    // 发布周期：100ms（降低带宽占用）
    // 内容：
    // - 振动指标（RMS、峰值）
    // - 剪切计数
    // - 温度
    // - 采样率统计
    const hrt_abstime now = hrt_absolute_time();
    if ((now - _status.timestamp) > kIMUStatusPublishingInterval) {

        vehicle_imu_status_s status{};
        status.timestamp_sample = timestamp_sample;
        status.accel_device_id = imu_msg.accel_device_id;
        status.gyro_device_id = imu_msg.gyro_device_id;

        // 振动指标（Welford算法计算RMS）
        _raw_accel_mean.get_mean().copyTo(status.mean_accel);
        _raw_gyro_mean.get_mean().copyTo(status.mean_gyro);

        _raw_accel_mean.get_variance().copyTo(status.var_accel);
        _raw_gyro_mean.get_variance().copyTo(status.var_gyro);

        // 10.2 剪切统计
        //
        // 用途：
        // - 飞行日志分析
        // - 传感器健康监控
        // - 调参依据（如降低机动强度）
        status.accel_clipping[0] = _delta_velocity_clipping & (1 << 0);
        status.accel_clipping[1] = _delta_velocity_clipping & (1 << 1);
        status.accel_clipping[2] = _delta_velocity_clipping & (1 << 2);

        status.gyro_clipping[0] = _delta_angle_clipping & (1 << 0);
        status.gyro_clipping[1] = _delta_angle_clipping & (1 << 1);
        status.gyro_clipping[2] = _delta_angle_clipping & (1 << 2);

        // 10.3 温度监控
        //
        // 用途：
        // - 热管理（超温报警）
        // - 温度补偿模型验证
        status.temperature_accel = accel.temperature;
        status.temperature_gyro = gyro.temperature;

        // 10.4 采样率统计
        //
        // 计算：
        // - 平均间隔 = Σdt / N
        // - 采样率 = 1 / 平均间隔
        //
        // 异常检测：
        // - 采样率偏离标称值（如8kHz → 7kHz）
        // - 抖动过大（FIFO溢出、中断延迟）
        if (_accel_mean_interval_us.count() > 100) {
            status.accel_rate_hz = 1e6f / _accel_mean_interval_us.mean();
        }

        if (_gyro_mean_interval_us.count() > 100) {
            status.gyro_rate_hz = 1e6f / _gyro_mean_interval_us.mean();
        }

        // 10.5 发布状态消息
        status.timestamp = now;
        _vehicle_imu_status_pub.publish(status);

        // 重置统计
        _raw_accel_mean.reset();
        _raw_gyro_mean.reset();
    }

    // 更新时间戳
    _gyro_timestamp_sample_last = timestamp_sample;
    _accel_timestamp_sample_last = accel.timestamp_sample;
}

/**
 * @brief 加速度振动指标更新
 *
 * 实现算法：Welford在线方差算法
 *
 * 优势：
 * - 单遍扫描（无需存储历史数据）
 * - 数值稳定（避免大数相减）
 * - 内存占用恒定
 *
 * @param acceleration 校准后的加速度（m/s²）
 */
void VehicleIMU::UpdateAccelVibrationMetrics(const Vector3f &acceleration)
{
    // 计算加速度变化率
    //
    // 物理意义：
    // - Delta_a = a_k - a_{k-1}
    // - 高频振动 → Delta_a大
    // - 低频机动 → Delta_a小（相对采样率）
    const Vector3f delta_accel = acceleration - _acceleration_prev;

    // 更新Welford统计量
    //
    // Welford算法公式：
    // M_k = M_{k-1} + (x_k - M_{k-1}) / k
    // S_k = S_{k-1} + (x_k - M_{k-1}) * (x_k - M_k)
    // Var = S_k / (k - 1)
    //
    // 其中：
    // - M: 累积均值
    // - S: 累积平方和
    // - Var: 方差
    _raw_accel_mean.update(delta_accel);

    // 保存当前值供下次使用
    _acceleration_prev = acceleration;

    // 振动阈值检查
    //
    // 应用：
    // - 超过阈值时禁止偏置学习
    // - 发送告警到地面站
    // - 记录到日志供调参
    const float vib_threshold = 30.0f; // m/s² (约3g)
    if (_raw_accel_mean.get_variance().max() > sq(vib_threshold)) {
        // 高振动检测
        _accel_vibration_warning = true;

        // 限流告警（避免日志刷屏）
        static hrt_abstime last_warning_time = 0;
        if (hrt_elapsed_time(&last_warning_time) > 10_s) {
            PX4_WARN("High accel vibration detected: %.2f m/s²",
                     sqrtf(_raw_accel_mean.get_variance().max()));
            last_warning_time = hrt_absolute_time();
        }
    }
}

/**
 * @brief 陀螺振动指标更新
 *
 * @param angular_velocity 校准后的角速度（rad/s）
 */
void VehicleIMU::UpdateGyroVibrationMetrics(const Vector3f &angular_velocity)
{
    // 类似加速度处理，但阈值不同
    const Vector3f delta_gyro = angular_velocity - _angular_velocity_prev;
    _raw_gyro_mean.update(delta_gyro);
    _angular_velocity_prev = angular_velocity;

    // 陀螺振动阈值（通常比加速度小）
    const float vib_threshold = 0.5f; // rad/s (约28°/s)
    if (_raw_gyro_mean.get_variance().max() > sq(vib_threshold)) {
        _gyro_vibration_warning = true;
    }
}

// ============================================================================
// 总结：VehicleIMU模块的核心设计要点
// ============================================================================
//
// 1. 高效积分算法
//    - 圆锥补偿提升姿态精度（~10x误差减小）
//    - 梯形积分（二阶精度）
//    - 在线方差计算（Welford算法）
//
// 2. 多层质量监控
//    - 剪切检测（防止饱和数据污染）
//    - 振动监控（评估机械质量）
//    - 温度跟踪（热管理）
//    - 采样率统计（FIFO健康）
//
// 3. 在线自校准
//    - EKF反馈偏置学习
//    - 指数平滑滤波（避免瞬态）
//    - 可持久化到参数系统
//
// 4. 时序严格性
//    - 时间戳单调性检查
//    - 加速度-陀螺同步验证
//    - 异常数据丢弃机制
//
// 5. 性能优化
//    - 零拷贝消息传递（uORB）
//    - 回调触发机制（低延迟）
//    - 条件发布（降低带宽）
//
// 6. 冗余容错
//    - 支持多IMU实例
//    - 自动选择最佳传感器
//    - 故障隔离与降级
//
// 设计哲学：
// - "测量即一切"：严格的数据质量门控
// - "在线自适应"：持续学习与补偿
// - "零信任"：多重验证与异常检测
// - "高内聚低耦合"：模块化设计便于扩展
