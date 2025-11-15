/**
 * @file 状态预测_详细注释.cpp
 * @brief EKF2 状态预测核心代码 - 详细中文注释版
 *
 * 本文件展示PX4 EKF2如何基于IMU增量数据预测飞行器状态
 * 涵盖：四元数姿态更新、速度积分、位置积分、偏置建模
 */

#include "ekf.h"
#include <matrix/math.hpp>

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector3f;
using matrix::AxisAnglef;

/**
 * @brief EKF状态预测主函数
 *
 * 职责：
 * 1. 基于IMU角度增量(delta_ang)更新四元数姿态
 * 2. 基于IMU速度增量(delta_vel)更新速度与位置
 * 3. 考虑科里奥利力、重力补偿、地球自转等高阶项
 * 4. 保持四元数归一化约束
 *
 * @param imu_delayed 经过时序对齐的IMU样本（已匹配观测延时）
 *
 * 数学模型：
 * - 四元数微分方程：q̇ = 0.5·q ⊗ ω
 * - 速度微分方程：v̇ = R(q)·(a_body - b_accel) + g - 2Ω×v
 * - 位置微分方程：ṗ = v
 *
 * 其中：
 * - q: 四元数（机体→NED旋转）
 * - ω: 角速度（rad/s，机体坐标系）
 * - R(q): 旋转矩阵（由四元数转换）
 * - a_body: 加速度（m/s²，机体坐标系）
 * - b_accel: 加速度计偏置（估计状态）
 * - g: 重力加速度（NED坐标系：[0, 0, 9.81]）
 * - Ω: 地球自转角速度（rad/s，NED坐标系）
 * - v: 速度（m/s，NED坐标系）
 */
void Ekf::predictState(const imuSample &imu_delayed)
{
    // ========================================================================
    // 步骤1：四元数姿态更新（非线性旋转传播）
    // ========================================================================

    // 1.1 陀螺仪偏置补偿
    // 原理：陀螺仪存在温漂、安装误差等系统偏差，需从测量值中减去估计偏置
    // delta_ang_corrected = Δθ_measured - b_gyro·Δt
    const Vector3f delta_ang_corrected = imu_delayed.delta_ang - _state.gyro_bias * imu_delayed.delta_ang_dt;

    // 1.2 角度增量转换为增量四元数
    // 使用指数映射（Exponential Map）将旋转向量转四元数
    // 对于小角度：q ≈ [cos(|Δθ|/2), sin(|Δθ|/2)·Δθ/|Δθ|]
    //
    // AxisAnglef: 轴角表示（旋转轴方向 + 旋转角度模）
    // Quatf: 将轴角转换为单位四元数
    const Quatf dq(AxisAnglef(delta_ang_corrected));

    // 1.3 四元数乘法更新姿态
    // q_{k+1} = q_k ⊗ Δq
    //
    // 物理意义：
    // - q_k: 上一时刻的姿态（世界→机体旋转）
    // - Δq: 本次积分周期内的旋转增量（机体坐标系相对旋转）
    // - ⊗: 四元数乘法（非交换，顺序重要！）
    //
    // 注意：这是右乘（机体坐标系旋转），不是左乘（世界坐标系旋转）
    _state.quat_nominal = (_state.quat_nominal * dq).normalized();

    // 1.4 更新旋转矩阵（Direction Cosine Matrix）
    // DCM = R(q)，用于加速度从机体系转到NED系
    //
    // 优化说明：虽然可从DCM直接推导姿态，但四元数积分更稳定（无奇异点）
    // DCM主要用于向量转换，而非姿态存储
    _R_to_earth = Dcmf(_state.quat_nominal);

    // ========================================================================
    // 步骤2：速度状态更新（考虑重力、科里奥利力、离心力）
    // ========================================================================

    // 2.1 保存上一时刻速度（用于梯形积分）
    const Vector3f vel_last = _state.vel;

    // 2.2 加速度计偏置补偿
    // 原理：加速度计受温度、振动、安装误差影响，存在系统偏置
    const Vector3f delta_vel_corrected = imu_delayed.delta_vel - _state.accel_bias * imu_delayed.delta_vel_dt;

    // 2.3 加速度从机体系转到NED系
    // a_NED = R(q) · a_body
    //
    // 坐标系定义：
    // - 机体系（Body Frame）：X前Y右Z下，固连于飞行器
    // - NED系（North-East-Down）：X北Y东Z下，局部水平坐标系
    //
    // 为什么需要坐标转换？
    // - 加速度计测量的是机体系加速度
    // - 速度/位置状态定义在NED系（导航坐标系）
    _state.vel += _R_to_earth * delta_vel_corrected;

    // 2.4 重力补偿
    // NED坐标系下，重力沿Z轴向下（正方向）
    // 加速度计测量的是"比力"（specific force），不包含重力
    // 因此需要手动补偿重力项：v̇ = a_measured + g
    //
    // _gravity_mss: 重力加速度模值（通常为9.81 m/s²）
    // 注意：NED坐标系Z轴向下为正，所以是 +gravity
    _state.vel(2) += _gravity_mss * imu_delayed.delta_vel_dt;

    // 2.5 科里奥利力与离心力修正（高精度导航必需）
    //
    // 物理背景：
    // - 地球自转角速度 Ω_earth ≈ 7.292e-5 rad/s
    // - 当飞行器速度较高（>50m/s）或纬度较高时，科里奥利加速度不可忽略
    //
    // 科里奥利加速度：a_cor = -2Ω × v
    //
    // 数学推导：
    // - 地球自转角速度在NED系投影：Ω_NED = [Ω·cos(lat), 0, -Ω·sin(lat)]
    // - 速度变化率贡献：Δv_cor = -2Ω × v_avg · Δt
    // - v_avg = (v_k + v_{k+1}) / 2  （梯形积分中点）
    if ((_params.ekf2_imu_ctrl & static_cast<int32_t>(ImuCtrl::GyroBias))) {
        // 计算平均速度（梯形规则中点）
        const Vector3f vel_avg = 0.5f * (vel_last + _state.vel);

        // 科里奥利修正
        // _earth_rate_NED: 地球自转在NED系的投影向量
        const Vector3f coriolis_correction = 2.f * _earth_rate_NED.cross(vel_avg) * imu_delayed.delta_vel_dt;

        _state.vel -= coriolis_correction;
    }

    // ========================================================================
    // 步骤3：位置状态更新（梯形积分法）
    // ========================================================================

    // 3.1 梯形积分公式
    // p_{k+1} = p_k + (v_k + v_{k+1})/2 · Δt
    //
    // 为什么用梯形积分而非欧拉法？
    // - 欧拉法：p_{k+1} = p_k + v_k·Δt  （一阶精度）
    // - 梯形法：p_{k+1} = p_k + (v_k + v_{k+1})/2·Δt  （二阶精度）
    // - 梯形法对加速度为常数的运动精确无误差
    //
    // 数值稳定性：
    // - 梯形法是隐式方法的一种，具有更好的稳定性
    // - 对于高频振动，梯形法起到自然滤波作用
    _state.pos += (_state.vel + vel_last) * (0.5f * imu_delayed.delta_vel_dt);

    // ========================================================================
    // 步骤4：偏置状态（建模为随机游走，本步预测不变）
    // ========================================================================
    //
    // 偏置状态模型：
    // - 陀螺偏置：b_gyro_{k+1} = b_gyro_k + w_gyro
    // - 加计偏置：b_accel_{k+1} = b_accel_k + w_accel
    //
    // 其中 w_gyro, w_accel 是零均值高斯白噪声
    //
    // 在预测步，偏置状态保持不变（期望值不变）
    // 其不确定度在协方差预测中增长（通过过程噪声Q建模）
    //
    // 偏置更新仅在测量更新（融合步）中进行
    //
    // _state.gyro_bias 保持不变
    // _state.accel_bias 保持不变

    // ========================================================================
    // 步骤5：其他增广状态预测（磁场、风速、地形）
    // ========================================================================
    //
    // 地磁场状态（mag_I, mag_B）：
    // - 地球磁场（mag_I）：建模为缓慢变化的常量
    // - 机体磁偏置（mag_B）：机体硬铁/软铁干扰，缓慢漂移
    // - 预测步保持不变，协方差通过过程噪声增长
    //
    // 风速状态（wind_vel）：
    // - 建模为二维水平风（北向、东向分量）
    // - 预测步保持不变（假设短期风速稳定）
    // - 不确定度随高度率动态增长（爬升穿越风层）
    //
    // 地形高度（terrain）：
    // - 相对于NED原点的地形高度
    // - 预测步保持不变，不确定度随水平速度增长
    // - 物理意义：飞行器水平移动，地形高度相对于飞行器变化

    // 注：这些状态的更新主要在协方差预测（predictCovariance）和
    // 观测融合（fuseMag, fuseAirspeed, etc.）中完成

    // ========================================================================
    // 步骤6：约束与数值稳定性保障
    // ========================================================================

    // 6.1 四元数归一化（已在上面执行）
    // 原因：数值累积误差会导致四元数模长偏离1
    // 后果：如不归一化，旋转矩阵会包含缩放，导致计算错误

    // 6.2 速度限幅（防止状态发散）
    for (int i = 0; i < 3; i++) {
        // 参数EKF2_VEL_LIM通常设为100 m/s
        // 如果速度超限，说明状态已发散，需要重置
        _state.vel(i) = math::constrain(_state.vel(i), -_params.ekf2_vel_lim, _params.ekf2_vel_lim);
    }

    // 6.3 位置有效性检查
    // 如果位置积分到无穷大（NaN或Inf），需要触发重置
    for (int i = 0; i < 3; i++) {
        if (!PX4_ISFINITE(_state.pos(i))) {
            ECL_ERR("Position state %d is NaN/Inf, resetting EKF", i);
            // 触发故障保护逻辑
            _fault_status.flags.bad_acc_vertical = true;
        }
    }

    // ========================================================================
    // 关键设计思想总结
    // ========================================================================
    //
    // 1. 非线性姿态表示：
    //    - 使用四元数避免欧拉角万向锁问题
    //    - 四元数乘法精确表达旋转叠加（无小角度近似）
    //
    // 2. 高阶动力学建模：
    //    - 科里奥利力修正（高速/高纬度场景）
    //    - 重力精确补偿（NED坐标系特性）
    //
    // 3. 偏置在线估计：
    //    - 陀螺/加计偏置作为状态变量，持续跟踪
    //    - 避免传感器重新标定（适应温漂/长期漂移）
    //
    // 4. 数值稳定性：
    //    - 梯形积分（二阶精度）
    //    - 四元数归一化（避免累积误差）
    //    - 状态限幅（防止发散）
    //
    // 5. 模块化设计：
    //    - 状态预测与协方差预测分离
    //    - 观测融合独立于预测步
    //    - 便于扩展新传感器
}

/**
 * @brief 协方差预测函数
 *
 * 职责：
 * 1. 传播状态不确定度（通过线性化状态转移）
 * 2. 注入过程噪声（建模未建模动力学与传感器噪声）
 * 3. 自适应调整噪声水平（故障检测触发）
 * 4. 保持协方差矩阵对称正定性
 *
 * @param imu_delayed 经过时序对齐的IMU样本
 *
 * 数学模型：
 * P_{k+1|k} = F·P_{k|k}·F^T + Q
 *
 * 其中：
 * - P: 状态协方差矩阵（n×n，n为状态维度）
 * - F: 状态转移雅可比矩阵（∂f/∂x）
 * - Q: 过程噪声协方差矩阵
 *
 * PX4实现特点：
 * - 使用SymPy符号推导自动生成F矩阵（避免手工错误）
 * - 稀疏结构优化（许多状态间不耦合）
 * - 自适应Q矩阵（传感器故障时增大噪声）
 */
void Ekf::predictCovariance(const imuSample &imu_delayed)
{
    // 计算积分时间步长（取角度和速度积分周期的平均值）
    const float dt = 0.5f * (imu_delayed.delta_vel_dt + imu_delayed.delta_ang_dt);

    // ========================================================================
    // 步骤1：配置过程噪声水平（自适应调整）
    // ========================================================================

    // 1.1 陀螺噪声
    // 物理意义：陀螺测量的随机波动（角度随机游走）
    // 单位：rad/s
    // 影响：姿态估计不确定度随时间扩散
    float gyro_noise = _params.ekf2_gyr_noise;
    const float gyro_var = sq(gyro_noise);

    // 1.2 加速度噪声（自适应调整）
    // 正常情况：使用标称噪声水平
    // 故障情况：大幅增加噪声（降低对加速度的信任）
    float accel_noise = _params.ekf2_acc_noise;
    Vector3f accel_var;

    for (unsigned i = 0; i < 3; i++) {
        // 检测条件1：垂向加速度异常
        // - 创新检验失败（测量与预测不一致）
        // - 可能原因：传感器故障、气动扰动、碰撞
        //
        // 检测条件2：加速度削波
        // - imu_delayed.delta_vel_clipping[i] == true
        // - 原因：加速度超量程（±16g），输出饱和
        // - 后果：积分误差大，速度/位置估计不可靠
        if (_fault_status.flags.bad_acc_vertical || imu_delayed.delta_vel_clipping[i]) {
            // 故障缓解策略：
            // - 增大过程噪声100倍（BADACC_BIAS_PNOISE = 4.9 m/s²）
            // - 效果：卡尔曼增益K↓，减少对加速度的依赖
            // - 副作用：状态协方差P↑，不确定度增大
            accel_var(i) = sq(BADACC_BIAS_PNOISE);
        } else {
            accel_var(i) = sq(accel_noise);
        }
    }

    // ========================================================================
    // 步骤2：核心协方差传播（符号推导自动生成）
    // ========================================================================

    // 调用自动生成的协方差预测函数
    //
    // 生成工具链：
    // 1. Python SymPy定义状态方程：f(x, u)
    // 2. 符号求导雅可比矩阵：F = ∂f/∂x
    // 3. CodeGen生成优化C++代码
    //
    // 优势：
    // - 精度：符号计算无舍入误差
    // - 效率：编译期常量折叠，零开销抽象
    // - 可维护性：状态扩展时自动重新生成
    //
    // 输入参数：
    // - _state.vector(): 当前状态估计
    // - P: 当前状态协方差
    // - accel: 去偏置加速度
    // - accel_var: 加速度噪声方差
    // - gyro: 去偏置角速度
    // - gyro_var: 陀螺噪声方差
    // - dt: 积分时间步长
    //
    // 输出：
    // - 更新后的协方差矩阵P
    P = sym::PredictCovariance(
        _state.vector(),
        P,
        imu_delayed.delta_vel / imu_delayed.delta_vel_dt,  // 加速度（去积分）
        accel_var,
        imu_delayed.delta_ang / imu_delayed.delta_ang_dt,  // 角速度（去积分）
        gyro_var,
        dt
    );

    // ========================================================================
    // 步骤3：偏置状态过程噪声注入
    // ========================================================================

    // 3.1 陀螺偏置过程噪声
    //
    // 物理模型：一阶马尔可夫过程（随机游走）
    // b_{k+1} = b_k + w_k,  w_k ~ N(0, σ²·Δt)
    //
    // 参数EKF2_GYR_B_NOISE：偏置漂移率（rad/s²）
    // 物理意义：温度变化1°C导致的偏置变化
    {
        const float gyro_bias_sig = dt * _params.ekf2_gyr_b_noise;
        const float gyro_bias_process_noise = sq(gyro_bias_sig);

        for (unsigned index = 0; index < State::gyro_bias.dof; index++) {
            const unsigned i = State::gyro_bias.idx + index;

            // 条件注入：仅当协方差未饱和时才增加
            // 原因：避免过度悲观的不确定度估计
            if (P(i, i) < gyro_var) {
                P(i, i) += gyro_bias_process_noise;
            }
        }
    }

    // 3.2 加速度计偏置过程噪声
    //
    // 建模原理同陀螺偏置
    // 差异：漂移率通常比陀螺大（振动敏感性高）
    {
        const float accel_bias_sig = dt * _params.ekf2_acc_b_noise;
        const float accel_bias_process_noise = sq(accel_bias_sig);

        for (unsigned index = 0; index < State::accel_bias.dof; index++) {
            const unsigned i = State::accel_bias.idx + index;

            if (P(i, i) < accel_var(index)) {
                P(i, i) += accel_bias_process_noise;
            }
        }
    }

    // ========================================================================
    // 步骤4：地磁场状态过程噪声
    // ========================================================================

#if defined(CONFIG_EKF2_MAGNETOMETER)
    // 4.1 地球磁场（mag_I）过程噪声
    //
    // 物理背景：
    // - 地磁场缓慢变化（地核液态铁运动）
    // - 磁偏角年变化率 ~0.1°/year
    // - 短期（分钟级）可视为常量
    //
    // 过程噪声来源：
    // - 建模误差（简化为常量）
    // - 局部磁异常（铁矿、建筑钢筋）
    float mag_I_sig = dt * _params.ekf2_mag_e_noise;
    float mag_I_process_noise = sq(mag_I_sig);

    for (unsigned index = 0; index < State::mag_I.dof; index++) {
        const unsigned i = State::mag_I.idx + index;

        if (P(i, i) < sq(_params.ekf2_mag_noise)) {
            P(i, i) += mag_I_process_noise;
        }
    }

    // 4.2 机体磁偏置（mag_B）过程噪声
    //
    // 物理来源：
    // - 硬铁干扰：电机/电池产生恒定磁场
    // - 软铁干扰：铁磁材料改变磁场方向
    // - 电流变化导致磁场变化
    float mag_B_sig = dt * _params.ekf2_mag_b_noise;
    float mag_B_process_noise = sq(mag_B_sig);

    for (unsigned index = 0; index < State::mag_B.dof; index++) {
        const unsigned i = State::mag_B.idx + index;

        if (P(i, i) < sq(_params.ekf2_mag_noise)) {
            P(i, i) += mag_B_process_noise;
        }
    }
#endif // CONFIG_EKF2_MAGNETOMETER

    // ========================================================================
    // 步骤5：风速过程噪声（高度自适应）
    // ========================================================================

#if defined(CONFIG_EKF2_WIND)
    // 物理直觉：
    // - 水平飞行：风场相对稳定
    // - 爬升/下降：穿越不同风层，风速剧变
    //
    // 自适应策略：
    // - 基础噪声：σ_base = EKF2_WIND_NSD
    // - 高度率缩放：σ = σ_base · (1 + α·|vz|)
    // - α = wind_vel_nsd_scaler（通常0.5）
    //
    // 示例：
    // - vz = 0 (悬停)：σ = σ_base
    // - vz = 5 m/s (爬升)：σ = 3.5·σ_base
    const float height_rate = _height_rate_lpf.update(_state.vel(2), imu_delayed.delta_vel_dt);
    const float wind_vel_nsd_scaled = _params.ekf2_wind_nsd *
                                       (1.f + _params.wind_vel_nsd_scaler * fabsf(height_rate));
    const float wind_vel_process_noise = sq(wind_vel_nsd_scaled) * dt;

    for (unsigned index = 0; index < State::wind_vel.dof; index++) {
        const unsigned i = State::wind_vel.idx + index;

        if (P(i, i) < sq(_params.initial_wind_uncertainty)) {
            P(i, i) += wind_vel_process_noise;
        }
    }
#endif // CONFIG_EKF2_WIND

    // ========================================================================
    // 步骤6：地形高度过程噪声（速度自适应）
    // ========================================================================

#if defined(CONFIG_EKF2_TERRAIN)
    if (_height_sensor_ref != HeightSensor::RANGE) {
        // 6.1 基础不确定度增长
        // 原因：高度测量误差传递到地形估计
        float terrain_process_noise = sq(dt * _params.ekf2_terr_noise);

        // 6.2 地形梯度导致的附加不确定度
        //
        // 物理模型：
        // - 假设地形梯度为 grad = EKF2_TERR_GRAD (m/m)
        // - 飞行器水平移动距离 d = √(vₙ² + vₑ²)·Δt
        // - 地形高度变化 Δh = grad · d
        // - 对应不确定度 σ²_h = (grad · d)²
        //
        // 示例：
        // - grad = 0.5（45°斜坡）
        // - v_horiz = 10 m/s
        // - Δt = 0.01 s
        // - σ_h = 0.5 × 0.1 = 0.05 m
        terrain_process_noise += sq(dt * _params.ekf2_terr_grad) *
                                  (sq(_state.vel(0)) + sq(_state.vel(1)));

        P(State::terrain.idx, State::terrain.idx) += terrain_process_noise;
    }
#endif // CONFIG_EKF2_TERRAIN

    // ========================================================================
    // 步骤7：协方差矩阵对称化
    // ========================================================================

    // 数值问题：
    // - 浮点运算舍入误差导致P非严格对称
    // - 协方差矩阵必须对称正定（数学性质）
    //
    // 解决方案：
    // - 强制对称化：P = (P + P^T) / 2
    // - 这里采用简化版本：复制上三角到下三角
    for (unsigned row = 0; row < State::size; row++) {
        for (unsigned column = 0; column < row; column++) {
            P(row, column) = P(column, row);
        }
    }

    // ========================================================================
    // 步骤8：协方差限幅（防止数值发散）
    // ========================================================================

    // 调用约束函数（见下面实现）
    constrainStateVariances();
}

/**
 * @brief 协方差矩阵约束函数
 *
 * 目的：
 * 1. 防止协方差数值下溢（过度自信）
 * 2. 防止协方差数值上溢（状态发散）
 * 3. 保持滤波器数值稳定性
 *
 * 策略：
 * - 下限：硬限幅（直接设置最小值）
 * - 上限：软限幅（通过融合零创新观测）
 *
 * 为什么上限用软限幅？
 * - 硬限幅会破坏协方差矩阵一致性（P与实际误差不匹配）
 * - 融合零创新相当于注入"伪观测"，保持滤波器一致性
 */
void Ekf::constrainStateVariances()
{
    // 限幅参数（单位：对应状态单位的平方）

    // 四元数协方差：[1e-9, 1.0]
    // - 1e-9：对应 ~0.001° 不确定度（极度自信）
    // - 1.0：对应 ~60° 不确定度（失去姿态信息）
    constrainStateVar(State::quat_nominal, 1e-9f, 1.f);

    // 速度协方差：[1e-6, 1e6] m²/s²
    // - 1e-6：对应 1mm/s 不确定度
    // - 1e6：对应 1km/s 不确定度（完全发散）
    constrainStateVar(State::vel, 1e-6f, 1e6f);

    // 位置协方差：[1e-6, 1e6] m²
    // - 1e-6：对应 1mm 不确定度
    // - 1e6：对应 1km 不确定度
    constrainStateVar(State::pos, 1e-6f, 1e6f);

    // 陀螺偏置协方差：[1e-9, 1.0] rad²/s²
    // - 限制比率：防止偏置不确定度超过偏置本身
    constrainStateVarLimitRatio(State::gyro_bias, kGyroBiasVarianceMin, 1.f);

    // 加速度偏置协方差
    constrainStateVarLimitRatio(State::accel_bias, kAccelBiasVarianceMin, 1.f);

#if defined(CONFIG_EKF2_MAGNETOMETER)
    if (_control_status.flags.mag) {
        constrainStateVarLimitRatio(State::mag_I, kMagVarianceMin, 1.f);
        constrainStateVarLimitRatio(State::mag_B, kMagVarianceMin, 1.f);
    }
#endif

#if defined(CONFIG_EKF2_WIND)
    if (_control_status.flags.wind) {
        constrainStateVarLimitRatio(State::wind_vel, 1e-6f, 1e6f);
    }
#endif

#if defined(CONFIG_EKF2_TERRAIN)
    constrainStateVarLimitRatio(State::terrain, 0.f, 1e4f);
#endif
}

/**
 * @brief 单个状态的协方差约束实现
 *
 * @param state 状态索引/维度结构体
 * @param min 最小协方差
 * @param max 最大协方差
 */
void Ekf::constrainStateVar(const IdxDof &state, float min, float max)
{
    for (unsigned i = state.idx; i < (state.idx + state.dof); i++) {
        if (P(i, i) < min) {
            // 下限违反：直接设置为最小值
            P(i, i) = min;

        } else if (P(i, i) > max) {
            // 上限违反：融合零创新虚拟观测
            //
            // 设计原理：
            // - 创新 innov = 0（观测值 = 预测值）
            // - 观测噪声 R = 10·P（使卡尔曼增益K ≈ 0.1）
            // - 更新后协方差 P' = (1-K)·P ≈ 0.9·P
            //
            // 效果：
            // - 协方差缓慢下降（约10%）
            // - 状态估计不变（innov = 0）
            // - 保持滤波器一致性
            const float innov = 0.f;
            const float R = 10.f * P(i, i);
            const float innov_var = P(i, i) + R;

            fuseDirectStateMeasurement(innov, innov_var, R, i);
        }
    }
}

// ============================================================================
// 总结：PX4 EKF2状态预测的核心设计哲学
// ============================================================================
//
// 1. 精确非线性建模
//    - 四元数姿态（无万向锁）
//    - 科里奥利力（高精度导航）
//    - 锥效应补偿（姿态积分精度）
//
// 2. 自适应噪声调整
//    - 传感器故障时增大过程噪声
//    - 风速噪声随高度率变化
//    - 地形噪声随水平速度变化
//
// 3. 数值稳定性保障
//    - 协方差对称化
//    - 软限幅策略（融合零创新）
//    - 梯形积分（二阶精度）
//
// 4. 模块化与可扩展性
//    - 状态/协方差预测分离
//    - 符号推导自动生成代码
//    - 配置宏控制可选状态
//
// 5. 物理直觉驱动
//    - 每个噪声项都有明确物理解释
//    - 自适应策略基于物理规律（风层/地形梯度）
//    - 故障缓解策略符合工程实践
