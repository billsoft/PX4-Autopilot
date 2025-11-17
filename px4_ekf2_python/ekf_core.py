"""
PX4 EKF2 核心算法模块
包含状态预测、协方差预测、观测更新
"""
import numpy as np
from typing import Optional
from state import StateVector, IMUSample, Parameters
from utils import (Quaternion, skew_symmetric, make_symmetric,
                   kalman_gain, joseph_form_covariance_update,
                   constrain)


class EKF2Core:
    """
    24维EKF核心算法

    主要功能:
    1. 状态预测（IMU积分）
    2. 协方差预测（线性化）
    3. 观测更新（卡尔曼增益）
    4. 自适应噪声调整
    """

    def __init__(self, params: Optional[Parameters] = None):
        """初始化EKF"""
        self.params = params if params is not None else Parameters()

        # 状态向量
        self.state = StateVector()

        # 协方差矩阵 (24x24)
        self.P = self._initialize_covariance()

        # 故障状态标志
        self.fault_status = {
            'bad_acc_vertical': False,
            'high_maneuver': False,
            'gps_lost': False
        }

        # 控制状态标志
        self.control_status = {
            'gravity_vector_fusion': False  # 重力融合是否激活
        }

        # 偏置学习抑制标志
        self.accel_bias_inhibit = np.zeros(3, dtype=bool)
        self.gyro_bias_inhibit = False

        # 机动检测（包络滤波器）
        self.accel_magnitude_filt = 0.0
        self.gyro_magnitude_filt = 0.0

        # 时间管理
        self.time_last_imu = 0

        # 地球自转角速度（NED坐标系）
        self._update_earth_rotation_vector()

    def _initialize_covariance(self) -> np.ndarray:
        """
        初始化协方差矩阵

        对角元素初始值:
        - 姿态: 1e-4 rad²
        - 速度: 1.0 m²/s²
        - 位置: 100.0 m²
        - 陀螺偏置: 1e-3 rad²/s²
        - 加计偏置: 1e-2 m²/s⁴
        """
        P = np.eye(StateVector.STATE_DIM) * 1e-6  # 小值初始化

        # 姿态（四元数误差状态是3维旋转向量）
        P[0:3, 0:3] = np.eye(3) * self.params.init_quat_var

        # 速度
        P[3:6, 3:6] = np.eye(3) * self.params.init_vel_var

        # 位置
        P[6:9, 6:9] = np.eye(3) * self.params.init_pos_var

        # 陀螺偏置
        P[9:12, 9:12] = np.eye(3) * self.params.init_gyro_bias_var

        # 加计偏置
        P[12:15, 12:15] = np.eye(3) * self.params.init_accel_bias_var

        # 其他状态（磁场、风速）使用小值
        return P

    def _update_earth_rotation_vector(self):
        """
        计算地球自转角速度在NED坐标系的投影

        Ω_earth_NED = Ω_earth * [cos(lat), 0, -sin(lat)]^T

        物理意义：
        - 北向分量: cos(纬度) * Ω，赤道为0，极点最大
        - 东向分量: 0（地球自转轴无东向分量）
        - 下向分量: -sin(纬度) * Ω，赤道最大，极点为0
        """
        omega_earth = self.params.earth_rotation_rate  # 7.2921e-5 rad/s
        lat = self.params.latitude

        self.earth_rate_NED = np.array([
            np.cos(lat) * omega_earth,  # 北向分量
            0.0,                        # 东向分量（无）
            -np.sin(lat) * omega_earth  # 下向分量
        ])

    def set_latitude(self, latitude_deg: float):
        """
        设置纬度（用于科里奥利修正）

        参数:
        latitude_deg: 纬度（度）

        影响：更新 earth_rate_NED
        """
        self.params.latitude = np.radians(latitude_deg)
        self._update_earth_rotation_vector()

    def predict_state(self, imu: IMUSample):
        """
        状态预测（IMU积分）

        状态方程:
        q̇ = 0.5 * q ⊗ ω                    # 四元数微分
        v̇ = R(q) * (a - b_a) + g           # 速度微分
        ṗ = v                               # 位置微分
        ḃ_gyro = 0                          # 偏置随机游走
        ḃ_accel = 0
        """
        dt = imu.delta_ang_dt

        # 1. 提取状态
        q = self.state.quat
        v = self.state.vel
        p = self.state.pos
        gyro_bias = self.state.gyro_bias
        accel_bias = self.state.accel_bias

        # 2. 补偿偏置
        delta_ang_corrected = imu.delta_ang - gyro_bias * dt
        delta_vel_corrected = imu.delta_vel - accel_bias * dt

        # 3. 姿态更新（四元数积分）
        # q_{k+1} = q_k ⊗ exp(Δθ/2)
        dq = Quaternion.from_rotation_vector(delta_ang_corrected)
        q_new = (q * dq).normalized()

        # 4. 速度更新（梯形法）
        # 机体系加速度 → NED加速度
        accel_body = imu.delta_vel / dt
        R_old = q.to_rotation_matrix()
        R_new = q_new.to_rotation_matrix()

        # 梯形积分: v += 0.5 * (R_old + R_new) * a_body * dt + g * dt
        accel_ned_old = R_old @ (accel_body - accel_bias)
        accel_ned_new = R_new @ (accel_body - accel_bias)
        accel_ned_avg = 0.5 * (accel_ned_old + accel_ned_new)

        # 添加重力（NED坐标系，向下为正）
        gravity_ned = np.array([0, 0, self.params.gravity])

        v_new = v + (accel_ned_avg + gravity_ned) * dt

        # 科里奥利力修正（高速/高纬度场景）
        # v̇_coriolis = -2 * Ω_earth × v
        # 使用梯形法：取平均速度
        if self.params.enable_coriolis_correction:
            v_avg = 0.5 * (v + v_new)
            coriolis_correction = -2.0 * np.cross(self.earth_rate_NED, v_avg) * dt
            v_new += coriolis_correction

        # 5. 位置更新（梯形法）
        # p += 0.5 * (v_old + v_new) * dt
        p_new = p + 0.5 * (v + v_new) * dt

        # 6. 更新状态
        self.state.quat = q_new
        self.state.vel = v_new
        self.state.pos = p_new

        # 偏置保持不变（随机游走模型）
        # gyro_bias, accel_bias 不变

        # 7. 更新机动检测
        self._update_maneuver_detection(imu)

    def predict_covariance(self, imu: IMUSample):
        """
        协方差预测

        P_{k+1|k} = F * P_k * F^T + Q

        F: 状态转移雅可比矩阵（24x24）
        Q: 过程噪声协方差矩阵
        """
        dt = imu.delta_ang_dt

        # 1. 计算状态转移矩阵 F（简化版）
        F = self._compute_state_transition_matrix(imu, dt)

        # 2. 计算过程噪声 Q（自适应）
        Q = self._compute_process_noise(imu, dt)

        # 3. 协方差预测
        # P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + Q

        # 4. 强制对称（数值稳定性）
        self.P = make_symmetric(self.P)

        # 5. 限制协方差增长（防止发散）
        self._constrain_covariance()

    def _compute_state_transition_matrix(self, imu: IMUSample, dt: float) -> np.ndarray:
        """
        计算状态转移雅可比矩阵 F

        注: PX4使用SymPy符号推导自动生成
        这里使用简化版本

        F ≈ I + ∂f/∂x * dt
        """
        n = StateVector.STATE_DIM
        F = np.eye(n)

        # 提取状态
        R = self.state.get_rotation_matrix()
        accel_body = imu.delta_vel / dt - self.state.accel_bias

        # 姿态 → 速度耦合: ∂v/∂θ = -R * [a]×
        # (旋转误差导致加速度投影误差)
        F[3:6, 0:3] = -R @ skew_symmetric(accel_body) * dt

        # 速度 → 位置: ∂p/∂v = I * dt
        F[6:9, 3:6] = np.eye(3) * dt

        # 加速度偏置 → 速度: ∂v/∂b_a = -R * dt
        F[3:6, 12:15] = -R * dt

        # 陀螺偏置 → 姿态: ∂θ/∂b_g = -I * dt
        F[0:3, 9:12] = -np.eye(3) * dt

        return F

    def _compute_process_noise(self, imu: IMUSample, dt: float) -> np.ndarray:
        """
        计算过程噪声协方差矩阵 Q

        自适应策略:
        1. 正常情况: 使用参数配置的噪声
        2. 高机动/故障: 增大100倍（降低对IMU依赖）
        """
        n = StateVector.STATE_DIM
        Q = np.zeros((n, n))

        # 陀螺噪声
        gyro_noise = self.params.gyro_noise
        if self.gyro_bias_inhibit or self.fault_status['high_maneuver']:
            gyro_noise *= 10.0  # 高机动时增大

        # 加计噪声
        accel_noise = self.params.accel_noise
        if self.fault_status['bad_acc_vertical'] or np.any(imu.delta_vel_clipping):
            accel_noise *= 100.0  # 剪切/故障时大幅增大

        # 姿态噪声（来自陀螺）
        Q[0:3, 0:3] = np.eye(3) * (gyro_noise * dt) ** 2

        # 速度噪声（来自加计）
        Q[3:6, 3:6] = np.eye(3) * (accel_noise * dt) ** 2

        # 位置噪声（来自速度积分）
        Q[6:9, 6:9] = np.eye(3) * (accel_noise * dt**2) ** 2

        # 陀螺偏置噪声（随机游走）
        Q[9:12, 9:12] = np.eye(3) * (self.params.gyro_bias_noise * dt) ** 2

        # 加计偏置噪声（随机游走）
        Q[12:15, 12:15] = np.eye(3) * (self.params.accel_bias_noise * dt) ** 2

        return Q

    def _update_maneuver_detection(self, imu: IMUSample):
        """
        更新机动检测（包络滤波器）

        用于偏置学习抑制判断
        """
        dt = imu.delta_ang_dt

        # 加速度幅值
        accel_norm = np.linalg.norm(imu.delta_vel / dt - self.state.accel_bias)

        # 角速度幅值
        gyro_norm = np.linalg.norm(imu.delta_ang / dt - self.state.gyro_bias)

        # 包络滤波器（取最大值，缓慢衰减）
        beta = 0.99  # 衰减系数
        self.accel_magnitude_filt = max(accel_norm, beta * self.accel_magnitude_filt)
        self.gyro_magnitude_filt = max(gyro_norm, beta * self.gyro_magnitude_filt)

        # 高机动判断
        high_accel = self.accel_magnitude_filt > self.params.accel_bias_lim
        high_gyro = self.gyro_magnitude_filt > self.params.gyro_bias_lim
        self.fault_status['high_maneuver'] = high_accel or high_gyro

        # 偏置学习抑制（轴向选择性）
        do_inhibit_all_axes = self.fault_status['high_maneuver'] or self.fault_status['bad_acc_vertical']

        # 陀螺偏置抑制（全轴）
        self.gyro_bias_inhibit = do_inhibit_all_axes

        # 加速度偏置抑制（轴向选择性）
        R = self.state.get_rotation_matrix()  # DCM: 机体→NED

        for axis in range(3):
            # 检查该轴是否可观（是否与重力对齐）
            is_bias_observable = True

            if self.control_status['gravity_vector_fusion']:
                # 重力融合启用时：仅Z轴（与重力对齐）可观
                # R[2, axis]: NED-Down方向在机体轴的投影
                # cos(15°) ≈ 0.966
                is_bias_observable = (abs(R[2, axis]) > 0.966)

            # 综合判断
            self.accel_bias_inhibit[axis] = (
                do_inhibit_all_axes or
                not is_bias_observable or
                imu.delta_vel_clipping[axis]
            )

    def _constrain_covariance(self):
        """
        限制协方差增长

        防止数值发散，设置上下限
        """
        # 姿态方差上限: 1 rad² (约57°)
        self.P[0:3, 0:3] = np.clip(self.P[0:3, 0:3], 1e-8, 1.0)

        # 速度方差上限: 100 m²/s²
        self.P[3:6, 3:6] = np.clip(self.P[3:6, 3:6], 1e-4, 100.0)

        # 位置方差上限: 10000 m² (100m标准差)
        self.P[6:9, 6:9] = np.clip(self.P[6:9, 6:9], 1e-2, 10000.0)

        # 偏置方差上限
        self.P[9:12, 9:12] = np.clip(self.P[9:12, 9:12], 1e-6, 0.1)   # 陀螺
        self.P[12:15, 12:15] = np.clip(self.P[12:15, 12:15], 1e-4, 1.0)  # 加计

    def update_scalar_observation(self, innovation: float, H: np.ndarray,
                                   R: float, gate: float = 5.0) -> bool:
        """
        标量观测更新（通用接口）

        参数:
        innovation: 创新（观测 - 预测）
        H: 观测矩阵（1 x 24）
        R: 观测噪声方差
        gate: 创新门限（标准差倍数）

        返回:
        是否融合成功
        """
        # 1. 创新协方差
        # S = H * P * H^T + R
        S = H @ self.P @ H.T + R

        # 2. 创新检验（马氏距离）
        # test_ratio = innovation² / (gate² * S)
        test_ratio = (innovation ** 2) / (gate ** 2 * S)

        if test_ratio > 1.0:
            # 创新过大，拒绝观测
            return False

        # 3. 计算卡尔曼增益
        K = (self.P @ H.T) / S

        # 4. 误差状态更新
        delta_x = K * innovation

        # 姿态误差状态 → 四元数更新
        rotvec = delta_x[0:3]
        if np.linalg.norm(rotvec) > 0.0:
            dq = Quaternion.from_rotation_vector(rotvec)
            self.state.quat = (self.state.quat * dq).normalized()
            self.state.x[StateVector.IDX_QUAT] = self.state.quat.q

        # 其余子状态增量更新
        self.state.vel += delta_x[3:6]
        self.state.pos += delta_x[6:9]
        self.state.gyro_bias += delta_x[9:12]
        self.state.accel_bias += delta_x[12:15]
        self.state.mag_I += delta_x[16:19]
        self.state.mag_B += delta_x[19:22]
        self.state.wind += delta_x[22:24]

        # 5. 协方差更新（Joseph形式）
        self.P = joseph_form_covariance_update(self.P, K.reshape(-1, 1), H.reshape(1, -1), R)

        return True

    def reset_covariance_submatrix(self, idx: slice, variance: float):
        """
        重置协方差子矩阵

        用于传感器切换或故障恢复
        """
        n = idx.stop - idx.start
        self.P[idx, idx] = np.eye(n) * variance

        # 清除该子块的协方差
        self.P[idx, :idx.start] = 0.0
        self.P[idx, idx.stop:] = 0.0
        self.P[:idx.start, idx] = 0.0
        self.P[idx.stop:, idx] = 0.0

    def get_state_variance(self, idx: slice) -> np.ndarray:
        """获取状态子块的方差（对角元素）"""
        return np.diag(self.P[idx, idx])
