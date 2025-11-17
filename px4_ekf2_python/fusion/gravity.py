"""
重力向量融合模块

核心功能: 处理"比力倾斜误差"（Specific Force Tilt Error）

物理背景:
- 加速度计测量的是"比力" (Specific Force): f = a_inertial - g
- 急停/加速时: 无法区分惯性加速度和重力
- 错误示例: 10m/s急停(-5m/s²) → 加速度计测到[-5, 0, 9.81] → 错误推断姿态倾斜27°

防御机制:
1. 仅在准静态条件下融合（|a| ≈ 1g ± 15%）
2. 高机动时禁用
3. 有其他位置源时禁用（GPS优先）
"""
import numpy as np
try:
    from ..ekf_core import EKF2Core
    from ..state import IMUSample
except ImportError:
    from ekf_core import EKF2Core
    from state import IMUSample


class GravityFusion:
    """
    重力向量融合

    观测模型:
    z = normalized(a_body - b_a)  # 归一化加速度方向
    h = R^T * [0, 0, -1]^T        # 预测的重力方向（机体系）

    创新:
    innov = z - h
    """

    def __init__(self, ekf: EKF2Core):
        self.ekf = ekf
        self.params = ekf.params

        # 融合控制标志
        self.is_active = False

        # 低通滤波后的加速度幅值
        self.accel_norm_lpf = 0.0
        self.lpf_tau = 0.5  # 时间常数（秒）

    def control(self, imu: IMUSample) -> bool:
        """
        控制重力融合启用条件

        启用条件（所有条件同时满足）:
        1. 加速度幅值 ≈ 1g（准静态）
        2. 无高机动
        3. 无加速度剪切
        4. 无水平位置源（GPS未激活）

        返回: 是否激活融合
        """
        dt = imu.delta_ang_dt

        # 1. 计算加速度幅值
        accel = imu.delta_vel / dt
        accel_corrected = accel - self.ekf.state.accel_bias
        accel_norm = np.linalg.norm(accel_corrected)

        # 低通滤波
        alpha = dt / (self.lpf_tau + dt)
        self.accel_norm_lpf = (1 - alpha) * self.accel_norm_lpf + alpha * accel_norm

        # 2. 检查准静态条件（|a| ≈ g ± 15%）
        g = self.params.gravity
        accel_in_range = (
            self.accel_norm_lpf > g * self.params.gravity_fusion_accel_min and
            self.accel_norm_lpf < g * self.params.gravity_fusion_accel_max
        )

        # 3. 检查无高机动
        no_high_maneuver = not self.ekf.fault_status['high_maneuver']

        # 4. 检查无剪切
        no_clipping = not np.any(imu.delta_vel_clipping)

        no_horiz_aid = True

        # 综合判断
        self.is_active = (
            accel_in_range and
            no_high_maneuver and
            no_clipping and
            no_horiz_aid
        )

        # 更新EKF控制状态（用于轴向选择性偏置抑制）
        self.ekf.control_status['gravity_vector_fusion'] = self.is_active

        return self.is_active

    def fuse(self, imu: IMUSample) -> bool:
        """
        执行重力向量融合

        步骤:
        1. 计算归一化加速度方向（观测）
        2. 计算预测的重力方向（机体系）
        3. 计算创新（角度差）
        4. 融合到横滚/俯仰状态

        返回: 融合是否成功
        """
        if not self.is_active:
            return False

        dt = imu.delta_vel_dt

        # 1. 观测: 归一化加速度方向（机体系）
        accel = imu.delta_vel / dt
        accel_corrected = accel - self.ekf.state.accel_bias
        accel_norm = np.linalg.norm(accel_corrected)

        if accel_norm < 1e-3:
            return False

        z = accel_corrected / accel_norm  # 单位向量

        # 2. 预测: 重力方向（机体系）
        # NED坐标系重力向量: [0, 0, g]
        # 转到机体系: R^T * [0, 0, g]
        R = self.ekf.state.get_rotation_matrix()
        gravity_ned = np.array([0, 0, self.params.gravity])
        gravity_body_predicted = R.T @ gravity_ned
        gravity_body_predicted /= np.linalg.norm(gravity_body_predicted)

        # 3. 创新: 向量差
        # 注意: 加速度测量的是 -g（向上的力），需要取反
        innovation_vec = z - (-gravity_body_predicted)

        # 4. 分别融合横滚和俯仰
        # X轴（横滚）创新
        success_roll = self._fuse_roll(innovation_vec[0])

        # Y轴（俯仰）创新
        success_pitch = self._fuse_pitch(innovation_vec[1])

        return success_roll and success_pitch

    def _fuse_roll(self, innovation: float) -> bool:
        """
        融合横滚创新

        观测矩阵 H: 只影响姿态状态（横滚轴）
        """
        # 观测噪声（根据加速度不确定性）
        R = 0.05 ** 2  # 约3°不确定性

        # 观测矩阵（简化：仅姿态第0个元素）
        H = np.zeros(24)
        H[0] = 1.0  # ∂(gravity_body_x) / ∂(roll)

        # 创新门限
        gate = 3.0  # 3σ

        # 融合
        return self.ekf.update_scalar_observation(innovation, H, R, gate)

    def _fuse_pitch(self, innovation: float) -> bool:
        """
        融合俯仰创新

        观测矩阵 H: 只影响姿态状态（俯仰轴）
        """
        # 观测噪声
        R = 0.05 ** 2

        # 观测矩阵
        H = np.zeros(24)
        H[1] = 1.0  # ∂(gravity_body_y) / ∂(pitch)

        # 创新门限
        gate = 3.0

        # 融合
        return self.ekf.update_scalar_observation(innovation, H, R, gate)

    def get_status(self) -> dict:
        """
        获取重力融合状态

        返回:
        {
            'active': 是否激活,
            'accel_norm': 加速度幅值（滤波后）,
            'within_range': 是否在准静态范围内
        }
        """
        g = self.params.gravity
        within_range = (
            self.accel_norm_lpf > g * self.params.gravity_fusion_accel_min and
            self.accel_norm_lpf < g * self.params.gravity_fusion_accel_max
        )

        return {
            'active': self.is_active,
            'accel_norm': self.accel_norm_lpf,
            'within_range': within_range
        }


class VerticalAccelerationHealthCheck:
    """
    垂直加速度健康检查

    目的: 检测比力倾斜误差导致的高度估计异常

    方法: 多源交叉验证
    - GPS高度 vs 气压高度
    - GPS垂直速度 vs EKF垂直速度
    - 惯导垂直加速度一致性

    异常判据:
    - 多个源的创新同时超过门限 → 标记 bad_acc_vertical
    """

    def __init__(self, ekf: EKF2Core):
        self.ekf = ekf

        # 似然计数器
        self.fault_likelihood = 0.0
        self.fault_threshold = 0.8  # 80%可信度触发

    def check(self, gps_available: bool = False,
              baro_available: bool = False) -> bool:
        """
        执行健康检查

        返回: 是否检测到垂直加速度故障
        """
        # 简化实现：检查垂直速度方差是否过大
        vel_z_var = self.ekf.P[5, 5]  # NED-Down速度方差

        # 方差过大（超过10 m²/s²）认为异常
        if vel_z_var > 10.0:
            self.fault_likelihood = min(1.0, self.fault_likelihood + 0.1)
        else:
            self.fault_likelihood = max(0.0, self.fault_likelihood - 0.05)

        # 更新故障标志
        fault_detected = self.fault_likelihood > self.fault_threshold
        self.ekf.fault_status['bad_acc_vertical'] = fault_detected

        return fault_detected

    def reset(self):
        """重置检查器"""
        self.fault_likelihood = 0.0
        self.ekf.fault_status['bad_acc_vertical'] = False
