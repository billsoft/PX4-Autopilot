"""
PX4 EKF2 IMU数据处理模块
包含积分、圆锥补偿、振动监控、剪切检测
"""
import numpy as np
from state import IMUSample
from utils import WelfordOnlineVariance


class IMUProcessor:
    """
    IMU数据处理器
    功能:
    1. 角速度/加速度积分
    2. 圆锥效应补偿（Coning Compensation）
    3. 振动监控（Welford算法）
    4. 剪切检测
    """

    def __init__(self, target_dt: float = 0.004):
        """
        target_dt: 目标积分周期（秒），默认4ms = 250Hz
        """
        self.target_dt = target_dt

        # 积分累加器
        self.delta_ang_accum = np.zeros(3)
        self.delta_vel_accum = np.zeros(3)
        self.integration_time = 0.0

        # 圆锥补偿历史
        self.delta_ang_prev = np.zeros(3)

        # 振动监控
        self.accel_variance = WelfordOnlineVariance()
        self.gyro_variance = WelfordOnlineVariance()

        # 剪切检测
        self.clipping_detected = np.zeros(3, dtype=bool)

        # 统计信息
        self.sample_count = 0

    def update(self, gyro: np.ndarray, accel: np.ndarray,
               dt: float, clipping: np.ndarray = None) -> IMUSample:
        """
        更新IMU数据，累积积分

        参数:
        gyro: 角速度（rad/s）
        accel: 加速度（m/s²）
        dt: 采样间隔（s）
        clipping: 剪切标志（可选）

        返回:
        IMUSample 或 None（未到积分周期）
        """
        # 角增量/速度增量
        delta_ang = gyro * dt
        delta_vel = accel * dt

        # 累积积分
        self.delta_ang_accum += delta_ang
        self.delta_vel_accum += delta_vel
        self.integration_time += dt

        # 振动监控（更新方差）
        self.accel_variance.update(np.linalg.norm(accel))
        self.gyro_variance.update(np.linalg.norm(gyro))

        # 剪切检测
        if clipping is not None:
            self.clipping_detected |= clipping

        self.sample_count += 1

        # 检查是否达到目标积分周期
        if self.integration_time >= self.target_dt:
            # 应用圆锥补偿
            delta_ang_compensated = self._apply_coning_compensation(
                self.delta_ang_accum
            )

            # 构造IMU样本
            sample = IMUSample(
                time_us=int(np.round(self.integration_time * 1e6)),
                delta_ang=delta_ang_compensated,
                delta_vel=self.delta_vel_accum,
                delta_ang_dt=self.integration_time,
                delta_vel_dt=self.integration_time,
                delta_vel_clipping=self.clipping_detected.copy()
            )

            # 重置累加器
            self._reset_accumulators()

            return sample

        return None

    def _apply_coning_compensation(self, delta_ang: np.ndarray) -> np.ndarray:
        """
        圆锥效应补偿

        物理背景:
        - 飞行器同时绕多个轴旋转时，旋转矢量不满足交换律
        - 简单积分会产生圆锥误差

        补偿公式（二阶精度）:
        Δθ_compensated = Δθ_k + (1/12) * (Δθ_{k-1} × Δθ_k)

        精度提升: 约10倍误差减小
        """
        # 叉乘项: Δθ_{k-1} × Δθ_k
        cross_correction = np.cross(self.delta_ang_prev, delta_ang)

        # 补偿系数: 1/12（来自泰勒展开）
        compensated = delta_ang + cross_correction / 12.0

        # 更新历史
        self.delta_ang_prev = delta_ang.copy()

        return compensated

    def _reset_accumulators(self):
        """重置积分累加器"""
        self.delta_ang_accum = np.zeros(3)
        self.delta_vel_accum = np.zeros(3)
        self.integration_time = 0.0
        self.clipping_detected = np.zeros(3, dtype=bool)

    def get_vibration_metrics(self) -> dict:
        """
        获取振动指标

        返回:
        {
            'accel_std': 加速度标准差（m/s²）,
            'gyro_std': 角速度标准差（rad/s）,
            'vibration_level': 振动等级（0-3）
        }
        """
        accel_std = self.accel_variance.get_std()
        gyro_std = self.gyro_variance.get_std()

        # 振动等级评估
        # 0: 优秀, 1: 良好, 2: 一般, 3: 差
        if accel_std < 0.5:
            level = 0
        elif accel_std < 2.0:
            level = 1
        elif accel_std < 5.0:
            level = 2
        else:
            level = 3

        return {
            'accel_std': accel_std,
            'gyro_std': gyro_std,
            'vibration_level': level
        }

    def reset_statistics(self):
        """重置统计信息"""
        self.accel_variance.reset()
        self.gyro_variance.reset()
        self.sample_count = 0


class ConingIntegrator:
    """
    圆锥积分器（独立实现）

    参考文献:
    Miller, R. B. (1983). "A new strapdown attitude algorithm"
    Journal of Guidance, Control, and Dynamics

    Savage, P. G. (1998). "Strapdown Analytics"
    """

    def __init__(self):
        self.alpha_prev = np.zeros(3)
        self.beta = np.zeros(3)
        self.q = None

    def integrate(self, gyro: np.ndarray, dt: float) -> np.ndarray:
        alpha = gyro * dt
        self.beta += np.cross(self.alpha_prev, alpha) / 12.0
        theta_step = alpha + self.beta

        if self.q is None:
            from utils import Quaternion
            self.q = Quaternion()

        from utils import Quaternion
        dq = Quaternion.from_rotation_vector(theta_step)
        self.q = (self.q * dq).normalized()

        w, x, y, z = self.q.q
        angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
        s = np.sqrt(max(1.0 - w * w, 0.0))
        if s < 1e-12:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            axis = np.array([x, y, z]) / s
        return axis * angle

    def reset(self):
        """重置积分器"""
        self.alpha_prev = np.zeros(3)
        self.beta = np.zeros(3)


class ScullingIntegrator:
    """
    划桨效应补偿器（Sculling Compensation）

    物理背景:
    - 振动环境下，加速度计测量包含高频振动
    - 速度积分会产生划桨误差

    补偿公式:
    Δv_compensated = Δv_k + (1/12) * (Δθ_{k-1} × Δv_k + Δv_{k-1} × Δθ_k)

    注: PX4默认未启用（效果不如圆锥补偿显著）
    """

    def __init__(self):
        self.delta_ang_prev = np.zeros(3)
        self.delta_vel_prev = np.zeros(3)
        self.sculling_accum = np.zeros(3)

    def integrate(self, delta_ang: np.ndarray, delta_vel: np.ndarray) -> np.ndarray:
        """
        应用划桨补偿

        参数:
        delta_ang: 角增量（rad）
        delta_vel: 速度增量（m/s）

        返回:
        补偿后的速度增量（m/s）
        """
        # 叉乘校正项
        cross1 = np.cross(self.delta_ang_prev, delta_vel)
        cross2 = np.cross(self.delta_vel_prev, delta_ang)

        self.sculling_accum += (cross1 + cross2) / 12.0

        # 补偿后的速度增量
        vel_compensated = delta_vel + self.sculling_accum

        # 更新历史
        self.delta_ang_prev = delta_ang
        self.delta_vel_prev = delta_vel

        return vel_compensated

    def reset(self):
        """重置积分器"""
        self.delta_ang_prev = np.zeros(3)
        self.delta_vel_prev = np.zeros(3)
        self.sculling_accum = np.zeros(3)
