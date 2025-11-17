"""
PX4 EKF2 工具函数模块
包含四元数运算、旋转矩阵、数值工具等
"""
import numpy as np
from typing import Tuple


class Quaternion:
    """
    四元数类: q = [w, x, y, z] = [scalar, vector]
    用于表示3D旋转，避免万向节死锁
    """

    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.q = np.array([w, x, y, z], dtype=np.float64)

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'Quaternion':
        """从numpy数组创建"""
        q = cls()
        q.q = arr.copy()
        return q

    @classmethod
    def from_axis_angle(cls, axis: np.ndarray, angle: float) -> 'Quaternion':
        """
        从轴角表示创建四元数
        axis: 旋转轴（单位向量）
        angle: 旋转角度（弧度）
        """
        half_angle = angle / 2.0
        sin_half = np.sin(half_angle)
        cos_half = np.cos(half_angle)

        q = cls()
        q.q[0] = cos_half
        q.q[1:4] = axis * sin_half
        return q

    @classmethod
    def from_rotation_vector(cls, rotvec: np.ndarray) -> 'Quaternion':
        """
        从旋转向量创建四元数
        rotvec: 旋转向量（方向=轴，模长=角度）
        用于IMU角增量积分
        """
        angle = np.linalg.norm(rotvec)

        if angle < 1e-10:
            # 小角度近似: q ≈ [1, θ/2]
            q = cls()
            q.q[0] = 1.0
            q.q[1:4] = rotvec * 0.5
            return q.normalized()

        axis = rotvec / angle
        return cls.from_axis_angle(axis, angle)

    def __mul__(self, other: 'Quaternion') -> 'Quaternion':
        """
        四元数乘法: q1 ⊗ q2
        注意: 非交换！顺序很重要
        物理意义: 先应用q2旋转，再应用q1旋转
        """
        w1, x1, y1, z1 = self.q
        w2, x2, y2, z2 = other.q

        result = Quaternion()
        result.q[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2
        result.q[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2
        result.q[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2
        result.q[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return result

    def conjugate(self) -> 'Quaternion':
        """共轭四元数: q* = [w, -x, -y, -z]"""
        result = Quaternion()
        result.q[0] = self.q[0]
        result.q[1:4] = -self.q[1:4]
        return result

    def normalized(self) -> 'Quaternion':
        """归一化四元数"""
        norm = np.linalg.norm(self.q)
        result = Quaternion()
        result.q = self.q / norm
        return result

    def to_rotation_matrix(self) -> np.ndarray:
        """
        转换为旋转矩阵 (DCM - Direction Cosine Matrix)
        R: 将机体坐标系向量转换到世界坐标系
        """
        w, x, y, z = self.q

        R = np.array([
            [1-2*(y**2+z**2),   2*(x*y-w*z),     2*(x*z+w*y)],
            [2*(x*y+w*z),       1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y),       2*(y*z+w*x),     1-2*(x**2+y**2)]
        ])
        return R

    def rotate_vector(self, v: np.ndarray) -> np.ndarray:
        """
        使用四元数旋转向量
        v_world = q ⊗ [0, v_body] ⊗ q*
        """
        # 构造纯四元数 [0, v]
        v_quat = Quaternion(0, v[0], v[1], v[2])

        # 旋转: q ⊗ v ⊗ q*
        result = self * v_quat * self.conjugate()
        return result.q[1:4]

    def to_euler(self) -> Tuple[float, float, float]:
        """
        转换为欧拉角 (roll, pitch, yaw)
        ZYX顺序，单位：弧度
        """
        w, x, y, z = self.q

        # Roll (X轴)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (Y轴)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # ±90°时的奇异点
        else:
            pitch = np.arcsin(sinp)

        # Yaw (Z轴)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """
    反对称矩阵: [v]×
    用于叉乘: v × w = [v]× * w
    """
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def constrain(val: float, min_val: float, max_val: float) -> float:
    """数值限幅"""
    return max(min_val, min(max_val, val))


def wrap_pi(angle: float) -> float:
    """将角度限制在[-π, π]范围内"""
    return np.arctan2(np.sin(angle), np.cos(angle))


def is_positive_definite(P: np.ndarray, tol: float = 1e-9) -> bool:
    """
    检查矩阵是否正定
    用于协方差矩阵数值稳定性验证
    """
    eigenvalues = np.linalg.eigvalsh(P)
    return np.all(eigenvalues > tol)


def make_symmetric(P: np.ndarray) -> np.ndarray:
    """
    强制矩阵对称
    P = (P + P^T) / 2
    用于修正浮点运算导致的非对称性
    """
    return (P + P.T) / 2.0


def innovation_variance(H: np.ndarray, P: np.ndarray, R: float) -> float:
    """
    计算创新协方差
    S = H * P * H^T + R
    """
    return (H @ P @ H.T) + R


def kalman_gain(P: np.ndarray, H: np.ndarray, S: float) -> np.ndarray:
    """
    计算卡尔曼增益
    K = P * H^T * S^(-1)
    """
    return (P @ H.T) / S


def joseph_form_covariance_update(P: np.ndarray, K: np.ndarray,
                                  H: np.ndarray, R: float) -> np.ndarray:
    """
    Joseph形式协方差更新（数值稳定）
    P = (I - K*H) * P * (I - K*H)^T + K * R * K^T

    优点: 保证正定性，即使浮点误差存在
    """
    n = P.shape[0]
    I = np.eye(n)
    IKH = I - K @ H

    P_updated = IKH @ P @ IKH.T + K * R * K.T
    return make_symmetric(P_updated)


class WelfordOnlineVariance:
    """
    Welford在线方差算法
    用于IMU振动监控，单遍扫描，数值稳定
    """

    def __init__(self):
        self.count = 0
        self.mean = 0.0
        self.M2 = 0.0  # 平方差之和

    def update(self, value: float):
        """添加新样本"""
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        delta2 = value - self.mean
        self.M2 += delta * delta2

    def get_variance(self) -> float:
        """获取方差"""
        if self.count < 2:
            return 0.0
        return self.M2 / self.count

    def get_std(self) -> float:
        """获取标准差"""
        return np.sqrt(self.get_variance())

    def reset(self):
        """重置"""
        self.count = 0
        self.mean = 0.0
        self.M2 = 0.0
