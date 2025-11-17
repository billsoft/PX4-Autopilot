"""
PX4 EKF2 状态向量定义
24维状态: [q, v, p, gyro_bias, accel_bias, mag_I, mag_B, wind]
"""
import numpy as np
from utils import Quaternion


class StateVector:
    """
    EKF2 24维状态向量

    索引映射:
    0-3:   q (四元数姿态)
    4-6:   v (NED速度, m/s)
    7-9:   p (NED位置, m)
    10-12: gyro_bias (陀螺偏置, rad/s)
    13-15: accel_bias (加计偏置, m/s²)
    16-18: mag_I (NED磁场强度, Gauss)
    19-21: mag_B (机体磁场偏置, Gauss)
    22-23: wind (水平风速 NE, m/s)
    """

    # 状态索引常量
    IDX_QUAT = slice(0, 4)
    IDX_VEL = slice(4, 7)
    IDX_POS = slice(7, 10)
    IDX_GYRO_BIAS = slice(10, 13)
    IDX_ACCEL_BIAS = slice(13, 16)
    IDX_MAG_I = slice(16, 19)
    IDX_MAG_B = slice(19, 22)
    IDX_WIND = slice(22, 24)

    STATE_DIM = 24

    def __init__(self):
        """初始化状态向量"""
        self.x = np.zeros(self.STATE_DIM, dtype=np.float64)

        # 默认姿态: 单位四元数（无旋转）
        self.x[self.IDX_QUAT] = np.array([1.0, 0.0, 0.0, 0.0])

    @property
    def quat(self) -> Quaternion:
        """姿态四元数"""
        return Quaternion.from_array(self.x[self.IDX_QUAT])

    @quat.setter
    def quat(self, q: Quaternion):
        self.x[self.IDX_QUAT] = q.q

    @property
    def vel(self) -> np.ndarray:
        """速度 (NED, m/s)"""
        return self.x[self.IDX_VEL]

    @vel.setter
    def vel(self, v: np.ndarray):
        self.x[self.IDX_VEL] = v

    @property
    def pos(self) -> np.ndarray:
        """位置 (NED, m)"""
        return self.x[self.IDX_POS]

    @pos.setter
    def pos(self, p: np.ndarray):
        self.x[self.IDX_POS] = p

    @property
    def gyro_bias(self) -> np.ndarray:
        """陀螺偏置 (rad/s)"""
        return self.x[self.IDX_GYRO_BIAS]

    @gyro_bias.setter
    def gyro_bias(self, b: np.ndarray):
        self.x[self.IDX_GYRO_BIAS] = b

    @property
    def accel_bias(self) -> np.ndarray:
        """加计偏置 (m/s²)"""
        return self.x[self.IDX_ACCEL_BIAS]

    @accel_bias.setter
    def accel_bias(self, b: np.ndarray):
        self.x[self.IDX_ACCEL_BIAS] = b

    @property
    def mag_I(self) -> np.ndarray:
        """NED磁场强度 (Gauss)"""
        return self.x[self.IDX_MAG_I]

    @mag_I.setter
    def mag_I(self, m: np.ndarray):
        self.x[self.IDX_MAG_I] = m

    @property
    def mag_B(self) -> np.ndarray:
        """机体磁场偏置 (Gauss)"""
        return self.x[self.IDX_MAG_B]

    @mag_B.setter
    def mag_B(self, m: np.ndarray):
        self.x[self.IDX_MAG_B] = m

    @property
    def wind(self) -> np.ndarray:
        """水平风速 (N-E, m/s)"""
        return self.x[self.IDX_WIND]

    @wind.setter
    def wind(self, w: np.ndarray):
        self.x[self.IDX_WIND] = w

    def get_rotation_matrix(self) -> np.ndarray:
        """获取旋转矩阵 (机体→NED)"""
        return self.quat.to_rotation_matrix()

    def get_euler_angles(self) -> tuple:
        """获取欧拉角 (roll, pitch, yaw)"""
        return self.quat.to_euler()

    def copy(self) -> 'StateVector':
        """深拷贝状态"""
        new_state = StateVector()
        new_state.x = self.x.copy()
        return new_state


class IMUSample:
    """IMU样本数据"""

    def __init__(self,
                 time_us: int = 0,
                 delta_ang: np.ndarray = None,
                 delta_vel: np.ndarray = None,
                 delta_ang_dt: float = 0.0,
                 delta_vel_dt: float = 0.0,
                 delta_vel_clipping: np.ndarray = None):
        """
        time_us: 时间戳（微秒）
        delta_ang: 角增量（rad）
        delta_vel: 速度增量（m/s）
        delta_ang_dt: 角增量积分时间（s）
        delta_vel_dt: 速度增量积分时间（s）
        delta_vel_clipping: 加速度剪切标志
        """
        self.time_us = time_us
        self.delta_ang = delta_ang if delta_ang is not None else np.zeros(3)
        self.delta_vel = delta_vel if delta_vel is not None else np.zeros(3)
        self.delta_ang_dt = delta_ang_dt
        self.delta_vel_dt = delta_vel_dt
        self.delta_vel_clipping = delta_vel_clipping if delta_vel_clipping is not None else np.zeros(3, dtype=bool)


class GNSSSample:
    """GPS样本数据"""

    def __init__(self,
                 time_us: int = 0,
                 pos: np.ndarray = None,
                 vel: np.ndarray = None,
                 pos_noise: float = 1.0,
                 vel_noise: float = 0.5,
                 nsats: int = 0,
                 pdop: float = 99.0):
        """
        time_us: 时间戳（微秒）
        pos: NED位置（m）
        vel: NED速度（m/s）
        pos_noise: 位置噪声标准差（m）
        vel_noise: 速度噪声标准差（m/s）
        nsats: 卫星数量
        pdop: 位置精度因子
        """
        self.time_us = time_us
        self.pos = pos if pos is not None else np.zeros(3)
        self.vel = vel if vel is not None else np.zeros(3)
        self.pos_noise = pos_noise
        self.vel_noise = vel_noise
        self.nsats = nsats
        self.pdop = pdop


class BaroSample:
    """气压计样本数据"""

    def __init__(self,
                 time_us: int = 0,
                 height: float = 0.0,
                 noise: float = 2.0):
        """
        time_us: 时间戳（微秒）
        height: 高度（m，负值表示海拔）
        noise: 高度噪声标准差（m）
        """
        self.time_us = time_us
        self.height = height
        self.noise = noise


class MagSample:
    """磁力计样本数据"""

    def __init__(self,
                 time_us: int = 0,
                 mag: np.ndarray = None,
                 noise: float = 0.05):
        """
        time_us: 时间戳（微秒）
        mag: 机体坐标系磁场（Gauss）
        noise: 磁场噪声标准差（Gauss）
        """
        self.time_us = time_us
        self.mag = mag if mag is not None else np.zeros(3)
        self.noise = noise


class Parameters:
    """EKF参数配置"""

    def __init__(self):
        # 重力加速度 (m/s²)
        self.gravity = 9.80665

        # IMU噪声参数
        self.gyro_noise = 0.015          # rad/s
        self.accel_noise = 0.35          # m/s²
        self.gyro_bias_noise = 0.001     # rad/s²（随机游走）
        self.accel_bias_noise = 0.01     # m/s³（随机游走）

        # 传感器噪声
        self.gps_pos_noise = 0.5         # m
        self.gps_vel_noise = 0.5         # m/s
        self.baro_noise = 2.0            # m
        self.mag_noise = 0.05            # Gauss

        # 创新门限（标准差倍数）
        self.gps_pos_gate = 5.0
        self.gps_vel_gate = 5.0
        self.baro_gate = 5.0
        self.mag_gate = 3.0

        # 偏置学习抑制阈值
        self.accel_bias_lim = 25.0       # m/s²
        self.gyro_bias_lim = 3.0         # rad/s

        # 初始协方差
        self.init_quat_var = 1e-4        # 姿态初始不确定性
        self.init_vel_var = 1.0          # m/s
        self.init_pos_var = 100.0        # m
        self.init_gyro_bias_var = 1e-3   # rad/s
        self.init_accel_bias_var = 1e-2  # m/s²

        # 重力融合参数
        self.gravity_fusion_accel_min = 0.85  # 0.85g
        self.gravity_fusion_accel_max = 1.15  # 1.15g

        # 地球自转参数（科里奥利修正）
        self.earth_rotation_rate = 7.2921150e-5  # rad/s
        self.enable_coriolis_correction = True   # 科里奥利修正使能
        self.latitude = 0.0  # 纬度（rad）默认赤道
