"""
PX4 EKF2 Python Implementation

基于PX4-Autopilot的EKF2算法的Python实现
包含完整的24维状态估计、IMU处理、传感器融合功能

主要模块:
- utils: 四元数运算、数值工具
- state: 状态向量定义、传感器数据结构
- imu_processor: IMU数据处理（圆锥补偿）
- ekf_core: EKF核心算法（预测、更新）
- fusion: 传感器融合模块（GPS、气压计、重力）

使用示例:
    from px4_ekf2_python import EKF2, IMUProcessor, Parameters
    from px4_ekf2_python.state import IMUSample

    # 初始化
    params = Parameters()
    ekf = EKF2(params)
    imu_proc = IMUProcessor()

    # 更新循环
    for gyro, accel, dt in imu_data:
        imu_sample = imu_proc.update(gyro, accel, dt)
        if imu_sample:
            ekf.update(imu_sample)

作者: 基于PX4-Autopilot源码分析
许可: BSD-3-Clause
"""

try:
    from .utils import Quaternion, WelfordOnlineVariance
    from .state import (StateVector, IMUSample, GNSSSample, BaroSample,
                       MagSample, Parameters)
    from .imu_processor import IMUProcessor, ConingIntegrator
    from .ekf_core import EKF2Core
    from .fusion import GravityFusion, GPSFusion, BaroFusion, MagFusion
except ImportError:
    from utils import Quaternion, WelfordOnlineVariance
    from state import (StateVector, IMUSample, GNSSSample, BaroSample,
                       MagSample, Parameters)
    from imu_processor import IMUProcessor, ConingIntegrator
    from ekf_core import EKF2Core
    from fusion.gravity import GravityFusion
    from fusion.gps import GPSFusion
    from fusion.baro import BaroFusion
    from fusion.mag import MagFusion

__version__ = '1.0.0'

__all__ = [
    # 核心类
    'EKF2',
    'IMUProcessor',
    'Parameters',

    # 工具类
    'Quaternion',
    'WelfordOnlineVariance',
    'ConingIntegrator',

    # 状态和数据
    'StateVector',
    'IMUSample',
    'GNSSSample',
    'BaroSample',
    'MagSample',

    # 融合模块
    'GravityFusion',
    'GPSFusion',
    'BaroFusion',
    'MagFusion',
]


class EKF2:
    """
    EKF2完整系统（便捷接口）

    集成了核心算法和所有融合模块
    """

    def __init__(self, params: Parameters = None):
        """
        初始化EKF2系统

        参数:
        params: 参数配置（可选，默认使用标准配置）
        """
        self.params = params if params is not None else Parameters()

        # 核心EKF
        self.core = EKF2Core(self.params)

        # 融合模块
        self.gravity_fusion = GravityFusion(self.core)
        self.gps_fusion = GPSFusion(self.core)
        self.baro_fusion = BaroFusion(self.core)
        self.mag_fusion = MagFusion(self.core)

        # 时间管理
        self.time_last_update = 0

    def update(self, imu: IMUSample):
        """
        主更新函数（IMU驱动）

        参数:
        imu: IMU样本数据
        """
        # 1. 状态预测
        self.core.predict_state(imu)

        # 2. 协方差预测
        self.core.predict_covariance(imu)

        # 3. 重力融合控制
        if self.gravity_fusion.control(imu):
            self.gravity_fusion.fuse(imu)

        # 更新时间
        self.time_last_update = imu.time_us

    def update_gps(self, gps: GNSSSample):
        """
        更新GPS观测

        参数:
        gps: GPS样本数据
        """
        was_active = self.gps_fusion.is_active
        pos_ok = self.gps_fusion.fuse_position(gps)
        vel_ok = self.gps_fusion.fuse_velocity(gps)
        if not was_active and (pos_ok or vel_ok):
            self.gps_fusion.reset_position_to_gps(gps)
            self.gps_fusion.reset_velocity_to_gps(gps)

    def update_baro(self, baro: BaroSample):
        """
        更新气压计观测

        参数:
        baro: 气压计样本数据
        """
        self.baro_fusion.fuse(baro)

    def update_mag(self, mag: MagSample):
        """
        更新磁力计观测

        参数:
        mag: 磁力计样本数据
        """
        self.mag_fusion.fuse(mag)

    @property
    def state(self) -> StateVector:
        """获取当前状态估计"""
        return self.core.state

    @property
    def P(self):
        """获取协方差矩阵"""
        return self.core.P

    def get_position(self):
        """获取位置（NED, m）"""
        return self.state.pos

    def get_velocity(self):
        """获取速度（NED, m/s）"""
        return self.state.vel

    def get_euler_angles(self):
        """获取欧拉角（roll, pitch, yaw, rad）"""
        return self.state.get_euler_angles()

    def get_rotation_matrix(self):
        """获取旋转矩阵（机体→NED）"""
        return self.state.get_rotation_matrix()

    def get_position_variance(self):
        """获取位置方差（m²）"""
        return self.core.get_state_variance(slice(6, 9))

    def get_velocity_variance(self):
        """获取速度方差（m²/s²）"""
        return self.core.get_state_variance(slice(3, 6))

    def get_status(self) -> dict:
        """
        获取系统状态

        返回:
        {
            'time_us': 时间戳,
            'position': NED位置,
            'velocity': NED速度,
            'euler': 欧拉角,
            'gravity_fusion_active': 重力融合是否激活,
            'gps_fusion_active': GPS融合是否激活,
            'fault_status': 故障状态
        }
        """
        roll, pitch, yaw = self.get_euler_angles()

        return {
            'time_us': self.time_last_update,
            'position': self.get_position(),
            'velocity': self.get_velocity(),
            'euler': np.degrees([roll, pitch, yaw]),  # 转换为度
            'gravity_fusion_active': self.gravity_fusion.is_active,
            'gps_fusion_active': self.gps_fusion.is_active,
            'fault_status': self.core.fault_status.copy()
        }
