"""
磁力计融合模块

观测模型：
z_mag = R^T * (mag_I + mag_B)

其中：
- mag_I: NED坐标系地磁场强度
- mag_B: 机体坐标系磁偏置（硬铁干扰）
- R: 旋转矩阵（机体→NED）
"""
import numpy as np
try:
    from ..ekf_core import EKF2Core
    from ..state import MagSample
except ImportError:
    from ekf_core import EKF2Core
    from state import MagSample


class MagFusion:
    """
    磁力计融合

    支持三种模式：
    1. 3D磁场融合：融合三轴磁场分量（精度高，受干扰大）
    2. 航向融合：仅融合偏航角（鲁棒性强）
    3. 自动模式：根据机动强度切换
    """

    def __init__(self, ekf: EKF2Core):
        self.ekf = ekf
        self.params = ekf.params

        # 融合模式
        self.mode = 'auto'  # 'auto', '3d', 'heading', 'none'

        # 地磁场初始化
        self.mag_I_initialized = False

        # 干扰检测
        self.mag_disturbance_detected = False

    def initialize_mag_field(self, mag: MagSample):
        """
        初始化地磁场状态

        使用当前姿态和磁场测量估算NED地磁场
        """
        # 转换到NED坐标系
        R = self.ekf.state.get_rotation_matrix()
        mag_ned = R @ mag.mag

        # 初始化地磁场状态（假设无机体偏置）
        self.ekf.state.mag_I = mag_ned
        self.ekf.state.mag_B = np.zeros(3)

        # 重置协方差
        self.ekf.reset_covariance_submatrix(
            slice(16, 19),  # mag_I
            variance=0.1 ** 2  # 0.1 Gauss
        )
        self.ekf.reset_covariance_submatrix(
            slice(19, 22),  # mag_B
            variance=0.05 ** 2  # 0.05 Gauss
        )

        self.mag_I_initialized = True

    def fuse_3d(self, mag: MagSample) -> bool:
        """
        3D磁场融合（融合三轴分量）

        观测方程：
        z = R^T * (mag_I + mag_B)

        返回：融合是否成功
        """
        if not self.mag_I_initialized:
            self.initialize_mag_field(mag)
            return True

        success_count = 0

        # 预测磁场（机体系）
        R = self.ekf.state.get_rotation_matrix()
        mag_pred = R.T @ (self.ekf.state.mag_I + self.ekf.state.mag_B)

        # 分轴融合
        for axis in range(3):
            # 观测
            z = mag.mag[axis]

            # 创新
            innovation = z - mag_pred[axis]

            # 观测矩阵（简化：需要完整雅可比）
            H = self._compute_mag_jacobian_axis(axis)

            # 观测噪声
            R_obs = mag.noise ** 2

            # 融合
            if self.ekf.update_scalar_observation(innovation, H, R_obs, self.params.mag_gate):
                success_count += 1

        return success_count >= 2

    def fuse_heading(self, mag: MagSample) -> bool:
        """
        航向融合（仅融合偏航角）

        计算磁航向：
        yaw_mag = atan2(mag_E, mag_N)

        返回：融合是否成功
        """
        if not self.mag_I_initialized:
            self.initialize_mag_field(mag)
            return True

        # 预测航向
        R = self.ekf.state.get_rotation_matrix()
        mag_ned_pred = R @ (self.ekf.state.mag_I + self.ekf.state.mag_B)

        # 预测磁航向
        yaw_pred = np.arctan2(mag_ned_pred[1], mag_ned_pred[0])

        # 观测航向
        mag_ned_meas = R @ mag.mag
        yaw_meas = np.arctan2(mag_ned_meas[1], mag_ned_meas[0])

        # 创新（角度差，wrap到±π）
        innovation = yaw_meas - yaw_pred
        innovation = np.arctan2(np.sin(innovation), np.cos(innovation))

        # 观测矩阵（对偏航的敏感度）
        H = np.zeros(24)
        H[2] = 1.0  # 简化：偏航轴

        # 观测噪声
        # 根据俯仰角调整（大俯仰时磁航向不可靠）
        _, pitch, _ = self.ekf.state.get_euler_angles()
        pitch_factor = 1.0 / max(np.cos(pitch), 0.1)  # 俯仰90°时噪声增大10倍
        R_obs = (mag.noise * pitch_factor) ** 2

        # 融合
        return self.ekf.update_scalar_observation(innovation, H, R_obs, self.params.mag_gate)

    def _compute_mag_jacobian_axis(self, axis: int) -> np.ndarray:
        """
        计算磁场观测的雅可比矩阵（简化版）

        完整版应使用符号推导：
        ∂(R^T * mag_I) / ∂q, ∂mag_I, ∂mag_B

        这里简化为常数近似
        """
        H = np.zeros(24)

        # 姿态影响（简化）
        if axis == 0:  # X轴
            H[0] = 0.5  # roll影响
        elif axis == 1:  # Y轴
            H[1] = 0.5  # pitch影响
        else:  # Z轴
            H[2] = 0.5  # yaw影响

        # 地磁场状态影响
        R = self.ekf.state.get_rotation_matrix()
        H[16:19] = R.T[axis, :]  # ∂(R^T * mag_I) / ∂mag_I = R^T[axis]

        # 机体偏置影响
        H[19:22] = R.T[axis, :]  # ∂(R^T * mag_B) / ∂mag_B = R^T[axis]

        return H

    def detect_disturbance(self, mag: MagSample) -> bool:
        """
        检测磁干扰

        方法：
        1. 幅值检查：|mag|偏离标准值
        2. 倾角检查：倾角突变
        3. 创新检查：持续大创新

        返回：是否检测到干扰
        """
        # 预测磁场
        R = self.ekf.state.get_rotation_matrix()
        mag_pred = R.T @ (self.ekf.state.mag_I + self.ekf.state.mag_B)

        # 幅值检查
        mag_norm_meas = np.linalg.norm(mag.mag)
        mag_norm_pred = np.linalg.norm(mag_pred)

        # 正常地磁场：0.25-0.65 Gauss（取决于纬度）
        if mag_norm_meas < 0.2 or mag_norm_meas > 0.8:
            self.mag_disturbance_detected = True
            return True

        # 幅值一致性检查
        mag_norm_diff = abs(mag_norm_meas - mag_norm_pred)
        if mag_norm_diff > 0.2:  # 20% Gauss差异
            self.mag_disturbance_detected = True
            return True

        # 创新检查
        innovation = mag.mag - mag_pred
        innovation_norm = np.linalg.norm(innovation)

        if innovation_norm > 0.3:  # 大创新
            self.mag_disturbance_detected = True
            return True

        # 恢复正常
        self.mag_disturbance_detected = False
        return False

    def control(self, mag: MagSample) -> str:
        """
        控制磁融合模式

        自动模式逻辑：
        - 干扰检测到 → 禁用
        - 高机动 → 航向融合
        - 正常 → 3D融合

        返回：激活的模式
        """
        # 干扰检测
        if self.detect_disturbance(mag):
            return 'none'

        # 高机动检测
        if self.ekf.fault_status.get('high_maneuver', False):
            return 'heading'

        # 根据设定模式
        if self.mode == 'auto':
            return '3d'  # 默认3D
        else:
            return self.mode

    def fuse(self, mag: MagSample) -> bool:
        """
        执行磁融合（自动选择模式）

        返回：融合是否成功
        """
        mode = self.control(mag)

        if mode == 'none':
            return False
        elif mode == '3d':
            return self.fuse_3d(mag)
        elif mode == 'heading':
            return self.fuse_heading(mag)
        else:
            return False

    def reset_field(self):
        """重置地磁场估计"""
        self.mag_I_initialized = False
        self.ekf.state.mag_I = np.zeros(3)
        self.ekf.state.mag_B = np.zeros(3)
