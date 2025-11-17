"""
GPS融合模块
"""
import numpy as np
try:
    from ..ekf_core import EKF2Core
    from ..state import GNSSSample
except ImportError:
    from ekf_core import EKF2Core
    from state import GNSSSample


class GPSFusion:
    """
    GPS位置和速度融合

    观测模型:
    z_pos = p_ned                # GPS位置（NED）
    z_vel = v_ned                # GPS速度（NED）

    创新:
    innov_pos = z_pos - p_state
    innov_vel = z_vel - v_state
    """

    def __init__(self, ekf: EKF2Core):
        self.ekf = ekf
        self.params = ekf.params

        # 质量检查
        self.min_sats = 6
        self.max_pdop = 5.0

        # 融合状态
        self.is_active = False

    def quality_check(self, gps: GNSSSample) -> bool:
        """
        GPS质量检查

        检查项:
        1. 卫星数 >= 6
        2. PDOP <= 5.0
        3. 位置/速度噪声合理

        返回: 是否通过质量检查
        """
        check_sats = gps.nsats >= self.min_sats
        check_pdop = gps.pdop <= self.max_pdop
        check_noise = gps.pos_noise < 10.0 and gps.vel_noise < 5.0

        return check_sats and check_pdop and check_noise

    def fuse_position(self, gps: GNSSSample) -> bool:
        """
        融合GPS位置（NED三个轴分别融合）

        返回: 融合是否成功
        """
        if not self.quality_check(gps):
            return False

        success_count = 0

        # 分别融合 N, E, D
        for axis in range(3):
            # 观测
            z = gps.pos[axis]

            # 预测
            p_predicted = self.ekf.state.pos[axis]

            # 创新
            innovation = z - p_predicted

            # 观测矩阵（仅影响对应位置轴）
            H = np.zeros(24)
            H[6 + axis] = 1.0  # 位置状态索引: 6-8

            # 观测噪声
            R = gps.pos_noise ** 2

            # 融合
            if self.ekf.update_scalar_observation(innovation, H, R, self.params.gps_pos_gate):
                success_count += 1

        self.is_active = success_count > 0
        self.ekf.fault_status['gps_lost'] = not self.is_active

        return success_count >= 2  # 至少2个轴成功

    def fuse_velocity(self, gps: GNSSSample) -> bool:
        """
        融合GPS速度（NED三个轴分别融合）

        返回: 融合是否成功
        """
        if not self.quality_check(gps):
            return False

        success_count = 0

        # 分别融合 N, E, D
        for axis in range(3):
            # 观测
            z = gps.vel[axis]

            # 预测
            v_predicted = self.ekf.state.vel[axis]

            # 创新
            innovation = z - v_predicted

            # 观测矩阵（仅影响对应速度轴）
            H = np.zeros(24)
            H[3 + axis] = 1.0  # 速度状态索引: 3-5

            # 观测噪声
            R = gps.vel_noise ** 2

            # 融合
            if self.ekf.update_scalar_observation(innovation, H, R, self.params.gps_vel_gate):
                success_count += 1

        return success_count >= 2  # 至少2个轴成功

    def reset_position_to_gps(self, gps: GNSSSample):
        """
        将EKF位置重置为GPS位置

        用于初始化或GPS重新捕获
        """
        self.ekf.state.pos = gps.pos.copy()

        # 重置位置协方差
        self.ekf.reset_covariance_submatrix(
            slice(6, 9),
            gps.pos_noise ** 2
        )

    def reset_velocity_to_gps(self, gps: GNSSSample):
        """
        将EKF速度重置为GPS速度
        """
        self.ekf.state.vel = gps.vel.copy()

        # 重置速度协方差
        self.ekf.reset_covariance_submatrix(
            slice(3, 6),
            gps.vel_noise ** 2
        )
