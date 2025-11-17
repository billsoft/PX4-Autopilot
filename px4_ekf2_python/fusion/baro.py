"""
气压计融合模块
"""
import numpy as np
try:
    from ..ekf_core import EKF2Core
    from ..state import BaroSample
except ImportError:
    from ekf_core import EKF2Core
    from state import BaroSample


class BaroFusion:
    """
    气压计高度融合

    观测模型:
    z_baro = -p_down + bias_baro  # 负号：NED-Down为正，海拔为负

    创新:
    innov = z_baro - (-p_down_state + bias_baro_state)
    """

    def __init__(self, ekf: EKF2Core):
        self.ekf = ekf
        self.params = ekf.params

        # 气压偏置（地效、温漂等）
        self.baro_bias = 0.0
        self.baro_bias_var = 25.0  # 初始偏置方差 (5m标准差)

        # 地效补偿
        self.ground_effect_compensation = 0.0

    def fuse(self, baro: BaroSample) -> bool:
        """
        融合气压计高度

        返回: 融合是否成功
        """
        # 观测: 气压高度
        z = baro.height

        # 预测: EKF高度 + 气压偏置
        # NED坐标系: Down为正，所以高度 = -pos_down
        height_predicted = -self.ekf.state.pos[2] + self.baro_bias

        # 创新
        innovation = z - height_predicted

        # 观测矩阵
        # ∂h/∂p_down = -1, ∂h/∂bias = 1
        H = np.zeros(24)
        H[8] = -1.0  # pos_down索引: 8

        # 观测噪声（包含气压噪声 + 偏置不确定性）
        R = baro.noise ** 2 + self.baro_bias_var

        # 融合
        success = self.ekf.update_scalar_observation(
            innovation, H, R, self.params.baro_gate
        )

        # 更新偏置（简化版：使用创新的一部分）
        if success:
            self._update_bias(innovation)

        return success

    def _update_bias(self, innovation: float):
        """
        更新气压偏置

        使用一阶低通滤波
        """
        # 学习率（小值 → 慢学习）
        learning_rate = 0.01

        # 偏置更新
        self.baro_bias += learning_rate * innovation

        # 限制偏置范围（±50m）
        self.baro_bias = np.clip(self.baro_bias, -50.0, 50.0)

        # 收敛时降低不确定性
        self.baro_bias_var *= 0.99
        self.baro_bias_var = max(self.baro_bias_var, 1.0)

    def set_ground_level(self, height: float):
        """
        设置地面高度（用于初始化）

        参数:
        height: 地面气压高度（m）
        """
        # 初始化EKF高度（NED-Down）
        self.ekf.state.pos[2] = -height

        # 重置偏置
        self.baro_bias = 0.0
        self.baro_bias_var = 25.0

        # 重置高度协方差
        self.ekf.P[8, 8] = 4.0  # 2m标准差

    def compensate_ground_effect(self, thrust: float, height_agl: float):
        """
        地效补偿

        地效: 旋翼下洗气流导致气压异常

        参数:
        thrust: 油门（0-1）
        height_agl: 离地高度（m）
        """
        # 地效显著高度: < 5m
        if height_agl < 5.0:
            # 补偿量与油门和高度相关
            # compensation = k * thrust / height
            k = 0.5  # 经验系数
            self.ground_effect_compensation = k * thrust / (height_agl + 0.1)
        else:
            self.ground_effect_compensation = 0.0

    def get_bias(self) -> float:
        """获取当前气压偏置估计"""
        return self.baro_bias

    def reset_bias(self):
        """重置气压偏置"""
        self.baro_bias = 0.0
        self.baro_bias_var = 25.0
