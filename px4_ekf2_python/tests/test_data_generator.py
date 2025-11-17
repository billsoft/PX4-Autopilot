"""
测试数据生成器
生成模拟飞行场景的传感器数据
"""
import numpy as np
import sys
import os

# 添加父目录到路径
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)

# 绝对导入
import state as state_module
IMUSample = state_module.IMUSample
GNSSSample = state_module.GNSSSample
BaroSample = state_module.BaroSample
MagSample = state_module.MagSample


class FlightTrajectory:
    """飞行轨迹生成器"""

    def __init__(self, duration: float, dt: float = 0.001):
        """
        duration: 仿真时长（秒）
        dt: IMU采样间隔（秒）
        """
        self.duration = duration
        self.dt = dt
        self.time = np.arange(0, duration, dt)
        self.n_samples = len(self.time)

        # 物理常数
        self.g = 9.80665  # m/s²

        # 真值轨迹
        self.true_attitude = None  # [roll, pitch, yaw] (rad)
        self.true_velocity = None  # NED (m/s)
        self.true_position = None  # NED (m)
        self.true_accel_body = None  # 机体系加速度 (m/s²)
        self.true_gyro_body = None  # 机体系角速度 (rad/s)

    def generate_hover_scenario(self):
        """
        场景1: 悬停
        - 起飞 → 悬停 → 降落
        """
        print("生成悬停场景...")

        # 初始化数组
        self.true_position = np.zeros((self.n_samples, 3))
        self.true_velocity = np.zeros((self.n_samples, 3))
        self.true_attitude = np.zeros((self.n_samples, 3))
        self.true_accel_body = np.zeros((self.n_samples, 3))
        self.true_gyro_body = np.zeros((self.n_samples, 3))

        for i, t in enumerate(self.time):
            if t < 2.0:
                # 起飞: 2 m/s向上
                self.true_velocity[i, 2] = -2.0
                self.true_accel_body[i, 2] = -(self.g + 2.0)  # 向上加速

            elif t < 5.0:
                # 悬停
                self.true_velocity[i, :] = 0
                self.true_accel_body[i, 2] = -self.g

            elif t < 7.0:
                # 降落
                self.true_velocity[i, 2] = 1.0
                self.true_accel_body[i, 2] = -(self.g - 1.0)

            else:
                # 着陆
                self.true_velocity[i, :] = 0
                self.true_accel_body[i, 2] = -self.g

            # 积分位置
            if i > 0:
                self.true_position[i] = self.true_position[i-1] + \
                    0.5 * (self.true_velocity[i-1] + self.true_velocity[i]) * self.dt

            # 姿态保持水平
            self.true_attitude[i] = [0, 0, 0]

    def generate_forward_flight_scenario(self):
        """
        场景2: 前飞
        - 起飞 → 悬停 → 加速前飞 → 减速 → 悬停 → 降落
        """
        print("生成前飞场景...")

        # 初始化数组
        self.true_position = np.zeros((self.n_samples, 3))
        self.true_velocity = np.zeros((self.n_samples, 3))
        self.true_attitude = np.zeros((self.n_samples, 3))
        self.true_accel_body = np.zeros((self.n_samples, 3))
        self.true_gyro_body = np.zeros((self.n_samples, 3))

        for i, t in enumerate(self.time):
            if t < 2.0:
                # 起飞
                self.true_velocity[i, 2] = -2.0
                self.true_accel_body[i, 2] = -(self.g + 2.0)

            elif t < 4.0:
                # 悬停
                self.true_velocity[i, :] = [0, 0, 0]
                self.true_accel_body[i, 2] = -self.g

            elif t < 7.0:
                # 加速前飞（北向）
                accel_north = 3.0  # m/s²
                dt_accel = t - 4.0
                self.true_velocity[i, 0] = -accel_north * dt_accel
                self.true_accel_body[i, 0] = -accel_north  # 机体X轴前向
                self.true_accel_body[i, 2] = -self.g
                # 俯仰5°
                self.true_attitude[i, 1] = -0.087  # -5° pitch

            elif t < 10.0:
                # 匀速
                self.true_velocity[i, 0] = -9.0
                self.true_accel_body[i, 2] = -self.g
                self.true_attitude[i, 1] = 0

            elif t < 13.0:
                # 减速
                accel_north = -3.0
                dt_decel = t - 10.0
                self.true_velocity[i, 0] = -9.0 + accel_north * dt_decel
                self.true_accel_body[i, 0] = accel_north
                self.true_accel_body[i, 2] = -self.g
                self.true_attitude[i, 1] = 0.087  # +5° pitch (减速)

            elif t < 15.0:
                # 悬停
                self.true_velocity[i, :] = 0
                self.true_accel_body[i, 2] = -self.g
                self.true_attitude[i, 1] = 0

            else:
                # 降落
                self.true_velocity[i, 2] = 1.0
                self.true_accel_body[i, 2] = -(self.g - 1.0)

            # 积分位置
            if i > 0:
                self.true_position[i] = self.true_position[i-1] + \
                    0.5 * (self.true_velocity[i-1] + self.true_velocity[i]) * self.dt

    def generate_imu_data(self, gyro_noise_std: float = 0.015,
                         accel_noise_std: float = 0.35,
                         gyro_bias: np.ndarray = None,
                         accel_bias: np.ndarray = None):
        """
        根据真值生成IMU测量数据

        返回: imu_data列表
        """
        if self.true_accel_body is None:
            raise ValueError("请先生成轨迹（调用generate_*_scenario）")

        print("生成IMU测量数据...")

        # 默认偏置
        if gyro_bias is None:
            gyro_bias = np.array([0.01, -0.01, 0.005])  # rad/s
        if accel_bias is None:
            accel_bias = np.array([0.1, -0.1, 0.05])  # m/s²

        imu_data = []

        for i in range(self.n_samples):
            # 真值 + 偏置 + 噪声
            gyro_true = self.true_gyro_body[i]
            accel_true = self.true_accel_body[i]

            gyro_meas = gyro_true + gyro_bias + \
                np.random.normal(0, gyro_noise_std, 3)
            accel_meas = accel_true + accel_bias + \
                np.random.normal(0, accel_noise_std, 3)

            # 创建IMU样本（未积分的瞬时值）
            imu_data.append({
                'time': self.time[i],
                'gyro': gyro_meas,
                'accel': accel_meas,
                'gyro_true': gyro_true,
                'accel_true': accel_true
            })

        return imu_data

    def generate_gps_data(self, gps_rate: float = 10.0,
                         pos_noise_std: float = 0.5,
                         vel_noise_std: float = 0.3):
        """
        生成GPS数据

        gps_rate: GPS更新频率（Hz）
        """
        print("生成GPS测量数据...")

        gps_dt = 1.0 / gps_rate
        gps_times = np.arange(0, self.duration, gps_dt)

        gps_data = []

        for t in gps_times:
            # 找到最近的真值索引
            idx = int(t / self.dt)
            if idx >= self.n_samples:
                idx = self.n_samples - 1

            # 真值 + 噪声
            pos_meas = self.true_position[idx] + \
                np.random.normal(0, pos_noise_std, 3)
            vel_meas = self.true_velocity[idx] + \
                np.random.normal(0, vel_noise_std, 3)

            gps = GNSSSample(
                time_us=int(t * 1e6),
                pos=pos_meas,
                vel=vel_meas,
                pos_noise=pos_noise_std,
                vel_noise=vel_noise_std,
                nsats=12,
                pdop=2.0
            )

            gps_data.append(gps)

        return gps_data

    def generate_baro_data(self, baro_rate: float = 50.0,
                          baro_noise_std: float = 0.5):
        """
        生成气压计数据
        """
        print("生成气压计测量数据...")

        baro_dt = 1.0 / baro_rate
        baro_times = np.arange(0, self.duration, baro_dt)

        baro_data = []

        for t in baro_times:
            idx = int(t / self.dt)
            if idx >= self.n_samples:
                idx = self.n_samples - 1

            # 高度 = -pos_down
            height_true = -self.true_position[idx, 2]
            height_meas = height_true + np.random.normal(0, baro_noise_std)

            baro = BaroSample(
                time_us=int(t * 1e6),
                height=height_meas,
                noise=baro_noise_std
            )

            baro_data.append(baro)

        return baro_data

    def generate_mag_data(self, mag_rate: float = 50.0,
                         mag_noise_std: float = 0.05,
                         mag_declination: float = 0.1):
        """
        生成磁力计数据

        mag_declination: 磁偏角（rad）
        """
        print("生成磁力计测量数据...")

        mag_dt = 1.0 / mag_rate
        mag_times = np.arange(0, self.duration, mag_dt)

        # 地磁场强度（NED坐标系）
        # 假设北半球中纬度: 北向0.3, 东向0.0, 下向0.5 Gauss
        mag_I = np.array([0.3, 0.0, 0.5])

        mag_data = []

        for t in mag_times:
            idx = int(t / self.dt)
            if idx >= self.n_samples:
                idx = self.n_samples - 1

            # 获取当前姿态
            roll, pitch, yaw = self.true_attitude[idx]

            # 构造旋转矩阵（NED → Body）
            # 简化：仅考虑偏航角
            R_ned_to_body = np.array([
                [np.cos(yaw), np.sin(yaw), 0],
                [-np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1]
            ])

            # 磁场从NED转到机体系
            mag_body_true = R_ned_to_body @ mag_I

            # 添加噪声
            mag_meas = mag_body_true + np.random.normal(0, mag_noise_std, 3)

            mag = MagSample(
                time_us=int(t * 1e6),
                mag=mag_meas,
                noise=mag_noise_std
            )

            mag_data.append(mag)

        return mag_data


def generate_test_dataset(scenario: str = 'forward_flight',
                         duration: float = 20.0,
                         save_path: str = None):
    """
    生成完整测试数据集

    scenario: 'hover' 或 'forward_flight'
    duration: 仿真时长（秒）
    save_path: 保存路径（可选）

    返回: dict包含所有传感器数据
    """
    traj = FlightTrajectory(duration=duration, dt=0.001)

    # 生成轨迹
    if scenario == 'hover':
        traj.generate_hover_scenario()
    elif scenario == 'forward_flight':
        traj.generate_forward_flight_scenario()
    else:
        raise ValueError(f"未知场景: {scenario}")

    # 生成传感器数据
    imu_data = traj.generate_imu_data()
    gps_data = traj.generate_gps_data()
    baro_data = traj.generate_baro_data()
    mag_data = traj.generate_mag_data()

    dataset = {
        'scenario': scenario,
        'duration': duration,
        'dt': traj.dt,
        'time': traj.time,
        'imu': imu_data,
        'gps': gps_data,
        'baro': baro_data,
        'mag': mag_data,
        'ground_truth': {
            'position': traj.true_position,
            'velocity': traj.true_velocity,
            'attitude': traj.true_attitude,
            'dt': traj.dt
        }
    }

    # 保存到文件
    if save_path:
        import pickle
        with open(save_path, 'wb') as f:
            pickle.dump(dataset, f)
        print(f"数据集已保存至: {save_path}")

    return dataset


if __name__ == '__main__':
    # 生成测试数据集
    print("="*60)
    print("生成PX4 EKF2测试数据集")
    print("="*60)

    # 场景1: 悬停
    dataset_hover = generate_test_dataset(
        scenario='hover',
        duration=10.0,
        save_path='tests/data/hover_scenario.pkl'
    )
    print(f"悬停场景: {len(dataset_hover['imu'])} IMU样本, "
          f"{len(dataset_hover['gps'])} GPS样本")

    # 场景2: 前飞
    dataset_forward = generate_test_dataset(
        scenario='forward_flight',
        duration=20.0,
        save_path='tests/data/forward_flight_scenario.pkl'
    )
    print(f"前飞场景: {len(dataset_forward['imu'])} IMU样本, "
          f"{len(dataset_forward['gps'])} GPS样本")

    print("\n数据生成完成！")
