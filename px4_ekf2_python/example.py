"""
PX4 EKF2 Python 使用示例

演示如何使用 px4_ekf2_python 包进行状态估计
"""
import numpy as np
import matplotlib.pyplot as plt
from px4_ekf2_python import EKF2, IMUProcessor, Parameters
from px4_ekf2_python.state import IMUSample, GNSSSample, BaroSample


def simulate_imu_data(duration: float, dt: float = 0.001):
    """
    模拟IMU数据（简单飞行轨迹）

    轨迹: 起飞 → 悬停 → 前进 → 悬停 → 降落

    参数:
    duration: 仿真时长（秒）
    dt: 采样间隔（秒）

    返回:
    (gyro_list, accel_list, time_list)
    """
    g = 9.80665
    time_list = np.arange(0, duration, dt)
    n = len(time_list)

    gyro_list = []
    accel_list = []

    for i, t in enumerate(time_list):
        # 阶段划分
        if t < 2.0:
            # 起飞: 向上加速
            accel_z = -12.0  # 向上加速（NED-Down为负）
            accel_x, accel_y = 0.0, 0.0
            gyro = np.zeros(3)

        elif t < 5.0:
            # 悬停: 仅重力
            accel_x, accel_y, accel_z = 0.0, 0.0, -g
            gyro = np.zeros(3)

        elif t < 8.0:
            # 前进: 俯仰5°，水平加速
            pitch = np.radians(5.0)
            accel_x = -2.0  # 向北加速（NED-North为负）
            accel_y = 0.0
            accel_z = -g * np.cos(pitch)
            gyro = np.array([0, 0.1, 0])  # 缓慢俯仰

        elif t < 12.0:
            # 悬停: 仅重力
            accel_x, accel_y, accel_z = 0.0, 0.0, -g
            gyro = np.zeros(3)

        else:
            # 降落: 向下加速
            accel_z = -8.0  # 减小向上加速度（下降）
            accel_x, accel_y = 0.0, 0.0
            gyro = np.zeros(3)

        # 添加噪声
        gyro_noise = np.random.normal(0, 0.015, 3)
        accel_noise = np.random.normal(0, 0.35, 3)

        gyro_meas = gyro + gyro_noise
        accel_meas = np.array([accel_x, accel_y, accel_z]) + accel_noise

        gyro_list.append(gyro_meas)
        accel_list.append(accel_meas)

    return gyro_list, accel_list, time_list


def simulate_gps_data(duration: float, dt_gps: float = 0.1):
    """
    模拟GPS数据（对应飞行轨迹）

    参数:
    duration: 仿真时长（秒）
    dt_gps: GPS更新间隔（秒）

    返回:
    gps_list
    """
    time_list = np.arange(0, duration, dt_gps)
    gps_list = []

    for i, t in enumerate(time_list):
        # 简化轨迹
        if t < 2.0:
            # 起飞
            pos_n = 0.0
            pos_e = 0.0
            pos_d = -t * 2.0  # 上升（NED-Down为负）
            vel_n, vel_e, vel_d = 0.0, 0.0, -2.0

        elif t < 5.0:
            # 悬停
            pos_n = 0.0
            pos_e = 0.0
            pos_d = -4.0
            vel_n, vel_e, vel_d = 0.0, 0.0, 0.0

        elif t < 8.0:
            # 前进
            dt = t - 5.0
            pos_n = -dt * 2.0  # 向北（NED-North为负）
            pos_e = 0.0
            pos_d = -4.0
            vel_n, vel_e, vel_d = -2.0, 0.0, 0.0

        elif t < 12.0:
            # 悬停
            pos_n = -6.0
            pos_e = 0.0
            pos_d = -4.0
            vel_n, vel_e, vel_d = 0.0, 0.0, 0.0

        else:
            # 降落
            dt = t - 12.0
            pos_n = -6.0
            pos_e = 0.0
            pos_d = -4.0 + dt * 1.0  # 下降（NED-Down增大）
            vel_n, vel_e, vel_d = 0.0, 0.0, 1.0

        # 添加GPS噪声
        pos_noise = np.random.normal(0, 0.5, 3)
        vel_noise = np.random.normal(0, 0.3, 3)

        gps = GNSSSample(
            time_us=int(t * 1e6),
            pos=np.array([pos_n, pos_e, pos_d]) + pos_noise,
            vel=np.array([vel_n, vel_e, vel_d]) + vel_noise,
            pos_noise=0.5,
            vel_noise=0.3,
            nsats=12,
            pdop=2.0
        )
        gps_list.append(gps)

    return gps_list


def main():
    """主函数：运行EKF2仿真"""
    print("=" * 60)
    print("PX4 EKF2 Python 仿真示例")
    print("=" * 60)

    # 1. 初始化
    params = Parameters()
    ekf = EKF2(params)
    imu_proc = IMUProcessor(target_dt=0.004)  # 250Hz IMU输出

    # 2. 生成仿真数据
    print("\n生成仿真数据...")
    duration = 15.0  # 15秒仿真
    gyro_list, accel_list, time_list = simulate_imu_data(duration, dt=0.001)  # 1kHz IMU
    gps_list = simulate_gps_data(duration, dt_gps=0.1)  # 10Hz GPS

    print(f"  IMU采样: {len(gyro_list)} 个样本 (1000Hz)")
    print(f"  GPS采样: {len(gps_list)} 个样本 (10Hz)")

    # 3. 运行EKF
    print("\n运行EKF2...")

    # 存储结果
    ekf_time = []
    ekf_pos = []
    ekf_vel = []
    ekf_euler = []

    gps_idx = 0

    for i, t in enumerate(time_list):
        # IMU更新
        imu_sample = imu_proc.update(
            gyro_list[i],
            accel_list[i],
            0.001  # 1ms采样间隔
        )

        # 当积分周期到达时，更新EKF
        if imu_sample is not None:
            ekf.update(imu_sample)

            # 记录状态
            ekf_time.append(t)
            ekf_pos.append(ekf.get_position().copy())
            ekf_vel.append(ekf.get_velocity().copy())
            ekf_euler.append(np.degrees(ekf.get_euler_angles()))

            # GPS更新（10Hz）
            if gps_idx < len(gps_list) and t >= gps_list[gps_idx].time_us / 1e6:
                ekf.update_gps(gps_list[gps_idx])
                gps_idx += 1

    # 转换为numpy数组
    ekf_time = np.array(ekf_time)
    ekf_pos = np.array(ekf_pos)
    ekf_vel = np.array(ekf_vel)
    ekf_euler = np.array(ekf_euler)

    print(f"  EKF更新: {len(ekf_time)} 次 (250Hz)")

    # 4. 显示最终状态
    print("\n最终状态估计:")
    final_status = ekf.get_status()
    print(f"  位置 (NED): {final_status['position']}")
    print(f"  速度 (NED): {final_status['velocity']}")
    print(f"  姿态 (°): roll={final_status['euler'][0]:.2f}, "
          f"pitch={final_status['euler'][1]:.2f}, yaw={final_status['euler'][2]:.2f}")
    print(f"  重力融合: {'激活' if final_status['gravity_fusion_active'] else '禁用'}")
    print(f"  GPS融合: {'激活' if final_status['gps_fusion_active'] else '禁用'}")

    # 5. 可视化结果
    print("\n生成可视化图表...")
    plot_results(ekf_time, ekf_pos, ekf_vel, ekf_euler, gps_list)

    print("\n仿真完成！")


def plot_results(ekf_time, ekf_pos, ekf_vel, ekf_euler, gps_list):
    """可视化EKF估计结果"""
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('PX4 EKF2 状态估计结果', fontsize=16)

    # GPS真值
    gps_time = [gps.time_us / 1e6 for gps in gps_list]
    gps_pos = np.array([gps.pos for gps in gps_list])

    # 1. 位置 (NED)
    ax = axes[0, 0]
    ax.plot(ekf_time, ekf_pos[:, 0], label='North (EKF)', linewidth=2)
    ax.plot(ekf_time, ekf_pos[:, 1], label='East (EKF)', linewidth=2)
    ax.plot(ekf_time, -ekf_pos[:, 2], label='Altitude (EKF)', linewidth=2)
    ax.plot(gps_time, gps_pos[:, 0], 'o', label='North (GPS)', markersize=3, alpha=0.5)
    ax.plot(gps_time, -gps_pos[:, 2], 's', label='Altitude (GPS)', markersize=3, alpha=0.5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('位置 (m)')
    ax.set_title('位置估计 (NED)')
    ax.legend()
    ax.grid(True)

    # 2. 速度 (NED)
    ax = axes[0, 1]
    ax.plot(ekf_time, ekf_vel[:, 0], label='North', linewidth=2)
    ax.plot(ekf_time, ekf_vel[:, 1], label='East', linewidth=2)
    ax.plot(ekf_time, ekf_vel[:, 2], label='Down', linewidth=2)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('速度 (m/s)')
    ax.set_title('速度估计 (NED)')
    ax.legend()
    ax.grid(True)

    # 3. 欧拉角
    ax = axes[1, 0]
    ax.plot(ekf_time, ekf_euler[:, 0], label='Roll', linewidth=2)
    ax.plot(ekf_time, ekf_euler[:, 1], label='Pitch', linewidth=2)
    ax.plot(ekf_time, ekf_euler[:, 2], label='Yaw', linewidth=2)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('角度 (°)')
    ax.set_title('姿态估计 (欧拉角)')
    ax.legend()
    ax.grid(True)

    # 4. 3D轨迹
    ax = axes[1, 1]
    ax = fig.add_subplot(3, 2, 4, projection='3d')
    ax.plot(ekf_pos[:, 1], ekf_pos[:, 0], -ekf_pos[:, 2], linewidth=2, label='EKF')
    ax.plot(gps_pos[:, 1], gps_pos[:, 0], -gps_pos[:, 2], 'ro',
            markersize=4, alpha=0.5, label='GPS')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('3D轨迹')
    ax.legend()

    # 5. 水平轨迹
    ax = axes[2, 0]
    ax.plot(ekf_pos[:, 1], ekf_pos[:, 0], linewidth=2, label='EKF')
    ax.plot(gps_pos[:, 1], gps_pos[:, 0], 'ro', markersize=4, alpha=0.5, label='GPS')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('水平轨迹 (俯视图)')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')

    # 6. 高度轨迹
    ax = axes[2, 1]
    ax.plot(ekf_time, -ekf_pos[:, 2], linewidth=2, label='EKF高度')
    ax.plot(gps_time, -gps_pos[:, 2], 'ro', markersize=4, alpha=0.5, label='GPS高度')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('高度 (m)')
    ax.set_title('高度时间历程')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig('ekf2_results.png', dpi=150)
    print("  图表已保存至: ekf2_results.png")

    # 显示图表（可选）
    # plt.show()


if __name__ == '__main__':
    main()
