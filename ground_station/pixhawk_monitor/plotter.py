"""
实时数据绘图模块

提供IMU、磁力计、姿态数据的实时可视化
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from typing import Optional
from .utils import quat_to_euler_deg


class RealtimePlotter:
    """实时数据绘图器"""

    def __init__(self, window_size: int = 500, update_interval: int = 50):
        """
        初始化绘图器

        参数:
            window_size: 显示的数据点数量
            update_interval: 更新间隔（毫秒）
        """
        self.window_size = window_size
        self.update_interval = update_interval

        # 数据缓冲区
        self.time_buffer = deque(maxlen=window_size)

        # IMU数据
        self.accel_x = deque(maxlen=window_size)
        self.accel_y = deque(maxlen=window_size)
        self.accel_z = deque(maxlen=window_size)

        self.gyro_x = deque(maxlen=window_size)
        self.gyro_y = deque(maxlen=window_size)
        self.gyro_z = deque(maxlen=window_size)

        # 磁力计数据
        self.mag_x = deque(maxlen=window_size)
        self.mag_y = deque(maxlen=window_size)
        self.mag_z = deque(maxlen=window_size)

        # 姿态数据
        self.roll = deque(maxlen=window_size)
        self.pitch = deque(maxlen=window_size)
        self.yaw = deque(maxlen=window_size)

        self.rate_x = deque(maxlen=window_size)
        self.rate_y = deque(maxlen=window_size)
        self.rate_z = deque(maxlen=window_size)

        # 统计信息
        self.imu_freq = 0.0
        self.att_freq = 0.0
        self.packet_loss = 0.0

        # 起始时间
        self.start_time = time.time()

        # Matplotlib图形对象
        self.fig = None
        self.axes = {}
        self.lines = {}

        # 动画对象
        self.ani = None

    def setup_plots(self):
        """设置绘图窗口"""
        # 创建3x3子图
        self.fig, axes_array = plt.subplots(3, 3, figsize=(16, 10))
        self.fig.canvas.manager.set_window_title('Pixhawk 实时数据监控')

        # 扁平化axes数组以便索引
        axes_flat = axes_array.flatten()

        # 子图1: 加速度计
        ax = axes_flat[0]
        ax.set_title('加速度计 (m/s²)', fontsize=10, weight='bold')
        ax.set_ylabel('加速度')
        ax.grid(True, alpha=0.3)
        self.lines['accel_x'], = ax.plot([], [], 'r-', label='X', linewidth=1.5)
        self.lines['accel_y'], = ax.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.lines['accel_z'], = ax.plot([], [], 'b-', label='Z', linewidth=1.5)
        ax.legend(loc='upper right', fontsize=8)
        self.axes['accel'] = ax

        # 子图2: 陀螺仪
        ax = axes_flat[1]
        ax.set_title('陀螺仪 (rad/s)', fontsize=10, weight='bold')
        ax.set_ylabel('角速度')
        ax.grid(True, alpha=0.3)
        self.lines['gyro_x'], = ax.plot([], [], 'r-', label='X', linewidth=1.5)
        self.lines['gyro_y'], = ax.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.lines['gyro_z'], = ax.plot([], [], 'b-', label='Z', linewidth=1.5)
        ax.legend(loc='upper right', fontsize=8)
        self.axes['gyro'] = ax

        # 子图3: 磁力计
        ax = axes_flat[2]
        ax.set_title('磁力计 (Gauss)', fontsize=10, weight='bold')
        ax.set_ylabel('磁场强度')
        ax.grid(True, alpha=0.3)
        self.lines['mag_x'], = ax.plot([], [], 'r-', label='X', linewidth=1.5)
        self.lines['mag_y'], = ax.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.lines['mag_z'], = ax.plot([], [], 'b-', label='Z', linewidth=1.5)
        ax.legend(loc='upper right', fontsize=8)
        self.axes['mag'] = ax

        # 子图4: Roll角
        ax = axes_flat[3]
        ax.set_title('Roll 滚转角 (度)', fontsize=10, weight='bold')
        ax.set_ylabel('角度 (°)')
        ax.grid(True, alpha=0.3)
        self.lines['roll'], = ax.plot([], [], 'r-', linewidth=2)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.axes['roll'] = ax

        # 子图5: Pitch角
        ax = axes_flat[4]
        ax.set_title('Pitch 俯仰角 (度)', fontsize=10, weight='bold')
        ax.set_ylabel('角度 (°)')
        ax.grid(True, alpha=0.3)
        self.lines['pitch'], = ax.plot([], [], 'g-', linewidth=2)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.axes['pitch'] = ax

        # 子图6: Yaw角
        ax = axes_flat[5]
        ax.set_title('Yaw 偏航角 (度)', fontsize=10, weight='bold')
        ax.set_ylabel('角度 (°)')
        ax.grid(True, alpha=0.3)
        self.lines['yaw'], = ax.plot([], [], 'b-', linewidth=2)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.axes['yaw'] = ax

        # 子图7: 角速度 Roll
        ax = axes_flat[6]
        ax.set_title('角速度 Roll (rad/s)', fontsize=10, weight='bold')
        ax.set_ylabel('角速度')
        ax.set_xlabel('时间 (秒)')
        ax.grid(True, alpha=0.3)
        self.lines['rate_x'], = ax.plot([], [], 'r-', linewidth=1.5)
        self.axes['rate_x'] = ax

        # 子图8: 角速度 Pitch
        ax = axes_flat[7]
        ax.set_title('角速度 Pitch (rad/s)', fontsize=10, weight='bold')
        ax.set_ylabel('角速度')
        ax.set_xlabel('时间 (秒)')
        ax.grid(True, alpha=0.3)
        self.lines['rate_y'], = ax.plot([], [], 'g-', linewidth=1.5)
        self.axes['rate_y'] = ax

        # 子图9: 角速度 Yaw
        ax = axes_flat[8]
        ax.set_title('角速度 Yaw (rad/s)', fontsize=10, weight='bold')
        ax.set_ylabel('角速度')
        ax.set_xlabel('时间 (秒)')
        ax.grid(True, alpha=0.3)
        self.lines['rate_z'], = ax.plot([], [], 'b-', linewidth=1.5)
        self.axes['rate_z'] = ax

        # 调整子图间距
        plt.tight_layout()

        # 添加总标题
        self.fig.suptitle('Pixhawk 传感器与姿态数据实时监控',
                         fontsize=14, weight='bold', y=0.995)

    def update_imu_data(self, data: dict):
        """
        更新IMU数据

        参数:
            data: IMU数据字典 (来自HIGHRES_IMU)
        """
        # 计算相对时间
        rel_time = data['timestamp'] - (self.start_time - time.time())

        self.time_buffer.append(rel_time)

        # 加速度
        self.accel_x.append(data['accel'][0])
        self.accel_y.append(data['accel'][1])
        self.accel_z.append(data['accel'][2])

        # 陀螺仪
        self.gyro_x.append(data['gyro'][0])
        self.gyro_y.append(data['gyro'][1])
        self.gyro_z.append(data['gyro'][2])

        # 磁力计
        self.mag_x.append(data['mag'][0])
        self.mag_y.append(data['mag'][1])
        self.mag_z.append(data['mag'][2])

        # 更新频率
        self.imu_freq = data.get('frequency', 0.0)

    def update_attitude_data(self, data: dict):
        """
        更新姿态数据

        参数:
            data: 姿态数据字典 (来自ATTITUDE_QUATERNION)
        """
        # 四元数转欧拉角
        roll_deg, pitch_deg, yaw_deg = quat_to_euler_deg(data['quaternion'])

        # 计算相对时间
        rel_time = data['timestamp'] - (self.start_time - time.time())

        if not self.time_buffer or abs(rel_time - self.time_buffer[-1]) < 1.0:
            # 如果时间戳合理，使用它
            if len(self.time_buffer) == 0 or rel_time > self.time_buffer[-1]:
                self.time_buffer.append(rel_time)
            else:
                # 时间戳有问题，使用最后一个时间戳+间隔
                rel_time = self.time_buffer[-1] + 0.01 if self.time_buffer else rel_time
                self.time_buffer.append(rel_time)

        # 欧拉角
        self.roll.append(roll_deg)
        self.pitch.append(pitch_deg)
        self.yaw.append(yaw_deg)

        # 角速度
        self.rate_x.append(data['rates'][0])
        self.rate_y.append(data['rates'][1])
        self.rate_z.append(data['rates'][2])

        # 更新频率
        self.att_freq = data.get('frequency', 0.0)

    def _update_plot(self, frame):
        """更新绘图（动画回调函数）"""
        if len(self.time_buffer) == 0:
            return list(self.lines.values())

        # 获取时间数据
        time_data = list(self.time_buffer)

        # 更新加速度计
        if len(self.accel_x) > 0:
            self.lines['accel_x'].set_data(time_data, list(self.accel_x))
            self.lines['accel_y'].set_data(time_data, list(self.accel_y))
            self.lines['accel_z'].set_data(time_data, list(self.accel_z))
            self._autoscale_axis(self.axes['accel'], time_data,
                                [self.accel_x, self.accel_y, self.accel_z])

        # 更新陀螺仪
        if len(self.gyro_x) > 0:
            self.lines['gyro_x'].set_data(time_data, list(self.gyro_x))
            self.lines['gyro_y'].set_data(time_data, list(self.gyro_y))
            self.lines['gyro_z'].set_data(time_data, list(self.gyro_z))
            self._autoscale_axis(self.axes['gyro'], time_data,
                                [self.gyro_x, self.gyro_y, self.gyro_z])

        # 更新磁力计
        if len(self.mag_x) > 0:
            self.lines['mag_x'].set_data(time_data, list(self.mag_x))
            self.lines['mag_y'].set_data(time_data, list(self.mag_y))
            self.lines['mag_z'].set_data(time_data, list(self.mag_z))
            self._autoscale_axis(self.axes['mag'], time_data,
                                [self.mag_x, self.mag_y, self.mag_z])

        # 更新姿态角
        if len(self.roll) > 0:
            self.lines['roll'].set_data(time_data, list(self.roll))
            self._autoscale_axis(self.axes['roll'], time_data, [self.roll])

        if len(self.pitch) > 0:
            self.lines['pitch'].set_data(time_data, list(self.pitch))
            self._autoscale_axis(self.axes['pitch'], time_data, [self.pitch])

        if len(self.yaw) > 0:
            self.lines['yaw'].set_data(time_data, list(self.yaw))
            self._autoscale_axis(self.axes['yaw'], time_data, [self.yaw])

        # 更新角速度
        if len(self.rate_x) > 0:
            self.lines['rate_x'].set_data(time_data, list(self.rate_x))
            self._autoscale_axis(self.axes['rate_x'], time_data, [self.rate_x])

        if len(self.rate_y) > 0:
            self.lines['rate_y'].set_data(time_data, list(self.rate_y))
            self._autoscale_axis(self.axes['rate_y'], time_data, [self.rate_y])

        if len(self.rate_z) > 0:
            self.lines['rate_z'].set_data(time_data, list(self.rate_z))
            self._autoscale_axis(self.axes['rate_z'], time_data, [self.rate_z])

        # 更新总标题显示统计信息
        title = (f'Pixhawk 传感器与姿态数据实时监控 | '
                f'IMU: {self.imu_freq:.1f} Hz | '
                f'姿态: {self.att_freq:.1f} Hz | '
                f'丢包: {self.packet_loss:.1f}%')
        self.fig.suptitle(title, fontsize=14, weight='bold', y=0.995)

        return list(self.lines.values())

    def _autoscale_axis(self, ax, time_data, data_lists):
        """自动缩放坐标轴"""
        if len(time_data) < 2:
            return

        # X轴：显示最近的数据
        ax.set_xlim(time_data[0], time_data[-1])

        # Y轴：自动缩放到数据范围
        all_data = []
        for data_list in data_lists:
            if len(data_list) > 0:
                all_data.extend(list(data_list))

        if len(all_data) > 0:
            y_min = min(all_data)
            y_max = max(all_data)
            y_range = y_max - y_min

            if y_range > 0:
                margin = y_range * 0.1  # 10%边距
                ax.set_ylim(y_min - margin, y_max + margin)
            else:
                # 数据没有变化，设置默认范围
                ax.set_ylim(y_min - 1, y_max + 1)

    def start(self, receiver=None):
        """
        启动实时绘图

        参数:
            receiver: PixhawkReceiver实例（可选，用于自动更新数据）
        """
        self.setup_plots()

        # 如果提供了receiver，设置回调函数
        if receiver:
            receiver.set_highres_imu_callback(self.update_imu_data)
            receiver.set_attitude_callback(self.update_attitude_data)

            # 定期更新统计信息
            def update_stats_callback(data):
                stats = receiver.get_stats()
                self.packet_loss = stats['loss_rate_percent']

            receiver.set_attitude_callback(
                lambda data: (self.update_attitude_data(data), update_stats_callback(data))
            )

        # 创建动画
        self.ani = FuncAnimation(
            self.fig,
            self._update_plot,
            interval=self.update_interval,
            blit=False,
            cache_frame_data=False
        )

        plt.show()

    def stop(self):
        """停止绘图"""
        if self.ani:
            self.ani.event_source.stop()
        plt.close(self.fig)


class SimplePlotter:
    """简化版绘图器（单窗口）"""

    def __init__(self, plot_type='attitude', window_size=500):
        """
        初始化

        参数:
            plot_type: 绘图类型 ('attitude', 'imu', 'mag')
            window_size: 数据窗口大小
        """
        self.plot_type = plot_type
        self.window_size = window_size

        self.time_buffer = deque(maxlen=window_size)
        self.data_buffers = {
            'line1': deque(maxlen=window_size),
            'line2': deque(maxlen=window_size),
            'line3': deque(maxlen=window_size)
        }

        self.start_time = time.time()
        self.fig = None
        self.ax = None
        self.lines = {}
        self.ani = None

    def setup_plot(self):
        """设置绘图窗口"""
        self.fig, self.ax = plt.subplots(figsize=(12, 6))

        if self.plot_type == 'attitude':
            self.ax.set_title('姿态角度 (度)', fontsize=12, weight='bold')
            self.ax.set_ylabel('角度 (°)')
            self.lines['line1'], = self.ax.plot([], [], 'r-', label='Roll', linewidth=2)
            self.lines['line2'], = self.ax.plot([], [], 'g-', label='Pitch', linewidth=2)
            self.lines['line3'], = self.ax.plot([], [], 'b-', label='Yaw', linewidth=2)

        elif self.plot_type == 'imu':
            self.ax.set_title('IMU 加速度 (m/s²)', fontsize=12, weight='bold')
            self.ax.set_ylabel('加速度')
            self.lines['line1'], = self.ax.plot([], [], 'r-', label='X', linewidth=1.5)
            self.lines['line2'], = self.ax.plot([], [], 'g-', label='Y', linewidth=1.5)
            self.lines['line3'], = self.ax.plot([], [], 'b-', label='Z', linewidth=1.5)

        elif self.plot_type == 'mag':
            self.ax.set_title('磁力计 (Gauss)', fontsize=12, weight='bold')
            self.ax.set_ylabel('磁场强度')
            self.lines['line1'], = self.ax.plot([], [], 'r-', label='X', linewidth=1.5)
            self.lines['line2'], = self.ax.plot([], [], 'g-', label='Y', linewidth=1.5)
            self.lines['line3'], = self.ax.plot([], [], 'b-', label='Z', linewidth=1.5)

        self.ax.set_xlabel('时间 (秒)')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper right')
        plt.tight_layout()

    def update_data(self, timestamp, values):
        """
        更新数据

        参数:
            timestamp: 时间戳
            values: [value1, value2, value3]
        """
        rel_time = timestamp - (self.start_time - time.time())
        self.time_buffer.append(rel_time)

        self.data_buffers['line1'].append(values[0])
        self.data_buffers['line2'].append(values[1])
        self.data_buffers['line3'].append(values[2])

    def _update_plot(self, frame):
        """更新绘图"""
        if len(self.time_buffer) == 0:
            return list(self.lines.values())

        time_data = list(self.time_buffer)

        for key in self.data_buffers:
            self.lines[key].set_data(time_data, list(self.data_buffers[key]))

        # 自动缩放
        self.ax.set_xlim(time_data[0], time_data[-1])
        self.ax.relim()
        self.ax.autoscale_view(scalex=False, scaley=True)

        return list(self.lines.values())

    def start(self):
        """启动绘图"""
        self.setup_plot()
        self.ani = FuncAnimation(self.fig, self._update_plot, interval=50, blit=False)
        plt.show()
