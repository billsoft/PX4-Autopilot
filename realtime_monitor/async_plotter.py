"""
异步实时绘图器模块

使用matplotlib绘制实时示波器式曲线

关键技术:
- matplotlib非阻塞模式 (plt.ion())
- asyncio定期更新
- 双缓冲避免闪烁
"""

import asyncio
import logging
import time
from typing import Optional

import matplotlib
matplotlib.use('TkAgg')  # 使用TkAgg后端（支持异步更新）

import matplotlib.pyplot as plt
import numpy as np

from .data_buffer import AttitudeBuffer, DataStatistics
from .config import get_config


logger = logging.getLogger(__name__)


class AsyncOscilloscopePlotter:
    """
    异步示波器式绘图器

    显示:
    - 四元数原始值（4条曲线）
    - 欧拉角（3条曲线）
    - 角速度（3条曲线）
    """

    def __init__(self, buffer: AttitudeBuffer):
        """
        初始化绘图器

        参数:
            buffer: AttitudeBuffer数据缓冲区
        """
        self.buffer = buffer
        plot_cfg = get_config('plot')

        # 配置参数
        self.window_size = plot_cfg.window_size
        self.update_interval = plot_cfg.update_interval
        self.figure_size = plot_cfg.figure_size

        # Matplotlib对象
        self.fig = None
        self.axes = {}
        self.lines = {}

        # 运行状态
        self._running = False
        self._update_task: Optional[asyncio.Task] = None

        # 性能统计
        self.update_count = 0
        self.last_update_time = time.time()
        self.actual_fps = 0.0

        # 初始化图形
        self._setup_figure()

    def _setup_figure(self):
        """设置图形窗口"""
        plot_cfg = get_config('plot')

        # 开启交互模式
        plt.ion()

        # 创建图形窗口（2x2布局）
        self.fig, axes_array = plt.subplots(2, 2, figsize=self.figure_size)
        self.fig.canvas.manager.set_window_title('Pixhawk 实时姿态监控 (异步协程版)')

        # 扁平化axes
        axes_flat = axes_array.flatten()

        # === 子图1: 四元数 ===
        ax = axes_flat[0]
        ax.set_title('四元数 Quaternion [w, x, y, z]', fontsize=12, weight='bold')
        ax.set_ylabel('值')
        ax.set_xlabel('时间 (秒)')
        ax.grid(True, alpha=plot_cfg.grid_alpha)

        self.lines['q_w'], = ax.plot([], [], color='purple', label='w', linewidth=plot_cfg.line_width)
        self.lines['q_x'], = ax.plot([], [], color='red', label='x', linewidth=plot_cfg.line_width)
        self.lines['q_y'], = ax.plot([], [], color='green', label='y', linewidth=plot_cfg.line_width)
        self.lines['q_z'], = ax.plot([], [], color='blue', label='z', linewidth=plot_cfg.line_width)
        ax.legend(loc='upper right', fontsize=9)
        ax.set_ylim(-1.1, 1.1)  # 四元数归一化，范围[-1, 1]
        self.axes['quaternion'] = ax

        # === 子图2: 欧拉角（度） ===
        ax = axes_flat[1]
        ax.set_title('欧拉角 Euler Angles (度)', fontsize=12, weight='bold')
        ax.set_ylabel('角度 (°)')
        ax.set_xlabel('时间 (秒)')
        ax.grid(True, alpha=plot_cfg.grid_alpha)

        self.lines['roll'], = ax.plot([], [], color=plot_cfg.color_roll,
                                       label='Roll (滚转)', linewidth=plot_cfg.line_width)
        self.lines['pitch'], = ax.plot([], [], color=plot_cfg.color_pitch,
                                        label='Pitch (俯仰)', linewidth=plot_cfg.line_width)
        self.lines['yaw'], = ax.plot([], [], color=plot_cfg.color_yaw,
                                      label='Yaw (偏航)', linewidth=plot_cfg.line_width)
        ax.legend(loc='upper right', fontsize=9)
        ax.axhline(y=0, color='black', linestyle='--', alpha=0.3, linewidth=0.8)
        self.axes['euler'] = ax

        # === 子图3: 角速度 ===
        ax = axes_flat[2]
        ax.set_title('角速度 Angular Rates (rad/s)', fontsize=12, weight='bold')
        ax.set_ylabel('角速度 (rad/s)')
        ax.set_xlabel('时间 (秒)')
        ax.grid(True, alpha=plot_cfg.grid_alpha)

        self.lines['roll_rate'], = ax.plot([], [], color=plot_cfg.color_roll,
                                           label='Roll Rate', linewidth=plot_cfg.line_width)
        self.lines['pitch_rate'], = ax.plot([], [], color=plot_cfg.color_pitch,
                                            label='Pitch Rate', linewidth=plot_cfg.line_width)
        self.lines['yaw_rate'], = ax.plot([], [], color=plot_cfg.color_yaw,
                                          label='Yaw Rate', linewidth=plot_cfg.line_width)
        ax.legend(loc='upper right', fontsize=9)
        ax.axhline(y=0, color='black', linestyle='--', alpha=0.3, linewidth=0.8)
        self.axes['rates'] = ax

        # === 子图4: 统计信息 ===
        ax = axes_flat[3]
        ax.set_title('统计信息 Statistics', fontsize=12, weight='bold')
        ax.axis('off')  # 关闭坐标轴
        self.stats_text = ax.text(0.1, 0.5, '', fontsize=10, family='monospace',
                                  verticalalignment='center')
        self.axes['stats'] = ax

        # 调整布局
        plt.tight_layout()

        # 显示窗口（非阻塞）
        plt.show(block=False)
        plt.pause(0.001)

    async def start(self):
        """启动异步绘图"""
        if self._running:
            logger.warning("绘图器已在运行")
            return

        self._running = True
        self._update_task = asyncio.create_task(self._update_loop())
        logger.info("📊 异步绘图已启动")

    async def stop(self):
        """停止异步绘图"""
        if not self._running:
            return

        self._running = False

        if self._update_task:
            self._update_task.cancel()
            try:
                await self._update_task
            except asyncio.CancelledError:
                pass

        plt.ioff()
        plt.close(self.fig)
        logger.info("📊 异步绘图已停止")

    async def _update_loop(self):
        """
        异步更新循环

        关键点:
        - 定期从buffer获取数据
        - 更新图形
        - plt.pause()让出控制权
        - 计算实际FPS
        """
        logger.info("开始绘图更新循环...")

        try:
            while self._running:
                start_time = time.time()

                # 更新图形
                self._update_plot()

                # 计算FPS
                self.update_count += 1
                elapsed = time.time() - self.last_update_time
                if elapsed >= 1.0:
                    self.actual_fps = self.update_count / elapsed
                    self.update_count = 0
                    self.last_update_time = time.time()

                # 让出控制权（关键！）
                # plt.pause会处理GUI事件并让出控制权给asyncio
                plt.pause(0.001)

                # 计算需要等待的时间
                update_time = time.time() - start_time
                wait_time = max(0, self.update_interval - update_time)

                if wait_time > 0:
                    await asyncio.sleep(wait_time)
                else:
                    await asyncio.sleep(0)  # 让出控制权

        except asyncio.CancelledError:
            logger.info("绘图更新循环已取消")
            raise
        except Exception as e:
            logger.error(f"绘图更新错误: {e}", exc_info=True)
            raise

    def _update_plot(self):
        """
        更新图形（同步方法）

        注意：这个方法在asyncio上下文中调用，但本身是同步的
        因为matplotlib不是协程安全的
        """
        try:
            # 从buffer获取数据
            data = self.buffer.get_data_for_plot(window_size=self.window_size)

            if len(data['timestamps']) == 0:
                return

            # 转换为numpy数组（性能优化）
            timestamps = np.array(data['timestamps'])

            # === 更新四元数 ===
            if data['quaternions']:
                quaternions = np.array(data['quaternions'])
                self.lines['q_w'].set_data(timestamps, quaternions[:, 0])
                self.lines['q_x'].set_data(timestamps, quaternions[:, 1])
                self.lines['q_y'].set_data(timestamps, quaternions[:, 2])
                self.lines['q_z'].set_data(timestamps, quaternions[:, 3])

                # 自动调整X轴
                self.axes['quaternion'].set_xlim(timestamps[0], timestamps[-1])

            # === 更新欧拉角 ===
            roll_deg = np.array(data['roll_deg'])
            pitch_deg = np.array(data['pitch_deg'])
            yaw_deg = np.array(data['yaw_deg'])

            self.lines['roll'].set_data(timestamps, roll_deg)
            self.lines['pitch'].set_data(timestamps, pitch_deg)
            self.lines['yaw'].set_data(timestamps, yaw_deg)

            # 自动调整Y轴
            all_angles = np.concatenate([roll_deg, pitch_deg, yaw_deg])
            if len(all_angles) > 0:
                y_min, y_max = all_angles.min(), all_angles.max()
                y_range = y_max - y_min
                margin = max(10, y_range * 0.1)  # 至少10度边距
                self.axes['euler'].set_ylim(y_min - margin, y_max + margin)

            self.axes['euler'].set_xlim(timestamps[0], timestamps[-1])

            # === 更新角速度 ===
            roll_rate = np.array(data['roll_rate'])
            pitch_rate = np.array(data['pitch_rate'])
            yaw_rate = np.array(data['yaw_rate'])

            self.lines['roll_rate'].set_data(timestamps, roll_rate)
            self.lines['pitch_rate'].set_data(timestamps, pitch_rate)
            self.lines['yaw_rate'].set_data(timestamps, yaw_rate)

            # 自动调整Y轴
            all_rates = np.concatenate([roll_rate, pitch_rate, yaw_rate])
            if len(all_rates) > 0:
                y_min, y_max = all_rates.min(), all_rates.max()
                y_range = y_max - y_min
                margin = max(0.1, y_range * 0.1)
                self.axes['rates'].set_ylim(y_min - margin, y_max + margin)

            self.axes['rates'].set_xlim(timestamps[0], timestamps[-1])

            # === 更新统计信息 ===
            stats = self.buffer.get_statistics()
            latest = self.buffer.get_latest()

            stats_text = f"""
数据统计:
━━━━━━━━━━━━━━━━━━━━━
数据包总数: {stats.total_packets}
丢包数量:   {stats.lost_packets}
丢包率:     {stats.loss_rate:.2f}%
平均频率:   {stats.avg_frequency:.1f} Hz
绘图FPS:    {self.actual_fps:.1f}

当前姿态:
━━━━━━━━━━━━━━━━━━━━━
Roll:  {roll_deg[-1]:7.2f}°
Pitch: {pitch_deg[-1]:7.2f}°
Yaw:   {yaw_deg[-1]:7.2f}°
"""
            self.stats_text.set_text(stats_text.strip())

            # 刷新画布（非阻塞）
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

        except Exception as e:
            logger.error(f"更新图形时出错: {e}")

    def is_running(self) -> bool:
        """返回是否正在运行"""
        return self._running


# ========== 简化版绘图器 ==========

class SimpleAnglePlotter:
    """
    简化版姿态角绘图器

    只显示3个欧拉角，更简洁
    """

    def __init__(self, buffer: AttitudeBuffer):
        """初始化"""
        self.buffer = buffer
        plot_cfg = get_config('plot')

        self.window_size = plot_cfg.window_size
        self.update_interval = plot_cfg.update_interval

        plt.ion()

        # 创建单个图形
        self.fig, self.ax = plt.subplots(figsize=(14, 6))
        self.fig.canvas.manager.set_window_title('姿态角实时监控 (简化版)')

        self.ax.set_title('欧拉角 Euler Angles', fontsize=14, weight='bold')
        self.ax.set_ylabel('角度 (°)', fontsize=12)
        self.ax.set_xlabel('时间 (秒)', fontsize=12)
        self.ax.grid(True, alpha=0.3)

        # 创建曲线
        self.line_roll, = self.ax.plot([], [], 'r-', label='Roll', linewidth=2)
        self.line_pitch, = self.ax.plot([], [], 'g-', label='Pitch', linewidth=2)
        self.line_yaw, = self.ax.plot([], [], 'b-', label='Yaw', linewidth=2)

        self.ax.legend(loc='upper right', fontsize=11)
        self.ax.axhline(y=0, color='black', linestyle='--', alpha=0.3)

        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.001)

        self._running = False
        self._update_task = None

    async def start(self):
        """启动"""
        self._running = True
        self._update_task = asyncio.create_task(self._update_loop())

    async def stop(self):
        """停止"""
        self._running = False
        if self._update_task:
            self._update_task.cancel()
            try:
                await self._update_task
            except asyncio.CancelledError:
                pass
        plt.ioff()
        plt.close(self.fig)

    async def _update_loop(self):
        """更新循环"""
        while self._running:
            try:
                start_time = time.time()

                data = self.buffer.get_data_for_plot(self.window_size)

                if len(data['timestamps']) > 0:
                    timestamps = np.array(data['timestamps'])
                    roll = np.array(data['roll_deg'])
                    pitch = np.array(data['pitch_deg'])
                    yaw = np.array(data['yaw_deg'])

                    self.line_roll.set_data(timestamps, roll)
                    self.line_pitch.set_data(timestamps, pitch)
                    self.line_yaw.set_data(timestamps, yaw)

                    self.ax.set_xlim(timestamps[0], timestamps[-1])

                    all_angles = np.concatenate([roll, pitch, yaw])
                    y_min, y_max = all_angles.min(), all_angles.max()
                    margin = max(10, (y_max - y_min) * 0.1)
                    self.ax.set_ylim(y_min - margin, y_max + margin)

                    self.fig.canvas.draw_idle()
                    self.fig.canvas.flush_events()

                plt.pause(0.001)

                elapsed = time.time() - start_time
                wait = max(0, self.update_interval - elapsed)
                await asyncio.sleep(wait if wait > 0 else 0)

            except asyncio.CancelledError:
                raise
            except Exception as e:
                logger.error(f"绘图错误: {e}")
                await asyncio.sleep(0.1)
