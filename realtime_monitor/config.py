"""
异步实时监控系统配置文件

包含所有可配置参数
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class SerialConfig:
    """串口配置"""
    port: str = "COM3"  # Windows: COMx, Linux: /dev/ttyUSBx
    baudrate: int = 921600
    timeout: float = 1.0
    auto_reconnect: bool = True
    reconnect_interval: float = 2.0  # 秒


@dataclass
class PlotConfig:
    """绘图配置"""
    # 窗口设置
    window_size: int = 1000  # 显示的数据点数量（5秒 @ 200Hz）
    figure_size: Tuple[int, int] = (16, 10)  # 图形窗口大小（英寸）

    # 更新设置
    update_interval: float = 0.05  # 绘图更新间隔（秒），50ms = 20fps

    # 显示设置
    show_quaternion: bool = True  # 是否显示四元数原始值
    show_euler: bool = True       # 是否显示欧拉角
    show_rates: bool = True       # 是否显示角速度

    # 颜色设置
    color_roll: str = 'red'
    color_pitch: str = 'green'
    color_yaw: str = 'blue'
    color_quat: str = 'purple'

    # 线条设置
    line_width: float = 1.5
    grid_alpha: float = 0.3

    # 性能设置
    use_blitting: bool = False  # matplotlib blitting优化（可能不稳定）
    max_fps: int = 30  # 最大帧率限制


@dataclass
class DataConfig:
    """数据处理配置"""
    # 缓冲设置
    buffer_size: int = 2000  # 数据缓冲区大小
    queue_maxsize: int = 500  # 异步队列最大长度

    # 频率计算
    freq_window_size: int = 50  # 频率计算窗口（数据点）

    # 丢包检测
    expected_interval_ms: float = 5.0  # 预期数据间隔（200Hz = 5ms）
    loss_threshold: float = 1.5  # 丢包判定阈值（倍数）

    # 数据处理
    enable_filtering: bool = False  # 是否启用数据滤波
    filter_window: int = 5  # 滑动平均窗口


@dataclass
class SystemConfig:
    """系统配置"""
    # 日志设置
    log_level: str = "INFO"  # DEBUG, INFO, WARNING, ERROR
    show_stats: bool = True  # 是否显示统计信息
    stats_interval: float = 1.0  # 统计信息更新间隔（秒）

    # 性能设置
    use_uvloop: bool = False  # 是否使用uvloop（需要安装，性能提升30-50%）

    # 保存设置
    auto_save_data: bool = False  # 是否自动保存数据
    save_path: str = "./data"  # 数据保存路径
    save_format: str = "csv"  # 保存格式：csv, json, pickle


# 默认配置实例
DEFAULT_CONFIG = {
    'serial': SerialConfig(),
    'plot': PlotConfig(),
    'data': DataConfig(),
    'system': SystemConfig()
}


def get_config(config_type: str = None):
    """
    获取配置

    参数:
        config_type: 配置类型 ('serial', 'plot', 'data', 'system')
                    如果为None，返回所有配置

    返回:
        配置对象或配置字典
    """
    if config_type is None:
        return DEFAULT_CONFIG

    return DEFAULT_CONFIG.get(config_type)


def update_config(config_type: str, **kwargs):
    """
    更新配置

    参数:
        config_type: 配置类型
        **kwargs: 要更新的配置项

    示例:
        update_config('serial', port='/dev/ttyUSB0', baudrate=115200)
    """
    if config_type in DEFAULT_CONFIG:
        config = DEFAULT_CONFIG[config_type]
        for key, value in kwargs.items():
            if hasattr(config, key):
                setattr(config, key, value)
            else:
                print(f"Warning: {config_type}.{key} is not a valid config option")
