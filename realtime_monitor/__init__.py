"""
Pixhawk 实时姿态监控系统 (异步协程版)

基于asyncio的高性能实时数据接收和可视化系统

主要模块:
- async_receiver: 异步MAVLink数据接收
- async_plotter: 异步实时绘图
- data_buffer: 数据缓冲和处理
- utils: 工具函数
- config: 配置管理
"""

__version__ = '1.0.0'
__author__ = 'Claude Code Assistant'

# 导出主要类
from .async_receiver import AsyncMAVLinkReceiver
from .async_plotter import AsyncOscilloscopePlotter, SimpleAnglePlotter
from .data_buffer import AttitudeBuffer, AttitudeData, DataQueue
from .config import get_config, update_config
from .utils import quat_to_euler, quat_to_euler_deg

__all__ = [
    'AsyncMAVLinkReceiver',
    'AsyncOscilloscopePlotter',
    'SimpleAnglePlotter',
    'AttitudeBuffer',
    'AttitudeData',
    'DataQueue',
    'get_config',
    'update_config',
    'quat_to_euler',
    'quat_to_euler_deg',
]
