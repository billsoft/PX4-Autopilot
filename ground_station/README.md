# Pixhawk 地面站工具

此目录包含用于Pixhawk数据接收、分析和可视化的Python工具。

## 目录结构

```
ground_station/
├── README.md                    # 本文件
├── requirements.txt             # Python依赖包
├── pixhawk_monitor/             # 数据监控包
│   ├── __init__.py
│   ├── receiver.py              # MAVLink数据接收
│   ├── plotter.py               # 实时数据绘图
│   └── utils.py                 # 工具函数
└── examples/                    # 示例脚本
    ├── simple_receiver.py       # 简单接收示例
    └── realtime_plot.py         # 实时绘图示例
```

## 安装

### 1. 安装Python依赖
```bash
pip install -r requirements.txt
```

### 2. 连接Pixhawk
- USB: 连接到 /dev/ttyACM0 (Linux) 或 COMx (Windows)
- UART4: 连接到 /dev/ttyUSB0 (使用USB转串口模块)

## 快速开始

### 简单数据接收
```bash
cd ground_station
python examples/simple_receiver.py --port /dev/ttyUSB0 --baud 921600
```

### 实时数据可视化
```bash
python examples/realtime_plot.py --port /dev/ttyUSB0 --baud 921600
```

## 功能特性

### 数据接收器 (receiver.py)
- MAVLink消息解析
- 多线程数据接收
- 数据队列缓冲
- 时间戳同步
- 丢包检测

### 实时绘图 (plotter.py)
- IMU数据绘图（加速度、陀螺仪）
- 磁力计数据绘图
- 姿态四元数/欧拉角绘图
- 实时更新（支持200Hz稳定/300Hz峰值）
- 数据统计（频率、丢包率）

## 使用说明

详细使用说明请参考 `docs/uart4_300hz_imu_output_configuration.md`

## 依赖包

- pymavlink: MAVLink协议支持
- matplotlib: 数据可视化
- numpy: 数值计算
- pyserial: 串口通信

## 故障排查

### 问题1：找不到串口设备
- Linux: 添加用户到dialout组 `sudo usermod -a -G dialout $USER`
- Windows: 检查设备管理器中的COM端口号

### 问题2：接收不到数据
- 确认Pixhawk已配置UART4 MAVLink输出
- 检查波特率匹配（921600）
- 验证硬件连接（TX-RX, RX-TX, GND-GND）

### 问题3：绘图卡顿
- 降低绘图更新频率
- 减少绘图窗口数量
- 使用更快的电脑

## 许可证

本代码遵循与PX4-Autopilot相同的BSD 3-Clause许可证。
