# PX4 EKF2 Python - 测试套件

完整的单元测试和端到端集成测试。

## 📁 目录结构

```
tests/
├── README.md                    # 本文档
├── run_all_tests.py             # 主测试运行脚本
├── test_data_generator.py       # 合成数据生成器
├── test_unit.py                 # 单元测试
├── test_integration.py          # 端到端集成测试
└── data/                        # 测试数据目录（自动生成）
    ├── hover_scenario.pkl
    └── forward_flight_scenario.pkl
```

## 🚀 快速开始

### 运行全部测试

```bash
cd px4_ekf2_python
python tests/run_all_tests.py
```

这将按顺序执行：
1. **数据生成**: 生成悬停和前飞场景的模拟传感器数据
2. **单元测试**: 测试各个模块的功能
3. **集成测试**: 运行完整EKF流程并验证精度

### 单独运行

```bash
# 仅生成数据
python tests/test_data_generator.py

# 仅单元测试
python tests/test_unit.py

# 仅集成测试（需要先生成数据）
python tests/test_integration.py
```

## 🧪 测试内容

### 1. 单元测试 (`test_unit.py`)

测试各个核心模块：

- ✅ **四元数运算**: 旋转、乘法、归一化
- ✅ **Welford在线方差**: 振动监控算法
- ✅ **圆锥补偿**: 角度积分精度
- ✅ **IMU处理器**: 降采样、积分
- ✅ **状态预测**: 速度、位置积分
- ✅ **科里奥利修正**: 高纬度飞行
- ✅ **偏置学习抑制**: 轴向选择性逻辑

### 2. 数据生成器 (`test_data_generator.py`)

生成两种飞行场景：

#### 场景1: 悬停 (10秒)
```
起飞(2s) → 悬停(3s) → 降落(2s) → 着陆(3s)
```

#### 场景2: 前飞 (20秒)
```
起飞(2s) → 悬停(2s) → 加速前飞(3s) → 匀速(3s) →
减速(3s) → 悬停(2s) → 降落(5s)
```

**生成的传感器数据**:
- IMU: 1000 Hz (陀螺仪、加速度计)
- GPS: 10 Hz (位置、速度)
- 气压计: 50 Hz (高度)
- 磁力计: 50 Hz (磁场)

**噪声模型**:
- 陀螺噪声: 0.015 rad/s
- 加速度噪声: 0.35 m/s²
- GPS位置噪声: 0.5 m
- GPS速度噪声: 0.3 m/s

### 3. 集成测试 (`test_integration.py`)

端到端运行EKF，验证估计精度：

**测试流程**:
1. 加载模拟传感器数据
2. IMU预处理（降采样、圆锥补偿）
3. EKF预测（状态、协方差）
4. 传感器融合（GPS、气压、磁力计）
5. 计算估计误差
6. 生成可视化结果

**精度要求**:
- 位置RMSE < 2.0 m
- 速度RMSE < 1.0 m/s
- 姿态RMSE < 5.0 °

## 📊 输出结果

### 终端输出示例

```
======================================================================
  步骤: 数据生成
======================================================================
生成悬停场景...
生成IMU测量数据...
生成GPS测量数据...
...
✅ 数据生成 成功 (耗时: 2.34秒)

======================================================================
  步骤: 单元测试
======================================================================
============================================================
测试: 四元数运算
============================================================
✓ 单位四元数正确
✓ 轴角转四元数正确
...
✅ 单元测试 成功 (耗时: 0.15秒)

======================================================================
  步骤: 集成测试
======================================================================
...
误差统计
============================================================
位置RMSE (NED): [0.523, 0.489, 0.312] m
速度RMSE (NED): [0.234, 0.198, 0.156] m/s
姿态RMSE (RPY): [0.812, 0.934, 1.234] °
...
✅ 集成测试 成功 (耗时: 5.67秒)

======================================================================
  测试总结
======================================================================
  数据生成              ✅ PASS
  单元测试              ✅ PASS
  集成测试              ✅ PASS

======================================================================
  🎉 所有测试通过！
  总耗时: 8.16秒
======================================================================
```

### 可视化结果

测试完成后生成PNG图表：

- `tests/hover_results.png`: 悬停场景结果
- `tests/forward_flight_results.png`: 前飞场景结果

图表包含：
1. 位置估计 vs 真值
2. 速度估计 vs 真值
3. 姿态估计 vs 真值
4. 位置误差时间历程
5. 速度误差时间历程
6. 姿态误差时间历程

## 🔧 自定义测试

### 修改场景参数

编辑 `test_data_generator.py`:

```python
# 修改仿真时长
dataset = generate_test_dataset(
    scenario='forward_flight',
    duration=30.0,  # 改为30秒
    save_path='tests/data/custom_scenario.pkl'
)
```

### 修改传感器噪声

编辑 `FlightTrajectory` 类:

```python
imu_data = traj.generate_imu_data(
    gyro_noise_std=0.02,    # 增大陀螺噪声
    accel_noise_std=0.5     # 增大加速度噪声
)
```

### 启用科里奥利修正

编辑 `test_integration.py`:

```python
ekf, results = run_ekf_integration_test(
    dataset,
    enable_coriolis=True,
    latitude_deg=60.0  # 设置纬度
)
```

## 🐛 故障排除

### 问题1: 导入错误

```
ModuleNotFoundError: No module named 'px4_ekf2_python'
```

**解决**:
```bash
# 确保在正确的目录运行
cd PX4-Autopilot/px4_ekf2_python
python tests/run_all_tests.py
```

### 问题2: 数据文件不存在

```
数据文件不存在: tests/data/hover_scenario.pkl
```

**解决**:
```bash
# 先生成数据
python tests/test_data_generator.py
# 再运行测试
python tests/test_integration.py
```

### 问题3: Matplotlib错误

```
ImportError: No module named matplotlib
```

**解决**:
```bash
pip install matplotlib numpy
```

## 📈 性能基准

在典型硬件上（Intel i7, Python 3.8）：

| 测试 | 耗时 | 备注 |
|-----|------|------|
| 数据生成 | ~2s | 生成20秒飞行数据 |
| 单元测试 | ~0.2s | 7个测试 |
| 集成测试 | ~6s | 2个场景，共30秒数据 |
| **总计** | **~8s** | 完整测试套件 |

## 📝 添加新测试

### 1. 添加单元测试

编辑 `test_unit.py`:

```python
def test_new_feature():
    """测试新功能"""
    print("="*60)
    print("测试: 新功能")
    print("="*60)

    # 测试代码
    result = my_function()
    assert result == expected, "测试失败"

    print("\n新功能测试通过 ✅\n")

# 在 run_all_unit_tests() 中调用
def run_all_unit_tests():
    # ...
    test_new_feature()
    # ...
```

### 2. 添加新场景

编辑 `test_data_generator.py`:

```python
def generate_custom_scenario(self):
    """自定义场景"""
    for i, t in enumerate(self.time):
        # 定义轨迹
        if t < 5.0:
            # 阶段1
            pass
        # ...
```

## 📚 参考

- 主文档: `../README.md`
- 算法清单: `../ALGORITHM_CHECKLIST.md`
- 实现总结: `../IMPLEMENTATION_SUMMARY.md`
- 使用示例: `../example.py`

---

**测试版本**: v1.0
**最后更新**: 2025-11-16
