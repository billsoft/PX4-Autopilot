# PX4 Autopilot 快速上手指南

**适用对象**: PX4 新手开发者
**预计时间**: 2-4 小时快速入门
**目标**: 快速了解 PX4 核心架构，能够运行 SITL 仿真，理解基本开发流程

---

## 📋 前置条件检查

在开始之前，确保你已经：

- [ ] 克隆了 PX4-Autopilot 仓库 (含子模块)
- [ ] 安装了构建工具链 (gcc-arm-none-eabi, cmake, ninja 等)
- [ ] 了解基本的 C/C++ 编程
- [ ] 熟悉 Linux/Unix 命令行基础

**如果尚未配置开发环境**，请先参考：
- [PX4 官方开发环境搭建](https://docs.px4.io/main/en/dev_setup/)
- 或项目根目录的 `README.md`

---

## 🚀 第一步：5 分钟试飞 (SITL 仿真)

### 1.1 编译并启动仿真

在项目根目录执行：

```bash
make px4_sitl gz_x500
```

这条命令会：
1. 编译 PX4 SITL (Software In The Loop) 版本
2. 启动 Gazebo 仿真器，加载 X500 四旋翼模型
3. 启动 PX4 交互式控制台 (pxh>)

**预期结果**：
- Gazebo 窗口打开，显示一个四旋翼无人机
- 终端显示 `pxh>` 提示符（PX4 Shell）

### 1.2 起飞测试

在 `pxh>` 控制台执行：

```bash
pxh> commander takeoff    # 起飞到 2.5 米高度
pxh> commander land        # 降落
pxh> shutdown              # 关闭仿真
```

**恭喜！** 你已经成功运行了第一个 PX4 仿真。

---

## 📚 第二步：理解核心架构 (30 分钟)

### 2.1 必读文档 (按顺序)

1. **项目根目录 CLAUDE.md** (15 分钟)
   - 位置: `PX4-Autopilot/CLAUDE.md`
   - 内容: 构建命令、架构概览、常用开发任务
   - **重点阅读**:
     - "Architecture" 章节 → 了解 uORB, 模块化设计
     - "Common Development Tasks" → 学习如何运行 SITL、添加模块

2. **uORB 消息总线教程** (15 分钟)
   - 位置: `.trae/documents/system/uorb.md`
   - 内容: PX4 核心通信机制
   - **重点理解**:
     - 发布-订阅模式 (Publish-Subscribe)
     - 消息定义 (`msg/*.msg`)
     - 如何发布/订阅消息

### 2.2 核心概念速查表

| 概念 | 说明 | 示例 |
|------|------|------|
| **uORB** | 微对象请求代理，PX4 的消息总线 | `sensor_accel`, `vehicle_attitude` |
| **模块 (Module)** | PX4 的功能单元 | `ekf2` (状态估计), `mc_pos_control` (位置控制) |
| **Work Queue** | 任务调度机制，节省内存 | `hp_default`, `lp_default` |
| **SITL** | 软件在环仿真 (PC 上运行) | `make px4_sitl gz_x500` |
| **NuttX** | 嵌入式硬件使用的 RTOS | Pixhawk 等飞控板 |
| **rcS** | 启动脚本 (相当于 Linux 的 init) | `ROMFS/px4fmu_common/init.d/rcS` |

---

## 🔧 第三步：动手实践 (1-2 小时)

### 3.1 查看实时消息

启动 SITL 后，在 `pxh>` 控制台尝试：

```bash
# 查看所有 uORB 主题的发布频率
pxh> uorb top

# 监听具体主题 (Ctrl+C 退出)
pxh> listener sensor_accel      # 加速度计数据
pxh> listener vehicle_attitude   # 姿态估计 (四元数)
pxh> listener vehicle_gps_position  # GPS 位置

# 查看参数
pxh> param show EKF2_*           # EKF2 所有参数
pxh> param set EKF2_AID_MASK 1   # 设置 GPS 融合
```

### 3.2 代码探索任务

**任务 1**: 找到 EKF2 模块的主文件

```bash
# 提示: 在 src/modules/ekf2/ 目录
# 主文件: EKF2.cpp 和 EKF2.hpp
```

打开 `src/modules/ekf2/EKF2.cpp`，找到：
- `Run()` 函数 → 主循环逻辑
- `ORB_ID(sensor_combined)` → 订阅传感器数据
- `_attitude_pub.publish()` → 发布姿态估计

**任务 2**: 理解 uORB 消息定义

打开 `msg/vehicle_attitude.msg`，观察：
```
uint64 timestamp        # 微秒时间戳 (必须字段)
float32[4] q            # 四元数 [w, x, y, z]
float32[3] delta_q_reset  # 四元数重置增量
```

这些 `.msg` 文件会自动生成 C++ 头文件: `build/px4_sitl_default/uORB/topics/vehicle_attitude.h`

**任务 3**: 修改一个参数并观察效果

```bash
# 打开 QGroundControl (地面站软件)
# 或在 pxh> 控制台修改参数

pxh> param set MC_ROLLRATE_P 0.5   # 降低横滚速率 P 增益
pxh> commander takeoff
# 观察飞行是否更"软"（响应变慢）
```

---

## 🎓 第四步：深入学习路径 (根据兴趣选择)

### 路径 A: 算法工程师

**目标**: 理解状态估计和控制算法

1. **EKF2 状态估计** (12-16 小时)
   - 文档: `.trae/documents/algorithms/ekf2.md` (⭐ 1500+ 行深度教程)
   - 代码: `src/modules/ekf2/`
   - 关键知识: 卡尔曼滤波、四元数、传感器融合

2. **飞行控制器** (10-14 小时)
   - 文档: `.trae/documents/algorithms/controllers.md`
   - 代码: `src/modules/mc_pos_control/`, `src/modules/mc_att_control/`
   - 关键知识: PID 控制、级联控制、飞行力学

3. **固定翼控制** (可选)
   - 文档: `.trae/documents/algorithms/tecs.md`
   - 代码: `src/lib/tecs/`

### 路径 B: 嵌入式工程师

**目标**: 移植 PX4 到自定义硬件

1. **NuttX RTOS 架构** (10-14 小时)
   - 文档: `.trae/documents/rtos/index.md` (⭐ 1267 行)
   - 关键知识: 任务调度、Work Queue、平台抽象层

2. **驱动开发** (8-10 小时)
   - 文档: `.trae/documents/drivers/nuttx_driver_development.md` (⭐ 2884 行)
   - 代码: `src/drivers/imu/`, `src/drivers/gps/`
   - 实战: IMU/GPS 驱动开发

3. **STM32H743 硬件移植** (8-12 小时)
   - 文档: `.trae/documents/通用基础系统/stm32h743_minimal_flight_controller_guide.md` (⭐ 26,069 tokens)
   - 完整移植指南: 时钟树、DMA、uORB 集成

### 路径 C: 系统集成工程师

**目标**: 对接外部系统 (地面站、ROS 2)

1. **启动脚本系统** (2-3 小时)
   - 文档: `.trae/documents/通用基础系统/rcS_注释版.md`
   - 代码: `ROMFS/px4fmu_common/init.d/rcS`

2. **MAVLink 协议** (4-5 小时)
   - 文档: `.trae/documents/interfaces/mavlink.md`
   - 代码: `src/modules/mavlink/`

3. **ROS 2 集成** (4-5 小时)
   - 文档: `.trae/documents/interfaces/dds.md`
   - 代码: `src/modules/uxrce_dds_client/`

---

## 🛠️ 常用开发命令速查

### 构建与测试

```bash
# SITL 仿真
make px4_sitl gz_x500              # 多旋翼
make px4_sitl gz_standard_vtol     # 垂直起降
HEADLESS=1 make px4_sitl gz_x500   # 无 GUI 模式

# 硬件构建
make px4_fmu-v6x_default           # Pixhawk 6X
make px4_fmu-v5_default upload     # 构建并烧录到飞控

# 测试
make tests                         # 单元测试
make tests TESTFILTER=ekf          # 只跑 EKF 相关测试

# 代码质量
make format                        # 自动格式化代码
make check_format                  # 检查格式
```

### 调试命令 (在 pxh> 控制台)

```bash
# 消息监控
listener <topic>      # 监听 uORB 消息
uorb top              # 所有主题的发布频率

# 参数管理
param show            # 列出所有参数
param set <NAME> <VAL>  # 设置参数
param save            # 保存到 EEPROM

# 性能分析
top                   # 任务 CPU 占用
perf                  # 性能计数器
work_queue status     # Work Queue 状态

# 模块控制
<module> start        # 启动模块 (如 ekf2 start)
<module> stop         # 停止模块
<module> status       # 查看状态
```

---

## 📖 完整文档索引

所有技术文档的详细索引和学习路径请参考：

**`.trae/documents/INDEX.md`**

该索引包含：
- 25 个技术文档的分类导航
- 按角色推荐的学习路径
- 文档质量评估和使用建议
- 主题快速查找表

---

## ❓ 常见问题

### Q1: SITL 编译失败怎么办？

**A**: 检查子模块是否完整：
```bash
make submodulesupdate
make distclean
make px4_sitl_default
```

### Q2: 如何查看某个模块的源码路径？

**A**: 使用 `grep` 或查看 CLAUDE.md 的架构图：
```bash
# 搜索模块定义
grep -r "MODULE modules__ekf2" src/
# 结果: src/modules/ekf2/CMakeLists.txt
```

### Q3: 修改代码后需要重新编译吗？

**A**: 是的，但 CMake 会增量编译（只编译修改的部分）：
```bash
# 修改代码后
make px4_sitl_default    # 只重新编译修改的文件
```

### Q4: 如何贡献代码到 PX4？

**A**: 参考项目根目录的 `CONTRIBUTING.md`：
1. Fork 仓库
2. 创建功能分支 (基于 `main`)
3. 遵循代码规范 (`make format`)
4. 运行测试 (`make tests`)
5. 提交 Pull Request

### Q5: 文档中的行号引用不准确怎么办？

**A**: 这是正常的，代码会持续更新。建议：
- 使用文档中的代码片段或注释定位
- 使用 IDE 的搜索功能（搜索函数名/变量名）
- 参考最新的在线文档: https://docs.px4.io

---

## 🎯 下一步行动清单

完成本快速上手指南后，建议：

- [ ] 成功运行 SITL 并完成起飞降落测试
- [ ] 阅读 `CLAUDE.md` 了解项目架构
- [ ] 阅读 `system/uorb.md` 理解消息总线
- [ ] 选择一条学习路径 (算法/嵌入式/系统集成)
- [ ] 尝试修改一个简单参数并观察效果
- [ ] 加入 PX4 社区:
  - [官方论坛](https://discuss.px4.io/)
  - [Discord](https://discord.gg/dronecode)
  - [GitHub Issues](https://github.com/PX4/PX4-Autopilot/issues)

---

## 📝 反馈与贡献

如果发现本指南有问题或需要改进，请：
1. 提交 Issue 或 Pull Request
2. 参考 `CONTRIBUTING.md` 了解贡献流程

**文档维护者**: AI Assistant (Claude Code)
**最后更新**: 2025-11-26
**版本**: 1.0

祝你学习愉快！ 🚁
