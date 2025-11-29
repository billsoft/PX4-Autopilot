# Nucleo-H743ZI-FC 开发文档索引

## 📋 项目目标

开发基于STM32 Nucleo-H743ZI的最小飞控系统:

### 硬件配置
- **双IMU**: 两个ICM-42688-P (SPI1 + SPI3)
- **磁力计**: BMM150 (I2C1)
- **CMOS同步**: GPIO接收行同步信号(用于时间戳对齐)
- **输出**: UART3 MAVLink输出姿态四元数
- **无SD卡**: 精简配置,无数据记录

### 功能范围
- ✅ 双IMU数据采集和融合
- ✅ 磁力计数据读取
- ✅ 四元数姿态输出(通过uORB)
- ✅ MAVLink通信
- ❌ **不包含**: EKF2, 导航, 位置控制, ADC, GPS, 电机输出, SD卡日志

---

## 📚 文档结构

### 核心开发文档

#### 1. [px4_custom_board_complete_guide.md](px4_custom_board_complete_guide.md) ⭐**最重要**
**状态**: ✅ 最新、完整、正确
**用途**: PX4自定义板开发完整指南
- 解释`CONFIG_ARCH_BOARD_CUSTOM`必要性
- board.h完全自包含要求
- 完整文件模板和代码示例
- 常见错误和解决方案
- **替代**: 旧的`nucleo_h743zi_step_by_step.md`(已删除,有错误)

**何时参考**:
- 创建新的自定义板
- 理解NuttX板级配置架构
- 解决GPIO宏定义错误
- 理解为什么需要自包含board.h

---

#### 2. [nanopix4pin.md](nanopix4pin.md) ⭐**硬件参考**
**状态**: ✅ 硬件设计参考
**用途**: 您的实际硬件引脚映射
- ICM-42688-P双IMU接口
- BMM150磁力计I2C接线
- 引脚冲突记录(PA7/PB13以太网冲突)
- 实际使用的GPIO分配

**何时参考**:
- 确认硬件接线
- 修改SPI/I2C引脚配置
- 解决引脚冲突

---

#### 3. [nucleo_h743zi_pinmap.md](nucleo_h743zi_pinmap.md)
**状态**: ✅ 官方参考
**用途**: Nucleo-H743ZI官方引脚映射
- Arduino接口引脚
- Zio接口引脚
- ST Morpho接口引脚
- 完整STM32H743ZI引脚复用表

**何时参考**:
- 选择可用引脚
- 理解Arduino D13/D12等标号
- 查询复用功能

---

### 技术深度文档

#### 4. [nuttx_stm32h7_driver_support.md](nuttx_stm32h7_driver_support.md)
**状态**: ✅ 技术参考
**用途**: NuttX STM32H7驱动支持分析
- 回答"需要自己写驱动吗?"
- NuttX已有驱动列表
- SPI/I2C/UART驱动配置方法
- defconfig配置示例

**何时参考**:
- 启用新外设前查询驱动支持
- 配置NuttX defconfig
- 理解NuttX驱动架构

---

#### 5. [build_system_complete_guide.md](build_system_complete_guide.md)
**状态**: ✅ 架构参考
**用途**: PX4构建系统深度指南
- 为什么不需要CubeMX
- CMake工作流程
- .px4board文件格式
- 模块配置方法

**何时参考**:
- 理解PX4构建流程
- 添加/删除模块
- 修改编译选项

---

### 开发计划文档

#### 6. [nucleo_h743zi_minimal_flight_controller_dev_plan.md](nucleo_h743zi_minimal_flight_controller_dev_plan.md)
**状态**: ⚠️ 需要更新
**用途**: 阶段1-3开发计划
- 硬件准备
- 板级移植
- 传感器驱动集成

**当前进度**: 阶段2已完成(板级移植成功)

---

#### 7. [nucleo_h743zi_fc_dev_plan_stages_4_to_8.md](nucleo_h743zi_fc_dev_plan_stages_4_to_8.md)
**状态**: ⚠️ 可能不适用最小系统
**用途**: 阶段4-8开发计划(完整飞控功能)
- EKF2融合(❌ 您使用自定义融合)
- 飞行控制(❌ 不需要)
- 导航和任务(❌ 不需要)

**注意**: 本文档面向完整飞控,与当前最小系统目标不符

---

## 🗂️ 文档使用流程

### 新手快速上手
1. 阅读 [px4_custom_board_complete_guide.md](px4_custom_board_complete_guide.md)
2. 参考 [nanopix4pin.md](nanopix4pin.md) 确认硬件
3. 查阅 [nuttx_stm32h7_driver_support.md](nuttx_stm32h7_driver_support.md) 了解驱动支持

### 遇到编译错误
1. 首先检查 [px4_custom_board_complete_guide.md](px4_custom_board_complete_guide.md) 的常见错误章节
2. 参考 [boards/px4/fmu-v6x](../../boards/px4/fmu-v6x/) 标准实现
3. 确认 [nanopix4pin.md](nanopix4pin.md) 引脚配置无冲突

### 添加新功能
1. 查阅 [build_system_complete_guide.md](build_system_complete_guide.md) 了解如何添加模块
2. 检查 [nuttx_stm32h7_driver_support.md](nuttx_stm32h7_driver_support.md) 确认驱动支持
3. 修改 [boards/st/nucleo-h743zi-fc/default.px4board](../../boards/st/nucleo-h743zi-fc/default.px4board)

---

## 📁 相关代码文件

### 板级配置文件
```
boards/st/nucleo-h743zi-fc/
├── default.px4board                    # 模块配置
├── src/
│   ├── board_config.h                  # PX4板级定义(CS引脚等)
│   ├── spi.cpp                         # SPI设备配置
│   └── led.c                           # LED驱动
└── nuttx-config/
    ├── nsh/defconfig                   # NuttX配置(外设启用)
    ├── include/board.h                 # NuttX硬件定义(GPIO/时钟)
    └── Kconfig                         # Kconfig配置
```

### 自定义模块
```
src/modules/dual_imu_fusion/            # 双IMU融合模块
└── (待实现)
```

---

## ✅ 当前开发状态

### 已完成
- ✅ 板级移植(`CONFIG_ARCH_BOARD_CUSTOM`配置)
- ✅ board.h完全自包含(454行,包含PLL配置)
- ✅ SPI1/SPI3配置(避免以太网冲突)
- ✅ I2C1配置
- ✅ UART3 MAVLink配置
- ✅ 最小模块配置(禁用SD卡/ADC/GPS等)
- ✅ NuttX符号链接验证

### 进行中
- 🔄 编译验证(WSL I/O性能问题)
- 🔄 双IMU融合模块开发

### 待完成
- ⏳ CMOS GPIO同步信号接口
- ⏳ 硬件测试和调试
- ⏳ MAVLink姿态输出验证

---

## 🔗 外部参考

- **PX4官方文档**: https://docs.px4.io/main/en/
- **NuttX文档**: https://nuttx.apache.org/docs/latest/
- **STM32H743参考手册**: https://www.st.com/resource/en/reference_manual/rm0433-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
- **Nucleo-H743ZI用户手册**: https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf

---

**最后更新**: 2025-11-28
**维护者**: 开发团队
**项目状态**: 活跃开发中
