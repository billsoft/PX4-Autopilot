# Nucleo-H743ZI-FC 开发文档索引

## 📋 项目目标

开发基于STM32 Nucleo-H743ZI的最小飞控系统:

### 硬件配置
- **双IMU**: 两个ICM45686 (SPI1 + SPI3)
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

#### 2. [nucleo_h743zi_pinmap.md](nucleo_h743zi_pinmap.md) ⭐**硬件参考**
**状态**: ✅ 官方/实用结合
**用途**: 开发板插槽/接口与CPU引脚映射、典型接线建议
- ICM45686 双IMU接口（SPI1 主用，SPI3 作为第二路以避以太网冲突）
- BMM150 磁力计 I2C1 接线（PB8/PB9）
- 以太网与 SPI2 冲突说明（PB13 等）
- 常用 GPIO/片选选择建议

**何时参考**:
- 确认硬件接线
- 修改SPI/I2C引脚配置
- 解决引脚冲突

---

 

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

当前采用最小系统迭代开发，原计划类文档已清理；以本 README 索引与保留技术文档为准。

---

## 🗂️ 文档使用流程

### 新手快速上手
1. 阅读 [px4_custom_board_complete_guide.md](px4_custom_board_complete_guide.md)
2. 参考 [nucleo_h743zi_pinmap.md](nucleo_h743zi_pinmap.md) 确认硬件
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
└── 已实现
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
- 🔄 硬件在环验证与调试

### 待完成
- ⏳ 传感器实际频率与融合参数调优
- ⏳ MAVLink带宽与流控参数调优

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
