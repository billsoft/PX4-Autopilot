# Nucleo-H743ZI-FC 开发文档索引

**最后更新**: 2025-12-02
**项目状态**: ✅ 编译成功，待硬件测试

---

## 📋 项目概述

基于STM32 Nucleo-H743ZI的最小飞控系统，目标是双IMU+磁力计融合输出姿态。

### 硬件配置
- **双IMU**: ICM45686 × 2 (SPI1 + SPI3, 使用icm42688p驱动兼容模式)
- **磁力计**: BMM150 (I2C1, PB6/PB9)
- **CMOS同步**: GPIO PE3/PE4 (行/帧同步信号)
- **输出**: USART3 (PD8/PD9) MAVLink 姿态四元数 @ 120Hz
- **LED指示**: 3个LED (PB0绿/PB7黄/PB14红)

### 功能范围
- ✅ 双IMU数据采集和Mahony融合算法
- ✅ 磁力计数据读取
- ✅ 120Hz姿态四元数输出 (vehicle_attitude)
- ✅ MAVLink通信 (ATTITUDE_QUATERNION)
- ✅ LED状态指示 (启动/运行/融合)
- ❌ **不包含**: EKF2, 导航, 控制器, GPS, 电机输出, SD卡

### 编译结果
```
Flash: 461.86 KB / 2 MB (22.49%)
SRAM:  11.36 KB / 512 KB (2.22%)
状态: ✅ 编译成功，已烧录
```

---

## 📚 核心文档（按优先级排序）

### 🔴 P0 - 必读文档

#### 1. [stm32_custom_board_bringup_tutorial_v2.md](stm32_custom_board_bringup_tutorial_v2.md) ⭐⭐⭐
**用途**: 从零到起飞的完整开发指南 (2700+ 行，知乎深度文章风格)
**内容**:
- **第一部分**: 理解本质 (为什么、两种方法、三大原则)
- **第二部分**: 准备阶段 (硬件/软件/参考板选择)
- **第三部分**: 核心实战 (目录结构、defconfig、board.h、PX4板级代码、模块配置、启动脚本)
- **第四部分**: 验证与调试 (待编写)
- **第五部分**: 进阶优化 (待编写)

**特点**:
- 深入解释每个决策背后的"为什么"
- 包含大量血泪教训和实战经验
- 400+ 行带注释的defconfig示例
- 完整的board.h时钟树配置
- 异步PX4初始化模式详解
- ICM45686兼容性问题深度剖析

**何时阅读**: 开始开发前必读，理解整个系统架构

---

#### 2. [TESTING_CHECKLIST.md](TESTING_CHECKLIST.md) ⭐⭐
**用途**: 系统测试检查清单
**内容**:
- 5分钟快速验证（6个检查项）
- 30分钟详细验证（10个测试项）
- 故障排查指南（4大类问题）
- 测试通过标准（最低要求 vs 理想状态）

**何时使用**: 烧录固件后立即使用

---

#### 3. [nsh_quick_test_card.md](nsh_quick_test_card.md) ⭐
**用途**: 5分钟快速测试卡片
**内容**:
- 一键测试脚本（可直接复制到NSH）
- 常见问题快速修复表
- 调试命令速查

**何时使用**: 每次烧录后快速验证系统状态

---

### 🟡 P1 - 重要参考

#### 4. [work_summary_2025-12-02.md](work_summary_2025-12-02.md)
**用途**: 最新工作总结（2025-12-02）
**内容**:
- 3个关键代码修复（I2C1 SDA引脚、MAVLink流、欧拉角输出）
- 编译烧录结果
- NSH测试文档索引
- 下一步行动计划

---

#### 5. [nsh_module_test_guide.md](nsh_module_test_guide.md)
**用途**: 完整NSH模块测试指南
**内容**:
- 12节完整测试流程
- 100+ NSH命令详解
- 故障排查速查表
- 完整测试脚本

---

#### 6. [test_results_template.md](test_results_template.md)
**用途**: 测试结果记录模板
**内容**:
- 9个主要测试项的详细表格
- 实际输出粘贴区域
- 问题记录和下一步计划

---

### 🟢 P2 - 技术参考

#### 7. [STM32_743zi_pin.md](STM32_743zi_pin.md)
**用途**: Nucleo-H743ZI 引脚映射完整参考
**内容**:
- Arduino接口 vs CPU引脚映射
- SPI1/SPI3引脚定义（避免以太网冲突）
- I2C1引脚定义（PB6/PB9，不是PB7！）
- USART3引脚定义
- LED引脚定义

---

#### 8. [STM32_743zi_clock.md](STM32_743zi_clock.md)
**用途**: STM32H743 时钟配置详解
**内容**:
- HSE 8MHz配置（ST-LINK MCO）
- PLL1配置（480MHz SYSCLK）
- PLL2配置（192MHz SPI123时钟）
- Flash等待周期设置

---

#### 9. [build_system_complete_guide.md](build_system_complete_guide.md)
**用途**: PX4构建系统完整指南
**内容**:
- 为什么不需要CubeMX
- CMake工作流程
- .px4board文件格式
- 模块配置方法

---

#### 10. [nuttx_stm32h7_driver_support.md](nuttx_stm32h7_driver_support.md)
**用途**: NuttX STM32H7驱动支持分析
**内容**:
- 回答"需要自己写驱动吗？"
- NuttX已有驱动列表
- SPI/I2C/UART驱动配置
- defconfig配置示例

---

### 🔵 P3 - 问题修复文档

#### 11. [code_review_complete_summary.md](code_review_complete_summary.md)
**用途**: 代码审查完整总结（2025-12-02）
**内容**:
- 3个必需修复方案
- 完整编译指南
- NSH测试命令清单
- 构建前检查清单

---

#### 12. [linker_errors_fix_guide.md](linker_errors_fix_guide.md)
**用途**: 链接错误修复指南
**内容**:
- HRT配置要求
- stm32_boardinitialize实现
- 常见链接错误解决方案

---

#### 13. [hrt_fix_complete_guide.md](hrt_fix_complete_guide.md)
**用途**: HRT（高精度定时器）修复完整指南
**内容**:
- TIM5与NuttX的冲突
- CONFIG_STM32H7_TIM5禁用要求
- 完整修复流程

---

#### 14. [spi_clock_fix_summary.md](spi_clock_fix_summary.md)
**用途**: SPI时钟配置修复总结
**内容**:
- PLL2P必须≤200MHz
- SPI123时钟源配置
- 修复验证方法

---

## 🗂️ 快速导航

### 场景1：从零开始开发自定义板
```
1. stm32_custom_board_bringup_tutorial_v2.md (完整阅读第1-3部分)
2. STM32_743zi_pin.md (确认硬件接线)
3. STM32_743zi_clock.md (理解时钟配置)
4. build_system_complete_guide.md (理解构建流程)
5. 开始编码...
```

### 场景2：编译遇到错误
```
1. linker_errors_fix_guide.md (链接错误)
2. hrt_fix_complete_guide.md (HRT相关)
3. spi_clock_fix_summary.md (SPI时钟)
4. stm32_custom_board_bringup_tutorial_v2.md 第9节 (board.h常见错误)
```

### 场景3：烧录后测试
```
1. TESTING_CHECKLIST.md (打印出来，逐项勾选)
2. nsh_quick_test_card.md (快速验证)
3. nsh_module_test_guide.md (详细测试)
4. test_results_template.md (记录结果)
```

### 场景4：添加新传感器/模块
```
1. nuttx_stm32h7_driver_support.md (确认NuttX驱动支持)
2. build_system_complete_guide.md (模块配置方法)
3. stm32_custom_board_bringup_tutorial_v2.md 第11-12节 (模块配置和启动脚本)
```

---

## 📁 相关代码文件

### 板级配置文件
```
boards/st/nucleo-h743zi-fc/
├── default.px4board                    # PX4模块配置
├── init/
│   └── rc.board_sensors                # 启动脚本（传感器初始化）
├── src/
│   ├── board_config.h                  # PX4硬件抽象层定义
│   ├── init.cpp                        # 板级初始化（异步模式）
│   ├── spi.cpp                         # SPI设备表
│   ├── i2c.cpp                         # I2C总线表
│   └── CMakeLists.txt                  # 编译配置
└── nuttx-config/
    ├── nsh/defconfig                   # NuttX配置（外设启用）
    ├── include/board.h                 # NuttX硬件定义（时钟/GPIO）
    └── Kconfig                         # Kconfig配置
```

### 自定义模块
```
src/modules/
├── dual_imu_fusion/                    # 双IMU Mahony融合算法
├── board_status_leds/                  # LED状态指示
├── cmos_sync/                          # CMOS相机同步
└── sensor_stub/                        # 传感器桩（测试用）
```

---

## ✅ 当前开发状态（2025-12-02）

### 已完成 ✅
- ✅ 板级移植 (CONFIG_ARCH_BOARD_CUSTOM配置)
- ✅ board.h完全自包含 (454行，包含PLL配置)
- ✅ SPI1/SPI3配置 (避免以太网RMII冲突)
- ✅ I2C1配置 (修复PB9引脚定义)
- ✅ USART3 MAVLink配置
- ✅ 最小模块配置 (禁用EKF2/导航/控制器/SD卡/GPS等)
- ✅ 双IMU融合算法实现 (Mahony滤波器 + 噪声估计)
- ✅ LED状态指示实现 (启动心跳/运行状态/融合指示)
- ✅ MAVLink流优化 (只保留120Hz ATTITUDE_QUATERNION)
- ✅ 固件编译成功 (461.86 KB Flash, 22.49%)
- ✅ 固件烧录成功 (STM32CubeProgrammer验证通过)
- ✅ 完整测试文档编写

### 待测试 🔄
- ⏳ NSH串口连接测试
- ⏳ 传感器驱动验证 (ICM45686 × 2, BMM150)
- ⏳ 双IMU融合算法验证
- ⏳ MAVLink姿态输出验证
- ⏳ LED状态指示验证
- ⏳ 性能测试 (CPU/内存/uORB速率)

### 待优化 ⏳
- ⏳ 传感器采样频率调优
- ⏳ 融合算法参数调优（增益、噪声阈值）
- ⏳ MAVLink带宽优化
- ⏳ 长时间稳定性测试 (>1小时)

---

## 🚀 下一步行动

### 立即执行
1. **连接串口**: Windows PuTTY 或 WSL picocom
2. **快速测试**: 按照 `TESTING_CHECKLIST.md` 执行5分钟验证
3. **记录结果**: 使用 `test_results_template.md` 记录实际输出

### 如果测试通过
4. **完整测试**: 按照 `nsh_module_test_guide.md` 执行30分钟详细测试
5. **动态测试**: 倾斜/旋转板子，验证姿态响应
6. **稳定性测试**: 连续运行1小时，监控漂移

### 如果测试失败
7. **问题诊断**: 参考 `TESTING_CHECKLIST.md` 故障排查章节
8. **修复代码**: 根据错误类型修复硬件/软件
9. **重新编译烧录**: 验证修复效果

---

## 🔗 外部参考

- **PX4官方文档**: https://docs.px4.io/main/en/
- **NuttX文档**: https://nuttx.apache.org/docs/latest/
- **STM32H743参考手册**: [RM0433](https://www.st.com/resource/en/reference_manual/rm0433-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- **Nucleo-H743ZI用户手册**: [UM2407](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf)
- **ICM-42688-P数据手册**: [TDK InvenSense](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- **BMM150数据手册**: [Bosch Sensortec](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm150/)

---

## 📧 维护信息

**维护者**: 开发团队
**联系方式**: 通过项目Issue反馈
**文档版本**: 3.0 (整合版)
**最后更新**: 2025-12-02 14:30

---

## 🗑️ 已清理的文档

以下文档已被新文档替代并删除：
- ~~code_review_plan.md~~ → code_review_complete_summary.md
- ~~stage1_board_config_review.md~~ → code_review_complete_summary.md
- ~~stage2_sensor_driver_review.md~~ → code_review_complete_summary.md
- ~~hardware_test_checklist.md~~ → TESTING_CHECKLIST.md
- ~~project_status_and_learning.md~~ → work_summary_2025-12-02.md
- ~~px4_custom_board_complete_guide.md~~ → stm32_custom_board_bringup_tutorial_v2.md
- ~~stm32_custom_board_bringup_tutorial.md~~ → stm32_custom_board_bringup_tutorial_v2.md

保持文档目录清洁、结构清晰！✨
