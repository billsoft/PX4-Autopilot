---
文档版本: 1.0
适用PX4版本: v1.14.x - v1.15.x
最后更新: 2025-11-26
文档类型: 构建系统深度教程
难度等级: ⭐⭐⭐⭐ (高级)
前置要求: CMake 基础, 交叉编译概念, Makefile 基础
预计学习时间: 6-10 小时
---

# PX4 构建系统完整指南：从源码到固件

## 重要提示：关于 CubeMX 的误区 ⚠️

**你不需要 CubeMX！**

很多从 STM32 裸机或 FreeRTOS 开发转过来的工程师会有这个误解。PX4 有自己完整的构建系统：

❌ **错误流程**：
```
CubeMX 创建项目 → 导出 CMake/Makefile → 复制 PX4 代码 → 手动整合
```

✅ **正确流程**：
```
PX4 源码 → 配置板级文件(.px4board) → CMake 构建 → 固件生成
```

**为什么不需要 CubeMX？**

1. **PX4 已包含 NuttX RTOS**（作为 git 子模块，位于 `platforms/nuttx/NuttX/`）
2. **NuttX 已包含完整的 STM32 HAL 和启动代码**（不依赖 CubeMX 生成的代码）
3. **PX4 的 CMake 系统会自动调用 NuttX 的构建系统**（处理所有底层细节）
4. **板级配置通过 `.px4board` 文件和 NuttX defconfig**（无需 CubeMX 图形界面）

---

## 目录

1. [构建系统架构概览](#1-构建系统架构概览)
2. [工具链详解](#2-工具链详解)
3. [编译过程分步解析](#3-编译过程分步解析)
4. [固件组成与生成](#4-固件组成与生成)
5. [板级支持包（BSP）](#5-板级支持包bsp)
6. [CMake 构建系统深度剖析](#6-cmake-构建系统深度剖析)
7. [实战：为 Nucleo-H743ZI 构建固件](#7-实战为-nucleo-h743zi-构建固件)
8. [调试构建问题](#8-调试构建问题)

---

## 1. 构建系统架构概览

### 1.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                    用户命令                                      │
│          make px4_fmu-v6x_default                               │
└──────────────────────┬──────────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────────┐
│              顶层 CMakeLists.txt                                 │
│  • 检测平台 (NuttX/POSIX/QURT)                                   │
│  • 加载板级配置 (.px4board)                                       │
│  • 设置工具链 (arm-none-eabi-gcc)                                │
└──────────────────────┬──────────────────────────────────────────┘
                       │
        ┌──────────────┴──────────────┐
        │                              │
        ▼                              ▼
┌──────────────────┐          ┌──────────────────┐
│   NuttX 构建      │          │   PX4 模块构建    │
│                  │          │                  │
│ • bootloader     │          │ • src/modules/   │
│ • RTOS 内核      │          │ • src/lib/       │
│ • 驱动框架       │          │ • src/drivers/   │
│ • 板级初始化     │          │ • uORB 消息      │
└────────┬─────────┘          └────────┬─────────┘
         │                              │
         └──────────────┬───────────────┘
                        ▼
         ┌──────────────────────────────┐
         │      链接 (Linker)            │
         │  • 链接脚本 (.ld)             │
         │  • 内存布局                   │
         │  • 符号表                     │
         └──────────────┬───────────────┘
                        ▼
         ┌──────────────────────────────┐
         │     固件后处理                │
         │  • ELF → BIN                 │
         │  • 添加元数据                 │
         │  • 生成 .px4 文件             │
         └──────────────┬───────────────┘
                        ▼
         ┌──────────────────────────────┐
         │   最终固件文件                │
         │  • xxx.elf (调试用)           │
         │  • xxx.bin (二进制)           │
         │  • xxx.px4 (带元数据)         │
         └──────────────────────────────┘
```

### 1.2 关键路径说明

| 路径 | 作用 | 示例文件 |
|------|------|----------|
| `boards/` | 板级配置目录 | `boards/px4/fmu-v6x/default.px4board` |
| `platforms/nuttx/NuttX/` | NuttX RTOS 子模块 | `nuttx/`, `apps/` |
| `platforms/nuttx/` | PX4 的 NuttX 平台抽象层 | `src/px4_platform_common/` |
| `src/modules/` | PX4 功能模块 | `ekf2/`, `mc_pos_control/` |
| `src/drivers/` | 硬件驱动 | `imu/`, `gps/` |
| `msg/` | uORB 消息定义 | `sensor_accel.msg` |
| `build/板级目标/` | 构建输出目录 | `build/px4_fmu-v6x_default/` |

---

## 2. 工具链详解

### 2.1 ARM 交叉编译工具链

**为什么需要交叉编译？**

- 你的开发机器：x86_64 架构（Windows/Linux/macOS）
- 目标飞控板：ARM Cortex-M7 (STM32H743)
- **无法直接编译**：需要 ARM 交叉编译工具链

**工具链组成**：

```bash
arm-none-eabi-gcc      # C 编译器
arm-none-eabi-g++      # C++ 编译器
arm-none-eabi-as       # 汇编器
arm-none-eabi-ld       # 链接器
arm-none-eabi-objcopy  # 目标文件转换工具 (ELF → BIN)
arm-none-eabi-objdump  # 反汇编工具
arm-none-eabi-size     # 固件大小分析
arm-none-eabi-gdb      # 调试器
```

### 2.2 工具链安装验证

```bash
# 检查工具链版本
arm-none-eabi-gcc --version
# 预期输出: arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1

# 检查目标架构
arm-none-eabi-gcc -print-multi-lib
# 应该包含: armv7e-m 相关的库
```

**PX4 推荐的工具链版本**：
- **ARM GCC 9.3.1** (9-2020-q2-update) - 最稳定
- ARM GCC 10.x 或 11.x 也支持
- 下载地址: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm

### 2.3 工具链在 CMake 中的配置

PX4 通过工具链文件自动配置编译器：

**文件位置**: `cmake/toolchains/Toolchain-arm-none-eabi.cmake`

```cmake
# 设置交叉编译目标系统
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 指定编译器
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# 编译器标志 (针对 Cortex-M7)
set(CMAKE_C_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")

# 链接器
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections")
```

**关键编译选项解释**：

| 选项 | 说明 |
|------|------|
| `-mcpu=cortex-m7` | 指定 CPU 核心为 Cortex-M7 |
| `-mthumb` | 使用 Thumb 指令集（代码密度更高）|
| `-mfpu=fpv5-d16` | 启用硬件浮点单元（双精度，16 个寄存器）|
| `-mfloat-abi=hard` | 硬件浮点 ABI（性能最优）|
| `-Wl,--gc-sections` | 链接时移除未使用的函数（减小固件大小）|

---

## 3. 编译过程分步解析

### 3.1 第一步：配置阶段 (CMake Configure)

当你执行 `make px4_fmu-v6x_default` 时：

```bash
# 1. Make 调用 CMake
mkdir -p build/px4_fmu-v6x_default
cd build/px4_fmu-v6x_default

# 2. CMake 读取顶层 CMakeLists.txt
cmake ../../ -DCONFIG=px4_fmu-v6x_default

# 3. 查找板级配置文件
# 路径: boards/px4/fmu-v6x/default.px4board
```

**板级配置文件解析** (`default.px4board`)：

```python
CONFIG_PLATFORM_NUTTX=y        # 使用 NuttX 平台
CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"  # 工具链
CONFIG_BOARD_ARCHITECTURE="cortex-m7"   # CPU 架构

# 启用的模块
CONFIG_MODULES_EKF2=y          # EKF2 状态估计器
CONFIG_MODULES_MC_POS_CONTROL=y  # 多旋翼位置控制
CONFIG_DRIVERS_IMU_INVENSENSE_ICM20948=y  # IMU 驱动

# NuttX 配置
CONFIG_BOARD_NUTTX_DEFCONFIG="nsh"  # 对应的 NuttX defconfig
```

### 3.2 第二步：NuttX 配置生成

CMake 会调用 NuttX 的配置系统：

```bash
# 路径: platforms/nuttx/NuttX/nuttx/
cd nuttx
make distclean

# 加载板级配置（defconfig）
# 示例: boards/arm/stm32h7/fmu-v6x/configs/nsh/defconfig
tools/configure.sh -l boards/arm/stm32h7/fmu-v6x/configs/nsh

# 生成 .config 文件 (NuttX 的配置)
make olddefconfig
```

**NuttX defconfig 关键配置**：

```makefile
# 架构配置
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32h7"
CONFIG_ARCH_CHIP_STM32H743ZI=y

# 内存配置
CONFIG_RAM_START=0x24000000    # SRAM1 起始地址
CONFIG_RAM_SIZE=524288         # 512KB SRAM

# 外设使能
CONFIG_STM32H7_SPI1=y          # SPI1 (连接 IMU)
CONFIG_STM32H7_I2C1=y          # I2C1 (连接磁力计)
CONFIG_STM32H7_USART1=y        # UART1 (GPS)

# 时钟配置
CONFIG_STM32H7_BOARD_HCLK=480000000  # 480 MHz
```

### 3.3 第三步：编译 NuttX

```bash
# NuttX 内核编译
cd platforms/nuttx/NuttX/nuttx
make

# 生成的重要文件：
# - nuttx (ELF 格式的内核)
# - libarch.a (架构相关代码)
# - libdrivers.a (NuttX 驱动)
# - libc.a (C 库)
```

**NuttX 编译产物**：

| 文件 | 说明 |
|------|------|
| `libarch.a` | ARM Cortex-M7 架构代码（中断、上下文切换）|
| `libboards.a` | 板级初始化代码（时钟、GPIO 配置）|
| `libdrivers.a` | NuttX 驱动（SPI、I2C、UART）|
| `libfs.a` | 文件系统（FAT、RomFS）|
| `libsched.a` | 任务调度器 |

### 3.4 第四步：编译 PX4 模块

```bash
# uORB 消息生成 (msg/*.msg → C++ 头文件)
python3 Tools/msg/px_generate_uorb_topic_files.py

# 编译各模块
cd src/modules/ekf2
arm-none-eabi-g++ -c EKF2.cpp -o EKF2.o \
  -mcpu=cortex-m7 -mthumb \
  -I../../platforms/nuttx/NuttX/include \
  -I../../build/uORB/topics

# 生成模块静态库
ar rcs libmodules__ekf2.a EKF2.o ekf2_main.o ...
```

**主要编译产物**：

```
build/px4_fmu-v6x_default/
├── uORB/
│   └── topics/                    # 自动生成的 uORB 消息头文件
│       ├── sensor_accel.h
│       └── vehicle_attitude.h
├── src/
│   ├── modules/
│   │   ├── libmodules__ekf2.a     # EKF2 模块
│   │   └── libmodules__mc_pos_control.a
│   ├── lib/
│   │   ├── liblib__matrix.a       # 矩阵运算库
│   │   └── liblib__geo.a          # 地理坐标转换
│   └── drivers/
│       └── libdrivers__imu.a      # IMU 驱动
└── platforms/nuttx/
    └── libplatforms__nuttx__px4_layer.a  # PX4-NuttX 平台层
```

### 3.5 第五步：链接 (Linking)

```bash
# 链接所有库生成 ELF 文件
arm-none-eabi-g++ \
  -T boards/px4/fmu-v6x/scripts/ld.script \  # 链接脚本
  -o px4_fmu-v6x_default.elf \
  libmodules__ekf2.a \
  libmodules__mc_pos_control.a \
  ... (所有 PX4 库) \
  libnuttx.a \                               # NuttX 内核
  libarch.a \                                # 架构代码
  -lm -lgcc -lstdc++                         # 标准库
```

**链接脚本解析** (`ld.script`)：

```ld
MEMORY
{
  /* STM32H743 内存布局 */
  flash (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K  /* Flash */
  sram1 (rwx) : ORIGIN = 0x24000000, LENGTH = 512K   /* SRAM1 */
  sram2 (rwx) : ORIGIN = 0x30000000, LENGTH = 288K   /* SRAM2 */
}

SECTIONS
{
  .text : {
    *(.vectors)        /* 中断向量表 (必须在 0x08000000) */
    *(.text)           /* 代码段 */
    *(.rodata)         /* 只读数据 (参数默认值) */
  } > flash

  .data : {
    *(.data)           /* 初始化数据 */
  } > sram1 AT > flash

  .bss : {
    *(.bss)            /* 未初始化数据 (全局变量) */
  } > sram1
}
```

### 3.6 第六步：固件后处理

```bash
# 1. 生成纯二进制文件 (用于烧录)
arm-none-eabi-objcopy -O binary \
  px4_fmu-v6x_default.elf \
  px4_fmu-v6x_default.bin

# 2. 添加 PX4 元数据，生成 .px4 文件
python3 Tools/px_mkfw.py \
  --prototype px4_fmu-v6x \
  --git_identity . \
  --image px4_fmu-v6x_default.bin \
  > px4_fmu-v6x_default.px4

# 3. 生成固件大小报告
arm-none-eabi-size px4_fmu-v6x_default.elf
```

**固件大小分析输出示例**：

```
   text    data     bss     dec     hex filename
 982304   12456  128432 1123192  112438 px4_fmu-v6x_default.elf

text  = 代码 + 常量数据 (存储在 Flash)
data  = 初始化数据 (Flash → RAM 复制)
bss   = 未初始化数据 (纯 RAM)
```

---

## 4. 固件组成与生成

### 4.1 固件文件类型对比

| 文件扩展名 | 格式 | 包含内容 | 用途 |
|-----------|------|---------|------|
| `.elf` | ELF 可执行文件 | 代码 + 数据 + 调试符号 + 段信息 | GDB 调试 |
| `.bin` | 纯二进制 | 仅代码和数据 (无元数据) | 直接烧录 |
| `.px4` | PX4 固件包 | `.bin` + JSON 元数据 | QGroundControl 烧录 |
| `.hex` | Intel HEX | 带地址的十六进制文本 | 某些烧录工具 |

### 4.2 .px4 固件包结构

```json
{
  "board_id": 50,                    // 板级 ID (Pixhawk 6X)
  "board_revision": 0,
  "version": "1.14.0",               // PX4 版本
  "build_time": 1732588800,          // 编译时间戳
  "git_hash": "a1b2c3d4e5f6...",    // Git 提交哈希
  "image_size": 982304,              // 固件大小 (字节)
  "image": "<base64 编码的二进制数据>",
  "summary": "PX4FMU_V6X"
}
```

**为什么需要 .px4 格式？**

1. **版本校验**：QGroundControl 会检查 `board_id` 是否匹配
2. **防止刷错固件**：避免把 Pixhawk 4 的固件刷到 Pixhawk 6X
3. **可追溯性**：包含 Git 哈希，可以追溯源码版本

### 4.3 固件内存布局

**Flash 布局** (STM32H743, 2MB Flash)：

```
0x08000000  ┌─────────────────────────┐
            │  Bootloader (16KB)      │  ← 启动引导程序
0x08004000  ├─────────────────────────┤
            │  Application (固件)      │
            │  • 中断向量表            │
            │  • .text (代码段)        │  ← arm-none-eabi-gcc 生成
            │  • .rodata (常量)        │
            │  • .data (初始化数据)    │
0x081XXXXX  ├─────────────────────────┤
            │  Parameters (参数区)     │  ← 持久化存储的参数
0x08200000  └─────────────────────────┘
```

**RAM 布局** (STM32H743, 512KB SRAM1)：

```
0x24000000  ┌─────────────────────────┐
            │  .data (已初始化变量)    │
            ├─────────────────────────┤
            │  .bss (未初始化变量)     │
            ├─────────────────────────┤
            │  Heap (动态内存)         │
            ├─────────────────────────┤
            │  Stack (栈空间)          │
0x2407FFFF  └─────────────────────────┘
```

---

## 5. 板级支持包（BSP）

### 5.1 PX4 板级配置文件 (.px4board)

**示例**: `boards/px4/fmu-v6x/default.px4board`

```python
# 板级硬件信息
CONFIG_BOARD_VENDOR="px4"
CONFIG_BOARD_MODEL="fmu-v6x"
CONFIG_BOARD_LABEL="default"

# 平台选择
CONFIG_PLATFORM_NUTTX=y
CONFIG_BOARD_ARCHITECTURE="cortex-m7"
CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"

# 芯片信息
CONFIG_BOARD_CONSTRAINED_FLASH=n     # Flash 充足 (2MB)
CONFIG_BOARD_CONSTRAINED_MEMORY=n    # RAM 充足 (512KB)

# 启用的驱动（每个驱动对应 src/drivers/ 下的一个模块）
CONFIG_DRIVERS_IMU_INVENSENSE_ICM20948=y      # IMU 传感器
CONFIG_DRIVERS_BAROMETER_MS5611=y             # 气压计
CONFIG_DRIVERS_MAGNETOMETER_LIS3MDL=y         # 磁力计
CONFIG_DRIVERS_GPS=y                          # GPS 驱动

# 启用的模块（每个模块对应 src/modules/ 下的一个目录）
CONFIG_MODULES_EKF2=y                         # EKF2 状态估计
CONFIG_MODULES_MC_POS_CONTROL=y               # 多旋翼位置控制
CONFIG_MODULES_MC_ATT_CONTROL=y               # 多旋翼姿态控制
CONFIG_MODULES_MAVLINK=y                      # MAVLink 通信

# 库
CONFIG_LIB_MATHLIB=y
CONFIG_LIB_MATRIX=y

# 系统命令
CONFIG_SYSTEMCMDS_PARAM=y
CONFIG_SYSTEMCMDS_TOP=y
```

### 5.2 NuttX 板级配置 (defconfig)

**路径**: `platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/fmu-v6x/configs/nsh/defconfig`

```makefile
# 架构配置
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32h7"
CONFIG_ARCH_CHIP_STM32H743ZI=y           # 具体芯片型号
CONFIG_ARCH_CORTEXM7=y
CONFIG_ARCH_FPU=y                        # 启用硬件浮点

# 时钟配置 (对应 CubeMX 的时钟树)
CONFIG_STM32H7_BOARD_HCLK=480000000      # 系统主频 480MHz
CONFIG_STM32H7_HSE_FREQUENCY=25000000    # 外部晶振 25MHz
CONFIG_STM32H7_VCO_FREQUENCY=960000000   # PLL VCO 频率

# Flash 配置
CONFIG_STM32H7_FLASH_CONFIG_DEFAULT=y
CONFIG_STM32H7_FLASH_PREFETCH=y

# SPI 外设 (连接 IMU)
CONFIG_STM32H7_SPI1=y
CONFIG_STM32H7_SPI2=y

# I2C 外设 (连接磁力计)
CONFIG_STM32H7_I2C1=y
CONFIG_STM32H7_I2C2=y

# UART 外设
CONFIG_STM32H7_USART1=y                  # GPS
CONFIG_STM32H7_USART2=y                  # 遥测
CONFIG_STM32H7_USART3=y                  # TELEM2

# DMA 配置
CONFIG_STM32H7_DMA1=y
CONFIG_STM32H7_DMA2=y

# GPIO 配置
CONFIG_STM32H7_GPIOA=y
CONFIG_STM32H7_GPIOB=y
# ... (其他 GPIO 端口)

# NuttX 系统配置
CONFIG_SCHED_HPWORK=y                    # 高优先级工作队列
CONFIG_SCHED_LPWORK=y                    # 低优先级工作队列
CONFIG_DEV_URANDOM=y                     # 随机数生成器
CONFIG_FS_ROMFS=y                        # RomFS 文件系统
CONFIG_FS_FAT=y                          # FAT 文件系统
```

### 5.3 板级初始化代码

**NuttX 板级初始化**: `platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/fmu-v6x/src/`

```c
// stm32_boot.c - 最早执行的板级初始化
void stm32_boardinitialize(void)
{
    // 1. 配置 LED (方便调试)
    stm32_configgpio(GPIO_LED1);

    // 2. 配置 SPI 片选引脚
    stm32_configgpio(GPIO_SPI1_CS_ICM20948);  // IMU 片选

    // 3. 配置电源控制引脚
    stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
    stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, true);  // 使能传感器供电
}

// stm32_spi.c - SPI 总线初始化
void stm32_spidev_initialize(void)
{
    // 配置 SPI1 (连接 IMU)
    stm32_configgpio(GPIO_SPI1_SCK);   // 时钟
    stm32_configgpio(GPIO_SPI1_MISO);  // 主入从出
    stm32_configgpio(GPIO_SPI1_MOSI);  // 主出从入
}
```

**PX4 板级初始化**: `boards/px4/fmu-v6x/src/`

```cpp
// board_config.h - 板级配置宏定义
#define BOARD_HAS_PWM    DIRECT_PWM_OUTPUT_CHANNELS  // PWM 输出通道数
#define BOARD_NUMBER_I2C_BUSES  4                     // I2C 总线数量

// GPIO 引脚定义 (对应硬件原理图)
#define GPIO_LED1        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN0)
#define GPIO_SPI1_CS_ICM20948  (GPIO_OUTPUT | GPIO_PORTD | GPIO_PIN7)

// init.c - PX4 层级的板级初始化
int board_app_initialize(uintptr_t arg)
{
    // 1. 初始化 SPI 总线
    px4_platform_init();

    // 2. 启动板级传感器
    board_spi_reset();

    // 3. 配置 I/O 定时器 (用于 PWM 输出)
    board_io_timer_init();

    return OK;
}
```

---

## 6. CMake 构建系统深度剖析

### 6.1 CMake 构建流程图

```
顶层 CMakeLists.txt
    │
    ├─→ 检测配置 (CONFIG=px4_fmu-v6x_default)
    │
    ├─→ 加载板级文件 boards/px4/fmu-v6x/default.px4board
    │
    ├─→ 设置工具链 cmake/toolchains/Toolchain-arm-none-eabi.cmake
    │
    ├─→ 配置 NuttX
    │   └─→ platforms/nuttx/NuttX/CMakeLists.txt
    │       └─→ 调用 NuttX 的 Makefile
    │
    ├─→ 生成 uORB 消息
    │   └─→ msg/CMakeLists.txt
    │       └─→ python Tools/msg/px_generate_uorb_topic_files.py
    │
    ├─→ 编译 PX4 模块
    │   ├─→ src/modules/ekf2/CMakeLists.txt
    │   ├─→ src/modules/mc_pos_control/CMakeLists.txt
    │   └─→ ... (根据 .px4board 启用的模块)
    │
    ├─→ 编译驱动
    │   ├─→ src/drivers/imu/invensense/icm20948/CMakeLists.txt
    │   └─→ ... (根据 .px4board 启用的驱动)
    │
    ├─→ 链接最终固件
    │   └─→ cmake/packaging/Linux.cmake (或 Windows/macOS)
    │
    └─→ 生成 .px4 固件包
        └─→ Tools/px_mkfw.py
```

### 6.2 关键 CMake 函数

**px4_add_module()** - 添加 PX4 模块

```cmake
# src/modules/ekf2/CMakeLists.txt
px4_add_module(
    MODULE modules__ekf2           # 模块名称
    MAIN ekf2                       # 主函数入口（ekf2_main）
    COMPILE_FLAGS
        -Wno-cast-align             # 编译选项
    SRCS
        EKF2.cpp                    # 源文件列表
        EKF2.hpp
        EKF2Selector.cpp
        Utility/PreFlightChecker.cpp
    MODULE_CONFIG
        module.yaml                 # 模块配置（参数定义）
    DEPENDS
        drivers_accelerometer       # 依赖的其他模块
        drivers_gyroscope
        ecl                         # ECL (Estimation & Control Library)
)
```

**px4_add_board()** - 定义板级目标

```cmake
# boards/px4/fmu-v6x/default.cmake
px4_add_board(
    PLATFORM nuttx
    VENDOR px4
    MODEL fmu-v6x
    LABEL default
    TOOLCHAIN arm-none-eabi
    ARCHITECTURE cortex-m7

    DRIVERS
        imu/invensense/icm20948
        barometer/ms5611
        magnetometer/lis3mdl
        gps

    MODULES
        ekf2
        mc_pos_control
        mc_att_control
        mavlink

    SYSTEMCMDS
        param
        top
        ver
)
```

### 6.3 构建系统文件结构

```
PX4-Autopilot/
├── CMakeLists.txt                        # 顶层 CMake 文件
├── cmake/
│   ├── common/
│   │   ├── px_base.cmake                 # px4_add_module 等函数定义
│   │   └── px_macros.cmake
│   ├── toolchains/
│   │   ├── Toolchain-arm-none-eabi.cmake # ARM 工具链配置
│   │   └── Toolchain-native.cmake        # SITL 工具链
│   └── packaging/
│       └── Linux.cmake                   # 固件打包脚本
├── boards/
│   └── px4/fmu-v6x/
│       ├── default.px4board              # 板级配置 (Kconfig 格式)
│       ├── default.cmake                 # 板级 CMake 配置
│       └── src/                          # 板级源码
│           ├── board_config.h
│           └── init.c
└── platforms/nuttx/
    ├── CMakeLists.txt                    # NuttX 平台的 CMake 入口
    └── NuttX/
        └── nuttx/
            └── boards/arm/stm32h7/fmu-v6x/
                └── configs/nsh/defconfig  # NuttX 配置
```

---

## 7. 实战：为 Nucleo-H743ZI 构建固件

### 7.1 核心概念澄清

**你需要做的（正确流程）**：

1. ✅ 在 PX4 源码中创建新的板级配置文件
2. ✅ 配置 NuttX defconfig（时钟、外设等）
3. ✅ 编写板级初始化代码（GPIO、SPI、I2C 等）
4. ✅ 使用 PX4 的 CMake 系统构建

**你不需要做的（错误流程）**：

1. ❌ 使用 CubeMX 创建项目
2. ❌ 手动复制 PX4 代码到 CubeMX 项目
3. ❌ 从零编写 Makefile

### 7.2 Nucleo-H743ZI 硬件差异分析

**与 Pixhawk 6X (STM32H743) 的差异**：

| 特性 | Nucleo-H743ZI | Pixhawk 6X |
|------|---------------|------------|
| 外部晶振 | 8MHz HSE | 25MHz HSE |
| IMU | **无** (需外接) | ICM20948 (SPI) |
| 磁力计 | **无** | LIS3MDL (I2C) |
| GPS | **无** | u-blox (UART) |
| PWM 输出 | Arduino 接口 | 8 路专用 PWM |
| LED | 3 个用户 LED | 自定义 LED |
| USB | USB OTG FS | USB OTG FS |

### 7.3 创建 Nucleo-H743ZI 板级配置

#### 步骤 1: 创建板级目录

```bash
cd PX4-Autopilot

# 创建板级目录结构
mkdir -p boards/st/nucleo-h743zi/
cd boards/st/nucleo-h743zi/

# 创建必要文件
touch default.px4board         # PX4 板级配置
touch default.cmake            # CMake 配置
mkdir -p src                   # 板级源码目录
```

#### 步骤 2: 编写 .px4board 配置

**文件**: `boards/st/nucleo-h743zi/default.px4board`

```python
# 板级标识
CONFIG_BOARD_VENDOR="st"
CONFIG_BOARD_MODEL="nucleo-h743zi"
CONFIG_BOARD_LABEL="default"

# 平台配置
CONFIG_PLATFORM_NUTTX=y
CONFIG_BOARD_ARCHITECTURE="cortex-m7"
CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"

# 资源受限配置（Nucleo 板没有外部传感器，减少模块）
CONFIG_BOARD_CONSTRAINED_FLASH=n
CONFIG_BOARD_CONSTRAINED_MEMORY=n

# 基础驱动（只启用 Nucleo 板上有的）
CONFIG_DRIVERS_LED=y
CONFIG_DRIVERS_TONE_ALARM=n            # Nucleo 无蜂鸣器

# 最小化模块集（用于测试）
CONFIG_MODULES_MAVLINK=y               # MAVLink 通信
CONFIG_SYSTEMCMDS_PARAM=y              # 参数系统
CONFIG_SYSTEMCMDS_VER=y                # 版本命令
CONFIG_SYSTEMCMDS_TOP=y                # 任务监控

# 不启用需要外部传感器的模块
CONFIG_MODULES_EKF2=n                  # 没有 IMU，无法运行 EKF2
CONFIG_MODULES_SENSORS=n               # 没有传感器驱动

# 启用 uORB (即使没有传感器，也需要测试消息总线)
CONFIG_COMMON_UORB=y
```

#### 步骤 3: 编写板级 CMake 配置

**文件**: `boards/st/nucleo-h743zi/default.cmake`

```cmake
px4_add_board(
    PLATFORM nuttx
    VENDOR st
    MODEL nucleo-h743zi
    LABEL default
    TOOLCHAIN arm-none-eabi
    ARCHITECTURE cortex-m7

    # NuttX 配置名称
    NUTTX_CONFIG nsh

    # 启用的模块（最小化配置）
    MODULES
        mavlink

    SYSTEMCMDS
        param
        ver
        top
        nshterm

    # 库
    LIBRARIES
        mathlib
        matrix
)
```

#### 步骤 4: 配置 NuttX defconfig

**路径**: `platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/nucleo-h743zi/configs/nsh/defconfig`

```makefile
# 架构
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32h7"
CONFIG_ARCH_CHIP_STM32H743ZI=y
CONFIG_ARCH_CORTEXM7=y
CONFIG_ARCH_FPU=y

# ⚠️ 关键差异：Nucleo 使用 8MHz 晶振（非 25MHz）
CONFIG_STM32H7_HSE_FREQUENCY=8000000      # 8MHz 外部晶振
CONFIG_STM32H7_BOARD_HCLK=480000000       # 目标主频 480MHz

# PLL 配置（需要根据 8MHz 晶振重新计算）
# HSE (8MHz) → PLLM (/2 = 4MHz) → PLLN (×240 = 960MHz) → PLLP (/2 = 480MHz)
CONFIG_STM32H7_PLLCFG_PLLM=2
CONFIG_STM32H7_PLLCFG_PLLN=240
CONFIG_STM32H7_PLLCFG_PLLP=2

# UART (Nucleo 板的 USART3 连接到 ST-Link 虚拟串口)
CONFIG_STM32H7_USART3=y
CONFIG_USART3_SERIAL_CONSOLE=y
CONFIG_USART3_BAUD=115200

# USB
CONFIG_STM32H7_OTGFS=y

# LED (Nucleo 有 3 个用户 LED)
# LD1: PB0 (绿色)
# LD2: PE1 (黄色)
# LD3: PB14 (红色)

# 文件系统
CONFIG_FS_ROMFS=y

# 工作队列
CONFIG_SCHED_HPWORK=y
CONFIG_SCHED_LPWORK=y
```

#### 步骤 5: 编写板级初始化代码

**文件**: `boards/st/nucleo-h743zi/src/board_config.h`

```cpp
#pragma once

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

// LED 定义（参考 Nucleo-H743ZI 原理图）
#define GPIO_LED1  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                    GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN0)   // LD1 (绿)

#define GPIO_LED2  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                    GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN1)   // LD2 (黄)

#define GPIO_LED3  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                    GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN14)  // LD3 (红)

// 按钮定义
#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN13)

// UART 控制台
#define BOARD_CONSOLE_UART  3  // USART3 连接到 ST-Link
```

**文件**: `boards/st/nucleo-h743zi/src/init.c`

```c
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <nuttx/board.h>
#include "board_config.h"

__EXPORT void board_peripheral_reset(int ms)
{
    // Nucleo 板没有外部传感器，此函数留空
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
    // 配置 LED
    px4_arch_configgpio(GPIO_LED1);
    px4_arch_configgpio(GPIO_LED2);
    px4_arch_configgpio(GPIO_LED3);

    // 闪烁 LED 表示启动成功
    px4_arch_gpiowrite(GPIO_LED1, true);
    usleep(100000);
    px4_arch_gpiowrite(GPIO_LED1, false);

    return OK;
}
```

#### 步骤 6: 构建固件

```bash
cd PX4-Autopilot

# 清理之前的构建
make clean

# 构建 Nucleo-H743ZI 固件
make st_nucleo-h743zi_default

# 构建输出路径
ls build/st_nucleo-h743zi_default/
# 应该看到:
# - st_nucleo-h743zi_default.elf
# - st_nucleo-h743zi_default.bin
# - st_nucleo-h743zi_default.px4
```

### 7.4 烧录固件到 Nucleo 板

**方法 1: 使用 st-flash (Linux/macOS)**

```bash
# 安装 stlink 工具
# Ubuntu: sudo apt-get install stlink-tools
# macOS: brew install stlink

# 烧录固件
st-flash write build/st_nucleo-h743zi_default/st_nucleo-h743zi_default.bin 0x08000000
```

**方法 2: 使用 STM32CubeProgrammer (跨平台)**

1. 连接 Nucleo 板的 USB ST-LINK 接口
2. 打开 STM32CubeProgrammer
3. 选择 ST-LINK 连接方式
4. 点击 "Connect"
5. 选择 `.bin` 文件：`build/st_nucleo-h743zi_default/st_nucleo-h743zi_default.bin`
6. 起始地址：`0x08000000`
7. 点击 "Start Programming"

**方法 3: 使用 make 自动上传**

```bash
# 需要配置 upload 目标（在 boards/st/nucleo-h743zi/default.cmake 中添加）
make st_nucleo-h743zi_default upload
```

### 7.5 验证固件运行

```bash
# 连接到 USART3 虚拟串口（115200 波特率）
# Linux: /dev/ttyACM0
# Windows: COMX (在设备管理器中查看)

# 使用 minicom 或 screen
screen /dev/ttyACM0 115200

# 应该看到 PX4 启动日志
```

预期输出：
```
NuttShell (NSH) NuttX-10.2.0
nsh>

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [px4] Startup script: /etc/init.d/rcS
INFO  [mavlink] MAVLink started
INFO  [uORB] uORB started

nsh> ver
PX4 git hash: a1b2c3d4
Build time: Nov 26 2025 12:00:00
Board: ST Nucleo-H743ZI

nsh> top
```

---

## 8. 调试构建问题

### 8.1 常见错误及解决方案

#### 错误 1: 找不到工具链

```
CMake Error: CMAKE_C_COMPILER not set, after EnableLanguage
```

**解决**：
```bash
# 检查工具链是否在 PATH 中
which arm-none-eabi-gcc

# 如果没有，添加到 PATH
export PATH=/opt/gcc-arm-none-eabi/bin:$PATH
```

#### 错误 2: NuttX 配置失败

```
No rule to make target 'nuttx/boards/arm/stm32h7/nucleo-h743zi/configs/nsh/defconfig'
```

**解决**：
1. 检查 defconfig 文件是否存在
2. 运行 `make distclean` 清理缓存

#### 错误 3: 链接时内存溢出

```
arm-none-eabi-ld: region `flash' overflowed by 12345 bytes
```

**解决**：
1. 减少启用的模块（修改 `.px4board`）
2. 启用编译优化 `-Os`（在 CMake 中配置）

#### 错误 4: uORB 消息未生成

```
fatal error: uORB/topics/sensor_accel.h: No such file or directory
```

**解决**：
```bash
# 手动触发 uORB 消息生成
python3 Tools/msg/px_generate_uorb_topic_files.py --topic-msg-dir msg

# 或清理重新构建
make clean
make st_nucleo-h743zi_default
```

### 8.2 调试技巧

**1. 查看详细构建日志**：
```bash
VERBOSE=1 make st_nucleo-h743zi_default
```

**2. 单独构建 NuttX**：
```bash
cd platforms/nuttx/NuttX/nuttx
make menuconfig  # 图形化配置界面
make
```

**3. 检查固件大小**：
```bash
arm-none-eabi-size build/st_nucleo-h743zi_default/st_nucleo-h743zi_default.elf
```

**4. 反汇编代码**：
```bash
arm-none-eabi-objdump -d build/st_nucleo-h743zi_default/st_nucleo-h743zi_default.elf > firmware.asm
```

---

## 总结

**PX4 构建系统的核心要点**：

1. ✅ **不需要 CubeMX**：PX4 有完整的 CMake 构建系统
2. ✅ **NuttX 作为子模块**：已包含 STM32 HAL 和启动代码
3. ✅ **板级配置通过 .px4board**：定义启用的模块和驱动
4. ✅ **工具链使用 arm-none-eabi-gcc**：自动交叉编译
5. ✅ **固件是单一文件**：`.elf` / `.bin` / `.px4` 格式

**Nucleo-H743ZI 开发关键步骤**：

1. 创建板级配置文件 (`boards/st/nucleo-h743zi/`)
2. 配置 NuttX defconfig（注意 8MHz 晶振）
3. 编写板级初始化代码（LED、UART 等）
4. 使用 `make st_nucleo-h743zi_default` 构建
5. 使用 st-flash 或 STM32CubeProgrammer 烧录

**下一步学习**：

- 阅读 `.trae/documents/rtos/index.md` 了解 NuttX 集成
- 参考 `.trae/documents/通用基础系统/stm32h743_minimal_flight_controller_guide.md`
- 研究现有板级配置（如 `boards/px4/fmu-v6x/`）
