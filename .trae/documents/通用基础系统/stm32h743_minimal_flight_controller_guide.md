---
文档版本: 1.0
适用PX4版本: v1.14.x - v1.15.x (通用)
最后更新: 2025-11-26
文档类型: 硬件移植指南
难度等级: ⭐⭐⭐⭐ (高级)
前置要求: STM32 HAL, NuttX RTOS, CMake 基础
预计学习时间: 8-12 小时
---

# 从零构建 STM32H743 最小化飞控系统完全指南

## 文档概述

本指南将**手把手**教你如何在 **STM32H743I 官方开发板**上，从零开始构建一个**简化版飞控系统**，包含：

- ✅ **NuttX RTOS**（实时操作系统）
- ✅ **uORB**（发布-订阅消息总线）
- ✅ **SPI IMU 驱动**（惯性测量单元）
- ✅ **I2C 磁力计驱动**
- ✅ **传感器融合应用**
- ✅ **完整构建系统**（CMake + 固件生成）

**目标**: 生成一个类似 `.px4` 的单一固件文件，包含 RTOS + 驱动 + 应用代码。

**开发板**: STM32H743I-EVAL 或 STM32H743I-DISCO（通用方案）

---

## 目录

1. [项目概述与架构](#1-项目概述与架构)
2. [开发环境搭建](#2-开发环境搭建)
3. [项目目录结构](#3-项目目录结构)
4. [NuttX RTOS 集成](#4-nuttx-rtos-集成)
5. [uORB 消息总线移植](#5-uorb-消息总线移植)
6. [SPI IMU 驱动开发](#6-spi-imu-驱动开发)
7. [I2C 磁力计驱动开发](#7-i2c-磁力计驱动开发)
8. [传感器融合应用](#8-传感器融合应用)
9. [构建系统（CMake）](#9-构建系统cmake)
10. [固件生成与烧录](#10-固件生成与烧录)
11. [调试方法](#11-调试方法)
12. [常见问题](#12-常见问题)

---

## 1. 项目概述与架构

### 1.1 系统架构

```
┌───────────────────────────────────────────────────────────┐
│                    应用层 (Application)                    │
├───────────────────────────────────────────────────────────┤
│  sensor_fusion_app                                        │
│  ├─ 订阅: sensor_accel, sensor_gyro, sensor_mag          │
│  └─ 发布: vehicle_attitude                               │
└───────────────────────────────────────────────────────────┘
                            ▲
                            │ uORB 消息
                            ▼
┌───────────────────────────────────────────────────────────┐
│                  uORB 消息总线 (Middleware)                │
├───────────────────────────────────────────────────────────┤
│  - 发布/订阅机制                                          │
│  - 多对多通信                                             │
│  - 零拷贝共享内存                                         │
└───────────────────────────────────────────────────────────┘
                            ▲
                            │
                            ▼
┌───────────────────────────────────────────────────────────┐
│                    驱动层 (Drivers)                        │
├───────────────────────────────────────────────────────────┤
│  imu_spi_driver          mag_i2c_driver                   │
│  ├─ ICM42688P (SPI1)     ├─ IST8310 (I2C1)               │
│  └─ 发布: sensor_accel   └─ 发布: sensor_mag             │
│           sensor_gyro                                     │
└───────────────────────────────────────────────────────────┘
                            ▲
                            │
                            ▼
┌───────────────────────────────────────────────────────────┐
│                  平台抽象层 (Platform)                     │
├───────────────────────────────────────────────────────────┤
│  - SPI/I2C HAL 封装                                       │
│  - GPIO/Timer 控制                                        │
│  - 工作队列调度                                           │
└───────────────────────────────────────────────────────────┘
                            ▲
                            │
                            ▼
┌───────────────────────────────────────────────────────────┐
│                  NuttX RTOS (Kernel)                      │
├───────────────────────────────────────────────────────────┤
│  - 任务调度                                               │
│  - 中断管理                                               │
│  - 内存管理                                               │
│  - 文件系统                                               │
└───────────────────────────────────────────────────────────┘
                            ▲
                            │
                            ▼
┌───────────────────────────────────────────────────────────┐
│                  硬件 (STM32H743I)                         │
├───────────────────────────────────────────────────────────┤
│  - ARM Cortex-M7 @ 480 MHz                                │
│  - 2 MB Flash, 1 MB RAM                                   │
│  - SPI, I2C, UART, Timer...                               │
└───────────────────────────────────────────────────────────┘
```

### 1.2 核心模块

| 模块 | 功能 | 来源 | 代码量 |
|------|------|------|--------|
| **NuttX RTOS** | 实时操作系统 | Apache NuttX | ~500K 行 |
| **uORB** | 消息总线 | 从 PX4 提取 | ~2K 行 |
| **IMU 驱动** | SPI 传感器驱动 | 自己开发 | ~500 行 |
| **磁力计驱动** | I2C 传感器驱动 | 自己开发 | ~300 行 |
| **传感器融合** | 姿态解算 | 自己开发 | ~400 行 |
| **构建系统** | CMake 脚本 | 自己开发 | ~200 行 |

### 1.3 技术栈

```
编程语言:    C/C++
构建系统:    CMake 3.20+
RTOS:        NuttX 12.7.0+
工具链:      arm-none-eabi-gcc 13.2+
调试器:      OpenOCD + GDB
IDE:         VSCode + Cortex-Debug 插件
版本控制:    Git
```

---

## 2. 开发环境搭建

### 2.1 开发工具选择

**⚠️ 重要：不使用 STM32CubeMX！**

原因：
- CubeMX 生成的 HAL 库与 NuttX 冲突
- NuttX 有自己的硬件抽象层
- CubeMX 不支持 RTOS 深度定制

**正确的开发流程**:

```
1. 直接使用 NuttX 的 STM32H7 BSP
2. 在 NuttX 配置系统 (Kconfig) 中配置引脚和外设
3. 使用 VSCode + CMake 进行开发
4. 使用 OpenOCD 进行调试和烧录
```

### 2.2 Windows 环境搭建

#### 2.2.1 安装 MSYS2（推荐）

```powershell
# 1. 下载 MSYS2
# 访问: https://www.msys2.org/
# 下载: msys2-x86_64-<version>.exe

# 2. 安装后，打开 MSYS2 MINGW64 终端

# 3. 更新包管理器
pacman -Syu

# 4. 安装开发工具
pacman -S base-devel
pacman -S git
pacman -S cmake
pacman -S python3
pacman -S python3-pip
pacman -S make
pacman -S vim
```

#### 2.2.2 安装 ARM 工具链

```bash
# 在 MSYS2 终端中

# 1. 下载 ARM GCC（从官方网站）
# https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
# 下载: arm-gnu-toolchain-13.2.rel1-mingw-w64-i686-arm-none-eabi.exe

# 2. 安装到默认路径（例如 C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\13.2 Rel1）

# 3. 添加到 PATH（在 MSYS2 中）
echo 'export PATH="/c/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.2 Rel1/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# 4. 验证
arm-none-eabi-gcc --version
# 预期输出: arm-none-eabi-gcc (Arm GNU Toolchain 13.2.Rel1 ...) 13.2.1
```

#### 2.2.3 安装 OpenOCD

```bash
# 在 MSYS2 终端中
pacman -S mingw-w64-x86_64-openocd

# 验证
openocd --version
# 预期输出: Open On-Chip Debugger 0.12.0
```

#### 2.2.4 安装 Python 依赖

```bash
pip3 install kconfiglib
pip3 install pyelftools
pip3 install crcmod
pip3 install pyserial
```

### 2.3 VSCode 配置

#### 2.3.1 安装 VSCode

下载并安装: https://code.visualstudio.com/

#### 2.3.2 安装必要插件

在 VSCode 中按 `Ctrl+Shift+X`，搜索并安装：

```
1. C/C++ (Microsoft)
2. CMake Tools (Microsoft)
3. Cortex-Debug (marus25)
4. Serial Monitor (Microsoft)
5. Hex Editor (Microsoft)
```

#### 2.3.3 配置 settings.json

在 VSCode 中按 `Ctrl+Shift+P`，输入 "Preferences: Open User Settings (JSON)"：

```json
{
    "cmake.configureOnOpen": false,
    "C_Cpp.default.compilerPath": "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.2 Rel1/bin/arm-none-eabi-gcc.exe",
    "C_Cpp.default.intelliSenseMode": "gcc-arm",
    "terminal.integrated.defaultProfile.windows": "MSYS2",
    "terminal.integrated.profiles.windows": {
        "MSYS2": {
            "path": "C:\\msys64\\usr\\bin\\bash.exe",
            "args": ["--login"],
            "env": {
                "MSYSTEM": "MINGW64",
                "CHERE_INVOKING": "1"
            }
        }
    }
}
```

### 2.4 硬件准备

#### 2.4.1 STM32H743I 开发板

推荐型号：
- **STM32H743I-EVAL** (官方评估板)
- **STM32H743I-DISCO** (探索套件)
- **Nucleo-H743ZI** (Nucleo 板)

#### 2.4.2 调试器

- **ST-Link V3** (推荐，官方调试器)
- **J-Link** (备选，更强大)
- 开发板通常自带 ST-Link

#### 2.4.3 传感器模块（可选）

| 传感器 | 型号 | 接口 | 购买链接示例 |
|--------|------|------|-------------|
| IMU | ICM42688P | SPI | 淘宝搜索 "ICM42688P 模块" |
| 磁力计 | IST8310 | I2C | 淘宝搜索 "IST8310 模块" |
| 气压计 | BMP388 | I2C/SPI | 可选 |

**接线参考**（稍后详细说明）：

```
STM32H743 ─────── ICM42688P (IMU)
  SPI1_SCK  ────►  SCK
  SPI1_MISO ◄────  SDO
  SPI1_MOSI ────►  SDA
  SPI1_NSS  ────►  CS
  GND       ────►  GND
  3.3V      ────►  VCC

STM32H743 ─────── IST8310 (Mag)
  I2C1_SCL  ────►  SCL
  I2C1_SDA  ◄───►  SDA
  GND       ────►  GND
  3.3V      ────►  VCC
```

---

## 3. 项目目录结构

### 3.1 完整目录树

```
MiniFlight/                          # 项目根目录
├── CMakeLists.txt                   # 顶层 CMake 文件
├── .vscode/                         # VSCode 配置
│   ├── launch.json                  # 调试配置
│   ├── tasks.json                   # 任务配置
│   └── c_cpp_properties.json        # C/C++ 配置
├── platforms/                       # 平台抽象层
│   └── nuttx/                       # NuttX 平台
│       ├── CMakeLists.txt
│       ├── NuttX/                   # NuttX 源码（Git 子模块）
│       │   ├── nuttx/               # NuttX 内核
│       │   └── apps/                # NuttX 应用
│       ├── cmake/                   # NuttX 构建脚本
│       │   ├── init.cmake
│       │   ├── platform.cmake
│       │   └── Toolchain-arm.cmake
│       └── src/                     # 平台特定代码
│           ├── common/              # 通用代码
│           │   ├── hrt.c            # 高精度定时器
│           │   ├── gpio.c           # GPIO 封装
│           │   └── spi_wrapper.c    # SPI 封装
│           └── stm32h7/             # STM32H7 特定代码
│               └── board_config.h   # 板级配置
├── boards/                          # 板级支持包
│   └── stm32h743i_eval/             # STM32H743I-EVAL 板
│       ├── nuttx-config/            # NuttX 配置
│       │   ├── include/
│       │   │   └── board.h          # 时钟、引脚配置
│       │   ├── scripts/
│       │   │   └── script.ld        # 链接脚本
│       │   └── nsh/
│       │       └── defconfig        # NuttX 配置文件
│       └── default.cmake            # 板级 CMake 配置
├── src/                             # 应用和驱动代码
│   ├── drivers/                     # 驱动层
│   │   ├── imu/                     # IMU 驱动
│   │   │   ├── CMakeLists.txt
│   │   │   ├── icm42688p.h
│   │   │   └── icm42688p.cpp
│   │   └── mag/                     # 磁力计驱动
│   │       ├── CMakeLists.txt
│   │       ├── ist8310.h
│   │       └── ist8310.cpp
│   ├── lib/                         # 库代码
│   │   ├── uorb/                    # uORB 消息总线
│   │   │   ├── CMakeLists.txt
│   │   │   ├── uORB.h
│   │   │   ├── uORBManager.cpp
│   │   │   ├── Publication.h
│   │   │   └── Subscription.h
│   │   └── math/                    # 数学库（矩阵、四元数等）
│   │       ├── CMakeLists.txt
│   │       ├── matrix.h
│   │       └── quaternion.h
│   ├── modules/                     # 应用模块
│   │   └── sensor_fusion/           # 传感器融合
│   │       ├── CMakeLists.txt
│   │       ├── sensor_fusion.cpp
│   │       └── sensor_fusion.h
│   └── msg/                         # uORB 消息定义
│       ├── sensor_accel.msg
│       ├── sensor_gyro.msg
│       ├── sensor_mag.msg
│       └── vehicle_attitude.msg
├── cmake/                           # CMake 工具脚本
│   ├── generate_msg.cmake           # 消息生成脚本
│   └── upload.cmake                 # 固件上传脚本
├── Tools/                           # 工具脚本
│   ├── msg/                         # 消息代码生成器
│   │   ├── generate_msg.py
│   │   └── templates/
│   └── upload_fw.sh                 # 固件上传脚本
└── build/                           # 构建输出（自动生成）
    └── stm32h743i_eval_default/
        ├── nuttx                    # NuttX ELF 文件
        ├── nuttx.bin                # 原始二进制
        └── miniflight.px4           # 打包后的固件
```

### 3.2 创建项目骨架

```bash
# 1. 创建项目根目录
mkdir -p MiniFlight
cd MiniFlight

# 2. 创建目录结构
mkdir -p platforms/nuttx/{cmake,src/{common,stm32h7}}
mkdir -p boards/stm32h743i_eval/nuttx-config/{include,scripts,nsh}
mkdir -p src/{drivers/{imu,mag},lib/{uorb,math},modules/sensor_fusion,msg}
mkdir -p cmake
mkdir -p Tools/{msg/templates}
mkdir -p .vscode

# 3. 初始化 Git
git init

# 4. 添加 NuttX 作为子模块
git submodule add https://github.com/apache/nuttx.git platforms/nuttx/NuttX/nuttx
git submodule add https://github.com/apache/nuttx-apps.git platforms/nuttx/NuttX/apps

# 5. 初始化子模块
git submodule update --init --recursive
```

---

## 4. NuttX RTOS 集成

### 4.1 配置 NuttX 内核

#### 4.1.1 创建板级配置（defconfig）

**文件**: `boards/stm32h743i_eval/nuttx-config/nsh/defconfig`

```makefile
# ============ 架构配置 ============
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32h7"
CONFIG_ARCH_CHIP_STM32H743II=y
CONFIG_ARCH_CHIP_STM32H7=y
CONFIG_ARCH_CORTEXM7=y
CONFIG_ARCH_FPU=y

# ============ 硬件优化 ============
CONFIG_ARMV7M_DCACHE=y                # D-Cache
CONFIG_ARMV7M_ICACHE=y                # I-Cache
CONFIG_ARMV7M_DTCM=y                  # DTCM
CONFIG_ARMV7M_USEBASEPRI=y            # BASEPRI 中断
CONFIG_ARMV7M_MEMCPY=y                # 优化的 memcpy

# ============ 内存配置 ============
CONFIG_RAM_START=0x20000000           # DTCM 起始
CONFIG_RAM_SIZE=131072                # 128 KB
CONFIG_MM_REGIONS=4                   # 4 个内存区域

# ============ 时钟配置 ============
CONFIG_STM32H7_CUSTOM_CLOCKCONFIG=y   # 自定义时钟（在 board.h 中）

# ============ 外设使能 ============
CONFIG_STM32H7_SPI1=y                 # SPI1（IMU）
CONFIG_STM32H7_I2C1=y                 # I2C1（磁力计）
CONFIG_STM32H7_USART3=y               # USART3（调试串口）
CONFIG_STM32H7_TIM5=y                 # TIM5（HRT 定时器）

# SPI 配置
CONFIG_STM32H7_SPI1_DMA=y             # SPI1 DMA
CONFIG_STM32H7_SPI1_DMA_BUFFER=1024

# I2C 配置
CONFIG_STM32H7_I2C1_DMA=y             # I2C1 DMA
CONFIG_I2C=y
CONFIG_I2C_RESET=y

# 串口配置
CONFIG_USART3_SERIAL_CONSOLE=y        # USART3 作为控制台
CONFIG_USART3_BAUD=115200

# ============ 调度器配置 ============
CONFIG_SCHED_HPWORK=y                 # 高优先级工作队列
CONFIG_SCHED_HPWORKPRIORITY=249
CONFIG_SCHED_HPWORKSTACKSIZE=2048

CONFIG_SCHED_LPWORK=y                 # 低优先级工作队列
CONFIG_SCHED_LPWORKPRIORITY=50
CONFIG_SCHED_LPWORKSTACKSIZE=2048

CONFIG_PRIORITY_INHERITANCE=y         # 优先级继承

# ============ 文件系统 ============
CONFIG_FS_PROCFS=y                    # /proc 文件系统
CONFIG_FS_ROMFS=y                     # ROMFS
CONFIG_FS_FAT=y                       # FAT（可选，用于 SD 卡）

# ============ C++ 支持 ============
CONFIG_HAVE_CXX=y
CONFIG_HAVE_CXXINITIALIZE=y

# ============ 调试支持 ============
CONFIG_DEBUG_SYMBOLS=y                # 调试符号
CONFIG_DEBUG_FULLOPT=y                # 保持优化
CONFIG_STACK_COLORATION=y             # 栈着色
CONFIG_SCHED_INSTRUMENTATION=y        # 调度监控

# ============ 应用支持 ============
CONFIG_NSH_BUILTIN_APPS=y             # 内置应用支持
CONFIG_BUILTIN=y
```

#### 4.1.2 创建板级头文件（board.h）

**文件**: `boards/stm32h743i_eval/nuttx-config/include/board.h`

```c
#ifndef __BOARDS_ARM_STM32H7_STM32H743I_EVAL_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H7_STM32H743I_EVAL_INCLUDE_BOARD_H

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* ============ 时钟配置 ============ */

/* 外部晶振 */
#define STM32_BOARD_XTAL        25000000ul   /* 25 MHz HSE */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* PLL1 配置（系统时钟）
 * VCO = (HSE / PLLM) * PLLN = (25 MHz / 5) * 192 = 960 MHz
 * SYSCLK = VCO / PLLP = 960 MHz / 2 = 480 MHz
 * PLL1Q = VCO / PLLQ = 960 MHz / 4 = 240 MHz
 * PLL1R = VCO / PLLR = 960 MHz / 8 = 120 MHz
 */
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(5)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(192)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)

#define STM32_VCO1_FREQUENCY     960000000ul
#define STM32_PLL1P_FREQUENCY    480000000ul
#define STM32_PLL1Q_FREQUENCY    240000000ul
#define STM32_PLL1R_FREQUENCY    120000000ul

/* 系统时钟 */
#define STM32_SYSCLK_FREQUENCY   STM32_PLL1P_FREQUENCY
#define STM32_CPUCLK_FREQUENCY   STM32_SYSCLK_FREQUENCY

/* 总线时钟 */
#define STM32_HCLK_FREQUENCY     240000000ul  /* AHB @ 240 MHz */
#define STM32_PCLK1_FREQUENCY    120000000ul  /* APB1 @ 120 MHz */
#define STM32_PCLK2_FREQUENCY    120000000ul  /* APB2 @ 120 MHz */

/* 定时器时钟（APB 倍频） */
#define STM32_APB1_TIM5_CLKIN    (2*STM32_PCLK1_FREQUENCY)  /* 240 MHz */

/* ============ GPIO 引脚定义 ============ */

/* LED（示例：PE3 = LED1） */
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz| \
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)

/* SPI1（IMU）
 * PA5  = SPI1_SCK
 * PA6  = SPI1_MISO
 * PA7  = SPI1_MOSI
 * PA4  = SPI1_NSS (CS)
 */
#define GPIO_SPI1_SCK   GPIO_SPI1_SCK_1   /* PA5 */
#define GPIO_SPI1_MISO  GPIO_SPI1_MISO_1  /* PA6 */
#define GPIO_SPI1_MOSI  GPIO_SPI1_MOSI_1  /* PA7 */
#define GPIO_SPI1_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                         GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

/* I2C1（磁力计）
 * PB6  = I2C1_SCL
 * PB7  = I2C1_SDA
 */
#define GPIO_I2C1_SCL   GPIO_I2C1_SCL_1   /* PB6 */
#define GPIO_I2C1_SDA   GPIO_I2C1_SDA_1   /* PB7 */

/* USART3（调试串口）
 * PD8  = USART3_TX
 * PD9  = USART3_RX
 */
#define GPIO_USART3_TX  GPIO_USART3_TX_3  /* PD8 */
#define GPIO_USART3_RX  GPIO_USART3_RX_3  /* PD9 */

/* ============ 其他配置 ============ */

/* HRT 定时器配置 */
#define HRT_TIMER       5                 /* 使用 TIM5 */
#define HRT_TIMER_CHANNEL 1

#endif /* __BOARDS_ARM_STM32H7_STM32H743I_EVAL_INCLUDE_BOARD_H */
```

#### 4.1.3 创建链接脚本（script.ld）

**文件**: `boards/stm32h743i_eval/nuttx-config/scripts/script.ld`

```ld
/* STM32H743II 链接脚本 */

MEMORY
{
    flash (rx)   : ORIGIN = 0x08000000, LENGTH = 2048K
    dtcm (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K
    sram (rwx)   : ORIGIN = 0x24000000, LENGTH = 512K
    sram1 (rwx)  : ORIGIN = 0x30000000, LENGTH = 128K
    sram4 (rwx)  : ORIGIN = 0x38000000, LENGTH = 64K
}

OUTPUT_ARCH(arm)
ENTRY(_stext)

SECTIONS
{
    .text : {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > flash

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*)))
        KEEP(*(.init_array))
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab : {
        *(.ARM.extab*)
    } > flash

    .ARM.exidx : {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > flash

    _eronly = ABSOLUTE(.);

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

    /* 栈（位于 DTCM，最快） */
    .stack : {
        . = ALIGN(8);
        _sstack = ABSOLUTE(.);
        . = . + CONFIG_IDLETHREAD_STACKSIZE;
        _estack = ABSOLUTE(.);
    } > dtcm

    /* DMA 缓冲区 */
    .dma_buffer (NOLOAD) : {
        *(.dma_buffer)
    } > sram1

    /* Crashdump 区域 */
    .bbsram (NOLOAD) : {
        *(.bbsram)
    } > sram4
}
```

#### 4.1.4 配置 NuttX

```bash
# 1. 进入 NuttX 目录
cd platforms/nuttx/NuttX/nuttx

# 2. 复制配置文件
cp ../../../../boards/stm32h743i_eval/nuttx-config/nsh/defconfig .config

# 3. 使用 menuconfig 调整配置（可选）
make menuconfig

# 4. 保存配置
make savedefconfig
cp defconfig ../../../../boards/stm32h743i_eval/nuttx-config/nsh/

# 5. 返回项目根目录
cd ../../../../
```

### 4.2 创建 HRT（高精度定时器）

**文件**: `platforms/nuttx/src/common/hrt.c`

```c
/**
 * 高精度定时器（High Resolution Timer）
 * 使用 TIM5 实现 1 微秒精度的时间戳
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "arm_internal.h"
#include "stm32_tim.h"
#include "hrt.h"

/* TIM5 寄存器地址 */
#define HRT_TIMER_BASE      STM32_TIM5_BASE

#define REG(_reg)           (*(volatile uint32_t *)(HRT_TIMER_BASE + _reg))
#define rCR1                REG(STM32_GTIM_CR1_OFFSET)
#define rCNT                REG(STM32_GTIM_CNT_OFFSET)
#define rPSC                REG(STM32_GTIM_PSC_OFFSET)
#define rARR                REG(STM32_GTIM_ARR_OFFSET)

/* 全局变量 */
static uint64_t base_time = 0;
static uint32_t last_count = 0;

/**
 * 初始化 HRT
 */
void hrt_init(void)
{
    /* 使能 TIM5 时钟 */
    modifyreg32(STM32_RCC_APB1LENR, 0, RCC_APB1LENR_TIM5EN);

    /* 复位定时器 */
    rCR1 = 0;

    /* 配置分频器：240 MHz / 240 = 1 MHz */
    rPSC = 239;  /* PSC = 240 - 1 */

    /* 32 位自由运行模式 */
    rARR = 0xffffffff;

    /* 启动定时器 */
    rCR1 = GTIM_CR1_CEN;

    /* 初始化基准时间 */
    base_time = 0;
    last_count = rCNT;
}

/**
 * 获取当前时间（微秒）
 */
uint64_t hrt_absolute_time(void)
{
    uint32_t count = rCNT;

    /* 检测溢出 */
    if (count < last_count) {
        /* 32 位计数器溢出（约 71.58 分钟） */
        base_time += 0x100000000ULL;
    }

    last_count = count;

    return base_time + count;
}

/**
 * 计算两个时间戳之间的差值（微秒）
 */
uint64_t hrt_elapsed_time(const uint64_t *then)
{
    return hrt_absolute_time() - *then;
}

/**
 * 微秒延迟
 */
void hrt_usleep(uint32_t usec)
{
    uint64_t start = hrt_absolute_time();
    while (hrt_elapsed_time(&start) < usec);
}
```

**头文件**: `platforms/nuttx/src/common/hrt.h`

```c
#ifndef __PLATFORM_NUTTX_HRT_H
#define __PLATFORM_NUTTX_HRT_H

#include <stdint.h>

/* 时间戳类型（微秒） */
typedef uint64_t hrt_abstime;

/* 初始化 HRT */
void hrt_init(void);

/* 获取当前时间（微秒） */
hrt_abstime hrt_absolute_time(void);

/* 计算时间差（微秒） */
uint64_t hrt_elapsed_time(const hrt_abstime *then);

/* 微秒延迟 */
void hrt_usleep(uint32_t usec);

#endif /* __PLATFORM_NUTTX_HRT_H */
```

---

## 5. uORB 消息总线移植

### 5.1 uORB 架构

uORB（Micro Object Request Broker）是 PX4 的核心消息总线，实现发布-订阅模式。

```
发布者 (Publisher)                订阅者 (Subscriber)
     │                                   │
     │ publish(sensor_accel)             │
     ▼                                   ▼
┌─────────────────────────────────────────────┐
│           uORB Manager (单例)               │
│  ┌──────────────────────────────────────┐  │
│  │  Topic: sensor_accel                 │  │
│  │  ├─ 最新数据（共享内存）              │  │
│  │  ├─ 订阅者列表                        │  │
│  │  └─ 发布计数器                        │  │
│  └──────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
     │                                   │
     │                                   │ copy()
     │                                   ▼
     │                            订阅者的本地缓冲区
```

### 5.2 消息定义

**文件**: `src/msg/sensor_accel.msg`

```
# 加速度计消息

uint64 timestamp        # 时间戳（微秒）

float32 x               # X 轴加速度（m/s^2）
float32 y               # Y 轴加速度（m/s^2）
float32 z               # Z 轴加速度（m/s^2）

float32 temperature     # 温度（摄氏度）

uint32 device_id        # 设备 ID
```

**文件**: `src/msg/sensor_gyro.msg`

```
# 陀螺仪消息

uint64 timestamp        # 时间戳（微秒）

float32 x               # X 轴角速度（rad/s）
float32 y               # Y 轴角速度（rad/s）
float32 z               # Z 轴角速度（rad/s）

float32 temperature     # 温度（摄氏度）

uint32 device_id        # 设备 ID
```

**文件**: `src/msg/sensor_mag.msg`

```
# 磁力计消息

uint64 timestamp        # 时间戳（微秒）

float32 x               # X 轴磁场强度（gauss）
float32 y               # Y 轴磁场强度（gauss）
float32 z               # Z 轴磁场强度（gauss）

float32 temperature     # 温度（摄氏度）

uint32 device_id        # 设备 ID
```

**文件**: `src/msg/vehicle_attitude.msg`

```
# 姿态消息

uint64 timestamp        # 时间戳（微秒）

float32 q[4]            # 四元数 [w, x, y, z]

float32 rollspeed       # 横滚角速度（rad/s）
float32 pitchspeed      # 俯仰角速度（rad/s）
float32 yawspeed        # 偏航角速度（rad/s）
```

### 5.3 消息代码生成器

**文件**: `Tools/msg/generate_msg.py`

```python
#!/usr/bin/env python3
"""
uORB 消息代码生成器
从 .msg 文件生成 C++ 头文件
"""

import os
import sys
import re

def parse_msg_file(msg_file):
    """解析 .msg 文件"""
    with open(msg_file, 'r') as f:
        lines = f.readlines()

    fields = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue

        # 解析字段：类型 名称 # 注释
        match = re.match(r'(\w+(?:\[\d+\])?)\s+(\w+)', line)
        if match:
            field_type = match.group(1)
            field_name = match.group(2)
            fields.append((field_type, field_name))

    return fields

def generate_header(msg_name, fields, output_dir):
    """生成 C++ 头文件"""
    header_guard = f'__{msg_name.upper()}_H__'
    header_file = os.path.join(output_dir, f'{msg_name}.h')

    with open(header_file, 'w') as f:
        f.write(f'#ifndef {header_guard}\n')
        f.write(f'#define {header_guard}\n\n')
        f.write('#include <stdint.h>\n\n')
        f.write(f'struct {msg_name}_s {{\n')

        for field_type, field_name in fields:
            # 类型映射
            if field_type == 'uint64':
                c_type = 'uint64_t'
            elif field_type == 'uint32':
                c_type = 'uint32_t'
            elif field_type == 'float32':
                c_type = 'float'
            elif '[' in field_type:
                # 数组类型
                base_type = field_type.split('[')[0]
                array_size = field_type.split('[')[1].rstrip(']')
                if base_type == 'float32':
                    c_type = f'float'
                    field_name = f'{field_name}[{array_size}]'
            else:
                c_type = field_type

            f.write(f'\t{c_type} {field_name};\n')

        f.write('};\n\n')
        f.write(f'#endif /* {header_guard} */\n')

    print(f'Generated: {header_file}')

def main():
    if len(sys.argv) < 3:
        print('Usage: generate_msg.py <msg_dir> <output_dir>')
        sys.exit(1)

    msg_dir = sys.argv[1]
    output_dir = sys.argv[2]

    os.makedirs(output_dir, exist_ok=True)

    for msg_file in os.listdir(msg_dir):
        if msg_file.endswith('.msg'):
            msg_name = msg_file[:-4]
            msg_path = os.path.join(msg_dir, msg_file)
            fields = parse_msg_file(msg_path)
            generate_header(msg_name, fields, output_dir)

if __name__ == '__main__':
    main()
```

### 5.4 uORB 核心实现（简化版）

**文件**: `src/lib/uorb/uORB.h`

```cpp
#pragma once

#include <stdint.h>
#include <stddef.h>

/* uORB 句柄类型 */
typedef void *orb_advert_t;

/* 消息元数据 */
struct orb_metadata {
    const char *name;       // 消息名称
    size_t size;            // 消息大小（字节）
};

/* 注册消息 */
#define ORB_DECLARE(_name) extern const struct orb_metadata _name##_metadata

/* 定义消息 */
#define ORB_DEFINE(_name, _struct) \
    const struct orb_metadata _name##_metadata = { \
        .name = #_name, \
        .size = sizeof(_struct) \
    }

/* 发布消息（首次） */
orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data);

/* 发布消息（后续） */
int orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data);

/* 订阅消息 */
int orb_subscribe(const struct orb_metadata *meta);

/* 复制消息数据 */
int orb_copy(const struct orb_metadata *meta, int handle, void *buffer);

/* 检查是否有新数据 */
bool orb_check(int handle);
```

**文件**: `src/lib/uorb/uORBManager.cpp`（简化实现）

```cpp
#include "uORB.h"
#include <string.h>
#include <pthread.h>
#include <stdlib.h>

/* 最大话题数量 */
#define MAX_TOPICS 32

/* 话题数据结构 */
struct topic_data {
    const struct orb_metadata *meta;
    void *data;                     // 最新数据
    uint32_t generation;            // 更新计数
    pthread_mutex_t lock;
    bool advertised;
};

/* 全局话题表 */
static struct topic_data topics[MAX_TOPICS];
static int topic_count = 0;
static pthread_mutex_t global_lock = PTHREAD_MUTEX_INITIALIZER;

/* 查找话题 */
static struct topic_data *find_topic(const struct orb_metadata *meta)
{
    for (int i = 0; i < topic_count; i++) {
        if (strcmp(topics[i].meta->name, meta->name) == 0) {
            return &topics[i];
        }
    }
    return NULL;
}

/* 发布消息（首次） */
orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data)
{
    pthread_mutex_lock(&global_lock);

    struct topic_data *topic = find_topic(meta);

    if (topic == NULL) {
        /* 创建新话题 */
        if (topic_count >= MAX_TOPICS) {
            pthread_mutex_unlock(&global_lock);
            return NULL;
        }

        topic = &topics[topic_count++];
        topic->meta = meta;
        topic->data = malloc(meta->size);
        topic->generation = 0;
        pthread_mutex_init(&topic->lock, NULL);
        topic->advertised = true;
    }

    /* 复制数据 */
    pthread_mutex_lock(&topic->lock);
    memcpy(topic->data, data, meta->size);
    topic->generation++;
    pthread_mutex_unlock(&topic->lock);

    pthread_mutex_unlock(&global_lock);

    return (orb_advert_t)topic;
}

/* 发布消息（后续） */
int orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
    struct topic_data *topic = (struct topic_data *)handle;

    pthread_mutex_lock(&topic->lock);
    memcpy(topic->data, data, meta->size);
    topic->generation++;
    pthread_mutex_unlock(&topic->lock);

    return 0;
}

/* 订阅消息 */
int orb_subscribe(const struct orb_metadata *meta)
{
    pthread_mutex_lock(&global_lock);

    struct topic_data *topic = find_topic(meta);

    if (topic == NULL) {
        /* 话题尚未发布，创建占位 */
        if (topic_count >= MAX_TOPICS) {
            pthread_mutex_unlock(&global_lock);
            return -1;
        }

        topic = &topics[topic_count++];
        topic->meta = meta;
        topic->data = malloc(meta->size);
        memset(topic->data, 0, meta->size);
        topic->generation = 0;
        pthread_mutex_init(&topic->lock, NULL);
        topic->advertised = false;
    }

    pthread_mutex_unlock(&global_lock);

    return (int)(topic - topics);  // 返回句柄（索引）
}

/* 复制消息数据 */
int orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
    if (handle < 0 || handle >= topic_count) {
        return -1;
    }

    struct topic_data *topic = &topics[handle];

    pthread_mutex_lock(&topic->lock);
    memcpy(buffer, topic->data, meta->size);
    pthread_mutex_unlock(&topic->lock);

    return 0;
}

/* 检查是否有新数据（简化版，总是返回 true）*/
bool orb_check(int handle)
{
    return true;  // 简化实现
}
```

### 5.5 在应用中使用 uORB

```cpp
#include <uORB/uORB.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>

/* 声明消息 */
ORB_DECLARE(sensor_accel);
ORB_DECLARE(sensor_gyro);

/* 定义消息 */
ORB_DEFINE(sensor_accel, sensor_accel_s);
ORB_DEFINE(sensor_gyro, sensor_gyro_s);

/* 发布者示例 */
void imu_driver_publish()
{
    static orb_advert_t accel_pub = NULL;

    sensor_accel_s accel;
    accel.timestamp = hrt_absolute_time();
    accel.x = read_accel_x();
    accel.y = read_accel_y();
    accel.z = read_accel_z();

    if (accel_pub == NULL) {
        accel_pub = orb_advertise(&sensor_accel_metadata, &accel);
    } else {
        orb_publish(&sensor_accel_metadata, accel_pub, &accel);
    }
}

/* 订阅者示例 */
void sensor_fusion_app()
{
    int accel_sub = orb_subscribe(&sensor_accel_metadata);

    while (1) {
        if (orb_check(accel_sub)) {
            sensor_accel_s accel;
            orb_copy(&sensor_accel_metadata, accel_sub, &accel);

            // 处理加速度计数据
            process_accel(&accel);
        }

        usleep(10000);  // 10 ms
    }
}
```

---

## 6. SPI IMU 驱动开发

### 6.1 ICM42688P 驱动（示例）

**文件**: `src/drivers/imu/icm42688p.h`

```cpp
#pragma once

#include <stdint.h>
#include <nuttx/spi/spi.h>

class ICM42688P
{
public:
    ICM42688P(int spi_bus);
    ~ICM42688P();

    /* 初始化传感器 */
    int init();

    /* 读取加速度和陀螺仪 */
    int read(float accel[3], float gyro[3], float *temp);

    /* 启动周期采样 */
    void start();

private:
    struct spi_dev_s *_spi;
    int _spi_bus;

    /* SPI 寄存器读写 */
    uint8_t read_reg(uint8_t reg);
    void write_reg(uint8_t reg, uint8_t value);
    void read_bulk(uint8_t reg, uint8_t *data, size_t len);

    /* 配置传感器 */
    int configure();

    /* 数据转换 */
    float convert_accel(int16_t raw);
    float convert_gyro(int16_t raw);
};
```

**文件**: `src/drivers/imu/icm42688p.cpp`

```cpp
#include "icm42688p.h"
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <string.h>
#include "hrt.h"

/* ICM42688P 寄存器 */
#define ICM42688P_REG_WHO_AM_I      0x75
#define ICM42688P_REG_PWR_MGMT0     0x4E
#define ICM42688P_REG_GYRO_CONFIG0  0x4F
#define ICM42688P_REG_ACCEL_CONFIG0 0x50
#define ICM42688P_REG_TEMP_DATA1    0x1D
#define ICM42688P_REG_ACCEL_DATA_X1 0x1F
#define ICM42688P_REG_GYRO_DATA_X1  0x25

#define ICM42688P_WHO_AM_I_VALUE    0x47

ICM42688P::ICM42688P(int spi_bus) :
    _spi(nullptr),
    _spi_bus(spi_bus)
{
}

ICM42688P::~ICM42688P()
{
    if (_spi) {
        /* 释放 SPI 设备 */
    }
}

int ICM42688P::init()
{
    /* 1. 打开 SPI 设备 */
    char devpath[16];
    snprintf(devpath, sizeof(devpath), "/dev/spi%d", _spi_bus);

    _spi = spi_open(devpath);
    if (_spi == NULL) {
        return -1;
    }

    /* 2. 配置 SPI 参数 */
    SPI_SETMODE(_spi, SPIDEV_MODE3);        // Mode 3 (CPOL=1, CPHA=1)
    SPI_SETBITS(_spi, 8);                   // 8-bit
    SPI_SETFREQUENCY(_spi, 10000000);       // 10 MHz

    /* 3. 验证设备 ID */
    uint8_t who_am_i = read_reg(ICM42688P_REG_WHO_AM_I);
    if (who_am_i != ICM42688P_WHO_AM_I_VALUE) {
        return -2;  // 设备ID不匹配
    }

    /* 4. 复位设备 */
    write_reg(0x11, 0x01);  // Soft reset
    hrt_usleep(1000);       // 等待1ms

    /* 5. 配置传感器 */
    return configure();
}

int ICM42688P::configure()
{
    /* 1. 使能加速度计和陀螺仪 */
    write_reg(ICM42688P_REG_PWR_MGMT0, 0x0F);  // Accel + Gyro on
    hrt_usleep(100);

    /* 2. 配置陀螺仪: ±2000 dps, ODR=1kHz */
    write_reg(ICM42688P_REG_GYRO_CONFIG0, 0x05);  // 2000dps, 1kHz

    /* 3. 配置加速度计: ±16g, ODR=1kHz */
    write_reg(ICM42688P_REG_ACCEL_CONFIG0, 0x05);  // 16g, 1kHz

    hrt_usleep(100);

    return 0;
}

int ICM42688P::read(float accel[3], float gyro[3], float *temp)
{
    uint8_t data[14];

    /* 读取温度 + 加速度 + 陀螺仪（14字节）*/
    read_bulk(ICM42688P_REG_TEMP_DATA1, data, 14);

    /* 解析温度 */
    int16_t temp_raw = (data[0] << 8) | data[1];
    *temp = (temp_raw / 132.48f) + 25.0f;

    /* 解析加速度 */
    int16_t accel_raw[3];
    accel_raw[0] = (data[2] << 8) | data[3];
    accel_raw[1] = (data[4] << 8) | data[5];
    accel_raw[2] = (data[6] << 8) | data[7];

    accel[0] = convert_accel(accel_raw[0]);
    accel[1] = convert_accel(accel_raw[1]);
    accel[2] = convert_accel(accel_raw[2]);

    /* 解析陀螺仪 */
    int16_t gyro_raw[3];
    gyro_raw[0] = (data[8] << 8) | data[9];
    gyro_raw[1] = (data[10] << 8) | data[11];
    gyro_raw[2] = (data[12] << 8) | data[13];

    gyro[0] = convert_gyro(gyro_raw[0]);
    gyro[1] = convert_gyro(gyro_raw[1]);
    gyro[2] = convert_gyro(gyro_raw[2]);

    return 0;
}

uint8_t ICM42688P::read_reg(uint8_t reg)
{
    uint8_t cmd[2] = {(uint8_t)(reg | 0x80), 0};  // 读命令（bit7=1）
    uint8_t result[2];

    SPI_EXCHANGE(_spi, cmd, result, 2);

    return result[1];
}

void ICM42688P::write_reg(uint8_t reg, uint8_t value)
{
    uint8_t cmd[2] = {(uint8_t)(reg & 0x7F), value};  // 写命令（bit7=0）

    SPI_SEND(_spi, cmd, 2);
}

void ICM42688P::read_bulk(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t cmd = reg | 0x80;

    SPI_SELECT(_spi, 0, true);
    SPI_SEND(_spi, &cmd, 1);
    SPI_RECVBLOCK(_spi, data, len);
    SPI_SELECT(_spi, 0, false);
}

float ICM42688P::convert_accel(int16_t raw)
{
    /* ±16g 量程，灵敏度 = 2048 LSB/g */
    return (raw / 2048.0f) * 9.80665f;  // 转换为 m/s²
}

float ICM42688P::convert_gyro(int16_t raw)
{
    /* ±2000 dps 量程，灵敏度 = 16.4 LSB/(dps) */
    return (raw / 16.4f) * (3.14159265f / 180.0f);  // 转换为 rad/s
}
```

### 6.2 IMU 驱动应用封装

**文件**: `src/drivers/imu/imu_app.cpp`

```cpp
#include "icm42688p.h"
#include <uORB/uORB.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include "hrt.h"
#include <pthread.h>

/* 声明并定义 uORB 话题 */
ORB_DECLARE(sensor_accel);
ORB_DECLARE(sensor_gyro);
ORB_DEFINE(sensor_accel, sensor_accel_s);
ORB_DEFINE(sensor_gyro, sensor_gyro_s);

static ICM42688P *g_imu = nullptr;
static orb_advert_t accel_pub = nullptr;
static orb_advert_t gyro_pub = nullptr;
static pthread_t imu_thread;
static bool thread_should_exit = false;

void *imu_thread_main(void *arg)
{
    while (!thread_should_exit) {
        float accel[3], gyro[3], temp;

        /* 读取传感器数据 */
        if (g_imu->read(accel, gyro, &temp) == 0) {
            uint64_t timestamp = hrt_absolute_time();

            /* 发布加速度计数据 */
            sensor_accel_s accel_msg;
            accel_msg.timestamp = timestamp;
            accel_msg.x = accel[0];
            accel_msg.y = accel[1];
            accel_msg.z = accel[2];
            accel_msg.temperature = temp;
            accel_msg.device_id = 0x42688;  // ICM42688

            if (accel_pub == nullptr) {
                accel_pub = orb_advertise(&sensor_accel_metadata, &accel_msg);
            } else {
                orb_publish(&sensor_accel_metadata, accel_pub, &accel_msg);
            }

            /* 发布陀螺仪数据 */
            sensor_gyro_s gyro_msg;
            gyro_msg.timestamp = timestamp;
            gyro_msg.x = gyro[0];
            gyro_msg.y = gyro[1];
            gyro_msg.z = gyro[2];
            gyro_msg.temperature = temp;
            gyro_msg.device_id = 0x42688;

            if (gyro_pub == nullptr) {
                gyro_pub = orb_advertise(&sensor_gyro_metadata, &gyro_msg);
            } else {
                orb_publish(&sensor_gyro_metadata, gyro_pub, &gyro_msg);
            }
        }

        /* 1kHz 采样率 */
        hrt_usleep(1000);
    }

    return nullptr;
}

extern "C" int imu_main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("Usage: imu {start|stop|status}\n");
        return 1;
    }

    if (strcmp(argv[1], "start") == 0) {
        if (g_imu != nullptr) {
            printf("IMU already running\n");
            return 1;
        }

        /* 创建 IMU 驱动实例（SPI1） */
        g_imu = new ICM42688P(1);

        if (g_imu->init() != 0) {
            printf("IMU init failed\n");
            delete g_imu;
            g_imu = nullptr;
            return 1;
        }

        printf("IMU initialized\n");

        /* 启动采样线程 */
        thread_should_exit = false;
        pthread_create(&imu_thread, nullptr, imu_thread_main, nullptr);

        printf("IMU started\n");
        return 0;
    }

    if (strcmp(argv[1], "stop") == 0) {
        if (g_imu == nullptr) {
            printf("IMU not running\n");
            return 1;
        }

        thread_should_exit = true;
        pthread_join(imu_thread, nullptr);

        delete g_imu;
        g_imu = nullptr;

        printf("IMU stopped\n");
        return 0;
    }

    if (strcmp(argv[1], "status") == 0) {
        if (g_imu == nullptr) {
            printf("IMU: not running\n");
        } else {
            printf("IMU: running\n");
        }
        return 0;
    }

    printf("Unknown command\n");
    return 1;
}
```

---

## 7. I2C 磁力计驱动开发

### 7.1 IST8310 驱动（示例）

**文件**: `src/drivers/mag/ist8310.h`

```cpp
#pragma once

#include <stdint.h>

class IST8310
{
public:
    IST8310(int i2c_bus);
    ~IST8310();

    int init();
    int read(float mag[3], float *temp);

private:
    int _i2c_fd;
    int _i2c_bus;
    uint8_t _i2c_addr;

    int i2c_read_reg(uint8_t reg);
    int i2c_write_reg(uint8_t reg, uint8_t value);
    int i2c_read_bulk(uint8_t reg, uint8_t *data, size_t len);
};
```

**文件**: `src/drivers/mag/ist8310.cpp`

```cpp
#include "ist8310.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <nuttx/i2c/i2c_master.h>
#include "hrt.h"

#define IST8310_ADDR            0x0E
#define IST8310_REG_WHO_AM_I    0x00
#define IST8310_REG_STAT1       0x02
#define IST8310_REG_DATAX_L     0x03
#define IST8310_REG_CNTL1       0x0A
#define IST8310_REG_CNTL2       0x0B

IST8310::IST8310(int i2c_bus) :
    _i2c_fd(-1),
    _i2c_bus(i2c_bus),
    _i2c_addr(IST8310_ADDR)
{
}

IST8310::~IST8310()
{
    if (_i2c_fd >= 0) {
        close(_i2c_fd);
    }
}

int IST8310::init()
{
    /* 1. 打开 I2C 设备 */
    char devpath[16];
    snprintf(devpath, sizeof(devpath), "/dev/i2c%d", _i2c_bus);

    _i2c_fd = open(devpath, O_RDWR);
    if (_i2c_fd < 0) {
        return -1;
    }

    /* 2. 设置 I2C 从地址 */
    ioctl(_i2c_fd, I2CIOC_SETADDRESS, _i2c_addr);

    /* 3. 验证设备 ID */
    int who_am_i = i2c_read_reg(IST8310_REG_WHO_AM_I);
    if (who_am_i != 0x10) {
        return -2;
    }

    /* 4. 软复位 */
    i2c_write_reg(IST8310_REG_CNTL2, 0x01);
    hrt_usleep(10000);  // 等待10ms

    /* 5. 配置：单次测量模式 */
    i2c_write_reg(IST8310_REG_CNTL1, 0x01);

    return 0;
}

int IST8310::read(float mag[3], float *temp)
{
    uint8_t data[6];

    /* 1. 触发测量 */
    i2c_write_reg(IST8310_REG_CNTL1, 0x01);

    /* 2. 等待数据就绪（轮询 STAT1 寄存器）*/
    int timeout = 100;  // 10ms
    while (timeout-- > 0) {
        int stat = i2c_read_reg(IST8310_REG_STAT1);
        if (stat & 0x01) {
            break;  // 数据就绪
        }
        hrt_usleep(100);
    }

    if (timeout <= 0) {
        return -1;  // 超时
    }

    /* 3. 读取 X, Y, Z 数据（6字节）*/
    i2c_read_bulk(IST8310_REG_DATAX_L, data, 6);

    /* 4. 解析数据 */
    int16_t mag_raw[3];
    mag_raw[0] = (data[1] << 8) | data[0];  // X
    mag_raw[1] = (data[3] << 8) | data[2];  // Y
    mag_raw[2] = (data[5] << 8) | data[4];  // Z

    /* 5. 转换为 gauss（灵敏度 3.0 mG/LSB = 0.003 gauss/LSB）*/
    mag[0] = mag_raw[0] * 0.003f;
    mag[1] = mag_raw[1] * 0.003f;
    mag[2] = mag_raw[2] * 0.003f;

    *temp = 25.0f;  // IST8310 无温度传感器

    return 0;
}

int IST8310::i2c_read_reg(uint8_t reg)
{
    uint8_t value;
    struct i2c_msg_s msg[2];

    /* 写寄存器地址 */
    msg[0].frequency = 400000;
    msg[0].addr      = _i2c_addr;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    /* 读数据 */
    msg[1].frequency = 400000;
    msg[1].addr      = _i2c_addr;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = &value;
    msg[1].length    = 1;

    ioctl(_i2c_fd, I2CIOC_TRANSFER, (unsigned long)msg);

    return value;
}

int IST8310::i2c_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    struct i2c_msg_s msg;

    msg.frequency = 400000;
    msg.addr      = _i2c_addr;
    msg.flags     = 0;
    msg.buffer    = buf;
    msg.length    = 2;

    return ioctl(_i2c_fd, I2CIOC_TRANSFER, (unsigned long)&msg);
}

int IST8310::i2c_read_bulk(uint8_t reg, uint8_t *data, size_t len)
{
    struct i2c_msg_s msg[2];

    msg[0].frequency = 400000;
    msg[0].addr      = _i2c_addr;
    msg[0].flags     = 0;
    msg[0].buffer    = &reg;
    msg[0].length    = 1;

    msg[1].frequency = 400000;
    msg[1].addr      = _i2c_addr;
    msg[1].flags     = I2C_M_READ;
    msg[1].buffer    = data;
    msg[1].length    = len;

    return ioctl(_i2c_fd, I2CIOC_TRANSFER, (unsigned long)msg);
}
```

---

## 8. 传感器融合应用

### 8.1 简化的姿态解算（互补滤波）

**文件**: `src/modules/sensor_fusion/sensor_fusion.cpp`

```cpp
#include <uORB/uORB.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>
#include "hrt.h"
#include <math.h>
#include <pthread.h>

/* 声明 uORB 话题 */
ORB_DECLARE(sensor_accel);
ORB_DECLARE(sensor_gyro);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(vehicle_attitude);

ORB_DEFINE(vehicle_attitude, vehicle_attitude_s);

/* 姿态状态（四元数） */
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // [w, x, y, z]

/* 互补滤波系数 */
static const float ALPHA = 0.98f;

/* 四元数归一化 */
static void quaternion_normalize(float q[4])
{
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}

/* 四元数乘法 */
static void quaternion_multiply(const float q1[4], const float q2[4], float result[4])
{
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

/* 从陀螺仪更新姿态（积分） */
static void update_from_gyro(float gyro[3], float dt)
{
    /* 构造角速度四元数 */
    float dq[4];
    dq[0] = 1.0f;
    dq[1] = gyro[0] * dt * 0.5f;
    dq[2] = gyro[1] * dt * 0.5f;
    dq[3] = gyro[2] * dt * 0.5f;

    /* 四元数更新 */
    float q_new[4];
    quaternion_multiply(q, dq, q_new);

    q[0] = q_new[0];
    q[1] = q_new[1];
    q[2] = q_new[2];
    q[3] = q_new[3];

    quaternion_normalize(q);
}

/* 从加速度计+磁力计修正姿态 */
static void correct_from_accel_mag(float accel[3], float mag[3])
{
    /* 1. 从加速度计计算 Roll 和 Pitch */
    float roll = atan2f(accel[1], accel[2]);
    float pitch = atan2f(-accel[0], sqrtf(accel[1]*accel[1] + accel[2]*accel[2]));

    /* 2. 从磁力计计算 Yaw（需要补偿倾斜）*/
    float mag_x = mag[0] * cosf(pitch) + mag[2] * sinf(pitch);
    float mag_y = mag[0] * sinf(roll) * sinf(pitch) + mag[1] * cosf(roll) - mag[2] * sinf(roll) * cosf(pitch);
    float yaw = atan2f(-mag_y, mag_x);

    /* 3. 将 Roll-Pitch-Yaw 转换为四元数 */
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    float q_accel_mag[4];
    q_accel_mag[0] = cr * cp * cy + sr * sp * sy;
    q_accel_mag[1] = sr * cp * cy - cr * sp * sy;
    q_accel_mag[2] = cr * sp * cy + sr * cp * sy;
    q_accel_mag[3] = cr * cp * sy - sr * sp * cy;

    /* 4. 互补滤波融合 */
    q[0] = ALPHA * q[0] + (1.0f - ALPHA) * q_accel_mag[0];
    q[1] = ALPHA * q[1] + (1.0f - ALPHA) * q_accel_mag[1];
    q[2] = ALPHA * q[2] + (1.0f - ALPHA) * q_accel_mag[2];
    q[3] = ALPHA * q[3] + (1.0f - ALPHA) * q_accel_mag[3];

    quaternion_normalize(q);
}

static pthread_t fusion_thread;
static bool thread_should_exit = false;

void *fusion_thread_main(void *arg)
{
    /* 订阅传感器数据 */
    int accel_sub = orb_subscribe(&sensor_accel_metadata);
    int gyro_sub = orb_subscribe(&sensor_gyro_metadata);
    int mag_sub = orb_subscribe(&sensor_mag_metadata);

    orb_advert_t att_pub = nullptr;

    uint64_t last_time = hrt_absolute_time();

    while (!thread_should_exit) {
        sensor_accel_s accel;
        sensor_gyro_s gyro;
        sensor_mag_s mag;

        /* 读取数据 */
        bool has_accel = orb_check(accel_sub) && (orb_copy(&sensor_accel_metadata, accel_sub, &accel) == 0);
        bool has_gyro = orb_check(gyro_sub) && (orb_copy(&sensor_gyro_metadata, gyro_sub, &gyro) == 0);
        bool has_mag = orb_check(mag_sub) && (orb_copy(&sensor_mag_metadata, mag_sub, &mag) == 0);

        if (has_gyro) {
            /* 计算时间差 */
            uint64_t now = hrt_absolute_time();
            float dt = (now - last_time) * 1e-6f;  // 微秒转秒
            last_time = now;

            /* 陀螺仪积分更新 */
            float gyro_data[3] = {gyro.x, gyro.y, gyro.z};
            update_from_gyro(gyro_data, dt);

            /* 加速度计+磁力计修正 */
            if (has_accel && has_mag) {
                float accel_data[3] = {accel.x, accel.y, accel.z};
                float mag_data[3] = {mag.x, mag.y, mag.z};
                correct_from_accel_mag(accel_data, mag_data);
            }

            /* 发布姿态 */
            vehicle_attitude_s att;
            att.timestamp = now;
            att.q[0] = q[0];
            att.q[1] = q[1];
            att.q[2] = q[2];
            att.q[3] = q[3];
            att.rollspeed = gyro.x;
            att.pitchspeed = gyro.y;
            att.yawspeed = gyro.z;

            if (att_pub == nullptr) {
                att_pub = orb_advertise(&vehicle_attitude_metadata, &att);
            } else {
                orb_publish(&vehicle_attitude_metadata, att_pub, &att);
            }
        }

        hrt_usleep(10000);  // 100 Hz
    }

    return nullptr;
}

extern "C" int sensor_fusion_main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("Usage: sensor_fusion {start|stop|status}\n");
        return 1;
    }

    if (strcmp(argv[1], "start") == 0) {
        thread_should_exit = false;
        pthread_create(&fusion_thread, nullptr, fusion_thread_main, nullptr);
        printf("Sensor fusion started\n");
        return 0;
    }

    if (strcmp(argv[1], "stop") == 0) {
        thread_should_exit = true;
        pthread_join(fusion_thread, nullptr);
        printf("Sensor fusion stopped\n");
        return 0;
    }

    if (strcmp(argv[1], "status") == 0) {
        printf("Sensor fusion: running\n");
        return 0;
    }

    return 1;
}
```

---

## 9. 构建系统（CMake）

### 9.1 顶层 CMakeLists.txt

**文件**: `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20)

project(MiniFlight)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

# 设置工具链
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/platforms/nuttx/cmake/Toolchain-arm.cmake)

# 选择目标板
set(BOARD "stm32h743i_eval")
set(BOARD_VARIANT "default")

# 包含板级配置
include(boards/${BOARD}/${BOARD_VARIANT}.cmake)

# 配置 NuttX
add_subdirectory(platforms/nuttx)

# 包含 uORB 消息生成
include(cmake/generate_msg.cmake)

# 添加驱动
add_subdirectory(src/drivers/imu)
add_subdirectory(src/drivers/mag)

# 添加库
add_subdirectory(src/lib/uorb)

# 添加应用
add_subdirectory(src/modules/sensor_fusion)

# 最终固件
add_custom_target(firmware ALL
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_BINARY_DIR}/nuttx ${CMAKE_BINARY_DIR}/miniflight.bin
    DEPENDS nuttx
)
```

### 9.2 工具链文件

**文件**: `platforms/nuttx/cmake/Toolchain-arm.cmake`

```cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# 设置编译器
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

# CPU 标志
set(cpu_flags "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags} -fno-exceptions -fno-rtti" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)

# 链接标志
set(CMAKE_EXE_LINKER_FLAGS "-T${CMAKE_SOURCE_DIR}/boards/${BOARD}/nuttx-config/scripts/script.ld" CACHE STRING "" FORCE)

# 跳过编译器检查
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)
```

### 9.3 消息生成 CMake

**文件**: `cmake/generate_msg.cmake`

```cmake
# 查找所有 .msg 文件
file(GLOB MSG_FILES "${CMAKE_SOURCE_DIR}/src/msg/*.msg")

# 设置输出目录
set(MSG_OUTPUT_DIR "${CMAKE_BINARY_DIR}/uorb/topics")
file(MAKE_DIRECTORY ${MSG_OUTPUT_DIR})

# 生成头文件
foreach(msg_file ${MSG_FILES})
    get_filename_component(msg_name ${msg_file} NAME_WE)
    set(output_file "${MSG_OUTPUT_DIR}/${msg_name}.h")

    add_custom_command(
        OUTPUT ${output_file}
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Tools/msg/generate_msg.py
                ${CMAKE_SOURCE_DIR}/src/msg ${MSG_OUTPUT_DIR}
        DEPENDS ${msg_file}
        COMMENT "Generating ${msg_name}.h"
    )

    list(APPEND GENERATED_MSG_HEADERS ${output_file})
endforeach()

# 创建自定义目标
add_custom_target(generate_messages ALL DEPENDS ${GENERATED_MSG_HEADERS})

# 添加包含路径
include_directories(${CMAKE_BINARY_DIR}/uorb)
```

---

## 10. 固件生成与烧录

### 10.1 编译固件

```bash
# 1. 创建构建目录
mkdir -p build
cd build

# 2. 配置 CMake
cmake ..

# 3. 编译
make -j8

# 4. 查看固件大小
arm-none-eabi-size nuttx

# 输出示例：
#    text    data     bss     dec     hex filename
#  512345   12456  145678  670479   a3b0f nuttx
```

### 10.2 使用 OpenOCD 烧录

**文件**: `Tools/upload_fw.sh`

```bash
#!/bin/bash

# OpenOCD 配置文件
OPENOCD_CFG="interface/stlink.cfg target/stm32h7x.cfg"

# 固件路径
FIRMWARE="build/miniflight.bin"

# 烧录
openocd \
    -f ${OPENOCD_CFG} \
    -c "init" \
    -c "reset halt" \
    -c "flash write_image erase ${FIRMWARE} 0x08000000" \
    -c "reset run" \
    -c "shutdown"
```

**使用方法**:

```bash
# 连接 ST-Link，然后运行：
./Tools/upload_fw.sh
```

### 10.3 VSCode 调试配置

**文件**: `.vscode/launch.json`

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (OpenOCD)",
            "type": "cortex-debug",
            "request": "launch",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/nuttx",
            "servertype": "openocd",
            "configFiles": [
                "interface/stlink.cfg",
                "target/stm32h7x.cfg"
            ],
            "svdFile": "${workspaceFolder}/Tools/svd/STM32H743.svd",
            "runToEntryPoint": "main",
            "showDevDebugOutput": "parsed"
        }
    ]
}
```

**使用方法**:

1. 在 VSCode 中按 `F5`
2. 断点调试
3. 查看寄存器、内存等

---

## 11. 调试方法

### 11.1 串口控制台

```bash
# 1. 找到串口（Windows）
# 设备管理器 → 端口 → ST-Link Virtual COM Port (COMx)

# 2. 连接（使用 PuTTY 或 Tera Term）
# 波特率: 115200
# 数据位: 8
# 停止位: 1
# 校验: None

# 3. 进入 NuttShell (NSH)
nsh> help
nsh> ps      # 查看进程
nsh> free    # 查看内存
nsh> top     # 查看 CPU 使用率
```

### 11.2 启动应用

```bash
# 在 NSH 中启动驱动和应用

nsh> imu start           # 启动 IMU 驱动
nsh> sensor_fusion start # 启动传感器融合

# 查看 uORB 话题
nsh> uorb top

# 输出示例：
#   TOPIC              RATE(Hz)  SUBS  LOST
#   sensor_accel       1000      1     0
#   sensor_gyro        1000      1     0
#   sensor_mag         100       1     0
#   vehicle_attitude   100       0     0
```

### 11.3 监控话题数据

```bash
# 订阅并打印话题数据

nsh> listener sensor_accel

# 输出示例：
# sensor_accel:
#   timestamp: 12345678
#   x: 0.123 m/s²
#   y: -0.456 m/s²
#   z: 9.801 m/s²
#   temperature: 25.3°C
```

### 11.4 GDB 调试

```bash
# 1. 启动 OpenOCD（新终端）
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg

# 2. 启动 GDB（另一个终端）
arm-none-eabi-gdb build/nuttx

# 3. 连接 OpenOCD
(gdb) target extended-remote localhost:3333

# 4. 加载固件
(gdb) load

# 5. 设置断点
(gdb) break sensor_fusion_main

# 6. 运行
(gdb) continue

# 7. 查看变量
(gdb) print q
# $1 = {1.0, 0.0, 0.0, 0.0}

# 8. 查看调用栈
(gdb) backtrace
```

---

## 12. 常见问题

### 12.1 编译错误

**问题 1**: `arm-none-eabi-gcc: command not found`

**解决**:
```bash
# 检查工具链是否安装
which arm-none-eabi-gcc

# 如果没有，重新安装并添加到 PATH
export PATH="/c/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.2 Rel1/bin:$PATH"
```

**问题 2**: `undefined reference to 'stm32_spi1select'`

**解决**:
需要在 `boards/stm32h743i_eval/src/` 中实现板级 SPI 选择函数：

```c
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
    /* 控制 CS 引脚 */
    stm32_gpiowrite(GPIO_SPI1_CS, !selected);
}
```

### 12.2 烧录错误

**问题**: `Error: timed out while waiting for target halted`

**解决**:
1. 检查 ST-Link 连接
2. 确认供电正常
3. 尝试擦除 Flash：
```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
    -c "init" -c "reset halt" -c "stm32h7x mass_erase 0" -c "shutdown"
```

### 12.3 运行时错误

**问题**: 系统启动后立即复位

**可能原因**:
1. 栈溢出
2. HardFault

**调试方法**:
```bash
# 使用 GDB 查看复位原因
(gdb) monitor reset halt
(gdb) info registers

# 查看 HardFault 寄存器
(gdb) print *(uint32_t*)0xE000ED28  # CFSR
(gdb) print *(uint32_t*)0xE000ED2C  # HFSR
```

### 12.4 传感器读取失败

**问题**: `IMU init failed`

**检查清单**:
1. ✅ 接线是否正确（参考第 2.4.3 节）
2. ✅ SPI/I2C 是否在 defconfig 中启用
3. ✅ 引脚定义是否匹配 board.h
4. ✅ 传感器供电是否正常（3.3V）
5. ✅ 使用逻辑分析仪检查 SPI/I2C 波形

---

## 13. 下一步优化

完成基础系统后，可以继续添加以下功能：

### 13.1 控制律

- PID 控制器
- 姿态控制回路
- PWM 输出（电机控制）

### 13.2 通信

- MAVLink 协议
- 地面站通信

### 13.3 存储

- SD 卡日志
- 参数保存到 Flash

### 13.4 扩展传感器

- 气压计（高度估计）
- GPS（位置融合）
- 光流（视觉定位）

---

## 附录 A：完整文件清单

```
MiniFlight/
├── CMakeLists.txt                  # ✅ 必须
├── boards/
│   └── stm32h743i_eval/
│       ├── nuttx-config/
│       │   ├── include/
│       │   │   └── board.h         # ✅ 必须
│       │   ├── scripts/
│       │   │   └── script.ld       # ✅ 必须
│       │   └── nsh/
│       │       └── defconfig       # ✅ 必须
│       └── default.cmake           # ✅ 必须
├── platforms/
│   └── nuttx/
│       ├── cmake/
│       │   └── Toolchain-arm.cmake # ✅ 必须
│       ├── NuttX/                  # Git 子模块
│       └── src/
│           └── common/
│               ├── hrt.c           # ✅ 必须
│               └── hrt.h           # ✅ 必须
├── src/
│   ├── drivers/
│   │   ├── imu/
│   │   │   ├── icm42688p.cpp      # ✅ 必须
│   │   │   ├── icm42688p.h        # ✅ 必须
│   │   │   └── imu_app.cpp        # ✅ 必须
│   │   └── mag/
│   │       ├── ist8310.cpp        # 可选
│   │       └── ist8310.h          # 可选
│   ├── lib/
│   │   └── uorb/
│   │       ├── uORB.h             # ✅ 必须
│   │       └── uORBManager.cpp    # ✅ 必须
│   ├── modules/
│   │   └── sensor_fusion/
│   │       └── sensor_fusion.cpp  # ✅ 必须
│   └── msg/
│       ├── sensor_accel.msg       # ✅ 必须
│       ├── sensor_gyro.msg        # ✅ 必须
│       └── vehicle_attitude.msg   # ✅ 必须
└── Tools/
    ├── msg/
    │   └── generate_msg.py        # ✅ 必须
    └── upload_fw.sh               # ✅ 必须
```

---

## 附录 B：快速开始检查清单

- [ ] 1. 安装 MSYS2 和 ARM 工具链
- [ ] 2. 安装 OpenOCD
- [ ] 3. 配置 VSCode
- [ ] 4. 克隆 NuttX 子模块
- [ ] 5. 创建 board.h（时钟配置）
- [ ] 6. 创建 defconfig（NuttX 配置）
- [ ] 7. 创建链接脚本 script.ld
- [ ] 8. 实现 HRT 定时器
- [ ] 9. 实现 uORB 消息总线
- [ ] 10. 编写 IMU 驱动
- [ ] 11. 编写传感器融合应用
- [ ] 12. 配置 CMake 构建系统
- [ ] 13. 编译固件
- [ ] 14. 烧录到开发板
- [ ] 15. 通过串口验证运行

---

**文档版本**: v1.0
**更新日期**: 2025-01-15
**作者**: MiniFlight 社区
**适用硬件**: STM32H743I（通用方案，可移植到其他 STM32H7）