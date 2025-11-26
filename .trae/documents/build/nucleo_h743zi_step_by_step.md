---
文档版本: 1.0
适用PX4版本: v1.14.x - v1.15.x
最后更新: 2025-11-26
文档类型: 实战操作手册
难度等级: ⭐⭐⭐ (中高级)
前置要求: 有 Nucleo-H743ZI 开发板, 完成 PX4 环境搭建
预计操作时间: 3-5 小时
硬件要求: ST Nucleo-H743ZI 开发板, USB 数据线
---

# Nucleo-H743ZI 开发板运行 PX4 实战指南

## 🎯 目标

将 PX4 Autopilot（包含 NuttX RTOS、uORB、MAVLink）成功运行在 Nucleo-H743ZI 开发板上。

**本教程结束后你将拥有**：
- ✅ 一个可运行的 PX4 固件（Nucleo-H743ZI 定制版）
- ✅ 能够通过串口访问 PX4 命令行（nsh>）
- ✅ 验证 uORB 消息总线正常工作
- ✅ 通过 MAVLink 与 QGroundControl 连接

---

## ⚠️ 重要提示：不要使用 CubeMX！

**错误流程** (❌)：
```
CubeMX 创建项目 → 选择 Nucleo-H743ZI → 生成代码 → 复制 PX4 代码
```

**正确流程** (✅)：
```
PX4 源码 → 创建板级配置 → CMake 构建 → 烧录到 Nucleo
```

**原因**：
1. PX4 已经包含 NuttX RTOS（内置 STM32 HAL）
2. NuttX 的 defconfig 完全替代了 CubeMX 的时钟树配置
3. PX4 的 CMake 系统会自动处理所有编译细节

💡 **深入理解**：想知道 NuttX 如何支持 STM32H7 的 SPI/I2C/UART 等外设？
**→ 请阅读 `nuttx_stm32h7_driver_support.md`** - 详细解释你需要做什么，不需要做什么

---

## 📋 前置条件检查

### 硬件清单

- [ ] ST Nucleo-H743ZI 开发板
- [ ] USB Type-A 转 Micro‑B 数据线（连接 ST-LINK，CN1）
- [ ] （可选）USB-TTL 串口模块（用于额外的 UART 调试）

### 软件清单

- [ ] PX4-Autopilot 源码（已克隆且包含子模块）
- [ ] ARM 交叉编译工具链（arm-none-eabi-gcc 9.3.1+）
- [ ] CMake 3.16+
- [ ] Python 3.7+
- [ ] （可选）STM32CubeProgrammer 或 st-flash

### 验证环境

```bash
# 检查 PX4 源码
cd PX4-Autopilot
git submodule status | head -5  # 应该看到 NuttX 等子模块

# 检查工具链
arm-none-eabi-gcc --version
# 预期: arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1

# 检查 CMake
cmake --version
# 预期: cmake version 3.16 或更高

# 检查 Python
python3 --version
# 预期: Python 3.7 或更高
```

---

## 第一步：创建板级配置文件

### 1.1 创建目录结构

```bash
cd PX4-Autopilot

# 创建 Nucleo-H743ZI 板级目录
mkdir -p boards/st/nucleo-h743zi/src
mkdir -p boards/st/nucleo-h743zi/nuttx-config

cd boards/st/nucleo-h743zi/
```

### 1.2 创建 PX4 板级配置文件

**文件**: `boards/st/nucleo-h743zi/default.px4board`

```bash
cat > default.px4board << 'EOF'
# Nucleo-H743ZI 板级配置
# 这是一个最小化配置，仅用于验证 PX4 能在 Nucleo 板上运行

# 板级标识
CONFIG_BOARD_VENDOR="st"
CONFIG_BOARD_MODEL="nucleo-h743zi"
CONFIG_BOARD_LABEL="default"

# 平台配置
CONFIG_PLATFORM_NUTTX=y
CONFIG_BOARD_ARCHITECTURE="cortex-m7"
CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"

# 资源配置
CONFIG_BOARD_CONSTRAINED_FLASH=n     # 2MB Flash, 不受限
CONFIG_BOARD_CONSTRAINED_MEMORY=n    # 512KB RAM, 不受限

# NuttX 配置名称
CONFIG_BOARD_NUTTX_DEFCONFIG="nsh"   # 使用 nsh (NuttShell) 配置

# === 启用的系统命令 ===
CONFIG_SYSTEMCMDS_PARAM=y            # 参数系统
CONFIG_SYSTEMCMDS_VER=y              # 版本查询
CONFIG_SYSTEMCMDS_TOP=y              # 进程监控
CONFIG_SYSTEMCMDS_NSHTERM=y          # NuttShell 终端
CONFIG_SYSTEMCMDS_UORB=y             # uORB 工具（listener, uorb top）

# === 启用的模块 ===
CONFIG_MODULES_MAVLINK=y             # MAVLink 通信协议
CONFIG_MODULES_LOGGER=y              # 日志记录

# === 启用的库 ===
CONFIG_COMMON_UORB=y                 # uORB 消息总线（核心组件）
CONFIG_LIB_MATHLIB=y                 # 数学库
CONFIG_LIB_MATRIX=y                  # 矩阵运算库

# === 禁用的功能（Nucleo 板没有传感器）===
CONFIG_MODULES_EKF2=n                # 没有 IMU，无法运行状态估计
CONFIG_MODULES_SENSORS=n             # 没有传感器驱动
CONFIG_MODULES_MC_POS_CONTROL=n      # 没有控制输出
CONFIG_MODULES_NAVIGATOR=n           # 没有 GPS
CONFIG_DRIVERS_IMU=n                 # 无板载 IMU
CONFIG_DRIVERS_MAGNETOMETER=n        # 无板载磁力计
CONFIG_DRIVERS_BAROMETER=n           # 无板载气压计
CONFIG_DRIVERS_GPS=n                 # 无 GPS 模块

# === 启用的基础驱动 ===
CONFIG_DRIVERS_LED=y                 # LED 驱动（Nucleo 有 3 个 LED）
EOF
```

### 1.3 创建 CMake 配置文件

**文件**: `boards/st/nucleo-h743zi/default.cmake`

```bash
cat > default.cmake << 'EOF'
# Nucleo-H743ZI CMake 板级配置

px4_add_board(
    PLATFORM nuttx
    VENDOR st
    MODEL nucleo-h743zi
    LABEL default
    TOOLCHAIN arm-none-eabi
    ARCHITECTURE cortex-m7

    # NuttX 配置
    NUTTX_CONFIG nsh

    # 启用的模块
    MODULES
        mavlink
        logger

    # 系统命令
    SYSTEMCMDS
        param
        ver
        top
        nshterm
        uorb

    # 库
    LIBRARIES
        mathlib
        matrix
        geo
        conversion
        drivers_led       # LED 驱动
)
EOF
```

### 1.4 创建板级配置头文件

**文件**: `boards/st/nucleo-h743zi/src/board_config.h`

```cpp
cat > src/board_config.h << 'EOF'
/**
 * @file board_config.h
 * Nucleo-H743ZI 板级配置头文件
 */

#pragma once

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/*
 * Nucleo-H743ZI 硬件资源定义
 * 参考: https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf
 */

// === LED 定义 ===
// LD1 (绿色): PB0
#define GPIO_LED1  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                    GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN0)

// LD2 (黄色): PE1
#define GPIO_LED2  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                    GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN1)

// LD3 (红色): PB14
#define GPIO_LED3  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                    GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN14)

// === 按钮定义 ===
// USER 按钮: PC13 (蓝色按钮)
#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN13)

// === UART 定义 ===
// USART3 连接到 ST-LINK 虚拟串口 (VCP)
// TX: PD8, RX: PD9
#define BOARD_CONSOLE_UART  3

// === USB 定义 ===
// USB OTG FS
#define BOARD_HAS_USB_OTG_FS  1

// === 时钟配置 ===
// Nucleo-H743ZI 使用 8MHz 外部晶振 (与 Pixhawk 的 25MHz 不同!)
#define BOARD_HSE_FREQUENCY  8000000

// === 板级标识 ===
#define BOARD_NAME "Nucleo-H743ZI"

// === 函数声明 ===
__BEGIN_DECLS

/**
 * 板级初始化
 */
int board_app_initialize(uintptr_t arg);

/**
 * 复位外设（Nucleo 板无外部传感器，留空）
 */
void board_peripheral_reset(int ms);

__END_DECLS
EOF
```

### 1.5 创建板级初始化代码

**文件**: `boards/st/nucleo-h743zi/src/init.c`

```c
cat > src/init.c << 'EOF'
/**
 * @file init.c
 * Nucleo-H743ZI 板级初始化
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/px4_manifest.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include "board_config.h"

/**
 * 复位外设
 * @note Nucleo 板没有外部传感器，此函数留空
 */
__EXPORT void board_peripheral_reset(int ms)
{
    // Nucleo 板没有需要复位的外设
    (void)ms;
}

/**
 * 板级应用初始化
 * 在 NuttX 完成基础初始化后调用
 */
__EXPORT int board_app_initialize(uintptr_t arg)
{
    (void)arg;

    // 1. 配置 LED GPIO
    px4_arch_configgpio(GPIO_LED1);  // 绿色 LED
    px4_arch_configgpio(GPIO_LED2);  // 黄色 LED
    px4_arch_configgpio(GPIO_LED3);  // 红色 LED

    // 2. 配置按钮 GPIO
    px4_arch_configgpio(GPIO_BTN_USER);

    // 3. LED 启动指示（闪烁序列）
    // 绿色 LED 闪烁 3 次表示启动成功
    for (int i = 0; i < 3; i++) {
        px4_arch_gpiowrite(GPIO_LED1, true);
        usleep(100000);  // 100ms
        px4_arch_gpiowrite(GPIO_LED1, false);
        usleep(100000);
    }

    // 4. 点亮黄色 LED 表示板级初始化完成
    px4_arch_gpiowrite(GPIO_LED2, true);

    return OK;
}
EOF
```

### 1.6 创建 CMakeLists.txt（板级源码）

**文件**: `boards/st/nucleo-h743zi/src/CMakeLists.txt`

```cmake
cat > src/CMakeLists.txt << 'EOF'
# Nucleo-H743ZI 板级源码编译

px4_add_library(drivers_board
    init.c
)

target_link_libraries(drivers_board
    PRIVATE
        nuttx_arch     # NuttX 架构层
        nuttx_drivers  # NuttX 驱动
)
EOF
```

---

## 第二步：配置 NuttX defconfig

### 2.1 创建 NuttX 配置目录

NuttX 的配置文件位于 `platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/` 下。我们需要为 Nucleo-H743ZI 创建配置。

```bash
cd PX4-Autopilot/platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/

# 复制现有的 Nucleo-H743ZI 配置（NuttX 官方自带）
# 如果没有，可以复制 fmu-v6x 并修改
cp -r nucleo-h743zi nucleo-h743zi-px4  # 备份原始配置
```

**重要**: Nucleo-H743ZI 的 NuttX 配置通常已经存在于 NuttX 源码中。我们只需要微调以适配 PX4。

### 2.2 修改 defconfig（关键差异）

**文件路径**: `platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/nucleo-h743zi/configs/nsh/defconfig`

**关键配置项**（与 Pixhawk 6X 的差异）：

```bash
# 打开 defconfig 文件
cd PX4-Autopilot
nano platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/nucleo-h743zi/configs/nsh/defconfig
```

确保以下配置存在：

```makefile
# === 架构配置 ===
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32h7"
CONFIG_ARCH_CHIP_STM32H743ZI=y
CONFIG_ARCH_CORTEXM7=y
CONFIG_ARCH_FPU=y
CONFIG_ARCH_HAVE_DPFPU=y

# === ⚠️ 关键差异：8MHz 晶振（非 25MHz）===
CONFIG_STM32H7_HSE_FREQUENCY=8000000     # 8MHz 外部晶振

# === 时钟配置 ===
# 目标: 480MHz 主频
# PLL 计算: HSE (8MHz) / PLLM (2) * PLLN (240) / PLLP (2) = 480MHz
CONFIG_STM32H7_BOARD_HCLK=480000000
CONFIG_STM32H7_PLLCFG_PLLM=2
CONFIG_STM32H7_PLLCFG_PLLN=240
CONFIG_STM32H7_PLLCFG_PLLP=2

# === UART 配置 ===
# USART3 连接到 ST-LINK 虚拟串口
CONFIG_STM32H7_USART3=y
CONFIG_USART3_SERIAL_CONSOLE=y
CONFIG_USART3_BAUD=115200

# === USB 配置 ===
CONFIG_STM32H7_OTGFS=y
CONFIG_USBDEV=y
CONFIG_CDCACM=y

# === 文件系统 ===
CONFIG_FS_ROMFS=y                # PX4 启动脚本存储在 RomFS
CONFIG_FS_FAT=y                  # SD 卡支持（可选）

# === 工作队列（PX4 必需）===
CONFIG_SCHED_HPWORK=y            # 高优先级工作队列
CONFIG_SCHED_LPWORK=y            # 低优先级工作队列
CONFIG_SCHED_HPWORKPRIORITY=249
CONFIG_SCHED_LPWORKPRIORITY=50

# === 内存配置 ===
CONFIG_RAM_START=0x24000000      # SRAM1 起始地址
CONFIG_RAM_SIZE=524288           # 512KB

# === 调试选项（可选）===
# CONFIG_DEBUG_SYMBOLS=y         # 启用调试符号（增大固件）
# CONFIG_DEBUG_NOOPT=y           # 禁用优化（便于调试）
```

---

## 第三步：构建固件

### 3.1 清理构建缓存

```bash
cd PX4-Autopilot

# 清理之前的构建
make clean

# 如果有问题，完全清理
make distclean
```

### 3.2 构建 Nucleo-H743ZI 固件

```bash
# 构建命令
make st_nucleo-h743zi_default

# 如果需要查看详细编译日志
VERBOSE=1 make st_nucleo-h743zi_default
```

**预期输出**（简化版）：

```
-- PX4 version: v1.14.0
-- Found toolchain: GNU ARM Embedded Toolchain 9.3.1
-- Configuring NuttX
-- Building NuttX kernel
[ 10%] Building C object nuttx/arch/arm/src/CMakeFiles/arch.dir/stm32h7/stm32_start.c.obj
[ 20%] Linking C static library libarch.a
[ 30%] Building CXX object src/lib/matrix/CMakeFiles/matrix.dir/matrix/Matrix.cpp.obj
[ 50%] Building CXX object src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_main.cpp.obj
[ 80%] Linking CXX executable st_nucleo-h743zi_default.elf
[ 90%] Generating st_nucleo-h743zi_default.bin
[100%] Built target st_nucleo-h743zi_default

Build finished successfully!
```

### 3.3 检查固件大小

```bash
arm-none-eabi-size build/st_nucleo-h743zi_default/st_nucleo-h743zi_default.elf
```

**预期输出示例**：

```
   text    data     bss     dec     hex filename
 512345   8456   65432  586233   8f1a9 st_nucleo-h743zi_default.elf

说明:
- text + data = Flash 占用 (~520KB / 2MB)
- bss = RAM 占用 (~64KB / 512KB)
- 剩余资源充足
```

### 3.4 生成的文件

```bash
ls -lh build/st_nucleo-h743zi_default/

# 应该看到:
# st_nucleo-h743zi_default.elf    # 带调试信息的可执行文件
# st_nucleo-h743zi_default.bin    # 纯二进制固件
# st_nucleo-h743zi_default.px4    # PX4 固件包（带元数据）
```

---

## 第四步：烧录固件

### 方法 1: 使用 st-flash (Linux/macOS)

```bash
# 安装 stlink 工具
# Ubuntu/Debian: sudo apt-get install stlink-tools
# macOS: brew install stlink

# 连接 Nucleo 板（ST-LINK USB 接口）
# 烧录固件
st-flash write build/st_nucleo-h743zi_default/st_nucleo-h743zi_default.bin 0x08000000

# 预期输出:
# st-flash 1.7.0
# 2025-11-26T12:00:00 INFO common.c: H74x/H75x: 2048 KiB SRAM, 2048 KiB flash in at least 128 KiB pages.
# file build/st_nucleo-h743zi_default/st_nucleo-h743zi_default.bin md5 checksum: a1b2c3d4...
# 2025-11-26T12:00:01 INFO common.c: Attempting to write 524288 (0x80000) bytes to stm32 address: 134217728 (0x8000000)
# 2025-11-26T12:00:03 INFO common.c: Finished writing
```

### 方法 2: 使用 STM32CubeProgrammer (跨平台，推荐)

1. **下载安装** STM32CubeProgrammer:
   https://www.st.com/en/development-tools/stm32cubeprog.html

2. **连接硬件**:
   - 用 USB 线连接 Nucleo 板的 **CN1 接口**（ST-LINK）
   - 绿色 LED (LD1) 应该常亮或闪烁（表示 ST-LINK 通电）

3. **烧录步骤**:
   - 打开 STM32CubeProgrammer
   - 连接方式: 选择 **ST-LINK**
   - 点击 **Connect** (右上角)
   - 点击 **Open file** 选择 `build/st_nucleo-h743zi_default/st_nucleo-h743zi_default.bin`
   - 起始地址填写: `0x08000000`
   - 点击 **Start Programming**
   - 等待进度条完成

4. **验证**:
   - 点击 **Verify** 按钮（可选）
   - 点击 **Disconnect**

### 方法 3: 使用 make upload (需配置)

在 `boards/st/nucleo-h743zi/default.cmake` 中添加上传配置后，可以使用：

```bash
make st_nucleo-h743zi_default upload
```

---

## 第五步：验证固件运行

### 5.1 连接串口

Nucleo-H743ZI 的 **USART3** 通过 ST-LINK 虚拟成 USB 串口 (VCP)。

**Windows**:
```powershell
# 在设备管理器中查找 "STMicroelectronics Virtual COM Port"
# 显示为 COM3, COM4 等

# 使用 PuTTY 或 TeraTerm 连接
# 波特率: 115200
# 数据位: 8
# 停止位: 1
# 校验位: None
```

**Linux**:
```bash
# 查找设备
ls /dev/ttyACM*
# 应该显示 /dev/ttyACM0

# 使用 screen 连接
screen /dev/ttyACM0 115200

# 或使用 minicom
minicom -D /dev/ttyACM0 -b 115200
```

**macOS**:
```bash
# 查找设备
ls /dev/tty.usbmodem*

# 使用 screen 连接
screen /dev/tty.usbmodem1234 115200
```

### 5.2 预期启动日志

连接串口后，按下 Nucleo 板的 **黑色 RESET 按钮**，应该看到：

```
NuttShell (NSH) NuttX-10.3.0
nsh>

______  __   __    ___
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

INFO  [px4] Startup script: /etc/init.d/rcS
INFO  [dataman] data manager file './dataman' size is 62560 bytes
INFO  [uORB] uORB started
INFO  [logger] logger started (mode=all)
INFO  [mavlink] mode: Normal, data rate: 100000 B/s on /dev/ttyS0 @ 57600B
INFO  [mavlink] MAVLink link on /dev/ttyS0 created

nsh>
```

**关键标志**：
- ✅ **PX4 Logo** 显示（ASCII 艺术字）
- ✅ **uORB started** （消息总线启动）
- ✅ **MAVLink link created** （MAVLink 协议启动）
- ✅ **nsh>** 提示符（可以输入命令）

### 5.3 测试基本命令

在 `nsh>` 提示符下输入：

```bash
# 查看 PX4 版本
nsh> ver
PX4 firmware version: 1.14.0
PX4 git hash: a1b2c3d4e5f6...
Build time: Nov 26 2025 12:00:00
OS: NuttX
OS version: 10.3.0 (184549631)
Toolchain: GNU GCC, 9.3.1
Board: Nucleo-H743ZI

# 查看进程
nsh> top
PID GROUP PRI POLICY   TYPE    NPX STATE   EVENT      SIGMASK  STACK COMMAND
  0   0   0 FIFO     Kthread - Ready              000000000000000000 0000000016  Idle Task
  1   1 100 RR       Task    - Waiting Semaphore 000000000000000000 0000000000  init
  2   2 175 RR       Task    - Running            000000000000000000 0000002864  uorb
  3   3 100 RR       Task    - Waiting Semaphore 000000000000000000 0000006240  logger
  4   4 300 RR       Task    - Waiting Semaphore 000000000000000000 0000003264  mavlink

# 查看参数
nsh> param show
Showing all parameters:
SYS_AUTOCONFIG     [0]     -> 1
SYS_AUTOSTART      [0]     -> 0
SYS_USE_IO         [0]     -> 0
...

# 测试 uORB (应该没有消息，因为没有传感器)
nsh> uorb top
```

### 5.4 LED 验证

- **绿色 LED (LD1)**: 启动时闪烁 3 次
- **黄色 LED (LD2)**: 启动后常亮（板级初始化完成）
- **红色 LED (LD3)**: 未使用

---

## 第六步：连接 QGroundControl

### 6.1 配置 MAVLink 串口

编辑启动脚本（如果需要自定义 MAVLink 配置）：

```bash
# 在 SD 卡上创建自定义启动脚本（可选）
# 或修改 ROMFS 中的 rc.serial
```

默认情况下，MAVLink 会在 USART3（虚拟串口）上启动。

### 6.2 使用 QGroundControl 连接

1. **下载安装** QGroundControl:
   https://qgroundcontrol.com/

2. **配置连接**:
   - 打开 QGroundControl
   - 点击左上角 **Q** 图标 → **Application Settings** → **Comm Links**
   - 点击 **Add** 添加新连接
   - 类型: **Serial**
   - 名称: `Nucleo-H743ZI`
   - 串口: 选择对应的 COM 端口（Windows）或 `/dev/ttyACM0` (Linux)
   - 波特率: `57600` (默认 MAVLink 波特率)

3. **连接**:
   - 点击 **Connect**
   - 应该看到 QGroundControl 右上角显示 **Connected**
   - 状态栏显示: `PX4 Firmware v1.14.0` 和 `MANUAL` 模式

### 6.3 验证 MAVLink 通信

在 QGroundControl 中：
- **Analyze Tools** → **MAVLink Inspector**
- 应该能看到 `HEARTBEAT`, `SYS_STATUS` 等消息

在 PX4 串口终端：
```bash
nsh> mavlink status
instance #0:
    GCS heartbeat valid: YES
    mavlink mode: Normal
    transport protocol: serial (/dev/ttyS0 @57600)
```

---

## 第七步：添加外部传感器（可选）

Nucleo-H743ZI 本身没有 IMU、GPS 等传感器。如果你想测试完整的 PX4 功能，需要外接传感器。

### 7.1 外接 IMU (SPI)

**推荐模块**: ICM-20948 分离模块（淘宝/eBay 有售）

**接线**（参考 Nucleo-H743ZI 引脚图）:

| ICM-20948 | Nucleo-H743ZI | Arduino 接口 |
|-----------|---------------|--------------|
| VCC       | 3.3V          | CN8-7        |
| GND       | GND           | CN7-8        |
| SCL       | PA5 (SPI1_SCK)  | CN7-10       |
| SDA/MOSI  | PA7 (SPI1_MOSI) | CN7-14       |
| AD0/MISO  | PA6 (SPI1_MISO) | CN7-12       |
| CS        | PD14 (GPIO)     | CN7-16       |

**修改配置**:

1. 在 `default.px4board` 中添加:
   ```python
   CONFIG_DRIVERS_IMU_INVENSENSE_ICM20948=y
   ```

2. 在 `board_config.h` 中添加:
   ```c
   #define GPIO_SPI1_CS_ICM20948  (GPIO_OUTPUT | GPIO_PORTD | GPIO_PIN14)
   ```

3. 重新编译并烧录固件

### 7.2 外接 GPS (UART)

**推荐模块**: u-blox NEO-M8N

**接线**:

| GPS 模块 | Nucleo-H743ZI |
|----------|---------------|
| VCC      | 5V            |
| GND      | GND           |
| TX       | PD9 (USART3_RX) ⚠️ 注意: 需要换用其他 UART，因为 USART3 已被控制台占用 |
| RX       | PD8 (USART3_TX) |

**建议使用 USART1**:

| GPS 模块 | Nucleo-H743ZI |
|----------|---------------|
| TX       | PB7 (USART1_RX) |
| RX       | PB6 (USART1_TX) |

### 7.3 外接第二路 IMU 与磁力计（完整接线方案）

**IMU2 (SPI2)**

- 接线：
  - `SCK → CN7-5 (PB13)`
  - `MISO → CN12-28 (PB14)` 或 `CN7-19 (PB4)`
  - `MOSI → CN7-3 (PB15)`
  - `CS → CN7-7 (PB12)` 或使用 `PD15`（Morpho/Zio GPIO）

**磁力计 (I2C1)**

- 接线：
  - `SCL → CN7-2 (PB8)`
  - `SDA → CN7-4 (PB9)`
  - `VCC → CN8-7 (3.3V)`
  - `GND → CN7-8 (GND)`

**MAVLink (VCP / USART3)**

- 接线：
  - `TX → CN11-67 (PD8)`
  - `RX → CN11-69 (PD9)`

### 7.4 PX4/NuttX 配置建议（使能外设驱动）

- 在 `boards/st/nucleo-h743zi/default.px4board` 中启用：
  - `CONFIG_MODULES_MAVLINK=y`
  - `CONFIG_SYSTEMCMDS_UORB=y`
  - `CONFIG_COMMON_MAGNETOMETER=y`
  - `CONFIG_DRIVERS_IMU_INVENSENSE_ICM20948=y`
  - `CONFIG_DRIVERS_GPS=y`

- 在 NuttX `defconfig` 中确认/启用：
  - `CONFIG_STM32H7_SPI1=y`、`CONFIG_STM32H7_SPI2=y`
  - `CONFIG_STM32H7_I2C1=y`
  - `CONFIG_STM32H7_USART3=y`（VCP）以及可选 `CONFIG_STM32H7_USART1=y`
  - `CONFIG_FS_ROMFS=y`

### 7.5 参考 pin 映射文档

- 详见 `nucleo_h743zi_pinmap.md` 的 “CN 快速索引（精选）” 部分，包含 CN7/10/11/12 关键引脚的编号与 MCU 引脚对应关系。

---

## 第八步：调试技巧

### 8.1 查看启动日志

如果固件无法启动，按 RESET 按钮后观察串口输出：

```bash
# 应该看到 NuttX 启动信息
NuttShell (NSH) NuttX-10.3.0
...
```

**常见问题**：
- 没有任何输出 → 检查串口波特率（应为 115200）
- 卡在 `NuttShell` 不继续 → NuttX 启动了但 PX4 没启动（检查链接脚本）
- 崩溃重启循环 → 内存不足或栈溢出

### 8.2 使用 GDB 调试（高级）

```bash
# 1. 启动 OpenOCD (连接 ST-LINK)
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg

# 2. 在另一个终端启动 GDB
arm-none-eabi-gdb build/st_nucleo-h743zi_default/st_nucleo-h743zi_default.elf

# 3. GDB 中连接到 OpenOCD
(gdb) target extended-remote :3333
(gdb) load
(gdb) break main
(gdb) continue
```

### 8.3 日志级别调整

在 `nsh>` 终端修改日志级别：

```bash
# 设置为 DEBUG 级别
nsh> param set SYS_LOGGER_LEVEL 0

# 重启
nsh> reboot
```

---

## 故障排除

### 问题 1: 编译错误 `No rule to make target 'nuttx/defconfig'`

**原因**: NuttX 配置文件未找到

**解决**:
```bash
# 检查 defconfig 是否存在
ls platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/nucleo-h743zi/configs/nsh/defconfig

# 如果不存在，从其他板复制并修改
```

### 问题 2: 固件运行但串口无输出

**检查清单**:
- [ ] 串口波特率是否为 115200
- [ ] 是否连接到正确的虚拟串口（USART3 VCP）
- [ ] 是否按了 RESET 按钮

### 问题 3: LED 不亮

**原因**: GPIO 配置错误或初始化失败

**调试**:
```c
// 在 init.c 中添加调试代码
syslog(LOG_INFO, "LED1 configured, writing HIGH\n");
px4_arch_gpiowrite(GPIO_LED1, true);
```

### 问题 4: MAVLink 无法连接

**检查**:
```bash
nsh> mavlink status
# 查看 MAVLink 是否启动在正确的串口
```

---

## 总结

🎉 **恭喜！** 你已经成功将 PX4 Autopilot 运行在 Nucleo-H743ZI 开发板上！

**你学到了**:
- ✅ PX4 板级配置文件（`.px4board`）的编写
- ✅ NuttX defconfig 的配置（特别是时钟树）
- ✅ 板级初始化代码的编写
- ✅ CMake 构建系统的使用
- ✅ 固件烧录和串口调试
- ✅ MAVLink 通信测试

**下一步建议**:
1. 阅读 `.trae/documents/system/uorb.md` 了解如何发布/订阅消息
2. 编写一个简单的 PX4 模块（参考 `src/examples/px4_simple_app/`）
3. 外接传感器，运行 EKF2 状态估计
4. 研究现有板级配置（如 `boards/px4/fmu-v6x/`）

**参考文档**:
- Nucleo-H743ZI 用户手册: https://www.st.com/resource/en/user_manual/um2407.pdf
- PX4 开发者指南: https://docs.px4.io/main/en/development/
- NuttX 文档: https://nuttx.apache.org/docs/latest/
