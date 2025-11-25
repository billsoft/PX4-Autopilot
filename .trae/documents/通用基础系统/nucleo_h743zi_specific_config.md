# Nucleo-H743ZI 专用配置补充文档

本文档是 `stm32h743_minimal_flight_controller_guide.md` 的**补充**，专门针对 **Nucleo-H743ZI** 开发板的差异进行说明。

---

## 1. Nucleo-H743ZI 硬件概述

### 1.1 主要特性

```
MCU:            STM32H743ZIT6
封装:           LQFP144 (144引脚)
Flash:          2 MB
RAM:            1 MB (512KB AXI SRAM + 128KB SRAM1/2/3/4 + 128KB DTCM)
主频:           480 MHz (最高)
调试器:         板载 ST-Link V3（USB虚拟串口+调试）
电源:           USB 或外部 5V
Arduino接口:    兼容 Arduino Uno Rev3
Morpho接口:     扩展所有引脚
```

### 1.2 与 STM32H743II (EVAL板) 的差异

| 项目 | Nucleo-H743ZI (ZI) | STM32H743I-EVAL (II) |
|------|-------------------|---------------------|
| **封装** | LQFP144 | LQFP176 |
| **HSE晶振** | 8 MHz (来自ST-Link MCO) | 25 MHz 或 16 MHz |
| **调试器** | 板载 ST-Link V3 | 外部 ST-Link |
| **LED** | LD1(绿,PB0), LD2(黄,PE1), LD3(红,PB14) | 不同 |
| **按钮** | USER(PC13), RESET | 不同 |
| **Arduino接口** | 有 | 无 |

### 1.3 引脚布局图

**Arduino 接口（推荐用于连接传感器）**：

```
Arduino CN9 (左侧):
  D0  (PD9)  - UART3 RX
  D1  (PD8)  - UART3 TX
  D2  (PF15)
  D3  (PE13)
  D4  (PF14)
  D5  (PE11)
  D6  (PE9)
  D7  (PF13)

Arduino CN7 (右侧):
  D8  (PF12)
  D9  (PD15)
  D10 (PD14) - SPI1_NSS
  D11 (PA7)  - SPI1_MOSI  ← IMU连接
  D12 (PA6)  - SPI1_MISO  ← IMU连接
  D13 (PA5)  - SPI1_SCK   ← IMU连接
  GND
  AREF
  SDA (PB9)  - I2C1_SDA   ← 磁力计连接
  SCL (PB8)  - I2C1_SCL   ← 磁力计连接
```

---

## 2. 修改 defconfig（Nucleo专用）

**文件**: `boards/nucleo_h743zi/nuttx-config/nsh/defconfig`

**关键修改点**：

```makefile
# ============ 架构配置（修改芯片型号）============
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32h7"
CONFIG_ARCH_CHIP_STM32H743ZI=y      # ← 改为 ZI（不是 II）
CONFIG_ARCH_CHIP_STM32H7=y
CONFIG_ARCH_CORTEXM7=y
CONFIG_ARCH_FPU=y

# ============ 板级配置 ============
CONFIG_ARCH_BOARD_CUSTOM=y
CONFIG_ARCH_BOARD_CUSTOM_DIR="../../../../boards/nucleo_h743zi/nuttx-config"  # ← 路径
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
CONFIG_ARCH_BOARD_CUSTOM_NAME="nucleo-h743zi"  # ← 板名

# ============ 外设配置（与原文档相同）============
CONFIG_STM32H7_SPI1=y
CONFIG_STM32H7_I2C1=y
CONFIG_STM32H7_USART3=y             # DEBUG串口（通过ST-Link虚拟串口）
CONFIG_STM32H7_TIM5=y

# USART3 配置（Nucleo的ST-Link虚拟串口）
CONFIG_USART3_SERIAL_CONSOLE=y      # 控制台
CONFIG_USART3_BAUD=115200

# SPI1 DMA
CONFIG_STM32H7_SPI1_DMA=y
CONFIG_STM32H7_SPI1_DMA_BUFFER=1024

# I2C1
CONFIG_STM32H7_I2C1=y
CONFIG_I2C=y

# 其他配置与原文档相同...
```

---

## 3. 修改 board.h（时钟配置）

**文件**: `boards/nucleo_h743zi/nuttx-config/include/board.h`

### 3.1 时钟源配置

**Nucleo-H743ZI 的 HSE 来自 ST-Link MCO，频率为 8 MHz**：

```c
/* ============ 时钟配置 ============ */

/* Nucleo板的HSE来自ST-Link MCO（8 MHz）*/
#define STM32_BOARD_XTAL        8000000ul    // ← 改为 8 MHz（不是25MHz）

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* PLL1 配置（系统时钟 - 达到 480 MHz）
 *
 * 输入: HSE = 8 MHz
 * VCO = (HSE / PLLM) * PLLN = (8 MHz / 2) * 240 = 960 MHz
 * SYSCLK = VCO / PLLP = 960 MHz / 2 = 480 MHz
 * PLL1Q = VCO / PLLQ = 960 MHz / 4 = 240 MHz
 * PLL1R = VCO / PLLR = 960 MHz / 8 = 120 MHz
 */

#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(2)    // ← 改为 2（8MHz÷2=4MHz）
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(240)      // ← 改为 240（4MHz×240=960MHz）
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)        // 960÷2=480MHz
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)        // 960÷4=240MHz
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)        // 960÷8=120MHz

#define STM32_VCO1_FREQUENCY     960000000ul
#define STM32_PLL1P_FREQUENCY    480000000ul
#define STM32_PLL1Q_FREQUENCY    240000000ul
#define STM32_PLL1R_FREQUENCY    120000000ul

/* 系统时钟 */
#define STM32_SYSCLK_FREQUENCY   STM32_PLL1P_FREQUENCY  // 480 MHz
#define STM32_CPUCLK_FREQUENCY   STM32_SYSCLK_FREQUENCY

/* 总线时钟（与原文档相同）*/
#define STM32_HCLK_FREQUENCY     240000000ul  // AHB @ 240 MHz
#define STM32_PCLK1_FREQUENCY    120000000ul  // APB1 @ 120 MHz
#define STM32_PCLK2_FREQUENCY    120000000ul  // APB2 @ 120 MHz

/* 定时器时钟 */
#define STM32_APB1_TIM5_CLKIN    (2*STM32_PCLK1_FREQUENCY)  // 240 MHz
```

**时钟树可视化（Nucleo专用）**：

```
       ST-Link MCO
            │
            ▼
         8 MHz (HSE)
            │
            │ ÷2 (PLLM=2)
            ▼
         4 MHz (PLL输入)
            │
            │ ×240 (PLLN=240)
            ▼
        960 MHz (VCO)
            │
            ├─► ÷2 → 480 MHz (SYSCLK)
            ├─► ÷4 → 240 MHz (PLL1Q)
            └─► ÷8 → 120 MHz (PLL1R)
```

### 3.2 GPIO 引脚定义（Nucleo专用）

```c
/* ============ GPIO 引脚定义（Nucleo-H743ZI）============ */

/* LED（Nucleo板载LED）*/
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz| \
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)   // LD1 绿色

#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz| \
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN1)   // LD2 黄色

#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz| \
                         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)  // LD3 红色

/* 按钮（USER按钮，PC13）*/
#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)

/* SPI1（Arduino D10-D13，用于IMU）
 * PA5  = SPI1_SCK  (Arduino D13)
 * PA6  = SPI1_MISO (Arduino D12)
 * PA7  = SPI1_MOSI (Arduino D11)
 * PD14 = SPI1_NSS  (Arduino D10, 软件控制)
 */
#define GPIO_SPI1_SCK   GPIO_SPI1_SCK_1   // PA5
#define GPIO_SPI1_MISO  GPIO_SPI1_MISO_1  // PA6
#define GPIO_SPI1_MOSI  GPIO_SPI1_MOSI_1  // PA7
#define GPIO_SPI1_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN14)  // D10

/* I2C1（Arduino SDA/SCL，用于磁力计）
 * PB8  = I2C1_SCL (Arduino SCL)
 * PB9  = I2C1_SDA (Arduino SDA)
 */
#define GPIO_I2C1_SCL   GPIO_I2C1_SCL_2   // PB8
#define GPIO_I2C1_SDA   GPIO_I2C1_SDA_2   // PB9

/* USART3（ST-Link虚拟串口，调试控制台）
 * PD8  = USART3_TX (Arduino D1)
 * PD9  = USART3_RX (Arduino D0)
 */
#define GPIO_USART3_TX  GPIO_USART3_TX_3  // PD8
#define GPIO_USART3_RX  GPIO_USART3_RX_3  // PD9

/* HRT 定时器配置（与原文档相同）*/
#define HRT_TIMER       5
#define HRT_TIMER_CHANNEL 1
```

---

## 4. 传感器接线（Nucleo专用）

### 4.1 IMU (ICM42688P) → SPI1 (Arduino接口)

```
Nucleo Arduino 接口 ─────► ICM42688P 模块

D13 (PA5,  SPI1_SCK)  ───► SCK
D12 (PA6,  SPI1_MISO) ◄──► SDO (MISO)
D11 (PA7,  SPI1_MOSI) ───► SDA (MOSI)
D10 (PD14, GPIO)      ───► CS
GND                   ───► GND
3.3V                  ───► VCC (注意电压！)
```

**⚠️ 重要提示**：
- Nucleo Arduino 接口的电源引脚提供 **3.3V 或 5V**（跳线选择）
- 确保跳线选择 **3.3V**（大多数IMU只支持3.3V）
- 检查跳线 JP1 和 JP2 设置

### 4.2 磁力计 (IST8310) → I2C1 (Arduino接口)

```
Nucleo Arduino 接口 ─────► IST8310 模块

SCL (PB8, I2C1_SCL) ───► SCL
SDA (PB9, I2C1_SDA) ◄──► SDA
GND                 ───► GND
3.3V                ───► VCC
```

### 4.3 调试串口（已集成！）

Nucleo 板载 ST-Link 提供虚拟串口，**无需外接 USB-TTL 模块**：

```
连接方式：
1. Nucleo 的 USB ST-LINK 插口（CN1）连接到 PC
2. 自动识别为两个设备：
   - ST-Link 调试器
   - 虚拟串口 (COMx)
3. USART3 (PD8/PD9) 自动映射到虚拟串口

Windows 设备管理器中查看：
- 端口 (COM 和 LPT)
  └─ STMicroelectronics Virtual COM Port (COMx)  ← 使用这个
```

---

## 5. 链接脚本（与原文档相同）

**文件**: `boards/nucleo_h743zi/nuttx-config/scripts/script.ld`

STM32H743ZI 和 STM32H743II 的内存布局相同，使用原文档的链接脚本即可：

```ld
MEMORY
{
    flash (rx)   : ORIGIN = 0x08000000, LENGTH = 2048K
    dtcm (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K
    sram (rwx)   : ORIGIN = 0x24000000, LENGTH = 512K
    sram1 (rwx)  : ORIGIN = 0x30000000, LENGTH = 128K
    sram4 (rwx)  : ORIGIN = 0x38000000, LENGTH = 64K
}

/* ... 其余与原文档相同 ... */
```

---

## 6. 烧录与调试（Nucleo专用）

### 6.1 OpenOCD 配置

Nucleo 板载 ST-Link V3，配置更简单：

**文件**: `Tools/upload_fw_nucleo.sh`

```bash
#!/bin/bash

# Nucleo 使用板载 ST-Link（board 配置文件包含接口和目标）
OPENOCD_CFG="board/st_nucleo_h743zi.cfg"

# 固件路径
FIRMWARE="build/miniflight.bin"

# 烧录
openocd \
    -f ${OPENOCD_CFG} \
    -c "init" \
    -c "reset halt" \
    -c "flash write_image erase ${FIRMWARE} 0x08000000" \
    -c "verify_image ${FIRMWARE} 0x08000000" \
    -c "reset run" \
    -c "shutdown"
```

**使用方法**：

```bash
# 1. 连接 Nucleo 的 USB ST-LINK 接口（CN1）到 PC
# 2. 运行烧录脚本
./Tools/upload_fw_nucleo.sh
```

### 6.2 VSCode 调试配置

**文件**: `.vscode/launch_nucleo.json`

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (Nucleo-H743ZI)",
            "type": "cortex-debug",
            "request": "launch",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/nuttx",
            "servertype": "openocd",
            "configFiles": [
                "board/st_nucleo_h743zi.cfg"
            ],
            "svdFile": "${workspaceFolder}/Tools/svd/STM32H743.svd",
            "runToEntryPoint": "main",
            "showDevDebugOutput": "parsed",
            "preLaunchTask": "Build"
        }
    ]
}
```

### 6.3 串口调试

**Windows**：

```powershell
# 1. 查看串口号
# 设备管理器 → 端口 → STMicroelectronics Virtual COM Port (COM3)

# 2. 使用 PuTTY 连接
# 串口: COM3
# 波特率: 115200
# 数据位: 8
# 停止位: 1
# 校验: None

# 3. 进入 NSH
nsh> help
```

**Linux**：

```bash
# 1. 查看串口设备
ls /dev/ttyACM*
# 通常是 /dev/ttyACM0

# 2. 使用 minicom
sudo minicom -D /dev/ttyACM0 -b 115200

# 或使用 screen
sudo screen /dev/ttyACM0 115200
```

---

## 7. 完整文件结构（Nucleo专用）

```
MiniFlight/
├── boards/
│   └── nucleo_h743zi/              # ← Nucleo专用板目录
│       ├── nuttx-config/
│       │   ├── include/
│       │   │   └── board.h         # ← 修改：8MHz HSE, Nucleo引脚
│       │   ├── scripts/
│       │   │   └── script.ld       # 与原文档相同
│       │   └── nsh/
│       │       └── defconfig       # ← 修改：STM32H743ZI
│       └── default.cmake
├── Tools/
│   └── upload_fw_nucleo.sh         # ← Nucleo专用烧录脚本
├── .vscode/
│   └── launch_nucleo.json          # ← Nucleo专用调试配置
└── ... (其余与原文档相同)
```

---

## 8. 快速开始步骤（Nucleo专用）

### 步骤 1：硬件准备

- [ ] Nucleo-H743ZI 开发板
- [ ] USB Type-A to Mini-B 线（连接 CN1 USB ST-LINK）
- [ ] ICM42688P IMU 模块（可选）
- [ ] IST8310 磁力计模块（可选）
- [ ] 杜邦线（连接传感器到 Arduino 接口）

### 步骤 2：跳线设置

检查 Nucleo 板的跳线：

```
JP1: 选择 3.3V（如果使用 3.3V 传感器）
JP2: ON（使能 ST-Link MCO 提供 HSE）
JP5: U5V (USB 供电)
JP6: E5V (外部 5V 供电，可选)
```

### 步骤 3：修改配置文件

按照本文档第 2、3 节修改：
- [ ] `boards/nucleo_h743zi/nuttx-config/nsh/defconfig`
- [ ] `boards/nucleo_h743zi/nuttx-config/include/board.h`

### 步骤 4：连接传感器

按照本文档第 4 节接线：
- [ ] IMU → Arduino D10-D13 (SPI1)
- [ ] 磁力计 → Arduino SDA/SCL (I2C1)
- [ ] 检查 3.3V 供电

### 步骤 5：编译与烧录

```bash
# 1. 编译
mkdir build && cd build
cmake .. -DBOARD=nucleo_h743zi
make -j8

# 2. 连接 Nucleo USB ST-LINK 接口到 PC

# 3. 烧录
./Tools/upload_fw_nucleo.sh
```

### 步骤 6：串口验证

```bash
# 打开串口（COMx）
nsh> help
nsh> free
nsh> uname -a

# 启动驱动
nsh> imu start
nsh> sensor_fusion start

# 监控数据
nsh> listener sensor_accel
```

---

## 9. Nucleo 特定问题

### 问题 1：串口无输出

**症状**：连接串口后无任何输出

**检查清单**：
- [ ] 是否连接了 USB ST-LINK 接口（CN1，不是 CN14）
- [ ] 串口波特率是否为 115200
- [ ] Windows 设备管理器中是否看到虚拟串口
- [ ] 检查 defconfig 中 `CONFIG_USART3_SERIAL_CONSOLE=y`

**解决**：
```bash
# 如果 Windows 没有识别虚拟串口，安装 ST-Link 驱动
# 下载: https://www.st.com/en/development-tools/stsw-link009.html
```

### 问题 2：OpenOCD 找不到板子

**症状**：`Error: libusb_open() failed with LIBUSB_ERROR_NOT_FOUND`

**解决**：
```bash
# 1. 检查 Nucleo 是否上电（LD1 应常亮）
# 2. 检查 USB 线是否连接到 CN1（ST-LINK）
# 3. 更新 ST-Link 固件
#    - 下载 ST-Link Upgrade: https://www.st.com/en/development-tools/stsw-link007.html
#    - 升级到最新版本（V3J11M3 或更高）
```

### 问题 3：SPI 读取 IMU 失败

**症状**：`IMU init failed, WHO_AM_I mismatch`

**检查清单**：
- [ ] 接线是否正确（参考第 4.1 节）
- [ ] IMU 供电是否为 3.3V（检查 JP1 跳线）
- [ ] CS 引脚是否为 PD14（Arduino D10）
- [ ] board.h 中 SPI1 引脚是否正确

**调试**：
```bash
# 使用逻辑分析仪查看 SPI 波形
# - SCK: PA5 (D13)
# - MOSI: PA7 (D11)
# - MISO: PA6 (D12)
# - CS: PD14 (D10)
#
# 检查：
# - 时钟频率: 10 MHz
# - 模式: Mode 3 (CPOL=1, CPHA=1)
# - CS 在传输时是否拉低
```

### 问题 4：时钟配置错误

**症状**：系统无法启动或频率不对

**验证时钟**：
```c
// 在 main() 中添加调试输出
printf("SYSCLK: %lu Hz\n", STM32_SYSCLK_FREQUENCY);
printf("HCLK: %lu Hz\n", STM32_HCLK_FREQUENCY);
printf("HSE: %lu Hz\n", STM32_HSE_FREQUENCY);

// 预期输出：
// SYSCLK: 480000000 Hz
// HCLK: 240000000 Hz
// HSE: 8000000 Hz
```

**如果不对，重新检查**：
- [ ] STM32_BOARD_XTAL = 8000000（不是 25000000）
- [ ] PLLM = 2, PLLN = 240（不是其他值）
- [ ] JP2 跳线是否闭合（使能 ST-Link MCO）

---

## 10. 参考资源

### 官方文档

- [Nucleo-H743ZI 用户手册 (UM2407)](https://www.st.com/resource/en/user_manual/um2407-stm32h7-nucleo144-boards-mb1364-stmicroelectronics.pdf)
- [STM32H743ZI 数据手册](https://www.st.com/resource/en/datasheet/stm32h743zi.pdf)
- [STM32H7 参考手册 (RM0433)](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

### 引脚映射

完整引脚映射：[Nucleo-H743ZI Pinout PDF](https://os.mbed.com/platforms/ST-Nucleo-H743ZI/)

### NuttX 支持

NuttX 已支持 Nucleo-H743ZI：
```bash
# 查看 NuttX 中的 Nucleo 配置示例
cd platforms/nuttx/NuttX/nuttx
./tools/configure.sh nucleo-h743zi:nsh
```

---

## 11. 总结

### Nucleo-H743ZI 核心修改点

| 配置项 | 原教程(EVAL板) | Nucleo-H743ZI |
|--------|--------------|---------------|
| **芯片型号** | STM32H743II | STM32H743ZI |
| **HSE频率** | 25 MHz | 8 MHz (ST-Link MCO) |
| **PLL1M** | 5 | 2 |
| **PLL1N** | 192 | 240 |
| **LED1** | PE3 | PB0 (LD1 绿) |
| **SPI1_CS** | PA4 | PD14 (D10) |
| **I2C1_SCL** | PB6 | PB8 (Arduino SCL) |
| **I2C1_SDA** | PB7 | PB9 (Arduino SDA) |
| **USART3_TX** | PD8 | PD8 (相同) |
| **USART3_RX** | PD9 | PD9 (相同) |
| **调试器** | 外部 ST-Link | 板载 ST-Link V3 |

### 优势

✅ **板载调试器**：无需外接 ST-Link
✅ **虚拟串口**：无需 USB-TTL 模块
✅ **Arduino接口**：传感器连接方便
✅ **官方支持**：NuttX 已有配置模板

---

**文档版本**: v1.0
**更新日期**: 2025-01-15
**适用硬件**: Nucleo-H743ZI
**主文档**: stm32h743_minimal_flight_controller_guide.md
