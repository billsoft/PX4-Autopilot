---
文档版本: 1.0
适用PX4版本: v1.14.x - v1.15.x
最后更新: 2025-11-26
文档类型: 技术原理详解
难度等级: ⭐⭐⭐ (中高级)
前置要求: 理解 STM32 外设概念, 了解 NuttX 基础
专门解答: "是否需要自己写驱动？是否需要 CubeMX 生成代码？"
---

# NuttX 对 STM32H7 外设的支持详解：你需要做什么？

## 🎯 本文档解答的核心问题

你有一个 **Nucleo-H743ZI 开发板**（或任何基于 STM32H743 的板子），想运行 PX4，你需要：

1. ❓ **自己写 SPI/I2C/UART 等外设驱动吗？**
2. ❓ **用 CubeMX 生成代码作为参考或基础吗？**
3. ❓ **NuttX 已经支持这些外设了吗？支持到什么程度？**
4. ❓ **那我到底需要做什么？**

**简短答案（然后我会详细解释）**：

- ✅ **不需要写驱动**：NuttX 已经完整实现了 STM32H7 的所有外设驱动
- ✅ **不需要 CubeMX**：NuttX 的驱动比 CubeMX HAL 更完善、更高效
- ✅ **你只需要配置**：通过 defconfig + 引脚映射完成配置
- ⚠️ **唯一需要写的**：板级初始化代码（GPIO 引脚配置、传感器片选等）

---

## 第一部分：NuttX 对 STM32H7 的支持现状

### 1.1 NuttX 已经支持的外设（完整列表）

NuttX 对 STM32H743 的支持**非常完善**，以下外设都有**现成的驱动代码**：

| 外设类型 | 支持状态 | 驱动代码位置 | 说明 |
|---------|---------|-------------|------|
| **SPI (1-6)** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_spi.c` | 支持 DMA、中断、轮询模式 |
| **I2C (1-4)** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_i2c.c` | 支持主从模式、DMA |
| **UART/USART (1-8)** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_serial.c` | 支持 DMA、流控、RS-485 |
| **USB OTG (FS/HS)** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_otgdev.c` | 支持 CDC-ACM、MSC 等 |
| **GPIO** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_gpio.c` | 支持中断、复用功能 |
| **DMA (1-2)** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_dma.c` | 支持双 DMA 控制器 |
| **定时器 (1-17)** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_tim.c` | 支持 PWM、编码器、输入捕获 |
| **ADC (1-3)** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_adc.c` | 支持 DMA、多通道 |
| **CAN/FDCAN** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_fdcan.c` | 支持 CAN FD |
| **以太网 (ETH)** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_ethernet.c` | 支持 LWIP 协议栈 |
| **SDMMC** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_sdmmc.c` | 支持 SD 卡、eMMC |
| **Flash** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_flash.c` | 支持内部 Flash 读写 |
| **RTC** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_rtc.c` | 支持低功耗模式 |
| **看门狗 (IWDG/WWDG)** | ✅ 完全支持 | `platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_wdg.c` | - |

**结论**：NuttX 对 STM32H743 的支持**覆盖所有常用外设**，不需要自己写底层驱动。

### 1.2 NuttX 驱动 vs CubeMX HAL 对比

让我们对比一下 NuttX 的驱动和 CubeMX 生成的 HAL 代码：

| 特性 | NuttX 驱动 | CubeMX HAL |
|------|-----------|-----------|
| **代码来源** | 开源社区，Apache/NuttX 项目维护 | ST 官方维护 |
| **RTOS 集成** | ✅ 深度集成 NuttX 调度器、Work Queue | ❌ 需要手动适配 FreeRTOS/其他 RTOS |
| **设备抽象** | ✅ 统一的 `/dev/` 设备节点（POSIX 风格）| ❌ 直接操作硬件寄存器 |
| **DMA 支持** | ✅ 自动管理 DMA、缓存一致性 | ⚠️ 需要手动配置 |
| **性能** | ✅ 优化过的中断处理、零拷贝 | ⚠️ 通用代码，未针对性能优化 |
| **内存占用** | ✅ 更小（PX4 优化过）| ⚠️ 较大（包含未使用的代码）|
| **多实例支持** | ✅ 原生支持（如多个 SPI 总线）| ⚠️ 需要手动处理 |
| **跨平台** | ✅ 统一 API（可移植到其他 MCU）| ❌ 只能用于 STM32 |
| **PX4 集成** | ✅ 无缝集成 uORB、Work Queue | ❌ 需要大量适配代码 |

**关键结论**：
- NuttX 的驱动**不是简单的 HAL 封装**，而是**专门为 RTOS 环境优化的驱动**
- CubeMX 生成的代码是**裸机或 FreeRTOS 导向的**，不适合 PX4 的架构
- 使用 NuttX 驱动可以直接享受 PX4 的**零拷贝、DMA 优化、中断管理**等特性

---

## 第二部分：你需要做什么？

### 2.1 配置层（defconfig）- 启用外设

**目标**：告诉 NuttX "我要使用 SPI1、I2C2、USART3"

**方法**：编辑 NuttX 的 defconfig 文件

**位置**：`platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/nucleo-h743zi/configs/nsh/defconfig`

#### 示例 1：启用 SPI1（用于连接 IMU）

```makefile
# 在 defconfig 中添加：
CONFIG_STM32H7_SPI1=y           # 启用 SPI1 外设
CONFIG_SPI=y                    # 启用 NuttX SPI 框架
CONFIG_SPI_EXCHANGE=y           # 启用双向传输
CONFIG_SPI_DRIVER=y             # 启用 /dev/spi1 设备节点

# 可选：DMA 支持（提高性能）
CONFIG_STM32H7_DMA1=y
CONFIG_SPI_DMA=y
```

**仅此而已！** 你不需要写任何 C 代码，NuttX 会自动：
1. 初始化 SPI1 外设
2. 配置时钟（从 APB2 派生）
3. 创建 `/dev/spi1` 设备节点
4. 提供标准的 `open()`, `read()`, `write()`, `ioctl()` 接口

#### 示例 2：启用 I2C1（用于连接磁力计）

```makefile
CONFIG_STM32H7_I2C1=y           # 启用 I2C1 外设
CONFIG_I2C=y                    # 启用 NuttX I2C 框架
CONFIG_I2C_DRIVER=y             # 启用 /dev/i2c1 设备节点
CONFIG_I2C_TRANSFER=y           # 启用标准传输接口
```

#### 示例 3：启用 USART3（用于控制台）

```makefile
CONFIG_STM32H7_USART3=y         # 启用 USART3 外设
CONFIG_USART3_SERIAL_CONSOLE=y  # 作为串口控制台
CONFIG_USART3_BAUD=115200       # 波特率 115200
CONFIG_USART3_BITS=8            # 数据位 8
CONFIG_USART3_PARITY=0          # 无校验
CONFIG_USART3_2STOP=0           # 1 个停止位

# 可选：DMA 支持
CONFIG_USART3_RXDMA=y           # 接收 DMA
CONFIG_USART3_TXDMA=y           # 发送 DMA
```

**对比 CubeMX**：
- CubeMX 中你需要在图形界面点选外设、配置参数、生成代码
- NuttX 中你只需要在 defconfig 中写几行配置

### 2.2 引脚层（GPIO 配置）- 映射引脚

**目标**：告诉 NuttX "SPI1 的 SCK 引脚是 PA5，MOSI 是 PA7"

**为什么需要配置引脚？**
- STM32 的引脚是**多功能复用的**（如 PA5 可以是 SPI1_SCK、TIM2_CH1、GPIO 等）
- 不同的板子可能使用不同的引脚映射（如 Pixhawk 6X 和 Nucleo-H743ZI 的引脚不同）

**方法**：在板级初始化代码中配置 GPIO

#### 你需要写的代码（在板级初始化文件中）

**文件位置**：`platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/nucleo-h743zi/src/stm32_spi.c`

```c
/**
 * @file stm32_spi.c
 * Nucleo-H743ZI 板级 SPI 初始化
 */

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

/**
 * SPI1 引脚配置（参考 Nucleo-H743ZI 原理图）
 *
 * Arduino 接口引脚映射：
 * - SCK:  PA5  (Arduino D13)
 * - MISO: PA6  (Arduino D12)
 * - MOSI: PA7  (Arduino D11)
 * - CS:   PD14 (Arduino D10) - 需要手动管理片选
 */

void stm32_spidev_initialize(void)
{
#ifdef CONFIG_STM32H7_SPI1
    /* 配置 SPI1 引脚 */
    /* NuttX 会自动配置 SCK/MISO/MOSI，我们只需要配置片选（CS）*/

    /* 配置 IMU 的片选引脚（假设 ICM-20948 连接到 PD14）*/
    stm32_configgpio(GPIO_SPI1_CS_ICM20948);

    /* 初始时拉高（未选中）*/
    stm32_gpiowrite(GPIO_SPI1_CS_ICM20948, true);
#endif
}

/**
 * SPI 片选控制（由 NuttX SPI 驱动调用）
 */
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
    switch (devid) {
    case SPIDEV_IMU(0):  /* IMU 传感器 */
        stm32_gpiowrite(GPIO_SPI1_CS_ICM20948, !selected);
        break;
    }
}
```

**关键点解释**：

1. **SCK/MISO/MOSI 不需要手动配置**：
   - NuttX 的 SPI 驱动会根据芯片的 **AFIO（复用功能 IO）** 自动配置
   - 它知道 STM32H743 的 SPI1_SCK 对应 PA5（复用功能 AF5）

2. **你只需要配置片选（CS）引脚**：
   - 因为片选是 GPIO，可以是任意引脚
   - 不同板子的片选引脚不同（如 Nucleo 用 Arduino 接口，Pixhawk 用专用引脚）

3. **引脚定义在 board_config.h**：

```c
// boards/st/nucleo-h743zi/src/board_config.h

/* SPI1 片选引脚定义 */
#define GPIO_SPI1_CS_ICM20948  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                                 GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN14)
```

#### 对比 CubeMX

**CubeMX 方式**：
```c
// CubeMX 生成的代码（在 main.c 中）
void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  // ... 还有几十行配置
  HAL_SPI_Init(&hspi1);
}

// 还需要配置 GPIO
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```

**NuttX 方式**：
```makefile
# defconfig 中只需要：
CONFIG_STM32H7_SPI1=y
```
NuttX 自动完成所有 GPIO 配置！

### 2.3 板级初始化层 - 设备树般的配置

**目标**：告诉 PX4 "这个板子有哪些传感器，连接在哪些总线上"

**文件位置**：`boards/st/nucleo-h743zi/src/board_config.h`

```c
/**
 * @file board_config.h
 * Nucleo-H743ZI 板级配置
 */

/* === SPI 设备定义 === */
#define PX4_SPI_BUS_SENSORS    1  /* SPI1 用于传感器 */

/* SPI1 设备：ICM-20948 IMU */
#define GPIO_SPI1_CS_ICM20948  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                                 GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN14)

/* === I2C 设备定义 === */
#define PX4_I2C_BUS_EXPANSION  1  /* I2C1 用于扩展传感器 */

/* === UART 定义 === */
#define PX4_UART_GPS           1  /* USART1 用于 GPS */
#define PX4_UART_TELEM1        2  /* USART2 用于遥测 */

/* === LED 定义 === */
#define GPIO_LED1  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                    GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN0)  // 绿色 LED
```

---

## 第三部分：实际操作流程（以 SPI IMU 为例）

### 场景：在 Nucleo-H743ZI 上连接 ICM-20948 IMU

**硬件连接**（通过 Arduino 接口）：

| ICM-20948 | Nucleo-H743ZI | STM32H743 引脚 | 说明 |
|-----------|---------------|----------------|------|
| VCC       | 3.3V          | 3.3V           | 供电 |
| GND       | GND           | GND            | 地 |
| SCL (SCK) | D13           | PA5 (SPI1_SCK) | SPI 时钟 |
| SDA (MOSI)| D11           | PA7 (SPI1_MOSI)| 主出从入 |
| AD0 (MISO)| D12           | PA6 (SPI1_MISO)| 主入从出 |
| CS        | D10           | PD14 (GPIO)    | 片选 |

### 步骤 1：配置 NuttX defconfig

```bash
cd PX4-Autopilot/platforms/nuttx/NuttX/nuttx
nano boards/arm/stm32h7/nucleo-h743zi/configs/nsh/defconfig
```

添加：
```makefile
# 启用 SPI1
CONFIG_STM32H7_SPI1=y
CONFIG_SPI=y
CONFIG_SPI_EXCHANGE=y
CONFIG_SPI_DRIVER=y

# 启用 DMA（可选，提高性能）
CONFIG_STM32H7_DMA1=y
CONFIG_SPI_DMA=y
```

### 步骤 2：配置板级引脚

**文件**：`boards/st/nucleo-h743zi/src/board_config.h`

```c
/* SPI1 片选引脚 */
#define GPIO_SPI1_CS_ICM20948  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                                 GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN14)

/* SPI 总线定义 */
#define PX4_SPI_BUS_SENSORS  1  /* SPI1 */
```

**文件**：`boards/st/nucleo-h743zi/src/stm32_spi.c`

```c
#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include "stm32_gpio.h"

void stm32_spidev_initialize(void)
{
#ifdef CONFIG_STM32H7_SPI1
    /* 配置 IMU 片选引脚 */
    stm32_configgpio(GPIO_SPI1_CS_ICM20948);
    stm32_gpiowrite(GPIO_SPI1_CS_ICM20948, true);  /* 拉高（未选中）*/
#endif
}

void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
    if (devid == SPIDEV_IMU(0)) {
        stm32_gpiowrite(GPIO_SPI1_CS_ICM20948, !selected);
    }
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
    return 0;
}
```

### 步骤 3：启用 PX4 的 IMU 驱动

**文件**：`boards/st/nucleo-h743zi/default.px4board`

```python
# 启用 ICM-20948 驱动
CONFIG_DRIVERS_IMU_INVENSENSE_ICM20948=y
```

**文件**：`boards/st/nucleo-h743zi/default.cmake`

```cmake
px4_add_board(
    ...
    DRIVERS
        imu/invensense/icm20948  # 添加 IMU 驱动
)
```

### 步骤 4：编译

```bash
make st_nucleo-h743zi_default
```

### 步骤 5：验证

烧录固件后，在串口终端：

```bash
nsh> icm20948 start -s  # 启动 IMU 驱动（SPI 模式）
nsh> icm20948 info
Device ID: 0xEA  # ICM-20948 的设备 ID
nsh> listener sensor_accel
TOPIC: sensor_accel
    timestamp: 123456789
    x: 0.05
    y: 0.02
    z: 9.81  # 重力加速度
```

---

## 第四部分：CubeMX 的作用（如果你一定要用）

虽然我强烈建议**不使用 CubeMX**，但如果你想用它作为**参考**，以下是合理的用法：

### ✅ 合理用法 1：查看引脚复用表

**问题**：我不知道 STM32H743 的 SPI1_SCK 可以映射到哪些引脚？

**解决**：
1. 打开 CubeMX，选择 STM32H743ZI
2. 点击 SPI1，查看 Pinout View
3. CubeMX 会高亮显示所有可用的引脚（如 PA5、PB3 等）

**但不要生成代码！** 只是查看引脚映射关系。

### ✅ 合理用法 2：确认硬件资源冲突

**问题**：我想同时使用 SPI1 和 TIM2_CH1，但 PA5 既是 SPI1_SCK 又是 TIM2_CH1，怎么办？

**解决**：
1. 在 CubeMX 中同时启用 SPI1 和 TIM2
2. CubeMX 会提示引脚冲突
3. 根据提示选择替代引脚（如 SPI1_SCK 改用 PB3）

**但不要生成代码！** 只是检查资源冲突。

### ✅ 合理用法 3：计算时钟树

**问题**：我想知道 SPI1 的最大时钟频率是多少？

**解决**：
1. 打开 CubeMX 的 Clock Configuration
2. 设置 HSE = 8MHz（Nucleo 板的晶振）
3. 查看 SPI1 所在的 APB2 总线频率（如 120MHz）
4. SPI1 最大频率 = APB2 / 2 = 60MHz

**但不要生成代码！** 只是计算时钟。

### ❌ 错误用法：生成代码并复制

**错误流程**：
```
CubeMX 生成代码 → 复制 stm32h7xx_hal_spi.c 到 PX4 → 编译失败
```

**为什么错误**：
1. CubeMX 的 HAL 代码与 NuttX 的驱动**架构完全不同**
2. HAL 使用 `HAL_SPI_Transmit()`，NuttX 使用 `SPI_EXCHANGE()`
3. HAL 依赖 `FreeRTOS` 或裸机，NuttX 有自己的 RTOS
4. 两者的**中断处理、DMA 管理、内存分配**都不兼容

---

## 第五部分：常见问题 FAQ

### Q1: NuttX 的驱动在哪里？我想看源码

**A**: 所有驱动都在 NuttX 子模块中：

```bash
cd PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/

ls -l
stm32_spi.c           # SPI 驱动
stm32_i2c.c           # I2C 驱动
stm32_serial.c        # UART 驱动
stm32_otgdev.c        # USB OTG 驱动
stm32_gpio.c          # GPIO 驱动
stm32_dma.c           # DMA 驱动
```

可以直接阅读这些 `.c` 文件来理解实现细节。

### Q2: 我改了 defconfig，为什么没生效？

**A**: NuttX 有缓存机制，需要清理：

```bash
make distclean        # 清理所有缓存
make st_nucleo-h743zi_default  # 重新配置并编译
```

### Q3: 我想用 USART6，但 defconfig 里没有 CONFIG_STM32H7_USART6 选项？

**A**: 检查 NuttX 的 Kconfig 文件：

```bash
cd platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7
grep -r "USART6" Kconfig
```

如果确实没有，说明 NuttX 还未支持 USART6。你需要：
1. 参考 USART1 的代码，添加 USART6 支持（这是真正需要写代码的地方）
2. 或者使用已支持的 USART1-5

### Q4: 我的板子用的是 QSPI Flash，NuttX 支持吗？

**A**: 支持，但需要配置：

```makefile
# defconfig
CONFIG_STM32H7_QUADSPI=y
CONFIG_MTD=y
CONFIG_MTD_BYTE_WRITE=y
```

然后在板级初始化代码中配置引脚（参考 `boards/px4/fmu-v6x/src/stm32_qspi.c`）。

### Q5: 我想知道某个引脚的复用功能编号（如 SPI1_SCK 是 AF5），在哪里查？

**A**: 查看芯片数据手册，或看 NuttX 的头文件：

```bash
cd platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7
cat chip/stm32h7x3xx_pinmap.h | grep SPI1_SCK

# 输出示例：
# #define GPIO_SPI1_SCK_1  (GPIO_ALT | GPIO_AF5 | GPIO_PORTA | GPIO_PIN5)
# #define GPIO_SPI1_SCK_2  (GPIO_ALT | GPIO_AF5 | GPIO_PORTB | GPIO_PIN3)
```

说明 SPI1_SCK 可以是 PA5（复用功能 5）或 PB3（复用功能 5）。

---

## 第六部分：总结 - 你的工作清单

### ✅ 你需要做的（必须）

| 步骤 | 位置 | 内容 | 示例 |
|------|------|------|------|
| 1. 配置 defconfig | `platforms/nuttx/.../defconfig` | 启用外设（SPI、I2C、UART）| `CONFIG_STM32H7_SPI1=y` |
| 2. 定义引脚宏 | `boards/.../src/board_config.h` | 定义片选、LED、按钮等 GPIO | `#define GPIO_LED1 (...)` |
| 3. 板级初始化 | `boards/.../src/init.c` | 配置 GPIO、上电传感器 | `stm32_configgpio(GPIO_LED1)` |
| 4. SPI 片选控制 | `boards/.../src/stm32_spi.c` | 实现片选回调函数 | `stm32_spi1select()` |
| 5. PX4 板级配置 | `boards/.../default.px4board` | 启用 PX4 驱动和模块 | `CONFIG_DRIVERS_IMU=y` |

### ❌ 你不需要做的（NuttX 已完成）

- ❌ 编写 SPI/I2C/UART 驱动代码
- ❌ 配置外设寄存器（如 SPI_CR1、I2C_CR1）
- ❌ 实现 DMA 传输逻辑
- ❌ 编写中断服务程序（ISR）
- ❌ 管理时钟树（NuttX 根据 defconfig 自动配置）
- ❌ 使用 CubeMX 生成代码

### 🤔 可选做的（提高性能/功能）

- ⚙️ 启用 DMA（在 defconfig 中配置）
- ⚙️ 调整 SPI 时钟分频（在驱动参数中配置）
- ⚙️ 启用硬件流控（UART）
- ⚙️ 配置低功耗模式

---

## 第七部分：与现有板级代码对比学习

**强烈建议**：参考 Pixhawk 6X（同样是 STM32H743）的板级代码

### 参考文件列表

```bash
# Pixhawk 6X 的板级代码（现成的完整示例）
boards/px4/fmu-v6x/src/
├── board_config.h          # ← 看这个：如何定义引脚
├── init.c                  # ← 看这个：如何初始化板子
├── spi.cpp                 # ← 看这个：如何配置 SPI
├── i2c.cpp                 # ← 看这个：如何配置 I2C
├── led.c                   # ← 看这个：如何控制 LED
└── timer_config.cpp        # ← 看这个：如何配置 PWM 定时器
```

**学习方法**：
1. 打开 `boards/px4/fmu-v6x/src/board_config.h`
2. 搜索 `GPIO_SPI1` 看 Pixhawk 6X 如何定义 SPI 引脚
3. 参考它的写法，改成 Nucleo-H743ZI 的引脚

### 示例对比

**Pixhawk 6X** (25MHz 晶振):
```c
// boards/px4/fmu-v6x/src/board_config.h
#define GPIO_SPI1_CS_ICM42688P  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                                  GPIO_OUTPUT_SET | GPIO_PORTH | GPIO_PIN5)
```

**Nucleo-H743ZI** (8MHz 晶振，Arduino 接口):
```c
// boards/st/nucleo-h743zi/src/board_config.h
#define GPIO_SPI1_CS_ICM20948  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                                 GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN14)
```

只是引脚不同（PH5 vs PD14），其他完全一样！

---

## 结论

### 核心要点（请牢记）

1. ✅ **NuttX 已经完整实现了 STM32H7 的所有外设驱动**
   - 你不需要写 SPI/I2C/UART 驱动
   - 你不需要操作寄存器
   - 你不需要实现 DMA

2. ✅ **你只需要配置，不需要编程底层驱动**
   - defconfig：启用外设（`CONFIG_STM32H7_SPI1=y`）
   - board_config.h：定义引脚（`#define GPIO_SPI1_CS ...`）
   - init.c：初始化板子（`stm32_configgpio()`）

3. ❌ **不要使用 CubeMX 生成代码**
   - CubeMX HAL 与 NuttX 架构不兼容
   - NuttX 驱动比 HAL 更优化、更适合 PX4
   - CubeMX 只能作为引脚查询工具，不要生成代码

4. 📖 **参考现有板级代码**
   - 学习 `boards/px4/fmu-v6x/` 的写法
   - 照着它的结构创建 Nucleo-H743ZI 的板级代码
   - 99% 的代码可以直接复用，只需改引脚定义

### 下一步建议

1. 阅读 `boards/px4/fmu-v6x/src/` 目录下的代码
2. 按照 `nucleo_h743zi_step_by_step.md` 创建板级配置
3. 不要纠结底层驱动实现，相信 NuttX 已经做好了
4. 专注于板级配置：引脚映射、传感器选型、功能启用

**记住**：PX4 + NuttX 的设计哲学是**配置优于编程**。你的大部分工作是**填写配置文件**，而不是**编写驱动代码**。
