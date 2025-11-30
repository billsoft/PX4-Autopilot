---
文档版本: 1.0
适用范围: 在 PX4 中新增 STM32 系列硬件开发板（以 Nucleo-H743ZI 为例，流程可推广）
最后更新: 2025-11-29
---

# STM32 新开发板完整移植教程（PX4 + NuttX）

## 目标与范围
- 在 PX4 核心仓库内新增一个 STM32 开发板目标，完整落地板级配置、NuttX 集成、PX4 模块与驱动启用、启动脚本与构建验证。
- 示例以 ST Nucleo‑H743ZI 焊接板为基准，包含两路 SPI IMU、一路 I2C 磁力计、两路 CMOS EXTI 同步、UART3 MAVLink 输出。
- 板级时钟采用 PX4 标准方法：HSE 8MHz（ST‑LINK MCO），SYSCLK=PLL1P=480MHz，SPI123=PLL2P=192MHz（≤200MHz）。

> 意图：为新手与工程师提供“从零到上线”的完整路线图，不仅教“怎么做”，更说明“为什么这样做”、遇到问题“如何修复”。
> 作用：将硬件资源、RTOS 驱动、PX4 模块、启动脚本与构建验证串成一个闭环，形成可复用的自定义板开发范式。

## 前置条件
- 已克隆 PX4 仓库：`git clone --recursive`
- 使用自定义板目录，不修改 NuttX 子模块（子模块改动会在 `make distclean` 时丢失）。
- 构建环境建议使用 WSL（Windows）或 Linux/macOS；遵循 `CLAUDE.md` 的构建指导。

## 流程总览
1. 创建板目录结构
2. 编写 NuttX `defconfig`（启用自定义板目录）
3. 编写自包含 `board.h`（时钟树 + 引脚宏）
4. 编写 PX4 板级代码（`board_config.h`、`spi.cpp`、`i2c.cpp`、`init.c`、`led.c`）
5. 配置板级模块与驱动（`default.px4board`）
6. 编写启动脚本（`init/rc.board_sensors`）
7. WSL 构建与固件产物验证
8. 运行时验证（uORB/MAVLink/EXTI）
9. 常见问题与维护（不改子模块、HRT 冲突、SPI 时钟限制等）

> 新手须知：读完“流程总览”不必立刻动手，建议先通读“方法论与原则”和“Pin/Clock 基础”，理解约束后再按章节实践。

---

## 第 1 步：创建板目录结构
在 `boards/{vendor}/{model}/` 下创建目录与文件：
```
boards/st/nucleo-h743zi-fc/
├─ default.px4board
├─ init/
│  └─ rc.board_sensors
├─ nuttx-config/
│  ├─ include/board.h
│  └─ nsh/defconfig
└─ src/
   ├─ board_config.h
   ├─ spi.cpp
   ├─ i2c.cpp
   ├─ init.c
   └─ led.c
```

> 提示：`CONFIG_ARCH_BOARD_CUSTOM` 要求使用自定义板目录，避免依赖 NuttX 内置板结构。

> 意图：把所有硬件差异封装在一个受控位置，保证“清晰边界+可维护+可升级”。
> 步骤：先建目录与文件，再逐步填充内容（后续章节会具体展开每个文件的职责与内容）。
> 产出与验证：运行 `make list_config_targets | grep nucleo-h743zi-fc` 能看到新板目标；构建阶段加载到该目录的 `.px4board` 与 `defconfig`。

---

## 第 2 步：NuttX `defconfig`（启用自定义板目录）
文件：`boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig`

关键配置（片段）：
```kconfig
# 使用自定义板目录（让 NuttX 读取我们自己的板配置而非内置板）
CONFIG_ARCH_BOARD_CUSTOM=y
# 自定义板目录路径（相对路径，兼容 Windows/WSL 构建目录层级）
CONFIG_ARCH_BOARD_CUSTOM_DIR="../../../../boards/st/nucleo-h743zi-fc/nuttx-config"
# 使用相对路径解析（避免绝对路径在不同环境下失效）
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
# 自定义板名字（用于目标与日志标识）
CONFIG_ARCH_BOARD_CUSTOM_NAME="nucleo-h743zi-fc"

# 平台/芯片选择（ARM 架构 + STM32H7 家族 + H743ZI 型号）
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32h7"
CONFIG_ARCH_CHIP_STM32H743ZI=y

# 启用计时器框架（PX4 HRT 依赖 NuttX 的 TIMER/ONESHOT 支持）
CONFIG_TIMER=y
CONFIG_ONESHOT=y
# 不启用 TIM5 驱动（TIM5 由 PX4 HRT 直接控制，避免冲突）
# CONFIG_STM32H7_TIM5 is not set

# 外设：启用 SPI1 和 SPI3 以及 SPI 框架与交换接口
CONFIG_STM32H7_SPI1=y
CONFIG_STM32H7_SPI3=y
CONFIG_SPI=y
CONFIG_SPI_EXCHANGE=y
CONFIG_SPI_DRIVER=y

# 外设：启用 I2C1 以及 I2C 框架/传输
CONFIG_STM32H7_I2C1=y
CONFIG_I2C=y
CONFIG_I2C_DRIVER=y
CONFIG_I2C_TRANSFER=y

# 外设：启用 USART3 并设为控制台（VCP），波特率 115200
CONFIG_STM32H7_USART3=y
CONFIG_USART3_SERIAL_CONSOLE=y
CONFIG_USART3_BAUD=115200
```

逐行中文注释版（defconfig 片段）：
```kconfig
# 使用自定义板目录（让 NuttX 读取我们自己的板配置而非内置板）
CONFIG_ARCH_BOARD_CUSTOM=y
# 设置自定义板目录路径（相对路径，兼容 Windows/WSL 构建目录层级）
CONFIG_ARCH_BOARD_CUSTOM_DIR="../../../../boards/st/nucleo-h743zi-fc/nuttx-config"
# 使用相对路径解析（避免绝对路径在不同环境下失效）
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
# 自定义板的名字（用于生成目标与日志标识）
CONFIG_ARCH_BOARD_CUSTOM_NAME="nucleo-h743zi-fc"

# 选择处理器架构为 ARM（PX4/NuttX 的 H7 系列属于 ARM Cortex-M）
CONFIG_ARCH="arm"
# 选择芯片家族为 STM32H7（H743ZI 属于该系列）
CONFIG_ARCH_CHIP="stm32h7"
# 具体芯片型号（确保寄存器与驱动配置与 H743ZI 匹配）
CONFIG_ARCH_CHIP_STM32H743ZI=y

# 启用计时器框架（PX4 的 HRT 依赖 NuttX 的定时器支持）
CONFIG_TIMER=y
# 启用一次性定时器（one-shot），用于高精度时间服务封装
CONFIG_ONESHOT=y
# 重要：不要启用 TIM5 的 NuttX 驱动，TIM5 由 PX4 HRT 直接控制（避免冲突）
# CONFIG_STM32H7_TIM5 is not set

# 外设：启用 SPI1 与 SPI3 驱动（驱动会在 /dev 下创建设备节点）
CONFIG_STM32H7_SPI1=y
CONFIG_STM32H7_SPI3=y
# 启用 SPI 框架与交换接口（PX4 通过驱动层与设备交互）
CONFIG_SPI=y
CONFIG_SPI_EXCHANGE=y
CONFIG_SPI_DRIVER=y

# 外设：启用 I2C1 驱动（磁力计等通过该总线连接）
CONFIG_STM32H7_I2C1=y
CONFIG_I2C=y
CONFIG_I2C_DRIVER=y
CONFIG_I2C_TRANSFER=y

# 外设：启用 USART3 并设为控制台（VCP，波特率 115200）
CONFIG_STM32H7_USART3=y
CONFIG_USART3_SERIAL_CONSOLE=y
CONFIG_USART3_BAUD=115200
```

为什么这样配置（defconfig 原理与原则）：
- 自定义板目录：隔离板差异，避免修改 NuttX 子模块；升级与维护成本最低。
- ARM/STM32H7/H743ZI：确保寄存器与驱动匹配，否则会出现外设初始化异常或编译期不兼容。
- TIMER/ONESHOT：PX4 的 HRT 使用微秒级调度，需要 NuttX 的计时器基础；ONESHOT 为高精度计数提供支撑。
- 不启用 TIM5 驱动：TIM5 由 PX4 直接控制，若启用 NuttX 的 TIM5 驱动将与 HRT 冲突（常见致命问题）。
- SPI1/SPI3：SPI2 与以太网复用易冲突，选择 SPI3 更安全；同时启用 SPI 框架与交换接口，确保驱动层的 `/dev/spiX` 节点与事务 API 可用。
- I2C1：本板磁力计使用 I2C1；启用 I2C 框架与传输接口，确保驱动能探测设备并进行读写。
- USART3 控制台：板载 VCP 接到 USART3，115200 稳定且通用，便于 `nsh` 与日志输出。
- 设置原则：最小可用集合，只启用所需外设与框架；引用 `boards/px4/fmu-v6x` 的成熟策略；避免任何与 HRT、SPI 时钟上限相冲突的配置。
- 验证方法：`make distclean && make ...` 构建无 TIM/HRT 冲突；启动后能看到 `/dev/ttyS2` 控制台、SPI 与 I2C 设备节点存在。

> 意图：让 NuttX 用我们的板配置（而不是内置板），并启用所需外设与计时框架。
> 新手须知：`defconfig` 是 Kconfig 的快照；修改时请走 `make menuconfig`→`make savedefconfig` 的流程以避免手工遗漏。
> 验证：构建日志中出现 `PX4 config file: boards/st/nucleo-h743zi-fc/default.px4board` 与 NuttX 配置加载路径指向自定义板目录。

---

## 第 3 步：自包含 `board.h`（时钟 + 引脚宏）
文件：`boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h`

关键要点：
- 时钟树：
  - `STM32_BOARD_XTAL = 8000000ul`（HSE 8MHz 来自 ST‑LINK MCO）
  - `PLL1P = 480MHz` 作为 `SYSCLK`
  - `SPI123` 时钟源选 `PLL2P = 192MHz`（≤200MHz）
- 引脚宏：定义 SPI1（PA5/PA6/PD7）、SPI3（PC10/PC11/PB2）、I2C1（PB6/PB9）、USART3（PD8/PD9）、以及 NSS（SPI3_NSS=PA15）

时钟片段示例：
```c
#define STM32_BOARD_XTAL        8000000ul // 外部高速晶振 8MHz（ST-LINK MCO）
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL // HSE 频率引用外部晶振宏
/* PLL1 VCO = (8MHz/2)*240 = 960MHz; PLL1P/R/Q = 480/120/240MHz */ // PLL1 计算说明
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(2)   // PLL1 预分频 M1=2：8MHz→4MHz
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(240)     // PLL1 倍频 N1=240：4MHz→960MHz VCO
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)       // PLL1P=2：960MHz→480MHz（SYSCLK）
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)       // PLL1Q=4：960MHz→240MHz（USB/SDMMC 等）
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)       // PLL1R=8：960MHz→120MHz（次时钟域）

/* PLL2 用于 SPI123，192MHz */ // SPI123 内核时钟来源，满足 ≤200MHz 限制
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)   // PLL2 预分频 M2=2：8MHz→4MHz
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(96)      // PLL2 倍频 N2=96：4MHz→384MHz VCO
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)       // PLL2P=2：384MHz→192MHz（SPI123 内核）
#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2 // 选择 SPI123 时钟源为 PLL2P
```

为什么这样配置（时钟树原理与原则）：
- HSE=8MHz：来自 ST-LINK MCO，稳定且全板统一；作为所有 PLL 的基准时钟。
- PLL1P=480MHz：为 CPU/HCLK 提供最高性能，满足融合与 MAVLink 的计算负载；与 H743 的数据手册匹配。
- PLL2P=192MHz：SPI123 内核时钟必须 ≤200MHz；192MHz 是 fmu-v6x 的成熟选择，兼顾性能与驱动硬约束。
- 分离来源：SYSCLK 用 PLL1，SPI123 用 PLL2，降低外设相互耦合引起的抖动与时序问题。
- 设置原则：先满足硬约束（驱动 `#error` 检查），再考虑性能与稳定性；尽量复用官方已验证的倍频参数。
- 验证方法：编译期不出现 `Not supported SPI123 frequency`；运行期 IMU 采样/传输稳定，无超时或时钟错误日志。

逐行中文注释版（时钟片段）：
```c
// 外部高速晶振频率：8MHz（来自 ST-LINK MCO 输出）
#define STM32_BOARD_XTAL        8000000ul
// 定义 HSE 频率等于外部晶振（统一使用该宏便于维护）
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
// 说明：PLL1 配置计算过程 —— VCO = (HSE/PLL1M)*PLL1N = (8MHz/2)*240 = 960MHz
// PLL1P/R/Q 分别得到 480/120/240MHz（P 作为 SYSCLK，R/Q 供其他外设）
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(2)   // 预分频 M1=2：8MHz→4MHz，提升相位噪声表现
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(240)     // 倍频 N1=240：4MHz→960MHz VCO（高稳定性）
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)       // 输出 P1=2：960MHz→480MHz，作为 SYSCLK
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)       // 输出 Q1=4：960MHz→240MHz，供 USB/SDMMC 等外设
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)       // 输出 R1=8：960MHz→120MHz，供次要外设/时钟域

// PLL2 专供 SPI123（驱动硬约束：fKERNEL ≤ 200MHz），配置到 192MHz 满足上限
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)   // 预分频 M2=2：8MHz→4MHz（保持与 PLL1M 一致）
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(96)      // 倍频 N2=96：4MHz→384MHz VCO
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)       // 输出 P2=2：384MHz→192MHz，作为 SPI123 内核时钟
#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2 // 选择 SPI123 时钟源为 PLL2P（避免超限）
```

引脚片段示例：
```c
/* SPI1: PA5/PA6/PD7 */
#define GPIO_SPI1_SCK   (GPIO_SPI1_SCK_1  | GPIO_SPEED_50MHz)   // PA5 作为 SPI1_SCK，50MHz 满足 IMU SCK 要求
#define GPIO_SPI1_MISO  (GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz)   // PA6 作为 SPI1_MISO，输入速度与驱动匹配
#define GPIO_SPI1_MOSI  (GPIO_SPI1_MOSI_3 | GPIO_SPEED_50MHz)   // PD7 作为 SPI1_MOSI，避开 PA7 的复用冲突

/* SPI3: PC10/PC11/PB2; NSS 对齐到 PA15 */
#define GPIO_SPI3_SCK   (GPIO_SPI3_SCK_2  | GPIO_SPEED_50MHz)   // PC10 作为 SPI3_SCK
#define GPIO_SPI3_MISO  (GPIO_SPI3_MISO_2 | GPIO_SPEED_50MHz)   // PC11 作为 SPI3_MISO
#define GPIO_SPI3_MOSI  (GPIO_SPI3_MOSI_3 | GPIO_SPEED_50MHz)   // PB2 作为 SPI3_MOSI
#define GPIO_SPI3_NSS   (GPIO_SPI3_NSS_1  | GPIO_SPEED_50MHz)   // PA15 对齐为 SPI3_NSS（文档一致，片选用 GPIO）

/* I2C1: PB6/PB9 */
#define GPIO_I2C1_SCL   (GPIO_I2C1_SCL_1  | GPIO_SPEED_50MHz)   // PB6 作为 I2C1_SCL（开漏，上拉由驱动处理）
#define GPIO_I2C1_SDA   (GPIO_I2C1_SDA_2  | GPIO_SPEED_50MHz)   // PB9 作为 I2C1_SDA

/* USART3: PD8/PD9 */
#define GPIO_USART3_TX  (GPIO_USART3_TX_3 | GPIO_SPEED_100MHz)  // PD8 作为 USART3_TX（VCP）
#define GPIO_USART3_RX  (GPIO_USART3_RX_3 | GPIO_SPEED_100MHz)  // PD9 作为 USART3_RX（VCP）
```

为什么这样配置（引脚映射原理与原则）：
- SPI1 MOSI 选 PD7：避开 PA7 与以太网/板载复用冲突，提升兼容性；SCK/MISO 选常用 PA5/PA6。
- SPI3 选 PC10/PC11/PB2：该组合在 Nucleo 焊接板上冲突少，便于接入 Zio/Morpho headers。
- NSS 对齐 PA15：硬件 NSS 仅用于文档一致性；实际片选改用 GPIO 更灵活可管理多设备。
- I2C1 选 PB6/PB9：Zio D15/D14 常用映射，布线便利；开漏上拉由驱动处理。
- USART3 PD8/PD9：板载 VCP 的固定映射；设置 100MHz 提升边沿质量以稳定串口输出。
- 设置原则：以板硬件文档与焊桥（SB/JP）为准，避开 ETH/USB/SWD/LED 等占用；优先选择可维护与易接线的 AF。
- 验证方法：`stm32_boardinitialize` 后，通过示波器或驱动日志确认 SCK/CS/EXTI 行为；串口输出稳定无丢字。

逐行中文注释版（引脚片段）：
```c
/* SPI1: 选择 SCK=PA5、MISO=PA6、MOSI=PD7（避让板载占用与 ETH 复用）*/
#define GPIO_SPI1_SCK   (GPIO_SPI1_SCK_1  | GPIO_SPEED_50MHz)   // PA5 复用为 SPI1_SCK；50MHz 满足 IMU SCK 要求
#define GPIO_SPI1_MISO  (GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz)   // PA6 复用为 SPI1_MISO；输入模式随驱动配置
#define GPIO_SPI1_MOSI  (GPIO_SPI1_MOSI_3 | GPIO_SPEED_50MHz)   // PD7 复用为 SPI1_MOSI（规避默认 PA7 的复用冲突）

/* SPI3: 选择 SCK=PC10、MISO=PC11、MOSI=PB2；NSS 对齐到 PA15（硬件 NSS 仅用于文档一致）*/
#define GPIO_SPI3_SCK   (GPIO_SPI3_SCK_2  | GPIO_SPEED_50MHz)   // PC10 复用为 SPI3_SCK；50MHz 适中且稳定
#define GPIO_SPI3_MISO  (GPIO_SPI3_MISO_2 | GPIO_SPEED_50MHz)   // PC11 复用为 SPI3_MISO（板载无冲突）
#define GPIO_SPI3_MOSI  (GPIO_SPI3_MOSI_3 | GPIO_SPEED_50MHz)   // PB2 复用为 SPI3_MOSI（Zio 接口易接入）
#define GPIO_SPI3_NSS   (GPIO_SPI3_NSS_1  | GPIO_SPEED_50MHz)   // PA15 对齐为 SPI3_NSS（文档标注，实际片选用 GPIO）

/* I2C1: 选择 SCL=PB6、SDA=PB9（Zio D15/D14），开漏 + 上拉由驱动配置 */
#define GPIO_I2C1_SCL   (GPIO_I2C1_SCL_1  | GPIO_SPEED_50MHz)   // PB6 复用为 I2C1_SCL；速度设置不影响开漏行为
#define GPIO_I2C1_SDA   (GPIO_I2C1_SDA_2  | GPIO_SPEED_50MHz)   // PB9 复用为 I2C1_SDA；与 SCL 搭配常见引脚

/* USART3: 选择 TX=PD8、RX=PD9（板载 VCP），高速以确保串口稳定输出 */
#define GPIO_USART3_TX  (GPIO_USART3_TX_3 | GPIO_SPEED_100MHz)  // PD8 复用为 USART3_TX；100MHz 提升边沿质量
#define GPIO_USART3_RX  (GPIO_USART3_RX_3 | GPIO_SPEED_100MHz)  // PD9 复用为 USART3_RX；配合 VCP 使用
```

> 原理：`board.h` 是 NuttX 板级的“硬件圣经”，包含 RCC 时钟树、GPIO 宏、DMA 映射等；PX4 层只依赖其提供的硬件定义进行更高层抽象。
> 新手须知：不要 `#include` 内置板的 `board.h`，而是复制后“自包含”；任何相对路径引用内置板文件都会导致隐式耦合与构建后期报错。
> 验证：编译期无 SPI 内核频率 `#error`；链接期无 HRT 符号缺失；运行时 UART/VCP 正常输出。

---

## 第 4 步：PX4 板级代码
文件：`boards/st/nucleo-h743zi-fc/src/board_config.h`
- LED 宏：绿/黄/红
- SPI 总线 ID：`PX4_SPI_BUS_SENSORS1=1`、`PX4_SPI_BUS_SENSORS2=3`
- 片选宏：`GPIO_SPI1_CS_ICM42688P=PD14`、`GPIO_SPI3_CS_ICM42688P=PA15`
- I2C 总线：`PX4_I2C_BUS_EXPANSION=1`
- CMOS EXTI：`GPIO_CMOS_SYNC_LINE=PE3`、`GPIO_CMOS_SYNC_FRAME=PE4`
- HRT：`HRT_TIMER=5`、`HRT_TIMER_CHANNEL=1`

文件：`boards/st/nucleo-h743zi-fc/src/spi.cpp`
```cpp
const px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = { // 强符号：声明本板的 SPI 总线与设备
    initSPIBus(SPI::Bus::SPI1, {                             // SPI1 总线配置
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortD, GPIO::Pin14}), // ICM42688P/42686/45686，CS=PD14
    }),
    initSPIBus(SPI::Bus::SPI3, {                             // SPI3 总线配置
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortA, GPIO::Pin15}), // 第二路 IMU，CS=PA15
    }),
};
static constexpr bool unused = validateSPIConfig(px4_spi_buses); // 编译期一致性校验
```

为什么这样配置（SPI 总线声明原理与原则）：
- 强符号数组：平台层在链接期解析，确保总线/设备被固件静态声明，避免运行期隐式依赖。
- GPIO CS：比硬件 NSS 更灵活，可支持多设备片选与自定义时序；驱动通过 GPIO 控制片选状态。
- validateSPIConfig：编译期校验防止设备漏配或越界，问题前移至编译阶段降低调试成本。
- 设置原则：只声明确实存在的设备；总线编号与 `board_config.h` 一致；CS 引脚与 `board.h` 文档对齐。
- 验证方法：`icm42688p start -s -b {1|3}` 能探测到设备；`icm42688p status` 显示两个实例且 CS 切换正常。

逐行中文注释版（SPI 总线声明）：
```cpp
// 声明强符号 px4_spi_buses：列出所有 SPI 总线及其下挂设备，供平台层与驱动解析
const px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
    // SPI1 总线：挂接 ICM42688P/42686/45686 变体，片选使用 PD14（Zio D10）
    initSPIBus(SPI::Bus::SPI1, {
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortD, GPIO::Pin14}), // 设备类型=ICM42688P；CS=PD14
    }),
    // SPI3 总线：第二路 IMU，片选使用 PA15（Zio D20），与板文档一致
    initSPIBus(SPI::Bus::SPI3, {
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortA, GPIO::Pin15}), // 设备类型相同；CS=PA15
    }),
};
// 编译期配置校验：确保总线/设备/CS 定义一致且无越界或遗漏
static constexpr bool unused = validateSPIConfig(px4_spi_buses);
```

> 意图：在 PX4 层以声明式方式描述“有哪些总线/设备、它们的 CS/DRDY/复位引脚是什么”。
> 原理：强符号数组由平台层在链接时解析；`validateSPIConfig` 在编译期验证配置一致性，避免运行期口对不上。
> 新手须知：片选（CS）用 GPIO 最灵活；硬件 NSS 宏仅用于文档对齐，实际片选由驱动调用 GPIO 完成。

### 代码结构与职责细化
- `board_config.h`：声明 LED、CS、EXTI、HRT、SPI/I2C 总线 ID，以及统一 GPIO 初始化列表（用于 `stm32_boardinitialize`）。
- `spi.cpp`：声明 `px4_spi_buses`，为每个总线列出设备及其 CS/DRDY/Reset；编译期 `validateSPIConfig` 校验。
- `i2c.cpp`：声明 `px4_i2c_buses` 强符号；每个总线列出外设；运行时由 PX4 驱动根据总线编号挂载设备。
- `init.c`：在 `board_app_initialize` 调用 `px4_platform_init`；可在此进行板级电源管理/复位控制（如有）。
- `led.c`：板载 LED 的初始化与控制（调试用）。

> 产出与验证：构建期能看到这些源文件参与编译与链接；运行期 `dmesg`/`top` 显示模块与设备任务正常。

文件：`boards/st/nucleo-h743zi-fc/src/i2c.cpp`
- 定义 `px4_i2c_buses` 为强符号，包含 I2C1 外部设备；与类型/维度严格匹配。

示例（建议实现）：
```cpp
#include <px4_platform_common/px4_config.h> // PX4 平台通用配置
#include <drivers/device/i2c.h>             // I2C 设备/总线辅助宏
#include <board_config.h>                   // 板级宏与总线编号

// I2C 总线与外设列表（仅启用 I2C1，挂接 BMM150 磁力计）
const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
    initI2CBus(I2C::Bus::I2C1, {                           // I2C1 总线
        initI2CDevice(DRV_MAG_DEVTYPE_BMM150),             // 设备=BMM150；地址/速率由驱动配置
    }),
};

static constexpr bool unused_i2c = validateI2CConfig(px4_i2c_buses); // 编译期一致性校验
```

为什么这样配置（I2C 总线声明原理与原则）：
- 强符号 `px4_i2c_buses`：避免链接期 `undefined reference`；明确告诉平台层有哪些 I2C 设备。
- 单总线最小化：仅启用 I2C1 挂接 BMM150，减少复杂性与排障范围。
- 设置原则：类型与维度严格匹配；设备枚举使用 PX4 统一的 `DRV_*` 常量，便于日志与参数管理。
- 验证方法：`bmm150 start -I -b 1` 正常；`listener sensor_mag` 有数据；I2C 传输无 `errno` 错误。

逐行中文注释版（I2C 总线声明）：
```cpp
#include <px4_platform_common/px4_config.h> // PX4 平台通用配置与可移植性宏
#include <drivers/device/i2c.h>             // 设备抽象：I2C 总线/设备初始化辅助宏
#include <board_config.h>                   // 板级宏：总线编号、引脚等（供驱动层引用）

// 强符号数组：定义本板使用的 I2C 总线及其下挂设备（仅 I2C1）
const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
    // I2C1 总线：挂接 BMM150 磁力计（地址/速率由驱动探测与参数控制）
    initI2CBus(I2C::Bus::I2C1, {
        initI2CDevice(DRV_MAG_DEVTYPE_BMM150), // 设备类型枚举用于驱动选择与 uORB 话题命名
    }),
};

// 编译期校验：确保数组维度与类型匹配，避免链接阶段找不到符号
static constexpr bool unused_i2c = validateI2CConfig(px4_i2c_buses);
```

文件：`boards/st/nucleo-h743zi-fc/src/init.c`
- 初始化 LED、SPI 片选 GPIO；保留 `board_app_initialize()`、`board_peripheral_reset()`；不做 TIM5 驱动初始化（由 HRT 处理）。

文件：`boards/st/nucleo-h743zi-fc/src/led.c`
- 提供 LED 初始化与 on/off/toggle 接口。

---

## 第 5 步：模块与驱动配置（`.px4board`）
文件：`boards/st/nucleo-h743zi-fc/default.px4board`
- 启用基础模块：`uORB`、`mavlink`、`sensors`、自定义融合模块 `dual_imu_fusion`
- 启用驱动：`imu/invensense/icm42688p`（用 `-6` 变体支持 42686/45686）、`magnetometer/bmm150`
- 关闭不需要的功能（SD 卡、ADC、GPS、PWM 等）以精简构建。

> 意图：让固件只包含你需要的模块与驱动；减少体积与编译复杂度。
> 原理：`.px4board` 是 PX4 层的“功能选择器”，以 `CONFIG_*` 开关把模块与驱动打包进目标。
> 验证：构建日志可见被启用/禁用模块列表；产物大小与链接符号随选择变化。

---

## 第 6 步：启动脚本（`init/rc.board_sensors`）
示例：
```sh
# IMU1 在 SPI1 启动（-s 使用 SPI；-b 1 总线 1；-R 0 不旋转；-6 选择 42686 兼容变体）
icm42688p start -s -b 1 -R 0 -6

# IMU2 在 SPI3 启动（-b 3 总线 3；-R 8 示例旋转，与机体坐标对齐）
icm42688p start -s -b 3 -R 8 -6

# 磁力计在 I2C1 启动（-I 使用 I2C；-b 1 总线 1）
bmm150 start -I -b 1

# 启动 CMOS 同步模块（从 GPIO/EXTI 获取帧/行同步触发，发布 gpio_in）
cmos_sync start

# 启动双 IMU 融合模块（120 Hz 发布 vehicle_attitude 四元数）
dual_imu_fusion start

# 启用 MAVLink 四元数与原始 IMU 数据流；欧拉角降低频率以节省带宽
mavlink stream -u -r 120 -s ATTITUDE_QUATERNION
mavlink stream -u -r 120 -s HIGHRES_IMU
mavlink stream -u -r 50  -s ATTITUDE
```

逐行中文注释版（启动脚本）：
```sh
# 在 SPI1 启动第一路 ICM42688P/42686/45686（-s 使用 SPI，-b 1 指定总线 1，-R 0 不旋转，-6 选择 42686 兼容变体）
icm42688p start -s -b 1 -R 0 -6

# 在 SPI3 启动第二路 IMU（-b 3 指定总线 3，-R 8 示例将第二路坐标系旋转以与机体坐标对齐）
icm42688p start -s -b 3 -R 8 -6

# 在 I2C1 启动 BMM150 磁力计（-I 使用 I2C，-b 1 指定总线 1）
bmm150 start -I -b 1

# 启动 CMOS 同步模块（读取 EXTI 对应 GPIO，发布 gpio_in 话题用于防抖时间戳）
cmos_sync start

# 启动双 IMU 融合模块（默认调度 120 Hz，发布 vehicle_attitude 四元数）
dual_imu_fusion start

# 启用 MAVLink 四元数流 120 Hz（相机防抖主要依赖姿态四元数）
mavlink stream -u -r 120 -s ATTITUDE_QUATERNION
# 启用原始高分辨率 IMU 流 120 Hz（外部算法可使用原始数据）
mavlink stream -u -r 120 -s HIGHRES_IMU
# 启用欧拉角姿态流 50 Hz（供人读或 UI 显示，降低带宽占用）
mavlink stream -u -r 50  -s ATTITUDE
```

为什么这样配置（启动脚本原理与原则）：
- 双 IMU：`-b 1` 与 `-b 3` 对应两路 SPI，总线编号与板声明一致；`-R` 在驱动层完成坐标系统一，融合层不再翻转。
- 42686/45686 变体：`-6` 选择兼容变体，复用 icm42688p 驱动栈，减少维护成本。
- 防抖链路：`cmos_sync` 发布 `gpio_in`，`dual_imu_fusion` 发布四元数，MAVLink 以 120 Hz 外发满足相机需求。
- 流率策略：四元数/IMU 120 Hz；欧拉角 50 Hz 避免带宽浪费；可根据串口带宽调优。
- 验证方法：`uorb top` 观测速率；`mavlink status` 查看流开启；相机侧接收端观测时间戳与姿态一致性。

> 意图：把传感器与融合启动顺序脚本化，便于调试与移植；MAVLink 流率根据带宽约束调整。
> 新手须知：双 IMU 旋转只在驱动层设置（`-R`）；融合层不再做坐标系翻转，避免重复。
> 验证：`listener vehicle_attitude`/`uorb top` 观测速率；`mavlink status` 查看流开启与速率。

---

## 第 7 步：WSL 构建与验证
遵循 `CLAUDE.md`：不自动构建，用户手动执行并回传日志。

建议命令（Windows/WSL）：
```bash
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make distclean && make submodulesupdate" # 清理构建并更新子模块
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j4 2>&1 | tee build_wsl.log" # 编译目标并保存日志
```

逐行中文注释版（WSL 构建命令）：
```bash
# 进入 WSL 环境运行 Bash；切换到项目目录；清理构建与缓存；更新子模块到匹配版本
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make distclean && make submodulesupdate"
# 在 WSL 中编译自定义板目标；-j4 并行 4 线程；将所有输出同时写入 build_wsl.log 便于回传分析
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j4 2>&1 | tee build_wsl.log"
```

为什么这样配置（WSL 构建原理与原则）：
- Windows 环境隔离：WSL 提供类 Linux 构建工具链，减少 Windows 本机工具链差异带来的问题。
- `distclean` 与 `submodulesupdate`：清理旧构建与同步子模块，保证依赖一致性，避免“编译使用旧配置”。
- `-j4` 并行：在多数 Windows 主机上兼顾速度与内存占用；如遇 OOM 可下调并行度。
- `tee build_wsl.log`：保留完整日志，便于回传分析编译器/链接器错误与驱动上限提示。
- 验证方法：生成 `*.elf/*.bin/*.px4` 产物；`arm-none-eabi-size` 报告符合预期；日志尾部无 HRT/TIM 冲突与 SPI 内核报错。

验证产物：
- `build/st_nucleo-h743zi-fc_default/st_nucleo-h743zi-fc_default.{elf,bin,px4}`
- 大小报告：`arm-none-eabi-size .../st_nucleo-h743zi-fc_default.elf`

> 意图：在 Windows 环境稳定构建；日志回传用于验收。
> 新手须知：遵循 `CLAUDE.md` 不自动构建（避免 IDE 阻塞与超时），手动执行并将日志尾部回传以便分析。
> 验证：构建日志无 HRT/TIM 冲突，无 SPI 内核频率报错；固件产物三件套存在。

---

## 第 8 步：运行时验证
串口连接至 VCP（USART3）：115200；进入 `nsh>`。

常用命令：
```sh
uorb top
listener vehicle_attitude
listener gpio_in
icm42688p status
mavlink status
```

> 意图：确认数据链闭环（驱动→uORB→融合→MAVLink）。
> 新手须知：串口为 VCP（USART3），波特率 115200；可用 QGC 的 MAVLink Console 观察日志。
> 验证：`gpio_in` 有 EXTI 触发，`vehicle_attitude` 有 120 Hz 更新，MAVLink 流有四元数/IMU 外发。

---

## 第 9 步：常见问题与维护
- 不修改 NuttX 子模块：所有板级改动位于 `boards/st/nucleo-h743zi-fc/` 下。
- HRT 与 TIM 驱动冲突：使用 `HRT_TIMER=5`，`defconfig` 不启用 `CONFIG_STM32H7_TIM5=y`。
- SPI 时钟限制：`SPI123_KERNEL_CLOCK ≤ 200 MHz`；使用 `PLL2P=192 MHz`。
- 引脚冲突（ETH/USB）：SPI2 与 ETH 复用，建议用 SPI3；必要时禁用 ETH 释放引脚。
- 链接符号缺失：检查 `board.h` 宏与 `spi.cpp/i2c.cpp` 定义、`CMake` 源文件包含是否完整。

---

## 参考与附录
- 参考板：`boards/px4/fmu-v6x`（STM32H7 系列完整实现）
- 项目文档：
  - `px4_custom_board_complete_guide.md`（自定义板完整指南）
  - `spi_clock_fix_summary.md`（SPI123 ≤ 200MHz 修复策略）
  - `linker_errors_fix_guide.md`（NuttX 链接错误排查）
  - `nucleo_h743zi_pinmap.md`（官方引脚映射）
  - `nuttx_stm32h7_driver_support.md`（驱动支持）

---

## 执行清单（Checklist）
- [ ] `defconfig` 使用自定义板目录，外设与计时框架启用正确
- [ ] `board.h` 自包含，时钟/引脚宏正确，SPI123 来源为 `PLL2P`
- [ ] `board_config.h` 定义 LED/CS/CMOS EXTI/HRT 宏
- [ ] `spi.cpp/i2c.cpp/init.c/led.c` 编译纳入，符号匹配
- [ ] `.px4board` 启用需要的模块与驱动
- [ ] `rc.board_sensors` 启动两路 IMU、磁力计、同步模块与 MAVLink 流
- [ ] WSL 构建成功并生成固件；串口验证与话题/流速检查通过

---

## 原理与方法论（知乎深度）

### 为什么选择“自定义板”开发（而不是改子模块）
- NuttX 是 PX4 的 RTOS 子模块，任何直接修改都会在 `make distclean` 时被覆盖；维护成本高且不可复用。
- 自定义板将所有硬件差异封装在 `boards/{vendor}/{model}/` 下，编译系统自动链接 NuttX 的驱动与架构层，保持可升级与可维护。
- 参考对象应为已稳定的 PX4 飞控板（如 `boards/px4/fmu-v6x`），而不是 NuttX 内置通用板；前者已经解决时钟、驱动、启动序等飞控场景的细节。

### 规则与底线
- 不修改子模块：`platforms/nuttx/NuttX/nuttx`、`apps`、驱动源码保持原样。
- HRT（High Resolution Timer）仅通过 `HRT_TIMER` 宏定义定时器编号；不要启用对应 NuttX TIMx 驱动，否则产生冲突（PX4 直接操作寄存器）。
- SPI123 时钟必须 ≤ 200 MHz（驱动硬性检查），否则编译阶段报错；正确做法是用 `PLL2P` 作为 `SPI123` 来源并计算到 192 MHz。
- 引脚映射遵循板硬件优先（如 Nucleo 焊接板的 ETH/USB 占用）；SPI2 与 ETH 复用时选择 SPI3；片选（CS）使用 GPIO 而非硬件 NSS（便于灵活控制）。
- 构建流程不依赖 CubeMX 生成代码；使用 PX4 的 CMake + NuttX Kconfig 完成时钟树与外设配置。

> 产出与验证：遵守底线后，构建日志不会出现子模块差异导致的“旧配置覆盖”；HRT/TIM 冲突与 SPI 驱动上限报错均消失。

### 时钟树与 SPI 内核频率的物理原理
- STM32H7 的 SPI 外设由“内核时钟（fKERNEL）”驱动，并通过分频器生成输出时钟（fSCK）。
- 数据手册给出上限：`fKERNEL ≤ 200 MHz`，`fSCK ≤ 150 MHz`；NuttX 驱动在编译时对 `fKERNEL` 进行 `#error` 检查。
- 系统主频（SYSCLK）与不同外设的时钟来源应分离：SYSCLK 用 `PLL1P`，SPI123 用 `PLL2P`；避免不同外设共享引擎引起时序抖动。

> 验证提示：驱动层在 `stm32_spi.c` 会对 `SPI123_KERNEL_CLOCK_FREQ` 做编译期检查；一旦超限立即 `#error` 终止编译，属于“硬约束”，因此必须先规划好 PLL 分配再构建。

### HRT 的设计意图与实现方式
- PX4 的高精度时间服务（`hrt_absolute_time()` 等）需要一个 32 位定时器在 1 MHz 计数；常用 TIM5（H7 系列 32 位）。
- 实现方式：PX4 平台层直接配置定时器寄存器与中断；不依赖 NuttX TIM 驱动，避免框架层叠导致的抖动与延迟。
- 冲突防护：`board.h` 中定义 `HRT_TIMER=5`，`defconfig` 不启用 `CONFIG_STM32H7_TIM5=y`；源码中内置冲突检查（若两者同时启用会报错）。

### 引脚映射策略的工程原则
- 以板硬件文档为准：Nucleo‑H743ZI 的 ETH、USB、VCP、LED、SWD 已占用关键 AF 引脚；根据占用情况选择备用 AF 或更换总线（如 SPI3）。
- CS 使用 GPIO：片选信号可任意映射到空闲 GPIO，避免硬件 NSS 复用限制，便于多个从设备管理。
- EXTI 引脚选择：两路 CMOS 同步建议使用 `PE3/PE4`（易焊、易接入 Morpho），避免与用户按键或板载功能冲突。

---

## 详细步骤 + 设计意图说明

### Step 1：创建目录结构（意图：封装差异）
- 目的：所有板相关的差异集中在自定义目录，保证可维护与可升级。
- 关键：不要引用 NuttX 内置板的 `board.h`；而是复制并自包含（400+ 行），用于时钟与引脚定义。

> 参考：可从 `platforms/nuttx/NuttX/nuttx/boards/arm/stm32/stm32_tiny/include/board.h` 或其他相近板复制为模板，再按本板硬件逐项替换。

### Step 2：`defconfig`（意图：让 NuttX 指向我们的自定义板目录）
- 通过 `CONFIG_ARCH_BOARD_CUSTOM` 告知 NuttX 使用 `boards/st/nucleo-h743zi-fc/nuttx-config` 作为板配置根。
- 启用外设（SPI1/3、I2C1、USART3）、启用计时框架（`CONFIG_TIMER=y/CONFIG_ONESHOT=y`），并明确不启用 TIM5 驱动以避免与 HRT 冲突。

### Step 3：`board.h`（意图：时钟树与引脚的权威定义）
- 自包含所有 PLL、分频器、时钟源选择；不依赖外部板文件。
- 原理：SYSCLK 用 `PLL1P=480MHz`；SPI123 用 `PLL2P=192MHz`；分别满足性能与 SPI 驱动约束。
- 引脚宏以板文档为准：SPI1（PA5/PA6/PD7）、SPI3（PC10/PC11/PB2）、I2C1（PB6/PB9）、USART3（PD8/PD9）、NSS=PA15。
> 产出与验证：`grep SPI123SRC board.h` 应为 `PLL2`；`grep GPIO_SPI3_NSS board.h` 应为 `PA15`；构建期无 SPI 内核频率 `#error`。

### Step 4：PX4 板级代码（意图：把硬件抽象为可启动的设备）
- `board_config.h`：LED、CS、CMOS EXTI、HRT 宏定义；`PX4_GPIO_INIT_LIST` 在 `stm32_boardinitialize` 统一初始化。
- `spi.cpp`：以 `px4_spi_buses` 静态描述 SPI 总线与设备（CS 引脚、DRDY 可选），用 `validateSPIConfig` 做编译期一致性校验。
- `i2c.cpp`：以 `px4_i2c_buses` 描述 I2C 总线；务必保证类型与维度匹配，避免链接时 `undefined reference`。
- `init.c/led.c`：板级初始化与调试指示；`board_app_initialize` 调用 `px4_platform_init()` 完成平台层准备。

### Step 5：`.px4board`（意图：最小可用系统 + 必要模块）
- 启用 `mavlink`、`sensors`、`dual_imu_fusion`；驱动启用 `icm42688p`、`bmm150`。
- 关闭 SD 卡、ADC、GPS、PWM 等不需要功能，避免资源浪费与驱动冲突。
> 验证提示：`make VERBOSE=1` 可观察被编译的模块与驱动；`arm-none-eabi-size` 报告产物大小变化符合预期。

### Step 6：`rc.board_sensors`（意图：把设备启动逻辑脚本化）
- 两路 IMU 启动并设置旋转（第二路 `-R 8` 示例），磁力计、CMOS 同步、融合与 MAVLink 流；四元数与 IMU 原始数据 120 Hz 满足防抖与外部相机需求。

> 验证提示：`icm42688p status` 显示两个实例；`listener gpio_in` 有 EXTI 触发；`mavlink status` 显示流启用与频率。

### Step 7：WSL 构建（意图：在 Windows 也能稳定构建）
- 按 `CLAUDE.md` 手动运行构建；回传日志以便核验 HRT、SPI 时钟限制、符号链接等是否正常。
> 参考命令：
```
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make distclean && make submodulesupdate"
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j4 2>&1 | tee build_wsl.log"
```

### Step 8：运行时验证（意图：观察链路是否闭合）
- 通过 `listener` 与 `uorb top` 查看话题速率；检查 `mavlink stream` 的外发速率与带宽；`cmos_sync` 发布 `gpio_in` 验证 EXTI。

---

## 常见坑位与修复（含根因解释）

### 1）`undefined reference to px4_i2c_buses`
- 根因：`px4_i2c_buses` 未在板目录强符号定义，或类型/维度不匹配；链接阶段找不到符号。
- 修复：在 `boards/.../src/i2c.cpp` 定义强符号数组，严格匹配 `px4_i2c_bus_t` 类型与 `I2C_BUS_MAX_BUS_ITEMS` 维度。

### 2）HRT 与 TIM5 冲突
- 根因：启用了 `CONFIG_STM32H7_TIM5=y` 导致 NuttX 驱动占用 TIM5，与 PX4 HRT 直接控制冲突。
- 修复：`defconfig` 不启用 TIM5；仅设置 `HRT_TIMER=5` 与 `CONFIG_TIMER/CONFIG_ONESHOT`。

> 验证提示：若误启用 TIM5，链接或编译期会报错；检查 `defconfig` 确认没有 `CONFIG_STM32H7_TIM5=y` 字样。

### 3）SPI123 频率报错
- 根因：驱动检查 `SPI123_KERNEL_CLOCK_FREQ > 200MHz`；使用 `PLL1Q` 可能超限。
- 修复：SPI123 时钟源改为 `PLL2P=192MHz`；参考 `fmu-v6x` 的策略。

> 验证提示：在 `board.h` 中 `SPI123SRC` 为 `PLL2`；构建日志不再有 `Not supported SPI123 frequency`。

### 4）子模块改动丢失
- 根因：对 NuttX 子模块做了本地修改；`make distclean` 清理后丢失。
- 修复：所有改动放在 `boards/...` 自定义板目录；子模块仅作为依赖使用。

> 验证提示：`git status` 顶层干净；子模块未显示 `M` 状态；构建不会在 `distclean` 后丢失改动。

### 5）双 IMU 旋转处理重复
- 根因：在融合中手动翻转轴，同时驱动已用 `-R` 旋转；导致双重旋转。
- 修复：融合层不做额外轴翻转；完全依赖驱动 `-R` 参数对齐坐标系。

> 验证提示：两路 IMU 的原始加速度/角速度在驱动旋转后在融合层坐标一致；去除重复翻转后姿态曲线更平滑。

---

## 性能、带宽与资源规划
- SYSCLK 480MHz：保证融合与 MAVLink 流在高负载下稳定运行。
- SPI 内核 192MHz：给 IMU 足够带宽（分频后满足设备 SCK 要求），同时满足驱动上限。
- MAVLink 流：四元数与 IMU 120 Hz；若带宽不足，可下调或切换串口（USART1 独立外接）。
- 内存与闪存：通过禁用不必要模块降低镜像大小；在构建日志中查看 `arm-none-eabi-size` 报告。

> 调优建议：
- 若 VCP 带宽不足，考虑移至独立 UART（USART1）并提高波特率；或将 `ATTITUDE_QUATERNION` 下调到 60–100 Hz。
- 双 IMU 融合频率可设为 120 Hz（工作队列周期 8333us），在 CPU 负载与观测噪声之间取得平衡。
- 通过禁用未用模块（EKF2/导航/控制）减少 Flash 占用与链接时间。

---

## 维护与升级策略
- 固件升级：保持自定义板与上游 PX4 同步，避免子模块差异；必要时基于 `fmu-v6x` 学习新改动。
- 文档更新：所有引脚与时钟改动在本教程与 `nucleo_h743zi_pinmap.md` 同步更新；启动脚本与板配置保持一致。
- 代码风格：遵循 PX4 CMake 与代码规范（`px_base.cmake`、`check_format`）。

> 操作建议：
- 设定上游 remote 并定期 `fetch`；维护本地集成分支，使用 `rebase` 保持线性历史，避免子模块冲突。
- 子模块管理：在 `.gitmodules` 设置 `ignore=all` 减少噪音；必要时使用 `git -C submodule reset --hard HEAD` 恢复；所有改动集中在自定义板目录。
- 文档治理：教程与索引统一入口（README），删除过期与重复文档，保留指导性文档（Pin/Clock/驱动/构建）。

---

## FAQ（工程师常问）
**Q：为什么不用 CubeMX 生成代码？**
— A：PX4/NuttX 构建系统已集成 HAL 与驱动，更适合 RTOS 环境与模块化架构；CubeMX 生成代码与 NuttX 驱动不兼容。

**Q：HSE 是 8MHz 会不会不够？**
— A：不影响；通过 PLL 倍频获得目标主频与外设频率；关键在于合理的 PLL2P 分配保证 SPI 驱动上限。

**Q：NSS 一定要硬件控制吗？**
— A：不需要；GPIO CS 更灵活，易于多设备管理；硬件 NSS 可对齐宏实现文档一致性但不参与实际片选流程。

**Q：多 IMU 如何保证坐标系一致？**
— A：在驱动层通过 `-R` 设置每路设备的旋转；融合层不重复旋转，避免坐标系混乱。

---

## 参考配置映射（Nucleo‑H743ZI 焊接板）
- SPI1：`PA5/PA6/PD7`，CS=`PD14`（Zio D10）
- SPI3：`PC10/PC11/PB2`，CS=`PA15`（Zio D20）
- I2C1：`PB6/ PB9`（Zio D15/D14）
- USART3：`PD8/PD9`（VCP）
- EXTI：`PE3/PE4`（Morpho）

---

## NuttX 板级目录与添加指南（深入）

### NuttX boards 目录结构与作用
- 路径：`platforms/nuttx/NuttX/nuttx/boards/arm/stm32/` 下包含大量参考板（`emw3162/`、`photon/`、`maple/` 等）
- 组成（典型）：
  - `Kconfig`：此板的配置入口（供 NuttX 配置系统识别）
  - `README.txt`：板介绍与引脚资源说明（部分板）
  - `src/`：板级 C 源文件（如 `stm32_spi.c`、`stm32_boardinitialize.c`、`stm32_boot.c` 等）
  - `include/board.h`：板级硬件定义（RCC 时钟、GPIO 宏、外设引脚、DMA 映射等）
- 注意：这些是“内置板”，作为学习参考很有价值，但不建议直接修改；自定义板应在 `boards/{vendor}/{model}/` 内实现，以便 PX4 构建系统统一管理。

> 新手须知：看到子模块中的 `src/stm32_spi.c`/`include/board.h` 很容易“手痒”去改；请抑制这种冲动。正确做法是把需要的宏与初始化逻辑复制到自定义板目录并适配，而不是直接改子模块。

---

## NuttX Menuconfig（图形界面）配置指南

目标：通过 NuttX 的图形化配置界面 `menuconfig` 完成 `.config` 的编辑，并生成最小可用的 `defconfig` 快照，确保与本项目板级目标一致。

打开方式（WSL 环境）：
```bash
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default menuconfig" # 进入图形界面
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make savedefconfig" # 保存为最小快照 defconfig
```

配置原则（为什么这么配）：
- 自定义板路径：保持 `CONFIG_ARCH_BOARD_CUSTOM*` 指向 `boards/st/nucleo-h743zi-fc/nuttx-config`，所有板差异集中，避免子模块修改。
- HRT 兼容：仅启用 `CONFIG_TIMER=y`/`CONFIG_ONESHOT=y`，不要启用 `CONFIG_STM32H7_TIM5=y`（TIM5 由 PX4 HRT 接管）。
- 总线与外设：启用 SPI1/SPI3、I2C1、USART3，其他未用外设（ETH/SDMMC/USB/ADC/PWM）全部禁用，降低冲突与体积。
- DMA 与串口能力：启用 DMA 控制器、`CONFIG_PIPES`、`CONFIG_SERIAL_TERMIOS`、`CONFIG_PM` 与 `CONFIG_STM32H7_SERIAL_DISABLE_REORDERING`，确保高频 IMU 与 MAVLink 串口稳定。
- 带宽冲突：避免 SPI2（与 ETH 复用）；使用 SPI3；SPI123 内核时钟上限受板 `board.h` 管理（192MHz），不在 menuconfig 更改时钟。

常用菜单路径与建议值（以本项目为例）：
- System Type → ARM Options → Chip Selection
  - 选择 `STM32H743ZI`（确保寄存器表与驱动匹配）
- RTOS Features → Timers
  - `CONFIG_TIMER=y`、`CONFIG_ONESHOT=y`
  - 确认 `CONFIG_STM32H7_TIM5` 未启用
- Device Drivers → SPI
  - `CONFIG_SPI=y`、`CONFIG_SPI_DRIVER=y`、`CONFIG_SPI_EXCHANGE=y`
  - `CONFIG_STM32H7_SPI1=y`、`CONFIG_STM32H7_SPI3=y`
- Device Drivers → I2C
  - `CONFIG_I2C=y`、`CONFIG_I2C_DRIVER=y`、`CONFIG_I2C_TRANSFER=y`
  - `CONFIG_STM32H7_I2C1=y`
- Device Drivers → Serial
  - `CONFIG_STM32H7_USART3=y`、`CONFIG_USART3_SERIAL_CONSOLE=y`、`CONFIG_USART3_BAUD=115200`
  - `CONFIG_SERIAL_TERMIOS=y`、`CONFIG_PIPES=y`、`CONFIG_PM=y`
  - `CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y`（避免端口顺序变化导致设备名不稳定）
- Device Drivers → DMA
  - 启用 DMA 控制器（如 `CONFIG_STM32H7_DMA=y` 或分控制器选项，根据 NuttX 版本呈现）
- File Systems / Networking / USB
  - 关闭 `MMCSD/FAT/NET/ETH/USB`（本板最小飞控不使用这些）

保存与验证：
- 在 `menuconfig` 中保存后执行 `make savedefconfig` 生成最小快照；对比 `boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig`。
- 运行构建，验证：无 HRT/TIM 冲突；`/dev/ttyS2` 控制台、`/dev/spi1`、`/dev/spi3`、`/dev/i2c1` 设备节点存在。

常见误区与修复：
- 误启 TIM5 驱动：会与 HRT 冲突，构建或链接报错；在 Timers 菜单关闭 `CONFIG_STM32H7_TIM5`。
- 忽略 `CONFIG_PIPES/TERMIOS`：MAVLink 串口可能异常或无法设置端口参数；确保两者启用。
- 使用 SPI2：与 ETH 复用导致引脚冲突；改用 SPI3 并在 `board.h`/`spi.cpp` 统一声明。

### NuttX 板级 bring-up 关键链路（原理）
- 配置阶段：Kconfig → defconfig → 生成 `.config`（启用架构/外设/驱动）
- 启动阶段（内核层）：
  - `stm32_boardinitialize()`（板级早期初始化，GPIO/电源等）
  - 驱动注册（SPI/I2C/UART 等）由 NuttX 驱动层完成（基于 defconfig）
  - `/dev` 设备节点由驱动层创建（如 `/dev/spi1`、`/dev/i2c1`、串口 `/dev/ttyS2` 等）
- 应用层（PX4）：
  - `board_app_initialize()`（平台初始化，PX4 层接管）
  - ROMFS 启动脚本（`rcS`、`rc.board_sensors`）启动模块与设备驱动（PX4 驱动/模块）
- 设计意图：用 NuttX 管理外设驱动生命周期与设备节点，用 PX4 管理高层模块与业务逻辑；板差异封装在 `board.h` 与板级源代码中。

> 产出与验证：构建期生成 `.config` 并编译内核；链接期把板级与 PX4 层静态库合并；运行期能看到 `/dev/spi1`、`/dev/i2c1`、`/dev/ttyS2` 等设备，以及 PX4 启动日志与 NuttShell 提示符。

### 如何在 NuttX boards 中定位参考实现
- 查找目标外设：例如 SPI 引脚宏在 `stm32h7x3xx_pinmap.h`，板级 `board.h` 会选定某组 `GPIO_SPIx_*` 宏；外设初始化在 `stm32_spi.c`；`stm32_boardinitialize.c` 负责统一的 GPIO 初始化逻辑。
- 对比路径（示例）：
  - `platforms/nuttx/NuttX/nuttx/boards/arm/stm32/stm32_tiny/`（参考小型 STM32 板）
  - `platforms/nuttx/NuttX/nuttx/boards/arm/stm32/emw3162/`（示例板，含 `Kconfig` 与 `README.txt`）
  - 以上仅用于理念和实现参考；我们的改动全部在 `boards/st/nucleo-h743zi-fc/` 下完成。

---

## STM32 Pin 与时钟配置方法、原则与注意事项（系统化）

### Pin（AF 复用）
- 每个引脚支持多个复用功能（AF），例如 `PA5` 可作 `SPI1_SCK`、`TIM2_CH1` 等；选择 AF 要避开板上固定功能（如 ETH/USB/LED/SWD）。
- GPIO 模式：`Input`/`Output`/`AF`/`Analog`；上拉/下拉、速度（Low/Medium/High/Very High）按设备需求设置。
- EXTI：任意支持 EXTI 的 GPIO 可用；建议选空闲且易焊的引脚（如 `PE3/PE4`）。
- 片选（CS）：推荐使用普通 GPIO 作为片选，便于管理多个从设备；硬件 NSS 可对齐宏（如 `SPI3_NSS=PA15`）但不参与实际片选逻辑。

> 新手须知：同一引脚可能同时被板载功能与你的外设占用（如 `PA7` 既连 D11 又接 ETH RMII_CRS_DV）；必须查阅板文档与焊桥配置（SB/JP）并在 `board.h` 中选择备用 AF 或更换总线。

### Clock（RCC/PLL/分频器）
- 约束：`SPI123 fKERNEL ≤ 200 MHz`、`SPI fSCK ≤ 150 MHz`；系统主频可到 480 MHz（`PLL1P`）。
- 原则：系统主频（CPU/HCLK）用 `PLL1P`；高带宽外设（SPI123）用 `PLL2P`；避免共享源引起耦合。
- 分配：
  - HSE 来自板上 ST‑LINK MCO：8 MHz；经 `PLLM/PLLN/PLLP` 生成目标频率。
  - `PLL1P=480 MHz` → SYSCLK；`PLL2P=192 MHz` → SPI123；USB/SDMMC 由 `PLL1Q` 或其他分配。
- 验证：编译后日志不会出现 `Not supported SPI123 frequency`；驱动层频率检查通过；固件链接成功。

> 新手须知：时钟树更改是“全局影响”，任何错误计算都可能在 NuttX 中期编译阶段才报错；建议先按 `fmu-v6x` 的模式抄写，再小步验证。

### DMA 与缓存一致性
- STM32H7 SPI/I2C/UART 驱动通常支持 DMA；NuttX 驱动负责配置 DMA 通道与缓存一致性（DCache 刷新/失效）
- 使用 DMA 有助于提升带宽（特别是高采样率 IMU）；注意在 defconfig 中启用 DMA 控制器（DMA1/DMA2）。

> 验证方法：在运行时观察 `icm42688p status` 的采样与传输速率；在高频下仍稳定且 CPU 占用合理，表明 DMA 生效且缓存一致性处理正确。

---

## NuttX 驱动开发 vs 裸机开发（对比与实践）

### 裸机（CubeMX/HAL）
- HAL 直接操作寄存器，通过初始化函数配置外设（`HAL_SPI_Init` 等）；以阻塞/中断方式控制设备；多线程与同步需自行管理。
- 优点：入门快；缺点：RTOS 集成差、设备抽象弱、代码复用低、不可与 PX4/NuttX 驱动栈直接兼容。

> 误区示例：直接把 `HAL_SPI_Transmit()` 相关代码移入 PX4 会与 NuttX 驱动的 SPI 接口冲突；正确路径是用 NuttX 的 `SPI_EXCHANGE`/设备节点由驱动层统一管理。

### NuttX 驱动（推荐）
- NuttX 提供统一的设备框架与 `/dev` 节点（SPI/I2C/UART）；驱动层管理中断、DMA、缓存一致性；RTOS 友好。
- 配置通过 defconfig（Kconfig）完成；板级 `board.h` 决定引脚与时钟；PX4 代码通过高层驱动访问传感器。
- 优势：模块化、可维护、跨平台支持好；可与 PX4 的消息总线（uORB）与工作队列无缝集成。

> 验证提示：SPI/I2C/UART 的 `/dev` 设备存在；PX4 驱动能通过总线编号与 CS 引脚成功探测设备；工作队列周期与 `hrt` 时间戳一致。

### 在 PX4 下开发驱动要点
- 尽量复用现有驱动（如 `imu/invensense/icm42688p`、`magnetometer/bmm150`）；通过板目录与启动脚本注入具体总线/片选。
- 若需新设备：遵循 PX4 的驱动开发模式（`px4_add_module`，设备枚举、参数化、uORB 发布），并在 `.px4board` 启用。

> 意图：用成熟的栈缩短开发周期，降低维护成本；把硬件差异统一到板与脚本层，驱动层复用生态能力。
> 新手须知：不要把 HAL 驱动直接复制到 PX4 项目中；这会与 NuttX 驱动栈冲突并导致不可预期行为。
> 产出与验证：新设备驱动在 `.px4board` 启用后可编译入镜像；运行期可通过 `driver status` 与 uORB 话题查看行为。

---

## Boot 与固件打包（PX4）

### Boot 流程
- ROMFS 脚本：`rcS` → `rc.base_core` → `rc.sensors`/板级脚本；根据 `.px4board` 启用的模块加载。
- 板级初始化：`stm32_boardinitialize` 完成 GPIO 初始化；PX4 层 `board_app_initialize` 完成平台准备。

### 固件打包与产物
- 构建产物：`*.elf`（带符号）、`*.bin`（纯二进制）、`*.px4`（带元数据的 PX4 包）
- 打包：`Tools/px_mkfw.py` 添加 `board_id`、版本、哈希等元数据；避免刷错固件（QGC 会校验）
- 验证大小与段布局：`arm-none-eabi-size` 与链接脚本（`.ld`）报告。

> 新手须知：`.px4` 包含板 ID 与版本元数据，QGC 刷写时会校验匹配性；不要手工修改 `.px4` 中的 JSON 结构。

---

## 进一步阅读与路径索引
- NuttX boards：`platforms/nuttx/NuttX/nuttx/boards/arm/stm32/`（作为参考学习实现结构与板级 bring-up）
- PX4 板参考：`boards/px4/fmu-v6x/`（STM32H7 的完整实现，强烈建议对比学习）
- 本板目录：`boards/st/nucleo-h743zi-fc/`（所有实际改动与实现）

> 索引建议：
- 快速路径：先读本文第 2/4/7/10/11 章，动手完成最小板与构建，再按需深入 HRT、驱动栈与排错手册。
- 参考矩阵：`fmu-v6x`（STM32H7 飞控标准实现）⇄ NuttX boards（内置板结构与 bring-up 模式）⇄ 本板目录（实际落地）。
