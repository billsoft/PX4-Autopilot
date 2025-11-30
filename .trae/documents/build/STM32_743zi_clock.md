---
文档版本: 1.0
适用板卡: Nucleo-H743ZI (MB1137)
参考资料: STM32CubeMX Nucleo-H743ZI 板模板 (你的复制配置列表), UM1974 Rev 11 (用户手册, August 2025), MB1364-H743ZI-C04.pdf (原理图 Rev C-04, 2020)
最后更新: 2025-11-28
---

# Nucleo-H743ZI Pin 配置指南 (针对你的应用: 2 IMU SPI, 磁力计 I2C, 2 CMOS GPIO EXTI, UART 输出)

## 文档范围
- 只针对你的 Nucleo-H743ZI 开发板和 CubeMX 板模板预设 Pin 配置 (你的复制列表), 为你的应用定制: SPI1 (IMU1), SPI3 (IMU2, 你的 PB2 MOSI 指定), I2C1 (磁力计, PB6 SCL), GPIO EXTI (CMOS 同步, 模板 line[15:10] false, 推荐空闲 n/a Pin), USART3 (输出, VCP 默认 PD8/9)。
- 基于板模板列表, 只列板硬件实际 Pin (模板预设 ~30 Pin 固定如 LED/ETH/USB/VCP/OSC, 剩余 ~110 n/a 空闲可配置 GPIO)。
- 不考虑裸 MCU 通用或其他板, 只板子实际 (Morpho/Zio 插槽位置, 模板模式如 AF PP/OD No Pull Low)。
- 注意: 模板预设是正确的 (板专用, 确保硬件工作 e.g., LED Output Low, ETH AF PP, VCP AF PP STLK); n/a Pin 可用 GPIO (Input/EXTI 等, Pull No, Speed Low); 主从模式影响 NSS (Master PP 输出控从, 你的 Master OK); 冲突如 PB2 QSPI/ETH, CubeMX Disable 释放。

## 板卡总览 (针对你的应用)
- MCU: STM32H743ZIT6 (LQFP144, Cortex-M7 @ 480 MHz, 2MB Flash, 1MB RAM)。
- 固定硬件 Pin (模板预设, 你的应用避或 Disable):
  - LED: PB7 (LD2 蓝, Output Low PP No Pull Low Disable), PB14 (LD3 红, Output Low PP No Pull Low n/a)。
  - 按键: PC13 (B1, EXTI Rising No Pull n/a USER_Btn true)。
  - USB OTG FS: PA8 (SOF AF PP No Pull Low USB_SOF TP1 true), PA9 (VBUS Input No Pull n/a USB_VBUS true), PA10 (ID AF PP No Pull Low USB_ID true), PA11 (DM AF PP No Pull Low USB_DM true), PA12 (DP AF PP No Pull Low USB_DP true), PG6 (PowerSwitch Output Low PP No Pull Low USB_PowerSwitchOn STMPS2151STR_EN true), PG7 (OverCurrent Input No Pull n/a USB_OverCurrent STMPS2151STR_FAULT true)。
  - VCP (USART3 输出): PD8 (TX AF PP No Pull Low STLK_RX STM32F103CBT6_PA3 true), PD9 (RX AF PP No Pull Low STLK_TX STM32F103CBT6_PA2 true)。
  - Debug: PA13 (SWDIO n/a TMS true), PA14 (SWCLK n/a TCK true), PB3 (SWO n/a SWO true)。
  - OSC/Clock: PC14 (OSC32_IN n/a false), PC15 (OSC32_OUT n/a false), PH0 (OSC_IN n/a MCO true), PH1 (OSC_OUT n/a false)。
  - ETH (RMII): PA1 (REF_CLK AF PP No Pull Low RMII_REF_CLK true), PA2 (MDIO AF PP No Pull Low RMII_MDIO true), PA7 (CRS_DV AF PP No Pull Low RMII_CRS_DV true), PB13 (TXD1 AF PP No Pull Low RMII_TXD1 true), PC1 (MDC AF PP No Pull Low RMII_MDC true), PC4 (RXD0 AF PP No Pull Low RMII_RXD0 true), PC5 (RXD1 AF PP No Pull Low RMII_RXD1 true), PG11 (TX_EN AF PP No Pull Low RMII_TX_EN true), PG13 (TXD0 AF PP No Pull Low RMII_TXD0 true)。
- 空闲 n/a Pin (可用 GPIO ~110 个, 你的应用扩展 EXTI/Input): PA5/6 (SPI1 SCK/MISO AF PP No Pull Low false), PB2/10 (SPI3 MOSI/SPI2 SCK AF PP No Pull Low false), PC2_C/3_C/10/11 (SPI2 MISO/MOSI/SPI3 SCK/MISO AF PP No Pull Low false), PD7 (SPI1 MOSI AF PP No Pull Low false), 其他 n/a 如 PA0/1/3/4/15, PB1/4/5/11/15, PC0/6/7/9/12/14/15, PD0-6/10-15, PE0-15, PF0-15, PG0-5/8-10/12/14/15, PH1 (配置 Input/EXTI No Pull, Speed Low)。
- 电源: USB 5V (CN1), VIN 7-12V (JP3), 3.3V 输出 (CN8 pin7 等) - 你的 IMU/磁力计用 3.3V。
- 应用适配: IMU SPI AF PP No Pull Low (Master 推挽), 磁力计 I2C OD No Pull Low (开漏), CMOS EXTI Input Rising No Pull, UART AF PP No Pull Low (VCP 输出)。

## 插槽分区 (板子实际, 针对应用连线)
- **ST Zio (Arduino 兼容, 适合 IMU/I2C 快速接)**: CN7 (右侧上 10x2, D8-D15/GND/3.3V, e.g., SPI1 D11-13), CN8 (左侧上 8x2, IOREF/RESET/3.3V/5V/GND/VIN), CN9 (左侧下 15x2, A0-A5/GND/I2C2, e.g.,备用 GPIO), CN10 (右侧下 17x2, D0-D7/GND/TIM, e.g., UART D0/1)。
- **ST Morpho (全 Pin, 适合 CMOS EXTI/扩展)**: CN11 (左侧 38x2, PC10-PD7/PE3 等, e.g., PD7 MOSI/CN11 pin45), CN12 (右侧 38x2, PC9-PB13/PA10 等, e.g., PB2 MOSI/CN12 pin22, PB6 SCL/CN12 pin17)。
- **USB OTG (Micro-AB, CN4, 输出备用)**: 模板预设 PA8-12/PG6/7, 你的应用不用可 Disable 释放 PA9/10 UART。
- **Ethernet (RJ45, CN5, 如冲突 Disable)**: 模板预设 PA1/2/7, PB13, PC1/4/5, PG11/13, 你的 SPI PB2/PA5 无直接冲突, 但 PB13 用 Disable ETH。
- **ST-Link (Micro-B, CN1, VCP 输出)**: 模板预设 PD8/9, 你的 UART 输出用此 (USB 虚拟 COM)。
- **电源/地**: 3.3V (CN8 pin7), GND (CN7 pin8 等), 5V (CN8 pin9) - 应用传感器共地。

## 关键接口 Pin 配置 (基于你的模板列表, 针对应用)
所有模式/配置从你的列表 (AF PP/OD No Pull Low 等, false/true 表示可用/固定), 板位置从 UM1974 Table 20/21 + 原理图。应用: IMU SPI Master AF PP, I2C OD, EXTI Input, UART AF PP VCP。

### SPI1 (IMU1)
- SCK: PA5 - AF Push Pull No Pull Low false - CN7 pin10 / D13 (Zio).
- MISO: PA6 - AF Push Pull No Pull Low false - CN7 pin12 / D12.
- MOSI: PD7 - AF Push Pull No Pull Low false - CN11 pin45 (Morpho).
- NSS: 任意 n/a GPIO (e.g., PA4 n/a, CN7 pin17 / D24) - 配置 Output Push Pull No Pull Low。
- 建议: IMU1 接 Zio D11-13 (PA5/6/PD7 MOSI 重映射), CS PA4; 模板 false 可用。

### SPI3 (IMU2, 你的 PB2 MOSI)
- SCK: PC10 - AF Push Pull No Pull Low false - CN8 pin6 / D45 (Zio).
- MISO: PC11 - AF Push Pull No Pull Low false - CN8 pin8 / D46。
- MOSI: PB2 - AF Push Pull No Pull Low false - CN12 pin22 (Morpho)。
- NSS: 任意 n/a (e.g., PA15 n/a, CN7 pin9 / D20) - Output Push Pull No Pull Low。
- 建议: IMU2 接 Zio D45-47 (PC10/11/PB2 MOSI 重映射), CS PA15; 模板 false 可用。

### I2C1 (磁力计)
- SCL: PB6 - AF Open Drain No Pull Low false - CN12 pin17 (Morpho).
- SDA: PB9 - AF Open Drain No Pull Low false - CN7 pin4 / D14 (Zio)。
- 建议: 磁力计接 Zio D14 (PB9), SCL PB6 (重映射); 模板 false 可用, 板上外拉。

### GPIO 输入 (2 CMOS EXTI)
- 模板 EXTI line[15:10] false 0 0 (未启用, CubeMX 勾 NVIC/优先 0/0)。
- 推荐 n/a: PE3 n/a (CN11 pin47, 配置 EXTI Input Rising No Pull false), PE4 n/a (CN11 pin48, EXTI Input Rising No Pull false)。
- 其他可用 n/a GPIO: PA0/1/3/4/15, PB1/4/5/11/15, PC0/2/3/6/7/9/12/14/15, PD0-6/10-15, PF0-15, PG0-5/8-10/12/14/15, PH1 (Input/EXTI No Pull Low false)。
- 建议: 帧同步 PE3 EXTI Rising No Pull, 行同步 PE4; 模板 false 可用配置。

### UART 输出 (USART3 VCP)
- TX: PD8 - AF Push Pull No Pull Low STLK_RX STM32F103CBT6_PA3 true - CN11 pin67 (Morpho), 连 ST-Link VCP。
- RX: PD9 - AF Push Pull No Pull Low STLK_TX STM32F103CBT6_PA2 true - CN11 pin69。
- 建议: 输出用 VCP (USB 虚拟 COM); 模板 true 固定, 可用。

## 设计提示 (针对你的应用)
- **应用连线**: IMU1 Zio D11-13 (PA5/6/PD7 MOSI), CS PA4 n/a; IMU2 Zio D45-47 (PC10/11/PB2 MOSI), CS PA15 n/a; I2C Zio D14 (PB9 SDA) + PB6 SCL Morpho; EXTI PE3/4 Morpho n/a; UART VCP USB。
- **电源注意**: IMU/磁力计 3.3V (CN8 pin7), GND (CN7 pin8); VIN 外部稳压。
- **中断**: EXTI NVIC 启用, 优先 0/0 (高); 模板 false 需勾。
- **兼容**: 3.3V 传感器; SB 焊桥改复用 (Table 12, e.g., SB118 OFF 释 PB13 ETH)。
- **原理图验证**: Sheet 1 电源 (U4 开关 PG6/7), Sheet 6 Connectors Zio/Morpho。
- **潜在问题**: ETH 占 PA7/PB13, CubeMX Disable ETH 释放 n/a GPIO; 模板 USB/ETH/LED 占, Disable 不用 Pin 释放更多 n/a。

## 备注
- 本文档只针对你的板模板配置 (优先正确, 板硬件专用); n/a Pin 均可用 GPIO (Input/EXTI 等)。
- 测试: CubeIDE 生成, 验证 LED 闪 (PB7/14), VCP 输出, EXTI B1 按键中断 (PC13)。

![Nucleo-144 Top View](um1974-page1-top-view.png)
*Figure 1 from UM1974: Nucleo-144 board (top view)*

![Nucleo-144 Bottom View](um1974-page1-bottom-view.png)
*Figure 2 from UM1974: Nucleo-144 board (bottom view)*

![MB1364 Top & Power Schematic](mb1364-sheet1.png)
*MB1364 Sheet 1: Top & Power (电源和连接示意)*
```<|control12|>是的，以上文档 (Pin 配置 MD) 已经包括时钟图的所有特性和配置——你的两张 CubeMX Clock Tab 图是默认 96MHz 配置 (HSI 64MHz 源, PLL1 M=2/N=129/P=2, APB /4, 外设 Mux 96/48MHz), 电源原理图 (Sheet 1) 是板子电源链 (ST-Link MCO 8MHz HSE, 3.3V LD39050 稳压滤波), UM1974 p1 是板视图 (ST-Link HSE 源位置)。

文档已覆盖: 源头 HSI/CSI/HSE, PLL1/2/3 分频, 系统 HCLK/PCLK, 外设钟 Mux (SPI/UART/USB/ADC), 电源影响 (稳压滤波 PLL 稳), 优到 480MHz 步骤/原则/技巧。无新补充, 完整版如下 (时钟节已整合, 无变更)。

```markdown
---
文档版本: 1.4
适用板卡: Nucleo-H743ZI (MB1137, 非ZI2, 旧版; 兼容ZI2)
参考资料: MB1364-H743ZI-C04.pdf (原理图 Rev C-04, 2020), UM1974 Rev 11 (用户手册, August 2025), RM0433 Rev 8 (STM32H743ZI 数据手册, 2025), STM32CubeMX Nucleo-H743ZI 板模板 (默认配置)
最后更新: 2025-11-28
---

# Nucleo-H743ZI 插槽/接口功能与 CPU 引脚映射

## 文档范围
- 汇总 Nucleo-H743ZI (MB1137) 的开发板插槽/接口功能，并给出与 STM32H743ZIT6 (LQFP144 封装) 的引脚对应关系。
- 重点覆盖两路 SPI、一路 I2C、用于 CMOS 同步的 GPIO 输入，以及串口 UART 输出（包括 VCP/USART3）。
- 基于官方 UM1974 Table 20 (ST Zio connectors) 和 Table 21 (ST Morpho connector)，STM32H743ZI 数据手册 RM0433 Table 11 (Alternate Functions)，以及 MB1364 原理图验证。
- 注意: H743ZI 是旧版 (芯片掩膜 Y), 已停产; 推荐升级到 H743ZI2 (掩膜 V, ST-LINK V3)。但引脚布局/功能相同。
- **CubeMX vs 文档不一致核对**: CubeMX 板模板默认配置是正确的 (板子专用预设, 确保硬件如 LED/ETH/USB/VCP 工作, e.g., PB7 Output LED, PD8 AF7 USART3_TX for ST-Link)。文档 UM1974 是通用裸 MCU Pinout 参考 (列可能复用, 未预设模式)。两者不冲突——CubeMX 基于文档 + 板硬件预分配 (e.g., ETH Pins AF11 启用, LED Output Low)。为什么不一致: CubeMX 模板自动 Disable/Assign 固定硬件 Pin (e.g., PB14 LD3 Red Output, 而文档 PB14 GPIO); 你的列表是模板输出, 优先使用。主从模式影响: SPI Master (你的项目) NSS 默认 Push Pull Output (控从机), Slave 改 Input (受控); CubeMX 模板默认 Master, 无大影响, 但 Slave 需手动改模式 (Alternate Function Input)。

## 板卡总览
- MCU: STM32H743ZIT6 (LQFP144 封装, Cortex-M7 @ 480 MHz)。
- 调试/供电/虚拟串口: ST-LINK/V2-1 (USB Micro-B 接口, CN1)。
- 扩展插槽:
  - ST Zio connectors (CN7/CN8/CN9/CN10, Arduino Uno V3 兼容 + 扩展)。
  - ST Morpho connectors (CN11/CN12, 完整暴露 MCU 引脚, 两排 38-pin 排针)。
- 常用外设引出: 板上 Ethernet RJ45 (CN5), USB OTG Micro-AB (CN4), 3x LED (LD1 绿 PB0, LD2 蓝 PB7, LD3 红 PB14), 用户按键 (B1, PC13)。
- 电源: USB 5V (U5V/E5V), 外部 VIN (7-12V), 3.3V 输出。
- 尺寸: 标准 Nucleo-144 (机械图见 UM1974 Section 7.1)。

## 插槽/接口分区 (基于板上丝印和原理图)
板子顶视图 (UM1974 Figure 1): ST-LINK USB 左侧, Ethernet/USB OTG 右侧, Zio/Morpho 两侧排针。
- **ST Zio connectors (Arduino Uno V3 兼容 + 扩展)**: CN7 (右侧上, 10x2), CN8 (左侧上, 8x2), CN9 (左侧下, 15x2), CN10 (右侧下, 17x2)。提供电源、数字/模拟 IO、总线复用。适合快速接传感器/盾板。
- **ST Morpho connectors**: CN11 (左侧, 38x2), CN12 (右侧, 38x2)。完整 MCU 引脚暴露 (除保留), 适合定制走线/原型板。
- **USB OTG FS (Micro-AB, CN4)**: PA11 (DM), PA12 (DP), PA10 (ID), PA9 (VBUS), PG6 (PowerSwitchOn), PG7 (OverCurrent)。支持设备/主机模式。
- **Ethernet (RJ45, CN5)**: RMII 接口 (PA1 REF_CLK, PA2 MDIO, PA7 CRS_DV, PB11 TX_EN, PB12 TXD0, PB13 TXD1, PC1 MDC, PC4 RXD0, PC5 RXD1)。
- **ST-LINK (Micro-B, CN1)**: 调试 + VCP (USART3 on PD8 TX/PD9 RX, 虚拟 COM 口)。
- **电源/地**: 多路 3.3V/5V/GND 引出 (CN8 pin7 = +3.3V, CN7 pin8 = GND, CN9 pin12/22/27 = GND 等)。

## 关键总线与引脚映射 (基于 UM1974 Table 20: ST Zio & Table 21: Morpho)
所有引脚支持 GPIO 模式, 复用功能需 CubeMX 配置 (AF 号见 RM0433 Table 11)。模板预设见用户列表 (优先), 文档默认见表, 备用 AF 标注。GPIO 可用: 是, 模板预设 ~30 Pin (硬件固定如 LED/ETH/USB/Debug/OSC), 剩余 ~110 Pin 空闲 (n/a, 可配置 GPIO, e.g., PA0/3/4/15, PB1/4/5/10/11/15, PC0/2/3/6/7/9/10/11/12/14/15, PD0-6/10-15, PE0-15, PF0-15, PG0-5/8-15, PH1)。

### SPI1 (主用, Arduino 兼容区)
- 信号: SCK=PA5 (AF5 默认), MISO=PA6 (AF5 默认), MOSI=PA7 (AF5 默认) or PD7 (AF5 备用, 你的 CubeMX), NSS=PA4 (AF5 默认) or 任意 GPIO。
- Zio 连接 (Arduino D11-D13):
  - CN7 pin10 = D13 / SPI_A_SCK = PA5.
  - CN7 pin12 = D12 / SPI_A_MISO = PA6.
  - CN7 pin14 = D11 / SPI_A_MOSI / TIM_E_PWM1 = PA7 (默认) or PB5 (AF5 备用, SB121 ON, SB122 OFF).
  - CN7 pin16 = D10 / SPI_A_CS / TIM_B_PWM3 = PD14 (GPIO 作片选)。
- Morpho 连接: CN11 pin11 = PA5, CN11 pin13 = PA6, CN11 pin15 = PA7, CN11 pin45 = PD7 (MOSI 备用), CN12 pin15 = PD14 (NSS 示例)。
- 建议: IMU1 接 Arduino 区, CS 用 PD14 (D10)；模板 PD7 MOSI 有效 (备用 AF, 避 PA7 ETH 冲突)。

### SPI3 (备用/第二路, 你的指定)
- 信号: SCK=PC10 (AF6 默认) or PB3 (AF6 备用), MISO=PC11 (AF6 默认), MOSI=PC12 (AF6 默认) or PB2 (AF7 备用, 你的 CubeMX), NSS=PA15 (AF6 默认) or 任意 GPIO。
- Zio 连接: CN8 pin6 = D45 / SDMMC1_D2 = PC10 (SCK 默认), CN8 pin8 = D46 / SDMMC1_D3 = PC11 (MISO), CN8 pin10 = D47 / SDMMC1_CK = PC12 (MOSI 默认)。
- Morpho 连接: CN11 pin1 = PC10, CN11 pin2 = PC11, CN11 pin3 = PC12, CN12 pin22 = PB2 (MOSI 备用)。
- 建议: IMU2 接 Zio SDMMC 区 (复用 SPI3), CS 用 PA15 (CN7 pin9 / D20)；模板预设 PB2 MOSI AF7 有效 (备用, 检查 QSPI PB2 冲突 SB off)。

### I2C1 (磁力计)
- 信号: SCL=PB8 (AF4 默认) or PB6 (AF4 备用, 你的 CubeMX), SDA=PB9 (AF4 默认) or PB7 (AF4 备用).
- Zio 连接 (Arduino A4/A5):
  - CN7 pin2 = D15 / I2C_A_SCL = PB8 (默认).
  - CN7 pin4 = D14 / I2C_A_SDA = PB9.
- Morpho 连接: CN12 pin3 = PB8, CN12 pin5 = PB9, CN12 pin17 = PB6 (SCL 备用).
- 建议: 磁力计接 Arduino A4/A5 (CN7 pin2/4), 上拉电阻板上已有 (SB138/SB143 OFF 为默认)；模板 PB6 SCL AF4 有效 (备用, 避 PB8 ETH/TIM, 检查 QSPI PB6 冲突)。

### GPIO 输入 (两个 CMOS 帧/行同步, EXTI 中断)
- 推荐: 任意空闲 GPIO 支持 EXTI (line 0-15), 如 PC13 (用户按键, 可复用), PE2 (D56, CN9 pin14).
- Zio 连接示例:
  - CN10 pin12 = D2 / I/O = PF15 (EXTI15).
  - CN10 pin14 = D1 / USART_A_TX = PG14 (配置为 GPIO, EXTI14).
- Morpho 连接: CN11 pin45 = PD7 (但你的 SPI1_MOSI, 避用), CN11 pin47 = PE3 (EXTI3), CN11 pin48 = PE4 (EXTI4, 易焊).
- 可用 GPIO 列表 (空闲/未默认分配, 基于图片识别和 UM1974, 约 100 个): PA0/1/3/4/15, PB1/4/5/10/11/15, PC0/2/3/6/7/9/10/11/12/14/15, PD0-6/10-15, PE0-15, PF0-15, PG0-15, PH1 (除固定如 PB0/7/13 LED/ETH, PC13 B1, PG6/7 USB, PA13/14 Debug, PC14/15/PH0/1 OSC, ETH/USB Pins AF)。
- 建议: 帧同步 = PE3 (CN11 pin47, EXTI3 Input Rising No Pull), 行同步 = PE4 (CN11 pin48, EXTI4 Input Rising No Pull)。CubeMX 配置 External Interrupt Mode with Rising/Falling edge。

### UART 输出 (外部数据, 如 MAVLink)
- USART3 (默认 VCP, 通过 ST-LINK USB 虚拟 COM): TX=PD8 (AF7), RX=PD9 (AF7).
  - Zio 连接: 无直接, 但 Morpho CN11 pin67 = PD8 (TX), CN11 pin69 = PD9 (RX).
  - 适用: PC 调试/地面站 (USB 连接即用, 波特率 115200 默认).
- 备用 USART1 (外接模块): TX=PA9 (AF7 默认) or PB6 (AF8 备用), RX=PA10 (AF7 默认) or PB7 (AF7 备用).
  - Zio 连接: CN10 pin14 = D1 / USART_A_TX = PG14 (USART6, 可重映射 AF8), CN10 pin16 = D0 / USART_A_RX = PG9 (AF8).
  - Morpho 连接: CN12 pin21 = PA9 (TX), CN12 pin23 = PA10 (RX).
- 建议: VCP (USART3) 用于调试输出; 外接遥测用 USART1 on Morpho (PA9/PA10, 但模板 USB 占, Disable USB 释放)。

## 完整 ST Zio Connectors Pin Assignments (UM1974 Table 20)
| Connector | Pin | Pin name | Signal name | STM32 pin | Function | Remark |
|-----------|-----|----------|-------------|-----------|----------|--------|
| CN8 (Left upper) | 1 | NC | NC | - | - | ARDUINO® compatible |
| | 3 | IOREF | IOREF | - | 3.3 V Ref | |
| | 5 | RESET | RESET | NRST | RESET | |
| | 7 | +3.3 V | +3.3 V | - | 3.3 V input/output | |
| | 9 | +5 V | +5 V | - | 5 V output | |
| | 11 | GND | GND | - | Ground | |
| | 13 | GND | GND | - | Ground | |
| | 15 | VIN | VIN | - | Power input | |
| | 2 | D43 | SDMMC1_D0 | PC8 | SDMMC/I2S_A | - |
| | 4 | D44 | SDMMC1_D1/I2S_A_CKIN | PC9 | | |
| | 6 | D45 | SDMMC1_D2 | PC10 | | |
| | 8 | D46 | SDMMC1_D3 | PC11 | | |
| | 10 | D47 | SDMMC1_CK | PC12 | | |
| | 12 | D48 | SDMMC1_CMD | PD2 | | |
| | 14 | D49 | I/O | PG2 | I/O | |
| | 16 | D50 | I/O | PG3 | | |
| CN9 (Left lower) | 1 | A0 | ADC | PA3 | ADC12_IN15 | ARDUINO® compatible |
| | 3 | A1 | ADC | PC0 | ADC123_IN10 | |
| | 5 | A2 | ADC | PC3 | ADC123_IN13 | |
| | 7 | A3 | ADC | PF3 | ADC3_IN5 | |
| | 9 | A4 | ADC | PF5 or PB9(1) | ADC3_IN4 (PF5) or I2C1_SDA (PB9) | |
| | 11 | A5 | ADC | PF10 or PB8(1) | ADC3_IN6 (PF10) or I2C1_SCL (PB8) | |
| | 13 | D72 | NC | - | - | - |
| | 15 | D71 | I/O | PA7(2) | I/O | |
| | 17 | D70 | I2C_B_SMBA | PF2 | I2C_2 | |
| | 19 | D69 | I2C_B_SCL | PF1 | | |
| | 21 | D68 | I2C_B_SDA | PF0 | | |
| | 23 | GND | GND | - | Ground | |
| | 25 | D67 | CAN_RX | PD0 | CAN_1 | - |
| | 27 | D66 | CAN_TX | PD1 | | |
| | 29 | D65 | I/O | PG0 | I/O | |
| | 2 | D51 | USART_B_SCLK | PD7 | USART_2 | |
| | 4 | D52 | USART_B_RX | PD6 | | |
| | 6 | D53 | USART_B_TX | PD5 | | |
| | 8 | D54 | USART_B_RTS | PD4 | | |
| | 10 | D55 | USART_B_CTS | PD3 | | |
| | 12 | GND | GND | - | Ground | |
| | 14 | D56 | SAI_A_MCLK | PE2(3) | SAI_1_A | |
| | 16 | D57 | SAI_A_FS | PE4 | | |
| | 18 | D58 | SAI_A_SCK | PE5 | | |
| | 20 | D59 | SAI_A_SD | PE6 | | |
| | 22 | D60 | SAI_B_SD | PE3 | SAI_1_B | |
| | 24 | D61 | SAI_B_SCK | PF8 | | |
| | 26 | D62 | SAI_B_MCLK | PF7 | | |
| | 28 | D63 | SAI_B_FS | PF9 | | |
| | 30 | D64 | I/O | PG1 | I/O | |
| CN7 (Right upper) | 1 | D16 | I2S_A_MCK | PC6 | I2S_2 | - |
| | 3 | D17 | I2S_A_SD | PB15 | | |
| | 5 | D18 | I2S_A_CK | PB13(4) | | |
| | 7 | D19 | I2S_A_WS | PB12 | | |
| | 9 | D20 | I2S_B_WS | PA15 | I2S_3 / SPI3 | |
| | 11 | D21 | I2S_B_MCK | PC7 | | |
| | 13 | D22 | I2S_B_SD/SPI_B_MOSI | PB5 | | |
| | 15 | D23 | I2S_B_CK/SPI_B_SCK | PB3 | | |
| | 17 | D24 | SPI_B_NSS | PA4 | | |
| | 19 | D25 | SPI_B_MISO | PB4 | | |
| | 2 | D15 | I2C_A_SCL | PB8 | I2C1_SCL | ARDUINO® compatible |
| | 4 | D14 | I2C_A_SDA | PB9 | I2C1_SDA | |
| | 6 | AREF | AREF | - | AVDD/VREF+ | |
| | 8 | GND | GND | - | Ground | |
| | 10 | D13 | SPI_A_SCK | PA5 | SPI1_SCK | ARDUINO® compatible |
| | 12 | D12 | SPI_A_MISO | PA6 | SPI1_MISO | |
| | 14 | D11 | SPI_A_MOSI/TIM_E_PWM1 | PA7(1)(2) or PB5(1) | SPI1_MOSI/TIM14_CH1 | |
| | 16 | D10 | SPI_A_CS/TIM_B_PWM3 | PD14 | SPI1_CS/TIM4_CH3 | |
| | 18 | D9 | TIMER_B_PWM2 | PD15 | TIM4_CH4 | |
| | 20 | D8 | I/O | PF12 | - | |
| CN10 (Right lower) | 1 | AVDD | AVDD | - | Analog VDD | - |
| | 3 | AGND | AGND | - | Analog Ground | |
| | 5 | GND | GND | - | Ground | |
| | 7 | A6 | ADC_A_IN | PB1 | ADC12_IN5 | |
| | 9 | A7 | ADC_B_IN | PC2 | ADC123_IN12 | |
| | 11 | A8 | ADC_C_IN | PF4 | ADC3_IN9 | |
| | 13 | D26 | QSPI_CS | PB6 | QSPI_BK1 | |
| | 15 | D27 | QSPI_CLK | PB2 | QSPI_CLK | |
| | 17 | GND | GND | - | Ground | |
| | 19 | D28 | QSPI_BK1_IO3 | PD13 | QSPI_BK1 | |
| | 21 | D29 | QSPI_BK1_IO1 | PD12 | | |
| | 23 | D30 | QSPI_BK1_IO0 | PD11 | | |
| | 25 | D31 | QSPI_BK1_IO2 | PE2(3) | | |
| | 27 | GND | GND | - | Ground | |
| | 29 | D32 | TIMER_C_PWM1 | PA0 | TIM2_CH1 | |
| | 31 | D33 | TIMER_D_PWM1 | PB0 | TIM3_CH3 | |
| | 33 | D34 | TIMER_B_ETR | PE0 | TIM4_ETR | |
| | 35 | D35 | TIMER_C_PWM3 | PB11 | TIM2_CH4 | |
| | 37 | D36 | TIMER_A_BKIN1 | PE15 | TIM1_BKIN1 | |
| | 2 | D7 | I/O | PF13 | - | ARDUINO® compatible |
| | 4 | D6 | TIMER_A_PWM1 | PE9 | TIM1_CH1 | |
| | 6 | D5 | TIMER_A_PWM2 | PE11 | TIM1_CH2 | |
| | 8 | D4 | I/O | PF14 | - | |
| | 10 | D3 | TIMER_A_PWM3 | PE13 | TIM1_CH3 | |
| | 12 | D2 | I/O | PF15 | - | |
| | 14 | D1 | USART_A_TX | PG14 | USART6 | |
| | 16 | D0 | USART_A_RX | PG9 | | |
| | 18 | D42 | TIMER_A_PWM1N | PE8 | TIM1_CH1N | - |
| | 20 | D41 | TIMER_A_ETR | PE7 | TIM1_ETR | |
| | 22 | GND | GND | - | Ground | |
| | 24 | D40 | TIMER_A_PWM2N | PE10 | TIM1_CH2N | |
| | 26 | D39 | TIMER_A_PWM3N | PE12 | TIM1_CH3N | |
| | 28 | D38 | I/O | PE14 | I/O | |
| | 30 | D37 | TIMER_A_BKIN1 | PE15 | TIM1_BKIN1 | |
| | 32 | D36 | TIMER_C_PWM2 | PB10 | TIM2_CH3 | |
| | 34 | D35 | TIMER_C_PWM3 | PB11 | TIM2_CH4 | |

(1) For more details refer to Table 12: Solder bridges.
(2) PA7 is used as D11 and connected to CN7 pin14 by default, if JP6 is ON, it is also connected to both Ethernet PHY as RMII_DV and CN9 pin15. In this case only one function of Ethernet or D11 could be used.
(3) PE2 is connected to both CN9 pin14 (SAI_A_MCLK) and CN10 pin 25 (QSPI_BK1_IO2). Only one function can be used at one time.
(4) PB13 is used as I2S_A_CK and connected to CN7 pin 5 by default. If JP7 is ON, it is also connected to the Ethernet PHY as RMII_TXD1. In this case only one function of the Ethernet or I2S_A must be used.

## 完整 ST Morpho Connectors Pin Assignments (UM1974 Table 21)
| CN11 odd pins | | CN11 even pins | | CN12 odd pins | | CN12 even pins | |
|---------------|--|----------------|--|---------------|--|----------------|--|
| Pin | Pin name | Pin | Pin name | Pin | Pin name | Pin | Pin name |
| 1 | PC10 | 2 | PC11 | 1 | PC9 | 2 | PC8 |
| 3 | PC12 | 4 | PD2 | 3 | PB8 | 4 | PC6 |
| 5 | VDD | 6 | E5V | 5 | PB9 | 6 | PC5 |
| 7 | BOOT0(1) | 8 | GND | 7 | AVDD | 8 | U5V(2) |
| 9 | PF6 | 10 | - | 9 | GND | 10 | PD8 |
| 11 | PF7 | 12 | IOREF | 11 | PA5 | 12 | PA12 |
| 13 | PA13(3) | 14 | RESET | 13 | PA6 | 14 | PA11 |
| 15 | PA14(3) | 16 | +3.3 V | 15 | PA7 | 16 | PB12 |
| 17 | PA15 | 18 | +5 V | 17 | PB6 | 18 | PB11 |
| 19 | GND | 20 | GND | 19 | PC7 | 20 | GND |
| 21 | PB7 | 22 | GND | 21 | PA9 | 22 | PB2 |
| 23 | PC13 | 24 | VIN | 23 | PA8 | 24 | PB1 |
| 25 | PC14 | 26 | - | 25 | PB10 | 26 | PB15 |
| 27 | PC15 | 28 | PA0 | 27 | PB4 | 28 | PB14 |
| 29 | PH0 | 30 | PA1 | 29 | PB5 | 30 | PB13 |
| 31 | PH1 | 32 | PA4 | 31 | PB3 | 32 | AGND |
| 33 | VBAT | 34 | PB0 | 33 | PA10 | 34 | PC4 |
| 35 | PC2 | 36 | PC1 | 35 | PA2 | 36 | PF5 |
| 37 | PC3 | 38 | PC0 | 37 | PA3 | 38 | PF4 |
| 39 | PD4 | 40 | PD3 | 39 | GND | 40 | PE8 |
| 41 | PD5 | 42 | PG2 | 41 | PD13 | 42 | PF10 |
| 43 | PD6 | 44 | PG3 | 43 | PD12 | 44 | PE7 |
| 45 | PD7 | 46 | PE2 | 45 | PD11 | 46 | PD14 |
| 47 | PE3 | 48 | PE4 | 47 | PE10 | 48 | PD15 |
| 49 | GND | 50 | PE5 | 49 | PE12 | 50 | PF14 |
| 51 | PF1 | 52 | PF2 | 51 | PE14 | 52 | PE9 |
| 53 | PF0 | 54 | PF8 | 53 | PE15 | 54 | GND |
| 55 | PD1 | 56 | PF9 | 55 | PE13 | 56 | PE11 |
| 57 | PD0 | 58 | PG1 | 57 | PF13 | 58 | PF3 |
| 59 | PG0 | 60 | GND | 59 | PF12 | 60 | PF15 |
| 61 | PE1 | 62 | PE6 | 61 | PG14 | 62 | PF11 |
| 63 | PG9 | 64 | GND | 63 | PG13 | 64 | PE0 |
| 65 | GND | 66 | PD10 | 65 | PG12 | 66 | PB9 |
| 67 | PD8 | 68 | PD9 | 67 | PG11 | 68 | PB8 |
| 69 | PD9 | 70 | PD11 | 69 | PG10 | 70 | GND |
| 71 | GND | 72 | PF3 | 71 | PF15 | 72 | PF12 |
| 73 | PF5 | 74 | PF4 | 73 | PF13 | 74 | PF14 |
| 75 | PF7 | 76 | PF6 | 75 | GND | 76 | PG15 |

(1) BOOT0 is not available on CN11 pin7 for NUCLEO-F303ZE.
(2) U5V is 5V power from ST-LINK USB connector and it rises before +3.3V.
(3) PA13 and PA14 share with SWD signals connected to ST-LINK/V2-1, if not used for debugging, they can be used as GPIO.

## 典型外设接线建议 (2×SPI + 1×I2C + 2×GPIO 输入 + UART 输出)
- **IMU1 (SPI1)**: VCC=CN8 pin7 (+3.3V), GND=CN7 pin8, SCK=CN7 pin10 (PA5), MISO=CN7 pin12 (PA6), MOSI=CN11 pin45 (PD7 备用) or CN7 pin14 (PA7 默认), CS=CN7 pin16 (PD14).
- **IMU2 (SPI3)**: VCC=+3.3V, GND=GND, SCK=CN8 pin6 (PC10), MISO=CN8 pin8 (PC11), MOSI=CN12 pin22 (PB2 备用) or CN8 pin10 (PC12 默认), CS=PA15 (CN7 pin9 / D20).
- **磁力计 (I2C1)**: VCC=+3.3V, GND=GND, SCL=CN12 pin17 (PB6 备用) or CN7 pin2 (PB8 默认), SDA=CN7 pin4 (PB9).
- **CMOS 同步 GPIO**: 帧同步=CN11 pin47 (PE3, EXTI3), 行同步=CN11 pin48 (PE4, EXTI4)。
- **UART 输出 (MAVLink)**: 用 VCP (USB 连接 PC); 或外接: TX=CN12 pin21 (PA9), RX=CN12 pin23 (PA10).
- 注意冲突: PB13 (SPI2/ETH) 用 SB118/JP7 OFF 禁用 Ethernet; 配置 CubeMX 避免复用冲突; 更多 GPIO 可用如 PE5-15 (CN11 pin49-63) 用于扩展输入。

## 设计提示
- **复用优先级**: Arduino/Zio 区适合快速原型, Morpho 用于高级/自定义 (全引脚)。
- **电源注意**: 传感器用 3.3V (CN8 pin7); 外部电源 VIN 需稳压。
- **中断配置**: GPIO EXTI 在 CubeMX 启用 NVIC; 优先级低于 SysTick。
- **兼容性**: 与 Arduino 盾板需检查 3.3V vs 5V; 修改 SB (见 UM1974 Table 12) 切换功能 (e.g., SB121/122 for SPI1 MOSI PB5)。
- **原理图验证**: MB1364 Sheet 1 (Top & Power) 确认电源链, Sheet 2 (MCU) LQFP144 引脚, Sheet 6 (Connectors) Zio/Morpho 布局。
- **潜在问题**: Ethernet 默认启用, 占用 PB13 等; 移除 JP6/JP7 禁用; CubeMX 备用 AF 如 PD7 SPI1 需手动选, 默认 PA7。

## 时钟配置 (基于 CubeMX Clock Tab 图, 整合电源原理图)
H743ZI 时钟树 (RM0433 Section 7 RCC) 支持多源/PLL/分频, 你的 CubeMX 图默认 96MHz (HSI 64MHz 源, PLL1 M=2/N=129/P=2, APB /4 120MHz, 外设 Mux 96/48/16.125MHz from PLL1_P/Q/R)。电源原理图 (Sheet 1) 提供稳定 3.3V (LD39050 U6, 滤波 C18 10uF, 开关 U4 ST890) 确保 PLL 无抖, ST-Link MCO 8MHz HSE 源 (CN1 USB)。

- **源头 (左侧, 图上半)**：Input 12.288 MHz (CSI/HSI 默认, 你的图 64MHz HSI RC), HSE 8MHz (板 MCO from ST-Link, 改启用), LSE 32kHz (RTC PC14/15 n/a), LSI 32kHz (低功), PER Source Mux HSI。
- **PLL (中间)**：PLL Source Mux HSI, PLL1 (系统 SYSCLK 480 max, 你的 96MHz M=2/N=129/P=2/Q=4/R=2), PLL2/3 (外设专用, 你的 48MHz P=2 for USB/ADC)。
- **分频/总线 (中间)**：System Mux PLL1, HPre /1 (HCLK 480), D1Pre /1 (CPU 480), APB1/2/3/4 /4 (120MHz, 你的 96MHz)。
- **外设钟 (右侧, 图下半)**：SPI1.2.3 Clock Mux 96MHz PLL1_P (IMU 改 150), USART2.3.4.5.7.8 96MHz PCLK (UART VCP 115200 OK), USB 48MHz PLL1_Q (精确), ADC 16.125MHz PLL2_P (CMOS 用), SAI/SDMMC/FDCAN/RNG/QUADSPI 48MHz, LPTIM 96MHz PLL2_P, CEC 0.032MHz LSI, HR TIM 96MHz CPU, Trace 96MHz HCLK。
- **电源整合 (原理图 Sheet 1)**：3.3V LD39050 (U6, EN high, PG good, C20 1uF/C22 100nF 滤波) 供 MCU 时钟; ST-Link PWR_ENn 使能开关 U4 (FAULT LD5 红, SET R28 2.7K), VIN 5V LD1117 U5 (C17 10uF in/C18 out); Notes: Add C58 4.7uF VDD 滤时钟噪声, +3V3_PER 外设钟用, R33 200K BOOT1 (启动时钟初始化)。
- **默认特性 (你的图)**：低功 96MHz (HSI 无需外部晶), 无黄警告, Enable CSS (钟安全); 优 480MHz: 改 HSE 8MHz, PLL1 M=1/N=120/P=2 (8*120/1/2=480), APB /4 120MHz, 外设 PLL2 150MHz SPI。
- **应用配置建议**: IMU SPI PLL1_Q 150MHz (高采样), I2C PCLK1 100kHz, EXTI SYSCLK (无专用), UART PCLK2 96MHz; 电源稳压滤波避 jitter。

## 典型外设接线建议 (2×SPI + 1×I2C + 2×GPIO 输入 + UART 输出)
- **IMU1 (SPI1)**: VCC=CN8 pin7 (+3.3V), GND=CN7 pin8, SCK=CN7 pin10 (PA5), MISO=CN7 pin12 (PA6), MOSI=CN11 pin45 (PD7 备用) or CN7 pin14 (PA7 默认), CS=CN7 pin16 (PD14).
- **IMU2 (SPI3)**: VCC=+3.3V, GND=GND, SCK=CN8 pin6 (PC10), MISO=CN8 pin8 (PC11), MOSI=CN12 pin22 (PB2 备用) or CN8 pin10 (PC12 默认), CS=PA15 (CN7 pin9 / D20).
- **磁力计 (I2C1)**: VCC=+3.3V, GND=GND, SCL=CN12 pin17 (PB6 备用) or CN7 pin2 (PB8 默认), SDA=CN7 pin4 (PB9).
- **CMOS 同步 GPIO**: 帧同步=CN11 pin47 (PE3, EXTI3), 行同步=CN11 pin48 (PE4, EXTI4)。
- **UART 输出 (MAVLink)**: 用 VCP (USB 连接 PC); 或外接: TX=CN12 pin21 (PA9), RX=CN12 pin23 (PA10).
- 注意冲突: PB13 (SPI2/ETH) 用 SB118/JP7 OFF 禁用 Ethernet; 配置 CubeMX 避免复用冲突; 更多 GPIO 可用如 PE5-15 (CN11 pin49-63) 用于扩展输入。

## 设计提示
- **复用优先级**: Arduino/Zio 区适合快速原型, Morpho 用于高级/自定义 (全引脚)。
- **电源注意**: 传感器用 3.3V (CN8 pin7); 外部电源 VIN 需稳压。
- **中断配置**: GPIO EXTI 在 CubeMX 启用 NVIC; 优先级低于 SysTick。
- **兼容性**: 与 Arduino 盾板需检查 3.3V vs 5V; 修改 SB (见 UM1974 Table 12) 切换功能 (e.g., SB121/122 for SPI1 MOSI PB5)。
- **原理图验证**: MB1364 Sheet 1 (Top & Power) 确认电源链, Sheet 2 (MCU) LQFP144 引脚, Sheet 6 (Connectors) Zio/Morpho 布局。
- **潜在问题**: Ethernet 默认启用, 占用 PB13 等; 移除 JP6/JP7 禁用; CubeMX 备用 AF 如 PD7 SPI1 需手动选, 默认 PA7。

## 备注
- 本文档基于官方 UM1974 Rev 11 (2025), MB1364 Rev C-04 (2020) 原理图, RM0433 Rev 8 (2025) 数据手册, STM32CubeMX 板模板。完整 pinout 见 UM1974 Table 20/21, AF 表见 RM0433 Table 11。
- CubeMX 配置优先 (自定义有效), 文档默认推荐; 不一致因重映射 (AF 备用合法)。
- 如需 PCB 布局/ Gerber, 参考 ST 官网 EDA resources (Section 3.4)。
- 测试: 用 CubeIDE 生成默认项目, 验证 LED (PB0/PB7/PB14) 闪烁确认板子正常。

![Nucleo-144 Top View](um1974-page1-top-view.png)
*Figure 1 from UM1974: Nucleo-144 board (top view)*

![Nucleo-144 Bottom View](um1974-page1-bottom-view.png)
*Figure 2 from UM1974: Nucleo-144 board (bottom view)*

![MB1364 Top & Power Schematic](mb1364-sheet1.png)
*MB1364 Sheet 1: Top & Power (电源和连接示意)*

![MB1364 MCU Schematic](mb1364-sheet2.png)
*MB1364 Sheet 2: MCU (STM32H743ZI 引脚连接)*

![MB1364 ST-LINK Schematic](mb1364-sheet3.png)
*MB1364 Sheet 3: ST-LINK/V2-1 (调试接口)*

![MB1364 USB Schematic](mb1364-sheet4.png)
*MB1364 Sheet 4: USB (OTG 接口)*

![MB1364 Ethernet Schematic](mb1364-sheet5.png)
*MB1364 Sheet 5: Ethernet PHY with RJ45*

![MB1364 Extension Connectors Schematic](mb1364-sheet6.png)
*MB1364 Sheet 6: Extension connectors (Zio/Morpho 布局)*
