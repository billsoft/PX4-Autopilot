87`											---
文档版本: 1.3
适用板卡: Nucleo-H743ZI (MB1137, 非ZI2, 旧版; 兼容ZI2)
参考资料: MB1364-H743ZI-C04.pdf (原理图 Rev C-04, 2020), UM1974 Rev 11 (用户手册, August 2025), RM0433 Rev 8 (STM32H743ZI 数据手册, 2025), STM32CubeMX Nucleo-H743ZI 板模板 (默认配置)
最后更新: 2025-11-27
---

# Nucleo-H743ZI Pin 配置指南 (针对你的应用: 2 IMU SPI, 磁力计 I2C, 2 CMOS GPIO EXTI, UART 输出)

## 文档范围
- 只针对你的 Nucleo-H743ZI 开发板, 基于 CubeMX 板模板预设 Pin 配置 (你的复制列表), 结合板子硬件 (ETH/USB/LED/VCP 固定) 和你的应用需求。
- 重点: SPI1/3 (IMU1/2, 你的模板 SPI1 PD7 MOSI + SPI3 PB2 MOSI), I2C1 (磁力计, PB6 SCL 备用), GPIO EXTI (CMOS 同步, 模板 EXTI line[15:10] false, 推荐空闲 Pin), USART3 (输出, VCP 默认 PD8/9)。
- 忽略裸 MCU/通用, 只列板子实际可用 (模板 n/a Pin 可配置 GPIO, ~110 个空闲)。
- 注意: CubeMX 模板预设是正确的 (板硬件专用, e.g., ETH/USB 启用 AF PP, LED Output Low), 文档 UM1974 是通用参考 (默认 AF, 未预设模式)。不一致因模板自动分配避冲突 (e.g., PB6 SCL AF OD 备用, 有效); 你的应用用模板优先, 主从模式影响 NSS (Master Push Pull 控从, Slave Input 受控, 你的 Master 无大问题)。

## 板卡总览 (针对你的应用)
- MCU: STM32H743ZIT6 (LQFP144, Cortex-M7 @ 480 MHz, 2MB Flash, 1MB RAM)。
- 固定硬件 Pin (模板预设, 不可改或需 Disable):
  - LED: PB0 (LD1 绿, Output Low), PB7 (LD2 蓝, Output Low), PB14 (LD3 红, Output Low)。
  - 按键: PC13 (B1, EXTI Rising)。
  - USB OTG FS: PA8 (SOF AF PP), PA9 (VBUS Input), PA10 (ID AF PP), PA11 (DM AF PP), PA12 (DP AF PP), PG6 (PowerSwitch Output Low), PG7 (OverCurrent Input)。
  - VCP (USART3 输出): PD8 (TX AF PP STLK_RX), PD9 (RX AF PP STLK_TX)。
  - Debug: PA13 (SWDIO n/a TMS), PA14 (SWCLK n/a TCK), PB3 (SWO n/a)。
  - OSC/Clock: PC14 (OSC32_IN n/a), PC15 (OSC32_OUT n/a), PH0 (OSC_IN n/a MCO true), PH1 (OSC_OUT n/a)。
  - ETH (RMII): PA1 (REF_CLK AF PP), PA2 (MDIO AF PP), PA7 (CRS_DV AF PP), PB13 (TXD1 AF PP), PC1 (MDC AF PP), PC4 (RXD0 AF PP), PC5 (RXD1 AF PP), PG11 (TX_EN AF PP), PG13 (TXD0 AF PP)。
- 空闲 GPIO: ~110 个 (模板 n/a, 可配置 Input/Output/EXTI), 如 PA0/1/3/4/15, PB1/4/5/10/11/15, PC0/2/3/6/7/9/10/11/12/14/15, PD0-6/10-15, PE0-15, PF0-15, PG0-5/8-10/12/14/15, PH1。电压 3.3V, 支持 Pull up/down, Speed Low-Very High。
- 电源: USB 5V (CN1), VIN 7-12V (JP3), 3.3V 输出 (CN8 pin7 等)。
- 应用适配: IMU SPI 高速 AF PP No Pull Low, I2C OD No Pull Low, EXTI Rising No Pull, UART AF PP No Pull Low; 板子 Morpho/Zio 易焊, 模板 ETH/USB 占 Pin, 如需释放 Disable 外设。

## 插槽分区 (板子实际, 针对应用连线)
- **ST Zio (Arduino 兼容, 适合 IMU/I2C 快速接)**: CN7 (右侧上 10x2, D8-D15/GND/3.3V), CN8 (左侧上 8x2, IOREF/RESET/3.3V/5V/GND/VIN), CN9 (左侧下 15x2, A0-A5/GND/CAN/I2C2), CN10 (右侧下 17x2, D0-D7/GND/QSPI/TIM/AVDD/AGND)。
- **ST Morpho (全 Pin, 适合 CMOS GPIO/扩展 UART)**: CN11 (左侧 38x2, PC10-PD7/PE3 等), CN12 (右侧 38x2, PC9-PB13/PA10 等)。
- **USB OTG (Micro-AB, CN4, 输出备用)**: 模板预设 PA8-12/PG6/7。
- **Ethernet (RJ45, CN5, 如冲突 Disable)**: 模板预设 PA1/2/7, PB13, PC1/4/5, PG11/13。
- **ST-Link (Micro-B, CN1, VCP 输出)**: 模板预设 PD8/9。
- **电源/地**: 3.3V (CN8 pin7), GND (CN7 pin8 等), 5V (CN8 pin9)。

## 关键接口 Pin 配置 (基于你的 CubeMX 模板, 针对应用)
所有模式/配置从你的列表 (AF PP/OD No Pull Low 等), 应用建议: IMU SPI Master AF PP (推挽输出), I2C OD (开漏), EXTI Input, UART AF PP; 主从影响小 (Master 控 NSS)。

### SPI1 (IMU1)
- SCK: PA5 - AF Push Pull No Pull Low - CN7 pin10 / D13 (Zio).
- MISO: PA6 - AF Push Pull No Pull Low - CN7 pin12 / D12.
- MOSI: PD7 - AF Push Pull No Pull Low - CN11 pin45 (Morpho).
- NSS: 任意空闲 GPIO (e.g., PA4 n/a, CN7 pin17 / D24) - 软件控 Output Push Pull。
- 建议: IMU1 接 Zio D11-13, CS PA4; 模板 PD7 MOSI 备用 AF5 (避 PA7 ETH)。

### SPI2 (IMU2 备用, 模板预设)
- SCK: PB10 - AF Push Pull No Pull Low - CN12 pin25 (Morpho) or CN10 pin32 / D36 (Zio).
- MISO: PC2 - AF Push Pull No Pull Low - CN11 pin35 (Morpho) or CN9 pin9 / A4 (需 Disable ADC).
- MOSI: PC3 - AF Push Pull No Pull Low - CN11 pin37 or CN9 pin5 / A2。
- NSS: 任意空闲 (e.g., PB9 n/a I2C SDA 冲突, 用 PE0 CN10 pin33 / D34) - Output Push Pull。
- 建议: IMU2 接 Morpho, CS PE0; 模板 PC2/3_C 表示 ADC_C 复用, Disable ADC 使用。

### SPI3 (IMU2, 你的指定)
- SCK: PC10 - AF Push Pull No Pull Low - CN8 pin6 / D45 (Zio).
- MISO: PC11 - AF Push Pull No Pull Low - CN8 pin8 / D46。
- MOSI: PB2 - AF Push Pull No Pull Low - CN12 pin22 (Morpho)。
- NSS: 任意空闲 (e.g., PA15 n/a, CN7 pin9 / D20) - Output Push Pull。
- 建议: IMU2 接 Zio D45-47 (SDMMC 复用 SPI3), CS PA15; 模板 PB2 MOSI 备用 AF7 (避 QSPI PB2)。

### I2C1 (磁力计)
- SCL: PB6 - AF Open Drain No Pull Low - CN12 pin17 (Morpho) or CN10 pin13 / D26 (QSPI CS 复用)。
- SDA: PB9 - AF Open Drain No Pull Low - CN7 pin4 / D14 (Zio)。
- 建议: 磁力计接 Zio D14 (PB9), SCL PB6 备用 (避 PB8 ETH/TIM); 模板 OD No Pull (板上外拉)。

### GPIO 输入 (2 CMOS EXTI)
- 模板 EXTI line[15:10] false (未启用, CubeMX 勾 NVIC/优先 0/0)。
- 推荐空闲: PE3 n/a (CN11 pin47, EXTI3 Input Rising No Pull), PE4 n/a (CN11 pin48, EXTI4)。
- 其他可用: PA0/1/3/4/15 n/a, PB1/4/5/11/15 n/a, PC0/2/3/6/7/9/10/11/12/14/15 n/a, PD0-6/10-15 n/a, PF0-15 n/a, PG0-5/8-10/12/14/15 n/a, PH1 n/a。
- 建议: 帧同步 PE3 EXTI Rising No Pull, 行同步 PE4; 模板 PC13 EXTI B1 (可复用, 但用户按键)。

### UART 输出 (USART3 VCP 默认)
- TX: PD8 - AF Push Pull No Pull Low STLK_RX - CN11 pin67 (Morpho), 连 ST-Link VCP。
- RX: PD9 - AF Push Pull No Pull Low STLK_TX - CN11 pin69。
- 建议: 输出用 VCP (USB 虚拟 COM); 备用 UART1 PA9/10 n/a (CN12 pin21/23, AF7)。

## 设计提示 (针对你的应用)
- **应用连线**: IMU1 Zio D11-13 (PA5/6/PD7 MOSI), CS PA4; IMU2 Zio D45-47 (PC10/11/PB2 MOSI), CS PA15; I2C Zio D14 (PB9 SDA) + PB6 SCL Morpho; EXTI PE3/4 Morpho; UART VCP USB。
- **电源注意**: IMU/磁力计 3.3V (CN8 pin7), GND (CN7 pin8); VIN 外部稳压。
- **中断**: EXTI NVIC 启用, 优先 0/0 (高); 模板 false 需勾。
- **兼容**: 3.3V 传感器; SB 焊桥改复用 (Table 12, e.g., SB118 OFF 释 PB13)。
- **原理图验证**: Sheet 1 电源 (U4 开关 PG6/7), Sheet 2 MCU Pin, Sheet 6 Connectors Zio/Morpho。
- **潜在问题**: ETH 占 PA7/PB13, CubeMX Disable ETH 释放; 模板预设 USB/ETH/LED, 你的应用 Disable 不用 Pin 释放 GPIO。

## 备注
- 本文档只针对你的板模板配置 (优先正确, 板硬件专用); 空闲 n/a Pin 均可用 GPIO (Input/EXTI 等)。
- 测试: CubeIDE 生成, 验证 LED 闪 (PB0/7/14), VCP 输出, EXTI B1 按键中断。
- 图片: UM1974 p1 板视图, MB1364 Sheet 1 电源 (你的图片)。

![Nucleo-144 Top View](um1974-page1-top-view.png)
*Figure 1 from UM1974: Nucleo-144 board (top view)*

![Nucleo-144 Bottom View](um1974-page1-bottom-view.png)
*Figure 2 from UM1974: Nucleo-144 board (bottom view)*

![MB1364 Top & Power Schematic](mb1364-sheet1.png)
*MB1364 Sheet 1: Top & Power (电源和连接示意)*
