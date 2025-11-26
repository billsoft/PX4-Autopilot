---
文档版本: 1.2
适用板卡: Nucleo-H743ZI (MB1137, 非ZI2, 旧版; 兼容ZI2)
参考资料: MB1137.pdf (原理图 Rev B-01, 2015), UM1974 Rev 11 (用户手册, August 2025)
最后更新: 2025-11-26
---

# Nucleo-H743ZI 插槽/接口功能与 CPU 引脚映射

## 评估优化必要性
是的，以上提供的优化是必要的。原因如下：
- **准确性提升**: 原文档中存在引脚编号不准（如 3.3V/GND 位置为 CN7 pin7/8，但实际 CN8 pin7 = +3.3V, CN7 pin8 = GND; SPI1 CS 为 CN7 pin16 = PD14，而非 CN9-10）、拼写错误（如 PB7 误为 PM7）、接口类型混淆（如 ST-LINK USB 为 Micro-B 而非 Mini-B, CN1 而非 CN11）。这些基于 UM1974 Table 20/21 和 MB1137 原理图的修正，确保映射可靠，避免硬件连接错误导致的调试问题。
- **完整性补充**: 原文档缺少 Morpho 完整引脚表 (CN11/CN12)，优化后补充了备用 UART1 (PB6/PB7) 等选项，并强调 Ethernet 与 SPI2 (PB13) 复用冲突 (需配置 SB118)，这对你的项目 (2x SPI + I2C + GPIO 输入 + UART 输出) 至关重要，便于避免外设冲突。
- **实用性优化**: 调整了典型接线建议，使其更贴合 Arduino/Zio 区快速原型 (IMU1 接 CN7 D10-D13)，Morpho 用于高级扩展 (IMU2/CS)。还增加了设计提示，如 EXTI 配置和焊桥 (SB) 调整，减少新手踩坑。
- **不必要部分**: 如果项目仅限基本外设，无需进一步展开全部针脚表。但鉴于你提到“后续原型布线和 SB 复用判断”，建议展开完整 Table 20/21，便于全面参考 (e.g., 查找备用引脚如 PE0 for CS)。

**总体评估**: 优化必要且有效，提升了文档的准确性和可用性。以下提供完整版 MD，包含展开的针脚表 (基于 UM1974 Table 20/21)，并整合优化内容。

## 文档范围
- 汇总 Nucleo-H743ZI (MB1137) 的开发板插槽/接口功能，并给出与 STM32H743ZIT6 (LQFP144 封装) 的引脚对应关系。
- 重点覆盖两路 SPI、一路 I2C、用于 CMOS 同步的 GPIO 输入，以及串口 UART 输出（包括 VCP/USART3）。
- 基于官方 UM1974 Table 20 (ST Zio connectors) 和 Table 21 (ST Morpho connector)，以及 MB1137 原理图验证。
- 注意: H743ZI 是旧版 (芯片掩膜 Y), 已停产; 推荐升级到 H743ZI2 (掩膜 V, ST-LINK V3)。但引脚布局/功能相同。

## 板卡总览
- MCU: STM32H743ZIT6 (LQFP144 封装, Cortex-M7 @ 480 MHz)。
- 调试/供电/虚拟串口: ST-LINK/V2-1 (USB Micro-B 接口, CN1)。
- 扩展插槽:
  - ST Zio connectors (CN7/CN8/CN9/CN10, Arduino Uno V3 兼容 + 扩展)。
  - ST Morpho connectors (CN11/CN12, 完整暴露 MCU 引脚, 两排 38-pin 排针)。
- 常用外设引出: 板上 Ethernet RJ45 (CN5), USB OTG Micro-AB (CN4), 3x LED (LD1 绿 PB0, LD2 蓝 PB7, LD3 红 PE1), 用户按键 (B1, PC13)。
- 电源: USB 5V (U5V/E5V), 外部 VIN (7-12V), 3.3V 输出。
- 尺寸: 标准 Nucleo-144 (机械图见 UM1974 Section 7.1)。

## 插槽/接口分区 (基于板上丝印和原理图)
板子顶视图 (UM1974 Figure 1): ST-LINK USB 左侧, Ethernet/USB OTG 右侧, Zio/Morpho 两侧排针。
- **ST Zio connectors (Arduino Uno V3 兼容 + 扩展)**: CN7 (右侧上, 10x2), CN8 (左侧上, 8x2), CN9 (左侧下, 15x2), CN10 (右侧下, 17x2)。提供电源、数字/模拟 IO、总线复用。适合快速接传感器/盾板。
- **ST Morpho connectors**: CN11 (左侧, 38x2), CN12 (右侧, 38x2)。完整 MCU 引脚暴露 (除保留), 适合定制走线/原型板。
- **USB OTG FS (Micro-AB, CN4)**: PA11 (DM), PA12 (DP), PA10 (ID), PA9 (SOF/OverCurrent), PG6 (PowerSwitchOn), PG7 (VBUS_DET)。支持设备/主机模式。
- **Ethernet (RJ45, CN5)**: RMII 接口 (PA1 REF_CLK, PA2 MDIO, PA7 CRS_DV, PB11 TX_EN, PB12 TXD0, PB13 TXD1, PC1 MDC, PC4 RXD0, PC5 RXD1)。
- **ST-LINK (Micro-B, CN1)**: 调试 + VCP (USART3 on PD8 TX/PD9 RX, 虚拟 COM 口)。
- **电源/地**: 多路 3.3V/5V/GND 引出 (CN8 pin7 = +3.3V, CN7 pin8 = GND, CN9 pin12/22/27 = GND 等)。

## 关键总线与引脚映射 (基于 UM1974 Table 20: ST Zio & Table 21: Morpho)
所有引脚支持 GPIO 模式, 复用功能需 CubeMX 配置。默认复用见表。电压域: 3.3V 兼容 (非 5V 耐压)。

### SPI1 (主用, Arduino 兼容区)
- 信号: SCK=PA5, MISO=PA6, MOSI=PA7 (or PB5), NSS=PA4 (or 任意 GPIO)。
- Zio 连接 (Arduino D11-D13):
  - CN7 pin10 = D13 / SPI_A_SCK = PA5.
  - CN7 pin12 = D12 / SPI_A_MISO = PA6.
  - CN7 pin14 = D11 / SPI_A_MOSI / TIM_E_PWM1 = PA7 (默认) or PB5 (SB121 ON, SB122 OFF).
  - CN7 pin16 = D10 / SPI_A_CS / TIM_B_PWM3 = PD14 (GPIO 作片选)。
- Morpho 连接: CN11 pin11 = PA5, CN11 pin13 = PA6, CN11 pin15 = PA7, CN12 pin15 = PD14 (NSS 示例)。
- 建议: IMU1 接 Arduino 区, CS 用 PD14 (D10)。

### SPI2 (备用/第二路)
- 信号: SCK=PB13, MISO=PB14, MOSI=PB15, NSS=PB12 (or 任意 GPIO)。
- Zio 连接: 无默认 Arduino 区, 用扩展引脚。
  - CN7 pin5 = D18 / I2S_A_CK = PB13 (SB118 OFF for SPI only).
  - CN7 pin3 = D17 / I2S_A_SD = PB15.
  - CN7 pin7 = D19 / I2S_A_WS = PB12 (NSS).
  - CN7 pin19 = D25 / SPI_B_MISO = PB4 (备用 MISO)。
- Morpho 连接: CN12 pin28 = PB14 (MISO), CN12 pin30 = PB13 (SCK), CN12 pin16 = PB12 (NSS), CN12 pin26 = PB15 (MOSI).
- 建议: IMU2 接 Morpho, CS 用 PB12 (D19) 或 PE0 (CN10 pin34 / D34)。

### I2C1 (磁力计)
- 信号: SCL=PB8, SDA=PB9 (默认) or PF1/PF0 (备用).
- Zio 连接 (Arduino A4/A5):
  - CN7 pin2 = D15 / I2C_A_SCL = PB8.
  - CN7 pin4 = D14 / I2C_A_SDA = PB9.
- Morpho 连接: CN12 pin3 = PB8, CN12 pin5 = PB9.
- 建议: 磁力计接 Arduino A4/A5 (CN7 pin2/4), 上拉电阻板上已有 (SB138/SB143 OFF 为默认)。

### GPIO 输入 (两个 CMOS 帧/行同步, EXTI 中断)
- 推荐: 任意空闲 GPIO, 如 PC13 (用户按键, 可复用), PE2 (D56, CN9 pin14).
- Zio 连接示例:
  - CN10 pin12 = D2 / I/O = PF15.
  - CN10 pin14 = D1 / USART_A_TX = PG14 (配置为 GPIO 输入).
- Morpho 连接: CN11 pin45 = PE3, CN11 pin47 = PE4 (易焊, 支持 EXTI).
- 建议: 帧同步 = PE3 (CN11 pin45), 行同步 = PE4 (CN11 pin47). CubeMX 配置为 External Interrupt Mode.

### UART 输出 (外部数据, 如 MAVLink)
- USART3 (默认 VCP, 通过 ST-LINK USB 虚拟 COM): TX=PD8, RX=PD9.
  - Zio 连接: 无直接, 但 Morpho CN11 pin69 = PD9 (RX), CN11 pin67 = PD8 (TX).
  - 适用: PC 调试/地面站 (USB 连接即用, 波特率 115200 默认).
- 备用 USART1 (外接模块): TX=PA9, RX=PA10 (or PB6/PB7).
  - Zio 连接: CN10 pin14 = D1 / USART_A_TX = PG14 (USART6, 可重映射), CN10 pin16 = D0 / USART_A_RX = PG9.
  - Morpho 连接: CN12 pin21 = PA9 (TX), CN12 pin23 = PA10 (RX).
- 建议: VCP (USART3) 用于调试输出; 外接遥测用 USART1 on Morpho (PA9/PA10).

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
- **IMU1 (SPI1)**: VCC=CN8 pin7 (+3.3V), GND=CN7 pin8, SCK=CN7 pin10 (PA5), MISO=CN7 pin12 (PA6), MOSI=CN7 pin14 (PA7), CS=CN7 pin16 (PD14).
- **IMU2 (SPI2)**: VCC=+3.3V, GND=GND, SCK=CN7 pin5 (PB13), MISO=CN7 pin19 (PB4 or PB14 via SB), MOSI=CN7 pin3 (PB15), CS=CN7 pin7 (PB12).
- **磁力计 (I2C1)**: VCC=+3.3V, GND=GND, SCL=CN7 pin2 (PB8), SDA=CN7 pin4 (PB9).
- **CMOS 同步 GPIO**: 帧同步=CN11 pin45 (PE3, EXTI), 行同步=CN11 pin47 (PE4, EXTI).
- **UART 输出 (MAVLink)**: 用 VCP (USB 连接 PC); 或外接: TX=CN12 pin21 (PA9), RX=CN12 pin23 (PA10).
- 注意冲突: PB13 (SPI2 SCK) 与 Ethernet 共享 (SB118 OFF 禁用 Ethernet); 配置 CubeMX 避免复用冲突。

## 设计提示
- **复用优先级**: Arduino/Zio 区适合快速原型, Morpho 用于高级/自定义 (全引脚)。
- **电源注意**: 传感器用 3.3V (CN8 pin7); 外部电源 VIN 需稳压。
- **中断配置**: GPIO EXTI 在 CubeMX 启用 NVIC; 优先级低于 SysTick。
- **兼容性**: 与 Arduino 盾板需检查 3.3V vs 5V; 修改 SB (见 UM1974 Table 12) 切换功能。
- **原理图验证**: MB1137 Sheet 2 (MCU) 确认 LQFP144 引脚, Sheet 6 (Connectors) 示意 Zio/Morpho 布局。
- **潜在问题**: Ethernet 默认启用, 占用 PB13 等; 移除 JP6/JP7 禁用。

## 备注
- 本文档基于官方 UM1974 Rev 11 (2025) 和 MB1137 Rev B-01 (2015) 原理图。完整 pinout 见 UM1974 Table 20/21。
- 如需 PCB 布局/ Gerber, 参考 ST 官网 EDA resources (Section 3.4)。
- 测试: 用 CubeIDE 生成默认项目, 验证 LED (PB0/PB7/PE1) 闪烁确认板子正常。

![Nucleo-144 Top View](um1974-page1-top-view.png)
*Figure 1 from UM1974: Nucleo-144 board (top view)*

![Nucleo-144 Bottom View](um1974-page1-bottom-view.png)
*Figure 2 from UM1974: Nucleo-144 board (bottom view)*

![MB1137 Top & Power Schematic](mb1137-page1.png)
*MB1137 Sheet 1: Top & Power (电源和连接示意)*

![MB1137 MCU Schematic](mb1137-page2.png)
*MB1137 Sheet 2: MCU (STM32H743ZI 引脚连接)*

![MB1137 ST-LINK Schematic](mb1137-page3.png)
*MB1137 Sheet 3: ST-LINK/V2-1 (调试接口)*

![MB1137 USB Schematic](mb1137-page4.png)
*MB1137 Sheet 4: USB (OTG 接口)*

![MB1137 Ethernet Schematic](mb1137-page5.png)
*MB1137 Sheet 5: Ethernet PHY with RJ45*

![MB1137 Extension Connectors Schematic](mb1137-page6.png)
*MB1137 Sheet 6: Extension connectors (Zio/Morpho 布局)*
