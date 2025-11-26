---
文档版本: 1.0
适用板卡: Nucleo‑H743ZI (MB1137，非 ZI2)
参考资料: MB1137.pdf，UM1974 (stm32‑nucleo144‑boards‑mb1137)
最后更新: 2025‑11‑26
---

# Nucleo‑H743ZI 插槽/接口功能与 CPU 引脚映射

## 文档范围
- 汇总 Nucleo‑H743ZI（MB1137）的开发板插槽/接口功能，并给出与 STM32H743ZI(LQFP144) 的引脚对应关系。
- 重点覆盖两路 SPI、一路 I2C，以及用于 MAVLink 的 UART（VCP/USART3）。

## 板卡总览
- MCU: `STM32H743ZIT6`，LQFP144 封装。
- 调试/供电: `ST‑LINK/V2‑1`（USB Micro‑B，CN1）。
- 扩展插槽: `Arduino UNO R3 / ST Zio (CN7/8/9/10)`、`ST Morpho (CN11/CN12，两侧各 38×2 排针)`。
- 常用外设引出: SPI1/2、I2C1/2、USART1/2/3、USB OTG FS（CN4）、Ethernet RJ45（CN5）、若干 GPIO/ADC。

## 插槽/接口分区
- ST‑LINK/VCP（虚拟串口）
  - 功能: 调试、虚拟串口访问 NuttX/PX4 `nsh>`。
  - 串口: `USART3` → `PD8 (TX)`、`PD9 (RX)`。
- Arduino / Zio 插槽（CN7/8/9/10，含电源/数字/模拟）
  - 提供 `SPI1`、`I2C1`、若干 GPIO/ADC，适合快速原型外接传感器。
- ST Zio 插槽（两排拓展）
  - 提供更多 GPIO/总线的复用引脚，常用于片选/额外外设连接。
- ST Morpho 插槽（CN11/CN12，完整暴露 MCU 引脚）
  - 提供除少数保留功能外的全部 MCU 引脚，适合定制走线与双路总线并行。

## 关键总线与引脚映射

### SPI1（主用，Arduino 兼容区）
- 信号: `SCK=PA5`、`MISO=PA6`、`MOSI=PA7`（可选 `PB5` 依焊桥）、`NSS(片选)=PA4 或任意可用 GPIO`
- 典型连接（Arduino/Zio 插槽）:
  - `CN7 pin10 = D13 / PA5 (SPI1_SCK)`
  - `CN7 pin12 = D12 / PA6 (SPI1_MISO)`
  - `CN7 pin14 = D11 / PA7 (SPI1_MOSI)`
  - 片选示例: `CN7 pin16 = D10 / PD14 (GPIO)` 作为 IMU1 的 CS

### SPI2（备用/第二路）
- 信号: `SCK=PB13`、`MISO=PB14`、`MOSI=PB15`、`NSS(片选)=PB12 或任意可用 GPIO`
- 典型连接（Morpho/Zio 插槽）:
  - Zio/Arduino 区：`CN7 pin5 → PB13 (SCK)`、`CN7 pin3 → PB15 (MOSI)`、`CN7 pin7 → PB12 (NSS)`、`CN7 pin19 → PB4 (备用 MISO)`
  - Morpho 区：`PB13 / PB14 / PB15` 引出在 CN12；`PB12` 可作 CS
  - 片选示例: 使用 `PD15 (GPIO)` 作为 IMU2 的 CS

### I2C1（磁力计）
- 信号: `SCL=PB8`、`SDA=PB9`
- 典型连接（Arduino 插槽）:
  - `CN7 pin2 = D15 / PB8 (SCL)`、`CN7 pin4 = D14 / PB9 (SDA)`
  - Morpho 引脚：`CN12 pin3 = PB8`、`CN12 pin5 = PB9`

### UART（MAVLink）
- `USART3`（默认连接 ST‑LINK VCP）: `PD8 (TX)`、`PD9 (RX)`
  - 适用: 调试串口 / MAVLink 与地面站（通过 USB 虚拟串口）
- 备用: `USART1` 可用于外接 GPS/遥测: `PA9 (TX)`、`PA10 (RX)` 或 `PB6 (TX)`、`PB7 (RX)`（Morpho 引出）

### USB OTG FS（设备模式常用）
- 信号: `PA11 (DM)`、`PA12 (DP)`（板上 Micro‑AB 接口，CN4）

### 供电/地（Arduino/Zio 常用位）
- `CN8 pin7 → +3.3V`、`CN7 pin8 → GND`、`CN8 pin9 → +5V`、`CN8 pin11/13 → GND`

## 典型外设接线建议（满足 2×SPI + 1×I2C + UART）

- IMU1（SPI1）
  - `VCC → +3.3V (CN8 pin7)`、`GND → GND (CN7 pin8)`
  - `SCK → CN7 pin10 (PA5)`、`MISO → CN7 pin12 (PA6)`、`MOSI → CN7 pin14 (PA7)`
  - `CS → CN7 pin16 (PD14)`

- IMU2（SPI2）
  - `SCK → CN7 pin5 (PB13)`、`MISO → CN12 pin28 (PB14)` 或 `CN7 pin19 (PB4)`、`MOSI → CN7 pin3 (PB15)`
  - `CS → CN7 pin7 (PB12)` 或使用 `PD15`（Zio/Morpho 任选合适 GPIO）

- 磁力计（I2C1）
  - `SCL → D15 / PB8`、`SDA → D14 / PB9`
  - `VCC → 3.3V`、`GND → GND`

- MAVLink（串口）
  - 调试/地面站: `USART3 → PD8/PD9`（通过 ST‑LINK VCP，USB 线连接 PC）
  - 外接遥测模块（可选）: `USART1 → PA9/PA10` 或 `PB6/PB7`（Morpho 引脚接射频/数传）

## 设计提示
- 选择片选（CS）时优先从 `PD14/PD15/PE0/PE1` 等通用 GPIO 中挑选，便于双路 SPI 并行且不与系统保留功能冲突。
- I2C 设备建议统一接 `I2C1(PB8/PB9)`，便于与 Arduino 兼容区的现成线序匹配。
- 如需更高串口速率的 MAVLink，可改用独立 UART（如 `USART1`），避免与 VCP 共享带来的限制。
 - `PB13`（SPI2_SCK）与 Ethernet 复用，必要时通过板载焊桥/跳线配置禁用以释放 SPI2（参考 UM1974 的 SB 配置）。

## 备注
- 本映射以 MB1137 与 UM1974 为依据，具体插槽丝印与引脚序号以板上印刷/资料为准。
- 重点场景与引脚已给出可直接落地的连接方案，适合两路 IMU + 一路磁力计 + 串口输出融合数据的飞控原型设计。

## CN 快速索引（精选，满足 2×SPI + 1×I2C + UART）

- CN7（右侧上，10×2）
  - `pin2 → D15 / PB8 (I2C1_SCL)`
  - `pin4 → D14 / PB9 (I2C1_SDA)`
  - `pin5 → D18 / PB13 (SPI2_SCK)`
  - `pin3 → D17 / PB15 (SPI2_MOSI)`
  - `pin19 → D25 / PB4 (SPI2 备用 MISO)`
  - `pin7 → D19 / PB12 (SPI2_NSS / 片选)`
  - `pin10 → D13 / PA5 (SPI1_SCK)`
  - `pin12 → D12 / PA6 (SPI1_MISO)`
  - `pin14 → D11 / PA7 (SPI1_MOSI)`
  - `pin16 → D10 / PD14 (GPIO 片选)`
  - `pin8 → GND`

- CN8（左侧上，8×2，电源/参考）
  - `pin7 → +3.3V`、`pin9 → +5V`、`pin11/13 → GND`、`pin3 → IOREF`、`pin5 → RESET`、`pin15 → VIN`

- CN10（Zio 右下，17×2）
  - `pin12 → D2 / PF15 (GPIO)`
  - `pin14 → D1 / PG14 (可作 GPIO/串口)`
  - `pin16 → D0 / PG9 (可作 GPIO/串口)`
  - `pin34 → D34 / PE0 (GPIO 片选备选)`

- CN11（Morpho 左，38×2）
  - `pin45 → PE3 (GPIO/EXTI)`
  - `pin47 → PE4 (GPIO/EXTI)`
  - `pin67 → PD8 (USART3_TX → VCP)`
  - `pin69 → PD9 (USART3_RX → VCP)`

- CN12（Morpho 右，38×2）
  - `pin3 → PB8 (I2C1_SCL)`
  - `pin5 → PB9 (I2C1_SDA)`
  - `pin16 → PB12 (SPI2_NSS)`
  - `pin26 → PB15 (SPI2_MOSI)`
  - `pin28 → PB14 (SPI2_MISO)`
  - `pin30 → PB13 (SPI2_SCK)`
  - `pin21 → PA9 (USART1_TX)`
  - `pin23 → PA10 (USART1_RX)`
