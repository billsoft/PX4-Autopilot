# Nucleo‑H743ZI Arduino 插槽硬件集成指南（双 IMU + 磁力计 + MAVLink）

## 概览
- 目标：在 Nucleo‑H743ZI（MB1137/MB1364）开发板的 Arduino 插槽上，集成两路 IMU（SPI1/SPI3，ICM42688P/42686/45686）与一路磁力计（I2C1，BMM150），并通过 USART3 输出 MAVLink。
- 与当前项目完全对齐：
  - SPI1：PA5/PA6/PD7，片选 CS=PD14
  - SPI3：PC10/PC11/PB2，片选 CS=PA15
  - I2C1：PB6（SCL）、PB9（SDA）
  - USART3（VCP/COM5）：PD8（TX）、PD9（RX）
  - LED：PB0（绿）、PB7（黄/蓝）、PB14（红）
- 逻辑电平：全部 3.3V TTL；禁止 5V 直连到 MCU 信号脚。

### 关于 COM5 与独立 UART 的建议
- COM5 是板载 ST‑LINK 的虚拟串口（VCP），映射到 `USART3`（PD8/PD9）。它可直接输出 MAVLink，满足调试与基本联机需求（参考 MAVLink C UART 接口示例的通用串口用法）。
- 量产与工程化建议：在盾板上引出一组独立 3.3V TTL UART 头座（优先 `USART2` 或 `USART1`），避免依赖 ST‑LINK。推荐：
  - `USART2`：TX=PD5、RX=PA3（可用，位于 Morpho；Arduino UNO D0/D1 对应的是 `USART3` 的 PD9/PD8，不是 `USART2`）
  - `USART1`：TX=PA9、RX=PA10（可用，常见在 Morpho；与部分外设冲突需检查）
  - 若使用 Arduino UNO D0/D1 头座进行外部串口连接，请知悉其对应 `USART3`（PD9/PD8），与 COM5 共用物理资源，不建议在量产场景中与 VCP 同时使用。

## 板卡与插槽位置（Zio/Morpho）
- Zio（Arduino 兼容）：便于快速接入 SPI3 与 I2C1、供电与地；
  - D13=PA5（SPI1 SCK）、D12=PA6（SPI1 MISO）、D11=PA7（保留以太网，SPI1 MOSI 用 PD7）
  - D45=PC10（SPI3 SCK）、D46=PC11（SPI3 MISO）、PB2（SPI3 MOSI，位于 Morpho）
  - D14=PB9（I2C1 SDA）、D26=PB6（I2C1 SCL）
  - 3.3V（CN8 pin7）、GND（如 CN7 pin8）
- Morpho（全引脚）：用于 SPI1 MOSI（PD7）与片选（PD14、PA15），以及 USART3 PD8/PD9 引出到外设。

### 插槽针脚参考（面向硬件布局）
- Zio 关键位：
  - D13=PA5（SPI1 SCK）、D12=PA6（SPI1 MISO）、D45=PC10（SPI3 SCK）、D46=PC11（SPI3 MISO）、D20=PA15（SPI3 CS）、D14=PB9（I2C1 SDA）、D26=PB6（I2C1 SCL）
- Morpho 关键位：
  - CN11：PD7（SPI1 MOSI）、PD14（SPI1 CS）
  - CN12：PB2（SPI3 MOSI）、PD8（USART3 TX）、PD9（USART3 RX）、可选 PA9/PA10（USART1 TX/RX）、PD5（USART2 TX）、PA3（USART2 RX）
  - 3.3V 与 GND 就近引出，优先在 Zio 提供（CN8 pin7 为 3.3V）

### 插槽针脚对照表（精选）
```
信号功能           | MCU管脚  | Arduino标号 | 连接器      | 备注
--------------------|----------|-------------|-------------|------------------------------
SPI1 SCK            | PA5      | D13         | Zio (CN10)  | 片选见 PD14
SPI1 MISO           | PA6      | D12         | Zio (CN10)  |
SPI1 MOSI           | PD7      | —           | Morpho (CN11)| 无原生 D 标号（走 Morpho）
SPI1 CS (IMU1)      | PD14     | —           | Morpho (CN11)| 片选建议独立线（串阻可选）

SPI3 SCK            | PC10     | D45         | Zio (CN8)   |
SPI3 MISO           | PC11     | D46         | Zio (CN8)   |
SPI3 MOSI           | PB2      | —           | Morpho (CN12)| 无原生 D 标号（走 Morpho）
SPI3 CS (IMU2)      | PA15     | D20         | Zio (CN7)   |

I2C1 SDA            | PB9      | D14         | Zio (CN7)   | 上拉 2.2–4.7kΩ 至 3.3V
I2C1 SCL            | PB6      | —           | Morpho (CN12)| 本项目选用 PB6 而非 PB8；走 Morpho

USART3 TX (MAVLink) | PD8      | D1          | Zio (CN10)  | COM5（ST‑LINK VCP）映射
USART3 RX (MAVLink) | PD9      | D0          | Zio (CN10)  | COM5（ST‑LINK VCP）映射

USART2 TX (可选)    | PD5      | —           | Morpho (CN11)| 独立 3.3V TTL 建议引出
USART2 RX (可选)    | PA3      | —           | Morpho (CN12)| 独立 3.3V TTL 建议引出

USART1 TX (可选)    | PA9      | —           | Morpho (CN12)| 需检查复用冲突
USART1 RX (可选)    | PA10     | —           | Morpho (CN12)| 需检查复用冲突

3.3V                | —        | 3V3         | Zio (CN8)   | CN8 pin7（板载 3.3V）
GND                 | —        | GND         | Zio (CN7/8) | 多处地脚（优先近端）
```
注：完整针脚映射与孔位编号请参考 ST 官方文档 UM1974（Nucleo‑144 用户手册）。以上列表仅列出本项目所涉关键信号与推荐引出位置。

## 接口映射与连线建议
- SPI1（IMU1）
  - SCK=PA5（Zio D13）、MISO=PA6（Zio D12）、MOSI=PD7（Morpho CN11）
  - CS（NSS）=PD14（Morpho CN11）
  - 典型速率：10–20 MHz；线长尽量短并加地参考；必要时 SCK/MOSI 串阻 22–33 Ω 减缓边沿。
- SPI3（IMU2）
  - SCK=PC10（Zio D45）、MISO=PC11（Zio D46）、MOSI=PB2（Morpho CN12）
  - CS（NSS）=PA15（Zio D20）
  - 与 SPI1 同速率建议；避免与 SDMMC 冲突的复用；
- I2C1（磁力计）
  - SCL=PB6（Zio D26/Morpho CN12）、SDA=PB9（Zio D14）
  - 上拉：2.2–4.7 kΩ 至 3.3V（位于板上或传感器模块上，二者择一，避免双重上拉过低）
  - BMM150 默认地址：0x10；注意 `i2cdetect -b 1` 检测结果应显示 0x10。
- USART3（MAVLink）
  - TX=PD8、RX=PD9；调试用 COM5（ST‑LINK VCP）即可满足验证需求；
  - 量产建议：将 PD8/PD9 通过 3.3V TTL 接插件引出至外设（相机/主机），或选用 USART1（PA9/PA10）作为独立通信口以避免与 ST‑LINK 共享。

### `USART2`（PA3/PD5）能否用于 Arduino 插槽
- `USART2` 的 PA3（RX）、PD5（TX）在 Nucleo‑H743ZI 上提供于 Morpho 连接器，不在 Arduino UNO 的 D0/D1 位置；因此，若要在盾板上使用 `USART2` 输出，请走 Morpho 对应针脚，不要假设可用 UNO 头座直接接入。

## 电源与时钟
- 供电：
  - 3.3V（CN8 pin7）与 GND（如 CN7 pin8）；各传感器模块就近布置 100 nF + 1 µF 去耦；IMU 建议再加 10 µF 钽或陶瓷。
  - 接地：优先单点接地与接地平面；走线下方保持连续地，避免跨分割。
- 时钟与外设时钟：
  - HSE 8 MHz（ST‑LINK MCO）→ SYSCLK=PLL1P=480 MHz；
  - SPI123 时钟源为 PLL2P，需 ≤ 200 MHz（推荐配置 192 MHz）；
  - USART3 来自 APB 时钟（约 96 MHz），足以 115200 波特率；
  - USB FS 使用 48 MHz（PLL1Q/PLL3Q），与本应用无冲突。

### USB/串口芯片说明
- 板载 ST‑LINK 单元集成 USB‑UART 桥（VCP），无需另行布置 USB 串口芯片即可获得 COM5；
- 若产品需要与 PC 直连且不依赖 ST‑LINK，建议在盾板上布置独立 USB‑UART 芯片（如 CP2102/CH34x）并接至 `USART2/USART1` 的 3.3V TTL；注意芯片需 3.3V 逻辑级别或带电平转换。

## 信号完整性与布局建议
- IMU 模块靠近插槽，减短 SPI 线长；优先差线走地参考（SCK/MOSI/MISO 旁有 GND 伴随回流路径）。
- 片选（NSS）线独立走线，拉低时边沿平滑；必要时串阻 22–33 Ω。
- I2C 上拉放置于磁力计附近；布线避免与强噪声路径（如 USB、ETH）并行。
- ESD 保护：在插槽引脚与模块接口处增设 TVS；连接器处适配 EMI 屏蔽。
- 统一 3.3V TTL 逻辑；严禁 5V 直连到 MCU。

## 设备与软件要点
- IMU 驱动：`icm42688p` 支持 `-6` 选项（兼容 42686/45686），当前脚本使用：
  - `icm42688p start -s -b 1 -R 0 -6`
  - `icm42688p start -s -b 3 -R 8 -6`
- 磁力计驱动：`bmm150 start -I -b 1 -R 0`
- LED 显示映射：
  - 绿=SPI1 IMU、黄=SPI3 IMU、红=I2C1 磁力计；
  - 无数据：慢闪；有数据：2 Hz 快闪；融合：绿+黄 3.3 Hz；
- MAVLink（USART3）：
  - `mavlink stream -u -r 120 -s ATTITUDE_QUATERNION`
  - `mavlink stream -u -r 120 -s HIGHRES_IMU`
  - `mavlink stream -u -r 50 -s ATTITUDE`

## Mermaid 拓扑图
```mermaid
flowchart LR
  Nucleo[Nucleo‑H743ZI\n3.3V TTL]:::board
  STLINK[ST-LINK\nUSB VCP COM5]:::board
  subgraph SPI1
    PA5[PA5 SCK]\nD13
    PA6[PA6 MISO]\nD12
    PD7[PD7 MOSI]\nMorpho
    PD14[PD14 CS]\nMorpho
  end
  subgraph SPI3
    PC10[PC10 SCK]\nD45
    PC11[PC11 MISO]\nD46
    PB2[PB2 MOSI]\nMorpho
    PA15[PA15 CS]\nD20
  end
  subgraph I2C1
    PB6[PB6 SCL]\nD26
    PB9[PB9 SDA]\nD14
  end
  subgraph UART3
    PD8[PD8 TX]\nVCP COM5
    PD9[PD9 RX]\nVCP COM5
  end
  subgraph UARTx(Optional)
    PD5[PD5 TX\nUSART2]\nMorpho
    PA3[PA3 RX\nUSART2]\nMorpho
    PA9[PA9 TX\nUSART1]\nMorpho
    PA10[PA10 RX\nUSART1]\nMorpho
  end
  IMU1[IMU1\nICM42688P]
  IMU2[IMU2\nICM42688P]
  MAG[BMM150\n0x10]
  CAM[External Device\nMAVLink]
  LDO[3.3V LDO\nDecoupling]
  ESD[TVS/ESD\nConnectors]

  PA5 --> IMU1
  PA6 --> IMU1
  PD7 --> IMU1
  PD14 --> IMU1

  PC10 --> IMU2
  PC11 --> IMU2
  PB2 --> IMU2
  PA15 --> IMU2

  PB6 --> MAG
  PB9 --> MAG

  PD8 --> CAM
  CAM --> PD9
  Nucleo --> LDO
  Nucleo --> ESD
  Nucleo --> STLINK

  classDef board fill:#eef,stroke:#55f,stroke-width:1px;
```

## 验证流程（NSH）
- 连接：115200‑8N1，`nsh>` 正常；复位有启动日志。
- 设备：
  - `i2cdetect -b 1` 应显示 `0x10`
  - `icm42688p status`/`bmm150 status`：Running
- 数据：
  - `listener sensor_accel 0` 静止水平 z≈‑9.8 m/s²
  - `listener vehicle_attitude` 约 120 Hz 更新
- LED：
  - `board_status_leds status` 可见快闪/融合 3.3 Hz
- MAVLink：
  - `mavlink status streams` 显示 `ATTITUDE_QUATERNION: 120 Hz` 等流

## 兼容与可选项
- 若不使用 COM5 调试：引出 PD8/PD9 至 3.3V TTL 插座；或改用 USART1（PA9/PA10）。
- 若使用不同 IMU/Mag 型号：保持 3.3V/接口时序相同，适配驱动与地址。

### 机械与安装（双面焊接与轴向对齐）
- IMU 模块建议一正一反分别安装于盾板的上下两面（Top/Bottom），使其轴向相互镜像，实现姿态/加速度误差对消；
- 磁力计同理可按正反面安装以评估磁偏误差，但实际融合中保留一枚作为主观量，另一枚作为冗余或校验；
- 明确在 silk 与文档中标注三个轴的方向箭头（X/Y/Z），并在软件中使用对应的旋转矩阵与符号约定进行对齐与求和；
- 连接器与固定孔位置需与 Zio/Morpho 参考图一致，避免后期返工。

## 参考与对齐文件
- `STM32_743zi_pin.md`（针脚与插槽对齐）
- `stm32_custom_board_bringup_tutorial_v2.md`（项目整体流程）
- `boards/st/nucleo-h743zi-fc/需求.md`（功能映射与 LED 逻辑）
