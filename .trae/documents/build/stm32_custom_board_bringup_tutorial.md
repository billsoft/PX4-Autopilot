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

---

## 第 2 步：NuttX `defconfig`（启用自定义板目录）
文件：`boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig`

关键配置（片段）：
```kconfig
# 使用自定义板目录
CONFIG_ARCH_BOARD_CUSTOM=y
CONFIG_ARCH_BOARD_CUSTOM_DIR="../../../../boards/st/nucleo-h743zi-fc/nuttx-config"
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
CONFIG_ARCH_BOARD_CUSTOM_NAME="nucleo-h743zi-fc"

# 平台/芯片
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32h7"
CONFIG_ARCH_CHIP_STM32H743ZI=y

# 计时框架支持（HRT 依赖）
CONFIG_TIMER=y
CONFIG_ONESHOT=y
# 注意：不启用 CONFIG_STM32H7_TIM5=y（由 PX4 HRT 直接控制 TIM5）

# 外设使能（示例）
CONFIG_STM32H7_SPI1=y
CONFIG_STM32H7_SPI3=y
CONFIG_SPI=y
CONFIG_SPI_EXCHANGE=y
CONFIG_SPI_DRIVER=y

CONFIG_STM32H7_I2C1=y
CONFIG_I2C=y
CONFIG_I2C_DRIVER=y
CONFIG_I2C_TRANSFER=y

CONFIG_STM32H7_USART3=y
CONFIG_USART3_SERIAL_CONSOLE=y
CONFIG_USART3_BAUD=115200
```

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
#define STM32_BOARD_XTAL        8000000ul
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
/* PLL1 VCO = (8MHz/2)*240 = 960MHz; PLL1P/R/Q = 480/120/240MHz */
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(2)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(240)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)

/* PLL2 用于 SPI123，192MHz */
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(96)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)
#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2
```

引脚片段示例：
```c
/* SPI1: PA5/PA6/PD7 */
#define GPIO_SPI1_SCK   (GPIO_SPI1_SCK_1  | GPIO_SPEED_50MHz)
#define GPIO_SPI1_MISO  (GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz)
#define GPIO_SPI1_MOSI  (GPIO_SPI1_MOSI_3 | GPIO_SPEED_50MHz) /* PD7 */

/* SPI3: PC10/PC11/PB2; NSS 对齐到 PA15 */
#define GPIO_SPI3_SCK   (GPIO_SPI3_SCK_2  | GPIO_SPEED_50MHz)
#define GPIO_SPI3_MISO  (GPIO_SPI3_MISO_2 | GPIO_SPEED_50MHz)
#define GPIO_SPI3_MOSI  (GPIO_SPI3_MOSI_3 | GPIO_SPEED_50MHz)
#define GPIO_SPI3_NSS   (GPIO_SPI3_NSS_1  | GPIO_SPEED_50MHz) /* PA15 */

/* I2C1: PB6/PB9 */
#define GPIO_I2C1_SCL   (GPIO_I2C1_SCL_1  | GPIO_SPEED_50MHz)
#define GPIO_I2C1_SDA   (GPIO_I2C1_SDA_2  | GPIO_SPEED_50MHz)

/* USART3: PD8/PD9 */
#define GPIO_USART3_TX  (GPIO_USART3_TX_3 | GPIO_SPEED_100MHz)
#define GPIO_USART3_RX  (GPIO_USART3_RX_3 | GPIO_SPEED_100MHz)
```

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
const px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
    initSPIBus(SPI::Bus::SPI1, {
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortD, GPIO::Pin14}),
    }),
    initSPIBus(SPI::Bus::SPI3, {
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortA, GPIO::Pin15}),
    }),
};
static constexpr bool unused = validateSPIConfig(px4_spi_buses);
```

文件：`boards/st/nucleo-h743zi-fc/src/i2c.cpp`
- 定义 `px4_i2c_buses` 为强符号，包含 I2C1 外部设备；与类型/维度严格匹配。

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

---

## 第 6 步：启动脚本（`init/rc.board_sensors`）
示例：
```sh
# IMU1 on SPI1
icm42688p start -s -b 1 -R 0 -6

# IMU2 on SPI3（第二路旋转示例）
icm42688p start -s -b 3 -R 8 -6

# Magnetometer on I2C1
bmm150 start -I -b 1

# CMOS sync GPIO
cmos_sync start

# Dual IMU fusion（120 Hz）
dual_imu_fusion start

# MAVLink 流（相机防抖需要四元数与原始 IMU）
mavlink stream -u -r 120 -s ATTITUDE_QUATERNION
mavlink stream -u -r 120 -s HIGHRES_IMU
mavlink stream -u -r 50  -s ATTITUDE
```

---

## 第 7 步：WSL 构建与验证
遵循 `CLAUDE.md`：不自动构建，用户手动执行并回传日志。

建议命令（Windows/WSL）：
```bash
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make distclean && make submodulesupdate"
wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j4 2>&1 | tee build_wsl.log"
```

验证产物：
- `build/st_nucleo-h743zi-fc_default/st_nucleo-h743zi-fc_default.{elf,bin,px4}`
- 大小报告：`arm-none-eabi-size .../st_nucleo-h743zi-fc_default.elf`

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

### 时钟树与 SPI 内核频率的物理原理
- STM32H7 的 SPI 外设由“内核时钟（fKERNEL）”驱动，并通过分频器生成输出时钟（fSCK）。
- 数据手册给出上限：`fKERNEL ≤ 200 MHz`，`fSCK ≤ 150 MHz`；NuttX 驱动在编译时对 `fKERNEL` 进行 `#error` 检查。
- 系统主频（SYSCLK）与不同外设的时钟来源应分离：SYSCLK 用 `PLL1P`，SPI123 用 `PLL2P`；避免不同外设共享引擎引起时序抖动。

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

### Step 2：`defconfig`（意图：让 NuttX 指向我们的自定义板目录）
- 通过 `CONFIG_ARCH_BOARD_CUSTOM` 告知 NuttX 使用 `boards/st/nucleo-h743zi-fc/nuttx-config` 作为板配置根。
- 启用外设（SPI1/3、I2C1、USART3）、启用计时框架（`CONFIG_TIMER=y/CONFIG_ONESHOT=y`），并明确不启用 TIM5 驱动以避免与 HRT 冲突。

### Step 3：`board.h`（意图：时钟树与引脚的权威定义）
- 自包含所有 PLL、分频器、时钟源选择；不依赖外部板文件。
- 原理：SYSCLK 用 `PLL1P=480MHz`；SPI123 用 `PLL2P=192MHz`；分别满足性能与 SPI 驱动约束。
- 引脚宏以板文档为准：SPI1（PA5/PA6/PD7）、SPI3（PC10/PC11/PB2）、I2C1（PB6/PB9）、USART3（PD8/PD9）、NSS=PA15。

### Step 4：PX4 板级代码（意图：把硬件抽象为可启动的设备）
- `board_config.h`：LED、CS、CMOS EXTI、HRT 宏定义；`PX4_GPIO_INIT_LIST` 在 `stm32_boardinitialize` 统一初始化。
- `spi.cpp`：以 `px4_spi_buses` 静态描述 SPI 总线与设备（CS 引脚、DRDY 可选），用 `validateSPIConfig` 做编译期一致性校验。
- `i2c.cpp`：以 `px4_i2c_buses` 描述 I2C 总线；务必保证类型与维度匹配，避免链接时 `undefined reference`。
- `init.c/led.c`：板级初始化与调试指示；`board_app_initialize` 调用 `px4_platform_init()` 完成平台层准备。

### Step 5：`.px4board`（意图：最小可用系统 + 必要模块）
- 启用 `mavlink`、`sensors`、`dual_imu_fusion`；驱动启用 `icm42688p`、`bmm150`。
- 关闭 SD 卡、ADC、GPS、PWM 等不需要功能，避免资源浪费与驱动冲突。

### Step 6：`rc.board_sensors`（意图：把设备启动逻辑脚本化）
- 两路 IMU 启动并设置旋转（第二路 `-R 8` 示例），磁力计、CMOS 同步、融合与 MAVLink 流；四元数与 IMU 原始数据 120 Hz 满足防抖与外部相机需求。

### Step 7：WSL 构建（意图：在 Windows 也能稳定构建）
- 按 `CLAUDE.md` 手动运行构建；回传日志以便核验 HRT、SPI 时钟限制、符号链接等是否正常。

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

### 3）SPI123 频率报错
- 根因：驱动检查 `SPI123_KERNEL_CLOCK_FREQ > 200MHz`；使用 `PLL1Q` 可能超限。
- 修复：SPI123 时钟源改为 `PLL2P=192MHz`；参考 `fmu-v6x` 的策略。

### 4）子模块改动丢失
- 根因：对 NuttX 子模块做了本地修改；`make distclean` 清理后丢失。
- 修复：所有改动放在 `boards/...` 自定义板目录；子模块仅作为依赖使用。

### 5）双 IMU 旋转处理重复
- 根因：在融合中手动翻转轴，同时驱动已用 `-R` 旋转；导致双重旋转。
- 修复：融合层不做额外轴翻转；完全依赖驱动 `-R` 参数对齐坐标系。

---

## 性能、带宽与资源规划
- SYSCLK 480MHz：保证融合与 MAVLink 流在高负载下稳定运行。
- SPI 内核 192MHz：给 IMU 足够带宽（分频后满足设备 SCK 要求），同时满足驱动上限。
- MAVLink 流：四元数与 IMU 120 Hz；若带宽不足，可下调或切换串口（USART1 独立外接）。
- 内存与闪存：通过禁用不必要模块降低镜像大小；在构建日志中查看 `arm-none-eabi-size` 报告。

---

## 维护与升级策略
- 固件升级：保持自定义板与上游 PX4 同步，避免子模块差异；必要时基于 `fmu-v6x` 学习新改动。
- 文档更新：所有引脚与时钟改动在本教程与 `nucleo_h743zi_pinmap.md` 同步更新；启动脚本与板配置保持一致。
- 代码风格：遵循 PX4 CMake 与代码规范（`px_base.cmake`、`check_format`）。

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

