**总体目标**

* 保持“最小飞控”需求：SPI1+SPI3 双 IMU、I2C1 磁力计、USART3 MAVLink、两路 CMOS EXTI，无 SD/ADC/PWM。

* 全面对齐 boards/px4/fmu-v6x 的成熟做法，先修复当前编译/链接错误，再恢复必要模块（MAVLink）。

**信息收集要点**

* 已核查：

  * HRT 使用 `TIM5`，`board_config.h:47-49` 已定义。

  * H7 RCC 宏已在 `stm32h7/include/px4_arch/micro_hal.h` 引入 `hardware/stm32_rcc.h`，解决 `'STM32_RCC_APB1LENR' undeclared`。

  * 当前链接错误集中于：

    * `px4_i2c_buses` 未定义（平台层 `platforms/common/i2c.cpp:49/102/471` 引用）。

    * `up_restoreusartint` 未定义（`NuttX arch stm32_serial.c:4036`）。

  * 参考 `boards/px4/fmu-v6x/nuttx-config/nsh/defconfig`：重要特性包含 `CONFIG_PIPES=y`、丰富的串口配置与 NuttX 串口选项。

**修复步骤**

* 步骤1：恢复/统一 I2C 总线描述（修复 `px4_i2c_buses` 未定义）

  * 文件：`boards/st/nucleo-h743zi-fc/src/i2c.cpp`

  * 动作：将数组定义为“定义”（非 `extern`），参考 fmu‑v6x 其它板用法：

    * `#include <px4_arch/i2c_hw_description.h>`

    * `const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = { initI2CBusExternal(1), /*如需扩展可加 initI2CBusInternal(x)*/ };`

  * 说明：平台层通过该数组判断外部/内部 I2C，总线迭代与设备枚举依赖此符号。

* 步骤2：消除 SPI 传感器电源控制的重复定义

  * 文件：`boards/st/nucleo-h743zi-fc/src/init.cpp` 与 `platforms/nuttx/src/px4/stm/stm32_common/spi/spi.cpp`

  * 动作：板级不再重定义 `board_control_spi_sensors_power*`（已移除）；统一使用通用实现，避免链接多重定义。

* 步骤3：对齐 NuttX 串口和管道配置（修复 `up_restoreusartint` 与 MAVLink pipe）

  * 文件：`boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig`

  * 动作：参考 fmu‑v6x：

    * 增加 `CONFIG_PIPES=y`（MAVLink Shell 需要 `pipe()`）。

    * 校验/补充串口选项：`CONFIG_SERIAL_TERMIOS=y`、`CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y`、`CONFIG_USART3_SERIAL_CONSOLE=y` 等，使 `stm32_serial.c` 内部函数编译路径一致。

  * 说明：`up_restoreusartint` 未定义通常由串口宏条件不匹配引起，采用 fmu‑v6x 组合可规避。

* 步骤4：恢复 MAVLink 与磁力计公共框架

  * 文件：`boards/st/nucleo-h743zi-fc/default.px4board`

  * 动作：

    * 重新启用 `CONFIG_MODULES_MAVLINK=y`。

    * 保持/启用 `CONFIG_COMMON_MAGNETOMETER=y` 与 `CONFIG_DRIVERS_MAGNETOMETER_BOSCH_BMM150=y`（符合 I2C1 外置磁力计）。

  * 说明：与需求一致，输出 MAVLink，磁力计经 `i2c.cpp` 外部总线描述。

* 步骤5：核对时钟与引脚对齐 fmu‑v6x

  * 文件：`boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h`

  * 动作：

    * 确认 `SPI123` 内核时钟使用 `PLL2P=192MHz` 与 `RCC_D2CCIP1R_SPI123SEL_PLL2`（满足 ≤200MHz 限制）。

    * SPI1: `PA5/PA6/PD7`；SPI3: `PC10/PC11/PB2`；I2C1: `PB6/PB9`；USART3: `PD8/PD9` 与 fmu‑v6x 的格式一致。

* 步骤6：WSL 构建验证（按 CLAUDE.md）

  * 命令：`wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j$(nproc)'`

  * 期望：

    * 无 RCC 宏未定义、无 `px4_i2c_buses` 未定义、无 `up_restoreusartint` 未定义。

    * 链接完成并输出内存表（FLASH ≈ 12–21%，AXI\_SRAM ≈ 2%）。

**与 fmu‑v6x 的关键对齐点**

* 板级总线描述：提供 `px4_i2c_buses` 与 `px4_spi_buses`（后者已在板级 `spi.cpp`）。

* NuttX 配置：启用 `CONFIG_PIPES` 与完整串口宏组合，确保 shell 与 MAVLink 正常。

* HRT：板级定义定时器，而非启用 NuttX TIM 驱动，避免冲突（与 v6x 一致的理念）。

**交付物**

* 修改的文件与具体变更摘要（i2c.cpp 定义、defconfig 新增、default.px4board 恢复）。

* 一次完整构建日志摘要与最终内存占用。

* 若仍有串口警告（`%x` 格式），提供非阻断修正建议。

