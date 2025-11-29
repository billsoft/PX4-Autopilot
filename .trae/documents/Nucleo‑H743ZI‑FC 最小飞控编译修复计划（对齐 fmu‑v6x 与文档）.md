**目标范围**
- 保留功能：SPI1+SPI3 双 ICM42688P、I2C1 BMM150、两路 CMOS EXTI、USART3 MAVLink；无 TF/SD、无 ADC/电机输出。
- 参考：`.trae/documents/build/*` 与 `boards/px4/fmu-v6x`；不修改 `platforms/nuttx/NuttX` 子模块。

**现状与错误归因**
- 早期错误：`STM32_RCC_APB1LENR` 未定义 → 已通过在 `stm32h7/include/px4_arch/micro_hal.h` 引入 `hardware/stm32_rcc.h` 解决。
- 现存链接错误：
  - `up_restoreusartint` 未定义（NuttX `stm32_serial.c`）。原因：串口代码路径仅在 `!SERIAL_HAVE_ONLY_DMA || CONFIG_PM || HAVE_RS485` 时生成；当前组合可能是“仅DMA且无PM”。
  - `px4_i2c_buses` 未定义（平台层 `i2c.cpp`/`i2c_spi_buses.cpp` 引用）。原因：板级符号未提供或被条件编译排除。

**对齐与修复步骤**
- 步骤1：启用 NuttX PM 修复串口链接
  - 文件：`boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig`
  - 变更：添加 `CONFIG_PM=y`，保持 `CONFIG_SERIAL_TERMIOS=y`、`CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y`、`CONFIG_PIPES=y`、`USART3 RXDMA/TXDMA=y`。
  - 说明：满足 `stm32_serial.c` 的条件编译，生成 `up_restoreusartint`；与 fmu‑v6x 串口风格一致。

- 步骤2：确保板级提供 I2C 总线符号
  - 文件：`boards/st/nucleo-h743zi-fc/src/i2c.cpp`
  - 检查：`const px4_i2c_bus_t px4_i2c_buses[...] = { initI2CBusExternal(1) };` 已存在（行1-8）。如需，移除 `#if defined(CONFIG_I2C)` 包裹，保证符号无条件存在。

- 步骤3：清理 SPI 传感器电源控制重复实现
  - 文件：`boards/st/nucleo-h743zi-fc/src/init.cpp`
  - 状态：重复的 `board_control_spi_sensors_power*` 已移除，统一使用通用实现，避免多重定义；保持此对齐。

- 步骤4：最小化 PX4 模块配置（保持需求）
  - 文件：`boards/st/nucleo-h743zi-fc/default.px4board`
  - 保留：`CONFIG_MODULES_MAVLINK=y`、`CONFIG_MODULES_SENSORS=y`、`CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P=y`、`CONFIG_COMMON_MAGNETOMETER=y`、`CONFIG_DRIVERS_MAGNETOMETER_BOSCH_BMM150=y`。
  - 禁用：`CONFIG_MODULES_DATAMAN=n`、`CONFIG_MODULES_LOGGER=n`、`CONFIG_DRIVERS_ADC_BOARD_ADC=n`、`CONFIG_DRIVERS_PWM_OUT=n`、`CONFIG_DRIVERS_DSHOT=n`、`CONFIG_DRIVERS_GPS=n`、`CONFIG_DRIVERS_BAROMETER=n` 等。

- 步骤5：引脚与时钟一致性核查（只读对比）
  - 引脚：SPI1 `PA5/PA6/PD7`、SPI3 `PC10/PC11/PB2`、I2C1 `PB6/PB9`、USART3 `PD8/PD9`（已与文档一致）。
  - 时钟：SPI123 使用 `PLL2P=192MHz`；HSE 8MHz、SYSCLK 480MHz（`defconfig:50-56`）。

- 步骤6：git 审查与子模块回滚（只读/执行）
  - 命令：`git status -s`、`git diff --name-only`；若子模块 `platforms/nuttx/NuttX/nuttx` 显示改动，执行 `git submodule update --init --recursive --checkout` 回滚。

- 步骤7：WSL 编译与验证（执行）
  - 命令：`wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j$(nproc)'`
  - 期望：无 `up_restoreusartint` / `px4_i2c_buses` 未定义；完成链接与内存占用输出。

**MAVLink 与传感器启动（ROMFS脚本对齐）**
- `boards/.../init/rc.board_sensors`：
  - `icm42688p start -s -b 1`（IMU1）；`icm42688p start -s -b 3 -R 8`（IMU2反向安装）；`bmm150 start -X -b 1`。
  - `sensors start`（融合后经 uORB → MAVLink 输出）。

**必要时使用 search agent**
- 若串口函数仍未生效或新错误出现，将启动搜索代理定位确切宏路径与依赖（例如 `SERIAL_HAVE_ONLY_DMA` 的定义与影响范围），并交叉对比 fmu‑v6x。

**交付**
- 修改文件及片段清单，WSL 完整编译日志与内存占用；与文档一致性说明（引脚与时钟）。

确认后我将按上述步骤依次修改与验证。