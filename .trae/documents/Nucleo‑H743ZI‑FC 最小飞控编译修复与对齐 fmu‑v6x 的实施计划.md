**对齐原则**
- 参考对象：`boards/px4/fmu-v6x`（H753 与 H743 在 NuttX 体系与串口/SPI/I2C 驱动宏高度一致），以其“自定义板 + 自包含 board.h + defconfig”的结构为准。
- 层级聚焦：优先修复 NuttX 层（串口、总线、板级符号）的编译/链接问题，再最小化 PX4 模块。

**已知错误与根因（NuttX 层）**
- 链接错误1：`up_restoreusartint` 未定义（`stm32_serial.c`）。
  - 根因：该函数仅在条件成立时生成：`!SERIAL_HAVE_ONLY_DMA || CONFIG_PM || HAVE_RS485`。当前组合可能为“仅DMA且无PM”，导致未生成。
- 链接错误2：`px4_i2c_buses` 未定义（平台层引用）。
  - 根因：板级未提供该常量数组或被条件编译排除，导致最终链接缺符号。

**对齐 fmu‑v6x 的修复方案**
- 步骤A：defconfig（NuttX 配置）对齐串口与管道
  - 添加/确认：
    - `CONFIG_ARCH_BOARD_CUSTOM=y` 与 `CONFIG_ARCH_BOARD_CUSTOM_*` 指向 `boards/st/nucleo-h743zi-fc/nuttx-config`（确保使用自定义板）。
    - `CONFIG_PIPES=y`（MAVLink shell 与内部通信需要）。
    - `CONFIG_SERIAL_TERMIOS=y`、`CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y`（与 fmu‑v6x 串口路径一致）。
  - 修复 `up_restoreusartint`：
    - 方案优先：启用 `CONFIG_PM=y`（满足条件编译，保留 USART3 DMA）。
    - 备选：若不启用 PM，则关闭控制台 UART 的 `TXDMA`（保留 `RXDMA`），打破 “only DMA” 路径。

- 步骤B：板级 I2C 总线符号（对齐 fmu‑v6x）
  - 在 `boards/st/nucleo-h743zi-fc/src/i2c.cpp` 定义：
    - 头：`#include <px4_arch/i2c_hw_description.h>` 与 `#include <px4_platform_common/i2c.h>`。
    - 常量：`const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = { initI2CBusExternal(1) };`
    - 移除不必要的条件编译（避免被 `CONFIG_I2C` 宏意外排除）。

- 步骤C：板级 SPI 电源控制重复定义清理
  - 移除板级 `board_control_spi_sensors_power*` 的重复实现，保留通用 `stm32_common/spi/spi.cpp`（对齐 fmu‑v6x，消除链接“多重定义”风险）。

- 步骤D：PX4 模块最小化（满足功能，减少报错）
  - 在 `default.px4board` 保留：`MAVLink`、`SENSORS`、`ICM42688P`、`BMM150` 与 `COMMON_MAGNETOMETER`。
  - 禁用：`DATAMAN`、`LOGGER`、`ADC`、`GPS`、`PWM/DShot`、`BAROMETER` 等非必需模块，符合“无 TF 卡、最小飞控”。

- 步骤E：时钟与引脚一致性核查（基于文档与模板）
  - 引脚：SPI1（PA5/PA6/PD7）、SPI3（PC10/PC11/PB2）、I2C1（PB6/PB9）、USART3（PD8/PD9），与 `开发板接口和Pin配置.md` 一致。
  - 时钟：SPI123 选择 `PLL2P=192MHz`，不超过 200MHz；HSE=8MHz，SYSCLK=480MHz（参照 `stm32_743_cubemx时钟.md`）。

- 步骤F：git 审查与子模块回滚
  - 执行：`git status -s` 与 `git diff --name-only`，确认改动仅限 `boards/st/nucleo-h743zi-fc/*` 与必要的 `platforms/common/*`。
  - 若看到 `platforms/nuttx/NuttX/nuttx` 子模块有改动：`git submodule update --init --recursive --checkout` 回滚，避免依赖子模块变更。

- 步骤G：WSL 构建与验证（按 CLAUDE.md）
  - 命令：`wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j$(nproc)'`。
  - 验证：
    - 无 `up_restoreusartint` 与 `px4_i2c_buses` 未定义；
    - 生成 `.elf/.px4`，输出内存占用表；
    - ROMFS 启动 `rc.board_sensors` 成功加载两路 IMU 与磁力计。

**参考映射（fmu‑v6x → 我们）**
- 目录结构、`CONFIG_ARCH_BOARD_CUSTOM`、自包含 `board.h`：完全对齐。
- I2C/SPI 总线数组：从 fmu‑v6x 的 `i2c.cpp`/`spi.cpp` 模式抽取到我们的板级。
- 串口与管道：启用 `PIPES` 与合适的串口宏；通过 `CONFIG_PM` 保障 `stm32_serial.c` 函数生成。

**交付内容**
- 具体修改文件列表与关键片段（defconfig、i2c.cpp、default.px4board）。
- 完整编译日志摘要（链接与内存占用）。
- 与 `.trae/documents/build/*.md` 的一致性说明（引脚与时钟对齐）。

如确认以上对齐与方案，我将实施：启用 `CONFIG_PM` 修复串口链接错误；定义板级 `px4_i2c_buses`；最小化模块；回滚子模块改动；WSL 编译验证并交付结果。