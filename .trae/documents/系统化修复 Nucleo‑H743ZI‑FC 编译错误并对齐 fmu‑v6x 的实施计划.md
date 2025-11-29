**对齐文档与目标**
- 板卡与功能：Nucleo‑H743ZI，SPI1+SPI3 两路 ICM42688P，I2C1 外置磁力计 BMM150，2×GPIO EXTI，USART3 MAVLink；无 TF/SD、禁用不必要模块。
- 参考与约束：严格按 `.trae/documents/build/*` 与 `boards/px4/fmu‑v6x` 实施；不修改 NuttX 子模块；使用 WSL 构建（CLAUDE.md）。

**错误与关联代码**
- 链接未定义：`px4_i2c_buses`
  - 引用：`platforms/common/i2c.cpp:41-51,63-66,80-91`；`platforms/common/i2c_spi_buses.cpp:329,471,647-711`
  - 声明：`platforms/common/include/px4_platform_common/i2c.h:47` → `extern const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS]`
  - 我们定义：`boards/st/nucleo-h743zi-fc/src/i2c.cpp:1-8` 当前为 `const`，可能保持内部链接导致未导出
- 串口路径：`up_restoreusartint` 未定义（已通过 `defconfig:CONFIG_PM=y` 设置解决）
- dataman 宏：`CONFIG_NUM_MISSION_ITMES_SUPPORTED` 必须在 `.px4board` 提供（参考 `boards/px4/fmu‑v6x/default.px4board:81`）

**实施步骤**
1. 修复 I2C 总线数组外部链接
- 文件：`boards/st/nucleo-h743zi-fc/src/i2c.cpp`
- 变更：
  - 引入 `#include <px4_platform_common/i2c.h>`（确保 `I2C_BUS_MAX_BUS_ITEMS` 可见）
  - 将数组定义改为外部链接定义：`extern const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = { initI2CBusExternal(1) };`
  - 注：与 `i2c.h:47` 的 `extern const` 声明匹配，避免内部链接导致符号缺失；若编译器仍移除，退回 `constexpr` 风格并在 `CMakeLists.txt` 保证对象保留（参考 fmu‑v6x）。

2. 明确 I2C 总线数量宏
- 文件：`boards/st/nucleo-h743zi-fc/src/board_config.h`
- 变更：添加 `#define BOARD_NUMBER_I2C_BUSES 2`（或 1，按最小需求），确保 `PX4_NUMBER_I2C_BUSES → I2C_BUS_MAX_BUS_ITEMS` 值正确，尾部哨兵（`bus == -1`）生效。

3. `.px4board` 保留最小模块并补足宏
- 文件：`boards/st/nucleo-h743zi-fc/default.px4board`
- 变更：
  - 保留 `CONFIG_MODULES_MAVLINK=y`、`CONFIG_MODULES_SENSORS=y`、`CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P=y`、`CONFIG_COMMON_MAGNETOMETER=y`、`CONFIG_DRIVERS_MAGNETOMETER_BOSCH_BMM150=y`
  - 添加 `CONFIG_NUM_MISSION_ITMES_SUPPORTED=100`（满足 `dataman.h:75-79`）
  - 禁用 SD/ADC/电机/GPS/Baro 等非必需项

4. 串口与管道配置确认（已完成）
- 文件：`boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig`
- 保留：`CONFIG_PIPES=y`、`CONFIG_SERIAL_TERMIOS=y`、`CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y`、`CONFIG_PM=y`

5. 引脚与时钟一致性核对（只读）
- 引脚：`board_config.h:16-25` 与 `开发板接口和Pin配置.md:42-77` 一致
- 时钟：`defconfig:49-56` 与 `stm32_743_cubemx时钟.md` 对齐；SPI123 使用 `PLL2P≤200MHz`

6. git 审查与子模块回滚（已执行）
- 保持子模块为干净状态（不改 NuttX / mavlink 子模块）

7. WSL 构建与验证
- 命令：`wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j$(nproc)'`
- 期望：
  - 不再出现 `px4_i2c_buses` 未定义
  - 完成链接并打印内存占用（FLASH≈21%，AXI_SRAM≈2%）

**如遇阻塞的替代方案**
- 若 `extern const` 仍被优化移除：切换为 `constexpr` 定义（对齐 fmu‑v6x），并在 `i2c.cpp` 同时包含 `<px4_platform_common/i2c.h>`；或将数组改为 `volatile const` 防止折叠
- 启动搜索代理定位 `PX4_NUMBER_I2C_BUSES` 的定义链，确保与 `BOARD_NUMBER_I2C_BUSES` 映射正确

**交付**
- 提供修改片段与文件列表、构建日志摘要、内存占用以及与文档一致性说明