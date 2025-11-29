**文档与目标对齐**
- 已全面参考：
  - `.trae/documents/build/nucleo_h743zi_minimal_flight_controller_dev_plan.md`（最小飞控功能范围：SPI1/SPI3 双IMU、I2C1 磁力计、USART3 MAVLink、两路 CMOS EXTI；不含 SD/ADC/PWM）。
  - `.trae/documents/build/nucleo_h743zi_fc_dev_plan_stages_4_to_8.md`（双IMU启动脚本、IMU2 使用 SPI3 并 `-R 8`）。
  - `.trae/documents/build/开发板接口和Pin配置.md`（CubeMX 模板引脚映射与冲突规避：SPI3=PC10/PC11/PB2，I2C1=PB6/PB9，MAVLink=USART3 PD8/PD9）。
  - `.trae/documents/build/stm32_743_cubemx时钟.md`（HSE=8MHz + PLL1=480MHz；SPI 核心钟来自 `PLL2P≤200MHz`；APB≤120MHz）。
  - `.trae/documents/build/README.md`（当前状态与文档索引）。

**错误根因与修复**
- 根因：STM32H7 的 RCC 寄存器宏未包含，`px4_arch/micro_hal.h:43` 映射到 `STM32_RCC_APB1LENR` 时不可见，导致未定义。
- 修复：在 `platforms/nuttx/src/px4/stm/stm32h7/include/px4_arch/micro_hal.h` 添加 `#include <hardware/stm32_rcc.h>`，确保 `STM32_RCC_APB1LENR` 等寄存器地址宏定义可见。
- 参考代码位置：
  - 映射：`platforms/nuttx/src/px4/stm/stm32h7/include/px4_arch/micro_hal.h:43-51`
  - 使用：`platforms/nuttx/src/px4/stm/stm32_common/hrt/hrt.c:92-126,427`
  - 定义：`platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/hardware/stm32h7x3xx_rcc.h:78-89,131-139`

**HRT 配置一致性**
- `boards/st/nucleo-h743zi-fc/src/board_config.h`：定义 `#define HRT_TIMER 5` 与 `#define HRT_TIMER_CHANNEL 1`。
- `boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig`：保留 `CONFIG_TIMER=y`、`CONFIG_ONESHOT=y`；不启用 `CONFIG_STM32H7_TIM5=y`（避免与 PX4 HRT 冲突）。

**时钟与外设核查**
- SPI123 时钟源：`board.h` 采用 `PLL2P=192MHz` 与 `RCC_D2CCIP1R_SPI123SEL_PLL2`，满足 NuttX ≤200MHz 限制。
- 引脚配置：核对 `board.h`/`board_config.h` 与 `.trae` 文档一致，避免 ETH/USB/ADC 冲突；IMU2 使用 SPI3 的 `PC10/PC11/PB2`，I2C1 使用 `PB6/PB9`，USART3 使用 `PD8/PD9`。

**代码与配置审查（执行时用 git）**
- `git status -s`、`git diff --name-only` 检查改动范围；回滚 NuttX 驱动层任何不必要改动；确保以上修复与配置生效。

**WSL 编译与验证**
- 按 CLAUDE.md 的方式使用 WSL：
  - `wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j$(nproc)'`
  - 说明：`/mnt/d` 对应 Windows `D:\`；首次运行可能出现“localhost 代理未镜像到 WSL”与 `fstab mount -a` 的提示，属环境警告，可忽略。
- 期望结果：
  - 不再出现 `STM32_RCC_APB1LENR` 未定义；
  - HRT 链接符号存在（`hrt_absolute_time`、`hrt_call_*`）；
  - 生成 `.elf/.px4` 并显示内存占用表。

**交付**
- 提交修改的文件与代码片段、编译日志摘要（含链接与内存表）、与 `.trae` 文档目标一致性说明。

如确认以上方案与 WSL 构建方式一致，我将立即按此计划实施修复与验证。