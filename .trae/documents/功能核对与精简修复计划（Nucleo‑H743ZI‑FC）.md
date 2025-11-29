## 已核对结论（现状）
- MAVLink：启用；`TELEM1=/dev/ttyS2` 映射正确（build/st_nucleo-h743zi-fc_default/px4_boardconfig.h:15）。
- uORB：`vehicle_attitude`、`gpio_in` 等话题发布路径完整：融合模块（src/modules/dual_imu_fusion/DualIMUFusion.cpp:57–63）、同步模块（src/modules/cmos_sync/cmos_sync_main.cpp:66–71）。
- 传感器驱动：ICM42688P（SPI1、SPI3）与 BMM150（I2C1）由 `rc.board_sensors` 启动（boards/st/nucleo-h743zi-fc/init/rc.board_sensors:2–11）。
- 总线配置：I2C 总线数组已定义（boards/st/nucleo-h743zi-fc/src/i2c.cpp:4–8）；SPI 片选与总线宏存在（boards/st/nucleo-h743zi-fc/src/board_config.h:16–23）。
- EXTI：CMOS 行/帧 GPIO 常量与初始化列表已加入（boards/st/nucleo-h743zi-fc/src/board_config.h:40–53）。
- 时钟与 NuttX：HSE=8MHz、SYSCLK=480MHz、PLL M/N/P/Q/R 设置齐备；串口管道/PM/TERMIOS 正确（boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig:49–57, 131–139）。
- 最小模块：`default.px4board` 已启用 MAVLink、Sensors、Dataman、CMOS 同步、融合模块；禁用SD/控制等多数非必需模块（boards/st/nucleo-h743zi-fc/default.px4board:26–54）。

## 问题与差距
- Kconfig 重复：`MODULES_DUAL_IMU_FUSION` 在 `default.px4board` 同时设置 `n` 与 `y`（行28–31）；`CONFIG_NUM_MISSION_ITMES_SUPPORTED` 同时出现两次（行13 与 67）。需清理以避免歧义。
- 磁力计驱动集合：生成 `px4_boardconfig.h` 显示多款磁力计驱动均启用（行27–41），超出最小需求。需收敛到仅 `BMM150`。
- EXTI 引脚选择：当前使用 `PE11/PE9`；文档推荐 `PE3/PE4`（更易焊、Morpho 区）。可按文档切换以统一。
- 弱符号兜底：平台层 `px4_i2c_buses` 弱定义仍存在（为防链接顺序问题）。在板级强定义稳定后可移除兜底以纯净化。
- 融合算法：当前为占位输出单位四元数；后续需实现互补或 Mahony 融合并参数化。

## 精确修复计划
1) 清理 `default.px4board` 重复与收敛：
   - 保留一次 `CONFIG_MODULES_DUAL_IMU_FUSION=y`；删除重复的 `=n`（boards/st/nucleo-h743zi-fc/default.px4board:28–31）。
   - 保留一次 `CONFIG_NUM_MISSION_ITMES_SUPPORTED=100`；删除重复的 `=1`（行13/67）。
   - 仅启用 `BMM150`：显式将其他磁力计 Kconfig 置 `n` 或移除 `CONFIG_COMMON_MAGNETOMETER` 以避免自动启用多驱动。
2) EXTI 引脚统一：
   - 将 `GPIO_CMOS_SYNC_LINE/FRAME` 从 `PE11/PE9` 改为 `PE3/PE4`（boards/st/nucleo-h743zi-fc/src/board_config.h:40–43），并保持 `PX4_GPIO_INIT_LIST`。
3) I2C 兜底清理（可选）：
   - 移除平台弱符号兜底（platforms/common/i2c.cpp 里弱导出声明），依赖板级强定义（boards/st/nucleo-h743zi-fc/src/i2c.cpp:4–8）。
4) 融合算法增强：
   - 在 `DualIMUFusion.cpp` 实现最小互补/Mahony滤波，订阅两路 IMU 与磁力计，参数化增益；输出 `vehicle_attitude`。
5) 构建与验证：
   - WSL：`make distclean && make st_nucleo-h743zi-fc_default -j4`。
   - 产物核对：`.px4` 包、`px4_boardconfig.h` 中仅 BMM150；`rc.board_sensors` 含融合与流；串口映射 `/dev/ttyS2`。
   - 运行级：`listener vehicle_attitude`、`listener gpio_in`、`mavlink status` 与流速；`icm42688p/bmm150/sensors status`。

## 验证点（文件与行）
- MAVLink 串口：build/st_nucleo-h743zi-fc_default/px4_boardconfig.h:15。
- I2C 数组：boards/st/nucleo-h743zi-fc/src/i2c.cpp:4–8。
- SPI 片选与总线：boards/st/nucleo-h743zi-fc/src/board_config.h:16–23。
- EXTI 引脚与初始化：boards/st/nucleo-h743zi-fc/src/board_config.h:40–53。
- 启动脚本：boards/st/nucleo-h743zi-fc/init/rc.board_sensors:2–12。
- NuttX 时钟与串口：boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig:49–57, 80–93, 131–139。
- 融合模块：src/modules/dual_imu_fusion/DualIMUFusion.cpp:57–63；构建入口：src/modules/dual_imu_fusion/CMakeLists.txt:1–8。

## 交付
- Kconfig 清理后的 `default.px4board`。
- 修正 EXTI 引脚后的 `board_config.h`。
- 可选移除平台弱符号与增强融合算法。
- 通过的 WSL 构建与产物，验证日志。

## 备注
- 没有通过注释“禁用”来掩盖功能；所有关闭均在 Kconfig 显式设置，传感器与输出路径完整、可运行验证。