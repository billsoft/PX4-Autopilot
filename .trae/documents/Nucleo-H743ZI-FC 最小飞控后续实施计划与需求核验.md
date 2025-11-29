## 需求核验
- SPI 两路 IMU：已在 `boards/st/nucleo-h743zi-fc/init/rc.board_sensors:2-3` 启动 `ICM42688P` 于 `SPI1` 与 `SPI3`；片选见 `boards/st/nucleo-h743zi-fc/src/board_config.h:18,22`。
- 磁力计：已在 `rc.board_sensors:5` 启动 `BMM150` 于 `I2C1`（外部索引 `-I -b 1`）。
- MAVLink 串口：`/dev/ttyS2`（USART3）映射为 `TELEM1`，见生成 `build/st_nucleo-h743zi-fc_default/px4_boardconfig.h:15` 与已启用 `CONFIG_MODULES_MAVLINK`。
- 构建：工程可正常构建；ROMFS 启动脚本包含 `rc.board_sensors`。
- 未完成项：
  - CMOS 同步 EXTI 输入采样与去抖、时间戳发布未实现。
  - 双 IMU + 磁力计姿态融合为四元数并通过 `uORB`/`MAVLink` 输出未实现。
  - 模块裁剪仍可进一步最小化（SD、PWM、ADC 等非必需）。

## 实施目标
- 保留并稳定：双路 IMU、单路磁力计、CMOS EXTI 同步、`vehicle_attitude` 四元数、MAVLink 输出。
- 最小化其它功能以减少干扰与报错（不改动必须依赖）。

## 技术方案
- EXTI 同步与时间戳：
  - 在 `boards/st/nucleo-h743zi-fc/src/board_config.h` 定义两路同步引脚常量（按 `.trae\documents\build\开发板接口和Pin配置.md`）。
  - 在 `boards/st/nucleo-h743zi-fc/src/init.cpp` 配置为 `GPIO_INPUT|GPIO_EXTI`，注册中断回调，使用 HRT (`TIM5`) 获取微秒时间戳并简单去抖（最小间隔与状态机）。
  - 发布 `uORB` 话题（新建如 `cmos_sync`），字段：`timestamp_us`、`line`（行/帧）、`edge`（上/下沿）、`debounced`。
- 双 IMU融合模块：
  - 新建 `src/modules/dual_imu_fusion/`（`DualIMUFusion.hpp/.cpp`、`module.yaml`、`CMakeLists.txt`）。
  - 订阅 2 路 `sensor_accel`/`sensor_gyro` 与 1 路 `sensor_mag`，支持旋转校正；实现轻量 Mahony/互补滤波融合输出 `vehicle_attitude` 与 `sensor_combined`（可选）。
  - 在 `boards/st/nucleo-h743zi-fc/default.px4board` 启用该模块并在 `rcS` 启动。
- MAVLink 流配置：
  - 在 `etc/init.d/rc.serial` 或新增片段中设置：`MAV_0_CONFIG=TELEM1`、`MAV_0_MODE=custom`、`MAV_0_RATE=200`。
  - 添加流：`mavlink stream -u -r 100 -s ATTITUDE_QUATERNION`、`mavlink stream -u -r 100 -s HIGHRES_IMU`、`mavlink stream -u -r 50 -s ATTITUDE
`。
- 模块裁剪：
  - 在 `boards/st/nucleo-h743zi-fc/default.px4board` 仅保留：`drivers__imu__icm42688p`、`drivers__magnetometer__bmm150`、`modules__sensors`、`modules__mavlink`、`modules__dual_imu_fusion`、必要系统命令与 `dataman`（满足依赖）。
  - 禁用非必需：`logger`、`navigator`、`uavcan`、`sd_*`、`pwm_*`、`adc` 等（按依赖图逐项验证）。
- 时钟与总线频率检查：
  - 参考 `.trae\documents\build\stm32_743_cubemx时钟.md`，确认 `PLL2P` 供 `SPI123` ≤ 200MHz，`ICM42688P` SPI 设定频率如 `-f 10000`（必要时在 `rc.board_sensors` 指定）。

## 变更点清单
- `boards/st/nucleo-h743zi-fc/src/board_config.h`：新增同步引脚定义；如需扩展 I2C 外部索引列表。
- `boards/st/nucleo-h743zi-fc/src/init.cpp`：EXTI GPIO 配置与中断回调，HRT 时间戳，uORB 发布。
- `src/modules/dual_imu_fusion/`：新增模块文件与构建清单。
- `boards/st/nucleo-h743zi-fc/default.px4board`：启用/禁用模块裁剪与 `MAVLink`；必要参数（如 `CONFIG_NUM_MISSION_ITMES_SUPPORTED` 已存在）。
- `etc/init.d/rcS`/`rc.serial`/`rc.board_sensors`：增加融合模块启动与 MAVLink 流配置。

## 验证步骤
- 编译：`python Tools/px4.py build st_nucleo-h743zi-fc`。
- 运行时检查：
  - `icm42688p status`、`bmm150 status`、`sensors status`。
  - `listener vehicle_attitude` 与 `listener cmos_sync` 查看话题。
  - `mavlink status` 与 `mavlink stream` 输出速率核验。
  - `gpio status` 或专用命令验证 EXTI 触发与去抖效果。

## 交付物
- EXTI 同步实现（板级 + uORB 话题）。
- `dual_imu_fusion` 模块（源代码、构建、启动）。
- 更新的 `default.px4board` 与 `rcS/rc.serial/rc.board_sensors`。
- 验证报告（构建日志、话题与 MAVLink 流截图/日志）。

## 风险与缓解
- 过度裁剪导致依赖缺失：逐项裁剪并构建验证；保留 `dataman` 与 `sensors`。
- EXTI 抖动与误触：最小间隔与边沿滤波、统计计数辅助调试。
- 融合稳定性：先用互补滤波，后续可切换 Mahony/参数化增益。