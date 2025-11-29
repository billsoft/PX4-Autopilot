## 目标与范围
- 将你的需求（2×SPI IMU、1×I2C 磁力计、2×CMOS GPIO EXTI、融合四元数、通过 uORB 与 MAVLink 经 UART 输出）完整重写进 `boards/st/nucleo-h743zi-fc/需求.md`，并引用 `.trae` 时钟与接口文档。
- 系统性检查当前代码是否与需求一致；若不一致，制定并执行重构计划，直至 WSL 构建（`make st_nucleo-h743zi-fc_default`）与 `px4.py` 构建均通过。

## 文档重写（需求.md）结构
- 硬件目标与资源：MCU、HSE/SYSCLK、供电、接口冲突（SPI2/ETH）、禁用 SD。
- 总线与引脚：SPI1/3（ICM42688P）、I2C1（BMM150）、GPIO EXTI（帧/行）、USART3（`/dev/ttyS2`）。
- 软件模块：ICM42688P 驱动、BMM150 驱动、cmos_sync、sensors、MAVLink、（新增）dual_imu_fusion。
- 启动与数据流：ROMFS 脚本启动顺序、uORB 话题、融合输出、MAVLink 流。
- 时钟与性能：参考 `.trae/documents/build/stm32_743_cubemx时钟.md`（PLL1 480MHz、SPI123≤120–150MHz建议、UART 115200）；约束与目标采样率。
- 验证与测试：NSH 命令、话题监听、MAVLink 流速检查、GPIO 同步事件。
- 参考资料链接：时钟、接口与 Pin、阶段计划文档。

## 现状对照检查（代码位置）
- I2C 总线：`boards/st/nucleo-h743zi-fc/src/i2c.cpp:1–9` 已定义 `px4_i2c_buses` 为 1/2/3 外部 + 4 内部，符合通用模式；平台弱符号兜底：`platforms/common/i2c.cpp:40`。
- 传感器启动：`boards/st/nucleo-h743zi-fc/init/rc.board_sensors:2–10` 已启动两路 ICM42688P（SPI1/SPI3）、BMM150（I2C1），并启动 `cmos_sync` 与 MAVLink 流。
- EXTI 同步 GPIO：板级常量已定义并初始化：`boards/st/nucleo-h743zi-fc/src/board_config.h:40–53`；cmos_sync 模块发布 `gpio_in`：`src/modules/cmos_sync/cmos_sync_main.cpp:1–92`。
- MAVLink 串口映射与模块：`build/st_nucleo-h743zi-fc_default/px4_boardconfig.h:15` 映射 `TELEM1=/dev/ttyS2`；`boards/st/nucleo-h743zi-fc/default.px4board:29` 使能 `CONFIG_MODULES_CMOS_SYNC=y` 与必要模块。
- NuttX 串口路径与 PM：`boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig:50–57, 80–93, 131–139` 启用 `CONFIG_PM`、`CONFIG_SERIAL_TERMIOS`、`CONFIG_PIPES`、`CONFIG_STM32H7_SERIAL_DISABLE_REORDERING`。
- HRT：`boards/st/nucleo-h743zi-fc/src/board_config.h:55–58` TIM5 配置；SPI CS：`board_config.h:18,22`。

## 差距与重构方案
- 融合算法模块缺失：需求要求“自己写融合算法融合四元数”，当前依赖系统 `sensors`/EKF 流；需新增最小 `dual_imu_fusion` 模块（订阅两路 IMU 与磁力计，输出 `vehicle_attitude` 与可选 `sensor_combined`）。
- EXTI 去抖与参数：当前 `cmos_sync` 做固定去抖（500 μs）；增加参数（最小间隔、触发边沿），并提供 `listener gpio_in` 验证脚本。
- MAVLink 流完善：将融合输出（`ATTITUDE_QUATERNION`）与原始高频 IMU（`HIGHRES_IMU`）流固定在启动脚本；保留自定义率参数。
- 模块裁剪：严禁禁用 MAVLink、Sensors；继续禁用非必需模块（SD、PWM、ADC 等），确保最小集与构建稳定。

## 实施步骤
1) 重写 `需求.md`：引用 `.trae` 时钟与接口文档，按上述结构补充目标、引脚、时钟、数据流、验证。
2) 新增融合模块 `src/modules/dual_imu_fusion/`：
   - 源文件：`DualIMUFusion.hpp/.cpp`，`CMakeLists.txt`，`module.yaml`。
   - 订阅：`sensor_accel`、`sensor_gyro`（2 路）与 `sensor_mag`；简单 Mahony/互补滤波输出 `vehicle_attitude`（四元数）。
   - 参数：增益/权重/对齐旋转；工作队列 `hp_default`。
3) 集成启动：
   - 在 `boards/st/nucleo-h743zi-fc/default.px4board` 使能 `modules__dual_imu_fusion`；
   - 在 `rc.board_sensors` 启动融合模块，维持 MAVLink 三条流；
   - `cmos_sync` 增加去抑参数（模块内部参数或脚本环境变量）。
4) 编译与验证：
   - `python Tools/px4.py build st_nucleo-h743zi-fc`（快速）与 WSL `make st_nucleo-h743zi-fc_default -j4`（严格）；
   - 运行级检查：`listener vehicle_attitude`、`listener gpio_in`、`mavlink status`、各驱动 `status`；
   - 记录构建与运行日志到 `.trae/documents/build/compilation_error_fix_report.md`。
5) WSL 构建稳定性：
   - 推荐在 WSL 本地目录构建，避免 `/mnt/d/...` 权限与子模块问题；如必须用 `/mnt/d`，先 `distclean` 并确保 `firmware.prototype` 存在（已补充）。

## 交付清单
- 重写后的 `boards/st/nucleo-h743zi-fc/需求.md`（含完整需求与参考、验证步骤）。
- 新增 `dual_imu_fusion` 模块源代码与构建配置。
- 更新的 `default.px4board` 与 `rc.board_sensors`。
- 构建产物与验证日志（含 `.px4` 包、`map`、脚本片段）。

## 成功判定
- 两种构建均通过；上电后 `icm42688p/bmm150/sensors` 工作、`gpio_in` 有同步事件、`vehicle_attitude` 正常发布；MAVLink 流稳定输出四元数与高频 IMU。

## 代码引用
- I2C 总线定义：`boards/st/nucleo-h743zi-fc/src/i2c.cpp:1–9`
- EXTI GPIO 常量与初始化：`boards/st/nucleo-h743zi-fc/src/board_config.h:40–53`
- ROMFS 启动脚本：`boards/st/nucleo-h743zi-fc/init/rc.board_sensors:2–10`
- MAVLink 串口映射：`build/st_nucleo-h743zi-fc_default/px4_boardconfig.h:15`
- NuttX 串口与 PM：`boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig:50–57, 80–93, 131–139`
- 平台 I2C 迭代：`platforms/common/i2c.cpp:65–102`