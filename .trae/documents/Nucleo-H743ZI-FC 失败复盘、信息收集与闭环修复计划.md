## 执行命令闭环

* 清理：`wsl make distclean`

* 构建：`wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j$(nproc)'`

* 失败即捕获首个错误并分类，按下述修复手册修正后重复上述两步，直至通过。

## 需求与文档对照

* 需求（boards/st/nucleo-h743zi-fc/需求.md）：双 SPI ICM42688P（SPI1/SPI3）、I2C1 BMM150、两路 CMOS EXTI、MAVLink（USART3=/dev/ttyS2），最小化其它模块。

* 时钟（.trae/build/stm32\_743\_cubemx时钟.md）：HSE=8MHz，SYSCLK=480MHz；SPI123时钟来自PLL，频率≤120MHz；UART3来自PCLK，115200即可。若构建报时钟相关宏缺失，参考RM0433与文档建议值（PLL M=2/N=240/P=2/Q=4/R=2）。

* 引脚与接口（.trae/build/开发板接口和Pin配置.md）：

  * SPI1：PA5/PA6/PD7，片选PD14

  * SPI3：PC10/PC11/PB2，片选PA15

  * I2C1：PB8/PB9（或PB6/PB9备用）

  * EXTI：PE3（帧）、PE4（行）推荐；本板当前使用 PE11/PE9 亦可（保持一致）

  * USART3：PD8/PD9 → `/dev/ttyS2`

* 分阶段计划（.trae/build/nucleo\_h743zi\_minimal\_flight\_controller\_dev\_plan.md 与 stages\_4\_to\_8.md）：启动脚本 `rc.board_sensors` 启动两路 ICM42688P 与 BMM150，随后 `sensors`；MAVLink 输出 ATTITUDE\_QUATERNION/HIGHRES\_IMU；可选 `dual_imu_fusion` 后续加入。

## 修复手册（分类与操作）

* 链接符号未定义

  * `px4_i2c_buses`：确保 `boards/st/nucleo-h743zi-fc/src/i2c.cpp` 定义 `constexpr px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS]`，至少包含 `initI2CBusExternal(1)`。对照 v6x（`boards/px4/fmu-v6c/src/i2c.cpp:39-43`）。

  * SPI/I2C 宏：核对 `board_config.h` 已定义 `PX4_SPI_BUS_SENSORS1=1`、`PX4_SPI_BUS_SENSORS2=3`、`PX4_I2C_BUS_EXPANSION=1` 与片选宏（PD14/PA15）。

* NuttX 串口/管道/PM路径问题

  * 在 `nuttx-config/nsh/defconfig` 保持：`CONFIG_PM=y`、`CONFIG_SERIAL_TERMIOS=y`、`CONFIG_PIPES=y`、`CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y`。

  * HSE 与 PLL：若出现频率宏不一致，按文档建议值修正（HSE=8000000，PLL M/N/P/Q/R 见上）。

* 模块缺失/裁剪不当

  * 板级 `.px4board` 必须启用：`CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P`、`CONFIG_DRIVERS_MAGNETOMETER_BOSCH_BMM150`、`CONFIG_COMMON_MAGNETOMETER`、`CONFIG_MODULES_SENSORS`、`CONFIG_MODULES_MAVLINK`、`CONFIG_MODULES_DATAMAN`。

  * 禁用不必需：`CONFIG_MODULES_LOGGER` 可留（内存日志），其余如控制器/导航/驱动保持禁用，避免依赖噪声。

* ROMFS 启动脚本

  * `boards/st/nucleo-h743zi-fc/init/rc.board_sensors` 应包含：

    * `icm42688p start -s -b 1 -R 0`

    * `icm42688p start -s -b 3 -R 8`

    * `usleep 100000`

    * `bmm150 start -I -b 1 -R 0`

    * `usleep 50000`

    * `cmos_sync start`

    * `sensors start`

    * `mavlink stream -u -r 100 -s ATTITUDE_QUATERNION`

    * `mavlink stream -u -r 100 -s HIGHRES_IMU`

    * `mavlink stream -u -r 50 -s ATTITUDE`

  * 若编译未将脚本打包，参考 v6x 在板级 `src/CMakeLists.txt` 添加 `px4_add_romfs_files(../init/rcS ../init/rc.board_sensors)`。

* EXTI 同步

  * `board_config.h` 定义两路同步 GPIO 并加入 `PX4_GPIO_INIT_LIST`；模块 `cmos_sync` 轮询发布 `gpio_in`。

## 验证清单（构建后）

* 生成物：`build/st_nucleo-h743zi-fc_default/st_nucleo-h743zi-fc_default.map` 存在；`etc/init.d/rc.board_sensors` 含上述行；`px4_boardconfig.h` 映射 `TELEM1=/dev/ttyS2`。

* WSL 成功构建后进行硬件验证（串口、uORB、MAVLink）：

  * `mavlink status` 显示串口 `/dev/ttyS2` 与配置的流。

  * `listener sensor_accel`/`sensor_gyro`/`sensor_mag`/`vehicle_attitude` 正常。

  * `listener gpio_in` 看到同步事件位掩码与时间戳。

## 成功标准

* WSL `make st_nucleo-h743zi-fc_default` 完成且无错误；最小功能保持；参考 v6x 的总线与脚本模式一致。

## 执行节奏

* 按上述命令循环：每次失败→按“修复手册”定位修复→再次 `distclean` + `make`，直至通过；期间严格对照 `.trae` 文档与 `boards/px4/fmu-v6x`。

