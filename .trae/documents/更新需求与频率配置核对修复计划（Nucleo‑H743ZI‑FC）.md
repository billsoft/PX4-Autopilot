## 需求更新
- 将 IMU 采样目标与融合频率补充到需求文档：
  - SPI1/SPI3 ICM42688P 采样频率 1000 Hz（驱动与管道复用 v6x）。
  - 融合输出姿态四元数频率 120 Hz。
  - GPIO EXTI 帧/行同步用于为融合四元数打时间戳，以配合相机防抖与滚动快门补偿。
  - I2C 接收磁力计数据；MAVLink 通过 UART 输出；内部数据交换采用 uORB。

## 搜索与核对
- 使用搜索代理查找：
  - ICM42688P 驱动 CLI/参数中关于采样率/ODR/频率配置（是否支持 `-f` 或参数设置）。
  - MAVLink 流配置命令用法，确认设置 120 Hz 的语法与影响。
  - ScheduledWorkItem 的频率设置，确认 120 Hz 对应调度周期（≈8333 µs）。

## 修复与实施
- 更新 `boards/st/nucleo-h743zi-fc/需求.md` 增加频率与时间戳目的说明。
- 将融合模块调度周期改为 8333 µs（120 Hz）。
- 调整 `rc.board_sensors` 中 MAVLink 流速：`ATTITUDE_QUATERNION` 120 Hz；`HIGHRES_IMU` 保守设为 120 Hz（避免 1000 Hz 串口过载；IMU 内部采样仍保持高频）。
- 如搜索表明 ICM42688P 支持 CLI 频率选项，则在 `rc.board_sensors` 增加对应参数；否则保留默认采样，后续通过参数配置或驱动默认实现。

## 构建与验证
- WSL：`make distclean && make st_nucleo-h743zi-fc_default -j4`；核对 `.px4` 产物与 ROMFS 启动。
- 运行级建议：`listener vehicle_attitude`（120 Hz）、`listener gpio_in`（同步事件）、`mavlink status`（流速）。

## 备注
- 不动其它功能；只增加与频率相关的配置与文档说明。