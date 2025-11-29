## 一致性核对结论
- MAVLink：启用且映射正确（/dev/ttyS2）；见 build/st_nucleo-h743zi-fc_default/px4_boardconfig.h:15。
- uORB：发布路径完整——融合模块发布 vehicle_attitude（src/modules/dual_imu_fusion/DualIMUFusion.cpp:57–63）；同步模块发布 gpio_in（src/modules/cmos_sync/cmos_sync_main.cpp:66–71）。
- 传感器驱动：ICM42688P 两路（SPI1/SPI3）与 BMM150（I2C1）由 rc.board_sensors 启动（boards/st/nucleo-h743zi-fc/init/rc.board_sensors:2–12）。
- 总线与引脚：
  - I2C 数组定义完整（boards/st/nucleo-h743zi-fc/src/i2c.cpp:4–8）。
  - SPI 片选与总线宏存在（boards/st/nucleo-h743zi-fc/src/board_config.h:16–23）。
  - EXTI 行/帧 GPIO 常量与初始化列表存在（boards/st/nucleo-h743zi-fc/src/board_config.h:40–53）。
- NuttX 与时钟：HSE=8MHz、SYSCLK=480MHz、PLL M/N/P/Q/R 配置齐备；串口路径/管道/PM 选项开启（boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig:49–57,131–139）。
- 构建：WSL 构建通过生成 .px4 包；ROMFS 启动脚本包含目标启动顺序。

## 问题与差距
- Kconfig 重复与歧义：
  - default.px4board 同时设置 MODULES_DUAL_IMU_FUSION=n 与 y（boards/st/nucleo-h743zi-fc/default.px4board:28–31）。
  - 同时存在 CONFIG_NUM_MISSION_ITMES_SUPPORTED=1 与 100（:13 与 :67）。
- 磁力计驱动集合过多：px4_boardconfig.h 显示多款磁力计驱动均启用（:27–41），与“仅 BMM150”目标不符。
- SPI3 片选：文档推荐 PA15，当前板宏设为 PB12（boards/st/nucleo-h743zi-fc/src/board_config.h:21–23），需统一以免驱动选择错误片选。
- 同步模块调度基类：cmos_sync 使用 WorkItem 并调用 ScheduleOnInterval；为风格一致与 API 规范，建议切换为 ScheduledWorkItem（dual_imu_fusion 已使用）。
- 平台 I2C 弱符号兜底：platforms/common/i2c.cpp 的弱定义仅为链接顺序容错；板级强定义稳定后建议移除，保持架构纯净。

## 修复计划（不改功能，仅净化与统一）
1) 清理 default.px4board：
   - 保留一次 MODULES_DUAL_IMU_FUSION=y，删除重复的 n。
   - 保留一次 CONFIG_NUM_MISSION_ITMES_SUPPORTED=100，删除 1。
   - 收敛磁力计驱动：只启用 BMM150；显式将其他磁力计驱动设为 n 或移除 COMMON_MAGNETOMETER 以避免自动启用多驱动。
2) 统一 SPI3 片选：
   - 将 GPIO_SPI3_CS_ICM42688P 从 PB12 改为 PA15，与文档一致；检查 rc.board_sensors 的总线/旋转参数无须变更。
3) 同步模块基类统一：
   - 将 cmos_sync 从 WorkItem 改为 ScheduledWorkItem，并包含相应头；保持 1 kHz 轮询与去抖逻辑不变。
4) 移除 I2C 弱符号兜底（可选）：
   - 删除 platforms/common/i2c.cpp 的弱导出，依赖板级强定义（i2c.cpp），确保架构整洁。
5) 构建与核验：
   - WSL：distclean + 重新构建；核对 px4_boardconfig.h 中磁力计驱动仅 BMM150。
   - 运行级：验证 vehicle_attitude、gpio_in 发布；MAVLink 流（ATTITUDE_QUATERNION/HIGHRES_IMU/ATTITUDE）。

## 交付与核对点
- default.px4board 清理完成（去重与收敛）。
- board_config.h 片选统一为 PA15。
- cmos_sync 基类调整为 ScheduledWorkItem。
- 可选移除 I2C 弱符号。
- 构建产物与验证日志。

## 备注
- 不通过“注释禁用”规避编译；所有关闭均通过 Kconfig 显式设置；所有功能路径可运行验证。