# rcS 中文注释版（分段行号对照）

说明：以下为入口脚本关键段落的中文说明与行号对照，帮助理解启动流程。源码路径 `ROMFS/px4fmu_common/init.d/rcS`。

## 1. 基本环境与默认变量（1-44）
- `set +e`：关闭“出错即退出”，保证脚本尽量继续执行。
- 设定常用路径与变量（SD 配置、额外脚本、参数文件、缓冲等）。
- `PARAM_DEFAULTS_VER`：机型参数版本，用于升级时触发参数重置策略。

## 2. 系统版本与 SD 卡挂载（45-118）
- `ver all` 打印版本。
- 检查 `/dev/mmcsd0` 并尝试挂载 `/fs/microsd`；必要时格式化。
- 若 SD 可用：
  - 处理硬故障日志（`hardfault_log`）。
  - 支持外部自动启动目录的热更新（108-114）。
  - 设置参数文件路径与备份文件路径（116-118）。

## 3. 外部覆盖脚本 rc.txt（121-131）
- 如存在 `/fs/microsd/etc/rc.txt` 则直接 `source`，覆盖默认 autostart 流程。
- 否则继续默认流程。

## 4. 参数与校准数据加载（129-185）
- 加载板载 MTD 校准数据与参数文件；失败时尝试从备份导入，并输出错误信息。
- 若 SD 可用，设置备份参数文件路径（select-backup）。
- 以太网网络管理（186-190）。

## 5. 参数重置与板级默认（191-230）
- 若 `SYS_AUTOCONFIG>0`：重置参数集合（保留机型/校准等指定参数）。
- 加载架构默认、板级默认、附加初始化脚本（如存在）。

## 6. 自动机型脚本与外部机型脚本（231-250）
- 执行自动机型脚本 `rc.autostart`，设置 `VEHICLE_TYPE`。
- 若仍为 `none` 且 SD 可用：尝试外部机型脚本 `rc.autostart_ext`。
- 若仍为 `none`：报错并提示 `SYS_AUTOSTART` 无效。

## 7. 参数版本一致性与提示音（253-269, 525-531）
- 参数版本不一致时：设置重置标志与重启（253-262）。
- 开机提示音在非静音或出错场景下播放（525-531）。

## 8. 外设与服务（265-301）
- `tone_alarm` 提示音驱动。
- `dataman` 数据存储后端（按参数选择后端）。
- `send_event` 事件处理器。
- `load_mon` 资源负载监视。
- `rgbled*` 状态指示灯。

## 9. 传感器系统与仿真（322-366）
- 若 HIL/SIH：按仿真模式启动传感器模拟器（332-339）。
- 否则：加载板级传感器脚本与 `rc.sensors`，启动 `sensors` 聚合。
- 电池状态/ESC 电池等按参数启用。

## 10. 状态估计器选择（369-385）
- 按参数分别启用 `ekf2` / `local_position_estimator` / `attitude_estimator_q`。

## 11. PX4IO 与 IMU 加热器（386-427）
- 若支持 PX4IO：检查并按需更新 IO 固件，然后 `px4io start`。
- 若启用热控：`heater start`。

## 12. RC 输入链路与外设捕获（430-461, 445-456）
- `rc_update` 和 `manual_control` 启动，建立遥控映射。
- 选配 `camera_trigger`、`camera_feedback`、`pps_capture`、`rpm_capture`、`camera_capture`。

## 13. Commander 与输出（465-481）
- HITL：`commander -h` + `pwm_out_sim`。
- 非 HITL：`commander start` + `dshot` + `pwm_out`。

## 14. 机型配置与校准（483-556）
- `rc.vehicle_setup` 根据 `VEHICLE_TYPE` 载入机型应用。
- 预起飞磁力计校准（`mag_bias_estimator`）。
- 板级 MAVLink 流（可选）。
- 串口配置与 MAVLink 启动（504-523）；USB 自动化失败回退 `sercon` 并 `mavlink start -d /dev/ttyACM0`。
- 导航器：`navigator start`。（基础模式下不启）
- 热校准、云台、BST、FFT/陀螺校准、PX4Flow、燃油/引擎控制等按参数选启。

## 15. 板级额外与 SD 附加脚本（586-604）
- `rc.board_extras` 与 SD 卡上的 `FEXTRAS`（`/fs/microsd/etc/extras.txt`）。

## 16. 日志管理（606-614）
- `rc.logging` 控制日志启动与参数。

## 17. autostart 后处理与引导器升级（616-629）
- `rc.autostart.post` 与板级引导器升级脚本。

## 18. UAVCAN/Cyphal/ZENOH（631-650）
- 按参数启动 UAVCAN 或 Cyphal；Zenoh 可选。

## 19. 环境清理与 MAVLink boot 完成（657-680）
- `unset` 清理脚本变量释放 RAM。
- `mavlink boot_complete` 通知系统就绪。

---

使用建议（基础模式）：
- 请使用 SD `/fs/microsd/etc/rc.txt` 覆盖默认流程，仅执行串口/传感器/MAVLink，不调用机型/估计器/控制/导航。
- 若需要 GNSS/RTK 原始数据：在 `rc.txt` 中显式 `gps start` 并通过 MAVLink 注入 RTCM。
