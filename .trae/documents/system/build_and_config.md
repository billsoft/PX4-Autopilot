# 构建与板级配置（精简版）

**核心流程**
- 使用 WSL2 + Ubuntu 构建：`bash Tools/setup/ubuntu.sh`；目标 `px4_fmu-v6x_default`。
- 构建：`make px4_fmu-v6x_default -j$(nproc)`；烧写：`make px4_fmu-v6x_default upload`。
- 切换/更新后清理：`make clean`；严重错乱：`make distclean && git submodule update --init --recursive`。

**体系结构**
- 配置入口：`CMakeLists.txt` → `cmake/px4_config.cmake` → `Kconfig` → `boards/*/*.px4board`。
- 板级裁剪通过 `.px4board` 选择模块与驱动。

**常见踩坑（避免）**
- 子模块未递归：克隆后务必 `--recursive` 或 `git submodule update --init --recursive`。
- 构建目标错误：Pixhawk 6X 选 `px4_fmu-v6x_default`，别用 v5x。
- 本机工具链混用：优先用 WSL2 `Tools/setup/ubuntu.sh` 安装的工具链。
- 未清缓存：切分支/改配置后先 `make clean`。

---

## DDS 模块最终说明（FMU‑v6X）
- 启用 `CONFIG_MODULES_UXRCE_DDS_CLIENT=y` 会显著增大固件；在 `px4_fmu-v6x_default` 链接期会触发 `FLASH` 溢出（约 16 KB），构建失败。
- 当前需求不依赖 DDS；保持 `CONFIG_MODULES_UXRCE_DDS_CLIENT=n`（`boards/px4/fmu-v6x/default.px4board:86`）即可稳定编译与运行。
- 若必须开启 DDS：需同时裁剪不必要驱动/模块并精简 DDS 话题映射，随后重建。

**网络与依赖拉取排查（WSL2）**
- 取消代理：`unset http_proxy https_proxy`；`git config --global --unset http.proxy`；`git config --global --unset https.proxy`。
- 降级 HTTP：`git config --global http.version HTTP/1.1`。
- 连通性测试：`git ls-remote https://github.com/eProsima/Micro-CDR.git`。

---

## 串口 UART4 输出姿态（稳定版）
- 启动脚本：`ROMFS/px4fmu_common/init.d/rc.uart4_mavlink`，端口 `/dev/ttyS3`（EXT2），波特 `921600`。
- 频率与限速：`HIGHRES_IMU@200 Hz`、`ATTITUDE_QUATERNION@200 Hz`；`-r 200000 bytes/s`（稳态低丢包）。
- 设备映射：`SERIAL_EXT2=/dev/ttyS3`（`boards/px4/fmu-v6x/default.px4board`）。
- 时间戳：
  - 四元数：`vehicle_attitude.timestamp/1000` 毫秒。
  - IMU：`sensor_combined.timestamp_sample` 微秒。

**MAVLink 字段确认**
- 四元数与角速度映射：`src/modules/mavlink/streams/ATTITUDE_QUATERNION.hpp:77-85`。
- Hamilton 序：`q1..q4 = [w, x, y, z]`；角速度来自 `vehicle_angular_velocity.xyz`。

**脚本打包**
- 已将 `rc.uart4_mavlink` 纳入 ROMFS 清单：`ROMFS/px4fmu_common/init.d/CMakeLists.txt:36-49`。
- 编译完成后可在 `build/px4_fmu-v6x_default/etc/init.d/rc.uart4_mavlink` 中看到该脚本。

---

## 验证清单（会议前）
- 目标：`px4_fmu-v6x_default`；DDS 关闭：`CONFIG_MODULES_UXRCE_DDS_CLIENT=n`。
- 子模块状态正常：`git submodule status` 无悬空条目。
- ROMFS 包含脚本：构建后存在 `etc/init.d/rc.uart4_mavlink`。
- 板端验证：
  - NSH：`mavlink status` 看端口 `/dev/ttyS3` 已启动；`mavlink stream` 流速为 200 Hz。
  - 上位机：运行 `ground_station/pixhawk_monitor/examples/simple_receiver.py` 验证四元数与频率。

---

## 快速命令清单
- 初始化：`git submodule update --init --recursive`
- 安装依赖（WSL2）：`bash Tools/setup/ubuntu.sh`
- 构建：`make px4_fmu-v6x_default -j$(nproc)`
- 烧写：`make px4_fmu-v6x_default upload`
- 清理：`make clean` / `make distclean`
