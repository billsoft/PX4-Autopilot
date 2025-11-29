**目标与范围**

* 保留并实现：双 IMU（SPI1+SPI3，ICM42688P）、I2C1 磁力计（BMM150）、两路 CMOS EXTI、USART3 MAVLink 输出。

* 严格参考：`boards/px4/fmu-v6x` 代码风格与结构；对齐 `.trae/documents/build` 中时钟与引脚文档；不修改 NuttX 子模块。

* 端到端完成：编译通过 + 板级脚本 + 驱动启动 + 融合模块 + MAVLink 输出。

**阻塞问题复盘**

* 链接期未定义：`px4_i2c_buses`（平台层引用）。根因：板级 `px4_i2c_buses` 定义的链接属性/数组维度/宏展开与平台侧不一致。

* 已修：RCC 宏未定义、串口路径（`CONFIG_PM`），但编译仍因 I2C 符号阻塞。

**阶段A：编译解阻（I2C 符号）**

* 统一 I2C 宏与数组维度（对齐 fmu‑v6x）：

  1. `boards/st/nucleo-h743zi-fc/src/i2c.cpp`

     * 替换为与 fmu‑v6x 相同风格的定义（不使用 `__EXPORT`，确保外部链接，尺寸与平台一致）：

     * 代码：

       ```cpp
       #include <px4_arch/i2c_hw_description.h>
       constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
         initI2CBusExternal(1),
         initI2CBusExternal(2),
         initI2CBusExternal(3),
         initI2CBusInternal(4)
       };
       ```

     * 说明：I2C 外部总线按 1..3，内部保留 4；与平台侧 `I2CBusIterator` 遍历逻辑匹配。
  2. `boards/st/nucleo-h743zi-fc/src/board_config.h`

     * 仅保留 `#define BOARD_NUMBER_I2C_BUSES 4`；不再定义 `PX4_NUMBER_I2C_BUSES`，避免与上层定义冲突（此前曾出现 redefined）。
  3. 若仍未解析：添加备用“强符号”定义文件 `boards/st/nucleo-h743zi-fc/src/i2c_link_fix.cpp`：

     * 代码：

       ```cpp
       #include <px4_platform_common/i2c.h>
       extern const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS];
       const px4_i2c_bus_t* __px4_i2c_buses_alias = px4_i2c_buses; // 防优化移除
       ```

     * 目的：防止链接器在 `-fvisibility=hidden` 与 LTO 情况下移除符号。

* 使用搜索代理验证：扫描 `PX4_NUMBER_I2C_BUSES` 的映射与使用处，确保平台与板级一致（尺寸=4）。

**阶段B：板级 SPI/I2C 总线与片选对齐**

* `boards/st/nucleo-h743zi-fc/src/spi.cpp`：维持 SPI1/3 两总线，IMU 设备片选分别 PD14 / PA4；与文档一致。

* `boards/st/nucleo-h743zi-fc/src/board_config.h`：保留片选 GPIO 宏与 I2C1（PB6/PB9），不调整已核对内容。

**阶段C：最小化 .px4board 并补足宏**

* `boards/st/nucleo-h743zi-fc/default.px4board`：

  * 保留：`CONFIG_MODULES_MAVLINK=y`、`CONFIG_MODULES_SENSORS=y`、`CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P=y`、`CONFIG_COMMON_MAGNETOMETER=y`、`CONFIG_DRIVERS_MAGNETOMETER_BOSCH_BMM150=y`。

  * 添加：`CONFIG_NUM_MISSION_ITMES_SUPPORTED=100`（满足 `dataman.h` 宏依赖）。

  * 禁用：SD/ADC/电机/GPS/Baro 等非必需（不影响核心功能）。

**阶段D：ROMFS 启动脚本（参考 v6x）**

* `boards/st/nucleo-h743zi-fc/init/rc.board_sensors`：双 IMU + 磁力计启动（与 `nucleo_h743zi_fc_dev_plan_stages_4_to_8.md` 一致）：

  * `icm42688p start -s -b 1 -R 0 -C 1`

  * `icm42688p start -s -b 3 -R 8 -C 2`

  * `usleep 100000`

  * `bmm150 start -I -b 1 -R 0`

  * `sensors start`

* `boards/st/nucleo-h743zi-fc/init/rcS`：主启动脚本（简化版）：

  * `dataman start`、`load_mon start`、`sh /etc/init.d/rc.board_sensors`

  * `logger start -t -b 8`

  * `mavlink start -d /dev/ttyS2 -b 115200 -m onboard -r 100000`

  * `mavlink stream -d /dev/ttyS2 -s ATTITUDE_QUATERNION -r 50` 等。

* `boards/st/nucleo-h743zi-fc/src/CMakeLists.txt` 增加 `px4_add_romfs_files(../init/rcS ../init/rc.board_sensors)`。

**阶段E：简化融合模块（可选，阶段 6）**

* `src/modules/dual_imu_fusion/`：按文档创建 `DualIMUFusion.hpp/.cpp`、`module.yaml`、`CMakeLists.txt`，发布 `vehicle_attitude`，以 500Hz 融合两路 IMU + 磁力计。

* 在 `.px4board` 启用：`CONFIG_MODULES_DUAL_IMU_FUSION=y`，并在 `rcS` 启动。

**阶段F：时钟与引脚一致性（只读核对）**

* 时钟：`defconfig` HSE=8MHz、SYSCLK=480MHz、SPI123 使用 `PLL2P≤200MHz`；与 `stm32_743_cubemx时钟.md` 一致。

* 引脚：SPI1/3、I2C1、USART3 与 `开发板接口和Pin配置.md` 一致。

**阶段G：WSL 构建验证与故障排查**

* 构建：`wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make clean && make st_nucleo-h743zi-fc_default -j$(nproc)'`

* 预期：

  * 不再出现 `px4_i2c_buses` 未定义；

  * 链接完成并输出内存占用（FLASH≈21%，AXI\_SRAM≈2%）。

* 若仍未解析：

  * 启动搜索代理确认 `PX4_NUMBER_I2C_BUSES` 展开值；

  * 将数组维度临时调整为 `I2C_BUS_MAX_BUS_ITEMS` 的实际值；

  * 必要时添加“强符号”别名文件（见阶段A第3步）。

**阶段H：交付与需求文件**

* 交付：编译日志摘要、内存占用、板级脚本、融合模块代码与启动、与文档一致性说明。

* 完成 `boards/st/nucleo-h743zi-fc/需求.md`：包含硬件资源、引脚配置、目标功能、模块启用列表、编译/烧录/测试步骤与验证项，引用上述文档与实现。

