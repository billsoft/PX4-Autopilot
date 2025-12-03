# 传感器桩模式快速参考卡

**最后更新**: 2025-12-02
**文件位置**: `boards/st/nucleo-h743zi-fc/init/rc.board_sensors`

---

## 🚨 关键配置行

```bash
# boards/st/nucleo-h743zi-fc/init/rc.board_sensors 第13行
USE_SENSOR_STUB=1  # ⭐ 修改这里：1=开发模式  0=生产模式
```

---

## 📋 快速对照表

| 配置值 | 模式 | 硬件要求 | 数据来源 | 适用场景 |
|--------|------|----------|----------|----------|
| `USE_SENSOR_STUB=1` | 🔧 **开发模式** | ❌ 无需硬件 | 模拟数据 | 软件开发/算法验证 |
| `USE_SENSOR_STUB=0` | 🚀 **生产模式** | ✅ 必须连接 | 真实数据 | 硬件验证/飞行测试 |

---

## ⚙️ 开发模式详情

### 配置
```bash
USE_SENSOR_STUB=1
```

### 行为
- ✅ 启动 `sensor_stub` 模块
- ✅ 发布模拟IMU数据（200Hz，双实例）
- ✅ 发布模拟磁力计数据（50Hz）
- ✅ 模拟静止状态：accel_z = 9.81 m/s²
- ✅ 融合算法正常运行（使用模拟数据）
- ✅ MAVLink正常输出姿态

### 启动日志标识
```
==========================================
  ⚠️  开发模式：使用传感器桩模块
  传感器数据为模拟数据，非真实硬件！
==========================================
OK: sensor_stub已启动（200Hz模拟数据）
✅ dual_imu_fusion已启动
✅ 开发模式启动完成
  使用 'listener vehicle_attitude' 查看姿态
  ⚠️  数据为模拟数据！
```

### 适用场景
- ✅ 无硬件连接时开发代码
- ✅ 验证融合算法逻辑
- ✅ 测试MAVLink通信
- ✅ 调试LED指示模块
- ✅ 编译通过性验证

### ⚠️ 警告
- ❌ **数据完全是假的！**
- ❌ 硬件故障也能正常启动
- ❌ **绝对不能用于飞行测试**
- ❌ 无法验证真实硬件状态

---

## 🚀 生产模式详情

### 配置
```bash
USE_SENSOR_STUB=0
```

### 行为
- ✅ 启动真实传感器驱动：`icm42688p`、`bmm150`
- ✅ **验证每个传感器启动状态**
- ✅ 任一传感器失败 → 拒绝启动融合算法
- ✅ 明确报错并退出（exit 1）

### 启动日志标识（成功）
```
==========================================
  生产模式：启动真实传感器
==========================================
启动 IMU1 (SPI1)...
启动 IMU2 (SPI3)...
启动 BMM150 (I2C1)...
✅ IMU1 (SPI1) 运行正常
✅ IMU2 (SPI3) 运行正常
✅ BMM150 (I2C1) 运行正常
✅ dual_imu_fusion已启动
✅ 生产模式启动完成
  所有传感器运行正常
```

### 启动日志标识（失败）
```
❌ ERROR: IMU1 (SPI1) 启动失败！
❌ ERROR: BMM150 (I2C1) 启动失败！
==========================================
  ⛔ 传感器硬件故障！
  请检查：
  1. 硬件连接是否正确
  2. 焊接是否良好
  3. 引脚配置是否匹配
  4. 使用 i2cdetect -b 1 检测I2C设备
==========================================
传感器失败，跳过融合算法启动
```

### 适用场景
- ✅ 硬件已连接
- ✅ 验证传感器工作正常
- ✅ 飞行前系统检查
- ✅ 生产环境部署

### ✅ 安全特性
- ✅ 硬件故障会被明确检测
- ✅ 融合算法不会使用假数据
- ✅ 系统拒绝启动（而非假装正常）
- ✅ LED红灯指示故障

---

## 🔄 模式切换流程

### 第一次使用（无硬件）

```bash
# 1. 确认开发模式
vim boards/st/nucleo-h743zi-fc/init/rc.board_sensors
# 第13行确认：USE_SENSOR_STUB=1

# 2. 编译
wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default'

# 3. 烧录（Windows）
STM32_Programmer_CLI.exe -c port=SWD -w build/st_nucleo-h743zi-fc_default/st_nucleo-h743zi-fc_default.elf -v -rst

# 4. 验证（COM5 @ 115200）
nsh> dmesg | grep "开发模式"
# 应该看到："⚠️  开发模式：使用传感器桩模块"

nsh> listener vehicle_attitude
# 应该看到姿态数据更新（~120Hz）

nsh> listener sensor_accel 0
# 应该看到 z ≈ 9.81 m/s²
```

### 切换到生产模式（硬件已连接）

```bash
# 1. 修改模式
vim boards/st/nucleo-h743zi-fc/init/rc.board_sensors
# 第13行改为：USE_SENSOR_STUB=0

# 2. 重新编译
wsl bash -lc 'cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default'

# 3. 重新烧录
STM32_Programmer_CLI.exe -c port=SWD -w build/st_nucleo-h743zi-fc_default/st_nucleo-h743zi-fc_default.elf -v -rst

# 4. 验证（COM5 @ 115200）
nsh> dmesg | grep "生产模式"
# 应该看到："生产模式：启动真实传感器"

# 5. 检查传感器状态
nsh> icm42688p status
# 应该看到 2个实例都是 "Running: yes"

nsh> bmm150 status
# 应该看到 "Running: yes"

# 6. 如果有错误
nsh> dmesg | grep ERROR
# 根据错误提示排查硬件问题
```

---

## 🛠️ 故障排查

### 问题1：开发模式下看不到数据

**症状**：
```bash
nsh> listener vehicle_attitude
never published
```

**排查**：
```bash
# 1. 检查 sensor_stub 是否启动
nsh> sensor_stub status
# 应该显示运行中

# 2. 检查 dual_imu_fusion 是否启动
nsh> dual_imu_fusion status
# 应该显示运行中

# 3. 查看完整启动日志
nsh> dmesg
# 查找 ERROR 或 FAIL
```

### 问题2：生产模式下传感器失败

**症状**：
```
❌ ERROR: IMU1 (SPI1) 启动失败！
```

**排查步骤**：
```bash
# 1. 检查I2C设备（磁力计）
nsh> i2cdetect -b 1
# 应该在 0x10 地址看到设备

# 2. 手动启动传感器查看详细错误
nsh> icm42688p start -s -b 1 -R 0 -6
# 查看错误信息

# 3. 检查板级配置
# 查看 boards/st/nucleo-h743zi-fc/src/board_config.h
# 确认 GPIO_SPI1_CS_ICM42688P 引脚定义

# 4. 检查SPI总线
nsh> ls /dev/spi*
# 应该看到 /dev/spi1 和 /dev/spi3
```

### 问题3：忘记切换模式就飞行

**症状**：系统正常但数据全是假的

**检测方法**：
```bash
# 飞行前必查！
nsh> dmesg | grep "开发模式"
# 如果有输出 → ⛔ 停止！立即切换到生产模式！

nsh> dmesg | grep "生产模式"
# 应该有输出 → ✅ 安全，可以继续
```

**修复**：
1. 立即停止飞行
2. 修改 `USE_SENSOR_STUB=0`
3. 重新编译烧录
4. 验证传感器状态
5. 再次飞行

---

## 📝 常用NSH命令

### 模式检测
```bash
dmesg | grep "模式"                    # 查看当前运行模式
```

### 传感器状态
```bash
icm42688p status                      # IMU状态
bmm150 status                         # 磁力计状态
sensor_stub status                    # 桩模块状态（开发模式）
```

### 数据监控
```bash
listener vehicle_attitude             # 姿态数据（120Hz）
listener sensor_accel 0               # IMU1加速度
listener sensor_accel 1               # IMU2加速度
listener sensor_mag 0                 # 磁力计数据
uorb top                              # 实时数据流速率
```

### 模块管理
```bash
dual_imu_fusion status                # 融合算法状态
sensors status                        # 传感器预处理模块
mavlink status streams                # MAVLink流配置
board_status_leds status              # LED指示模块
```

---

## ⚡ 一句话总结

- **开发时**：`USE_SENSOR_STUB=1` → 无硬件也能跑，数据是假的
- **生产时**：`USE_SENSOR_STUB=0` → 硬件必须正常，否则拒绝启动
- **飞行前**：`dmesg | grep "开发模式"` → 有输出就停止！

---

**提示**：将此文件保存到文档目录并添加到README索引中，方便快速查阅。
