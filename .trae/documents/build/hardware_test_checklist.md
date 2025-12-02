# Nucleo-H743ZI-FC 硬件测试验证清单

**创建日期**: 2025-12-02
**编译状态**: ✅ 成功
**当前阶段**: 硬件功能验证

---

## 🎯 测试目标

验证所有硬件功能正常工作，确保满足需求.md中的所有功能点。

---

## 📋 测试准备

### 硬件连接
- [ ] Nucleo-H743ZI开发板
- [ ] USB连接（供电 + VCP串口）
- [ ] ICM42688P IMU1 连接到SPI1（可选）
- [ ] ICM42688P IMU2 连接到SPI3（可选）
- [ ] BMM150磁力计 连接到I2C1（可选）
- [ ] CMOS传感器（帧/行同步信号到PE3/PE4，可选）

### 软件工具
- [ ] 串口终端（PuTTY/Tera Term/minicom）配置115200-8N1
- [ ] QGroundControl（MAVLink验证）
- [ ] Python `mavlink_shell.py`（可选）

### 固件烧录
```bash
# 方法1: 使用自定义脚本
python Tools/flash/flash_fw.py

# 方法2: 使用make
make st_nucleo-h743zi-fc_default upload

# 方法3: STM32CubeProgrammer
# 烧录 build/st_nucleo-h743zi-fc_default/*.bin 或 *.elf
```

---

## 🧪 阶段1: 基本系统验证（无传感器）

### 1.1 启动与NSH可用性

**操作**:
1. 上电，连接USART3（VCP）
2. 观察启动日志

**预期输出**:
```
NuttShell (NSH) NuttX-12.x.x
nsh>

启动日志应包含:
[Init] Started px4_init task (id=XX)
[InitThread] Starting px4_platform_init in 10s...
[InitThread] Calling px4_platform_init
[InitThread] px4_platform_init returned: 0
[InitThread] px4_platform_configure returned: 0
[InitThread] starting board_status_leds
[InitThread] board_status_leds started
```

**验证命令**:
```bash
nsh> help
nsh> dmesg
nsh> free
nsh> uname -a
```

**通过标准**:
- ✅ NSH提示符出现
- ✅ 无HardFault/Panic错误
- ✅ 10秒后PX4初始化日志出现
- ✅ `help`命令列出所有可用命令

---

### 1.2 LED启动指示

**观察**:
1. 上电后3秒内，观察LED行为

**预期现象**:
- 绿色LED闪烁3次（每次100ms亮 + 100ms暗）
- 随后3秒内，三色LED以2秒周期慢闪（启动心跳）
- 3秒后进入正常运行模式

**通过标准**:
- ✅ 绿灯闪3次清晰可见
- ✅ 启动心跳三色LED错开闪烁
- ✅ 无LED常亮或全灭异常

---

### 1.3 uORB系统

**验证命令**:
```bash
nsh> uorb status
# 预期: uORB running, XX topics registered

nsh> uorb top -1
# 预期: 列出所有话题（sensor_accel, sensor_mag, vehicle_attitude等）

nsh> listener -n 1 sensor_accel
# 预期（无传感器）: 无输出或超时
```

**通过标准**:
- ✅ uORB正常运行
- ✅ 话题列表包含预期话题
- ✅ listener命令不崩溃

---

### 1.4 MAVLink系统

**验证命令**:
```bash
nsh> mavlink status
# 预期输出:
#   instance #0: /dev/ttyS2 (USART3)
#   rate: ...
#   streams: ATTITUDE_QUATERNION, HIGHRES_IMU, ATTITUDE
```

**通过标准**:
- ✅ MAVLink实例运行在ttyS2
- ✅ 配置的3条流已启用
- ✅ 无错误计数增长

---

### 1.5 LED测试模式

**验证命令**:
```bash
nsh> board_status_leds status
# 预期输出:
#   Running: yes
#   IMU1: no_data / Mag: no_data / ...

nsh> board_status_leds test 10
# 观察10秒循环: 绿灯亮2秒 → 黄灯亮2秒 → 红灯亮2秒
```

**通过标准**:
- ✅ status显示模块运行中
- ✅ test模式循环显示清晰
- ✅ 测试结束后恢复正常模式

---

### 1.6 参数系统

**验证命令**:
```bash
nsh> param show
# 预期: 列出所有参数

nsh> param set TEST_VALUE 123
nsh> param get TEST_VALUE
# 预期: 显示 123

nsh> param save
nsh> reboot
# 重启后
nsh> param get TEST_VALUE
# 预期: 仍为 123（参数持久化）
```

**通过标准**:
- ✅ 参数读写正常
- ✅ 参数持久化到闪存

---

## 🧪 阶段2: SPI传感器验证

### 2.1 SPI1 - ICM42688P IMU1

**硬件连接**:
```
ICM42688P → Nucleo
VDD  → 3.3V
GND  → GND
SCK  → PA5 (SPI1_SCK)
MISO → PA6 (SPI1_MISO)
MOSI → PD7 (SPI1_MOSI)
CS   → PD14
```

**验证命令**:
```bash
nsh> icm42688p status
# 预期输出:
#   Running on SPI bus 1
#   Device ID: 0x47 (ICM42688P)
#   Temperature: XX °C

nsh> listener -n 5 sensor_accel
# 预期: 显示加速度数据，约120Hz更新
# 示例:
#   timestamp: 123456789
#   x: 0.XX, y: 0.XX, z: 9.8X (重力加速度)
#   device_id: 0xXXXXXX47

nsh> listener -n 5 sensor_gyro
# 预期: 显示陀螺仪数据
```

**LED观察**:
- ✅ 绿色LED从慢闪(2s)变为快闪(2Hz)

**通过标准**:
- ✅ WHO_AM_I正确（0x47 = ICM42688P）
- ✅ 加速度Z轴接近9.8m/s² (静止时)
- ✅ 陀螺仪数据稳定，无明显漂移
- ✅ 绿色LED快闪

---

### 2.2 SPI3 - ICM42688P IMU2

**硬件连接**:
```
ICM42688P → Nucleo
SCK  → PC10 (SPI3_SCK)
MISO → PC11 (SPI3_MISO)
MOSI → PB2 (SPI3_MOSI)
CS   → PA15
```

**验证命令**:
```bash
nsh> icm42688p status
# 预期: 显示两个实例
#   Instance #0: SPI bus 1
#   Instance #1: SPI bus 3

nsh> uorb top -1 | grep sensor_accel
# 预期: sensor_accel 0 (IMU1) 和 sensor_accel 1 (IMU2)

nsh> listener sensor_accel 1
# 预期: 显示IMU2数据
```

**LED观察**:
- ✅ 黄色LED从慢闪变为快闪

**通过标准**:
- ✅ 两个IMU实例都正常运行
- ✅ IMU2数据独立更新
- ✅ 黄色LED快闪

---

## 🧪 阶段3: I2C传感器验证

### 3.1 I2C总线扫描

**验证命令**:
```bash
nsh> i2cdetect
# 预期输出（无BMM150）:
#   I2C1: no devices found

# 预期输出（有BMM150）:
#   I2C1: 0x10 (或 0x11，取决于SDO引脚)
```

**通过标准**:
- ✅ i2cdetect不崩溃
- ✅ 检测到BMM150地址（如已连接）

---

### 3.2 BMM150磁力计

**硬件连接**:
```
BMM150 → Nucleo
VDD → 3.3V
GND → GND
SCL → PB6 (I2C1_SCL)
SDA → PB9 (I2C1_SDA)
SDO → GND (地址0x10) 或 VDD (地址0x11)
```

**验证命令**:
```bash
nsh> bmm150 status
# 预期输出:
#   Running on I2C bus 1
#   Device ID: 0x32 (BMM150)

nsh> listener -n 5 sensor_mag
# 预期: 显示磁场数据 (μT)
# 示例:
#   x: XX.X, y: XX.X, z: XX.X
#   (地球磁场约20-60μT，取决于方向)
```

**LED观察**:
- ✅ 红色LED从慢闪变为快闪

**通过标准**:
- ✅ CHIP_ID正确（0x32）
- ✅ 磁场数据合理范围
- ✅ 旋转板子时磁场数据变化
- ✅ 红色LED快闪

---

## 🧪 阶段4: 双IMU融合验证

**前提**: IMU1和IMU2都正常运行

### 4.1 融合算法运行

**验证命令**:
```bash
nsh> dual_imu_fusion status
# 预期输出:
#   Running
#   IMU1: active
#   IMU2: active
#   Output rate: 120 Hz

nsh> listener -n 10 vehicle_attitude
# 预期: 显示四元数姿态（120Hz）
# 示例:
#   timestamp: ...
#   q[0]: 1.0  (w分量，静止时接近1.0)
#   q[1]: 0.0  (x分量)
#   q[2]: 0.0  (y分量)
#   q[3]: 0.0  (z分量)
```

**LED观察**:
- ✅ 绿色和黄色LED**同步**以3.3Hz快速闪烁（融合指示）

**通过标准**:
- ✅ vehicle_attitude话题更新频率约120Hz
- ✅ 四元数模长接近1.0（|q|² = q[0]² + q[1]² + q[2]² + q[3]² ≈ 1）
- ✅ 静止时姿态稳定
- ✅ 绿黄LED同步3.3Hz闪烁

---

### 4.2 姿态响应测试

**操作**:
1. 缓慢旋转开发板（Roll/Pitch/Yaw）
2. 观察`listener vehicle_attitude`输出

**预期现象**:
- Roll（横滚）: 左右倾斜，q[1]变化
- Pitch（俯仰）: 前后倾斜，q[2]变化
- Yaw（偏航）: 旋转平面，q[3]变化

**通过标准**:
- ✅ 姿态响应旋转方向正确
- ✅ 姿态稳定，无明显漂移
- ✅ 旋转停止后姿态收敛

---

## 🧪 阶段5: MAVLink输出验证

### 5.1 QGroundControl连接

**操作**:
1. 打开QGroundControl
2. 连接到Nucleo的VCP串口（115200-8N1）

**预期现象**:
- QGC显示"Connected"
- 姿态指示器实时更新
- HUD显示Roll/Pitch角度

**通过标准**:
- ✅ QGC成功连接
- ✅ 姿态数据实时显示
- ✅ 无通信错误

---

### 5.2 MAVLink流频率验证

**工具**: `mavlink_shell.py`或QGC MAVLink Inspector

**检查项**:
```
HEARTBEAT: 1 Hz
ATTITUDE_QUATERNION: 120 Hz
HIGHRES_IMU: 120 Hz
ATTITUDE: 50 Hz
```

**通过标准**:
- ✅ 所有流频率符合配置
- ✅ 无丢包或延迟

---

## 🧪 阶段6: CMOS同步验证（可选）

**前提**: 连接CMOS传感器

### 6.1 GPIO中断配置

**验证命令**:
```bash
nsh> cmos_sync status
# 预期输出:
#   Running
#   Frame pin: PE3
#   Line pin: PE4
```

**通过标准**:
- ✅ cmos_sync模块运行中

---

### 6.2 中断时间戳验证

**验证命令**:
```bash
nsh> listener -n 20 gpio_in
# 预期: 显示EXTI中断事件与时间戳
# 示例:
#   timestamp: 123456789
#   gpio_id: 3 (PE3帧同步)
#
#   timestamp: 123456812
#   gpio_id: 4 (PE4行同步)
```

**通过标准**:
- ✅ 中断事件捕获正常
- ✅ 时间戳精度微秒级
- ✅ 帧/行同步频率符合CMOS传感器规格

---

## 🧪 阶段7: 系统压力测试

### 7.1 长时间运行稳定性

**测试**:
1. 运行12小时以上
2. 定期检查`dmesg`和`perf`

**监控指标**:
```bash
nsh> perf
# 关注:
#   - 无 HardFault 计数
#   - 无内存泄漏（free命令观察）
#   - 无异常重启

nsh> uorb top
# 关注:
#   - 话题发布频率稳定
#   - 无显著丢包
```

**通过标准**:
- ✅ 无崩溃或重启
- ✅ 内存使用稳定
- ✅ CPU负载合理

---

### 7.2 性能优化（可选）

**优化项**:
1. 启用SPI DMA:
   ```kconfig
   CONFIG_SPI_DMA=y
   CONFIG_SPI1_DMA=y
   CONFIG_SPI3_DMA=y
   ```

2. 启用UART DMA:
   ```kconfig
   CONFIG_USART3_RXDMA=y
   CONFIG_USART3_TXDMA=y
   ```

3. 调整任务优先级（如需要）

**验证**:
```bash
nsh> perf
# 比较DMA启用前后的CPU使用率
```

---

## 📊 测试记录表

| 测试项 | 预期结果 | 实际结果 | 通过/失败 | 备注 |
|--------|---------|---------|-----------|------|
| NSH启动 | 正常进入NSH | | ☐ | |
| LED启动指示 | 绿灯闪3次 | | ☐ | |
| uORB系统 | 正常运行 | | ☐ | |
| MAVLink系统 | 正常运行 | | ☐ | |
| LED测试模式 | 循环显示 | | ☐ | |
| 参数系统 | 读写持久化 | | ☐ | |
| SPI1 IMU1 | WHO_AM_I=0x47 | | ☐ | |
| SPI3 IMU2 | WHO_AM_I=0x47 | | ☐ | |
| I2C1 Mag | CHIP_ID=0x32 | | ☐ | |
| 双IMU融合 | 120Hz姿态 | | ☐ | |
| LED融合指示 | 绿黄3.3Hz | | ☐ | |
| QGC连接 | 成功连接 | | ☐ | |
| CMOS同步 | 中断捕获 | | ☐ | |
| 长期稳定性 | 12小时无崩溃 | | ☐ | |

---

## 🐛 常见问题排查

### 问题1: NSH无输出
**可能原因**:
- 串口波特率不匹配（应为115200）
- 串口线RX/TX接反
- USART3未正确配置

**排查**:
```bash
# 检查defconfig
CONFIG_USART3_SERIAL_CONSOLE=y
CONFIG_USART3_BAUD=115200
```

---

### 问题2: 传感器WHO_AM_I错误
**可能原因**:
- SPI接线错误
- CS引脚定义错误
- SPI时钟配置问题

**排查**:
1. 万用表测量CS引脚电平（应为高电平）
2. 示波器观察SPI时序
3. 检查`board_config.h`中CS引脚定义

---

### 问题3: I2C设备无响应
**可能原因**:
- I2C地址错误（0x10 vs 0x11）
- SDA/SCL上拉电阻缺失
- I2C总线冲突

**排查**:
```bash
nsh> i2cdetect
# 扫描实际地址

# 尝试两个地址
nsh> bmm150 start -I -b 1 -a 0x10
nsh> bmm150 start -I -b 1 -a 0x11
```

---

### 问题4: 融合算法无输出
**可能原因**:
- IMU数据未正常到达
- 融合算法未正确配置

**排查**:
```bash
nsh> listener sensor_accel 0  # 检查IMU1
nsh> listener sensor_accel 1  # 检查IMU2
nsh> dual_imu_fusion status   # 检查融合状态
```

---

### 问题5: LED不亮或常亮
**可能原因**:
- LED极性配置错误
- GPIO初始化失败
- board_status_leds未启动

**排查**:
```bash
nsh> board_status_leds status
nsh> board_status_leds test 10  # 强制测试

# 手动控制GPIO
nsh> gpio write /dev/gpio0 1  # 绿灯
```

---

## ✅ 验收标准

### 基本功能（必须全部通过）
- ✅ NSH正常启动
- ✅ LED启动指示正常
- ✅ uORB系统运行
- ✅ MAVLink心跳正常

### 传感器功能（至少一个IMU通过）
- ✅ SPI1或SPI3 IMU读取正常
- ✅ I2C1磁力计读取正常（如有硬件）

### 高级功能（建议通过）
- ✅ 双IMU融合输出120Hz姿态
- ✅ LED状态指示符合逻辑
- ✅ QGC连接成功

### 可选功能
- ☐ CMOS同步中断捕获
- ☐ 长期稳定性测试通过
- ☐ DMA优化性能提升

---

**文档状态**: ✅ 完成
**下一步**: 开始硬件测试，按阶段逐步验证
