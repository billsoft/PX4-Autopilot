# NSH模块测试完整指南

**测试日期**: 2025-12-02
**固件版本**: st_nucleo-h743zi-fc_default
**编译状态**: ✅ 成功 (461.86 KB Flash, 2.22% AXI_SRAM)
**烧录状态**: ✅ 成功验证

---

## 📋 测试流程概览

```
连接串口 → 系统启动检查 → 硬件外设测试 → 传感器数据验证
→ 融合模块检查 → LED状态验证 → MAVLink输出检查 → 性能分析
```

---

## 1️⃣ 连接串口并启动系统

### 1.1 Windows串口连接

**查找COM口**:
```cmd
mode
# 或在设备管理器中查看 "端口(COM和LPT)"
# 通常为 COM3, COM4 等
```

**使用PuTTY连接**:
- 连接类型: Serial
- Serial line: COM3 (根据实际情况)
- Speed: 115200
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

**使用Tera Term连接**:
- Setup → Serial port
- Port: COM3
- Baud rate: 115200
- Data: 8 bit
- Parity: none
- Stop: 1 bit

### 1.2 WSL串口连接

```bash
# 查找设备
ls /dev/ttyACM*

# 使用picocom
picocom /dev/ttyACM0 -b 115200

# 或使用minicom
minicom -D /dev/ttyACM0 -b 115200
```

### 1.3 预期启动日志

连接后按下复位按钮，应该看到以下启动序列：

```
NuttShell (NSH) NuttX-12.x.x
nsh>
[Init] DMA alloc init done
[Init] Started px4_init task (id=XX)
[InitThread] Starting px4_platform_init in 10s...
nsh>
```

**关键检查点**:
- ✅ 看到 `nsh>` 提示符（说明NSH已启动）
- ✅ 看到 `[Init] Started px4_init task` （异步初始化已启动）
- ✅ 10秒后看到 `[InitThread] Calling px4_platform_init` （PX4初始化开始）

**如果没有输出**:
- 检查COM口是否正确
- 检查波特率是否为115200
- 检查USB线是否连接到ST-LINK USB口（不是Arduino USB口）
- 按下复位按钮 (黑色按钮)

---

## 2️⃣ 系统启动检查

### 2.1 查看启动日志

```bash
nsh> dmesg
```

**期望输出** (关键部分):
```
[Init] Board initialization complete
[Init] Started px4_init task (id=5)
[InitThread] Starting px4_platform_init in 10s...
[InitThread] Calling px4_platform_init
[InitThread] px4_platform_init returned: 0
[InitThread] px4_platform_configure returned: 0
[InitThread] starting board_status_leds
[InitThread] board_status_leds started

INFO  [dataman] data manager file './dataman' size is 62476 bytes
INFO  [icm42688p] SPI bus 1 devid 0x2a6d09 found ICM-42688-P
INFO  [icm42688p] on SPI bus 1 at device 1
INFO  [icm42688p] SPI bus 3 devid 0x5a6d09 found ICM-42688-P
INFO  [icm42688p] on SPI bus 3 at device 1
INFO  [bmm150] I2C bus 1 at address 0x10
INFO  [dual_imu_fusion] Starting at 120 Hz
INFO  [sensors] sensor publisher created
INFO  [mavlink] mode: Normal, data rate: 57600 B/s on /dev/ttyS2 @ 115200B
```

**关键检查**:
- [ ] 无 `ERROR` 或 `HardFault` 消息
- [ ] `icm42688p` 在SPI1和SPI3都检测到
- [ ] `bmm150` 在I2C1检测到
- [ ] `dual_imu_fusion` 启动成功
- [ ] `mavlink` 在USART3启动

**如果有ERROR**:
```bash
# 过滤错误信息
nsh> dmesg | grep -i error
nsh> dmesg | grep -i fail
nsh> dmesg | grep -i hardfault
```

---

## 3️⃣ 硬件外设检查

### 3.1 I2C1总线扫描 (BMM150磁力计)

```bash
nsh> i2cdetect -b 1
```

**期望输出**:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: 10 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --  ✅ 0x10 = BMM150
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

**✅ 成功**: 显示 `10` 在地址0x10位置
**❌ 失败**: 显示 `--` 或无设备

**如果失败**:
```bash
# 尝试备用地址
nsh> i2cdetect -b 1 -a 0x13

# 检查I2C配置
nsh> dmesg | grep i2c

# 重启BMM150驱动
nsh> bmm150 stop
nsh> bmm150 start -I -b 1 -R 0 -v  # -v启用详细日志
nsh> dmesg | grep bmm150
```

### 3.2 SPI设备检查

```bash
# 检查SPI设备节点
nsh> ls /dev/spi*
```

**期望输出**:
```
/dev/spi1  ✅
/dev/spi3  ✅
```

---

## 4️⃣ 传感器驱动状态检查

### 4.1 ICM42688P (IMU1 - SPI1)

```bash
nsh> icm42688p status
```

**期望输出**:
```
INFO  [icm42688p] SPI bus 1
        Device ID: 0x2a6d09
        Chip: ICM-42688-P (or ICM-45686 compatible)
        Temperature: 25.3 C
        Accel range: ±16 g
        Gyro range: ±2000 deg/s
        Sample rate: 1000 Hz
        FIFO: enabled
        Running: yes  ✅
```

**关键字段**:
- `Running: yes` - 驱动正在运行
- `Temperature` - 温度合理 (20-40°C)
- `Device ID` - 芯片ID正确

**如果显示 "Not running"**:
```bash
# 查看详细错误
nsh> dmesg | grep icm42688p

# 重启驱动
nsh> icm42688p stop
nsh> icm42688p start -s -b 1 -R 0 -6 -v
```

### 4.2 ICM42688P (IMU2 - SPI3)

```bash
# ICM42688P会自动显示所有实例
nsh> icm42688p status
```

**期望输出**:
```
INFO  [icm42688p] SPI bus 1  (第一个实例)
        ...
INFO  [icm42688p] SPI bus 3  ✅ (第二个实例)
        Device ID: 0x5a6d09
        Chip: ICM-42688-P (or ICM-45686 compatible)
        Running: yes
```

### 4.3 BMM150 (磁力计 - I2C1)

```bash
nsh> bmm150 status
```

**期望输出**:
```
INFO  [bmm150] I2C bus 1
        Device address: 0x10
        Chip ID: 0x32  ✅ (BMM150标识)
        Temperature: 24.8 C
        Sample rate: 100 Hz
        Running: yes  ✅
```

**如果失败**:
```bash
# 检查I2C通信
nsh> i2cdetect -b 1

# 重启驱动
nsh> bmm150 stop
nsh> bmm150 start -I -b 1 -R 0 -a 0x10
```

---

## 5️⃣ 传感器数据验证

### 5.1 监听IMU1加速度计数据 (实例0)

```bash
nsh> listener sensor_accel 0
```

**期望输出** (实时刷新, Ctrl+C退出):
```
sensor_accel #0
timestamp: 12345678 (123 us ago)  # 延迟应 <1ms
x:  0.123 m/s^2
y: -0.456 m/s^2
z: -9.81 m/s^2  ✅ (静止时接近重力加速度)
temperature: 25.3 C
device_id: 2769161
error_count: 0
clip_counter: [0 0 0]
samples: 1234
```

**关键检查**:
- `z` 轴: -9.6 ~ -10.2 m/s² (静止水平放置时)
- `temperature`: 20-40°C
- `error_count`: 0 (无通信错误)
- 更新频率: 约100-200Hz

**测试**: 倾斜板子，观察加速度值变化
- 向前倾: `x` 应增大 (正值)
- 向左倾: `y` 应增大 (正值)
- 垂直放置: `z` 应接近0, `x`或`y`接近-9.8

### 5.2 监听IMU2加速度计数据 (实例1)

```bash
nsh> listener sensor_accel 1
```

**期望输出**: 与IMU1类似，但受 `-R 8` (270度旋转) 影响
```
sensor_accel #1
timestamp: 12345678
x:  0.123 m/s^2  # 坐标系已旋转270度
y: -0.456 m/s^2
z: -9.81 m/s^2  ✅
```

### 5.3 监听陀螺仪数据

```bash
# IMU1陀螺仪
nsh> listener sensor_gyro 0
```

**期望输出** (静止时):
```
sensor_gyro #0
timestamp: 12345678
x:  0.001 rad/s  # 应接近0 (±0.05 rad/s噪声)
y: -0.002 rad/s
z:  0.000 rad/s
temperature: 25.3 C
```

**测试**: 旋转板子
- 绕X轴旋转 (Roll): `x` 应变化
- 绕Y轴旋转 (Pitch): `y` 应变化
- 绕Z轴旋转 (Yaw): `z` 应变化

### 5.4 监听磁力计数据

```bash
nsh> listener sensor_mag 0
```

**期望输出**:
```
sensor_mag #0
timestamp: 12345678
x:  0.25 Gauss  # 取决于地磁场方向
y: -0.10 Gauss
z:  0.42 Gauss
temperature: 24.8 C
is_external: false
```

**关键检查**:
- 磁场强度: `sqrt(x^2 + y^2 + z^2)` ≈ 0.3-0.6 Gauss (地磁场)
- 更新频率: 约100Hz
- 旋转板子应该看到x/y/z值变化

**如果数值异常** (全0或很大):
- 远离电脑、手机等强磁场源
- 检查I2C通信: `i2cdetect -b 1`

---

## 6️⃣ 融合模块检查

### 6.1 dual_imu_fusion模块状态

```bash
nsh> ps
```

**期望输出** (查找dual_imu_fusion进程):
```
PID   PRI   POLICY   TYPE    NPX   STATE    EVENT     SIGMASK   STACK  COMMAND
  ...
  8    100   FIFO     Kthread -     Waiting  Signal    00000000  2048   dual_imu_fusion  ✅
  9    100   FIFO     Kthread -     Waiting  Signal    00000000  2048   board_status_leds ✅
  ...
```

**检查**:
- `STATE`: Waiting (正常工作队列状态)
- `STACK`: 未溢出

**如果未找到**:
```bash
# 手动启动
nsh> dual_imu_fusion start
nsh> dmesg | grep dual_imu_fusion
```

### 6.2 监听姿态输出

```bash
nsh> listener vehicle_attitude
```

**期望输出** (120Hz):
```
vehicle_attitude #0
timestamp: 12345678 (83 us ago)  ✅ 延迟 <1ms
timestamp_sample: 12345000

# 四元数 (归一化: q[0]^2 + q[1]^2 + q[2]^2 + q[3]^2 = 1)
q[0]:  0.9998 (w)
q[1]:  0.0012 (x)
q[2]:  0.0023 (y)
q[3]:  0.0034 (z)

# 欧拉角 (静止时应接近0)
roll:   0.0024 rad ( 0.14 deg) ✅
pitch:  0.0046 rad ( 0.26 deg) ✅
yaw:    0.0068 rad ( 0.39 deg) ✅

rollspeed:  0.000 rad/s
pitchspeed: 0.000 rad/s
yawspeed:   0.000 rad/s

delta_q_reset: [0.000 0.000 0.000 0.000]
quat_reset_counter: 0
```

**关键检查**:
1. **更新频率**: 约8.3ms周期 (120Hz) ✅
2. **四元数归一化**: `sqrt(q[0]^2+...+q[3]^2) ≈ 1.0` ✅
3. **静止姿态**: roll/pitch/yaw ±2度以内 ✅
4. **延迟**: `timestamp - timestamp_sample` <1ms ✅

**手动倾斜测试**:
```bash
# 启动listener
nsh> listener vehicle_attitude

# 倾斜板子观察：
# - 向前倾 (Nose down): pitch 应减小 (负值)
# - 向后倾 (Nose up):   pitch 应增大 (正值)
# - 向右倾 (Right):     roll 应增大 (正值)
# - 向左倾 (Left):      roll 应减小 (负值)
# - 顺时针旋转 (CW):    yaw 应增大 (正值)
```

**如果姿态不更新**:
```bash
# 检查双IMU数据同步
nsh> listener sensor_accel 0  # 记录timestamp
nsh> listener sensor_accel 1  # 对比timestamp
# 时间差应 <1ms

# 检查融合模块日志
nsh> dmesg | grep dual_imu_fusion

# 重启融合模块
nsh> dual_imu_fusion stop
nsh> dual_imu_fusion start
```

---

## 7️⃣ LED状态验证

### 7.1 观察LED行为

**启动前3秒** (心跳模式):
- LED1 (绿, PB0): 慢闪 (2秒周期)
- LED2 (黄, PB7): 慢闪 (相位差0.7秒)
- LED3 (红, PB14): 慢闪 (相位差1.4秒)

**正常运行** (有传感器数据):
- LED1 (绿): 快闪 (0.5秒周期, 2Hz) → IMU1有数据
- LED2 (黄): 快闪 (0.5秒周期, 2Hz) → IMU2有数据
- LED3 (红): 快闪 (0.5秒周期, 2Hz) → 磁力计有数据

**融合激活**:
- LED1+LED2: 融合闪 (0.3秒周期, 3.3Hz, 同步) → 姿态融合工作中
- LED3: 快闪 (独立)

### 7.2 LED调试日志

```bash
nsh> dmesg | grep leds
```

**期望输出** (每5秒):
```
INFO  [board_status_leds] leds tick=50 imu1=1(1234) imu2=1(2345) mag=1(3456) fusion=1(567)
INFO  [board_status_leds] leds tick=100 imu1=1(1234) imu2=1(2345) mag=1(3456) fusion=1(567)
...
```

**字段含义**:
- `tick`: 计数器 (每100ms +1)
- `imu1=1`: IMU1有数据 (1=有, 0=无)
- `(1234)`: 距上次数据的微秒数 (<500000表示数据新鲜)
- `fusion=1`: 姿态融合有输出

**如果LED全灭**:
```bash
# 检查board_status_leds模块
nsh> ps | grep board_status_leds

# 手动测试LED
nsh> led_control -c green -l on
nsh> led_control -c yellow -l on
nsh> led_control -c red -l on
nsh> led_control -c green -l off

# 重启LED模块
nsh> board_status_leds stop
nsh> board_status_leds start
```

**测试循环模式** (10秒):
```bash
nsh> board_status_leds test 10
# 应看到 绿→黄→红 循环闪烁
```

---

## 8️⃣ MAVLink输出检查

### 8.1 查看MAVLink状态

```bash
nsh> mavlink status
```

**期望输出**:
```
instance #0:
    GCS heartbeat: valid  ✅
    MAVLink version: 2
    transport protocol: serial (/dev/ttyS2 @ 115200B)
    mode: Normal
    data rate: 57600 B/s (target)
    tx: 1234 messages, 123456 bytes
    rx: 0 messages, 0 bytes
    txerr: 0
```

**关键检查**:
- `transport protocol`: serial /dev/ttyS2 (USART3)
- `txerr: 0` (无发送错误)

### 8.2 查看MAVLink流配置

```bash
nsh> mavlink status streams
```

**期望输出**:
```
instance #0 streams:
    DEBUG: 1 Hz (enabled)  ✅
    ATTITUDE_QUATERNION: 120 Hz (enabled)  ✅
```

**不应该出现**:
- ❌ `HIGHRES_IMU` (已注释)
- ❌ `ATTITUDE` (已注释)

**如果流配置错误**:
```bash
# 手动配置流
nsh> mavlink stream -d /dev/ttyS2 -s ATTITUDE_QUATERNION -r 120

# 或重启MAVLink
nsh> mavlink stop
nsh> mavlink start -d /dev/ttyS2 -b 115200
nsh> mavlink stream -d /dev/ttyS2 -s ATTITUDE_QUATERNION -r 120
```

### 8.3 查看MAVLink消息内容

```bash
# 需要使用地面站软件接收 (QGroundControl, Mission Planner等)
# 或使用mavlink工具解析串口数据
```

**QGroundControl验证**:
1. 连接到USART3串口
2. 查看 "Analyze Tools" → "MAVLink Inspector"
3. 应该看到 `ATTITUDE_QUATERNION` (msgid=31) 以120Hz更新

---

## 9️⃣ 性能分析

### 9.1 CPU使用率

```bash
nsh> top
```

**期望输出**:
```
PID   PRI   TYPE      CPU    SIGMASK   STACK   USED    COMMAND
  1   224   Task      0.0%   00000000  3000    456     init
  2   100   Kthread   0.0%   00000000  2048    234     hpwork
  3    50   Kthread   0.0%   00000000  2048    234     lpwork
  ...
  8   100   Kthread   3.2%   00000000  2048    678     dual_imu_fusion  ✅
  9   100   Kthread   0.5%   00000000  2048    345     board_status_leds ✅
 10   100   Kthread   1.8%   00000000  2048    567     mavlink  ✅
  ...
```

**期望CPU占用**:
- `dual_imu_fusion`: 2-5% (120Hz运行)
- `board_status_leds`: <1% (10Hz运行)
- `mavlink`: 1-3%
- `icm42688p` (驱动): 2-4%
- `bmm150` (驱动): <1%
- **总计**: <15%

**如果CPU过高** (>50%):
- 检查死循环: `dmesg | grep -i warn`
- 检查栈溢出: `STACK USED` 接近 `STACK SIZE`

### 9.2 性能计数器

```bash
nsh> perf
```

**期望输出** (部分):
```
Perf counters:
  dual_imu_fusion:Run
    events: 1234
    elapsed: 12345678 us
    avg:    100 us  ✅ (120Hz = 8333us周期，处理时间100us正常)
    max:    250 us

  board_status_leds:Run
    events: 123
    avg:    50 us
```

**关键指标**:
- `dual_imu_fusion` 平均执行时间: <500us
- `board_status_leds` 平均执行时间: <100us

### 9.3 uORB消息速率

```bash
nsh> uorb top
```

**期望输出**:
```
update  interval  instance  #messages  topic name
200Hz   5.0ms     0         1234       sensor_accel  ✅
200Hz   5.0ms     1         1234       sensor_accel  ✅
200Hz   5.0ms     0         1234       sensor_gyro   ✅
200Hz   5.0ms     1         1234       sensor_gyro   ✅
100Hz   10.0ms    0         123        sensor_mag    ✅
120Hz   8.3ms     0         1456       vehicle_attitude  ✅
```

**关键检查**:
- `sensor_accel` 实例0和1: 约200Hz
- `sensor_gyro` 实例0和1: 约200Hz
- `sensor_mag`: 约100Hz
- `vehicle_attitude`: 120Hz ✅

---

## 🔟 故障排查速查表

| 问题 | 现象 | 检查命令 | 可能原因 | 解决方案 |
|------|------|----------|----------|----------|
| **无串口输出** | 黑屏 | - | USB未连接 / COM口错误 | 检查连接，复位板子 |
| **I2C扫描无设备** | `i2cdetect`全是`--` | `dmesg \| grep i2c` | SDA/SCL接线错误 | 检查PB6/PB9连接 |
| **IMU初始化失败** | `icm42688p: Not running` | `dmesg \| grep icm42688p` | SPI通信失败 | 检查SPI接线 |
| **加速度全0** | `listener sensor_accel 0` 显示0 | `icm42688p status` | 芯片未响应 | 重启驱动，检查CS引脚 |
| **姿态不更新** | `vehicle_attitude` 时间戳不变 | `listener sensor_accel 0` | IMU数据断流 | 检查双IMU是否同步 |
| **姿态漂移严重** | yaw快速变化 | `listener sensor_gyro 0` | 陀螺仪零偏大 | 静止校准，调整融合参数 |
| **LED全灭** | 所有LED熄灭 | `led_control -c green -l on` | GPIO配置错误 | 检查board_config.h极性 |
| **LED常亮** | LED不闪烁 | `ps \| grep board_status_leds` | LED模块未启动 | `board_status_leds start` |
| **MAVLink无输出** | 地面站无数据 | `mavlink status streams` | 流未配置 | `mavlink stream -s ATTITUDE_QUATERNION -r 120` |
| **CPU占用100%** | 系统卡顿 | `top` | 死循环 | 查看 `dmesg` 错误日志 |

---

## 1️⃣1️⃣ 完整测试脚本

复制以下命令到NSH逐条执行：

```bash
# ==================== 1. 系统检查 ====================
echo "=== System Boot Check ==="
dmesg | head -30
dmesg | grep -i error

# ==================== 2. 硬件外设检查 ====================
echo "=== I2C1 Scan (BMM150) ==="
i2cdetect -b 1

echo "=== SPI Devices ==="
ls /dev/spi*

# ==================== 3. 驱动状态检查 ====================
echo "=== ICM42688P Status ==="
icm42688p status

echo "=== BMM150 Status ==="
bmm150 status

# ==================== 4. 进程检查 ====================
echo "=== Process List ==="
ps | grep -E "dual_imu_fusion|board_status_leds|mavlink"

# ==================== 5. 传感器数据采样 (3秒) ====================
echo "=== Sensor Accel 0 (3s sample) ==="
timeout 3 listener sensor_accel 0

echo "=== Sensor Gyro 0 (3s sample) ==="
timeout 3 listener sensor_gyro 0

echo "=== Sensor Mag 0 (3s sample) ==="
timeout 3 listener sensor_mag 0

# ==================== 6. 姿态融合检查 (5秒) ====================
echo "=== Vehicle Attitude (5s sample) ==="
timeout 5 listener vehicle_attitude

# ==================== 7. LED调试日志 ====================
echo "=== LED Status Logs ==="
dmesg | grep leds | tail -5

# ==================== 8. MAVLink状态 ====================
echo "=== MAVLink Status ==="
mavlink status
mavlink status streams

# ==================== 9. 性能分析 ====================
echo "=== uORB Top (5s sample) ==="
timeout 5 uorb top

echo "=== Perf Counters ==="
perf

# ==================== 10. 总结 ====================
echo "=== Test Complete ==="
echo "Check above for any ERRORS or warnings"
```

---

## 1️⃣2️⃣ 预期完整测试通过标准

| 测试项 | 命令 | 通过标准 | 状态 |
|--------|------|----------|------|
| 系统启动 | `dmesg` | 无ERROR/HardFault | ⏳ 待测 |
| I2C1磁力计 | `i2cdetect -b 1` | 显示0x10 | ⏳ 待测 |
| SPI1 IMU1 | `icm42688p status` | Running: yes | ⏳ 待测 |
| SPI3 IMU2 | `icm42688p status` | 第二实例Running | ⏳ 待测 |
| IMU1数据 | `listener sensor_accel 0` | z≈-9.8, 200Hz | ⏳ 待测 |
| IMU2数据 | `listener sensor_accel 1` | z≈-9.8, 200Hz | ⏳ 待测 |
| 磁力计数据 | `listener sensor_mag 0` | 磁场0.3-0.6G, 100Hz | ⏳ 待测 |
| 姿态融合 | `listener vehicle_attitude` | 120Hz, 姿态稳定 | ⏳ 待测 |
| LED指示 | 目视 | 快闪/融合闪正确 | ⏳ 待测 |
| MAVLink | `mavlink status streams` | ATTITUDE_QUATERNION 120Hz | ⏳ 待测 |

---

**文档版本**: v1.0
**下一步**: 连接串口，执行测试命令，记录实际日志并分析
