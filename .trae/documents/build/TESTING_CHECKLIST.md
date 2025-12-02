# Nucleo-H743ZI-FC 测试检查清单

**测试日期**: _______________
**测试人员**: _______________
**固件版本**: st_nucleo-h743zi-fc_default
**编译日期**: 2025-12-02
**Flash使用**: 461.86 KB / 2 MB (22.49%)

---

## ⚡ 快速验证（5分钟）

### 1️⃣ 串口连接
- [ ] COM口已连接，波特率 115200-8N1
- [ ] 看到 `nsh>` 提示符
- [ ] 按复位按钮后能看到启动日志

### 2️⃣ 系统启动检查
```bash
nsh> dmesg | grep -i error
```
**结果**: ☐ 无ERROR ✅ / ☐ 有ERROR ❌

```bash
nsh> ps | grep dual_imu
```
**结果**: ☐ dual_imu_fusion 运行中 ✅ / ☐ 未运行 ❌

### 3️⃣ 硬件设备检查
```bash
nsh> i2cdetect -b 1
```
**结果**: ☐ 显示 0x10 (BMM150) ✅ / ☐ 无设备 ❌

```bash
nsh> icm42688p status
```
**IMU1 (SPI1)**: ☐ Running ✅ / ☐ Not running ❌
**IMU2 (SPI3)**: ☐ Running ✅ / ☐ Not running ❌

```bash
nsh> bmm150 status
```
**BMM150 (I2C1)**: ☐ Running ✅ / ☐ Not running ❌

### 4️⃣ 传感器数据验证
```bash
nsh> listener sensor_accel 0
```
**静止水平时 z轴**: _______ m/s² (期望: -9.6 ~ -10.2)
**结果**: ☐ 正常 ✅ / ☐ 异常 ❌

```bash
nsh> listener vehicle_attitude
```
**更新频率**: _______ Hz (期望: 120Hz, 周期 8.3ms)
**roll**: _______ rad (期望: ±0.035 rad = ±2°)
**pitch**: _______ rad (期望: ±0.035 rad = ±2°)
**结果**: ☐ 正常 ✅ / ☐ 异常 ❌

### 5️⃣ MAVLink流配置
```bash
nsh> mavlink status streams
```
**检查项**:
- [ ] ATTITUDE_QUATERNION: 120 Hz ✅
- [ ] DEBUG: 1 Hz ✅
- [ ] 无 HIGHRES_IMU ✅
- [ ] 无 ATTITUDE ✅

**结果**: ☐ 配置正确 ✅ / ☐ 配置错误 ❌

### 6️⃣ LED状态观察
**观察到的LED状态**:
- [ ] 启动时: LED1(绿) 闪3次
- [ ] 正常运行: LED1+LED2 快闪或融合闪 (3.3Hz)
- [ ] LED3(红) 独立闪烁

**结果**: ☐ 正常 ✅ / ☐ 异常 ❌

---

## 🔍 详细验证（30分钟）

### 7️⃣ 完整 dmesg 日志
```bash
nsh> dmesg
```
**粘贴完整日志**:
```
[粘贴 dmesg 完整输出]
```

**关键检查项**:
- [ ] `[board] board_app_initialize: Starting`
- [ ] `[px4_init] Starting px4_platform_init`
- [ ] `INFO [icm42688p] Starting on SPI1`
- [ ] `INFO [icm42688p] WHO_AM_I: 0x91 (ICM45686)`
- [ ] `INFO [bmm150] Starting on I2C1`
- [ ] `INFO [bmm150] Chip ID: 0x32`
- [ ] `INFO [dual_imu_fusion] Started successfully`

### 8️⃣ 进程列表
```bash
nsh> ps
```
**关键进程检查**:
- [ ] dual_imu_fusion
- [ ] board_status_leds
- [ ] sensors
- [ ] mavlink
- [ ] icm42688p (实例0)
- [ ] icm42688p (实例1)
- [ ] bmm150

### 9️⃣ 性能分析
```bash
nsh> top
```
**CPU使用率**: _______% (期望: <20%)
**dual_imu_fusion CPU**: _______% (期望: <5%)
**board_status_leds CPU**: _______% (期望: <1%)

```bash
nsh> uorb top
```
**Topic更新率检查**:
- sensor_accel (实例0): _______ Hz (期望: ~200Hz)
- sensor_accel (实例1): _______ Hz (期望: ~200Hz)
- sensor_gyro (实例0): _______ Hz (期望: ~200Hz)
- sensor_gyro (实例1): _______ Hz (期望: ~200Hz)
- sensor_mag (实例0): _______ Hz (期望: ~100Hz)
- vehicle_attitude: _______ Hz (期望: 120Hz)

```bash
nsh> free
```
**内存使用**:
- 总内存: _______ KB
- 已用: _______ KB
- 可用: _______ KB
- 使用率: _______% (期望: <80%)

### 🔟 动态测试
**倾斜测试** (用 `listener vehicle_attitude` 观察):
- [ ] 向前倾: pitch 减小 (负值)
- [ ] 向后倾: pitch 增大 (正值)
- [ ] 向右倾: roll 增大 (正值)
- [ ] 向左倾: roll 减小 (负值)
- [ ] 顺时针旋转: yaw 增大 (正值)

**加速度计测试** (用 `listener sensor_accel 0` 观察):
- [ ] 静止水平: z ≈ -9.8 m/s²
- [ ] 垂直放置: z ≈ 0, x或y ≈ ±9.8
- [ ] 倒置: z ≈ +9.8 m/s²

---

## ⚠️ 故障排查

### 问题1: 系统无串口输出
**可能原因**:
- [ ] 串口线未连接
- [ ] COM口选择错误
- [ ] 波特率设置错误（应为115200）
- [ ] 板子未上电或未复位

**解决方案**:
1. 检查USB线连接
2. 设备管理器确认COM口号
3. 重新上电或按复位按钮
4. 检查USART3引脚是否正确 (PD8/PD9)

### 问题2: dmesg显示ERROR
**常见错误**:
```
ERROR [icm42688p] SPI1: WHO_AM_I mismatch
```
**原因**: 启动命令缺少 `-6` 参数
**解决**: 检查 `rc.board_sensors` 脚本

```
ERROR [bmm150] I2C1: Device not found
```
**原因**: I2C引脚错误或设备地址错误
**解决**:
1. 检查 I2C1_SDA 是否为 PB9 (不是PB7!)
2. `i2cdetect -b 1` 扫描设备

```
ERROR [dual_imu_fusion] No IMU data
```
**原因**: IMU驱动未启动或数据未发布
**解决**: 检查 `icm42688p status` 确认驱动状态

### 问题3: 姿态不更新
**检查步骤**:
1. `listener sensor_accel 0` - 确认IMU有数据
2. `listener sensor_gyro 0` - 确认陀螺仪有数据
3. `dual_imu_fusion status` - 确认融合模块运行
4. 检查时间戳是否同步

### 问题4: MAVLink流配置错误
**现象**: QGC看不到姿态数据
**检查**:
```bash
nsh> mavlink status streams
```
**期望输出**:
```
ATTITUDE_QUATERNION: 120 Hz
DEBUG: 1 Hz
```

**修复**:
```bash
nsh> mavlink stream -u -r 120 -s ATTITUDE_QUATERNION
```

---

## ✅ 测试通过标准

### 最低要求（系统可用）
- [x] dmesg 无 ERROR
- [x] I2C扫描显示 0x10
- [x] 两个IMU都 Running
- [x] 加速度 z 轴 -9.6 ~ -10.2 m/s²
- [x] 姿态 120Hz 更新
- [x] MAVLink流配置正确

### 理想状态（生产就绪）
- [x] 所有最低要求通过
- [x] LED状态指示正常
- [x] 动态测试全部通过
- [x] CPU使用率 < 20%
- [x] 内存使用率 < 80%
- [x] 连续运行30分钟无崩溃

---

## 📝 测试记录

**测试结果**: ☐ 完全通过 / ☐ 部分通过 / ☐ 失败

**通过项**: _____ / 30 (通过率: _____%)

**发现的问题**:
1. ________________________________________
2. ________________________________________
3. ________________________________________

**下一步计划**:
- [ ] ________________________________________
- [ ] ________________________________________
- [ ] ________________________________________

**备注**:
```
[其他重要信息]
```

---

**签名**: ________________
**日期**: ________________
