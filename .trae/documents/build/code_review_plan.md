# Nucleo-H743ZI-FC 代码检查计划

**创建日期**: 2025-12-02
**版本**: 1.0
**目的**: 逐模块检查代码实现，确保符合需求规格说明书

---

## 📋 检查原则

1. **对照需求**: 每个检查项都对应需求.md中的功能点
2. **模块独立**: 每个模块独立检查，确保解耦
3. **uORB正确性**: 检查订阅/发布的消息类型和实例
4. **频率验证**: 确认运行频率符合120Hz要求
5. **代码质量**: 检查错误处理、日志输出、资源释放

---

## 🎯 检查清单概览

| 模块 | 状态 | 文件数 | 关键功能 | 优先级 |
|------|------|--------|---------|--------|
| 1. 板级配置 | ⏳ 待检查 | 5 | GPIO/时钟/HRT | 🔴 高 |
| 2. 传感器驱动 | ⏳ 待检查 | 已存在(PX4) | SPI/I2C通信 | 🔴 高 |
| 3. 双IMU融合 | ⏳ 待检查 | 2-3 | 噪声滤波/姿态融合 | 🔴 高 |
| 4. LED状态指示 | ✅ 已完成 | 2 | 状态监控/LED控制 | 🟡 中 |
| 5. MAVLink输出 | ⏳ 待检查 | 配置 | 流配置 | 🟢 低 |
| 6. CMOS同步 | ⏳ 待检查 | 2-3 | EXTI中断 | 🟢 低 |
| 7. 启动脚本 | ⏳ 待检查 | 1 | 模块启动顺序 | 🟡 中 |

---

## 🔍 模块1: 板级配置

### 检查目标
确保硬件定义正确，GPIO/时钟/HRT配置符合需求

### 检查文件

#### 1.1 `boards/st/nucleo-h743zi-fc/src/board_config.h`

**检查项**:
- [ ] LED GPIO定义正确（PB0/PB7/PB14）
  ```c
  GPIO_nLED_GREEN  (... | GPIO_PORTB | GPIO_PIN0)
  GPIO_nLED_YELLOW (... | GPIO_PORTB | GPIO_PIN7)
  GPIO_nLED_RED    (... | GPIO_PORTB | GPIO_PIN14)
  ```
- [ ] LED极性确认（OUTPUT_CLEAR初始化，BOARD_LED_ON=1）
- [ ] SPI CS引脚定义正确（PD14 / PA15）
- [ ] I2C总线号正确（PX4_I2C_BUS_EXPANSION = 1）
- [ ] HRT配置存在（HRT_TIMER=5, HRT_TIMER_CHANNEL=1）
- [ ] CMOS同步GPIO定义（PE3 / PE4）
- [ ] PX4_GPIO_INIT_LIST包含所有必需GPIO

**检查命令**:
```bash
# 读取文件内容
cat boards/st/nucleo-h743zi-fc/src/board_config.h | grep -E "GPIO_nLED|BOARD_LED|HRT_TIMER|GPIO_CMOS"
```

---

#### 1.2 `boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h`

**检查项**:
- [ ] PLL1配置: SYSCLK = 480MHz
- [ ] PLL2P配置: SPI时钟 = 192MHz (≤200MHz)
- [ ] SPI1 GPIO定义: PA5/PA6/PD7
- [ ] SPI3 GPIO定义: PC10/PC11/PB2
- [ ] I2C1 GPIO定义: PB6/PB9
- [ ] USART3 GPIO定义: PD8/PD9

**检查命令**:
```bash
# 检查PLL配置
grep -A 10 "STM32_PLLCFG_PLL2N" boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h

# 检查GPIO定义
grep "GPIO_SPI1\|GPIO_SPI3\|GPIO_I2C1" boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h
```

---

#### 1.3 `boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig`

**检查项**:
- [ ] HSE频率: CONFIG_STM32H7_HSE_FREQUENCY=8000000
- [ ] SPI1/SPI3启用: CONFIG_STM32H7_SPI1=y, CONFIG_STM32H7_SPI3=y
- [ ] I2C1启用: CONFIG_STM32H7_I2C1=y
- [ ] I2C晚期初始化: CONFIG_BOARD_I2C_LATEINIT=y
- [ ] USART3配置: 115200波特率，DMA初期禁用
- [ ] HRT框架: CONFIG_TIMER=y, CONFIG_ONESHOT=y
- [ ] **不启用**: CONFIG_STM32H7_TIM5=y（冲突！）
- [ ] 电源管理: CONFIG_PM=y, CONFIG_STM32H7_PWR=y
- [ ] 调试支持: CONFIG_DEBUG_HARDFAULT_ALERT=y

**检查命令**:
```bash
grep -E "CONFIG_STM32H7_(SPI|I2C|USART|TIM5)|CONFIG_TIMER|CONFIG_PM" boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig
```

---

#### 1.4 `boards/st/nucleo-h743zi-fc/src/init.cpp`

**检查项**:
- [ ] `board_app_initialize()`函数存在
- [ ] LED GPIO初始化（全部设为关闭）
- [ ] SPI CS引脚初始化（高电平去选）
- [ ] 绿色LED闪3次（启动指示）
- [ ] 创建异步任务`px4_init_thread`
- [ ] 延迟10秒启动PX4初始化
- [ ] 使用`syslog()`输出日志（不是PX4_INFO）
- [ ] 调用`px4_platform_init()`和`px4_platform_configure()`
- [ ] 启动`board_status_leds`模块

**检查命令**:
```bash
# 查看init.cpp关键函数
grep -A 20 "board_app_initialize" boards/st/nucleo-h743zi-fc/src/init.cpp
grep -A 10 "px4_init_thread" boards/st/nucleo-h743zi-fc/src/init.cpp
```

---

#### 1.5 `boards/st/nucleo-h743zi-fc/default.px4board`

**检查项**:
- [ ] 核心模块启用:
  - CONFIG_MODULES_SENSORS=y
  - CONFIG_MODULES_DUAL_IMU_FUSION=y
  - CONFIG_MODULES_BOARD_STATUS_LEDS=y
  - CONFIG_MODULES_MAVLINK=y
  - CONFIG_MODULES_DATAMAN=y
- [ ] 禁用模块确认:
  - CONFIG_MODULES_LOGGER=n
  - CONFIG_MODULES_COMMANDER=n
  - CONFIG_MODULES_EKF2=n
  - CONFIG_MODULES_NAVIGATOR=n
- [ ] 系统命令启用:
  - CONFIG_SYSTEMCMDS_I2CDETECT=y
  - CONFIG_SYSTEMCMDS_TOPIC_LISTENER=y

**检查命令**:
```bash
grep -E "CONFIG_MODULES_(SENSORS|DUAL_IMU_FUSION|BOARD_STATUS_LEDS|LOGGER|EKF2)" boards/st/nucleo-h743zi-fc/default.px4board
```

**预期问题**:
- 可能缺少`CONFIG_MODULES_DUAL_IMU_FUSION=y`（需要添加）

---

## 🔍 模块2: 传感器驱动层

### 检查目标
确认PX4现有驱动配置正确，无需修改代码

### 2.1 ICM42688P驱动（SPI）

**驱动位置**: `src/drivers/imu/invensense/icm42688p/`

**检查项**（仅配置，不检查驱动代码）:
- [ ] 驱动已编译到固件
  ```bash
  # 检查.px4board中是否启用
  grep -r "icm42688p" boards/st/nucleo-h743zi-fc/default.px4board
  ```
- [ ] SPI总线配置（`src/drivers/imu/invensense/icm42688p/`支持`-b`参数）
- [ ] 旋转矩阵支持（`-R`参数）
- [ ] `-6`参数支持（兼容42686/45686）

**启动命令验证**（在rc.board_sensors中）:
```bash
icm42688p start -s -b 1 -R 0 -6  # SPI1, 0度旋转
icm42688p start -s -b 3 -R 8 -6  # SPI3, 270度旋转(8)
```

**发布的uORB消息**:
- `sensor_accel` 实例0 (SPI1)
- `sensor_accel` 实例1 (SPI3)
- `sensor_gyro` 实例0 (SPI1)
- `sensor_gyro` 实例1 (SPI3)

**验证方法**:
```bash
nsh> icm42688p status
# 预期: 显示两个实例运行中

nsh> listener sensor_accel 0
nsh> listener sensor_accel 1
# 预期: 显示加速度数据（约200Hz）
```

---

### 2.2 BMM150驱动（I2C）

**驱动位置**: `src/drivers/magnetometer/bosch/bmm150/`

**检查项**（仅配置）:
- [ ] 驱动已编译到固件
- [ ] I2C模式支持（`-I`参数）
- [ ] 总线号支持（`-b`参数）

**启动命令验证**:
```bash
bmm150 start -I -b 1 -R 0  # I2C1, 0度旋转
```

**发布的uORB消息**:
- `sensor_mag` 实例0

**验证方法**:
```bash
nsh> bmm150 status
# 预期: Running on I2C bus 1

nsh> i2cdetect
# 预期: 检测到0x10或0x11地址

nsh> listener sensor_mag
# 预期: 显示磁场数据（约100Hz）
```

---

## 🔍 模块3: 双IMU融合算法 ⭐️核心模块

### 检查目标
实现噪声估计、滤波、姿态融合，输出120Hz四元数

### 3.1 模块文件结构

**预期文件**:
```
src/modules/dual_imu_fusion/
├── CMakeLists.txt
├── module.yaml
├── DualImuFusion.hpp
├── DualImuFusion.cpp
├── Kconfig (可选)
└── params.c (可选，参数定义)
```

**检查命令**:
```bash
ls -la src/modules/dual_imu_fusion/
```

**如果不存在**: 需要创建模块

---

### 3.2 `DualImuFusion.hpp` 检查

**必需的类定义**:
```cpp
class DualImuFusion : public ModuleBase<DualImuFusion>,
                      public ModuleParams,
                      public px4::ScheduledWorkItem {
public:
    DualImuFusion();
    ~DualImuFusion() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;  // 120Hz运行

    // uORB订阅
    uORB::Subscription _accel0_sub{ORB_ID(sensor_accel), 0};
    uORB::Subscription _accel1_sub{ORB_ID(sensor_accel), 1};
    uORB::Subscription _gyro0_sub{ORB_ID(sensor_gyro), 0};
    uORB::Subscription _gyro1_sub{ORB_ID(sensor_gyro), 1};
    uORB::Subscription _mag_sub{ORB_ID(sensor_mag), 0};

    // uORB发布
    uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};

    // 滤波器
    LowPassFilter2pFloat _noise_lpf_x;
    LowPassFilter2pFloat _noise_lpf_y;
    LowPassFilter2pFloat _noise_lpf_z;

    // 融合算法相关
    matrix::Quatf _q;  // 四元数姿态
    float _kp = 2.0f;  // 比例增益
    float _ki = 0.1f;  // 积分增益
};
```

**检查项**:
- [ ] 继承`ModuleBase`, `ModuleParams`, `ScheduledWorkItem`
- [ ] 订阅5个传感器话题（accel0/1, gyro0/1, mag0）
- [ ] 发布1个姿态话题（vehicle_attitude）
- [ ] 包含低通滤波器成员（噪声滤波用）
- [ ] 包含四元数成员（姿态状态）
- [ ] 120Hz调度（构造函数中`ScheduleOnInterval(8333)` - 1/120s=8.33ms）

---

### 3.3 `DualImuFusion.cpp` 核心逻辑检查

#### 3.3.1 初始化函数 `init()`

**检查项**:
```cpp
bool DualImuFusion::init() {
    // ✅ 设置120Hz运行频率（8.33ms周期）
    ScheduleOnInterval(8333);  // 微秒

    // ✅ 初始化低通滤波器（噪声滤波用，截止频率10Hz）
    _noise_lpf_x.set_cutoff_frequency(120.0f, 10.0f);
    _noise_lpf_y.set_cutoff_frequency(120.0f, 10.0f);
    _noise_lpf_z.set_cutoff_frequency(120.0f, 10.0f);

    // ✅ 初始化四元数（单位四元数，表示无旋转）
    _q = matrix::Quatf(1.0f, 0.0f, 0.0f, 0.0f);

    return true;
}
```

---

#### 3.3.2 主运行函数 `Run()` - 融合算法核心

**算法步骤**（按需求.md第3.2节）:

```cpp
void DualImuFusion::Run() {
    // === 步骤1: 读取传感器数据 ===
    sensor_accel_s accel0{}, accel1{};
    sensor_gyro_s gyro0{}, gyro1{};
    sensor_mag_s mag{};

    bool accel0_updated = _accel0_sub.update(&accel0);
    bool accel1_updated = _accel1_sub.update(&accel1);
    bool gyro0_updated = _gyro0_sub.update(&gyro0);
    bool gyro1_updated = _gyro1_sub.update(&gyro1);
    bool mag_updated = _mag_sub.update(&mag);

    // ✅ 检查: 至少IMU0有数据
    if (!accel0_updated || !gyro0_updated) {
        return;
    }

    // === 步骤2: 噪声估计（仅当双IMU都有数据时） ===
    matrix::Vector3f noise_estimate(0, 0, 0);

    if (accel1_updated) {
        // 噪声估计 = (IMU1 + IMU2) / 2
        noise_estimate(0) = (accel0.x + accel1.x) / 2.0f;
        noise_estimate(1) = (accel0.y + accel1.y) / 2.0f;
        noise_estimate(2) = (accel0.z + accel1.z) / 2.0f;
    } else {
        // 单IMU模式，噪声估计为0
        noise_estimate(0) = accel0.x;
        noise_estimate(1) = accel0.y;
        noise_estimate(2) = accel0.z;
    }

    // === 步骤3: 低通滤波噪声估计 ===
    float noise_lpf_x = _noise_lpf_x.apply(noise_estimate(0));
    float noise_lpf_y = _noise_lpf_y.apply(noise_estimate(1));
    float noise_lpf_z = _noise_lpf_z.apply(noise_estimate(2));

    // === 步骤4: 去除噪声，得到干净的IMU1数据 ===
    matrix::Vector3f accel_clean;
    accel_clean(0) = accel0.x - noise_lpf_x;
    accel_clean(1) = accel0.y - noise_lpf_y;
    accel_clean(2) = accel0.z - noise_lpf_z;

    matrix::Vector3f gyro_clean(gyro0.x, gyro0.y, gyro0.z);

    // === 步骤5: Mahony互补滤波器姿态融合 ===
    // 输入: accel_clean（重力方向）
    //       gyro_clean（角速度）
    //       mag（磁场，可选）

    const float dt = 1.0f / 120.0f;  // 8.33ms

    // 归一化加速度计（重力方向）
    matrix::Vector3f accel_norm = accel_clean.normalized();

    // 预测的重力方向（从四元数推导）
    matrix::Vector3f gravity_pred = _q.conjugate_inversed().rotateVector(matrix::Vector3f(0, 0, 1));

    // 误差向量（加速度计修正）
    matrix::Vector3f error_accel = accel_norm.cross(gravity_pred);

    // 磁力计修正（如果有数据）
    matrix::Vector3f error_mag(0, 0, 0);
    if (mag_updated) {
        matrix::Vector3f mag_vec(mag.x, mag.y, mag.z);
        matrix::Vector3f mag_norm = mag_vec.normalized();

        // 预测的磁场方向
        matrix::Vector3f mag_pred = _q.conjugate_inversed().rotateVector(matrix::Vector3f(1, 0, 0));

        // 误差向量（磁力计修正）
        error_mag = mag_norm.cross(mag_pred);
    }

    // 总误差
    matrix::Vector3f error_total = error_accel + error_mag;

    // PI控制器
    static matrix::Vector3f error_integral(0, 0, 0);
    error_integral += error_total * dt * _ki;

    matrix::Vector3f gyro_corrected = gyro_clean + error_total * _kp + error_integral;

    // 四元数积分（更新姿态）
    matrix::Quatf dq(1.0f,
                     gyro_corrected(0) * dt / 2.0f,
                     gyro_corrected(1) * dt / 2.0f,
                     gyro_corrected(2) * dt / 2.0f);
    _q = _q * dq;
    _q.normalize();

    // === 步骤6: 发布姿态消息 ===
    vehicle_attitude_s att{};
    att.timestamp = hrt_absolute_time();

    // 四元数 (w, x, y, z)
    att.q[0] = _q(0);
    att.q[1] = _q(1);
    att.q[2] = _q(2);
    att.q[3] = _q(3);

    // 角速度（修正后的陀螺仪数据）
    att.rollspeed = gyro_corrected(0);
    att.pitchspeed = gyro_corrected(1);
    att.yawspeed = gyro_corrected(2);

    // 可选: 欧拉角（从四元数转换）
    matrix::Eulerf euler(_q);
    att.roll = euler.phi();
    att.pitch = euler.theta();
    att.yaw = euler.psi();

    _attitude_pub.publish(att);
}
```

**检查项**:
- [ ] **步骤1**: 正确订阅5个传感器话题
- [ ] **步骤2**: 噪声估计公式正确（(IMU1+IMU2)/2）
- [ ] **步骤3**: 使用低通滤波器平滑噪声估计
- [ ] **步骤4**: IMU1数据减去滤波后的噪声
- [ ] **步骤5**: Mahony滤波器实现（或互补滤波器）
- [ ] **步骤6**: 正确填充`vehicle_attitude_s`结构体
- [ ] **频率**: 确认`ScheduleOnInterval(8333)` = 120Hz
- [ ] **时间戳**: 使用`hrt_absolute_time()`
- [ ] **归一化**: 四元数每次更新后归一化

---

### 3.4 CMakeLists.txt 检查

**预期内容**:
```cmake
px4_add_module(
    MODULE modules__dual_imu_fusion
    MAIN dual_imu_fusion
    SRCS
        DualImuFusion.cpp
    DEPENDS
        drivers__magnetometer  # 依赖磁力计驱动
        matrix                  # 数学库（四元数）
    MODULE_CONFIG
        module.yaml
)
```

**检查项**:
- [ ] 模块名称: `modules__dual_imu_fusion`
- [ ] 主函数: `dual_imu_fusion`
- [ ] 依赖matrix库（四元数运算）

---

### 3.5 module.yaml 检查

**预期内容**:
```yaml
module_name: dual_imu_fusion
serial_config:
    - command: dual_imu_fusion start
      port_config_param:
        name:
        group: Sensors
parameters:
    - group: Dual IMU Fusion
      definitions:
          DIMU_KP:
              description:
                  short: Mahony filter proportional gain
              type: float
              default: 2.0
              min: 0.0
              max: 10.0
          DIMU_KI:
              description:
                  short: Mahony filter integral gain
              type: float
              default: 0.1
              min: 0.0
              max: 1.0
          DIMU_LPF_CUTOFF:
              description:
                  short: Noise LPF cutoff frequency (Hz)
              type: float
              default: 10.0
              min: 1.0
              max: 50.0
```

**检查项**:
- [ ] 定义3个可调参数（KP, KI, LPF_CUTOFF）
- [ ] 参数有合理的默认值和范围

---

## 🔍 模块4: LED状态指示模块 ✅ (已完成)

### 检查目标
验证现有`board_status_leds`模块是否符合需求

### 4.1 文件检查

**文件位置**: `src/modules/board_status_leds/BoardStatusLEDs.cpp`

**检查项** (已在之前验证):
- [x] 订阅`sensor_accel 0/1` 和 `sensor_mag 0`
- [x] 订阅`vehicle_attitude 0`
- [x] 500ms窗口判断数据新鲜度
- [x] 启动3秒心跳模式（三色LED慢闪）
- [x] 正常模式: 无数据慢闪(2s)、有数据快闪(2Hz)、融合超快闪(3.3Hz)
- [x] 测试模式: `board_status_leds test N`

**状态**: ✅ 已完成，无需修改

---

## 🔍 模块5: MAVLink输出配置

### 检查目标
确认MAVLink只输出融合姿态，简化配置

### 5.1 `rc.board_sensors` 中的MAVLink配置

**预期配置**:
```bash
# 启动MAVLink（minimal模式，仅心跳+姿态）
mavlink start -d /dev/ttyS2 -b 115200 -m minimal

# 配置单条流：120Hz姿态四元数
mavlink stream -d /dev/ttyS2 -s ATTITUDE_QUATERNION -r 120
```

**检查项**:
- [ ] 设备: `/dev/ttyS2` (USART3)
- [ ] 波特率: 115200
- [ ] 模式: minimal（不是onboard/extvision）
- [ ] **不输出原始IMU**: 无`HIGHRES_IMU`流配置
- [ ] 只输出`ATTITUDE_QUATERNION`: 120Hz

**对比当前配置**:
```bash
# 当前可能有3条流（需要简化）
mavlink stream -u -r 120 -s ATTITUDE_QUATERNION
mavlink stream -u -r 120 -s HIGHRES_IMU        # ❌ 删除
mavlink stream -u -r 50 -s ATTITUDE            # ❌ 删除或保留（可选）
```

**修改建议**:
```bash
# 简化版（按需求2.0）
mavlink start -d /dev/ttyS2 -b 115200 -m minimal
mavlink stream -d /dev/ttyS2 -s ATTITUDE_QUATERNION -r 120
```

---

## 🔍 模块6: CMOS同步 (可选)

### 检查目标
验证EXTI中断捕获功能（低优先级，可后续开发）

### 6.1 预期文件结构

```
src/modules/cmos_sync/
├── CMakeLists.txt
├── module.yaml
├── CmosSync.hpp
├── CmosSync.cpp
└── params.c (可选)
```

### 6.2 核心功能检查

**检查项**:
- [ ] 配置PE3为EXTI3中断（帧同步）
- [ ] 配置PE4为EXTI4中断（行同步）
- [ ] 中断服务程序使用`hrt_absolute_time()`获取时间戳
- [ ] 发布`gpio_in`消息（包含gpio_id和timestamp）
- [ ] 支持上升沿/下降沿/双沿触发（可配置）

**状态**: ⏳ 待开发（优先级低）

---

## 🔍 模块7: 启动脚本

### 检查目标
确认模块启动顺序正确，无遗漏

### 7.1 `rc.board_sensors` 脚本检查

**预期内容**（按需求2.0）:
```bash
#!/bin/sh
usleep 50000

# 1. 启动传感器驱动
icm42688p start -s -b 1 -R 0 -6
icm42688p start -s -b 3 -R 8 -6
bmm150 start -I -b 1 -R 0

# 2. 启动融合算法
dual_imu_fusion start

# 3. 启动传感器预处理
sensors start

# 4. 启动LED指示
board_status_leds start

# 5. 配置MAVLink（简化版）
mavlink start -d /dev/ttyS2 -b 115200 -m minimal
mavlink stream -d /dev/ttyS2 -s ATTITUDE_QUATERNION -r 120

# 6. 传感器桩（仅开发调试）
if icm42688p status | grep -q "Not running"; then
  if bmm150 status | grep -q "Not running"; then
    sensor_stub start
  fi
fi
```

**检查项**:
- [ ] 启动顺序正确（驱动→融合→预处理→LED→MAVLink）
- [ ] 双IMU驱动都启动
- [ ] 磁力计驱动启动
- [ ] `dual_imu_fusion`在`sensors`之前启动
- [ ] `board_status_leds`在数据流之后启动
- [ ] MAVLink配置简化（只有ATTITUDE_QUATERNION）
- [ ] 传感器桩条件正确（未连硬件时启动）

---

## 📝 检查执行计划

### 阶段1: 板级配置检查（优先级🔴高）
**时间**: 15分钟
**操作**:
1. 读取5个板级配置文件
2. 对照需求.md检查每个配置项
3. 记录差异和问题

### 阶段2: 双IMU融合模块检查（优先级🔴高）
**时间**: 30-45分钟
**操作**:
1. 检查模块文件是否存在
2. 阅读核心算法代码（`Run()`函数）
3. 验证噪声估计、滤波、融合逻辑
4. 检查频率配置（8333us = 120Hz）
5. 验证uORB订阅/发布

### 阶段3: LED模块快速验证（优先级🟡中）
**时间**: 5分钟
**操作**:
1. 确认现有代码符合需求
2. 无需修改

### 阶段4: MAVLink配置检查（优先级🟢低）
**时间**: 5分钟
**操作**:
1. 检查rc.board_sensors中的MAVLink配置
2. 简化流配置（删除HIGHRES_IMU和ATTITUDE）

### 阶段5: 启动脚本检查（优先级🟡中）
**时间**: 10分钟
**操作**:
1. 检查模块启动顺序
2. 验证参数正确性

### 阶段6: CMOS同步（优先级🟢低，可延后）
**时间**: 根据需要
**操作**:
1. 确认模块是否已存在
2. 如不存在，标记为"待开发"

---

## 🐛 预期问题清单

### 问题1: `dual_imu_fusion`模块不存在
**可能性**: 高
**影响**: 🔴 严重 - 核心功能缺失
**解决**: 需要从头创建模块

### 问题2: MAVLink配置过于复杂
**可能性**: 中
**影响**: 🟡 中等 - 输出冗余数据
**解决**: 简化rc.board_sensors中的流配置

### 问题3: defconfig中启用了`CONFIG_STM32H7_TIM5=y`
**可能性**: 低（已在编译阶段修复）
**影响**: 🔴 严重 - HRT冲突
**解决**: 删除该配置行

### 问题4: `default.px4board`缺少`CONFIG_MODULES_DUAL_IMU_FUSION=y`
**可能性**: 高
**影响**: 🔴 严重 - 模块不编译
**解决**: 添加该配置

---

## ✅ 检查完成标准

### 必需项（全部通过才算完成）
- [ ] 所有板级配置文件检查无误
- [ ] `dual_imu_fusion`模块存在且逻辑正确
- [ ] 120Hz运行频率确认
- [ ] 噪声估计+滤波+融合算法实现
- [ ] uORB订阅/发布正确
- [ ] MAVLink配置简化
- [ ] 启动脚本顺序正确

### 可选项（可延后）
- [ ] CMOS同步模块实现
- [ ] 参数可调性（通过params.c）
- [ ] 欧拉角输出（vehicle_attitude可选字段）

---

## 📊 检查报告模板

**检查日期**: YYYY-MM-DD
**检查人**:
**版本**:

| 模块 | 状态 | 问题数 | 关键问题 | 备注 |
|------|------|--------|---------|------|
| 板级配置 | ⏳/✅/❌ | N | ... | ... |
| 传感器驱动 | ⏳/✅/❌ | N | ... | ... |
| 双IMU融合 | ⏳/✅/❌ | N | ... | ... |
| LED状态 | ✅ | 0 | 无 | 已完成 |
| MAVLink | ⏳/✅/❌ | N | ... | ... |
| CMOS同步 | ⏳/✅/❌ | N | ... | ... |
| 启动脚本 | ⏳/✅/❌ | N | ... | ... |

**总体结论**:
- ✅ 可以开始硬件测试
- ⏳ 需要完成XX模块后才能测试
- ❌ 存在严重问题，需要重新开发

---

**文档状态**: ✅ 完成
**下一步**: 开始执行阶段1检查（板级配置）
