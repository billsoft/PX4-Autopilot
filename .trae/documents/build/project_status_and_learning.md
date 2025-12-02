# Nucleo-H743ZI-FC 项目状态总结与知识学习

**创建时间**: 2025-12-02
**编译状态**: ✅ 编译成功通过
**当前阶段**: 准备硬件测试与功能验证

---

## 🎯 核心需求回顾（基于需求.md）

### 硬件配置
- **MCU**: STM32H743ZI (LQFP144)
- **时钟**: HSE 8MHz → SYSCLK 480MHz
- **双IMU**: ICM42688P on SPI1 (PD14 CS) + SPI3 (PA15 CS)
- **磁力计**: BMM150 on I2C1 (PB6 SCL / PB9 SDA)
- **CMOS同步**: PE3 (帧) + PE4 (行) EXTI中断
- **LED指示**: PB0 (绿/IMU1) + PB7 (黄/IMU2) + PB14 (红/Mag) - **高电平有效**
- **通信**: USART3 (PD8/PD9) MAVLink 115200

### 软件功能目标
1. **传感器数据采集**
   - SPI1/SPI3 读取双ICM42688P（兼容ICM42686/ICM45686通过`-6`参数）
   - I2C1 读取BMM150磁力计
   - EXTI捕获CMOS帧/行同步并打时间戳

2. **数据融合与输出**
   - `dual_imu_fusion`模块融合双IMU数据
   - 输出`vehicle_attitude`四元数到uORB（120Hz）
   - MAVLink流输出：
     - `ATTITUDE_QUATERNION` 120Hz
     - `HIGHRES_IMU` 120Hz
     - `ATTITUDE` 50Hz

3. **LED状态指示逻辑**
   - **启动阶段**（前3秒）：三色LED慢闪（2秒周期）表示系统存活
   - **正常运行**：
     - **无数据**：慢闪约2s周期
     - **有数据**：2Hz快闪
     - **有融合**：绿+黄同步3.3Hz闪烁
   - **窗口判断**：默认500ms，可调
   - **测试模式**：`board_status_leds test N` 循环显示

### 最小系统设计
- **无SD卡**：禁用logger
- **无飞控功能**：禁用commander/navigator/控制器
- **无PWM输出**：不需要电机驱动
- **最小模块集**：sensors + dual_imu_fusion + mavlink + board_status_leds

---

## 📊 当前实现状态

### ✅ 已完成的核心功能

#### 1. 硬件初始化与时钟配置
- **PLL配置**（`board.h`）:
  - PLL1: SYSCLK 480MHz
  - PLL2P: 192MHz (SPI123时钟源，满足≤200MHz要求)
  - USART3 PCLK: 96MHz (支持115200波特率)

- **GPIO配置**（`board_config.h`）:
  ```c
  // LED - 高电平有效（已修正）
  GPIO_nLED_GREEN:  PB0  (OUTPUT_CLEAR初始关闭)
  GPIO_nLED_YELLOW: PB7  (OUTPUT_CLEAR初始关闭)
  GPIO_nLED_RED:    PB14 (OUTPUT_CLEAR初始关闭)

  // SPI CS引脚
  SPI1_CS: PD14 (IMU1)
  SPI3_CS: PA15 (IMU2)

  // CMOS同步GPIO
  CMOS_SYNC_LINE:  PE3 (行同步)
  CMOS_SYNC_FRAME: PE4 (帧同步)
  ```

- **HRT配置**（解决了关键的时间服务问题）:
  ```c
  HRT_TIMER = 5        // 使用TIM5（32位定时器）
  HRT_TIMER_CHANNEL = 1
  ```
  - defconfig中**不启用**`CONFIG_STM32H7_TIM5=y`（避免与PX4 HRT冲突）
  - 启用`CONFIG_TIMER=y`和`CONFIG_ONESHOT=y`框架支持

#### 2. NuttX配置优化（`defconfig`）
- **禁用DMA**（初期调试）:
  ```kconfig
  CONFIG_SPI_DMA=n
  CONFIG_USART3_RXDMA=n
  CONFIG_USART3_TXDMA=n
  ```
  **原因**: DMA在早期可能引起问题，先确保轮询模式稳定

- **I2C晚期初始化**:
  ```kconfig
  CONFIG_BOARD_I2C_LATEINIT=y
  CONFIG_I2C_RESET=n
  ```
  **原因**: 早期I2C初始化易触发HardFault

- **电源管理**:
  ```kconfig
  CONFIG_PM=y
  CONFIG_STM32H7_PWR=y
  ```
  **原因**: 支持`up_restoreusartint`等串口函数

- **调试支持**:
  ```kconfig
  CONFIG_DEBUG_HARDFAULT_ALERT=y
  CONFIG_DEBUG_MEMFAULT=y
  CONFIG_DEBUG_TCBINFO=y
  ```

#### 3. 异步初始化架构（`init.cpp`）
这是**关键创新**，解决了PX4初始化阻塞NSH的问题：

```cpp
static int px4_init_thread(int argc, char *argv[])
{
    syslog(LOG_INFO, "[InitThread] Starting px4_platform_init in 10s...\n");
    usleep(10000000);  // 延迟10秒，确保NSH完全启动

    int ret = px4_platform_init();      // 挂载ROMFS、启动uORB等
    int conf = px4_platform_configure();

    // 启动LED状态指示模块
    const char *argv_leds[] = {"board_status_leds", "start"};
    board_status_leds_main(2, (char **)argv_leds);

    return ret;
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
    // 1. 配置LED GPIO（初始全关闭）
    // 2. 配置SPI CS引脚（高电平去选）
    // 3. 启动LED闪烁3次（表示板级初始化成功）
    // 4. 初始化SPI
    // 5. 初始化DMA
    // 6. **创建异步任务**启动PX4平台初始化

    int taskid = task_create("px4_init", 100, 4096, px4_init_thread, NULL);
    return OK;  // 立即返回，不阻塞NSH
}
```

**优势**:
- NSH立即可用，方便调试
- `syslog`输出可见（不被PX4日志淹没）
- 避免初始化竞态条件
- 符合PX4开发提示："先保证NSH与平台初始化稳定"

#### 4. 模块配置（`default.px4board`）

**启用的核心模块**:
```python
CONFIG_MODULES_SENSORS=y              # 传感器预处理
CONFIG_MODULES_MAVLINK=y              # MAVLink通信
CONFIG_MODULES_CMOS_SYNC=y            # CMOS同步（自定义）
CONFIG_MODULES_DUAL_IMU_FUSION=y      # 双IMU融合（自定义）
CONFIG_MODULES_BOARD_STATUS_LEDS=y    # LED状态指示（自定义）
CONFIG_MODULES_SENSOR_STUB=y          # 传感器桩（开发用）
CONFIG_MODULES_DATAMAN=y              # 必需（提供CONFIG_NUM_MISSION_ITMES_SUPPORTED）
```

**禁用的模块**:
```python
CONFIG_MODULES_LOGGER=n               # 无SD卡
CONFIG_MODULES_COMMANDER=n            # 避免geofence等参数依赖
CONFIG_MODULES_EKF2=n                 # 用自定义融合替代
CONFIG_MODULES_NAVIGATOR=n            # 无GPS
CONFIG_MODULES_MC_POS_CONTROL=n       # 无飞控
```

**系统命令**:
```python
CONFIG_SYSTEMCMDS_I2CDETECT=y         # I2C设备扫描
CONFIG_SYSTEMCMDS_LED_CONTROL=y       # LED控制
CONFIG_SYSTEMCMDS_TOPIC_LISTENER=y    # 话题监听器
CONFIG_SYSTEMCMDS_PARAM=y             # 参数系统
CONFIG_SYSTEMCMDS_PERF=y              # 性能计数器
CONFIG_SYSTEMCMDS_UORB=y              # uORB工具
CONFIG_SYSTEMCMDS_VER=y               # 版本信息
```

#### 5. LED状态指示模块（`board_status_leds`）

**核心逻辑**（100ms周期 = 10Hz）:
```cpp
class BoardStatusLEDs : public px4::ScheduledWorkItem {
    uORB::Subscription _accel0{ORB_ID(sensor_accel), 0};  // SPI1 IMU
    uORB::Subscription _accel1{ORB_ID(sensor_accel), 1};  // SPI3 IMU
    uORB::Subscription _mag{ORB_ID(sensor_mag), 0};       // I2C1 Mag
    uORB::Subscription _att{ORB_ID(vehicle_attitude), 0}; // 融合输出

    uint64_t _window_us = 500000;  // 500ms窗口判断

    void Run() override {
        // 检查数据新鲜度
        bool imu1 = (now - _t_accel0) < _window_us;
        bool imu2 = (now - _t_accel1) < _window_us;
        bool mag  = (now - _t_mag) < _window_us;
        bool fusion = (now - _t_att) < _window_us;

        // 闪烁模式
        bool blink_slow = 2秒周期（20个tick）
        bool blink_fast2 = 2Hz（5个tick周期）
        bool blink_fusion = 3.3Hz（3个tick周期）

        // LED1 (绿 - IMU1)
        if (fusion) {
            绿灯3.3Hz闪烁
        } else if (imu1) {
            绿灯2Hz快闪
        } else {
            绿灯2s慢闪
        }

        // LED2 (黄 - IMU2) 同理
        // LED3 (红 - Mag) 独立指示，无融合状态
    }
};
```

**测试命令**:
```bash
board_status_leds status     # 查看状态
board_status_leds test 10    # 测试模式10秒（循环显示绿→黄→红）
```

#### 6. 启动脚本（`rc.board_sensors`）

```bash
# 启动ICM42688P双IMU（-6表示兼容42686/45686）
icm42688p start -s -b 1 -R 0 -6   # SPI1, 旋转0度
icm42688p start -s -b 3 -R 8 -6   # SPI3, 旋转270度(8)

# 启动BMM150磁力计
bmm150 start -I -b 1 -R 0          # I2C1, 旋转0度

# 启动自定义模块
cmos_sync start                    # CMOS同步
dual_imu_fusion start              # 双IMU融合
sensors start                      # 传感器预处理
board_status_leds start            # LED状态指示

# MAVLink流配置
mavlink stream -u -r 120 -s ATTITUDE_QUATERNION
mavlink stream -u -r 120 -s HIGHRES_IMU
mavlink stream -u -r 50 -s ATTITUDE

# 传感器桩（开发用）
if icm42688p status | grep -q "Not running"; then
  if bmm150 status | grep -q "Not running"; then
    sensor_stub start   # 未连传感器时启动模拟数据
  fi
fi
```

---

## 🔧 关键技术点学习

### 1. STM32H7与F4/F7的差异

**时钟寄存器命名差异**（`micro_hal.h`已修复）:
```c
// STM32F4/F7
STM32_RCC_APB1ENR      // 直接使用

// STM32H7（双总线架构）
STM32_RCC_APB1LENR     // APB1 Low
STM32_RCC_APB1HENR     // APB1 High

// PX4兼容层
#define STM32_RCC_APB1ENR  STM32_RCC_APB1LENR  // 映射到Low总线
```

**原因**: STM32H7为了高性能，APB1/APB2都分为L/H两条总线。

### 2. PX4 HRT架构深度理解

**HRT不使用NuttX定时器驱动**:
```
错误理解 ❌:
  defconfig中启用 CONFIG_STM32H7_TIM5=y
  → NuttX提供标准POSIX定时器接口
  → PX4使用这个接口

正确理解 ✅:
  board_config.h中定义 HRT_TIMER=5
  → PX4 HRT直接操作TIM5硬件寄存器
  → 不经过NuttX驱动层
  → 提供微秒级高精度时间服务
```

**冲突检测机制**（`hrt.c`）:
```c
#if CONFIG_STM32_TIM5
#  error must not set CONFIG_STM32_TIM5=y and HRT_TIMER=5
#endif
```

**为什么这样设计**:
- PX4飞控对时间精度要求极高（1μs分辨率）
- NuttX标准定时器接口有额外开销
- 直接操作寄存器性能最优

### 3. 异步初始化的必要性

**问题场景**:
```
同步初始化流程:
  NuttX启动 → board_app_initialize()
  → px4_platform_init()（挂载ROMFS、启动uORB等，耗时较长）
  → 初始化完成才返回
  → NSH才能使用

问题:
  1. 初始化阻塞NSH，无法调试
  2. syslog输出可能被PX4日志覆盖
  3. 初始化失败无法进入NSH排查
```

**异步方案优势**:
```
异步初始化流程:
  NuttX启动 → board_app_initialize()
  → 创建task_create("px4_init", ...)
  → 立即返回OK
  → NSH立即可用

  同时:
  px4_init线程延迟10秒启动
  → 确保NSH完全启动
  → syslog日志清晰可见
  → 初始化过程可监控
```

**实现技巧**:
```cpp
// 使用syslog而非PX4_INFO（确保早期输出可见）
syslog(LOG_INFO, "[InitThread] Starting...\n");

// 延迟启动（给NSH缓冲时间）
usleep(10000000);  // 10秒

// 日志输出每个关键步骤
syslog(LOG_INFO, "[InitThread] px4_platform_init returned: %d\n", ret);
```

### 4. I2C早期初始化陷阱

**问题**:
```
早期I2C初始化 → 硬件未完全就绪
→ I2C总线操作 → HardFault崩溃
```

**解决方案**（已配置）:
```kconfig
CONFIG_BOARD_I2C_LATEINIT=y   # I2C延迟初始化
CONFIG_I2C_RESET=n            # 禁用I2C复位（可能触发问题）
```

**机制**:
- NuttX不在启动时初始化I2C硬件
- 由驱动首次使用时初始化
- 避免早期硬件不稳定期操作

### 5. DMA配置策略

**当前策略**（禁用DMA）:
```kconfig
CONFIG_SPI_DMA=n
CONFIG_USART3_RXDMA=n
CONFIG_USART3_TXDMA=n
```

**原因**:
1. **调试优先**: 轮询模式更稳定，问题易排查
2. **串口可见**: DMA可能丢失早期启动日志
3. **最小复杂度**: 先确保基本功能

**后期优化方向**:
```kconfig
# 系统稳定后，逐步启用DMA提升性能
CONFIG_SPI_DMA=y           # SPI DMA（高速传感器读取）
CONFIG_USART3_RXDMA=y      # MAVLink接收DMA
```

---

## 🎓 从文档中学到的关键经验

### 从 `hrt_fix_complete_guide.md`

**核心教训**:
> "参考板级配置时，不仅要看存在什么，更要看**不存在**什么！"

- fmu-v6x定义了`HRT_TIMER=8`
- 但defconfig中**没有**`CONFIG_STM32H7_TIM8=y`
- 这说明两者是互斥的，而非协同的

### 从 `linker_errors_fix_guide.md`

**编译错误定位模式**:
- [0-300]: PX4模块编译 → 检查.px4board和头文件
- [300-530]: NuttX内核编译 → 检查defconfig和GPIO定义
- [530-536]: 链接阶段 → 检查缺失的.c文件和宏定义

### 从 `stm32_custom_board_bringup_tutorial.md`

**硬件调试顺序**:
1. LED闪烁（最简单，验证GPIO和时钟）
2. 串口输出（验证USART和时钟配置）
3. SPI通信（验证SPI时钟和CS引脚）
4. I2C通信（验证I2C总线和设备地址）
5. 中断服务（验证EXTI和中断优先级）

---

## 📋 当前待验证功能清单

### 硬件验证步骤（按需求.md）

#### 1. NSH与平台初始化
```bash
# 连接串口（USART3, 115200）
# 观察启动日志
help                    # NSH可用性
dmesg                   # 查看系统消息
uorb status             # uORB是否启动
uorb top -1             # 查看话题列表
```

**预期输出**:
```
[Init] Started px4_init task (id=...)
[InitThread] Starting px4_platform_init in 10s...
[InitThread] px4_platform_init returned: 0
[InitThread] px4_platform_configure returned: 0
[InitThread] board_status_leds started
```

#### 2. 传感器检测
```bash
# SPI1 IMU1
icm42688p status
# 预期: Running, Device ID: 0x47 (ICM42688P)

# SPI3 IMU2
icm42688p status
# 预期: Running, Device ID: 0x47

# I2C1 Mag
bmm150 status
# 预期: Running, Device ID: 0x32

# I2C总线扫描
i2cdetect
# 预期: 发现BMM150地址（0x10或0x11）
```

#### 3. LED状态指示
```bash
board_status_leds status
# 预期输出:
#   IMU1: <data/no_data>
#   IMU2: <data/no_data>
#   Mag:  <data/no_data>
#   Fusion: <active/inactive>

board_status_leds test 5
# 观察LED循环显示：绿→黄→红
```

#### 4. 数据流验证
```bash
listener sensor_accel
# 预期: 显示IMU1和IMU2的加速度数据（120Hz）

listener sensor_mag
# 预期: 显示磁力计数据

listener vehicle_attitude
# 预期: 显示融合后的四元数姿态（120Hz）
```

#### 5. MAVLink输出
```bash
mavlink status
# 检查USART3连接状态

# 使用QGroundControl或mavlink_shell.py监听
# 预期消息:
#   - HEARTBEAT
#   - ATTITUDE_QUATERNION (120Hz)
#   - HIGHRES_IMU (120Hz)
#   - ATTITUDE (50Hz)
```

#### 6. CMOS同步测试（硬件连接后）
```bash
# 连接CMOS传感器的帧/行同步信号到PE3/PE4
listener gpio_in
# 预期: 显示EXTI中断时间戳
```

---

## ⚠️ 已知潜在问题与应对

### 问题1: 传感器未连接时系统稳定性
**现象**: 驱动启动失败，可能影响后续模块

**应对**（已实现）:
```bash
# rc.board_sensors中的条件启动
if icm42688p status | grep -q "Not running"; then
  if bmm150 status | grep -q "Not running"; then
    sensor_stub start   # 启动模拟数据
  fi
fi
```

### 问题2: I2C设备地址不确定
**BMM150可能地址**: 0x10 或 0x11（取决于SDO引脚）

**验证方法**:
```bash
i2cdetect
# 扫描I2C1总线，查看实际地址
```

### 问题3: SPI通信速率
**当前配置**: SPI时钟≤192MHz（PLL2P）

**ICM42688P支持**: 最高24MHz SPI时钟

**验证**: 检查驱动中的SPI频率配置是否合理

### 问题4: LED极性
**已修正**: 从"低有效"改为"高有效"
```c
// board_config.h
GPIO_OUTPUT_CLEAR    // 初始低电平（LED关闭）
BOARD_LED_ON = 1     // 高电平点亮
BOARD_LED_OFF = 0    // 低电平熄灭
```

---

## 🚀 下一步行动计划

### 立即行动
1. **烧录固件**:
   ```bash
   python Tools/flash/flash_fw.py
   ```

2. **连接串口**，观察启动日志

3. **进入NSH**，执行验证命令

### 调试策略
1. **不连传感器**: 验证基本系统（NSH、uORB、MAVLink、LED测试模式）
2. **连单个传感器**: 逐个验证SPI1、SPI3、I2C1
3. **全部连接**: 验证双IMU融合和完整LED逻辑
4. **CMOS同步**: 最后验证EXTI中断

### 性能优化（系统稳定后）
1. 启用SPI/UART DMA
2. 调整任务优先级
3. 优化融合算法参数
4. 测量实际输出频率

---

## 📖 知识总结

### PX4板级开发核心要点
1. **两层架构**: NuttX提供RTOS，PX4实现飞控逻辑
2. **时间精度**: HRT直接控制硬件，不经过NuttX驱动
3. **异步初始化**: 避免阻塞NSH，便于调试
4. **最小配置**: 禁用不需要的功能，降低复杂度
5. **参考选择**: 同MCU系列板级（fmu-v6x）比不同系列更可靠

### STM32H7特殊性
1. **双总线**: APB1/APB2分为L/H总线，寄存器命名不同
2. **时钟树**: 更复杂的PLL配置（PLL1/PLL2/PLL3）
3. **电源域**: 多个电压域，初始化顺序重要
4. **I2C晚期初始化**: 避免早期硬件不稳定

### LED指示设计哲学
- **渐进式反馈**: 启动→无数据→有数据→融合，状态清晰
- **独立指示**: 每个传感器独立LED，问题定位快
- **视觉区分**: 慢闪(2s)、快闪(2Hz)、融合(3.3Hz)频率差异明显
- **测试模式**: 循环显示，硬件功能验证

---

**文档状态**: ✅ 完成
**编译状态**: ✅ 成功
**下一步**: 硬件测试与功能验证
