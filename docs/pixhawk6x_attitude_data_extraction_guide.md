# Pixhawk 6X 姿态数据提取技术方案

## 目录
- [概述](#概述)
- [姿态数据源分析](#姿态数据源分析)
- [硬件接口分析](#硬件接口分析)
- [实施方案](#实施方案)
- [代码示例](#代码示例)
- [时间戳对齐](#时间戳对齐)
- [性能优化](#性能优化)
- [故障排查](#故障排查)

---

## 概述

### 需求说明
从Pixhawk 6X飞控板获取融合后的姿态数据（包括四元数和欧拉角），并传输到外部开发板进行进一步处理。

### 关键数据
- **数据类型**：四元数 (q[4])、角速度 (xyz[3])
- **数据来源**：EKF2 模块融合 IMU + GPS + 磁力计 + 气压计
- **更新频率**：100-250Hz（取决于IMU采样率）
- **数据精度**：float32

### 目标开发板接口优先级
1. **首选**：UART/串口（最成熟可靠）
2. **次选**：Ethernet（最快，适合高带宽）
3. **不推荐**：I2C/SPI（需要自行开发slave模式，PX4未提供原生支持）

---

## 姿态数据源分析

### 1. uORB Topic: `vehicle_attitude`

#### 消息定义
位置：`msg/versioned/VehicleAttitude.msg`

```c
uint64 timestamp                # 系统启动后时间 (微秒)
uint64 timestamp_sample         # 原始数据时间戳 (微秒)
float32[4] q                    # 四元数 (Hamilton约定, w,x,y,z顺序)
                                # 从FRD机体坐标系到NED地理坐标系的旋转
float32[4] delta_q_reset        # 四元数重置时的变化量
uint8 quat_reset_counter        # 四元数重置计数器
```

#### 数据流向
```
IMU原始数据 → EKF2模块 → vehicle_attitude (uORB) → MAVLink/UXRCE-DDS → 外部设备
         ↑
    GPS/磁力计/气压计
```

### 2. EKF2 发布频率

**代码位置**：`src/modules/ekf2/EKF2.cpp:761`

```cpp
// 每次IMU数据更新时立即发布姿态（使用输出预测器降低延时）
void EKF2::Run() {
    // ...
    _ekf.setIMUData(imu_sample_new);
    PublishAttitude(now);  // 立即发布
    // ...
}
```

**发布频率**：
- **Pixhawk 6X 默认IMU频率**：1000Hz（ICM42688P）
- **EKF2处理频率**：通常降采样到200-250Hz
- **vehicle_attitude 发布频率**：约 200-250Hz

### 3. 姿态数据内容

**代码位置**：`src/modules/ekf2/EKF2.cpp:1046-1070`

```cpp
void EKF2::PublishAttitude(const hrt_abstime &timestamp)
{
    if (_ekf.attitude_valid()) {
        vehicle_attitude_s att;
        att.timestamp_sample = timestamp;
        const Quatf q{_ekf.getQuaternion()};  // 获取融合后的四元数
        q.copyTo(att.q);
        // 发布到 uORB
        _attitude_pub.publish(att);
    }
}
```

**四元数到欧拉角转换**（如需要）：
```cpp
#include <matrix/math.hpp>
using matrix::Eulerf;
using matrix::Quatf;

Quatf q(att.q[0], att.q[1], att.q[2], att.q[3]);
Eulerf euler(q);  // 自动转换
float roll = euler.phi();    // 滚转角 (弧度)
float pitch = euler.theta(); // 俯仰角 (弧度)
float yaw = euler.psi();     // 偏航角 (弧度)
```

---

## 硬件接口分析

### Pixhawk 6X 可用接口总览

根据板子配置文件 `boards/px4/fmu-v6x/default.px4board` 和官方文档：

| 接口类型 | 数量 | 设备文件 | 适用性 | 推荐度 |
|---------|------|---------|--------|--------|
| **UART** | 6个 | /dev/ttyS0-S7 | ⭐⭐⭐⭐⭐ | 首选 |
| **Ethernet** | 1个 | eth0 | ⭐⭐⭐⭐ | 次选（高速） |
| **I2C** | 1个外部 | /dev/i2c-X | ⭐ | 不推荐 |
| **SPI** | 1个外部 | SPI5 | ⭐ | 不推荐 |
| **CAN** | 2个 | can0, can1 | ⭐⭐ | 不适合高频 |
| **USB** | 1个 | /dev/ttyACM0 | ⭐⭐⭐ | 调试用 |

### 接口详细说明

#### 1. UART 接口（推荐）

**板子配置**：
```
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"   # GPS1端口
CONFIG_BOARD_SERIAL_GPS2="/dev/ttyS7"   # GPS2端口
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS6"   # TELEM1端口 ⭐推荐用于姿态数据
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS4"   # TELEM2端口 ⭐推荐用于姿态数据
CONFIG_BOARD_SERIAL_TEL3="/dev/ttyS1"   # TELEM3端口
CONFIG_BOARD_SERIAL_EXT2="/dev/ttyS3"   # EXT2端口
CONFIG_BOARD_SERIAL_RC="/dev/ttyS5"     # RC输入端口
```

**推荐端口**：
- **TELEM1** (`/dev/ttyS6`)：默认57600波特率，可配置为921600
- **TELEM2** (`/dev/ttyS4`)：默认57600波特率，可配置为921600

**物理连接**：
- TELEM1/2端口为6针JST-GH接口
- 针脚定义：1-VCC, 2-TX, 3-RX, 4-CTS, 5-RTS, 6-GND

#### 2. Ethernet 接口（高速场景）

**硬件配置**：
```c
// boards/px4/fmu-v6x/src/board_config.h:280
#define GPIO_ETH_POWER_EN  /* PG15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|...)
CONFIG_BOARD_ETHERNET=y
```

**特点**：
- 100Mbps带宽
- 支持MAVLink over UDP
- 支持UXRCE-DDS（ROS2）
- 延迟极低（< 1ms）

#### 3. I2C 接口（不推荐用于姿态输出）

**问题**：
- PX4仅支持I2C **Master模式**（读取传感器）
- 无原生I2C Slave模式支持
- 需要自行开发底层驱动

**硬件资源**：
```c
// I2C总线定义
#define PX4_I2C_BUS_MTD  4,5  // 内部EEPROM总线
// 外部I2C端口可用，但需实现slave模式
```

#### 4. SPI 接口（不推荐用于姿态输出）

**问题**：
- PX4仅支持SPI **Master模式**
- 外部SPI5有2个片选线，但没有slave模式实现

**硬件资源**：
```c
// boards/px4/fmu-v6x/src/board_config.h:127
#define SPI6_nRESET_EXTERNAL1  /* PF10 */ (GPIO_OUTPUT|...)
// External SPI bus (SPI5) with 2 chip select lines
```

---

## 实施方案

### 方案对比

| 方案 | 优势 | 劣势 | 复杂度 | 延迟 | 带宽 |
|------|------|------|--------|------|------|
| **UART + MAVLink** | 成熟稳定，文档完善 | 波特率限制 | ⭐ 低 | ~5ms | 适中 |
| **Ethernet + MAVLink** | 高速低延迟 | 需要以太网模块 | ⭐⭐ 中 | ~1ms | 高 |
| **Ethernet + UXRCE-DDS** | 原生ROS2支持 | 配置复杂 | ⭐⭐⭐ 中高 | ~1ms | 高 |
| **自定义I2C Slave** | 硬件简单 | 需开发驱动 | ⭐⭐⭐⭐⭐ 极高 | 未知 | 低 |
| **自定义SPI Slave** | 速度快 | 需开发驱动 | ⭐⭐⭐⭐⭐ 极高 | 未知 | 中 |

### 推荐方案：UART + MAVLink

---

## 方案一：UART + MAVLink（推荐）

### 优势
✅ PX4原生支持，无需额外开发
✅ 文档和工具链完善
✅ 支持多种波特率（最高921600）
✅ 外部开发板库支持广泛（MAVSDK, MAVLink C库等）
✅ 可同时传输多种数据（姿态、位置、速度等）

### 配置步骤

#### 步骤1：配置MAVLink实例

**在PX4控制台或启动脚本中配置**：

```bash
# 方法1：直接在PX4控制台输入
mavlink start -d /dev/ttyS6 -b 921600 -m onboard -r 80000

# 参数说明：
# -d /dev/ttyS6    : 使用TELEM1端口
# -b 921600        : 波特率921600
# -m onboard       : onboard模式（高频率）
# -r 80000         : 最大数据速率 80KB/s
```

**或修改启动脚本**（永久生效）：

位置：`ROMFS/px4fmu_common/init.d/rcS` 或创建自定义脚本

```bash
# 在 rc.local 或 extras.txt 中添加
mavlink start -d /dev/ttyS6 -b 921600 -m onboard
mavlink stream -d /dev/ttyS6 -s ATTITUDE_QUATERNION -r 200
mavlink stream -d /dev/ttyS6 -s ATTITUDE -r 200
```

#### 步骤2：设置消息流速率

```bash
# ATTITUDE_QUATERNION: 包含四元数 + 角速度
mavlink stream -d /dev/ttyS6 -s ATTITUDE_QUATERNION -r 200

# 可选：同时启用ATTITUDE（欧拉角形式）
mavlink stream -d /dev/ttyS6 -s ATTITUDE -r 200

# 验证配置
mavlink status
```

**MAVLink消息内容**（`ATTITUDE_QUATERNION`, ID #31）：

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| time_boot_ms | uint32 | ms | 系统启动时间 |
| q1, q2, q3, q4 | float | - | 四元数 (w,x,y,z) |
| rollspeed | float | rad/s | 滚转角速度 |
| pitchspeed | float | rad/s | 俯仰角速度 |
| yawspeed | float | rad/s | 偏航角速度 |
| repr_offset_q | float[4] | - | 显示偏移四元数 |

**代码位置**：`src/modules/mavlink/streams/ATTITUDE_QUATERNION.hpp:64-109`

```cpp
bool send() override {
    vehicle_attitude_s att;
    if (_att_sub.update(&att)) {
        mavlink_attitude_quaternion_t msg{};
        msg.time_boot_ms = att.timestamp / 1000;
        msg.q1 = att.q[0];  // w
        msg.q2 = att.q[1];  // x
        msg.q3 = att.q[2];  // y
        msg.q4 = att.q[3];  // z
        // ...
        mavlink_msg_attitude_quaternion_send_struct(_mavlink->get_channel(), &msg);
    }
}
```

#### 步骤3：硬件连接

**Pixhawk 6X TELEM1 端口 → 外部开发板 UART**

```
Pixhawk 6X TELEM1       外部开发板
┌─────────────┐         ┌──────────┐
│ 1. VCC(5V)  │────────>│ 5V (可选)|
│ 2. TX       │────────>│ RX       │
│ 3. RX       │<────────│ TX       │
│ 4. CTS      │<────────│ RTS(可选)|
│ 5. RTS      │────────>│ CTS(可选)|
│ 6. GND      │────────>│ GND      │
└─────────────┘         └──────────┘
```

**注意事项**：
- 电平匹配：Pixhawk为3.3V UART，确保外部板兼容
- 硬件流控：如果波特率≥115200，建议连接CTS/RTS
- 供电：外部板最好独立供电，避免从TELEM端口取电

---

### 外部开发板接收代码

#### Arduino/STM32示例（使用MAVLink C库）

**1. 安装MAVLink库**

```bash
# Arduino：在库管理器中搜索 "MAVLink"
# PlatformIO：platformio.ini中添加
lib_deps =
    https://github.com/mavlink/c_library_v2.git
```

**2. 接收代码**

```cpp
#include <mavlink.h>

// UART配置
HardwareSerial &px4Serial = Serial1;  // 使用Serial1
const uint32_t baudrate = 921600;

// 姿态数据存储
struct AttitudeData {
    uint32_t timestamp_ms;
    float q[4];           // 四元数 w,x,y,z
    float rollspeed;      // rad/s
    float pitchspeed;
    float yawspeed;

    // 转换为欧拉角（deg）
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} attitude;

void setup() {
    Serial.begin(115200);   // 调试串口
    px4Serial.begin(baudrate);  // PX4连接

    Serial.println("MAVLink Attitude Receiver Started");
}

void loop() {
    // 接收MAVLink消息
    while (px4Serial.available()) {
        uint8_t byte = px4Serial.read();
        mavlink_message_t msg;
        mavlink_status_t status;

        // 解析MAVLink字节流
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            handleMAVLinkMessage(&msg);
        }
    }
}

void handleMAVLinkMessage(mavlink_message_t* msg) {
    switch(msg->msgid) {
        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:  // ID 31
        {
            mavlink_attitude_quaternion_t att;
            mavlink_msg_attitude_quaternion_decode(msg, &att);

            // 存储数据
            attitude.timestamp_ms = att.time_boot_ms;
            attitude.q[0] = att.q1;  // w
            attitude.q[1] = att.q2;  // x
            attitude.q[2] = att.q3;  // y
            attitude.q[3] = att.q4;  // z
            attitude.rollspeed = att.rollspeed;
            attitude.pitchspeed = att.pitchspeed;
            attitude.yawspeed = att.yawspeed;

            // 四元数转欧拉角
            quaternionToEuler(attitude.q,
                              &attitude.roll_deg,
                              &attitude.pitch_deg,
                              &attitude.yaw_deg);

            // 打印（可选）
            printAttitude();
            break;
        }
    }
}

// 四元数转欧拉角（NED坐标系）
void quaternionToEuler(float q[4], float* roll, float* pitch, float* yaw) {
    float w = q[0], x = q[1], y = q[2], z = q[3];

    // Roll (φ)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    *roll = atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

    // Pitch (θ)
    float sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        *pitch = copysign(M_PI / 2, sinp) * 180.0 / M_PI;
    else
        *pitch = asin(sinp) * 180.0 / M_PI;

    // Yaw (ψ)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    *yaw = atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

void printAttitude() {
    Serial.print("Time: "); Serial.print(attitude.timestamp_ms);
    Serial.print(" | Q: [");
    Serial.print(attitude.q[0], 4); Serial.print(", ");
    Serial.print(attitude.q[1], 4); Serial.print(", ");
    Serial.print(attitude.q[2], 4); Serial.print(", ");
    Serial.print(attitude.q[3], 4); Serial.print("]");
    Serial.print(" | Euler: R="); Serial.print(attitude.roll_deg, 2);
    Serial.print(" P="); Serial.print(attitude.pitch_deg, 2);
    Serial.print(" Y="); Serial.println(attitude.yaw_deg, 2);
}
```

#### Python示例（使用pymavlink）

```python
from pymavlink import mavutil
import time

# 连接到Pixhawk
connection = mavutil.mavlink_connection(
    '/dev/ttyUSB0',  # Linux
    # 'COM3',        # Windows
    baud=921600
)

print("等待心跳包...")
connection.wait_heartbeat()
print(f"已连接到系统 {connection.target_system}:{connection.target_component}")

# 接收循环
while True:
    msg = connection.recv_match(type='ATTITUDE_QUATERNION', blocking=True, timeout=1)

    if msg:
        # 提取数据
        timestamp = msg.time_boot_ms
        q = [msg.q1, msg.q2, msg.q3, msg.q4]  # w,x,y,z
        rates = [msg.rollspeed, msg.pitchspeed, msg.yawspeed]

        # 四元数转欧拉角
        import math
        w, x, y, z = q

        # Roll
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        # Pitch
        pitch = math.asin(2*(w*y - z*x))
        # Yaw
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        # 转换为度
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        print(f"[{timestamp}ms] Euler: R={roll_deg:.2f}° P={pitch_deg:.2f}° Y={yaw_deg:.2f}° | "
              f"Rates: {rates[0]:.3f} {rates[1]:.3f} {rates[2]:.3f} rad/s")
```

---

## 方案二：Ethernet + MAVLink（高性能场景）

### 适用场景
- 需要极低延迟（< 1ms）
- 需要高频率数据（500Hz+）
- 外部开发板支持以太网（如树莓派、Jetson等）

### 配置步骤

#### 步骤1：配置Pixhawk以太网

**修改启动脚本** `ROMFS/px4fmu_common/init.d/rcS`：

```bash
# 启动以太网接口
if param compare SYS_ETHERNET 1
then
    # 配置静态IP（或使用DHCP）
    ifconfig eth0 192.168.1.10 netmask 255.255.255.0

    # 启动MAVLink over UDP
    mavlink start -d eth0 -m onboard -r 4000000 -u 14550
    mavlink stream -d eth0 -s ATTITUDE_QUATERNION -r 500
fi
```

#### 步骤2：外部设备连接

**硬件连接**：
```
Pixhawk 6X Ethernet端口 ←→ 以太网交换机/路由器 ←→ 外部开发板
或直连：
Pixhawk 6X (192.168.1.10) ←→ 交叉网线 ←→ 开发板 (192.168.1.20)
```

**Python接收代码**：

```python
from pymavlink import mavutil

# 通过UDP连接
connection = mavutil.mavlink_connection('udp:192.168.1.10:14550')

connection.wait_heartbeat()
print("已连接到Pixhawk via Ethernet")

while True:
    msg = connection.recv_match(type='ATTITUDE_QUATERNION', blocking=True)
    # ... 处理数据（同上）
```

---

## 方案三：Ethernet + UXRCE-DDS（ROS2集成）

### 适用场景
- 使用ROS2进行机器人开发
- 需要订阅多个PX4话题
- 需要双向通信（控制 + 数据接收）

### 优势
✅ 原生ROS2 DDS通信
✅ 自动类型转换
✅ 支持QoS策略
✅ 可订阅所有uORB topics

### 配置步骤

#### 步骤1：启用UXRCE-DDS客户端

**在Pixhawk中配置**（已在default.px4board中启用）：

```bash
# 启动UXRCE-DDS客户端（通过以太网）
uxrce_dds_client start -t udp -h 192.168.1.20 -p 8888
```

#### 步骤2：在外部ROS2设备上运行Agent

**安装Micro-XRCE-DDS Agent**：

```bash
# Ubuntu/Debian
sudo apt install ros-humble-micro-xrce-dds-agent

# 或从源码安装
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install
```

**运行Agent**：

```bash
MicroXRCEAgent udp4 -p 8888
```

#### 步骤3：订阅姿态数据

**ROS2 Python节点**：

```python
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitude
import math

class AttitudeSubscriber(Node):
    def __init__(self):
        super().__init__('attitude_subscriber')

        # 订阅 vehicle_attitude topic
        self.subscription = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            10)

    def attitude_callback(self, msg):
        # 提取四元数
        q = msg.q  # [w, x, y, z]

        # 转换为欧拉角
        roll, pitch, yaw = self.quaternion_to_euler(q)

        self.get_logger().info(
            f'Timestamp: {msg.timestamp} | '
            f'Euler: R={math.degrees(roll):.2f}° '
            f'P={math.degrees(pitch):.2f}° '
            f'Y={math.degrees(yaw):.2f}°'
        )

    def quaternion_to_euler(self, q):
        w, x, y, z = q[0], q[1], q[2], q[3]

        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        return roll, pitch, yaw

def main():
    rclpy.init()
    node = AttitudeSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 时间戳对齐

### PX4时间戳格式

**uORB消息时间戳**：
```c
uint64 timestamp        // 系统启动后微秒数 (μs)
uint64 timestamp_sample // 传感器采样时间戳 (μs)
```

**MAVLink时间戳**：
```c
uint32 time_boot_ms  // 系统启动后毫秒数 (ms)
```

### 时间同步策略

#### 方法1：相对时间差（推荐）

```cpp
// 外部开发板代码
uint32_t first_px4_timestamp = 0;
uint32_t first_local_timestamp = 0;
bool time_synced = false;

void handleAttitude(mavlink_attitude_quaternion_t* att) {
    // 第一次接收时建立时间基准
    if (!time_synced) {
        first_px4_timestamp = att->time_boot_ms;
        first_local_timestamp = millis();  // Arduino的本地时间
        time_synced = true;
    }

    // 计算对齐的本地时间戳
    uint32_t px4_elapsed = att->time_boot_ms - first_px4_timestamp;
    uint32_t local_timestamp = first_local_timestamp + px4_elapsed;

    // 使用local_timestamp与其他传感器数据对齐
}
```

#### 方法2：NTP时间同步（以太网场景）

```bash
# 在Pixhawk上启用NTP客户端
param set SYS_NTP_SERVER "192.168.1.1"
# 外部设备同步到同一NTP服务器
sudo ntpdate 192.168.1.1
```

#### 方法3：硬件时间同步（精确场景）

使用GPIO产生PPS（每秒脉冲）信号进行硬件时间对齐：

```cpp
// PX4侧：产生PPS信号
// boards/px4/fmu-v6x/src/board_config.h
#define GPIO_SYNC  /* PE9 */ (GPIO_OUTPUT|...)

// 外部设备：捕获PPS中断
void PPS_InterruptHandler() {
    // 对齐时钟
}
```

### 延迟补偿

**典型延迟链**：
```
传感器采样 → EKF2处理 → uORB发布 → MAVLink编码 → UART传输 → 外部板解析
   ~0.1ms      ~2ms       0.05ms      ~0.5ms      ~2ms@921600   ~0.5ms
                            总延迟: ~5-10ms
```

**补偿方法**：
```cpp
// 使用timestamp_sample而不是timestamp
uint64_t actual_time = att.timestamp_sample;  // 传感器原始时间
// 或手动补偿
uint64_t compensated_time = att.timestamp - ESTIMATED_DELAY_US;
```

---

## 性能优化

### 1. 波特率优化

**最大数据量计算**：

ATTITUDE_QUATERNION消息大小：
- MAVLink v2：32字节 + 12字节头部 = 44字节/消息
- 200Hz频率：44 × 200 = 8800字节/秒 = 70.4 kbps

**推荐波特率**：
- **115200 bps**：最多100Hz姿态数据
- **460800 bps**：最多400Hz姿态数据
- **921600 bps**：最多800Hz姿态数据（推荐）

### 2. 降低延迟

**PX4侧**：
```cpp
// 使用onboard模式（高优先级）
mavlink start -d /dev/ttyS6 -b 921600 -m onboard
```

**外部板侧**：
```cpp
// 使用DMA接收
HAL_UART_Receive_DMA(&huart1, rx_buffer, BUFFER_SIZE);

// 或使用中断
void USART1_IRQHandler() {
    // 立即处理
}
```

### 3. 数据打包

同时传输多个消息以提高效率：

```bash
# 批量配置高频流
mavlink stream -d /dev/ttyS6 -s ATTITUDE_QUATERNION -r 200
mavlink stream -d /dev/ttyS6 -s LOCAL_POSITION_NED -r 100
mavlink stream -d /dev/ttyS6 -s GLOBAL_POSITION_INT -r 10
```

---

## 故障排查

### 问题1：收不到数据

**检查清单**：
```bash
# 1. 验证MAVLink实例是否启动
mavlink status

# 2. 检查串口设备
ls -l /dev/ttyS*

# 3. 验证波特率
stty -F /dev/ttyS6 speed

# 4. 查看实时数据流
mavlink stream -d /dev/ttyS6 -s ATTITUDE_QUATERNION -r 50
listener vehicle_attitude  # 验证uORB有数据
```

### 问题2：数据频率异常

**诊断**：
```bash
# 查看实际发布频率
uorb top | grep vehicle_attitude

# 查看MAVLink统计
mavlink status
# 注意 "rate mult" 和 "rate max" 参数
```

**解决方案**：
```bash
# 增加数据速率限制
mavlink start -d /dev/ttyS6 -b 921600 -m onboard -r 100000  # 100KB/s
```

### 问题3：时间戳跳变

**原因**：
- EKF2重置
- 系统时钟跳变

**检测**：
```cpp
uint32_t last_timestamp = 0;
void handleAttitude(mavlink_attitude_quaternion_t* att) {
    if (att->time_boot_ms < last_timestamp) {
        Serial.println("WARNING: Timestamp jump detected!");
        // 重新初始化时间基准
    }
    last_timestamp = att->time_boot_ms;
}
```

### 问题4：四元数数据异常

**验证四元数有效性**：
```cpp
bool isQuaternionValid(float q[4]) {
    // 检查模长是否接近1
    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    return (abs(norm - 1.0) < 0.01);
}
```

### 问题5：USB端口占用

如果UART端口被占用，可临时使用USB：

```bash
# PX4侧：USB自动启动MAVLink
# 外部设备连接：Linux /dev/ttyACM0, Windows COMx

# Python示例
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
```

---

## 附录

### A. 相关代码文件索引

| 功能 | 文件路径 |
|------|---------|
| 板子配置 | `boards/px4/fmu-v6x/default.px4board` |
| 硬件定义 | `boards/px4/fmu-v6x/src/board_config.h` |
| EKF2姿态发布 | `src/modules/ekf2/EKF2.cpp:1046` |
| uORB消息定义 | `msg/versioned/VehicleAttitude.msg` |
| MAVLink姿态流 | `src/modules/mavlink/streams/ATTITUDE_QUATERNION.hpp` |
| MAVLink模块 | `src/modules/mavlink/mavlink_main.cpp` |
| UXRCE-DDS客户端 | `src/modules/uxrce_dds_client/uxrce_dds_client.cpp` |

### B. 参考文档

- [PX4官方文档 - MAVLink消息](https://docs.px4.io/main/en/mavlink/)
- [MAVLink消息定义](https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION)
- [Pixhawk 6X硬件文档](https://docs.px4.io/main/en/flight_controller/pixhawk6x.html)
- [UXRCE-DDS用户指南](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

### C. 常用MAVLink消息

| 消息ID | 消息名 | 频率推荐 | 内容 |
|--------|--------|---------|------|
| 30 | ATTITUDE | 50-200Hz | 欧拉角+角速度 |
| 31 | ATTITUDE_QUATERNION | 50-200Hz | 四元数+角速度 |
| 32 | LOCAL_POSITION_NED | 50Hz | 本地位置+速度 |
| 33 | GLOBAL_POSITION_INT | 10Hz | GPS位置+高度 |
| 105 | HIGHRES_IMU | 200Hz | 原始IMU数据 |

---

## 结论与建议

### 推荐实施路径

**阶段1：快速验证（1天）**
- 使用USB连接 + Python pymavlink
- 验证数据接收和解析

**阶段2：硬件集成（2-3天）**
- UART TELEM1端口 + 921600波特率
- Arduino/STM32 MAVLink库集成
- 验证200Hz数据接收

**阶段3：性能优化（可选）**
- 如需更高频率，切换到Ethernet + MAVLink
- 如需ROS2集成，使用UXRCE-DDS

### 最终方案选择建议

| 你的需求 | 推荐方案 | 预期性能 |
|---------|---------|---------|
| 简单可靠，中等频率 | **UART + MAVLink** | 200Hz, ~5ms延迟 |
| 高频低延迟 | **Ethernet + MAVLink** | 500Hz, ~1ms延迟 |
| ROS2生态集成 | **Ethernet + UXRCE-DDS** | 250Hz, ~2ms延迟 |
| 学习/原型验证 | **USB + pymavlink** | 50Hz, ~10ms延迟 |

### I2C/SPI方案可行性评估

**不推荐原因**：
❌ PX4无原生Slave模式支持
❌ 需要修改NuttX内核驱动
❌ 开发工作量巨大（预计2-4周）
❌ 维护成本高
❌ 社区支持少

**如果必须使用I2C/SPI**：
1. 参考PX4 I2C驱动框架：`platforms/nuttx/src/px4/common/i2c_spi_buses.cpp`
2. 实现I2C Slave模式寄存器映射
3. 创建自定义模块读取uORB并写入I2C寄存器
4. 估计开发时间：**3-4周**（含测试）

---

**文档版本**：v1.0
**最后更新**：2025-11-21
**适用PX4版本**：v1.14+
**测试硬件**：Pixhawk 6X (FMUv6X)
