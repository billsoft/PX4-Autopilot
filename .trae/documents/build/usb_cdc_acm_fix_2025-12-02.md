# USB CDC ACM 修复报告 (2025-12-02)

## 问题描述

用户报告虽然 USB CDC ACM 配置已添加（defconfig 中启用 `CONFIG_USBDEV` 和 `CONFIG_CDCACM`），且板级 `usb.c` 已创建，但 `/dev/ttyACM0` 设备可以手动通过 `sercon` 连接，MAVLink 可以启动，但系统启动时 USB CDC ACM 未自动工作。

## 根本原因分析

### 1. **init.cpp 缺少 USB 初始化调用**

**问题**：
- 板级 `usb.c` 中定义了 `stm32_usbinitialize()` 函数
- 但 `init.cpp` 的 `board_app_initialize()` 从未调用它
- USB OTG FS 硬件外设未被初始化，GPIO 引脚未配置

**参考其他板子**（如 fmu-v6x）：
```cpp
// boards/px4/fmu-v6x/src/init.cpp:181-183
/* configure USB interfaces */
stm32_usbinitialize();
```

### 2. **启动脚本未调用 `sercon` 连接 CDC ACM**

**问题**：
- NuttX CDC ACM 驱动加载后，需要显式调用 `sercon` 命令连接设备
- 原启动脚本只是等待 `/dev/ttyACM0` 出现，但从未触发连接

**PX4 标准做法**（`rc.base_core`）：
```bash
if ! cdcacm_autostart start
then
    sercon  # 连接 USB CDC ACM 设备
    mavlink start -d /dev/ttyACM0
fi
```

**为什么需要 `sercon`**：
- `sercon` = "serial console connect"
- 它触发 NuttX USB 设备栈枚举 USB 设备
- 创建 `/dev/ttyACM0` 字符设备节点
- 使 PC 端能够识别 USB 虚拟串口

---

## 修复方案

### 修复 1: 在 init.cpp 中添加 USB 初始化

**文件**: `boards/st/nucleo-h743zi-fc/src/init.cpp`

**修改位置**: 第 123-125 行（在 `stm32_spiinitialize()` 之后）

```cpp
/* SPI Init */
stm32_spiinitialize();

/* USB Init (for CDC ACM virtual serial port) */
extern void stm32_usbinitialize(void);
stm32_usbinitialize();
```

**效果**：
- 初始化 USB OTG FS 硬件外设
- 配置 PA11/PA12 引脚为 USB D+/D-
- 配置 GPIO_OTGFS_VBUS（如果定义）

### 修复 2: 在启动脚本中添加 `sercon` 调用

**文件**: `boards/st/nucleo-h743zi-fc/init/rc.board_sensors`

**修改位置**: 第 148-151 行（在生产模式 MAVLink 启动之前）

```bash
if [ $USE_SENSOR_STUB -eq 0 ]; then
    echo "[rc.board_sensors] Starting MAVLink on USB CDC ACM..."

    # Connect USB CDC ACM device (sercon)
    echo "Connecting USB CDC ACM..."
    sercon
    usleep 500000  # Wait 500ms for device enumeration

    # Wait for USB CDC ACM device to appear (max 5 seconds)
    USB_READY=0
    for i in 1 2 3 4 5; do
        if [ -c /dev/ttyACM0 ]; then
            USB_READY=1
            echo "OK: USB CDC ACM device ready (/dev/ttyACM0)"
            break
        fi
        ...
```

**效果**：
- 显式连接 USB CDC ACM 设备
- 触发 PC 端枚举（显示为 "PX4 Nucleo-H743ZI"）
- 创建 `/dev/ttyACM0` 字符设备
- 使 MAVLink 可以打开并使用该设备

---

## 技术背景：`sercon` 命令详解

### `sercon` 是什么？

`sercon` 是 NuttX 提供的系统命令，用于连接 USB CDC ACM 虚拟串口。

**源码位置**：
```
platforms/nuttx/NuttX/apps/system/cdcacm/cdcacm_main.c
```

**功能**：
1. 打开 USB CDC ACM 设备驱动
2. 触发 USB 设备栈启动枚举过程
3. 向 PC 端发送 USB 设备描述符
4. 创建 `/dev/ttyACM0` 字符设备节点
5. 使设备对应用程序可见

### USB CDC ACM 初始化流程

```
1. 硬件层：stm32_usbinitialize()
    ↓
2. NuttX 驱动加载：CONFIG_CDCACM=y
    ↓
3. 连接设备：sercon
    ↓
4. USB 枚举：PC 端识别设备
    ↓
5. 设备节点创建：/dev/ttyACM0
    ↓
6. 应用层使用：mavlink start -d /dev/ttyACM0
```

**每一步都必不可少！**

### 为什么之前手动 `sercon` 有效？

用户报告手动运行 `sercon` 后，USB CDC ACM 工作正常：

```bash
nsh> sercon
nsh> mavlink start -d /dev/ttyACM0
INFO  [mavlink] mode: Onboard, data rate: ...
```

**原因**：
- 硬件初始化（`stm32_usbinitialize`）可能在 NuttX 启动时自动执行（板子相关）
- 但 **设备连接** 必须显式调用 `sercon`
- 手动调用后，USB 枚举完成，设备可用

**现在的修复**：
- ✅ 确保硬件初始化在 `board_app_initialize` 中调用
- ✅ 确保 `sercon` 在启动脚本中自动调用
- ✅ 无需手动干预，开机即可使用

---

## 验证步骤

### 步骤 1: 重新编译固件

```bash
make st_nucleo-h743zi-fc_default
```

**预期**：
- 编译成功
- Flash 大小增加约 1-2 KB（USB 初始化代码）

### 步骤 2: 烧录固件

```bash
make st_nucleo-h743zi-fc_default upload
```

### 步骤 3: 切换到生产模式

修改 `rc.board_sensors` 第 13 行：
```bash
USE_SENSOR_STUB=0  # 生产模式（启用 MAVLink）
```

重新编译烧录。

### 步骤 4: 硬件连接

1. **ST-LINK USB（CN1）** → PC（控制台）
2. **用户 USB（CN13）** → PC（MAVLink）

### 步骤 5: 启动日志验证（NSH 控制台）

```bash
nsh> dmesg | grep -i usb
# 应显示：USB initialization...

nsh> dmesg | grep -i sercon
# 应显示：Connecting USB CDC ACM...

nsh> ls /dev
/dev:
 console
 ttyS0
 ttyACM0      ← 应自动出现

nsh> mavlink status
instance #0:
    transport protocol: serial (/dev/ttyACM0 @115200)
    mode: Onboard
    type: USB CDC
    GCS heartbeat valid: YES
```

### 步骤 6: PC 端验证

**Linux**:
```bash
# 查看 USB 设备
lsusb | grep PX4
# 输出：Bus 001 Device XXX: ID 26ac:0011 PX4 PX4 Nucleo-H743ZI

# 查看串口
ls -l /dev/ttyACM0
# 输出：crw-rw---- 1 root dialout 166, 0 Dec  2 16:30 /dev/ttyACM0

# 测试通信
minicom -D /dev/ttyACM0 -b 115200
# 应看到 MAVLink 二进制数据流
```

**Windows**:
1. 设备管理器 → 端口 (COM 和 LPT)
2. 应显示：`PX4 Nucleo-H743ZI (COMx)`
3. QGroundControl → 连接该端口 → 应看到实时数据

---

## 预期效果

### 开发模式（`USE_SENSOR_STUB=1`）

- ✅ NSH 控制台正常
- ❌ MAVLink 不启动（避免干扰控制台）
- ✅ `/dev/ttyACM0` 仍然存在（但未使用）
- ✅ `listener vehicle_attitude` 可验证数据

### 生产模式（`USE_SENSOR_STUB=0`）

- ✅ NSH 控制台正常（ST-LINK USB）
- ✅ MAVLink 自动启动（用户 USB）
- ✅ `/dev/ttyACM0` 自动创建并连接
- ✅ QGroundControl 可连接并看到 120Hz 姿态流
- ✅ 双通道独立工作，互不干扰

---

## 常见问题排查

### 问题 1: 编译后 USB 仍不工作

**排查**：
```bash
nsh> dmesg | grep -i "usb\|USB"
# 检查是否有 "USB initialization" 日志
```

**如果没有日志**：
- 确认 `init.cpp` 修改已生效
- 确认 `stm32_usbinitialize()` 被调用
- 检查编译是否使用了正确的源文件

### 问题 2: `sercon` 命令不存在

**原因**：`CONFIG_SYSTEM_CDCACM` 未启用

**解决**：
```bash
# 检查 defconfig
grep CONFIG_SYSTEM_CDCACM boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig
# 应显示：CONFIG_SYSTEM_CDCACM=y
```

如果缺失，添加并重新编译。

### 问题 3: PC 端未枚举 USB 设备

**Linux 排查**：
```bash
dmesg | tail -20
# 查看 USB 枚举日志

lsusb -v | grep -A 10 "26ac:0011"
# 查看设备详细信息
```

**Windows 排查**：
- 设备管理器 → 通用串行总线控制器
- 应显示 "PX4 Nucleo-H743ZI"
- 如果显示"未知设备"，手动安装 CDC ACM 驱动（通常自动）

**硬件排查**：
- 确认用户 USB 线是数据线（非仅充电线）
- 更换 USB 端口（使用主板 USB 3.0）
- 检查板子是否供电正常（LED 指示）

### 问题 4: MAVLink 启动失败

**NSH 验证**：
```bash
nsh> ls -l /dev/ttyACM0
# 确认设备存在

nsh> mavlink start -d /dev/ttyACM0 -b 115200 -m onboard
# 手动启动查看错误信息
```

**常见错误**：
- `Device or resource busy`: 设备已被占用（重启板子）
- `No such device`: `sercon` 未执行或失败
- `Permission denied`: NuttX 权限问题（不应出现）

---

## 相关文件清单

### 修改的文件

1. **boards/st/nucleo-h743zi-fc/src/init.cpp**
   - 添加 `stm32_usbinitialize()` 调用

2. **boards/st/nucleo-h743zi-fc/init/rc.board_sensors**
   - 添加 `sercon` 调用
   - 添加 500ms 等待时间

### 已有的支持文件（无需修改）

3. **boards/st/nucleo-h743zi-fc/src/usb.c**
   - 板级 USB 初始化接口

4. **boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig**
   - USB 配置已完整

5. **boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h**
   - USB 引脚定义已存在

---

## 总结

### 修复前

```
启动流程：
1. NuttX 启动
2. board_app_initialize() 执行
3. ❌ stm32_usbinitialize() 未调用 → USB 硬件未初始化
4. ❌ sercon 未调用 → USB 设备未连接
5. ❌ /dev/ttyACM0 不存在
6. ❌ MAVLink 启动失败

需要手动干预：
nsh> sercon  ← 手动连接
nsh> mavlink start -d /dev/ttyACM0  ← 手动启动
```

### 修复后

```
启动流程：
1. NuttX 启动
2. board_app_initialize() 执行
3. ✅ stm32_usbinitialize() 调用 → USB 硬件初始化
4. 启动脚本执行 rc.board_sensors
5. ✅ sercon 自动调用 → USB 设备连接
6. ✅ /dev/ttyACM0 自动创建
7. ✅ MAVLink 自动启动并运行

无需手动干预，开机即用！
```

### 核心要点

1. **USB 硬件初始化** 必须在 `board_app_initialize` 中调用 `stm32_usbinitialize()`
2. **USB 设备连接** 必须在启动脚本中调用 `sercon`
3. **两者缺一不可**：硬件未初始化，`sercon` 无效；不调用 `sercon`，设备不枚举

---

*文档版本*: v1.0
*更新日期*: 2025-12-02
*修复内容*: USB CDC ACM 自动初始化和连接
*影响范围*: 生产模式（`USE_SENSOR_STUB=0`）的 MAVLink 遥测功能
