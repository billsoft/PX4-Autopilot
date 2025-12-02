# 从零到起飞：PX4自定义STM32飞控板完全开发指南

> **写在前面**：这是一篇面向实战的深度教程。不同于官方文档的简略说明，本文将带你完整走过从硬件到软件、从NuttX到PX4、从编译到运行的全流程，并深挖每个决策背后的"为什么"。无论你是初次接触飞控开发的新手，还是希望深入理解PX4架构的工程师，这篇文章都将为你提供系统性的知识与可复用的实战经验。

**文档信息**:
- **版本**: 3.0 (整合版)
- **适用**: PX4 v1.14.x - v1.15.x
- **示例硬件**: ST Nucleo-H743ZI + 双ICM45686 IMU + BMM150磁力计
- **难度**: ⭐⭐⭐⭐ (高级，但新手可跟随操作)
- **预计时间**: 首次 8-12小时，熟练后 2-4小时
- **更新**: 2025-12-02

---

## 目录导航

### 第一部分：理解本质
1. [为什么需要自定义飞控板？](#1-为什么需要自定义飞控板)
2. [两条岔路：你应该选哪一条？](#2-两条岔路你应该选哪一条)
3. [三大原则：避开90%的坑](#3-三大原则避开90的坑)

### 第二部分：准备阶段
4. [硬件准备清单](#4-硬件准备清单)
5. [软件环境配置](#5-软件环境配置)
6. [参考板选择策略](#6-参考板选择策略)

### 第三部分：核心实战
7. [创建板目录结构](#7-创建板目录结构)
8. [配置NuttX: defconfig深度解析](#8-配置nuttx-defconfig深度解析)
9. [编写board.h: 时钟树与引脚映射](#9-编写boardh-时钟树与引脚映射)
10. [PX4板级代码实现](#10-px4板级代码实现)
11. [模块与驱动配置](#11-模块与驱动配置)
12. [启动脚本编写](#12-启动脚本编写)

### 第四部分：验证与调试
13. [编译验证](#13-编译验证)
14. [烧录与启动](#14-烧录与启动)
15. [运行时验证](#15-运行时验证)
16. [常见问题与解决方案](#16-常见问题与解决方案)

### 第五部分：进阶与优化
17. [性能优化](#17-性能优化)
18. [长期维护策略](#18-长期维护策略)
19. [从原型到产品](#19-从原型到产品)

---

## 第一部分：理解本质

### 1. 为什么需要自定义飞控板？

#### 1.1 现实场景

你可能正面临以下情况之一：

**场景A：科研项目的特殊需求**
> "我们的无人机需要搭载双路IMU做冗余融合，还要同步CMOS相机的帧信号，Pixhawk买来的板子根本没这个接口..."

**场景B：产品化的成本压力**
> "Pixhawk 6X一块要$300，我们要量产1000台。用STM32 Nucleo板子自己开发，硬件成本只要$50，能省下25万美元！"

**场景C：学习的深度需求**
> "我想真正理解PX4的底层架构，而不是停留在调参和写上层应用。从板级开发入手，能让我看到完整的系统是如何运转的。"

这些都是合理且常见的需求。但问题来了：

**PX4官方文档只有寥寥数页，关键步骤一笔带过。**

你可能看过这样的说明：

```
1. Create boards/{vendor}/{model}/ directory
2. Copy defconfig from similar board
3. Modify pins in board.h
4. Build and test
```

然后...就没有然后了。

#### 1.2 真实的痛点

实际开发中，你会遇到这些问题（我全踩过）：

**痛点1：编译通过，链接失败**
```bash
undefined reference to `stm32_boardinitialize'
undefined reference to `hrt_absolute_time'
undefined reference to `up_restoreusartint'
```
😱 **10多个链接错误**，错误信息指向NuttX深处，完全不知道从哪里下手。

**痛点2：烧录成功，系统不启动**
- 串口黑屏，无任何输出
- 或者只有NuttX启动日志，PX4永远卡在初始化

**痛点3：传感器读不到数据**
```bash
nsh> i2cdetect -b 1
# 全是 "--"，设备扫描不到
```
I2C引脚定义明明是对的，为什么不work？

**痛点4：修改丢失**
```bash
make distclean  # 清理构建
# ... 重新编译 ...
# 😱 刚改的board.h又变回去了！
```

#### 1.3 本文的价值

经过两年的实战与踩坑，我将Nucleo-H743ZI从零到完整运行的全过程整理成这篇教程。

**你将学到**：
1. **完整流程**：从目录创建到NSH验证的每一步
2. **深度原理**：为什么要这样配置？不这样会怎么样？
3. **避坑指南**：我踩过的所有坑和解决方案
4. **可复用模式**：不只是Nucleo，任何STM32 H7/F7/F4板子都适用

**你不会学到**：
- ❌ 如何设计硬件电路（假设你已有硬件）
- ❌ PX4上层应用开发（专注板级移植）
- ❌ 飞行调参技巧（那是另一个话题）

---

### 2. 两条岔路：你应该选哪一条？

在开始之前，你需要做一个重要决策。

#### 2.1 岔路口：两种板子类型

PX4中有两种自定义板子的方式，**选错了路，后面全是坑**。

##### ❌ 路线A：使用NuttX内置板子 (不推荐)

```
boards/st/nucleo-h743zi/
└─ (直接使用 NuttX 的 nucleo-h743zi)
```

**看起来很简单**：
- NuttX已经有`nucleo-h743zi`的完整配置
- 我只需要在PX4层加一个wrapper
- board.h都不用写，直接include NuttX的

**实际是个陷阱**：

1. **修改会丢失**
```bash
# 你改了NuttX子模块里的board.h
vim platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/nucleo-h743zi/include/board.h

# 构建清理时，子模块重置
make distclean
# 💥 你的修改全没了！
```

2. **无法自定义引脚**
   - NuttX内置板子的引脚是固定的
   - 你的IMU用的是PD14，内置配置是PB12
   - 改？回到问题1，修改会丢失

3. **违反PX4规范**
   - 修改子模块 = 维护噩梦
   - 升级PX4时，子模块冲突
   - 无法提交到上游（PX4不接受修改NuttX子模块的PR）

**什么时候用路线A？**
- 🟢 **仅用于快速原型验证**（几个小时的实验）
- 🟢 你的硬件引脚与内置板**完全一致**
- 🔴 **绝不能**用于产品或长期项目

##### ✅ 路线B：完全自定义板子 (标准做法)

```
boards/st/nucleo-h743zi-fc/
├─ nuttx-config/
│  ├─ include/board.h        ⭐ 400+行完整定义
│  ├─ nsh/defconfig           ⭐ CONFIG_ARCH_BOARD_CUSTOM=y
│  └─ Kconfig
├─ src/
│  ├─ board_config.h          ⭐ PX4层配置
│  ├─ spi.cpp
│  ├─ i2c.cpp
│  └─ init.cpp
├─ init/
│  └─ rc.board_sensors        ⭐ 启动脚本
└─ default.px4board           ⭐ 模块配置
```

**这就是 fmu-v6x、fmu-v5 等官方板子的做法**。

**优势**：
1. ✅ **完全控制**：每个引脚、每个时钟都由你定义
2. ✅ **持久化**：所有配置在PX4源码树内，不修改子模块
3. ✅ **可维护**：目录结构清晰，易于review和升级
4. ✅ **可扩展**：后续添加新外设，只需修改自己的文件
5. ✅ **符合规范**：可以提交PR到上游

**代价**：
- 需要写完整的board.h（400+行）
- 需要理解NuttX的配置系统
- 初次开发时间较长（但只需一次）

**本教程采用路线B！**

#### 2.2 决策树

```
你的硬件与NuttX内置板完全一致？
├─ 是 → 只是快速验证？
│  ├─ 是 → 可以用路线A (但仍建议B)
│  └─ 否 → 必须用路线B
└─ 否 → 必须用路线B

你的项目需要长期维护？
└─ 是 → 必须用路线B

你希望理解PX4板级开发？
└─ 是 → 强烈建议路线B
```

---

### 3. 三大原则：避开90%的坑

在深入实战前，理解这三个原则，能让你避开90%的坑。

#### 原则1：永远不修改NuttX子模块

**❌ 错误做法**：
```bash
vim platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/nucleo-h743zi/include/board.h
# 改了个引脚定义
```

**为什么错误？**

1. **子模块的工作机制**：
```bash
platforms/nuttx/NuttX/
├─ .git/          # 这是一个独立的git仓库
└─ nuttx/
   └─ boards/     # 这里的修改不受PX4 git管理
```

2. **distclean会重置子模块**：
```bash
make distclean
# 内部执行：
# git submodule update --init --recursive --force
# 💥 你的修改被强制重置！
```

3. **升级PX4时的冲突**：
```bash
git pull origin main
git submodule update
# 💥 你的修改与上游冲突，merge hell！
```

**✅ 正确做法**：
```bash
# 所有配置都在PX4源码树内
vim boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h
vim boards/st/nucleo-h743zi-fc/src/board_config.h
```

**核心思想**：
> 把NuttX子模块当作"只读库"，所有自定义配置都在`boards/`目录下完成。

---

#### 原则2：必须使用 CONFIG_ARCH_BOARD_CUSTOM

这是**最容易被忽略，但最致命**的配置。

**问题现场**：

你创建了`boards/st/my-board/`目录，写了`default.px4board`，然后：

```bash
make st_my-board_default
# 编译通过！✅

# 但是... board.h 用的是 NuttX 的默认配置
# 你的引脚定义根本没生效！
```

**根本原因**：

NuttX不知道你有自定义板子目录，它仍然在用：
```
platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/<某个内置板>/
```

**解决方法**：

在`defconfig`中明确告诉NuttX：

```kconfig
# boards/st/my-board/nuttx-config/nsh/defconfig

# ⭐⭐⭐ 这是关键！
CONFIG_ARCH_BOARD_CUSTOM=y
CONFIG_ARCH_BOARD_CUSTOM_DIR="../../../../boards/st/my-board/nuttx-config"
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
CONFIG_ARCH_BOARD_CUSTOM_NAME="my-board"

# ❌ 不要用这些（会指向NuttX内置板）:
# CONFIG_ARCH_BOARD="nucleo-h743zi"
# CONFIG_ARCH_BOARD_NUCLEO_H743ZI=y
```

**路径解析**：
```
构建目录：build/st_my-board_default/
           └─ NuttX/nuttx-config/
                                 └─ (相对路径) ../../../../boards/st/my-board/nuttx-config
                                                           └─ include/board.h  ✅ 找到了！
```

**验证方法**：

编译时查看日志：
```bash
make st_my-board_default 2>&1 | grep "Board config"
# 应该显示：
# Board config directory: ../../../../boards/st/my-board/nuttx-config
```

---

#### 原则3：board.h 必须完全自包含

**❌ 错误认知**：

> "我可以include NuttX内置板的board.h，然后只override几个宏。"

```c
// ❌ 错误示例
#include "../../../../../../platforms/nuttx/NuttX/nuttx/boards/arm/stm32h7/nucleo-h743zi/include/board.h"

#undef GPIO_SPI1_CS
#define GPIO_SPI1_CS (GPIO_OUTPUT | GPIO_PD14)  // 只改这一个
```

**为什么行不通？**

1. **路径依赖地狱**：
   - 构建目录在`build/st_my-board_default/NuttX/`
   - 相对路径层数不对，找不到文件
   - 不同操作系统（Windows/Linux/WSL）路径格式不同

2. **宏定义顺序问题**：
   - NuttX内部有大量`#ifndef ... #endif`保护
   - 你的`#undef`可能不生效
   - 或者你undef了不该undef的宏，导致其他错误

3. **维护噩梦**：
   - 上游board.h更新，你的override可能失效
   - 依赖的宏结构变化，你的代码要跟着改

**✅ 正确做法**：

**完全重写board.h，400-500行**。

```c
// boards/st/my-board/nuttx-config/include/board.h

#ifndef __BOARDS_ST_MY_BOARD_NUTTX_CONFIG_INCLUDE_BOARD_H
#define __BOARDS_ST_MY_BOARD_NUTTX_CONFIG_INCLUDE_BOARD_H

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* ========== 1. 时钟配置 ========== */
#define STM32_BOARD_XTAL        8000000ul
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
// ... (100行时钟树配置)

/* ========== 2. 引脚定义 ========== */
#define GPIO_SPI1_SCK     (GPIO_SPI1_SCK_1 | GPIO_SPEED_50MHz)   // PA5
#define GPIO_SPI1_MISO    (GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz)  // PA6
// ... (300行引脚定义)

#endif /* __BOARDS_ST_MY_BOARD_NUTTX_CONFIG_INCLUDE_BOARD_H */
```

**参考示例**：
- ✅ `boards/px4/fmu-v6x/nuttx-config/include/board.h` (476行)
- ✅ `boards/ark/fmu-v6x/nuttx-config/include/board.h` (464行)
- ✅ 本项目：`boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h` (463行)

**工作量**：
- 首次编写：2-4小时（复制参考板 + 修改引脚）
- 后续维护：10-30分钟（只需改动的引脚）

**核心思想**：
> 宁可多写200行，也不要依赖不受控的外部文件。

---

## 第二部分：准备阶段

### 4. 硬件准备清单

#### 4.1 本教程使用的硬件

**开发板**：ST Nucleo-H743ZI
- MCU：STM32H743ZIT6 (LQFP144, 480MHz, 2MB Flash, 1MB RAM)
- 调试接口：ST-LINK/V2-1 (板载)
- 时钟源：8MHz from ST-LINK MCO
- 价格：约$25 (官方) / ¥150 (国内)

**传感器**：
- IMU1: ICM45686 (SPI1)
- IMU2: ICM45686 (SPI3)
- 磁力计: BMM150 (I2C1)

**为什么选这个组合？**

1. **与FMU-V6X相同的传感器**：
   - 驱动已验证，兼容性好
   - ICM45686可用icm42688p驱动的`-6`兼容模式

2. **引脚灵活**：
   - Nucleo板有Arduino接口和Morpho接口
   - 100+个GPIO可选，不像Pixhawk那样受限

3. **性价比高**：
   - 硬件成本 <$100
   - 性能与Pixhawk 6X相当 (同样是H7@480MHz)

#### 4.2 硬件连接

##### SPI1 - IMU1 (Arduino接口)

| 信号 | Nucleo引脚 | Arduino标号 | 说明 |
|------|-----------|------------|------|
| SCK  | PA5 | D13 | SPI时钟 |
| MISO | PA6 | D12 | 主机输入 |
| MOSI | PD7 | Morpho CN12-19 | 主机输出 |
| CS   | PD14 | D10 | 片选(低有效) |

##### SPI3 - IMU2 (Morpho接口)

| 信号 | Nucleo引脚 | 位置 | 说明 |
|------|-----------|------|------|
| SCK  | PC10 | Morpho CN12-1 | SPI时钟 |
| MISO | PC11 | Morpho CN12-2 | 主机输入 |
| MOSI | PB2 | Morpho CN12-22 | 主机输出 |
| CS   | PA15 | Arduino D9 | 片选(低有效) |

##### I2C1 - 磁力计 (Arduino接口)

| 信号 | Nucleo引脚 | Arduino标号 | 说明 |
|------|-----------|------------|------|
| SCL  | PB6 | Morpho CN12-17 | I2C时钟 |
| SDA  | PB9 | D14 | I2C数据 |

**⚠️ 重要**：SDA是PB9，不是PB7！(CubeMX生成的配置)

##### LED指示 (板载)

| LED | Nucleo引脚 | 颜色 | 用途 |
|-----|-----------|------|------|
| LED1 | PB0 | 绿色 | IMU1状态 |
| LED2 | PB7 | 黄色 | IMU2状态 |
| LED3 | PB14 | 红色 | 磁力计状态 |

**极性**：高电平有效 (1=亮, 0=灭)

##### 串口 (VCP)

| 信号 | Nucleo引脚 | 说明 |
|------|-----------|------|
| TX   | PD8 (USART3_TX) | 发送到PC |
| RX   | PD9 (USART3_RX) | 接收自PC |

**波特率**：115200-8N1
**用途**：NSH控制台 + MAVLink输出

#### 4.3 硬件验证清单

在开始软件开发前，确认：

- [ ] 开发板能正常供电，LED常亮
- [ ] ST-LINK USB连接到PC，能被识别
- [ ] 串口能正常通信（用串口助手测试）
- [ ] 传感器焊接正确，无短路/虚焊

---

### 5. 软件环境配置

#### 5.1 开发环境

**推荐**：WSL2 (Windows Subsystem for Linux)

```bash
# Windows 10/11
wsl --install
wsl --set-default-version 2
wsl --install Ubuntu-22.04
```

**为什么用WSL？**
- ✅ 原生Linux工具链，编译快
- ✅ 可以访问Windows文件系统
- ✅ 串口调试用Windows工具(PuTTY/Tera Term)
- ✅ 烧录用Windows的STM32CubeProgrammer

**备选**：
- Linux原生 (最佳性能)
- macOS (arm64版本有些小坑)
- Windows原生 (编译慢，不推荐)

#### 5.2 PX4源码

```bash
# 克隆PX4 (recursive很重要！)
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# 检查子模块
git submodule status | head -5
# 应该显示：
#  <commit> platforms/nuttx/NuttX/apps (heads/...)
#  <commit> platforms/nuttx/NuttX/nuttx (heads/...)
# ...
```

#### 5.3 编译工具链

**安装PX4工具链**：

```bash
cd PX4-Autopilot

# Ubuntu/WSL
bash ./Tools/setup/ubuntu.sh

# 安装后重启终端
exit
# 重新进入WSL
```

**验证安装**：

```bash
# 检查GCC
arm-none-eabi-gcc --version
# 应该显示 arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1

# 检查Make
make --version
# 应该显示 GNU Make 4.3

# 检查Python
python3 --version
# 应该显示 Python 3.10+
```

#### 5.4 烧录工具

**STM32CubeProgrammer** (Windows)

1. 下载：https://www.st.com/en/development-tools/stm32cubeprog.html
2. 安装到：`C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\`
3. 添加到PATH：
   ```cmd
   setx PATH "%PATH%;C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin"
   ```

**验证**：
```cmd
STM32_Programmer_CLI.exe --version
# 应该显示 STM32CubeProgrammer v2.21.0
```

#### 5.5 串口工具

**Windows推荐**：
- PuTTY (轻量)
- Tera Term (功能全)

**Linux/WSL**：
```bash
sudo apt install picocom
picocom /dev/ttyACM0 -b 115200
```

---

### 6. 参考板选择策略

**不要盲目复制！** 选错参考板，后面全是坑。

#### 6.1 参考板选择矩阵

| 你的MCU | 推荐参考板 | 原因 |
|---------|-----------|------|
| STM32H743 | px4/fmu-v6x 或 ark/fmu-v6x | 同芯片，时钟树相同 |
| STM32H753 | px4/fmu-v6x | H7系列，差异小 |
| STM32F767 | px4/fmu-v5 | F7系列参考 |
| STM32F427 | px4/fmu-v3 | F4系列参考 |

**关键差异对比**：

| 特性 | fmu-v6x (H743) | 本项目 (H743) | 注意事项 |
|------|---------------|--------------|----------|
| HSE频率 | 16 MHz | 8 MHz | ⚠️ **PLL配置不同** |
| SYSCLK | 480 MHz | 480 MHz | ✅ 相同 |
| SPI时钟源 | PLL2P 192MHz | PLL2P 192MHz | ✅ 相同 |
| SPI1 MOSI | PA7 | PD7 | ⚠️ **引脚不同** |
| I2C1 SDA | PB7 | PB9 | ⚠️ **引脚不同** |
| LED数量 | 2个 | 3个 | ✅ 可扩展 |

#### 6.2 复制策略

**推荐流程**：

```bash
# 1. 创建目录
mkdir -p boards/st/nucleo-h743zi-fc/nuttx-config/include
mkdir -p boards/st/nucleo-h743zi-fc/nuttx-config/nsh
mkdir -p boards/st/nucleo-h743zi-fc/src
mkdir -p boards/st/nucleo-h743zi-fc/init

# 2. 复制参考板文件
cp boards/px4/fmu-v6x/nuttx-config/include/board.h \
   boards/st/nucleo-h743zi-fc/nuttx-config/include/

cp boards/px4/fmu-v6x/nuttx-config/nsh/defconfig \
   boards/st/nucleo-h743zi-fc/nuttx-config/nsh/

cp boards/px4/fmu-v6x/src/board_config.h \
   boards/st/nucleo-h743zi-fc/src/

# 3. 然后逐个修改 (下一章详述)
```

**不要**：
- ❌ 复制简单的板子（如`nucleo-f767zi`），它们太简化
- ❌ 复制非STM32板子（如`px4_fmu-v6c`是H7B3，不同芯片）
- ❌ 复制F4板子去适配H7（寄存器结构差异大）

---

## 第三部分：核心实战

### 7. 创建板目录结构

#### 7.1 标准目录树

```
boards/st/nucleo-h743zi-fc/
├─ default.px4board              # PX4模块配置
├─ nuttx-config/
│  ├─ include/
│  │  └─ board.h                 # NuttX硬件定义 (⭐核心⭐)
│  ├─ nsh/
│  │  └─ defconfig               # NuttX内核配置 (⭐核心⭐)
│  └─ Kconfig                    # 可选
├─ src/
│  ├─ board_config.h             # PX4板级配置 (⭐核心⭐)
│  ├─ init.cpp                   # PX4初始化 (⭐核心⭐)
│  ├─ spi.cpp                    # SPI设备表
│  ├─ i2c.cpp                    # I2C总线配置
│  ├─ board_hw_info.c            # 硬件版本信息
│  └─ CMakeLists.txt             # 编译配置
├─ init/
│  └─ rc.board_sensors           # 传感器启动脚本 (⭐核心⭐)
└─ firmware/
   └─ (空，运行时生成)
```

**核心文件**（5个必须）：
1. `default.px4board` - 告诉PX4启用哪些模块
2. `nuttx-config/nsh/defconfig` - 告诉NuttX启用哪些外设
3. `nuttx-config/include/board.h` - 定义时钟和引脚
4. `src/board_config.h` - PX4层的配置
5. `init/rc.board_sensors` - 启动时运行的脚本

#### 7.2 创建命令

```bash
cd PX4-Autopilot

# 创建目录结构
mkdir -p boards/st/nucleo-h743zi-fc/nuttx-config/include
mkdir -p boards/st/nucleo-h743zi-fc/nuttx-config/nsh
mkdir -p boards/st/nucleo-h743zi-fc/src
mkdir -p boards/st/nucleo-h743zi-fc/init

# 创建占位文件（后续填充内容）
touch boards/st/nucleo-h743zi-fc/default.px4board
touch boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h
touch boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig
touch boards/st/nucleo-h743zi-fc/src/board_config.h
touch boards/st/nucleo-h743zi-fc/src/init.cpp
touch boards/st/nucleo-h743zi-fc/init/rc.board_sensors
```

#### 7.3 验证目录创建

```bash
# 检查目录结构
tree boards/st/nucleo-h743zi-fc/

# 应该显示：
boards/st/nucleo-h743zi-fc/
├── default.px4board
├── init
│   └── rc.board_sensors
├── nuttx-config
│   ├── include
│   │   └── board.h
│   └── nsh
│       └── defconfig
└── src
    ├── board_config.h
    └── init.cpp
```

---

### 8. 配置NuttX: defconfig深度解析

这是**最关键**的一步。defconfig告诉NuttX：
- 使用哪个芯片
- 启用哪些外设
- 从哪里加载板级配置

#### 8.1 defconfig的核心作用

**NuttX配置系统**是基于Kconfig的（类似Linux内核）：

```
.config (构建时生成)
   ↑
defconfig (你提供的默认配置)
   ↑
Kconfig (所有可用选项)
```

**流程**：
1. 构建开始
2. NuttX读取`defconfig`
3. 生成`.config`
4. 根据`.config`编译代码

#### 8.2 完整defconfig示例

**文件路径**：`boards/st/nucleo-h743zi-fc/nuttx-config/nsh/defconfig`

```kconfig
#
# Nucleo-H743ZI-FC NuttX Configuration
# 生成方法：make menuconfig → make savedefconfig
#

# ============================================================
# 第1部分：使用自定义板目录 (⭐⭐⭐ 最重要！)
# ============================================================

# 启用自定义板目录模式
CONFIG_ARCH_BOARD_CUSTOM=y

# 自定义板目录路径（相对于NuttX构建目录）
# 构建目录：build/st_nucleo-h743zi-fc_default/NuttX/
# 目标目录：boards/st/nucleo-h743zi-fc/nuttx-config/
# 相对路径：../../../../boards/st/nucleo-h743zi-fc/nuttx-config
CONFIG_ARCH_BOARD_CUSTOM_DIR="../../../../boards/st/nucleo-h743zi-fc/nuttx-config"

# 使用相对路径解析
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y

# 板子名称（用于日志和标识）
CONFIG_ARCH_BOARD_CUSTOM_NAME="nucleo-h743zi-fc"

# ⚠️ 不要启用这些（会指向NuttX内置板）:
# CONFIG_ARCH_BOARD="nucleo-h743zi"
# CONFIG_ARCH_BOARD_NUCLEO_H743ZI=y

# ============================================================
# 第2部分：平台与芯片选择
# ============================================================

# ARM Cortex-M架构
CONFIG_ARCH="arm"

# STM32H7芯片家族
CONFIG_ARCH_CHIP="stm32h7"
CONFIG_ARCH_CHIP_STM32H7=y

# 具体型号：STM32H743ZI
CONFIG_ARCH_CHIP_STM32H743ZI=y

# 栈dump（调试用）
CONFIG_ARCH_STACKDUMP=y

# D-Cache与I-Cache
CONFIG_ARMV7M_DCACHE=y
CONFIG_ARMV7M_DCACHE_WRITETHROUGH=n  # 写回模式，性能更好
CONFIG_ARMV7M_DTCM=y
CONFIG_ARMV7M_ICACHE=y

# ============================================================
# 第3部分：时钟配置
# ============================================================

# HSE频率：8MHz（来自ST-LINK MCO）
CONFIG_STM32H7_HSE_FREQUENCY=8000000

# SYSCLK目标：480MHz
CONFIG_STM32H7_BOARD_HCLK=480000000

# PLL1配置：(8MHz / M) * N / P = (8 / 2) * 240 / 2 = 480MHz
CONFIG_STM32H7_PLLCFG_PLLM=2
CONFIG_STM32H7_PLLCFG_PLLN=240
CONFIG_STM32H7_PLLCFG_PLLP=2
CONFIG_STM32H7_PLLCFG_PLLQ=4
CONFIG_STM32H7_PLLCFG_PLLR=2

# ============================================================
# 第4部分：定时器框架 (⭐ HRT依赖)
# ============================================================

# 启用定时器框架
CONFIG_TIMER=y

# 启用一次性定时器（PX4 HRT需要）
CONFIG_ONESHOT=y

# ⚠️ 重要：不要启用TIM5驱动（PX4 HRT直接控制TIM5）
# CONFIG_STM32H7_TIM5 is not set

# 说明：
# - PX4的HRT (High Resolution Timer) 使用TIM5提供微秒级时间服务
# - HRT_TIMER=5 在board_config.h中定义
# - 如果启用CONFIG_STM32H7_TIM5，NuttX会初始化TIM5，与PX4 HRT冲突
# - 编译时会出现链接错误：undefined reference to hrt_absolute_time

# ============================================================
# 第5部分：SPI配置
# ============================================================

# 启用SPI1和SPI3硬件
CONFIG_STM32H7_SPI1=y
CONFIG_STM32H7_SPI3=y

# 启用SPI框架
CONFIG_SPI=y
CONFIG_SPI_EXCHANGE=y  # 全双工收发
CONFIG_SPI_DRIVER=y    # 创建/dev/spi*设备节点

# DMA配置（初期禁用，稳定后可启用）
CONFIG_STM32H7_DMA1=y
CONFIG_STM32H7_DMA2=y
# CONFIG_SPI_DMA is not set        # 全局SPI DMA开关（暂不启用）
# CONFIG_SPI1_DMA is not set        # SPI1 DMA（暂不启用）
# CONFIG_SPI3_DMA is not set        # SPI3 DMA（暂不启用）

# 说明：
# - 初期禁用DMA，确保SPI通信稳定
# - 调试完成后，可启用DMA提升性能
# - DMA能减少CPU占用，提高传感器采样率

# ============================================================
# 第6部分：I2C配置
# ============================================================

# 启用I2C1硬件
CONFIG_STM32H7_I2C1=y

# 启用I2C框架
CONFIG_I2C=y
CONFIG_I2C_DRIVER=y
CONFIG_I2C_TRANSFER=y

# I2C复位功能（禁用，避免早期HardFault）
# CONFIG_I2C_RESET is not set

# I2C动态超时
CONFIG_STM32H7_I2C_DYNTIMEO=y
CONFIG_STM32H7_I2C_DYNTIMEO_USECPERBYTE=10

# I2C延迟初始化（⭐ 关键配置）
CONFIG_BOARD_I2C_LATEINIT=y

# 说明：
# - I2C_LATEINIT：延迟到board_app_initialize()时初始化I2C
# - 原因：早期初始化I2C可能因时钟/电源未就绪导致HardFault
# - 在init.cpp中调用px4_i2c_initialize()完成初始化

# ============================================================
# 第7部分：UART配置
# ============================================================

# 启用USART3
CONFIG_STM32H7_USART3=y

# USART3作为控制台
CONFIG_USART3_SERIAL_CONSOLE=y

# 波特率配置
CONFIG_USART3_BAUD=115200
CONFIG_USART3_BITS=8
CONFIG_USART3_PARITY=0
CONFIG_USART3_2STOP=0

# UART DMA（初期禁用）
CONFIG_USART3_RXDMA=n
CONFIG_USART3_TXDMA=n

# 缓冲区大小
CONFIG_USART3_RXDMA_BUFFER_SIZE=256
CONFIG_USART3_TXDMA_BUFFER_SIZE=2048
CONFIG_USART3_RXBUFSIZE=1024
CONFIG_USART3_TXBUFSIZE=2048

# 串口终端
CONFIG_SERIAL_TERMIOS=y
CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y

# ============================================================
# 第8部分：GPIO配置
# ============================================================

# 启用GPIO端口
CONFIG_STM32H7_GPIOA=y
CONFIG_STM32H7_GPIOB=y
CONFIG_STM32H7_GPIOD=y
CONFIG_STM32H7_GPIOE=y

# 说明：只启用用到的GPIO端口，减少功耗

# ============================================================
# 第9部分：工作队列配置 (PX4必需)
# ============================================================

# 高优先级工作队列
CONFIG_SCHED_HPWORK=y
CONFIG_SCHED_HPWORKPRIORITY=249
CONFIG_SCHED_HPWORKSTACKSIZE=2048

# 低优先级工作队列
CONFIG_SCHED_LPWORK=y
CONFIG_SCHED_LPWORKPRIORITY=50
CONFIG_SCHED_LPWORKSTACKSIZE=2048

# 说明：
# - PX4的ScheduledWorkItem依赖NuttX工作队列
# - 高优先级用于实时任务（IMU采样、融合）
# - 低优先级用于非实时任务（LED、日志）

# ============================================================
# 第10部分：内存配置
# ============================================================

# RAM配置
CONFIG_RAM_START=0x24000000  # AXI SRAM起始地址
CONFIG_RAM_SIZE=524288        # 512KB AXI SRAM

# 内存区域数量（H743有多个RAM域）
CONFIG_MM_REGIONS=4

# 栈大小
CONFIG_INIT_STACKSIZE=3000
CONFIG_IDLETHREAD_STACKSIZE=2048
CONFIG_USERMAIN_STACKSIZE=4096
CONFIG_PTHREAD_STACK_MIN=512
CONFIG_PTHREAD_STACK_DEFAULT=2048

# 栈着色（调试栈溢出）
CONFIG_STACK_COLORATION=y

# ============================================================
# 第11部分：文件系统配置
# ============================================================

# ROMFS（PX4启动脚本）
CONFIG_FS_ROMFS=y

# 说明：
# - ROMFS存储PX4启动脚本（rcS, rc.board_sensors等）
# - 编译时打包到固件中，无需SD卡

# ============================================================
# 第12部分：系统配置
# ============================================================

# NSH (NuttShell)
CONFIG_SYSTEM_NSH=y
CONFIG_NSH_BUILTIN_APPS=y
CONFIG_NSH_FILEIOSIZE=512
CONFIG_NSH_LINELEN=64
CONFIG_NSH_READLINE=y

# NSH启动配置
CONFIG_NSH_ROMFSETC=y
CONFIG_NSH_ARCHINIT=y

# 板级控制
CONFIG_BOARDCTL=y
CONFIG_BOARDCTL_ROMDISK=y
CONFIG_BOARDCTL_RESET=y

# 调试配置
CONFIG_DEBUG_ASSERTIONS=y
CONFIG_DEBUG_FEATURES=y
CONFIG_DEBUG_HARDFAULT_ALERT=y
CONFIG_DEBUG_MEMFAULT=y
CONFIG_DEBUG_TCBINFO=y

# 符号表（调试用）
CONFIG_DEBUG_SYMBOLS=y

# 管道（MAVLink shell）
CONFIG_PIPES=y

# 任务配置
CONFIG_TASK_NAME_SIZE=24
CONFIG_SCHED_WAITPID=y

# 信号配置
CONFIG_SIG_SIGSTOP_ACTION=y

# 定时器配置
CONFIG_PREALLOC_TIMERS=4

# 调度间隔
CONFIG_RR_INTERVAL=200

# ============================================================
# 第13部分：电源管理
# ============================================================

# 启用电源管理（修复up_restoreusartint链接错误）
CONFIG_PM=y
CONFIG_STM32H7_PWR=y

# 说明：
# - 某些UART功能（如DMA恢复）需要电源管理支持
# - 不启用会导致：undefined reference to up_restoreusartint

# ============================================================
# 第14部分：构建配置
# ============================================================

# 启用实验性功能
CONFIG_EXPERIMENTAL=y

# C++支持
CONFIG_HAVE_CXX=y
CONFIG_HAVE_CXXINITIALIZE=y

# 构建应用
CONFIG_BUILTIN=y

# 二进制格式
CONFIG_INTELHEX_BINARY=y
CONFIG_RAW_BINARY=y

# 入口点
CONFIG_INIT_ENTRYPOINT="nsh_main"

# 循环计数（调校用）
CONFIG_BOARD_LOOPSPERMSEC=43103

# 日期配置
CONFIG_START_YEAR=2011
CONFIG_START_MONTH=12
CONFIG_START_DAY=6
```

#### 8.3 defconfig常见错误

**错误1：忘记CONFIG_ARCH_BOARD_CUSTOM**

❌ 现象：
```bash
make st_nucleo-h743zi-fc_default
# 编译通过，但引脚定义不生效
```

✅ 解决：
```kconfig
CONFIG_ARCH_BOARD_CUSTOM=y
CONFIG_ARCH_BOARD_CUSTOM_DIR="../../../../boards/st/nucleo-h743zi-fc/nuttx-config"
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
CONFIG_ARCH_BOARD_CUSTOM_NAME="nucleo-h743zi-fc"
```

**错误2：启用了CONFIG_STM32H7_TIM5**

❌ 现象：
```bash
undefined reference to `hrt_absolute_time'
undefined reference to `hrt_call_after'
```

✅ 解决：
```kconfig
CONFIG_TIMER=y
CONFIG_ONESHOT=y
# ⚠️ 不要启用：
# CONFIG_STM32H7_TIM5 is not set
```

**错误3：HSE频率设置错误**

❌ 现象：
- 编译通过
- 烧录后系统不启动或串口乱码

✅ 解决：
```kconfig
# Nucleo板是8MHz（ST-LINK MCO）
CONFIG_STM32H7_HSE_FREQUENCY=8000000

# 不是16MHz！（那是fmu-v6x的配置）
# CONFIG_STM32H7_HSE_FREQUENCY=16000000  # ❌ 错误
```

**错误4：忘记启用I2C_LATEINIT**

❌ 现象：
- 系统启动时HardFault
- dmesg显示I2C初始化错误

✅ 解决：
```kconfig
CONFIG_BOARD_I2C_LATEINIT=y
```

然后在`init.cpp`中：
```cpp
__EXPORT int board_app_initialize(uintptr_t arg) {
    // ... 其他初始化 ...

    // 延迟初始化I2C
    (void)px4_i2c_buses;

    return OK;
}
```

---

### 9. 编写board.h: 时钟树与引脚映射

这是最"吓人"的一步：400+行的C头文件。

**但别担心**！大部分是复制粘贴+修改。

#### 9.1 board.h的作用

**board.h定义了硬件的"语言"**：

- **时钟树**: 告诉NuttX如何配置RCC (Reset and Clock Control)
- **引脚映射**: 告诉NuttX哪个GPIO对应哪个功能
- **外设配置**: 告诉NuttX SPI/I2C/UART的参数

**对比理解**：
```
CubeMX .ioc文件    →  硬件配置（图形化）
NuttX board.h      →  硬件配置（代码化）
```

#### 9.2 完整board.h框架

**文件路径**：`boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h`

由于board.h非常长(463行)，我会分段讲解核心部分。完整文件请参考实际项目。

##### 第1部分：文件头与保护

```c
/****************************************************************************
 * boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h
 *
 * Nucleo-H743ZI Flight Controller Board Definitions
 ****************************************************************************/

#ifndef __BOARDS_ST_NUCLEO_H743ZI_FC_NUTTX_CONFIG_INCLUDE_BOARD_H
#define __BOARDS_ST_NUCLEO_H743ZI_FC_NUTTX_CONFIG_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */
/* 不要include其他板子的board.h！完全自包含！ */
```

**要点**：
- ✅ 只include `<nuttx/config.h>`
- ❌ 不要include NuttX内置板的board.h
- ✅ 使用标准的头文件保护

##### 第2部分：时钟源定义

```c
/****************************************************************************
 * Pre-processor Definitions - Clock Configuration
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Nucleo-144 board provides the following clock sources:
 *
 *   MCO: 8 MHz from MCO output of ST-LINK is used as input clock (default)
 *   X2:  32.768 KHz crystal for LSE
 *   X3:  HSE crystal oscillator (not provided)
 *
 * So we have these clock source available within the STM32:
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 8 MHz from MCO output of ST-LINK  ⭐ 我们用这个
 *   LSE: 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul  /* ST-LINK MCO */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL  /* 8 MHz */
#define STM32_LSE_FREQUENCY     32768
```

**⚠️ 关键点**：

- **HSE = 8MHz**：来自ST-LINK的MCO输出
- **不是25MHz晶振**（Nucleo板没有外部晶振）
- **不是16MHz**（那是fmu-v6x的配置）

**如何确认你的板子的HSE频率？**

1. 查看原理图（Nucleo UM1974手册）
2. 或者在CubeMX中配置后查看
3. 或者参考ST官方示例代码

##### 第3部分：PLL配置（时钟树核心）

这是**最难理解但最重要**的部分。

**H743时钟树简图**：

```
HSE (8MHz)
  ↓
PLL1 → [/M1=2] → 4MHz → [*N1=240] → 960MHz VCO
                                       ↓
                                    [/P1=2] → 480MHz → SYSCLK ⭐
                                       ↓
                                    [/Q1=4] → 240MHz → USB/SDMMC
                                       ↓
                                    [/R1=8] → 120MHz → 其他外设

PLL2 → [/M2=2] → 4MHz → [*N2=96] → 384MHz VCO
                                      ↓
                                   [/P2=2] → 192MHz → SPI123 ⭐
```

**代码实现**：

```c
/* Main PLL Configuration.
 *
 * PLL source is HSE = 8,000,000
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *     1 <= PLLM <= 63
 *     4 <= PLLN <= 512
 *   150 MHz <= PLL_VCOL <= 420MHz
 *   192 MHz <= PLL_VCOH <= 836MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * CPUCLK  = SYSCLK / D1CPRE
 * Subject to:
 *   PLLP1   = {2, 4, 6, 8, ..., 128}
 *   PLLP2,3 = {2, 3, 4, ..., 128}
 *   CPUCLK <= 480 MHz
 */

#define STM32_BOARD_USEHSE
#define STM32_HSEBYP_ENABLE  /* HSE bypassed (来自MCO，不是晶振) */

#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (8,000,000 / 2) * 240 = 960 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 960 MHz / 2   = 480 MHz  ⭐ SYSCLK
 *   PLL1Q = PLL1_VCO/4  = 960 MHz / 4   = 240 MHz
 *   PLL1R = PLL1_VCO/8  = 960 MHz / 8   = 120 MHz
 */

#define STM32_PLLCFG_PLL1CFG     (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                                  RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
                                  RCC_PLLCFGR_DIVP1EN | \
                                  RCC_PLLCFGR_DIVQ1EN | \
                                  RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(2)   /* /2 */
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(240)     /* *240 */
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)       /* /2 → 480MHz */
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)       /* /4 → 240MHz */
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)       /* /8 → 120MHz */

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 240)  /* 960MHz */
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)         /* 480MHz */
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)         /* 240MHz */
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 8)         /* 120MHz */

/* PLL2 - Used for SPI123 clock (must be ≤200MHz) ⭐
 *
 *   PLL2_VCO = (8,000,000 / 2) * 96 = 384 MHz
 *
 *   PLL2P = PLL2_VCO/2  = 384 MHz / 2  = 192 MHz  ⭐ SPI123 clock
 */

#define STM32_PLLCFG_PLL2CFG (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
                              RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
                              RCC_PLLCFGR_DIVP2EN | \
                              RCC_PLLCFGR_DIVQ2EN | \
                              RCC_PLLCFGR_DIVR2EN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)   /* /2 */
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(96)      /* *96 */
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)       /* /2 → 192MHz */
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(2)
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(2)

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 96)  /* 384MHz */
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)        /* 192MHz */
#define STM32_PLL2Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2R_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)

/* PLL3 - Not used */
#define STM32_PLLCFG_PLL3CFG 0
#define STM32_PLLCFG_PLL3M   0
#define STM32_PLLCFG_PLL3N   0
#define STM32_PLLCFG_PLL3P   0
#define STM32_PLLCFG_PLL3Q   0
#define STM32_PLLCFG_PLL3R   0
```

**为什么PLL2P=192MHz？**

1. **SPI驱动有硬限制**：
```c
// platforms/nuttx/src/px4/stm/stm32h7/spi/spi.cpp
#if STM32_PCLK_MAX > 200000000
#  error "The SPI123 clock must be ≤200 MHz"
#endif
```

2. **选择192MHz的原因**：
   - 满足≤200MHz限制 ✅
   - 是8MHz的整数倍，时序干净 ✅
   - fmu-v6x验证过的配置 ✅

##### 第4部分：系统时钟分频

```c
/* SYSCLK = PLL1P = 480 MHz
 * CPUCLK = SYSCLK / 1 = 480 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)  /* 不分频 */
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)     /* 480 MHz */
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1) /* 480 MHz */

/* AHB clock (HCLK) is SYSCLK/2 (200 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4
 */

#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_ACLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* 240 MHz */
#define STM32_HCLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* 240 MHz */

/* APB1 clock (PCLK1) is HCLK/2 (120 MHz max) */
#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/2)      /* 120 MHz */

/* APB2 clock (PCLK2) is HCLK/2 (120 MHz max) */
#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/2)      /* 120 MHz */

/* Timer clock frequencies */
/* Timers driven from APB1 will be twice PCLK1 */
#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)  /* 240 MHz */
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)  /* ⭐ TIM5 for HRT */
```

**时钟树总结**：

| 时钟域 | 频率 | 用途 |
|--------|------|------|
| SYSCLK | 480 MHz | CPU时钟 |
| HCLK | 240 MHz | AHB总线 |
| PCLK1 | 120 MHz | APB1外设(I2C/UART) |
| PCLK2 | 120 MHz | APB2外设(SPI) |
| TIM5 | 240 MHz | PX4 HRT |
| SPI123 Kernel | 192 MHz | SPI内核时钟 |

##### 第5部分：外设时钟源选择

```c
/* Kernel Clock Configuration */

/* I2C123 clock source - HSI (16MHz内部RC) */
#define STM32_RCC_D2CCIP2R_I2C123SRC RCC_D2CCIP2R_I2C123SEL_HSI

/* SPI123 clock source - PLL2P (192 MHz) ⭐ 关键! */
#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2

/* USB clock source - HSI48 */
#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_HSI48
```

**⚠️ 最重要的一行**：

```c
#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2
```

**如果不设置这行，SPI时钟会用默认源（可能是PCLK2=120MHz），导致IMU采样率不足！**

##### 第6部分：Flash等待周期

```c
/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *                 240 MHz    3  ⭐ 我们用这个
 *                 480 MHz    4
 *  ------------ ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 4  /* 480MHz需要4个等待周期 */
```

##### 第7部分：引脚定义 - SPI

```c
/* Alternate function pin selections ***************************************/

/* SPI1 - ICM42688P IMU1 (使用CubeMX实际配置的引脚) */

#define GPIO_SPI1_SCK     (GPIO_SPI1_SCK_1 | GPIO_SPEED_50MHz)   /* PA5 - D13 */
#define GPIO_SPI1_MISO    (GPIO_SPI1_MISO_1 | GPIO_SPEED_50MHz)  /* PA6 - D12 */
#define GPIO_SPI1_MOSI    (GPIO_SPI1_MOSI_3 | GPIO_SPEED_50MHz)  /* PD7 - CubeMX Config */

/* SPI3 - ICM42688P IMU2 (使用CubeMX实际配置的引脚) */

#define GPIO_SPI3_SCK     (GPIO_SPI3_SCK_2 | GPIO_SPEED_50MHz)  /* PC10 - D45 */
#define GPIO_SPI3_MISO    (GPIO_SPI3_MISO_2 | GPIO_SPEED_50MHz) /* PC11 - D46 */
#define GPIO_SPI3_MOSI    (GPIO_SPI3_MOSI_3 | GPIO_SPEED_50MHz) /* PB2 - CubeMX Config */
#define GPIO_SPI3_NSS     (GPIO_SPI3_NSS_1 | GPIO_SPEED_50MHz)  /* PA15 */
```

**引脚宏的格式**：

```c
GPIO_SPI1_MOSI_3
      ↑     ↑  ↑
      |     |  └─ 备用功能编号 (从芯片手册查)
      |     └─ 信号名称 (MOSI/MISO/SCK...)
      └─ 外设名称 (SPI1/SPI2...)
```

**如何查备用功能编号？**

1. 打开STM32H743数据手册
2. 查找"Alternate function mapping"表格
3. 找到你想用的GPIO (如PD7)
4. 查看该GPIO的AFx列，找到SPI1_MOSI对应的AF编号

例如PD7：
```
PD7: AF5 = SPI1_MOSI  ← 所以是 GPIO_SPI1_MOSI_3 (第3个可选位置)
```

##### 第8部分：引脚定义 - I2C

```c
/* I2C1 - BMM150 Magnetometer (CubeMX配置) */

#define GPIO_I2C1_SCL     (GPIO_I2C1_SCL_1 | GPIO_SPEED_50MHz) /* PB6 - Morpho */
#define GPIO_I2C1_SDA     (GPIO_I2C1_SDA_2 | GPIO_SPEED_50MHz) /* PB9 - D14 */
```

**⚠️ 血泪教训**：

在最初的board.h文件末尾，我误加了：

```c
// ❌ 文件末尾的错误重复定义
#define GPIO_I2C1_SCL  GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA  GPIO_I2C1_SDA_1  // ← PB7，错误！
```

这导致I2C1_SDA被覆盖为PB7（而不是PB9），BMM150无法通信。

**`make distclean`后又忘了删除这两行，又踩了一次坑！**

**教训**：
- ✅ board.h写完后，全文搜索所有GPIO宏，确保没有重复定义
- ✅ 使用`grep -n "GPIO_I2C1_SDA" board.h`检查

##### 第9部分：引脚定义 - UART

```c
/* USART3 (Nucleo Virtual Console) */

#define GPIO_USART3_RX    (GPIO_USART3_RX_3 | GPIO_SPEED_100MHz) /* PD9 */
#define GPIO_USART3_TX    (GPIO_USART3_TX_3 | GPIO_SPEED_100MHz) /* PD8 */

#define DMAMAP_USART3_RX DMAMAP_DMA12_USART3RX_0
#define DMAMAP_USART3_TX DMAMAP_DMA12_USART3TX_1
```

##### 第10部分：DMA映射

```c
/* DMA **********************************************************************/

#define DMAMAP_SPI1_RX DMAMAP_DMA12_SPI1RX_0 /* DMA1 */
#define DMAMAP_SPI1_TX DMAMAP_DMA12_SPI1TX_0 /* DMA1 */

#define DMAMAP_SPI3_RX DMAMAP_DMA12_SPI3RX_0 /* DMA1 */
#define DMAMAP_SPI3_TX DMAMAP_DMA12_SPI3TX_0 /* DMA1 */
```

**说明**：
- DMA映射用于SPI/UART的DMA传输
- 初期禁用DMA（在defconfig中），稳定后再启用

##### 第11部分：LED定义

```c
/* LED definitions **********************************************************/

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_YELLOW  BOARD_LED2  /* 我们用黄色LED，不是蓝色 */
#define BOARD_LED_RED     BOARD_LED3
```

##### 第12部分：文件结尾

```c
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ST_NUCLEO_H743ZI_FC_NUTTX_CONFIG_INCLUDE_BOARD_H */
```

**⚠️ 重要**：
- 文件末尾**不要再添加任何宏定义**！
- 特别是不要重复定义GPIO宏！

#### 9.3 board.h常见错误总结

| 错误 | 现象 | 解决方案 |
|------|------|----------|
| HSE频率错误 (16MHz vs 8MHz) | 系统不启动或串口乱码 | 查原理图，确认HSE来源 |
| SPI时钟源未设置 | IMU数据采样率低 | 添加`STM32_RCC_D2CCIP1R_SPI123SRC` |
| PLL2P >200MHz | 编译错误 | 调整PLL2配置，确保≤200MHz |
| 引脚GPIO_SPI1_MOSI错误 | SPI通信失败 | 查数据手册，确认AF编号 |
| 重复定义GPIO宏 | I2C/SPI引脚不对 | 全文搜索，删除重复定义 |
| 缺少Flash等待周期 | 随机崩溃 | 根据SYSCLK设置等待周期 |

---

### 10. PX4板级代码实现

完成了NuttX层的配置（defconfig、board.h）后,现在进入PX4应用层。这是让PX4真正"认识"你的硬件的关键一步。

#### 10.1 PX4板级代码的职责分工

先理解一个关键区别：

**NuttX层 (`nuttx-config/`)**:
- 告诉操作系统"硬件长什么样"（时钟、引脚、外设）
- 纯C语言，遵循NuttX规范
- 对应Linux内核的设备树(DT)概念

**PX4层 (`src/`)**:
- 告诉PX4"硬件怎么用"（传感器、外设逻辑抽象）
- C++语言，遵循PX4规范
- 对应Linux用户空间的板级支持包(BSP)

#### 10.2 核心文件一览

`boards/st/nucleo-h743zi-fc/src/`目录结构：

```
src/
├── CMakeLists.txt         # 编译配置
├── board_config.h         # PX4硬件抽象层定义（最重要！）
├── init.cpp               # 板级初始化入口
├── spi.cpp                # SPI设备表
├── i2c.cpp                # I2C总线表
├── led.c                  # LED驱动（可选，我们用自定义模块替代）
└── manifest.c             # 板信息（可选）
```

让我逐一讲解：

---

#### 10.3 board_config.h：PX4的"硬件字典"

这个文件是PX4与硬件的接口契约。所有模块通过它获取硬件信息。

##### 完整的board_config.h示例

```cpp
#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>  /* 引用NuttX的GPIO宏 */

/* ========== 第1部分：LED定义 ========== */
/*
 * PX4使用 "GPIO_nLED_*" 命名约定
 * 配置说明：
 *   GPIO_OUTPUT      - 输出模式
 *   GPIO_PUSHPULL    - 推挽输出
 *   GPIO_SPEED_50MHz - 50MHz速度（LED不需要太快）
 *   GPIO_OUTPUT_CLEAR - 初始状态为低电平
 *   GPIO_PORTx|GPIO_PINy - 引脚定义
 */
#define GPIO_nLED_GREEN  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_nLED_YELLOW (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN7)
#define GPIO_nLED_RED    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)

/* LED极性定义（我们的LED是高电平点亮） */
#define BOARD_LED_ON   1
#define BOARD_LED_OFF  0

/* ========== 第2部分：SPI总线与CS引脚 ========== */
/*
 * PX4_SPI_BUS_SENSORS1/2 - PX4内部用的总线编号
 * 对应NuttX的SPI1/SPI3
 */
#define PX4_SPI_BUS_SENSORS1  1  /* IMU1在SPI1 */
#define GPIO_SPI1_CS_ICM42688P  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN14)

#define PX4_SPI_BUS_SENSORS2  3  /* IMU2在SPI3 */
#define GPIO_SPI3_CS_ICM42688P  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)

/* ========== 第3部分：I2C总线映射 ========== */
#define PX4_I2C_BUS_EXPANSION  1  /* BMM150磁力计在I2C1 */
#define BOARD_NUMBER_I2C_BUSES 2  /* 我们启用了2条I2C总线 */

/* ⚠️ 重要配置：延迟I2C初始化 */
#define BOARD_I2C_LATEINIT 1
/*
 * 为什么需要 BOARD_I2C_LATEINIT?
 * - NuttX默认在启动早期初始化I2C
 * - 但PX4的GPIO初始化在 board_app_initialize() 中
 * - 如果I2C太早初始化，GPIO引脚还未配置好，会触发HardFault
 * - 这是我实战中踩过的血泪坑！
 */

/* ========== 第4部分：串口配置 ========== */
#define BOARD_CONSOLE_UART  3  /* USART3作为调试串口 */
#define BOARD_ENABLE_CONSOLE_BUFFER  /* 启用控制台缓冲 */

/* ========== 第5部分：板信息 ========== */
#define BOARD_NAME "Nucleo-H743ZI-FC"
#define BOARD_HAS_NO_BOOTLOADER  1  /* 没有Bootloader，直接Flash程序 */

/* 硬件版本信息（用于日志识别） */
#define HW_INFO_INIT_PREFIX    "NUCH743FC"
#define BOARD_HAS_VERSIONING   1

/* ========== 第6部分：HRT（高精度定时器）配置 ========== */
/*
 * HRT是PX4的心跳！用于微秒级时间戳
 * 必须使用32位定时器（TIM2/TIM5）
 */
#define HRT_TIMER               5  /* 使用TIM5 */
#define HRT_TIMER_CHANNEL       1  /* 通道1 */

/* ⚠️ 关键：defconfig中 **不要** 启用 CONFIG_STM32H7_TIM5 */
/* PX4会自己接管TIM5，NuttX启用会冲突 */

/* ========== 第7部分：自定义GPIO（可选） ========== */
/* CMOS相机同步信号输入 */
#define GPIO_CMOS_SYNC_LINE   (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN3)
#define GPIO_CMOS_SYNC_FRAME  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN4)

/* ========== 第8部分：GPIO初始化列表 ========== */
/*
 * 这个宏会在 init.cpp 的 board_app_initialize() 中使用
 * 列出所有需要在启动时配置的GPIO
 */
#define PX4_GPIO_INIT_LIST { \
	GPIO_nLED_GREEN, \
	GPIO_nLED_YELLOW, \
	GPIO_nLED_RED, \
	GPIO_SPI1_CS_ICM42688P, \
	GPIO_SPI3_CS_ICM42688P, \
	GPIO_CMOS_SYNC_LINE, \
	GPIO_CMOS_SYNC_FRAME, \
}

/* ========== 第9部分：SPI配置 ========== */
#define BOARD_SPI_BUS_MAX_BUS_ITEMS 2  /* 我们有2条SPI总线 */

/* ========== 第10部分：函数声明 ========== */
__BEGIN_DECLS

extern int board_app_initialize(uintptr_t arg);  /* NuttX调用的初始化入口 */
extern void board_peripheral_reset(int ms);      /* 传感器复位 */
extern void board_control_spi_sensors_power(bool enable_power, int bus_mask);

#include <px4_platform_common/board_common.h>  /* 引入PX4通用定义 */
__END_DECLS
```

**关键要点总结**：

| 配置项 | 作用 | 常见错误 |
|--------|------|----------|
| `GPIO_nLED_*` | LED引脚定义 | 忘记设置初始状态(SET/CLEAR) |
| `PX4_SPI_BUS_SENSORSx` | SPI总线编号 | 与spi.cpp不匹配 |
| `BOARD_I2C_LATEINIT` | 延迟I2C初始化 | **未设置导致HardFault** ⭐ |
| `HRT_TIMER` | 高精度定时器 | 使用了16位定时器(TIM1/3/4) |
| `BOARD_HAS_NO_BOOTLOADER` | 无Bootloader标记 | 未设置导致Flash地址错误 |

---

#### 10.4 init.cpp：板级初始化的"指挥官"

这是NuttX启动后第一个调用的PX4代码。

##### 完整的init.cpp示例（带详细注释）

```cpp
/****************************************************************************
 * boards/st/nucleo-h743zi-fc/src/init.cpp
 * Nucleo-H743ZI Flight Controller Board Initialization
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform/gpio.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include "board_config.h"
#include <px4_platform_common/i2c.h>
#include <px4_platform_common/init.h>
#include <syslog.h>
#include <errno.h>
#include <px4_platform/board_dma_alloc.h>
#include <drivers/drv_board_led.h>
#include <stm32_spi.h>
#include <sched.h>
#include <unistd.h>

/* 外部函数声明 */
extern "C" int board_status_leds_main(int argc, char *argv[]);
void stm32_spiinitialize(void);

/****************************************************************************
 * 名称: px4_init_thread
 *
 * 功能: 异步PX4平台初始化线程
 *
 * 说明: 为什么要用单独线程？
 *       - px4_platform_init() 会挂载ROMFS、启动uORB等耗时操作
 *       - 如果在 board_app_initialize() 中同步执行，会阻塞NSH启动
 *       - 用户连上串口后看不到 "nsh>" 提示符，以为系统死机
 *       - 异步执行后，NSH可以正常启动，用户可以用dmesg查看初始化日志
 ****************************************************************************/
static int px4_init_thread(int argc, char *argv[])
{
    /* 延迟10秒启动，确保NuttX系统完全稳定 */
    syslog(LOG_INFO, "[InitThread] Starting px4_platform_init in 10s...\n");
    usleep(10000000);  /* 10秒 */

    /* PX4平台初始化（挂载ROMFS、启动参数系统、uORB等） */
    syslog(LOG_INFO, "[InitThread] Calling px4_platform_init\n");
    int ret = px4_platform_init();
    syslog(LOG_INFO, "[InitThread] px4_platform_init returned: %d\n", ret);

    /* PX4平台配置（加载启动脚本） */
    int conf = px4_platform_configure();
    syslog(LOG_INFO, "[InitThread] px4_platform_configure returned: %d\n", conf);

    /* 启动LED状态指示模块（可选） */
    const char *argv_leds[] = {"board_status_leds", "start"};
    syslog(LOG_INFO, "[InitThread] starting board_status_leds\n");
    (void)board_status_leds_main(2, (char **)argv_leds);

    return ret;
}

/****************************************************************************
 * 名称: board_peripheral_reset
 *
 * 功能: 复位外部传感器
 *
 * 说明: 标准PX4接口，Commander模块会调用
 ****************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	/* 通过控制CS引脚实现传感器复位 */
	board_control_spi_sensors_power(false, 0xffff);  /* 断电 */
	usleep(ms * 1000);
	board_control_spi_sensors_power(true, 0xffff);   /* 上电 */
}

/****************************************************************************
 * 名称: board_control_spi_sensors_power
 *
 * 功能: 控制SPI传感器电源（通过CS引脚模拟）
 *
 * 说明: 我们的板子没有独立的传感器电源控制引脚
 *       用CS引脚的高低电平模拟电源控制
 ****************************************************************************/
__EXPORT void board_control_spi_sensors_power(bool enable_power, int bus_mask)
{
    /* 根据bus_mask控制对应总线的CS引脚 */
    if (bus_mask & (1 << 0)) {  /* SPI1 */
        px4_arch_gpiowrite(GPIO_SPI1_CS_ICM42688P, enable_power);
    }
    if (bus_mask & (1 << 2)) {  /* SPI3 */
        px4_arch_gpiowrite(GPIO_SPI3_CS_ICM42688P, enable_power);
    }
}

/****************************************************************************
 * 名称: board_app_initialize
 *
 * 功能: 板级应用初始化入口（NuttX启动后调用）
 *
 * 调用时机: NuttX启动完成后，在创建NSH任务前调用
 *
 * ⚠️ 重要: 这个函数必须快速返回！
 *          耗时操作放到 px4_init_thread 中异步执行
 ****************************************************************************/
__EXPORT int board_app_initialize(uintptr_t arg)
{
	(void)arg;

	/* ========== 1. 配置LED GPIO ========== */
	px4_arch_configgpio(GPIO_nLED_GREEN);
	px4_arch_configgpio(GPIO_nLED_YELLOW);
	px4_arch_configgpio(GPIO_nLED_RED);

	/* 初始状态：全部LED熄灭 */
	px4_arch_gpiowrite(GPIO_nLED_GREEN, BOARD_LED_OFF);
	px4_arch_gpiowrite(GPIO_nLED_YELLOW, BOARD_LED_OFF);
	px4_arch_gpiowrite(GPIO_nLED_RED, BOARD_LED_OFF);

	/* ========== 2. 配置SPI CS引脚 ========== */
	px4_arch_configgpio(GPIO_SPI1_CS_ICM42688P);
	px4_arch_gpiowrite(GPIO_SPI1_CS_ICM42688P, true);  /* 高电平=未选中 */

	px4_arch_configgpio(GPIO_SPI3_CS_ICM42688P);
	px4_arch_gpiowrite(GPIO_SPI3_CS_ICM42688P, true);

	/* ========== 3. LED启动指示序列 ========== */
	/* 绿灯闪烁3次，表示板级初始化成功 */
	for (int i = 0; i < 3; i++) {
		px4_arch_gpiowrite(GPIO_nLED_GREEN, BOARD_LED_ON);
		usleep(100000);  /* 100ms */
		px4_arch_gpiowrite(GPIO_nLED_GREEN, BOARD_LED_OFF);
		usleep(100000);
	}

	/* ========== 4. 初始化传感器电源（全部SPI总线上电） ========== */
	board_control_spi_sensors_power(true, 0xffff);

	/* ========== 5. 确保I2C总线表被链接进来 ========== */
	/*
	 * px4_i2c_buses 定义在 i2c.cpp 中
	 * 这里引用一下，防止链接器优化掉
	 */
	(void)px4_i2c_buses;

	/* ========== 6. 初始化SPI ========== */
	stm32_spiinitialize();

	/* ========== 7. 初始化DMA内存池 ========== */
	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[Init] DMA alloc init failed\n");
	}

	/* ========== 8. 启动PX4初始化线程（异步） ========== */
	/*
	 * 参数说明:
	 *   "px4_init" - 任务名称
	 *   100        - 优先级（数值越小优先级越高，100是中等优先级）
	 *   4096       - 栈大小（字节）
	 *   px4_init_thread - 线程入口函数
	 *   NULL       - 传递给线程的参数
	 */
	int taskid = task_create("px4_init", 100, 4096, px4_init_thread, NULL);
	if (taskid < 0) {
		syslog(LOG_ERR, "[Init] Failed to start px4_init task: %d\n", errno);
	} else {
		syslog(LOG_INFO, "[Init] Started px4_init task (id=%d)\n", taskid);
	}

	return OK;  /* 返回OK，NuttX继续启动NSH */
}
```

**init.cpp的设计模式总结**：

这是PX4板级初始化的**标准异步模式**，记住这个模式！

```
NuttX启动
  ↓
board_app_initialize() 被调用
  ↓
[快速执行]
  1. 配置GPIO（LED、CS、自定义引脚）
  2. 初始化SPI/DMA
  3. 创建 px4_init_thread 异步任务
  4. 返回OK
  ↓
NuttX启动NSH（用户可以交互）
  ‖
  ‖ (并行执行)
  ‖
px4_init_thread 异步执行
  1. 延迟10秒（等待系统稳定）
  2. px4_platform_init()（挂载ROMFS、启动uORB）
  3. px4_platform_configure()（执行启动脚本）
  4. 启动应用模块
  ↓
PX4完全启动
```

**为什么这样设计？**

- 如果同步执行`px4_platform_init()`，会阻塞2-5秒
- 用户连上串口后看到黑屏，以为系统死机
- 异步后，NSH立即可用，用户可以用`dmesg`查看初始化日志

**血泪教训**：
> 我第一次实现时，直接在`board_app_initialize()`中同步调用了`px4_platform_init()`。结果串口连上后黑屏30秒才出现提示符，期间完全不知道系统在干什么，还以为是时钟配置错了导致串口波特率不对。后来看了fmu-v6x的代码，才发现要用异步模式。

---

#### 10.5 spi.cpp：SPI设备表

告诉PX4："哪条SPI总线上挂了哪些设备，CS引脚是什么"。

```cpp
#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

/*
 * PX4的SPI设备表
 *
 * 结构说明:
 *   px4_spi_buses[N] - 数组，N = BOARD_SPI_BUS_MAX_BUS_ITEMS
 *   initSPIBus() - 初始化一条SPI总线
 *   initSPIDevice() - 在总线上添加一个设备
 */
const px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
    /* SPI1总线：IMU1 */
    initSPIBus(SPI::Bus::SPI1, {
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortD, GPIO::Pin14}),
    }),

    /* SPI3总线：IMU2 */
    initSPIBus(SPI::Bus::SPI3, {
        initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortA, GPIO::Pin15}),
    }),
};

/* 编译期校验（确保配置正确） */
static constexpr bool unused = validateSPIConfig(px4_spi_buses);
```

**要点说明**：

1. **设备类型**：`DRV_IMU_DEVTYPE_ICM42688P`
   - 定义在`src/drivers/drv_sensor.h`
   - ICM45686使用ICM42688P的兼容类型
   - 驱动通过`-6`参数识别芯片型号

2. **CS引脚**：`SPI::CS{GPIO::PortD, GPIO::Pin14}`
   - 不需要写`GPIO_SPI1_CS_ICM42688P`这样的宏
   - 直接用端口+引脚编号
   - 必须与`board_config.h`中的定义一致

3. **总线编号**：`SPI::Bus::SPI1`对应`PX4_SPI_BUS_SENSORS1=1`

**常见错误**：

| 错误 | 现象 | 解决方案 |
|------|------|----------|
| CS引脚错误 | `icm42688p status`显示"WHO_AM_I错误" | 用示波器检查CS信号 |
| 总线编号错误 | `icm42688p start`找不到设备 | 对照`board_config.h`检查编号 |
| 设备类型错误 | 驱动启动失败 | 查`drv_sensor.h`确认类型名 |

---

#### 10.6 i2c.cpp：I2C总线表

告诉PX4："有哪些I2C总线，是内部总线还是外部总线"。

```cpp
#include <px4_arch/i2c_hw_description.h>
#include <px4_platform_common/i2c.h>

/*
 * PX4的I2C总线表
 *
 * Internal vs External:
 *   - Internal: 板载传感器，固定设备
 *   - External: 外部扩展接口，可热插拔
 *
 * BMM150是焊在板上的，但我们用External模式
 * 原因: 方便调试时用i2cdetect扫描
 */
#pragma GCC visibility push(default)
const px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
    initI2CBusExternal(1),  /* I2C1：BMM150磁力计 */
    initI2CBusExternal(2),  /* I2C2：预留 */
    initI2CBusExternal(3),  /* I2C3：预留 */
    initI2CBusInternal(4),  /* I2C4：内部总线（如果有EEPROM等） */
};
#pragma GCC visibility pop
```

**要点说明**：

1. **总线编号**：必须与`BOARD_NUMBER_I2C_BUSES`一致
2. **External vs Internal**：
   - External：驱动会主动扫描设备地址
   - Internal：需要在启动脚本中明确指定地址
3. **visibility属性**：确保符号不被链接器优化掉

---

#### 10.7 CMakeLists.txt：编译配置

```cmake
px4_add_board(
    PLATFORM nuttx
    VENDOR st
    MODEL nucleo-h743zi-fc
    TOOLCHAIN arm-none-eabi
    ARCHITECTURE cortex-m7
    CONSTRAINED_MEMORY  # 标记为内存受限板（影响优化策略）

    DRIVERS
        # 板级驱动（必需）
        barometer/bmp388  # 如果你有气压计

    MODULES
        # 你的自定义模块会在 default.px4board 中启用

    SERIAL_PORTS
        GPS1:/dev/ttyS0  # 如果你有GPS
        TEL1:/dev/ttyS2  # MAVLink串口（USART3）
)
```

**说明**：
- 这个文件通常可以从参考板复制并修改
- 主要配置在`default.px4board`中，这里只是框架

---

#### 10.8 实战检查清单

完成以上文件后，请逐项检查：

- [ ] `board_config.h`中的`HRT_TIMER`与defconfig中的TIM配置不冲突
- [ ] `GPIO_nLED_*`引脚与原理图一致
- [ ] `PX4_SPI_BUS_SENSORSx`编号与`spi.cpp`一致
- [ ] CS引脚在`board_config.h`和`spi.cpp`中定义一致
- [ ] `BOARD_I2C_LATEINIT`已设置为1
- [ ] `init.cpp`使用了异步`px4_init_thread`模式
- [ ] `i2c.cpp`中的总线数量与`BOARD_NUMBER_I2C_BUSES`一致
- [ ] 所有源文件包含了`board_config.h`

**编译验证**：

```bash
make st_nucleo-h743zi-fc_default
```

如果编译通过，说明PX4板级代码没有语法错误。但能否运行，还需要下一节的模块配置。

---

### 11. 模块与驱动配置

有了板级代码，现在要告诉PX4："启用哪些模块和驱动"。这通过`default.px4board`文件配置。

#### 11.1 .px4board文件的作用

**PX4的配置哲学**：

- **defconfig（NuttX）**：操作系统层配置（内核选项、驱动框架）
- **.px4board（PX4）**：应用层配置（启用哪些模块、传感器驱动）

两者分工明确，不要混淆！

#### 11.2 最小系统配置策略

我们的目标是"双IMU+磁力计融合，输出姿态到MAVLink"，这是一个**最小验证系统**。

**启用原则**：
1. ✅ **只启用必需模块**：传感器、融合、MAVLink
2. ❌ **禁用所有不需要的**：EKF2、导航、控制器、电机输出
3. 🔧 **保留调试工具**：dmesg、i2cdetect、listener、perf

**为什么这样？**

- 减少编译时间（从5分钟降到1分钟）
- 减少Flash占用（从1.5MB降到460KB）
- 减少排查范围（只有5个核心模块）
- 加快启动速度（从30秒降到10秒）

#### 11.3 完整的default.px4board配置

`boards/st/nucleo-h743zi-fc/default.px4board`：

```bash
# ============================================================
# Nucleo-H743ZI 最小飞控系统板级配置
# 目标: 双IMU+磁力计融合，输出姿态到MAVLink
# ============================================================

# === 板级标识 ===
CONFIG_BOARD_TOOLCHAIN="arm-none-eabi"
CONFIG_BOARD_ARCHITECTURE="cortex-m7"
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS2"  # USART3用于MAVLink

# === 系统配置 ===
CONFIG_NUM_MISSION_ITMES_SUPPORTED=100  # 最小值，不使用任务规划

# ============================================================
# 第1部分：驱动配置
# ============================================================

# === IMU驱动 ===
CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P=y
# 说明: ICM45686使用ICM42688P驱动兼容模式
#       启动时需加 -6 参数: icm42688p start -s -b 1 -R 0 -6

# === 磁力计驱动 ===
CONFIG_DRIVERS_MAGNETOMETER_BOSCH_BMM150=y
# CONFIG_DRIVERS_MAGNETOMETER_ISENTEK_IST8310=y  # 备选（如果换传感器）
CONFIG_COMMON_MAGNETOMETER=y  # 通用磁力计支持

# ============================================================
# 第2部分：核心模块配置（最小必需）
# ============================================================

CONFIG_MODULES_SENSORS=y              # 传感器预处理（必需！）
CONFIG_MODULES_MAVLINK=y              # MAVLink通信（必需！）
CONFIG_MODULES_DATAMAN=y              # 数据管理器（提供CONFIG_NUM_MISSION_ITMES_SUPPORTED宏）

# === 自定义模块 ===
CONFIG_MODULES_DUAL_IMU_FUSION=y      # 双IMU融合模块（我们的核心算法）
CONFIG_MODULES_BOARD_STATUS_LEDS=y   # LED状态指示
CONFIG_MODULES_CMOS_SYNC=y           # CMOS相机同步（可选）
CONFIG_MODULES_SENSOR_STUB=y         # 传感器桩模块（测试用）

# ============================================================
# 第3部分：禁用不需要的模块（重要！）
# ============================================================

# === 禁用高级估计器 ===
CONFIG_MODULES_EKF2=n                 # 我们用自定义融合替代EKF2
CONFIG_MODULES_ATTITUDE_ESTIMATOR_Q=n # 禁用姿态估计器

# === 禁用导航与控制 ===
CONFIG_MODULES_COMMANDER=n            # 不需要飞行模式管理
CONFIG_MODULES_NAVIGATOR=n            # 无GPS，不需要导航
CONFIG_MODULES_MC_POS_CONTROL=n       # 不需要位置控制
CONFIG_MODULES_MC_ATT_CONTROL=n       # 不需要姿态控制
CONFIG_MODULES_MC_RATE_CONTROL=n      # 不需要角速度控制
CONFIG_MODULES_FW_POS_CONTROL=n       # 不需要固定翼控制
CONFIG_MODULES_FW_ATT_CONTROL=n
CONFIG_MODULES_LAND_DETECTOR=n        # 不需要着陆检测

# === 禁用电源与电池管理 ===
CONFIG_MODULES_BATTERY_STATUS=n       # 无电池监控
CONFIG_MODULES_LOAD_MON=n             # 不需要负载监控

# === 禁用日志与事件 ===
CONFIG_MODULES_LOGGER=n               # 无SD卡，不需要日志
CONFIG_MODULES_EVENTS=n               # 最小系统不需要事件系统

# ============================================================
# 第4部分：禁用不需要的驱动
# ============================================================

CONFIG_DRIVERS_ADC_BOARD_ADC=n        # 不需要ADC
CONFIG_DRIVERS_BAROMETER=n            # 不需要气压计
CONFIG_DRIVERS_BATT_SMBUS=n           # 不需要SMBus电池
CONFIG_DRIVERS_GPS=n                  # 不需要GPS
CONFIG_DRIVERS_TONE_ALARM=n           # 无蜂鸣器
CONFIG_DRIVERS_PWM_OUT=n              # 无电机输出
CONFIG_DRIVERS_DSHOT=n                # 无DShot ESC
CONFIG_DRIVERS_RC_INPUT=n             # 无遥控器输入
CONFIG_DRIVERS_SAFETY_BUTTON=n        # 无安全按钮
CONFIG_DRIVERS_TELEMETRY=n            # 无数传电台

# ============================================================
# 第5部分：系统命令（调试必需）
# ============================================================

CONFIG_SYSTEMCMDS_DMESG=y             # 内核日志（必需！）
CONFIG_SYSTEMCMDS_I2CDETECT=y         # I2C设备扫描（调试必需！）
CONFIG_SYSTEMCMDS_LED_CONTROL=y       # LED控制
CONFIG_SYSTEMCMDS_NSHTERM=y           # NuttShell终端
CONFIG_SYSTEMCMDS_PARAM=y             # 参数系统
CONFIG_SYSTEMCMDS_PERF=y              # 性能计数器
CONFIG_SYSTEMCMDS_UORB=y              # uORB工具（listener, top）
CONFIG_SYSTEMCMDS_VER=y               # 版本信息
CONFIG_SYSTEMCMDS_TOPIC_LISTENER=y    # 话题监听器（listener命令）

# === 禁用不需要的系统命令（可选） ===
CONFIG_SYSTEMCMDS_TOP=n               # top命令（可用ps替代）
CONFIG_SYSTEMCMDS_HARDFAULT_LOG=n     # HardFault日志（产品化再启用）
```

#### 11.4 配置项详解

##### 11.4.1 驱动配置的技巧

**ICM45686兼容性问题**：

```bash
# ❌ 错误做法: 找ICM45686驱动
# PX4官方仓库根本没有ICM45686驱动！

# ✅ 正确做法: 使用ICM42688P驱动 + -6参数
CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P=y
```

**为什么这样可行？**

- ICM45686和ICM42688P是同一系列（ICM-426xx）
- 寄存器布局兼容，只是性能参数不同
- 驱动通过读取WHO_AM_I寄存器自动识别
- `-6`参数告诉驱动"可能是426xx系列"

**血泪教训**：
> 我一开始在PX4仓库里全局搜索"ICM45686"，一无所获。然后去TDK官网看数据手册，发现它属于ICM-426xx系列。再查px4/fmu-v6x的配置（它用的也是ICM45686），发现用的是`icm42688p`驱动。这才恍然大悟：**传感器驱动不是按型号一一对应，而是按系列共用**！

##### 11.4.2 模块依赖关系

有些模块之间有隐式依赖，必须一起启用：

| 主模块 | 依赖模块 | 原因 |
|--------|----------|------|
| `dual_imu_fusion` | `sensors` | 需要sensors模块预处理IMU数据 |
| `mavlink` | `dataman` | 需要dataman提供任务管理接口 |
| `commander` | `land_detector`, `navigator` | 飞行模式管理需要着陆检测 |
| `mc_att_control` | `commander` | 控制器需要飞行模式状态 |

**我们的最小系统**：

```
dual_imu_fusion ← sensors ← icm42688p驱动
                          ← bmm150驱动
        ↓
   vehicle_attitude
        ↓
   mavlink ← dataman (仅提供编译宏)
```

依赖链非常简单！这就是最小系统的优势。

##### 11.4.3 禁用模块的陷阱

**陷阱1：禁用了commander，导致参数系统不工作**

- **现象**：`param show`显示空列表
- **原因**：commander负责加载参数文件
- **解决**：我们的最小系统没有参数持久化，参数都在代码中硬编码

**陷阱2：禁用了logger，无法排查运行时问题**

- **现象**：系统崩溃后无日志可查
- **解决**：用`dmesg`查看内核日志，用`listener`查看实时数据

**陷阱3：禁用了events，某些模块无法启动**

- **现象**：`commander start`失败
- **原因**：commander依赖events系统
- **解决**：禁用commander，我们不需要它

#### 11.5 编译验证

配置完成后，测试编译：

```bash
make st_nucleo-h743zi-fc_default
```

**预期输出**：

```
[79/79] Linking CXX executable st_nucleo-h743zi-fc_default.elf
Memory region         Used Size  Region Size  %age Used
           flash:      461856 B       2 MB     22.49%  ✅
        AXI_SRAM:       11636 B       512 KB    2.22%  ✅
```

**关键指标**：

- Flash < 30%：✅ 充足，可继续添加功能
- Flash 30-70%：⚠️ 中等，谨慎添加模块
- Flash > 70%：🔴 紧张，需要优化或禁用模块

我们的系统只用了22.49%，非常宽裕！

#### 11.6 常见配置错误排查

| 错误现象 | 可能原因 | 检查方法 |
|----------|----------|----------|
| 编译找不到模块 | `CONFIG_MODULES_XXX=y`拼写错误 | 查`src/modules/CMakeLists.txt` |
| 链接失败"undefined reference" | 模块依赖未启用 | 查看错误日志，补充依赖模块 |
| Flash超过100% | 启用了太多模块 | 禁用不需要的模块 |
| 运行时模块无法启动 | 驱动未启用 | 检查`CONFIG_DRIVERS_XXX` |

---

### 12. 启动脚本编写

配置好模块后，还需要告诉PX4："启动时按什么顺序初始化这些模块"。这通过启动脚本实现。

#### 12.1 PX4启动流程概览

PX4的启动是一个**多阶段的脚本链**：

```
NuttX启动
  ↓
board_app_initialize() (在init.cpp中)
  ↓
px4_platform_init() (挂载ROMFS)
  ↓
px4_platform_configure() (执行启动脚本)
  ↓
/etc/init.d/rcS (主启动脚本)
  ↓
├─ rc.board_sensors (板级传感器初始化) ← 我们要编写的！
├─ rc.sensors (通用传感器处理)
├─ rc.mc_apps (多旋翼应用)
└─ rc.logging (日志系统)
  ↓
PX4完全启动
```

我们要编写的是`rc.board_sensors`，它在启动早期执行，负责：
1. 启动传感器驱动（IMU、磁力计）
2. 启动自定义融合模块
3. 配置MAVLink流

#### 12.2 rc.board_sensors脚本设计

**设计原则**：

1. **顺序很重要**：先启动底层驱动，再启动上层模块
2. **加延迟等待**：驱动初始化需要时间，用`usleep`等待
3. **做错误处理**：如果传感器失败，启动备用模块
4. **简洁明了**：每行一个命令，注释清晰

##### 完整的rc.board_sensors脚本

`boards/st/nucleo-h743zi-fc/init/rc.board_sensors`：

```bash
#!/bin/sh
#
# Nucleo-H743ZI-FC 板级传感器初始化脚本
#
# 启动顺序：
#   1. IMU驱动 (双ICM45686)
#   2. 磁力计驱动 (BMM150)
#   3. 自定义模块 (CMOS同步、双IMU融合)
#   4. 通用传感器模块
#   5. MAVLink流配置
#   6. LED状态指示
#   7. 传感器桩备用模块
#

# ============================================================
# 第1步：启动IMU1 (SPI1)
# ============================================================
# 参数说明:
#   -s         : SPI模式
#   -b 1       : 总线1 (SPI1)
#   -R 0       : 旋转矩阵0 (无旋转)
#   -6         : 使能ICM-426xx系列支持 (兼容ICM45686)
icm42688p start -s -b 1 -R 0 -6

# ============================================================
# 第2步：启动IMU2 (SPI3)
# ============================================================
# 参数说明:
#   -b 3       : 总线3 (SPI3)
#   -R 8       : 旋转矩阵8 (YAW 180度)
#               为什么旋转180度？因为IMU2在板子背面，安装方向相反
icm42688p start -s -b 3 -R 8 -6

# 等待IMU初始化完成（100ms）
usleep 100000

# ============================================================
# 第3步：启动磁力计 (I2C1)
# ============================================================
# 参数说明:
#   -I         : 内部I2C模式 (使用板载I2C)
#   -b 1       : 总线1 (I2C1)
#   -R 0       : 无旋转
bmm150 start -I -b 1 -R 0

# 等待磁力计初始化完成（50ms）
usleep 50000

# ============================================================
# 第4步：启动自定义模块
# ============================================================

# CMOS相机同步模块（可选）
cmos_sync start

# 双IMU融合模块（核心算法！）
dual_imu_fusion start

# ============================================================
# 第5步：启动通用传感器预处理模块
# ============================================================
# sensors模块负责：
#   - 传感器数据校准
#   - 传感器故障检测
#   - 发布预处理后的sensor_combined消息
sensors start

# ============================================================
# 第6步：MAVLink配置
# ============================================================

# 查看MAVLink状态（调试用）
mavlink status

# 启动LED状态指示模块
board_status_leds start

# 配置MAVLink流
# DEBUG流：1Hz，用于调试日志
mavlink stream -u -r 1 -s DEBUG

# ATTITUDE_QUATERNION流：120Hz，姿态四元数输出
mavlink stream -u -r 120 -s ATTITUDE_QUATERNION

# 禁用冗余流（节省带宽）
# mavlink stream -u -r 120 -s HIGHRES_IMU  # 原始IMU数据，不需要
# mavlink stream -u -r 50 -s ATTITUDE      # 与ATTITUDE_QUATERNION冗余

# ============================================================
# 第7步：传感器故障备用方案
# ============================================================
# 如果IMU或磁力计启动失败，启动传感器桩模块
# 传感器桩会发布虚拟数据，防止系统卡死
if icm42688p status | grep -q "Not running"; then
  if bmm150 status | grep -q "Not running"; then
    # 两个传感器都失败了，启动桩模块
    sensor_stub start
  fi
fi

# ============================================================
# 脚本结束
# ============================================================
# 如果看到这里，说明启动脚本执行完毕
# 可以用以下命令检查状态：
#   nsh> dmesg                    # 查看启动日志
#   nsh> icm42688p status         # 查看IMU状态
#   nsh> bmm150 status            # 查看磁力计状态
#   nsh> dual_imu_fusion status   # 查看融合模块状态
#   nsh> mavlink status streams   # 查看MAVLink流配置
```

#### 12.3 脚本要点详解

##### 12.3.1 ICM45686的-6参数

```bash
icm42688p start -s -b 1 -R 0 -6
                            ↑
                        关键参数！
```

**为什么需要-6？**

icm42688p驱动的源码中：

```cpp
// src/drivers/imu/invensense/icm42688p/ICM42688P.cpp
if (external_bus || force_external) {
    _interface = I2C_SPI::instantiate<ICM42686>(I2C_SPI_INTERFACE_SPI, ...);
    //                                  ↑ 使能ICM-426xx系列支持
} else {
    _interface = I2C_SPI::instantiate<ICM42688P>(I2C_SPI_INTERFACE_SPI, ...);
}
```

`-6`参数会设置`force_external=true`，让驱动使用ICM42686实例，该实例兼容整个ICM-426xx系列（包括ICM45686）。

**如果不加-6会怎样？**

- 驱动读取WHO_AM_I寄存器，得到0x91 (ICM45686的ID)
- 但期望的是0x47 (ICM42688P的ID)
- 驱动初始化失败，报错"WHO_AM_I mismatch"

##### 12.3.2 IMU2的旋转矩阵R=8

```bash
icm42688p start -s -b 3 -R 8 -6
                        ↑
                    YAW 180度旋转
```

**为什么旋转180度？**

如果IMU2焊在板子背面（或者朝向相反），它的X/Y轴方向与IMU1相反：

```
     IMU1 (正面)          IMU2 (背面)
         ↑ +Y                 ↓ -Y
         |                    |
    +X ←─┼                    ┼─→ -X
```

如果不做旋转，两个IMU的数据会相反，融合算法会失效！

**旋转矩阵编号**：

| R值 | 旋转 | 说明 |
|-----|------|------|
| 0 | 无旋转 | 默认 |
| 4 | ROLL 180° | 上下翻转 |
| 8 | YAW 180° | 前后翻转 |
| 12 | ROLL 180° + YAW 180° | 上下+前后翻转 |

查看完整旋转矩阵定义：`src/lib/conversion/rotation.h`

##### 12.3.3 usleep延迟的必要性

```bash
icm42688p start -s -b 1 -R 0 -6
icm42688p start -s -b 3 -R 8 -6
usleep 100000  # ← 为什么要延迟？
bmm150 start -I -b 1 -R 0
```

**为什么需要延迟？**

- `icm42688p start`命令是**异步**的，立即返回
- 实际的驱动初始化（SPI通信、读取寄存器、配置传感器）在后台线程中执行
- 如果立即启动下一个驱动，可能导致：
  - SPI总线冲突（两个驱动同时访问）
  - I2C总线繁忙（共享引脚的GPIO初始化未完成）

**延迟多久合适？**

| 传感器类型 | 推荐延迟 | 原因 |
|------------|----------|------|
| SPI IMU | 100ms | SPI通信快，主要是寄存器配置时间 |
| I2C传感器 | 50ms | I2C慢，但初始化简单 |
| 气压计/GPS | 200ms | 需要内部校准 |

##### 12.3.4 传感器桩（sensor_stub）的作用

```bash
if icm42688p status | grep -q "Not running"; then
  if bmm150 status | grep -q "Not running"; then
    sensor_stub start  # ← 最后的保险
  fi
fi
```

**为什么需要桩模块？**

如果传感器驱动全部失败（硬件故障、接线错误），会导致：
- `sensors`模块无数据输入，卡在等待状态
- `dual_imu_fusion`无数据，无法发布vehicle_attitude
- `mavlink`无姿态数据，QGC显示"No attitude"

**sensor_stub的功能**：

发布虚拟的传感器数据，让系统继续运行，方便调试：

```cpp
// 发布虚拟加速度（静止状态）
sensor_accel.x = 0.0f;
sensor_accel.y = 0.0f;
sensor_accel.z = -9.81f;  // 重力加速度

// 发布虚拟陀螺仪（静止）
sensor_gyro.x = 0.0f;
sensor_gyro.y = 0.0f;
sensor_gyro.z = 0.0f;
```

这样你就可以：
- 用`listener vehicle_attitude`看到姿态数据（虽然是假的）
- 用`mavlink`连接QGC，验证通信链路
- 排查问题时不会被"系统卡死"误导

#### 12.4 启动脚本调试技巧

##### 12.4.1 查看启动日志

```bash
nsh> dmesg
```

**期望输出**：

```
[    0.010000] [board] board_app_initialize: Starting
[   10.020000] [px4_init] Starting px4_platform_init in 10s...
[   20.030000] [px4_init] Calling px4_platform_init
[   20.040000] INFO [px4] Mounting ROMFS at /fs/mtd_romfs
[   20.050000] INFO [px4] Executing /fs/mtd_romfs/etc/init.d/rcS
[   20.100000] INFO [icm42688p] Starting on SPI1
[   20.150000] INFO [icm42688p] WHO_AM_I: 0x91 (ICM45686)
[   20.200000] INFO [icm42688p] Starting on SPI3
[   20.250000] INFO [icm42688p] WHO_AM_I: 0x91 (ICM45686)
[   20.350000] INFO [bmm150] Starting on I2C1
[   20.400000] INFO [bmm150] Chip ID: 0x32
[   20.450000] INFO [dual_imu_fusion] Started successfully
```

**如果看到ERROR**：

```
ERROR [icm42688p] SPI1: WHO_AM_I mismatch  ← 忘记加-6参数
ERROR [bmm150] I2C1: Device not found       ← I2C接线错误或地址错误
ERROR [dual_imu_fusion] No IMU data         ← IMU驱动未启动
```

##### 12.4.2 手动执行启动脚本

如果系统启动失败，可以在NSH中手动执行脚本调试：

```bash
nsh> sh /fs/mtd_romfs/etc/init.d/rc.board_sensors
```

这会逐行执行脚本，你能看到每一步的输出。

##### 12.4.3 分段测试

把脚本拆成小段，逐段测试：

```bash
# 第1段：只测试IMU1
nsh> icm42688p start -s -b 1 -R 0 -6
nsh> usleep 100000
nsh> icm42688p status
# 如果成功，继续下一段

# 第2段：测试IMU2
nsh> icm42688p start -s -b 3 -R 8 -6
nsh> usleep 100000
nsh> icm42688p status

# 第3段：测试磁力计
nsh> bmm150 start -I -b 1 -R 0
nsh> usleep 50000
nsh> bmm150 status
```

#### 12.5 常见启动脚本错误

| 错误现象 | 可能原因 | 解决方案 |
|----------|----------|----------|
| 脚本执行到一半卡死 | 某个命令阻塞（如传感器初始化超时） | 在命令后加`&`后台执行 |
| IMU启动失败"WHO_AM_I错误" | 忘记加`-6`参数 | 检查启动命令 |
| 磁力计扫描不到 | I2C引脚错误或总线编号错误 | 用`i2cdetect -b 1`扫描 |
| 融合模块无数据 | IMU或磁力计未启动 | 检查`dmesg`确认驱动状态 |
| MAVLink流未生效 | 流配置在模块启动前执行 | 把流配置放到脚本末尾 |

#### 12.6 启动脚本的高级技巧

##### 技巧1：条件启动

```bash
# 只在有SD卡时启动日志
if [ -d /fs/microsd ]; then
    logger start -b 1000  # 1MB缓冲
fi
```

##### 技巧2：多种传感器备选

```bash
# 优先启动BMM150，失败则启动IST8310
if ! bmm150 start -I -b 1 -R 0; then
    ist8310 start -I -b 1 -R 0
fi
```

##### 技巧3：日志打点

```bash
# 在关键步骤打印日志，方便排查
echo "[rcS] Starting IMU1..."
icm42688p start -s -b 1 -R 0 -6
echo "[rcS] IMU1 started"
```

---

**至此，第三部分"核心实战"已全部完成！**

现在你已经掌握了：
- ✅ NuttX配置（defconfig、board.h）
- ✅ PX4板级代码（board_config.h、init.cpp、spi.cpp、i2c.cpp）
- ✅ 模块配置（default.px4board）
- ✅ 启动脚本（rc.board_sensors）

下一步，我们进入**第四部分：验证与调试**，编译固件、烧录板子、运行测试！

---

