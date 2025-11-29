# SPI时钟配置修复总结

## ❌ 问题现象

编译到[34/533]时失败:
```
chip/stm32_spi.c:191:6: error: #error Not supported SPI123 frequency
```

## 🔍 根本原因

### NuttX SPI驱动限制
```c
// platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_spi.c:190-191
#if SPI123_KERNEL_CLOCK_FREQ > 200000000
#  error Not supported SPI123 frequency
#endif
```

**要求**: SPI123时钟频率必须 ≤ 200 MHz

### 错误配置 (修复前)

```c
// board.h - 错误配置
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(240)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)

// 计算:
// PLL1_VCO = (8MHz / 2) * 240 = 960 MHz
// PLL1Q = 960 MHz / 4 = 240 MHz  ❌ 超过200MHz限制!

// SPI123使用PLL1Q作为时钟源
#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL1
```

## ✅ 正确配置 (修复后)

### 1. 重新配置PLL2

```c
/* PLL2 - Used for SPI123 clock (must be ≤200MHz)
 *
 *   PLL2_VCO = (8,000,000 / 2) * 96 = 384 MHz
 *
 *   PLL2P = PLL2_VCO/2  = 384 MHz / 2  = 192 MHz (SPI123 clock)
 *   PLL2Q = PLL2_VCO/2  = 384 MHz / 2  = 192 MHz
 *   PLL2R = PLL2_VCO/2  = 384 MHz / 2  = 192 MHz
 */

#define STM32_PLLCFG_PLL2CFG (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
                              RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
                              RCC_PLLCFGR_DIVP2EN | \
                              RCC_PLLCFGR_DIVQ2EN | \
                              RCC_PLLCFGR_DIVR2EN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(96)   // 96 instead of 200
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)     // 384/2 = 192MHz
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(2)
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(2)

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 96)
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)  // 192 MHz ✅
```

### 2. 修改SPI123时钟源

```c
/* SPI123 clock source - PLL2P (192 MHz, within 200MHz limit) */
#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2
```

**从**: `RCC_D2CCIP1R_SPI123SEL_PLL1` (240 MHz ❌)
**改为**: `RCC_D2CCIP1R_SPI123SEL_PLL2` (192 MHz ✅)

## 📊 参考: fmu-v6x配置

fmu-v6x使用相同的策略 (虽然HSE频率不同):

```c
// fmu-v6x board.h
#define STM32_BOARD_XTAL        16000000ul  // 16 MHz HSE

// PLL2配置
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(4)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(48)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(2)

// 计算:
// VCO2 = 16MHz / 4 * 48 = 192 MHz
// PLL2P = 192 MHz / 2 = 96 MHz ✅

// SPI时钟源
#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2
```

## 🎯 关键经验教训

### 1. **参考对象选择错误**

❌ **错误**: 参考NuttX官方`nucleo-h743zi/include/board.h`
- 该配置针对通用应用
- 未考虑PX4飞控的外设时钟需求
- 使用PLL1Q作为SPI时钟源,导致超限

✅ **正确**: 参考同系列芯片的**PX4飞控板配置**
- fmu-v6x (STM32H753) ← 最佳参考
- fmu-v5 (STM32F765)
- 这些板已经解决了时钟分配问题

### 2. **时钟树规划原则**

PX4飞控板的时钟分配策略:

| PLL | 用途 | 频率要求 |
|-----|------|----------|
| PLL1P | 系统时钟(SYSCLK) | 最大480MHz |
| PLL1Q | USB/SDMMC等高速外设 | 48/120/240MHz |
| **PLL2P** | **SPI123** | **≤200MHz** ✅ |
| PLL3 | 其他外设 | 按需配置 |

**核心原则**:
- SYSCLK使用PLL1P追求最高性能
- **SPI必须使用PLL2P**,避免超限
- 不同外设使用不同PLL,避免相互干扰

### 3. **NuttX驱动限制检查**

在配置时钟前,必须检查NuttX驱动源码中的限制:

```bash
# 检查SPI驱动时钟限制
grep -n "error.*frequency" platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/stm32_spi.c

# 检查其他外设限制
grep -rn "#error.*frequency" platforms/nuttx/NuttX/nuttx/arch/arm/src/stm32h7/
```

### 4. **为什么错误总在编译中期出现?**

编译顺序:
1. [0-30/533] CMake配置 + PX4代码编译 ✅
2. [31-67/533] **NuttX内核编译** ❌ ← SPI驱动编译时检查时钟配置
3. [68-533/533] 链接和打包

**NuttX驱动的`#error`检查在编译时触发**,不是配置时,所以错误出现在编译中期。

## 📁 修改文件清单

### 唯一修改文件

**boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h**

修改内容:
1. 第122-145行: PLL2配置 (PLLN: 200→96, 增加Q/R输出)
2. 第234-236行: SPI123时钟源 (PLL1→PLL2)

## ✅ 验证方法

### 编译前验证

```bash
# 检查board.h中的SPI时钟源
grep "SPI123SRC" boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h
# 预期输出: RCC_D2CCIP1R_SPI123SEL_PLL2

# 检查PLL2P频率计算
grep -A 3 "PLL2P_FREQUENCY" boards/st/nucleo-h743zi-fc/nuttx-config/include/board.h
# 预期: (384 MHz / 2) = 192 MHz
```

### 编译验证

```bash
make clean
make st_nucleo-h743zi-fc_default -j$(nproc)
```

**预期结果**: 编译通过,不再出现"Not supported SPI123 frequency"错误

## 🔗 相关文档

- [px4_custom_board_complete_guide.md](px4_custom_board_complete_guide.md) - 自定义板开发完整指南
- [nuttx_stm32h7_driver_support.md](nuttx_stm32h7_driver_support.md) - NuttX驱动支持分析
- fmu-v6x参考配置: `boards/px4/fmu-v6x/nuttx-config/include/board.h`

---

**修复日期**: 2025-11-28
**修复人**: Claude
**状态**: ✅ 已修复,待编译验证
