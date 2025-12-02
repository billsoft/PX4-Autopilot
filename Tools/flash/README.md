# 固件烧写脚本使用说明

## 前置
- 安装 STM32CubeProgrammer（包含 `STM32_Programmer_CLI`）。
- 若未安装到默认路径，可设置环境变量 `STM32_CLI` 指向 CLI 可执行文件。
- 默认 CLI 路径：`C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe`
- 验证 CLI：在 PowerShell 执行 `"C:\\Program Files\\STMicroelectronics\\STM32Cube\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe" -h`
- 若 CLI 未找到：请将其目录加入 `PATH` 或设置 `STM32_CLI` 环境变量。
- 下载地址：`https://www.st.com/en/development-tools/stm32cubeprog.html`

## 生成固件
- 在项目根目录执行 WSL 构建以生成产物：
  - `wsl bash -lc "cd /mnt/d/code/px4/PX4-Autopilot && make st_nucleo-h743zi-fc_default -j4"`
- 产物位于 `build/st_nucleo-h743zi-fc_default/`：`.bin`、`.elf`、`.px4`。

## 烧写
- 使用默认目标与 `.bin` 到 `0x08000000`：
  - `python tools/flash/flash_fw.py`
- 指定文件或目标：
  - `python tools/flash/flash_fw.py --file build/st_nucleo-h743zi-fc_default/st_nucleo-h743zi-fc_default.bin`
  - `python tools/flash/flash_fw.py --target st_nucleo-h743zi-fc_default --image elf`
- 指定 CLI 路径：
  - `python tools/flash/flash_fw.py --cli "C:\\Program Files\\STMicroelectronics\\STM32Cube\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe"`
- 修改地址或擦除策略：
  - `python tools/flash/flash_fw.py --address 0x08000000 --erase all`

## 说明
- `.bin` 适用于无 Bootloader 的直接烧写；`.elf` 可由 CLI自动解析段地址；`.px4` 需配合 PX4 Bootloader 及 QGC。
- 脚本通过 ST-LINK 以 SWD 连接，执行擦除、写入、校验与复位。
 - 常用 CLI 选项：`-c port=SWD`、`-e all`、`-w <file> [address]`、`-v`、`-rst`；读取上传：`--upload <start> <size> <file>`。

## GUI
- 位置：`tools/flash/gui/flash_gui.py`
- 运行：`python tools/flash/gui/flash_gui.py`
- 功能：
  - 自动探测 CLI 与固件产物，支持手动浏览
  - 设置目标、镜像类型、地址、擦除策略
  - 开始烧写并显示实时日志与状态
  - 打开构建目录；从 ELF 导出 BIN（如装有 `arm-none-eabi-objcopy`）

## 排错
- CLI 未找到：安装 STM32CubeProgrammer 并配置 `PATH` 或 `STM32_CLI`
- ST-LINK 无法连接：安装/更新 ST-LINK 驱动；确认 USB 线缆与供电正常
- 地址错误：写 `bin` 时需指定 Flash 起始地址（如 `0x08000000`）


