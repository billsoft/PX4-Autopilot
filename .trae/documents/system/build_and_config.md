# 构建与板级配置

构建系统与板级裁剪决定了平台、驱动与模块的组合方式。

```mermaid
flowchart LR
  A[CMakeLists.txt] --> B[cmake/px4_config.cmake]
  B --> C[Kconfig]
  C --> D[boards/*/*.px4board]
  D --> E[模块/驱动裁剪]
```

- 顶层入口：`CMakeLists.txt:102-141, 171-194` 设置版本、属性与配置注入。
- Kconfig+板级清单：`boards/<vendor>/<board>/default.px4board` 选择 `CONFIG_MODULES_*` 与驱动。
- 平台工具链注入：`platforms/*/cmake/px4_impl_os.cmake`。
- Python 构建辅助与版本工具：`Tools/px4.py:96-101`。

实践建议：
- 新板支持：从已有板目录拷贝最小集合，完成 `src/*` Bring-up 与 `.px4board` 最小模块集。
- 编译目标：使用 `PX4_PLATFORM` 切换 `nuttx/posix/qurt`，确保关键模块在目标平台可用。

