# 脚本与工具

- 版本与维护工具：`Tools/px4.py:60-87, 96-101`。
- 消息文档生成：`Tools/msg/generate_msg_docs.py`。
- 仿真/测试脚本：`platforms/posix/*`、`launch/px4.launch` 及 `Tools/*` 下辅助脚本。

作为上位机开发者的建议：
- 从 `msg/*.msg` 与 `uORB` 流入手，使用仿真环境（SITL）快速验证上层算法与数据路径。
- 利用 `src/systemcmds/*` 的系统指令进行在线调试（如 `param/top/uorb/work_queue`）。

