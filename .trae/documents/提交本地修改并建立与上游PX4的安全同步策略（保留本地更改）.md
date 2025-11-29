## 一次性提交到GitHub
- 检查与提交：
  - `git status -s`
  - `git add -A`
  - `git commit -m "nucleo-h743zi-fc: minimal FC + fusion + sync + MAVLink 120Hz"`
- 远程与推送：
  - `git remote -v`（若无origin则：`git remote add origin https://github.com/<your-account>/PX4-Autopilot.git`）
  - `git branch -M main`
  - `git push -u origin main`

## 建立与上游PX4的同步（保留本地改动）
- 添加上游：
  - `git remote add upstream https://github.com/PX4/PX4-Autopilot.git`
  - `git fetch upstream`
- 分支策略：
  - 本地主线：`main`（你的fork主分支）
  - 上游追踪：创建 `upstream-main`（仅跟踪上游）→ `git checkout -b upstream-main upstream/main`
  - 集成分支：`integration`（用来融合上游更新与本地改动）→ `git checkout -b integration main`
- 同步循环（每次上游更新时执行）：
  1) 更新上游快照：`git fetch upstream && git checkout upstream-main && git reset --hard upstream/main`
  2) 回到集成：`git checkout integration`
  3) 融合上游：优先选择“重放本地改动到上游之上”以减少冲突
     - 推荐：`git rebase upstream-main`（如有复杂历史可用 `git rebase --rebase-merges upstream-main`）
     - 若rebase冲突：逐文件解决（保留本地板级与模块改动；公共层尽量跟随上游），`git add <files>` 后 `git rebase --continue`
     - 如rebase不合适：改用`git merge upstream-main`（可用策略：`-X theirs`只在公共层使用上游版本；板目录使用你本地）
  4) 成功后更新主线：`git checkout main && git merge --ff-only integration && git push origin main`

## 冲突处理准则（保留本地，减少未来冲突）
- 尽量将改动集中在板目录与自建模块：
  - `boards/st/nucleo-h743zi-fc/**`
  - `src/modules/cmos_sync/**`、`src/modules/dual_imu_fusion/**`
  - `boards/st/nucleo-h743zi-fc/init/rc.board_sensors`
  - `boards/st/nucleo-h743zi-fc/default.px4board`
- 公共层（如 `platforms/common/*`、`platforms/nuttx/*`）尽量避免长期本地改动；如必须，严格跟随上游接口并以最小Diff实现（便于rebase/merge）。
- 发生冲突时：
  - 板级/模块文件：优先保留本地（ours）
  - 通用平台与驱动公共部分：优先上游（theirs），再补本地兼容性调整
  - 使用 `git rerere` 记录冲突解决（`git config rerere.enabled true`）以减少后续重复劳动

## 自动化同步（可选）
- GitHub Actions 定时同步（保留本地改动）：
  - 在你的仓库添加 `.github/workflows/sync-upstream.yml`：
    - 每日/每周执行：`actions/checkout`（你的fork）→ 添加 upstream → fetch → 在 `integration` 分支 `rebase upstream/main` → 推送到 `integration` → fast-forward合并到 `main`（安全检查通过才执行）
  - 或使用GitHub的“Sync fork”功能手动同步→随后在本地执行上述rebase/merge策略

## 子模块与依赖
- 每次同步或构建前：
  - `git submodule sync && git submodule update --init --recursive`
  - 上游构建脚本会自动检测并提示更新（`Tools/check_submodules.sh`）

## 验证与保护
- 集成后运行CI/本地构建：`python Tools/px4.py build st_nucleo-h743zi-fc` 与 `make st_nucleo-h743zi-fc_default -j4`
- 保护分支：将 `main` 设置为保护分支，只允许通过 `integration` 合并（防止直接覆盖本地改动）

## 备注
- 所有推送需使用你GitHub账户（或PAT）权限；若需要我代为执行，请提供远程与凭据或在你的环境中执行上述命令。
- 如你希望长期维护补丁队列以便与上游差异清晰，可考虑使用 `git format-patch`（本地改动 → 上游之上）并采用 `git am` 应用；这在大版本升级时更高效。