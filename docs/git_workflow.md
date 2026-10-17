# Git 提交和拉取

## 当前工作空间

PowerShell 中进入目录：

```powershell
cd E:\unitree_go2_ws
```

远程仓库为 `https://github.com/Yangqinhe/unitree_go2.git`，日常使用 `main` 分支。

## 每次开始工作

```bash
git status
git pull --ff-only
```

建议先同步，再开始修改。`--ff-only` 在本地和远程分叉时会停止，避免自动产生合并提交。

## 修改后提交并上传

```bash
git status
git diff
git add .
git diff --cached --stat
git commit -m "添加或修复了什么"
git push
```

`git add .` 暂存当前目录内的新增、修改和删除；在提交前检查暂存内容。`commit` 保存到本地，`push` 上传到 GitHub。没有修改时不需要提交。

## 有未提交修改但需要拉取

```bash
git stash push -u -m "拉取前暂存"
git pull --ff-only
git stash pop
```

`-u` 同时暂存未跟踪文件，但不包括被忽略的构建输出。恢复时若冲突，手动解决后再提交；确认恢复完整前不要删除 stash。

## 推送因远程更新被拒绝

在本地修改已提交、工作区干净的情况下：

```bash
git fetch origin
git rebase origin/main
git push
```

若出现冲突，编辑冲突文件、执行 `git add <文件路径>`，再执行 `git rebase --continue`。需要取消此次变基时使用 `git rebase --abort`。不要用强制推送覆盖远程修改。

## 在另一台电脑首次获取

```bash
git clone https://github.com/Yangqinhe/unitree_go2.git
cd unitree_go2
```

克隆只获取源码，ROS 2、设备 SDK 和其他系统依赖需另行配置，再按各功能包 README 构建运行。

## 第三方源码

`src/unitree_ros2` 和 `src/realsense-ros` 以源码快照方式管理。修改这两个目录后，直接在工作空间根目录提交即可。

初始化时，两者原有的 Git 元数据备份在本机 `.git/vendor-git-backups/` 中，不上传 GitHub。该备份用于保留原仓库历史；不要直接搬回源码目录，否则会再次形成嵌套仓库。
