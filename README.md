# unitree_go2

Unitree Go2 ROS 2 工作空间，包含机器人控制、前置视频、局部代价地图、路径规划以及 RealSense 和 Unitree ROS 2 源码。

## 目录

- `src/`：ROS 2 功能包及第三方源码，各包的使用方法见对应 README。
- `scripts/`：辅助脚本。
- `docs/`：方案文档和 [Git 使用说明](docs/git_workflow.md)。

`build/`、`install/`、`log/` 是本地生成目录，不提交到 Git。克隆后需要在已配置 ROS 2 和依赖的环境中重新构建。

## 获取工程

```bash
git clone https://github.com/Yangqinhe/unitree_go2.git
cd unitree_go2
```

第三方源码作为普通目录一并保存，包含当前工作空间的本地修改，无需初始化 Git 子模块。原第三方项目的许可证保留在各自目录中。

## 日常同步

开始工作前，在工作区干净时执行：

```bash
git pull --ff-only
```

修改完成后执行：

```bash
git status
git add .
git diff --cached --stat
git commit -m "说明本次修改"
git push
```
