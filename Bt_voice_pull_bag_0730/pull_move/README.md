# pull_move 力矩拖拽控制安装包 (pull_move_0717_bag)

肩/脖子力矩 → `/cmd_vel` 映射桥。仅在 **FSM=行走模式(EXEC_DEFAULT)** 时发布速度。

本包为**发布运行时**（Python 字节码，不含 .py 源码）。

## 一键安装

```bash
cd pull_move_0717_bag
sudo ./install.sh
```

## 前置条件

- Ubuntu 20.04 + ROS Noetic
- 已编译 `sim2real` 工作空间
- 机器人主控已运行 `roscore` / `sim2real_master`

## 目录结构

```
pull_move_0717_bag/
  install.sh / uninstall.sh
  README.md / VERSION
  app/                    # 运行时（字节码 + 脚本）
```

## 常用命令

```bash
sudo systemctl status torque-cmd-vel
journalctl -u torque-cmd-vel -f
cd app && ./run_torque_bridge.sh          # 前台调试
cd app && ./run_torque_bridge.sh --dry-run
sudo ./uninstall.sh
```

## 可选参数

编辑 `/etc/default/torque-cmd-vel`：

```bash
EXTRA_ARGS=(--no-arms)    # 仅脖子
EXTRA_ARGS=(--side right)
```

然后 `sudo systemctl restart torque-cmd-vel`
