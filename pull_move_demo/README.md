# pull_move 力矩拖拽控制

肩/脖子力矩 → `/cmd_vel` 映射，仅在 FSM 行走模式下发速度。

## 源码运行

```bash
cd ~/Bird_ws/pull_move_demo
./run_torque_bridge.sh
```

## 打发布包

```bash
./scripts/build_release.sh
# → ../pull_move_0717_bag/ 与 ../pull_move_0717_bag.tar.gz
```

## 开机自启

```bash
sudo ./install-autostart.sh
```
