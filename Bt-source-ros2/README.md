# Bt-source-ros2（ROS2 源码树）

由 `Bt_voice_pull_bag` 复制而来，**ROS 桥只适配 ROS2 Foxy**。小程序 BLE 协议不变；板端优先调用量产 `hightorque_controller` 服务切到 **AMP**，摇杆仍发 `sensor_msgs/Joy` 到 `/joy`。

## 版本

当前版本见 `VERSION`：

```
Bt-source-ros2-1.0.2-260825
```

ROS1 原版仍在 `~/Bird_ws/Bt_voice_pull_bag`，不要和本目录同时安装 `bird-ble.service`。

## 与 ROS1 / 旧 BFM 桥的差异

| 小程序指令 | ROS1（原版） | 本目录 ROS2（量产 AMP） |
| --- | --- | --- |
| `X,Y,Z` 摇杆 | `/cmd_vel` | `/joy` 左摇杆平移 + 右摇杆转向 |
| `GAIT ON` | `/joy_msg` `LT+RT+LB` | 确认 `default_bt` → `SwitchPolicy("amp")` 并等待 `current_policy` → `change_state(toggle_policy)` 进 RUNNING；失败再 `/joy` LT+RT+LB |
| `GAIT OFF` | 同上切 standby | `toggle_policy` 回 STANDBY，摇杆回中 |
| `LT+RT+start` | 起立 | `change_state(standing)`，失败则 `/joy` LT+RT+START 1s |
| `LT+RT+RB` | 坐下 | `change_state(siting)`，失败则 `/joy` LT+RT+RB 1s |
| `LT ON` | lt 加速 | `/joy` **LT 扳机** `axes[2]=-1` |
| `LT+RT+B` / `M_protect` | 卸力 | `/hightorque_controller/change_fsm_state` `protect`，失败则 `/joy` LT+RT |
| `M_init` | FSM | `change_fsm_state` `init`（不连按 LT+RT+B） |
| `M_resetzero` | FSM 循环 | `init` → `prev` → `confirm` |
| 挥手踢球等 | `custom_action` | 组合键叠加到 `/joy`（动作库尚未按 ROS2 重做） |
| `MP ON/OFF` | `livelybot_power` | `hightorque_power/PowerSwitch` |

禁止切换到 `bfm` / `amp_lower`（忽略 `LT+Y` / `LT+RS` 等键位），也不会启动 `instinct_onboard`。

量产接口：

```text
/hightorque_controller/change_fsm_state   # hightorque_msgs/srv/ChangeState
/hightorque_controller/change_state       # 上层 STANDING/STANDBY/RUNNING
/hightorque_controller/switch_policy      # policy_name: amp
/hightorque_controller/state              # 确认 current_mode/state/policy
/fsm_state                                # 确认底层 FSM 数值
```

## 一键安装

```bash
cd ~/Bird_ws/Bt-source-ros2
sudo ./install.sh
```

安装会：

- 覆盖现有 `bird-ble` systemd 单元
- 写入 `~/colcon_ws/ros2_start.sh` 并启用 XFCE 自启 `Pi_plus_ros2.desktop`
- 关闭 ROS1 步态自启 `Pi_plus_start.desktop`（`~/sim2real/sim2real.sh`）

ROS2 工作空间默认 `/home/hightorque/colcon_ws`。若还没有 `sim2real_master` 包，自启会退化为 `pi_plus_rknn.launch.py`（硬件 bringup）；算法包装上后会走 `robot_bringup.launch.py`（默认策略 `amp`）。

## 手动启动

```bash
# 算法栈（有 sim2real_master 时）
source /opt/ros/foxy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch hightorque_bringup robot_bringup.launch.py

# BLE 桥
cd ~/Bird_ws/Bt-source-ros2/ble/app
./start.sh
```

实体手柄已 remap 到 `joy_input`，小程序独占 `/joy`。

## 常用命令

```bash
sudo systemctl status bird-ble
journalctl -u bird-ble -f
cd ble/app && ./scripts/check.sh
```
