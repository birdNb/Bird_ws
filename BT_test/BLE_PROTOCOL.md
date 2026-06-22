# Bird BLE 通讯协议

> 全链路 **UTF-8 纯文本**，单条不含换行，经 **FFE2 notify** / **FFE1 write** 传输。

| UUID | 方向 | 用途 |
|------|------|------|
| FFE0 | — | 服务 |
| FFE1 | 小程序 → 机器人 | 控制指令 |
| FFE2 | 机器人 → 小程序 | ACK + 状态遥测 |

## 终端日志颜色

| 颜色 | 标记 | 含义 |
|------|------|------|
| **红** | `RX` | 收到小程序数据（FFE1 写入） |
| **绿** | `TX` | 板子发出数据（FFE2 notify） |
| 白 | — | 链路/系统信息 |
| 黄 | `WARN` | 警告 |

---

## 一、上行：小程序 → 机器人（FFE1 write）

### 1.1 摇杆

| 字段 | 范围 | 说明 |
|------|------|------|
| X | ±1.00 | 前后（前 +） |
| Y | ±1.00 | 左右（右 +） |
| Z | ±1.50 | 转向（右转 +） |

```text
X:{x},Y:{y},Z:{z}
X:{x},Y:{y},Z:{z},N:{seq}    # 可选序号，20Hz 保活
```

- 频率 **20 Hz**，`writeNoResponse`
- 摇杆无 ACK

### 1.2 模式

| 指令 | 说明 |
|------|------|
| `M_default` | 默认（进遥控页首发） |
| `M_init` | 初始化 |
| `M_protect` | 保护 |
| `M_resetzero` | 调零 |
| `M_tech` | 示教 |

### 1.3 组合键

| 功能 | 指令 |
|------|------|
| 起立 | `LT+RT+start` |
| 蹲下 | `LT+RT+RB` |
| 挥双手 | `RT+A` |
| 挥单手 | `RT+X` |
| 步态 | `LT+RT+LB` |
| 卸力 | `LT+RT+B` |

### 1.4 脖子控制

| 指令 | 说明 |
|------|------|
| `P{n}Y{m}` | pitch / yaw 步进偏移（整数，可带 `+`/`-`） |
| `neck0` | 脖子回中（脖子切换按钮） |

每步 **10°**：

| 轴 | `+` | `-` |
|----|-----|-----|
| P（pitch） | 往上 | 往下 |
| Y（yaw） | 往右 | 往左 |

示例：

```text
P1Y0      # 抬头 10°
P-1Y0     # 低头 10°
P0Y1      # 右转 10°
P0Y-1     # 左转 10°
P0Y0      # 无偏移
neck0     # 回中
```

板端发布 `/pi_plus_absolute`（`head_yaw_joint` / `head_pitch_joint`）。

### 1.5 人脸跟踪（locate_face）

| 指令 | 说明 |
|------|------|
| `locate_face ON` | 启动 `locate_face_cpp`（默认后台；`--gui` 才显示预览） |
| `locate_face OFF` | 停止头追进程 |

视觉伺服脖子跟随；与手动脖子步进共用 `/pi_plus_absolute`，建议二选一使用。

### 1.6 手势控制（HI）

| 指令 | 说明 |
|------|------|
| `HI ON` | 启动 `hand_identify_cpp/start.sh --no-joy`（默认 `--gesture_action`） |
| `HI OFF` | 停止手势控制（`vision_controller`） |

### 1.7 电机电源（MP）

| 指令 | 小程序操作 | 说明 |
|------|------------|------|
| `MP ON` | **长按** | 接通电机供电（功率板 `power_switch=1`） |
| `MP OFF` | **点击** | 断开电机供电（功率板 `power_switch=0`） |

板端发布 ROS `/power_switch_control`（`livelybot_power/Power_switch`，`control_switch=1`）。

### 1.8 音量调节（V）

| 指令 | 说明 |
|------|------|
| `V {0-100}` | 将系统播放音量设为指定百分比 |

用户在小程控件里调节完音量后发送，例如调到 **10%** 时：

```text
V 10
```

- 范围自动钳位到 `0`–`100`
- 板端通过 PulseAudio `amixer -D pulse sset Master {n}%` 生效
- 有 ACK：`ACK:V 10`

### 1.9 指令 ACK（FFE2 notify）

```text
ACK:{原文}
```

模式、动作、脖子、locate_face、HI、MP、V 有 ACK；摇杆无。

---

## 二、下行：机器人 → 小程序（FFE2 notify）

订阅 FFE2 后，板端主动推送状态。

### 2.1 局域网 IP

```text
IP:19.11
```

| 规则 | 说明 |
|------|------|
| 格式 | `IP:` + IP 末两段 |
| 示例 | `192.168.19.11` → `IP:19.11` |
| 时机 | 订阅后立即推一次；局域网 IP 变化时再推 |

### 2.2 电量

```text
pwr:83
```

| 规则 | 说明 |
|------|------|
| 格式 | `pwr:` + 0~100 整数（实际剩余电量，如 83% → `pwr:83`） |
| 含义 | `pwr:83` = 剩余 83% 电量 |
| 连接后 | **FFE2 订阅成功立即推送**当前电量 |
| 运行中 | 电量**下降**时再推（任意降幅，上升不推） |
| 数据源 | ROS `/battery_level`（`std_msgs/UInt8`，主）；备选 `/pwr`、`/battery_percent`、`/battery_state` |

### 2.3 电机电源状态

```text
mp:ON
mp:OFF
```

| 规则 | 说明 |
|------|------|
| 格式 | `mp:` + `ON` / `OFF` |
| 含义 | 电机供电接通 / 断开 |
| 连接后 | **FFE2 订阅成功 5 秒后**，连发 **3 次**（间隔 1s） |
| 运行中 | `/power_switch_state` 变化时再推 |
| 数据源 | ROS `/power_switch_state`（`livelybot_power/Power_switch`） |

### 2.4 机器 FSM 模式

```text
fsm:5
```

| 规则 | 说明 |
|------|------|
| 格式 | `fsm:` + `/fsm_state` 整型值 |
| 时机 | **状态变化时连发 3 次**（间隔 50ms，防丢包） |
| 订阅时 | 推送当前 FSM 一次 |

### 2.5 FSM 状态对照（参考）

| 值 | 含义 |
|----|------|
| 0 | INIT |
| 5 | EXEC_DEFAULT |
| 8 | PROTECTION_SHUTDOWN |
| 14 | EXEC_TEACHING |
| … | 见 `ble_ros_bridge.py` `FSM_STATE_NAMES` |

### 2.6 下行数据汇总

| 类型 | 格式 | 发送时机 |
|------|------|----------|
| IP | `IP:19.11` | 订阅时 + IP 变化 |
| 电量 | `pwr:83` | 订阅时立即推；之后电量下降再推 |
| 电机电源 | `mp:ON` / `mp:OFF` | 连接 5s 后连发 3 次；状态变化时再推 |
| FSM | `fsm:5` | 变化时连发 3 次；订阅时 1 次 |
| ACK | `ACK:M_default` | 模式/动作/脖子/MP 等指令回执 |

---

## 三、连接流程

```text
扫描 FFE0 → createBLEConnection → setBLEMTU(247)
→ 发现特征 → 订阅 FFE2
→ write FFE1 "M_default"
→ 收到 IP: / pwr: / fsm: 状态包
→ 5 秒后收到 3 次 mp:ON/OFF
→ 20Hz 摇杆 writeNoResponse
```

---

## 四、板端 ROS 映射

| 上行指令 | ROS |
|----------|-----|
| 摇杆 | `/cmd_vel` 20Hz |
| 模式/动作 | `/joy_msg` |
| 脖子 `P{n}Y{m}` / `neck0` | `/pi_plus_absolute` |
| `locate_face ON/OFF` | 启停 `locate_face/locate_face.py` |
| `HI ON/OFF` | 启停 `hand_identify_cpp/start.sh`（手势控制） |
| `MP ON/OFF` | `/power_switch_control`（电机供电开/关） |

参考代码：`docs/miniprogram_ble_snippet.js`
