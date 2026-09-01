# 蓝牙通讯协议与指令总表（ROS2 直连版 · 0828）

> **对照依据**：`ROS2/ROS2功能与服务对照表0828.md`  
> **板端代码**：`ble/app/ble_command_dispatcher.py`、`ble/app/ble_ros_bridge.py`  
> **设计目标**：小程序发 **BLE 文本指令** → 板端 **直接调 ROS2 服务/话题**，不模拟手柄组合键。

**图例（板端实现列）**

| 标记 | 含义 |
|------|------|
| ✅ | 当前固件已支持该 BLE 指令 |
| 🔜 | 文档已定义，板端待实现（`WP:`/`SPD`/`VEL`） |
| — | 无 ROS，纯 BLE / 系统 |

---

## 0. 操作 ↔ BLE 指令 ↔ ROS2 服务/话题 总对照表

> 下表为**唯一权威映射**。小程序按「用户操作 → BLE 指令」列下发；板端按「ROS2 全名 + 参数」列调用。

### 0.1 建链 / 心跳 / 模式（小程序模式菜单）

| 用户操作 | BLE 指令（FFE1） | 类型 | ROS2 全名 | 服务/消息类型 | 请求参数 / 说明 | 板端 |
|----------|------------------|------|-----------|---------------|-----------------|------|
| 进入遥控页握手 | `M_default` | — | — | — | **仅 BLE 建链**，不调 ROS | ✅ |
| 切到 **初始化** 模式 | `M_init` | 服务 | `/hightorque_controller/change_fsm_state` | `hightorque_msgs/srv/ChangeState` | `{states: ['init']}` | ✅ |
| 切到 **保护/急停** 模式 | `M_protect` | 服务 | `/hightorque_controller/change_fsm_state` | `hightorque_msgs/srv/ChangeState` | `{states: ['protect']}` | ✅ |
| 切到 **调零** 模式 | `M_resetzero` | 服务×3 | `/hightorque_controller/change_fsm_state` | `hightorque_msgs/srv/ChangeState` | 板端自动串行：`init` → `prev` → `confirm`；小程序**只发一条** | ✅ |
| 切到 **示教** 模式 | `M_tech` | — | — | — | ROS2 已移除，**忽略**，仅 ACK | ✅ |
| 切到 **默认控制**（FSM=5） | `FSM:default` | 服务 | `/hightorque_controller/change_fsm_state` | `hightorque_msgs/srv/ChangeState` | `{states: ['default']}` | ✅ |
| 同上（等价短名） | `M_default` | — | — | — | ⚠️ **`M_default` 不等于切默认**；切默认请用 `FSM:default` | — |

**模式菜单推荐用法（一条指令，无需退出/切换/确认组合键）：**

```text
M_init        →  change_fsm_state {states: ['init']}
M_protect     →  change_fsm_state {states: ['protect']}
M_resetzero   →  板端内部 init→prev→confirm（一条 BLE 即可）
FSM:default   →  change_fsm_state {states: ['default']}   # 「默认模式」按钮
```

### 0.2 底层 FSM 细项（一般不由模式菜单逐步操作）

| 用户操作 | BLE 指令 | 类型 | ROS2 全名 | 服务类型 | 请求参数 | 板端 |
|----------|----------|------|-----------|----------|----------|------|
| FSM 候选下一项 | `FSM:next` | 服务 | `/hightorque_controller/change_fsm_state` | `ChangeState` | `{states: ['next']}` | ✅ |
| FSM 候选上一项 | `FSM:prev` | 服务 | 同上 | 同上 | `{states: ['prev']}` | ✅ |
| 确认 FSM 候选 | `FSM:confirm` | 服务 | 同上 | 同上 | `{states: ['confirm']}` | ✅ |
| 进 DEFAULT | `FSM:default` | 服务 | 同上 | 同上 | `{states: ['default']}` | ✅ |
| 进 INIT | `FSM:init` | 服务 | 同上 | 同上 | `{states: ['init']}` | ✅（等价 `M_init`） |
| 进 PROTECT | `FSM:protect` | 服务 | 同上 | 同上 | `{states: ['protect']}` | ✅（等价 `M_protect`） |

### 0.3 上层状态（行为树 default_bt）

| 用户操作 | BLE 指令 | 类型 | ROS2 全名 | 服务类型 | 请求参数 | 板端 |
|----------|----------|------|-----------|----------|----------|------|
| **站立** | `ST:standing` | 服务 | `/hightorque_controller/change_state` | `hightorque_msgs/srv/ChangeState` | `{states: ['standing']}` | ✅ |
| **坐下** | `ST:sit` | 服务 | 同上 | 同上 | `{states: ['siting']}`（仅 standby） | ✅ |
| **启停策略** toggle | `ST:toggle` | 服务 | 同上 | 同上 | `{states: ['toggle_policy']}` | ✅ |
| **启动策略** | `ST:start` | 服务 | `/hightorque_controller/start_policy` | `hightorque_msgs/srv/Common` | `{enable: true, str: ''}` | ✅ |
| **停止策略** | `ST:stop` | 服务 | `/hightorque_controller/stop_policy` | `hightorque_msgs/srv/Common` | `{enable: false, str: ''}` | ✅ |
| **开启行走**（复合） | `GAIT ON` | 服务×N | 见 §0.3.1 | — | — | ✅ |
| **关闭行走/回站立** | `GAIT OFF` | 服务 | `/hightorque_controller/change_state` | `ChangeState` | `{states: ['toggle_policy']}` → standby | ✅ |

#### 0.3.1 `GAIT ON` 板端服务调用序列

| 步骤 | ROS2 全名 | 参数 |
|------|-----------|------|
| 1 | `/hightorque_controller/change_fsm_state` | `{states: ['default']}`（若未在 default_bt） |
| 2 | `/hightorque_controller/switch_policy` | `{policy_name: 'amp'}` |
| 3 | 等待 `/hightorque_controller/state`.current_policy = `amp` | — |
| 4 | `/hightorque_controller/change_state` | `{states: ['toggle_policy']}` → running |

**前提**：`/imu` 须由 `yesense_imu_node` 持续发布（`/dev/ttyUSB0`）。仅有 `/joint_states` 可站立，但无 IMU 时 AMP 开环会倒；板端 `GAIT ON` 会拒绝并打日志。

`GAIT OFF`：步骤 4 反向（toggle → standby）+ `/cmd_vel` 清零。

### 0.4 基础运动（话题）

| 用户操作 | BLE 指令 | 类型 | ROS2 全名 | 消息类型 | 字段 / 说明 | 板端 |
|----------|----------|------|-----------|----------|-------------|------|
| 摇杆前后左右转 | `X:{x},Y:{y},Z:{z}` | 话题 | `/cmd_vel`（BFM running）或 `/joy`（AMP） | `Twist` / `Joy` | BFM：符号+阈值→±0.8；AMP：经 joy_mapper | ✅ |
| 显式速度 | `VEL:{lx},{ly},{az}` | 话题 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 直接填三轴 | 🔜 |
| 停止 | `X:0,Y:0,Z:0` | 话题 | `/cmd_vel` | `Twist` | 全 0 → BFM standing | ✅ |
| **2 倍速** | `SPD ON` | 话题 | `/cmd_vel` | `Twist` | 发布前线/角速度 ×2 | 🔜 |
| 关闭 2 倍速 | `SPD OFF` | 话题 | `/cmd_vel` | `Twist` | 恢复 ×1 | 🔜 |

**前提**：`MP ON` + 上电 2.5s + `GAIT ON`（`current_state=running`）。BFM 还须 `current_policy=bfm`。

**BFM：XYZ → `/cmd_vel`（板端映射，与文档 §5 一致）**

| BLE 轴 | 映射到 Twist | 激活 | 输出 |
|--------|--------------|------|------|
| X（前+） | `linear.x` | \|v\|≥0.3 / 释放 0.25 | ±0.8 |
| Y（右+） | `linear.y`（取反→左+） | 同上 | ±0.8 |
| Z（右转+） | `angular.z`（取反→左转+） | 同上 | ±0.8 |

50Hz 在摇杆有效时持续发布三轴（可同时非零 → 斜向/行进转向）；松杆发一次零速后停发。BFM 不向 `/joy` 写摇杆；`joy_mapper` 回中也不再发零速，避免盖掉 BLE 的左右/自转/复合方向。

### 0.5 步态 / 策略切换（服务 switch_policy）

服务全名：`/hightorque_controller/switch_policy` · 类型：`hightorque_msgs/srv/SwitchPolicy` · 字段：`policy_name`

| 用户操作 | BLE 指令 | policy_name | Jetson |
|----------|----------|-------------|--------|
| BFM 全身动作 | `POL:bfm` | `bfm` | ✅ |
| AMP 全身走路 | `POL:amp` | `amp` | ✅ |
| AMP 下半身走路 | `POL:amp_lower` | `amp_lower` | ✅ |
| PiPlus Walk | `POL:piplus_walk` | `piplus_walk` | ❌ |
| 足球步态 | `POL:pi_plus_soccer` | `pi_plus_soccer` | ❌ |

**前提**：`current_mode=default_bt`，`current_state` 为 `standby` 或 `running`。

### 0.6 一次性动作策略（同 switch_policy）

| 用户操作 | BLE 指令 | policy_name | Jetson |
|----------|----------|-------------|--------|
| 小脚踢球 | `POL:byd_small_kick` | `byd_small_kick` | ✅ |
| 秀肌肉 | `POL:byd_power` | `byd_power` | ✅ |
| 上勾拳 | `POL:pi_plus_shanggouquan` | `pi_plus_shanggouquan` | ✅ |
| 直踢腿 | `POL:pi_plus_zhidengtui` | `pi_plus_zhidengtui` | ✅ |
| 重拳 | `POL:pi_plus_zhongquan` | `pi_plus_zhongquan` | ✅ |
| 疯狂动物城舞 | `POL:pi_plus_zoo` | `pi_plus_zoo` | ✅ |
| 超级冠军舞 | `POL:pi_plus_guanjun` | `pi_plus_guanjun` | ✅ |
| Fantastic Baby 舞 | `POL:byd_bb` | `byd_bb` | ✅ |
| 猪猪侠舞 | `POL:byd_zzx` | `byd_zzx` | ✅ |
| SP8 | `POL:SP8` | `SP8` | ✅ |

板端：✅ 步态策略 `POL:bfm|amp|amp_lower`；编舞策略同表 ✅；`piplus_walk` / `pi_plus_soccer` 仍 ❌

### 0.7 Waypoint 动作（服务 execute_waypoint）

服务全名：`/hightorque_controller/execute_waypoint` · 类型：`hightorque_msgs/srv/ExecuteWaypoint` · 字段：`action_name`

| 用户操作 | BLE 指令 | action_name | 条件 | H1W |
|----------|----------|-------------|------|-----|
| 双手欢呼 / 挥双手 | `WP:cheer` | `cheer` | running 可并行 | ✅ |
| 握手 | `WP:woshou` | `woshou` | running 可并行 | ✅ |
| 招手 | `WP:hello` | `hello` | running 可并行 | ✅ |
| 点头 | `WP:diantou` | `diantou` | running 可并行 | ⚠️ |
| 挠头 | `WP:naotou` | `naotou` | running 可并行 | ⚠️ |
| 右高握手 | `WP:right_woshou_high` | `right_woshou_high` | running 可并行 | ⚠️ |
| 右低握手 | `WP:right_woshou_low` | `right_woshou_low` | running 可并行 | ⚠️ |
| 防御抖动 | `WP:defense_shake` | `defense_shake` | running 可并行 | ✅ |
| 俯卧起身 | `WP:fuwo` | `fuwo` | 仅 standby | ✅ |
| 仰卧起身 | `WP:yangwo` | `yangwo` | 仅 standby | ❌ |
| 坐下 waypoint | `WP:sitdown` | `sitdown` | 仅 standby | ✅ |

板端：🔜

### 0.8 电机电源 / 脖子 / 拖拽 / 头追

| 用户操作 | BLE 指令 | 类型 | ROS2 全名 | 消息/服务类型 | 参数 / 说明 | 板端 |
|----------|----------|------|-----------|---------------|-------------|------|
| 电机上电 | `MP ON` | 话题 | `/power_switch_control` | `hightorque_power/msg/PowerSwitch` | `control_switch=1` | ✅ |
| 电机断电 | `MP OFF` | 话题 | `/power_switch_control` | 同上 | `control_switch=0`；先停 `/cmd_vel` | ✅ |
| 脖子步进 | `P{n}Y{m}` | 话题 | `/pi_plus_absolute` | `sensor_msgs/msg/JointState` | `head_pitch_joint` / `head_yaw_joint` | ✅ |
| 脖子回中 | `neck0` | 话题 | `/pi_plus_absolute` | 同上 | 平滑回中 | ✅ |
| 开启拖拽 | `PULL ON` | 系统 | — | — | 启动 `torque-cmd-vel.service`；桥接订阅 `/error_joint_states` → 发布 `/cmd_vel` | ✅ |
| 关闭拖拽 | `PULL OFF` | 系统 | — | — | 停止 service + `/cmd_vel` 清零 | ✅ |
| 人脸跟踪开 | `locate_face ON` | 进程 | — | — | 启动 `locate_face_cpp`；内部用 `/pi_plus_absolute`、`/fsm_state` | ✅ |
| 人脸跟踪关 | `locate_face OFF` | 进程 | — | — | 停进程 + 脖子回中 | ✅ |

### 0.9 非 ROS 系统指令

| 用户操作 | BLE 指令 | 说明 | 板端 |
|----------|----------|------|------|
| 链路心跳 | `1` | 空闲 ≥3s 发送 | ✅ |
| 设置音量 | `V {0..100}` | PulseAudio `amixer` | ✅ |
| 实时语音开/关 | `SOUND ON` / `SOUND OFF` | FFE1 二进制 PCM（首字节 `0x0B`） | ✅ |
| 固定语音 | `LYJXD` 等短码 | `voice_remind/conversation_bag/*.wav` | ✅ |
| Wi-Fi 配网 | `WIFI {ssid} {password}` | nmcli | ✅ |
| 改广播名 | `rename+{name}` | 仅 BLE；持久化 `/var/lib/bird-ble/ble_device_name.conf` | ✅ |

### 0.10 状态查询（仅上行 FFE2，小程序不下发）

| 用户可见信息 | FFE2 格式 | ROS2 数据源 | 消息类型 |
|--------------|-----------|-------------|----------|
| 上层状态 | `state:{value}` | `/hightorque_controller/state` | `hightorque_msgs/msg/ControllerState`.current_state |
| 上层模式 | `mode:{value}` | 同上 | `.current_mode` |
| 当前策略 | `policy:{name}` | 同上 | `.current_policy` |
| 底层 FSM 编号 | `fsm:{n}` | `/fsm_state` | `std_msgs/msg/Int32` |
| 模式语义（兼容） | `mode:M_default` 等 | 板端由 state/fsm 推断 | — |
| 电量 | `pwr:{0..100}` | `/battery_level` | `std_msgs/msg/UInt8` |
| 电机电源 | `mp:ON/OFF` | `/power_switch_state` | `hightorque_power/msg/PowerSwitch` |
| IP | `IP:{ipv4}` | 系统网络 | — |

---

## 1. 标准操作流程

| 步骤 | 用户操作 | BLE 指令 | ROS2 调用 |
|------|----------|----------|-----------|
| ① | 连接握手 | `M_default` | 无 |
| ② | 电机上电 | `MP ON` | `/power_switch_control` |
| ③ | 进 DEFAULT | `FSM:default` | `change_fsm_state {states: ['default']}` |
| ④ | 站立 | `ST:standing` | `change_state {states: ['standing']}` |
| ⑤ | 启行走 | `GAIT ON` | `switch_policy(amp)` + `change_state(toggle)` |
| ⑥ | 摇杆走 | `X:…,Y:…,Z:…` | `/cmd_vel` |
| ⑦ | 做动作 | `WP:cheer` 等 | `execute_waypoint` |
| ⑧ | 停行走 | `GAIT OFF` | `change_state(toggle)` → standby |
| ⑨ | 急停 | `M_protect` | `change_fsm_state {states: ['protect']}` |

---

## 2. ROS2 服务/话题速查（完整路径）

| 用途 | 全名 | 类型 |
|------|------|------|
| 上层状态 | `/hightorque_controller/change_state` | `hightorque_msgs/srv/ChangeState` |
| 启动策略 | `/hightorque_controller/start_policy` | `hightorque_msgs/srv/Common` |
| 停止策略 | `/hightorque_controller/stop_policy` | `hightorque_msgs/srv/Common` |
| 底层 FSM | `/hightorque_controller/change_fsm_state` | `hightorque_msgs/srv/ChangeState` |
| 策略/编舞 | `/hightorque_controller/switch_policy` | `hightorque_msgs/srv/SwitchPolicy` |
| Waypoint 动作 | `/hightorque_controller/execute_waypoint` | `hightorque_msgs/srv/ExecuteWaypoint` |
| 行走速度 | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| 电机电源 | `/power_switch_control` | `hightorque_power/msg/PowerSwitch` |
| 电源状态 | `/power_switch_state` | `hightorque_power/msg/PowerSwitch` |
| 脖子 | `/pi_plus_absolute` | `sensor_msgs/msg/JointState` |
| 控制器状态 | `/hightorque_controller/state` | `hightorque_msgs/msg/ControllerState` |
| FSM 编号 | `/fsm_state` | `std_msgs/msg/Int32` |
| 电量 | `/battery_level` | `std_msgs/msg/UInt8` |
| 拖拽力矩源 | `/error_joint_states` | `sensor_msgs/msg/JointState`（PULL 桥接订阅） |

**终端调用示例：**

```bash
source /opt/ros/foxy/setup.bash
source ~/hightorque_workspace/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/nvidia/cyclonedds.xml

# 站立
ros2 service call /hightorque_controller/change_state \
  hightorque_msgs/srv/ChangeState "{states: ['standing']}"

# 保护
ros2 service call /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState "{states: ['protect']}"

# 行走速度
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 挥手
ros2 service call /hightorque_controller/execute_waypoint \
  hightorque_msgs/srv/ExecuteWaypoint "{action_name: 'cheer'}"
```

---

## 3. 通讯约定

### 3.1 GATT

| UUID | 方向 | 用途 |
| --- | --- | --- |
| FFE0 | — | 主服务 |
| FFE1 | 小程序 → 机器人 | 下行指令 |
| FFE2 | 机器人 → 小程序 | 上行状态 / ACK |

### 3.2 建链

1. 写入 `M_default`
2. 订阅 FFE2 Notify
3. 空闲 ≥3s 发心跳 `1`；从机 7s 无下行断链

### 3.3 指令命名前缀

| 前缀 | 对应 ROS2 |
|------|-----------|
| `M_*` | `change_fsm_state`（`M_default` 除外） |
| `FSM:*` | `change_fsm_state` |
| `ST:*` | `change_state` / `start_policy` / `stop_policy` |
| `POL:*` | `switch_policy` |
| `WP:*` | `execute_waypoint` |
| `X:Y:Z` / `VEL:` | `/cmd_vel` |
| `GAIT` / `MP` / `SPD` | 见 §0.3、§0.4、§0.8 |

---

## 4. 上行 FFE2 与 ACK

| 类型 | 格式 | 适用 |
|------|------|------|
| ACK | `ACK:{原文}` | `M_*`、`ST:*`、`FSM:*`、`POL:*`、`WP:*`、`locate_face`、`SPD`、`V` |
| 原文回显 | 与下发相同 | `MP ON/OFF`、`GAIT ON/OFF`、`sound`、`PULL` |
| 无回执 | — | 摇杆、`P{n}Y{m}`、心跳 `1` |

---

## 5. Pi_Plus 动作库 → BLE 指令

| 动作名 | BLE 指令 | ROS2 全名 | 参数字段 |
|--------|----------|-----------|----------|
| 挥双手 | `WP:cheer` | `/hightorque_controller/execute_waypoint` | `action_name: cheer` |
| 握手 | `WP:woshou` | 同上 | `action_name: woshou` |
| 点头 | `WP:diantou` | 同上 | `action_name: diantou` |
| 挠头 | `WP:naotou` | 同上 | `action_name: naotou` |
| 高握手 | `WP:right_woshou_high` | 同上 | `action_name: right_woshou_high` |
| 低握手 | `WP:right_woshou_low` | 同上 | `action_name: right_woshou_low` |
| 直踢腿 | `POL:pi_plus_zhidengtui` | `/hightorque_controller/switch_policy` | `policy_name: pi_plus_zhidengtui` |
| 后手翻 | `WP:fuwo` | `/hightorque_controller/execute_waypoint` | `action_name: fuwo` |
| 小脚踢球 | `POL:byd_small_kick` | `/hightorque_controller/switch_policy` | `policy_name: byd_small_kick` |
| 展示力量 | `POL:byd_power` | 同上 | `policy_name: byd_power` |
| 组合拳 | `POL:pi_plus_zhongquan` | 同上 | `policy_name: pi_plus_zhongquan` |
| 疯狂动物城等编舞 | `POL:pi_plus_zoo` 等 | 同上 | 见 §0.6 |

---

## 6. 旧指令迁移（废弃 → 新指令）

| 废弃 BLE 指令 | 新 BLE 指令 | ROS2 全名 |
|---------------|-------------|-----------|
| `LT+RT+start` | `ST:standing` | `/hightorque_controller/change_state` |
| `LT+RT+RB` | `ST:sit` | 同上 |
| `LT+RT+LB` | `GAIT ON` / `ST:toggle` | `change_state` + `switch_policy` |
| `LT+RT+B` | `M_protect` / `FSM:protect` | `/hightorque_controller/change_fsm_state` |
| `LT+RT+A` | `FSM:confirm` | 同上 |
| `center` | `FSM:default` | 同上 |
| `start+B` | `M_init` / `FSM:init` | 同上 |
| `LT ON/OFF` | `SPD ON/OFF` | `/cmd_vel` 缩放 |
| `RT+A` | `WP:cheer` | `/hightorque_controller/execute_waypoint` |
| `RT+B` | `WP:woshou` | 同上 |

---

## 7. 一键安装（install.sh）

```bash
cd ~/Bird_ws/Bt-source-ros2 && sudo bash install.sh
```

| 服务 | 说明 |
|------|------|
| `bird-ble.service` | BLE GATT + ROS 桥接 |
| `torque-cmd-vel.service` | 拖拽（`PULL ON` 启动，默认不开机自启） |
| ROS2 自启 | `bfm_real.launch.py` |

环境：`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，`CYCLONEDDS_URI=file://~/cyclonedds.xml`，`COLCON_WS=~/hightorque_workspace`。

---

## 8. 板端实现索引

| 模块 | 路径 |
|------|------|
| 指令分类 | `ble/app/ble_command_dispatcher.py` |
| ROS 桥接 | `ble/app/ble_ros_bridge.py` |
| 详细协议 | `ble/docs/BLE_PROTOCOL.md` |
