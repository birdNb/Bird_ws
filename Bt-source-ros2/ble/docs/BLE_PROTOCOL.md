# Bird BLE 通讯协议（Bt-source-ros2 / ROS2 Foxy）

> 全链路 **UTF-8 纯文本**，单条不含换行，经 **FFE2 notify** / **FFE1 write** 传输。  
> 板端控制桥优先调用量产 `hightorque_controller` 服务切到 **AMP**（不用 BFM）；摇杆仍发 ROS2 `/joy`。小程序指令格式与 ROS1 版相同。

> 全链路 **UTF-8 纯文本**，单条不含换行，经 **FFE2 notify** / **FFE1 write** 传输。


| UUID | 方向        | 用途         |
| ---- | --------- | ---------- |
| FFE0 | —         | 服务         |
| FFE1 | 小程序 → 机器人 | 控制指令       |
| FFE2 | 机器人 → 小程序 | ACK + 状态遥测 |


## 终端日志颜色


| 颜色    | 标记     | 含义                  |
| ----- | ------ | ------------------- |
| **红** | `RX`   | 收到小程序数据（FFE1 写入）    |
| **绿** | `TX`   | 板子发出数据（FFE2 notify） |
| 白     | —      | 链路/系统信息             |
| 黄     | `WARN` | 警告                  |


---

## 一、上行：小程序 → 机器人（FFE1 write）

### 1.1 摇杆


| 字段  | 范围    | 说明       |
| --- | ----- | -------- |
| X   | ±1.80 | 前后（前 +）  |
| Y   | ±1.80 | 左右（右 +）  |
| Z   | ±1.50 | 转向（右转 +） |


```text
X:{x},Y:{y},Z:{z}
X:{x},Y:{y},Z:{z},N:{seq}    # 可选序号，20Hz 保活
```

- 频率 **20 Hz**，`writeNoResponse`
- 摇杆无 ACK

### 1.2 模式


| 指令            | 说明         |
| ------------- | ---------- |
| `M_default`   | 默认（进遥控页首发） |
| `M_init`      | 初始化        |
| `M_protect`   | 保护         |
| `M_resetzero` | 调零         |
| `M_tech`      | 示教         |


### 1.3 组合键与自定义动作

**通用手柄识别：** 任意合法手柄组合键文本经 `ble_gamepad` 解析后，由板端叠加到 ROS2 **`/joy`**。量产算法状态/策略优先走服务：底层 FSM 用 `change_fsm_state`，上层步态用 `switch_policy(amp)` + `change_state(toggle_policy)`。

| 功能     | 指令            | ROS2 动作                         |
| ------ | ------------- | ---------------------------------- |
| 起立     | `LT+RT+start` | `change_state(standing)`，失败则 LT+RT+START 长按 1s |
| 进/退步态  | `GAIT ON` / `GAIT OFF` | 先确认 `default_bt`，`SwitchPolicy(amp)` 并等待 `current_policy`，再 `toggle_policy` |
| 坐下     | `LT+RT+RB` / `ST:sit` | running 先自动 `toggle→standby`（回执 `GAIT OFF`）再 `siting`；失败则手柄兜底 |
| 急停     | `LT+RT+B`     | `change_fsm_state(protect)`，失败则 LT+RT 扳机 |
| 加速     | `LT ON`       | LT 扳机 `axes[2]=-1`                 |
| 自定义    | 挥手/踢球等      | 组合键叠加到 `/joy`（动作库尚未按 ROS2 重做） |


步态（`GAIT ON`）会先切 AMP 再 `toggle_policy` 进入 running，FFE2 回传 `GAIT ON`。`GAIT OFF` 回到 standby，摇杆回中并回传 `GAIT OFF`。电机电源（`MP ON/OFF`）收到后原样回传确认。

**保护/急停：** 调用 `/hightorque_controller/change_fsm_state`，`states: ['protect']`；失败则发 `LT+RT+B`。

### 1.4 脖子控制


| 指令         | 说明                              |
| ---------- | ------------------------------- |
| `P{n}Y{m}` | pitch / yaw 步进偏移（整数，可带 `+`/`-`） |
| `neck0`    | 脖子回中（脖子切换按钮）                    |


每步 **10°**：


| 轴        | `+` | `-` |
| -------- | --- | --- |
| P（pitch） | 往上  | 往下  |
| Y（yaw）   | 往右  | 往左  |


示例：

```text
P1Y0      # 抬头 10°
P-1Y0     # 低头 10°
P0Y1      # 右转 10°
P0Y-1     # 左转 10°
P0Y0      # 平滑回中（同 neck0）
neck0     # 回中
```

板端解析后走 ROS2 量产电机接口：`/request_control`（头 yaw/pitch）+ `/control_command`；并兼容发布 `/pi_plus_absolute`（`head_yaw_joint` / `head_pitch_joint`）。实现见 `ble_neck_bridge.py`。需 `hightorque_midware_node` 在线。

### 1.5 人脸跟踪（locate_face）


| 指令                | 说明                                       |
| ----------------- | ---------------------------------------- |
| `locate_face ON`  | 启动 `locate_face_cpp`（默认后台；`--gui` 才显示预览） |
| `locate_face OFF` | 停止头追进程                                   |


视觉伺服脖子跟随；与手动脖子步进共用 `/pi_plus_absolute`，建议二选一使用。

### 1.6 电机电源（MP）


| 指令       | 小程序操作  | 说明                           |
| -------- | ------ | ---------------------------- |
| `MP ON`  | **长按** | 接通电机供电（功率板 `power_switch=1`）；上电后约 2.5s 才允许走路 |
| `MP OFF` | **点击** | 先发 LT+RT 急停再断电 |


板端发布 ROS2 `/power_switch_control`（`hightorque_power/PowerSwitch`，`control_switch=1`）。收到后 FFE2 **原样回传** `MP ON` / `MP OFF` 确认（无 `ACK:` 前缀）。

**断电后安全闸：** `MP OFF` 之后，板端会拒绝摇杆、进策略、急停、`GAIT`、二档速度等指令，直到再次 `MP ON` 且等待约 2.5s。

### 1.8 疾跑（LT）


| 指令       | 说明                                                                           |
| -------- | ---------------------------------------------------------------------------- |
| `LT ON`  | 开启加速：持续向 `/joy` 发布 LT 扳机 `axes[2]=-1` |
| `LT OFF` | 关闭加速：松开 LT 扳机 |


- 与摇杆 `/joy` 并行
- 断连自动关闭并松开 LT 扳机
- 有 ACK：`ACK:LT ON` / `ACK:LT OFF`

### 1.9 实时语音（sound）


| 指令          | 说明                       |
| ----------- | ------------------------ |
| `sound ON`  | 开启语音接收，FFE1 二进制 PCM 实时播放 |
| `sound OFF` | 关闭语音，停止播放                |


- **音频走 FFE1**（与控制同一写入特征），与摇杆文本包区分：音频首字节为 `**0x0B`**
- 包格式：`[0x0B][seq_hi][seq_lo][pcm...]`，约 **180 字节 PCM**，8kHz / s16le / mono
- 收到后 FFE2 **原样回传** `sound ON` / `sound OFF`（无 `ACK:` 前缀）
- 未 `sound ON` 时音频包丢弃；`sound ON` 后终端 stderr 显示**实时电平条**
- 可选：`--enable-voice` 额外注册 FFE3 备用通道

### 1.9.1 固定语音（conversation_bag）

小程序发送录音文案 **前 5 个汉字拼音首字母（大写）**，板端播放 `voice_remind/conversation_bag/` 下同名 WAV。

示例：文案 `蓝牙就绪待连接` → 前5字 `蓝牙就绪待` → 发送 **`LYJXD`** → 播放 `LYJXD.wav`

- 有 ACK：`ACK:LYJXD`
- 上传：WAV 以大写首字母命名，并编辑 `manifest.json`（详见 `conversation_bag/README.md`）
- 不足 5 字取全部汉字（如 `行走模式` → `XZMS`）

### 1.10 音量调节（V）


| 指令          | 说明             |
| ----------- | -------------- |
| `V {0-100}` | 将系统播放音量设为指定百分比 |


用户在小程控件里调节完音量后发送，例如调到 **10%** 时：

```text
V 10
```

- 范围自动钳位到 `0`–`100`
- 板端通过 PulseAudio `amixer -D pulse sset Master {n}%` 生效
- 有 ACK：`ACK:V 10`

### 1.10.1 拉动 pull_move 控制（PULL ON/OFF）

| 指令 | 说明 |
| ---- | ---- |
| `PULL ON` | 开启 `torque-cmd-vel.service`（肩/脖子力矩→`/cmd_vel` 拖拽映射），语音提示「拖拽模式已打开」 |
| `PULL OFF` | 关闭 `torque-cmd-vel.service`（停止拖拽映射），语音提示「拖拽模式已关闭」 |

**默认行为：** 开机时拖拽模式关闭；收到摇杆、模式、动作、步态、疾跑、脖子、电机电源等控制指令时，若拖拽已开启则**自动关闭**并播报「拖拽模式已关闭」。

**语音打断：** 新对话语音（conversation_bag code）或系统提示音会打断当前正在播放的语音。

### 1.10.2 WiFi 配网（WIFI）

小程序经 FFE1 下发：

```text
WIFI <SSID> <PASSWORD>
```

| 规则 | 说明 |
| ---- | ---- |
| SSID | 最长 32；无引号时密码前全部为 SSID（`WIFI Bird Phone 12345678` → `Bird Phone`） |
| PASSWORD | 最长 63；无引号时最后一个 token；含空格须加引号 |
| 板端流程 | 解析 → 断开当前 WiFi → `nmcli` 连接 → FFE2 回结果 |
| 成功 | 先 `WIFI OK`，再单独推 `IP:x.x.x.x`；播报 `wifi_connected` |
| 失败 | `WIFI FAIL <reason>` |
| 链路断开 | FFE2 `WiFi disconnected`（小程序需原样识别）；播报 `wifi_disconnected` |
| 链路恢复 | 同成功：`WIFI OK` + `IP:…` |

### 1.11 指令 ACK（FFE2 notify）

```text
ACK:{原文}
```

模式、动作、locate_face、LT 疾跑、V 有 `ACK:` 回执；步态（生效后）、电机电源、`PULL ON/OFF`、**sound ON/OFF**、**WIFI 最终结果** 为**原文回显**；摇杆、脖子无回执。

---

## 二、下行：机器人 → 小程序（FFE2 notify）

订阅 FFE2 后，板端主动推送状态。

### 2.1 局域网 IP

```text
IP:192.168.19.11
```


| 规则  | 说明                                   |
| --- | ------------------------------------ |
| 格式  | `IP:` + IPv4 四段完整地址                  |
| 示例  | `192.168.19.11` → `IP:192.168.19.11` |
| 时机  | 订阅后立即推一次；局域网 IP 变化时再推                |


### 2.2 电量

```text
pwr:83
```


| 规则  | 说明                                                                                     |
| --- | -------------------------------------------------------------------------------------- |
| 格式  | `pwr:` + 0~100 整数（实际剩余电量，如 83% → `pwr:83`）                                             |
| 含义  | `pwr:83` = 剩余 83% 电量                                                                   |
| 连接后 | **FFE2 订阅成功立即推送**当前电量                                                                  |
| 运行中 | 电量**下降**时再推（任意降幅，上升不推）                                                                 |
| 数据源 | ROS `/battery_level`（`std_msgs/UInt8`，主）；备选 `/pwr`、`/battery_percent`、`/battery_state` |


### 2.3 电机电源状态

```text
mp:ON
mp:OFF
```


| 规则  | 说明                                                        |
| --- | --------------------------------------------------------- |
| 格式  | `mp:` + `ON` / `OFF`                                      |
| 含义  | 电机供电接通 / 断开                                               |
| 连接后 | **FFE2 订阅成功 5 秒后**，连发 **2 次**（间隔 1s）                      |
| 运行中 | `MP ON/OFF` 指令生效后推送（连发 2 次）；硬件状态经板端意图过滤后再推 |
| 数据源 | 板端 `MotorPowerController`（指令意图 + `/power_switch_state`） |


### 2.4 机器 FSM 模式

```text
fsm:5
```


| 规则  | 说明                           |
| --- | ---------------------------- |
| 格式  | `fsm:` + `/fsm_state` 整型值    |
| 时机  | **订阅时与状态变化时均连发 2 次**（间隔 50ms） |


### 2.4.1 功能开关同步（订阅时 + 变化时，各连发 2 次）

```text
locate_face ON
GAIT OFF
PULL OFF
sound OFF
LT OFF
```


| 报文 | 含义 |
| --- | --- |
| `locate_face ON/OFF` | 人脸追踪是否在跑 |
| `GAIT ON/OFF` | 行走 / **站立**（`OFF`=站立模式） |
| `PULL ON/OFF` | 拖拽模式 |
| `sound ON/OFF` | 实时语音开关 |
| `LT ON/OFF` | 疾跑开关 |


### 2.5 FSM 状态对照（参考）


| 值   | 含义                                      |
| --- | --------------------------------------- |
| 0   | INIT                                    |
| 1   | ERROR                                   |
| 2   | CANDIDATE_DEFAULT                       |
| 5   | EXEC_DEFAULT（量产默认）                    |
| 8   | PROTECTION_SHUTDOWN                     |
| 9   | CANDIDATE_RESET_ZERO                    |
| 10  | EXEC_RESET_ZERO                         |
| 11  | EXEC_RESET_ZERO_SUCCESSFULLY            |
| 12  | EXEC_RESET_ZERO_FAILED                  |


### 2.6 下行数据汇总


| 类型   | 格式                          | 发送时机                  |
| ---- | --------------------------- | --------------------- |
| IP   | `IP:192.168.19.11`          | 订阅时 + IP 变化           |
| 电量   | `pwr:83`                    | 订阅时立即推；之后电量下降再推       |
| 电机电源 | `mp:ON` / `mp:OFF`          | 连接 5s 后连发 2 次；变化时再推 2 次 |
| FSM  | `fsm:5`                     | 订阅/变化均连发 2 次           |
| 人脸追踪 | `locate_face ON/OFF`        | 订阅时同步；启停后再推 2 次        |
| 站立/行走 | `GAIT ON/OFF`               | 订阅时同步；步态变化后再推 2 次（OFF=站立） |
| 拖拽 | `PULL ON/OFF`               | 订阅时同步；开关后再推 2 次         |
| 语音 | `sound ON/OFF`              | 订阅时同步；开关后再推 2 次         |
| 疾跑 | `LT ON/OFF`                 | 订阅时同步；开关后再推 2 次         |
| ACK  | `ACK:M_default`             | 模式/动作/脖子等指令回执         |
| 原文确认 | `MP ON/OFF` / `GAIT ON/OFF` | 电机电源收到即回传；步态由桥接回传 |


---

## 三、连接流程


---

## 三、连接流程

**广播策略：** 手机连接成功后，板端**立即停止 BLE 广播**，小程序/other 设备无法再扫到该机器人，避免连接被抢占；断连后自动恢复可扫描。

```text
扫描 FFE0 → createBLEConnection → setBLEMTU(247)
→ 发现特征 → 订阅 FFE2
→ write FFE1 "M_default"
→ 收到 IP: / pwr: / mode:M_default / fsm:5（ROS2 默认 EXEC_DEFAULT）及 locate_face / GAIT / PULL / sound / LT
→ 5 秒后收到 2 次 mp:ON/OFF
→ 20Hz 摇杆 writeNoResponse
```

---

## 四、指令 → 动作映射总表（板端实现）

> 小程序经 **FFE1 write** 发 UTF-8 文本；板端 `ble_command_dispatcher` 分类后，由 `ble_gatt_server` / `ble_ros_bridge` 等模块执行。  
> 摇杆发 ROS2 **`/joy`**；状态/策略优先走 `hightorque_controller` 服务，不发 `/cmd_vel`、`/joy_msg`。

### 4.1 总览


| FFE1 指令           | 分类   | 底层触发             | ROS / 系统动作                          | FFE2 回执               | 备注                        |
| ----------------- | ---- | ---------------- | ----------------------------------- | --------------------- | ------------------------- |
| `X:…,Y:…,Z:…`     | 摇杆   | 解析 XYZ           | `/joy` axes 50Hz                    | 无                     | X→ly 前+；Y→lx 右-；Z→rx 右转- |
| `M_default`       | 握手   | 无                | 无                                   | `ACK:M_default`       | 仅 BLE 握手                  |
| `M_init`          | 模式   | FSM INIT          | `change_fsm_state` `init`           | `ACK:M_init`          | 失败不连按 LT+RT+B            |
| `M_protect`       | 模式   | FSM PROTECT       | `change_fsm_state` `protect`        | `ACK:M_protect`       | 失败则 LT+RT+B                |
| `M_resetzero`     | 模式   | FSM RESETZERO     | `init` → `prev` → `confirm`         | `ACK:M_resetzero`     |                           |
| `M_tech`          | 模式   | 无                | **忽略**                              | `ACK:M_tech`          |                           |
| `LT+RT+start`     | 起立   | standing          | `change_state`，失败则 `/joy` 1s     | `ACK:LT+RT+start`     | standing                  |
| `LT+RT+RB`        | 坐下   | siting            | running 时先 `toggle→standby` 再 `siting`；失败则 `/joy` 1s | `ACK:LT+RT+RB` / `GAIT OFF` | BFM/AMP 不可直接从 running 蹲 |
| `LT+RT+B`         | 急停   | protect           | `change_fsm_state`，失败则 LT+RT     | `ACK:LT+RT+B`         |                           |
| `RT+A`            | 动作   | 短脉冲             | `/joy` 叠加                             | `ACK:RT+A`            |                           |
| `RT+X`            | 动作   | 短脉冲             | `/joy` 叠加                             | `ACK:RT+X`            |                           |
| `RT+Y`            | 动作   | 短脉冲             | `/joy` 叠加                             | `ACK:RT+Y`            |                           |
| `RT+B`            | 动作   | 短脉冲             | `/joy` 叠加                             | `ACK:RT+B`            |                           |
| `A`               | 自定义  | 短脉冲             | `/joy` 叠加                             | `ACK:A`               |                           |
| `X`               | 自定义  | 短脉冲             | `/joy` 叠加                             | `ACK:X`               |                           |
| `LT+RT+DPU`       | 自定义  | 无                | **忽略**                              | `ACK:LT+RT+DPU`       |                           |
| `LT+RT+DPR`       | 自定义  | 无                | **忽略**                              | `ACK:LT+RT+DPR`       |                           |
| `LT+DPR`          | 自定义  | 无                | **忽略**                              | `ACK:LT+DPR`          |                           |
| `LT+DPD`          | 自定义  | 无                | **忽略**                              | `ACK:LT+DPD`          |                           |
| `LT+DPL`          | 自定义  | 无                | **忽略**                              | `ACK:LT+DPL`          |                           |
| `GAIT ON`         | 步态   | AMP + RUNNING     | `switch_policy(amp)` + `toggle_policy` | `GAIT ON`          | 先确认 default_bt           |
| `GAIT OFF`        | 步态   | STANDBY           | `toggle_policy` + 摇杆回中            | `GAIT OFF`            |                             |
| `LT ON`           | 加速   | LT 扳机            | `/joy` `axes[2]=-1`                 | `ACK:LT ON`           | 与摇杆并行                     |
| `LT OFF`          | 加速   | 松开 LT            | `/joy` 松开扳机                         | `ACK:LT OFF`          | 断连自动 OFF                  |
| `P{n}Y{m}`        | 脖子   | 步进解析             | `/request_control`+`/control_command` | 无                     | 每步 10°；兼发 `/pi_plus_absolute` |
| `neck0`           | 脖子   | 回中               | 同上                                  | 无                     | 平滑回中；`P0Y0` 同效            |
| `locate_face ON`  | 头追   | 启进程              | `locate_face_cpp`                   | `ACK:locate_face ON`  |                           |
| `locate_face OFF` | 头追   | 停进程 + 回中         | 杀进程 + 脖子回中                          | `ACK:locate_face OFF` |                           |
| `MP ON`           | 电机电源 | `power_switch=1` | `/power_switch_control`             | 原文 `MP ON`            | 长按语义                      |
| `MP OFF`          | 电机电源 | `power_switch=0` | `/power_switch_control`             | 原文 `MP OFF`           |                           |
| `sound ON`        | 语音   | 开接收              | PulseAudio 播放                       | 原文 `sound ON`         | FFE1 二进制 `0x0B` 音频包       |
| `sound OFF`       | 语音   | 停播放              | 停止音频                                | 原文 `sound OFF`        |                           |
| `V {0-100}`       | 音量   | `amixer`         | 系统 Master 音量                        | `ACK:V {n}`           |                           |
| `rename HT_{8位}`  | 广播名  | HCI 刷新           | BLE 广播名 + 存盘                        | `rename HT_…`         | 需重扫连接                     |
| `WIFI <SSID> <PWD>` | WiFi 配网 | nmcli connect    | 保存配置并连接 wlan                       | `WIFI OK` / `WIFI FAIL …` | 成功后再推 `IP:…`；约 45s 超时 |


### 4.2 模式指令

量产算法底层 FSM 与手柄解绑。`M_protect`/`M_init`/`M_resetzero` 走 `/hightorque_controller/change_fsm_state`（`states: ['protect'|'init']`；调零为 `init`→`prev`→`confirm`）。失败再用 `/joy` 兜底（init 不连按）。

### 4.3 摇杆 → `/joy` 轴


| 协议轴     | `/joy`     | 映射                         | 死区             |
| ------- | ---------- | -------------------------- | -------------- |
| X（前后 +） | `axes[1]` ly | 前为正         | \|v\| < 0.10 → 0 |
| Y（左右 +） | `axes[0]` lx | 右为负            | 同上             |
| Z（右转 +） | `axes[3]` rx | 右转为负       | 同上             |


### 4.4 实现文件索引


| 模块        | 文件                           | 职责                    |
| --------- | ---------------------------- | --------------------- |
| 指令分类      | `ble_command_dispatcher.py`  | FFE1 文本 → 类型 + wire   |
| 手柄解析      | `ble_gamepad.py`             | 组合键识别、custom_action 加载 |
| GATT / 握手 | `ble_gatt_server.py`         | 连接、FFE2 notify、rename |
| ROS 桥接    | `ble_ros_bridge.py`          | 摇杆 / AMP 策略 / FSM / 起立坐下 / 急停 |
| 状态下行      | `ble_status_telemetry.py`    | IP / pwr / mp |
| 脖子        | `ble_neck_bridge.py`         | `P{n}Y{m}` / `neck0`  |
| 电机电源      | `ble_motor_power_manager.py` | `MP ON/OFF`           |
| 头追        | `ble_locate_face_manager.py` | `locate_face ON/OFF`  |
| WiFi 配网     | `ble_wifi_manager.py`        | `WIFI <SSID> <PASSWORD>` |


参考小程序示例：`docs/miniprogram_ble_snippet.js` 