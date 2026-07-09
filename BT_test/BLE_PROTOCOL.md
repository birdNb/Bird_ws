# Bird BLE 通讯协议

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


| 功能     | 指令            | 底层组合键（`/joy_msg`）         |
| ------ | ------------- | ------------------------- |
| 起立     | `LT+RT+start` | `lt+rt+start` 长按 1s       |
| 蹲下     | `LT+RT+RB`    | `lt+rt+rb` 长按 1s          |
| 卸力     | `LT+RT+B`     | `lt+rt+b` 长按 1s           |
| 挥双手    | `RT+A`        | `rt+a` 短脉冲 ~0.35s         |
| 挥单手    | `RT+X`        | `rt+x` 短脉冲 ~0.35s         |
| 小脚踢球   | `A`           | `a` 短脉冲（`byd_small_kick`） |
| 秀肌肉    | `X`           | `x` 短脉冲（`byd_power`）      |
| byd_bb | `LT+RT+DPU`   | `lt+rt+dpu` 短脉冲           |
| 猪猪侠    | `LT+RT+DPR`   | `lt+rt+dpr` 短脉冲           |
| 踢腿     | `LT+DPR`      | `lt+dpr` 短脉冲              |
| 重拳     | `LT+DPD`      | `lt+dpd` 短脉冲              |
| 上勾拳    | `LT+DPL`      | `lt+dpl` 短脉冲              |
| 步态开启   | `GAIT ON`     | `lt+rt+lb` 长按 1s（步态开）     |
| 步态关闭   | `GAIT OFF`    | `lt+rt+lb` 长按 1s（步态关）     |


步态（`GAIT ON/OFF`）、电机电源（`MP ON/OFF`）收到后，板端经 FFE2 **原样回传相同指令**（无 `ACK:` 前缀）作为确认。

**倒地保护（FSM=8/1）特殊规则：**

- `GAIT OFF` 会被板端**拒绝**（避免跌倒后无法起立）
- `LT+RT+start` 起立时自动先执行 `GAIT ON`（`lt+rt+lb`），再执行起立组合键

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
P0Y0      # 无偏移
neck0     # 回中
```

板端发布 `/pi_plus_absolute`（`head_yaw_joint` / `head_pitch_joint`）。

### 1.5 人脸跟踪（locate_face）


| 指令                | 说明                                       |
| ----------------- | ---------------------------------------- |
| `locate_face ON`  | 启动 `locate_face_cpp`（默认后台；`--gui` 才显示预览） |
| `locate_face OFF` | 停止头追进程                                   |


视觉伺服脖子跟随；与手动脖子步进共用 `/pi_plus_absolute`，建议二选一使用。

### 1.6 电机电源（MP）


| 指令       | 小程序操作  | 说明                           |
| -------- | ------ | ---------------------------- |
| `MP ON`  | **长按** | 接通电机供电（功率板 `power_switch=1`） |
| `MP OFF` | **点击** | 断开电机供电（功率板 `power_switch=0`） |


板端发布 ROS `/power_switch_control`（`livelybot_power/Power_switch`，`control_switch=1`）。收到后 FFE2 **原样回传** `MP ON` / `MP OFF` 确认（无 `ACK:` 前缀）。

### 1.8 疾跑（LT）


| 指令       | 说明                                                                           |
| -------- | ---------------------------------------------------------------------------- |
| `LT ON`  | 开启疾跑：持续向 `/joy_msg` 发布 `lt=-1.0`（模拟按住 LT），AMP Soccer 策略最大前进速度由 0.80 提升至 1.50 |
| `LT OFF` | 关闭疾跑：松开 LT                                                                   |


- 与摇杆 `/cmd_vel` 并行；疾跑开启时 `/cmd_vel` 线速度额外 ×1.5
- 断连自动关闭疾跑并松开 LT
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

### 1.11 指令 ACK（FFE2 notify）

```text
ACK:{原文}
```

模式、动作、locate_face、LT 疾跑、V 有 `ACK:` 回执；步态、电机电源、**sound ON/OFF** 为**原文回显**；摇杆、脖子无回执。

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
| 连接后 | **FFE2 订阅成功 5 秒后**，连发 **3 次**（间隔 1s）                      |
| 运行中 | `/power_switch_state` 变化时再推                               |
| 数据源 | ROS `/power_switch_state`（`livelybot_power/Power_switch`） |


### 2.4 机器 FSM 模式

```text
fsm:5
```


| 规则  | 说明                           |
| --- | ---------------------------- |
| 格式  | `fsm:` + `/fsm_state` 整型值    |
| 时机  | **状态变化时连发 3 次**（间隔 50ms，防丢包） |
| 订阅时 | 推送当前 FSM 一次                  |


### 2.5 FSM 状态对照（参考）


| 值   | 含义                                      |
| --- | --------------------------------------- |
| 0   | INIT                                    |
| 5   | EXEC_DEFAULT                            |
| 8   | PROTECTION_SHUTDOWN                     |
| 14  | EXEC_TEACHING                           |
| …   | 见 `ble_ros_bridge.py` `FSM_STATE_NAMES` |


### 2.6 下行数据汇总


| 类型   | 格式                          | 发送时机                  |
| ---- | --------------------------- | --------------------- |
| IP   | `IP:192.168.19.11`          | 订阅时 + IP 变化           |
| 电量   | `pwr:83`                    | 订阅时立即推；之后电量下降再推       |
| 电机电源 | `mp:ON` / `mp:OFF`          | 连接 5s 后连发 3 次；状态变化时再推 |
| FSM  | `fsm:5`                     | 变化时连发 3 次；订阅时 1 次     |
| ACK  | `ACK:M_default`             | 模式/动作/脖子等指令回执         |
| 原文确认 | `MP ON/OFF` / `GAIT ON/OFF` | 电机电源、步态：回传与上行相同指令     |


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

## 四、指令 → 动作映射总表（板端实现）

> 小程序经 **FFE1 write** 发 UTF-8 文本；板端 `ble_command_dispatcher` 分类后，由 `ble_gatt_server` / `ble_ros_bridge` 等模块执行。  
> 与实体手柄一致的组合键均发布到 `**/joy_msg`**（`sim2real_msg/Joy`）；摇杆发布 `**/cmd_vel**`。

### 4.1 总览


| FFE1 指令           | 分类   | 底层触发             | ROS / 系统动作                          | FFE2 回执               | 备注                        |
| ----------------- | ---- | ---------------- | ----------------------------------- | --------------------- | ------------------------- |
| `X:…,Y:…,Z:…`     | 摇杆   | 解析 XYZ           | `/cmd_vel` 20Hz                     | 无                     | 超时 0.2s 后停止发布；疾跑时线速度 ×1.5 |
| `M_default`       | 模式   | `center` 等菜单序列   | `/joy_msg` → FSM **5** EXEC_DEFAULT | `ACK:M_default`       | 进遥控页握手首发；同步处理             |
| `M_init`          | 模式   | `lt+rt+b`（视 FSM） | `/joy_msg` → 进入 Init 菜单             | `ACK:M_init`          |                           |
| `M_protect`       | 模式   | `lt+rt+b`        | `/joy_msg` → 保护/卸力                  | `ACK:M_protect`       |                           |
| `M_resetzero`     | 模式   | 菜单导航 + `lt+rt+a` | `/joy_msg` → 调零候选/执行                | `ACK:M_resetzero`     | 调零中 FSM=10 会等待完成          |
| `M_tech`          | 模式   | 菜单导航 + 确认        | `/joy_msg` → 示教候选/执行                | `ACK:M_tech`          |                           |
| `LT+RT+start`     | 动作   | `lt+rt+start` 1s | `/joy_msg` 起立                       | `ACK:LT+RT+start`     | 保护态先 `lt+rt+lb` 开步态       |
| `LT+RT+RB`        | 动作   | `lt+rt+rb` 1s    | `/joy_msg` 蹲下                       | `ACK:LT+RT+RB`        |                           |
| `LT+RT+B`         | 动作   | `lt+rt+b` 1s     | `/joy_msg` 卸力                       | `ACK:LT+RT+B`         |                           |
| `RT+A`            | 动作   | `rt+a` 短脉冲       | `/joy_msg` 挥双手                      | `ACK:RT+A`            | 冷却 8s                     |
| `RT+X`            | 动作   | `rt+x` 短脉冲       | `/joy_msg` 挥单手                      | `ACK:RT+X`            | 冷却 8s                     |
| `A`               | 自定义  | `a` 短脉冲          | `/joy_msg` `byd_small_kick`         | `ACK:A`               | 冷却 8s                     |
| `X`               | 自定义  | `x` 短脉冲          | `/joy_msg` `byd_power`              | `ACK:X`               | 冷却 8s                     |
| `LT+RT+DPU`       | 自定义  | `lt+rt+dpu`      | `/joy_msg` `byd_bb`                 | `ACK:LT+RT+DPU`       | 冷却 8s                     |
| `LT+RT+DPR`       | 自定义  | `lt+rt+dpr`      | `/joy_msg` `byd_zzx`                | `ACK:LT+RT+DPR`       | 冷却 8s                     |
| `LT+DPR`          | 自定义  | `lt+dpr`         | `/joy_msg` `byd_zhidengtui`         | `ACK:LT+DPR`          | 冷却 8s                     |
| `LT+DPD`          | 自定义  | `lt+dpd`         | `/joy_msg` `byd_zhongquan`          | `ACK:LT+DPD`          | 冷却 8s                     |
| `LT+DPL`          | 自定义  | `lt+dpl`         | `/joy_msg` `byd_shanggouquan`       | `ACK:LT+DPL`          | 冷却 8s                     |
| `GAIT ON`         | 步态   | `lt+rt+lb` 1s    | `/joy_msg` 步态开                      | 原文 `GAIT ON`          |                           |
| `GAIT OFF`        | 步态   | `lt+rt+lb` 1s    | `/joy_msg` 步态关                      | 原文 `GAIT OFF`         | FSM=8/1 时拒绝               |
| `LT ON`           | 疾跑   | `lt` 持续按住        | `/joy_msg` lt=-1                    | `ACK:LT ON`           | 与摇杆并行                     |
| `LT OFF`          | 疾跑   | 松开 `lt`          | `/joy_msg` lt 释放                    | `ACK:LT OFF`          | 断连自动 OFF                  |
| `P{n}Y{m}`        | 脖子   | 步进解析             | `/pi_plus_absolute`                 | 无                     | 每步 10°；`P0Y0` 平滑回中        |
| `neck0`           | 脖子   | 回中               | `/pi_plus_absolute`                 | 无                     | 平滑回中                      |
| `locate_face ON`  | 头追   | 启进程              | `locate_face_cpp`                   | `ACK:locate_face ON`  |                           |
| `locate_face OFF` | 头追   | 停进程 + 回中         | 杀进程 + 脖子回中                          | `ACK:locate_face OFF` |                           |
| `MP ON`           | 电机电源 | `power_switch=1` | `/power_switch_control`             | 原文 `MP ON`            | 长按语义                      |
| `MP OFF`          | 电机电源 | `power_switch=0` | `/power_switch_control`             | 原文 `MP OFF`           |                           |
| `sound ON`        | 语音   | 开接收              | PulseAudio 播放                       | 原文 `sound ON`         | FFE1 二进制 `0x0B` 音频包       |
| `sound OFF`       | 语音   | 停播放              | 停止音频                                | 原文 `sound OFF`        |                           |
| `V {0-100}`       | 音量   | `amixer`         | 系统 Master 音量                        | `ACK:V {n}`           |                           |
| `rename HT_{8位}`  | 广播名  | HCI 刷新           | BLE 广播名 + 存盘                        | `rename HT_…`         | 需重扫连接                     |


### 4.2 模式指令 → 手柄序列（依当前 FSM 动态生成）


| 指令            | 目标      | 典型 `/joy_msg` 序列             | 目标 FSM                |
| ------------- | ------- | ---------------------------- | --------------------- |
| `M_default`   | 默认执行态   | `center`（菜单中 A 确认）           | 5 EXEC_DEFAULT        |
| `M_init`      | Init 菜单 | 保护态：`lt+rt+b`；否则：`lt+rt+b`×2 | 0 INIT                |
| `M_protect`   | 保护/卸力   | `lt+rt+b`                    | 8 PROTECTION_SHUTDOWN |
| `M_resetzero` | 调零      | 进 Init → 导航到校准项 → `lt+rt+a`  | 9→10 校准               |
| `M_tech`      | 示教      | 进 Init → 导航到示教项 → `lt+rt+a`  | 13→14 示教              |


菜单导航键：`lt+rt+→` 下一项、`lt+rt+←` 上一项、`lt+rt+a` 确认、`center` 默认模式确认。

### 4.3 摇杆 → `/cmd_vel` 字段


| 协议轴     | `/cmd_vel`  | 缩放             | 死区             |
| ------- | ----------- | -------------- | -------------- |
| X（前后 +） | `linear.x`  | ×1.5（疾跑 ×1.5）  | |v| < 0.10 → 0 |
| Y（左右 +） | `linear.y`  | ×0.7（疾跑 ×1.5）  | 同上             |
| Z（右转 +） | `angular.z` | ×1.57（疾跑 ×1.2） | 同上             |


### 4.4 实现文件索引


| 模块        | 文件                           | 职责                    |
| --------- | ---------------------------- | --------------------- |
| 指令分类      | `ble_command_dispatcher.py`  | FFE1 文本 → 类型 + wire   |
| GATT / 握手 | `ble_gatt_server.py`         | 连接、FFE2 notify、rename |
| ROS 桥接    | `ble_ros_bridge.py`          | 摇杆、模式、动作、步态、疾跑        |
| 脖子        | `ble_neck_bridge.py`         | `P{n}Y{m}` / `neck0`  |
| 电机电源      | `ble_motor_power_manager.py` | `MP ON/OFF`           |
| 头追        | `ble_locate_face_manager.py` | `locate_face ON/OFF`  |
| 状态下行      | `ble_status_telemetry.py`    | IP / pwr / mp / fsm   |


参考小程序示例：`docs/miniprogram_ble_snippet.js`