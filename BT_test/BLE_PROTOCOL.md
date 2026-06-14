# Bird BLE 通讯协议

> 全链路 UTF-8 纯文本，单条不含换行，经 **FFE2 notify** / **FFE1 write** 传输。

| UUID | 方向 | 用途 |
|------|------|------|
| FFE0 | — | 服务 |
| FFE1 | 小程序 → 机器人 | 控制指令 |
| FFE2 | 机器人 → 小程序 | ACK + 状态遥测 |

终端日志：**RX 红色**（收到）、**TX 绿色**（发出）。

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
| 步态 | `LT+RT+LB` |
| 卸力 | `LT+RT+B` |

### 1.4 指令 ACK（FFE2 notify）

```text
ACK:{原文}
```

仅模式/动作有 ACK；摇杆无。

---

## 二、下行：机器人 → 小程序（FFE2 notify）

订阅 FFE2 后，板端主动推送状态。

### 2.1 局域网 IP

```text
IP:19.11
```

| 规则 | 说明 |
|------|------|
| 格式 | `IP:` + 末两段（如 `192.168.19.11` → `19.11`） |
| 时机 | 订阅后推送一次；IP 变化时再推 |

### 2.2 电量

```text
pwr:50
```

| 规则 | 说明 |
|------|------|
| 格式 | `pwr:` + 0~100 整数 |
| 步进 | 按 **5%** 量化（100、95、90、85…） |
| 时机 | 订阅后推当前值；**电量每下降 5%** 再推（上升不推） |
| 数据源 | sysfs `/sys/class/power_supply/*/capacity` 或 ROS `/pwr`、`/battery_percent`、`/battery_state` |

### 2.3 机器 FSM 模式

```text
fsm:5
```

| 规则 | 说明 |
|------|------|
| 格式 | `fsm:` + `/fsm_state` 整型值 |
| 时机 | **状态变化时连发 3 次**（间隔 50ms，防丢包） |
| 订阅时 | 推送当前 FSM 一次 |

### 2.4 FSM 状态对照（参考）

| 值 | 含义 |
|----|------|
| 0 | INIT |
| 5 | EXEC_DEFAULT |
| 8 | PROTECTION_SHUTDOWN |
| … | 见 `ble_ros_bridge.py` FSM_STATE_NAMES |

---

## 三、连接流程

```
扫描 FFE0 → createBLEConnection → setBLEMTU(247)
→ 发现特征 → 订阅 FFE2
→ write FFE1 "M_default"
→ 收到 IP:/pwr:/fsm: 状态包
→ 20Hz 摇杆 writeNoResponse
```

---

## 四、板端 ROS 映射

| 上行指令 | ROS |
|----------|-----|
| 摇杆 | `/cmd_vel` 20Hz |
| 模式/动作 | `/joy_msg` |

参考代码：`docs/miniprogram_ble_snippet.js`
