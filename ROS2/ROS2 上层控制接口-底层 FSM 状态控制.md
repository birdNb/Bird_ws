# ROS2 上层控制接口-底层 FSM 状态控制

# ROS2 上层控制接口-底层 FSM 状态控制

更新日期：2026-08-21

非手柄方式控制机器人底层状态。

## 1. 适用范围

本文说明如何不经过手柄，直接通过 ROS 2 service 控制 `hightorque_controller` 的底层 FSM 状态。

底层 FSM 与 `default_bt` 上层状态是两套不同的状态：

*   底层 FSM：`INIT`、`EXEC_DEFAULT`、`PROTECTION_SHUTDOWN` 等
    
*   `default_bt` 上层状态：`STANDING`、`STANDBY`、`RUNNING`、`SITING` 等
    

控制底层 FSM 应使用：

```text
/hightorque_controller/change_fsm_state
```

不要误用控制 `default_bt` 上层状态的：

```text
/hightorque_controller/change_state
```

## 2. 完整底层 FSM 状态名称映射

| 业务名称 | FSM 状态 | 数值 | 控制模式 `current_mode` | 进入方式 | 备注 |
| --- | --- | --- | --- | --- | --- |
| INITIAL | `INIT` | 0 | `init` | 直接发送 `init` | 进入候选前需要先进入`INIT` |
| ERROR | `ERROR` | 1 | `error` | 电机位置异常时自动进入 |  |
| DEFAULT 候选 | `CANDIDATE_DEFAULT` | 2 | `none` | `next` 或 `prev` 选择 | 需要先进入`INIT`才可以进入候选 |
| DEFAULT | `EXEC_DEFAULT` | 5 | `default_bt` | 发送 `default_bt`，或确认 `CANDIDATE_DEFAULT` | 控制器启动后默认进入 |
| PROTECT | `PROTECTION_SHUTDOWN` | 8 | `protect` | 发送 `protect` |  |
| RESETZERO 候选 | `CANDIDATE_RESET_ZERO` | 9 | `none` | `next` 或 `prev` 选择 |  |
| RESETZERO | `EXEC_RESET_ZERO` | 10 | `reset_zero` | 确认 `CANDIDATE_RESET_ZERO` | 执行完成后自动进入成功或失败状态 |
| RESET DONE | `EXEC_RESET_ZERO_SUCCESSFULLY` | 11 | `none` | RESET\_ZERO 成功后自动进入 | 保持约 3 秒后自动回到 `INIT` |
| RESET FAIL | `EXEC_RESET_ZERO_FAILED` | 12 | `none` | RESET\_ZERO 失败后自动进入 | 保持约 3 秒后自动回到 `INIT` |

代码中实际使用 `INIT`，不是 `INITIAL`。当前版本已经移除 `CUSTOM`、`REMOTE`、`TEACHING` 和 `DEVELOP` 相关状态，不能再使用这些状态名称或按旧顺序选择。

`/fsm_state` 仍保留历史数值兼容性，因此数值 `3`、`4`、`6`、`7`、`13`、`14`、`15`、`16` 没有对应的当前 FSM 状态，不应作为有效状态使用。

所有 `CANDIDATE_*` 状态同样映射为 `none`。使用 `next` 或 `prev` 进入候选状态时，当前控制器会被停止；真机上进行候选状态选择前必须确保机器人已经可靠固定。

控制器启动时，FSM 管理器会将当前状态设为 `EXEC_DEFAULT`，对应控制模式为 `default_bt`。

## 3. 服务接口

服务名称：

```text
/hightorque_controller/change_fsm_state
```

服务类型：

```text
hightorque_msgs/srv/ChangeState
```

接口定义：

```text
string[] states
---
bool success
string message
string current_mode
string current_state
string current_policy
```

当前实现只处理 `states[0]`。一次调用只能发送一条命令，多步切换必须分多次调用。

当前 `change_fsm_state` 服务处理函数明确填写的是 `success` 和 `message`；其余三个响应字段可能保持为空。状态确认应以 `/hightorque_controller/state` 和 `/fsm_state` 的实际发布值为准。

## 4. 常用切换命令

### 4.1 DEFAULT 切换到 PROTECT

```bash
ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['protect']}"
```

预期结果：

```text
response:
hightorque_msgs.srv.ChangeState_Response(success=True, message='fsm command accepted: protect, current state: PROTECTION_SHUTDOWN')
```

### 4.2 PROTECT 切换到 INIT

```bash
ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['init']}"
```

预期结果：

```text
response:
hightorque_msgs.srv.ChangeState_Response(success=True, message='fsm command accepted: protect, current state: INIT')
```

在 `PROTECT` 状态再次发送 `protect` 也会切换到 `INIT`，但自动化程序应优先使用语义明确的 `init`。

### 4.3 INIT 切换到 DEFAULT

```bash
ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['default']}"
```

预期结果：

```text
response:
hightorque_msgs.srv.ChangeState_Response(success=True, message='fsm command accepted: default, current state: EXEC_DEFAULT')
```

## 5. 状态确认

监听控制模式、`default_bt` 上层状态和当前策略：

```bash
ros2 topic echo /hightorque_controller/state
```

消息示例：

```yaml
current_mode: default_bt
current_state: init
current_policy: ''
```

监听底层 FSM 数值：

```bash
ros2 topic echo /fsm_state
```

自动化控制程序不应只依赖 service 返回值。每次调用后还应等待 `/hightorque_controller/state` 的 `current_mode` 变成目标模式，最好再通过 `/fsm_state` 的返回值确认，然后执行下一步。

## 6. 底层 FSM 控制命令

当前 `change_fsm_state` 支持以下命令：

| 命令 | 别名 | 行为 |
| --- | --- | --- |
| `default` | `center` | 直接进入 `EXEC_DEFAULT` |
| `protect` | `toggle_protect` | 非 `INIT` 状态进入保护；在保护状态下切换到 `INIT`；在 `INIT` 状态下无动作 |
| `init` | 无 | 直接进入 `INIT` |
| `next` | `right` | 向后选择一个候选状态 |
| `prev` | `left` | 向前选择一个候选状态 |
| `confirm` | 无 | 确认当前候选状态 |

`reset_zero` 不能直接用同名命令进入，需要通过 `next`、`prev` 和 `confirm` 选择。`custom`、`remote`、`teaching` 和 `develop` 已不再是当前版本支持的 FSM 命令或状态。

## 7. 候选状态选择顺序

当内部 `candidateState_` 为 `INIT` 时，连续发送 `next` 的顺序是：

```text
CANDIDATE_DEFAULT
  -> CANDIDATE_RESET_ZERO
  -> CANDIDATE_DEFAULT
```

连续发送 `prev` 时的顺序是：

```text
CANDIDATE_RESET_ZERO
  -> CANDIDATE_DEFAULT
  -> CANDIDATE_RESET_ZERO
```

建议先发送 `init`，将内部候选值重置为 `INIT`，再进行选择。以 `INIT` 为起点时：

| 目标执行状态 | `next` 次数 | `prev` 次数 | 最后命令 |
| --- | --- | --- | --- |
| `EXEC_DEFAULT` | 1 | 2 | `confirm` |
| `EXEC_RESET_ZERO` | 2 | 1 | `confirm` |

例如，在内部候选值确定为 `INIT` 的前提下，进入 RESET\_ZERO：

```bash
ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['prev']}"

ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['confirm']}"
```

从 `INIT` 进入 DEFAULT 候选并确认：

```bash
ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['next']}"

ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['confirm']}"
```

从 `INIT` 进入 RESET\_ZERO 候选并确认：

```bash
ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['init']}"

ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['prev']}"

ros2 service call \
  /hightorque_controller/change_fsm_state \
  hightorque_msgs/srv/ChangeState \
  "{states: ['confirm']}"
```

`transitionTo(INIT)` 会同时把内部 `candidateState_` 重置为 `INIT`，因此自动化程序可以在每次选择前先发送 `init`，再按上述次数发送 `next` 或 `prev`。

## 8. 真机安全说明

### 8.1 PROTECT

`PROTECT` 对所有已取得控制权的电机使用：

```text
kp = 0
kd = 1
```

这是阻尼控制，不是断电，也不是位置保持。

### 8.2 INIT

`INIT` 对所有已取得控制权的电机发送：

```text
kp = 0
kd = 0
tau = 0
```

这是零力矩状态。机器人可能立即失去支撑，因此执行 `PROTECT -> INIT` 前必须使用吊架、支撑装置或安排人员可靠扶持机器人。

## 9. 推荐操作顺序

1.  使用吊架或其他方式固定机器人。
    
2.  启动控制器并监听 `/hightorque_controller/state` 和 `/fsm_state`。
    
3.  每次只发送一条底层 FSM 命令。
    
4.  等待 `current_mode` 确认切换成功。
    
5.  确认机器人姿态和电机状态正常后再进行下一次切换。
    
6.  不要根据 service 的 `success: true` 直接连续发送后续命令。