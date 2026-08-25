# ROS2上层控制接口-default\_bt 策略切换控制

# ROS2上层控制接口-default\_bt 策略切换控制

非手柄方式切换机器人当前运行策略。

## 1. 适用范围

本文说明如何不经过手柄，直接通过 ROS 2 service 切换 `hightorque_controller` 的 `default_bt` 策略，例如 `amp_lower`、`amp` 和 `bfm`。

底层 FSM、`default_bt` 上层状态和策略是三个不同的概念：

*   底层 FSM：`INIT`、`EXEC_DEFAULT`、`PROTECTION_SHUTDOWN` 等
    
*   `default_bt` 上层状态：`STANDING`、`STANDBY`、`RUNNING`、`SITING` 等
    
*   当前策略：`amp_lower`、`amp`、`bfm` 等
    

切换策略应使用：

```text
/hightorque_controller/switch_policy
```

不要误用上层状态服务 `/hightorque_controller/change_state` 或底层 FSM 服务 `/hightorque_controller/change_fsm_state`。

## 2. 使用前提

必须同时满足以下条件：

1.  底层 FSM 已进入 `EXEC_DEFAULT`。
    
2.  `current_mode` 为 `default_bt`。
    
3.  `current_state` 为 `STANDBY` 或 `RUNNING`。
    
4.  目标策略已在动作库中注册，并且适用于当前平台。
    

先查看当前状态：

```bash
ros2 topic echo /hightorque_controller/state
```

消息示例：

```yaml
current_mode: default_bt
current_state: standby
current_policy: bfm
```

如果尚未进入 `default_bt`，参考[《ROS2上层控制接口-底层FSM状态控制.adoc》](https://alidocs.dingtalk.com/api/doc/transit?dentryUuid=R1zknDm0WRoYpL7BTzMvo9ENVBQEx5rG&queryString=utm_medium%3Ddingdoc_doc_plugin_card%26utm_source%3Ddingdoc_doc)切换底层 FSM。如果上层状态不满足条件，参考 [《ROS2上层控制接口-default\_bt 上层状态控制.adoc》](https://alidocs.dingtalk.com/api/doc/transit?dentryUuid=qnYMoO1rWx1XZ29OU9DgAEgrV47Z3je9&queryString=utm_medium%3Ddingdoc_doc_plugin_card%26utm_source%3Ddingdoc_doc) 切换 `default_bt` 上层状态。

## 3. 服务接口

服务名称：

```text
/hightorque_controller/switch_policy
```

服务类型：

```text
hightorque_msgs/srv/SwitchPolicy
```

接口定义：

```text
string policy_name
---
bool success
string message
string current_policy
```

`policy_name` 是要切换到的目标策略名称。`success: true` 只表示切换请求已被接收，不表示策略加载和姿态过渡已经完成。

## 4. 策略名称映射

| 业务名称 | `policy_name` | 动作库用途 | Jetson 可用 |
| --- | --- | --- | --- |
| BFM | `bfm` | BFM 动作策略，当前默认策略 | 是 |
| AMP | `amp` | AMP 行走策略 | 是 |
| AMP\_LOWER | `amp_lower` | AMP 下半身行走策略，同时用作 waypoint 支持策略 | 是 |

策略名称区分大小写，必须使用小写的 `bfm`、`amp` 和 `amp_lower`。不能发送 `BFM`、`AMP` 或 `AMP_LOWER`。

## 5. 策略切换命令

### 5.1 切换到 AMP\_LOWER

```bash
ros2 service call \
  /hightorque_controller/switch_policy \
  hightorque_msgs/srv/SwitchPolicy \
  "{policy_name: 'amp_lower'}"
```

### 5.2 切换到 AMP

```bash
ros2 service call \
  /hightorque_controller/switch_policy \
  hightorque_msgs/srv/SwitchPolicy \
  "{policy_name: 'amp'}"
```

### 5.3 切换到 BFM

```bash
ros2 service call \
  /hightorque_controller/switch_policy \
  hightorque_msgs/srv/SwitchPolicy \
  "{policy_name: 'bfm'}"
```

请求被接收时，响应格式类似：

```text
response:
hightorque_msgs.srv.SwitchPolicy_Response(success=True,
message='switch policy request accepted', current_policy='amp')
```

响应中的 `current_policy` 是已接收的目标策略，不能用它判断异步切换已完成。

## 6. 不同上层状态下的切换行为

### 6.1 STANDBY 状态

在 `STANDBY` 中发送切换请求后，行为树会：

```text
验证目标策略
  -> 停止当前策略线程
  -> 加载目标策略
  -> 执行站立过程
  -> 返回 STANDBY
```

策略切换后不会因为 service 请求自动进入 `RUNNING`。

### 6.2 RUNNING 状态

当前代码允许在 `RUNNING` 中直接提交策略切换请求。行为树会尝试执行策略减速、`amp_lower` 下半身接管、停止当前策略、加载目标策略和关节过渡。

虽然该路径已经实现，真机运行时仍建议先返回 `STANDBY` 再切换，特别是目标策略为 `bfm` 时。

## 7. 切换完成确认

监听控制模式、上层状态和当前策略：

```bash
ros2 topic echo /hightorque_controller/state
```

例如，切换到 `amp` 完成后应看到：

```yaml
current_mode: default_bt
current_state: standby
current_policy: amp
```

自动化程序应等待 topic 中的 `current_policy` 变成目标策略，再发送后续命令。不要仅根据 service 响应中的 `success: true` 判断切换完成。

## 8. 推荐操作顺序

真机上建议使用以下顺序：

1.  使用吊架或其他方式可靠固定机器人。
    
2.  确认 `current_mode` 为 `default_bt`。
    
3.  如果当前为 `RUNNING`，发送 `toggle_policy` 并等待 `STANDBY`。
    
4.  调用 `/hightorque_controller/switch_policy`。
    
5.  等待 `current_policy` 变成目标策略，并确认上层状态为 `STANDBY`。
    
6.  需要运行策略时，发送 `toggle_policy`。
    
7.  等待 `current_state` 变成 `RUNNING`。
    

完整流程：

```text
RUNNING
  -> toggle_policy
  -> 等待 STANDBY
  -> switch_policy
  -> 等待 current_policy 更新
  -> toggle_policy
  -> 等待 RUNNING
```

## 9. 常见错误与安全说明

| 响应消息 | 原因 | 处理方式 |
| --- | --- | --- |
| `default_bt controller is not active` | 底层 FSM 未进入 `EXEC_DEFAULT` | 先切换底层 FSM |
| `policy_name is empty` | 未提供策略名称 | 填写有效的 `policy_name` |
| `unknown or unavailable policy: ...` | 名称错误或策略未加载 | 检查小写策略名和动作库 |
| `policy switch rejected: controller is not in STANDBY or RUNNING state` | 上层状态不允许切换 | 等待或切换到 `STANDBY` |

真机操作时还需要注意：

1.  不要在 `STANDING`、`SITING`、`TRAJ` 或 waypoint 并行执行过程中强制切换策略。
    
2.  发送请求前先检查 `current_policy`，不要重复切换到已加载的策略。
    
3.  每次只发送一个策略切换请求，确认完成后再执行下一步。
    
4.  切换到 `bfm` 前应确认 BFM motion source 和观测数据正常；在 `STANDBY` 切换后再启动策略，可以让行为树执行观测检查。
    
5.  service 接口不会代替真机安全防护，切换前必须确保机器人姿态、支撑和周边环境安全。