# 自动调零方案：力矩碰限位 + 电机写零

本文给出可行性结论，以及落地实现方式。核心流程：**力矩（或限力矩）朝固定方向顶到硬限位 → 用限位位置反推应对齐的机械零位 → 把该关节走到该位置 → 只对该电机发官方写零并保存。**

结论：**可行**，且与现有调零栈一致。零点仍写入**电机内部 Flash**，不会生成板端 `zero.yaml`。

**本 demo 只覆盖 4 自由度手臂**（左右各 4 轴，无腕、无夹爪），与当前实机 `S-12L8A0G2H0W`（`arm_dof: 4`）一致。不要用 `5_DOF_ARM_IK` 的腕关节列表。

---

## 1. 可行性（针对本机 SDK）

本机 `livelybot_serial` 协议已具备所需指令：

| 指令 | 码 | 作用 |
| --- | --- | --- |
| `MODE_TORQUE` | `0x82` | 纯力矩控制，可用来缓慢顶限位 |
| `MODE_POS_VEL_TQE` | `0x90` | 位置/速度 + **最大力矩**，更适合寻限位 |
| `MODE_RESET_ZERO` | `0x01` | 把**当前电机角**设为 0 |
| `MODE_CONF_WRITE` | `0x02` | 把电机设置（含零点）写入 Flash |

对应 C++ 接口：

- 寻限位：`motor::torque()` 或 `motor::pos_vel_MAXtqe(pos, vel, torque_max)`
- 单电机写零：`canport::set_reset_zero(id)` / `robot::set_reset_zero({motor_index})`
- 保存：`canport::set_conf_write(id)`（`set_reset_zero` 内部一般会等待电机保存；日志为 *awaiting settings save*）

**SDK 不支持「指定任意编码器值作为零点」。** 只能把**发令那一瞬间的当前位置**写成 0。因此不能在限位处直接写零就当运动学零，除非限位本身就是出厂校准姿态。

正确含义是：

1. 硬限位只是可重复路标。
2. 「对应位置」= 出厂/控制栈约定的**写零姿态**（电机 0 应对齐的那套机械角），不是 URDF 软件限位，也不一定是 `q=0`。
3. 从限位反推该姿态，**先走到那里，再对该电机 `MODE_RESET_ZERO`**。

写零后 `pd.yaml` 的 `urdf_offset` / `direction` **不要改**。它们是电机零 → URDF 的固定映射；出厂校准也是在同一写零姿态上执行 `set_reset_zero()`。若在限位处写零，`urdf_offset` 会整体错掉。

当前实机（`~/.sim2real_info.json`）为 **PiPlus `S-12L8A0G2H0W`**，运行配置：

`/home/hightorque/sim2real/install/share/sim2real/robot_config/pi_plus-S-12L8A0G2H0W/pd.yaml`

单臂 4 关节（`pd.yaml` / `robot_param.yaml`）：

| 顺序 | 左臂 | 右臂 |
| --- | --- | --- |
| 1 肩 pitch | `l_shoulder_pitch_joint` | `r_shoulder_pitch_joint` |
| 2 肩 roll | `l_shoulder_roll_joint` | `r_shoulder_roll_joint` |
| 3 上臂 yaw | `l_upper_arm_joint` | `r_upper_arm_joint` |
| 4 肘 | `l_elbow_joint` | `r_elbow_joint` |

无 `*_wrist_joint`、无 `*_claw_joint`。`pd.yaml` 共 22 自由度：腿 12 + 臂 8 + 头 2。

手臂 `urdf_offset`（2.1.1，与电机零配套，调零时只读不改）：

```
左:  1.95, -1.57,  0.00, -1.57
右:  1.95,  1.57,  0.00, -1.57
```

自动调零必须复现「与这份 offset 配套」的电机零，而不是另起一套应用层补偿。

---

## 2. 原理

设某关节：

```
q_enc          : 写零前电机反馈（rad）
q_enc_limit    : 堵转采样得到的限位编码器角
q_travel       : 写零姿态 → 该侧硬限位 的已知行程（CAD/实测，带符号）
q_enc_home     : 写零姿态在当前编码器下的位置
```

```
q_enc_home = q_enc_limit - q_travel
```

`q_travel` 与寻限位方向同号：朝正限位则 `q_travel > 0`。

然后：

1. 卸力并离开挡块（backoff）。
2. 把该关节从当前位置相对移动到 `q_enc_home`（写零前坐标仍有效）。
3. 确认到位、力矩接近空载。
4. **仅该电机** `MODE_RESET_ZERO` + 等待 `MODE_CONF_WRITE` 成功。
5. 写零后该处反馈应跳到约 `0`；上层仍走原来的 `direction` + `urdf_offset`。

几何关系（单侧限位）：

```
硬限位  ----q_travel----  写零姿态(电机0)  ----urdf_offset----  URDF 零
                ↑ 力矩顶到这里采样                 ↑ 现有 pd.yaml 已包含
```

---

## 3. 实现方式

### 3.1 推荐控制：限力矩顶限位，而不是裸力矩

裸 `MODE_TORQUE` 在检测失败时会持续加速，风险高。推荐：

- **主方案**：低速位置斜坡 + `torque_max = 1.0 N·m`（`pos_vel_MAXtqe`）。顶到挡块后位置跟不上目标，力矩顶满，用 `|τ|≥1` 且 `|q̇|≈0` 判到位。
- **备选**：小力矩开环（例如 `0.3→1.0 N·m` 斜坡），仅在位置环不可用时使用，必须有行程/时间硬超时。

力矩反馈：`/error_joint_states.effort` 或 `/livelybot_real_real/<joint>_controller/state` 的 `MotorState.tau`（N·m）。

### 3.2 运行环境

| 项 | 要求 |
| --- | --- |
| FSM | 建议 `EXEC_DEVELOP(16)`，能直接对单电机发力矩/写零；禁止行走、保护卸力、官方校准 FSM=10 并行 |
| 互斥 | 停掉 `/pi_plus_absolute` 其它发布者（头追、IK、BLE 脖子） |
| 写零粒度 | **按电机 ID / 关节名逐个写**，禁止 `set_reset_zero()` 全机 |
| 其它关节 | 保持安全折叠位，位置环锁住 |

官方手柄调零（BLE `M_resetzero`，FSM 9→10）也是 `set_reset_zero()` 写电机。本 demo 与它写同一处 Flash，**不要两套同时跑**。

### 3.3 单关节状态机

一次只动一个关节。

```
IDLE
  → PREP            读 q0、τ_bias；确认该关节 motor id；其它轴 hold
  → SEEK            沿 seek_dir 低速运动，τ 上限 1 N·m
  → STALL_HOLD      |τ|≥1 N·m 且 |q̇|<ε，保持 150～300 ms
  → SAMPLE          堵转窗口位置中位数 → q_enc_limit
  → BACKOFF         反向离开挡块 2°～5°，力矩收回
  → MOVE_HOME       运动到 q_enc_home = q_enc_limit - q_travel
  → SETTLE          到位、|τ| 回到空载附近
  → RESET_ZERO      该电机 MODE_RESET_ZERO
  → CONF_WRITE      等待保存成功；读回位置应 ≈ 0
  → VERIFY          允许误差（建议 |q_fb|<2°）；再小范围活动确认方向
  → DONE / FAIL
```

`FAIL` 时**不得**对该电机写零，并立即 `MODE_STOP` / 卸力。触发条件：

- 超时未堵转，或走过超过软件行程上限仍无 1 N·m。
- `|τ| ≥ τ_abort`（建议 2 N·m）。
- `min_travel` 未满足就判堵（静摩擦/重力误触发）。
- `q_enc_home` 相对起点不合理（算出的零位还在限位上、或要转过整圈）。
- `RESET_ZERO` / `CONF_WRITE` 失败，或写零后位置未跳到 0 附近。
- 写零过程中该电机通讯丢失。

### 3.4 堵转判定

```
stall = (|τ_filt| ≥ 1.0) AND (|q̇| < 0.02 rad/s) 连续 ≥ 200 ms
        AND (已走过 ≥ min_travel)
```

| 参数 | 初值 |
| --- | --- |
| `τ_stall` | 1.0 N·m |
| `τ_abort` | 2.0 N·m |
| `T_stall` | 200 ms |
| `min_travel` | 5° |
| `Δq_backoff` | 2°～5° |

重力轴（肩 pitch）寻限位前采 0.3 s `τ_bias`，用增量 `|τ−τ_bias|`，或提高该轴阈值。必须带速度≈0，避免尖峰误触发。

本机手臂无腕、无夹爪，调零关节就是上表 8 个臂电机。1 N·m 仍建议按轴微调（肘比肩更易顶死）。

### 3.5 「对应位置」怎么算、怎么走到

每个关节在 `config.yaml` 配置相对**写零姿态**（不是 URDF limit）的限位行程：

```yaml
# 仅 4 自由度臂；左右对称各 4 项
joints:
  - name: r_shoulder_pitch_joint
    motor_index: 16          # 与 robot_param / Motors[] 对齐，实机核对
    seek_dir: -1
    q_travel_rad: <实测>     # 写零姿态走到该侧硬限位的有符号角
    tau_stall_nm: 1.0
  - name: r_shoulder_roll_joint
    ...
  - name: r_upper_arm_joint
    ...
  - name: r_elbow_joint
    ...
  # 左臂同名四轴：l_shoulder_pitch/roll、l_upper_arm、l_elbow
```

`q_travel` 首次可用 CAD，但必须以实机为准：在已正确出厂调零的机器人上，走一次寻限位，记录 `q_enc_limit`（此时写零姿态约为 0，故 `q_travel ≈ q_enc_limit`）。之后乱零/重装电机，用保存的 `q_travel` 反推。

走到 `q_enc_home` 时仍用**写零前**的编码器。写零瞬间位置环必须先停，否则反馈跳 0 会让位置环猛拉。

顺序建议：

```
停该轴指令 → RESET_ZERO → 等保存成功 → 确认 pos≈0 → 再恢复位置保持（目标改为 0）
```

### 3.6 多关节顺序

每条臂 4 轴，顺序：**肘 → 上臂 → 肩 roll → 肩 pitch**（远端先、近端后）。

1. 寻限位前把已完成关节收到安全折叠位，不要停在挡块上。
2. 肩 pitch/roll 寻限位时，肘应收起，避免手打躯干。
3. 左右臂不要同时堵转。
4. 本 demo 默认只调手臂；头（`head_yaw` / `head_pitch`）若要做，单独批次，不要和臂并行。

### 3.7 模块划分

| 模块 | 职责 |
| --- | --- |
| `config.yaml` | 关节、电机 id、方向、`q_travel`、1 N·m、超时、顺序、安全位 |
| `torque_seek` | 限力矩寻限位 + 堵转检测 |
| `home_mover` | backoff 后运动到 `q_enc_home` |
| `motor_zero` | 单电机 `MODE_RESET_ZERO` + `MODE_CONF_WRITE`，校验跳零 |
| `safety` | 超时、τ_abort、行程帽、急停、FAIL 不写 Flash |
| `log` | 记录 `q_enc_limit`、`q_travel`、写零前后位置（可选 yaml 日志，**不是**零点本体） |

日志文件（例如 `auto_set_zero_demo/zero_run_log.yaml`）只做追溯，真正零点在电机 Flash。

### 3.8 Demo 操作

1. 周围无障碍，手臂能顶到挡块；机体固定防倾倒。
2. 进入开发态，关掉头追 / 拖拽 / IK。
3. `--dry-run`：打印方向、电机 id、拟用 `q_travel`，不出力、不写零。
4. `--no-write`：可实机顶限位并走到 home，但不发 `MODE_RESET_ZERO`。
5. 单关节 `--joints r_elbow_joint` 成功后，再跑该侧 4 轴，最后左右臂。
6. 写零成功后断电再上电，确认姿态仍对齐（Flash 已保存）。

---

## 4. 风险点

### 4.1 写零语义（本方案特有）

| 风险 | 原因 | 后果 | 缓解 |
| --- | --- | --- | --- |
| 在限位处直接写零 | 误把挡块当电机 0 | 与 `urdf_offset` 叠加，整臂系统性偏 | **必须先 MOVE_HOME**；禁止 STALL 态写零 |
| 写零后位置环未停 | 反馈突变为 0 | 关节猛甩 | 先停指令再 RESET_ZERO |
| 全机 `set_reset_zero()` | 其它轴当前姿态被当成 0 | 腿/头零点被破坏 | 只对当前 `motor id` |
| 与官方校准抢写 | FSM=10 同时写 Flash | 不确定哪次生效 | 调零前确认不在校准态 |
| `CONF_WRITE` 未成功就断电 | 只改了 RAM 零点 | 重启回到旧零 | 必须等到 saved successfully |
| 带载写零 | 1 N·m 把结构压出弹性角 | 卸力后零点漂 | BACKOFF + SETTLE 后再写 |
| `q_travel` 用了 URDF limit | 软件限位 ≠ 硬挡块 | 走到错误 home | 用已校准机实测 `q_travel` |

### 4.2 安全与结构

| 风险 | 缓解 |
| --- | --- |
| 1 N·m 对小关节过大 / 对肩抗重力偏小 | 分关节阈值；肩用 `τ−τ_bias` |
| 顶到桌子/线束/人体 | 清空空间；`min_travel` + home 合理性检查 |
| 裸力矩失控 | 优先限力矩位置斜坡；行程/时间硬超时；`τ_abort` |
| 串联臂干涉 | 远端先调；安全折叠 |
| 持续堵转发热 | hold 短；FAIL 立即 STOP |

### 4.3 检测与精度

| 风险 | 缓解 |
| --- | --- |
| 力矩噪声 / 单位不是关节 N·m | 滤波 + 连续时间；与 `MotorState.tau` 交叉验证 |
| `seek_dir` 与 τ 符号相反 | 每轴 `tau_sign`；SEEK 前几度检查 τ 与运动同号 |
| 挡块弹性、回差 | 刚过阈值采样；单侧限位只保证该侧重复性 |
| 写零后 `direction` 使读数符号翻转 | VERIFY 以运动学/目视为准，不要只看原始编码器 |

### 4.4 系统

| 风险 | 缓解 |
| --- | --- |
| EXEC_DEFAULT 下发不了 `MODE_TORQUE` / 写零 | 开发态或确认 master 是否转发 SDK 指令 |
| 其它节点抢 `/pi_plus_absolute` | 启动前停进程 |
| 错误零点写入不可逆 | `--no-write` 预演；保留出厂校准记录；提供按关节重跑 |
| 无 τ 数据仍写零 | 无有效力矩流禁止进入 RESET_ZERO |

### 4.5 固有局限

- 单侧限位消不掉减速器回差。
- 挡块磨损或改装配后必须重测 `q_travel`。
- 不能替代绝对编码器；本方案只是把官方「当前姿态写零」自动化。
- 1 N·m 不是所有关节最优，按轴配置。

---

## 5. 验收

在人为打乱单关节零点（或只对该电机重写错误零）后，自动流程 5 次：

- 写零后该电机 `|q_fb| < 2°`，上电后仍成立。
- 走到站立/默认臂姿，与调零前出厂姿态目视一致（`urdf_offset` 未改）。
- 峰值力矩 `< τ_abort`，无结构异响。
- 中途放障碍必须 `FAIL` 且 Flash 未被更新。
- 未选中的电机零点不变。

---

## 6. 与现有代码的关系

| 模块 | 用途 |
| --- | --- |
| `liblivelybot_serial`：`MODE_TORQUE` / `MODE_RESET_ZERO` / `MODE_CONF_WRITE` | 本方案直接调用的能力 |
| `livelybot_bringup/motor_set_zero` | 官方「当前姿态 → 全机或电机写零」，本 demo 应做成**单关节、先寻限位再走到 home** 的包装 |
| `pd.yaml` `urdf_offset` | **只读**；自动调零对齐的是电机 0，不是改这份文件 |
| `robot_param.yaml` | 关节名 ↔ CAN id，用来填 `motor_index`（臂在 CANport_3/4，各 4 电机） |
| `5_DOF_ARM_IK` | **不要当本机关节表**（含腕，机型是 10A 带夹爪）；本机无腕 |
| `pull_move_demo` 力矩订阅 | 堵转检测 |
| BLE `M_resetzero` | 同一套电机 Flash；互斥 |

---

## 7. 实现清单

1. `config.yaml`：左右各 4 关节（pitch/roll/upper_arm/elbow），电机 id、`seek_dir`、实测 `q_travel`、τ 阈值、顺序。
2. 在已出厂校准的机上跑「只寻限位不写零」，把各轴 `q_enc_limit` 写入 `q_travel`。
3. 限力矩 SEEK → SAMPLE → BACKOFF → MOVE_HOME → 停环 → 单电机 RESET_ZERO → CONF_WRITE → 校验跳 0。
4. `--dry-run` / `--no-write` / `--joints`。
5. FAIL 路径保证不写 Flash；成功后断电复测。
