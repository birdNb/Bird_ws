# assets 系统提示音对照表

本目录 WAV 为板端固定播报（来源：`voice_bag/notice_bag_baby`）。**文件名勿改**，只替换音频内容。

| 文件名 | 中文文案 | 对应指令 / 触发 |
|--------|----------|-----------------|
| `ble_ready.wav` | 蓝牙就绪，待连接 | BLE 服务就绪 |
| `ble_connected.wav` | 蓝牙已连接 | 小程序连上 |
| `ble_disconnected.wav` | 蓝牙已断开 | 小程序断开 |
| `auto_stand.wav` | 自动站立 | `LT+RT+start` |
| `motor_on.wav` | 电机电源已开启 | `MP ON` |
| `motor_off.wav` | 电机电源已关闭 | `MP OFF` |
| `walk_mode.wav` | 行走模式 | `GAIT ON` |
| `stand_mode.wav` | 站立模式 | `GAIT OFF` |
| `pull_on.wav` | 拖拽模式已开启 | `PULL ON` |
| `pull_off.wav` | 拖拽模式已关闭 | `PULL OFF` |
| `sprint_on.wav` | 疾跑模式已开启 | `LT ON` |
| `sprint_off.wav` | 疾跑模式已关闭 | `LT OFF` |
| `mode_default.wav` | 默认模式 | `M_default` |
| `mode_init.wav` | 初始化模式 | `M_init` |
| `mode_protect.wav` | 保护模式 | `M_protect` |
| `mode_resetzero.wav` | 调零模式 | `M_resetzero` |
| `mode_tech.wav` | 示教模式 | `M_tech` |
| `squat.wav` | 蹲下 | `LT+RT+RB` |
| `face_on.wav` | 人脸跟随已开启 | `locate_face ON` |
| `face_off.wav` | 人脸跟随已关闭 | `locate_face OFF` |
| `sound_on.wav` | 提示音已开启 | `sound ON` |
| `sound_off.wav` | 提示音已关闭 | `sound OFF`（播完本条后不再播系统提示；conversation 仍可播） |
| `battery_50.wav` | 当前电量百分之五十 | 电量下降穿越 50% |
| `battery_25.wav` | 当前电量百分之二十五 | 电量下降穿越 25% |
| `battery_10.wav` | 当前电量百分之十 | 电量下降穿越 10% |
| `battery_5.wav` | 当前电量百分之五，请立即更换电池 | 电量下降穿越 5% |

## 说明

- 文案与 `../prompts.py` 一致；缺某个 WAV 时仅跳过该条
- 电量音在电量**下降穿越** 50 / 25 / 10 / 5% 时各播一次
- 格式：WAV PCM 16-bit mono 24 kHz
