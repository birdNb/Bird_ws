# assets 系统提示音对照表

本目录 WAV 为板端固定播报。**文件名勿改**，只替换音频内容。  
可用 `Qwen3-TTS-Openai-Fastapi` 按「中文文案」重做（见该仓库 `README.md` 的 `/v1/audio/speech`）。

| 文件名 | 中文文案 | 对应指令 / 触发 |
|--------|----------|-----------------|
| `ble_ready.wav` | 蓝牙就绪，待连接 | BLE 服务就绪 |
| `ble_connected.wav` | 蓝牙已连接 | 小程序连上 |
| `ble_disconnected.wav` | 蓝牙已断开 | 小程序断开 |
| `auto_stand.wav` | 自动站立 | `LT+RT+start` |
| `motor_on.wav` | 电机已上电 | `MP ON` |
| `walk_mode.wav` | 行走模式 | `GAIT ON` |
| `stand_mode.wav` | 站立模式 | `GAIT OFF` |
| `pull_on.wav` | 拖拽模式已打开 | `PULL ON` |
| `pull_off.wav` | 拖拽模式已关闭 | `PULL OFF` |
| `sprint_on.wav` | 启动疾跑 | `LT ON` |
| `sprint_off.wav` | 关闭疾跑 | `LT OFF` |
| `mode_default.wav` | 默认模式 | `M_default` |
| `mode_init.wav` | 初始化模式 | `M_init` |
| `mode_protect.wav` | 保护模式 | `M_protect` |
| `mode_resetzero.wav` | 调零模式 | `M_resetzero` |
| `mode_tech.wav` | 示教模式 | `M_tech` |
| `squat.wav` | 蹲下 | `LT+RT+RB` |
| `locate_face.wav` | 人脸追踪 | `locate_face ON` |
| `sound_on.wav` | 打开语音提示 | `sound ON`（之后恢复系统提示音） |
| `sound_off.wav` | 关闭语音提示 | `sound OFF`（播完本条后不再播系统提示；conversation / 小程序音频仍可播） |
| `battery_50.wav` | 剩余电量百分之五十 | 电量下降穿越 50% |
| `battery_25.wav` | 剩余电量百分之二十五 | 电量下降穿越 25% |
| `battery_10.wav` | 剩余电量百分之十 | 电量下降穿越 10% |
| `battery_5.wav` | 剩余电量百分之五 | 电量下降穿越 5% |

## 用 Qwen3-TTS 重做示例

先启动 TTS（默认 `http://localhost:8880`）：

```bash
# 打开语音提示 → sound_on.wav
curl --fail --show-error \
  http://localhost:8880/v1/audio/speech \
  -H 'Content-Type: application/json' \
  -d '{
    "model": "tts-1-zh",
    "voice": "Vivian",
    "input": "打开语音提示",
    "response_format": "wav"
  }' \
  --output sound_on.wav
```

其余条目：`input` 换上表中文，`--output` 换对应文件名，覆盖本目录即可。

## 说明

- 文案与 `../prompts.py` 一致；缺某个 WAV 时仅跳过该条，不影响已有提示音
- 电量音在电量**下降穿越** 50 / 25 / 10 / 5% 时各播一次
- 格式建议：WAV（`paplay` / `aplay` 可播）
