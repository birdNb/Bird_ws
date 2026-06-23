# Bird 语音传输 Demo（sound_demo）

小程序经 BLE 发送语音 PCM，板端实时从机器人扬声器播放；与现有 `BT_test` 蓝牙遥控**并行兼容**。

---

## 一、通道设计

| 通道 | UUID | 用途 |
|------|------|------|
| FFE0 | 服务 | 不变 |
| **FFE1** | write | 摇杆/模式/**sound ON/OFF** + **0x0B 音频包** |
| FFE2 | notify | ACK / 遥测 / `sound ON/OFF` 回显 |
| FFE3 | write（可选） | `--enable-voice` 时备用音频通道 |

- 摇杆文本包以 `X:` 开头；音频包以 **`0x0B`** 开头，在 `ble_gatt_server` 层分流，**不走** 64 字节文本解析。
- `sound ON` 后终端 **stderr** 显示实时电平条动画。

---

## 二、音频格式（与小程序一致）

| 项 | 值 |
|----|-----|
| 编码 | PCM signed 16-bit little-endian |
| 声道 | 1（mono） |
| 采样率 | 8000 Hz |
| 每包 PCM | 约 **180 字节** |

### FFE1 音频包格式

```text
[0x0B][seq_hi][seq_lo][pcm...]
  seq = (seq_hi << 8) | seq_lo
  pcm = 后续全部字节
```

示例：183 字节包 = 3 字节头 + 180 字节 PCM。

---

## 三、控制流程

```text
1. FFE1 write "sound ON"     → FFE2 回显 sound ON，启动播放+电平条
2. FFE1 write 0x0B 音频包    → 实时扬声器播放（writeNoResponse）
3. FFE1 write "sound OFF"    → FFE2 回显 sound OFF，停止
```

摇杆 `X:0.00,Y:0.00,Z:0.00` 与音频包可交替发送。

---

## 四、终端电平条（sound ON 时）

stderr 单行刷新，不刷屏：

```text
[sound] |████████░░░░░░░░| ▇  42.3% seq=  128 帧=  128  23040B
```

---

## 五、目录结构

```text
sound_demo/
  protocol.py        # 0x0B 帧解析
  audio_player.py    # pacat 播放
  audio_session.py   # 会话
  audio_meter.py     # 电平条
  integrate.py       # BLE 集成
  test_local.py      # 本地测试
```

---

## 六、使用

```bash
# 本地验证（电平条 + 扬声器）
cd ~/Bird_ws && python3 sound_demo/test_local.py

# 与 BLE 遥控一起（语音默认已启用，无需 --enable-voice）
cd ~/Bird_ws/BT_test && ./start.sh
```

---

## 七、排查无声音

| 现象 | 原因 |
|------|------|
| 日志 `收到音频但会话未开启` | 未先发 `sound ON` |
| 电平条不动 | 未收到 0x0B 包，或 PCM 全零 |
| `pacat` 失败 | `sudo apt install pulseaudio-utils` |
| 无回显 `sound ON` | 未订阅 FFE2 或 echo 被 `--no-echo` 关闭 |
