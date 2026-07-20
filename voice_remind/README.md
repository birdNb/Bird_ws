# voice_remind

机器人固定语音提示，本地 WAV 播放（paplay / aplay）。

## assets/ 系统提示音（英文文件名）

| 文件 | 文本 |
|------|------|
| `ble_ready.wav` | 蓝牙就绪，待连接 |
| `ble_connected.wav` | 蓝牙已连接 |
| `auto_stand.wav` | 自动站立 |
| `motor_on.wav` | 电机已上电 |
| `walk_mode.wav` | 行走模式 |
| `stand_mode.wav` | 站立模式 |
| `battery_50.wav` … `battery_5.wav` | 低电量提醒 |

电量语音在**下降穿越** 50 / 25 / 10 / 5% 时各播报一次。

```bash
sudo apt-get install -y espeak-ng
cd ~/Bird_ws/voice_remind
python3 generate_assets.py
```

可替换为真人录音，**保持英文文件名不变**。

## conversation_bag/ 对话语音（首字母大写命名）

小程序 FFE1 发送录音文案 **前 5 字拼音首字母大写**，如 `LYJXD` → 播放 `LYJXD.wav`。

```bash
pip3 install pypinyin
python3 make_conv_code.py "蓝牙就绪待连接"
```

详见 `conversation_bag/README.md`（目录已清空，待上传实际录音）。

## 关闭

```bash
export VOICE_REMIND=0
sudo systemctl restart bird-ble
```
