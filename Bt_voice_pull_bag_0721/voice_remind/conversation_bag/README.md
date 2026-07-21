# conversation_bag 对话语音包

上传实际录音后，小程序用 **录音文案前 5 个汉字的拼音首字母（大写）** 触发播放。

## 调用规则

1. 录音完整文案示例：`蓝牙就绪待连接`
2. 取前 5 字：`蓝牙就绪待`
3. 拼音首字母大写 → **`LYJXD`**
4. 小程序 FFE1 发送：`LYJXD`
5. 板端播放：`LYJXD.wav`
6. 回执：`ACK:LYJXD`

不足 5 字时取全部汉字，例如 `行走模式`（4 字）→ **`XZMS`**

## 上传步骤

1. 准备 WAV，文件名 = code + `.wav`，如 `LYJXD.wav`
2. 放入本目录
3. （推荐）在 `manifest.json` 增加条目：

```json
"LYJXD": {
  "text": "蓝牙就绪待连接",
  "file": "LYJXD.wav"
}
```

4. 修改 `manifest.json` 后自动重载，无需重启

## 生成 code

```bash
pip3 install pypinyin
python3 ../make_conv_code.py "蓝牙就绪待连接"
# → LYJXD
```

## code 格式

- 大写字母开头，仅 `A-Z` 和 `0-9`，长度 2–32
- WAV 文件名与 code 一致（大写）
