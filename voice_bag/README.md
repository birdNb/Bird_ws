# voice_bag 导入暂存区

将 Windows `d:\HT_File\Bird_ws\voice_bag` 中的录音复制到本目录后，运行：

```bash
pip3 install pypinyin
python3 ~/Bird_ws/voice_remind/import_voice_bag.py
```

## 从 Windows 上传（PowerShell）

```powershell
scp -r D:\HT_File\Bird_ws\voice_bag hightorque@192.168.x.x:/home/hightorque/Bird_ws/
```

## 文件命名

- **推荐**：文件名即中文文案，如 `蓝牙就绪待连接.wav`
- 或 `manifest.json` / 同名 `.txt` 指定文案（见 `voice_remind/import_voice_bag.py` 说明）

导入后自动复制到 `voice_remind/conversation_bag/{CODE}.wav` 并更新 manifest。
