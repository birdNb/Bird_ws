# Bt_voice_pull_bag（源码树）

蓝牙遥控 + 语音提醒 + 拖拽控制 + 头追 **四合一**开发目录。  
**本目录放源码**；带日期后缀的目录（如 `Bt_voice_pull_bag_0730`）才是加密发布包。

| 路径 | 用途 |
|------|------|
| `Bt_voice_pull_bag/` | 源码 / 联调（可含 `.py`） |
| `Bt_voice_pull_bag_XXXX/` | 发布包（仅 `.pyc` + 二进制，无业务源码） |

## 出加密包

```bash
bash scripts/build_release_0730.sh
# → ../Bt_voice_pull_bag_0730/
# → ../Bt_voice_pull_bag_0730.tar.gz
```

## 一键安装（请用日期包，不要直接装本源码树）

```bash
cd ../Bt_voice_pull_bag_0730
sudo ./install.sh
```

## 功能默认

| 功能 | 默认 | 开启 |
|------|------|------|
| 语音提示 `sound` | 开 | `sound OFF` 可关 |
| 人脸追踪 | 关 | `locate_face ON` |
| 拖拽 | 关 | `PULL ON` |

## 平台

- **RK3588**：板载蓝牙 + 头追优先 D435i `/dev/video4`
- **Jetson Orin**：USB 蓝牙 + 头追优先 ZED Mini `/dev/video0`（并排裁左眼）
