# locate_face_cpp（运行时）

BLE `locate_face ON/OFF` 启停。适配：

- **Jetson Orin + ZED Mini**：自动选 `/dev/video0`，并排双目取左眼
- **RK3588 + D435i**：优先 `/dev/video4` 彩色流
- 覆盖：`export LOCATE_FACE_CAMERA=0`

```bash
./start.sh
./start.sh --gui
```
