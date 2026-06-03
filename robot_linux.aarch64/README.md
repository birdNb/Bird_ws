# 机器人端部署包

**目标平台**: linux.aarch64
**代码混淆**: 是

## 启动方式

### 首次使用：配置环境

```bash
# 配置Python 3.10 conda环境（首次使用必须执行）
./setup_env.sh
```

### 方式一：手动启动

```bash
# 前台启动
./start_robot.sh

# 后台启动
./start_robot.sh -d

# 停止后台服务
./stop_robot.sh
```

### 方式二：安装为系统服务（开机自启）

```bash
# 安装服务
./install_service.sh

# 服务管理
systemctl --user start robot-control    # 启动
systemctl --user stop robot-control     # 停止
systemctl --user restart robot-control  # 重启
systemctl --user status robot-control   # 状态
systemctl --user enable robot-control   # 启用自启动
systemctl --user disable robot-control  # 禁用自启动

# 查看日志
journalctl --user -u robot-control -f   # 实时日志
journalctl --user -u robot-control -n 100  # 最近100行

# 更新服务配置（修改配置文件后）
./update_service.sh

# 卸载服务
./uninstall_service.sh
```

## 配置文件

编辑 `config/robot.yaml` 修改配置：
- robot_id: 机器人ID
- robot_name: 机器人名称
- http_port: Web 服务端口（默认 45000）
- udp_receive_port: UDP 接收端口（默认 48888）
- default_mode: 默认模式（'self' 或 'group'）

## 访问界面

启动后访问: http://localhost:45000

## 注意事项

- 本部署包针对 linux.aarch64 平台编译
- 配置文件未混淆，可直接编辑
- 代码已混淆，请勿在其他平台使用
