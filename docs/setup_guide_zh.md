# BionicFace Panel 配置与运行指南
接下来你们实际使用时，建议固定成这条顺序：

修改 config.py
运行 python raspi/export_config_json.py
启动树莓派 servo_server.py
启动 npm run tauri dev
## 0. 当前版本说明

当前代码已切换到“可视化硬件标定与数据采集控制台”架构：

- 前端：
  - 32 个滑条直控
- 上位机 Rust：
  - 读取由 `raspi/config.py` 导出的 JSON 配置
  - 执行 `clamp + offset`
  - 以 100Hz 插值并通过 UDP JSON 下发
- 树莓派：
  - 作为极简 UDP 执行器
  - 不再负责限位与插值

当前配置同步入口：

```bash
python3 raspi/export_config_json.py
```

该命令会将真实配置导出到：

- `src-tauri/config/motor_config.json`

当前树莓派默认监听：

- `0.0.0.0:6000`

本文档面向项目使用者与开发者，说明如何在 macOS、Linux、Windows 上配置上位机环境，以及如何在树莓派上配置下位机控制服务。

## 1. 系统架构

- 上位机：
  - Tauri 桌面应用
  - Rust 负责控制逻辑、时间戳、日志、UDP 通信
  - React + TypeScript 负责 UI
- 下位机：
  - Raspberry Pi
  - Python 驱动 PCA9685 和舵机
- 通信方式：
  - 局域网内 UDP JSON
  - 默认使用 `<树莓派IP>:6000`

推荐网络拓扑：

- 上位机和树莓派连接到同一个局域网
- 上位机通过固定 IP 或 DHCP 保留地址访问树莓派
- 不建议把实时控制链路放到公网或 Cloudflare Tunnel 上

## 2. 仓库结构

- `src-tauri/`
  - Tauri + Rust 后端
- `src/`
  - React 前端
- `raspi/`
  - Raspberry Pi 端示例服务脚本
- `docs/setup_guide_zh.md`
  - 当前文档

注意：

- 当前项目以 `raspi/config.py` 为真实标定源
- `python3 raspi/export_config_json.py` 会把其导出为 `src-tauri/config/motor_config.json`
- 当前上位机负责 `clamp + offset + interpolation`
- 当前树莓派只负责执行最终角度数组

## 3. 上位机通用前置要求

无论你使用哪种桌面系统，都需要：

- Node.js LTS
- npm
- Rust 工具链
- 系统自带或可安装的 C/C++ 编译环境
- WebView 运行环境

建议版本：

- Node.js 20.x 或更高 LTS
- npm 10.x 或更高
- Rust stable 最新版

## 4. macOS 配置说明

### 4.1 安装系统依赖

安装 Xcode Command Line Tools：

```bash
xcode-select --install
```

安装 Rust：

```bash
curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh
```

确认版本：

```bash
rustc --version
cargo --version
node -v
npm -v
```

### 4.2 安装项目依赖

进入项目目录：

```bash
cd /Users/seanja/Developer/BionicFace-Panel
```

安装前端依赖：

```bash
npm install
```

### 4.3 运行开发版

启动 Tauri 开发模式：

```bash
npm run tauri dev
```

说明：

- 该命令会先启动前端开发服务器 `http://localhost:1420`
- 再打开一个本地桌面窗口
- 这是最适合开发和联调的方式

### 4.4 构建 macOS 应用

```bash
npm run tauri build
```

构建产物通常位于：

- `src-tauri/target/release/bundle/`

## 5. Linux 配置说明

以下以 Ubuntu/Debian 系为例。

### 5.1 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  curl \
  wget \
  file \
  libxdo-dev \
  libssl-dev \
  libayatana-appindicator3-dev \
  librsvg2-dev \
  patchelf \
  pkg-config \
  libgtk-3-dev \
  libwebkit2gtk-4.1-dev
```

安装 Rust：

```bash
curl --proto '=https' --tlsv1.2 https://sh.rustup.rs -sSf | sh
source "$HOME/.cargo/env"
```

确认版本：

```bash
rustc --version
cargo --version
node -v
npm -v
```

### 5.2 安装项目依赖并运行

```bash
cd /path/to/BionicFace-Panel
npm install
npm run tauri dev
```

### 5.3 构建 Linux 桌面应用

```bash
npm run tauri build
```

输出通常包括：

- `.AppImage`
- `.deb`
- 视 Tauri 配置和系统环境而定的其他包格式

## 6. Windows 配置说明

### 6.1 安装系统依赖

需要安装：

- Microsoft Visual Studio 2022 Build Tools
  - 勾选“使用 C++ 的桌面开发”
- Microsoft WebView2 Runtime
- Node.js LTS
- Rust 工具链

安装 Rust：

```powershell
winget install Rustlang.Rustup
```

安装 Node.js：

```powershell
winget install OpenJS.NodeJS.LTS
```

确认版本：

```powershell
rustc --version
cargo --version
node -v
npm -v
```

### 6.2 安装项目依赖并运行

```powershell
cd C:\path\to\BionicFace-Panel
npm install
npm run tauri dev
```

### 6.3 构建 Windows 应用

```powershell
npm run tauri build
```

输出通常位于：

- `src-tauri\target\release\bundle\`

常见格式：

- `.msi`
- `.exe`

## 7. 仅使用 localhost 的说明

当前项目支持两种运行方式的理解：

### 7.1 Tauri 桌面模式

- 通过 `npm run tauri dev` 或构建后的安装包运行
- 前端界面运行在本地桌面容器中
- Rust 后端命令可正常调用
  - 可以执行：
  - UDP 下发
  - 日志写入
  - 单电机控制
  - 32 通道标定

### 7.2 纯浏览器 localhost 模式

只运行：

```bash
npm run dev
```

然后在浏览器访问：

- `http://localhost:1420`

此时：

- UI 可以显示
- 但 Tauri 命令不可用
- 即不能完整控制树莓派

所以：

- `localhost` 可用于纯前端预览
- 不可作为当前架构下的完整交付形式

如果后续要做“无安装版”的浏览器使用方式，需要额外开发一个独立本地服务或远程服务层，而不是直接复用当前 Tauri 命令接口。

## 8. 树莓派配置说明

推荐硬件：

- Raspberry Pi 4B 或更高
- Raspberry Pi OS 64-bit
- 稳定 5V 电源
- 舵机独立供电
- I2C 已启用

## 9. 树莓派系统初始化

### 9.1 刷写系统

推荐使用 Raspberry Pi Imager 写入：

- Raspberry Pi OS Lite 或 Desktop 都可以

建议同时设置：

- 主机名，例如 `bionic-pi`
- SSH 开启
- 用户名和密码
- Wi-Fi 或有线网络

### 9.2 首次更新系统

```bash
sudo apt update
sudo apt upgrade -y
```

### 9.3 启用 I2C

```bash
sudo raspi-config
```

进入：

- `Interface Options`
- `I2C`
- 选择启用

然后重启：

```bash
sudo reboot
```

### 9.4 安装 I2C 工具

```bash
sudo apt install -y i2c-tools python3-pip python3-venv
```

检查 I2C 设备：

```bash
sudo i2cdetect -y 1
```

如果两块 PCA9685 地址分别是 `0x40` 和 `0x41`，应当能在扫描结果里看到。

## 10. 树莓派 Python 环境配置

### 10.1 创建虚拟环境

```bash
mkdir -p ~/bionicFace
cd ~/bionicFace
python3 -m venv .venv
source .venv/bin/activate
```

### 10.2 安装 Python 依赖

```bash
pip install --upgrade pip
pip install adafruit-circuitpython-servokit
```

如果你还要直接使用底层 GPIO/I2C 调试工具，也可以补充：

```bash
pip install adafruit-blinka
```

## 11. 树莓派配置文件说明

本项目树莓派执行器默认读取：

- `raspi/config.py`

最少需要定义：

```python
MOTOR_MAP = {
    0: (0, 0),
    1: (0, 1),
    2: (0, 2),
    ...
    31: (2, 5),
}
```

含义：

- key：电机编号
- value 第 1 项：板卡索引
- value 第 2 项：PCA9685 channel

当前上位机配置导出依赖的典型字段：

```python
BOARD_ADDRESSES = [0x40, 0x41]
UDP_PORT = 6000

MOTOR_LIMITS = {
    0: (170, 210),
    1: (10, 40),
}

MOTOR_OFFSET = {
    0: 3,
    1: 3,
}
```

说明：

- `BOARD_ADDRESSES`：PCA9685 地址列表
- `UDP_PORT`：树莓派监听端口
- `MOTOR_LIMITS`：物理最终角度范围
- `MOTOR_OFFSET`：零位补偿

## 12. 树莓派控制服务启动

假设你已将项目目录同步到树莓派：

```bash
cd ~/BionicFace-Panel
source ~/bionicFace/.venv/bin/activate
python3 raspi/servo_server.py
```

默认情况下：

- 服务会读取 `raspi/config.py`
- 初始化 PCA9685
- 启动 UDP 执行器

如果你想显式指定配置文件路径，可以设置环境变量：

```bash
export BIONIC_FACE_CONFIG=/home/pi/BionicFace-Panel/raspi/config.py
python3 raspi/servo_server.py
```

## 13. 将树莓派控制脚本注册为 systemd 服务

创建服务文件：

```bash
sudo nano /etc/systemd/system/bionic-face-servo.service
```

写入以下内容：

```ini
[Unit]
Description=BionicFace Servo Server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/BionicFace-Panel
Environment=BIONIC_FACE_CONFIG=/home/pi/BionicFace-Panel/raspi/config.py
ExecStart=/home/pi/bionicFace/.venv/bin/python3 /home/pi/BionicFace-Panel/raspi/servo_server.py
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

启用服务：

```bash
sudo systemctl daemon-reload
sudo systemctl enable bionic-face-servo.service
sudo systemctl start bionic-face-servo.service
```

查看状态：

```bash
sudo systemctl status bionic-face-servo.service
```

查看日志：

```bash
journalctl -u bionic-face-servo.service -f
```

## 14. 上位机与树莓派联调流程

### 14.1 获取树莓派 IP

在树莓派执行：

```bash
hostname -I
```

例如得到：

```text
192.168.1.50
```

### 14.2 启动树莓派服务

确认树莓派服务已运行，监听：

```text
0.0.0.0:6000
```

### 14.3 启动桌面端

在 macOS / Linux / Windows 上执行：

```bash
npm run tauri dev
```

然后在 UI 中将 Endpoint 设置为：

```text
192.168.1.50:6000
```

点击：

- `Connect`
- `Center All`
- `Flush`

若连接成功，即可开始：

- 调整 32 个电机滑条
- 查看上位机本地日志

## 15. 常见问题排查

### 15.1 Tauri 构建时报 icon 错误

表现：

- `failed to open icon ... src-tauri/icons/icon.png`

解决：

- 确保存在合法 PNG 文件：
  - `src-tauri/icons/icon.png`

### 15.2 浏览器里页面打开了，但按钮没反应

原因：

- 只运行了 `npm run dev`
- 浏览器环境没有 Tauri runtime

解决：

- 使用 `npm run tauri dev`

### 15.3 上位机连接不上树莓派

检查：

- 两台设备是否在同一局域网
- 树莓派服务是否启动
- IP 地址是否正确
- 防火墙是否拦截 5555 端口

### 15.4 树莓派检测不到 PCA9685

检查：

- I2C 是否启用
- 电源是否正常
- 接线是否正确
- 地址拨码是否与配置一致

使用：

```bash
sudo i2cdetect -y 1
```

### 15.5 舵机动作异常或反向

优先检查：

- `zero_offset`
- `min_angle`
- `max_angle`
- `actuation_range`

建议一次只校准一个电机。

## 16. 推荐使用方式

开发阶段推荐：

- 上位机使用 `npm run tauri dev`
- 树莓派直接运行 Python 脚本或 systemd 服务

实验室部署推荐：

- 上位机构建成安装包运行
- 树莓派使用 systemd 自启动
- 局域网固定地址通信

如果你们后续要做更正式的交付，可以在此文档基础上继续拆分为：

- 开发者环境安装文档
- 实验室部署文档
- 树莓派硬件接线与校准文档
- 操作员使用手册
