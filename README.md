# BionicFace Panel

用于仿生人脸前期硬件标定与数据采集的桌面控制台。

当前架构不是"AI 直接控制表情"，而是一个面向实验阶段的 32 通道可视化控制系统。控制逻辑全部集中在上位机，树莓派只做执行：

```text
React 滑条 --invoke--> Rust ControlService --UDP JSON--> Raspberry Pi --I2C--> PCA9685
```

技术栈：Tauri 2.11 + React 18 + Rust（上位机），Python + adafruit-servokit（树莓派）。

## 项目目标

帮助团队先把这些事情做稳定：

- 电机映射与硬件通道校准
- 限位与零位补偿验证
- 面部结构联动调试（下巴联动等）
- 常用表情的快速复现与数据采集时的重复控制、时间记录

复杂的 Blendshape 映射和 AI 驱动不在当前主链路里。

## 分层职责

### 前端（[src/App.tsx](src/App.tsx)）

- 显示 32 个通道滑条，展示每个通道的逻辑值与当前应用值
- 显示 UDP 目标地址、禁用通道和最后一帧数据
- 提供表情预设按钮，一键把 32 通道同时设置到预设角度
- 滑条变化通过 Tauri `invoke` 发给 Rust，本地即时回显，IPC 调用合并节流到约 30Hz，滑条行做了 memo 化以减少无关重渲染

前端只发"目标逻辑值"（或预设 id），不做任何安全计算。

### 上位机 Rust 后端（[src-tauri/src/control.rs](src-tauri/src/control.rs)）

实际的控制大脑：

- 启动时读取导出的硬件配置 JSON（通道标定、下巴联动配置、表情预设）
- 对每个通道做 `clamp + offset` 补偿
- 以 100Hz 对目标角度做插值（每 tick 最大步进 2°，即 200°/s 转速限制）；所有通道到位后自动降到 10Hz 保活帧，避免空转时打满总线和网络
- 应用下巴联动：motor 25（`jaw_right_upper`）作为主升轴，25 偏离中位的量按比例传导给 26、27，方向可各自配置（详见下方"下巴联动"）
- 应用表情预设：一次性把 32 个通道设置为预设定义的应用角度，覆盖联动/滑条当前值
- 通过 UDP JSON 把 32 通道最终角度发往树莓派（心跳与手动 flush 共用一个 socket）
- 记录发送日志（JSONL + CSV 双份），只记录运动帧，停稳或每秒批量刷盘一次，减少 SD 卡写入

### 树莓派执行器（[raspi/servo_server.py](raspi/servo_server.py)）

哑执行器，只做三件事：

- 监听 UDP 端口，积压时只取最新一帧（旧帧是过期目标，直接丢弃）
- 解析 `angles` 数组
- 把角度写入 PCA9685，未变化的通道跳过 I2C 写入（每次 `.angle` 赋值是多次 I2C 寄存器写入，高帧率下这个跳过很关键）

限位计算、offset 补偿、插值、表情映射、联动都不在树莓派上。

## 通信协议

线上数据帧（UDP JSON）：

```json
{
  "frameId": 123,
  "timestampNs": 1742600000000000000,
  "source": "udp-heartbeat",
  "angles": [32 个最终角度]
}
```

- `frameId`：帧编号，单调递增
- `timestampNs`：高精度时间戳
- `source`：`udp-heartbeat`（运动中）、`udp-keepalive`（静止保活）或 `manual-flush`（手动触发）
- `angles`：最终发往 PCA9685 的 32 通道物理角度

发送节奏：

- 有通道在向目标插值时按 100Hz 全速发送
- 所有通道到位后降为 10Hz 保活帧
- RFC3339 时间戳只保留在本地日志（JSONL/CSV）里，不进 UDP 协议

日志行为：只记录运动帧（保活帧不落盘），停稳时或每秒批量刷盘一次。日志位于运行目录下的 `./logs/`（`udp_frames.jsonl` / `udp_frames.csv`）。

## 配置机制

唯一真实标定源是 [raspi/config.py](raspi/config.py)，定义：

- `BOARD_ADDRESSES`、`UDP_PORT`
- `MOTOR_NAMES`、`MOTOR_MAP`
- `MOTOR_LIMITS`、`MOTOR_OFFSET`
- `MOTOR_INITIAL_APPLIED`（启动/复位时的显式中位角度）
- `DISABLED_MOTORS`
- `JAW_COUPLING`（下巴联动参数）

表情预设数据来自 [emotion.md](emotion.md)，导出脚本会把里面每个表情的 32 通道应用角度打包进配置 JSON。

Rust 不直接解析 Python，修改配置或 `emotion.md` 后需要重新导出：

```bash
python3 raspi/export_config_json.py
```

导出结果写入 [src-tauri/config/motor_config.json](src-tauri/config/motor_config.json)，Rust 启动时读取。**改完 `config.py` 或 `emotion.md` 忘记导出是最常见的配置不生效原因。**

## 逻辑角度与物理角度

- 逻辑值：前端滑条输入的目标值
- 应用值：Rust 做过补偿后真正下发的物理角度

```text
applied = clamp(logical + offset, minApplied, maxApplied)
```

前端只负责调目标，Rust 负责把目标变成安全可执行值，树莓派只负责执行。

## 下巴联动

下巴通道有主从联动：当前配置 motor 26（`jaw_right_lower`）为主动，27（`jaw_left`）跟随。

- 从动相对自身中位的补偿量 = 主动相对中位的偏移量 × `ratio`
- 每个从动有独立的 `direction`（+1 跟随，-1 相反）；当前 27 为 `-1`（镜像）
- **单向联动**：只有拖动主动通道才驱动从动；直接拖从动通道时它单独运动、不牵连主动，方便对不准时单独微调从动
- 应用表情预设时不触发联动重算——预设的 32 个角度本身就是"已经算好联动"的最终值

配置项在 [raspi/config.py](raspi/config.py) 的 `JAW_COUPLING` 里：`master_motor_id`、`slave_motor_ids`、`ratio`、`directions`。

## 表情预设系统

常用表情（喜悦、悲伤、愤怒、惊讶、恐惧、厌恶等）不需要每次手动摆 32 个滑条，预设系统把这些角度固化下来：

- 预设数据源是 [emotion.md](emotion.md)，每个表情一段 32 个应用角度
- 导出脚本把 `emotion.md` 解析进 `motor_config.json` 的 `expressionPresets` 字段
- Rust 端提供 `list_expression_presets`（返回 id/label 列表）和 `apply_expression_preset`（按 id 应用，内部会做和滑条一致的 `clamp` 校验）两个 Tauri 命令
- 前端在顶部渲染一排预设按钮，点击即把全部 32 通道设置为该表情的目标角度，并高亮当前生效的预设
- 手动拖动任意滑条会清除"当前预设"高亮，因为已经偏离了预设值

## 通道禁用与脖子电机

协议固定为 32 通道。30、31 两个脖子电机已随脖子结构恢复而重新启用，当前全部 32 个通道均参与标定。**但 30、31 尚未完成正式的运动范围标定**，`MOTOR_LIMITS` 暂时保守收紧到 `(75, 105)`（中位 90° 上下各 15°），后续完成标定后再放宽。

如需临时下线某些电机，把编号加入 `config.py` 的 `DISABLED_MOTORS` 并重新导出配置即可：

- 通道仍保留在 32 通道协议里，不破坏数据结构和通道索引
- 被禁用的通道在 UI 中锁定，并持续保持安全中位

## 工作流

1. 修改 [raspi/config.py](raspi/config.py)（标定/联动/禁用）或 [emotion.md](emotion.md)（表情预设）
2. 运行 `python3 raspi/export_config_json.py`
3. 树莓派上启动执行器：`python3 raspi/servo_server.py`
4. 上位机启动面板：`npm run tauri dev`
5. 在 UI 中连接 `<pi-ip>:6000`
6. 开始单通道校准，或点击预设按钮批量核对表情

环境配置与部署细节见 [docs/setup_guide_zh.md](docs/setup_guide_zh.md)。

## 关键文件

| 文件 | 作用 |
|---|---|
| [raspi/config.py](raspi/config.py) | 硬件标定源（唯一真实来源，含限位、联动、禁用配置） |
| [emotion.md](emotion.md) | 表情预设数据源 |
| [raspi/export_config_json.py](raspi/export_config_json.py) | 配置导出脚本（Python + emotion.md → Rust JSON） |
| [raspi/servo_server.py](raspi/servo_server.py) | 树莓派 UDP 执行器 |
| [src-tauri/src/control.rs](src-tauri/src/control.rs) | Rust 控制核心（补偿、插值、联动、预设、心跳、日志） |
| [src/App.tsx](src/App.tsx) | 前端控制台 UI |
| [docs/setup_guide_zh.md](docs/setup_guide_zh.md) | 环境配置与部署文档 |

## 当前边界

适合：标定、结构联调、表情复现、数据采集控制，以及后续接入更复杂控制算法前的底座。

不负责：自动表情求解、视觉驱动表情映射、ROS2 联动。这些在控制台稳定之后作为下一层叠加。
