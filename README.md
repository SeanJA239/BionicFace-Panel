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

表情预设数据来自 [presets.json](presets.json)，导出脚本把里面每个表情的 32 通道归一化系数原样打包进配置 JSON（见下方"表情预设系统"）。

Rust 不直接解析 Python，修改配置或 `presets.json` 后需要重新导出：

```bash
python3 raspi/export_config_json.py
```

导出结果写入 [src-tauri/config/motor_config.json](src-tauri/config/motor_config.json)，Rust 启动时读取。**改完 `config.py` 或 `presets.json` 忘记导出是最常见的配置不生效原因。**

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

- 预设的规范存储形式是 **[presets.json](presets.json)**：每个表情是 `{id, label, norm}`，`norm` 是 32 个**双极归一化系数（-1..1）**，语义与 `control.rs` 里 `MotorChannel::norm_to_applied`/`applied_to_norm` 完全一致——以该通道**校准后的中位（neutral_applied）**为锚点，`0` 是中位，`+1`/`-1` 分别是该通道限位的上/下端点。这样一个预设不管标定端点之后怎么改，换算出来的姿态相对位置不变。
- 历史数据 [emotion.legacy.md](emotion.legacy.md) 保留作参考，里面是旧的 32 通道原始物理角度（`applied`），**不再参与导出**。一次性迁移脚本 [raspi/migrate_presets_to_normalized.py](raspi/migrate_presets_to_normalized.py) 把它按当前 `config.py` 标定换算成 `presets.json`；超出 `MOTOR_LIMITS` 的角度会被钳位并打印警告。`DISABLED_MOTORS` 对应位置写 `0.0`（该通道自身的归一化中位），应用时会被 `apply_motor_target_norm` 的 `!enabled` 分支忽略。
- 导出脚本 [raspi/export_config_json.py](raspi/export_config_json.py) 直接读取 `presets.json` 并原样（钳位到 `[-1, 1]`）打包进 `motor_config.json` 的 `expressionPresets` 字段——因为系数已经是标定无关的，这一步不需要再查 `MOTOR_LIMITS`/`MOTOR_OFFSET`。
- Rust 端提供三个 Tauri 命令：`list_expression_presets`（返回 id/label 列表）、`apply_expression_preset`（按 id 应用，内部经 `apply_motor_target_norm` 做和滑条一致的换算与钳位）、`apply_expression_preset_scaled(presetId, intensity)`（按强度缩放应用，见下）。
- 前端在顶部渲染一个强度滑杆（默认 1.0）和一排预设按钮，点击按钮把全部 32 通道设置为"按当前强度缩放"后的该表情目标，并高亮当前生效的预设。手动拖动任意滑条会清除"当前预设"高亮，因为已经偏离了预设值。

**强度缩放语义**：`apply_expression_preset_scaled` 不是把系数往每个通道的校准中位（`norm=0`）缩放，而是往 **`rest` 预设自身的系数向量**缩放（若没有 `rest` 预设则退化为往 `0` 缩放）：

```text
c' = c_rest + intensity * (c - c_rest)   // 按通道
```

原因：`rest`（静息）预设的 32 个系数本身大多不是 `0`——静息姿态和每个电机独立标定出来的物理中位并不是一回事（例如 `eyebrow_right_inner` 的 `rest` 系数是 `-0.25` 而不是 `0`）。如果把强度缩放锚定在 `norm=0`，`intensity=0` 会让脸回到一堆互不相关的电机中位，而不是回到静息表情；锚定在 `rest` 的系数向量上，`intensity=0` 才真正回到静息脸，`intensity=1` 是完整表情，中间是两者的线性插值。

> **与本任务文档原始描述的差异**：文档最初设想的是单极 `[0, 1]` 系数 `coefficient = (applied - min_applied) / (max_applied - min_applied)`。仓库在此之前的"norm"系列提交（见 `git log`）已经实现并全面接入了一套双极 `[-1, 1]`、以校准中位为锚点、两侧跨度可不对称的归一化方案，贯穿滑条、下巴联动、点头动作。按仓库现状优先的约定，本次迁移复用了这套已有方案而不是另起一套不兼容的单极系数，`DISABLED_MOTORS` 的占位值相应从文档里的 `0.5` 改为 `0.0`（该方案里的"中位"就是 `0`）。

## 通道禁用与脖子电机

协议固定为 32 通道。30、31 两个脖子电机已随脖子结构恢复而重新启用，当前全部 32 个通道均参与标定。**但 30、31 尚未完成正式的运动范围标定**，`MOTOR_LIMITS` 暂时保守收紧到 `(75, 105)`（中位 90° 上下各 15°），后续完成标定后再放宽。

如需临时下线某些电机，把编号加入 `config.py` 的 `DISABLED_MOTORS` 并重新导出配置即可：

- 通道仍保留在 32 通道协议里，不破坏数据结构和通道索引
- 被禁用的通道在 UI 中锁定，并持续保持安全中位

## 工作流

1. 修改 [raspi/config.py](raspi/config.py)（标定/联动/禁用）或 [presets.json](presets.json)（表情预设，32 个 `-1..1` 归一化系数）
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
| [presets.json](presets.json) | 表情预设数据源（32 通道归一化系数，标定无关） |
| [emotion.legacy.md](emotion.legacy.md) | 迁移前的原始物理角度预设，仅供参考，不参与导出 |
| [raspi/migrate_presets_to_normalized.py](raspi/migrate_presets_to_normalized.py) | 一次性迁移脚本：`emotion.legacy.md` 角度 → `presets.json` 系数 |
| [raspi/export_config_json.py](raspi/export_config_json.py) | 配置导出脚本（Python + presets.json → Rust JSON） |
| [raspi/servo_server.py](raspi/servo_server.py) | 树莓派 UDP 执行器 |
| [src-tauri/src/control.rs](src-tauri/src/control.rs) | Rust 控制核心（补偿、插值、联动、预设、心跳、日志） |
| [src/App.tsx](src/App.tsx) | 前端控制台 UI |
| [docs/setup_guide_zh.md](docs/setup_guide_zh.md) | 环境配置与部署文档 |

## 当前边界

适合：标定、结构联调、表情复现、数据采集控制，以及后续接入更复杂控制算法前的底座。

不负责：自动表情求解、视觉驱动表情映射、ROS2 联动。这些在控制台稳定之后作为下一层叠加。
