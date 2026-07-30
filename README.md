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
- 监听独立的外部系数输入端口（默认 6100），和 Manual 来源仲裁出唯一当前控制源（详见"控制源仲裁与外部输入"）
- Manual 静止超过阈值后自动进入待机噪声/眨眼调度（详见"待机噪声与眨眼调度器"）

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

## 控制源仲裁与外部输入

除了前端滑条/预设/序列（统称 **Manual**），Rust 端还监听一个独立的 UDP 端口，接收外部进程（例如 [tools/mediapipe_driver.py](tools/mediapipe_driver.py)）实时推送的 32 通道系数流（**External**）。两者互斥，由 `control.rs` 里的控制源仲裁裁决：

- **协议**（默认端口 `6100`，另一个独立于树莓派 6000 端口的 socket）：

  ```json
  { "seq": 123, "timestampNs": 1742600000000000000, "coefficients": [32 个 0..1 浮点或 null] }
  ```

  - `coefficients[i] = null` 表示该帧不驱动通道 `i`（保持当前目标不变）。
  - 每个非空系数钳位到 `[0, 1]` 后，**线性映射整段限位**（`applied = minApplied + c * (maxApplied - minApplied)`），得到 applied 目标——这是单极映射，和表情预设用的双极 `norm` 空间是两套独立的、各自成立的表示：预设需要"以校准中位为锚点"的相对语义，外部输入流只需要一个简单的、和常见 ML 输出（比如 blendshape 0..1）直接对应的绝对映射。
  - `seq` 必须严格递增，回退或重复的帧会被丢弃（防止乱序帧覆盖更新的目标）。
  - 禁用通道（`DISABLED_MOTORS`）忽略外部系数。
  - 应用后仍会触发下巴联动重算，并走现有的限速插值/UDP 心跳路径——外部输入和滑条最终共用同一条"写目标数组"管线，不存在绕过 `ControlService` 的第二条链路。

- **仲裁规则**：任意一帧合法外部系数到达即把控制源切到 `External`；此时后端会拒绝所有 Manual 写操作（滑条、预设、`center_all`、`nod`、`wink`，均返回错误），前端同步把这些控件置灰。超过超时时间（默认 `500ms`，`config.py` 的 `EXTERNAL_INPUT_TIMEOUT_MS`）没有新帧，100Hz 心跳循环会在下一个 tick 自动把控制源降回 `Manual`——目标保持断流前的最后值，不会跳变回中位。也可以调用 `force_manual_control` 立即强制切回（如果外部源仍在发送，它的下一帧会重新抢回 `External`，这只是一次性的"帮我拿回面板"操作，不是永久屏蔽）。
- **配置**：`raspi/config.py` 的 `EXTERNAL_INPUT_PORT` / `EXTERNAL_INPUT_TIMEOUT_MS`，导出后进入 `motor_config.json` 的 `externalInput` 字段；缺省时 Rust 侧回退到 `6100`/`500ms`。
- **Tauri 命令**：`get_external_input_status`（端口、是否 active、最近 seq、EMA 帧率、超时配置）、`force_manual_control`（立即切回 Manual）。前端每 300ms 轮询一次 `get_runtime_state` + `get_external_input_status`，用于顶部"Control source"状态展示和控件置灰，即使用户没有手动操作也能实时感知外部驱动的接入/断开。

> 三态仲裁的第三个源 `Idle`（Manual 且长时间静止，触发待机噪声/眨眼）由"待机噪声与眨眼调度器"一节接入，判定条件依赖序列播放器（是否有序列在播）与待机调度器本身，具体规则见该节。

## 缓动与表情序列播放

原有的恒速插值（100Hz、每 tick 最多 2°/`step_towards`）仍然是安全底层，**永远在跑，不会被移除或放宽**。在它之上加了一层"轨迹生成"：预设/序列不再把最终目标直接写进 `target_applied`，而是由一个"缓动过渡"（`ActiveTransition`）按经过的时间逐 tick 算出中间目标，恒速插值继续对这个不断变化的中间目标做限速追赶——如果缓动曲线某一刻需要的速度超过 200°/s，`current_applied` 就会短暂落后于目标，而不是安全上限被抬高。

- **缓动函数**（`control.rs` 的 `EasingKind`）：`linear`、`ease_in_out_cubic`、`min_jerk`（`10t³-15t⁴+6t⁵`，两端速度和加速度都为零，是三者里最"顺"的一个，因此是默认值）。
- **预设按钮**：点击后不再瞬间跳变，而是按默认过渡（`600ms` + `min_jerk`，`control.rs` 里的常量，改动需要重新编译）经缓动层过渡到目标。
- **序列脚本**：JSON 文件放在 **`src-tauri/sequences/*.json`**（和 `src-tauri/config/motor_config.json` 平级，Rust 启动时扫描整个目录；单个文件解析失败只跳过并打日志，不影响其它序列或整个应用启动）。字段用蛇形命名（`transition_ms`、`hold_ms`），这是有意和协议其余部分的 camelCase 区分开的——序列文件是运维手写/编辑的资产，直接照抄任务文档给的示例格式：

  ```json
  {
    "id": "demo_1",
    "label": "演示序列1：中性→惊讶→喜悦→回中性",
    "steps": [
      { "preset": "惊讶", "intensity": 1.0, "transition_ms": 500, "easing": "min_jerk", "hold_ms": 1200 },
      { "preset": "喜悦", "intensity": 0.8, "transition_ms": 700, "easing": "ease_in_out_cubic", "hold_ms": 1500 }
    ],
    "loop": false
  }
  ```

  仓库自带两个演示序列：[demo_1.json](src-tauri/sequences/demo_1.json)（中性→惊讶→喜悦→回中性）、[demo_wink_loop.json](src-tauri/sequences/demo_wink_loop.json)（循环眨眼）。`preset` 字段引用的是 `presets.json` 里的表情 id（本仓库是中文标签如 `"惊讶"`/`"喜悦"`/`"wink"`/`"rest"`，不是任务文档示例里的英文 id `"surprise"`/`"happy"`——照仓库现状为准）。
- **播放器**：Rust 端在心跳循环里驱动，每步先启动该步骤的缓动过渡（复用预设强度缩放逻辑），等待 `transition_ms + hold_ms` 后进入下一步；`loop: true` 时播完最后一步回到第一步继续。Tauri 命令：`list_sequences`、`play_sequence(sequenceId)`、`stop_sequence`、`get_sequence_playback_status`（当前序列/步骤索引/总步数）。
- **抢占规则**：任何手动写操作（拖滑条、点预设、`center_all`、`nod`、`wink`）都会立刻清掉正在跑的缓动过渡和序列播放，目标保持在被打断那一刻的值——不会先播完当前步骤再让位。External 源激活时序列无法开始播放（`play_sequence` 会像其它手动命令一样返回错误）。前端每 300ms 轮询播放状态，播放中的序列按钮高亮，并显示"当前步骤/总步骤"。

## 待机噪声与眨眼调度器

`control_source` 仲裁的第三态 `Idle` 在这里接入：当控制源是 `Manual`、没有序列在播放、也没有正在跑的缓动过渡，且目标已经静止超过 `idle_after_seconds`（默认 `3s`）时，自动进入 `Idle`；一旦有任何手动写操作（拖滑条、点预设、`center_all`/`nod`/`wink`、开始播放序列）或外部系数流接入，立刻退出 `Idle` 回到 `Manual`/`External`，目标保持在被打断那一刻的值。

- **待机噪声**：进入 `Idle` 那一刻的 32 通道目标被当作"基准姿态"快照下来；配置的通道子集（默认眉毛 `0-3`、眼球 `8`/`13`、脖子 `30`/`31`）在各自的归一化系数空间上叠加一个平滑随机游走——每隔一个随机周期（由 `noiseFreqMinHz`~`noiseFreqMaxHz`，默认 `0.2~0.5Hz` 换算出的周期区间）重新挑一个 `[-noiseAmplitude, noiseAmplitude]`（默认 `±0.03`）范围内的随机目标，当前噪声值再按固定平滑系数向那个随机目标指数逼近，而不是每次瞬间跳变。叠加结果重新钳位后照常走限速插值。
- **眨眼**：随机间隔 `blinkMinIntervalSeconds`~`blinkMaxIntervalSeconds`（默认 `2~6s`）触发一次，固定作用于眼睑通道 `9`/`10`/`11`/`12`：闭合 `80ms` → 保持 `60ms` → 打开 `120ms`（打开目标是眨眼开始前的基准姿态，不是固定的"全开"值），闭合/打开都用 `ease_in_out_cubic` 复用缓动层实现。眨眼期间该通道暂停噪声叠加。**"闭合"对应哪个方向的物理极限尚未经硬件验证**（约定与 `tools/face_visualizer.py` 的眼睑覆盖度一致：归一化系数 `+1` = 闭合），标了 TODO 留给硬件联调。
- **配置**：集中在 `raspi/config.py` 的 `IDLE_BEHAVIOR` 字典，导出进 `motor_config.json` 的 `idleBehavior` 字段（每个键都有默认值，缺失也能正常加载）。运行时还有一个独立于配置文件的**总开关**：Tauri 命令 `set_idle_behavior_enabled(enabled)`，前端顶部有对应的勾选框；关闭时如果正处于 `Idle` 会立即退回 `Manual`。
- External 源激活时待机行为完全不生效（`Idle` 只能从 `Manual` 进入，`External` 会话期间连判定条件都不会检查）。

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
| [raspi/servo_server.py](raspi/servo_server.py) | 树莓派 UDP 执行器（支持 `--dry-run`，无硬件时跳过 I2C，只打印帧率/摘要） |
| [tools/face_visualizer.py](tools/face_visualizer.py) | 无硬件开发用：监听 UDP、渲染 2D 简笔人脸，可完全替代树莓派执行器 |
| [src-tauri/sequences/](src-tauri/sequences) | 表情序列脚本（`*.json`），启动时扫描整个目录 |
| [src-tauri/src/control.rs](src-tauri/src/control.rs) | Rust 控制核心（补偿、插值、联动、预设、缓动/序列、外部输入仲裁、心跳、日志） |
| [src/App.tsx](src/App.tsx) | 前端控制台 UI |
| [docs/setup_guide_zh.md](docs/setup_guide_zh.md) | 环境配置与部署文档 |

## 当前边界

适合：标定、结构联调、表情复现、数据采集控制，以及后续接入更复杂控制算法前的底座。

不负责：自动表情求解、视觉驱动表情映射、ROS2 联动。这些在控制台稳定之后作为下一层叠加。
