# BionicFace Panel

用于仿生人脸前期硬件标定与数据采集的桌面控制台。

当前架构不是“AI 直接控制表情”，而是一个面向实验阶段的 32 通道可视化控制系统：

- React 前端提供 32 个滑条
- Tauri/Rust 后端负责约束、补偿、插值与日志
- Raspberry Pi 只做 UDP 接收和 PCA9685 执行

## 项目目标

这套系统的目标是帮助团队先把这些事情做稳定：

- 电机映射与硬件通道校准
- 限位与零位补偿验证
- 面部结构联动调试
- 数据采集时的重复控制与时间记录

目前不把复杂的 Blendshape 映射或 AI 驱动放在主链路里。

## 当前架构

### 1. 前端

位置：

- [src/App.tsx](/Users/seanja/Developer/BionicFace-Panel/src/App.tsx)

作用：

- 显示 32 个滑条
- 展示每个通道的逻辑值与当前应用值
- 显示当前 UDP 目标地址、禁用通道和最后一帧数据
- 通过 Tauri `invoke` 把滑条变化发送给 Rust

### 2. 上位机 Rust 后端

位置：

- [src-tauri/src/main.rs](/Users/seanja/Developer/BionicFace-Panel/src-tauri/src/main.rs)
- [src-tauri/src/control.rs](/Users/seanja/Developer/BionicFace-Panel/src-tauri/src/control.rs)

作用：

- 读取导出的硬件配置 JSON
- 对每个通道做 `clamp + offset`
- 对目标角度做 100Hz 插值
- 以 UDP JSON 持续发送 32 通道最终角度
- 记录发送日志

### 3. 树莓派执行器

位置：

- [raspi/servo_server.py](/Users/seanja/Developer/BionicFace-Panel/raspi/servo_server.py)

作用：

- 监听 UDP 端口
- 解析包含 `angles` 数组的 JSON
- 直接将角度写入 PCA9685

树莓派不负责：

- 限位计算
- offset 补偿
- 插值
- 表情映射

这部分逻辑全部放在上位机。

## 通信机制

### 上位机内部

前端 React 不直接访问树莓派。

链路如下：

```text
React Slider -> Tauri invoke -> Rust ControlService
```

也就是说：

- 前端只是发“目标逻辑值”
- Rust 才是实际控制大脑

### 上位机到树莓派

当前使用 UDP JSON。

链路如下：

```text
Rust -> UDP JSON -> Raspberry Pi
```

典型数据帧结构：

```json
{
  "frameId": 123,
  "timestampNs": 1742600000000000000,
  "timestampRfc3339": "2026-03-21T12:00:00.000000Z",
  "source": "udp-heartbeat",
  "angles": [32 个最终角度]
}
```

其中：

- `frameId`：帧编号
- `timestampNs`：高精度时间戳
- `angles`：最终发往 PCA9685 的 32 通道物理角度

## 配置机制

唯一真实标定源是：

- [raspi/config.py](/Users/seanja/Developer/BionicFace-Panel/raspi/config.py)

这个文件当前定义：

- `BOARD_ADDRESSES`
- `UDP_PORT`
- `MOTOR_NAMES`
- `DISABLED_MOTORS`
- `MOTOR_MAP`
- `MOTOR_LIMITS`
- `MOTOR_OFFSET`

### 配置导出流程

Rust 不直接解析 Python。

所以我们通过导出脚本把 `config.py` 转成 JSON：

- [raspi/export_config_json.py](/Users/seanja/Developer/BionicFace-Panel/raspi/export_config_json.py)

执行：

```bash
python3 raspi/export_config_json.py
```

导出结果：

- [motor_config.json](/Users/seanja/Developer/BionicFace-Panel/src-tauri/config/motor_config.json)

Rust 启动时读取这份 JSON。

## 逻辑角度与物理角度

在当前系统里有两层概念：

- 逻辑值：前端滑条输入的目标值
- 应用值：Rust 做过 `clamp + offset` 后真正下发的物理角度

计算关系：

```text
applied = clamp(logical + offset, minApplied, maxApplied)
```

所以：

- 前端只负责调目标
- Rust 负责把目标变成安全可执行值
- 树莓派只负责执行

## 为什么脖子通道还保留

当前协议仍然固定为 32 通道。

虽然 30、31 两个脖子电机暂时不参与数据采集，但它们仍保留在协议里，原因是：

- 不破坏 32 通道数据结构
- 后续脖子结构恢复时不需要改通信协议
- 上下位机仍能保持一致的通道索引

当前这两个通道会被标记为 disabled，并保持安全中位。

## 关键文件总览

- [raspi/config.py](/Users/seanja/Developer/BionicFace-Panel/raspi/config.py)
  - 硬件标定源
- [raspi/export_config_json.py](/Users/seanja/Developer/BionicFace-Panel/raspi/export_config_json.py)
  - Python 配置导出到 Rust JSON
- [raspi/servo_server.py](/Users/seanja/Developer/BionicFace-Panel/raspi/servo_server.py)
  - 树莓派 UDP 执行器
- [src-tauri/src/control.rs](/Users/seanja/Developer/BionicFace-Panel/src-tauri/src/control.rs)
  - Rust 控制核心
- [src/App.tsx](/Users/seanja/Developer/BionicFace-Panel/src/App.tsx)
  - 前端控制台 UI
- [docs/setup_guide_zh.md](/Users/seanja/Developer/BionicFace-Panel/docs/setup_guide_zh.md)
  - 环境配置与部署文档

## 当前建议工作流

1. 修改 [config.py](/Users/seanja/Developer/BionicFace-Panel/raspi/config.py)
2. 运行 `python3 raspi/export_config_json.py`
3. 启动树莓派执行器
4. 启动 `npm run tauri dev`
5. 在 UI 中连接 `<pi-ip>:6000`
6. 开始单通道或多通道校准

## 当前边界

目前这套系统适合：

- 标定
- 结构联调
- 数据采集控制
- 后续接入更复杂控制算法前的底座搭建

目前不负责：

- 自动表情求解
- 视觉驱动表情映射
- ROS2 联动控制链路

这些可以在当前控制台稳定之后，作为下一层叠加上去。
