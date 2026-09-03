# C100 调试作业手册

按任务顺序排列的操作流程。`CAPABILITIES.md` 记录相机是什么，`PARAM_LOCK.md` 解释每个参数为什么这样设，
**本文只回答「现在该敲哪条命令」**。第一次上手请从 §0 概念开始，熟手直接跳 §2 检查清单。

---

## 0. 先搞懂五个概念

这一节不需要相机，读完再动手会少走很多弯路。

**UVC** — USB Video Class，USB 摄像头的标准协议。免驱就是因为它标准化了：操作系统自带的
`uvcvideo` 驱动能认所有 UVC 相机。代价是能力受协议限制，相机厂商的私有功能一律用不上。

**V4L2 与 CID** — Linux 的视频设备接口。相机的每个可调项（曝光、增益、色温……）叫一个「控制项」，
每项有一个数值 ID 叫 **CID**。CID 是稳定不变的，而控制项的**名字换内核版本会变**
（`exposure_auto` 后来改叫 `auto_exposure`）。所以本仓库代码一律按 CID 寻址，命令行才用名字。

**主从约束** — 曝光、白平衡、对焦各有一对「自动开关」和「数值项」。自动开关没关掉时，
写数值项会被驱动**直接拒绝**（返回 `-EACCES`）。所以写入顺序是固定的：先关自动，再写值。

**卷帘快门** — C100 的传感器逐行曝光而非整帧同时曝光，所以快速运动会被拍歪。
采集机器人脸时脸是静止的，影响很小，但要知道这回事。

**噪声底** — 相机和 MediaPipe 都不完美：拍一个**完全静止**的目标，前后两帧算出的关键点位置
也会有微小差异。这个差异的大小就是噪声底。它的意义是：将来某个舵机通道动了，
但关键点变化小于噪声底，那说明**该改光照或机位，而不是怀疑数据**。这是整个自建模项目的地基数字。

### 为什么非得锁参数

自建模数据集要求「关键点差异只来自舵机指令」。自动曝光会随画面明暗漂移、自动白平衡会改变肤色通道、
自动增益会改变噪声水平 —— 它们都在往数据里注入与表情无关的变化。采集可能跨越好几天，
所以参数不只要关掉，还要**每次开机恢复到同一组值**。

---

## 1. 环境准备（每次重新插拔都要过一遍）

### 1.1 把相机接进来

**真 Linux 机器**（树莓派 / Linux 上位机）：插上即可，跳到 §1.3。

**Windows + WSL2**：需要 usbipd-win 转发。首次安装：

```powershell
winget install usbipd
```

每次接入（PowerShell，`bind` 需要管理员且只需一次，`attach` 每次都要）：

```powershell
usbipd list                                    # 找到 2ce3:c670 那一行,记下 BUSID
usbipd bind --busid=<BUSID>                    # 管理员,持久生效
usbipd attach --wsl --busid=<BUSID>            # 免管理员,重启/重插后要重做
```

用完还给 Windows：`usbipd detach --busid=<BUSID>`。
**不要用 `bind --force`** —— 那会让设备在 detach 后也回不到 Windows。

### 1.2 节点权限（WSL2 首次做一次）

WSL 没有 logind seat，`/dev/video*` 是 `root:video 0660`，普通用户打不开，而且**每次 re-attach
都会重建节点，手工 `chmod` 会被冲掉**。用 udev 规则一次性解决（需要真实终端，sudo 要 TTY）：

```bash
sudo tee /etc/udev/rules.d/70-wheeltec-camera.rules >/dev/null <<'EOF'
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="2ce3", ATTRS{idProduct}=="c670", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=video4linux
```

### 1.3 确认设备就位

```bash
ls -l /dev/video*                              # 应该出现两个节点,权限可读写
cat /sys/class/video4linux/video0/index         # 必须是 0 —— 这才是 capture 节点
```

UVC 相机会创建两个节点，**编号较小（`index=0`）的才是取流用的**，另一个是 metadata。

### 1.4 模型文件（首次）

```bash
curl -L -o tools/face_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/latest/face_landmarker.task
```

---

## 2. 检查清单（熟手看这个就够）

```bash
# ① 设备在不在、有哪些控制项
uv run python tools/camera_capture.py list --device /dev/video0

# ② 参数还是不是上次锁的那组(冷启动后第一件事)
uv run python tools/camera_capture.py verify

# ③ 全链路自检:锁参 → 连拍 100 帧测帧率 → 重开设备逐项回读
uv run python tools/camera_capture.py selftest

# ④ 摆机位(实时画面 + 取景框 + 脸占比读数)
uv run python tools/preview.py

# ⑤ 端到端时延与检出率
uv run python tools/latency_check.py --duration 30

# ⑥ 噪声底(需要静止目标)
uv run python tools/noise_floor.py --label "布光方案名" --seconds 30
```

退出码约定：**0 正常**，**1 有参数差异或检出率过低**，**2 出错**（设备打不开、参数组未标定等）。

---

## 3. 首次标定一台相机

顺序不能乱 —— 装反了拿到的是错的标定值。

### 3.1 布好现场

先把机位和光照弄成**最终采集时的样子**，再标定。在临时机位上标出来的值，换到三脚架上就作废了。

```bash
uv run python tools/preview.py
```

调到读数那行变绿显示 `OK`（脸高占画面 60~80%），按 `s` 存一张作为标准机位记录，`q` 退出。
同时盯左上角：`clip hi` 不该超过百分之几，`mean` 别太低。

### 3.2 让自动模式收敛

相机出厂是全自动。让它在最终光照下流几十秒，把 AE 算到稳定：

```bash
uv run python tools/preview.py --no-landmarks --seconds 60
```

### 3.3 抓成参数组

```bash
uv run python tools/camera_capture.py dump --config tools/camera_params.json
```

### 3.4 手工改自动开关

打开 `tools/camera_params.json`，把四个自动项改成锁定值：

| 键 | 改成 | 理由 |
| --- | --- | --- |
| `auto_exposure` | `1` | 1 = Manual Mode |
| `white_balance_automatic` | `0` | 关 AWB |
| `focus_automatic_continuous` | `0` | 关 AF |
| `exposure_dynamic_framerate` | `0` | 名义上禁止动态改帧率（C100 实际不遵守，见下） |

然后检查三个数值项：

- **`exposure_time_absolute`** — 单位 100 µs。**30 fps 的上限是 300**（30 ms）。
  超过这个数相机会悄悄降帧率而 `CAP_PROP_FPS` 照样报 30。亮度不够优先加灯，不要拉曝光。
- **`gain`** — 增益是噪声主源。**优先加光把增益压到 0**。
- **`white_balance_temperature`** — `dump` 出来的值**不是 AWB 的实际结果**（AWB 开着时这一项
  停在量程下限，不回报）。需要自己选：写几个值各拍一帧，比较 R/G/B 通道均值，挑最接近中性的。

### 3.5 应用并验证

```bash
uv run python tools/camera_capture.py lock        # 逐项回读,不一致直接报错
uv run python tools/camera_capture.py selftest    # 连拍测实际帧率 + 重开设备比对
```

`selftest` 报的实测帧率如果明显低于 30，回到 §3.4 检查曝光是不是超了 300。

---

## 4. 每次开机 / 重新插拔之后

控制值存在**相机内部**，掉电或重新插拔就回出厂默认（也就是全自动）。所以：

```bash
uv run python tools/camera_capture.py verify      # 先看有没有变
uv run python tools/camera_capture.py lock        # 变了就重锁
```

采集进程自己会锁（`Camera.lock_params()`），所以正常流程里不用手动敲。
要「插上就是锁定状态」见 `PARAM_LOCK.md §7` 的 udev + systemd 方案。

---

## 5. 故障对照表

| 现象 | 原因 | 怎么办 |
| --- | --- | --- |
| `cannot open /dev/video0: Permission denied` | 不在 `video` 组，节点是 0660 | 做 §1.2 的 udev 规则 |
| `cannot open /dev/video0: No such file or directory` | 没 attach，或流中途掉线 | `usbipd list` 看状态，重新 `attach` |
| 流到几十帧突然断，日志有 `vhci_hcd: connection closed` | USB/IP 掉线（等时流套 TCP 的固有弱点） | 重新 attach 重试。长时间录制建议换真 Linux 机器 |
| `did not accept the requested format` | 配置里的 fourcc/分辨率/fps 这台相机没有 | `list` 看清楚有哪些档；C100 只用 MJPG，YUYV 档位表不可信 |
| `controls not calibrated yet: ...` | 参数组里还是 `null` | 走一遍 §3 |
| `VIDIOC_S_CTRL ...: Permission denied (其主控制项仍是自动)` | 写从属项前没关自动开关 | 用 `lock` 而不要手写顺序，它已经排好 |
| `controls did not hold: 写了 X 读回 Y` | 驱动静默截断了值 | 看 `list` 里的真实量程和步长，改成合法值。**不要改成「大概能过」的数** |
| `selftest` 帧率明显低于 30 | 曝光超过 300 | 见 §3.4；加灯而不是拉曝光 |
| `latency_check` 报检出率过低 | 机位/光照问题，不是性能问题 | 先用 `preview.py` 把脸调进取景框。**低检出率下的时延数字偏乐观**（没检出时管线短路，推理只要 2.5ms） |
| `noise_floor` 报 `only 0 of N frames produced landmarks` | 目标没被检出 | 同上。它拒绝编造噪声底是正确行为 |
| 画面能出但一片惨白/漆黑 | 曝光或色温离谱 | `preview.py` 看 `clip hi` / `mean`，回 §3.4 |

---

## 6. 判断做完了没有

1. 冷启动后 `verify` 零差异。
2. `selftest` 实测帧率接近 30，且**遮挡镜头或开关房间灯时帧率不变**（证明没退回自动曝光）。
3. 遮挡镜头两秒再放开，画面亮度不做「呼吸」式自适应（证明 AE/AWB 已关）。
4. `latency_check` 检出率接近 100%，推理 p95 明显小于 33 ms。
5. `noise_floor` 在最终布光下测过，数字和热力图已入库。
6. C100 已刷过 `4.C100暗处曝光闪白问题更新固件/` 的固件，否则暗处曝光不可复现。

---

## 7. 相关文档

- `CAPABILITIES.md` — 相机能力速查表 + 真机实测数据（档位表、控制项量程、时延）
- `PARAM_LOCK.md` — 每个控制项的 CID、主从约束、开机恢复方案、为什么不用 `cap.set()`
- `tools/camera_capture.py` — 采集与锁参核心，五个子命令
- `tools/preview.py` — 取景预览
- `tools/latency_check.py` / `tools/noise_floor.py` — 板块二两个测量工具
