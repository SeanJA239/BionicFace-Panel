# 关闭并固定自动曝光 / 白平衡 / 增益 / 对焦

目标：让 C100/C70 每次上电都输出**同一组成像参数**，使表情驱动链路（`tools/mediapipe_driver.py`）
上游的图像统计量可复现。实现在 [`tools/camera_capture.py`](../../tools/camera_capture.py)。

适用范围：Linux + `uvcvideo`（树莓派 / Linux 上位机）。本文所有控制项语义、编号、主从约束均引自
Linux 内核源码，出处逐条给在表里。

## 0. 运行环境：为什么必须是 Linux

参数锁定这件事绑在 V4L2 上，不是偷懒。Windows 原生（DirectShow）能关掉自动曝光/白平衡/对焦，
但拿不到同等强度的保证——查 OpenCV 5.x `cap_dshow.cpp` 可见：

- 有范围查询能力却不外露：`IAMVideoProcAmp::GetRange` / `IAMCameraControl::GetRange` 只在后端内部
  调用（`:1943`、`:1999`），Python API 无对应接口，拿不到 min/max/step/default/flags，
  「越界即报错而不 clamp」失去依据。
- 写入不做范围校验：`:1870`、`:1952` 两处注释原文
  `// Perhaps add a check that lValue and Flags are within the range acquired from GetRange above`，
  与 V4L2 后端那句「driver may clamp … ignored here」是同一类问题。
- 回读语义不同：`CAP_PROP_AUTO_WB` 读回的是 `flags == CameraControl_Flags_Auto ? 1.0 : 0.0`
  （`:3438-3440`），不是实际数值。
- 无 master/slave 约束，写入顺序错了不会有任何提示。
- 曝光标度与 V4L2 的 100 µs 单位不同（DirectShow 文档为 log₂ 秒），**参数组不可跨系统复用**。

要在 Windows 上做到等强度，需用 `comtypes` 直接驱动 `IAMCameraControl` / `IAMVideoProcAmp`
另写一层，规模与现有 V4L2 层相当。本项目不走这条路。

### 开发期在 WSL2 上验证

WSL2 默认内核**自带 UVC 支持**，无需重编内核，确认一下即可：

```bash
grep -E 'CONFIG_(VIDEO_DEV|USB_VIDEO_CLASS)=' /proc/config.gz  # 或 zcat
ls /lib/modules/$(uname -r)/kernel/drivers/media/usb/uvc/uvcvideo.ko
```

把相机从 Windows 转发进来（[usbipd-win](https://github.com/dorssel/usbipd-win)）：

```powershell
winget install usbipd
usbipd list                        # 找到相机的 BUSID
usbipd bind --busid=<BUSID>        # 需管理员，持久生效
usbipd attach --wsl --busid=<BUSID>  # 无需管理员，重启/重插后要重做
```

WSL 侧确认节点出现，必要时手动加载模块：

```bash
sudo modprobe uvcvideo
ls /dev/video*
```

两个坑：

1. **`attach` 不持久**。重启、重插、设备 reset 之后都要重新 attach，而每次重新 attach 相当于相机
   重新上电——控制值回出厂默认，必须重跑 `lock`。这正是 §7「开机恢复」要解决的问题，在 WSL2 下会
   更频繁地遇到。
2. **WSL2 是独立网络命名空间**。`tools/mediapipe_driver.py` 默认往 `127.0.0.1:6100` 发包，在非镜像
   网络模式下到不了 Windows 上的 ControlService，需要改用 Windows 主机 IP，或开启 WSL 镜像网络模式。
   USB/IP 转发本身也会引入额外抖动，实测帧率请以最终部署机器为准，不要拿 WSL2 的数据验收。

## 1. 前置条件

1. C100 若未刷过 `docs/WHEELTEC C100 C70产品资料/.../4.C100暗处曝光闪白问题更新固件/` 的固件，
   暗处曝光行为本身不可复现（该目录即为「暗处曝光闪白」缺陷修复包，Windows 工具）。先刷固件，再锁参数。
2. 确认 capture 节点。UVC 相机会创建两个 `/dev/video*` 节点（capture + metadata），取**编号较小**
   的那个（厂商 `5.Python例程/readme.txt` 也是这个说法）。判据以 `index` 属性为准：
   ```bash
   udevadm info -q property /dev/video0 | grep -E 'ID_V4L_CAPABILITIES|ID_MODEL|ID_VENDOR_ID|ID_MODEL_ID'
   ```
3. 先协商格式再锁曝光。`V4L2_CID_EXPOSURE_ABSOLUTE` 的取值上限受**帧间隔**限制（内核文档
   `Documentation/userspace-api/media/v4l/ext-ctrls-camera.rst`：「The exposure time is limited by
   the frame interval」），所以顺序固定为：设分辨率/fourcc/fps → 锁控制项。

## 2. 为什么不用 OpenCV 的 `cap.set()`

OpenCV 的 V4L2 后端会把 `CAP_PROP_*` 映射到 V4L2 CID（`modules/videoio/src/cap_v4l.cpp`
`capPropertyToV4L2()`，如 `CAP_PROP_AUTO_EXPOSURE → V4L2_CID_EXPOSURE_AUTO`、
`CAP_PROP_EXPOSURE → V4L2_CID_EXPOSURE_ABSOLUTE`），但它**不适合做参数锁定**，理由都在 5.x 源码里：

1. `icvControl()` 里 `VIDIOC_S_CTRL` 的返回被吞掉，源码自己写着
   `/* The driver may clamp the value or return ERANGE, ignored here */`（`cap_v4l.cpp:1786`），
   失败只在 `CV_LOG_DEBUG` 级别打一行（`cap_v4l.cpp:1789`）。**驱动截断或拒绝，`set()` 依然返回 true。**
2. 取值范围可能被隐式重标定：`normalizePropRange` 由环境变量
   `OPENCV_VIDEOIO_V4L_RANGE_NORMALIZED` 控制（`cap_v4l.cpp:931`，默认 false），
   也可运行时通过属性改（`cap_v4l.cpp:1928`）。开启后 `CAP_PROP_AUTO_EXPOSURE` 按 `Range(0,4)`
   重映射，同一份数值在两台机器上含义可以不同。
3. 没有 `min/max/step/default/flags` 查询接口，无法判定某控制项是否存在、是否处于 INACTIVE。

因此本仓库的做法：**取流用 OpenCV，控制项用 V4L2 ioctl 直接读写，并逐项回读校验**。控制项通过
**数值 CID** 寻址（CID 是稳定 ABI），不依赖控制项名字（名字随内核版本变过，见 §3 注）。
厂商官方示例 `cam_usb_set/set.cpp` 也是同一路线：单独 `open("/dev/video0", O_RDWR)` 后用
`VIDIOC_QUERYCTRL` / `VIDIOC_S_CTRL`，与 OpenCV 取流互不干扰。

## 3. 控制项清单

CID 数值由 `include/uapi/linux/v4l2-controls.h` 推出（`V4L2_CID_BASE = 0x00980900`，
`V4L2_CID_CAMERA_CLASS_BASE = 0x009A0900`）。「v4l2-ctl 名」是 `v4l2-ctl` 对驱动上报名字做
小写化+非字母数字转下划线后的结果（`v4l-utils` 的 `name2var()`），名字取自
`v4l2-core/v4l2-ctrls-defs.c`。

| JSON 键（本仓库） | v4l2-ctl 名 | CID 宏 | CID 值 | 类型 | 锁定值 | 说明 |
| --- | --- | --- | --- | --- | --- | --- |
| `auto_exposure` | `auto_exposure` | `V4L2_CID_EXPOSURE_AUTO` | `0x009A0901` | menu | **1** | 菜单：0=Auto Mode, 1=Manual Mode, 2=Shutter Priority, 3=Aperture Priority。UVC 设备通常只暴露 1 和 3 |
| `exposure_time_absolute` | `exposure_time_absolute` | `V4L2_CID_EXPOSURE_ABSOLUTE` | `0x009A0902` | int | 实测值 | **单位 100 µs**（1 = 1/10000 s，10000 = 1 s）。从属于 `auto_exposure` |
| `exposure_dynamic_framerate` | `exposure_dynamic_framerate` | `V4L2_CID_EXPOSURE_AUTO_PRIORITY` | `0x009A0903` | bool | **0** | 0 = 不允许驱动/相机动态改帧率。仅在 AE 处于 Auto/Aperture Priority 时生效，但为防回退到自动仍显式写 0 |
| `white_balance_automatic` | `white_balance_automatic` | `V4L2_CID_AUTO_WHITE_BALANCE` | `0x0098090C` | bool | **0** | AWB 开关 |
| `white_balance_temperature` | `white_balance_temperature` | `V4L2_CID_WHITE_BALANCE_TEMPERATURE` | `0x0098091A` | int | 实测值 | 色温（K）。从属于 `white_balance_automatic` |
| `gain` | `gain` | `V4L2_CID_GAIN` | `0x00980913` | int | 实测值 | 见 §4「增益的坑」 |
| `focus_automatic_continuous` | `focus_automatic_continuous` | `V4L2_CID_FOCUS_AUTO` | `0x009A090C` | bool | **0** | 连续自动对焦开关；这两款很可能不存在此控制项，见 §5 |
| `focus_absolute` | `focus_absolute` | `V4L2_CID_FOCUS_ABSOLUTE` | `0x009A090A` | int | 实测值 | 从属于 `focus_automatic_continuous` |
| `power_line_frequency` | `power_line_frequency` | `V4L2_CID_POWER_LINE_FREQUENCY` | `0x00980918` | menu | **1** | 0=Disabled, 1=50Hz, 2=60Hz, 3=Auto。国内市电取 1，抑制条纹；`Auto` 会引入自适应行为 |
| `backlight_compensation` | `backlight_compensation` | `V4L2_CID_BACKLIGHT_COMPENSATION` | `0x0098091C` | int | 实测值 | 规格书标称「背光补偿：自动」，若存在此项应显式定死 |
| `brightness` | `brightness` | `V4L2_CID_BRIGHTNESS` | `0x00980900` | int | 实测值 | 以下 5 项不是自动算法开关，但同属「换台机器就变」的画质参数，一并入锁 |
| `contrast` | `contrast` | `V4L2_CID_CONTRAST` | `0x00980901` | int | 实测值 | |
| `saturation` | `saturation` | `V4L2_CID_SATURATION` | `0x00980902` | int | 实测值 | |
| `sharpness` | `sharpness` | `V4L2_CID_SHARPNESS` | `0x0098091B` | int | 实测值 | |
| `gamma` | `gamma` | `V4L2_CID_GAMMA` | `0x00980910` | int | 实测值 | |

> **名字注**：`auto_exposure` / `exposure_time_absolute` / `white_balance_automatic` /
> `focus_automatic_continuous` 是较新内核的名字（分别来自内核字符串 `"Auto Exposure"`、
> `"Exposure Time, Absolute"`、`"White Balance, Automatic"`、`"Focus, Automatic Continuous"`）。
> 旧内核上 `v4l2-ctl -l` 可能显示 `exposure_auto` / `exposure_absolute` /
> `white_balance_temperature_auto` / `focus_auto`。**命令行按本机 `v4l2-ctl -l` 输出为准；代码按 CID
> 寻址，不受影响。** 本仓库 JSON 键固定用上表的新名字。

厂商教程 `USB相机标定教程.pdf` §2.2 给出的 C100 参考画质参数：`Brightness 0 / Contrast 90 /
Saturation 80 / Sharpness 0 / Gamma 40`。注意同一资料包里 `set.cpp` 把 Gamma 的范围写成
`100~500` 而默认值写 `40`（自相矛盾），且它对越界值做静默 clamp。**这些数只能当起点，实际范围一律以
`VIDIOC_QUERYCTRL` 回报的 min/max/step 为准**——本仓库的实现对越界值直接报错而不 clamp。

## 4. 写入顺序：必须先关自动，再写数值

`drivers/media/usb/uvc/uvc_ctrl.c` 的映射表里，三对控制项是 master/slave 关系：

| master | slave | master 必须等于 |
| --- | --- | --- |
| `V4L2_CID_EXPOSURE_AUTO` | `V4L2_CID_EXPOSURE_ABSOLUTE` | `V4L2_EXPOSURE_MANUAL`（=1） |
| `V4L2_CID_AUTO_WHITE_BALANCE` | `V4L2_CID_WHITE_BALANCE_TEMPERATURE`（还有 `BLUE/RED_BALANCE`） | `0` |
| `V4L2_CID_FOCUS_AUTO` | `V4L2_CID_FOCUS_ABSOLUTE` | `0` |

约束是驱动强制的，不是习惯问题：`uvc_ctrl_is_accessible()` 在写从属控制项时会读 master，
若 master 不等于 `master_manual` 就返回 **`-EACCES`**（`uvc_ctrl.c` 内
`return ctrls->controls[i].value == mapping->master_manual ? 0 : -EACCES;` 及其后的
`if (ret >= 0 && val != mapping->master_manual) return -EACCES;`）。同时 `QUERYCTRL` 会给该从属项打上
`V4L2_CTRL_FLAG_INACTIVE`（`0x0010`），可用于判定当前是否真的处于手动模式。

`uvcvideo` 只实现了 `g/s_ext_ctrls`，但内核 `v4l2-ioctl.c` 会把传统 `VIDIOC_QUERYCTRL` /
`G_CTRL` / `S_CTRL` 转发过去（`v4l_queryctrl()` / `v4l_g_ctrl()` / `v4l_s_ctrl()`），所以用简单 ioctl
即可，且同样受上面的 EACCES 检查保护。

**顺序**：`auto_exposure=1` → `white_balance_automatic=0` → `focus_automatic_continuous=0`
（存在则写）→ `exposure_dynamic_framerate=0` → 其余数值项任意序。
`tools/camera_capture.py` 把前四项固定排在最前（`_APPLY_FIRST`）。

### 增益的坑

UVC **没有独立的 AGC 开关**：AGC 归自动曝光管，`auto_exposure=1` 之后增益就不再自动变化。
但「不再变化」不等于「可复现」——它被冻结在 AE 最后一次算出的值上，而那取决于关 AE 瞬间的照度。
所以必须**显式写 `gain`**。若 `v4l2-ctl -l` 里没有 `gain`（部分 UVC 固件不暴露），则该机型的增益无法
用软件定值，只能：固定照明 → 预热 → 关 AE → `dump` 记录当次的实际值 → 每次开机用 `verify` 确认，
差异超阈值就报警。这一限制要写进验收标准，不要假装锁住了。

## 5. 对焦

`docs/camera/CAPABILITIES.md` 里对焦形式和最近对焦距离都是 `UNKNOWN`：C100/C70 的资料全篇没有出现
自动对焦、对焦行程、MOD 等任何字样，规格书只写「2.1mm/2.8mm 标准镜头，可以带 IRCUT」。这类模组通常
是螺纹手动调焦的定焦镜头，但资料不能自证。判定方法：

```bash
v4l2-ctl -d /dev/video0 -l | grep -i focus
```

- **没有输出** → 相机不暴露对焦控制项，软件层无需（也无法）锁定。此时「固定对焦」是机械问题：
  调焦到位后在镜筒螺纹上点厌氧胶/UV 胶锁死，并在装配文档里记录。本仓库配置里把
  `focus_automatic_continuous` / `focus_absolute` 留空即可（`null`，见 §6）。
- **有输出** → 按 §3/§4 一起锁：先 `focus_automatic_continuous=0`，再 `focus_absolute=<实测值>`。

## 6. 参数组文件与命令

参数组是一个 JSON 文件（默认 `tools/camera_params.json`），`controls` 里 `null` 表示
**该项尚未在真机上定标**，`lock` 时会直接报错而不是拿默认值凑——不产生编造的数值。

```jsonc
{
  "device": "/dev/video0",
  "width": 640, "height": 480, "fps": 30.0, "fourcc": "MJPG",
  "controls": {
    "auto_exposure": 1,                  // 1 = Manual Mode
    "exposure_time_absolute": null,      // 100µs 单位，真机 dump
    "exposure_dynamic_framerate": 0,
    "white_balance_automatic": 0,
    "white_balance_temperature": null,   // K，真机 dump
    "gain": null,
    "power_line_frequency": 1            // 50Hz
  }
}
```

首次在真机上建立参数组：

```bash
# 1) 看这台相机到底有哪些控制项、范围、默认值、是否 INACTIVE
python tools/camera_capture.py list --device /dev/video0

# 2) 布好最终光照，让 AE/AWB 收敛（出厂默认即自动），肉眼确认画面可用
ffplay /dev/video0            # 或 python docs/.../5.Python例程/camera.py

# 3) 把当前实际生效的值抓成参数组（此时仍是自动模式下的收敛值）
python tools/camera_capture.py dump --device /dev/video0 --config tools/camera_params.json

# 4) 手工把 auto_* 开关改成锁定值（auto_exposure=1 等），微调数值项后写回
# 5) 应用并逐项回读校验
python tools/camera_capture.py lock --config tools/camera_params.json
```

等价的纯 `v4l2-ctl` 命令（诊断用；注意名字按本机 `-l` 输出）：

```bash
v4l2-ctl -d /dev/video0 -l                     # 查看全部控制项与当前值
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_dynamic_framerate=0
v4l2-ctl -d /dev/video0 --set-ctrl=power_line_frequency=1
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=<值>,gain=<值>,white_balance_temperature=<值>
v4l2-ctl -d /dev/video0 --get-ctrl=auto_exposure,exposure_time_absolute,gain,white_balance_temperature
```

## 7. 开机恢复同一组参数

控制值保存在**相机内部**，掉电或重新插拔即回到固件默认（自动曝光/自动白平衡）。同一次上电内多次
open/close 一般保持不变，但不作为保证。因此每次上电都要重放参数组，并且**回读校验**。

### 方案 A（首选）：取流进程自己锁

应用进程打开相机后立刻锁参数，顺序与 §1.3 一致，一步到位，不依赖系统服务：

```python
from pathlib import Path

from tools.camera_capture import Camera, CaptureConfig

config = CaptureConfig.load(Path("tools/camera_params.json"))
with Camera(config) as cam:      # open() 里先协商 fourcc/分辨率/fps
    cam.lock_params()            # 再写控制项，逐项回读，不一致直接抛 ParamLockError
    while True:
        frame = cam.grab()       # frame.timestamp 为单调时钟秒（帧到达本进程的时刻）
        ...
```

`Camera.open()` 对协商结果做硬校验：fourcc/宽/高不符，或 fps 偏差超过 0.5，直接抛
`CameraError` 并提示去看 `--list-formats-ext`，不会静默降档跑一个不是你要的模式。

`lock_params()` 不一致就抛异常，不做静默重试——参数没锁住时宁可起不来，也不要跑出一份对不上账的数据。

### 方案 B：udev + systemd，插上/开机即锁

相机被别的进程占用、或希望「插上就是锁定状态」时用。先取 VID:PID（资料未给出，需实测）：

```bash
lsusb                          # 找到相机那一行
udevadm info -a /dev/video0 | grep -E 'idVendor|idProduct' | head -4
```

`/etc/udev/rules.d/80-bionicface-camera.rules`（把 `1234`/`5678` 换成实测值）：

```
# 只匹配 capture 节点（index==0），跳过 metadata 节点；给出稳定符号链接并拉起锁参服务
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", ATTR{index}=="0", \
  SYMLINK+="bionicface-cam", TAG+="systemd", ENV{SYSTEMD_WANTS}+="camera-param-lock@video0.service"
```

`/etc/systemd/system/camera-param-lock@.service`：

```ini
[Unit]
Description=Lock UVC imaging params on /dev/%i
BindsTo=dev-%i.device
After=dev-%i.device

[Service]
Type=oneshot
ExecStart=/opt/bionicface/.venv/bin/python /opt/bionicface/tools/camera_capture.py \
    lock --config /etc/bionicface/camera_params.json --device /dev/%i
# 锁完立刻回读校验，不一致则本次启动失败（journal 里能看到逐项差异）
ExecStartPost=/opt/bionicface/.venv/bin/python /opt/bionicface/tools/camera_capture.py \
    verify --config /etc/bionicface/camera_params.json --device /dev/%i
```

生效：

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=video4linux
systemctl status 'camera-param-lock@video0.service'
```

注意 `SYMLINK` 给的 `/dev/bionicface-cam` 用于业务进程；服务里仍用 `/dev/%i` 真实节点名，避免符号链接
建立时序问题。参数组文件放 `/etc/bionicface/camera_params.json`（与代码分离，便于每台机器有自己的定标值）。

### 重启后核对

```bash
python tools/camera_capture.py verify --config tools/camera_params.json
```

逐项打印 `期望值 / 实际值 / min-max-step / flags`，有差异则以非零码退出并列出差异行。
真机重启验收就跑这一条。完整链路（锁定 → 连拍 100 帧统计帧率 → 重开设备回读比对）跑：

```bash
python tools/camera_capture.py selftest --config tools/camera_params.json
```

## 8. 验收标准

1. `verify` 在冷启动后输出零差异。
2. `selftest` 连拍 100 帧的实际帧率与协商 fps 偏差在可接受范围内，且**遮挡镜头/开关房间灯时帧率不变**
   （证明 `exposure_dynamic_framerate=0` 生效、AE 未回退）。
3. 遮挡镜头 2 秒再放开，画面亮度不做「呼吸」式自适应（证明 AE/AWB 已关）。
4. 若该机型不暴露 `gain`（见 §4），必须在验收记录里写明「增益为冻结值而非定值」，并附
   `verify` 的实际读数作为基线。
5. C100 已刷 `4.` 目录固件（见 §1.1），否则暗处曝光不可复现，前四条在暗场下不成立。
