# unilidar_l2

宇树 **Unitree L2** 激光雷达的串口协议库 + 上位机点云查看软件。

协议为 **MAVLink v2 @ 2,000,000 8N1**，完整逆向记录见
[`PROTOCOL.md`](PROTOCOL.md)。

- `unilidar_l2/` —— 纯 Python 库（只依赖 numpy，自带 Win32/POSIX 串口实现，**不需要 pyserial**）
- `viewer.py` —— PySide6 上位机，软件渲染 3D 点云（不依赖 OpenGL，远程桌面也能用）

## 安装依赖

```powershell
pip install numpy PySide6
```

或者：

```powershell
pip install -r requirements.txt
```

## 运行上位机

```powershell
python viewer.py
```

或双击 `run_viewer.bat`。

界面说明：

- 左侧：串口 / 波特率、连接、回放 `.bin`、颜色模式、点大小、最大点数、保存 PLY/PNG、录制原始数据、雷达命令
- 中间：3D 点云
  - 左键拖动：旋转
  - 右键 / 中键拖动：平移
  - 滚轮：缩放
  - `R` 或双击：复位视角
- 右侧：固件版本、统计、IMU、扫描状态
- 底部：日志

没有硬件也可以点“回放 .bin”，选择 `samples/sample_l2.bin` 试一下。

颜色模式：高度 Z / 反射强度 / 距离 / 方位角 / 单色。

## 命令行工具

```powershell
# 列出串口
python -m unilidar_l2 ports

# 读取设备信息（固件、IMU、扫描率等）
python -m unilidar_l2 info --port COM8 --seconds 2

# 采集点云保存为 PLY（CloudCompare / MeshLab 可直接打开）
python -m unilidar_l2 ply --port COM8 --seconds 5 -o scan.ply

# 录制原始字节，供离线分析 / 回放
python -m unilidar_l2 dump --port COM8 --seconds 5 -o capture.bin

# 离线解析录制文件（可顺便导出 PLY）
python -m unilidar_l2 replay capture.bin --ply scan.ply
```

## 作为库使用

```python
import numpy as np
from unilidar_l2 import UnitreeL2, ScanEvent, ImuEvent, Calibration, save_ply

lidar = UnitreeL2("COM8", baudrate=2_000_000, calibration=Calibration())
lidar.open()
lidar.request_version()

points = []
imu = None
version = None

for ev in lidar.iter_events(timeout=0.05):
    if isinstance(ev, ScanEvent):
        points.append(ev.points)          # (N, 4) x, y, z, intensity
    elif isinstance(ev, ImuEvent):
        imu = ev.imu
    elif ev.__class__.__name__ == "VersionEvent":
        version = ev.version

# 按需停止后：
lidar.close()
if points:
    save_ply("scan.ply", np.concatenate(points))
```

带超时的轮询写法（适合放进线程 / 定时器）：

```python
from unilidar_l2 import UnitreeL2, ScanEvent

with UnitreeL2("COM8") as lidar:
    lidar.request_version()
    while True:
        for ev in lidar.poll(0.05):
            if isinstance(ev, ScanEvent):
                print(ev.packet_id, ev.points.shape, ev.aux.scan_rate_hz)
```

发送命令：

```python
lidar.request_version()
lidar.request_time_sync()
lidar.set_working_mode(1)   # 1 NORMAL, 2 STANDBY
lidar.set_led_mode(8)       # 六段呼吸
```

## 目录结构

```
unilidar_l2/
├─ unilidar_l2/
│  ├─ protocol.py      # MAVLink v2 帧、CRC、消息编解码、命令
│  ├─ geometry.py      # 角度 + 距离 -> XYZ，PLY 导出
│  ├─ serial_io.py     # Win32 ctypes / POSIX termios 串口
│  ├─ lidar.py         # 高层读取器（配对、事件、统计）
│  ├─ __main__.py      # 命令行工具
│  └─ __init__.py
├─ viewer.py           # PySide6 上位机
├─ PROTOCOL.md         # 协议逆向记录
├─ requirements.txt
├─ pyproject.toml
└─ run_viewer.bat
```

## 常见问题

**连上但收不到数据**

1. 确认波特率是 `2000000`（本协议默认值）。
2. 雷达是否独立供电？拆机雷达常见问题是只接了串口、没给雷达供电。
3. 若之前发送过探测命令，雷达可能进入待机/异常状态，**给雷达断电再上电**。
4. 确认 COM 口不是被别的程序占用。

**数据能收到但点云很乱**

- 检查 `yaw_bias`（默认 `-38.5°`）是否符合你的安装方向；该值只影响整体旋转。
- 距离单位是毫米，`range == 0` 的点必须丢弃。
- 别把 `AUX(N)` 配到上一圈的 `DIST(N)`：按时间顺序配对，见 `PROTOCOL.md`。

**没有 OpenGL 能用吗**

- 可以。`viewer.py` 用 numpy 软件渲染，通过远程桌面也能显示。

## 说明

- 本库面向宇树 Unitree L2 激光雷达（USB CDC 串口链路）。
- `request_type` / `cmd_type` 的完整用途未全部验证，未知命令请勿随意发送。
