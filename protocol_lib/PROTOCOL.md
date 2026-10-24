# Unitree L2 LiDAR —— 串口协议（逆向记录）

本文件记录了从宇树 Unitree L2 激光雷达上**黑盒逆向**得到的串口协议，以及验证方法。
实现见同目录 `unilidar_l2/` 库。

---

## 1. 物理层

| 项目 | 值 | 说明 |
|---|---|---|
| 接口 | USB CDC（虚拟串口） | 实测为 Horco CMSIS-DAP v2 / DAPLink 复合设备：VID `0xFAED` PID `0x4873`，MI_02 为 CDC，MI_04 为 U 盘 |
| 端口 | `COM8`（以枚举为准） | DAPLink 的 UART 桥，接雷达 UART TX/RX |
| 波特率 | **2 000 000** | 8 数据位、无校验、1 停止位、无流控（8N1） |
| 实测吞吐 | ≈ 97 kB/s | 约 180 个测距包/s（5.56 ms 一个）+ 250 Hz IMU |
| 逻辑协议 | **MAVLink v2** | 魔数 `0xFD`，SysMavlink 自定义消息集 |

> 关键点：波特率必须是 2,000,000。用其它波特率抓到的数据是“半字节混叠”垃圾
> （只出现 `{0,6,8,E}` 组成的 16 种字节），不会通过校验。

---

## 2. 帧格式（MAVLink v2）

```
偏移  长度  字段
0     1     0xFD                 STX
1     1     len                  payload 实际长度（v2 会裁掉尾部 0x00！）
2     1     incompat_flags       = 0
3     1     compat_flags         = 0
4     1     seq                  MAVLink 序号，全局自增（所有消息共用一个计数器）
5     1     sysid                = 1
6     1     compid               = 1
7     3     msgid                小端 24 位
10    len   payload
10+len 2    checksum             CRC-16/X25，小端
```

### CRC

```
crc = X25( bytes[1 .. 9+len] + crc_extra(msgid) )
```

- CRC-16/X25（初值 `0xFFFF`，多项式反射 `0x8408`）
- `crc_extra` 是每种消息固定的一字节，见下表
- CRC 覆盖范围**从 `len` 字节开始**（不含 `0xFD`），到 payload 结束

### 必须注意的 v2 截断

MAVLink v2 会去掉 payload 末尾的 `0x00`。实测距离包 `len=245`（标准长度 246），
最后一个 `0x00` 被省略。因此解析时要先按标准长度**补零**，否则最后一个点会越界。

---

## 3. 消息表

| ID | 名称 | 标准负载长 | crc_extra | 方向 |
|---:|---|---:|---:|---|
| 11 | DEVICE_REQUEST_DATA | 1 | 106 | 上位机 → 雷达 |
| 12 | DEVICE_COMMAND | 1 | 204 | 上位机 → 雷达 |
| 13 | RET_LIDAR_VERSION | 38 | 62 | 雷达 → 上位机 |
| 14 | CONFIG_LIDAR_WORKING_MODE | 1 | 84 | 上位机 → 雷达 |
| 15 | CONFIG_LED_RING_TABLE_PACKET | 56 | 236 | 上位机 → 雷达 |
| 16 | RET_LIDAR_DISTANCE_DATA_PACKET | 246 | 74 | 雷达 → 上位机 |
| 17 | RET_LIDAR_AUXILIARY_DATA_PACKET | 209 | 99 | 雷达 → 上位机 |
| 18 | RET_LIDAR_TIME_SYNC_DATA | 5 | 12 | 雷达 → 上位机 |
| 19 | RET_IMU_ATTITUDE_DATA_PACKET | 42 | 110 | 雷达 → 上位机 |

所有多字节字段均为**小端**，浮点为 IEEE-754 float32。

### 16 — RET_LIDAR_DISTANCE_DATA_PACKET（测距）

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u16 | packet_id（与辅助包配对用） |
| 2 | u16 | packet_cnt |
| 4 | u16 | payload_size（=240） |
| 6 | u8[240] | point_data：120 个 `u16` 距离，单位 **mm**，0 = 无效 |

### 17 — RET_LIDAR_AUXILIARY_DATA_PACKET（辅助/姿态/角度）

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u32 | lidar_sync_delay_time（us） |
| 4 | u32 | time_stamp_s（秒） |
| 8 | u32 | time_stamp_us（微秒） |
| 12 | u32 | sys_rotation_period（低速电机周期 us） |
| 16 | u32 | com_rotation_period（高速电机周期 us） |
| 20 | f32 | com_horizontal_angle_start（°） |
| 24 | f32 | com_horizontal_angle_step（°） |
| 28 | f32 | sys_vertical_angle_start（°） |
| 32 | f32 | sys_vertical_angle_span（°） |
| 36 | f32 | apd_temperature |
| 40 | f32 | dirty_index（保护罩脏污指数） |
| 44 | f32 | imu_temperature |
| 48 | f32 | up_optical_q |
| 52 | f32 | down_optical_q |
| 56 | f32 | apd_voltage |
| 60 | f32 | imu_angle_x_offset |
| 64 | f32 | imu_angle_y_offset |
| 68 | f32 | imu_angle_z_offset |
| 72 | f32 | b_axis_dist |
| 76 | f32 | theta_angle |
| 80 | f32 | ksi_angle |
| 84 | u16 | packet_id |
| 86 | u16 | payload_size（=120） |
| 88 | u8 | lidar_work_status |
| 89 | u8[120] | reflect_data（0..255 反射强度） |

### 19 — RET_IMU_ATTITUDE_DATA_PACKET（IMU）

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | f32[4] | quaternion (x, y, z, w) |
| 16 | f32[3] | angular_velocity（rad/s） |
| 28 | f32[3] | linear_acceleration（m/s²） |
| 40 | u16 | packet_id |

### 13 — RET_LIDAR_VERSION

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | char[13] | sys_soft_version |
| 13 | char[13] | com_soft_version |
| 26 | char[6] | sys_hard_version |
| 32 | char[6] | com_hard_version |

### 18 — RET_LIDAR_TIME_SYNC_DATA

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u32 | lidar_sync_count |
| 4 | u8 | lidar_sync_type |

### 11 — DEVICE_REQUEST_DATA（请求）

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | request_type |

实测：
- `request_type = 2` → 雷达回 **13 版本包**
- `request_type = 6` → 雷达回 **18 时间同步包**
- 其它值（0,1,3,4,5,7,8,9）在探测中无响应

版本请求帧（13 字节，seq 任意）：

```
FD 01 00 00 02 01 01 0B 00 00 02 2B 8D
```

### 14 — CONFIG_LIDAR_WORKING_MODE

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | mode：1 = NORMAL，2 = STANDBY |

### 15 — CONFIG_LED_RING_TABLE_PACKET

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u32 | led_rotation_period |
| 4 | u32 | led_zero_point_offset（ms） |
| 8 | u16 | packet_id |
| 10 | u8 | led_display_mode：2..8 |
| 11 | u8[45] | led_table（360 个 LED 按位表示） |

LED 模式：2 正向慢、3 正向快、4 反向慢、5 反向快、6 三连翻转、7 三连呼吸、8 六段呼吸。

### 12 — DEVICE_COMMAND

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u8 | cmd_type（用途未完全验证，慎用） |

---

## 4. 时间顺序与配对

实测数据流中的顺序大致为：

```
IMU(19)  DIST(16)  IMU  AUX(17)  IMU  DIST  AUX  IMU  ...
```

- `DIST` 与 `AUX` 用相同 `packet_id` 配对；实测 **AUX(N) 在前，DIST(N) 紧随其后**。
- 每个包约 5.56 ms（= `sys_rotation_period` 5560 us）。
- 每转约 16 个包（`com_rotation_period` ≈ 90.5 ms，约 11 Hz）。
- IMU 约 250 Hz，穿插在测距包之间。
- MAVLink `seq` 全局连续递增，可用于检测丢帧（实测 2430 次相邻帧 delta 全为 1）。

---

## 5. 点云坐标换算（与官方 SDK 一致）

对配对好的 `(aux, dist)`，取 120 个点：

```python
range_scale  = 0.001          # mm -> m
z_bias       = 0.0445
yaw_bias     = -38.5          # 度（安装标定）
points_num   = 120

bias_b       = aux.b_axis_dist / 1000
sin_t, cos_t = sin(aux.theta_angle), cos(aux.theta_angle)
sin_k, cos_k = sin(aux.ksi_angle),  cos(aux.ksi_angle)

pitch = aux.sys_vertical_angle_start        + k * aux.sys_vertical_angle_span        # 每个点
yaw   = aux.com_horizontal_angle_start + yaw_bias + k * aux.com_horizontal_angle_step / points_num

A = (-cos_t*sin_k + sin_t*sin(pitch)*cos_k) * range + bias_b
B =  cos(pitch)*cos_k * range

x = cos(yaw)*A - sin(yaw)*B
y = sin(yaw)*A + cos(yaw)*B
z = (sin_t*sin_k + cos_t*sin(pitch)*cos_k) * range + z_bias
intensity = aux.reflect_data[k]
```

其中 `k = 0..119`，`range = dist.point_data[k] * 0.001`，`range == 0` 的点丢弃。

> 该公式与 `unilidar_sdk/include/parse_range_auxiliary_data_to_cloud.h` 完全一致。
> `yaw_bias = -38.5°` 为 SDK v1.0.5 起的默认标定值，仅影响点云整体朝向。

---

## 6. 验证记录

- 串口原始抓包在 2 Mbaud 下逐帧 CRC 全部通过；其它波特率 0 帧通过
- 固件读取结果：`SYS 1.2.3+240716 / COM 1.0.3+231017`，硬件 `SYS 0.4.1 / COM 0.2.6`
- IMU 静止时四元数模长 ≈ 0.998，加速度模长 ≈ 9.7–9.8 m/s²
- 点云重建：4 s 抓包解出 710 帧、约 6.8 万有效点，能清楚看到约 6.2 m 的平整天花板、
  立柱和地面小物体，几何关系成立

---

## 7. 适用范围

本文件描述 **宇树 Unitree L2** 激光雷达经 USB CDC（串口）链路的通信协议，
已在实测设备上验证通过。以太网（UDP）链路的消息集与此不同，不要混用。

---

## 8. 附：最小解析流程

1. 以 2 Mbaud 打开串口
2. 累积字节，寻找 `0xFD`
3. 读 `len`，等待 `10 + len + 2` 字节
4. 校验 CRC（含 `crc_extra`）
5. 按 `msgid` 解析，payload 补零到标准长度
6. `AUX(N)` 与随后的 `DIST(N)` 配对，按第 5 节公式生成点云
