# C 板 USB 虚拟串口（视觉通信）协议说明

> 适用工程：`Qust_version`（STM32F407IGH6 / C 板）
> 对接上位机：`sp_vision_25`（`io/gimbal/gimbal.hpp`，`standard_mpc` 等程序）
> 协议版本：43 B 上行 / 29 B 下行，CRC-16/MCRF4XX

---

## 1. 概述

C 板通过 MicroUSB（USB_OTG_FS，Device 模式，CDC-ACM 类）枚举成 Linux 下的 `/dev/ttyACM*`，
再经 udev 固定为 `/dev/gimbal`，与视觉小电脑交换定长二进制帧：

```
        ┌──────────────┐   USB CDC (43 B/帧, 1 kHz)   ┌──────────────────┐
        │   C 板 F407   │ ───────────────────────────► │  视觉 NUC (Linux) │
        │  电控/姿态     │ ◄─────────────────────────── │  io::Gimbal       │
        └──────────────┘   USB CDC (29 B/帧, 100 Hz)  └──────────────────┘
              上行：姿态四元数 / yaw·pitch / 弹速 / 发弹数
              下行：目标 yaw·pitch + 角速度 + 角加速度（前馈）+ 控制/开火模式
```

设计要点：

- **定长帧**：不依赖"长度字段"，靠 `'S''P'` 帧头 + 固定长度 + CRC 同步；
- **前馈量下发**：视觉不只给目标角，还给角速度和角加速度，供电控做计算力矩控制；
- **单向职责**：视觉只给参考量，闭环、限幅、保护仍在电控侧。

---

## 2. USB 配置（CubeMX）

| 项目 | 配置值 |
| --- | --- |
| Connectivity → USB_OTG_FS | Mode = **Device_Only**（PA11 = DM，PA12 = DP） |
| Middleware → USB_DEVICE | Class For FS IP = **Communication Device Class (Virtual Port Com)** |
| Clock | HSE 12 MHz，PLLM=6 / PLLN=168 / PLLP=2 / **PLLQ=7** → USB 48 MHz |
| NVIC | USB OTG FS 全局中断使能 |
| 生成目录 | `USB_DEVICE/App/{usb_device,usbd_cdc_if,usbd_desc}.c`、`USB_DEVICE/Target/usbd_conf.c`、`Middlewares/ST/STM32_USB_Device_Library/` |

### ⚠️ 必须显式初始化 USB

CubeMX 默认把 `MX_USB_DEVICE_Init()` 生成在 **`__weak led_task()`** 里；
本工程 `applications/led_task.cpp` 覆盖了该弱函数，导致那句调用永远不会执行。

现象：**程序正常运行，但插上 USB 电脑毫无反应（`dmesg` 无 `cdc_acm`）**。

修复：`Core/Src/main.c` 的 `USER CODE BEGIN 2` 中显式调用一次（调度器启动前）：

```c
/* USER CODE BEGIN 2 */
MX_USB_DEVICE_Init();
/* USER CODE END 2 */
```

---

## 3. 帧格式

### 3.1 通用约定

| 项目 | 约定 |
| --- | --- |
| 字节序 | 小端（little-endian），float 为 IEEE-754 单精度（4 B），与 Cortex-M4 原生一致 |
| 对齐 | 结构体全部 `__attribute__((packed))`，`sizeof` 即线上字节数 |
| 帧头 | `0x53 0x50`（ASCII `'S' 'P'`） |
| 校验 | CRC-16/MCRF4XX，覆盖 **末尾 2 字节之前** 的全部字节，结果低字节在前 |
| 长度 | 上行 43 B，下行 29 B（编译期 `static_assert` 卡死） |
| 频率 | 上行 1 kHz（必须 ≥200 Hz），下行 100 Hz（视觉 `plan_thread`） |

### 3.2 板 → 机：`GimbalToVision`（43 B）

| 偏移 | 长度 | 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- | --- | --- |
| 0 | 2 | `head[2]` | uint8 | — | `'S' 'P'` |
| 2 | 1 | `mode` | uint8 | — | 操作手档位：0 空闲 / 1 自瞄 / 2 小符 / 3 大符 |
| 3 | 16 | `q[4]` | float×4 | — | 姿态四元数，**wxyz 顺序**（`q[0]=w`） |
| 19 | 4 | `yaw` | float | rad | 云台/车体 yaw |
| 23 | 4 | `yaw_vel` | float | rad/s | yaw 角速度 |
| 27 | 4 | `pitch` | float | rad | pitch |
| 31 | 4 | `pitch_vel` | float | rad/s | pitch 角速度 |
| 35 | 4 | `bullet_speed` | float | m/s | 弹速（视觉解弹道用） |
| 39 | 2 | `bullet_count` | uint16 | 发 | 累计发弹数（视觉判定"枪已响"） |
| 41 | 2 | `crc16` | uint16 | — | 覆盖 0~40 字节 |

> 单位全部是**弧度**，不是度。视觉侧直接把这些量当 SI 量用，填成度会导致云台转 57 倍。

### 3.3 机 → 板：`VisionToGimbal`（29 B）

| 偏移 | 长度 | 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- | --- | --- |
| 0 | 2 | `head[2]` | uint8 | — | `'S' 'P'` |
| 2 | 1 | `mode` | uint8 | — | 0 不控制 / 1 控云台不开火 / 2 控云台且开火 |
| 3 | 4 | `yaw` | float | rad | 目标绝对角度 |
| 7 | 4 | `yaw_vel` | float | rad/s | 前馈角速度 |
| 11 | 4 | `yaw_acc` | float | rad/s² | 前馈角加速度 |
| 15 | 4 | `pitch` | float | rad | 目标绝对角度 |
| 19 | 4 | `pitch_vel` | float | rad/s | 前馈角速度 |
| 23 | 4 | `pitch_acc` | float | rad/s² | 前馈角加速度 |
| 27 | 2 | `crc16` | uint16 | — | 覆盖 0~26 字节 |

### 3.4 mode 语义

| 方向 | 值 | 含义 |
| --- | --- | --- |
| 上行 | 0 / 1 / 2 / 3 | 空闲 / 自瞄 / 小符 / 大符（操作手拨杆决定） |
| 下行 | 0 | 不控制：电控应保持/回中，忽略角度 |
| 下行 | 1 | 控制云台，**禁止开火** |
| 下行 | 2 | 控制云台，**允许开火** |

视觉侧 `fire` 由 `mode == 2` 解出；`mode == 0` 时 `control = false`，视为无目标。

### 3.5 CRC-16 计算

| 参数 | 值 |
| --- | --- |
| 名称 | CRC-16/MCRF4XX（= CRC-16/CCITT 反射式，裁判系统同款） |
| 初值 | `0xFFFF` |
| 多项式 | `0x1021`（反射后 `0x8408`） |
| 输入/输出反射 | 是 |
| 最终异或 | `0x0000` |
| 字节序 | 校验值**低字节在前** |

位运算实现（板端 `vision.cpp` 使用，省一张 512 B 表）：

```c
static uint16_t vision_crc16(const uint8_t * data, uint32_t len)
{
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0x8408U) : (uint16_t)(crc >> 1);
  }
  return crc;
}
```

**参考校验值**（可用来自测实现是否正确）：

| 帧 | 字节数 | CRC |
| --- | --- | --- |
| 下行样例（见附录 B） | 29 | `0x3387` |
| 上行样例（见附录 B） | 43 | `0x99A8` |

---

## 4. 收发策略

### 4.1 接收（USB 中断上下文）

`CDC_Receive_FS` → `vision_callback()` → `sp::Vision::update()`：

1. 追加到 128 B 缓冲（单次回调 ≤64 B，溢出则整段丢弃重建）；
2. 循环切帧：
   - 不是 `'S''P'` → **丢 1 字节**滑窗；
   - 帧头对但不足 29 B → 保留，等下一批数据；
   - CRC 错 → **丢 1 字节**滑窗（帧头可能是数据里巧合撞上的，整帧跳会漏掉真帧）；
   - CRC 对 → 拷贝出结构体、更新状态、游标前进 29 B；
3. 中断里只做拷贝与校验，不做控制、不打印、不延时。

### 4.2 发送（任务上下文）

`vision_task` 每 1 ms 调一次 `send()`：

1. 填 `GimbalToVision` 字段 → 算 CRC → 调 `CDC_Transmit_FS`；
2. 返回 `USBD_BUSY`（上一帧主机还没取走）时**直接放弃本帧**并计数，绝不忙等；
3. 视觉掉线（100 ms 无合法帧）时，上行 `mode` 强制填 0。

### 4.3 频率要求（重要）

视觉侧 `io::Gimbal` 的读线程是**忙轮询 + 非阻塞读**：连续 5000 次读失败（约几毫秒）就会触发
`reconnect()`，关闭串口、`sleep 1 s` 再重开，期间主循环卡死。

因此：

- **上行必须 ≥200 Hz，建议 1 kHz**（本实现 `osDelay(1)`）；
- 不要"有数据才发"，要**周期无条件发**，即使姿态没变化。

### 4.4 数据竞争

- `update()` 在中断写、`get_control()` 在任务读：读侧用 `PRIMASK` 关中断做一次 9 个量的快照，保证同帧；
- `send()` 只由 `vision_task` 一个线程调用，`tx_data_` 无需加锁。

---

## 5. 代码结构

| 文件 | 职责 |
| --- | --- |
| `sp_middleware/io/vision/vision.hpp` | 两个协议结构体、`static_assert`、`sp::Vision` 类声明 |
| `sp_middleware/io/vision/vision.cpp` | CRC、切帧解析、打包发送、掉线判定、调试计数 |
| `applications/vision_task.cpp` | 1 kHz 上报；档位映射；`vision_cmd` 指令快照；`vision_callback` 定义 |
| `USB_DEVICE/App/usbd_cdc_if.c` | `CDC_Receive_FS` 内调用 `vision_callback(Buf, *Len)`（USER CODE 区） |
| `Core/Src/main.c` | `MX_USB_DEVICE_Init()` 显式调用（USER CODE 区） |
| `Core/Src/freertos.c` | 注册 `VISION` 任务（Normal，栈 256 字） |

### 主要接口

```cpp
namespace sp {
struct Control {            // 给云台控制任务用的一帧快照
  bool valid;               // 视觉在线且 mode != 0
  bool control; bool fire;
  float yaw, yaw_vel, yaw_acc;
  float pitch, pitch_vel, pitch_acc;
  uint32_t stamp_ms;
};

class Vision {
 public:
  void update(uint8_t * buf, uint32_t len);        // 中断里调用
  uint8_t send(uint8_t mode, const float q[4],     // 任务里调用，43 B
               float yaw, float yaw_vel,
               float pitch, float pitch_vel,
               float bullet_speed, uint16_t bullet_count);
  bool get_control(Control & out);                 // 取快照，返回 valid
  bool alive(uint32_t now_ms) const;               // 100 ms 内收到过合法帧

  volatile uint32_t rx_frame_count, rx_crc_error,
                    rx_resync_bytes, tx_drop_count;  // 调试计数（调试器查看）
};
}
```

### 使用方式（`vision_task`）

```cpp
uint8_t mode = mode_from_remote(HAL_GetTick());   // 右拨杆上 = 1，其余 = 0
if (!vis.alive(now_ms)) mode = 0;                 // 掉线主动回空闲
vis.send(mode, imu.q, imu.yaw, imu.vyaw,
         imu.pitch, imu.vpitch, BULLET_SPEED, bullet_count);
vis.get_control(vision_cmd);                      // 供云台任务读取
osDelay(1);                                       // 1 kHz
```

---

## 6. 与视觉侧字段对应

| 板端字段 | 视觉侧 | 备注 |
| --- | --- | --- |
| `mode` | `GimbalMode` | 0 IDLE / 1 AUTO_AIM / 2 SMALL_BUFF / 3 BIG_BUFF |
| `q[4]` | `rx_data_.q` | 视觉 `gimbal.q(t)` 用它做图像-四元数对齐，**必须 wxyz** |
| `yaw / pitch` | `GimbalState` | 用于显示与开火判定参考 |
| `bullet_speed` | `Planner::plan` | 弹道解算输入，偏差直接导致打高/打低 |
| `bullet_count` | `GimbalState` | 视觉靠它统计"已开火"，`gimbal_test -f` 也用它 |
| 下行 `yaw/pitch` | `Plan` | 视觉的 MPC 输出 |
| 下行 `yaw_acc/pitch_acc` | `Plan` | 这是相对旧 CAN 方案新增的前馈量 |

> 视觉侧结构体在同一仓库的 `sp_vision_25/io/gimbal/gimbal.hpp`，两边任一字段增删都必须同步改，
> 且 `static_assert` 的长度也要一起改。

---

## 7. 集成步骤（CubeMX 重新生成后）

本工程的所有手写接入点都在 **USER CODE 区**，CubeMX 重新生成不会覆盖：

1. `USB_DEVICE/App/usbd_cdc_if.c` → `CDC_Receive_FS` 的 `USER CODE BEGIN 6`：
   保留原有两行重新挂接收，然后加 `vision_callback(Buf, *Len);`
2. `Core/Src/main.c` → `USER CODE BEGIN 2`：`MX_USB_DEVICE_Init();`
3. `Core/Src/freertos.c` → `USER CODE BEGIN RTOS_THREADS`：`osThreadDef/osThreadCreate(VISION)`
4. `CMakeLists.txt` → `target_sources` 中加入 `applications/vision_task.cpp`、`sp_middleware/io/vision/vision.cpp`

编译（本机实测命令）：

```powershell
$env:PATH="D:\xpack-arm-none-eabi-gcc-15.2.1-1.1-win32-x64\xpack-arm-none-eabi-gcc-15.2.1-1.1\bin;D:\mingw\w64devkit\bin;$env:PATH"
cd D:\RoboMaster\Qust_version
cmake -B build/vision_verify -G Ninja "-DCMAKE_BUILD_TYPE=Debug" `
      "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake"
cmake --build build/vision_verify
```

实测资源占用（Debug）：

```
RAM:   27864 B / 128 KB (21.26%)
FLASH: 79800 B / 1 MB   (7.61%)
```

---

## 8. 验证

### 8.1 设备枚举

```bash
dmesg | tail -5          # 期望: cdc_acm 1-1:1.0: ttyACM0: USB ACM device
lsusb                    # 期望: 0483:5740 STMicroelectronics STM32 Virtual ComPort
```

### 8.2 只收不发（验证上行 43 B）

```bash
python3 - <<'EOF'
import serial, struct
s = serial.Serial('/dev/ttyACM0', 921600, timeout=0.2)
while True:
    b = s.read(43)
    if len(b) == 43 and b[:2] == b'SP':
        mode, = b[2:3]
        q = struct.unpack('<4f', b[3:19])
        print("mode", mode, "q", ["%.3f" % x for x in q])
        break
EOF
```

期望：`mode` 随拨杆变化，转动板子时 `q` 实时变化且模长 ≈1。

### 8.3 只发不收（验证下行 29 B）

```bash
python3 - <<'EOF'
import serial, struct
def crc16(d):
    c = 0xFFFF
    for x in d:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
    return c
body = b'SP' + bytes([1]) + struct.pack('<6f', 0.5, 0, 0, 0.1, 0, 0)
frame = body + struct.pack('<H', crc16(body))
assert len(frame) == 29
serial.Serial('/dev/ttyACM0', 921600).write(frame)
EOF
```

期望：云台（或调试变量 `vision_cmd`）里 `yaw=0.5`、`pitch=0.1`、`control=true`、`fire=false`。

### 8.4 端到端

```bash
cd ~/Desktop/sp_vision_25
./build/gimbal_test configs/standard3.yaml
```

期望：能打印四元数、`Gimbal mode changed: ...`、发弹计数变化。

---

## 9. 故障排查

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 插上 USB 完全没反应 | `MX_USB_DEVICE_Init()` 没执行 / 时钟不是 48 MHz / 纯充电线 | 检查 `main.c` 调用；确认 PLLQ=7；换数据线 |
| 有 `lsusb` 无 `/dev/ttyACM*` | CDC 类没选、描述符没生成 | CubeMX 重选 CDC 并生成 |
| 有串口但视觉卡在 `[Gimbal] read_thread started.` | 帧长/字段不匹配 | 对照第 3 节，`static_assert` 必须过 |
| 一直 CRC 失败 | 忘了 `packed`（插入 padding）或 CRC 覆盖范围错 | `sizeof` 应为 43；CRC 只算末尾 2 字节之前 |
| 云台转 57 倍 / 反向 | 用了度而不是弧度，或 yaw 符号约定不同 | 板端统一弧度；符号在控制层取反，不要在协议层改 |
| 跑一会卡 1 秒 | 上行频率过低，视觉侧重连 | 提高到 500 Hz~1 kHz |
| `tx_drop_count` 持续增长 | 主机没取包 / 发送频率高于 USB 帧率 | 检查主机是否打开串口；1 kHz 时偶发丢帧可接受 |
| 模式不变化 | 拨杆映射不符 | 改 `vision_task.cpp` 的 `mode_from_remote()` |

---

## 10. 已知限制与 TODO

- **多一个消费端**：目前 `vision_cmd` 只被存放，尚未接入云台电机控制（需要按云台电机数量/CAN ID/控制律补一个任务）；
- `VISION_BULLET_SPEED = 24.0f` 为占位值，需按实车弹速标定；
- `g_bullet_count` 固定 0，需接发弹检测（红外/裁判系统 `shoot_data`）；
- 未实现超电数据上报：43 B 帧里没有超电字段；若需要，另开 `'S','C'` 大包，**不要**往 43 B 里加字段（会与视觉侧长度断言冲突）；
- 未做 USB 断线检测重连：STM32 侧由 USB 复位自动重初始化 CDC，无需软件重连；若出现"掉线后不再发送"，检查 `TxState` 是否卡在 1（主机异常断开导致），可在 `CDC_TransmitCplt_FS`/`CDC_Init_FS` 里复位。

---

## 附录 A：结构体定义

```cpp
namespace sp {

struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;
  float q[4];            // wxyz
  float yaw, yaw_vel;
  float pitch, pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  uint16_t crc16;
};
static_assert(sizeof(GimbalToVision) == 43, "must be 43 bytes");

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;
  float yaw, yaw_vel, yaw_acc;
  float pitch, pitch_vel, pitch_acc;
  uint16_t crc16;
};
static_assert(sizeof(VisionToGimbal) == 29, "must be 29 bytes");

}
```

## 附录 B：参考帧（可用于单元测试）

**下行 29 B**（mode=2, yaw=1.0, yaw_vel=2.0, yaw_acc=3.0, pitch=0.5, pitch_vel=0.6, pitch_acc=0.7）：

```
53 50 02 00 00 80 3F 00 00 00 40 00 00 40 40
00 00 00 3F 9A 99 19 3F 33 33 33 3F 87 33
CRC = 0x3387（低字节 0x87 在偏移 27，高字节 0x33 在偏移 28）
```

**上行 43 B**（mode=1, q=(1,0,0,0), yaw=0.3, yaw_vel=1.2, pitch=-0.1, pitch_vel=0.4, bullet_speed=24, bullet_count=7）：

```
53 50 01 00 00 80 3F 00 00 00 00 00 00 00 00
00 00 00 00 9A 99 99 3E 9A 99 99 3F CD CC CC BD
CD CC CC 3E 00 00 C0 41 07 00 A8 99
CRC = 0x99A8（低字节 0xA8 在偏移 41，高字节 0x99 在偏移 42）
```

两组 CRC 已与视觉侧 `sp_vision_25/tools/crc.cpp` 的查表实现交叉验证一致。
