# 制导飞镖舵机子系统 v0.2 · 调试报告

> 面向后续 AI / 队友：记录需求、架构、协议、**每个坑的根因与修法**、参数与构建流程。
> 项目根目录：`D:\workp\control-2026`（RoboMaster 团队 C 板固件框架，STM32F407 + FreeRTOS）
> 归档版：`UserApp/robot/archive/dart_servo_v0.2_20260923`（2026-09-23 实测可用）
> 工作版：`UserApp/robot/dart_servo_v0.2`

---

## 0. 背景与目标

制导飞镖本体的 **4 路 X 型舵面**（PTK7350 舵机）控制子系统。

目标：
1. 4 路舵机 50Hz PWM 输出、单路手动测试、标定（方向/零点/比例）；
2. 状态机（待机/手动/混控/自检）；
3. ESP32 WiFi 网页调参 + 控制；
4. 所有人为设定**掉电不丢**；
5. 为制导（比例导引）提供归一化控制量接口。

> 约束（`agent_must_read.txt`）：**不许改 Bsp/Modules/Hardware 底层库**，只能新建 app；大改新建版本目录。

---

## 1. 版本演进

| 版本 | 说明 |
|---|---|
| `dart_servo_v0.1` | 初版，单 Bank 保存，逻辑角 ±90°（把 270° 级舵机当 180° 用），前端有解析错位等 bug；已废弃（目录丢失） |
| `dart_servo_v0.2` | **完全重写**：行程修正 279°、双 Bank 掉电保存、前端重写、两步标定、协议精简 |

---

## 2. 系统架构

### 2.1 硬件链路

```
[手机]--WiFi(AP DART_SERVO)-->[ESP32-S3]--UART1(115200)-->[C板STM32F407]--PWM-->4×PTK7350
                                                              TIM1 CH1~4
```
- 手机连 `DART_SERVO` / `12345678`，浏览器开 `http://192.168.4.1`
- ESP32 UART1：TX=GPIO17, RX=GPIO18；C 板 USART6：PG14(TX)/PG9(RX)，交叉 + 共地
- C 板 PWM：PWM1=PE9(TIM1_CH1) 右上 / PWM2=PE11(CH2) 左上 / PWM3=PE13(CH3) 左下 / PWM4=PE14(CH4) 右下

### 2.2 C 端固件模块（app 目录内）

| 文件 | 作用 |
|---|---|
| `robot_config.h` | 全部参数：行程/脉宽/限幅/矩阵/超时/Flash 地址 |
| `dart_pwm.[ch]` | 4 路 50Hz PWM，只管写比较寄存器 |
| `dart_axis.[ch]` | 4 路舵面：标定(方向/零点/增益) + 状态机 + 速率限幅 |
| `dart_cfg.[ch]` | 掉电保存（**双 Bank 交替 + 序号 + CRC + 写后校验**） |
| `dart_proto.[ch]` | 与 ESP32 的 ASCII 串口协议 + 遥测 + 链路超时 |
| `robot.c` | 入口：Init + 每周期 Task |

底层 PWM/Flash 全部走 BSP（`bsp_usart`/`bsp_flash`/`bsp_dwt`），不碰 Modules。

### 2.3 ESP32 模块（MicroPython，`esp32/`）

| 文件 | 作用 |
|---|---|
| `bus.py` | `Bus`：UART 收发、`T` 遥测解析、SAVED/SAVEERR 标志 |
| `servo.py` | `Servo` 基类 + `PTK7350/PTK7465`（发 `ANG/TRIM/SCALE/DIR/ZERO/RSTCAL`） |
| `task.py` | 状态机对象（`MODE`/`MIX`/`SAVE`/心跳） |
| `web.py` | HTTP + 结构化 API（`/api/servo|task`、`/state`、`/api/status`） |
| `main.py` | 组装：AP + Bus + 对象 + HTTP；200ms 心跳 |
| `www/` | 网页：`index.html` `style.css` `js/{app,servo,task}.js` |

### 2.4 浏览器网页（六个界面）

- 界面1 总线/总览：模式、链路、保存状态、四路角度、自检/保存/Ping
- 界面2~5 单路舵面：实际角、脉宽、命令滑块(±139.5°)、转到、同步、回中位、调零、方向、两步标定(记0°/记90°)、trim/scale
- 界面6 总控：状态机切换、混控 p/y/r 百分比滑块、保存

---

## 3. 通信协议（ESP32 ↔ C 板，UART，115200，ASCII，`\n` 结尾）

### ESP32 → C
```
PING                         -> PONG
SAVE                         -> SAVED / SAVEERR
H                            心跳(刷新超时)
MODE,<m>                     0待机 1手动 2混控 3自检
ANG,<ch>,<deg10>             手动角 (0.1°)
MIX,<p1000>,<y1000>,<r1000>  混控指令 (×1000)
TRIM,<ch>,<deg10>            零点 (0.1°)
SCALE,<ch>,<scale1000>       增益 (×1000)
DIR,<ch>                     方向取反
ZERO,<ch>                    当前位置记为 0°
RSTCAL,<ch>                  清除标定
```

### C → ESP32（100ms）
```
T,<mode>,<link>,
  <a0>,<p0>,<d0>,<t0>,<s0>, ... x4,
  <mp>,<my>,<mr>
a=角度x10  p=脉宽us  d=方向  t=trim x10  s=scale x1000  m=mix x1000
```
共 25 个数：2 + 4×5 + 3。

---

## 4. 关键 Bug 与根因（核心章节）

### 坑 #1：舵机行程不是 180°（是 279°）
- **现象**：命令 90° 实际转不到 90°，末端还会"往外翻"。
- **根因**：PTK7350MG-D 是 **270° 级舵机**，500~2500us 对应约 **279°** 总行程；v0.1 却按 180° 映射（±90°），且安全脉宽放到 400~2600us 会越过行程。
- **修**：`DART_TRAVEL_HALF_DEG = 139.5`（279°/2），安全脉宽严格锁 `500~2500us`。
- **资料**：`C:\Users\zhong\Documents\ptk7350` 官方驱动注释即 `500-2500us, 0-270deg`。

### 坑 #2：调零时方向反转会把舵面翻转
- **根因**：`ZeroHere` 把**镜像后**的 applied 当 trim 存，`SetAngle` 又镜像一次。
- **修**：反推 trim 时先还原方向：`trim = reversed ? -applied : applied`。

### 坑 #3：调零后舵面跳变
- **根因**：调零只归零逻辑角，没同步速率限幅器状态 `out`，下一周期从旧角插值 → 跳。
- **修**：调零同时 `out = 0`。

### 坑 #4：网页遥测字段错位（UI 全乱）
- **根因**：C 端每路发 **6** 项（`idx,angle10,pulse10,dir,trim10,scale1000`），网页只解析 **5** 项。
- **修**：v0.2 前端集中到 `parseFrame()`，带长度校验，与帧一一对应。

### 坑 #5：制导指令双重缩放（永远满舵）
- **根因**：旧前端发 -100~100，后端 `set_guide_cmd` 内部又 ×100 → C 端 `SG` 恒 ≥1 → 钳到 ±1 满偏。
- **修**：接口约定前端发 **-1~1**（百分比显示 -100%~100%）。

### 坑 #6：滑块松手回弹
- **根因**：遥测每帧回写滑块；命令并发被丢。
- **修**：① 命令总线**串行发送**（一次一条在途，匹配 ESP32 单连接服务器）；② 滑块是"命令"，遥测只更新"实际角"，首帧同步一次 + "同步当前角"按钮；③ 拖拽中周期性重申手动状态。

### 坑 #7（最重要）：掉电保存 —— 单 Bank 被写坏导致"标定还原"
- **现象**：改标定、保存，断电重启后回到默认。
- **根因**：单 Bank（sector 10）方案下，**一次擦写被打断**（掉电 / 复位 / 调试器在擦写瞬间暂停 CPU），唯一的扇区就损坏。实测抓到：
  | Bank | seq | 存储 CRC | 应有 CRC | 结果 |
  |---|---|---|---|---|
  | A | 0 | `0x22000000` | `0x08289445` | **损坏** |
  | B | 1 | `0xA6344165` | `0xA6344165` | 有效 |
  上电 CRC 校验失败 → 回默认。
- **修**：照发射架方案做 **双 Bank 交替**（详见第 6 节），并加**写后回读校验 + 失败回退**。

### 坑 #8：PING 在中断里阻塞发送
- **根因**：串口 RX 回调（中断上下文）里直接 `HAL_UART_Transmit` 阻塞发送 PONG，极端情况卡死 ISR。
- **修**：中断里只置标志，任务上下文回复。

---

## 5. 标定设计

模型：
```
施加角 applied = 命令角 × scale + trim
脉宽   pulse   = 1500 + applied × (1000 / 139.5)     // ±1000us = ±139.5°
方向 reversed  : applied = -applied
```

- `trim`：零点偏移；`scale`：比例增益；`reversed`：方向。
- **两步标定**（网页"标定"里）：
  1. 手动拖到物理 **0° 标记** → `记 0°`（写 trim，把当前位置记为逻辑 0°）
  2. 拖到物理 **90° 标记** → `记 90°`：读滑块值 L，`scale = scale旧 × L / 90`，再把滑块归到 90°（舵面停在 90°）
- 原理：记零后 `物理 = k·scale·L`；在 L=L₉₀ 时物理=90°，故 `k·scale = 90/L₉₀`；要"命令=物理"需 `k·scale新=1` → `scale新 = scale旧·L₉₀/90`。

---

## 6. 掉电保存设计（双 Bank）

- 两个 Flash 扇区各存一份完整配置：
  - Bank A = `0x080C0000`（sector 10）
  - Bank B = `0x080E0000`（sector 11）
- 结构：`magic(0xD2A70002) + seq + crc(FNV-1a) + data(方向/trim/scale)`
- 保存：写到**非当前**的 bank，`seq+1`；**写后回读校验**，通过才切 active；失败**回退写另一个**。
- 上电：读两份，校验通过且 `seq` 大者胜出。
- 好处：**写一半掉电，另一份仍完好**；单扇区坏也不丢配置。
- 时机：标定改动去抖 **800ms** 自动保存；"保存标定"按钮立即强制保存。
- 注意：**sector 11 实测可写**（v0.1 注释"不生效"是错的）；128KB 扇区擦除约 **1s**，期间关中断。

---

## 7. 与比例导引（PNG）对接

百分比 = **归一化控制量**，就是 PNG 的天然接口。`dart_fc` 里已是这么做的：

```
dart_guidance.c: a = N · Vc · λ̇   (m/s²), 钳到 ±max_g·g
dart_control.c : yaw_cmd = bx / max_g;  pitch_cmd = by / max_g;   // -> -1..1
```
链路：
```
视线角速率 λ̇ → PNG → 加速度 a → ÷(max_g·g) → 归一化指令(-1..1 = 百分比)
            → 混控矩阵 → 舵面角(百分比 × 35°) → 舵机
```
- 100% = 满控制权限 = 混控 `DART_MIX_MAX_DEG = 35°` 舵面偏角（**不是**舵机的 139.5°）。
- 真实制导**不会一直 100%**：那是饱和上限，常态 10~40%；一直满舵会 bang-bang、过冲、振荡，反而打不准。
- 两个"角度"别混：**手动/标定**是舵机物理角 ±139.5°；**混控/制导**是归一化指令 ±100%。

---

## 8. 构建与烧录

### C 板编译（注意 `-DROBOT_TYPE` 必须加引号，否则 PowerShell 会吃掉 `.2`）
```powershell
cmake -G Ninja -S . -B cmake-build-dart_servo_v0.2 -DCMAKE_BUILD_TYPE=Debug -DBOARD_TYPE=GIMBAL_BOARD "-DROBOT_TYPE=dart_servo_v0.2"
cmake --build cmake-build-dart_servo_v0.2
```

### J-Link 烧录
```
si SWD
speed 4000
device STM32F407IG
connect
r
h
loadfile D:/workp/control-2026/cmake-build-dart_servo_v0.2/control-2026.hex
r
g
qc
```
`JLink.exe -NoGui 1 -CommanderScript flash.jlink`（克隆探针别联网更新固件；掉线拔插 USB 可复活）

### ESP32 上传（PC 上位机先关，释放 COM8）
```powershell
python -m mpremote connect COM8 fs cp esp32\bus.py   :bus.py
python -m mpremote connect COM8 fs cp esp32\servo.py :servo.py
python -m mpremote connect COM8 fs cp esp32\task.py  :task.py
python -m mpremote connect COM8 fs cp esp32\web.py   :web.py
python -m mpremote connect COM8 fs cp esp32\main.py  :main.py
python -m mpremote connect COM8 fs cp esp32\www\index.html :www/index.html
python -m mpremote connect COM8 fs cp esp32\www\style.css  :www/style.css
python -m mpremote connect COM8 fs cp esp32\www\js\app.js  :www/js/app.js
python -m mpremote connect COM8 fs cp esp32\www\js\servo.js :www/js/servo.js
python -m mpremote connect COM8 fs cp esp32\www\js\task.js  :www/js/task.js
python -m mpremote connect COM8 reset
```

### 快速自检
- 访问 `http://192.168.4.1/state`：应以 **`T,`** 开头（v0.1 是 `F,`）
- 页面左上角显示 **v0.2**

---

## 9. 已知限制 / TODO

- **只保存标定**（trim/scale/方向），**不保存手动角/模式**（上电固定回待机 0°）；如需保存手动角再说。
- Flash 擦除约 1s 且关中断：改标定后等约 2s 再断电（界面1"保存"栏显示"成功"即可）。
- **混控矩阵符号**（`DART_MIX_*`）沿用 `dart_fc` 约定，仍需实物确认舵面偏转方向。
- 舵机供电：PTK7350 堵转电流较大，建议外部 5~6V + 共地。
- 制导飞镖本体（IMU/姿态/导引）在 `dart_fc`，本工程仅舵机执行层。
