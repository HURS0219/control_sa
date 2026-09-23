# 制导飞镖发射架 · 网页控制 + 工业相机自瞄 —— 调试报告

> 面向后续 AI / 队友：本文记录需求、架构、协议、**每个坑的根因与修法**、以及当前参数。
> 项目根目录：`D:\workp\control-2026`（RoboMaster 团队 C 板固件框架，STM32F407 + FreeRTOS）
> 最新工作版：`UserApp/robot/dart_launcher_web_v5_HIK`（自瞄测试版）

---

## 0. 背景与目标

飞镖发射架有 4 个电机 + 1 个舵机：

| 角色 | 型号 | CAN ID | 反馈 ID | 用途 |
|---|---|---|---|---|
| 拉簧 A | M3508 | 2 | 0x202 | 同步带拉簧(主) |
| 拉簧 B | M3508 | 3 | 0x203 | 同步带拉簧(从) |
| 扳机丝杆 | M3508 | 4 | 0x204 | 丝杆带动发射扳机 |
| Yaw | M2006 | 1 | 0x201 | 丝杆改变发射架偏转角 |
| 扳机舵机 | PWM | TIM1_CH1 | — | 扣扳机/锁发射台 |

目标：
1. 手机网页（6 界面）通过 ESP32 WiFi 调参 + 控制 4 电机 + 舵机；
2. 自动化发射时序状态机；
3. 用**工业相机**做自瞄（PC 处理图像 → 发绿光坐标 → C 板驱动 Yaw）；
4. 所有人为设定**掉电不丢**。

> 关键约束（`agent_must_read.txt`）：**不许改 Bsp/Modules/Hardware 底层库**，只能新建 app；
> 大改要新建版本目录。`agent_must_read_dart.txt` 是需求与附录。

---

## 1. 版本演进（重要）

| 版本 | 电机层实现 | 说明 |
|---|---|---|
| `dart_launcher_web` (v1) | 自研 `simple_motor`（裸 CAN） | 第一版，能跑；存在 URL 编码等 bug |
| `dart_launcher_web_v2` | 自研 `dart_motor` | 加方向切换/堵转/三模式(速度/角度/圈数) |
| `dart_launcher_web_v3` | 复用 `DJImotor` 库 | 按要求改用 `DJIMotorInit/SetPIDRef/OuterLoop/Stop` |
| `dart_launcher_web_v4` | **直接暴露 `DJIMotorInstance*`** | 去掉自定义电机类，队友可直接看实例 |
| `dart_launcher_web_v5` | 同 v4 | 新增**双 Bank+校验**掉电保存 |
| `dart_launcher_web_v5_HIK` | 同 v5 | 工业相机自瞄分支（PC 串口发坐标） |

归档在 `UserApp/robot/archive/`：`v4_20260920`、`v5_20260920`、`v5_HIK_aim_20260920`。

---

## 2. 系统架构

### 2.1 硬件链路

```
[工业相机]--USB-->[PC(OpenCV识别绿光)]--USB(COM8)-->[ESP32-S3]--UART1-->[C板STM32F407]--CAN1-->4电机
                                                          ^
[手机]--WiFi(AP DART_CTRL)---------------------------------+   (HTTP 网页)
```

- 手机连 ESP32 的 AP（`DART_CTRL`/`12345678`），浏览器开 `http://192.168.4.1`。
- PC **不连 WiFi**，走 **COM8（ESP32 的 USB 串口）** 发坐标，避免占用 ESP32 的 HTTP 带宽（否则手机按钮会卡）。
- ESP32 做 **USB↔UART1 双向桥**：PC 发指令→C 板；C 板遥测→（可选）回传 PC。

### 2.2 C 板固件模块（app 目录内）

| 文件 | 作用 |
|---|---|
| `robot_config.h` | 全部参数：电机配置宏、减速比、方向、Yaw/视觉、舵机、串口、任务 |
| `dart_motor.[ch]` | 四电机 = **原始 `DJIMotorInstance`** + 少量应用状态(`MotorAxis`)，直接调库 |
| `dart_servo.[ch]` | 扳机舵机（PWM，角度制） |
| `dart_vision.[ch]` | 绿光坐标(注入) + 超时判定 |
| `dart_fsm.[ch]` | 发射架状态机 + 自瞄环 + 自动化时序 |
| `dart_link.[ch]` | 与 ESP32 的 ASCII 协议解析 + 遥测 |
| `dart_store.[ch]` | 掉电保存（双 Bank + CRC） |
| `robot.c` | 入口：Init + 每周期 Task |

电机层**不做任何自研控制环**：PID/串级/发送全部复用 `Modules/motor/DJImotor` + `controller`，
CAN 发送由基类 `Modules/motor/motor_task.c` 的 `DJIMotorTask()` 负责。

### 2.3 ESP32 模块（MicroPython，`esp32/`）

| 文件 | 作用 |
|---|---|
| `main.py` | 组装：AP + Bus + 电机/舵机/任务对象 + HTTP + **USB串口桥** |
| `proto.py` | `Bus` 类：UART 收发、遥测解析 |
| `motor.py` | `Motor` 基类 + `M3508/M2006/GM6020` 子类（调协议） |
| `servo.py` | `Servo` 基类 + `PTK7350/PTK7465` |
| `task.py` | `Task` 状态机对象 |
| `web.py` | HTTP 服务 + 结构化 API（`/api/motor|servo|task|vision`） |
| `www/` | 网页：`index.html` `style.css` `js/{motor,servo,task,yaw,app}.js` |

### 2.4 浏览器网页（OO 卡片）

- `MotorCard`(基类) + `M3508Card/M2006Card/GM6020Card`：三模式(速度/角度/圈数)、同屏显示、归零/取零点、PID 面板；圈数模式带状态机切换。
- `ServoCard`：角度 + 状态机 + 取零点。
- `YawCard`：**三模式(手动/自瞄/制导)**，自瞄只保留**启动/关闭**按钮。
- `TaskCard`：8 步时序 + 距离打击表。

### 2.5 上位机视觉（`pc_vision/hik_vision.py`）

- 海康 MVS SDK 取帧（`MvImport` 路径已内置）→ OpenCV 识别绿光 → **串口(COM8)** 发 `C,<x>,<center>`。
- 彩色像素格式强制 `BayerRG8`；绿光阈值已实测标定；识别加了高斯模糊+闭运算+x 时序平滑。

---

## 3. 通信协议

### 3.1 ESP32 ↔ C 板（UART1，115200，ASCII，`\n` 结尾）

ESP32→C：
```
PING                 -> PONG
S                    扫描 CAN 反馈 ID -> S,<n>,<id>...
Z / Z,<slot>         设零
M,<slot>,<mode>,<v>  0停 1速度(rpm*10) 2角度(deg*10) 3圈数(turns*100)
P,<slot>,<id>,<v>    设参数(浮点*100)
R,<slot>             恢复默认
D,<slot>             方向反转
SAVE                 立即保存 -> SAVED
W,<turns*100>        拉簧预备圈数
Y,<mode>             yaw 0手动 1自瞄 2制导
A,<rpm*100>          自瞄最大转速
C,<x>,<center>       注入绿光坐标
V,<a>,<b>            舵机 0设标准 1设预备 2去标准 3去预备 4直接角 5取零
G,<cmd>              0拉簧标准 1拉簧预备 2舵机标准 3舵机预备 10自动开始 11停止 12急停 13解除
H                    心跳(刷新超时)
```

C→ESP32 遥测帧（150ms）：
```
F,<n>,
  [每电机12项 slot,type,id,online,dir,mode,target100,rpm,angle100,turns100,temp,cur]*n,
  [舵机4 cur10,state,std10,prep10],
  [任务5 spring100,step,yaw,estop,aim100],
  [视觉4 x,ok,center,err],
  [每电机11参数]*n
```

### 3.2 PC ↔ ESP32（USB 串口 COM8）

- PC 发整行命令（如 `C,720,720`），ESP32 `usb_rx()` 读取并 `bus.send()` 转发给 C 板。
- C 板遥测经 ESP32 回传（可关，见坑 #7）。

### 3.3 网页 ↔ ESP32（HTTP）

- `/state` 遥测原文；`/cmd?c=...` 透传；`/api/motor|servo|task|vision` 结构化；`/scan` `/scanres` `/ping` `/save`。

---

## 4. 关键 Bug 与根因（核心章节）

> 这些都是“看起来玄学、实则有确定根因”的坑，按发现顺序。

### 坑 #1：网页命令下发无效（URL 编码）
- **现象**：遥测正常，但网页滑块/按钮控制电机全无效。
- **根因**：网页用 `encodeURIComponent` 把逗号编成 `%2C`，ESP32 直接透传给 C 板 → C 板解析到 `M%2C0...` 丢弃。
- **修**：ESP32 端 `urldecode()` 后再写串口。

### 坑 #2：C 板 CAN 层用错（初版）
- **现象**：部分电机识别不到。
- **根因**：接线/ID 问题（硬件），但排查手段值得记：用**全通滤波扫描**（`SimpleMotorScanBegin`）打印总线上真实反馈 ID，确认是硬件而非软件。
- **教训**：`M3508` 与 `M2006` 的 ID1~4 **都走 0x200 控制帧**（附录 A 说 M2006 用 0x1FF 是错的）。

### 坑 #3：电机不转 —— DWT 的 `dt=0` 导致 NaN（**最隐蔽**）
- **现象**：改用 DJImotor 库后，命令生效(mode 变了)但电机纹丝不动、电流≈0。
- **诊断**：加调试字段打印 `speed_PID` 的 `Ref/Measure/Output/MaxOut/DeadBand`，发现 `Ref` 正常但 `Output=0`；
  再打印 `DWT->CYCCNT`/`CoreDebug->DEMCR`，发现 **`DEMCR=0`（TRCENA 被清）→ CYCCNT 不走 → `dt=0`**。
- **根因**：`controller.c` 的 PID 用 `DWT_GetDeltaT` 做积分/微分；`dt=0` 时 `Kd*(...)/dt` 产生 **NaN**，
  `Output` 变 NaN，`(int16_t)NaN=0` → 电机输出 0。J-Link `qc` 断开调试器时会清掉 `CoreDebug->DEMCR`。
- **修**：在每周期任务里兜底重使能 DWT：
  ```c
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
  ```

### 坑 #4：掉电保存失效 —— Flash 擦除被中断打断
- **现象**：保存后 magic 变成 `...B000`（应为 `...B008`），数据丢失。
- **根因**：Flash 擦写期间 CAN/串口中断触发，打断擦除 → 扇区没擦干净；未擦除的位只能 1→0，写入被“与”运算破坏。
- **修**：擦写加临界区 `__disable_irq()/__enable_irq()`。

### 坑 #5：命令太快被合并（一帧只解析第一条）
- **现象**：连续发多条命令（间隔 <~ 数百 ms）只生效第一条。
- **根因**：`bsp_usart` 用**空闲中断+DMA** 收帧，间隔短时并成一帧；旧解析只看 `buf[0]`。
- **修**：`DartDecode` 按 `\n` 拆分，逐条 `ProcessCmd()`。

### 坑 #6：两拉簧电机较劲/发抖/不同时停
- **现象**：从动电机(B)嗡嗡抖、两电机不同时停、圈数对不上。
- **根因**：同步带刚性耦合，两电机各跑**独立角度环**、各用**各自零点**；且**方向(反向标志)不一致**时同命令反向发力 → 顶牛。
- **诊断**：命令给很小速度，看两电机**电流符号**：相反=顶牛，相同=协同。
- **修**：① 确认方向（数据判定 B 不应反向）；② `MotorSet` 对 A/B 命令强制同步；③ 取零时 A/B 一起取。
- **曾踩的雷**：尝试用库的 `DJIMotorChangeFeed` + `other_*_feedback_ptr` 让 B 用 A 的反馈做“严格从动”，
  但**方向若反 → 变正反馈 → 飞车(300rpm封转)**。**只有在方向确认无误后才可用从动**，否则保持独立环（安全）。

### 坑 #7：ESP32 USB 遥测狂打把桥卡死
- **现象**：PC 发坐标后自瞄不动；甚至 ESP32 从 USB 掉线。
- **根因**：ESP32 每帧 `print` 完整遥测到 USB；上位机若来不及读，CDC 缓冲满 → `print` 阻塞 → 主循环卡死 → 串口桥断。
- **修**：去掉完整遥测打印，只保留一条 **2Hz 短调试行**（几十字节）。

### 坑 #8：海康相机枚举/格式问题
- **现象**：`nDeviceNum=0` 或识别不到绿光。
- **根因**：① MVS SDK 枚举**偶发返回 0 台**（重试即可）；② 重新插拔后**像素格式变成灰度**，转 BGR 后是灰的，HSV 找不到绿色。
- **修**：枚举重试 10 次；打开后**强制设 `PixelFormat=BayerRG8`**（彩色）；再转 BGR8。

### 坑 #9：自瞄“对不准中心” —— M2006 速度环死区
- **现象**：自瞄能追，但最后差几十像素推不动（“蠕动”）。
- **根因**：M2006 速度 PID 的 `DeadBand=10rpm`；自瞄算出的速度指令 <10rpm 时速度环直接输出 0 → 推不动。
- **修**：M2006 速度死区 `10 → 3`；并给自瞄加**最小转速**(防静摩擦)与 **PI**(消静差)。

### 坑 #10：J-Link 克隆探针
- **现象**：官方软件提示固件更新后，探针挂死（`ShowEmuList` 空 / 连接超时）。
- **恢复**：**拔插 USB** 后，J-Link 会自动用 bootloader 重刷固件并复活；期间要用 `-NoGui 1`，否则 GUI 会挂起。
- **教训**：克隆探针**别联网更新固件**。

---

## 5. 掉电保存设计（v5）

- 两个 Flash 扇区各存一份完整配置：Bank A=`0x080C0000`(sector10)、Bank B=`0x080E0000`(sector11)。
- 结构：`magic + seq + crc(FNV-1a) + data`。
- 保存：写到“非当前”那份，`seq+1`；上电读两份，**校验通过且 seq 大者胜出**。
- 好处：**写一半掉电，另一份仍完好**；最坏只丢“断电瞬间那一次改动”。
- 时机：设定变化后去抖 300ms；电机在动最多再等 2s 强制写。擦写加临界区。
- 保存内容：四电机 11 项参数 / 零点 / 方向、舵机标准位+预备位+偏移、拉簧圈数、yaw 模式、自瞄转速。

---

## 6. 自瞄控制设计（v5_HIK）

- PC 每 ~25ms 发 `C,x,720`；C 板 `g_vis_err = x-720`，超时 500ms 判丢目标。
- 控制律（`dart_fsm.c` YawHandler，约 1kHz）：
  ```c
  aim_i += err*0.001; clamp(±I_LIMIT);
  spd = KP*err + KI*aim_i; clamp(±aim_rpm);
  if (|spd| < MIN_RPM) spd = sign(err)*MIN_RPM;   // 防蠕动
  if (|err| < deadband) spd = 0;                   // 死区停
  MotorSet(M_YAW, MODE_SPEED, spd);
  ```
- 当前参数：`KP=3.0, KI=5.0, I_LIMIT=200, deadband=2px, aim_rpm=200, MIN_RPM=30`（**已定稿，不要再改**）。
- 机械增益实测约 **5 px / 输出圈**（1 圈 → 画面动 5 像素）。这是“快慢”的物理上限：
  - 目标移动 ≤~100px：200rpm 基本够（5s 内）
  - 移动几百 px：当前机构 5s 内做不到，需改机械或换电机。

---

## 7. 构建与烧录

```powershell
# C 板编译（示例）
cmake -G Ninja -S . -B cmake-build-dart_launcher_web_v5_HIK -DCMAKE_BUILD_TYPE=Debug `
      -DBOARD_TYPE=GIMBAL_BOARD -DROBOT_TYPE=dart_launcher_web_v5_HIK
cmake --build cmake-build-dart_launcher_web_v5_HIK

# 烧录（J-Link，注意 -NoGui 1）
# script: si SWD / speed 4000 / device STM32F407IG / connect / r / h / loadfile <hex> / r / g / qc
JLink.exe -NoGui 1 -CommanderScript flash.jlink

# ESP32 上传（mpremote，COM8；上传时 PC 上位机要关掉，否则端口占用）
python -m mpremote connect COM8 fs cp esp32\main.py :main.py
python -m mpremote connect COM8 fs cp esp32\www\js\yaw.js :www/js/yaw.js
python -m mpremote connect COM8 reset
```

---

## 8. 已知限制 / TODO

- 自瞄机械增益低，大位移无法 5s 完成（机构问题）。
- 自瞄大误差时过冲较大（I 大所致，已按要求“允许过冲”）。
- 相机 MVS 枚举偶发 0 台（已加重试）；重新插拔后需确认 `PixelFormat` 为彩色。
- 视觉目前是 PC 端；附录 D 规划的是 OpenMV→SPI，未接入。
- 指示灯/急停硬件、SPI 帧格式等 TODO 见 `agent_must_read_dart.txt` 附录 F。
- 舵机 PID 面板为占位（PWM 无反馈）；若换 PTK 总线舵机需补实现。
